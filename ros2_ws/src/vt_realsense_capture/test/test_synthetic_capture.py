"""Recorder-only ROS lifecycle smoke test for the capture controller."""

from __future__ import annotations

from collections.abc import Callable
from enum import IntEnum
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vt_camera_msgs.msg import CaptureCommand, CaptureEvent, CaptureStatus

from synthetic_capture_support import make_timing_message


CAMERA_NAMES = ('d405_1', 'd405_2', 'd436')
TARGET_DATA_TOPICS = frozenset(
    topic
    for camera_name in CAMERA_NAMES
    for topic in (
        f'/{camera_name}/color/image_raw',
        f'/{camera_name}/depth/image_rect_raw',
        f'/{camera_name}/color/camera_info',
        f'/{camera_name}/frame_timing',
    )
) | frozenset(
    f'/vive/{role}/sample'
    for role in ('left_wrist', 'right_wrist', 'torso')
)
ROS_INFRASTRUCTURE_TOPICS = frozenset({'/parameter_events', '/rosout'})


class CaptureState(IntEnum):
    RECORDING = CaptureStatus.RECORDING
    COMPLETE = CaptureStatus.COMPLETE


def _scan_recorder_processes(output_root: Path) -> set[tuple[int, int]]:
    output_root_text = str(output_root)
    processes: set[tuple[int, int]] = set()
    for process_dir in Path('/proc').iterdir():
        if not process_dir.name.isdigit():
            continue
        try:
            command = (process_dir / 'cmdline').read_bytes().replace(
                b'\0', b' '
            ).decode(errors='replace')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if 'ros2 bag record' not in command or output_root_text not in command:
            continue
        pid = int(process_dir.name)
        try:
            processes.add((pid, os.getpgid(pid)))
        except ProcessLookupError:
            continue
    return processes


def _scan_recorder_process_groups(output_root: Path) -> set[int]:
    return {pgid for _pid, pgid in _scan_recorder_processes(output_root)}


def _process_group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_for_process_group_exit(pgid: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _process_group_exists(pgid):
            return True
        time.sleep(0.05)
    return not _process_group_exists(pgid)


def _cleanup_recorder_process_groups(
    output_root: Path,
    registered_groups: set[int],
    *,
    scan: Callable[[Path], set[int]] = _scan_recorder_process_groups,
    process_group_exists: Callable[[int], bool] = _process_group_exists,
    killpg: Callable[[int, signal.Signals], None] = os.killpg,
    wait_for_exit: Callable[[int, float], bool] = _wait_for_process_group_exit,
) -> None:
    errors: list[BaseException] = []
    process_groups = set(registered_groups)
    try:
        process_groups.update(scan(output_root))
    except BaseException as exc:
        errors.append(exc)

    for pgid in sorted(process_groups):
        try:
            if not process_group_exists(pgid):
                continue
            killpg(pgid, signal.SIGKILL)
            if not wait_for_exit(pgid, 5.0):
                raise AssertionError(
                    f'failed to force-clean Recorder process group {pgid}'
                )
        except ProcessLookupError:
            continue
        except BaseException as exc:
            errors.append(exc)

    if errors:
        raise BaseExceptionGroup('Recorder cleanup failed', errors)


def _terminate_launch_process(
    launch_process: object,
    killpg: Callable[[int, signal.Signals], None],
) -> None:
    if launch_process.poll() is not None:
        return

    errors: list[BaseException] = []
    escalation = (
        (signal.SIGINT, 10.0),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 5.0),
    )
    for next_signal, timeout in escalation:
        if launch_process.poll() is not None:
            return
        try:
            killpg(launch_process.pid, next_signal)
        except ProcessLookupError:
            return
        except BaseException as exc:
            errors.append(exc)
            continue
        try:
            launch_process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        except BaseException as exc:
            errors.append(exc)
            continue
        return

    if launch_process.poll() is None:
        errors.append(AssertionError('failed to reap synthetic launch process'))
    if errors:
        raise BaseExceptionGroup('synthetic launch cleanup failed', errors)


def _teardown_synthetic_stack(
    stack: object,
    launch_process: object,
    *,
    killpg: Callable[[int, signal.Signals], None],
    shutdown_rclpy: Callable[[], None],
) -> None:
    errors: list[BaseException] = []

    def capture(operation: Callable[[], None]) -> None:
        try:
            operation()
        except BaseException as exc:
            errors.append(exc)

    try:
        try:
            capture(stack.close)
        finally:
            try:
                capture(lambda: _terminate_launch_process(launch_process, killpg))
            finally:
                capture(stack.force_cleanup_recorders)
    finally:
        capture(shutdown_rclpy)

    if errors:
        raise BaseExceptionGroup('synthetic teardown failed', errors)


class SyntheticStack:
    def __init__(
        self,
        launch_process: subprocess.Popen[str],
        output_root: Path,
    ) -> None:
        self._launch_process = launch_process
        self._output_root = output_root
        self._recorder_process_groups: set[int] = set()
        self._launch_output: list[str] = []
        self._node = Node('synthetic_capture_smoke_test')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._command_publisher = self._node.create_publisher(
            CaptureCommand, '/capture/command', reliable
        )
        self._statuses: list[CaptureStatus] = []
        self._events: list[CaptureEvent] = []
        self._node.create_subscription(
            CaptureStatus, '/capture/status', self._statuses.append, reliable
        )
        self._node.create_subscription(
            CaptureEvent, '/capture/event', self._events.append, reliable
        )
        self._output_thread = threading.Thread(
            target=self._collect_launch_output,
            name='synthetic-launch-output',
            daemon=True,
        )
        self._output_thread.start()

    def _collect_launch_output(self) -> None:
        assert self._launch_process.stdout is not None
        for line in self._launch_process.stdout:
            self._launch_output.append(line.rstrip())

    def _assert_launch_alive(self) -> None:
        return_code = self._launch_process.poll()
        if return_code is not None:
            output = '\n'.join(self._launch_output[-80:])
            raise AssertionError(
                f'synthetic launch exited with code {return_code}:\n{output}'
            )

    def _spin_until(self, predicate, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._assert_launch_alive()
            rclpy.spin_once(self._node, timeout_sec=0.05)
            result = predicate()
            if result is not None:
                return result
        output = '\n'.join(self._launch_output[-80:])
        raise AssertionError(f'timed out waiting for {description}:\n{output}')

    def assert_minimal_graph(self, timeout: float = 10.0) -> None:
        def graph_if_ready() -> set[str] | None:
            if ('synthetic_capture_support', '/') not in set(
                self._node.get_node_names_and_namespaces()
            ):
                return None
            publishers = self._node.get_publisher_names_and_types_by_node(
                'synthetic_capture_support', '/'
            )
            support_topics = {
                name
                for name, _types in publishers
                if name not in ROS_INFRASTRUCTURE_TOPICS
            }
            graph_topics = {
                name
                for name, _types in self._node.get_topic_names_and_types()
                if name not in ROS_INFRASTRUCTURE_TOPICS
            }
            if support_topics != TARGET_DATA_TOPICS:
                return None
            if not TARGET_DATA_TOPICS.issubset(graph_topics):
                return None
            return graph_topics

        graph_topics = self._spin_until(
            graph_if_ready, timeout, 'the minimal synthetic ROS graph'
        )
        unexpected = {
            name
            for name in graph_topics
            if name not in TARGET_DATA_TOPICS
            and not name.startswith('/capture/')
        }
        assert unexpected == set()

    def _publish_command(
        self,
        command: int,
        request_id: str,
        *,
        planned_duration_sec: int = 0,
    ) -> None:
        self._spin_until(
            lambda: True
            if self._command_publisher.get_subscription_count() > 0
            else None,
            10.0,
            'the controller command subscription',
        )
        message = CaptureCommand()
        message.request_id = request_id
        message.command = command
        message.session_label = 'synthetic-smoke'
        message.planned_duration_sec = planned_duration_sec
        self._command_publisher.publish(message)

    def publish_start(
        self, request_id: str, *, planned_duration_sec: int
    ) -> None:
        self._publish_command(
            CaptureCommand.START,
            request_id,
            planned_duration_sec=planned_duration_sec,
        )

    def publish_stop(self, request_id: str) -> None:
        self._publish_command(CaptureCommand.STOP, request_id)

    def wait_for_state(
        self, state: CaptureState, *, timeout: float
    ) -> CaptureStatus:
        return self._spin_until(
            lambda: next(
                (
                    status
                    for status in reversed(self._statuses)
                    if int(status.state) == int(state)
                ),
                None,
            ),
            timeout,
            f'capture state {state.name}',
        )

    def wait_for_recorder_process(
        self, *, timeout: float
    ) -> tuple[int, int]:
        def recorder_if_started() -> tuple[int, int] | None:
            for pid, pgid in _scan_recorder_processes(self._output_root):
                self._recorder_process_groups.add(pgid)
                return pid, pgid
            return None

        return self._spin_until(
            recorder_if_started, timeout, 'the Recorder child process'
        )

    @staticmethod
    def _process_group_exists(pgid: int) -> bool:
        return _process_group_exists(pgid)

    def wait_for_process_group_exit(self, pgid: int, *, timeout: float) -> bool:
        return _wait_for_process_group_exit(pgid, timeout)

    def interrupt_launch(self) -> int:
        if self._launch_process.poll() is None:
            os.killpg(self._launch_process.pid, signal.SIGINT)
        return self._launch_process.wait(timeout=20.0)

    def force_cleanup_recorders(self) -> None:
        _cleanup_recorder_process_groups(
            self._output_root,
            self._recorder_process_groups,
        )

    def recorder_shutdown_diagnostics(self, pgid: int) -> str:
        process_table = subprocess.run(
            ['ps', '-eo', 'pid,ppid,pgid,stat,args'],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        group_lines = [
            line for line in process_table.splitlines()
            if len(line.split(maxsplit=4)) >= 3
            and line.split(maxsplit=4)[2] == str(pgid)
        ]
        launch_output = '\n'.join(self._launch_output[-80:])
        return (
            f'Recorder process group {pgid} survived controller shutdown\n'
            f'process group members:\n' + '\n'.join(group_lines)
            + f'\nlaunch output:\n{launch_output}'
        )

    def close(self) -> None:
        self._node.destroy_node()


@pytest.fixture
def synthetic_stack(tmp_path: Path) -> SyntheticStack:
    domain_id = 1 + (os.getpid() % 100)
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init()
    output_root = (tmp_path / 'capture-output').resolve()
    output_root.mkdir()
    environment = dict(os.environ)
    launch_process = subprocess.Popen(
        [
            'ros2',
            'launch',
            'vt_realsense_capture',
            'synthetic_capture.launch.py',
            f'output_root:={output_root}',
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    stack = SyntheticStack(launch_process, output_root)
    try:
        yield stack
    finally:
        _teardown_synthetic_stack(
            stack,
            launch_process,
            killpg=os.killpg,
            shutdown_rclpy=rclpy.shutdown,
        )


def test_synthetic_start_stop_reaches_complete_without_quality_gate(
    synthetic_stack,
) -> None:
    synthetic_stack.assert_minimal_graph()

    synthetic_stack.publish_start('synthetic-start', planned_duration_sec=0)
    recording = synthetic_stack.wait_for_state(CaptureState.RECORDING, timeout=10.0)
    assert recording.streams == []
    assert recording.camera_groups == []
    assert recording.recorder_alive is True

    synthetic_stack.publish_stop('synthetic-stop')
    complete = synthetic_stack.wait_for_state(CaptureState.COMPLETE, timeout=10.0)
    assert complete.recorder_alive is False
    assert complete.detail == 'Recorder process lifecycle complete'


def test_synthetic_timing_does_not_claim_unpopulated_callback_clocks() -> None:
    message = make_timing_message(
        'd436', Time(sec=1, nanosec=2), 123456789, frame_number=7
    )

    callback_clock_bits = (
        message.VALID_HOST_MONOTONIC | message.VALID_HOST_REALTIME
    )
    assert message.color_validity_flags & callback_clock_bits == 0
    assert message.depth_validity_flags & callback_clock_bits == 0
    assert (
        message.group_validity_flags & message.GROUP_VALID_CALLBACK_CLOCKS
    ) == 0


def test_sigint_without_stop_reaps_the_real_recorder_process_group(
    synthetic_stack: SyntheticStack,
) -> None:
    synthetic_stack.publish_start(
        'synthetic-shutdown-start', planned_duration_sec=0
    )
    synthetic_stack.wait_for_state(CaptureState.RECORDING, timeout=10.0)
    recorder_pid, recorder_pgid = synthetic_stack.wait_for_recorder_process(
        timeout=10.0
    )
    assert recorder_pid == recorder_pgid

    synthetic_stack.interrupt_launch()

    exited = synthetic_stack.wait_for_process_group_exit(
        recorder_pgid, timeout=10.0
    )
    assert exited, synthetic_stack.recorder_shutdown_diagnostics(recorder_pgid)


def test_teardown_escalates_launch_and_runs_finalizers_after_failures() -> None:
    operations: list[str] = []
    signals: list[tuple[int, signal.Signals]] = []

    class FakeLaunch:
        pid = 4242

        def __init__(self) -> None:
            self.wait_timeouts: list[float] = []
            self._wait_results: list[object] = [
                subprocess.TimeoutExpired('launch', 10.0),
                subprocess.TimeoutExpired('launch', 5.0),
                0,
            ]

        def poll(self) -> int | None:
            return None if self._wait_results else 0

        def wait(self, timeout: float) -> int:
            self.wait_timeouts.append(timeout)
            result = self._wait_results.pop(0)
            if isinstance(result, BaseException):
                raise result
            return int(result)

    class FakeStack:
        def close(self) -> None:
            operations.append('close')

        def force_cleanup_recorders(self) -> None:
            operations.append('recorder-cleanup')
            raise OSError('one recorder cleanup failed')

    launch = FakeLaunch()

    with pytest.raises(ExceptionGroup, match='synthetic teardown failed'):
        _teardown_synthetic_stack(
            FakeStack(),
            launch,
            killpg=lambda pid, sig: signals.append((pid, sig)),
            shutdown_rclpy=lambda: operations.append('rclpy-shutdown'),
        )

    assert signals == [
        (launch.pid, signal.SIGINT),
        (launch.pid, signal.SIGTERM),
        (launch.pid, signal.SIGKILL),
    ]
    assert launch.wait_timeouts == [10.0, 5.0, 5.0]
    assert operations == ['close', 'recorder-cleanup', 'rclpy-shutdown']


def test_recorder_cleanup_rescans_output_root_and_continues_after_failure(
    tmp_path: Path,
) -> None:
    output_root = (tmp_path / 'unique-output').resolve()
    output_root.mkdir()
    scanned_roots: list[Path] = []
    kill_attempts: list[int] = []
    wait_attempts: list[int] = []

    def scan(root: Path) -> set[int]:
        scanned_roots.append(root)
        return {202, 303}

    def killpg(pgid: int, _sig: signal.Signals) -> None:
        kill_attempts.append(pgid)
        if pgid == 101:
            raise OSError('first group cleanup failed')

    def wait_for_exit(pgid: int, _timeout: float) -> bool:
        wait_attempts.append(pgid)
        return True

    with pytest.raises(ExceptionGroup, match='Recorder cleanup failed'):
        _cleanup_recorder_process_groups(
            output_root,
            {101},
            scan=scan,
            process_group_exists=lambda _pgid: True,
            killpg=killpg,
            wait_for_exit=wait_for_exit,
        )

    assert scanned_roots == [output_root]
    assert kill_attempts == [101, 202, 303]
    assert wait_attempts == [202, 303]
