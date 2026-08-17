import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from vt_camera_msgs.msg import CaptureEvent, CaptureStatus, SessionInfo


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from vt_realsense_capture.controller import (
    CaptureEventFact,
    CaptureStatusFact,
    EventSeverity,
    RecorderStartSpec,
    SessionInfoFact,
)
from vt_realsense_capture.recorder import required_topics
from vt_realsense_capture.controller_node import (
    ManagedRecorder,
    project_capture_event,
    project_capture_status,
    project_session_info,
    shutdown_controller_worker,
)
from vt_realsense_capture.session import CaptureState


SOURCE = (
    PACKAGE_ROOT
    / "vt_realsense_capture"
    / "controller_node.py"
).read_text()


def test_ros_boundary_has_no_camera_data_or_gate_dependencies() -> None:
    forbidden = (
        "sensor_msgs.msg import Image",
        "CameraFrameTiming",
        "create_client(DeviceInfo",
        "AsyncParameterClient",
        "RECORDER_RESUME_SERVICE",
        '"/timing_normalizer/flush"',
        "FullBagValidator",
        "image_subscription_sink",
        "recorder_subscriptions_ready",
    )
    assert all(token not in SOURCE for token in forbidden)


def test_ros_boundary_only_subscribes_to_capture_command() -> None:
    assert SOURCE.count("self.create_subscription(") == 1
    assert '"/capture/command"' in SOURCE


def test_managed_recorder_start_only_spawns_process(tmp_path: Path) -> None:
    session_dir = tmp_path / "session"
    session_dir.mkdir()
    storage_config = tmp_path / "mcap_writer_options.yaml"
    storage_config.write_text("compression: None\n")
    qos_overrides = session_dir / "qos_overrides.yaml"
    qos_overrides.write_text("{}\n")
    output_path = session_dir / "bag"
    starts: list[bool] = []

    class FakeProcess:
        def start(self) -> None:
            starts.append(True)

        def poll(self) -> int | None:
            raise AssertionError("start must not poll for Recorder readiness")

    def process_factory(
        command: list[str], *, session_parent: Path
    ) -> FakeProcess:
        assert "--output" in command
        assert command[command.index("--output") + 1] == str(output_path)
        assert session_parent == session_dir
        return FakeProcess()

    recorder = ManagedRecorder(
        storage_config,
        process_factory=process_factory,
    )

    recorder.start(
        RecorderStartSpec(
            session_id="session",
            session_dir=session_dir,
            output_path=output_path,
            qos_overrides_path=qos_overrides,
            topics=required_topics(("d405_1", "d405_2", "d436")),
        )
    )

    assert starts == [True]
    assert not output_path.exists()


@pytest.mark.parametrize(
    (
        "returncode",
        "reason",
        "termination_confirmed",
        "possibly_alive",
    ),
    [
        pytest.param(0, "clean_exit", True, False, id="clean"),
        pytest.param(17, "nonzero_exit", True, False, id="nonzero"),
        pytest.param(None, "sigkill_wait_failed", False, True, id="uncertain"),
    ],
)
def test_managed_recorder_stop_maps_every_process_result_field(
    tmp_path: Path,
    returncode: int | None,
    reason: str,
    termination_confirmed: bool,
    possibly_alive: bool,
) -> None:
    session_dir = tmp_path / "session"
    session_dir.mkdir()
    storage_config = tmp_path / "mcap_writer_options.yaml"
    storage_config.write_text("compression: None\n")
    qos_overrides = session_dir / "qos_overrides.yaml"
    qos_overrides.write_text("{}\n")

    class FakeProcess:
        def start(self) -> None:
            pass

        def stop(self) -> object:
            return SimpleNamespace(
                returncode=returncode,
                reason=reason,
                termination_confirmed=termination_confirmed,
                possibly_alive=possibly_alive,
            )

    recorder = ManagedRecorder(
        storage_config,
        process_factory=lambda *_args, **_kwargs: FakeProcess(),
    )
    recorder.start(
        RecorderStartSpec(
            session_id="session",
            session_dir=session_dir,
            output_path=session_dir / "bag",
            qos_overrides_path=qos_overrides,
            topics=required_topics(("d405_1", "d405_2", "d436")),
        )
    )

    fact = recorder.stop()

    assert fact.returncode == returncode
    assert fact.detail == reason
    assert fact.termination_confirmed is termination_confirmed
    assert fact.possibly_alive is possibly_alive


def test_capture_event_projection_maps_every_fact_field() -> None:
    stamp = Time(sec=12, nanosec=34)
    fact = CaptureEventFact(
        request_id="request-1",
        session_id="session-1",
        severity=EventSeverity.FATAL,
        code="termination_unconfirmed",
        camera_name="d436",
        stream_name="depth",
        detail="recorder may be alive",
    )

    message = project_capture_event(CaptureEvent(), fact, stamp)

    assert message.header.stamp == stamp
    assert message.request_id == fact.request_id
    assert message.session_id == fact.session_id
    assert message.severity == int(fact.severity)
    assert message.code == fact.code
    assert message.camera_name == fact.camera_name
    assert message.stream_name == fact.stream_name
    assert message.detail == fact.detail


def test_capture_status_projection_maps_every_fact_field() -> None:
    stamp = Time(sec=56, nanosec=78)
    fact = CaptureStatusFact(
        request_id="request-2",
        session_id="session-2",
        state=CaptureState.FINALIZING,
        streams=(),
        camera_groups=(),
        recorder_alive=True,
        disk_free_bytes=123456,
        observed_write_mb_s=78.5,
        detail="finalizing",
    )

    message = project_capture_status(CaptureStatus(), fact, stamp)

    assert message.header.stamp == stamp
    assert message.request_id == fact.request_id
    assert message.session_id == fact.session_id
    assert message.state == int(fact.state)
    assert message.streams == []
    assert message.camera_groups == []
    assert message.recorder_alive is fact.recorder_alive
    assert message.disk_free_bytes == fact.disk_free_bytes
    assert message.observed_write_mb_s == fact.observed_write_mb_s
    assert message.detail == fact.detail


def test_session_info_projection_maps_every_fact_and_empty_metadata_field() -> None:
    stamp = Time(sec=90, nanosec=12)
    fact = SessionInfoFact("session-3", "request-3", "trial")

    message = project_session_info(SessionInfo(), fact, stamp)

    assert message.header.stamp == stamp
    assert message.session_id == fact.session_id
    assert message.request_id == fact.request_id
    assert message.session_label == fact.session_label
    assert message.hostname == ""
    assert message.kernel_version == ""
    assert message.ros_distro == ""
    assert message.realsense_ros_version == ""
    assert message.librealsense_version == ""
    assert message.git_commit == ""
    assert message.config_sha256 == ""
    assert message.cameras == []
    assert message.streams == []


def test_shutdown_controller_worker_runs_after_queued_work_on_same_thread() -> None:
    worker = ThreadPoolExecutor(max_workers=1)
    order: list[str] = []
    worker_thread_ids: list[int] = []

    def queued_work() -> None:
        order.append("queued")
        worker_thread_ids.append(threading.get_ident())

    def shutdown() -> bool:
        order.append("shutdown")
        worker_thread_ids.append(threading.get_ident())
        return True

    worker.submit(queued_work)

    assert shutdown_controller_worker(worker, shutdown) is True
    assert order == ["queued", "shutdown"]
    assert len(set(worker_thread_ids)) == 1
    with pytest.raises(RuntimeError, match="shutdown"):
        worker.submit(lambda: None)


def test_shutdown_controller_worker_finishes_after_keyboard_interrupts() -> None:
    class InterruptingFuture:
        def __init__(self) -> None:
            self.calls = 0

        def result(self) -> bool:
            self.calls += 1
            if self.calls == 1:
                raise KeyboardInterrupt
            return True

    class InterruptingWorker:
        def __init__(self) -> None:
            self.future = InterruptingFuture()
            self.shutdown_calls = 0

        def submit(self, callback):
            assert callback() is True
            return self.future

        def shutdown(self, *, wait: bool, cancel_futures: bool) -> None:
            assert wait is True
            assert cancel_futures is True
            self.shutdown_calls += 1
            if self.shutdown_calls == 1:
                raise KeyboardInterrupt

    worker = InterruptingWorker()

    assert shutdown_controller_worker(worker, lambda: True) is True
    assert worker.future.calls == 2
    assert worker.shutdown_calls == 2
