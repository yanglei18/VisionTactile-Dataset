#!/usr/bin/python3
"""Thin ROS 2 adapter for the Recorder-only capture controller."""

from __future__ import annotations

import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, TypeVar

from vt_realsense_capture.controller import (
    CaptureCommandFact,
    CaptureController,
    CaptureEventFact,
    CaptureStatusFact,
    CommandKind,
    RawCaptureCommandFact,
    RecorderHealth,
    RecorderStartSpec,
    RecorderStopFact,
    SessionInfoFact,
    submit_raw_capture_command,
)
from vt_realsense_capture.recorder import (
    RecorderProcess,
    build_record_command,
)
from vt_realsense_capture.session import RequestValidationError
from vt_realsense_capture.storage import FileStorage


MessageT = TypeVar("MessageT")


def project_capture_event(
    message: MessageT, fact: CaptureEventFact, stamp: object
) -> MessageT:
    """Project every controller Event fact field onto a ROS message."""

    message.header.stamp = stamp  # type: ignore[attr-defined]
    message.request_id = fact.request_id  # type: ignore[attr-defined]
    message.session_id = fact.session_id  # type: ignore[attr-defined]
    message.severity = int(fact.severity)  # type: ignore[attr-defined]
    message.code = fact.code  # type: ignore[attr-defined]
    message.camera_name = fact.camera_name  # type: ignore[attr-defined]
    message.stream_name = fact.stream_name  # type: ignore[attr-defined]
    message.detail = fact.detail  # type: ignore[attr-defined]
    return message


def project_capture_status(
    message: MessageT, fact: CaptureStatusFact, stamp: object
) -> MessageT:
    """Project every controller Status fact field onto a ROS message."""

    message.header.stamp = stamp  # type: ignore[attr-defined]
    message.request_id = fact.request_id  # type: ignore[attr-defined]
    message.session_id = fact.session_id  # type: ignore[attr-defined]
    message.state = int(fact.state)  # type: ignore[attr-defined]
    message.streams = list(fact.streams)  # type: ignore[attr-defined]
    message.camera_groups = list(fact.camera_groups)  # type: ignore[attr-defined]
    message.recorder_alive = fact.recorder_alive  # type: ignore[attr-defined]
    message.disk_free_bytes = fact.disk_free_bytes  # type: ignore[attr-defined]
    message.observed_write_mb_s = fact.observed_write_mb_s  # type: ignore[attr-defined]
    message.detail = fact.detail  # type: ignore[attr-defined]
    return message


def project_session_info(
    message: MessageT, fact: SessionInfoFact, stamp: object
) -> MessageT:
    """Project the minimal truthful session facts onto a ROS message."""

    message.header.stamp = stamp  # type: ignore[attr-defined]
    message.session_id = fact.session_id  # type: ignore[attr-defined]
    message.request_id = fact.request_id  # type: ignore[attr-defined]
    message.session_label = fact.session_label  # type: ignore[attr-defined]
    message.hostname = ""  # type: ignore[attr-defined]
    message.kernel_version = ""  # type: ignore[attr-defined]
    message.ros_distro = ""  # type: ignore[attr-defined]
    message.realsense_ros_version = ""  # type: ignore[attr-defined]
    message.librealsense_version = ""  # type: ignore[attr-defined]
    message.git_commit = ""  # type: ignore[attr-defined]
    message.config_sha256 = ""  # type: ignore[attr-defined]
    message.cameras = []  # type: ignore[attr-defined]
    message.streams = []  # type: ignore[attr-defined]
    return message


def shutdown_controller_worker(
    worker: object, shutdown: Callable[[], bool]
) -> bool:
    """Run controller shutdown behind queued work despite process SIGINT."""

    future = worker.submit(shutdown)  # type: ignore[attr-defined]
    try:
        while True:
            try:
                return bool(future.result())
            except KeyboardInterrupt:
                # ROS launch delivers SIGINT to the node while cleanup is
                # already under way. The Recorder owns a separate process
                # group, so abandoning this wait can orphan ros2 bag.
                continue
    finally:
        while True:
            try:
                worker.shutdown(  # type: ignore[attr-defined]
                    wait=True, cancel_futures=True
                )
                break
            except KeyboardInterrupt:
                continue


class SystemClock:
    """Expose the monotonic clock required by the pure controller."""

    def monotonic_ns(self) -> int:
        clock_id = getattr(time, "CLOCK_MONOTONIC_RAW", time.CLOCK_MONOTONIC)
        return time.clock_gettime_ns(clock_id)


class ManagedRecorder:
    """Adapt ``RecorderProcess`` to the controller's Recorder protocol."""

    def __init__(
        self,
        storage_config_path: Path,
        *,
        process_factory: Callable[..., RecorderProcess] = RecorderProcess,
    ) -> None:
        self._storage_config_path = Path(storage_config_path)
        self._process_factory = process_factory
        self._process: RecorderProcess | None = None

    def start(self, spec: RecorderStartSpec) -> None:
        if self._process is not None:
            raise RuntimeError("managed Recorder may start only once")
        command = build_record_command(
            output_path=spec.output_path,
            storage_config_path=self._storage_config_path,
            qos_overrides_path=spec.qos_overrides_path,
            topics=spec.topics,
        )
        process = self._process_factory(command, session_parent=spec.session_dir)
        process.start()
        self._process = process

    def health(self) -> RecorderHealth:
        if self._process is None:
            return RecorderHealth.UNCERTAIN
        try:
            return (
                RecorderHealth.ALIVE
                if self._process.poll() is None
                else RecorderHealth.EXITED
            )
        except Exception:
            return RecorderHealth.UNCERTAIN

    def stop(self) -> RecorderStopFact:
        if self._process is None:
            return RecorderStopFact(
                returncode=None,
                detail="Recorder never started",
                termination_confirmed=True,
                possibly_alive=False,
            )
        result = self._process.stop()
        return RecorderStopFact(
            returncode=result.returncode,
            detail=result.reason,
            termination_confirmed=result.termination_confirmed,
            possibly_alive=result.possibly_alive,
        )


def new_session_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return f"vt-{timestamp}-{uuid.uuid4().hex[:12]}"


def main() -> None:
    """Run the ROS boundary with workflow work serialized off-executor."""

    from concurrent.futures import Future, ThreadPoolExecutor
    from threading import Lock

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from vt_camera_msgs.msg import (
        CaptureCommand,
        CaptureEvent,
        CaptureStatus,
        SessionInfo,
    )
    from vt_realsense_capture.config import load_config

    class CaptureControllerNode(Node):
        def __init__(self) -> None:
            super().__init__("capture_controller")
            capture_share = Path(
                get_package_share_directory("vt_realsense_capture")
            )
            default_config = capture_share / "config" / "cameras.yaml"
            self.declare_parameter("config_path", str(default_config))
            self.declare_parameter("output_root", "")
            config_path = Path(
                self.get_parameter("config_path")
                .get_parameter_value()
                .string_value
            )
            output_root_text = (
                self.get_parameter("output_root")
                .get_parameter_value()
                .string_value
            )
            self._config = load_config(config_path)
            self._worker = ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="capture-workflow"
            )
            self._closed = False
            self._submission_lock = Lock()
            self._tick_future: Future[object] | None = None
            self._status_future: Future[object] | None = None
            self._submission_group = MutuallyExclusiveCallbackGroup()

            reliable = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            session_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._status_publisher = self.create_publisher(
                CaptureStatus, "/capture/status", reliable
            )
            self._event_publisher = self.create_publisher(
                CaptureEvent, "/capture/event", reliable
            )
            self._session_publisher = self.create_publisher(
                SessionInfo, "/capture/session_info", session_qos
            )
            storage_config = capture_share / "config" / "mcap_writer_options.yaml"
            self._controller = CaptureController(
                config=self._config,
                output_root=Path(output_root_text),
                clock=SystemClock(),
                storage=FileStorage(),
                recorder_factory=lambda: ManagedRecorder(storage_config),
                event_sink=self._publish_event,
                status_sink=self._publish_status,
                session_sink=self._publish_session,
                session_id_factory=new_session_id,
            )
            self._command_subscription = self.create_subscription(
                CaptureCommand,
                "/capture/command",
                self._command_callback,
                reliable,
                callback_group=self._submission_group,
            )
            self._workflow_timer = self.create_timer(0.1, self._submit_tick)
            self._status_timer = self.create_timer(1.0, self._submit_status)

        def _command_callback(self, message: CaptureCommand) -> None:
            if self._closed:
                return
            try:
                future = submit_raw_capture_command(
                    message,
                    lambda raw: self._submit_work(self._accept_command, raw),
                )
            except RequestValidationError as exc:
                self.get_logger().error(f"invalid capture command: {exc}")
                return
            if future is None:
                return
            future.add_done_callback(self._log_worker_failure)

        def _submit_work(
            self, callback: Callable[..., object], *args: object
        ) -> Future[object] | None:
            with self._submission_lock:
                if self._closed:
                    return None
                return self._worker.submit(callback, *args)

        def _accept_command(self, command: RawCaptureCommandFact) -> None:
            fact = CaptureCommandFact(
                request_id=command.request_id,
                command=CommandKind(command.command),
                session_label=command.session_label,
                planned_duration_sec=command.planned_duration_sec,
            )
            self._controller.accept(fact)

        def _submit_tick(self) -> None:
            if self._closed:
                return
            if self._tick_future is None or self._tick_future.done():
                future = self._submit_work(self._controller.tick)
                if future is not None:
                    self._tick_future = future
                    future.add_done_callback(self._log_worker_failure)

        def _submit_status(self) -> None:
            if self._closed:
                return
            if self._status_future is None or self._status_future.done():
                future = self._submit_work(
                    self._controller.publish_periodic_status
                )
                if future is not None:
                    self._status_future = future
                    future.add_done_callback(self._log_worker_failure)

        def _log_worker_failure(self, future: Future[object]) -> None:
            try:
                future.result()
            except Exception as exc:
                self.get_logger().error(f"capture workflow failed: {exc}")

        def _publish_event(self, fact: CaptureEventFact) -> None:
            message = project_capture_event(
                CaptureEvent(), fact, self.get_clock().now().to_msg()
            )
            self._event_publisher.publish(message)

        def _publish_status(self, fact: CaptureStatusFact) -> None:
            message = project_capture_status(
                CaptureStatus(), fact, self.get_clock().now().to_msg()
            )
            self._status_publisher.publish(message)

        def _publish_session(self, fact: SessionInfoFact) -> None:
            message = project_session_info(
                SessionInfo(), fact, self.get_clock().now().to_msg()
            )
            self._session_publisher.publish(message)

        def close(self) -> None:
            with self._submission_lock:
                if self._closed:
                    return
                self._closed = True
                self._workflow_timer.cancel()
                self._status_timer.cancel()
                try:
                    termination_confirmed = shutdown_controller_worker(
                        self._worker, self._controller.shutdown
                    )
                except Exception as exc:
                    self.get_logger().error(
                        f"capture shutdown failed: {exc}"
                    )
                    return
                if not termination_confirmed:
                    self.get_logger().error(
                        "capture shutdown left Recorder termination unconfirmed"
                    )

    rclpy.init()
    node = CaptureControllerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the independently sessioned Recorder before shutting down the
        # ROS executor. Otherwise launch SIGINT can interrupt executor cleanup
        # and leave ros2 bag adopted by PID 1 in containers.
        node.close()
        try:
            executor.shutdown(timeout_sec=5.0)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
