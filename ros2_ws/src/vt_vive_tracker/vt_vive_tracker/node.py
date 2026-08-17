from __future__ import annotations

import fcntl
import os
import stat
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header
from vt_tracker_msgs.msg import TrackerSample, TrackerStatus

from .health import BoundedSampleQueue, TrackerHealthBook
from .model import StampedTrackerSample, stamp_from_realtime_ns
from .pose_source import ReadOnlyPoseSource
from .roles import load_role_map


@dataclass(frozen=True)
class RuntimeConfig:
    bundle_path: Path
    role_map_path: Path
    frame_id: str
    status_rate_hz: float
    read_timeout_ms: int
    disconnect_timeout_ms: int
    queue_capacity: int


@dataclass
class PublisherGroup:
    sample: object
    pose: object
    status: object


def make_runtime_config(
    *,
    bundle_path: object,
    role_map_path: object,
    frame_id: object,
    status_rate_hz: object,
    read_timeout_ms: object,
    disconnect_timeout_ms: object,
    queue_capacity: object,
    use_sim_time: object,
) -> RuntimeConfig:
    if type(use_sim_time) is not bool:
        raise TypeError("use_sim_time must be bool")
    if use_sim_time:
        raise ValueError("use_sim_time=true is forbidden for hardware input")
    if type(bundle_path) is not str or not bundle_path:
        raise ValueError("bundle_path is required")
    if type(role_map_path) is not str or not role_map_path:
        raise ValueError("role_map_path is required")
    if type(frame_id) is not str or not frame_id:
        raise ValueError("frame_id is required")
    if type(status_rate_hz) not in (int, float) or isinstance(
        status_rate_hz, bool
    ):
        raise TypeError("status_rate_hz must be numeric")
    if not 0.1 <= float(status_rate_hz) <= 100.0:
        raise ValueError("status_rate_hz is out of range")
    for name, value, minimum, maximum in (
        ("read_timeout_ms", read_timeout_ms, 1, 100),
        ("disconnect_timeout_ms", disconnect_timeout_ms, 1, 60_000),
        ("queue_capacity", queue_capacity, 1, 1_000_000),
    ):
        if type(value) is not int:
            raise TypeError(f"{name} must be int")
        if not minimum <= value <= maximum:
            raise ValueError(f"{name} is out of range")
    return RuntimeConfig(
        bundle_path=Path(bundle_path).expanduser(),
        role_map_path=Path(role_map_path).expanduser(),
        frame_id=frame_id,
        status_rate_hz=float(status_rate_hz),
        read_timeout_ms=read_timeout_ms,
        disconnect_timeout_ms=disconnect_timeout_ms,
        queue_capacity=queue_capacity,
    )


def _validate_private_file(path: Path, name: str) -> None:
    metadata = path.lstat()
    if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise ValueError(f"{name} must be a regular file")
    if metadata.st_uid != os.getuid():
        raise PermissionError(f"{name} must be owned by the current user")
    if stat.S_IMODE(metadata.st_mode) & 0o077:
        raise PermissionError(f"{name} must have owner-only permissions")


class ViveTrackerNode(Node):
    def __init__(
        self,
        *,
        parameter_overrides: Sequence[object] | None = None,
        source_factory: Callable[..., object] = ReadOnlyPoseSource,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        realtime_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        super().__init__(
            "vt_vive_tracker",
            parameter_overrides=parameter_overrides,
        )
        self._destroyed = False
        self._source = None
        self._source_started = False
        self._lock_descriptor: int | None = None
        self.lock_path: Path | None = None
        self._monotonic_ns = monotonic_ns
        self._realtime_ns = realtime_ns
        self._drain_timer = None
        self._status_timer = None

        self.declare_parameter("bundle_path", "")
        self.declare_parameter("role_map_path", "")
        self.declare_parameter("frame_id", "vive_map")
        self.declare_parameter("status_rate_hz", 10.0)
        self.declare_parameter("read_timeout_ms", 100)
        self.declare_parameter("disconnect_timeout_ms", 1000)
        self.declare_parameter("queue_capacity", 4096)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        config = make_runtime_config(
            bundle_path=self.get_parameter("bundle_path").value,
            role_map_path=self.get_parameter("role_map_path").value,
            frame_id=self.get_parameter("frame_id").value,
            status_rate_hz=self.get_parameter("status_rate_hz").value,
            read_timeout_ms=self.get_parameter("read_timeout_ms").value,
            disconnect_timeout_ms=self.get_parameter(
                "disconnect_timeout_ms"
            ).value,
            queue_capacity=self.get_parameter("queue_capacity").value,
            use_sim_time=self.get_parameter("use_sim_time").value,
        )
        self._config = config
        _validate_private_file(config.bundle_path, "bundle_path")
        _validate_private_file(config.role_map_path, "role_map_path")
        role_map = load_role_map(config.role_map_path)
        self._health = TrackerHealthBook(
            role_map,
            disconnect_timeout_ns=config.disconnect_timeout_ms * 1_000_000,
        )
        self._queue = BoundedSampleQueue(
            self._health, capacity=config.queue_capacity
        )

        sample_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher_groups = {
            role: PublisherGroup(
                sample=self.create_publisher(
                    TrackerSample, f"/vive/{role}/sample", sample_qos
                ),
                pose=self.create_publisher(
                    PoseStamped, f"/vive/{role}/pose", sample_qos
                ),
                status=self.create_publisher(
                    TrackerStatus, f"/vive/{role}/status", status_qos
                ),
            )
            for role in sorted(role_map.by_role)
        }

        try:
            self._acquire_lock()
            self._drain_timer = self.create_timer(0.001, self.drain_once)
            self._status_timer = self.create_timer(
                1.0 / config.status_rate_hz,
                self.publish_status_once,
            )
            self._source = source_factory(
                config.bundle_path,
                role_map,
                monotonic_ns=monotonic_ns,
                realtime_ns=realtime_ns,
                read_timeout_ms=config.read_timeout_ms,
            )
            self._source.start(
                self._on_sample,
                self._on_invalid,
                self._on_error,
            )
            self._source_started = True
        except BaseException:
            self._cleanup_runtime()
            raise

    def _acquire_lock(self) -> None:
        runtime_value = os.environ.get("XDG_RUNTIME_DIR")
        runtime_dir = (
            Path(runtime_value)
            if runtime_value
            else Path(f"/run/user/{os.getuid()}")
        )
        metadata = runtime_dir.stat()
        if (
            not stat.S_ISDIR(metadata.st_mode)
            or metadata.st_uid != os.getuid()
            or stat.S_IMODE(metadata.st_mode) & 0o077
        ):
            raise PermissionError("XDG runtime directory is not private")
        path = runtime_dir / "vt_vive_tracker.lock"
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(path, flags, 0o600)
        try:
            metadata = os.fstat(descriptor)
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_uid != os.getuid()
                or stat.S_IMODE(metadata.st_mode) & 0o077
            ):
                raise PermissionError("tracker lock file is not private")
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(descriptor)
            raise
        self._lock_descriptor = descriptor
        self.lock_path = path

    def _on_sample(self, sample: StampedTrackerSample) -> None:
        self._health.observe_sample(sample)
        self._queue.put(sample)

    def _on_invalid(
        self, role: str, tracker_id: str, monotonic_ns: int
    ) -> None:
        self._health.observe_invalid(role, tracker_id, monotonic_ns)

    def _on_error(self, error: Exception) -> None:
        self.get_logger().error(str(error))

    def _header(self, realtime_ns: int) -> Header:
        seconds, nanoseconds = stamp_from_realtime_ns(realtime_ns)
        header = Header()
        header.stamp.sec = seconds
        header.stamp.nanosec = nanoseconds
        header.frame_id = self._config.frame_id
        return header

    def _sample_message(
        self, sample: StampedTrackerSample
    ) -> TrackerSample:
        message = TrackerSample()
        message.header = self._header(sample.host_realtime_ns)
        message.role = sample.role
        message.tracker_id = sample.tracker_id
        message.host_monotonic_ns = sample.host_monotonic_ns
        message.host_realtime_ns = sample.host_realtime_ns
        message.packet_index = sample.packet_index
        message.tracking_status = sample.tracking_status
        message.raw_buttons = sample.raw_buttons
        message.pose_valid = sample.pose_valid
        message.pose.position.x = sample.position[0]
        message.pose.position.y = sample.position[1]
        message.pose.position.z = sample.position[2]
        message.pose.orientation.x = sample.quaternion_xyzw[0]
        message.pose.orientation.y = sample.quaternion_xyzw[1]
        message.pose.orientation.z = sample.quaternion_xyzw[2]
        message.pose.orientation.w = sample.quaternion_xyzw[3]
        message.linear_acceleration.x = sample.acceleration[0]
        message.linear_acceleration.y = sample.acceleration[1]
        message.linear_acceleration.z = sample.acceleration[2]
        message.angular_velocity.x = sample.angular_velocity_xyz[0]
        message.angular_velocity.y = sample.angular_velocity_xyz[1]
        message.angular_velocity.z = sample.angular_velocity_xyz[2]
        return message

    def drain_once(self) -> None:
        for sample in self._queue.drain():
            group = self.publisher_groups[sample.role]
            sample_message = self._sample_message(sample)
            group.sample.publish(sample_message)
            if sample.pose_valid:
                pose_message = PoseStamped()
                pose_message.header = sample_message.header
                pose_message.pose = sample_message.pose
                group.pose.publish(pose_message)
        if self._health.consume_status_publish_request():
            self.publish_status_once()

    def publish_status_once(self) -> None:
        now_monotonic_ns = self._monotonic_ns()
        header = self._header(self._realtime_ns())
        for snapshot in self._health.snapshot(now_monotonic_ns):
            message = TrackerStatus()
            message.header = header
            message.role = snapshot.role
            message.tracker_id = snapshot.tracker_id
            message.state = snapshot.state
            message.tracking_status = snapshot.tracking_status
            message.valid_sample_count = snapshot.valid_sample_count
            message.invalid_report_count = snapshot.invalid_report_count
            message.dropped_queue_count = snapshot.dropped_queue_count
            message.last_report_monotonic_ns = (
                snapshot.last_report_monotonic_ns
            )
            message.last_valid_pose_monotonic_ns = (
                snapshot.last_valid_pose_monotonic_ns
            )
            self.publisher_groups[snapshot.role].status.publish(message)
        self._health.consume_status_publish_request()

    def _cleanup_runtime(self) -> None:
        for timer in (self._drain_timer, self._status_timer):
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
        self._drain_timer = None
        self._status_timer = None
        if self._source is not None and self._source_started:
            self._source.stop()
            self._source_started = False
        if self._lock_descriptor is not None:
            try:
                fcntl.flock(self._lock_descriptor, fcntl.LOCK_UN)
            finally:
                os.close(self._lock_descriptor)
                self._lock_descriptor = None

    def destroy_node(self) -> bool:
        if self._destroyed:
            return True
        self._destroyed = True
        self._cleanup_runtime()
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ViveTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
