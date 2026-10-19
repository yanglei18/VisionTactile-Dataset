from __future__ import annotations

import math
import time
from typing import Callable, Sequence

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray
from vt_tracker_msgs.msg import TrackerStatus

from .visualization_model import (
    FIXED_ROLES,
    PoseValue,
    RoleSnapshot,
    TrackerVisualizationModel,
    VisualHealth,
)


ROLE_COLORS = {
    "left_wrist": (0.0, 0.85, 1.0),
    "right_wrist": (1.0, 0.0, 0.85),
    "torso": (1.0, 0.5, 0.0),
}
DIAGNOSTIC_ANCHORS = {
    "left_wrist": (-0.30, 0.0, 0.10),
    "right_wrist": (0.0, 0.0, 0.10),
    "torso": (0.30, 0.0, 0.10),
}
HEALTH_COLORS = {
    VisualHealth.FRESH: (0.0, 1.0, 0.0),
    VisualHealth.DELAYED: (1.0, 0.75, 0.0),
    VisualHealth.OFFLINE: (1.0, 0.0, 0.0),
}


def _set_color(
    marker: Marker,
    color: tuple[float, float, float],
    alpha: float = 1.0,
) -> None:
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = alpha


def _base_marker(
    snapshot: RoleSnapshot,
    stamp: Time,
    kind: str,
    marker_type: int,
    offset: int,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = "vive_map"
    marker.header.stamp = stamp
    marker.ns = f"{snapshot.role}/{kind}"
    marker.id = FIXED_ROLES.index(snapshot.role) * 10 + offset
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _put_position(marker: Marker, value: tuple[float, float, float]) -> None:
    marker.pose.position.x = value[0]
    marker.pose.position.y = value[1]
    marker.pose.position.z = value[2]


def _health_marker(
    snapshot: RoleSnapshot,
    stamp: Time,
    anchor: tuple[float, float, float],
) -> Marker:
    marker = _base_marker(
        snapshot, stamp, "health", Marker.SPHERE, 2
    )
    _put_position(marker, anchor)
    marker.scale.x = 0.06
    marker.scale.y = 0.06
    marker.scale.z = 0.06
    _set_color(marker, HEALTH_COLORS[snapshot.health])
    return marker


def _label_marker(
    snapshot: RoleSnapshot,
    stamp: Time,
    anchor: tuple[float, float, float],
) -> Marker:
    marker = _base_marker(
        snapshot, stamp, "label", Marker.TEXT_VIEW_FACING, 3
    )
    _put_position(marker, anchor)
    marker.scale.z = 0.06
    _set_color(marker, HEALTH_COLORS[snapshot.health])
    if snapshot.status is None:
        counters = "valid=? invalid=? dropped=?"
    else:
        counters = (
            f"valid={snapshot.status.valid_sample_count} "
            f"invalid={snapshot.status.invalid_report_count} "
            f"dropped={snapshot.status.dropped_queue_count}"
        )
    marker.text = (
        f"{snapshot.role} {snapshot.health.value}\n"
        f"{snapshot.receive_rate_hz:.1f} Hz  {counters}"
    )
    return marker


def markers_for_snapshot(
    snapshot: RoleSnapshot, stamp: Time
) -> tuple[Marker, ...]:
    if type(snapshot) is not RoleSnapshot:
        raise TypeError("snapshot must be RoleSnapshot")
    if snapshot.role not in FIXED_ROLES:
        raise ValueError("snapshot role is not fixed")
    if snapshot.pose is None:
        anchor = DIAGNOSTIC_ANCHORS[snapshot.role]
        label_anchor = (anchor[0], anchor[1], anchor[2] + 0.10)
        return (
            _health_marker(snapshot, stamp, anchor),
            _label_marker(snapshot, stamp, label_anchor),
        )

    arrow = _base_marker(
        snapshot, stamp, "arrow", Marker.ARROW, 0
    )
    _put_position(arrow, snapshot.pose.position)
    (
        arrow.pose.orientation.x,
        arrow.pose.orientation.y,
        arrow.pose.orientation.z,
        arrow.pose.orientation.w,
    ) = snapshot.pose.orientation_xyzw
    arrow.scale.x = 0.16
    arrow.scale.y = 0.035
    arrow.scale.z = 0.035

    trail = _base_marker(
        snapshot, stamp, "trail", Marker.LINE_STRIP, 1
    )
    trail.scale.x = 0.012
    trail.points = [
        Point(x=value.position[0], y=value.position[1], z=value.position[2])
        for value in snapshot.trail
    ]

    if snapshot.health is VisualHealth.FRESH:
        role_color = ROLE_COLORS[snapshot.role]
        role_alpha = 1.0
    elif snapshot.health is VisualHealth.DELAYED:
        role_color = ROLE_COLORS[snapshot.role]
        role_alpha = 0.35
    else:
        role_color = (0.35, 0.35, 0.35)
        role_alpha = 0.35
    _set_color(arrow, role_color, role_alpha)
    _set_color(trail, role_color, role_alpha)

    position = snapshot.pose.position
    health_anchor = (position[0], position[1], position[2] + 0.12)
    label_anchor = (position[0], position[1], position[2] + 0.22)
    return (
        arrow,
        trail,
        _health_marker(snapshot, stamp, health_anchor),
        _label_marker(snapshot, stamp, label_anchor),
    )


class TrackerVisualizerNode(Node):
    def __init__(
        self,
        *,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        super().__init__("vt_vive_tracker_visualizer")
        self._monotonic_ns = monotonic_ns
        self.model = TrackerVisualizationModel(
            monotonic_ns=monotonic_ns
        )
        self._last_warning_ns: dict[tuple[str, str], int] = {}
        pose_qos = QoSProfile(
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
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tracker_subscriptions = []
        for role in FIXED_ROLES:
            self.tracker_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f"/vive/{role}/pose",
                    lambda message, role=role: self.on_pose(
                        role, message
                    ),
                    pose_qos,
                )
            )
            self.tracker_subscriptions.append(
                self.create_subscription(
                    TrackerStatus,
                    f"/vive/{role}/status",
                    lambda message, role=role: self.on_status(
                        role, message
                    ),
                    status_qos,
                )
            )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/vive/visualization/markers",
            marker_qos,
        )
        self.marker_timer = self.create_timer(
            0.05, self.publish_markers
        )

    def _warn_throttled(
        self, kind: str, role: str, message: str
    ) -> None:
        now_ns = self._monotonic_ns()
        key = (kind, role)
        previous_ns = self._last_warning_ns.get(key)
        if (
            previous_ns is not None
            and now_ns - previous_ns < 5_000_000_000
        ):
            return
        self._last_warning_ns[key] = now_ns
        self.get_logger().warning(message)

    def on_pose(self, role: str, message: PoseStamped) -> None:
        if role not in FIXED_ROLES:
            return
        if message.header.frame_id != "vive_map":
            self._warn_throttled(
                "frame",
                role,
                f"unexpected pose frame for role={role}",
            )
            return
        components = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in components):
            self._warn_throttled(
                "finite",
                role,
                f"non-finite pose rejected for role={role}",
            )
            return
        self.model.observe_pose(
            role,
            PoseValue(
                tuple(float(value) for value in components[:3]),
                tuple(float(value) for value in components[3:]),
            ),
            arrival_ns=self._monotonic_ns(),
        )

    def on_status(self, role: str, message: TrackerStatus) -> None:
        if role not in FIXED_ROLES or message.role != role:
            self._warn_throttled(
                "status",
                role,
                f"mismatched status role for expected={role}",
            )
            return
        self.model.observe_status(
            role,
            int(message.state),
            int(message.valid_sample_count),
            int(message.invalid_report_count),
            int(message.dropped_queue_count),
            tracker_id=message.tracker_id,
            tracking_status=int(message.tracking_status),
        )

    def publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = []
        for snapshot in self.model.snapshots():
            markers.extend(markers_for_snapshot(snapshot, stamp))
        self.marker_publisher.publish(MarkerArray(markers=markers))


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrackerVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
