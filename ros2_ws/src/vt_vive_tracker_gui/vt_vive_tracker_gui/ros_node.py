from __future__ import annotations

import math
import time
from typing import Callable

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from vt_tracker_msgs.msg import TrackerStatus

from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    PoseValue,
    TrackerVisualizationModel,
)

from .snapshot_store import LatestSnapshotStore


class TrackerGuiNode(Node):
    def __init__(
        self,
        store: LatestSnapshotStore,
        *,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        super().__init__(
            "vt_vive_tracker_gui",
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.store = store
        self._monotonic_ns = monotonic_ns
        self.model = TrackerVisualizationModel(monotonic_ns=monotonic_ns)
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
        self.tracker_subscriptions = []
        for role in FIXED_ROLES:
            self.tracker_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f"/vive/{role}/pose",
                    lambda message, role=role: self.on_pose(role, message),
                    pose_qos,
                )
            )
            self.tracker_subscriptions.append(
                self.create_subscription(
                    TrackerStatus,
                    f"/vive/{role}/status",
                    lambda message, role=role: self.on_status(role, message),
                    status_qos,
                )
            )
        self.snapshot_timer = self.create_timer(0.05, self.publish_snapshot)

    def _warn_throttled(
        self, kind: str, role: str, message: str
    ) -> None:
        now_ns = self._monotonic_ns()
        key = (kind, role)
        previous_ns = self._last_warning_ns.get(key)
        if previous_ns is not None and now_ns - previous_ns < 5_000_000_000:
            return
        self._last_warning_ns[key] = now_ns
        self.get_logger().warning(message)
        self.store.publish_diagnostic(message, now_ns)

    def on_pose(self, role: str, message: PoseStamped) -> None:
        if role not in FIXED_ROLES or message.header.frame_id != "vive_map":
            self._warn_throttled(
                "pose_frame", role, "unexpected pose frame"
            )
            return
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self._warn_throttled(
                "pose_finite", role, "non-finite pose rejected"
            )
            return
        now_ns = self._monotonic_ns()
        if self.model.observe_pose(
            role,
            PoseValue(
                tuple(map(float, values[:3])),
                tuple(map(float, values[3:])),
            ),
            arrival_ns=now_ns,
        ):
            self.store.publish(self.model.snapshots(now_ns))

    def on_status(self, role: str, message: TrackerStatus) -> None:
        if role not in FIXED_ROLES or message.role != role:
            self._warn_throttled(
                "status_role", role, "mismatched status role"
            )
            return
        if self.model.observe_status(
            role,
            int(message.state),
            int(message.valid_sample_count),
            int(message.invalid_report_count),
            int(message.dropped_queue_count),
            tracker_id=message.tracker_id,
            tracking_status=int(message.tracking_status),
        ):
            now_ns = self._monotonic_ns()
            self.store.publish(self.model.snapshots(now_ns))
        else:
            self._warn_throttled(
                "status_value", role, "invalid status rejected"
            )

    def publish_snapshot(self) -> None:
        now_ns = self._monotonic_ns()
        self.store.publish(self.model.snapshots(now_ns))
