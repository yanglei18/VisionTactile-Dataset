from __future__ import annotations

import argparse
import json
import math
import os
import stat
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from vt_tracker_msgs.msg import TrackerSample, TrackerStatus


ROLES = ("left_wrist", "right_wrist", "torso")
_LOWERCASE_HEX = frozenset("0123456789abcdef")


def _stamp_ns(header: object) -> int:
    return (
        int(header.stamp.sec) * 1_000_000_000
        + int(header.stamp.nanosec)
    )


def _valid_tracker_id(value: object) -> bool:
    return (
        type(value) is str
        and len(value) == 64
        and all(character in _LOWERCASE_HEX for character in value)
    )


@dataclass
class _RoleMetrics:
    tracker_id: str = ""
    current_tracker_id: str = ""
    sample_count: int = 0
    valid_pose_count: int = 0
    pose_message_count: int = 0
    status_count: int = 0
    dropped_queue_count: int = 0
    nonmonotonic_count: int = 0
    timestamp_mismatch_count: int = 0
    last_monotonic_ns: int | None = None
    last_valid_monotonic_ns: int | None = None
    max_gap_ns: int = 0
    expected_pose_stamps: set[int] = field(default_factory=set)
    observed_pose_stamps: set[int] = field(default_factory=set)


class TopicValidationSession:
    def __init__(
        self,
        *,
        duration_seconds: float,
        minimum_rate_hz: float = 30.0,
        maximum_gap_ms: float = 100.0,
        minimum_valid_ratio: float = 0.90,
    ) -> None:
        for name, value in (
            ("duration_seconds", duration_seconds),
            ("minimum_rate_hz", minimum_rate_hz),
            ("maximum_gap_ms", maximum_gap_ms),
            ("minimum_valid_ratio", minimum_valid_ratio),
        ):
            if type(value) not in (int, float) or isinstance(value, bool):
                raise TypeError(f"{name} must be numeric")
            if not math.isfinite(float(value)) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if minimum_valid_ratio > 1.0:
            raise ValueError("minimum_valid_ratio must not exceed one")
        self.duration_seconds = float(duration_seconds)
        self.minimum_rate_hz = float(minimum_rate_hz)
        self.maximum_gap_ms = float(maximum_gap_ms)
        self.minimum_valid_ratio = float(minimum_valid_ratio)
        self._values = {role: _RoleMetrics() for role in ROLES}
        self._identity_swap_count = 0

    def _metrics(self, topic_role: str) -> _RoleMetrics:
        try:
            return self._values[topic_role]
        except KeyError:
            raise ValueError("unexpected tracker topic role") from None

    def _observe_identity(
        self, topic_role: str, message_role: str, tracker_id: str
    ) -> None:
        value = self._metrics(topic_role)
        identity = tracker_id if _valid_tracker_id(tracker_id) else ""
        if message_role != topic_role or not identity:
            self._identity_swap_count += 1
            return
        if not value.tracker_id:
            value.tracker_id = identity
            value.current_tracker_id = identity
        elif identity != value.current_tracker_id:
            self._identity_swap_count += 1
            value.current_tracker_id = identity

    def observe_sample(self, topic_role: str, message: object) -> None:
        value = self._metrics(topic_role)
        value.sample_count += 1
        self._observe_identity(
            topic_role, message.role, message.tracker_id
        )
        monotonic_ns = int(message.host_monotonic_ns)
        if (
            value.last_monotonic_ns is not None
            and monotonic_ns <= value.last_monotonic_ns
        ):
            value.nonmonotonic_count += 1
        else:
            value.last_monotonic_ns = monotonic_ns
        if _stamp_ns(message.header) != int(message.host_realtime_ns):
            value.timestamp_mismatch_count += 1
        if bool(message.pose_valid):
            value.valid_pose_count += 1
            value.expected_pose_stamps.add(_stamp_ns(message.header))
            if (
                value.last_valid_monotonic_ns is not None
                and monotonic_ns > value.last_valid_monotonic_ns
            ):
                value.max_gap_ns = max(
                    value.max_gap_ns,
                    monotonic_ns - value.last_valid_monotonic_ns,
                )
            if (
                value.last_valid_monotonic_ns is None
                or monotonic_ns > value.last_valid_monotonic_ns
            ):
                value.last_valid_monotonic_ns = monotonic_ns

    def observe_pose(self, topic_role: str, header: object) -> None:
        value = self._metrics(topic_role)
        value.pose_message_count += 1
        value.observed_pose_stamps.add(_stamp_ns(header))

    def observe_status(self, topic_role: str, message: object) -> None:
        value = self._metrics(topic_role)
        value.status_count += 1
        self._observe_identity(
            topic_role, message.role, message.tracker_id
        )
        value.dropped_queue_count = max(
            value.dropped_queue_count,
            int(message.dropped_queue_count),
        )

    def report(self) -> dict[str, object]:
        identities = [
            value.tracker_id
            for value in self._values.values()
            if value.tracker_id
        ]
        identity_collision_count = len(identities) - len(set(identities))
        role_reports: dict[str, dict[str, object]] = {}
        passed = (
            self._identity_swap_count == 0
            and identity_collision_count == 0
        )
        for role in ROLES:
            value = self._values[role]
            valid_rate_hz = (
                value.valid_pose_count / self.duration_seconds
            )
            valid_ratio = (
                value.valid_pose_count / value.sample_count
                if value.sample_count
                else 0.0
            )
            max_gap_ms = value.max_gap_ns / 1_000_000.0
            matched_pose_count = len(
                value.expected_pose_stamps & value.observed_pose_stamps
            )
            pose_rate_hz = matched_pose_count / self.duration_seconds
            role_passed = (
                bool(value.tracker_id)
                and value.sample_count > 0
                and value.status_count > 0
                and valid_rate_hz >= self.minimum_rate_hz
                and pose_rate_hz >= self.minimum_rate_hz
                and valid_ratio >= self.minimum_valid_ratio
                and max_gap_ms <= self.maximum_gap_ms
                and value.dropped_queue_count == 0
                and value.nonmonotonic_count == 0
                and value.timestamp_mismatch_count == 0
            )
            passed &= role_passed
            role_reports[role] = {
                "tracker_id": value.tracker_id,
                "sample_count": value.sample_count,
                "valid_pose_count": value.valid_pose_count,
                "pose_message_count": value.pose_message_count,
                "matched_pose_count": matched_pose_count,
                "status_count": value.status_count,
                "valid_rate_hz": round(valid_rate_hz, 6),
                "pose_rate_hz": round(pose_rate_hz, 6),
                "valid_ratio": round(valid_ratio, 6),
                "max_gap_ms": round(max_gap_ms, 6),
                "dropped_queue_count": value.dropped_queue_count,
                "nonmonotonic_count": value.nonmonotonic_count,
                "timestamp_mismatch_count": (
                    value.timestamp_mismatch_count
                ),
            }
        return {
            "verdict": "PASS" if passed else "FAIL",
            "roles": role_reports,
            "identity_swap_count": self._identity_swap_count,
            "identity_collision_count": identity_collision_count,
        }


class _ValidationNode(Node):
    def __init__(self, session: TopicValidationSession) -> None:
        super().__init__("vt_vive_tracker_validator")
        sample_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tracker_subscriptions = []
        for role in ROLES:
            self._tracker_subscriptions.extend(
                (
                    self.create_subscription(
                        TrackerSample,
                        f"/vive/{role}/sample",
                        lambda message, role=role: session.observe_sample(
                            role, message
                        ),
                        sample_qos,
                    ),
                    self.create_subscription(
                        PoseStamped,
                        f"/vive/{role}/pose",
                        lambda message, role=role: session.observe_pose(
                            role, message.header
                        ),
                        sample_qos,
                    ),
                    self.create_subscription(
                        TrackerStatus,
                        f"/vive/{role}/status",
                        lambda message, role=role: session.observe_status(
                            role, message
                        ),
                        status_qos,
                    ),
                )
            )


def _is_inside_git_worktree(path: Path) -> bool:
    return any((ancestor / ".git").exists() for ancestor in path.parents)


def _validate_output_path(path: Path) -> Path:
    if not path.is_absolute():
        raise ValueError("output must be an absolute path")
    if path.is_symlink():
        raise ValueError("output must not be a symlink")
    resolved = path.resolve(strict=False)
    if _is_inside_git_worktree(resolved):
        raise ValueError("output must be outside the Git worktree")
    return resolved


def _write_private_report(path: Path, report: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
    )
    temporary_path = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                report,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        directory_descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        temporary_path.unlink(missing_ok=True)
        raise
    if stat.S_IMODE(path.stat().st_mode) != 0o600:
        raise PermissionError("validation report is not private")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate three VIVE Tracker ROS 2 streams."
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    output = _validate_output_path(arguments.output)
    session = TopicValidationSession(duration_seconds=arguments.duration)
    rclpy.init()
    node = _ValidationNode(session)
    try:
        deadline = time.monotonic() + arguments.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report = session.report()
    _write_private_report(output, report)
    dropped = sum(
        value["dropped_queue_count"]
        for value in report["roles"].values()
    )
    print(
        f"status={report['verdict']} roles={len(ROLES)} "
        f"identity_swaps={report['identity_swap_count']} "
        f"dropped={dropped}"
    )
    return 0 if report["verdict"] == "PASS" else 4
