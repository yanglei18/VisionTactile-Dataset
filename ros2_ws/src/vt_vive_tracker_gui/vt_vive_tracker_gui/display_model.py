from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    RoleSnapshot,
    VisualHealth,
)


class OverallState(Enum):
    LIVE = "LIVE"
    DEGRADED = "DEGRADED"
    DISCONNECTED = "DISCONNECTED"


@dataclass(frozen=True)
class TrackerCardModel:
    role: str
    tracker_id: str
    health: VisualHealth
    position: tuple[str, str, str]
    quaternion: tuple[str, str, str, str]
    rpy_degrees: tuple[str, str, str]
    rate: str
    age: str
    counters: str


def overall_state(roles: tuple[RoleSnapshot, ...]) -> OverallState:
    role_names = (
        tuple(item.role for item in roles)
        if type(roles) is tuple
        else ()
    )
    if role_names != FIXED_ROLES:
        raise ValueError("roles must use the fixed role order")

    health_values = tuple(item.health for item in roles)
    if all(value is VisualHealth.FRESH for value in health_values):
        return OverallState.LIVE
    if all(value is VisualHealth.OFFLINE for value in health_values):
        return OverallState.DISCONNECTED
    return OverallState.DEGRADED


def quaternion_to_rpy_degrees(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    if type(quaternion) is not tuple or len(quaternion) != 4:
        raise ValueError("quaternion must contain four finite values")
    try:
        values = tuple(float(value) for value in quaternion)
    except (TypeError, ValueError) as error:
        message = "quaternion must contain four finite values"
        raise ValueError(message) from error
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite values")

    scale = max(abs(value) for value in values)
    if scale == 0.0:
        raise ValueError("quaternion norm must be non-zero")
    scaled_values = tuple(value / scale for value in values)
    scaled_norm = math.sqrt(
        sum(value * value for value in scaled_values)
    )
    x, y, z, w = (
        value / scaled_norm for value in scaled_values
    )

    roll = math.atan2(
        2 * (w * x + y * z),
        1 - 2 * (x * x + y * y),
    )
    sin_pitch = 2 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2, sin_pitch)
        if abs(sin_pitch) >= 1
        else math.asin(sin_pitch)
    )
    yaw = math.atan2(
        2 * (w * z + x * y),
        1 - 2 * (y * y + z * z),
    )
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def card_for_snapshot(snapshot: RoleSnapshot) -> TrackerCardModel:
    if snapshot.pose is None:
        position = ("—", "—", "—")
        quaternion = ("—", "—", "—", "—")
        rpy_degrees = ("—", "—", "—")
    else:
        position = tuple(f"{value:.4f}" for value in snapshot.pose.position)
        quaternion = tuple(
            f"{value:.5f}" for value in snapshot.pose.orientation_xyzw
        )
        rpy_degrees = tuple(
            f"{value:.2f}°"
            for value in quaternion_to_rpy_degrees(
                snapshot.pose.orientation_xyzw
            )
        )

    if snapshot.status is None:
        tracker_id = "—"
        counters = "valid — · invalid — · dropped —"
    else:
        tracker_id = snapshot.status.tracker_id or "—"
        counters = (
            f"valid {snapshot.status.valid_sample_count} · "
            f"invalid {snapshot.status.invalid_report_count} · "
            f"dropped {snapshot.status.dropped_queue_count}"
        )

    if snapshot.pose is None or snapshot.pose_age_ns is None:
        age = "—"
    elif snapshot.pose_age_ns < 1_000_000_000:
        age = f"{snapshot.pose_age_ns / 1_000_000:.1f} ms"
    else:
        age = f"{snapshot.pose_age_ns / 1_000_000_000:.1f} s"

    return TrackerCardModel(
        role=snapshot.role,
        tracker_id=tracker_id,
        health=snapshot.health,
        position=position,
        quaternion=quaternion,
        rpy_degrees=rpy_degrees,
        rate=f"{snapshot.receive_rate_hz:.1f} Hz",
        age=age,
        counters=counters,
    )
