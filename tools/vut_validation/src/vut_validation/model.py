from __future__ import annotations

from dataclasses import dataclass
import math


def canonical_tracker_id(mac: str) -> str:
    values = [int(part, 16) for part in mac.split(":")]
    if len(values) != 6:
        raise ValueError("MAC must contain six octets")
    if not any(values):
        return "usb-direct"
    values[1] &= 0xF8
    return ":".join(f"{value:02x}" for value in values)


@dataclass(frozen=True)
class PoseSample:
    tracker_id: str
    host_monotonic_ns: int
    host_realtime_ns: int
    upstream_timestamp_ms: int
    position: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    acceleration: tuple[float, float, float]
    angular_velocity: tuple[float, float, float, float]
    tracking_status: int
    buttons: int

    def __post_init__(self) -> None:
        values = (
            *self.position,
            *self.quaternion_wxyz,
            *self.acceleration,
            *self.angular_velocity,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose fields must be finite")


@dataclass(frozen=True)
class ValidationThresholds:
    duration_s: float = 300.0
    min_hz: float = 30.0
    max_gap_ms: float = 100.0
    full_tracking_status: int = 2
    quaternion_norm_min: float = 0.90
    quaternion_norm_max: float = 1.10
