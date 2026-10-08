from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass

from .roles import EXPECTED_ROLES


_LOWERCASE_HEX = frozenset("0123456789abcdef")


def _require_unsigned(name: str, value: object, maximum: int) -> None:
    if type(value) is not int:
        raise TypeError(f"{name} must be int")
    if not 0 <= value <= maximum:
        raise ValueError(f"{name} is out of range")


def _require_vector(
    name: str,
    value: object,
    length: int,
    *,
    finite_length: int | None = None,
) -> None:
    if type(value) is not tuple:
        raise TypeError(f"{name} must be tuple")
    if len(value) != length:
        raise ValueError(f"{name} must contain exactly {length} components")
    if any(type(component) is not float for component in value):
        raise TypeError(f"{name} components must be float")
    required = length if finite_length is None else finite_length
    if not all(math.isfinite(component) for component in value[:required]):
        raise ValueError(f"{name} components must be finite")


def _require_timestamp(name: str, value: object) -> None:
    if type(value) is not int:
        raise TypeError(f"{name} must be int")
    if value < 0:
        raise ValueError(f"{name} must not be negative")


def _is_tracker_id(value: object) -> bool:
    return (
        type(value) is str
        and len(value) == 64
        and all(character in _LOWERCASE_HEX for character in value)
    )


@dataclass(frozen=True, repr=False)
class NativePose:
    address: bytes
    packet_index: int
    tracker_index: int
    buttons: int
    position: tuple[float, float, float]
    quaternion_wzyx: tuple[float, float, float, float]
    acceleration: tuple[float, float, float]
    angular_velocity_native: tuple[float, float, float, float]
    tracking_status: int

    def __post_init__(self) -> None:
        if type(self.address) is not bytes or len(self.address) != 6:
            raise ValueError("address must contain six bytes")
        _require_unsigned("packet_index", self.packet_index, 0xFFFF)
        _require_unsigned("tracker_index", self.tracker_index, 0xFF)
        _require_unsigned("buttons", self.buttons, 0xFF)
        _require_unsigned("tracking_status", self.tracking_status, 0xFF)
        _require_vector("position", self.position, 3)
        _require_vector("quaternion_wzyx", self.quaternion_wzyx, 4)
        _require_vector("acceleration", self.acceleration, 3)
        _require_vector(
            "angular_velocity_native",
            self.angular_velocity_native,
            4,
            finite_length=3,
        )

    def __repr__(self) -> str:
        fingerprint = hashlib.sha256(self.address).hexdigest()
        return (
            "NativePose("
            f"address_fingerprint={fingerprint!r}, "
            f"packet_index={self.packet_index}, "
            f"tracker_index={self.tracker_index}, "
            f"buttons={self.buttons}, "
            f"tracking_status={self.tracking_status})"
        )


@dataclass(frozen=True)
class StampedTrackerSample:
    role: str
    tracker_id: str
    host_monotonic_ns: int
    host_realtime_ns: int
    packet_index: int
    tracking_status: int
    raw_buttons: int
    pose_valid: bool
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    acceleration: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]

    def __post_init__(self) -> None:
        if self.role not in EXPECTED_ROLES:
            raise ValueError("role is unsupported")
        if not _is_tracker_id(self.tracker_id):
            raise ValueError("tracker_id must be a lowercase SHA-256 value")
        _require_timestamp("host_monotonic_ns", self.host_monotonic_ns)
        _require_timestamp("host_realtime_ns", self.host_realtime_ns)
        _require_unsigned("packet_index", self.packet_index, 0xFFFF)
        _require_unsigned("tracking_status", self.tracking_status, 0xFF)
        _require_unsigned("raw_buttons", self.raw_buttons, 0xFF)
        if type(self.pose_valid) is not bool:
            raise TypeError("pose_valid must be bool")
        _require_vector("position", self.position, 3)
        _require_vector("quaternion_xyzw", self.quaternion_xyzw, 4)
        _require_vector("acceleration", self.acceleration, 3)
        _require_vector("angular_velocity_xyz", self.angular_velocity_xyz, 3)


def stamp_from_realtime_ns(value: int) -> tuple[int, int]:
    _require_timestamp("host_realtime_ns", value)
    return divmod(value, 1_000_000_000)


def normalize_pose(
    native: NativePose,
    *,
    role: str,
    tracker_id: str,
    host_monotonic_ns: int,
    host_realtime_ns: int,
) -> StampedTrackerSample:
    if type(native) is not NativePose:
        raise TypeError("native must be NativePose")
    NativePose.__post_init__(native)
    quaternion_xyzw = (
        native.quaternion_wzyx[3],
        native.quaternion_wzyx[2],
        native.quaternion_wzyx[1],
        native.quaternion_wzyx[0],
    )
    quaternion_norm = math.sqrt(
        sum(component * component for component in quaternion_xyzw)
    )
    pose_valid = (
        native.tracking_status & 0x0F == 2
        and 0.90 <= quaternion_norm <= 1.10
    )
    return StampedTrackerSample(
        role=role,
        tracker_id=tracker_id,
        host_monotonic_ns=host_monotonic_ns,
        host_realtime_ns=host_realtime_ns,
        packet_index=native.packet_index,
        tracking_status=native.tracking_status,
        raw_buttons=native.buttons,
        pose_valid=pose_valid,
        position=native.position,
        quaternion_xyzw=quaternion_xyzw,
        acceleration=native.acceleration,
        angular_velocity_xyz=native.angular_velocity_native[:3],
    )
