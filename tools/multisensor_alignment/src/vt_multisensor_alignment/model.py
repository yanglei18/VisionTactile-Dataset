from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Mapping

import numpy as np


_TRACKER_ID = re.compile(r"[0-9a-f]{64}")


def _timestamp(value: object, name: str = "timestamp_ns") -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


@dataclass(frozen=True)
class Transform:
    translation: np.ndarray
    quaternion_xyzw: np.ndarray

    def __post_init__(self) -> None:
        translation = np.asarray(self.translation, dtype=np.float64).reshape(-1)
        quaternion = np.asarray(self.quaternion_xyzw, dtype=np.float64).reshape(-1)
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError("translation must contain three finite values")
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError("quaternion must contain four finite values")
        norm = float(np.linalg.norm(quaternion))
        if not 0.90 <= norm <= 1.10:
            raise ValueError("quaternion norm is outside the accepted range")
        translation = translation.copy()
        quaternion = quaternion / norm
        translation.setflags(write=False)
        quaternion.setflags(write=False)
        object.__setattr__(self, "translation", translation)
        object.__setattr__(self, "quaternion_xyzw", quaternion)

    @classmethod
    def identity(cls) -> Transform:
        return cls(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))

    def compose(self, child: Transform) -> Transform:
        if not isinstance(child, Transform):
            raise TypeError("child must be Transform")
        rotated = _rotate_vector(self.quaternion_xyzw, child.translation)
        return Transform(
            self.translation + rotated,
            _quaternion_multiply(self.quaternion_xyzw, child.quaternion_xyzw),
        )

    def interpolate(self, other: Transform, fraction: float) -> Transform:
        if not isinstance(other, Transform):
            raise TypeError("other must be Transform")
        if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
            raise ValueError("fraction must be finite and between zero and one")
        translation = self.translation + fraction * (
            other.translation - self.translation
        )
        quaternion = _slerp(
            self.quaternion_xyzw, other.quaternion_xyzw, fraction
        )
        return Transform(translation, quaternion)

    def as_matrix(self) -> np.ndarray:
        """Return this parent-from-child transform as a read-only 4x4 matrix."""
        x, y, z, w = self.quaternion_xyzw
        matrix = np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                    self.translation[0],
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                    self.translation[1],
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                    self.translation[2],
                ],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        matrix.setflags(write=False)
        return matrix

    def as_document(self) -> dict[str, list[float]]:
        return {
            "translation_m": [float(value) for value in self.translation],
            "quaternion_xyzw": [
                float(value) for value in self.quaternion_xyzw
            ],
        }


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )


def _rotate_vector(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    xyz = quaternion[:3]
    scalar = quaternion[3]
    return (
        2.0 * np.dot(xyz, vector) * xyz
        + (scalar * scalar - np.dot(xyz, xyz)) * vector
        + 2.0 * scalar * np.cross(xyz, vector)
    )


def _slerp(left: np.ndarray, right: np.ndarray, fraction: float) -> np.ndarray:
    target = right.copy()
    dot = float(np.dot(left, target))
    if dot < 0.0:
        target = -target
        dot = -dot
    dot = min(max(dot, -1.0), 1.0)
    if dot > 0.9995:
        value = left + fraction * (target - left)
        return value / np.linalg.norm(value)
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) / sine * left
        + math.sin(fraction * angle) / sine * target
    )


@dataclass(frozen=True)
class MessageRef:
    topic: str
    sequence: int
    bag_timestamp_ns: int
    source_timestamp_ns: int

    def __post_init__(self) -> None:
        if not self.topic.startswith("/"):
            raise ValueError("topic must be absolute")
        _timestamp(self.sequence, "sequence")
        _timestamp(self.bag_timestamp_ns, "bag_timestamp_ns")
        _timestamp(self.source_timestamp_ns, "source_timestamp_ns")

    def as_document(self) -> dict[str, object]:
        return {
            "topic": self.topic,
            "sequence": self.sequence,
            "bag_timestamp_ns": self.bag_timestamp_ns,
            "source_timestamp_ns": self.source_timestamp_ns,
        }


@dataclass(frozen=True)
class ClockObservation:
    realtime_ns: int
    monotonic_ns: int

    def __post_init__(self) -> None:
        _timestamp(self.realtime_ns, "realtime_ns")
        _timestamp(self.monotonic_ns, "monotonic_ns")


@dataclass(frozen=True)
class CameraFrame:
    camera_name: str
    source_timestamp_ns: int
    host_realtime_ns: int
    host_monotonic_ns: int
    color: MessageRef
    depth: MessageRef
    timing: MessageRef

    def __post_init__(self) -> None:
        if not self.camera_name:
            raise ValueError("camera_name must not be empty")
        _timestamp(self.source_timestamp_ns, "source_timestamp_ns")
        _timestamp(self.host_realtime_ns, "host_realtime_ns")
        _timestamp(self.host_monotonic_ns, "host_monotonic_ns")
        references = (self.color, self.depth, self.timing)
        if any(not isinstance(value, MessageRef) for value in references):
            raise TypeError("color, depth, and timing must be MessageRef values")
        if any(
            value.source_timestamp_ns != self.source_timestamp_ns
            for value in references
        ):
            raise ValueError("camera references must use the same source timestamp")


@dataclass(frozen=True)
class TimedPose:
    role: str
    tracker_id: str
    host_realtime_ns: int
    host_monotonic_ns: int
    transform: Transform
    reference: MessageRef

    def __post_init__(self) -> None:
        if not self.role:
            raise ValueError("role must not be empty")
        if _TRACKER_ID.fullmatch(self.tracker_id) is None:
            raise ValueError("tracker_id must be a lowercase SHA-256 value")
        _timestamp(self.host_realtime_ns, "host_realtime_ns")
        _timestamp(self.host_monotonic_ns, "host_monotonic_ns")
        if not isinstance(self.transform, Transform):
            raise TypeError("transform must be Transform")


@dataclass(frozen=True)
class InterpolatedPose:
    timestamp_ns: int
    transform: Transform
    bracket_gap_ns: int
    before_sequence: int
    after_sequence: int

    def __post_init__(self) -> None:
        _timestamp(self.timestamp_ns)
        if not isinstance(self.transform, Transform):
            raise TypeError("transform must be Transform")
        _timestamp(self.bracket_gap_ns, "bracket_gap_ns")
        _timestamp(self.before_sequence, "before_sequence")
        _timestamp(self.after_sequence, "after_sequence")
        if self.before_sequence > self.after_sequence:
            raise ValueError("before_sequence must not exceed after_sequence")


@dataclass(frozen=True)
class GenericSample:
    stream_name: str
    timestamp_ns: int
    reference: MessageRef

    def __post_init__(self) -> None:
        if not self.stream_name:
            raise ValueError("stream_name must not be empty")
        _timestamp(self.timestamp_ns)
        if not isinstance(self.reference, MessageRef):
            raise TypeError("reference must be MessageRef")
        if self.reference.source_timestamp_ns != self.timestamp_ns:
            raise ValueError("reference source timestamp must match sample timestamp")


@dataclass(frozen=True)
class TimingRecord:
    source_timestamp_ns: int
    host_realtime_ns: int
    host_monotonic_ns: int
    reference: MessageRef

    def __post_init__(self) -> None:
        _timestamp(self.source_timestamp_ns, "source_timestamp_ns")
        _timestamp(self.host_realtime_ns, "host_realtime_ns")
        _timestamp(self.host_monotonic_ns, "host_monotonic_ns")
        if not isinstance(self.reference, MessageRef):
            raise TypeError("reference must be MessageRef")
        if self.reference.source_timestamp_ns != self.source_timestamp_ns:
            raise ValueError("timing reference source timestamp mismatch")


@dataclass(frozen=True)
class BagDataset:
    bag_path: Path
    storage_identifier: str
    topic_types: Mapping[str, str]
    message_counts: Mapping[str, int]
    accepted_counts: Mapping[str, int]
    camera_frames: Mapping[str, tuple[CameraFrame, ...]]
    tracker_poses: Mapping[str, tuple[TimedPose, ...]]
    additional_samples: Mapping[str, tuple[GenericSample, ...]]
    tracker_ids: Mapping[str, str]
    incomplete_camera_groups: Mapping[str, int]
    clock_observations: Mapping[str, tuple[ClockObservation, ...]]

    def __post_init__(self) -> None:
        if not isinstance(self.bag_path, Path) or not self.bag_path.is_absolute():
            raise ValueError("bag_path must be an absolute Path")
        if not self.storage_identifier:
            raise ValueError("storage_identifier must not be empty")
