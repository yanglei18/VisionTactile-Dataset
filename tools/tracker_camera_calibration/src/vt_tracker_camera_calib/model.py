from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .transforms import Transform


def _require_timestamp(timestamp_ns: object) -> int:
    if type(timestamp_ns) is not int or timestamp_ns < 0:
        raise ValueError("timestamp_ns must be a non-negative integer")
    return timestamp_ns


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    camera_matrix: np.ndarray
    distortion: np.ndarray
    distortion_model: str = "plumb_bob"

    def __post_init__(self) -> None:
        if type(self.width) is not int or self.width <= 0:
            raise ValueError("width must be a positive integer")
        if type(self.height) is not int or self.height <= 0:
            raise ValueError("height must be a positive integer")
        matrix = np.asarray(self.camera_matrix, dtype=np.float64)
        distortion = np.asarray(self.distortion, dtype=np.float64).reshape(-1)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("camera_matrix must be a finite 3x3 matrix")
        if distortion.size not in {0, 4, 5, 8, 12, 14}:
            raise ValueError("distortion must use an OpenCV-supported length")
        if not np.all(np.isfinite(distortion)):
            raise ValueError("distortion must contain only finite values")
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if not 0.0 <= matrix[0, 2] < self.width:
            raise ValueError("camera principal point x must lie inside the image")
        if not 0.0 <= matrix[1, 2] < self.height:
            raise ValueError("camera principal point y must lie inside the image")
        if not np.allclose(matrix[2], [0.0, 0.0, 1.0], atol=1e-12):
            raise ValueError("camera_matrix final row must be [0, 0, 1]")
        matrix = matrix.copy()
        distortion = distortion.copy()
        matrix.setflags(write=False)
        distortion.setflags(write=False)
        object.__setattr__(self, "camera_matrix", matrix)
        object.__setattr__(self, "distortion", distortion)
        if not self.distortion_model:
            raise ValueError("distortion_model must not be empty")


@dataclass(frozen=True)
class TimedTransform:
    timestamp_ns: int
    transform: Transform

    def __post_init__(self) -> None:
        _require_timestamp(self.timestamp_ns)
        if not isinstance(self.transform, Transform):
            raise TypeError("transform must be Transform")


@dataclass(frozen=True)
class BoardObservation:
    timestamp_ns: int
    camera_from_board: Transform
    reprojection_rms_px: float
    corner_count: int
    source_stamp_ns: int

    def __post_init__(self) -> None:
        _require_timestamp(self.timestamp_ns)
        _require_timestamp(self.source_stamp_ns)
        if not isinstance(self.camera_from_board, Transform):
            raise TypeError("camera_from_board must be Transform")
        if not math.isfinite(self.reprojection_rms_px) or self.reprojection_rms_px < 0:
            raise ValueError("reprojection_rms_px must be finite and non-negative")
        if type(self.corner_count) is not int or self.corner_count < 4:
            raise ValueError("corner_count must be at least four")


@dataclass(frozen=True)
class CalibrationPair:
    timestamp_ns: int
    world_from_tracker: Transform
    camera_from_board: Transform
    reprojection_rms_px: float
    corner_count: int
    interpolation_gap_ns: int
    local_translation_motion_m: float
    local_rotation_motion_deg: float

    def __post_init__(self) -> None:
        _require_timestamp(self.timestamp_ns)
        if not isinstance(self.world_from_tracker, Transform):
            raise TypeError("world_from_tracker must be Transform")
        if not isinstance(self.camera_from_board, Transform):
            raise TypeError("camera_from_board must be Transform")
        for name in (
            "reprojection_rms_px",
            "local_translation_motion_m",
            "local_rotation_motion_deg",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if type(self.corner_count) is not int or self.corner_count < 4:
            raise ValueError("corner_count must be at least four")
        if type(self.interpolation_gap_ns) is not int or self.interpolation_gap_ns < 0:
            raise ValueError("interpolation_gap_ns must be non-negative")
