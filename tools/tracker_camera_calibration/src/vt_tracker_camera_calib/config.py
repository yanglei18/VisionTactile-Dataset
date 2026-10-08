from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import re

import numpy as np
import yaml

from .model import CameraIntrinsics


_TOPIC_PATTERN = re.compile(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+")
_FRAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_/]*")


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _text(mapping: dict[str, object], key: str, *, pattern: re.Pattern[str] | None = None) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{key} must be a non-empty string")
    if pattern is not None and pattern.fullmatch(value) is None:
        raise ValueError(f"{key} has an invalid value")
    return value


def _positive_float(mapping: dict[str, object], key: str) -> float:
    value = mapping.get(key)
    if (
        type(value) not in {int, float}
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or float(value) <= 0
    ):
        raise ValueError(f"{key} must be positive")
    return float(value)


def _positive_int(mapping: dict[str, object], key: str, minimum: int = 1) -> int:
    value = mapping.get(key)
    if type(value) is not int or value < minimum:
        raise ValueError(f"{key} must be an integer >= {minimum}")
    return value


@dataclass(frozen=True)
class BoardConfig:
    squares_x: int
    squares_y: int
    square_length_m: float
    marker_length_m: float
    dictionary: str
    min_corners: int
    max_reprojection_rms_px: float

    def __post_init__(self) -> None:
        if self.squares_x < 3 or self.squares_y < 3:
            raise ValueError("ChArUco board must contain at least 3x3 squares")
        if not 0.0 < self.marker_length_m < self.square_length_m:
            raise ValueError("marker length must be smaller than square length")
        maximum_corners = (self.squares_x - 1) * (self.squares_y - 1)
        if not 4 <= self.min_corners <= maximum_corners:
            raise ValueError("min_corners is outside the board corner count")
        if self.max_reprojection_rms_px <= 0:
            raise ValueError("max_reprojection_rms_px must be positive")


@dataclass(frozen=True)
class PairingConfig:
    max_interpolation_gap_ms: float
    stability_window_ms: float
    max_static_translation_m: float
    max_static_rotation_deg: float
    min_time_separation_s: float
    min_pose_translation_m: float
    min_pose_rotation_deg: float

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class AcceptanceConfig:
    min_pairs: int
    holdout_fraction: float
    target_translation_rms_m: float
    target_rotation_rms_deg: float
    maximum_translation_rms_m: float
    maximum_rotation_rms_deg: float

    def __post_init__(self) -> None:
        if self.min_pairs < 10:
            raise ValueError("min_pairs must be at least 10")
        if not 0.1 <= self.holdout_fraction <= 0.4:
            raise ValueError("holdout_fraction must be within [0.1, 0.4]")
        for name in (
            "target_translation_rms_m",
            "target_rotation_rms_deg",
            "maximum_translation_rms_m",
            "maximum_rotation_rms_deg",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.target_translation_rms_m > self.maximum_translation_rms_m:
            raise ValueError("target translation threshold exceeds maximum")
        if self.target_rotation_rms_deg > self.maximum_rotation_rms_deg:
            raise ValueError("target rotation threshold exceeds maximum")


@dataclass(frozen=True)
class CalibrationConfig:
    camera_name: str
    camera_serial: str
    camera_model: str
    camera_frame: str
    image_topic: str
    camera_info_topic: str
    timing_topic: str
    tracker_role: str
    tracker_frame: str
    tracker_sample_topic: str
    world_frame: str
    board_frame: str
    board: BoardConfig
    pairing: PairingConfig
    acceptance: AcceptanceConfig
    fallback_intrinsics: CameraIntrinsics | None = None

    def __post_init__(self) -> None:
        if self.tracker_role not in {"torso", "left_wrist", "right_wrist"}:
            raise ValueError("tracker role is unsupported")
        topics = {
            self.image_topic,
            self.camera_info_topic,
            self.timing_topic,
            self.tracker_sample_topic,
        }
        if len(topics) != 4:
            raise ValueError("camera and Tracker topics must be distinct")
        frames = {
            self.camera_frame,
            self.tracker_frame,
            self.world_frame,
            self.board_frame,
        }
        if len(frames) != 4:
            raise ValueError("camera, Tracker, world, and board frames must be distinct")


def _load_intrinsics(value: object) -> CameraIntrinsics | None:
    if value is None:
        return None
    mapping = _mapping(value, "intrinsics")
    matrix = mapping.get("camera_matrix")
    distortion = mapping.get("distortion", [])
    return CameraIntrinsics(
        width=_positive_int(mapping, "width"),
        height=_positive_int(mapping, "height"),
        camera_matrix=np.asarray(matrix, dtype=np.float64),
        distortion=np.asarray(distortion, dtype=np.float64),
        distortion_model=str(mapping.get("distortion_model", "plumb_bob")),
    )


def load_config(path: str | Path) -> CalibrationConfig:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"configuration file does not exist: {source}")
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    root = _mapping(document, "configuration")
    if root.get("schema_version") != 1:
        raise ValueError("schema_version must be 1")
    camera = _mapping(root.get("camera"), "camera")
    tracker = _mapping(root.get("tracker"), "tracker")
    frames = _mapping(root.get("frames"), "frames")
    board_data = _mapping(root.get("board"), "board")
    pairing_data = _mapping(root.get("pairing"), "pairing")
    acceptance_data = _mapping(root.get("acceptance"), "acceptance")

    board = BoardConfig(
        squares_x=_positive_int(board_data, "squares_x", 3),
        squares_y=_positive_int(board_data, "squares_y", 3),
        square_length_m=_positive_float(board_data, "square_length_m"),
        marker_length_m=_positive_float(board_data, "marker_length_m"),
        dictionary=_text(board_data, "dictionary"),
        min_corners=_positive_int(board_data, "min_corners", 4),
        max_reprojection_rms_px=_positive_float(
            board_data, "max_reprojection_rms_px"
        ),
    )
    pairing = PairingConfig(
        max_interpolation_gap_ms=_positive_float(
            pairing_data, "max_interpolation_gap_ms"
        ),
        stability_window_ms=_positive_float(pairing_data, "stability_window_ms"),
        max_static_translation_m=_positive_float(
            pairing_data, "max_static_translation_m"
        ),
        max_static_rotation_deg=_positive_float(
            pairing_data, "max_static_rotation_deg"
        ),
        min_time_separation_s=_positive_float(
            pairing_data, "min_time_separation_s"
        ),
        min_pose_translation_m=_positive_float(
            pairing_data, "min_pose_translation_m"
        ),
        min_pose_rotation_deg=_positive_float(
            pairing_data, "min_pose_rotation_deg"
        ),
    )
    acceptance = AcceptanceConfig(
        min_pairs=_positive_int(acceptance_data, "min_pairs", 10),
        holdout_fraction=_positive_float(acceptance_data, "holdout_fraction"),
        target_translation_rms_m=_positive_float(
            acceptance_data, "target_translation_rms_m"
        ),
        target_rotation_rms_deg=_positive_float(
            acceptance_data, "target_rotation_rms_deg"
        ),
        maximum_translation_rms_m=_positive_float(
            acceptance_data, "maximum_translation_rms_m"
        ),
        maximum_rotation_rms_deg=_positive_float(
            acceptance_data, "maximum_rotation_rms_deg"
        ),
    )
    return CalibrationConfig(
        camera_name=_text(camera, "name"),
        camera_serial=_text(camera, "serial"),
        camera_model=_text(camera, "model"),
        camera_frame=_text(camera, "frame_id", pattern=_FRAME_PATTERN),
        image_topic=_text(camera, "image_topic", pattern=_TOPIC_PATTERN),
        camera_info_topic=_text(
            camera, "camera_info_topic", pattern=_TOPIC_PATTERN
        ),
        timing_topic=_text(camera, "timing_topic", pattern=_TOPIC_PATTERN),
        tracker_role=_text(tracker, "role"),
        tracker_frame=_text(tracker, "frame_id", pattern=_FRAME_PATTERN),
        tracker_sample_topic=_text(
            tracker, "sample_topic", pattern=_TOPIC_PATTERN
        ),
        world_frame=_text(frames, "world", pattern=_FRAME_PATTERN),
        board_frame=_text(frames, "board", pattern=_FRAME_PATTERN),
        board=board,
        pairing=pairing,
        acceptance=acceptance,
        fallback_intrinsics=_load_intrinsics(camera.get("intrinsics")),
    )
