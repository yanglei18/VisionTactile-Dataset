from __future__ import annotations

import os
from pathlib import Path
import math

import yaml

from vt_capture_core.calibration_policy import (
    acceptance_defaults, board_detection_defaults, board_geometry_defaults, pairing_defaults,
)

from .config import load_config


REFERENCE_CAMERAS: dict[str, tuple[str, str]] = {
    "d405_1": ("D405", "260322278433"),
    "d405_2": ("D405", "260322276463"),
    "d436": ("D436", "408322071716"),
}
TRACKER_ROLES = ("torso", "left_wrist", "right_wrist")


def build_config_document(
    *,
    camera_name: str,
    tracker_role: str,
    square_length_mm: float,
    marker_length_mm: float,
) -> dict[str, object]:
    if camera_name not in REFERENCE_CAMERAS:
        raise ValueError(f"unsupported reference camera: {camera_name}")
    if tracker_role not in TRACKER_ROLES:
        raise ValueError(f"unsupported Tracker role: {tracker_role}")
    if (
        not math.isfinite(square_length_mm)
        or not math.isfinite(marker_length_mm)
        or square_length_mm <= 0.0
        or marker_length_mm <= 0.0
    ):
        raise ValueError("measured board lengths must be positive")
    if marker_length_mm >= square_length_mm:
        raise ValueError("marker length must be smaller than square length")
    model, serial = REFERENCE_CAMERAS[camera_name]
    geometry = board_geometry_defaults()
    return {
        "schema_version": 1,
        "camera": {
            "name": camera_name,
            "model": model,
            "serial": serial,
            "frame_id": f"{camera_name}_color_optical_frame",
            "image_topic": f"/{camera_name}/color/image_raw",
            "camera_info_topic": f"/{camera_name}/color/camera_info",
            "timing_topic": f"/{camera_name}/frame_timing",
        },
        "tracker": {
            "role": tracker_role,
            "frame_id": f"vive_tracker_{tracker_role}",
            "sample_topic": f"/vive/{tracker_role}/sample",
        },
        "frames": {
            "world": "vive_map",
            "board": "charuco_board",
        },
        "board": {
            "squares_x": geometry["squares_x"],
            "squares_y": geometry["squares_y"],
            "square_length_m": round(square_length_mm / 1000.0, 9),
            "marker_length_m": round(marker_length_mm / 1000.0, 9),
            "dictionary": geometry["dictionary"],
            **board_detection_defaults(),
        },
        "pairing": pairing_defaults(),
        "acceptance": acceptance_defaults(),
    }


def write_config(path: str | Path, document: dict[str, object]) -> Path:
    target = Path(path).resolve()
    if target.suffix.lower() not in {".yaml", ".yml"}:
        raise ValueError("configuration output must be a YAML file")
    if not target.parent.is_dir():
        raise FileNotFoundError(
            f"configuration output parent does not exist: {target.parent}"
        )
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(target, flags, 0o600)
    try:
        payload = yaml.safe_dump(document, sort_keys=False).encode("utf-8")
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        load_config(target)
    except BaseException:
        target.unlink(missing_ok=True)
        raise
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    return target
