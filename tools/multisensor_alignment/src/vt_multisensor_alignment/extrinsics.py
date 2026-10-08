from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import re
from typing import Mapping

import numpy as np
import yaml

from .config import AlignmentConfig, CameraConfig
from .model import Transform


_TRACKER_ID = re.compile(r"[0-9a-f]{64}")


@dataclass(frozen=True)
class ExtrinsicBinding:
    camera_name: str
    camera_model: str
    camera_serial: str
    camera_frame: str
    tracker_role: str
    tracker_id: str
    tracker_frame: str
    world_frame: str
    tracker_from_camera: Transform
    source_path: Path
    sha256: str

    def as_manifest_document(self) -> dict[str, object]:
        return {
            "camera_name": self.camera_name,
            "camera_model": self.camera_model,
            "camera_serial": self.camera_serial,
            "camera_frame": self.camera_frame,
            "tracker_role": self.tracker_role,
            "tracker_id": self.tracker_id,
            "tracker_frame": self.tracker_frame,
            "world_frame": self.world_frame,
            "transform": self.tracker_from_camera.as_document(),
            "source_file": self.source_path.name,
            "sha256": self.sha256,
        }


def _mapping(value: object, context: str) -> dict[str, object]:
    if type(value) is not dict:
        raise ValueError(f"{context} must be a mapping")
    return value


def _text(value: Mapping[str, object], key: str, context: str) -> str:
    result = value.get(key)
    if type(result) is not str or not result:
        raise ValueError(f"{context}.{key} must be a non-empty string")
    return result


def _vector(
    value: Mapping[str, object], key: str, size: int, context: str
) -> np.ndarray:
    observed = value.get(key)
    if type(observed) is not list or len(observed) != size:
        raise ValueError(f"{context}.{key} must contain {size} values")
    if any(type(item) not in {int, float} for item in observed):
        raise ValueError(f"{context}.{key} must contain numeric values")
    result = np.asarray(observed, dtype=np.float64)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{context}.{key} must contain finite values")
    return result


def _require_equal(
    observed: str, expected: str, context: str
) -> None:
    if observed != expected:
        raise ValueError(
            f"{context} mismatch: expected {expected}, got {observed}"
        )


def _load_one(
    path: Path,
    camera: CameraConfig,
    config: AlignmentConfig,
    tracker_ids: Mapping[str, str],
) -> ExtrinsicBinding:
    raw = path.read_bytes()
    try:
        root = _mapping(yaml.safe_load(raw), str(path))
    except yaml.YAMLError as error:
        raise ValueError(f"invalid extrinsic YAML: {path}: {error}") from error
    if root.get("schema_version") != 1:
        raise ValueError(f"unsupported extrinsic schema_version: {path}")
    if root.get("status") != "VALID":
        raise ValueError(f"extrinsic status VALID is required: {path}")
    camera_doc = _mapping(root.get("camera"), "camera")
    _require_equal(_text(camera_doc, "name", "camera"), camera.name, "camera.name")
    _require_equal(
        _text(camera_doc, "model", "camera"), camera.model, "camera.model"
    )
    _require_equal(
        _text(camera_doc, "serial", "camera"), camera.serial, "camera.serial"
    )
    _require_equal(
        _text(camera_doc, "frame_id", "camera"),
        camera.frame_id,
        "camera.frame_id",
    )
    tracker_config = config.tracker_by_role[camera.tracker_role]
    tracker_doc = _mapping(root.get("tracker"), "tracker")
    _require_equal(
        _text(tracker_doc, "role", "tracker"),
        camera.tracker_role,
        "tracker.role",
    )
    tracker_id = _text(tracker_doc, "tracker_id", "tracker")
    if _TRACKER_ID.fullmatch(tracker_id) is None:
        raise ValueError("tracker.tracker_id is malformed")
    expected_id = tracker_ids.get(camera.tracker_role)
    if expected_id is None or tracker_id != expected_id:
        raise ValueError(
            f"tracker.tracker_id mismatch for role {camera.tracker_role}"
        )
    _require_equal(
        _text(tracker_doc, "frame_id", "tracker"),
        tracker_config.frame_id,
        "tracker.frame_id",
    )
    transform_doc = _mapping(root.get("transform"), "transform")
    _require_equal(
        _text(transform_doc, "semantics", "transform"),
        "parent_from_child",
        "transform.semantics",
    )
    _require_equal(
        _text(transform_doc, "parent_frame", "transform"),
        tracker_config.frame_id,
        "transform.parent_frame",
    )
    _require_equal(
        _text(transform_doc, "child_frame", "transform"),
        camera.frame_id,
        "transform.child_frame",
    )
    frames = _mapping(root.get("frames"), "frames")
    _require_equal(
        _text(frames, "world", "frames"), config.world_frame, "frames.world"
    )
    transform = Transform(
        _vector(transform_doc, "translation_m", 3, "transform"),
        _vector(transform_doc, "quaternion_xyzw", 4, "transform"),
    )
    return ExtrinsicBinding(
        camera_name=camera.name,
        camera_model=camera.model,
        camera_serial=camera.serial,
        camera_frame=camera.frame_id,
        tracker_role=camera.tracker_role,
        tracker_id=tracker_id,
        tracker_frame=tracker_config.frame_id,
        world_frame=config.world_frame,
        tracker_from_camera=transform,
        source_path=path,
        sha256=hashlib.sha256(raw).hexdigest(),
    )


def load_extrinsics(
    directory: str | Path,
    config: AlignmentConfig,
    tracker_ids: Mapping[str, str],
) -> dict[str, ExtrinsicBinding]:
    root = Path(directory).resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"extrinsics directory does not exist: {root}")
    result: dict[str, ExtrinsicBinding] = {}
    for camera in config.cameras:
        path = root / f"{camera.name}.yaml"
        if not path.is_file():
            raise FileNotFoundError(f"extrinsic file does not exist: {path}")
        result[camera.name] = _load_one(path, camera, config, tracker_ids)
    return result
