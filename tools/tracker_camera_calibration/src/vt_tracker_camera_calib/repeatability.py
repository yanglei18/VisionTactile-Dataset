from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import json
import math
import os
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml

from .transforms import (
    Transform,
    rotation_distance_rad,
    transform_mean,
)


@dataclass(frozen=True)
class ExtrinsicIdentity:
    camera_name: str
    camera_model: str
    camera_serial: str
    camera_frame: str
    tracker_role: str
    tracker_id: str
    tracker_frame: str
    parent_frame: str
    child_frame: str


@dataclass(frozen=True)
class ExtrinsicRun:
    path: Path
    identity: ExtrinsicIdentity
    transform: Transform


@dataclass(frozen=True)
class PairwiseDifference:
    left: str
    right: str
    translation_m: float
    rotation_deg: float


@dataclass(frozen=True)
class RepeatabilityReport:
    status: str
    identity: ExtrinsicIdentity
    run_count: int
    maximum_translation_m: float
    maximum_rotation_deg: float
    threshold_translation_m: float
    threshold_rotation_deg: float
    consensus_transform: Transform
    recommended_input: str
    pairwise: tuple[PairwiseDifference, ...]


def _run_label(run: ExtrinsicRun) -> str:
    return f"{run.path.parent.name}/{run.path.name}"


def _mapping(value: object, name: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _text(mapping: dict[str, object], key: str, context: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context}.{key} must be a non-empty string")
    return value


def load_extrinsic(path: str | Path) -> ExtrinsicRun:
    source = Path(path).resolve()
    if not source.is_file():
        raise FileNotFoundError(f"extrinsic file does not exist: {source}")
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    root = _mapping(document, "extrinsic document")
    if root.get("schema_version") != 1:
        raise ValueError(f"unsupported extrinsic schema: {source}")
    if root.get("status") != "VALID":
        raise ValueError(f"extrinsic status is not VALID: {source}")
    camera = _mapping(root.get("camera"), "camera")
    tracker = _mapping(root.get("tracker"), "tracker")
    transform_data = _mapping(root.get("transform"), "transform")
    if transform_data.get("semantics") != "parent_from_child":
        raise ValueError(f"unsupported transform semantics: {source}")
    identity = ExtrinsicIdentity(
        camera_name=_text(camera, "name", "camera"),
        camera_model=_text(camera, "model", "camera"),
        camera_serial=_text(camera, "serial", "camera"),
        camera_frame=_text(camera, "frame_id", "camera"),
        tracker_role=_text(tracker, "role", "tracker"),
        tracker_id=_text(tracker, "tracker_id", "tracker"),
        tracker_frame=_text(tracker, "frame_id", "tracker"),
        parent_frame=_text(transform_data, "parent_frame", "transform"),
        child_frame=_text(transform_data, "child_frame", "transform"),
    )
    if identity.parent_frame != identity.tracker_frame:
        raise ValueError(f"parent frame is not the Tracker frame: {source}")
    if identity.child_frame != identity.camera_frame:
        raise ValueError(f"child frame is not the camera frame: {source}")
    transform = Transform.from_quaternion_xyzw(
        transform_data.get("translation_m"),
        transform_data.get("quaternion_xyzw"),
    )
    return ExtrinsicRun(path=source, identity=identity, transform=transform)


def compare_extrinsics(
    paths: Sequence[str | Path],
    *,
    threshold_translation_m: float = 0.005,
    threshold_rotation_deg: float = 0.5,
) -> RepeatabilityReport:
    if len(paths) < 3:
        raise ValueError("repeatability comparison requires at least three runs")
    if (
        not math.isfinite(threshold_translation_m)
        or threshold_translation_m <= 0.0
    ):
        raise ValueError("translation threshold must be finite and positive")
    if not math.isfinite(threshold_rotation_deg) or threshold_rotation_deg <= 0.0:
        raise ValueError("rotation threshold must be finite and positive")
    runs = tuple(load_extrinsic(path) for path in paths)
    resolved_paths = tuple(run.path for run in runs)
    if len(set(resolved_paths)) != len(resolved_paths):
        raise ValueError("repeatability inputs must be distinct files")
    reference_identity = runs[0].identity
    for run in runs[1:]:
        if run.identity != reference_identity:
            raise ValueError(
                "repeatability inputs do not describe the same camera, "
                "Tracker, and frame binding"
            )

    differences: list[PairwiseDifference] = []
    for left, right in combinations(runs, 2):
        differences.append(
            PairwiseDifference(
                left=_run_label(left),
                right=_run_label(right),
                translation_m=float(
                    np.linalg.norm(
                        left.transform.translation - right.transform.translation
                    )
                ),
                rotation_deg=math.degrees(
                    rotation_distance_rad(left.transform, right.transform)
                ),
            )
        )
    maximum_translation_m = max(
        difference.translation_m for difference in differences
    )
    maximum_rotation_deg = max(
        difference.rotation_deg for difference in differences
    )
    consensus = transform_mean([run.transform for run in runs])
    normalized_distances = []
    for run in runs:
        normalized_distances.append(
            (
                float(
                    np.linalg.norm(
                        run.transform.translation - consensus.translation
                    )
                )
                / threshold_translation_m
                + math.degrees(rotation_distance_rad(run.transform, consensus))
                / threshold_rotation_deg,
                _run_label(run),
            )
        )
    recommended_input = min(normalized_distances)[1]
    status = (
        "PASS"
        if maximum_translation_m <= threshold_translation_m
        and maximum_rotation_deg <= threshold_rotation_deg
        else "FAIL"
    )
    return RepeatabilityReport(
        status=status,
        identity=reference_identity,
        run_count=len(runs),
        maximum_translation_m=maximum_translation_m,
        maximum_rotation_deg=maximum_rotation_deg,
        threshold_translation_m=threshold_translation_m,
        threshold_rotation_deg=threshold_rotation_deg,
        consensus_transform=consensus,
        recommended_input=recommended_input,
        pairwise=tuple(differences),
    )


def _report_document(report: RepeatabilityReport) -> dict[str, object]:
    identity = report.identity
    consensus = report.consensus_transform
    return {
        "schema_version": 1,
        "status": report.status,
        "identity": {
            "camera_name": identity.camera_name,
            "camera_model": identity.camera_model,
            "camera_serial": identity.camera_serial,
            "camera_frame": identity.camera_frame,
            "tracker_role": identity.tracker_role,
            "tracker_id": identity.tracker_id,
            "tracker_frame": identity.tracker_frame,
            "parent_frame": identity.parent_frame,
            "child_frame": identity.child_frame,
        },
        "run_count": report.run_count,
        "thresholds": {
            "translation_m": report.threshold_translation_m,
            "rotation_deg": report.threshold_rotation_deg,
        },
        "metrics": {
            "maximum_pairwise_translation_m": report.maximum_translation_m,
            "maximum_pairwise_rotation_deg": report.maximum_rotation_deg,
        },
        "recommended_input": report.recommended_input,
        "consensus_transform": {
            "semantics": "parent_from_child",
            "translation_m": [
                float(value) for value in consensus.translation
            ],
            "quaternion_xyzw": [
                float(value) for value in consensus.quaternion_xyzw
            ],
        },
        "pairwise": [
            {
                "left": difference.left,
                "right": difference.right,
                "translation_m": difference.translation_m,
                "rotation_deg": difference.rotation_deg,
            }
            for difference in report.pairwise
        ],
    }


def write_repeatability_report(
    path: str | Path, report: RepeatabilityReport
) -> Path:
    target = Path(path).resolve()
    if target.suffix.lower() != ".json":
        raise ValueError("repeatability output must be a JSON file")
    if not target.parent.is_dir():
        raise FileNotFoundError(
            f"repeatability output parent does not exist: {target.parent}"
        )
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(target, flags, 0o600)
    try:
        payload = (
            json.dumps(
                _report_document(report),
                indent=2,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        target.unlink(missing_ok=True)
        raise
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    return target
