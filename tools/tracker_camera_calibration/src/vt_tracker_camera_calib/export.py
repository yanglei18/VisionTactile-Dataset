from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
import shutil
import tempfile

import yaml

from .bag_reader import BagObservations
from .config import CalibrationConfig
from .handeye import CalibrationSolution
from .model import CalibrationPair


def _finite_or_none(value: float) -> float | None:
    return value if value != float("inf") and value == value else None


def _solution_document(
    *,
    config: CalibrationConfig,
    solution: CalibrationSolution,
    observations: BagObservations,
    bag_name: str,
    config_sha256: str,
    calibrated_at: str,
) -> dict[str, object]:
    transform = solution.tracker_from_camera
    return {
        "schema_version": 1,
        "status": "VALID" if solution.quality != "REJECTED" else "REJECTED",
        "camera": {
            "name": config.camera_name,
            "model": config.camera_model,
            "serial": config.camera_serial,
            "frame_id": config.camera_frame,
        },
        "tracker": {
            "role": config.tracker_role,
            "tracker_id": observations.tracker_id,
            "frame_id": config.tracker_frame,
        },
        "transform": {
            "semantics": "parent_from_child",
            "parent_frame": config.tracker_frame,
            "child_frame": config.camera_frame,
            "translation_m": [float(value) for value in transform.translation],
            "quaternion_xyzw": [float(value) for value in transform.quaternion_xyzw],
        },
        "quality": {
            "grade": solution.quality,
            "method": solution.method,
            "sample_count": solution.pair_count,
            "training_count": solution.training_count,
            "holdout_count": solution.holdout_count,
            "validation_translation_rms_m": solution.validation_translation_rms_m,
            "validation_rotation_rms_deg": solution.validation_rotation_rms_deg,
            "all_translation_rms_m": solution.all_translation_rms_m,
            "all_rotation_rms_deg": solution.all_rotation_rms_deg,
            "reprojection_rms_px": solution.reprojection_rms_px,
        },
        "frames": {
            "world": config.world_frame,
            "board": config.board_frame,
        },
        "provenance": {
            "calibrated_at": calibrated_at,
            "source_bag_name": bag_name,
            "config_sha256": config_sha256,
            "tool": "vt-tracker-camera-calibration",
            "tool_version": "0.3.0",
        },
    }


def _report_document(
    solution: CalibrationSolution,
    observations: BagObservations,
    selected_pairs: tuple[CalibrationPair, ...],
) -> dict[str, object]:
    return {
        "quality": solution.quality,
        "selected_method": solution.method,
        "counts": {
            "images": observations.image_count,
            "images_with_timing": observations.timed_image_count,
            "images_rejected_or_without_board": observations.rejected_image_count,
            "board_observations": len(observations.board_observations),
            "valid_tracker_samples": len(observations.tracker_samples),
            "selected_static_pairs": len(selected_pairs),
        },
        "metrics": {
            "validation_translation_rms_m": solution.validation_translation_rms_m,
            "validation_rotation_rms_deg": solution.validation_rotation_rms_deg,
            "all_translation_rms_m": solution.all_translation_rms_m,
            "all_rotation_rms_deg": solution.all_rotation_rms_deg,
            "reprojection_rms_px": solution.reprojection_rms_px,
        },
        "methods": [
            {
                "method": score.method,
                "valid": score.valid,
                "translation_rms_m": _finite_or_none(score.translation_rms_m),
                "rotation_rms_deg": _finite_or_none(score.rotation_rms_deg),
                "score": _finite_or_none(score.score),
                "reason": score.reason,
            }
            for score in solution.method_scores
        ],
    }


def _write_svg(
    path: Path,
    solution: CalibrationSolution,
    config: CalibrationConfig,
) -> None:
    width = 1000
    height = 520
    margin = 60
    plot_width = width - 2 * margin
    plot_height = 150
    residuals = solution.residuals
    count = max(1, len(residuals) - 1)

    def points(values: list[float], maximum: float, top: float) -> str:
        result = []
        for index, value in enumerate(values):
            x = margin + plot_width * index / count
            y = top + plot_height * (1.0 - min(value / maximum, 1.0))
            result.append(f"{x:.1f},{y:.1f}")
        return " ".join(result)

    translation_values = [residual.translation_m * 1000.0 for residual in residuals]
    rotation_values = [residual.rotation_deg for residual in residuals]
    translation_max = max(
        config.acceptance.maximum_translation_rms_m * 1500.0,
        max(translation_values, default=1.0),
        1.0,
    )
    rotation_max = max(
        config.acceptance.maximum_rotation_rms_deg * 1.5,
        max(rotation_values, default=0.1),
        0.1,
    )
    title = html.escape(
        f"{config.camera_name} / {config.tracker_role} — {solution.quality} ({solution.method})"
    )
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/>
<text x="{margin}" y="35" font-family="sans-serif" font-size="22">{title}</text>
<text x="{margin}" y="65" font-family="sans-serif" font-size="14">Per-pose closure residuals; clipped at plot maximum</text>
<rect x="{margin}" y="90" width="{plot_width}" height="{plot_height}" fill="#f6f8fa" stroke="#8c959f"/>
<polyline fill="none" stroke="#0969da" stroke-width="2" points="{points(translation_values, translation_max, 90)}"/>
<text x="{margin}" y="82" font-family="sans-serif" font-size="14">Translation (0–{translation_max:.1f} mm)</text>
<rect x="{margin}" y="310" width="{plot_width}" height="{plot_height}" fill="#f6f8fa" stroke="#8c959f"/>
<polyline fill="none" stroke="#cf222e" stroke-width="2" points="{points(rotation_values, rotation_max, 310)}"/>
<text x="{margin}" y="302" font-family="sans-serif" font-size="14">Rotation (0–{rotation_max:.2f} deg)</text>
<text x="{margin}" y="500" font-family="sans-serif" font-size="13">Validation RMS: {solution.validation_translation_rms_m * 1000:.2f} mm, {solution.validation_rotation_rms_deg:.3f} deg</text>
</svg>
"""
    path.write_text(svg, encoding="utf-8")


def export_result(
    *,
    output_directory: str | Path,
    config_path: str | Path,
    bag_path: str | Path,
    config: CalibrationConfig,
    observations: BagObservations,
    selected_pairs: tuple[CalibrationPair, ...],
    solution: CalibrationSolution,
) -> Path:
    target = Path(output_directory).resolve()
    if target.exists():
        raise FileExistsError(f"refusing to overwrite output directory: {target}")
    if not target.parent.is_dir():
        raise FileNotFoundError(f"output parent does not exist: {target.parent}")
    config_bytes = Path(config_path).read_bytes()
    config_sha256 = hashlib.sha256(config_bytes).hexdigest()
    calibrated_at = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{target.name}.", dir=target.parent)
    )
    try:
        solution_document = _solution_document(
            config=config,
            solution=solution,
            observations=observations,
            bag_name=Path(bag_path).name,
            config_sha256=config_sha256,
            calibrated_at=calibrated_at,
        )
        (temporary / "extrinsics.yaml").write_text(
            yaml.safe_dump(solution_document, sort_keys=False), encoding="utf-8"
        )
        report = _report_document(solution, observations, selected_pairs)
        (temporary / "report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        with (temporary / "residuals.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "pair_index",
                    "translation_m",
                    "rotation_deg",
                    "reprojection_rms_px",
                ]
            )
            for residual in solution.residuals:
                writer.writerow(
                    [
                        residual.pair_index,
                        f"{residual.translation_m:.12g}",
                        f"{residual.rotation_deg:.12g}",
                        f"{residual.reprojection_rms_px:.12g}",
                    ]
                )
        _write_svg(temporary / "diagnostics.svg", solution, config)
        os.rename(temporary, target)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return target
