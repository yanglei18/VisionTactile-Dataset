from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .config import AcceptanceConfig
from .model import CalibrationPair
from .transforms import Transform, rotation_distance_rad, transform_mean


_METHODS = {
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass(frozen=True)
class Residual:
    pair_index: int
    translation_m: float
    rotation_deg: float
    reprojection_rms_px: float


@dataclass(frozen=True)
class MethodScore:
    method: str
    translation_rms_m: float
    rotation_rms_deg: float
    score: float
    valid: bool
    reason: str = ""


@dataclass(frozen=True)
class CalibrationSolution:
    tracker_from_camera: Transform
    method: str
    pair_count: int
    training_count: int
    holdout_count: int
    validation_translation_rms_m: float
    validation_rotation_rms_deg: float
    all_translation_rms_m: float
    all_rotation_rms_deg: float
    reprojection_rms_px: float
    quality: str
    method_scores: tuple[MethodScore, ...]
    residuals: tuple[Residual, ...]


def _rms(values: list[float] | tuple[float, ...]) -> float:
    if not values:
        raise ValueError("RMS requires at least one value")
    return math.sqrt(sum(value * value for value in values) / len(values))


def _solve_method(pairs: tuple[CalibrationPair, ...], method: int) -> Transform:
    rotations_gripper_to_base = [pair.world_from_tracker.rotation for pair in pairs]
    translations_gripper_to_base = [
        pair.world_from_tracker.translation.reshape(3, 1) for pair in pairs
    ]
    rotations_target_to_camera = [pair.camera_from_board.rotation for pair in pairs]
    translations_target_to_camera = [
        pair.camera_from_board.translation.reshape(3, 1) for pair in pairs
    ]
    rotation, translation = cv2.calibrateHandEye(
        rotations_gripper_to_base,
        translations_gripper_to_base,
        rotations_target_to_camera,
        translations_target_to_camera,
        method=method,
    )
    return Transform(rotation, np.asarray(translation).reshape(3))


def _world_from_board(pair: CalibrationPair, tracker_from_camera: Transform) -> Transform:
    return (
        pair.world_from_tracker
        @ tracker_from_camera
        @ pair.camera_from_board
    )

def _residuals(
    pairs: tuple[CalibrationPair, ...],
    tracker_from_camera: Transform,
    anchor: Transform,
    original_indices: tuple[int, ...],
) -> tuple[Residual, ...]:
    output = []
    for index, pair in zip(original_indices, pairs, strict=True):
        estimate = _world_from_board(pair, tracker_from_camera)
        output.append(
            Residual(
                pair_index=index,
                translation_m=float(
                    np.linalg.norm(estimate.translation - anchor.translation)
                ),
                rotation_deg=math.degrees(rotation_distance_rad(anchor, estimate)),
                reprojection_rms_px=pair.reprojection_rms_px,
            )
        )
    return tuple(output)


def _require_excitation(pairs: tuple[CalibrationPair, ...]) -> None:
    if len(pairs) < 8:
        raise ValueError("at least eight pose pairs are required by the solver")
    reference = pairs[0].world_from_tracker
    rotation_vectors = []
    translation_span = 0.0
    maximum_rotation = 0.0
    for pair in pairs[1:]:
        relative_rotation = reference.rotation.T @ pair.world_from_tracker.rotation
        vector, _ = cv2.Rodrigues(relative_rotation)
        vector = vector.reshape(3)
        angle = float(np.linalg.norm(vector))
        maximum_rotation = max(maximum_rotation, angle)
        translation_span = max(
            translation_span,
            float(
                np.linalg.norm(
                    pair.world_from_tracker.translation - reference.translation
                )
            ),
        )
        if angle >= math.radians(5.0):
            rotation_vectors.append(vector / angle)
    if maximum_rotation < math.radians(15.0):
        raise ValueError("pose set needs at least 15 degrees of rotational span")
    if translation_span < 0.05:
        raise ValueError("pose set needs at least 50 mm of translational span")
    if len(rotation_vectors) < 2:
        raise ValueError("pose set lacks usable rotational excitation")
    singular_values = np.linalg.svd(np.stack(rotation_vectors), compute_uv=False)
    if len(singular_values) < 2 or singular_values[1] < 0.15:
        raise ValueError("pose set must rotate around at least two non-collinear axes")


def _split_indices(count: int, fraction: float) -> tuple[tuple[int, ...], tuple[int, ...]]:
    holdout_count = max(3, round(count * fraction))
    holdout_count = min(holdout_count, count - 8)
    if holdout_count < 1:
        raise ValueError("not enough pairs for an independent holdout set")
    candidates = np.linspace(1, count - 2, holdout_count, dtype=int)
    holdout = tuple(sorted(set(int(value) for value in candidates)))
    training = tuple(index for index in range(count) if index not in set(holdout))
    return training, holdout


def solve_hand_eye(
    pairs: list[CalibrationPair] | tuple[CalibrationPair, ...],
    acceptance: AcceptanceConfig,
) -> CalibrationSolution:
    values = tuple(pairs)
    if len(values) < acceptance.min_pairs:
        raise ValueError(
            f"need at least {acceptance.min_pairs} selected pairs, got {len(values)}"
        )
    _require_excitation(values)
    training_indices, holdout_indices = _split_indices(
        len(values), acceptance.holdout_fraction
    )
    training = tuple(values[index] for index in training_indices)
    holdout = tuple(values[index] for index in holdout_indices)
    _require_excitation(training)

    method_scores: list[MethodScore] = []
    training_solutions: dict[str, Transform] = {}
    for name, method in _METHODS.items():
        try:
            candidate = _solve_method(training, method)
            training_solutions[name] = candidate
            anchor = transform_mean(
                [_world_from_board(pair, candidate) for pair in training]
            )
            residuals = _residuals(
                holdout, candidate, anchor, holdout_indices
            )
            translation_rms = _rms(
                [residual.translation_m for residual in residuals]
            )
            rotation_rms = _rms(
                [residual.rotation_deg for residual in residuals]
            )
            score = (
                translation_rms / acceptance.maximum_translation_rms_m
                + rotation_rms / acceptance.maximum_rotation_rms_deg
            )
            method_scores.append(
                MethodScore(
                    method=name,
                    translation_rms_m=translation_rms,
                    rotation_rms_deg=rotation_rms,
                    score=score,
                    valid=True,
                )
            )
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            method_scores.append(
                MethodScore(
                    method=name,
                    translation_rms_m=math.inf,
                    rotation_rms_deg=math.inf,
                    score=math.inf,
                    valid=False,
                    reason=str(error),
                )
            )
    valid_scores = [score for score in method_scores if score.valid]
    if not valid_scores:
        raise RuntimeError("all hand-eye methods failed")
    selected_score = min(valid_scores, key=lambda score: score.score)
    validation_transform = training_solutions[selected_score.method]
    training_anchor = transform_mean(
        [_world_from_board(pair, validation_transform) for pair in training]
    )
    validation_residuals = _residuals(
        holdout, validation_transform, training_anchor, holdout_indices
    )
    validation_translation_rms = _rms(
        [residual.translation_m for residual in validation_residuals]
    )
    validation_rotation_rms = _rms(
        [residual.rotation_deg for residual in validation_residuals]
    )

    final_transform = _solve_method(values, _METHODS[selected_score.method])
    final_anchor = transform_mean(
        [_world_from_board(pair, final_transform) for pair in values]
    )
    final_residuals = _residuals(
        values, final_transform, final_anchor, tuple(range(len(values)))
    )
    all_translation_rms = _rms(
        [residual.translation_m for residual in final_residuals]
    )
    all_rotation_rms = _rms(
        [residual.rotation_deg for residual in final_residuals]
    )
    reprojection_rms = _rms(
        [pair.reprojection_rms_px for pair in values]
    )
    if (
        validation_translation_rms <= acceptance.target_translation_rms_m
        and validation_rotation_rms <= acceptance.target_rotation_rms_deg
    ):
        quality = "TARGET"
    elif (
        validation_translation_rms <= acceptance.maximum_translation_rms_m
        and validation_rotation_rms <= acceptance.maximum_rotation_rms_deg
    ):
        quality = "ACCEPTABLE"
    else:
        quality = "REJECTED"
    return CalibrationSolution(
        tracker_from_camera=final_transform,
        method=selected_score.method,
        pair_count=len(values),
        training_count=len(training),
        holdout_count=len(holdout),
        validation_translation_rms_m=validation_translation_rms,
        validation_rotation_rms_deg=validation_rotation_rms,
        all_translation_rms_m=all_translation_rms,
        all_rotation_rms_deg=all_rotation_rms,
        reprojection_rms_px=reprojection_rms,
        quality=quality,
        method_scores=tuple(method_scores),
        residuals=final_residuals,
    )
