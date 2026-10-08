from __future__ import annotations

from bisect import bisect_left
import math

import numpy as np

from .config import PairingConfig
from .model import BoardObservation, CalibrationPair, TimedTransform
from .transforms import Transform, rotation_distance_rad


def _validated_tracker_samples(
    samples: list[TimedTransform] | tuple[TimedTransform, ...],
) -> tuple[TimedTransform, ...]:
    values = tuple(sorted(samples, key=lambda sample: sample.timestamp_ns))
    if len(values) < 2:
        raise ValueError("at least two tracker samples are required")
    timestamps = [sample.timestamp_ns for sample in values]
    if len(timestamps) != len(set(timestamps)):
        raise ValueError("tracker timestamps must be unique")
    return values


def interpolate_tracker_pose(
    samples: tuple[TimedTransform, ...],
    timestamp_ns: int,
    max_gap_ns: int,
) -> tuple[Transform, int] | None:
    timestamps = [sample.timestamp_ns for sample in samples]
    right = bisect_left(timestamps, timestamp_ns)
    if right < len(samples) and samples[right].timestamp_ns == timestamp_ns:
        return samples[right].transform, 0
    if right == 0 or right == len(samples):
        return None
    left = right - 1
    before = samples[left]
    after = samples[right]
    gap = after.timestamp_ns - before.timestamp_ns
    if gap > max_gap_ns:
        return None
    fraction = (timestamp_ns - before.timestamp_ns) / gap
    return before.transform.interpolate(after.transform, fraction), gap


def _local_motion(
    samples: tuple[TimedTransform, ...],
    timestamp_ns: int,
    window_ns: int,
    max_gap_ns: int,
) -> tuple[float, float] | None:
    before = interpolate_tracker_pose(samples, timestamp_ns - window_ns, max_gap_ns)
    after = interpolate_tracker_pose(samples, timestamp_ns + window_ns, max_gap_ns)
    if before is None or after is None:
        return None
    translation = float(
        np.linalg.norm(after[0].translation - before[0].translation)
    )
    rotation_deg = math.degrees(rotation_distance_rad(before[0], after[0]))
    return translation, rotation_deg


def pair_static_observations(
    board_observations: list[BoardObservation] | tuple[BoardObservation, ...],
    tracker_samples: list[TimedTransform] | tuple[TimedTransform, ...],
    config: PairingConfig,
) -> tuple[CalibrationPair, ...]:
    samples = _validated_tracker_samples(tracker_samples)
    observations = tuple(
        sorted(board_observations, key=lambda observation: observation.timestamp_ns)
    )
    max_gap_ns = round(config.max_interpolation_gap_ms * 1_000_000)
    window_ns = round(config.stability_window_ms * 1_000_000)
    candidates: list[CalibrationPair] = []
    for observation in observations:
        interpolated = interpolate_tracker_pose(
            samples, observation.timestamp_ns, max_gap_ns
        )
        if interpolated is None:
            continue
        motion = _local_motion(
            samples, observation.timestamp_ns, window_ns, max_gap_ns
        )
        if motion is None:
            continue
        translation_motion, rotation_motion_deg = motion
        if (
            translation_motion > config.max_static_translation_m
            or rotation_motion_deg > config.max_static_rotation_deg
        ):
            continue
        candidates.append(
            CalibrationPair(
                timestamp_ns=observation.timestamp_ns,
                world_from_tracker=interpolated[0],
                camera_from_board=observation.camera_from_board,
                reprojection_rms_px=observation.reprojection_rms_px,
                corner_count=observation.corner_count,
                interpolation_gap_ns=interpolated[1],
                local_translation_motion_m=translation_motion,
                local_rotation_motion_deg=rotation_motion_deg,
            )
        )

    selected: list[CalibrationPair] = []
    minimum_time_ns = round(config.min_time_separation_s * 1_000_000_000)
    for candidate in candidates:
        if selected and candidate.timestamp_ns - selected[-1].timestamp_ns < minimum_time_ns:
            continue
        if selected:
            diverse = any(
                float(
                    np.linalg.norm(
                        candidate.world_from_tracker.translation
                        - existing.world_from_tracker.translation
                    )
                )
                >= config.min_pose_translation_m
                or math.degrees(
                    rotation_distance_rad(
                        existing.world_from_tracker,
                        candidate.world_from_tracker,
                    )
                )
                >= config.min_pose_rotation_deg
                for existing in selected
            )
            if not diverse:
                continue
        selected.append(candidate)
    return tuple(selected)
