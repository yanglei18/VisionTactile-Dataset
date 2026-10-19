from __future__ import annotations

from bisect import bisect_left, bisect_right
import math

import numpy as np

from .clock import validate_optional_clock_sequence
from .config import PairingConfig
from .model import BoardObservation, CalibrationPair, TimedTransform
from .transforms import Transform, rotation_distance_rad


class _TrackerTimeline:
    """Validated samples and a reusable timestamp index for one pairing run."""

    def __init__(
        self, samples: list[TimedTransform] | tuple[TimedTransform, ...]
    ) -> None:
        validate_optional_clock_sequence(
            "TrackerSample",
            ((value.timestamp_ns, value.host_monotonic_ns) for value in samples),
        )
        self.samples = tuple(sorted(samples, key=lambda sample: sample.timestamp_ns))
        if len(self.samples) < 2:
            raise ValueError("at least two tracker samples are required")
        self.timestamps = tuple(sample.timestamp_ns for sample in self.samples)
        if any(
            left == right
            for left, right in zip(self.timestamps, self.timestamps[1:])
        ):
            raise ValueError("tracker timestamps must be unique")

    def interpolate(
        self, timestamp_ns: int, max_gap_ns: int
    ) -> tuple[Transform, int] | None:
        right = bisect_left(self.timestamps, timestamp_ns)
        if right < len(self.samples) and self.timestamps[right] == timestamp_ns:
            return self.samples[right].transform, 0
        if right == 0 or right == len(self.samples):
            return None
        before = self.samples[right - 1]
        after = self.samples[right]
        gap = after.timestamp_ns - before.timestamp_ns
        if gap > max_gap_ns:
            return None
        fraction = (timestamp_ns - before.timestamp_ns) / gap
        return before.transform.interpolate(after.transform, fraction), gap

    def window_motion(
        self, timestamp_ns: int, window_ns: int, max_gap_ns: int
    ) -> tuple[float, float] | None:
        """Diameter of measured poses and interpolated window endpoints.

        Both endpoints must be covered, and every interval touching the window
        must satisfy the interpolation gap limit. Returning to the starting pose
        cannot hide an excursion or a period without observations.
        """
        start_ns, stop_ns = timestamp_ns - window_ns, timestamp_ns + window_ns
        before = self.interpolate(start_ns, max_gap_ns)
        after = self.interpolate(stop_ns, max_gap_ns)
        if before is None or after is None:
            return None
        start = bisect_right(self.timestamps, start_ns)
        stop = bisect_left(self.timestamps, stop_ns)
        interior = self.samples[start:stop]
        timestamps = (start_ns, *(sample.timestamp_ns for sample in interior), stop_ns)
        if any(
            right - left > max_gap_ns
            for left, right in zip(timestamps, timestamps[1:])
        ):
            return None
        poses = (before[0], *(sample.transform for sample in interior), after[0])
        translations = np.stack([pose.translation for pose in poses])
        rotations = np.stack([pose.rotation for pose in poses])
        maximum_translation = 0.0
        minimum_cosine = 1.0
        # O(window samples) temporary memory; no recording-sized pairwise matrix.
        for index in range(len(poses) - 1):
            distances = np.linalg.norm(
                translations[index + 1:] - translations[index], axis=1
            )
            maximum_translation = max(maximum_translation, float(np.max(distances)))
            traces = np.einsum("ij,nij->n", rotations[index], rotations[index + 1:])
            minimum_cosine = min(
                minimum_cosine, float(np.min((traces - 1.0) / 2.0))
            )
        maximum_rotation = math.degrees(math.acos(max(-1.0, minimum_cosine)))
        return maximum_translation, maximum_rotation


def interpolate_tracker_pose(
    samples: tuple[TimedTransform, ...], timestamp_ns: int, max_gap_ns: int
) -> tuple[Transform, int] | None:
    """Interpolate a single query; batch pairing reuses one timeline index."""
    return _TrackerTimeline(samples).interpolate(timestamp_ns, max_gap_ns)


def poses_are_distinct(
    candidate: Transform, existing: Transform, config: PairingConfig
) -> bool:
    return (
        float(np.linalg.norm(candidate.translation - existing.translation))
        >= config.min_pose_translation_m
        or math.degrees(rotation_distance_rad(existing, candidate))
        >= config.min_pose_rotation_deg
    )


def pair_static_observations(
    board_observations: list[BoardObservation] | tuple[BoardObservation, ...],
    tracker_samples: list[TimedTransform] | tuple[TimedTransform, ...],
    config: PairingConfig,
) -> tuple[CalibrationPair, ...]:
    timeline = _TrackerTimeline(tracker_samples)
    validate_optional_clock_sequence(
        "BoardObservation",
        (
            (value.timestamp_ns, value.host_monotonic_raw_ns)
            for value in board_observations
        ),
    )
    observations = tuple(sorted(board_observations, key=lambda value: value.timestamp_ns))
    if any(
        left.timestamp_ns == right.timestamp_ns
        for left, right in zip(observations, observations[1:])
    ):
        raise ValueError("observation timestamps must be unique")
    max_gap_ns = round(config.max_interpolation_gap_ms * 1_000_000)
    window_ns = round(config.stability_window_ms * 1_000_000)
    minimum_time_ns = round(config.min_time_separation_s * 1_000_000_000)
    selected: list[CalibrationPair] = []
    for observation in observations:
        if (
            selected
            and observation.timestamp_ns - selected[-1].timestamp_ns < minimum_time_ns
        ):
            continue
        interpolated = timeline.interpolate(observation.timestamp_ns, max_gap_ns)
        if interpolated is None:
            continue
        if not all(
            poses_are_distinct(interpolated[0], existing.world_from_tracker, config)
            for existing in selected
        ):
            continue
        motion = timeline.window_motion(observation.timestamp_ns, window_ns, max_gap_ns)
        if motion is None:
            continue
        translation_motion, rotation_motion_deg = motion
        if (
            translation_motion > config.max_static_translation_m
            or rotation_motion_deg > config.max_static_rotation_deg
        ):
            continue
        selected.append(
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
    return tuple(selected)
