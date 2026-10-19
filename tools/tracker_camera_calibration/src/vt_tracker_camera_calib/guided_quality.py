"""Incremental selection and independent final validation, without ROS or Qt.

The original offline solver keeps its method-selection holdout. Guided runs
reserve every fifth accepted pose *before* calling that solver, so those poses
never participate in method selection, fitting, or the board reference estimate.
"""
from __future__ import annotations

from collections import deque
import math

import numpy as np

from vt_capture_core.calibration_policy import GUIDED_VALIDATION_PAIRS, acceptance_defaults

from .clock import HostClockContinuity
from .motion import excitation_failure, motion_coverage
from .config import AcceptanceConfig, PairingConfig
from .handeye import method_selection_indices, solve_hand_eye
from .model import BoardObservation, CalibrationPair, TimedTransform
from .pairing import pair_static_observations, poses_are_distinct
from .transforms import rotation_distance_rad, transform_mean

POLICY_VERSION = 'guided-v1'


def collection_target(acceptance: AcceptanceConfig) -> int:
    return max(acceptance_defaults()["min_pairs"] + GUIDED_VALIDATION_PAIRS,
               math.ceil(acceptance.min_pairs * 5 / 4))


def split_validation_pairs(pairs, acceptance):
    values = tuple(pairs)
    required = collection_target(acceptance)
    if len(values) < required:
        raise ValueError(f'need at least {required} distinct pairs including independent validation')
    training = tuple(pair for index, pair in enumerate(values) if index % 5 != 4)
    validation = tuple(pair for index, pair in enumerate(values) if index % 5 == 4)
    return training, validation


def solve_with_independent_validation(pairs, acceptance):
    training, validation = split_validation_pairs(pairs, acceptance)
    solution = solve_hand_eye(training, acceptance)
    transform = solution.tracker_from_camera
    anchor = transform_mean([p.world_from_tracker @ transform @ p.camera_from_board for p in training])
    translations, rotations = [], []
    for pair in validation:
        estimate = pair.world_from_tracker @ transform @ pair.camera_from_board
        translations.append(float(np.linalg.norm(estimate.translation - anchor.translation)))
        rotations.append(math.degrees(rotation_distance_rad(anchor, estimate)))
    translation = float(np.sqrt(np.mean(np.square(translations))))
    rotation = float(np.sqrt(np.mean(np.square(rotations))))
    valid = (solution.quality != 'REJECTED'
             and translation <= acceptance.maximum_translation_rms_m
             and rotation <= acceptance.maximum_rotation_rms_deg)
    return solution, {
        'schema_version': 1, 'policy_version': POLICY_VERSION,
        'status': 'PASS' if valid else 'FAIL',
        'training_count': len(training), 'validation_count': len(validation),
        'translation_rms_m': translation, 'rotation_rms_deg': rotation,
        'training_timestamps_ns': [p.timestamp_ns for p in training],
        'validation_timestamps_ns': [p.timestamp_ns for p in validation],
        'reference': 'training_board_mean', 'used_for_fitting': False,
    }


def guidance(pairs, acceptance) -> tuple[bool, str, dict]:
    training = tuple(p for i, p in enumerate(pairs) if i % 5 != 4)
    metrics = motion_coverage(training)
    metrics.update(training_count=len(training), validation_count=len(pairs) // 5,
                   training_target=max(acceptance_defaults()["min_pairs"], acceptance.min_pairs),
                   validation_target=collection_target(acceptance) // 5)
    if len(pairs) < 8:
        return False, 'MOVE_TO_NEW_POSE', metrics
    failure = excitation_failure(metrics)
    if failure is not None:
        return False, failure[0], metrics
    if len(pairs) < collection_target(acceptance):
        return False, 'COLLECT_VALIDATION' if len(pairs) >= 40 else 'MOVE_TO_NEW_POSE', metrics
    # The original solver ranks methods on its own holdout. Its fitting subset
    # must be excited too; adequate aggregate coverage alone cannot prove this.
    selection_indices, _ = method_selection_indices(len(training), acceptance.holdout_fraction)
    selection = motion_coverage(tuple(training[i] for i in selection_indices))
    metrics['method_training_coverage'] = selection
    failure = excitation_failure(selection)
    if failure is not None:
        return False, failure[0], metrics
    return True, 'ROUND_COMPLETE', metrics


class IncrementalSampler:
    """Bounded timeline, complete stability windows, and globally distinct poses.

    Caller serializes access. Tracker samples and observations must arrive in
    their own increasing host-clock order. No interpolation across missing
    intervals is accepted. Overflow is a session failure, never silent success.
    """
    def __init__(self, config: PairingConfig, *, maximum_pairs: int = 500):
        self.config = config
        self.maximum_pairs = maximum_pairs
        self._tracker = deque(maxlen=8192)
        self._pending = deque()
        self._accepted = []
        self._tracker_clock = HostClockContinuity('TrackerSample')
        self._board_clock = HostClockContinuity('BoardObservation')
        self.latest_tracker_ns = -1
        self._latest_board_ns = -1
        self.hint_code = 'HOLD_STILL'

    @property
    def accepted(self):
        return tuple(self._accepted)

    @property
    def tracker_buffer_size(self):
        return len(self._tracker)

    def add_tracker(self, sample: TimedTransform):
        if sample.host_monotonic_ns is not None:
            self._tracker_clock.observe(sample.timestamp_ns, sample.host_monotonic_ns)
        if sample.timestamp_ns <= self.latest_tracker_ns:
            raise ValueError('non-increasing tracker timestamp')
        self.latest_tracker_ns = sample.timestamp_ns
        if len(self._tracker) == self._tracker.maxlen:
            raise ValueError('Tracker buffer capacity exceeded')
        self._tracker.append(sample)
        # Preserve enough history for a detection that takes up to one second.
        cutoff = sample.timestamp_ns - 2_000_000_000
        if self._pending:
            cutoff = min(cutoff, self._pending[0].timestamp_ns - 200_000_000)
        while len(self._tracker) > 2 and self._tracker[1].timestamp_ns < cutoff:
            self._tracker.popleft()

    def add_observation(self, observation: BoardObservation):
        if observation.host_monotonic_raw_ns is not None:
            self._board_clock.observe(observation.timestamp_ns, observation.host_monotonic_raw_ns)
        if observation.timestamp_ns <= self._latest_board_ns:
            raise ValueError('non-increasing board timestamp')
        self._latest_board_ns = observation.timestamp_ns
        if len(self._pending) >= 32:
            raise ValueError('Board observation buffer capacity exceeded')
        self._pending.append(observation)

    def advance(self):
        window = round(self.config.stability_window_ms * 1_000_000)
        separation = round(self.config.min_time_separation_s * 1_000_000_000)
        while self._pending and self._pending[0].timestamp_ns + window <= self.latest_tracker_ns:
            observation = self._pending.popleft()
            if len(self._tracker) < 2:
                continue
            candidates = pair_static_observations([observation], tuple(self._tracker), self.config)
            if not candidates:
                self.hint_code = 'HOLD_STILL'
                continue
            candidate = candidates[0]
            if (self._accepted and candidate.timestamp_ns - self._accepted[-1].timestamp_ns < separation
                    or not all(poses_are_distinct(candidate.world_from_tracker, old.world_from_tracker, self.config)
                               for old in self._accepted)):
                self.hint_code = 'MOVE_TO_NEW_POSE'
                continue
            if len(self._accepted) >= self.maximum_pairs:
                raise ValueError('Accepted-pose capacity exceeded; restart with broader motion')
            self._accepted.append(candidate)
            self.hint_code = 'MOVE_TO_NEW_POSE'


class StableGuidance:
    """Avoid frame-to-frame action flicker without hiding lost input or completion."""
    def __init__(self, minimum_duration_s: float = .6):
        self.minimum_duration_s = minimum_duration_s
        self._displayed = None
        self._candidate = None
        self._since = 0.

    def update(self, code: str, now: float) -> str:
        if self._displayed is None or code in {'WAITING_INPUT', 'SHOW_BOARD', 'ROUND_COMPLETE'}:
            self._displayed = code
        if code != self._candidate:
            self._candidate, self._since = code, now
        elif now - self._since >= self.minimum_duration_s:
            self._displayed = code
        return self._displayed
