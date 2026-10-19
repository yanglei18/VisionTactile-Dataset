from dataclasses import replace

import numpy as np
import pytest

from test_handeye import ACCEPTANCE, synthetic_pairs
from vt_tracker_camera_calib.config import PairingConfig
from vt_tracker_camera_calib.guided_quality import (
    IncrementalSampler, split_validation_pairs, solve_with_independent_validation,
)
from vt_tracker_camera_calib.model import BoardObservation, TimedTransform
from vt_tracker_camera_calib.pairing import pair_static_observations
from vt_tracker_camera_calib.transforms import Transform

PAIRING = PairingConfig(50, 100, .003, .5, .5, .02, 5)


def test_validation_samples_never_enter_solver(monkeypatch):
    import vt_tracker_camera_calib.guided_quality as quality
    pairs, _ = synthetic_pairs(50)
    training, validation = split_validation_pairs(pairs, ACCEPTANCE)
    assert len(training) == 40 and len(validation) == 10
    assert not set(p.timestamp_ns for p in training) & set(p.timestamp_ns for p in validation)
    original = quality.solve_hand_eye
    seen = []
    monkeypatch.setattr(quality, 'solve_hand_eye', lambda p, a: (seen.extend(p), original(p, a))[1])
    solution, report = solve_with_independent_validation(pairs, ACCEPTANCE)
    assert seen == list(training)
    assert report['status'] == 'PASS'
    assert report['translation_rms_m'] < 1e-8
    assert solution.pair_count == 40


def test_validation_failure_cannot_pass_training_quality():
    pairs, _ = synthetic_pairs(50)
    corrupted = tuple(replace(p, camera_from_board=Transform(p.camera_from_board.rotation,
        p.camera_from_board.translation + [.1, 0, 0])) if i % 5 == 4 else p
        for i, p in enumerate(pairs))
    solution, report = solve_with_independent_validation(corrupted, ACCEPTANCE)
    assert solution.quality == 'TARGET'
    assert report['status'] == 'FAIL'
    assert report['translation_rms_m'] > .05


def test_count_alone_does_not_satisfy_training_and_independent_validation():
    pairs, _ = synthetic_pairs(40)
    with pytest.raises(ValueError, match='50'):
        split_validation_pairs(pairs, ACCEPTANCE)


def test_incremental_sampler_matches_offline_and_waits_for_complete_window():
    pairs, _ = synthetic_pairs(6)
    tracker, observations = [], []
    for i, pair in enumerate(pairs):
        stamp = (i + 1) * 1_000_000_000
        tracker.extend(TimedTransform(stamp + offset, pair.world_from_tracker,
                                      stamp + offset + 1_000_000_000)
                       for offset in range(-150_000_000, 151_000_000, 25_000_000))
        observations.append(BoardObservation(stamp, pair.camera_from_board, .2, 35, stamp,
                                              stamp + 2_000_000_000))
    sampler = IncrementalSampler(PAIRING)
    for obs in observations:
        for sample in tracker:
            if sample.timestamp_ns <= obs.timestamp_ns and sample.timestamp_ns > sampler.latest_tracker_ns:
                sampler.add_tracker(sample)
        sampler.add_observation(obs)
        before = len(sampler.accepted)
        sampler.advance()
        assert len(sampler.accepted) == before
        for sample in tracker:
            if obs.timestamp_ns < sample.timestamp_ns <= obs.timestamp_ns + 150_000_000:
                sampler.add_tracker(sample)
        sampler.advance()
    assert [p.timestamp_ns for p in sampler.accepted] == [p.timestamp_ns for p in
        pair_static_observations(observations, tracker, PAIRING)]
    assert sampler.tracker_buffer_size < len(tracker)


def test_incremental_rejects_revisited_pose_and_clock_reset():
    pair = synthetic_pairs(1)[0][0]
    sampler = IncrementalSampler(PAIRING)
    for center in (1_000_000_000, 2_000_000_000):
        for offset in range(-150_000_000, 151_000_000, 25_000_000):
            sampler.add_tracker(TimedTransform(center + offset, pair.world_from_tracker,
                                               center + offset + 1_000_000_000))
        sampler.add_observation(BoardObservation(center, pair.camera_from_board, .2, 35, center,
                                                  center + 2_000_000_000))
        sampler.advance()
    assert len(sampler.accepted) == 1
    assert sampler.hint_code == 'MOVE_TO_NEW_POSE'
    with pytest.raises(ValueError, match='non-increasing'):
        sampler.add_tracker(TimedTransform(1, pair.world_from_tracker, 1))


def test_action_hint_waits_for_stable_change_but_input_loss_is_immediate():
    from vt_tracker_camera_calib.guided_quality import StableGuidance
    hints = StableGuidance()
    assert hints.update('HOLD_STILL', 0.) == 'HOLD_STILL'
    assert hints.update('MOVE_TO_NEW_POSE', .1) == 'HOLD_STILL'
    assert hints.update('HOLD_STILL', .2) == 'HOLD_STILL'
    assert hints.update('INCREASE_TRANSLATION', .3) == 'HOLD_STILL'
    assert hints.update('INCREASE_TRANSLATION', 1.) == 'INCREASE_TRANSLATION'
    assert hints.update('WAITING_INPUT', 1.1) == 'WAITING_INPUT'


def test_guidance_checks_excitation_of_solver_selection_subset():
    from vt_tracker_camera_calib.guided_quality import guidance
    pairs, _ = synthetic_pairs(50)
    # Diverse rotations occur only in the solver's method-selection holdout.
    # They cannot compensate for degenerate poses in the actual training subset.
    method_holdout = set(np.linspace(1, 38, 8, dtype=int))
    changed, training_index = [], 0
    for index, pair in enumerate(pairs):
        if index % 5 == 4:
            changed.append(pair)
            continue
        rotation = [.5, .2, 0.] if training_index in method_holdout else [0., 0., .02 * training_index]
        pose = Transform.from_rvec_tvec(rotation, [.01 * training_index, 0., 0.])
        changed.append(replace(pair, world_from_tracker=pose))
        training_index += 1
    ready, hint, _ = guidance(changed, ACCEPTANCE)
    assert ready is False
    assert hint == 'ROTATE_SECOND_AXIS'
