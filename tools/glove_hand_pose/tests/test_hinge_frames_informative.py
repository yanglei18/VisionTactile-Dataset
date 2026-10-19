"""Sign-evidence geometry, not independent physical calibration accuracy."""
import copy

import numpy as np
import pytest

from vt_glove_pose.hinge_frames import build_hinge_frames


def arguments(p, d=None):
    p = np.asarray(p, float)
    d = p.copy() if d is None else np.asarray(d, float)
    def acc(component):
        return np.column_stack((component, np.zeros(len(component)), np.sqrt(1-component**2)))
    return dict(axis_prox=[1., 0., 0.], axis_dist=[1., 0., 0.],
                prox_acc_g=acc(p), dist_acc_g=acc(d),
                neutral_prox_acc_g=[.2, 0., np.sqrt(.96)],
                neutral_dist_acc_g=[.2, 0., np.sqrt(.96)],
                sign_strategy="informative_blocks")


def test_informative_sign_survives_appended_uninformative_samples():
    short = arguments(np.full(180, .3))
    long = arguments(np.r_[np.full(180, .3), np.zeros(8100)])
    legacy = build_hinge_frames(**dict(long, sign_strategy="whole_capture"))
    assert legacy["status"] == "sign_unobservable"
    before, after = build_hinge_frames(**short), build_hinge_frames(**long)
    for result in (before, after):
        assert result["status"] == "frames_ready"
        assert result["relative_axis_sign_resolved_conditionally"] is True
        assert result["usable_as_live_calibration"] is False
        assert result["sign_metrics"]["block_evidence"]["supported_blocks"] == 3
    np.testing.assert_array_equal(before["dist_sensor_to_functional"], after["dist_sensor_to_functional"])
    assert before["sign_metrics"]["block_evidence"]["blocks"] == after["sign_metrics"]["block_evidence"]["blocks"]


@pytest.mark.parametrize("component", [0., .09])
def test_many_insufficient_components_do_not_accumulate_into_evidence(component):
    result = build_hinge_frames(**arguments(np.full(3000, component)))
    assert result["status"] == "sign_unobservable"
    assert result["sign_metrics"]["block_evidence"]["supported_blocks"] == 0
    assert result["dist_sensor_to_functional"] is None


def test_single_informative_point_per_block_is_not_a_vote():
    p = np.zeros(600)
    p[::60] = .5
    result = build_hinge_frames(**arguments(p))
    assert result["status"] == "sign_unobservable"
    assert result["sign_metrics"]["block_evidence"]["populated_blocks"] == 0


def test_one_strong_contrary_block_cannot_be_hidden_by_majority_or_padding():
    p, d = np.full(6000, .3), np.full(6000, .3)
    d[60:120] *= -1
    result = build_hinge_frames(**arguments(p, d))
    assert result["sign_metrics"]["selected_residual_p95"] == 0.
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["conflicting_informative_axis_sign_blocks"]
    assert result["dist_sensor_to_functional"] is None


def test_one_contaminated_point_does_not_flip_block_direction():
    p, d = np.full(240, .11), np.full(240, .11)
    p[0], d[0] = 1., -1.
    assert np.mean(p[:60]*d[:60]) < 0
    result = build_hinge_frames(**arguments(p, d))
    block = result["sign_metrics"]["block_evidence"]["blocks"][0]
    assert block["sign"] == 1
    assert result["status"] == "frames_ready"
    assert result["sign_metrics"]["selected_distal_sign_from_canonical_line"] == 1


def test_mixed_or_noisy_block_is_not_outvoted_by_good_blocks():
    p, d = np.full(6000, .3), np.full(6000, .3)
    d[60:90] *= -1
    result = build_hinge_frames(**arguments(p, d))
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["informative_axis_sign_block_inconsistent"]


def test_original_indices_prevent_compacting_sparse_points_into_valid_blocks():
    args = arguments(np.full(180, .3))
    assert build_hinge_frames(**args)["status"] == "frames_ready"
    args["sample_indices"] = np.arange(180)*60
    result = build_hinge_frames(**args)
    assert result["status"] == "sign_unobservable"
    assert result["sign_block_index_source"] == "caller_original_sample_indices"


def test_negative_sign_and_axis_line_sign_scaling_invariance():
    args = arguments(np.full(240, .3), np.full(240, -.3))
    args["neutral_dist_acc_g"][0] *= -1
    original = build_hinge_frames(**args)
    assert original["status"] == "frames_ready"
    assert original["sign_metrics"]["selected_distal_sign_from_canonical_line"] == -1
    changed = copy.deepcopy(args)
    changed["axis_prox"] = [-2., 0., 0.]
    changed["axis_dist"] = [-3., 0., 0.]
    assert build_hinge_frames(**changed) == original


def test_global_inconsistency_is_not_hidden_by_informative_selection():
    p = np.r_[np.full(180, .3), np.full(420, .09)]
    d = np.r_[np.full(180, .3), np.full(420, -.09)]
    result = build_hinge_frames(**arguments(p, d))
    assert result["sign_metrics"]["block_evidence"]["supported_blocks"] == 3
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["sequence_block_axis_gravity_components_inconsistent"]


def test_rms_outlier_cannot_suppress_persistent_opposite_block():
    p, d = np.full(6000, .11), np.full(6000, .11)
    d[60:120] *= -1
    p[60], d[60] = 1., 1.
    result = build_hinge_frames(**arguments(p, d))
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["sequence_block_axis_gravity_components_inconsistent"]


def test_weak_projection_block_can_contradict_even_without_a_sign_vote():
    p, d = np.full(6000, .3), np.full(6000, .3)
    p[60:120], d[60:120] = .09, -.09
    result = build_hinge_frames(**arguments(p, d))
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["sequence_block_axis_gravity_components_inconsistent"]


def test_informative_subset_residual_cannot_be_hidden_by_empty_blocks():
    p = np.r_[np.full(180, .3), np.tile(np.r_[np.full(2, .3), np.zeros(58)], 100)]
    d = p.copy()
    d[180:] *= -1
    result = build_hinge_frames(**arguments(p, d))
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["informative_axis_gravity_components_inconsistent"]


@pytest.mark.parametrize("key,value", [
    ("sign_strategy", "guess"), ("sign_strategy", ["informative_blocks"]),
    ("sign_block_samples", True), ("sign_block_samples", 20),
    ("min_sign_blocks", 1), ("min_sign_blocks", 2.5),
    ("sample_indices", [0]*180), ("sample_indices", list(range(179))),
    ("sample_indices", [True]+list(range(1, 180))),
    ("sample_indices", list(np.arange(180, dtype=float))),
])
def test_invalid_block_configuration(key, value):
    args = arguments(np.full(180, .3))
    args[key] = value
    with pytest.raises(ValueError):
        build_hinge_frames(**args)
