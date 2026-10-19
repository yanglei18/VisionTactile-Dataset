"""Recovery advice follows exact evidence and does not modify fit thresholds."""
from copy import deepcopy
import json

import pytest

from vt_glove_pose.hinge_recovery import classify_hinge_failure


@pytest.mark.parametrize("status,codes,action,neutral,review,illustration", [
    ("frames_neutral_unobservable", ["explicit_neutral_gravity_nearly_parallel_to_hinge"],
     "neutral", True, False, "rotate"),
    ("frames_neutral_unobservable", [], "neutral", True, False, "rotate"),
    ("frames_inconsistent", ["explicit_neutral_acceleration_norm_inconsistent"],
     "export", True, True, "export"),
    ("frames_inconsistent", ["explicit_neutral_axis_gravity_mismatch"],
     "export", False, True, "export"),
    ("frames_sign_unobservable", ["insufficient_parallel_gravity_component_to_resolve_axis_sign"],
     "motion", False, False, "rotate"),
    ("axis_insufficient_excitation", ["training_tangent_sensitivity_too_small"],
     "motion", False, False, "flex"),
    ("axis_insufficient_excitation", ["training_motion_below_noise"],
     "motion", False, False, "flex"),
    ("frames_inconsistent", ["insufficient_norm_consistent_acceleration"],
     "export", False, True, "export"),
    ("frames_inconsistent", ["paired_axis_gravity_components_inconsistent"],
     "export", False, True, "export"),
    ("neutral_invalid", ["neutral_gyro_not_quiet"], "neutral", True, False, "still"),
    ("neutral_invalid", ["neutral_pair_count_insufficient"], "neutral", True, False, "still"),
    ("input_near_limit", ["motion_raw_near_limit"], "motion", False, False, "flex"),
    ("axis_inconsistent", ["optimizer_not_converged"], "export", False, True, "export"),
])
def test_exact_failure_routes(status, codes, action, neutral, review, illustration):
    before = list(codes)
    result = classify_hinge_failure(status, codes)
    assert result["suggested_action"] == action
    assert result["requires_new_neutral"] is neutral
    assert result["needs_review"] is review
    assert result["illustration"] == illustration
    assert result["profile_status"] == status and result["reason_codes"] == codes
    assert all(result[key] for key in ("title", "instruction", "detail", "stage"))
    result["reason_codes"].append("mutation")
    assert codes == before
    json.dumps(result, allow_nan=False)


def test_geometry_instructions_require_changed_hand_tilt_not_same_pose_repetition():
    result = classify_hinge_failure("frames_neutral_unobservable", [])
    assert "保持手指形状不变" in result["instruction"]
    assert "改变整手倾斜" in result["instruction"]
    assert "原来的整手朝向" in result["detail"]


def test_reference_axis_mismatch_does_not_claim_which_source_is_wrong():
    result = classify_hinge_failure("frames_inconsistent", ["explicit_neutral_axis_gravity_mismatch"])
    assert all(word in result["detail"] for word in ("运动轴估计", "固定", "参考数据", "不能断定"))
    assert not result["requires_new_neutral"]


def test_unknown_failures_are_review_not_substring_based_neutral_or_motion_retry():
    for status, codes in [("unknown_neutral_problem", ["new_unknown_code"]),
                           ("axis_insufficient_excitation", ["unrecognized_failure"]),
                           ("fit_exception", ["unclassified_fit_exception"]),
                           ("configuration_error", [])]:
        result = classify_hinge_failure(status, codes)
        assert result["suggested_action"] == "export" and result["needs_review"]
        assert not result["requires_new_neutral"]


def test_measured_acceleration_variation_can_retry_but_norm_or_missing_evidence_cannot():
    codes = ["neutral_acceleration_not_stable_gravity_like"]
    metrics = dict(mean_accel_norm_g=[.99, 1.01], accel_vector_std_g=[.04, .02])
    before = deepcopy(metrics)
    result = classify_hinge_failure("neutral_invalid", codes, metrics=metrics)
    assert result["suggested_action"] == "neutral" and not result["needs_review"]
    assert "波动" in result["title"] and metrics == before
    for invalid in (None, {}, dict(mean_accel_norm_g=[.7, 1.], accel_vector_std_g=[.04, .02]),
                    dict(mean_accel_norm_g=[.99, 1.01], accel_vector_std_g=[.01, .02])):
        result = classify_hinge_failure("neutral_invalid", codes, metrics=invalid)
        assert result["suggested_action"] == "export" and result["needs_review"]
        assert result["requires_new_neutral"]


def test_composite_calibrator_neutral_reason_uses_actual_metrics_without_accepting_it():
    result = classify_hinge_failure("neutral_invalid", ["neutral_not_quiet_or_gravity_like"],
        metrics=dict(mean_accel_norm_g=[1., 1.], accel_vector_std_g=[.031, .01]))
    assert result["suggested_action"] == "neutral" and result["requires_new_neutral"]
    assert result["profile_status"] == "neutral_invalid"


@pytest.mark.parametrize("status,codes", [(None, []), ("", []), ("failed", "single_string"), ("failed", [1])])
def test_invalid_api_input_raises_value_error(status, codes):
    with pytest.raises(ValueError):
        classify_hinge_failure(status, codes)
