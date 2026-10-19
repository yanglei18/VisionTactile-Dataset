"""Opt-in robust A calibration: bounded transients, not looser physics gates."""
from copy import deepcopy
import json

import numpy as np
import pytest

from vt_glove_pose.math3d import from_axis_angle, to_matrix
from vt_glove_pose.thumb_calibration import (
    ThumbCalibrationError, calibrate_thumb_pair, fit_thumb_common_rotation,
)
from test_thumb_calibration import make_capture, common_kwargs, rx


@pytest.fixture
def capture():
    return make_capture()


def fit(data):
    return fit_thumb_common_rotation(**common_kwargs(data), method="robust_v2")


def test_default_strict_method_preserves_exact_result(capture):
    data, _ = capture
    assert fit_thumb_common_rotation(**common_kwargs(data)) == fit_thumb_common_rotation(
        **common_kwargs(data), method="strict_v1")
    assert calibrate_thumb_pair(**data) == calibrate_thumb_pair(**data, method="strict_v1")


def test_clean_capture_has_independent_holdout_and_compact_json(capture):
    data, truth = capture
    result = fit(data)
    np.testing.assert_allclose(result["rotation"], truth["rotation"], atol=1e-12)
    quality = result["quality"]
    assert quality["effective_fraction"] == 1.
    assert quality["retained_motion_fraction"] == 1.
    assert quality["train"]["moving_samples"] > 100
    assert quality["holdout"]["moving_samples"] > 100
    assert quality["holdout"]["gyro_residual_rms_dps"] < 1e-10
    assert quality["retained_segment_count"] == 1
    assert quality["longest_retained_segment_s"] == 8.
    assert len(json.dumps(result, ensure_ascii=False, allow_nan=False).encode()) < 16000


@pytest.mark.parametrize("sensor", ["common_prox_gyro_dps", "common_dist_gyro_dps"])
def test_two_low_amplitude_samples_no_longer_reject_an_entire_capture(capture, sensor):
    data, truth = capture
    data[sensor][250:252] = 0.
    with pytest.raises(ThumbCalibrationError, match="amplitude"):
        fit_thumb_common_rotation(**common_kwargs(data))
    result = fit(data)
    np.testing.assert_allclose(result["rotation"], truth["rotation"], atol=1e-12)
    assert .9 < result["quality"]["effective_fraction"] < 1.


@pytest.mark.parametrize("start", [140, 200])
def test_small_direction_transients_rejected_with_training_and_holdout_separate(capture, start):
    data, truth = capture
    original = fit(data)
    # start=140 lies in a train block; 200 in a held-out block. Equal magnitudes
    # deliberately pass the amplitude prefilter, exercising robust rotation.
    disturbance = to_matrix(from_axis_angle([.4, 1., 2.], 1.0))
    data["common_dist_gyro_dps"][start:start+8] = data["common_dist_gyro_dps"][start:start+8] @ disturbance
    result = fit(data)
    np.testing.assert_allclose(result["rotation"], truth["rotation"], atol=1e-12)
    assert result["quality"]["retained_motion_fraction"] >= .8
    if start == 200:
        # Holdout perturbations must not affect a single fitted coefficient.
        np.testing.assert_array_equal(result["rotation"], original["rotation"])


def test_holdout_continuity_cannot_indirectly_select_training_vectors(capture):
    data, _ = capture
    data["common_dist_gyro_dps"][:50] = 0.
    # Train rows [50:60] have only ten locally reliable samples. In the old
    # preprocessor, whether they reached a 30-sample segment depended on the
    # next holdout block [60:120]. A slight train perturbation makes that
    # hidden selection leak measurable even when both captures are accepted.
    disturbance = to_matrix(from_axis_angle([.4, 1., 2.], .04))
    data["common_dist_gyro_dps"][50:60] = data["common_dist_gyro_dps"][50:60] @ disturbance
    changed = deepcopy(data)
    changed["common_dist_gyro_dps"][60:80] = 0.
    original, modified = fit(data), fit(changed)
    np.testing.assert_array_equal(original["rotation"], modified["rotation"])
    assert original["quality"]["train_fitting_samples"] == modified["quality"]["train_fitting_samples"] == 430
    # Validation still uses the complete original timeline: changed holdout
    # quality legitimately changes continuity/coverage, not fitted rotation.
    assert original["quality"]["train_candidate_samples"] == 430
    assert modified["quality"]["train_candidate_samples"] == 420
    assert original["quality"]["effective_fraction"] > modified["quality"]["effective_fraction"] >= .8


def test_short_non_saturated_acceleration_transient_can_be_excluded(capture):
    data, truth = capture
    data["common_dist_acc_g"][360:365] *= 2.5
    result = fit(data)
    np.testing.assert_allclose(result["rotation"], truth["rotation"], atol=1e-12)
    assert .9 < result["quality"]["effective_fraction"] < 1.


def test_few_bad_frames_do_not_fragment_good_runs_at_holdout_boundaries(capture):
    data, _ = capture
    # Three 4-frame interruptions, including one at a split boundary. The long
    # surviving intervals must not be split into artificial <.25-second pieces.
    for start in (58, 418, 778):
        data["common_dist_gyro_dps"][start:start+4] = 0.
    result = fit(data)
    assert result["quality"]["effective_fraction"] == pytest.approx(948/960)
    assert result["quality"]["retained_segment_count"] == 4


@pytest.mark.parametrize("kind", ["scale", "half_rotation", "holdout_rotation", "frequent_outliers"])
def test_broad_disagreement_cannot_be_hidden_by_selecting_good_samples(capture, kind):
    data, _ = capture
    transform = to_matrix(from_axis_angle([.4, 1., 2.], .6))
    if kind == "scale":
        data["common_dist_gyro_dps"] *= 1.3
    elif kind == "half_rotation":
        data["common_dist_gyro_dps"][480:] = data["common_dist_gyro_dps"][480:] @ transform
        data["common_dist_acc_g"][480:] = data["common_dist_acc_g"][480:] @ transform
    elif kind == "holdout_rotation":
        holdout = (np.arange(960)//60) % 2 == 1
        data["common_dist_gyro_dps"][holdout] = data["common_dist_gyro_dps"][holdout] @ transform
        data["common_dist_acc_g"][holdout] = data["common_dist_acc_g"][holdout] @ transform
    else:
        data["common_dist_gyro_dps"][::4] *= .4
    with pytest.raises(ThumbCalibrationError) as info:
        fit(data)
    assert info.value.details["method"] == "robust_v2"
    assert "当前不能确定物理原因" in str(info.value)
    assert "thumb shape was not fixed" not in str(info.value)


def test_absolute_residual_gate_remains_strict_even_when_relative_error_is_small(capture):
    data, _ = capture
    # 8% is below the amplitude/relative rejection thresholds, but at these
    # speeds it exceeds the unchanged 2 degrees/s absolute RMS gate.
    data["common_dist_gyro_dps"] *= 1.08
    with pytest.raises(ThumbCalibrationError) as info:
        fit(data)
    assert info.value.details["failure_code"] == "validation_residual"
    assert max(info.value.details[name]["gyro_residual_rms_dps"] for name in ("train", "holdout")) > 2.
    assert "门限仍为 2/4/0.12/8°" in str(info.value)


def test_gravity_validation_is_not_replaced_by_good_gyro_fit(capture):
    data, truth = capture
    data["common_prox_acc_g"][:] = [0., 0., 1.]
    data["common_dist_acc_g"][:] = np.array([np.sin(np.deg2rad(10)), 0., np.cos(np.deg2rad(10))]) @ truth["rotation"]
    with pytest.raises(ThumbCalibrationError) as info:
        fit(data)
    assert info.value.details["failure_code"] == "validation_residual"
    assert info.value.details["holdout"]["gravity_direction_p95_deg"] == pytest.approx(10.)


def test_one_axis_is_not_enough_even_with_perfect_residual(capture):
    data, truth = capture
    data["common_prox_gyro_dps"][:] = [15., 0., 0.]
    data["common_dist_gyro_dps"][:] = np.array([15., 0., 0.]) @ truth["rotation"]
    with pytest.raises(ThumbCalibrationError) as info:
        fit(data)
    assert info.value.details["failure_code"] == "train_excitation"


@pytest.mark.parametrize("raw_mask", [False, True])
def test_even_one_saturated_frame_still_hard_fails(capture, raw_mask):
    data, _ = capture
    kwargs = common_kwargs(data)
    if raw_mask:
        kwargs["saturated"] = np.zeros((960, 2), dtype=bool)
        kwargs["saturated"][3, 1] = True
    else:
        data["common_dist_gyro_dps"][3, 0] = 286.
    with pytest.raises(ThumbCalibrationError) as info:
        fit_thumb_common_rotation(**kwargs, method="robust_v2")
    assert info.value.details["failure_code"] == "saturation"
    assert info.value.details["saturated_rows"] == 1


def test_robust_calibrate_revalidates_raw_a_and_does_not_mutate_inputs(capture):
    data, truth = capture
    data["common_dist_gyro_dps"][250:252] = 0.
    original = deepcopy(data)
    result = calibrate_thumb_pair(**data, method="robust_v2")
    np.testing.assert_allclose(result["shared_axis_prox"], truth["axis_prox"], atol=1e-12)
    assert result["quality"]["stage_b_method"] == "strict_hinge_v1_no_trimming"
    assert result["quality"]["common_motion"]["method"] == "robust_v2"
    assert len(json.dumps(result, ensure_ascii=False, allow_nan=False).encode()) < 16000
    for key in data:
        np.testing.assert_array_equal(data[key], original[key])
    data["common_dist_gyro_dps"] *= 1.3
    with pytest.raises(ThumbCalibrationError) as info:
        calibrate_thumb_pair(**data, method="robust_v2")
    assert info.value.details["stage"] == "A"


def test_robust_b_reports_actual_proximal_motion_and_preserves_strict_gate(capture):
    data, _ = capture
    data["flex_prox_gyro_dps"][:, 0] = 3.
    with pytest.raises(ThumbCalibrationError, match="proximal segment moved"):
        calibrate_thumb_pair(**data)
    with pytest.raises(ThumbCalibrationError) as info:
        calibrate_thumb_pair(**data, method="robust_v2")
    assert info.value.details["stage"] == "B"
    assert info.value.details["proximal_gyro_p95_dps"] == 3.
    assert "p95=3.000" in str(info.value)
    assert "门限分别 2/5" in str(info.value)


def test_robust_b_degenerate_gravity_cannot_leak_nan_quality(capture):
    data, _ = capture
    data["flex_prox_acc_g"][480:] *= -1.
    with pytest.raises(ThumbCalibrationError) as info:
        calibrate_thumb_pair(**data, method="robust_v2")
    assert info.value.details["failure_code"] == "proximal_gravity_degenerate"
    json.dumps(info.value.details, allow_nan=False)


def test_robust_b_never_discards_a_short_ambiguous_first_bout(capture):
    data, truth = capture
    data["flex_dist_gyro_dps"][:3] = truth["axis_dist"] * 60
    data["flex_dist_gyro_dps"][3:6] = 0.
    with pytest.raises(ThumbCalibrationError) as info:
        calibrate_thumb_pair(**data, method="robust_v2")
    assert info.value.details["failure_code"] == "first_flexion_ambiguous"
    assert info.value.details["first_moving_sample"] == 0
    assert "不能删掉首段" in str(info.value)


def test_robust_b_preserves_first_flexion_before_old_point_four_trim(capture):
    data, truth = capture
    speed = np.zeros(960)
    speed[:36] = 60.
    speed[36:72] = -60.
    for start in range(120, 840, 240):
        speed[start:start+120] = 30.
        speed[start+120:start+240] = -30.
    theta = .2 + np.deg2rad(np.cumsum(speed) / 120)
    gravity = truth["mp"] @ data["neutral_prox_acc_g"]
    data["flex_dist_gyro_dps"] = speed[:, None] * truth["axis_dist"]
    data["flex_dist_acc_g"] = np.asarray([rx(angle) @ gravity for angle in theta]) @ truth["md"]
    result = calibrate_thumb_pair(**data, method="robust_v2")
    np.testing.assert_allclose(result["shared_axis_dist"], truth["axis_dist"], atol=1e-12)


@pytest.mark.parametrize("bad", ["unknown", None, True])
def test_method_is_explicit_and_not_silently_accepted(capture, bad):
    data, _ = capture
    with pytest.raises(ThumbCalibrationError, match="method"):
        fit_thumb_common_rotation(**common_kwargs(data), method=bad)


def test_nonfinite_inputs_produce_chinese_robust_failure_details(capture):
    data, _ = capture
    data["common_dist_acc_g"][20, 0] = np.nan
    with pytest.raises(ThumbCalibrationError) as info:
        fit(data)
    assert info.value.details["failure_code"] == "input_invalid"
    assert "有限数值" in str(info.value)
