"""Independent synthetic trajectories: physical two-link hinge, no devices."""
import json

import numpy as np
import pytest

from vt_glove_pose.hinge_axes import fit_hinge_axes, _basis, _residual_jacobian
from vt_glove_pose.math3d import from_axis_angle, to_matrix


def hinge_capture(seed=4, noise=0., n=1200):
    """Common 3-axis rotation plus relative hinge rotation, random installations.

    The proximal body angular velocity is arbitrary; distal velocity is that
    common velocity expressed in the rotated distal frame, plus hinge rate.
    Neither link is held fixed. No solver code generates the true axes.
    """
    rng = np.random.default_rng(seed)
    mp = to_matrix(from_axis_angle(rng.normal(size=3), rng.uniform(-2, 2)))
    md = to_matrix(from_axis_angle(rng.normal(size=3), rng.uniform(-2, 2)))
    rate = 120.
    t = np.arange(n) / rate
    theta = .8 * np.sin(1.7 * t) + .25 * np.sin(3.1 * t + .2)
    theta_dot = np.rad2deg(1.36 * np.cos(1.7 * t) + .775 * np.cos(3.1 * t + .2))
    proximal = np.column_stack((23. * np.sin(1.2 * t),
                               31. * np.cos(.85 * t),
                               17. * np.sin(1.4 * t + .4)))
    distal = []
    for angle, speed, omega in zip(theta, theta_dot, proximal):
        rotation = to_matrix(from_axis_angle([1., 0., 0.], angle))
        distal.append(rotation.T @ omega + np.array([speed, 0., 0.]))
    gp, gd = proximal @ mp, np.asarray(distal) @ md
    gp += rng.normal(0, noise, gp.shape)
    gd += rng.normal(0, noise, gd.shape)
    return gp, gd, mp.T @ [1., 0., 0.], md.T @ [1., 0., 0.]


def line_error_deg(actual, expected):
    return float(np.degrees(np.arccos(np.clip(abs(np.dot(actual, expected)), 0., 1.))))


@pytest.mark.parametrize("seed", [3, 9, 22])
def test_recovers_axis_lines_with_both_links_moving_and_random_installation(seed):
    gp, gd, jp, jd = hinge_capture(seed=seed, noise=.08)
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    assert result["candidate_status"] == "candidate_supported", result["reason_codes"]
    assert line_error_deg(result["axis_prox"], jp) < .25
    assert line_error_deg(result["axis_dist"], jd) < .25
    assert result["identifiability"]["numerical_rank"] == 4
    assert result["holdout"]["p95_abs_dps"] < .4
    assert not result["usable_as_live_calibration"]
    assert not result["anatomical_accuracy_verified"]
    assert result["axis_sign_convention"] == "independent_axis_line_signs_unresolved"
    json.dumps(result, allow_nan=False)


def test_independent_axis_signs_leave_residual_unchanged():
    gp, gd, jp, jd = hinge_capture(n=240)
    original = _residual_jacobian(gp, gd, jp, jd)[0]
    for p_sign, d_sign in ((1, -1), (-1, 1), (-1, -1)):
        residual = _residual_jacobian(gp, gd, p_sign * jp, d_sign * jd)[0]
        np.testing.assert_array_equal(residual, original)


def test_tangent_jacobian_matches_finite_differences():
    gp, gd, _, _ = hinge_capture(n=240)
    jp = np.array([.2, .4, .8]); jp /= np.linalg.norm(jp)
    jd = np.array([.8, -.2, .1]); jd /= np.linalg.norm(jd)
    _, jac, bp, bd = _residual_jacobian(gp, gd, jp, jd)
    epsilon = 1e-6
    for k in range(4):
        pstep = bp[:, k] if k < 2 else np.zeros(3)
        dstep = bd[:, k - 2] if k >= 2 else np.zeros(3)
        plus_p, minus_p = jp + epsilon * pstep, jp - epsilon * pstep
        plus_d, minus_d = jd + epsilon * dstep, jd - epsilon * dstep
        plus = _residual_jacobian(gp, gd, plus_p / np.linalg.norm(plus_p), plus_d / np.linalg.norm(plus_d))[0]
        minus = _residual_jacobian(gp, gd, minus_p / np.linalg.norm(minus_p), minus_d / np.linalg.norm(minus_d))[0]
        np.testing.assert_allclose((plus - minus) / (2 * epsilon), jac[:, k], atol=1e-5)
    np.testing.assert_allclose(_basis(jp).T @ _basis(jp), np.eye(2), atol=1e-14)


@pytest.mark.parametrize("noise", [0., .05])
def test_static_data_never_returns_supported(noise):
    rng = np.random.default_rng(4)
    gp, gd = rng.normal(0, noise, (2, 600, 3))
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    assert result["candidate_status"] == "insufficient_excitation"
    assert result["reason_codes"] == ["training_motion_below_noise"]
    assert result["axis_prox"] is None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("noise", [0., .08])
def test_common_rigid_motion_rejected_despite_low_residual(noise):
    gp, _, _, _ = hinge_capture(n=600)
    rotation = to_matrix(from_axis_angle([2., -.4, .8], .9))
    rng = np.random.default_rng(6)
    gd = gp @ rotation + rng.normal(0, noise, gp.shape)
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    assert result["candidate_status"] == "insufficient_excitation"
    assert "training_compatible_with_common_rigid_motion" in result["reason_codes"]
    assert result["holdout"]["p95_abs_dps"] < .3
    assert result["identifiability"]["minimum_sensitivity_dps_per_radian"] < .2
    json.dumps(result, allow_nan=False)


def test_fixed_proximal_pure_hinge_is_not_enough_to_identify_both_axes():
    t = np.arange(600) / 120.
    gp = np.zeros((600, 3))
    gd = np.column_stack((40 * np.sin(2 * t), np.zeros((600, 2))))
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    assert result["candidate_status"] == "insufficient_excitation"
    assert result["identifiability"]["numerical_rank"] < 4


def test_robust_fit_tolerates_small_fraction_of_large_motion_outliers():
    gp, gd, jp, jd = hinge_capture(noise=.08)
    rng = np.random.default_rng(7)
    outliers = np.arange(5, len(gp), 31)
    gd[outliers] += rng.normal(0, 25, (len(outliers), 3))
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    assert result["candidate_status"] == "candidate_supported", result["reason_codes"]
    assert line_error_deg(result["axis_prox"], jp) < 1.
    assert line_error_deg(result["axis_dist"], jd) < 1.
    assert result["train"]["inlier_fraction"] < 1.


def test_holdout_changes_status_but_never_axes_initializers_or_selection():
    gp, gd, _, _ = hinge_capture(n=600, noise=.05)
    original = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    holdout = (np.arange(len(gp)) // 60) % 2 == 1
    bad_gp, bad_gd = gp.copy(), gd.copy()
    rng = np.random.default_rng(19)
    bad_gp[holdout] = rng.normal(0, 150, (holdout.sum(), 3))
    bad_gd[holdout] = rng.normal(0, 150, (holdout.sum(), 3))
    damaged = fit_hinge_axes(bad_gp, bad_gd, sample_rate_hz=120.)
    assert original["candidate_status"] == "candidate_supported", original["reason_codes"]
    assert damaged["candidate_status"] == "inconsistent"
    for field in ("axis_prox", "axis_dist", "optimization", "train", "identifiability", "common_rigid_diagnostic"):
        assert original[field] == damaged[field], field
    assert "holdout_constraint_residual_inconsistent" in damaged["reason_codes"]


def test_inconsistent_independent_motion_does_not_report_success():
    rng = np.random.default_rng(25)
    gp, gd = rng.normal(0, 30, (2, 600, 3))
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    assert result["candidate_status"] != "candidate_supported"
    assert "train_constraint_residual_inconsistent" in result["reason_codes"]


def test_static_holdout_has_vacuous_zero_error_not_validation():
    gp, gd, _, _ = hinge_capture(n=600)
    original = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    holdout = (np.arange(len(gp)) // 60) % 2 == 1
    gp[holdout], gd[holdout] = 0., 0.
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120.)
    assert result["candidate_status"] == "insufficient_excitation"
    assert "too_few_active_holdout_samples" in result["reason_codes"]
    assert result["holdout"]["rms_dps"] == 0.
    assert result["axis_prox"] == original["axis_prox"]
    assert result["axis_dist"] == original["axis_dist"]
    assert result["optimization"] == original["optimization"]


def test_preserves_inputs_and_does_not_use_global_random_state():
    gp, gd, _, _ = hinge_capture(n=360)
    original_p, original_d = gp.copy(), gd.copy()
    np.random.seed(7)
    state = np.random.get_state()
    result = fit_hinge_axes(gp, gd, sample_rate_hz=120., n_starts=4)
    np.testing.assert_array_equal(gp, original_p)
    np.testing.assert_array_equal(gd, original_d)
    new_state = np.random.get_state()
    assert state[0] == new_state[0]
    np.testing.assert_array_equal(state[1], new_state[1])
    assert state[2:] == new_state[2:]
    json.dumps(result, allow_nan=False)


def test_valid_but_short_capture_is_insufficient_not_format_error():
    gp = np.ones((20, 3)) * 20
    result = fit_hinge_axes(gp, gp, sample_rate_hz=120.)
    assert result["candidate_status"] == "insufficient_excitation"
    assert result["reason_codes"] == ["too_few_samples_in_fixed_split"]


@pytest.mark.parametrize("bad", [[], np.zeros(3), np.zeros((3, 3)), np.zeros((30001, 3)),
                                  np.zeros((8, 4)), [[float("nan"), 0, 0]] * 8,
                                  [[float("inf"), 0, 0]] * 8, [[1e308, 0, 0]] * 8,
                                  [[True, False, True]] * 8, [["1", "2", "3"]] * 8,
                                  np.ones((8, 3), complex)])
def test_invalid_vectors_raise_clear_value_error(bad):
    with pytest.raises(ValueError, match="prox_gyro_dps"):
        fit_hinge_axes(bad, np.ones((8, 3)), sample_rate_hz=120.)


def test_pair_lengths_must_match():
    with pytest.raises(ValueError, match="equal lengths"):
        fit_hinge_axes(np.ones((100, 3)), np.ones((101, 3)), sample_rate_hz=120.)


@pytest.mark.parametrize("kwargs", [dict(sample_rate_hz=0), dict(sample_rate_hz=True),
                                      dict(sample_rate_hz=float("nan")), dict(noise_dps=0),
                                      dict(n_starts=3), dict(n_starts=4.5), dict(seed=-1),
                                      dict(max_iterations=True), dict(min_inlier_fraction=1.1),
                                      dict(residual_limit_dps=-1), dict(holdout_block_s=0),
                                      dict(min_tangent_sensitivity_dps=0), dict(max_condition_number=.5)])
def test_invalid_configuration_rejected_even_for_short_capture(kwargs):
    options = dict(sample_rate_hz=120.)
    options.update(kwargs)
    with pytest.raises(ValueError):
        fit_hinge_axes(np.ones((8, 3)), np.ones((8, 3)), **options)
