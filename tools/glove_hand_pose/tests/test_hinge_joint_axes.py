"""Independent paired kinematic snapshots; no hardware or proprietary code."""

import json

import numpy as np
import pytest

from vt_glove_pose.hinge_joint_axes import (
    _joint_residual_jacobian, evaluate_joint_hinge_axes, fit_joint_hinge_axes,
)
from vt_glove_pose.math3d import from_axis_angle, to_matrix


def snapshots(seed=3, count=720, noise=.03):
    """Snapshot constraints from known common and relative rotations.

    The acceleration's common direction is varied independently for coverage;
    this tests instantaneous geometry, not an IMU time-integration model.
    """
    rng = np.random.default_rng(seed)
    mp = to_matrix(from_axis_angle(rng.normal(size=3), .9))
    md = to_matrix(from_axis_angle(rng.normal(size=3), -.7))
    t = np.arange(count) / 120.
    theta = .7*np.sin(1.7*t)+.2*np.sin(2.9*t)
    speed = np.rad2deg(1.19*np.cos(1.7*t)+.58*np.cos(2.9*t))
    gp0 = np.column_stack((17*np.sin(t), 23*np.cos(.9*t), 11*np.sin(1.2*t+.2)))
    ap0 = np.column_stack((.6*np.sin(.7*t+.5), .6*np.cos(1.3*t), np.ones(count)))
    ap0 /= np.linalg.norm(ap0, axis=1)[:, None]
    gd0, ad0 = [], []
    for angle, v, acc, s in zip(theta, gp0, ap0, speed):
        relative = to_matrix(from_axis_angle([1, 0, 0], angle))
        gd0.append(relative.T @ v + [s, 0, 0])
        ad0.append(relative.T @ acc)
    gp, gd, ap, ad = gp0 @ mp, np.asarray(gd0) @ md, ap0 @ mp, np.asarray(ad0) @ md
    gp += rng.normal(0, noise, gp.shape)
    gd += rng.normal(0, noise, gd.shape)
    ap += rng.normal(0, noise/30, ap.shape)
    ad += rng.normal(0, noise/30, ad.shape)
    return (gp, gd, ap, ad), mp.T @ [1, 0, 0], md.T @ [1, 0, 0]


def line_error(a, b):
    return np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), 0., 1.)))


@pytest.mark.parametrize("seed", [3, 9, 22])
def test_joint_candidate_recovers_axes_and_relative_sign(seed):
    data, jp, jd = snapshots(seed)
    result = fit_joint_hinge_axes(*data, sample_rate_hz=120)
    assert result["candidate_status"] == "candidate_supported", result["reason_codes"]
    assert line_error(result["axis_prox"], jp) < .3
    assert line_error(result["axis_dist"], jd) < .3
    assert np.dot(result["axis_prox"], jp)*np.dot(result["axis_dist"], jd) > 0
    assert result["identifiability"]["numerical_rank"] == 4
    assert result["holdout"]["gyro"]["p95_abs_dps"] < .2
    assert result["offline_only"] and not result["usable_as_live_calibration"]
    assert not result["anatomical_accuracy_verified"]
    assert not result["scale_verified_by_this_function"]
    json.dumps(result, allow_nan=False)


def test_joint_tangent_jacobian_matches_finite_differences():
    (gp, gd, ap, ad), _, _ = snapshots(count=240)
    jp = np.array([.2, .4, .8]); jp /= np.linalg.norm(jp)
    jd = np.array([.8, -.2, .1]); jd /= np.linalg.norm(jd)
    mask = np.arange(len(gp)) % 3 != 0
    _, jac, bp, bd = _joint_residual_jacobian(gp, gd, ap, ad, jp, jd, mask, .7, .02)
    for k in range(4):
        pstep = bp[:, k] if k < 2 else np.zeros(3)
        dstep = bd[:, k-2] if k >= 2 else np.zeros(3)
        residuals = []
        for sign in (1, -1):
            pp, dd = jp+sign*1e-6*pstep, jd+sign*1e-6*dstep
            residuals.append(_joint_residual_jacobian(
                gp, gd, ap, ad, pp/np.linalg.norm(pp), dd/np.linalg.norm(dd), mask, .7, .02)[0])
        np.testing.assert_allclose((residuals[0]-residuals[1])/2e-6, jac[:, k], atol=1e-5)


def test_heldout_gyro_and_acceleration_never_affect_fitting():
    data, _, _ = snapshots(count=600)
    expected = fit_joint_hinge_axes(*data, sample_rate_hz=120)
    bad = [v.copy() for v in data]
    holdout = (np.arange(600)//60) % 2 == 1
    rng = np.random.default_rng(19)
    for v in bad[:2]:
        v[holdout] = rng.normal(0, 150, (holdout.sum(), 3))
    bad[2][holdout] = [0, 0, 1]
    bad[3][holdout] = [0, 1, 0]
    result = fit_joint_hinge_axes(*bad, sample_rate_hz=120)
    assert expected["candidate_status"] == "candidate_supported", expected["reason_codes"]
    assert result["candidate_status"] != "candidate_supported"
    for field in ("axis_prox", "axis_dist", "optimization", "train", "identifiability", "common_rigid_diagnostic"):
        assert expected[field] == result[field], field
    assert "holdout_gyro_constraint_residual_inconsistent" in result["reason_codes"]


def test_same_sign_flip_preserves_both_constraints_but_independent_flip_does_not():
    data, jp, jd = snapshots(noise=0.)
    base = evaluate_joint_hinge_axes(jp, jd, *data)
    flipped = evaluate_joint_hinge_axes(-jp, -jd, *data)
    wrong = evaluate_joint_hinge_axes(jp, -jd, *data)
    assert base == flipped
    assert base["acceleration"]["eligible_rms_g"] < 1e-12
    assert wrong["acceleration"]["eligible_rms_g"] > .3
    assert base["gyro"] == wrong["gyro"]
    assert not base["fit_performed"]


def test_heldout_acceleration_alone_can_reject_without_changing_axes():
    data, _, _ = snapshots(count=600)
    expected = fit_joint_hinge_axes(*data, sample_rate_hz=120)
    bad = [v.copy() for v in data]
    holdout = (np.arange(600)//60) % 2 == 1
    bad[2][holdout] = [0, 0, 1]
    bad[3][holdout] = [0, 1, 0]
    result = fit_joint_hinge_axes(*bad, sample_rate_hz=120)
    assert expected["candidate_status"] == "candidate_supported"
    assert result["candidate_status"] == "inconsistent"
    assert "holdout_acceleration_constraint_residual_inconsistent" in result["reason_codes"]
    assert not result["hinge_model_supported_by_heldout_constraints"]
    assert result["holdout"]["gyro"] == expected["holdout"]["gyro"]
    for field in ("axis_prox", "axis_dist", "optimization", "train", "identifiability"):
        assert expected[field] == result[field], field


def test_bad_gyro_is_not_rescued_by_good_acceleration():
    data, _, _ = snapshots()
    gp, gd, ap, ad = data
    rng = np.random.default_rng(41)
    gp = gp+rng.normal(0, 6, gp.shape)
    gd = gd+rng.normal(0, 6, gd.shape)
    result = fit_joint_hinge_axes(gp, gd, ap, ad, sample_rate_hz=120)
    assert result["candidate_status"] != "candidate_supported"
    assert "holdout_gyro_constraint_residual_inconsistent" in result["reason_codes"]
    assert not result["hinge_model_supported_by_heldout_constraints"]


def test_static_and_common_rigid_data_never_produce_a_supported_candidate():
    data, _, _ = snapshots()
    gp, _, ap, _ = data
    quiet = fit_joint_hinge_axes(np.zeros_like(gp), np.zeros_like(gp), ap, ap, sample_rate_hz=120)
    assert quiet["reason_codes"] == ["training_motion_below_noise"]
    assert quiet["axis_prox"] is None
    rigid = fit_joint_hinge_axes(gp, gp, ap, ap, sample_rate_hz=120)
    assert rigid["candidate_status"] == "insufficient_excitation"
    assert "training_compatible_with_common_rigid_motion" in rigid["reason_codes"]


def test_quiet_holdout_is_not_independent_validation():
    data, _, _ = snapshots()
    holdout = (np.arange(len(data[0]))//60) % 2 == 1
    for v in data[:2]:
        v[holdout] = 0
    result = fit_joint_hinge_axes(*data, sample_rate_hz=120)
    assert result["candidate_status"] == "insufficient_excitation"
    assert "too_few_active_holdout_samples" in result["reason_codes"]


def test_acceleration_coverage_is_explicit_not_hidden_selection():
    data, _, _ = snapshots()
    data[2][:] *= 2
    result = fit_joint_hinge_axes(*data, sample_rate_hz=120)
    assert result["candidate_status"] == "insufficient_excitation"
    assert result["reason_codes"] == ["too_few_eligible_training_acceleration_samples"]


def test_inputs_and_global_rng_are_unchanged_and_json_is_finite():
    data, _, _ = snapshots()
    saved = [v.copy() for v in data]
    np.random.seed(4)
    state = np.random.get_state()
    result = fit_joint_hinge_axes(*data, sample_rate_hz=120)
    for actual, before in zip(data, saved):
        np.testing.assert_array_equal(actual, before)
    after = np.random.get_state()
    assert state[0] == after[0]
    np.testing.assert_array_equal(state[1], after[1])
    assert state[2:] == after[2:]
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("kwargs", [{"noise_dps": True}, {"accel_noise_g": 0},
    {"accel_max_rate_dps": float("nan")}, {"n_starts": 2}, {"sample_rate_hz": 0}])
def test_invalid_configuration_is_rejected(kwargs):
    data, _, _ = snapshots(count=240)
    parameters = {"sample_rate_hz": 120, **kwargs}
    with pytest.raises(ValueError):
        fit_joint_hinge_axes(*data, **parameters)


@pytest.mark.parametrize("damage", ["nan", "complex", "length", "boolean", "oversized"])
def test_invalid_acceleration_is_rejected(damage):
    data, _, _ = snapshots(count=240)
    damaged = list(data)
    if damage == "nan":
        damaged[2][0, 0] = np.nan
    elif damage == "complex":
        damaged[2] = damaged[2].astype(complex)
    elif damage == "length":
        damaged[2] = damaged[2][:-1]
    elif damage == "boolean":
        damaged[2] = damaged[2] > 0
    else:
        damaged[2][0, 0] = 101
    with pytest.raises(ValueError):
        fit_joint_hinge_axes(*damaged, sample_rate_hz=120)
