"""Offline geometry and estimator integration; synthetic, not real-hand proof."""
import copy
import json

import numpy as np
import pytest

from vt_glove_pose.hinge_frames import build_hinge_frames
from vt_glove_pose.hinge_axes import fit_hinge_axes
from vt_glove_pose.math3d import from_axis_angle, to_matrix
from vt_glove_pose.thumb_flexion import ThumbFlexionEstimator


def rx(angle):
    return to_matrix(from_axis_angle([1., 0., 0.], angle))


def capture(seed=12):
    rng = np.random.default_rng(seed)
    mp = to_matrix(from_axis_angle(rng.normal(size=3), rng.uniform(-2, 2)))
    md = to_matrix(from_axis_angle(rng.normal(size=3), rng.uniform(-2, 2)))
    jp, jd = mp.T @ [1., 0., 0.], md.T @ [1., 0., 0.]
    g = np.array([.35, .15, .92]); g /= np.linalg.norm(g)
    neutral_theta = .3
    ap, ad = [], []
    for t in np.arange(600) / 120.:
        common = to_matrix(from_axis_angle([.3, 1., .4], .35 * np.sin(.9 * t)))
        proximal_gravity = common.T @ g
        angle = neutral_theta + .5 * (1 - np.cos(1.8 * t))
        ap.append(proximal_gravity @ mp)
        ad.append(rx(angle).T @ proximal_gravity @ md)
    args = dict(axis_prox=jp, axis_dist=jd, prox_acc_g=np.asarray(ap), dist_acc_g=np.asarray(ad),
                neutral_prox_acc_g=g @ mp, neutral_dist_acc_g=rx(neutral_theta).T @ g @ md)
    return args, dict(mp=mp, md=md, g=g, neutral_theta=neutral_theta, jp=jp, jd=jd)


@pytest.mark.parametrize("seed", [3, 12, 22])
def test_random_installations_build_proper_unified_frames(seed):
    args, truth = capture(seed)
    result = build_hinge_frames(**args)
    assert result["status"] == "frames_ready", result["reason_codes"]
    fp, fd = np.asarray(result["prox_sensor_to_functional"]), np.asarray(result["dist_sensor_to_functional"])
    for frame in (fp, fd):
        np.testing.assert_allclose(frame @ frame.T, np.eye(3), atol=1e-14)
        assert np.linalg.det(frame) == pytest.approx(1.)
    np.testing.assert_allclose(fp @ result["signed_axis_prox"], [-1, 0, 0], atol=1e-14)
    np.testing.assert_allclose(fd @ result["signed_axis_dist"], [-1, 0, 0], atol=1e-14)
    np.testing.assert_allclose(result["functional_neutral_prox_acc_g"],
                               result["functional_neutral_dist_acc_g"], atol=1e-14)
    assert not result["usable_as_live_calibration"]
    assert not result["anatomical_flexion_sign_verified"]
    assert not result["static_neutral_verified_by_this_function"]
    assert not result["linear_acceleration_excluded_by_this_function"]
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("prox_sign,dist_sign", [(1, -1), (-1, 1), (-1, -1)])
def test_input_axis_line_signs_do_not_change_functional_frames(prox_sign, dist_sign):
    args, _ = capture()
    original = build_hinge_frames(**args)
    args["axis_prox"] *= prox_sign
    args["axis_dist"] *= dist_sign
    changed = build_hinge_frames(**args)
    assert changed == original


def test_scaled_axis_lines_are_normalized_not_used_as_scale_factors():
    args, _ = capture()
    original = build_hinge_frames(**args)
    args["axis_prox"] *= 12.4
    args["axis_dist"] *= -8.1
    result = build_hinge_frames(**args)
    np.testing.assert_allclose(original["prox_sensor_to_functional"], result["prox_sensor_to_functional"], atol=1e-14)
    np.testing.assert_allclose(original["dist_sensor_to_functional"], result["dist_sensor_to_functional"], atol=1e-14)


@pytest.mark.parametrize("seed", [3, 12, 22])
def test_existing_estimator_follows_mathematical_sign_and_returns_without_lag(seed):
    args, truth = capture(seed)
    result = build_hinge_frames(**args)
    fp, fd = np.asarray(result["prox_sensor_to_functional"]), np.asarray(result["dist_sensor_to_functional"])
    estimator = ThumbFlexionEstimator(120.)
    estimator.reset(result["functional_neutral_prox_acc_g"], result["functional_neutral_dist_acc_g"])
    sign = float(np.sign(np.dot(result["signed_axis_prox"], truth["jp"])))
    previous, worst = 0., 0.
    for tick in range(1, 20 * 240 + 1):
        t = tick / 120.
        relative_deg = 30. * (1 - np.cos(np.pi * t))
        # Difference rate makes the synthetic discrete gyro integral exact.
        rate = (relative_deg - previous) * 120.
        angle = truth["neutral_theta"] + np.radians(relative_deg)
        pg = np.zeros(3)
        dg = np.array([rate, 0., 0.]) @ truth["md"]
        pa = truth["g"] @ truth["mp"]
        da = rx(angle).T @ truth["g"] @ truth["md"]
        state = estimator.update(fp @ pg, fp @ pa, fd @ dg, fd @ da)
        worst = max(worst, abs(state["angle_deg"] - sign * relative_deg))
        assert state["quality"] == "tracking"
        previous = relative_deg
    assert worst < 1e-9
    assert abs(state["angle_deg"]) < 1e-9
    assert state["history_loss"] is False


def test_zero_is_separate_reference_not_first_calibration_pose():
    args, truth = capture()
    # Neutral intentionally differs from every initial observation: this is
    # not an instruction to treat first frame or relaxed pose as straight.
    reference = truth["neutral_theta"] + .45
    args["neutral_dist_acc_g"] = rx(reference).T @ truth["g"] @ truth["md"]
    result = build_hinge_frames(**args)
    assert result["status"] == "frames_ready"
    fp, fd = np.asarray(result["prox_sensor_to_functional"]), np.asarray(result["dist_sensor_to_functional"])
    estimator = ThumbFlexionEstimator(120.)
    current_p = truth["g"] @ truth["mp"]
    current_d = rx(reference + .2).T @ truth["g"] @ truth["md"]
    state = estimator.reset(result["functional_neutral_prox_acc_g"], result["functional_neutral_dist_acc_g"],
                            current_proximal_acc=fp @ current_p, current_distal_acc=fd @ current_d)
    sign = np.sign(np.dot(result["signed_axis_prox"], truth["jp"]))
    assert state["angle_deg"] == pytest.approx(sign * np.degrees(.2), abs=1e-10)


def test_estimated_axes_to_frames_to_existing_estimator_full_offline_chain():
    _, truth = capture(12)
    gp, gd, ap, ad = [], [], [], []
    for t in np.arange(960) / 120.:
        yaw, pitch, roll = .4 * np.sin(.7 * t), .3 * np.sin(1.1 * t), .2 * np.sin(1.7 * t)
        rz = to_matrix(from_axis_angle([0, 0, 1], yaw))
        ry = to_matrix(from_axis_angle([0, 1, 0], pitch))
        rr = rx(roll)
        root = rz @ ry @ rr
        omega = (rr.T @ ry.T @ [0, 0, .28 * np.cos(.7 * t)]
                 + rr.T @ [0, .33 * np.cos(1.1 * t), 0]
                 + np.array([.34 * np.cos(1.7 * t), 0, 0]))
        theta = truth["neutral_theta"] + .5 * (1 - np.cos(1.8 * t))
        theta_rate = .9 * np.sin(1.8 * t)
        common_gravity = root.T @ truth["g"]
        gp.append(np.degrees(omega) @ truth["mp"])
        gd.append(np.degrees(rx(theta).T @ omega + [theta_rate, 0, 0]) @ truth["md"])
        ap.append(common_gravity @ truth["mp"])
        ad.append(rx(theta).T @ common_gravity @ truth["md"])
    axes = fit_hinge_axes(np.asarray(gp), np.asarray(gd), sample_rate_hz=120.)
    assert axes["candidate_status"] == "candidate_supported", axes["reason_codes"]
    frames = build_hinge_frames(axes["axis_prox"], axes["axis_dist"], ap, ad,
                                neutral_prox_acc_g=truth["g"] @ truth["mp"],
                                neutral_dist_acc_g=rx(truth["neutral_theta"]).T @ truth["g"] @ truth["md"])
    assert frames["status"] == "frames_ready", frames["reason_codes"]
    fp, fd = np.asarray(frames["prox_sensor_to_functional"]), np.asarray(frames["dist_sensor_to_functional"])
    estimator = ThumbFlexionEstimator(120.)
    estimator.reset(frames["functional_neutral_prox_acc_g"], frames["functional_neutral_dist_acc_g"])
    sign = np.sign(np.dot(frames["signed_axis_prox"], truth["jp"]))
    worst = 0.
    for tick in range(1, len(gp)):
        state = estimator.update(fp @ gp[tick], fp @ ap[tick], fd @ gd[tick], fd @ ad[tick])
        expected = sign * np.degrees(.5 * (1 - np.cos(1.8 * tick / 120.)))
        worst = max(worst, abs(state["angle_deg"] - expected))
        assert "off_axis_motion_mismatch" not in state["reasons"]
    # Endpoint gyro integration has discretization error; do not mislabel it
    # as physical accuracy. This only proves the offline convention chain.
    assert worst < .5
    assert not state["history_loss"]


def test_hinge_perpendicular_to_all_capture_gravity_cannot_resolve_sign():
    args = dict(axis_prox=[1, 0, 0], axis_dist=[-1, 0, 0],
                prox_acc_g=np.tile([0., 0., 1.], (60, 1)),
                dist_acc_g=np.asarray([rx(angle) @ [0., 0., 1.] for angle in np.linspace(0, 1, 60)]),
                neutral_prox_acc_g=[.4, 0, np.sqrt(.84)],
                neutral_dist_acc_g=[.4, 0, np.sqrt(.84)])
    result = build_hinge_frames(**args)
    assert result["status"] == "sign_unobservable"
    assert result["prox_sensor_to_functional"] is None
    assert result["sign_metrics"]["selected_distal_sign_from_canonical_line"] is None


def test_single_nonperpendicular_sample_does_not_resolve_axis_sign():
    acc = np.tile([0., 0., 1.], (60, 1))
    acc[0] = [.9, 0., np.sqrt(.19)]
    result = build_hinge_frames([1, 0, 0], [1, 0, 0], acc, acc,
                                neutral_prox_acc_g=[.4, 0, np.sqrt(.84)],
                                neutral_dist_acc_g=[.4, 0, np.sqrt(.84)])
    assert result["status"] == "sign_unobservable"
    assert result["sign_metrics"]["informative_sample_count"] == 1


@pytest.mark.parametrize("axis_component", [1., .97])
def test_explicit_neutral_nearly_parallel_to_axis_is_unobservable(axis_component):
    args, truth = capture()
    near_parallel = [axis_component, 0., np.sqrt(1 - axis_component**2)]
    args["neutral_prox_acc_g"] = np.asarray(near_parallel) @ truth["mp"]
    args["neutral_dist_acc_g"] = rx(.3).T @ near_parallel @ truth["md"]
    result = build_hinge_frames(**args)
    assert result["status"] == "neutral_unobservable"
    assert result["prox_sensor_to_functional"] is None
    json.dumps(result, allow_nan=False)


def test_unit_norm_contradictory_acceleration_is_not_accepted_as_gravity():
    args, truth = capture()
    # Norm remains exactly 1g, but deliberately violates the hinge projection.
    args["dist_acc_g"] = np.tile([.9, 0, np.sqrt(.19)], (600, 1)) @ truth["md"]
    result = build_hinge_frames(**args)
    assert result["status"] == "inconsistent"
    assert result["sign_metrics"]["usable_fraction"] == 1.
    assert result["reason_codes"] == ["paired_axis_gravity_components_inconsistent"]


def test_inconsistent_neutral_cannot_be_replaced_by_good_current_capture():
    args, truth = capture()
    args["neutral_dist_acc_g"] = np.asarray([.8, 0., .6]) @ truth["md"]
    result = build_hinge_frames(**args)
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["explicit_neutral_axis_gravity_mismatch"]


def test_non_gravity_neutral_norm_rejected_without_dividing_zero():
    args, _ = capture()
    args["neutral_prox_acc_g"] = [0., 0., 0.]
    result = build_hinge_frames(**args)
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["explicit_neutral_acceleration_norm_inconsistent"]
    json.dumps(result, allow_nan=False)


def test_capture_norm_outliers_not_silently_accepted_as_static():
    args, _ = capture()
    args["prox_acc_g"][:240] *= 2.
    result = build_hinge_frames(**args)
    assert result["status"] == "inconsistent"
    assert result["reason_codes"] == ["insufficient_norm_consistent_acceleration"]


def test_function_is_pure_and_short_capture_returns_status():
    args, _ = capture()
    original = copy.deepcopy(args)
    result = build_hinge_frames(**args)
    for key in args:
        np.testing.assert_array_equal(args[key], original[key])
    result["prox_sensor_to_functional"][0][0] = 555
    assert build_hinge_frames(**args)["prox_sensor_to_functional"][0][0] != 555
    args["prox_acc_g"], args["dist_acc_g"] = args["prox_acc_g"][:3], args["dist_acc_g"][:3]
    result = build_hinge_frames(**args)
    assert result["status"] == "insufficient_data"
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("field,value", [
    ("axis_prox", [0, 0, 0]), ("axis_dist", [float("nan"), 0, 1]),
    ("prox_acc_g", []), ("dist_acc_g", [[0, 0, 1]]),
    ("prox_acc_g", np.zeros((30001, 3))),
    ("neutral_prox_acc_g", [True, False, True]),
    ("neutral_dist_acc_g", [float("inf"), 0, 1]),
    ("neutral_prox_acc_g", ["0", "0", "1"]),
    ("min_samples", True), ("min_parallel_rms", 0), ("min_sign_margin", -1),
    ("max_axis_mismatch", float("nan")), ("min_neutral_projection", .1),
    ("min_usable_fraction", 1.1),
])
def test_invalid_inputs_raise_value_error(field, value):
    args, _ = capture()
    args[field] = value
    with pytest.raises(ValueError):
        build_hinge_frames(**args)
