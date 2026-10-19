"""Real-QMT little-PIP regression with explicit independent heading gauges.

The shared-world motion is encoded through each sensor's own K_i. It is NOT
the physically different operation of left-multiplying every raw quaternion
by one rotation. Known child yaw corruption is synthetic, not glove accuracy.
"""
from copy import deepcopy
import importlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

pytest.importorskip("qmt")

from test_heading_correction import W, fixture_model, make_frame
from vt_fsglove12.hinge_heading import LittleFingerHeading


# These shared-world poses keep the hinge observable (rating > .50).
ROOTS = ((0., 0., 0.), (20., -15., 10.), (-30., 40., 90.),
         (100., 20., -130.), (-55., -20., 60.))


@pytest.fixture
def model():
    value = fixture_model()
    value[2]["profile_version"] = 2
    return value


def frame_for_bend(model, stamp, bend, drift_deg=0., *, root=ROOTS[0], vertical=False):
    """Encode known physical relative motion, including arbitrary IMU mounts."""
    _, _, _, k, local, rest, a0, axis = model
    frame, _ = make_frame(model, stamp, drift_deg=0.)
    if vertical:
        world = R.align_vectors(np.array([[0., 0., 1.]]),
                                (rest[8] * a0).apply(axis)[None, :])[0]
    else:
        world = R.from_euler("xyz", root, degrees=True)
    parent = W.inv() * world
    child = parent * rest[8] * a0 * R.from_rotvec(axis * bend)
    parent_raw = k[7].inv() * parent * local[7]
    child_raw = k[8].inv() * child * local[8]
    frame["nodes"]["9"]["quat_xyzw"] = parent_raw.as_quat().tolist()
    frame["nodes"]["10"]["quat_xyzw"] = (
        R.from_euler("z", drift_deg, degrees=True) * child_raw).as_quat().tolist()
    return frame


def apply_read_only(corrector, frame, model):
    calibrator, binding, profile, *_ = model
    original_frame, original_profile = deepcopy(frame), deepcopy(profile)
    original_binding, original_state = deepcopy(binding), deepcopy(vars(calibrator))
    angle, diagnostic = corrector.apply(frame, profile, calibrator, binding)
    assert frame == original_frame
    assert profile == original_profile
    assert binding == original_binding
    for name, value in original_state.items():
        np.testing.assert_array_equal(getattr(calibrator, name), value)
    return angle, diagnostic


def wrapped(angle):
    return float((angle + np.pi) % (2. * np.pi) - np.pi)


@pytest.mark.parametrize("root", ROOTS)
@pytest.mark.parametrize("bend", [0., .35, 1.2])
def test_clean_input_preserves_known_bend_with_independent_heading_gauges(model, root, bend):
    frame = frame_for_bend(model, 1., bend, root=root)
    angle, diagnostic = apply_read_only(LittleFingerHeading(), frame, model)
    assert angle == pytest.approx(bend, abs=1e-8)
    assert diagnostic["status"] == "corrected"
    assert abs(wrapped(diagnostic["delta_rad"])) < 1e-8
    assert diagnostic["residual_deg"] < 1e-6


@pytest.mark.parametrize("drift", [20., -25., 170.])
def test_fixed_bend_is_invariant_to_true_common_root_motion_with_child_yaw_error(model, drift):
    corrector = LittleFingerHeading()
    for index, root in enumerate(ROOTS):
        frame = frame_for_bend(model, 1. + index / 30., .75, drift, root=root)
        angle, diagnostic = apply_read_only(corrector, frame, model)
        assert angle == pytest.approx(.75, abs=1e-7)
        assert diagnostic["status"] == "corrected" and diagnostic["applied"]
        assert abs(wrapped(diagnostic["delta_rad"] + np.deg2rad(drift))) < 1e-7
        assert diagnostic["residual_deg"] < 1e-5


@pytest.mark.parametrize("drift", [20., -25., 170.])
def test_current_bend_follows_without_angle_smoothing_or_locking(model, drift):
    corrector = LittleFingerHeading()
    bends = [.1, 1.25, .35, 1.05, .2]
    actual = []
    for index, (bend, root) in enumerate(zip(bends, ROOTS)):
        frame = frame_for_bend(model, 1. + index / 30., bend, drift, root=root)
        angle, diagnostic = apply_read_only(corrector, frame, model)
        assert diagnostic["status"] == "corrected"
        actual.append(angle)
    np.testing.assert_allclose(actual, bends, atol=1e-7, rtol=0.)


@pytest.mark.parametrize("drift", [0., 20., -25., 170.])
def test_vertical_hinge_without_valid_heading_history_returns_none(model, drift):
    frame = frame_for_bend(model, 1., .7, drift, vertical=True)
    angle, diagnostic = apply_read_only(LittleFingerHeading(), frame, model)
    assert angle is None
    assert diagnostic["status"] == "passthrough" and not diagnostic["applied"]


@pytest.mark.parametrize("drift", [20., -25., 170.])
def test_vertical_hinge_reuses_heading_but_keeps_following_current_bend(model, drift):
    corrector = LittleFingerHeading()
    first = frame_for_bend(model, 1., .4, drift)
    angle, previous = apply_read_only(corrector, first, model)
    assert angle == pytest.approx(.4, abs=1e-7)
    for index, bend in enumerate((.6, 1.1, .3), 1):
        frame = frame_for_bend(model, 1. + index / 30., bend, drift, vertical=True)
        angle, diagnostic = apply_read_only(corrector, frame, model)
        assert diagnostic["status"] == "unobservable_using_last_heading"
        assert diagnostic["applied"]
        assert diagnostic["delta_rad"] == pytest.approx(previous["delta_rad"], abs=1e-12)
        assert angle == pytest.approx(bend, abs=1e-7)


@pytest.mark.parametrize("invalid", [
    "missing_parent", "missing_child", "wrong_epoch", "stale", "skew",
    "zero_quaternion", "nonfinite_quaternion", "invalid_sample_index",
    "orientation_invalid",
    "session_mismatch", "binding_mismatch", "disabled", "missing_reference",
])
def test_invalid_pair_is_rejected_without_mutation(model, invalid):
    corrector = LittleFingerHeading()
    first = frame_for_bend(model, 1., .4, 20.)
    assert apply_read_only(corrector, first, model)[0] == pytest.approx(.4, abs=1e-7)
    frame = frame_for_bend(model, 1. + 1. / 30., .7, 20.)
    node = frame["nodes"]["10"]
    profile = model[2]
    if invalid == "missing_parent":
        del frame["nodes"]["9"]
    elif invalid == "missing_child":
        del frame["nodes"]["10"]
    elif invalid == "wrong_epoch":
        node["epoch"] += 1
    elif invalid == "stale":
        node["age_s"] = .2
    elif invalid == "skew":
        node["age_s"] = .05
    elif invalid == "zero_quaternion":
        node["quat_xyzw"] = [0., 0., 0., 0.]
    elif invalid == "nonfinite_quaternion":
        # Inf equality remains well-defined for the immutable-source check.
        node["quat_xyzw"] = [float("inf"), 0., 0., 1.]
    elif invalid == "invalid_sample_index":
        node["sample_index"] = 0
    elif invalid == "orientation_invalid":
        node["orientation_valid"] = False
    elif invalid == "session_mismatch":
        frame["session_id"] = "different-session"
    elif invalid == "binding_mismatch":
        profile["functional_fingers"]["little"]["binding"]["config_sha256"] = "different"
    elif invalid == "disabled":
        profile["functional_enabled"] = False
    else:
        del profile["functional_fingers"]["little"]["slot_refs"]["8"]
    angle, diagnostic = apply_read_only(corrector, frame, model)
    assert angle is None
    assert diagnostic["status"] == "passthrough" and not diagnostic["applied"]


@pytest.mark.parametrize("node", ["9", "10"])
@pytest.mark.parametrize("flag", ["gyro_rejected", "accel_rejected"])
@pytest.mark.parametrize("drift", [20., -25., 170.])
def test_short_input_protection_reuses_heading_but_follows_current_pose_without_jump(
        model, monkeypatch, node, flag, drift):
    corrector = LittleFingerHeading()
    first = frame_for_bend(model, 1., .75, drift)
    angle, initial = apply_read_only(corrector, first, model)
    assert angle == pytest.approx(.75, abs=1e-7)
    estimator = importlib.import_module("qmt.functions.heading_correction")

    def forbidden_update(*args, **kwargs):
        pytest.fail("filtered input must not update the QMT heading estimate")

    with monkeypatch.context() as patch:
        patch.setattr(estimator, "estimateDelta1d", forbidden_update)
        for index, (bend, root) in enumerate(zip((.75, 1.1, .3), ROOTS[1:4]), 1):
            frame = frame_for_bend(model, 1. + index / 30., bend, drift, root=root)
            frame["nodes"][node].update(orientation_valid=True, input_filter={flag: True})
            angle, diagnostic = apply_read_only(corrector, frame, model)
            assert diagnostic["status"] == "input_protection_using_last_heading"
            assert diagnostic["applied"] and diagnostic["residual_deg"] <= 15.
            assert diagnostic["delta_rad"] == pytest.approx(initial["delta_rad"], abs=1e-12)
            assert angle == pytest.approx(bend, abs=1e-7)
    last_protected_angle = angle
    recovered = frame_for_bend(model, 1. + 4. / 30., .3, drift, root=ROOTS[3])
    recovered_angle, diagnostic = apply_read_only(corrector, recovered, model)
    assert diagnostic["status"] == "corrected"
    assert recovered_angle == pytest.approx(last_protected_angle, abs=1e-7)
    assert diagnostic["delta_rad"] == pytest.approx(initial["delta_rad"], abs=1e-7)


@pytest.mark.parametrize("node", ["9", "10"])
@pytest.mark.parametrize("flag", ["gyro_rejected", "accel_rejected"])
def test_first_filtered_input_without_heading_reference_returns_none(model, monkeypatch, node, flag):
    corrector = LittleFingerHeading()
    frame = frame_for_bend(model, 1., .75, 20.)
    frame["nodes"][node].update(orientation_valid=True, input_filter={flag: True})
    estimator = importlib.import_module("qmt.functions.heading_correction")

    def forbidden_update(*args, **kwargs):
        pytest.fail("first filtered input must not create a heading reference")

    monkeypatch.setattr(estimator, "estimateDelta1d", forbidden_update)
    angle, diagnostic = apply_read_only(corrector, frame, model)
    assert angle is None
    assert diagnostic["status"] == "passthrough" and not diagnostic["applied"]
    assert diagnostic["reason"] == "input_protection_without_reference"


@pytest.mark.parametrize("node", ["9", "10"])
def test_prolonged_invalid_orientation_clears_heading_reference(model, node):
    corrector = LittleFingerHeading()
    first = frame_for_bend(model, 1., .4, 20.)
    assert apply_read_only(corrector, first, model)[0] == pytest.approx(.4, abs=1e-7)
    invalid = frame_for_bend(model, 1. + 1. / 30., .7, 20.)
    invalid["nodes"][node].update(
        orientation_valid=False, input_filter={"gyro_rejected": True, "accel_rejected": True})
    angle, diagnostic = apply_read_only(corrector, invalid, model)
    assert angle is None and not diagnostic["applied"]
    fresh_vertical = frame_for_bend(model, 1. + 2. / 30., .7, 20., vertical=True)
    angle, diagnostic = apply_read_only(corrector, fresh_vertical, model)
    assert angle is None, "prolonged invalid orientation must clear the prior heading reference"
    assert diagnostic["status"] == "passthrough" and not diagnostic["applied"]


@pytest.mark.parametrize("change", ["explicit_reset", "gap", "clock_reversal", "profile_version", "binding_hash", "epoch", "session"])
def test_heading_history_is_never_reused_across_reset_or_context_change(model, change):
    corrector = LittleFingerHeading()
    first = frame_for_bend(model, 1., .4, 20.)
    assert apply_read_only(corrector, first, model)[0] == pytest.approx(.4, abs=1e-7)
    frame = frame_for_bend(model, 1. + 1. / 30., .7, 20., vertical=True)
    binding, profile = model[1:3]
    saved_binding = profile["functional_fingers"]["little"]["binding"]
    if change == "explicit_reset":
        corrector.reset()
    elif change == "gap":
        frame["timestamp_s"] = 1.251
    elif change == "clock_reversal":
        frame["timestamp_s"] = .99
    elif change == "profile_version":
        profile["profile_version"] += 1
    elif change == "binding_hash":
        binding["upstream_state_sha256"] = saved_binding["upstream_state_sha256"] = "new-core"
    elif change == "epoch":
        frame["nodes"]["10"]["epoch"] = 1
        binding["nodes_epochs"]["10"] = saved_binding["nodes_epochs"]["10"] = 1
    else:
        frame["session_id"] = profile["session_id"] = binding["session_id"] = "new-session"
        saved_binding["session_id"] = "new-session"
    angle, diagnostic = apply_read_only(corrector, frame, model)
    assert angle is None
    assert diagnostic["status"] == "passthrough" and not diagnostic["applied"]


@pytest.mark.parametrize("protected", [False, True])
def test_excessive_corrected_off_axis_residual_is_not_accepted(model, protected):
    corrector = LittleFingerHeading()
    if protected:
        first = frame_for_bend(model, 1., .4, 20.)
        assert apply_read_only(corrector, first, model)[0] == pytest.approx(.4, abs=1e-7)
    frame = frame_for_bend(model, 1. + 1. / 30. if protected else 1., .6, 20.)
    if protected:
        frame["nodes"]["10"].update(orientation_valid=True, input_filter={"gyro_rejected": True})
    _, _, _, k, local, rest, a0, axis = model
    hinge_world = (rest[8] * a0).apply(axis)
    tilt_axis = np.cross(hinge_world, [0., 0., 1.])
    tilt_axis /= np.linalg.norm(tilt_axis)
    tilt = R.from_rotvec(tilt_axis * np.deg2rad(40.))
    original = R.from_quat(frame["nodes"]["10"]["quat_xyzw"])
    # Known non-hinge tilt changes axis elevation and cannot be removed by yaw.
    sensor_tilt = k[8].inv() * W.inv() * tilt * W * k[8]
    frame["nodes"]["10"]["quat_xyzw"] = (sensor_tilt * original).as_quat().tolist()
    angle, diagnostic = apply_read_only(corrector, frame, model)
    assert angle is None
    assert diagnostic["status"] == "passthrough" and not diagnostic["applied"]
    assert diagnostic["residual_deg"] > 15.
