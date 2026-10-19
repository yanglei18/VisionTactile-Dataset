"""Software-only 1DOF simulations, not validation of real thumb anatomy."""
import copy
import json
import math

import numpy as np
import pytest

from vt_glove_pose.thumb_flexion import ThumbFlexionEstimator


RATE = 120.
DT = 1. / RATE


def rx(degrees):
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def rotation(axis, degrees):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    a = math.radians(degrees)
    return np.eye(3) + math.sin(a) * cross + (1. - math.cos(a)) * (cross @ cross)


def observation(angle, rate=0., proximal_acc=(0., 0., 1.), proximal_gyro=(0., 0., 0.)):
    """Exact same-hinge body vectors, distal=proximal Rx(-angle)."""
    pa, pg = np.asarray(proximal_acc, float), np.asarray(proximal_gyro, float)
    relative_inverse = rx(angle)
    return pg, pa, relative_inverse @ pg + [-rate, 0., 0.], relative_inverse @ pa


def ready(neutral=0.):
    estimator = ThumbFlexionEstimator(RATE)
    _, pa, _, da = observation(neutral)
    estimator.reset(pa, da)
    return estimator


def move_to(estimator, start, end, seconds=1., offset=0., bias=0.):
    steps = round(seconds * RATE)
    for tick in range(1, steps + 1):
        angle = start + (end - start) * tick / steps
        estimator.update(*observation(angle + offset, (end - start) / seconds + bias))
    return estimator.snapshot()


def test_reset_is_explicit_and_nonzero_neutral_is_subtracted():
    estimator = ThumbFlexionEstimator(RATE)
    assert estimator.snapshot()["angle_deg"] is None
    assert estimator.snapshot()["quality"] == "uninitialized"
    with pytest.raises(ValueError, match="reset"):
        estimator.update(*observation(0.))
    estimator = ready(neutral=17.)
    result = move_to(estimator, 0., 42., offset=17.)
    assert result["angle_deg"] == pytest.approx(42., abs=1e-9)
    assert result["acc_angle_deg"] == pytest.approx(42., abs=1e-9)
    assert result["valid"] and not result["uncertain"]


@pytest.mark.parametrize("neutral,current,expected", [(0., 45., 45.), (17., 62., 45.),
                                                      (17., 17., 0.), (170., 195., 25.)])
def test_enabling_after_axis_change_uses_current_pose_without_replacing_neutral(neutral, current, expected):
    estimator = ready()
    move_to(estimator, 0., 45.)
    _, np_, _, nd = observation(neutral)
    _, cp, _, cd = observation(current)
    result = estimator.reset(np_, nd, current_proximal_acc=cp, current_distal_acc=cd)
    assert result["angle_deg"] == pytest.approx(expected, abs=1e-9)
    assert result["quality"] == "tracking"
    assert result["samples"] == 0
    assert estimator.export_state()["state"]["neutral_angle_deg"] == pytest.approx((neutral + 180) % 360 - 180)
    for _ in range(120):
        result = estimator.update(*observation(current))
        assert result["angle_deg"] == pytest.approx(expected, abs=1e-9)
        assert result["quality"] == "tracking"
    restored = ThumbFlexionEstimator.from_state(estimator.export_state())
    assert restored.update(*observation(current)) == estimator.update(*observation(current))


@pytest.mark.parametrize("cp,cd,reason", [
    ([0., 0., 1.], None, "both current"),
    ([0., 0., 1.4], [0., 0., 1.], "current gravity.*acc_norm"),
    ([1., 0., 0.], [1., 0., 0.], "current gravity.*low_gravity_projection"),
    ([.4, 0., math.sqrt(.84)], [0., 0., 1.], "current gravity.*axis_gravity_mismatch"),
])
def test_invalid_current_pose_rejects_initialization_without_fake_zero(cp, cd, reason):
    estimator = ready()
    move_to(estimator, 0., 45.)
    before = estimator.export_state()
    with pytest.raises(ValueError, match=reason):
        estimator.reset([0., 0., 1.], [0., 0., 1.], current_proximal_acc=cp, current_distal_acc=cd)
    assert estimator.export_state() == before


def test_observable_current_pose_cannot_bypass_inconsistent_neutral_reference():
    estimator = ready()
    before = estimator.export_state()
    neutral_p = [.35516, 0., math.sqrt(1 - .35516**2)]
    neutral_d = [.53625, 0., math.sqrt(1 - .53625**2)]
    with pytest.raises(ValueError, match="neutral gravity.*axis_gravity_mismatch"):
        estimator.reset(neutral_p, neutral_d, current_proximal_acc=[0., 0., 1.],
                        current_distal_acc=rx(45) @ [0., 0., 1.])
    assert estimator.export_state() == before


def test_twenty_cycles_bias_correction_and_bent_hold_do_not_pull_to_zero():
    estimator = ready()
    previous = 0.
    max_error = 0.
    for tick in range(1, 20 * 2 * int(RATE) + 1):
        t = tick / RATE
        angle = 35. * (1. - math.cos(math.pi * t))
        rate = (angle - previous) / DT
        result = estimator.update(*observation(angle, rate + .4))
        max_error = max(max_error, abs(result["angle_deg"] - angle))
        previous = angle
    assert max_error < 1.0
    assert abs(result["angle_deg"]) < 1.0
    move_to(estimator, 0., 65., bias=.4)
    for _ in range(round(10 * RATE)):
        result = estimator.update(*observation(65., .4))
    assert result["angle_deg"] == pytest.approx(65., abs=.5)
    assert result["angle_deg"] > 64.
    assert result["quality"] == "tracking"


@pytest.mark.parametrize("seconds", [.25, .5, 2.])
def test_consistent_gyro_has_no_correction_time_constant_return_lag(seconds):
    estimator = ready()
    bent = move_to(estimator, 0., 60., seconds)
    straight = move_to(estimator, 60., 0., seconds)
    assert bent["angle_deg"] == pytest.approx(60., abs=1e-8)
    assert straight["angle_deg"] == pytest.approx(0., abs=1e-8)


def test_common_three_dimensional_rotation_cancels_with_bent_joint():
    estimator = ready()
    move_to(estimator, 0., 47.)
    axis = np.array([.3, .7, .2])
    axis /= np.linalg.norm(axis)
    corrected = 0
    for tick in range(1, 600):
        # Entire fixed-bent pair rotates rigidly. Distal yz gyro is rotated,
        # not equal to proximal yz; incorrectly subtracting yz rejects it.
        root = rotation(axis, tick * DT * 38.)
        pa = root.T @ [0., 0., 1.]
        pg = axis * 38.
        result = estimator.update(*observation(47., 0., pa, pg))
        assert result["angle_deg"] == pytest.approx(47., abs=1e-8)
        assert "off_axis_motion_mismatch" not in result["reasons"]
        corrected += result["correction_active"]
    assert corrected > 400


def test_vertical_hinge_axis_is_prediction_only_not_fake_straightening():
    estimator = ready()
    vertical_gravity = (1., 0., 0.)
    for _ in range(120):
        result = estimator.update(*observation(25., 25., vertical_gravity))
    assert result["angle_deg"] == pytest.approx(25.)
    assert result["quality"] == "prediction_only"
    assert not result["valid"] and not result["acc_observable"]
    assert "low_gravity_projection" in result["reasons"]
    for _ in range(1200):
        result = estimator.update(*observation(25., 0., vertical_gravity))
    assert result["angle_deg"] == pytest.approx(25.)
    assert not result["correction_active"]


def test_saturation_integrates_reported_rate_but_never_claims_lost_trajectory():
    estimator = ready()
    result = estimator.update(*observation(2., 120.), distal_saturated=True)
    assert result["angle_deg"] == pytest.approx(1.)
    assert result["history_loss"] and result["uncertain"] and not result["valid"]
    assert result["quality"] == "uncertain"
    assert "saturated_input" in result["reasons"]
    assert not result["correction_active"]


def test_stable_bent_gravity_reacquires_large_error_with_loss_flag_preserved():
    estimator = ready()
    # Deliberately missing angular information. The reported rate cannot tell
    # us how the joint reached 39 degrees; present gravity can observe it.
    estimator.update(*observation(39., 0.), distal_saturated=True)
    reacquiring = False
    previous = estimator.snapshot()["angle_deg"]
    for tick in range(1200):
        result = estimator.update(*observation(39., 0.))
        if tick < 58:
            assert not result["correction_active"]
        assert abs(result["angle_deg"] - previous) <= 8. * DT + 1e-12
        previous = result["angle_deg"]
        reacquiring |= result["quality"] == "reacquiring"
    assert reacquiring
    assert result["angle_deg"] == pytest.approx(39., abs=.1)
    assert result["quality"] == "observed_after_loss"
    assert result["valid"] and result["uncertain"] and result["history_loss"]
    assert "past_unobserved_motion" in result["reasons"]
    # Only a caller's explicit accepted neutral capture clears provenance.
    estimator.reset([0., 0., 1.], [0., 0., 1.])
    assert not estimator.snapshot()["history_loss"]


def test_slowly_changing_gravity_does_not_count_as_stable_reacquisition():
    estimator = ready()
    estimator.update(*observation(39.), distal_saturated=True)
    for tick in range(240):
        # 5 degrees/s of unexplained accel direction change: per-sample
        # differences are tiny, but the complete half-second is not stable.
        result = estimator.update(*observation(39. + 5. * tick / RATE))
        assert not result["correction_active"]
        assert result["quality"] == "uncertain"
    assert result["angle_deg"] == 0.


@pytest.mark.parametrize("pa,da,reason", [
    ([0., 0., 1.4], [0., 0., 1.], "acc_norm"),
    ([.35516, 0., math.sqrt(1 - .35516**2)],
     [.53625, 0., math.sqrt(1 - .53625**2)], "axis_gravity_mismatch"),
    ([1., 0., 0.], [1., 0., 0.], "low_gravity_projection"),
])
def test_bad_gravity_gate_and_reset_are_explicit_and_transactional(pa, da, reason):
    estimator = ready()
    before = estimator.export_state()
    with pytest.raises(ValueError, match=reason):
        estimator.reset(pa, da)
    assert estimator.export_state() == before
    result = estimator.update([0., 0., 0.], pa, [0., 0., 0.], da)
    assert reason in result["reasons"]
    assert not result["acc_observable"]
    assert not result["correction_active"]


def test_off_axis_motion_mismatch_is_not_accepted_as_a_hinge_measurement():
    estimator = ready()
    result = estimator.update([0., 70., 0.], [0., 0., 1.],
                              [0., 0., 70.], [0., 0., 1.])
    assert "off_axis_motion_mismatch" in result["reasons"]
    assert result["quality"] == "prediction_only"


def test_norm_preserving_acceleration_jump_is_rejected_by_motion_consistency():
    estimator = ready()
    # Both accel magnitudes are 1g, but an instantaneous 8-degree indicated
    # relative gravity change has no corresponding angular velocity.
    result = estimator.update(*observation(8., 0.))
    assert result["acc_observable"]
    assert "acc_motion_inconsistent" in result["reasons"]
    assert not result["correction_active"]
    assert result["angle_deg"] == 0.


def test_angle_wrap_does_not_create_a_hundred_degree_display_jump():
    estimator = ready(neutral=170.)
    result = move_to(estimator, 0., 25., offset=170.)
    assert result["angle_deg"] == pytest.approx(25., abs=1e-8)
    assert result["valid"]


def test_known_missing_pair_is_uncertain_and_does_not_invent_motion():
    estimator = ready()
    move_to(estimator, 0., 30.)
    result = estimator.update(None, None, None, None, pair_valid=False)
    assert result["angle_deg"] == pytest.approx(30.)
    assert result["history_loss"] and result["quality"] == "uncertain"
    assert "pair_unavailable" in result["reasons"]


@pytest.mark.parametrize("dt", [0., -DT, DT / 5, DT * 5, math.inf, math.nan, True, "0.01"])
def test_invalid_dt_rejected_without_altering_state(dt):
    estimator = ready()
    before = estimator.export_state()
    with pytest.raises(ValueError, match="dt_s"):
        estimator.update(*observation(0.), dt_s=dt)
    assert estimator.export_state() == before


@pytest.mark.parametrize("bad", [[math.nan, 0., 0.], [0., math.inf, 0.], [0., 0.],
                                [True, False, True], ["0", "0", "0"]])
def test_invalid_vectors_rejected_without_altering_state(bad):
    estimator = ready()
    before = estimator.export_state()
    with pytest.raises(ValueError):
        estimator.update(bad, [0., 0., 1.], [0., 0., 0.], [0., 0., 1.])
    assert estimator.export_state() == before


def test_snapshots_do_not_advance_or_mutate_the_estimator():
    estimator = ready()
    result = move_to(estimator, 0., 30.)
    before = estimator.export_state()
    for _ in range(100):
        assert estimator.snapshot() == result
    result["reasons"].append("caller modification")
    exported = estimator.export_state()
    exported["state"]["reasons"].append("caller modification")
    assert estimator.export_state() == before


@pytest.mark.parametrize("condition", ["empty", "tracking", "predicting", "lost", "reacquiring", "observed_after_loss"])
def test_strict_json_checkpoint_replays_exact_future(condition):
    original = ThumbFlexionEstimator(RATE) if condition == "empty" else ready()
    if condition == "tracking":
        move_to(original, 0., 30.)
    elif condition == "predicting":
        original.update(*observation(30., 30., (1., 0., 0.)))
    elif condition in ("lost", "reacquiring", "observed_after_loss"):
        original.update(*observation(39.), distal_saturated=True)
        count = {"lost": 0, "reacquiring": 120, "observed_after_loss": 1200}[condition]
        for _ in range(count):
            original.update(*observation(39.))
    data = json.loads(json.dumps(original.export_state(), allow_nan=False))
    restored = ThumbFlexionEstimator.from_state(data)
    assert restored.sample_rate_hz == RATE
    assert restored.export_state() == original.export_state()
    if condition == "empty":
        original.reset([0., 0., 1.], [0., 0., 1.])
        restored.reset([0., 0., 1.], [0., 0., 1.])
    for tick in range(120):
        sample = observation(40. + tick * .1, 12.)
        assert original.update(*sample) == restored.update(*sample)
    data["state"]["reasons"].append("caller modification")
    assert restored.export_state() == original.export_state()


@pytest.mark.parametrize("key,value", [
    ("initialized", 1), ("angle_deg", math.nan), ("angle_deg", True),
    ("neutral_angle_deg", 181.), ("gyro_phase_deg", math.inf),
    ("rate_dps", 30000.), ("correction_dps", 9.), ("samples", True),
    ("samples", -1), ("samples", 2**53), ("quality", "perfect"),
    ("reasons", ["invented"]), ("reasons", ["acc_norm", "acc_norm"]),
    ("reasons", ["saturated_input"]), ("reasons", ["neutral_required"]),
    ("reasons", ["axis_gravity_mismatch"]),
    ("reasons", ["innovation_too_large"]), ("reasons", ["acc_motion_inconsistent"]),
    ("acc_reference_phase_deg", None), ("history_loss", True),
])
def test_corrupt_runtime_state_is_rejected(key, value):
    data = ready().export_state()
    data["state"][key] = value
    with pytest.raises(ValueError):
        ThumbFlexionEstimator.from_state(data)


@pytest.mark.parametrize("key,value", [("kind", "other"), ("schema_version", True),
                                      ("schema_version", 2), ("offline_only", False),
                                      ("sample_rate_hz", 0.), ("sample_rate_hz", "120")])
def test_corrupt_checkpoint_header_is_rejected(key, value):
    data = ready().export_state()
    data[key] = value
    with pytest.raises(ValueError):
        ThumbFlexionEstimator.from_state(data)


def test_missing_extra_and_contradictory_checkpoint_fields_are_rejected():
    valid = ready().export_state()
    cases = []
    changed = copy.deepcopy(valid)
    changed["extra"] = 1
    cases.append(changed)
    changed = copy.deepcopy(valid)
    del changed["state"]["gyro_phase_deg"]
    cases.append(changed)
    changed = copy.deepcopy(valid)
    changed["state"]["extra"] = 1
    cases.append(changed)
    changed = copy.deepcopy(valid)
    changed["state"]["acc_observable"] = False
    cases.append(changed)
    changed = copy.deepcopy(valid)
    changed["state"]["quality"] = "prediction_only"
    cases.append(changed)
    changed = copy.deepcopy(valid)
    changed["state"]["quality"] = "reacquiring"
    cases.append(changed)
    changed = ThumbFlexionEstimator(RATE).export_state()
    changed["state"]["samples"] = 1
    cases.append(changed)
    estimator = ready()
    estimator.update(*observation(0., 0., (1., 0., 0.)))
    changed = estimator.export_state()
    changed["state"]["acc_angle_deg"] = 0.
    cases.append(changed)
    for data in cases:
        with pytest.raises(ValueError):
            ThumbFlexionEstimator.from_state(data)
