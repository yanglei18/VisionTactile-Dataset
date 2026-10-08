"""Synthetic angle/bias observer tests; no physical accuracy claims."""
import json
import math

import numpy as np
import pytest

from vt_glove_pose.hinge_observer import ConstrainedHingeObserver
from vt_glove_pose.thumb_flexion import ThumbFlexionEstimator


RATE = 120.
DT = 1. / RATE


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def rotation(axis, degrees):
    axis = np.array(axis, dtype=float, copy=True)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    a = math.radians(degrees)
    return np.eye(3) + math.sin(a) * cross + (1. - math.cos(a)) * (cross @ cross)


def observation(angle, rate=0., bias=0., pa=(0., 0., 1.), pg=(0., 0., 0.)):
    pa, pg = np.asarray(pa, float), np.asarray(pg, float)
    gd = rx(angle) @ pg + [-rate, 0., 0.]
    # A proximal residual x bias affects measured relative rate only.
    return pg + [bias, 0., 0.], pa, gd, rx(angle) @ pa


def ready(neutral=0., cls=ConstrainedHingeObserver):
    observer = cls(RATE)
    _, pa, _, da = observation(neutral)
    observer.reset(pa, da)
    return observer


def test_explicit_bent_reference_and_current_initialization():
    observer = ConstrainedHingeObserver(RATE)
    assert observer.snapshot()["angle_deg"] is None
    with pytest.raises(ValueError, match="reset"):
        observer.update(*observation(0.))
    _, pa, _, da = observation(28.)
    _, cp, _, cd = observation(63.)
    result = observer.reset(pa, da, current_proximal_acc=cp, current_distal_acc=cd)
    assert result["angle_deg"] == pytest.approx(35.)
    for _ in range(240):
        result = observer.update(*observation(63.))
    assert result["angle_deg"] == pytest.approx(35., abs=1e-9)
    assert result["relative_bias_dps"] == pytest.approx(0.)
    json.dumps(result, allow_nan=False)


def test_twenty_cycles_with_residual_bias_learn_bias_and_return():
    observer = ready()
    previous, worst = 0., 0.
    for tick in range(1, 20 * 240 + 1):
        angle = 30. * (1. - math.cos(math.pi * tick / RATE))
        rate = (angle - previous) / DT
        result = observer.update(*observation(angle, rate, bias=1.2))
        worst = max(worst, abs(result["angle_deg"] - angle))
        previous = angle
    assert worst < .6
    assert abs(result["angle_deg"]) < .1
    assert result["relative_bias_dps"] == pytest.approx(1.2, abs=.03)
    assert not result["history_loss"]


def test_long_bent_hold_estimates_bias_without_straightening():
    observer = ready()
    for tick in range(1, 121):
        observer.update(*observation(45. * tick / 120, 45., bias=.8))
    for _ in range(60 * 120):
        result = observer.update(*observation(45., bias=.8))
    assert result["angle_deg"] == pytest.approx(45., abs=.05)
    assert result["relative_bias_dps"] == pytest.approx(.8, abs=.02)
    assert result["rate_dps"] == pytest.approx(0., abs=.02)
    assert result["raw_relative_rate_dps"] == pytest.approx(.8)


def test_constant_shape_common_rotation_observes_angle_from_transverse_gyro():
    observer = ready(neutral=17.)
    common_gyro = np.array([3., 28., 34.])
    for tick in range(1, 241):
        root = rotation(common_gyro, np.linalg.norm(common_gyro) * tick / RATE)
        result = observer.update(*observation(17., pg=common_gyro, pa=root.T @ [0., 0., 1.]))
    assert result["angle_deg"] == pytest.approx(0., abs=1e-9)
    assert "transverse_gyro" in result["observation_sources"]
    assert not result["history_loss"]


def test_gyro_observes_angle_when_gravity_magnitude_is_unusable():
    observer = ready()
    previous, worst = 0., 0.
    for tick in range(1, 1201):
        angle = 20. * (1. - math.cos(math.pi * tick / RATE))
        rate = (angle - previous) / DT
        common_gyro = np.array([0., 35., 40.])
        root = rotation(common_gyro, np.linalg.norm(common_gyro) * tick / RATE)
        data = list(observation(angle, rate, bias=.5, pg=common_gyro, pa=root.T @ [0., 0., 1.]))
        data[1] *= 1.4
        data[3] *= 1.4
        result = observer.update(*data)
        worst = max(worst, abs(result["angle_deg"] - angle))
        previous = angle
    assert worst < 1.
    assert result["observation_sources"] == ["transverse_gyro"]
    assert not result["acc_observable"]
    assert result["relative_bias_dps"] == pytest.approx(.5, abs=.1)


def test_no_observation_grows_uncertainty_and_never_learns_bias():
    observer = ready()
    initial = observer.snapshot()
    for _ in range(600):
        result = observer.update(*observation(0., bias=1., pa=(1., 0., 0.)))
    assert result["quality"] == "prediction_only"
    assert result["angle_std_deg"] > initial["angle_std_deg"] + 5.
    assert result["relative_bias_dps"] == 0.
    assert result["angle_deg"] == pytest.approx(5., abs=1e-9)
    assert result["observation_sources"] == []


def test_invalid_norm_acceleration_burst_does_not_reset_or_learn_bias():
    observer = ready(neutral=27.)
    for _ in range(120):
        observer.update(*observation(27.))
    for _ in range(240):
        pg, pa, dg, da = observation(27.)
        result = observer.update(pg, pa, dg, da + [0., 1.5, .5])
        assert result["angle_deg"] == pytest.approx(0., abs=1e-9)
        assert result["relative_bias_dps"] == 0.
        assert not result["correction_active"]


@pytest.mark.parametrize("disturbance", ["norm", "axis", "direction"])
def test_rejected_acceleration_recovers_tracking_without_fake_history_loss(disturbance):
    observer = ready()
    for tick in range(1, 121):
        observer.update(*observation(45. * tick / RATE, 45.))
    for _ in range(120):
        pg, pa, dg, da = observation(45.)
        if disturbance == "norm":
            da *= 1.4
        elif disturbance == "axis":
            da = [.5, da[1] * np.sqrt(.75), da[2] * np.sqrt(.75)]
        else:
            da = rx(30.) @ da
        result = observer.update(pg, pa, dg, da)
        assert not result["history_loss"]
        assert not result["current_angle_observed"]
        assert result["observation_rejection_active"]
        assert result["angle_deg"] == pytest.approx(45., abs=1e-8)
    for _ in range(120):
        result = observer.update(*observation(45.))
    assert result["quality"] == "tracking"
    assert result["current_angle_observed"]
    assert not result["uncertain"]
    assert not result["history_loss"]
    assert result["history_loss_causes"] == []
    assert result["observation_rejection_count"] >= 120
    assert result["angle_deg"] == pytest.approx(45., abs=1e-8)


def test_sustained_unit_norm_direction_jump_is_not_blessed_after_timeout():
    observer = ready()
    for _ in range(120):
        observer.update(*observation(0.))
    for _ in range(600):
        pg, pa, dg, _ = observation(0.)
        result = observer.update(pg, pa, dg, rx(35.) @ pa)
        assert result["angle_deg"] == pytest.approx(0., abs=1e-9)
        assert not result["correction_active"]
        assert "gravity_continuous_observation_jump" in result["reasons"]
    assert not result["history_loss"]
    assert result["observation_rejection_count"] >= 600
    result = observer.update(*observation(0.))
    assert result["angle_deg"] == pytest.approx(0., abs=1e-9)
    assert not result["history_loss"]
    assert result["quality"] == "tracking"
    assert result["current_angle_observed"]
    assert result["time_since_angle_observation_s"] == 0.


def test_gyro_magnitude_mismatch_cannot_be_hidden_by_gravity():
    observer = ready()
    pg, pa, dg, da = observation(0., pg=(0., 30., 30.))
    dg[1:] *= 1.5
    result = observer.update(pg, pa, dg, da)
    assert not result["history_loss"]
    assert result["observation_rejection_count"] == 1
    assert result["observation_sources"] == []
    assert "hinge_motion_inconsistent" in result["reasons"]


def test_new_gyro_source_cannot_launder_a_continuous_gravity_jump():
    observer = ready()
    for _ in range(120):
        observer.update(*observation(0.))
    for _ in range(1200):
        # Both apparent angle observations jump, but no relative angular
        # velocity or missing pair explains it. The gyro angle source was
        # previously unavailable, not previously accepted and then lost.
        result = observer.update(*observation(35., pg=(0., 30., 40.)))
        assert abs(result["angle_deg"]) < 1e-9
        assert not result["correction_active"]
    assert not result["history_loss"]
    assert result["observation_rejection_count"] >= 1200


def test_saturation_reacquires_current_bent_angle_faster_than_rate_limited_baseline():
    new, old = ready(), ready(cls=ThumbFlexionEstimator)
    # Simulate a lost/clipped rotation: actual hinge reaches 65 degrees but
    # the reported integral reaches only 20 degrees. Both receive the same data.
    for tick in range(1, 13):
        data = observation(65. * tick / 12., 200.)
        for estimator in (new, old):
            estimator.update(*data, distal_saturated=True)
    recovered = None
    for tick in range(1, 241):
        current = new.update(*observation(65.))
        baseline = old.update(*observation(65.))
        if recovered is None and abs(current["angle_deg"] - 65.) < 2.:
            recovered = tick / RATE
    assert recovered is not None and recovered < .4
    assert abs(current["angle_deg"] - 65.) < .5
    assert abs(baseline["angle_deg"] - 65.) > 20.
    assert current["history_loss"]
    assert "gyro_saturation" in current["history_loss_causes"]
    assert current["current_angle_observed"]
    assert current["quality"] == "observed_after_loss"
    assert current["angle_deg"] > 64.


def test_long_gravity_gap_with_changed_bias_recovers_observed_angle_not_zero():
    observer = ready()
    for _ in range(600):
        observer.update(*observation(0.))
    for tick in range(1, 121):
        observer.update(*observation(40. * tick / RATE, 40.))
    # Force an observation gap while gyro bias changes; full time history is
    # intentionally unavailable from gravity. A returned pose is still bent.
    for _ in range(1200):
        observer.update(*observation(40., bias=3., pa=(1., 0., 0.)))
    assert abs(observer.snapshot()["angle_deg"] - 40.) > 20.
    for _ in range(120):
        result = observer.update(*observation(40., bias=3.))
    assert result["angle_deg"] == pytest.approx(40., abs=1.)
    assert result["history_loss"]
    assert "large_observation_reacquisition" in result["history_loss_causes"]


def test_known_missing_pair_is_not_zero_velocity_and_history_remains_latched():
    observer = ready(neutral=15.)
    prior = observer.snapshot()
    for _ in range(120):
        result = observer.update(None, None, None, None, pair_valid=False)
    assert result["angle_deg"] == prior["angle_deg"]
    assert result["angle_std_deg"] > 40.
    assert result["history_loss"]
    assert "missing_pair" in result["history_loss_causes"]
    for _ in range(120):
        result = observer.update(*observation(65.))
    assert result["angle_deg"] == pytest.approx(50., abs=.2)
    assert result["history_loss"]


def test_modulo_angle_innovation_does_not_create_360_degree_jump():
    observer = ready(neutral=170.)
    for tick in range(1, 121):
        result = observer.update(*observation(170. + tick / 120. * 30., 30.))
    assert result["angle_deg"] == pytest.approx(30., abs=1e-8)


@pytest.mark.parametrize("sample_rate", [10., 120., 500.])
def test_explicit_dt_and_covariance_stay_finite_symmetric_positive(sample_rate):
    observer = ConstrainedHingeObserver(sample_rate)
    observer.reset([0, 0, 1], [0, 0, 1])
    rng = np.random.default_rng(8)
    previous = 0.
    for _ in range(300):
        dt = float(rng.uniform(.5, 1.5) / sample_rate)
        angle = previous + 15. * dt
        result = observer.update(*observation(angle, 15., bias=.3), dt_s=dt)
        np.testing.assert_allclose(observer._p, observer._p.T, atol=1e-12)
        assert np.linalg.eigvalsh(observer._p).min() >= -1e-12
        json.dumps(result, allow_nan=False)
        previous = angle


@pytest.mark.parametrize("kwargs", [dict(dt_s=0), dict(dt_s=float("nan")), dict(pair_valid=1),
                                     dict(proximal_saturated=1), dict(distal_saturated="yes")])
def test_invalid_update_parameters_do_not_mutate_state(kwargs):
    observer = ready()
    before = observer.snapshot()
    with pytest.raises(ValueError):
        observer.update(*observation(0.), **kwargs)
    assert observer.snapshot() == before


@pytest.mark.parametrize("position,value", [(0, [float("nan"), 0, 0]), (1, [0, 0]),
                                           (2, [float("inf"), 0, 0]), (3, [True, False, True])])
def test_invalid_vectors_do_not_mutate_state(position, value):
    observer = ready()
    before = observer.snapshot()
    data = list(observation(0.)); data[position] = value
    with pytest.raises(ValueError):
        observer.update(*data)
    assert observer.snapshot() == before


def test_failed_reset_is_transactional_and_snapshots_are_independent():
    observer = ready(neutral=20.)
    before = observer.snapshot()
    with pytest.raises(ValueError):
        observer.reset([1, 0, 0], [1, 0, 0])
    assert observer.snapshot() == before
    with pytest.raises(ValueError, match="both current"):
        observer.reset([0, 0, 1], [0, 0, 1], current_proximal_acc=[0, 0, 1])
    assert observer.snapshot() == before
    before["observation_sources"].append("fake")
    before["reasons"].append("fake")
    assert "fake" not in observer.snapshot()["reasons"]
    assert "fake" not in observer.snapshot()["observation_sources"]


@pytest.mark.parametrize("rate", [0., 5., 2001., True, float("nan"), "120"])
def test_constructor_rejects_invalid_rate(rate):
    with pytest.raises(ValueError):
        ConstrainedHingeObserver(rate)
