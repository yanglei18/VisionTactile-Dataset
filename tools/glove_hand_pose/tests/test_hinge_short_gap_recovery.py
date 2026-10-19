"""Synthetic policy checks, not physical latency or hardware accuracy tests."""
import math

import numpy as np
import pytest

from vt_glove_pose.hinge_observer import ConstrainedHingeObserver


RATE = 120.


def rx(degrees):
    a = math.radians(degrees)
    return np.array([[1., 0., 0.], [0., math.cos(a), -math.sin(a)],
                     [0., math.sin(a), math.cos(a)]])


def data(angle, rate=0., *, norm_ok=True, pa=(0., 0., 1.), common=(0., 0., 0.)):
    pa, pg = np.array(pa, float), np.array(common, float)
    da = rx(angle) @ pa
    if not norm_ok:
        da *= 1.4
    return pg, pa, rx(angle) @ pg + [-rate, 0., 0.], da


def ready(angle=25.):
    out = ConstrainedHingeObserver(RATE)
    _, pa, _, da = data(angle)
    out.reset([0., 0., 1.], [0., 0., 1.], current_proximal_acc=pa, current_distal_acc=da)
    return out


def assert_confirmation_then_observation(observer, angle):
    # First returning sample starts (rather than finishes) confirmation.
    result = observer.update(*data(angle))
    assert not result["current_angle_observed"]
    assert "gravity_observation_confirming" in result["reasons"]
    for _ in range(17):
        assert not observer.update(*data(angle))["current_angle_observed"]
    return observer.update(*data(angle))


@pytest.mark.parametrize("missing_observations", [1, 3, 12])
def test_short_acc_norm_dropout_immediately_resumes_established_observation(missing_observations):
    observer = ready()
    for _ in range(missing_observations):
        result = observer.update(*data(25., norm_ok=False))
        assert not result["current_angle_observed"]
    result = observer.update(*data(25.))
    assert result["current_angle_observed"] and result["valid"]
    assert result["quality"] == "tracking"
    assert result["angle_deg"] == pytest.approx(25.)
    assert result["relative_bias_dps"] == pytest.approx(0.)
    assert not result["history_loss"]
    assert "gravity_short_observation_gap_recovered" in result["reasons"]


def test_real_motion_during_short_dropout_uses_integrated_phase_not_last_angle():
    observer = ready()
    angle = 25.
    for _ in range(6):
        angle += 2.
        observer.update(*data(angle, 240., norm_ok=False))
    angle += 2.
    result = observer.update(*data(angle, 240.))
    assert result["current_angle_observed"]
    assert result["angle_deg"] == pytest.approx(39., abs=1e-9)
    assert abs(result["innovation_deg"]) < 1e-9
    assert not result["history_loss"]


def test_short_projection_dropout_recovers_without_straightening_bent_reference():
    observer = ConstrainedHingeObserver(RATE)
    _, pa, _, da = data(18.)
    _, cp, _, cd = data(55.)
    observer.reset(pa, da, current_proximal_acc=cp, current_distal_acc=cd)
    for _ in range(4):
        result = observer.update(*data(55., pa=(1., 0., 0.)))
        assert not result["current_angle_observed"]
    result = observer.update(*data(55.))
    assert result["current_angle_observed"]
    assert result["angle_deg"] == pytest.approx(37., abs=1e-9)


@pytest.mark.parametrize("intermittent_invalid", [False, True])
def test_short_dropout_cannot_launder_persistent_unit_norm_jump(intermittent_invalid):
    observer = ready(0.)
    observer.update(*data(0., norm_ok=False))
    for tick in range(1200):
        # Repeated invalid single frames must not grow a fictitious long gap,
        # and filter covariance growth must not replace anchor continuity.
        norm_ok = not (intermittent_invalid and tick % 50 == 49)
        result = observer.update(*data(35., norm_ok=norm_ok))
        assert not result["current_angle_observed"]
        assert result["angle_deg"] == pytest.approx(0., abs=1e-9)
        assert result["relative_bias_dps"] == pytest.approx(0., abs=1e-9)
        assert not result["history_loss"]
    # The original anchor remains reusable even after a long period of rejects.
    result = observer.update(*data(0.))
    assert result["current_angle_observed"] and result["valid"]
    assert not result["history_loss"]


def test_new_transverse_source_cannot_launder_short_dropout_gravity_jump():
    observer = ready(0.)
    observer.update(*data(0., norm_ok=False))
    for _ in range(240):
        result = observer.update(*data(35., common=(0., 30., 40.)))
        assert not result["current_angle_observed"]
        assert result["angle_deg"] == pytest.approx(0., abs=1e-9)


@pytest.mark.parametrize("loss", ["missing_pair", "proximal_saturation", "distal_saturation"])
def test_real_gyro_history_loss_still_requires_confirmation(loss):
    observer = ready()
    if loss == "missing_pair":
        observer.update(None, None, None, None, pair_valid=False)
    else:
        kwargs = {"proximal_saturated" if loss.startswith("proximal") else "distal_saturated": True}
        observer.update(*data(25.), **kwargs)
    result = assert_confirmation_then_observation(observer, 25.)
    assert result["current_angle_observed"] and result["history_loss"]
    assert "gravity_short_observation_gap_recovered" not in result["reasons"]


def test_repeated_missing_pair_clears_pending_and_preserves_confirmation_requirement():
    observer = ready()
    observer.update(None, None, None, None, pair_valid=False)
    for _ in range(10):
        assert not observer.update(*data(25.))["current_angle_observed"]
    observer.update(None, None, None, None, pair_valid=False)
    result = assert_confirmation_then_observation(observer, 25.)
    assert result["current_angle_observed"] and result["history_loss"]


def test_gap_longer_than_point_one_second_still_confirms():
    observer = ready()
    for _ in range(13):
        observer.update(*data(25., norm_ok=False))
    result = assert_confirmation_then_observation(observer, 25.)
    assert result["current_angle_observed"] and not result["history_loss"]
    assert "gravity_short_observation_gap_recovered" not in result["reasons"]


def test_first_new_observation_source_still_confirms():
    observer = ready(25.)
    for _ in range(18):
        result = observer.update(*data(25., norm_ok=False, common=(0., 30., 40.)))
        assert not result["current_angle_observed"]
        assert "transverse_gyro_observation_confirming" in result["reasons"]
    result = observer.update(*data(25., norm_ok=False, common=(0., 30., 40.)))
    assert result["observation_sources"] == ["transverse_gyro"]


def test_old_latched_loss_does_not_force_reacquisition_after_new_anchor_short_dropout():
    observer = ready()
    observer.update(None, None, None, None, pair_valid=False)
    for _ in range(360):
        before = observer.update(*data(25.))
    assert before["history_loss"] and before["current_angle_observed"]
    observer.update(*data(25., norm_ok=False))
    result = observer.update(*data(25.))
    assert result["current_angle_observed"] and result["valid"]
    assert result["quality"] == "observed_after_loss"
    assert result["history_loss_causes"] == before["history_loss_causes"]
    assert "conditional_observation_reacquisition" not in result["reasons"]
    assert result["relative_bias_std_dps"] <= before["relative_bias_std_dps"]*1.01


def test_actual_new_long_gap_can_reestablish_conditional_observation_after_rejected_jump():
    observer = ready(0.)
    observer.update(*data(0., norm_ok=False))
    for _ in range(120):
        assert not observer.update(*data(35.))["current_angle_observed"]
    for _ in range(120):
        observer.update(*data(35., norm_ok=False))
    first = observer.update(*data(35.))
    assert not first["current_angle_observed"]
    for _ in range(119):
        result = observer.update(*data(35.))
    assert result["current_angle_observed"] and result["history_loss"]
    assert result["angle_deg"] == pytest.approx(35., abs=.1)
    assert "large_observation_reacquisition" in result["history_loss_causes"]


def test_policy_identifier_explicitly_distinguishes_software_continuity_from_hardware_verification():
    state = ready().snapshot()
    assert state["recovery_policy"] == "bounded_short_gap_anchor_v2"
    assert "not_verified_device_sampling" in state["gyro_history_continuity_basis"]
