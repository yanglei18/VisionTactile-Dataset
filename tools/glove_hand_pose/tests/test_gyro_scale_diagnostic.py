import numpy as np
import pytest

from vt_glove_pose.gyro_scale_diagnostic import gyro_scale_diagnostic


def trajectory(gain=1.):
    t = np.arange(1200)/120.
    speed = .5  # physical rad/s about sensor x; independently analytic gravity
    a = np.column_stack((np.zeros(len(t)), np.sin(speed*t), np.cos(speed*t)))
    g = np.tile([np.rad2deg(speed)/gain, 0., 0.], (len(t), 1))
    return g, a, t


@pytest.mark.parametrize("gain", [.5, 1., 2.])
def test_independent_physical_trajectory_detects_scale(gain):
    g, a, t = trajectory(gain)
    result = gyro_scale_diagnostic(g, a, t, neutral_gyro_dps=np.zeros((240,3)))
    assert result["suggested_multiplier"] == pytest.approx(gain, rel=.001)
    assert result["vector_cosine"] > .999
    expected = "no_large_scale_discrepancy_detected" if gain == 1. else "possible_scale_mismatch"
    assert result["status"] == expected
    assert result["hardware_units_verified"] is False
    assert result["changes_configuration"] is False


def test_constant_acceleration_scale_and_bias_do_not_change_estimate():
    g, a, t = trajectory(2.)
    bias = np.array([.3, -.2, .1])
    result = gyro_scale_diagnostic(g+bias, a*8197, t, neutral_gyro_dps=np.tile(bias,(240,1)))
    assert result["suggested_multiplier"] == pytest.approx(2., rel=.001)


def test_static_does_not_certify_units():
    _, a, t = trajectory()
    a[:] = [0.,0.,1.]
    result = gyro_scale_diagnostic(np.zeros_like(a), a, t, neutral_gyro_dps=np.zeros((240,3)))
    assert result["status"] == "insufficient_information"
    assert result["suggested_multiplier"] is None


def test_gaps_do_not_look_like_scale_changes():
    g,a,t = trajectory()
    t[600:] += 1.
    result = gyro_scale_diagnostic(g,a,t,neutral_gyro_dps=np.zeros((240,3)))
    assert result["status"] == "insufficient_information"
    assert "time" in result["reason"]


def test_wrong_rotation_direction_is_inconclusive_not_negative_scale():
    g,a,t = trajectory()
    result = gyro_scale_diagnostic(-g,a,t,neutral_gyro_dps=np.zeros((240,3)))
    assert result["status"] == "inconclusive"
