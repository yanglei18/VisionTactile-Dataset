"""Manual recalibration preserves a qualified same-stream zero rate, not orientation."""
from importlib.resources import files

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from vqf import VQF

from vt_fsglove12 import live_stream
from vt_fsglove12.config import load_config
from vt_fsglove12.input_contract import raw_to_si
from vt_fsglove12.live_stream import OnlineFusion, RawSample


def sample(stamp, raw_z=86):
    return RawSample(11, (0, 0, raw_z, 0, 0, 4098), stamp, stamp + 1000.)


@pytest.fixture
def stationary(monkeypatch):
    config = load_config(files("vt_fsglove12").joinpath("configs/right_new_imu.json"))
    fusion = OnlineFusion(config)
    now = [0.]
    monkeypatch.setattr(live_stream.host_clock, "monotonic", lambda: now[0])
    # An injected 3.01 deg/s bias while physically static at 104 Hz.
    for i in range(8 * 104):
        now[0] = 1. + i / 104.
        fusion.update([sample(now[0])])
    assert fusion.states["11"]["filter"].getRestDetected()
    assert fusion.input_health.node_status("11", now[0])["state"] == "healthy"
    return fusion, config, now


def fresh_estimate(row, bias_sigma=None):
    estimator = VQF(1. / 104., biasClip=8.)
    if bias_sigma is not None:
        estimator.setBiasEstimate(*bias_sigma)
    estimator.update(*raw_to_si(row.values, gyro_dps_per_lsb=.035, accel_g_per_lsb=.000244))
    return estimator


def test_fresh_settled_recalibration_keeps_bias_and_measures_motion(stationary):
    fusion, _, now = stationary
    previous = fusion.states["11"]["filter"]
    bias_sigma = previous.getBiasEstimate()
    reset = fusion.reset_for_calibration()
    assert fusion.epochs["11"] == 1 and not fusion.snapshot(now[0])["nodes"]
    for i in range(2 * 104):
        now[0] += 1. / 104.
        row = sample(now[0], raw_z=486)  # Closed-loop: a real 14 deg/s turn starts immediately.
        fusion.update([row])
        if i == 0:
            current = fusion.states["11"]["filter"]
            assert current is not previous
            expected = fresh_estimate(row, bias_sigma)
            np.testing.assert_array_equal(current.getQuat6D(), expected.getQuat6D())
    angle = Rotation.from_quat(fusion.states["11"]["quat_xyzw"]).as_rotvec()[2]
    assert np.rad2deg(angle) == pytest.approx(28., abs=.25)
    assert reset["bias_seed_nodes"] == ["11"]


@pytest.mark.parametrize("reason", ["stale", "unhealthy", "unconfirmed_gap", "unconverged",
                                    "moving", "guarded", "reconnect",
                                    "late_input", "before_reset", "delayed_processing", "combined_gap",
                                    "clock_rollback", "second_reset", "first_input_guarded"])
def test_recalibration_does_not_seed_a_discontinuous_or_unqualified_stream(stationary, monkeypatch, reason):
    fusion, config, now = stationary
    last = now[0]
    if reason == "stale":
        now[0] += .2
    elif reason == "before_reset":
        now[0] += .005
    elif reason == "combined_gap":
        now[0] += .12  # Reset and next input look fresh separately but straddle an outage.
    elif reason == "unhealthy":
        monkeypatch.setattr(fusion.input_health, "node_status", lambda *args: {"state": "faulted"})
    elif reason == "unconfirmed_gap":
        monkeypatch.setattr(fusion._arrival_gaps, "is_pending", lambda key: key == "11")
    elif reason == "unconverged":
        estimator = fusion.states["11"]["filter"]
        bias, _ = estimator.getBiasEstimate()
        estimator.setBiasEstimate(bias, np.deg2rad(estimator.params["biasSigmaInit"]))
    elif reason == "moving":
        for i in range(104):
            now[0] += 1. / 104.
            fusion.update([sample(now[0], raw_z=486)])
    elif reason == "guarded":
        now[0] += 1. / 104.
        fusion.update([sample(now[0], raw_z=32767)])
    fusion.reset_for_calibration()
    stamp = now[0] + .01
    if reason == "reconnect":
        fusion = OnlineFusion(config)
    elif reason == "late_input":
        stamp += .2
    elif reason == "before_reset":
        stamp = last + .001  # Buffered before the reset, but not a clock rollback.
    elif reason == "combined_gap":
        stamp += .04
    elif reason == "clock_rollback":
        stamp = last - 1.
    elif reason == "second_reset":
        fusion.reset_for_calibration()
    now[0] = stamp + (.2 if reason == "delayed_processing" else 0.)
    if reason == "first_input_guarded":
        fusion.update([sample(stamp, raw_z=32767)])
        stamp += .01
        now[0] = stamp
    row = sample(stamp)
    fusion.update([row])
    # A carried bias would already be near +3 deg/s; a new estimate is still zero.
    np.testing.assert_allclose(fusion.states["11"]["filter"].getBiasEstimate()[0],
                               fresh_estimate(row).getBiasEstimate()[0], atol=1e-6)
