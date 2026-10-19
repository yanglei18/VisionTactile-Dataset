"""Real VQF/engine regression tests; generated motion is not hardware truth."""
import math

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.kinematics import HandModel
from vt_glove_pose.math3d import conj, from_axis_angle, rotate
from vt_glove_pose.records import Sample


def ready(hand="right", arbitrary_mounts=True):
    cfg = load_config()
    cfg["hand"] = hand
    engine = PoseEngine(cfg)
    identity = np.array([1., 0., 0., 0.])
    engine.mounts = {n: identity.copy() for n in (5, 6, 12)}
    if arbitrary_mounts:
        engine.mounts[5] = from_axis_angle([1, 2, -1], .6)
        engine.mounts[6] = from_axis_angle([-2, 1, 3], -.9)
    engine.neutral_acc = {n: rotate(conj(engine.mounts.get(n, identity)), [0., 0., 1.]) for n in range(1, 13)}
    for n in range(1, 13):
        raw = np.rint(engine.neutral_acc[n] / cfg["accel_g_per_lsb"]).astype(int)
        engine.process(Sample(n, (0, 0, 0, *raw), 10., 1700000010., 0))
    engine.zero = {n: s["quat"].copy() for n, s in engine.states.items()}
    engine.axis_sign_versions = {n: 2 for n in engine.mounts}
    engine.neutral_ready = engine.mount_confirmed = True
    engine._reset_thumb_flexion()
    return engine


def step(engine, tick, angle, speed, *, skip_proximal=False, saturated=False):
    time = 10. + tick / engine.config["sample_rate_hz"]
    q = from_axis_angle([1, 0, 0], math.radians(angle))
    for n in (5, 6, 12):
        if n == 5 and skip_proximal:
            continue
        gyro = np.array([-speed, 0., 0.]) if n == 6 else np.zeros(3)
        acc = rotate(q, [0., 0., 1.]) if n == 6 else np.array([0., 0., 1.])
        gyro = rotate(conj(engine.mounts[n]), gyro)
        acc = rotate(conj(engine.mounts[n]), acc)
        raw_gyro = np.rint(gyro / engine.config["gyro_dps_per_lsb"]).astype(int)
        if n == 6 and saturated:
            raw_gyro[0] = -32764
        raw_acc = np.rint(acc / engine.config["accel_g_per_lsb"]).astype(int)
        engine.process(Sample(n, tuple([*raw_gyro, *raw_acc]), time, 1700000000. + time, 0))
    return engine.get_snapshot(now=time)


@pytest.mark.parametrize("hand", ["right", "left"])
def test_twenty_cycles_with_different_mounts_and_real_vqf(hand):
    engine = ready(hand)
    previous, worst = 0., 0.
    for tick in range(1, 4801):
        angle = 32. * (1 - math.cos(math.pi * tick / 120.))
        # Inject differential gyro bias. Gravity, not a guessed straight pose,
        # supplies the long-term relative reference.
        result = step(engine, tick, angle, (angle - previous) * 120 + .4)
        thumb = result["model"]["fingers"]["thumb"]
        assert thumb["ip_estimator"]["quality"] == "tracking"
        worst = max(worst, abs(thumb["ip_deg"] - angle))
        previous = angle
    assert worst < 1.
    assert abs(thumb["ip_deg"]) < 1.
    assert engine.thumb_estimator.snapshot()["samples"] == 4799


def test_snapshot_reads_cannot_drive_or_repeat_pair_integration():
    engine = ready()
    step(engine, 1, 1., 120.)
    state = engine.thumb_estimator.export_state()
    for _ in range(100):
        engine.get_snapshot(now=engine.last_time)
    assert engine.thumb_estimator.export_state() == state
    assert state["state"]["samples"] == 0  # Initial pair observes pose, not a second integration.


def test_missing_pair_is_reported_not_silently_integrated_twice():
    engine = ready()
    step(engine, 1, 1., 120.)
    step(engine, 2, 2., 120., skip_proximal=True)
    result = step(engine, 3, 3., 120.)
    estimator = result["model"]["fingers"]["thumb"]["ip_estimator"]
    assert estimator["history_loss"]
    assert "pair_unavailable" in estimator["reasons"]


def test_rejected_half_frame_cannot_pair_with_next_frame_in_one_usb_batch():
    engine = ready(arbitrary_mounts=False)
    step(engine, 1, 0., 0.)
    t = engine.last_time + .008
    # All rows from this USB read share the SAME host timestamp. Frame A's
    # node 5 is rejected, followed by frame B's normal node 5 and node 6.
    engine.process(Sample(5, (0, 0, 0, 0, 0, 0), t, 1700000000.+t, 0))
    engine.process(Sample(6, (0, 0, 0, 0, 0, 8197), t, 1700000000.+t, 0))
    before = engine.thumb_estimator.export_state()
    engine.process(Sample(5, (0, 0, 0, 0, 0, 8197), t, 1700000000.+t, 0))
    assert engine.thumb_estimator.export_state() == before
    engine.process(Sample(6, (0, 0, 0, 0, 0, 8197), t, 1700000000.+t, 0))
    assert engine.thumb_estimator.snapshot()["history_loss"]
    assert not engine.get_snapshot(now=t)["model"]["fingers"]["thumb"]["ip_confident"]


def test_observed_old_neutral_axis_mismatch_never_silently_uses_legacy_ip():
    engine = ready(arbitrary_mounts=False)
    engine.neutral_acc[5] = np.array([.35516, 0., math.sqrt(1-.35516**2)])
    engine.neutral_acc[6] = np.array([.53625, 0., math.sqrt(1-.53625**2)])
    thumb = step(engine, 1, 0., 0.)["model"]["fingers"]["thumb"]
    assert engine.thumb_estimator is None
    assert thumb["ip_deg"] is None
    assert not thumb["ip_valid"]
    assert thumb["ip_estimator"]["quality"] == "reference_inconsistent"
    assert "axis_gravity_mismatch" in thumb["ip_estimator"]["reasons"][-1]


def test_saturation_is_visible_and_does_not_stop_other_nodes():
    engine = ready(arbitrary_mounts=False)
    step(engine, 1, 0., 0.)
    result = step(engine, 2, 1., 120., saturated=True)
    thumb = result["model"]["fingers"]["thumb"]
    assert thumb["ip_estimator"]["history_loss"]
    assert not thumb["ip_confident"]
    assert thumb["quality"] == "thumb_uncertain"
    assert result["nodes"]["12"]["samples"] == 3
    assert result["gap_count"] == 0


def test_prediction_and_reacquisition_remain_visible_not_a_fake_straight_pose():
    cfg = load_config()
    values = {n: np.array([1., 0., 0., 0.]) for n in (5, 6, 12)}
    for quality in ("prediction_only", "uncertain", "reacquiring"):
        result = HandModel(cfg).solve(values, set(values), set(values), True,
            thumb_flexion=dict(angle_deg=43., valid=False, uncertain=True, quality=quality, reasons=[]))
        thumb = result["fingers"]["thumb"]
        assert thumb["ip_deg"] == 43.
        assert thumb["geometry_live"] and thumb["ip_valid"]
        assert not thumb["ip_confident"]
        assert thumb["quality"] == "thumb_" + quality


def test_new_ip_geometry_never_wraps_179_to_minus_179():
    cfg = load_config()
    values = {n: np.array([1., 0., 0., 0.]) for n in (5, 6, 12)}
    out = []
    for angle in (179., 181.):
        thumb = HandModel(cfg).solve(values, set(values), set(values), True,
            thumb_flexion=dict(angle_deg=angle, valid=False, uncertain=True, quality="uncertain", reasons=[]))["fingers"]["thumb"]
        assert thumb["raw_angles_deg"]["ip_deg"] == angle
        assert "ip_deg" in thumb["clipped_angles"]
        out.append(thumb["ip_deg"])
    assert out == [90., 90.]


def test_prediction_does_not_hide_stale_palm_priority():
    cfg = load_config()
    values = {n: np.array([1., 0., 0., 0.]) for n in (5, 6, 12)}
    thumb = HandModel(cfg).solve(values, {5, 6}, set(values), True,
        thumb_flexion=dict(angle_deg=43., valid=False, uncertain=True, quality="prediction_only", reasons=[]))["fingers"]["thumb"]
    assert thumb["quality"] == "palm_stale"
    assert not thumb["geometry_live"]
    assert thumb["ip_deg"] == 43.
    assert thumb["ip_estimator"]["quality"] == "prediction_only"


def test_enabling_joint_axes_while_bent_initializes_current_pose_not_zero():
    engine = ready(arbitrary_mounts=False)
    thumb = step(engine, 1, 45., 0.)["model"]["fingers"]["thumb"]
    assert thumb["ip_deg"] == pytest.approx(45., abs=.01)
    engine._reset_thumb_flexion()  # Same lifecycle as accepted joint axes.
    thumb = step(engine, 2, 45., 0.)["model"]["fingers"]["thumb"]
    assert thumb["ip_deg"] == pytest.approx(45., abs=.01)
    assert thumb["ip_estimator"]["quality"] == "tracking"
