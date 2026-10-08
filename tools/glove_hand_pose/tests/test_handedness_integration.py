"""Physically mirrored inputs, not just mirrored copies of synthetic output.

Mount rotations map sensor coordinates into right-handed physical segment
coordinates.  Left-hand physical splay and rotations must be reflected by the
engine before entering the model's canonical right-hand angle convention.
"""

import math

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.math3d import conj, from_axis_angle, mul, rotate
from vt_glove_pose.records import Sample


I = np.array([1., 0., 0., 0.])


def q(axis, degrees):
    return from_axis_angle(axis, math.radians(degrees))


def physical_rotation(canonical, hand):
    return canonical * [1., 1., -1., -1.] if hand == "left" else canonical.copy()


def make_engine(hand):
    config = load_config()
    config["hand"] = hand
    return PoseEngine(config)


def index_nodes(engine):
    return tuple(engine.config["fingers"]["index"][key] for key in ("proximal", "distal"))


def set_physical_poses(engine, canonical_poses, common=I):
    """Supply exact sensor quaternions with distinct mounts and yaw gauges."""
    neutral = {node: I.copy() for node in range(1, 13)}
    for finger in engine.config["fingers"].values():
        for node in (finger["proximal"], finger["distal"]):
            neutral[node] = q([0, 0, 1], finger["neutral_splay_deg"])
    for node in range(1, 13):
        mount = q([1 + node % 3, -2, 3], 13 * node)
        gauge = q([0, 0, 1], 21 * node)
        engine.mounts[node] = mount
        physical_neutral = physical_rotation(neutral[node], engine.config["hand"])
        physical_pose = physical_rotation(canonical_poses.get(node, neutral[node]), engine.config["hand"])
        engine.zero[node] = mul(gauge, mul(physical_neutral, mount))
        sensor = mul(gauge, mul(common, mul(physical_pose, mount)))
        engine.states[node] = {"quat": -sensor if node % 2 else sensor, "time": 10.}
    engine.neutral_ready = True
    return engine.model.solve(engine._orientations(), set(range(1, 13)), set(engine.mounts), True)


@pytest.mark.parametrize("hand", ["right", "left"])
@pytest.mark.parametrize("common", [q([-1, 0, 0], 30), q([1, -2, 3], 75)])
def test_common_rigid_rotation_preserves_neutral_thumb_and_fingers(hand, common):
    engine = make_engine(hand)
    model = set_physical_poses(engine, {}, common=common)
    for name, finger in model["fingers"].items():
        assert finger["valid"]
        assert finger["mcp_deg"] == pytest.approx(0, abs=1e-10)
        assert finger["splay_deg"] == pytest.approx(0, abs=1e-10)
        assert finger["ip_deg" if name == "thumb" else "pip_deg"] == pytest.approx(0, abs=1e-10)


def test_mirrored_physical_inputs_with_arbitrary_mounts_preserve_flex_and_splay():
    poses = {}
    settings = load_config()
    for finger in settings["fingers"].values():
        proximal = mul(q([0, 0, 1], finger["neutral_splay_deg"] - 15), q([-1, 0, 0], 25))
        poses[finger["proximal"]] = proximal
        poses[finger["distal"]] = mul(proximal, q([-1, 0, 0], 40))
    right = set_physical_poses(make_engine("right"), poses, common=q([1, -2, 3], 43))
    left = set_physical_poses(make_engine("left"), poses, common=q([-3, 1, 2], 61))
    for name in settings["fingers"]:
        for model in (right, left):
            finger = model["fingers"][name]
            assert finger["mcp_deg"] == pytest.approx(25)
            assert finger["splay_deg"] == pytest.approx(-15)
            assert finger["ip_deg" if name == "thumb" else "pip_deg"] == pytest.approx(40)
        np.testing.assert_allclose(left["fingers"][name]["points"],
                                   np.array(right["fingers"][name]["points"]) * [-1, 1, 1], atol=1e-10)


def test_left_physical_positive_z_splay_moves_toward_negative_x():
    engine = make_engine("left")
    # A physical +20-degree yaw of an extended finger points toward -x.
    # Its canonical right-hand representation is -20 degrees, not +20.
    canonical = q([0, 0, 1], -20)
    finger = set_physical_poses(engine, {node: canonical for node in index_nodes(engine)})["fingers"]["index"]
    direction = np.diff(np.array(finger["points"]), axis=0)[0] / 42
    np.testing.assert_allclose(direction, [-math.sin(math.radians(20)), math.cos(math.radians(20)), 0], atol=1e-10)
    assert finger["splay_deg"] == pytest.approx(-20)


@pytest.mark.parametrize("hand", ["right", "left"])
def test_real_vqf_splay_with_tilted_distinct_sensor_mounts(hand):
    engine = make_engine(hand)
    mounts = {node: q([1 + node % 3, -2, 3], node * 13) for node in range(1, 13)}
    engine.mounts = mounts.copy()
    settings = engine.config
    moving_nodes = index_nodes(engine)
    neutral = {node: I.copy() for node in range(1, 13)}
    for finger in settings["fingers"].values():
        for node in (finger["proximal"], finger["distal"]):
            neutral[node] = physical_rotation(q([0, 0, 1], finger["neutral_splay_deg"]), hand)

    def feed(tick, angle=0., speed=0.):
        for node in range(1, 13):
            segment = q([0, 0, 1], angle) if node in moving_nodes else neutral[node]
            sensor = mul(segment, mounts[node])
            gyro = rotate(conj(mounts[node]), [0, 0, speed if node in moving_nodes else 0.])
            accel = rotate(conj(sensor), [0, 0, 1.])
            raw = np.rint(np.concatenate((gyro / settings["gyro_dps_per_lsb"], accel / settings["accel_g_per_lsb"]))).astype(int)
            stamp = 10 + tick / settings["sample_rate_hz"]
            engine.process(Sample(node, tuple(raw), stamp, stamp))

    for tick in range(600):
        feed(tick)
    engine.zero = {node: state["quat"].copy() for node, state in engine.states.items()}
    engine.neutral_ready = True
    for tick in range(600, 720):
        feed(tick, angle=20 * (tick - 599) / 120, speed=20.)
    result = engine.get_snapshot(now=10 + 719 / 120)["model"]["fingers"]["index"]
    direction = np.diff(np.array(result["points"]), axis=0)[0] / 42
    # Same physical motion points toward -x for both hands; only the canonical
    # splay sign changes.  End-only mirroring would point the left finger +x.
    np.testing.assert_allclose(direction, [-math.sin(math.radians(20)), math.cos(math.radians(20)), 0], atol=0.002)
    assert result["splay_deg"] == pytest.approx(-20 if hand == "left" else 20, abs=0.1)
    assert result["mcp_deg"] == pytest.approx(0, abs=0.1)
    assert result["pip_deg"] == pytest.approx(0, abs=0.1)


@pytest.mark.parametrize("hand", ["right", "left"])
@pytest.mark.parametrize("bias_axis", [0, 1, 2])
def test_real_vqf_neutral_rebuilds_histories_with_new_static_bias(hand, bias_axis):
    engine = make_engine(hand)
    engine.mounts = {node: I.copy() for node in range(1, 13)}
    biased_nodes = index_nodes(engine)

    def feed(tick):
        for node in range(1, 13):
            # Accepted static bias exceeds VQF's automatic rest detector's
            # default clip.  A historical quaternion mean offsets yaw, while
            # changing only bias leaves inclination histories drifting in X/Y.
            # The gyro readings remain constant; the hand is static.
            raw = [0, 0, 0, 0, 0, 8197]
            if node in biased_nodes:
                raw[bias_axis] = 686
            stamp = 10 + tick / 120
            engine.process(Sample(node, raw, stamp, stamp))

    feed(0)
    engine.begin_neutral(duration=5, confirm_mount=True)
    for tick in range(1, 601):
        feed(tick)
    assert engine.neutral_ready
    assert engine.bias[biased_nodes[0]][bias_axis] == pytest.approx(6.0025)
    # Ten further seconds catch the old inclination LPF settling transient,
    # which previously accumulated about 18 degrees despite a stationary hand.
    for tick in range(600, 1801):
        if tick > 600:
            feed(tick)
        if tick not in (600, 601, 620, 720, 840, 1200, 1800):
            continue
        finger = engine.get_snapshot(now=10 + tick / 120)["model"]["fingers"]["index"]
        assert finger["valid"]
        assert finger["mcp_deg"] == pytest.approx(0, abs=1e-8)
        assert finger["splay_deg"] == pytest.approx(0, abs=1e-8)
        assert finger["pip_deg"] == pytest.approx(0, abs=1e-8)
        for node in biased_nodes:
            np.testing.assert_allclose(mul(conj(engine.zero[node]), engine.states[node]["quat"]), I, atol=1e-8)


def test_neutral_filter_rebuild_is_transactional_and_preserves_sample_accounting(monkeypatch):
    engine = make_engine("right")
    for node in range(1, 13):
        engine.process(Sample(node, (0, 0, 0, 0, 0, 8197), 10., 10.))
    engine.neutral_ready = True
    engine.zero = {node: I.copy() for node in range(1, 13)}
    engine.neutral_acc = {node: np.array([0., 0., 1.]) for node in range(1, 13)}
    old_filters = {node: state["filter"] for node, state in engine.states.items()}
    old_zero, old_bias, old_acc = engine.zero, engine.bias, engine.neutral_acc
    old_accounting = {node: (state["count"], state["time"], list(state["times"])) for node, state in engine.states.items()}
    engine.begin_neutral(duration=5, confirm_mount=True)
    for node in range(1, 13):
        engine.active["rows"][node] = [(10 + tick / 120, np.zeros(3), np.array([0., 0., 1.]), I.copy()) for tick in range(600)]
    capture = engine.active
    real_factory = engine._new_filter

    def failing_factory(node, bias=None):
        if node == 12:
            raise ValueError("injected last-node filter initialization failure")
        return real_factory(node, bias=bias)

    monkeypatch.setattr(engine, "_new_filter", failing_factory)
    engine._finish_calibration()
    assert "未通过" in engine.message
    assert engine.neutral_ready
    assert engine.zero is old_zero and engine.bias is old_bias and engine.neutral_acc is old_acc
    assert all(engine.states[node]["filter"] is filt for node, filt in old_filters.items())

    monkeypatch.setattr(engine, "_new_filter", real_factory)
    engine.active = capture
    engine._finish_calibration()
    assert "归零完成" in engine.message
    assert all(engine.states[node]["filter"] is not filt for node, filt in old_filters.items())
    assert {node: (state["count"], state["time"], list(state["times"])) for node, state in engine.states.items()} == old_accounting


def test_fresh_unmounted_engine_nodes_are_not_mislabeled_stale():
    engine = make_engine("right")
    for node in range(1, 13):
        engine.process(Sample(node, (0, 0, 0, 0, 0, 8197), 10., 10.))
    engine.zero = {node: I.copy() for node in range(1, 13)}
    engine.neutral_ready = True
    snapshot = engine.get_snapshot(now=10)
    assert all(node["fresh"] for node in snapshot["nodes"].values())
    assert all(finger["quality"] == "axes_unverified" for finger in snapshot["model"]["fingers"].values())
    assert any("Palm segment axes are unverified" in warning for warning in snapshot["model"]["warnings"])
    assert not any("stale" in warning.lower() for warning in snapshot["model"]["warnings"])

    engine.mounts.update({node: I.copy() for node in index_nodes(engine)})
    finger = engine.get_snapshot(now=10)["model"]["fingers"]["index"]
    assert finger["quality"] == "palm_axes_unverified"
    assert finger["pip_valid"] and not finger["mcp_valid"]
    assert not finger["valid"]

    # Actual loss still takes precedence over missing axes.
    snapshot = engine.get_snapshot(now=11)
    assert all(finger["quality"] == "stale" for finger in snapshot["model"]["fingers"].values())
