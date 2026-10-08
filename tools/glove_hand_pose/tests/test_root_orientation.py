"""Root-follow contract and gyro-consistent real-VQF response acceptance.

These are software-only physical simulations, not device accuracy evidence.
"""

import math

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.engine import PoseEngine, demo_snapshot
from vt_glove_pose.kinematics import HandModel
from vt_glove_pose.math3d import conj, from_axis_angle, mul, rotate, to_matrix
from vt_glove_pose.records import Sample


I = np.array([1., 0., 0., 0.])


def q(axis, degrees):
    return from_axis_angle(axis, math.radians(degrees))


def canonical(physical, hand):
    return physical * [1., 1., -1., -1.] if hand == "left" else physical.copy()


def orientations(config, physical_root):
    root = canonical(physical_root, config["hand"])
    result = {node: root.copy() for node in range(1, 13)}
    for finger in config["fingers"].values():
        segment = mul(root, q([0, 0, 1], finger["neutral_splay_deg"]))
        result[finger["proximal"]] = segment
        result[finger["distal"]] = segment.copy()
    return result


@pytest.mark.parametrize("hand", ["right", "left"])
@pytest.mark.parametrize("root", [q([1, 0, 0], 35), q([0, 1, 0], -25), q([1, -2, 3], 73)])
def test_physical_root_attitude_is_separate_from_local_finger_angles(hand, root):
    config = load_config()
    config["hand"] = hand
    model = HandModel(config)
    baseline = model.solve(orientations(config, I), set(range(1, 13)), set(range(1, 13)), True)
    result = model.solve(orientations(config, root), set(range(1, 13)), set(range(1, 13)), True)
    assert result["root_orientation_valid"] and result["palm_valid"]
    np.testing.assert_allclose(to_matrix(result["root_orientation_wxyz"]), to_matrix(root), atol=1e-12)
    assert result["root_translation_mm"] == [0, 0, 0]
    assert result["root_reference"] == "session_neutral"
    assert not result["root_heading_observable"]
    assert not result["root_fixed"] and result["root_translation_fixed"]
    np.testing.assert_allclose(result["palm_points"], baseline["palm_points"], atol=1e-12)
    for name, finger in result["fingers"].items():
        assert finger["valid"]
        assert finger["mcp_deg"] == pytest.approx(0, abs=1e-10)
        assert finger["splay_deg"] == pytest.approx(0, abs=1e-10)
        assert finger["ip_deg" if name == "thumb" else "pip_deg"] == pytest.approx(0, abs=1e-10)
        np.testing.assert_allclose(finger["points"], baseline["fingers"][name]["points"], atol=1e-10)
        # Renderer contract: apply root once to already mirrored local points.
        displayed = np.array([rotate(result["root_orientation_wxyz"], point) for point in finger["points"]])
        expected = np.array(baseline["fingers"][name]["points"]) @ to_matrix(root).T
        np.testing.assert_allclose(displayed, expected, atol=1e-10)


@pytest.mark.parametrize("condition", ["stale", "unmounted", "no_neutral", "invalid_quaternion"])
def test_invalid_root_is_not_the_last_good_attitude(condition):
    config = load_config()
    model = HandModel(config)
    values = orientations(config, q([1, 2, -3], 61))
    fresh = set(range(1, 13))
    axes = fresh.copy()
    assert model.solve(values, fresh, axes, True)["root_orientation_valid"]
    if condition == "stale":
        fresh.remove(12)
    elif condition == "unmounted":
        axes.remove(12)
        del values[12]
    elif condition == "invalid_quaternion":
        values[12] = np.array([np.nan, 0, 0, 0])
    result = model.solve(values, fresh, axes, condition != "no_neutral")
    assert not result["root_orientation_valid"]
    assert not result["palm_valid"]
    np.testing.assert_array_equal(result["root_orientation_wxyz"], I)
    assert result["root_translation_mm"] == [0, 0, 0]


@pytest.mark.parametrize("hand", ["right", "left"])
def test_demo_has_physical_palm_motion_and_keeps_articulation(hand):
    config = load_config()
    config["hand"] = hand
    zero = demo_snapshot(0, config)
    result = demo_snapshot(1.3, config)
    assert result["source"] == "demo"
    assert any("DEMO" in message for message in result["warnings"])
    root = result["model"]["root_orientation_wxyz"]
    assert result["model"]["root_orientation_valid"]
    assert not np.allclose(to_matrix(root), to_matrix(zero["model"]["root_orientation_wxyz"]))
    assert any(abs(finger["mcp_deg"]) > 10 for finger in result["model"]["fingers"].values())
    other = load_config()
    other["hand"] = "left" if hand == "right" else "right"
    np.testing.assert_allclose(to_matrix(root), to_matrix(demo_snapshot(1.3, other)["model"]["root_orientation_wxyz"]), atol=1e-12)


def response_after_straightening(gyro_ratio, move_seconds=.5):
    """Quantized 45-degree MCP / 60-degree PIP motion through the real engine."""
    config = load_config()
    engine = PoseEngine(config)
    engine.mounts = {node: I.copy() for node in range(1, 13)}
    proximal_node, distal_node = (config["fingers"]["index"][key] for key in ("proximal", "distal"))
    previous = {proximal_node: 0., distal_node: 0.}

    def feed(t, proximal=0., distal=0.):
        for node in range(1, 13):
            angle = proximal if node == proximal_node else distal if node == distal_node else 0.
            speed = (angle - previous[node]) * 120 if node in previous else 0.
            raw = (round(-speed * gyro_ratio / config["gyro_dps_per_lsb"]), 0, 0, 0,
                   round(-math.sin(math.radians(angle)) / config["accel_g_per_lsb"]),
                   round(math.cos(math.radians(angle)) / config["accel_g_per_lsb"]))
            engine.process(Sample(node, raw, 10 + t, 10 + t))
            if node in previous:
                previous[node] = angle

    for tick in range(600):
        feed(tick / 120)
    engine.zero = {node: state["quat"].copy() for node, state in engine.states.items()}
    engine.neutral_ready = True
    end = 2 * move_seconds + 3.
    observations = {}
    checks = {round((end + offset) * 120): offset for offset in (0., 1., 4.)}
    for tick in range(1, round((end + 4) * 120) + 1):
        t = tick / 120
        fraction = t / move_seconds if t <= move_seconds else 1. if t <= move_seconds + 3 else 1 - (t - move_seconds - 3) / move_seconds if t <= end else 0.
        feed(5 + t, 45 * fraction, 105 * fraction)
        if tick in checks:
            observations[checks[tick]] = engine.get_snapshot(now=15 + t)["model"]["fingers"]["index"]
    return observations


@pytest.mark.parametrize("move_seconds", [.5, 1., 2.])
def test_gyro_consistent_real_vqf_straightens_without_acc_time_constant_lag(move_seconds):
    result = response_after_straightening(1., move_seconds)
    for finger in result.values():
        assert finger["valid"]
        assert finger["mcp_deg"] == pytest.approx(0, abs=.05)
        assert finger["pip_deg"] == pytest.approx(0, abs=.05)


def test_wrong_gyro_scale_can_create_slow_return_despite_valid_packets():
    result = response_after_straightening(.25)
    # Diagnostic guard, not a claim that this was the hardware session's cause.
    # A unit mismatch creates long inclination-correction tails; tauAcc itself
    # does not delay a gyro-consistent trajectory by three seconds.
    assert 20 < result[0.]["mcp_deg"] < 23
    assert 28 < result[0.]["pip_deg"] < 32
    assert result[1.]["mcp_deg"] > 20
    assert result[4.]["mcp_deg"] > 9
