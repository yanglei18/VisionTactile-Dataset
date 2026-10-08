"""Focused no-hardware checks for the explicitly estimated whole-hand mode."""
import math
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from vt_glove_pose.config import gyro_scale, load_config
from vt_glove_pose.math3d import conj, from_axis_angle, mul, normalize, rotate
from vt_glove_pose.quick_hand import QuickHandEngine
from vt_glove_pose.records import Sample


class IntegratingFilter:
    def __init__(self, dt, **kwargs):
        self.dt = dt
        self.bias = np.zeros(3)
        self.quat = np.array([1., 0., 0., 0.])

    def setBiasEstimate(self, bias):
        self.bias = np.asarray(bias)

    def update(self, gyro, acceleration):
        omega = gyro - self.bias
        magnitude = np.linalg.norm(omega)
        if magnitude:
            self.quat = normalize(mul(self.quat, from_axis_angle(omega, magnitude * self.dt)))

    def getQuat6D(self):
        return self.quat.copy()


@pytest.fixture
def engine(monkeypatch):
    monkeypatch.setitem(sys.modules, "vqf", SimpleNamespace(VQF=IntegratingFilter))
    return QuickHandEngine(load_config())


def feed(engine, stamp, *, omit=(), gyro=None, acceleration=None):
    for node in sorted(engine.required_nodes):
        if node in omit:
            continue
        g = np.zeros(3) if gyro is None else np.asarray(gyro(node))
        a = np.array([0., 0., 1.]) if acceleration is None else np.asarray(acceleration(node))
        raw = np.rint(np.concatenate((g / gyro_scale(engine.config, node), a / engine.config["accel_g_per_lsb"]))).astype(int)
        engine.process(Sample(node, tuple(raw), stamp, 1700000000. + stamp))


def reference(engine):
    feed(engine, 10.)
    engine.begin_reference(duration=.25)
    for i in range(1, 32):
        feed(engine, 10. + i / 120.)
    assert engine.neutral_ready
    return engine.fusion.last_time


def test_reference_outputs_all_five_fingers_without_auxiliary_11(engine):
    now = reference(engine)
    snapshot = engine.get_snapshot(now)
    assert snapshot["quick_mode"]
    assert snapshot["calibration"]["neutral_ready"]
    assert not snapshot["nodes"]["11"]["fresh"]
    assert snapshot["model"]["root_orientation_valid"]
    assert len(snapshot["model"]["fingers"]) == 5
    for finger in snapshot["model"]["fingers"].values():
        assert finger["geometry_live"]
        assert finger["quality"] == "quick_estimate"
        assert not finger["axes_calibrated"]
        assert set(finger["axes_source"].values()) == {"gravity_board_axis_assumption"}
    assert engine.fusion.algorithm_mode == "legacy_quaternion_v1"
    assert engine.fusion.thumb_estimator is None


def test_no_automatic_reference_and_no_fabricated_missing_data(engine):
    for i in range(40):
        feed(engine, 10. + i / 120., omit=(6,))
    assert not engine.get_snapshot(engine.fusion.last_time)["calibration"]["neutral_ready"]
    with pytest.raises(ValueError, match="新鲜"):
        engine.begin_reference()
    assert not engine.zero


@pytest.mark.parametrize("finger_name", ["thumb", "index", "middle", "ring", "little"])
def test_each_finger_flexes_and_extends_in_assumed_mount_frame(engine, finger_name):
    now = reference(engine)
    mapping = engine.config["fingers"][finger_name]
    for key, angle in (("proximal", 25.), ("distal", 65.)):
        node = mapping[key]
        engine.fusion.states[node]["quat"] = mul(engine.zero[node], from_axis_angle([-1, 0, 0], math.radians(angle)))
    item = engine.get_snapshot(now)["model"]["fingers"][finger_name]
    assert item["mcp_deg"] == pytest.approx(25.)
    assert item["ip_deg" if finger_name == "thumb" else "pip_deg"] == pytest.approx(40.)
    for node in (mapping["proximal"], mapping["distal"]):
        engine.fusion.states[node]["quat"] = engine.zero[node].copy()
    item = engine.get_snapshot(now)["model"]["fingers"][finger_name]
    assert item["mcp_deg"] == pytest.approx(0.)
    assert item["ip_deg" if finger_name == "thumb" else "pip_deg"] == pytest.approx(0.)


def test_common_rotation_cancels_when_assumed_mounts_match_actual_mounts(engine):
    """Not a claim that a single reference solves arbitrary mounting heading."""
    now = reference(engine)
    common = from_axis_angle([.3, .5, .2], .7)
    neutral = {engine.config["palm_node"]: np.array([1., 0., 0., 0.])}
    for finger in engine.model_config["fingers"].values():
        q = from_axis_angle([0, 0, 1], math.radians(finger["neutral_splay_deg"]))
        neutral[finger["proximal"]] = neutral[finger["distal"]] = q
    for node, q in neutral.items():
        # Sensor-body rotation from the physical neutral orientation q.
        delta = mul(conj(q), mul(common, q))
        engine.fusion.states[node]["quat"] = mul(engine.zero[node], delta)
    model = engine.get_snapshot(now)["model"]
    np.testing.assert_allclose(model["root_orientation_wxyz"], common, atol=1e-8)
    for name, item in model["fingers"].items():
        assert item["mcp_deg"] == pytest.approx(0., abs=1e-8)
        assert item["splay_deg"] == pytest.approx(0., abs=1e-8)
        assert item["ip_deg" if name == "thumb" else "pip_deg"] == pytest.approx(0., abs=1e-8)


def test_one_finger_missing_only_grays_that_finger_and_reconnect_does_not_zero(engine):
    now = reference(engine)
    for i in range(1, 80):
        feed(engine, now + i / 120., omit=(3,))
    stamp = engine.fusion.last_time
    model = engine.get_snapshot(stamp)["model"]
    assert not model["fingers"]["index"]["geometry_live"]
    assert all(model["fingers"][name]["geometry_live"] for name in ("thumb", "middle", "ring", "little"))
    feed(engine, stamp + 1 / 120.)
    snapshot = engine.get_snapshot(engine.fusion.last_time)
    assert snapshot["calibration"]["reference_missing_nodes"] == [3]
    assert snapshot["calibration"]["neutral_ready"]
    assert not snapshot["model"]["fingers"]["index"]["geometry_live"]
    assert snapshot["model"]["fingers"]["middle"]["geometry_live"]


def test_learning_retains_weak_nodes_and_does_not_zero_current_fist(engine):
    now = reference(engine)
    zeros = {n: q.copy() for n, q in engine.zero.items()}
    engine.begin_motion_learning(duration=.5)
    # Only index moves. All other fingers remain valid with provisional mounts.
    for i in range(1, 63):
        feed(engine, now + i / 120., gyro=lambda n: [-35., 0., 0.] if n in (3, 4) else [0., 0., 0.])
    assert engine.active is None
    assert engine.axes_source[3] == engine.axes_source[4] == "motion_pca_estimate"
    assert engine.axes_source[1] == "gravity_board_axis_assumption"
    for n in zeros:
        np.testing.assert_array_equal(zeros[n], engine.zero[n])
    result = engine.get_snapshot(engine.fusion.last_time)
    assert result["model"]["fingers"]["index"]["mcp_deg"] > 10
    assert all(f["geometry_live"] for f in result["model"]["fingers"].values())
    # Merely querying or continuing stationary data does not pull this to zero.
    angle = result["model"]["fingers"]["index"]["mcp_deg"]
    for i in range(1, 30):
        feed(engine, engine.fusion.last_time + 1 / 120.)
    assert engine.get_snapshot(engine.fusion.last_time)["model"]["fingers"]["index"]["mcp_deg"] == pytest.approx(angle)


def test_mild_reference_motion_warns_without_rejecting_or_treating_it_as_bias(engine):
    feed(engine, 10.)
    engine.begin_reference(duration=.25)
    for i in range(1, 32):
        feed(engine, 10. + i / 120., gyro=lambda n: [6., 0., 0.] if n == 5 else [0., 0., 0.])
    snapshot = engine.get_snapshot(engine.fusion.last_time)
    assert snapshot["calibration"]["neutral_ready"]
    assert any("节点 5" in warning and "有动作" in warning for warning in snapshot["warnings"])
    np.testing.assert_array_equal(engine.fusion.bias[5], [0., 0., 0.])


def test_learning_sign_ignores_a_single_opposite_spike(engine):
    now = reference(engine)
    engine.begin_motion_learning(duration=.5)
    for i in range(1, 63):
        speed = 50. if i == 1 else 0. if i < 15 else -35.
        feed(engine, now + i / 120., gyro=lambda n: [speed, 0., 0.] if n in (3, 4) else [0., 0., 0.])
    assert engine.axes_source[3] == "motion_pca_estimate"
    assert engine.get_snapshot(engine.fusion.last_time)["model"]["fingers"]["index"]["mcp_deg"] > 10.


def test_cancel_and_stale_snapshot_never_advance_filters_or_replace_zero(engine):
    now = reference(engine)
    zero = engine.zero[1].copy()
    engine.begin_reference()
    engine.cancel_calibration()
    assert engine.active is None
    np.testing.assert_array_equal(engine.zero[1], zero)
    count = engine.fusion.states[1]["count"]
    snapshot = engine.get_snapshot(now + 1.)
    assert not snapshot["model"]["root_orientation_valid"]
    assert not any(f["geometry_live"] for f in snapshot["model"]["fingers"].values())
    assert engine.fusion.states[1]["count"] == count


def test_real_vqf_finger_motion_with_per_node_scales():
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175}
    engine = QuickHandEngine(config)
    now = reference(engine)
    for i in range(1, 61):
        angle = math.radians(40. * i / 120.)
        q = from_axis_angle([-1., 0., 0.], angle)
        feed(engine, now + i / 120.,
             gyro=lambda n: [-40., 0., 0.] if n in (5, 6) else [0., 0., 0.],
             acceleration=lambda n: rotate(conj(q), [0., 0., 1.]) if n in (5, 6) else [0., 0., 1.])
    snapshot = engine.get_snapshot(engine.fusion.last_time)
    thumb = snapshot["model"]["fingers"]["thumb"]
    assert thumb["mcp_deg"] == pytest.approx(20., abs=1.)
    assert thumb["ip_deg"] == pytest.approx(0., abs=1.)
    assert snapshot["model"]["fingers"]["middle"]["mcp_deg"] == pytest.approx(0., abs=.1)
