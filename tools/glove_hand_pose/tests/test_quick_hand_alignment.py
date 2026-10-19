"""Physical-frame regressions: arbitrary IMU mounts, shared motion, real thumb."""
import math

import numpy as np
import pytest

from vt_glove_pose.config import gyro_scale, load_config
from vt_glove_pose.math3d import conj, from_axis_angle, mul, rotate, to_matrix
from vt_glove_pose.quick_hand import QuickHandEngine, _direction_registration
from vt_glove_pose.records import Sample


def _mounts(engine):
    # Distinct installation yaw AND tilt. These are sensor-to-hand at neutral,
    # not all conveniently aligned boards or a globally shared VQF yaw gauge.
    return {n: mul(from_axis_angle([0, 0, 1], -.9 + n * .19),
                   from_axis_angle([1, .4, 0], .1 + n * .027)) for n in engine.required_nodes}


def _frame(engine, mounts, stamp, angle=0., speed=0., omit_gyro=()):
    hand = from_axis_angle([-1, 0, 0], math.radians(angle))
    for node in sorted(engine.required_nodes):
        orientation = mul(hand, mounts[node])
        gyro = rotate(conj(orientation), [-speed, 0., 0.]) if node not in omit_gyro else np.zeros(3)
        acc = rotate(conj(orientation), [0., 0., 1.])
        raw = np.rint(np.r_[gyro / gyro_scale(engine.config, node), acc / engine.config["accel_g_per_lsb"]]).astype(int)
        engine.process(Sample(node, tuple(raw), stamp, 1700000000. + stamp))


def _aligned(hand="right", omit_gyro=()):
    config = load_config()
    config["hand"] = hand
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175}
    engine = QuickHandEngine(config)
    mounts = _mounts(engine)
    _frame(engine, mounts, 10.)
    engine.begin_reference(.3)
    for i in range(1, 38):
        _frame(engine, mounts, 10. + i / 120.)
    zeros = {n: value.copy() for n, value in engine.zero.items()}
    old_mounts = {n: value.copy() for n, value in engine.mounts.items()}
    start = engine.fusion.last_time
    engine.begin_frame_alignment(1.5)
    for i in range(1, 183):
        t = i / 120.
        angle = 15. * math.sin(2. * math.pi * t / 1.5)
        speed = 15. * 2. * math.pi / 1.5 * math.cos(2. * math.pi * t / 1.5)
        _frame(engine, mounts, start + t, angle, speed, omit_gyro=omit_gyro)
    assert engine.active is None
    for n in zeros:
        np.testing.assert_array_equal(engine.zero[n], zeros[n])
    return engine, mounts, old_mounts


def _set_physical_pose(engine, mounts, common, flexion=None):
    """Known exact sensor-relative rotations, retaining real VQF references."""
    flexion = {} if flexion is None else flexion
    for node, mounting in mounts.items():
        neutral_bone = engine._neutral_splay(node)
        local_bend = from_axis_angle([-1., 0., 0.], math.radians(flexion.get(node, 0.)))
        actual = mul(common, mul(neutral_bone, mul(local_bend, mul(conj(neutral_bone), mounting))))
        delta = mul(conj(mounting), actual)
        engine.fusion.states[node]["quat"] = mul(engine.zero[node], delta)


@pytest.mark.parametrize("hand", ["right", "left"])
def test_registration_recovers_arbitrary_mounts_and_common_rotation_cancels(hand):
    engine, mounts, _ = _aligned(hand)
    palm = engine.config["palm_node"]
    for node in engine.required_nodes:
        assert engine.axes_source[node] == "common_frame_alignment_estimate"
        expected = mul(conj(engine._neutral_splay(node)), mounts[node])
        np.testing.assert_allclose(to_matrix(engine.mounts[node]), to_matrix(expected), atol=.002)
    for axis in ([1, 0, 0], [0, 1, 0], [.3, .4, .6]):
        _set_physical_pose(engine, mounts, from_axis_angle(axis, math.radians(45)))
        snapshot = engine.get_snapshot(engine.fusion.last_time)
        assert snapshot["model"]["coordinate_convention"] == "anatomical_direct_v1"
        for name, item in snapshot["model"]["fingers"].items():
            assert item["common_frame_aligned"]
            assert item["mcp_deg"] == pytest.approx(0., abs=.1)
            assert item["splay_deg"] == pytest.approx(0., abs=.1)
            assert item["ip_deg" if name == "thumb" else "pip_deg"] == pytest.approx(0., abs=.1)


def test_real_thumb_and_all_other_fingers_bend_after_shared_registration():
    engine, mounts, _ = _aligned()
    assert engine.model_config["fingers"]["thumb"]["neutral_splay_deg"] == 50.
    assert engine.model_config["fingers"]["thumb"]["base_mm"][0] < 0.
    common = from_axis_angle([.2, .3, .4], .6)
    for name, mapping in engine.config["fingers"].items():
        _set_physical_pose(engine, mounts, common, {mapping["proximal"]: 25., mapping["distal"]: 65.})
        fingers = engine.get_snapshot(engine.fusion.last_time)["model"]["fingers"]
        item = fingers[name]
        assert item["mcp_deg"] == pytest.approx(25., abs=.1)
        assert item["splay_deg"] == pytest.approx(0., abs=.1)
        assert item["ip_deg" if name == "thumb" else "pip_deg"] == pytest.approx(40., abs=.1)
        for other, unchanged in fingers.items():
            if other != name:
                assert unchanged["mcp_deg"] == pytest.approx(0., abs=.1)


def test_missing_motion_one_node_keeps_its_previous_mount_and_is_not_called_aligned():
    engine, mounts, previous = _aligned(omit_gyro=(3,))
    np.testing.assert_array_equal(engine.mounts[3], previous[3])
    result = engine.get_snapshot(engine.fusion.last_time)
    assert not result["nodes"]["3"]["common_frame_aligned"]
    assert not result["model"]["fingers"]["index"]["common_frame_aligned"]
    assert result["model"]["fingers"]["middle"]["common_frame_aligned"]
    assert any("节点 3 未完成共同坐标对齐" in message for message in result["warnings"])


def test_direction_fit_ignores_constant_gyro_magnitude_scale_difference():
    rotation = from_axis_angle([.2, .3, .7], .9)
    pairs = []
    for i in range(100):
        gp = np.array([20. * math.cos(i / 20.), 12. * math.sin(i / 20.), 0.])
        ap = np.array([0., 0., 1.])
        pairs.append(((i / 120., .5 * rotate(conj(rotation), gp), rotate(conj(rotation), ap)),
                      (i / 120., gp, ap)))
    result, quality = _direction_registration(pairs, np.zeros(3), np.zeros(3), .00875, .00875)
    np.testing.assert_allclose(to_matrix(result), to_matrix(rotation), atol=1e-10)
    assert quality["direction_residual_p95_deg"] < 1e-5


def test_degenerate_collinear_motion_and_gravity_are_not_installed():
    pairs = [((i / 120., np.array([0., 0., 20.]), np.array([0., 0., 1.])),
              (i / 120., np.array([0., 0., 20.]), np.array([0., 0., 1.]))) for i in range(30)]
    with pytest.raises(ValueError, match="退化"):
        _direction_registration(pairs, np.zeros(3), np.zeros(3), .00875, .00875)


def test_many_static_gravity_samples_do_not_hide_large_active_gyro_residual():
    pairs = []
    for i in range(1000):
        # The same sensor gyro direction cannot simultaneously rotate into
        # both +Y and -Y. Gravity, however, agrees in every stationary row.
        gs = np.array([20., 0., 0.]) if i < 20 else np.zeros(3)
        gp = np.array([0., 20. if i % 2 else -20., 0.]) if i < 20 else np.zeros(3)
        gravity = np.array([0., 0., 1.])
        pairs.append(((i / 120., gs, gravity), (i / 120., gp, gravity)))
    _, quality = _direction_registration(pairs, np.zeros(3), np.zeros(3), .00875, .00875)
    assert quality["direction_residual_p95_deg"] < 1e-5
    assert quality["gravity_direction_residual_median_deg"] < 1e-5
    assert quality["gravity_direction_residual_p95_deg"] < 1e-5
    assert quality["gyro_direction_residual_median_deg"] > 60.
    assert quality["gyro_direction_residual_p95_deg"] > 60.


def test_alignment_warning_uses_active_gyro_not_mixed_residual(monkeypatch):
    from vt_glove_pose import quick_hand
    original = quick_hand._direction_registration

    def reported_mismatch(*args, **kwargs):
        rotation, quality = original(*args, **kwargs)
        quality["direction_residual_p95_deg"] = 0.
        quality["gyro_direction_residual_p95_deg"] = 90.
        return rotation, quality

    monkeypatch.setattr(quick_hand, "_direction_registration", reported_mismatch)
    engine, _, _ = _aligned()
    snapshot = engine.get_snapshot(engine.fusion.last_time)
    assert any("活动陀螺方向残差较大" in message for message in snapshot["warnings"])
    # This change adds truthful diagnostics, not another blocking gate.
    assert all(f["common_frame_aligned"] for f in snapshot["model"]["fingers"].values())


def test_reference_is_required_and_export_declares_new_coordinate_convention():
    engine = QuickHandEngine(load_config())
    with pytest.raises(ValueError, match="张手参考"):
        engine.begin_frame_alignment()
    aligned, _, _ = _aligned()
    profile = aligned.export_profile()
    assert profile["coordinate_convention"] == "anatomical_direct_v1"
    profile["coordinate_convention"] = "legacy_mirrored"
    with pytest.raises(ValueError):
        aligned.load_profile(profile)
