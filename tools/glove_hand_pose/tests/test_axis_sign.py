"""First instructed flexion must not be replaced by a later extension sign."""

from copy import deepcopy
import json

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.math3d import from_axis_angle, rotate
from vt_glove_pose.records import Sample


I = np.array([1., 0., 0., 0.])
INDEX_NODES = tuple(load_config()["fingers"]["index"][key] for key in ("proximal", "distal"))
FLEX_AXES = dict(zip(INDEX_NODES, (np.array([-1., 0., 0.]), np.array([0., -1., 0.]))))


def axis_engine(first_half_seconds=.3, prefix=None):
    engine = PoseEngine(load_config())
    for node in range(1, 13):
        engine.process(Sample(node, (0, 0, 0, 0, 0, 8197), 10., 10.))
    engine.zero = {node: I.copy() for node in range(1, 13)}
    engine.neutral_acc = {node: np.array([0., 0., 1.]) for node in range(1, 13)}
    engine.neutral_ready = True
    engine.mount_confirmed = True
    engine.begin_axis("index", duration=8)
    for node in (*INDEX_NODES, engine.config["palm_node"]):
        rows = []
        for tick in range(960):
            seconds = tick / 120
            sign = 1 if int((seconds + 1e-10) / first_half_seconds) % 2 == 0 else -1
            gyro = FLEX_AXES[node] * sign * 60 if node in FLEX_AXES else np.zeros(3)
            if prefix is not None:
                gyro = prefix(node, tick, gyro)
            rows.append((10 + seconds, gyro, np.array([0., 0., 1.]), I.copy()))
        engine.active["rows"][node] = rows
    return engine


@pytest.mark.parametrize("first_half_seconds", [.3, 1.])
def test_first_flexion_before_trim_still_determines_axis_sign(first_half_seconds):
    engine = axis_engine(first_half_seconds)
    engine._finish_calibration()
    assert "功能轴已保存" in engine.message
    for node, physical_flex in FLEX_AXES.items():
        np.testing.assert_allclose(rotate(engine.mounts[node], physical_flex), [-1, 0, 0], atol=1e-12)
        assert engine.axes_quality[node]["linearity"] == pytest.approx(1.)
        assert engine.axis_sign_versions[node] == 2
        assert engine.axes_quality[node]["axis_sign_algorithm_version"] == 2


def test_isolated_tiny_prefix_noise_cannot_flip_sign():
    def noise(node, tick, gyro):
        if node not in FLEX_AXES:
            return gyro
        if tick == 0:
            return -FLEX_AXES[node] * 8  # One wrong-sign sample, <0.1 degree.
        if tick == 1:
            return np.zeros(3)
        if tick == 2:
            return np.array([0., 0., 8.])  # One off-axis disturbance.
        if tick == 3:
            return np.zeros(3)
        return gyro

    engine = axis_engine(prefix=noise)
    engine._finish_calibration()
    assert "功能轴已保存" in engine.message
    for node, axis in FLEX_AXES.items():
        np.testing.assert_allclose(rotate(engine.mounts[node], axis), [-1, 0, 0], atol=1e-12)


@pytest.mark.parametrize("failure", ["short_first", "off_axis_first"])
def test_ambiguous_second_sensor_preserves_both_previous_axes_transactionally(failure):
    def prefix(node, tick, gyro):
        if node != INDEX_NODES[1]:
            return gyro
        if failure == "short_first" and tick < 3:
            return FLEX_AXES[node] * 60
        if failure == "short_first" and tick < 6:
            return np.zeros(3)
        if failure == "off_axis_first" and tick < 12:
            return np.array([60., 0., 0.])
        return gyro

    engine = axis_engine(prefix=prefix)
    engine.mounts = {INDEX_NODES[0]: from_axis_angle([0, 0, 1], .3), INDEX_NODES[1]: I.copy()}
    engine.axes_quality = {node: {"previous": True} for node in INDEX_NODES}
    engine.axis_sign_versions = {node: 1 for node in INDEX_NODES}
    previous = deepcopy((engine.mounts, engine.axes_quality, engine.axis_sign_versions))
    engine._finish_calibration()
    assert "第一次动作太短或方向不明确" in engine.message
    for node, mount in previous[0].items():
        np.testing.assert_array_equal(engine.mounts[node], mount)
    assert engine.axes_quality == previous[1]
    assert engine.axis_sign_versions == previous[2]


def test_empty_or_quiet_first_motion_has_no_guessed_sign():
    engine = axis_engine()
    capture = engine.active
    proximal = INDEX_NODES[0]
    capture["rows"][proximal] = []
    with pytest.raises(ValueError, match="第一次屈曲"):
        engine._first_motion_sign(capture, proximal, np.array([1., 0., 0.]))
    capture["rows"][proximal] = [(10 + tick / 120, np.zeros(3), np.array([0., 0., 1.]), I.copy()) for tick in range(960)]
    with pytest.raises(ValueError, match="第一次屈曲"):
        engine._first_motion_sign(capture, proximal, np.array([1., 0., 0.]))


def test_axis_sign_version_roundtrip_and_legacy_resave_stays_unverified(tmp_path):
    engine = axis_engine()
    engine._finish_calibration()
    current = tmp_path / "current.json"
    engine.save_calibration(current)
    saved = json.loads(current.read_text())
    assert saved["axis_sign_algorithm_version"] == 2
    assert saved["axis_sign_versions"] == {str(node): 2 for node in INDEX_NODES}
    restored = PoseEngine(load_config())
    restored.load_calibration(current)
    assert restored.axis_sign_versions == {node: 2 for node in INDEX_NODES}
    assert "方向反转" not in restored.message

    del saved["axis_sign_algorithm_version"]
    del saved["axis_sign_versions"]
    legacy = tmp_path / "legacy.json"
    legacy.write_text(json.dumps(saved))
    restored.load_calibration(legacy)
    assert restored.axis_sign_versions == {node: 0 for node in INDEX_NODES}
    assert "方向反转" in restored.message
    assert any("旧版" in warning for warning in restored.get_snapshot(now=10)["warnings"])
    # Simulate the required new session neutral: it must not upgrade axis provenance.
    restored.neutral_ready = True
    resaved = tmp_path / "legacy-resaved.json"
    restored.save_calibration(resaved)
    assert json.loads(resaved.read_text())["axis_sign_versions"] == {str(node): 0 for node in INDEX_NODES}
