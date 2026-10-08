"""Anatomical visual cues: no hardware, calibration changes or pose resets."""
from copy import deepcopy
import math
import os
import sys
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import numpy as np
import pytest
from PyQt5 import QtWidgets

from vt_glove_pose.canvas import HandCanvas, anatomical_display_model
from vt_glove_pose.config import load_config
from vt_glove_pose.kinematics import HandModel
from vt_glove_pose.math3d import conj, from_axis_angle, mul
from vt_glove_pose.mesh import orientation_matrix
from vt_glove_pose.quick_hand import QuickHandEngine


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication(["surface-tests"])


def example(hand="right", flex=0, splay=0, thumb_neutral=-50):
    config = load_config()
    config["hand"] = hand
    config["fingers"]["thumb"]["neutral_splay_deg"] = thumb_neutral
    solver = HandModel(config)
    model = solver.solve({}, set(), set(), False)
    for name, finger in solver.fingers.items():
        model["fingers"][name]["points"] = solver._points(finger, flex, 0, splay, 0)
    return config, model


@pytest.mark.parametrize("hand,sign", [("right", -1), ("left", 1)])
@pytest.mark.parametrize("thumb_neutral", [-50, -35])
def test_correct_chirality_with_explicit_custom_template_and_no_source_mutation(hand, sign, thumb_neutral):
    config, model = example(hand, thumb_neutral=thumb_neutral)
    original, original_config = deepcopy(model), deepcopy(config)
    shown = anatomical_display_model(model, config)
    thumb = np.asarray(shown["fingers"]["thumb"]["points"])
    assert sign * thumb[0, 0] > 0
    assert sign * (thumb[1, 0] - thumb[0, 0]) > 0
    for name in model["fingers"]:
        before = np.asarray(model["fingers"][name]["points"])
        after = np.asarray(shown["fingers"][name]["points"])
        assert after[0, 0] == pytest.approx(-before[0, 0])
        np.testing.assert_allclose(np.linalg.norm(np.diff(before, axis=0), axis=1),
                                   np.linalg.norm(np.diff(after, axis=0), axis=1))
        for key, value in model["fingers"][name].items():
            if key != "points":
                assert shown["fingers"][name][key] == value
    np.testing.assert_allclose(np.asarray(shown["palm_points"])[:, 0], -np.asarray(model["palm_points"])[:, 0])
    assert model == original and config == original_config


@pytest.mark.parametrize("hand", ["right", "left"])
@pytest.mark.parametrize("splay", [-20, 20])
def test_finger_dynamic_splay_and_palmar_flexion_are_not_mirrored(hand, splay):
    config, model = example(hand, flex=35, splay=splay)
    shown = anatomical_display_model(model, config)
    for name in ("index", "middle", "ring", "little"):
        before = np.diff(model["fingers"][name]["points"], axis=0)
        after = np.diff(shown["fingers"][name]["points"], axis=0)
        np.testing.assert_allclose(after, before, atol=1e-12)
        assert after[0, 2] < 0  # Positive flexion goes toward palm, not nails.
    thumb = np.asarray(shown["fingers"]["thumb"]["points"])
    assert thumb[1, 2] < thumb[0, 2]


@pytest.mark.parametrize("hand", ["right", "left"])
@pytest.mark.parametrize("axis", [[0, 0, 1], [1, 0, 0], [1, 2, 3]])
def test_display_preserves_physical_root_rotation_not_mirrored_world(app, hand, axis):
    config, model = example(hand)
    q = from_axis_angle(axis, math.pi / 2)
    model.update(root_orientation_valid=True, root_orientation_wxyz=q.tolist())
    shown = anatomical_display_model(model, config)
    canvas = HandCanvas(config=config)
    canvas.set_model(model, stale=False)
    canvas.yaw = canvas.pitch = 0
    points = np.asarray(shown["fingers"]["thumb"]["points"])
    np.testing.assert_allclose(canvas.model_to_view(points), points @ orientation_matrix(q).T, atol=1e-12)
    assert shown["root_orientation_wxyz"] == model["root_orientation_wxyz"]
    if axis == [0, 0, 1]:
        # A true +90deg root rotates right thumb x<0 to y<0.
        assert canvas.model_to_view(points)[0, 1] * (-1 if hand == "right" else 1) > 0
    canvas.close()


@pytest.mark.parametrize("hand,word", [("right", "右手"), ("left", "左手")])
def test_surface_presets_and_identity_follow_drawn_camera_not_fixed_thumb_side(app, hand, word):
    config, model = example(hand)
    model.update(root_orientation_valid=True,
                 root_orientation_wxyz=from_axis_angle([1, 2, 3], 1.7).tolist())
    original = deepcopy(model)
    canvas = HandCanvas(config=config)
    canvas.set_model(model, stale=False)
    canvas.resize(700, 620)
    for face, label in (("dorsal", "手背朝向你"), ("palmar", "手心朝向你")):
        canvas.set_surface_view(face)
        assert canvas.visible_surface() == face
        assert word in canvas.view_identity() and label in canvas.view_identity()
        assert not canvas.grab().isNull()
        assert canvas.drawn_triangles > 100
        assert canvas.follow_root
        assert model == original
    canvas.set_root_follow(False)
    canvas.yaw, canvas.pitch = math.pi / 2, 0
    assert canvas.visible_surface() == "edge"
    assert "侧面视角" in canvas.view_identity()
    assert model == original
    canvas.close()


def test_unknown_template_and_invalid_root_never_claim_measured_hand_direction(app):
    config, model = example()
    canvas = HandCanvas()
    canvas.set_model(model, stale=False)
    assert anatomical_display_model(model, None) is model
    assert "显示模板未提供" in canvas.view_identity()
    canvas.set_geometry_config(config)
    assert "显示模板未提供" not in canvas.view_identity()
    assert "参考外形" in canvas.view_identity()
    del model["hand"]
    assert "手别未标注" in canvas.view_identity()
    canvas.set_model(model, stale=True)
    assert "历史画面" in canvas.view_identity()
    with pytest.raises(ValueError):
        canvas.set_surface_view("guess")
    canvas.close()


@pytest.mark.parametrize("root", [[0, 0, 0, 0], [float("nan"), 0, 0, 0], None])
def test_invalid_quaternion_fallback_has_reference_label(app, root):
    config, model = example()
    model.update(root_orientation_valid=True, root_orientation_wxyz=root)
    canvas = HandCanvas(config=config)
    canvas.set_model(model, stale=False)
    canvas.set_surface_view("dorsal")
    assert "参考外形" in canvas.view_identity()
    assert not canvas.grab().isNull()
    canvas.close()


def test_missing_thumb_template_heading_is_not_guessed():
    config, model = example()
    del config["fingers"]["thumb"]["neutral_splay_deg"]
    with pytest.raises(ValueError, match="Missing neutral splay"):
        anatomical_display_model(model, config)


@pytest.mark.parametrize("hand", ["right", "left"])
def test_direct_anatomical_model_is_not_corrected_twice(hand):
    config, legacy = example(hand, flex=25, splay=12)
    direct = anatomical_display_model(legacy, config)
    direct["coordinate_convention"] = "anatomical_direct_v1"
    original = deepcopy(direct)
    assert anatomical_display_model(direct, config) is direct
    assert direct == original
    # Unmarked historical snapshots still use their existing correction.
    assert anatomical_display_model(legacy, config) != legacy


def _ideal_quick_snapshot(monkeypatch, hand, common, *, thumb_mcp=0., thumb_ip=0., thumb_splay=0.):
    # Geometry-only test: supply exact sensor attitudes, never run a filter.
    monkeypatch.setitem(sys.modules, "vqf", SimpleNamespace(VQF=object))
    config = load_config()
    config["hand"] = hand
    original = deepcopy(config)
    engine = QuickHandEngine(config)
    engine.neutral_ready = True
    thumb = config["fingers"]["thumb"]
    sign = 1 if hand == "right" else -1
    for node in engine.required_nodes:
        is_thumb = node in (thumb["proximal"], thumb["distal"])
        # Actual right-thumb neutral heading is +50 degrees, independently
        # specified here rather than copied from the engine's model template.
        neutral = from_axis_angle([0, 0, 1], math.radians(sign * 50 if is_thumb else 0))
        heading = from_axis_angle([0, 0, 1], math.radians(sign * (50 + thumb_splay) if is_thumb else 0))
        bend = (thumb_mcp + (thumb_ip if node == thumb["distal"] else 0)) if is_thumb else 0
        segment = mul(heading, from_axis_angle([-1, 0, 0], math.radians(bend)))
        # Sensor axes coincide with local bone axes (M=I). Its attitude change
        # is R0^-1 G R_segment, not a common world quaternion for every sensor.
        delta = mul(conj(neutral), mul(common, segment))
        engine.zero[node] = np.array([1., 0., 0., 0.])
        engine.mounts[node] = np.array([1., 0., 0., 0.])
        engine.fusion.states[node] = dict(quat=delta, time=1.)
    snapshot = engine.get_snapshot(1.)
    assert config == original and engine.config == original
    return config, snapshot["model"]


@pytest.mark.parametrize("hand,side", [("right", -1), ("left", 1)])
@pytest.mark.parametrize("axis", [[1, 0, 0], [0, 1, 0], [1, 2, 3]])
def test_quick_physical_thumb_and_rigid_root_are_not_remirrored(app, monkeypatch, hand, side, axis):
    common = from_axis_angle(axis, math.radians(45))
    config, model = _ideal_quick_snapshot(monkeypatch, hand, common)
    assert model["coordinate_convention"] == "anatomical_direct_v1"
    assert anatomical_display_model(model, config) is model
    for name, finger in model["fingers"].items():
        assert finger["geometry_live"]
        assert finger["mcp_deg"] == pytest.approx(0., abs=1e-10)
        assert finger["splay_deg"] == pytest.approx(0., abs=1e-10)
        assert finger["ip_deg" if name == "thumb" else "pip_deg"] == pytest.approx(0., abs=1e-10)
    points = np.asarray(model["fingers"]["thumb"]["points"])
    direction = points[1] - points[0]
    assert side * points[0, 0] > 0
    assert math.degrees(math.atan2(-direction[0], direction[1])) == pytest.approx(-side * 50)
    # Wrist endpoint must be on the same radial side as the thumb base.
    assert side * model["palm_points"][0][0] > 0
    assert side * model["palm_points"][-2][0] < 0
    canvas = HandCanvas(config=config)
    canvas.set_model(model, stale=False)
    canvas.yaw = canvas.pitch = 0
    np.testing.assert_allclose(canvas.model_to_view(points), points @ orientation_matrix(common).T, atol=1e-10)
    assert not canvas.grab().isNull()
    assert canvas.drawn_triangles > 100
    assert canvas._display_model == model
    canvas.close()


@pytest.mark.parametrize("hand,sign", [("right", 1), ("left", -1)])
def test_quick_thumb_local_flexion_and_splay_match_drawn_segment(monkeypatch, hand, sign):
    common = from_axis_angle([1, 2, 3], .7)
    config, model = _ideal_quick_snapshot(monkeypatch, hand, common, thumb_mcp=25., thumb_ip=40., thumb_splay=12.)
    shown = anatomical_display_model(model, config)
    thumb = shown["fingers"]["thumb"]
    assert thumb["mcp_deg"] == pytest.approx(25.)
    assert thumb["ip_deg"] == pytest.approx(40.)
    assert thumb["splay_deg"] == pytest.approx(12.)
    actual = np.diff(thumb["points"], axis=0)
    actual /= np.linalg.norm(actual, axis=1)[:, None]
    for index, bend in enumerate((25., 65.)):
        physical = mul(from_axis_angle([0, 0, 1], math.radians(sign * 62)),
                       from_axis_angle([-1, 0, 0], math.radians(bend)))
        expected = orientation_matrix(physical) @ np.array([0., 1., 0.])
        np.testing.assert_allclose(actual[index], expected, atol=1e-10)
