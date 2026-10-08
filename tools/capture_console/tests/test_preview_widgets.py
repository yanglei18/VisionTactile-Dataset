"""Offscreen PySide6 previews: no ROS, serial devices, licensed assets or GPU."""

import base64
import json
import os
from pathlib import Path
import subprocess
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import pytest

pytest.importorskip("PySide6")
from PySide6 import QtCore, QtGui, QtWidgets, QtTest

from vt_capture_console.ui.widgets import PreviewWidget
from vt_capture_console.ui.widgets import preview as module


@pytest.fixture(scope="module")
def application():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def widgets(application):
    values = []

    def create(kind):
        widget = PreviewWidget(kind)
        widget.resize(360, 280)
        values.append(widget)
        return widget

    yield create
    for widget in values:
        widget.close()
        widget.deleteLater()
    application.processEvents()


def device(kind, preview=None, **changes):
    result = {"kind": kind, "id": kind + ":right", "label": "Right preview", "status": "online",
              "side": "right", "preview": preview or {}}
    result.update(changes)
    return result


def render(widget):
    image = QtGui.QImage(widget.size(), QtGui.QImage.Format.Format_ARGB32)
    image.fill(QtGui.QColor("#FFFFFF"))
    widget.render(image)
    return image


def pixels(image):
    return bytes(image.constBits())


def jpeg_preview(width=64, height=36):
    image = QtGui.QImage(width, height, QtGui.QImage.Format.Format_RGB32)
    image.fill(QtGui.QColor("#3A7BC2"))
    buffer = QtCore.QBuffer()
    assert buffer.open(QtCore.QIODevice.OpenModeFlag.WriteOnly)
    assert image.save(buffer, "JPEG")
    return {"format": "jpeg", "data_b64": base64.b64encode(bytes(buffer.data())).decode("ascii"),
            "width": width, "height": height}


def joints():
    result = [[0., 0., 0.]]
    for finger in range(5):
        for joint in range(4):
            result.append([(finger - 2) * .025, .035 + .027 * joint, .004 * joint])
    return result


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
def test_unknown_placeholder_and_explicit_demo_paint_without_hardware(widgets, kind):
    widget = widgets(kind)
    widget.set_device(device(kind, status="unknown"))
    empty = render(widget)
    assert widget._frame is None
    widget.set_device(device(kind, {"demo": True, "phase": .4}))
    demo = render(widget)
    assert widget._mode == "demo" and widget._frame is not None
    assert pixels(empty) != pixels(demo)
    widget.set_device(device(kind, {"demo": True, "phase": 1.7}))
    assert pixels(demo) != pixels(render(widget))


def test_camera_real_jpeg_and_last_valid_image_survive_empty_or_bad_preview(widgets):
    widget = widgets("camera")
    payload = jpeg_preview()
    widget.set_device(device("camera", payload))
    original = widget._image
    assert original.size() == QtCore.QSize(64, 36)
    assert not widget._held and widget._mode == "live"
    assert not render(widget).isNull()
    widget.set_device(device("camera", status="stale"))
    assert widget._image is original and widget._held
    widget.set_device(device("camera", {**payload, "data_b64": "%%% invalid"}))
    assert widget._image is original and widget._held and widget._preview_error
    assert not render(widget).isNull()
    widget.set_device(device("camera", payload))
    assert widget._image is original and not widget._held and not widget._preview_error


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
@pytest.mark.parametrize("status", ["stale", "offline"])
def test_unhealthy_demo_updates_never_animate_or_replace_last_good_frame(widgets, kind, status):
    widget = widgets(kind)
    widget.set_device(device(kind, {"demo": True, "phase": .2}))
    original = widget._frame
    widget.set_device(device(kind, {"demo": True, "phase": 2.4}, status=status))
    assert widget._frame is original and widget._held
    assert widget._mode == "demo"
    before = render(widget)
    widget.set_device(device(kind, {"demo": True, "phase": 3.1}, status=status))
    assert widget._frame is original
    assert pixels(render(widget)) == pixels(before)
    widget.set_device(device(kind, {"demo": True, "phase": 3.1}, status="online"))
    assert widget._frame is not original and not widget._held


def test_real_stale_preview_cannot_overwrite_last_good_camera(widgets):
    widget = widgets("camera")
    widget.set_device(device("camera", jpeg_preview()))
    original_image, original_frame = widget._image, widget._frame
    widget.set_device(device("camera", jpeg_preview(48, 48), status="stale"))
    assert widget._image is original_image and widget._frame is original_frame and widget._held


@pytest.mark.parametrize("change", [
    {"width": 4097}, {"height": 0}, {"width": True}, {"width": 65},
    {"format": "png"}, {"data_b64": "YWJj"}, {"data_b64": "\ud800"},
])
def test_camera_malformed_or_mismatched_inputs_fail_closed_without_exception(widgets, change):
    widget = widgets("camera")
    widget.set_device(device("camera", {**jpeg_preview(), **change}))
    assert widget._frame is None and widget._preview_error
    render(widget)


def test_camera_checks_base64_and_decoded_dimensions_before_allocating(widgets, monkeypatch):
    widget = widgets("camera")
    payload = jpeg_preview()
    monkeypatch.setattr(module, "MAX_BASE64_BYTES", 12)
    widget.set_device(device("camera", payload))
    assert "大小限制" in widget._preview_error
    monkeypatch.setattr(module, "MAX_BASE64_BYTES", 8_000_000)
    monkeypatch.setattr(module, "MAX_IMAGE_PIXELS", 100)
    widget.set_device(device("camera", payload))
    assert "尺寸限制" in widget._preview_error
    assert widget._image is None


def test_tactile_real_20x20_preserves_row_major_values_valid_zero_and_missing_cells(widgets):
    widget = widgets("tactile")
    widget.set_tactile_view("matrix")
    values, mask = list(range(400)), [True] * 400
    mask[1] = False
    payload = {"rows": 20, "cols": 20, "values": values, "valid_mask": mask, "levels": [0, 399]}
    widget.set_device(device("tactile", payload))
    assert widget._frame["values"] == tuple(range(400))
    assert widget._frame["valid_mask"][0] and not widget._frame["valid_mask"][1]
    assert len(widget._cell_colors) == 400
    image = render(widget)
    matrix = widget._heatmap_rect(widget._canvas_rect())
    size = matrix.width() / 20

    def color(row, col):
        return image.pixelColor(round(matrix.left() + (col + .5) * size), round(matrix.top() + (row + .5) * size))

    assert color(0, 1) == module._INVALID
    assert color(0, 0) != color(19, 19)
    assert color(0, 0) == widget._cell_colors[0]
    assert color(19, 19) == widget._cell_colors[-1]
    values[0] = 999
    assert widget._frame["values"][0] == 0  # Widget owns immutable measurement copies.


@pytest.mark.parametrize("preview", [
    {"rows": 80, "cols": 80, "values": [0] * 400, "valid_mask": [True] * 400},
    {"values": [0] * 399, "valid_mask": [True] * 400},
    {"values": [float("nan")] * 400, "valid_mask": [True] * 400},
    {"values": [1] * 400, "valid_mask": [1] * 400},
    {"values": [1] * 400, "valid_mask": [True] * 400, "levels": [1, 1]},
])
def test_tactile_invalid_shape_and_nonfinite_measurements_do_not_crash(widgets, preview):
    widget = widgets("tactile")
    widget.set_device(device("tactile", preview))
    assert widget._frame is None and widget._preview_error
    render(widget)


def test_tactile_stale_and_all_invalid_retain_last_frame_with_history_flag(widgets):
    widget = widgets("tactile")
    payload = {"values": [100.] * 400, "valid_mask": [True] * 400}
    widget.set_device(device("tactile", payload))
    original = widget._frame
    widget.set_device(device("tactile", {**payload, "valid_mask": [False] * 400}))
    assert widget._frame is original and widget._held
    widget.set_device(device("tactile", status="offline"))
    assert widget._frame is original and widget._held
    render(widget)


def test_real_21_joint_skeleton_and_invalid_pose_hold_previous_measurements(widgets):
    widget = widgets("glove")
    payload = {"joints": joints(), "valid": True}
    widget.set_device(device("glove", payload))
    assert len(widget._frame["joints"]) == 21
    assert widget._mode == "live" and not widget._held
    rendered = render(widget)
    assert len(set(pixels(rendered))) > 30
    original = widget._frame
    widget.set_device(device("glove", {"joints": joints(), "valid": False}))
    assert widget._held and widget._frame is original
    widget.set_device(device("glove", {"joints": [[0, 0, 0]] * 20, "valid": True}))
    assert widget._frame is original and widget._preview_error


def test_tracker_orientation_and_bounded_measured_trajectory(widgets):
    widget = widgets("tracker")
    for index in range(200):
        widget.set_device(device("tracker", {"position": [index * .001, 0., .01],
            "orientation": [0., 0., 0., 1.], "valid": True}))
    assert len(widget._trail) == module.MAX_TRAIL_POINTS
    before = render(widget)
    widget.set_device(device("tracker", {"position": [.199, 0., .01],
        "orientation": [0., 0., .70710678, .70710678], "valid": True}))
    assert pixels(before) != pixels(render(widget))
    original = widget._frame
    widget.set_device(device("tracker", {"position": [0., 0., 0.], "orientation": [0., 0., 0., 0.], "valid": True}))
    assert widget._frame is original and widget._preview_error


@pytest.mark.parametrize("kind", ["tracker", "glove"])
def test_pose_drag_changes_only_view_and_double_click_resets(widgets, kind):
    widget = widgets(kind)
    widget.set_device(device(kind, {"demo": True, "phase": .3}))
    original = widget._frame
    angles = (widget._yaw, widget._pitch)
    widget.show()
    QtTest.QTest.mousePress(widget, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(100, 100))
    QtTest.QTest.mouseMove(widget, QtCore.QPoint(140, 120))
    QtTest.QTest.mouseRelease(widget, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(140, 120))
    assert (widget._yaw, widget._pitch) != angles
    assert widget._frame is original
    QtTest.QTest.mouseDClick(widget, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(140, 120))
    assert (widget._yaw, widget._pitch) == angles


def test_switching_hand_or_demo_to_live_clears_old_identity_but_not_ordinary_empty_update(widgets):
    widget = widgets("glove")
    widget.set_device(device("glove", {"demo": True, "phase": 0.}))
    previous = widget._frame
    widget.set_device(device("glove", status="offline"))
    assert widget._frame is previous and widget._mode == "demo" and widget._held
    widget.set_device(device("glove", {"valid": False}))
    assert widget._frame is None and widget._mode == "live"
    widget.set_device(device("glove", {"joints": joints(), "valid": True}))
    widget.set_device(device("glove", id="glove:left", side="left"))
    assert widget._frame is None


def test_replay_uses_supplied_measurements_and_never_synthetic_motion(widgets):
    widget = widgets("tracker")
    payload = {"replay": True, "position": [1., 2., 3.],
               "orientation": [0., 0., 0., 1.], "valid": True, "phase": 2.3}
    widget.set_device(device("tracker", payload))
    assert widget._mode == "replay"
    assert widget._frame["position"] == (1., 2., 3.)
    frame = widget._frame
    widget.set_device(device("tracker", status="stale"))
    assert widget._held and widget._mode == "replay" and widget._frame is frame
    render(widget)
    widget.set_device(device("tracker", {"valid": False}))
    assert widget._mode == "live" and widget._frame is None


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
def test_compact_silver_preview_keeps_mode_and_held_labels(widgets, kind, monkeypatch):
    widget = widgets(kind)
    widget.resize(310, 166)
    labels = []
    original = PreviewWidget._text

    def capture(painter, rect, text, **kwargs):
        labels.append(text)
        return original(painter, rect, text, **kwargs)

    monkeypatch.setattr(PreviewWidget, "_text", staticmethod(capture))
    widget.set_device(device(kind, {"demo": True, "phase": .4}))
    assert not render(widget).isNull()
    assert "DEMO · 演示" in labels
    labels.clear()
    widget.set_device(device(kind, {"demo": True, "phase": 2.}, status="stale"))
    assert not render(widget).isNull()
    assert "DEMO · 演示" in labels
    assert "历史画面 · 已保持" in labels


def test_widget_module_does_not_import_other_qt_ros_torch_or_serial():
    source = """
import json, sys
from vt_capture_console.ui.widgets import PreviewWidget
print(json.dumps([name for name in sys.modules if name.split('.')[0] in
    {'PyQt5', 'PyQt6', 'rclpy', 'torch', 'serial'}]))
"""
    env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1] / "src"))
    result = subprocess.run([sys.executable, "-c", source], env=env, capture_output=True, text=True, check=True)
    assert json.loads(result.stdout) == []
