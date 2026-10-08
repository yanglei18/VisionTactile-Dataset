"""Candidate hand rendering shares only raw geometry, never hardware ownership."""

import json
import os
import subprocess
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6 import QtCore, QtGui, QtTest

from vt_capture_console.ui.widgets import PreviewWidget
from vt_capture_console.ui.widgets import preview as module
from vt_tactile_capture.hand_layout import load_layout


@pytest.fixture
def widgets(qapp):
    created = []

    def make():
        widget = PreviewWidget("tactile")
        widget.resize(640, 620)
        created.append(widget)
        return widget

    yield make
    for widget in created:
        widget.close()
        widget.deleteLater()
    qapp.processEvents()


def device(*, side="right", status="online", payload=None):
    return {"kind": "tactile", "id": f"{side}_tactile", "label": "Tactile preview",
            "side": side, "status": status,
            "preview": payload if payload is not None else {
                "values": list(range(400)), "valid_mask": [True] * 400,
                "rows": 20, "cols": 20, "levels": [0, 399]}}


def render(widget):
    image = QtGui.QImage(widget.size(), QtGui.QImage.Format.Format_ARGB32)
    image.fill(QtGui.QColor("white"))
    widget.render(image)
    return image


def point_position(widget, row, col):
    geometry = widget._tactile_geometry
    point = next(point for point in geometry.layout.points if (point.row, point.col) == (row, col))
    return geometry.transform(widget._hand_rect(widget._canvas_rect())).map(QtCore.QPointF(point.x, point.y))


def move_to(widget, position):
    event = QtGui.QMouseEvent(QtCore.QEvent.Type.MouseMove, position, position,
                             QtCore.Qt.MouseButton.NoButton, QtCore.Qt.MouseButton.NoButton,
                             QtCore.Qt.KeyboardModifier.NoModifier)
    widget.mouseMoveEvent(event)


def test_default_right_hand_reuses_all_packaged_addresses_and_marks_candidate(widgets):
    widget = widgets()
    widget.set_device(device())
    assert widget.tactile_view == "hand"
    assert widget.tactile_hand_available
    assert widget._tactile_geometry.layout == load_layout(expected_hand="right")
    assert len(widget._tactile_geometry.cells) == 400
    assert widget._tactile_geometry.layout.covered_indices == tuple(range(400))
    assert "候选" in widget.tactile_layout_note
    assert "9 个已观测锚点" in widget.tactile_layout_note
    assert "未逐点验证" in widget.tactile_layout_note
    assert "不代表压力" in widget.tactile_layout_note
    assert widget.accessibleDescription() == widget.tactile_layout_note
    assert not render(widget).isNull()


def test_local_rounded_cells_have_exact_row_col_hit_testing_and_no_x_mirror(widgets):
    widget = widgets()
    widget.set_device(device())
    geometry = widget._tactile_geometry
    rect = widget._hand_rect(widget._canvas_rect())
    for point, path in geometry.cells:
        position = geometry.transform(rect).map(QtCore.QPointF(point.x, point.y))
        assert geometry.point_at(position, rect) == point
        assert path.contains(QtCore.QPointF(point.x, point.y))
    assert point_position(widget, 19, 19).x() < point_position(widget, 19, 1).x()  # Thumb at left.
    assert point_position(widget, 19, 10).y() < point_position(widget, 8, 10).y()  # Tip at top.
    assert geometry.point_at(QtCore.QPointF(-100, -100), rect) is None


def test_hand_colors_preserve_valid_zero_invalid_mask_and_immutable_adc(widgets):
    widget = widgets()
    snapshot = device()
    snapshot["preview"]["valid_mask"][1] = False
    snapshot["preview"]["values"][1] = None
    widget.set_device(snapshot)
    image = render(widget)
    for row, col in ((0, 0), (0, 1), (19, 0)):
        position = point_position(widget, row, col)
        assert image.pixelColor(round(position.x()), round(position.y())) == widget._cell_colors[row * 20 + col]
    assert widget._cell_colors[1] == module._INVALID
    assert widget._cell_colors[0] != module._INVALID
    snapshot["preview"]["values"][0] = 999
    assert widget._frame["values"][0] == 0


def test_matrix_switch_preserves_measurements_and_signals_effective_changes(widgets):
    widget = widgets()
    spy = QtTest.QSignalSpy(widget.tactile_view_changed)
    widget.set_device(device())
    frame, levels = widget._frame, widget._levels
    before = bytes(render(widget).constBits())
    widget.set_tactile_view("matrix")
    assert widget.tactile_view == "matrix" and widget.tactile_hand_available
    assert widget._frame is frame and widget._levels is levels
    assert bytes(render(widget).constBits()) != before
    widget.set_tactile_view("matrix")
    assert spy.count() == 2
    assert spy.at(0)[0] == "hand" and spy.at(1)[0] == "matrix"
    widget.set_tactile_view("hand")
    assert widget._frame is frame and widget.tactile_view == "hand"
    with pytest.raises(ValueError):
        widget.set_tactile_view("unsupported")


@pytest.mark.parametrize("side", ["left", "", None, "unknown"])
def test_left_or_unknown_never_silently_uses_or_mirrors_right_hand(widgets, side):
    widget = widgets()
    widget.set_device(device(side=side))
    assert widget.tactile_view == "matrix"
    assert not widget.tactile_hand_available
    assert "未映射" in widget.tactile_layout_note
    assert "不镜像" in widget.tactile_layout_note
    assert widget._tactile_geometry is None
    widget.set_tactile_view("hand")
    assert widget.tactile_view == "matrix"
    assert not render(widget).isNull()


def test_explicit_matrix_preference_survives_changes_to_available_hand(widgets):
    widget = widgets()
    widget.set_tactile_view("matrix")
    widget.set_device(device())
    spy = QtTest.QSignalSpy(widget.tactile_view_changed)
    widget.set_device(device(side="left"))
    assert spy.count() == 1 and spy.at(0)[0] == "matrix"
    widget.set_device(device(side="right"))
    assert spy.count() == 2 and spy.at(1)[0] == "matrix"
    assert widget.tactile_view == "matrix" and widget.tactile_hand_available


def test_missing_layout_falls_back_to_matrix_without_breaking_preview(widgets, monkeypatch):
    def unavailable():
        raise FileNotFoundError("layout.json missing")

    monkeypatch.setattr(module, "right_hand_geometry", unavailable)
    widget = widgets()
    widget.set_device(device())
    assert widget.tactile_view == "matrix" and not widget.tactile_hand_available
    assert "布局不可用" in widget.tactile_layout_note
    assert widget._frame is not None
    render(widget)


def test_hand_and_matrix_hover_show_raw_address_validity_and_candidate(widgets):
    widget = widgets()
    snapshot = device()
    snapshot["preview"]["valid_mask"][1] = False
    widget.set_device(snapshot)
    move_to(widget, point_position(widget, 0, 0))
    assert "row 0, col 0" in widget.toolTip() and "0 ADC" in widget.toolTip()
    assert "手掌（掌心 / 掌根）" in widget.toolTip()
    assert "候选位置未验证" in widget.toolTip()
    move_to(widget, point_position(widget, 0, 1))
    assert "无效 / 未观测" in widget.toolTip()
    move_to(widget, point_position(widget, 19, 19))
    assert "拇指" in widget.toolTip() and "已观测锚点" in widget.toolTip()
    widget.set_device(device(status="offline", payload={}))
    move_to(widget, point_position(widget, 19, 19))
    assert "历史画面" in widget.toolTip()
    widget.set_tactile_view("matrix")
    matrix = widget._heatmap_rect(widget._canvas_rect())
    move_to(widget, QtCore.QPointF(matrix.left() + matrix.width() / 40,
                                 matrix.top() + matrix.height() / 40))
    assert "row 0, col 0" in widget.toolTip() and "0 ADC" in widget.toolTip()
    assert "历史画面" in widget.toolTip()


@pytest.mark.parametrize("status", ["stale", "offline"])
def test_held_frame_remains_frozen_across_view_switch_and_read_only_copy(widgets, status):
    source, target = widgets(), widgets()
    source.set_device(device())
    original = source._frame
    changed = device(status=status)
    changed["preview"]["values"] = [1000.] * 400
    source.set_device(changed)
    assert source._frame is original and source._held
    target.set_tactile_view("matrix")
    target.copy_display_state_from(source)
    assert target.tactile_view == "matrix" and target.tactile_hand_available
    assert target._held and target._device["status"] == status
    assert target._frame == original and target._frame is not original
    assert target._levels == source._levels and target._mode == source._mode
    assert target._frame["values"][0] == 0
    target.set_tactile_view("hand")
    assert target._held
    render(target)
    target.set_device(device(side="left"))
    with pytest.raises(ValueError):
        target.copy_display_state_from(source)


def test_all_invalid_or_bad_updates_hold_hand_and_device_change_clears_history(widgets):
    widget = widgets()
    widget.set_device(device())
    frame = widget._frame
    invalid = device()
    invalid["preview"]["valid_mask"] = [False] * 400
    widget.set_device(invalid)
    assert widget._held and widget._frame is frame
    invalid["preview"]["rows"] = 80
    widget.set_device(invalid)
    assert widget._held and widget._frame is frame and widget._preview_error
    widget.set_device(device(side="left", payload={}))
    assert widget._frame is None and widget.tactile_view == "matrix"


def test_hand_rendering_imports_no_legacy_qt_ros_serial_numpy_or_viewer():
    script = '''
import json, sys
from PySide6.QtWidgets import QApplication
from vt_capture_console.ui.widgets import PreviewWidget
app = QApplication([])
widget = PreviewWidget("tactile")
widget.set_device({"id": "right", "kind": "tactile", "side": "right", "status": "online", "preview": {}})
assert widget.tactile_hand_available
blocked = ("PyQt5", "PyQt6", "rclpy", "serial", "numpy", "pyqtgraph", "vt_tactile_capture.viewer")
print(json.dumps([name for name in sys.modules if any(name == prefix or name.startswith(prefix + ".") for prefix in blocked)]))
'''
    result = subprocess.run([sys.executable, "-c", script], check=True, capture_output=True, text=True,
                            env={**os.environ, "QT_QPA_PLATFORM": "offscreen"})
    assert json.loads(result.stdout) == []
