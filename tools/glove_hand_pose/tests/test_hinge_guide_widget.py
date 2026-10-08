"""Offscreen checks for the isolated, explicitly synthetic teaching diagram."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5")
from PyQt5 import QtCore, QtGui, QtTest, QtWidgets

from vt_glove_pose.hinge_guide_widget import DISCLAIMER, MotionGuideCanvas


@pytest.fixture(scope="module")
def app():
    instance = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["guide-tests"])
    yield instance


def test_all_instructions_render_with_existing_light_and_dark_palettes(app):
    widget = MotionGuideCanvas()
    widget.resize(330, 230)
    for dark in (False, True):
        palette = widget.palette()
        if dark:
            palette.setColor(QtGui.QPalette.Base, QtGui.QColor("#202124"))
            palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor("#35373b"))
            palette.setColor(QtGui.QPalette.Text, QtGui.QColor("#eeeeee"))
        widget.setPalette(palette)
        for illustration in ("connect", "still", "flex", "rotate", "flex_once", "hold", "return", "export"):
            widget.set_instruction(illustration, "prepare")
            image = QtGui.QImage(widget.size(), QtGui.QImage.Format_ARGB32)
            image.fill(QtCore.Qt.transparent)
            widget.render(image)
            assert not image.isNull()
            # The hand and footer contain actual painted content, not a blank widget.
            colors = {image.pixelColor(x, y).rgba() for x in range(10, 320, 5) for y in range(55, 228, 3)}
            assert len(colors) > 8
            assert DISCLAIMER in widget.accessibleName()
            assert "不是传感器测量" in widget.toolTip()
    widget.close()


def test_idle_prepare_and_unknown_phases_never_animate(app):
    widget = MotionGuideCanvas()
    widget.show()
    for phase in ("idle", "prepare", "done", "error"):
        widget.set_instruction("flex", phase)
        assert widget._illustrated_pose(0.) == widget._illustrated_pose(2.)
        assert not widget._timer.isActive()
    widget.set_instruction("hold", "capture")
    assert not widget._timer.isActive()
    widget.close()


def test_capture_animates_only_the_requested_demonstration_and_repeated_refresh_does_not_restart(app):
    widget = MotionGuideCanvas()
    widget.show()
    widget.set_instruction("flex", "capture")
    assert widget._timer.isActive()
    assert widget._illustrated_pose(0.) == (0., 0.)
    assert widget._illustrated_pose(2.5) == (1., 0.)
    QtTest.QTest.qWait(55)
    before = widget._elapsed.elapsed()
    widget.set_instruction("flex", "capture")
    assert widget._elapsed.elapsed() >= before
    widget.set_instruction("rotate", "capture")
    assert widget._illustrated_pose(0.)[0] == widget._illustrated_pose(1.75)[0]
    assert widget._illustrated_pose(1.75)[1] == 16.
    widget.hide()
    assert not widget._timer.isActive()
    widget.close()


def test_single_flex_and_return_do_not_repeat_and_no_anatomical_label_is_claimed(app):
    widget = MotionGuideCanvas()
    widget.set_instruction("flex_once", "capture")
    assert widget._illustrated_pose(0.)[0] == 0.
    assert widget._illustrated_pose(3.)[0] == widget._illustrated_pose(50.)[0] == 1.
    widget.set_instruction("return", "capture")
    assert widget._illustrated_pose(0.)[0] > 0
    assert widget._illustrated_pose(3.)[0] == widget._illustrated_pose(50.)[0] == 0.
    assert "IP" not in widget.accessibleDescription()
    assert "不一定是完全伸直" in widget.accessibleDescription()
    widget.close()


def test_resizes_and_rejects_unknown_instruction_without_changing_state(app):
    widget = MotionGuideCanvas()
    assert widget.sizeHint() == QtCore.QSize(330, 230)
    widget.set_instruction("still", "idle")
    with pytest.raises(ValueError, match="illustration"):
        widget.set_instruction("live_angle", "capture")
    assert widget._illustration == "still"
    for size in ((260, 190), (660, 460), (480, 230)):
        widget.resize(*size)
        image = QtGui.QImage(widget.size(), QtGui.QImage.Format_ARGB32)
        widget.render(image)
        assert not image.isNull()
    widget.close()
