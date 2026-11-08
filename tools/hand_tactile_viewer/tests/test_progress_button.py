"""Visible progress, text and interaction under the actual application theme."""
import numpy as np
from PySide6 import QtCore, QtGui, QtTest, QtWidgets
import pytest

from vt_capture_console.theme import stylesheet
from vt_fsglove12.ui_theme import PALETTE
from vt_hand_tactile_viewer.progress_button import ProgressButton


@pytest.fixture
def button():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    old_style, old_sheet = app.style().objectName(), app.styleSheet()
    app.setStyle("Fusion")
    app.setStyleSheet(stylesheet())
    widget = ProgressButton("Capture this pose")
    widget.resize(420, 48)
    widget.show()
    app.processEvents()
    try:
        yield widget
    finally:
        widget.close()
        widget.deleteLater()
        app.processEvents()
        app.setStyleSheet(old_sheet)
        app.setStyle(old_style)


def pixels(widget):
    image = widget.grab().toImage().convertToFormat(QtGui.QImage.Format.Format_RGBA8888)
    rows = np.frombuffer(image.constBits(), dtype=np.uint8).reshape(image.height(), image.bytesPerLine())
    return rows[:, :image.width() * 4].reshape(image.height(), image.width(), 4)[:, :, :3].copy()


def color_fraction(image, role):
    color = QtGui.QColor(PALETTE[role])
    return np.all(image == (color.red(), color.green(), color.blue()), axis=2).mean()


def test_progress_fills_button_background_and_keeps_its_text_readable(button):
    fractions = []
    for value in (None, .5, 1.):
        button.set_progress(value)
        QtWidgets.QApplication.processEvents()
        image = pixels(button)
        fractions.append(color_fraction(image, "border"))
        # The label must survive the progress overlay, including a full fill.
        middle = image[image.shape[0] // 4:3 * image.shape[0] // 4,
                       image.shape[1] // 4:3 * image.shape[1] // 4]
        assert np.count_nonzero(np.all(middle > 160, axis=2)) > 20
    # Pixel area, not QProgressBar.value(): a three-pixel strip fails this check.
    assert fractions[0] < .10
    assert .30 < fractions[1] < .60
    assert fractions[2] > .70
    assert fractions[2] - fractions[0] > .60


def test_idle_track_is_visible_and_does_not_intercept_the_button_click(button):
    button.set_progress()
    QtWidgets.QApplication.processEvents()
    bar = button.progress_bar
    assert bar.isVisible()
    assert bar.width() * bar.height() > button.width() * button.height() * .80
    assert color_fraction(pixels(button), "raised") > .70
    assert bar.testAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
    clicked = []
    button.clicked.connect(lambda: clicked.append(True))
    # Send a window-system event at the overlay, so Qt performs child hit testing.
    QtTest.QTest.mouseClick(button.windowHandle(), QtCore.Qt.MouseButton.LeftButton,
                           QtCore.Qt.KeyboardModifier.NoModifier, button.rect().center())
    assert clicked == [True]


def test_busy_background_actually_animates_when_button_is_disabled(button):
    button.setEnabled(False)
    button.set_progress(busy=True)
    QtTest.QTest.qWait(60)
    first = pixels(button)
    clock = QtCore.QElapsedTimer()
    clock.start()
    while clock.elapsed() < 1200:
        QtTest.QTest.qWait(40)
        changed = pixels(button)
        # Require a visible moving region, not a cursor or one changed pixel.
        if np.count_nonzero(np.any(changed != first, axis=2)) > 100:
            break
    else:
        pytest.fail("Disabled capture button's busy background did not animate")
