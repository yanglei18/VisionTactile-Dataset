"""Presentation motion respects visibility, input, and page ownership."""

from PySide6 import QtCore, QtTest, QtWidgets
import pytest

from vt_capture_console.ui.widgets.motion import AmbientBackdrop, TransitionStack


@pytest.fixture
def backdrop(qapp):
    widget = AmbientBackdrop()
    widget.resize(640, 360)
    yield widget
    widget.close()
    widget.deleteLater()
    qapp.processEvents()


@pytest.fixture
def stack(qapp):
    widget = TransitionStack()
    widget.resize(640, 360)
    for name in ("First", "Second", "Third"):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.addWidget(QtWidgets.QPushButton(name))
        widget.addWidget(page)
    widget.show()
    qapp.processEvents()
    yield widget
    widget.close()
    widget.deleteLater()
    qapp.processEvents()


def test_backdrop_timer_only_runs_when_visible_and_enabled(backdrop, qapp):
    assert not backdrop._timer.isActive()
    assert backdrop._timer.interval() >= 34
    backdrop.show()
    qapp.processEvents()
    assert backdrop._timer.isActive()
    QtTest.QTest.qWait(80)
    assert backdrop._phase > 0
    backdrop.hide()
    assert not backdrop._timer.isActive()
    phase = backdrop._phase
    QtTest.QTest.qWait(50)
    assert backdrop._phase == phase
    backdrop.show()
    # The offscreen platform does not automatically reactivate a hidden window.
    qapp.setActiveWindow(backdrop)
    qapp.processEvents()
    assert backdrop._timer.isActive()
    backdrop.setEnabled(False)
    assert not backdrop._timer.isActive()
    backdrop.setEnabled(True)
    assert backdrop._timer.isActive()


def test_reduced_motion_remains_static_across_hide_show(backdrop, qapp):
    backdrop.show()
    backdrop.set_motion_enabled(False)
    assert not backdrop._timer.isActive()
    phase = backdrop._phase
    backdrop.hide()
    backdrop.show()
    backdrop.set_mode("spatial")
    qapp.processEvents()
    assert not backdrop._timer.isActive()
    assert backdrop._phase == phase
    assert not backdrop.grab().isNull()
    backdrop.set_motion_enabled(True)
    assert backdrop._timer.isActive()


def test_parent_minimize_and_hide_stop_backdrop_timer(qapp):
    parent = QtWidgets.QWidget()
    backdrop = AmbientBackdrop(parent)
    layout = QtWidgets.QVBoxLayout(parent)
    layout.addWidget(backdrop)
    parent.show()
    qapp.processEvents()
    assert backdrop._timer.isActive()
    parent.showMinimized()
    qapp.processEvents()
    assert not backdrop._timer.isActive()
    parent.showNormal()
    qapp.processEvents()
    assert backdrop._timer.isActive()
    parent.hide()
    assert not backdrop._timer.isActive()
    parent.close()
    parent.deleteLater()


def test_stack_changes_real_page_immediately_and_clears_snapshots(stack, qapp):
    pages = [stack.widget(index) for index in range(stack.count())]
    changes = QtTest.QSignalSpy(stack.currentChanged)
    stack.setCurrentIndex(1)
    assert stack.currentWidget() is pages[1]
    assert changes.count() == 1
    overlay = stack._overlay
    assert overlay is not None
    assert overlay.testAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
    assert overlay.focusPolicy() == QtCore.Qt.FocusPolicy.NoFocus
    assert not overlay.previous.isNull() and not overlay.current.isNull()
    # Even while snapshots are shown the live page receives input.
    button = pages[1].findChild(QtWidgets.QPushButton)
    assert qapp.widgetAt(button.mapToGlobal(button.rect().center())) is button
    clicks = QtTest.QSignalSpy(button.clicked)
    QtTest.QTest.mouseClick(button, QtCore.Qt.MouseButton.LeftButton)
    assert clicks.count() == 1
    QtTest.QTest.qWait(360)
    assert stack._overlay is None
    assert stack._animation.state() == QtCore.QAbstractAnimation.State.Stopped
    assert [stack.widget(index) for index in range(stack.count())] == pages


@pytest.mark.parametrize("action", ["hide", "disable", "resize", "close", "reduce"])
def test_interrupted_transition_releases_images_and_animation(stack, action, qapp):
    stack.setCurrentIndex(1)
    overlay = stack._overlay
    assert overlay is not None
    if action == "hide":
        stack.hide()
    elif action == "disable":
        stack.setEnabled(False)
    elif action == "resize":
        stack.resize(700, 400)
    elif action == "close":
        stack.close()
    else:
        stack.set_motion_enabled(False)
    assert stack._overlay is None
    assert overlay.previous.isNull() and overlay.current.isNull()
    assert stack._animation.state() == QtCore.QAbstractAnimation.State.Stopped
    assert stack.currentIndex() == 1


def test_quiet_stack_and_hidden_stack_switch_without_animation(stack):
    stack.set_motion_enabled(False)
    stack.setCurrentWidget(stack.widget(2))
    assert stack.currentIndex() == 2
    assert stack._overlay is None
    stack.set_motion_enabled(True)
    stack.hide()
    stack.setCurrentIndex(0)
    assert stack.currentIndex() == 0
    assert stack._overlay is None


def test_quick_navigation_discards_obsolete_snapshot(stack):
    stack.setCurrentIndex(1)
    previous_overlay = stack._overlay
    stack.setCurrentIndex(2)
    assert stack.currentIndex() == 2
    assert stack._overlay is not previous_overlay
    assert previous_overlay.previous.isNull()
    assert previous_overlay.current.isNull()
    stack.setCurrentIndex(2)
    assert stack.currentIndex() == 2
    stack.setCurrentIndex(-1)
    assert stack.currentIndex() == 2


def test_background_stops_backdrop_and_preserves_user_choice(backdrop, qapp):
    backdrop.show()
    qapp.processEvents()
    other = QtWidgets.QWidget()
    try:
        other.show()
        other.activateWindow()
        qapp.processEvents()
        assert not backdrop._timer.isActive()
        phase = backdrop._phase
        QtTest.QTest.qWait(90)
        assert backdrop._phase == phase
        backdrop.activateWindow()
        qapp.processEvents()
        assert backdrop._timer.isActive()
        backdrop.set_motion_enabled(False)
        other.activateWindow()
        qapp.processEvents()
        backdrop.activateWindow()
        qapp.processEvents()
        assert not backdrop._timer.isActive()
    finally:
        other.close()


def test_background_discards_transition_and_does_not_capture_new_snapshots(stack, qapp):
    stack.setCurrentIndex(1)
    assert stack._overlay is not None
    other = QtWidgets.QWidget()
    try:
        other.show()
        other.activateWindow()
        qapp.processEvents()
        assert stack._overlay is None
        assert stack._animation.state() == QtCore.QAbstractAnimation.State.Stopped
        stack.setCurrentIndex(2)
        assert stack.currentIndex() == 2 and stack._overlay is None
    finally:
        other.close()
