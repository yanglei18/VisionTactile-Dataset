import math

import pytest
from PySide6.QtCore import QEvent, QPoint, QPointF, Qt
from PySide6.QtGui import QInputDevice, QMouseEvent, QNativeGestureEvent, QPointingDevice, QWheelEvent
from PySide6.QtWidgets import QGestureEvent, QPinchGesture

from vt_capture_console.backend.scene_demo import DemoSceneBackend, demo_scene
from vt_capture_console.ui.widgets.spatial_page import SpatialPage


@pytest.fixture
def scene(qapp):
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    page.resize(1200, 760)
    page.show()
    page.apply_frame(demo_scene(0, scenario='static'), dict(state='ready'))
    qapp.processEvents()
    yield page
    page.close()
    backend.close()


@pytest.fixture
def touchpad(qapp):
    return QPointingDevice('test touchpad', 42, QInputDevice.DeviceType.TouchPad,
                           QPointingDevice.PointerType.Finger, QInputDevice.Capability.Position, 2, 0)


def native(device, kind, value=0.):
    return QNativeGestureEvent(kind, device, 2, QPointF(200, 200), QPointF(200, 200),
                               QPointF(200, 200), value, QPointF())


def test_pinch_changes_view_only_and_stops_cruise(scene, qapp, touchpad):
    view = scene.scene.viewport
    frame = scene.model.frame
    before = (view.scale, view.target, view.azimuth, view.elevation)
    scene.cruise_button.setChecked(True)
    for kind, value in [(Qt.NativeGestureType.BeginNativeGesture, 0),
                        (Qt.NativeGestureType.ZoomNativeGesture, .25),
                        (Qt.NativeGestureType.EndNativeGesture, 0)]:
        event = native(touchpad, kind, value)
        qapp.sendEvent(view, event)
        assert event.isAccepted()
    assert view.scale == pytest.approx(before[0] * 1.25)
    assert (view.target, view.azimuth, view.elevation) == before[1:]
    assert scene.model.frame is frame
    assert not scene.cruise_button.isChecked()
    qapp.sendEvent(view, native(touchpad, Qt.NativeGestureType.ZoomNativeGesture, -.2))
    assert view.scale == pytest.approx(before[0])


def test_pixel_only_two_finger_scroll_and_horizontal_scroll(scene, qapp, touchpad):
    view = scene.scene.viewport
    before = view.scale
    def scroll(pixels):
        return QWheelEvent(QPointF(200, 200), QPointF(200, 200), pixels, QPoint(),
                           Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
                           Qt.ScrollPhase.ScrollUpdate, False,
                           Qt.MouseEventSource.MouseEventNotSynthesized, touchpad)
    qapp.sendEvent(view, scroll(QPoint(0, 24)))
    assert view.scale == pytest.approx(before * math.exp(.1))
    qapp.sendEvent(view, scroll(QPoint(0, -24)))
    assert view.scale == pytest.approx(before)
    qapp.sendEvent(view, scroll(QPoint(50, 0)))
    assert view.scale == pytest.approx(before)


def test_qt_pinch_uses_increment_not_cumulative_scale(scene, qapp):
    view = scene.scene.viewport
    before = view.scale
    pinch = QPinchGesture()
    pinch.setChangeFlags(QPinchGesture.ChangeFlag.ScaleFactorChanged)
    pinch.setScaleFactor(1.1)
    pinch.setLastScaleFactor(1.1)
    pinch.setTotalScaleFactor(1.21)
    qapp.sendEvent(view, QGestureEvent([pinch]))
    qapp.sendEvent(view, QGestureEvent([pinch]))
    assert view.scale == pytest.approx(before * 1.21)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -1., -2.])
def test_bad_native_scale_does_not_break_view(scene, qapp, touchpad, value):
    view = scene.scene.viewport
    before = view.scale
    qapp.sendEvent(view, native(touchpad, Qt.NativeGestureType.ZoomNativeGesture, value))
    assert view.scale == before


def move(qapp, view, x, y, *, shift=False, buttons=Qt.MouseButton.NoButton):
    event = QMouseEvent(QEvent.Type.MouseMove, QPointF(x, y), QPointF(x, y),
                        Qt.MouseButton.NoButton, buttons,
                        Qt.KeyboardModifier.ShiftModifier if shift else Qt.KeyboardModifier.NoModifier)
    qapp.sendEvent(view, event)


@pytest.mark.parametrize('buttons', [Qt.MouseButton.NoButton, Qt.MouseButton.LeftButton])
def test_shift_touchpad_pan_moves_screen_without_rotation_or_data_changes(scene, qapp, buttons):
    view = scene.scene.viewport
    frame = scene.model.frame
    before = (view.azimuth, view.elevation, view.scale)
    point = (1., 2., 3.)
    projected = view._project(point)
    move(qapp, view, 100, 100)
    scene.cruise_button.setChecked(True)
    move(qapp, view, 130, 120, shift=True, buttons=buttons)
    after = view._project(point)
    assert after.x()-projected.x() == pytest.approx(30.)
    assert after.y()-projected.y() == pytest.approx(20.)
    assert (view.azimuth, view.elevation, view.scale) == before
    assert scene.model.frame is frame
    assert not scene.cruise_button.isChecked()
    target = view.target
    move(qapp, view, 150, 140)
    assert view.target == target
    move(qapp, view, 160, 150, buttons=Qt.MouseButton.LeftButton)
    assert (view.azimuth, view.elevation) != before[:2]
    assert view.target == target


def test_shift_reentry_does_not_jump(scene, qapp):
    view = scene.scene.viewport
    move(qapp, view, 100, 100)
    qapp.sendEvent(view, QEvent(QEvent.Type.Leave))
    target = view.target
    move(qapp, view, 500, 400, shift=True)
    assert view.target == target
    move(qapp, view, 510, 410, shift=True)
    assert view.target != target
