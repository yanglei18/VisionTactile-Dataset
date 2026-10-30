"""Touchpad zoom shared by the OpenGL and projection scene viewports."""
import math

from PySide6.QtCore import QEvent, Qt
from PySide6.QtWidgets import QApplication, QPinchGesture


def handle_zoom_gesture(event, zoom):
    if event.type() == QEvent.Type.NativeGesture:
        kind = event.gestureType()
        if kind == Qt.NativeGestureType.ZoomNativeGesture:
            # Qt supplies an incremental magnification, not a wheel angle.
            factor = 1. + event.value()
            if math.isfinite(factor) and factor > 0:
                zoom(factor)
        elif kind not in (Qt.NativeGestureType.BeginNativeGesture, Qt.NativeGestureType.EndNativeGesture):
            return False
        event.accept()
        return True
    if event.type() == QEvent.Type.Gesture:
        pinch = event.gesture(Qt.GestureType.PinchGesture)
        if pinch is None:
            return False
        # QPinchGesture.scaleFactor is incremental. TouchEnd can retain the
        # last change flags; do not apply that last increment a second time.
        if (pinch.state() not in (Qt.GestureState.GestureFinished, Qt.GestureState.GestureCanceled)
                and pinch.changeFlags() & QPinchGesture.ChangeFlag.ScaleFactorChanged):
            factor = pinch.scaleFactor()
            if math.isfinite(factor) and factor > 0:
                zoom(factor)
        event.accept(pinch)
        return True
    return False


def scroll_zoom_factor(event):
    """Also accept two-finger scrolling, including pixel-only touchpad events."""
    pixels, angle = event.pixelDelta().y(), event.angleDelta().y()
    # Qt documents X11 pixelDelta as driver-dependent: prefer its angle when
    # available. Other backends provide smooth pixel scrolling directly.
    if pixels and (QApplication.platformName() != 'xcb' or not angle):
        delta = pixels / 240.
    else:
        delta = angle / 1200.
    # Qt has already applied the desktop's natural scrolling direction.
    return math.exp(max(-4., min(4., delta)))
