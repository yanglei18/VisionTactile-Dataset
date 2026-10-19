"""Native, bounded presentation motion; no sensor data or backend access.

``AmbientBackdrop`` is a normal layout-bearing QWidget. ``TransitionStack``
can replace QStackedWidget without changing page ownership or navigation.
Both accept ``set_motion_enabled(False)`` for a quiet, static presentation.
"""

from __future__ import annotations

import math

from PySide6 import QtCore, QtGui, QtWidgets


def _foreground(widget: QtWidgets.QWidget) -> bool:
    window = widget.window()
    return widget.isVisible() and window.isActiveWindow() and not window.isMinimized()


class AmbientBackdrop(QtWidgets.QWidget):
    """Slow abstract light and orbital forms behind the working interface.

    This is atmospheric decoration, deliberately without grids, coordinates,
    numeric telemetry, or paths that could be mistaken for recorded motion.
    The timer runs at at most 15 fps, only in the foreground and enabled.
    Orbit and star updates invalidate only their old/new footprints. The very
    slow light field is sampled once a second into a small cached surface.
    """

    # Native caches are individually capped at 64 MiB on very large displays.
    _MAX_STATIC_CACHE_PIXELS = 16_777_216
    _MAX_LIGHT_CACHE_PIXELS = 262_144

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_OpaquePaintEvent)
        self._motion_enabled = True
        self._phase = 0.0
        self._mode = "capture"
        self._static_background: QtGui.QPixmap | None = None
        self._static_key: tuple | None = None
        self._light_background: QtGui.QPixmap | None = None
        self._light_key: tuple | None = None
        self._observed_window = None
        self._clock = QtCore.QElapsedTimer()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(67)
        self._timer.timeout.connect(self._advance)

    def set_motion_enabled(self, enabled: bool) -> None:
        if self._motion_enabled == bool(enabled):
            return
        self._motion_enabled = bool(enabled)
        self._clear_static_background()
        self._sync_timer()
        self.update()

    def set_mode(self, mode: str) -> None:
        """Change the light composition, without restarting its animation."""
        if self._mode == str(mode):
            return
        self._mode = str(mode)
        self._clear_static_background()
        self.update()

    def _sync_timer(self) -> None:
        active = self._motion_enabled and self.isEnabled() and _foreground(self)
        if active and not self._timer.isActive():
            self._clock.start()
            self._timer.start()
        elif not active:
            self._timer.stop()
            self._clock.invalidate()

    def _advance(self) -> None:
        # Keep phase continuous when a hidden/minimized window returns.
        previous = self._phase
        if self._clock.isValid():
            self._phase += min(self._clock.restart() / 1000.0, 0.1)
        if int(previous) != int(self._phase):
            self.update()
        else:
            # Transparent controls inherit the parent backing store. Updating
            # the whole parent here also repaints every control at 15 fps.
            self.update(self._motion_region(previous) | self._motion_region(self._phase))

    def showEvent(self, event: QtGui.QShowEvent) -> None:
        super().showEvent(event)
        window = self.window()
        if window is not self._observed_window:
            if self._observed_window is not None:
                self._observed_window.removeEventFilter(self)
            self._observed_window = window
            window.installEventFilter(self)
        self._sync_timer()

    def hideEvent(self, event: QtGui.QHideEvent) -> None:
        self._timer.stop()
        self._clock.invalidate()
        super().hideEvent(event)

    def changeEvent(self, event: QtCore.QEvent) -> None:
        super().changeEvent(event)
        if event.type() in (QtCore.QEvent.Type.EnabledChange,
                            QtCore.QEvent.Type.WindowStateChange,
                            QtCore.QEvent.Type.ActivationChange):
            self._sync_timer()

    def eventFilter(self, watched: QtCore.QObject, event: QtCore.QEvent) -> bool:
        if watched is self._observed_window and event.type() in (
                QtCore.QEvent.Type.WindowStateChange, QtCore.QEvent.Type.ActivationChange):
            self._sync_timer()
        return super().eventFilter(watched, event)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._timer.stop()
        self._clock.invalidate()
        super().closeEvent(event)

    @staticmethod
    def _light(painter: QtGui.QPainter, bounds: QtCore.QRectF,
               center: QtCore.QPointF, radius: float, color: QtGui.QColor) -> None:
        gradient = QtGui.QRadialGradient(center, radius)
        gradient.setColorAt(0.0, color)
        middle = QtGui.QColor(color)
        middle.setAlpha(round(color.alpha() * 0.37))
        gradient.setColorAt(0.42, middle)
        gradient.setColorAt(1.0, QtGui.QColor(color.red(), color.green(), color.blue(), 0))
        painter.fillRect(bounds, gradient)

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        try:
            if self._motion_enabled:
                self._paint_scene(painter)
                return

            # Transparent child widgets can invalidate their parent even with
            # decoration motion disabled. Reuse the artwork instead of
            # rasterizing three full-window radial gradients for every update.
            ratio = self.devicePixelRatioF()
            key = (self.width(), self.height(), ratio, self._mode, self._phase)
            if key != self._static_key:
                self._clear_static_background()
                width = math.ceil(self.width() * ratio)
                height = math.ceil(self.height() * ratio)
                if 0 < width * height <= self._MAX_STATIC_CACHE_PIXELS:
                    surface = QtGui.QPixmap(width, height)
                    if not surface.isNull():
                        surface.setDevicePixelRatio(ratio)
                        surface.fill(QtCore.Qt.GlobalColor.black)
                        cached_painter = QtGui.QPainter(surface)
                        try:
                            self._paint_scene(cached_painter)
                        finally:
                            cached_painter.end()
                        self._static_background = surface
                        self._static_key = key
            if self._static_background is not None:
                painter.drawPixmap(0, 0, self._static_background)
            else:
                self._paint_scene(painter)
        finally:
            painter.end()

    def _clear_static_background(self) -> None:
        self._static_background = None
        self._static_key = None

    def _paint_scene(self, painter: QtGui.QPainter) -> None:
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = float(self.width()), float(self.height())
        if w <= 0 or h <= 0:
            return
        self._paint_cached_light(painter, w, h)
        self._paint_orbit(painter, w, h)
        self._paint_stars(painter, w, h)

    def _paint_cached_light(self, painter: QtGui.QPainter, w: float, h: float) -> None:
        # Illumination changes only a few pixels per second at desktop sizes;
        # its soft gradients need neither native resolution nor 15 evaluations
        # per second. Thin orbit lines and stars remain at native resolution.
        phase = int(self._phase)
        ratio = self.devicePixelRatioF()
        key = (w, h, ratio, self._mode, phase)
        if self._light_key != key:
            self._light_background = None
            self._light_key = None
            scale = min(1.0, math.sqrt(self._MAX_LIGHT_CACHE_PIXELS / (w * h)))
            width, height = max(1, int(w * scale)), max(1, int(h * scale))
            surface = QtGui.QPixmap(width, height)
            if not surface.isNull():
                surface.fill(QtCore.Qt.GlobalColor.black)
                light_painter = QtGui.QPainter(surface)
                try:
                    light_painter.scale(width / w, height / h)
                    self._paint_lighting(light_painter, w, h, phase)
                finally:
                    light_painter.end()
                native_width, native_height = math.ceil(w * ratio), math.ceil(h * ratio)
                if native_width * native_height <= self._MAX_STATIC_CACHE_PIXELS:
                    native_surface = surface.scaled(native_width, native_height,
                                                    QtCore.Qt.AspectRatioMode.IgnoreAspectRatio,
                                                    QtCore.Qt.TransformationMode.SmoothTransformation)
                    if not native_surface.isNull():
                        surface = native_surface
                        surface.setDevicePixelRatio(ratio)
                self._light_background = surface
                self._light_key = key
        if self._light_background is None:
            self._paint_lighting(painter, w, h, phase)
        else:
            if (self._light_background.width() == math.ceil(w * ratio) and
                    self._light_background.height() == math.ceil(h * ratio)):
                painter.drawPixmap(0, 0, self._light_background)
            else:
                painter.save()
                painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
                painter.drawPixmap(QtCore.QRectF(0, 0, w, h), self._light_background,
                                   QtCore.QRectF(self._light_background.rect()))
                painter.restore()

    def _paint_lighting(self, painter: QtGui.QPainter, w: float, h: float,
                        phase: float) -> None:
        bounds = QtCore.QRectF(0, 0, w, h)
        wash = QtGui.QLinearGradient(0, 0, w * 0.6, h)
        wash.setColorAt(0, QtGui.QColor("#000000"))
        wash.setColorAt(0.5, QtGui.QColor("#030303"))
        wash.setColorAt(1, QtGui.QColor("#000000"))
        painter.fillRect(bounds, wash)

        shift = math.sin(phase * 0.07)
        spatial = self._mode == "spatial"
        highlight = QtCore.QPointF(w * (0.83 + shift * 0.025), h * (0.26 if spatial else 0.18))
        self._light(painter, bounds, highlight, max(w, h) * 0.66, QtGui.QColor(255, 255, 255, 10))
        self._light(painter, bounds, QtCore.QPointF(w * 0.14, h * 0.86),
                    max(w, h) * 0.42, QtGui.QColor(255, 255, 255, 3))
        self._light(painter, bounds, QtCore.QPointF(w * 0.54, h * 1.06),
                    max(w, h) * 0.5, QtGui.QColor(255, 255, 255, 4))

    @staticmethod
    def _orbit_geometry(w: float, h: float, phase: float) -> tuple[QtGui.QTransform, QtCore.QRectF]:
        transform = QtGui.QTransform()
        transform.translate(w * 0.78, h * 0.72)
        transform.rotate(-24 + math.sin(phase * 0.025) * 1.2)
        radius = max(w, h) * 0.69
        return transform, QtCore.QRectF(-radius, -radius * 0.56, radius * 2, radius * 1.12)

    def _paint_orbit(self, painter: QtGui.QPainter, w: float, h: float) -> None:
        # A vast, cropped rim gives the screen depth without becoming a chart.
        painter.save()
        transform, orbit = self._orbit_geometry(w, h, self._phase)
        painter.setTransform(transform, combine=True)
        for spread, alpha, width in ((12, 3, 16), (5, 6, 5), (0, 22, 0.7)):
            painter.setPen(QtGui.QPen(QtGui.QColor(200, 200, 200, alpha), width))
            painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            painter.drawEllipse(orbit.adjusted(-spread, -spread, spread, spread))
        painter.setPen(QtGui.QPen(QtGui.QColor(170, 170, 170, 12), 0.6))
        painter.drawEllipse(orbit.adjusted(44, 27, -44, -27))
        painter.restore()

    @staticmethod
    def _star_geometry(w: float, h: float, phase: float, index: int) -> tuple[float, float, float]:
        base_x = ((index * 0.61803398875 + 0.11) % 1.0)
        base_y = ((index * 0.41421356237 + 0.17) % 1.0)
        return (w * base_x + math.sin(phase * 0.03 + index) * 7,
                h * base_y + math.cos(phase * 0.025 + index * 0.73) * 5,
                0.65 if index % 4 else 1.15)

    def _paint_stars(self, painter: QtGui.QPainter, w: float, h: float) -> None:
        phase = self._phase
        # Deterministic and sparse: no allocation of per-frame particle state.
        for index in range(19):
            x, y, size = self._star_geometry(w, h, phase, index)
            alpha = round(24 + 13 * (math.sin(phase * 0.15 + index) + 1))
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(QtGui.QColor(210, 210, 210, alpha))
            painter.drawEllipse(QtCore.QPointF(x, y), size, size)

    def _motion_region(self, phase: float) -> QtGui.QRegion:
        w, h = float(self.width()), float(self.height())
        transform, orbit = self._orbit_geometry(w, h, phase)
        region = QtGui.QRegion()
        stroker = QtGui.QPainterPathStroker()
        # Quantize damage into 16 px tiles. An exact ellipse region creates
        # thousands of one-pixel clip spans that are expensive for transparent
        # descendants; these wider bands remain conservative and compact.
        tile = 16.0
        small = QtGui.QTransform.fromScale(1 / tile, 1 / tile)
        large = QtGui.QTransform.fromScale(tile, tile)
        for spread, width in ((12, 16), (5, 5), (0, 0.7), (None, 0.6)):
            path = QtGui.QPainterPath()
            rect = (orbit.adjusted(44, 27, -44, -27) if spread is None else
                    orbit.adjusted(-spread, -spread, spread, spread))
            path.addEllipse(rect)
            stroker.setWidth(width / tile + 4)
            outline = stroker.createStroke(small.map(transform.map(path)))
            region |= QtGui.QRegion(outline.toFillPolygon().toPolygon(),
                                    QtCore.Qt.FillRule.WindingFill)
        region = large.map(region)
        for index in range(19):
            x, y, size = self._star_geometry(w, h, phase, index)
            extent = size + 2
            region |= QtGui.QRegion(QtCore.QRectF(x - extent, y - extent,
                                                 extent * 2, extent * 2).toAlignedRect())
        return region & QtGui.QRegion(self.rect())


class _TransitionOverlay(QtWidgets.QWidget):
    """Short-lived, input-transparent snapshots, never a second live page."""

    def __init__(self, parent: QtWidgets.QWidget,
                 previous: QtGui.QPixmap, current: QtGui.QPixmap) -> None:
        super().__init__(parent)
        self.previous = previous
        self.current = current
        self.progress = 0.0
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_NoSystemBackground)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.setGeometry(parent.rect())

    def clear(self) -> None:
        self.previous = QtGui.QPixmap()
        self.current = QtGui.QPixmap()
        self.hide()
        self.deleteLater()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#000000"))
        progress = self.progress
        painter.setOpacity(1.0 - progress)
        painter.drawPixmap(QtCore.QPointF(-10 * progress, 0), self.previous)
        painter.setOpacity(progress)
        painter.drawPixmap(QtCore.QPointF(18 * (1 - progress), 0), self.current)


class TransitionStack(QtWidgets.QStackedWidget):
    """Immediately switch pages, then reveal them with a 280 ms transition.

    The actual current widget, signals, ownership, and input remain native
    QStackedWidget behavior. Only two temporary pixel snapshots are animated.
    """

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._motion_enabled = True
        self._overlay: _TransitionOverlay | None = None
        self._observed_window = None
        self._animation = QtCore.QVariantAnimation(self)
        self._animation.setDuration(280)
        self._animation.setStartValue(0.0)
        self._animation.setEndValue(1.0)
        self._animation.setEasingCurve(QtCore.QEasingCurve.Type.OutCubic)
        self._animation.valueChanged.connect(self._advance_transition)
        self._animation.finished.connect(self._clear_overlay)

    def set_motion_enabled(self, enabled: bool) -> None:
        self._motion_enabled = bool(enabled)
        if not self._motion_enabled:
            self._finish_transition()

    def setCurrentWidget(self, widget: QtWidgets.QWidget) -> None:
        index = self.indexOf(widget)
        if index >= 0:
            self.setCurrentIndex(index)

    def setCurrentIndex(self, index: int) -> None:
        if index == self.currentIndex() or index < 0 or index >= self.count():
            return
        self._finish_transition()
        animate = (self._motion_enabled and _foreground(self) and self.isEnabled()
                   and self.currentWidget() is not None
                   and self.width() > 0 and self.height() > 0)
        previous = self.grab() if animate else None
        super().setCurrentIndex(index)
        if not animate:
            return
        self.layout().activate()
        current = self.grab()
        self._overlay = _TransitionOverlay(self, previous, current)
        self._overlay.show()
        self._overlay.raise_()
        self._animation.start()

    def _advance_transition(self, value: float) -> None:
        if self._overlay is not None:
            self._overlay.progress = float(value)
            self._overlay.update()

    def _clear_overlay(self) -> None:
        if self._overlay is not None:
            self._overlay.clear()
            self._overlay = None

    def _finish_transition(self) -> None:
        self._animation.stop()
        self._clear_overlay()

    def showEvent(self, event: QtGui.QShowEvent) -> None:
        super().showEvent(event)
        window = self.window()
        if window is not self._observed_window:
            if self._observed_window is not None:
                self._observed_window.removeEventFilter(self)
            self._observed_window = window
            window.installEventFilter(self)

    def hideEvent(self, event: QtGui.QHideEvent) -> None:
        self._finish_transition()
        super().hideEvent(event)

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        self._finish_transition()
        super().resizeEvent(event)

    def changeEvent(self, event: QtCore.QEvent) -> None:
        super().changeEvent(event)
        if event.type() == QtCore.QEvent.Type.EnabledChange and not self.isEnabled():
            self._finish_transition()

    def eventFilter(self, watched: QtCore.QObject, event: QtCore.QEvent) -> bool:
        if (watched is self._observed_window
                and event.type() in (QtCore.QEvent.Type.WindowStateChange, QtCore.QEvent.Type.ActivationChange)
                and not _foreground(self)):
            self._finish_transition()
        return super().eventFilter(watched, event)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._finish_transition()
        super().closeEvent(event)
