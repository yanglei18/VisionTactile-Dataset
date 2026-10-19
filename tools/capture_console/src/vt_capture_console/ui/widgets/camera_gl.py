"""GPU scaling of an already decoded camera image; no acquisition or timers.

Only enlarged previews use this surface. Qt's OpenGL paint engine manages the
image texture and its context lifetime. JPEG decoding stays in the shared cache.
"""
from functools import lru_cache

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtOpenGL import QOpenGLPaintDevice
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from ..gpu_status import inspect_current_context


@lru_cache(maxsize=1)
def hardware_camera_surface_available():
    """Probe once before promoting a raster window to OpenGL composition."""
    if QtWidgets.QApplication.platformName() in ("offscreen", "minimal"):
        return False
    previous = QtGui.QOpenGLContext.currentContext()
    previous_surface = previous.surface() if previous is not None else None
    context = QtGui.QOpenGLContext()
    surface = QtGui.QOffscreenSurface()
    try:
        if not context.create():
            return False
        surface.setFormat(context.format())
        surface.create()
        if not surface.isValid() or not context.makeCurrent(surface):
            return False
        return inspect_current_context().acceleration == "hardware"
    finally:
        context.doneCurrent()
        surface.destroy()
        if previous is not None and previous_surface is not None:
            previous.makeCurrent(previous_surface)


class CameraGLCanvas(QOpenGLWidget):
    """Image-only child of PreviewWidget; its parent owns labels and history."""
    failed = QtCore.Signal()

    def __init__(self, parent):
        super().__init__(parent)
        self.setObjectName("cameraGpuCanvas")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self._image = QtGui.QImage()
        self._frame_key = None
        self.available = False
        self._failed = False
        self.renderer_info = None
        self.paint_engine = None

    def initializeGL(self):
        self.renderer_info = inspect_current_context()
        self.available = self.renderer_info.acceleration == "hardware"
        if not self.available:
            self._fail()

    def _fail(self):
        self.available = False
        if not self._failed:
            self._failed = True
            self.failed.emit()  # The owner connects with QueuedConnection.

    def showEvent(self, event):
        super().showEvent(event)
        # A failed widget context can bypass initializeGL entirely. This is a
        # one-shot validity check, not a render loop; deletion cancels it.
        QtCore.QTimer.singleShot(0, self, self._check_context)

    def _check_context(self):
        if self.isVisible() and (not self.isValid() or not self.available):
            self._fail()

    def set_frame(self, image, held):
        key = (image.cacheKey(), bool(held))
        if key == self._frame_key:
            return
        self._image = QtGui.QImage(image)  # Implicitly shared, not a pixel copy.
        self._frame_key = key
        if self.isVisible():
            self.update()

    def paintGL(self):
        if not self.available:
            return
        # QWidget.grab()/render() redirects QPainter(self) onto a raster target.
        # QOpenGLWidget still expects paintGL to populate its framebuffer; the
        # mixed paths corrupt snapshots and page-transition images. Draw through
        # a dedicated GL device, including at fractional / high display scale.
        device = QOpenGLPaintDevice(self.size() * self.devicePixelRatioF())
        device.setDevicePixelRatio(self.devicePixelRatioF())
        painter = QtGui.QPainter(device)
        try:
            if not painter.isActive() or painter.paintEngine() is None:
                self._fail()
                return
            engine = painter.paintEngine().type()
            if engine in (QtGui.QPaintEngine.Type.OpenGL, QtGui.QPaintEngine.Type.OpenGL2):
                self.paint_engine = engine
            # A redirected presentation target is not proof of context loss.
            # Context/renderer checks own fallback; retain the last GL identity.
            painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
            rect = QtCore.QRectF(self.rect())
            painter.fillRect(rect, QtGui.QColor("#000000"))
            if not self._image.isNull():
                size = QtCore.QSizeF(self._image.size())
                size.scale(rect.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
                target = QtCore.QRectF(rect.center().x() - size.width() / 2,
                                      rect.center().y() - size.height() / 2,
                                      size.width(), size.height())
                painter.drawImage(target, self._image)
                if self._frame_key[1]:
                    self.parentWidget().paint_history_overlay(painter, rect)
        finally:
            if painter.isActive():
                painter.end()


def create_camera_canvas(parent):
    return CameraGLCanvas(parent) if hardware_camera_surface_available() else None
