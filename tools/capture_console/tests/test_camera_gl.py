"""Camera GPU admission and preview lifecycle, without an OpenGL surface."""
from types import SimpleNamespace

import pytest
from PySide6 import QtCore, QtGui, QtWidgets

from vt_capture_console.ui.gpu_status import OpenGLRenderer
from vt_capture_console.ui.widgets import camera_gl
from vt_capture_console.ui.widgets.preview import PreviewWidget
from test_preview_widgets import device, jpeg_preview, render


@pytest.fixture(autouse=True)
def isolated_gpu_admission(monkeypatch):
    monkeypatch.delenv("VT_CAMERA_RENDERER", raising=False)
    camera_gl.hardware_camera_surface_available.cache_clear()
    yield
    camera_gl.hardware_camera_surface_available.cache_clear()


@pytest.fixture
def previews(qapp):
    created = []

    def create(*, presentation="stage", width=900):
        widget = PreviewWidget("camera")
        widget.set_presentation(presentation)
        widget.resize(width, 500)
        created.append(widget)
        return widget

    yield create
    for widget in created:
        widget.close()
        widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    qapp.processEvents()


class CameraSurface(QtWidgets.QWidget):
    """Use the production frame-sharing method, replacing only the GL widget."""
    failed = QtCore.Signal()

    def __init__(self, parent):
        super().__init__(parent)
        self._image = QtGui.QImage()
        self._frame_key = None
        self.available = True
        self._failed = False
        self.context_valid = True
        self.frames = []
        self.render_requests = 0

    def set_frame(self, image, held):
        self.frames.append((image.cacheKey(), held))
        camera_gl.CameraGLCanvas.set_frame(self, image, held)

    def update(self):
        self.render_requests += 1
        super().update()

    _fail = camera_gl.CameraGLCanvas._fail

    def isValid(self):
        return self.context_valid


@pytest.fixture
def hardware_surface(monkeypatch):
    created = []
    monkeypatch.setattr(QtWidgets.QApplication, "platformName", staticmethod(lambda: "xcb"))

    def create(parent):
        surface = CameraSurface(parent)
        created.append(surface)
        return surface

    monkeypatch.setattr(camera_gl, "create_camera_canvas", create)
    return created


def assert_raster_pixels(preview):
    assert not preview._using_camera_gpu()
    rendered = render(preview)
    assert rendered.pixelColor(preview._canvas_rect().center().toPoint()) == preview._image.pixelColor(0, 0)


@pytest.mark.parametrize("platform", ["offscreen", "minimal"])
def test_headless_preview_and_probe_never_construct_gl_context(previews, monkeypatch, platform):
    monkeypatch.setattr(QtWidgets.QApplication, "platformName", staticmethod(lambda: platform))

    def forbidden(*args):
        pytest.fail("a headless camera preview must not construct an OpenGL context")

    monkeypatch.setattr(QtGui, "QOpenGLContext", forbidden)
    assert not camera_gl.hardware_camera_surface_available()
    preview = previews()
    preview.set_device(device("camera", jpeg_preview()))
    preview.show()
    assert preview._camera_gpu is None
    assert_raster_pixels(preview)


def install_probe(monkeypatch, renderer, *, failure=None):
    """Provide context lifecycle boundaries without running a graphics driver."""
    calls = []
    previous_surface = object()
    previous = SimpleNamespace(surface=lambda: previous_surface,
                               makeCurrent=lambda surface: calls.append(("restore", surface)))

    class Context:
        currentContext = staticmethod(lambda: previous)

        def create(self):
            calls.append("create")
            return failure != "create"

        def format(self):
            return None

        def makeCurrent(self, surface):
            calls.append("current")
            return failure != "current"

        def doneCurrent(self):
            calls.append("done")

    class Surface:
        def setFormat(self, value):
            pass

        def create(self):
            calls.append("surface")

        def isValid(self):
            return failure != "surface"

        def destroy(self):
            calls.append("destroy")

    monkeypatch.setattr(QtWidgets.QApplication, "platformName", staticmethod(lambda: "xcb"))
    monkeypatch.setattr(QtGui, "QOpenGLContext", Context)
    monkeypatch.setattr(QtGui, "QOffscreenSurface", Surface)
    monkeypatch.setattr(camera_gl, "inspect_current_context", lambda: OpenGLRenderer(renderer=renderer))
    return calls, previous_surface


@pytest.mark.parametrize("renderer", ["llvmpipe (LLVM 20)", "softpipe", "virgl", ""])
def test_unconfirmed_acceleration_uses_raster_and_probes_only_once(previews, monkeypatch, renderer):
    calls, previous_surface = install_probe(monkeypatch, renderer)
    first, second = previews(), previews()
    for preview in (first, second):
        preview.set_device(device("camera", jpeg_preview()))
        preview.show()
        assert preview._camera_gpu is None
        assert_raster_pixels(preview)
    assert calls.count("create") == 1
    assert calls[-1] == ("restore", previous_surface)


@pytest.mark.parametrize("failure", ["create", "surface", "current"])
def test_probe_failure_restores_previous_context_and_falls_back(qapp, monkeypatch, failure):
    calls, previous_surface = install_probe(monkeypatch, "NVIDIA GeForce RTX 5060 Ti", failure=failure)
    assert not camera_gl.hardware_camera_surface_available()
    assert calls[-2:] == ["destroy", ("restore", previous_surface)]


def test_explicit_raster_mode_skips_gpu_and_can_hide_existing_surface(previews, hardware_surface, monkeypatch):
    monkeypatch.setenv("VT_CAMERA_RENDERER", "raster")
    preview = previews()
    payload = jpeg_preview()
    preview.set_device(device("camera", payload))
    preview.show()
    assert hardware_surface == []
    assert_raster_pixels(preview)
    monkeypatch.delenv("VT_CAMERA_RENDERER")
    preview.set_device(device("camera", payload))
    assert preview._using_camera_gpu()
    canvas = hardware_surface[0]
    monkeypatch.setenv("VT_CAMERA_RENDERER", "raster")
    preview.set_device(device("camera", payload))
    assert canvas.isHidden()
    assert_raster_pixels(preview)


@pytest.mark.parametrize("presentation, width", [("thumbnail", 900), ("standard", 320), ("stage", 320)])
def test_small_or_thumbnail_previews_keep_raster_without_creating_gl(previews, hardware_surface, presentation, width):
    preview = previews(presentation=presentation, width=width)
    preview.set_device(device("camera", jpeg_preview()))
    preview.show()
    assert hardware_surface == []
    assert_raster_pixels(preview)


def test_enlarged_preview_shares_decoded_image_and_repeated_frame_does_not_render_again(
        previews, hardware_surface, monkeypatch):
    source = previews(presentation="thumbnail")
    source.set_device(device("camera", jpeg_preview()))
    preview = previews()
    preview.show()

    def forbidden(*args):
        pytest.fail("enlarging or redrawing a cached image must not decode JPEG again")

    monkeypatch.setattr(QtGui, "QImageReader", forbidden)
    preview.copy_camera_tracker_state_from(source)
    canvas = hardware_surface[0]
    assert preview._using_camera_gpu()
    assert canvas._image.cacheKey() == preview._image.cacheKey() == source._image.cacheKey()
    assert canvas._image is not preview._image and preview._image is not source._image
    before = canvas.render_requests
    for _ in range(8):
        preview.copy_camera_tracker_state_from(source)
    assert canvas.render_requests == before
    assert len(hardware_surface) == 1


def test_raster_screenshot_redirection_preserves_live_gpu_surface(previews, hardware_surface, monkeypatch):
    preview = previews()
    preview.set_device(device("camera", jpeg_preview()))
    preview.show()
    canvas = hardware_surface[0]
    canvas.paint_engine = QtGui.QPaintEngine.Type.OpenGL2
    screenshot = QtGui.QImage(canvas.size(), QtGui.QImage.Format.Format_RGB32)
    screenshot.fill(QtCore.Qt.GlobalColor.black)

    class RedirectedPainter(QtGui.QPainter):
        def __init__(self, widget):
            super().__init__(screenshot)

    monkeypatch.setattr(camera_gl.QtGui, "QPainter", RedirectedPainter)
    camera_gl.CameraGLCanvas.paintGL(canvas)
    assert not canvas._failed
    assert canvas.available and preview._using_camera_gpu()
    assert canvas.paint_engine == QtGui.QPaintEngine.Type.OpenGL2
    assert screenshot.pixelColor(screenshot.width()//2, screenshot.height()//2) == canvas._image.pixelColor(0, 0)


def test_gpu_live_hold_demo_recovery_never_shows_old_camera_in_demo(previews, hardware_surface):
    preview = previews()
    payload = jpeg_preview()
    preview.set_device(device("camera", payload))
    preview.show()
    canvas = hardware_surface[0]
    live_key = canvas._image.cacheKey()
    assert preview._using_camera_gpu() and canvas.frames[-1] == (live_key, False)
    preview.set_device(device("camera", status="stale"))
    assert preview._held and preview._using_camera_gpu()
    assert canvas.frames[-1] == (live_key, True)
    preview.set_device(device("camera", {"demo": True, "phase": .3}))
    assert canvas.isHidden() and not preview._using_camera_gpu()
    assert preview._mode == "demo" and preview._image is None
    preview.set_device(device("camera", jpeg_preview(48, 48)))
    assert preview._mode == "live" and not preview._held
    assert preview._using_camera_gpu() and len(hardware_surface) == 1
    assert canvas._image.size() == QtCore.QSize(48, 48)
    assert canvas.frames[-1][1] is False


def test_hidden_preview_retains_latest_frame_without_gpu_upload_until_show(previews, hardware_surface):
    preview = previews()
    preview.set_device(device("camera", jpeg_preview()))
    preview.show()
    canvas = hardware_surface[0]
    preview.hide()
    frames, renders = len(canvas.frames), canvas.render_requests
    for width in (80, 96, 128):
        preview.set_device(device("camera", jpeg_preview(width, 72)))
    assert len(canvas.frames) == frames and canvas.render_requests == renders
    assert not preview._using_camera_gpu()
    preview.show()
    assert preview._using_camera_gpu()
    assert len(canvas.frames) == frames + 1
    assert canvas._image.size() == QtCore.QSize(128, 72)
    assert canvas._image.cacheKey() == preview._image.cacheKey()


def test_context_falling_back_to_software_restores_visible_raster_pixels(
        qapp, previews, hardware_surface, monkeypatch):
    preview = previews()
    payload = jpeg_preview()
    preview.set_device(device("camera", payload))
    preview.show()
    canvas = hardware_surface[0]
    monkeypatch.setattr(camera_gl, "inspect_current_context",
                        lambda: OpenGLRenderer(renderer="llvmpipe (LLVM 20)"))
    camera_gl.CameraGLCanvas.initializeGL(canvas)
    assert not canvas.available
    # Initialization requests disposal through the event loop, not on its GL stack.
    assert preview._camera_gpu is canvas
    qapp.processEvents()
    assert preview._camera_gpu is None and canvas.isHidden()
    assert_raster_pixels(preview)
    for _ in range(5):
        preview.set_device(device("camera", payload))
    assert len(hardware_surface) == 1
    assert_raster_pixels(preview)


@pytest.mark.parametrize("failure", [None, ImportError("GL unavailable"), RuntimeError("context unavailable")])
def test_canvas_creation_failure_keeps_last_good_pixels_and_does_not_retry_each_frame(
        previews, hardware_surface, monkeypatch, failure):
    attempts = []

    def unavailable(parent):
        attempts.append(parent)
        if failure is not None:
            raise failure
        return None

    monkeypatch.setattr(camera_gl, "create_camera_canvas", unavailable)
    preview = previews()
    payload = jpeg_preview()
    preview.set_device(device("camera", payload))
    preview.show()
    for _ in range(5):
        preview.set_device(device("camera", payload))
    assert len(attempts) == 1
    assert_raster_pixels(preview)


def test_failed_widget_context_without_initialize_callback_returns_to_raster(
        qapp, previews, hardware_surface):
    preview = previews()
    preview.set_device(device("camera", jpeg_preview()))
    preview.show()
    canvas = hardware_surface[0]
    canvas.context_valid = False
    camera_gl.CameraGLCanvas._check_context(canvas)
    assert preview._camera_gpu is canvas  # Destruction remains queued.
    qapp.processEvents()
    assert preview._camera_gpu is None and canvas.isHidden()
    assert_raster_pixels(preview)


@pytest.mark.parametrize("active", [False, True])
def test_inactive_or_missing_painter_does_not_leave_blank_gpu_surface(
        qapp, previews, hardware_surface, monkeypatch, active):
    preview = previews()
    preview.set_device(device("camera", jpeg_preview()))
    preview.show()
    canvas = hardware_surface[0]

    class Painter:
        def __init__(self, target):
            self.active = active

        def isActive(self):
            return self.active

        def paintEngine(self):
            return None

        def end(self):
            self.active = False

    with monkeypatch.context() as graphics:
        graphics.setattr(QtGui, "QPainter", Painter)
        camera_gl.CameraGLCanvas.paintGL(canvas)
    qapp.processEvents()
    assert preview._camera_gpu is None and canvas.isHidden()
    assert_raster_pixels(preview)
