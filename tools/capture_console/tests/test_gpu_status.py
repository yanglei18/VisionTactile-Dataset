"""Renderer identity comes from the active GL context, not context availability."""
import pytest
from types import SimpleNamespace
from PySide6.QtCore import Signal
from PySide6.QtGui import QOpenGLContext

from vt_capture_console.i18n import language, set_language
from vt_capture_console.ui.gpu_status import OpenGLRenderer, classify_renderer, inspect_current_context
from vt_capture_console.ui.gpu_status import GL_RENDERER, GL_VENDOR, GL_VERSION
from vt_capture_console.ui.widgets import spatial_scene


@pytest.mark.parametrize("renderer", [
    "llvmpipe (LLVM 20.1.2, 256 bits)",
    "softpipe", "Software Rasterizer", "Mesa X11 swrast", "GDI Generic",
    "SwiftShader Device (Subzero)", "lavapipe", "OpenSWR",
    "ANGLE (Microsoft, Microsoft Basic Render Driver Direct3D11)",
    "ANGLE (NVIDIA GeForce RTX 5060 Ti, SwiftShader)",
])
def test_software_gl_is_not_reported_as_gpu(renderer):
    assert classify_renderer(renderer) == "software"


@pytest.mark.parametrize("renderer", [
    "NVIDIA GeForce RTX 5060 Ti/PCIe/SSE2",
    "Mesa Intel(R) Graphics (ADL GT2)",
    "AMD Radeon RX 7900 XTX (radeonsi, LLVM 19.1.1, DRM 3.61)",
    "ANGLE (Intel, Intel(R) UHD Graphics 620 Direct3D11 vs_5_0 ps_5_0)",
    "Apple M3", "Adreno (TM) 740",
])
def test_known_physical_renderer_identifies_hardware(renderer):
    assert classify_renderer(renderer) == "hardware"


@pytest.mark.parametrize("renderer", ["", "Unknown", "virgl (NVIDIA)", "SVGA3D; build", "Future Renderer"])
def test_unknown_and_virtual_contexts_do_not_claim_acceleration(renderer):
    assert classify_renderer(renderer) == "unknown"


def test_vendor_and_gl_version_alone_do_not_establish_acceleration():
    assert OpenGLRenderer(vendor="NVIDIA Corporation", version="4.6").acceleration == "unknown"


def test_probe_reads_all_strings_once_and_decodes_driver_bytes(monkeypatch):
    values = {GL_RENDERER: b"NVIDIA GeForce RTX 5060 Ti/PCIe/SSE2",
              GL_VENDOR: "NVIDIA Corporation", GL_VERSION: b"4.6.0 NVIDIA 580.178.04"}
    calls = []
    def read(key):
        calls.append(key)
        return values[key]
    context = SimpleNamespace(functions=lambda: SimpleNamespace(glGetString=read))
    monkeypatch.setattr(QOpenGLContext, "currentContext", lambda: context)
    info = inspect_current_context()
    assert info.acceleration == "hardware"
    assert info.renderer == "NVIDIA GeForce RTX 5060 Ti/PCIe/SSE2"
    assert info.vendor == "NVIDIA Corporation"
    assert info.version == "4.6.0 NVIDIA 580.178.04"
    assert calls == [GL_RENDERER, GL_VENDOR, GL_VERSION]


def test_probe_failure_is_diagnostic_and_does_not_raise(monkeypatch):
    def unavailable(_key):
        raise RuntimeError("context query unavailable")
    context = SimpleNamespace(functions=lambda: SimpleNamespace(glGetString=unavailable))
    monkeypatch.setattr(QOpenGLContext, "currentContext", lambda: context)
    info = inspect_current_context()
    assert info.acceleration == "unknown"
    assert info.error == "context query unavailable"


def test_probe_without_context_never_claims_gpu(monkeypatch):
    monkeypatch.setattr(QOpenGLContext, "currentContext", lambda: None)
    info = inspect_current_context()
    assert info.acceleration == "unknown"
    assert info.error == "No current OpenGL context"


def test_probe_with_unavailable_strings_never_claims_gpu(monkeypatch):
    context = SimpleNamespace(functions=lambda: SimpleNamespace(glGetString=lambda _key: None))
    monkeypatch.setattr(QOpenGLContext, "currentContext", lambda: context)
    assert inspect_current_context().acceleration == "unknown"


@pytest.fixture
def simulated_gl_scene(qapp, monkeypatch):
    class SimulatedGLViewport(spatial_scene.ProjectionViewport):
        failed = Signal(str)
        renderer_ready = Signal(object)
    monkeypatch.delenv("VT_SPATIAL_RENDERER", raising=False)
    monkeypatch.setattr(spatial_scene.QApplication, "platformName", lambda: "xcb")
    monkeypatch.setattr(spatial_scene, "create_gl_view", SimulatedGLViewport)
    original_language = language()
    set_language("zh_CN", persist=False)
    scene = spatial_scene.SpatialScene()
    try:
        yield scene
    finally:
        scene.close()
        scene.deleteLater()
        qapp.processEvents()
        set_language(original_language, persist=False)


def test_scene_displays_actual_software_status_and_retranslates(qapp, simulated_gl_scene):
    scene = simulated_gl_scene
    assert scene.render_mode == "opengl"
    assert scene.renderer_info is None
    assert scene.note.text() == "三维 · 待确认"
    info = OpenGLRenderer(renderer="llvmpipe (LLVM 20.1.2, 256 bits)",
                          vendor="Mesa", version="4.5 Mesa")
    scene.viewport.renderer_ready.emit(info)
    qapp.processEvents()
    assert scene.renderer_info is info
    assert scene.note.text() == "三维 · CPU"
    assert "llvmpipe" in scene.note.toolTip()
    set_language("en", persist=False)
    assert scene.note.text() == "3D · CPU"
    assert "GPU acceleration is not active" in scene.note.toolTip()
    assert "Vendor: Mesa" in scene.note.toolTip()


def test_scene_keeps_fallback_and_ignores_stale_gpu_signal(qapp, simulated_gl_scene):
    scene = simulated_gl_scene
    info = OpenGLRenderer(renderer="NVIDIA GeForce RTX 5060 Ti/PCIe/SSE2")
    scene.viewport.renderer_ready.emit(info)
    qapp.processEvents()
    assert scene.note.text() == "三维 · GPU"
    scene.viewport.failed.emit("test context failure")
    qapp.processEvents()
    assert scene.render_mode == "projection"
    assert scene.renderer_info is None
    assert scene.note.text() == "投影 · CPU"
    assert scene.note.toolTip() == "test context failure"
    scene._set_renderer_info(info)
    assert scene.note.text() == "投影 · CPU"
    assert scene.renderer_info is None
