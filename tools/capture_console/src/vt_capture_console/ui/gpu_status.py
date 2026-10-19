"""Identify the renderer of a current GL context without probing other devices.

Creating an OpenGL context alone does not establish GPU acceleration: Mesa and
other implementations can render on the CPU. Unknown or virtual renderers stay
unconfirmed rather than being reported as hardware accelerated.
"""
from __future__ import annotations

from dataclasses import dataclass


# OpenGL's fixed string query identifiers; importing PyOpenGL here would select
# a process-wide GLX/EGL platform even when the caller only uses Qt OpenGL.
GL_VENDOR = 0x1F00
GL_RENDERER = 0x1F01
GL_VERSION = 0x1F02


_SOFTWARE_RENDERERS = (
    "llvmpipe", "softpipe", "swrast", "software rasterizer", "software renderer",
    "swiftshader", "lavapipe", "openswr", "gdi generic", "basic render driver",
    "warp rasterizer",
)
_VIRTUAL_RENDERERS = ("virgl", "svga3d", "virtualbox", "vmware", "chromium")
_HARDWARE_RENDERERS = (
    "nvidia", "geforce", "quadro", "tesla", "radeon", "firepro", "intel",
    "apple m", "apple gpu", "adreno", "mali-", "powervr", "panfrost", "v3d",
)


def classify_renderer(renderer: str) -> str:
    """Conservatively classify an actual GL_RENDERER string, never its vendor.

Software markers take priority because, for example, ANGLE may report both a
physical adapter name and a software implementation in the same string.
"""
    value = renderer.casefold()
    if any(marker in value for marker in _SOFTWARE_RENDERERS):
        return "software"
    if any(marker in value for marker in _VIRTUAL_RENDERERS):
        return "unknown"
    if any(marker in value for marker in _HARDWARE_RENDERERS):
        return "hardware"
    return "unknown"


@dataclass(frozen=True)
class OpenGLRenderer:
    renderer: str = ""
    vendor: str = ""
    version: str = ""
    error: str = ""

    @property
    def acceleration(self) -> str:
        return classify_renderer(self.renderer)


def _gl_text(value) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").strip()
    return value.strip() if isinstance(value, str) else ""


def inspect_current_context() -> OpenGLRenderer:
    """Read once in initializeGL, where Qt already made this context current.

    This does not create a context, select a GPU, import PyOpenGL, or run an
    external command. A diagnostic failure must not disable a working viewport.
    """
    try:
        from PySide6.QtGui import QOpenGLContext
        context = QOpenGLContext.currentContext()
        if context is None:
            return OpenGLRenderer(error="No current OpenGL context")
        get_string = context.functions().glGetString
        return OpenGLRenderer(
            renderer=_gl_text(get_string(GL_RENDERER)),
            vendor=_gl_text(get_string(GL_VENDOR)),
            version=_gl_text(get_string(GL_VERSION)),
        )
    except Exception as exc:
        return OpenGLRenderer(error=str(exc))
