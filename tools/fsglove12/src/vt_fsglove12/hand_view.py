"""Shared MANO surface renderer for calibration and read-only workbench views.

The embedding application selects its Qt binding before importing this module.
Only pyqtgraph's selected binding is used here; importing the renderer does not
import the calibration window, a model backend, ROS, or a hardware controller.
It consumes already prepared geometry and display colors, never estimates pose.
"""
from __future__ import annotations

import numpy as np
from OpenGL.GL import (GL_BLEND, GL_DEPTH_TEST, glDeleteProgram, glDeleteTextures,
                       glIsProgram, glIsTexture)
from pyqtgraph.Qt import QtGui
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import shaders

from .live_display import display_basis, neutral_geometry_slot_map
from .live_mesh import FixedTopologyMeshData


def _local_shader(parts):
    """Create an uncached program: GL names belong to one widget context.

    pyqtgraph 0.13 registers every constructor call, including name=None. Use
    a temporary unique key and remove only that entry, preserving its built-in
    default shader and programs used by unrelated renderers.
    """
    key = object()
    program = shaders.ShaderProgram(key, parts)
    shaders.ShaderProgram.names.pop(key)
    return program


def _surface_shader():
    return _local_shader([
        shaders.VertexShader("""
            varying vec3 normal;
            void main() {
                normal = normalize(gl_NormalMatrix * gl_Normal);
                gl_FrontColor = gl_Color; gl_BackColor = gl_Color;
                gl_Position = ftransform();
            }
        """),
        shaders.FragmentShader("""
            varying vec3 normal;
            void main() {
                float diffuse = max(0., dot(normalize(normal), normalize(vec3(.3, .4, 1.))));
                gl_FragColor = vec4(gl_Color.rgb * (.65 + .35 * diffuse), gl_Color.a);
            }
        """),
    ])


class HandView(gl.GLViewWidget):
    """Real MANO surface with persistent palmar/dorsal colors and proper basis."""

    def __init__(self, neutral_geometry, model_axes, parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 430)
        self.setBackgroundColor("#e9f0f2")
        self.basis = display_basis(model_axes)
        self.faces = np.asarray(neutral_geometry["faces"], dtype=int)
        original = np.asarray(neutral_geometry["vertices_m"])
        normals = gl.MeshData(vertexes=original, faces=self.faces).vertexNormals()
        normals = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        dorsal = .5 + .5 * np.clip(normals @ np.asarray(model_axes["palm_up_axis"]) / .4, -1, 1)
        self.colors = (dorsal[:, None] * np.array([.25, .62, .57, 1.])
                       + (1 - dorsal[:, None]) * np.array([.71, .91, .86, 1.]))
        self.vertex_slots, self.joint_slots = neutral_geometry_slot_map(neutral_geometry)
        self.tactile_binding = None
        self._surface_colors = self.colors.copy()
        self._joint_style = None
        self._joint_colors = np.tile([1., .68, .20, 1.], (21, 1))
        self._joint_visible = np.ones(21, dtype=bool)
        self._shader_context = None
        self.mesh_data = FixedTopologyMeshData(original @ self.basis.T, self.faces,
                                               vertexColors=self.colors)
        self.mesh = gl.GLMeshItem(meshdata=self.mesh_data, smooth=True,
                                  shader=_surface_shader(), drawEdges=False)
        self.addItem(self.mesh)
        self.joints = gl.GLScatterPlotItem(pos=np.zeros((21, 3)), size=5., color=(1., .68, .20, 1.))
        # An explicit X-ray annotation, not a claim that joint centres lie on skin.
        self.joints.setGLOptions({GL_DEPTH_TEST: False, GL_BLEND: False})
        self.addItem(self.joints)
        self.show_joints = True
        self.reset_camera()
        self.set_geometry(neutral_geometry)

    def initializeGL(self):
        # Top-level windows need not share contexts. Reparenting a QOpenGLWidget
        # can also recreate its context; old integer handles must not survive.
        self.mesh.setShader(_surface_shader())
        self.joints.shader = None
        self.joints.pointTexture = None
        self.joints.initialize()
        point_program = shaders.getShaderProgram("pointSprite")
        self.joints.shader = _local_shader([
            shaders.Shader(part.shaderType, part.code) for part in point_program.shaders
        ])
        super().initializeGL()
        self._shader_context = self.context()
        self._shader_context.aboutToBeDestroyed.connect(self._release_gl_resources)

    def _release_gl_resources(self):
        """Release only handles belonging to the context being destroyed."""
        context = self._shader_context
        if context is None:
            return
        # Qt emits this before context destruction. Keep GPU deletion inside
        # that exact current context, never a different widget's name space.
        if context is self.context() and context.isValid():
            self.makeCurrent()
            if QtGui.QOpenGLContext.currentContext() is context:
                for program in (self.mesh.shader(), self.joints.shader):
                    if program is not None and program.prog is not None and program.prog > 0:
                        if glIsProgram(program.prog):
                            glDeleteProgram(program.prog)
                        program.prog = None
                texture = getattr(self.joints, "pointTexture", None)
                if texture is not None and glIsTexture(texture):
                    glDeleteTextures([texture])
                self.doneCurrent()
        self._shader_context = None

    def reset_camera(self, palmar=False):
        self.opts["center"] = QtGui.QVector3D(0., .085, 0.)
        self.setCameraPosition(distance=.33, elevation=-65 if palmar else 65, azimuth=-90)

    def set_geometry(self, geometry):
        # Validate the complete candidate before replacing any previously good
        # geometry. A partial/NaN update must never corrupt the frozen view.
        vertices = np.asarray(geometry["vertices_m"], dtype=float)
        joints = np.asarray(geometry["joints_m"], dtype=float)
        if (vertices.shape != (len(self.vertex_slots), 3) or joints.shape != (21, 3)
                or not np.isfinite(vertices).all() or not np.isfinite(joints).all()):
            raise ValueError("display geometry must contain finite fixed-size vertices and joints")
        self.mesh_data.update_vertices(vertices @ self.basis.T)
        self.mesh.meshDataChanged()
        self._joint_positions = joints @ self.basis.T
        self.joints.setData(pos=self._joint_positions[self._joint_visible],
                            color=self._joint_colors[self._joint_visible])
        self.set_hand_visible(True)

    def set_joint_quality(self, provisional, hidden=(False,) * 16):
        """Compatibility seam: quality is now text-only, never surface deletion."""
        if len(provisional) != 16 or len(hidden) != 16:
            raise ValueError("joint presentation requires 16 flags")

    def set_surface_colors(self, colors=None):
        """Replace display-only RGBA colors, retaining geometry and base styling.

        ``None`` restores the original palm/back colors. Validation precedes
        mutation, so malformed input leaves the last complete display intact.
        The caller is responsible for the meaning of its palette (raw ADC or
        baseline difference); this renderer does not reinterpret measurements.
        """
        candidate = self.colors if colors is None else np.asarray(colors, dtype=float)
        if (candidate.shape != self.colors.shape or not np.isfinite(candidate).all()
                or np.any(candidate < 0.) or np.any(candidate > 1.)):
            raise ValueError("surface colors must be finite fixed-size RGBA values in 0..1")
        if np.array_equal(candidate, self._surface_colors):
            return False
        self._surface_colors = candidate.copy()
        self.mesh_data.setVertexColors(self._surface_colors)
        self.mesh.meshDataChanged()
        return True

    def set_tactile(self, delta=None, valid_mask=None, *, limit=3000.):
        """Display-only vertex colors: never replace geometry or base styling."""
        colors = self.colors
        if self.tactile_binding is not None and delta is not None and valid_mask is not None:
            colors = self.tactile_binding.vertex_colors(delta, valid_mask, self.colors, limit=limit)
        self.set_surface_colors(colors)

    def set_show_joints(self, value):
        """Toggle the existing X-ray markers without changing hand visibility."""
        self.show_joints = bool(value)
        self.joints.setVisible(self.show_joints and self.mesh.visible())

    def set_hand_visible(self, value):
        self.mesh.setVisible(bool(value))
        self.joints.setVisible(bool(value) and self.show_joints)
