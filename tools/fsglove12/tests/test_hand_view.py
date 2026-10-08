"""Shared renderer contracts; no hardware, model load, or GL paint required."""
from copy import deepcopy
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

pytest.importorskip("pyqtgraph.opengl", reason="Optional live GUI dependencies unavailable")
from pyqtgraph.Qt import QtWidgets

from vt_fsglove12.hand_view import HandView


def neutral_geometry():
    joints = np.zeros((21, 3))
    for finger, base in enumerate((1, 5, 9, 13, 17)):
        joints[base:base + 4] = [[(finger - 2) * .02, .04 + i * .025, 0.] for i in range(4)]
    return {"vertices_m": [[0., 0., 0.], [-.1, 0., 0.], [0., .1, 0.]],
            "faces": [[0, 1, 2]], "joints_m": joints}


@pytest.fixture
def view(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = HandView(neutral_geometry(), {"hand": "right", "roll_axis": [0., 1., 0.],
        "pitch_axis": [-1., 0., 0.], "palm_up_axis": [0., 0., 1.]})
    try:
        yield widget
    finally:
        widget.close()
        app.processEvents()


def test_surface_colors_preserve_geometry_topology_and_original_style(view):
    vertices = view.mesh_data.vertexes().copy()
    topology, mesh = view.mesh_data.faces(), view.mesh
    base = view.colors.copy()
    overlay = np.tile([1., .3, .1, 1.], (3, 1))
    assert view.set_surface_colors(overlay)
    assert not view.set_surface_colors(overlay)
    np.testing.assert_array_equal(view.colors, base)
    np.testing.assert_allclose(view.mesh_data.vertexColors(), overlay)
    np.testing.assert_array_equal(view.mesh_data.vertexes(), vertices)
    assert view.mesh is mesh and view.mesh_data.faces() is topology
    overlay[:] = 0.  # The caller cannot mutate an accepted color buffer.
    assert np.any(view._surface_colors)
    assert view.set_surface_colors()
    np.testing.assert_array_equal(view._surface_colors, base)
    assert not view.set_surface_colors()


@pytest.mark.parametrize("bad", [np.zeros((2, 4)), np.full((3, 4), np.nan),
    np.full((3, 4), np.inf), np.full((3, 4), -0.1), np.full((3, 4), 1.1)])
def test_bad_colors_preserve_last_complete_surface(view, bad):
    view.set_surface_colors(np.tile([1., .3, .1, 1.], (3, 1)))
    accepted = view._surface_colors.copy()
    with pytest.raises(ValueError, match="surface colors"):
        view.set_surface_colors(bad)
    np.testing.assert_array_equal(view._surface_colors, accepted)
    np.testing.assert_allclose(view.mesh_data.vertexColors(), accepted)


def test_geometry_and_camera_api_remains_calibration_compatible(view):
    view.reset_camera(palmar=True)
    assert view.opts["elevation"] == -65 and view.opts["azimuth"] == -90
    view.reset_camera()
    assert view.opts["elevation"] == 65
    candidate = deepcopy(neutral_geometry())
    candidate["vertices_m"] = np.asarray(candidate["vertices_m"]) + .01
    view.set_geometry(candidate)
    np.testing.assert_allclose(view.mesh_data.vertexes(), candidate["vertices_m"], atol=1e-8)
    accepted = view.mesh_data.vertexes().copy()
    candidate["joints_m"][0, 0] = np.nan
    with pytest.raises(ValueError, match="display geometry"):
        view.set_geometry(candidate)
    np.testing.assert_array_equal(view.mesh_data.vertexes(), accepted)


def test_joint_toggle_does_not_restore_hidden_hand(view):
    view.set_show_joints(False)
    assert view.mesh.visible() and not view.joints.visible()
    view.set_show_joints(True)
    assert view.joints.visible()
    view.set_hand_visible(False)
    view.set_show_joints(True)
    assert not view.mesh.visible() and not view.joints.visible()
    view.set_hand_visible(True)
    assert view.joints.visible()


def test_tactile_delta_path_reuses_surface_api_and_preserves_base(view):
    base = view.colors.copy()

    class Binding:
        def vertex_colors(self, delta, mask, colors, *, limit):
            assert delta.shape == mask.shape == (20, 20)
            assert limit == 1200.
            result = colors.copy()
            result[0] = [1., .3, .1, 1.]
            return result

    view.tactile_binding = Binding()
    view.set_tactile(np.ones((20, 20)), np.ones((20, 20), bool), limit=1200.)
    assert not np.array_equal(view._surface_colors, base)
    np.testing.assert_array_equal(view.colors, base)
    view.set_tactile()
    np.testing.assert_array_equal(view._surface_colors, base)


def test_shader_programs_are_private_per_view_not_named_global_cache(view):
    from pyqtgraph.opengl import shaders

    registered = dict(shaders.ShaderProgram.names)
    other = HandView(neutral_geometry(), {"hand": "right", "roll_axis": [0., 1., 0.],
        "pitch_axis": [-1., 0., 0.], "palm_up_axis": [0., 0., 1.]})
    try:
        assert other.mesh.shader() is not view.mesh.shader()
        assert other.mesh.shader().shaders[0] is not view.mesh.shader().shaders[0]
        assert shaders.ShaderProgram.names == registered
        assert view.mesh.shader() not in registered.values()
    finally:
        other.close()


@pytest.mark.skipif(importlib.util.find_spec("PySide6") is None, reason="PySide6 not installed")
def test_pyside6_renderer_never_imports_pyqt5_or_live_runtime():
    # A separate interpreter prevents another test's Qt choice from interfering.
    source_root = Path(__file__).resolve().parents[1] / "src"
    script = f"""
import sys
sys.path.insert(0, {str(source_root)!r})
from PySide6 import QtWidgets
from vt_fsglove12.hand_view import HandView
from pyqtgraph.Qt import QT_LIB
assert QT_LIB == 'PySide6', QT_LIB
assert issubclass(HandView, QtWidgets.QWidget)
assert not any(name == 'PyQt5' or name.startswith('PyQt5.') for name in sys.modules)
assert 'vt_fsglove12.live_ui' not in sys.modules
assert 'vt_fsglove12.live_runtime' not in sys.modules
assert 'vt_fsglove12.mano' not in sys.modules
assert 'torch' not in sys.modules
assert 'rclpy' not in sys.modules
assert 'serial' not in sys.modules
"""
    result = subprocess.run([sys.executable, "-I", "-c", script],
        env=dict(os.environ, QT_QPA_PLATFORM="offscreen", PYQTGRAPH_QT_LIB="PySide6"),
        text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(os.environ.get("FSGLOVE12_TEST_OPENGL") != "1",
                   reason="Explicit X11 OpenGL smoke test; never opens windows in ordinary tests")
def test_two_top_level_contexts_and_reparent_keep_valid_colored_surface():
    source_root = Path(__file__).resolve().parents[1] / "src"
    script = f"""
import sys, time
sys.path.insert(0, {str(source_root)!r})
import numpy as np
from pyqtgraph.Qt import QtWidgets, QtGui
from OpenGL.GL import glGetError, glIsProgram
from vt_fsglove12.hand_view import HandView
app = QtWidgets.QApplication([])
geometry = dict(vertices_m=[[0.,0.,0.],[-.1,0.,0.],[0.,.1,0.]],
                faces=[[0,1,2]], joints_m=np.zeros((21,3)))
axes = dict(hand='right', roll_axis=[0.,1.,0.],pitch_axis=[-1.,0.,0.],palm_up_axis=[0.,0.,1.])
parents, views = [], []
for i in range(2):
    parent = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(parent)
    view = HandView(geometry, axes)
    view.opts['center'] = QtGui.QVector3D(-.04,.04,0.)
    view.setCameraPosition(distance=.25, elevation=90., azimuth=-90.)
    layout.addWidget(view)
    parent.show()
    parents.append(parent)
    views.append(view)
def paint_and_check():
    for _ in range(15):
        app.processEvents()
        time.sleep(.01)
    for view in views:
        image = view.grabFramebuffer()
        assert not image.isNull()
        view.makeCurrent()
        assert glIsProgram(view.mesh.shader().prog)
        assert glIsProgram(view.joints.shader.prog)
        assert glGetError() == 0
        view.doneCurrent()
        image = image.convertToFormat(QtGui.QImage.Format.Format_RGBA8888)
        data = image.constBits()
        if hasattr(data, 'setsize'):
            data.setsize(image.sizeInBytes())
        pixels = np.frombuffer(data, dtype=np.uint8).reshape(image.height(), image.bytesPerLine())
        pixels = pixels[:, :image.width()*4].reshape(image.height(),image.width(),4)
        assert np.count_nonzero(np.any(pixels[:,:,:3] != pixels[0,0,:3], axis=2)) > 1000
paint_and_check()
assert views[0].mesh.shader() is not views[1].mesh.shader()
assert views[0].joints.shader is not views[1].joints.shader
old_context = views[0].context()
parents[0].layout().removeWidget(views[0])
parents[1].layout().addWidget(views[0])
views[0].show()
paint_and_check()
assert views[0].context() is not old_context
for parent in parents:
    parent.close()
    parent.deleteLater()
app.processEvents()
print('MULTI_CONTEXT_GL_PASS')
"""
    result = subprocess.run([sys.executable, "-I", "-c", script],
        env=dict(os.environ, QT_QPA_PLATFORM="xcb"),
        text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "MULTI_CONTEXT_GL_PASS" in result.stdout
    assert "Error while drawing" not in result.stdout + result.stderr
