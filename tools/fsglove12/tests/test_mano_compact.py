"""Compact live surfaces: exact geometry, immutable ownership and IPC parity."""
from copy import deepcopy
import json
import multiprocessing as mp
import os
from pathlib import Path
import pickle
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from test_mano import mocked_backend
from test_pose_export import inputs as export_inputs
from vt_fsglove12 import mano
from vt_fsglove12.config import load_config
from vt_fsglove12.live_process import _child_main, _idle, _publish
from vt_fsglove12.pose_export import build_state


def assert_surface_contract(geometry):
    for key, shape in (("vertices_m", (778, 3)), ("faces", (1, 3))):
        value = geometry[key]
        assert isinstance(value, np.ndarray) and value.shape == shape
        assert value.flags.c_contiguous and not value.flags.writeable
        with pytest.raises(ValueError):
            value[0, 0] = 99
    assert isinstance(geometry["joints_m"], list) and len(geometry["joints_m"]) == 21
    assert isinstance(geometry["mano_pose_rotvec"], list)
    json.dumps({key: value for key, value in geometry.items() if key not in {"vertices_m", "faces"}})


@pytest.mark.parametrize("method", ["render", "render_mano"])
def test_compact_surface_matches_default_without_changing_public_default(mocked_backend, method):
    regular, _ = mocked_backend
    compact = mano.ManoBackend(regular.assets_root, compact_live_geometry=True)
    angles = np.linspace(-.2, .4, 48).reshape(16, 3)
    expected = getattr(regular, method)(angles)
    actual = getattr(compact, method)(angles)
    assert not regular.compact_live_geometry
    assert isinstance(expected["vertices_m"], list) and isinstance(expected["faces"], list)
    json.dumps(expected)
    assert_surface_contract(actual)
    for key in ("vertices_m", "faces", "joints_m", "mano_pose_rotvec"):
        np.testing.assert_array_equal(actual[key], expected[key])
    assert actual["provenance"] == expected["provenance"]
    another = getattr(compact, method)(angles + .1)
    assert another["faces"] is actual["faces"]
    assert not np.shares_memory(another["vertices_m"], actual["vertices_m"])
    np.testing.assert_array_equal(actual["vertices_m"], expected["vertices_m"])


def test_compact_arrays_never_alias_tensor_backing_and_remain_readonly_after_copy(mocked_backend):
    regular, _ = mocked_backend
    compact = mano.ManoBackend(regular.assets_root, compact_live_geometry=True)
    vertices = np.arange(778 * 3, dtype=np.float32).reshape(778, 3)
    faces = np.array([[0, 1, 2]], dtype=np.int64)
    surface, triangles = compact._surface_output(vertices, faces)
    expected_vertices, expected_faces = vertices.copy(), faces.copy()
    vertices[:] = -1
    faces[:] = 0
    np.testing.assert_array_equal(surface, expected_vertices)
    np.testing.assert_array_equal(triangles, expected_faces)
    for original in (surface, triangles):
        for copied in (deepcopy(original), pickle.loads(pickle.dumps(original, protocol=pickle.HIGHEST_PROTOCOL))):
            assert isinstance(copied, np.ndarray) and not copied.flags.writeable
            assert not np.shares_memory(copied, original)
            np.testing.assert_array_equal(copied, original)


def test_compact_snapshot_deepcopy_and_actual_mailbox_pickle_preserve_geometry(mocked_backend):
    regular, _ = mocked_backend
    compact = mano.ManoBackend(regular.assets_root, compact_live_geometry=True)
    geometry = compact.render_mano(np.zeros((16, 3)))
    snapshot = _idle()
    snapshot["geometry"] = snapshot["display_geometry"] = geometry
    copied = deepcopy(snapshot)
    assert copied["geometry"] is copied["display_geometry"]
    assert not np.shares_memory(copied["geometry"]["vertices_m"], geometry["vertices_m"])
    context = mp.get_context("spawn")
    mailbox = (context.RawArray("B", 128 * 1024), context.RawValue("I", 0),
               context.RawValue("Q", 0), context.Lock())
    assert _publish(mailbox, copied, 7)
    payload = bytes(memoryview(mailbox[0]).cast("B")[:mailbox[1].value])
    _, command_seen, received = pickle.loads(payload)
    assert command_seen == 7 and mailbox[2].value == 1
    assert_surface_contract(received["geometry"])
    assert received["geometry"] is received["display_geometry"]
    for key in ("vertices_m", "faces", "joints_m", "mano_pose_rotvec"):
        np.testing.assert_array_equal(received["geometry"][key], geometry[key])


def test_compact_geometry_keeps_results_only_ros_joints_json_serializable(mocked_backend):
    regular, _ = mocked_backend
    compact = mano.ManoBackend(regular.assets_root, compact_live_geometry=True)
    frame, pose, _ = export_inputs()
    geometry = compact.render_mano(pose["mano_pose_rotvec"])
    config = load_config(Path(mano.__file__).parent / "configs" / "right_current.json")
    result = build_state(frame, pose, geometry, config, root_node="12",
                         solve_completed_monotonic_ns=1_030_000_000)
    assert result["joint_positions_m"] == np.asarray(geometry["joints_m"]).reshape(-1).tolist()
    assert len(result["joint_positions_m"]) == 63
    assert "vertices_m" not in json.dumps(result)


def test_production_child_enables_compact_only_for_default_real_factory(monkeypatch):
    calls = []

    def factory(*args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(export_rest=lambda: None)

    monkeypatch.setattr(mano, "ManoBackend", factory)
    stop = threading.Event()
    stop.set()  # Initialization only; never create a runtime, device or GUI.
    options = dict(mano_assets="unused", hand="right")
    _child_main({}, {}, {}, options, None, stop, None, None, None, None)
    assert calls[-1] == (("unused",), {"hand": "right", "compact_live_geometry": True})
    _child_main({}, {}, {}, options, None, stop, None, None, None, factory)
    assert calls[-1] == (("unused",), {"hand": "right"})


def test_compact_readonly_arrays_are_accepted_by_live_mesh_and_hand_view(mocked_backend, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PyQt5")
    pytest.importorskip("pyqtgraph.opengl")
    from PyQt5 import QtWidgets
    from vt_fsglove12.live_mesh import FixedTopologyMeshData
    from vt_fsglove12.live_ui import HandView
    regular, _ = mocked_backend
    compact = mano.ManoBackend(regular.assets_root, compact_live_geometry=True)
    neutral = compact.render_mano(np.zeros((16, 3)))
    posed = compact.render_mano(np.full((16, 3), .1))
    mesh = FixedTopologyMeshData(neutral["vertices_m"], neutral["faces"])
    mesh.update_vertices(posed["vertices_m"])
    np.testing.assert_array_equal(mesh.vertexes(), np.asarray(posed["vertices_m"], dtype=np.float32))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    axes = dict(roll_axis=[0., 1., 0.], pitch_axis=[-1., 0., 0.], palm_up_axis=[0., 0., 1.])
    view = HandView(neutral, axes)
    try:
        view.set_geometry(posed)
        np.testing.assert_allclose(view.mesh_data.vertexes(),
                                   np.asarray(posed["vertices_m"]) @ view.basis.T, atol=1e-7)
    finally:
        view.close()
        app.processEvents()


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_MANO_ASSETS"), reason="No explicit licensed MANO assets")
def test_real_compact_and_list_mano_outputs_are_bitwise_equal():
    regular = mano.ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    compact = mano.ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"], compact_live_geometry=True)
    regular.export_rest()
    regular._torch.set_num_threads(1)
    for bend in (0., .3, .7):
        pose = np.zeros((16, 3))
        pose[0] = [.2, -.3, .4]
        pose[[2, 5, 8, 11], 2] = bend
        betas = np.linspace(-.1, .1, 10)
        expected, actual = regular.render_mano(pose, betas), compact.render_mano(pose, betas)
        assert actual["vertices_m"].shape == (778, 3) and actual["faces"].shape == (1538, 3)
        for key in ("vertices_m", "faces", "joints_m", "mano_pose_rotvec"):
            np.testing.assert_array_equal(actual[key], expected[key])
        assert not actual["vertices_m"].flags.writeable and not actual["faces"].flags.writeable
