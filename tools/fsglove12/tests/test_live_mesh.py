"""CPU mesh cache/shading contracts; timings do not measure GPU presentation."""
import json
import os
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

gl = pytest.importorskip("pyqtgraph.opengl", reason="Optional live GUI dependencies unavailable")

from vt_fsglove12.live_mesh import FixedTopologyMeshData, JointVisibilityMeshItem


def tetrahedron():
    vertices = np.array([[1., 1., 1.], [-1., -1., 1.], [-1., 1., -1.], [1., -1., -1.]], dtype=np.float32)
    faces = np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]], dtype=np.uint32)
    colors = np.array([[.2, .6, .5, 1.], [.7, .9, .8, 1.], [.3, .5, .6, 1.], [.6, .8, .7, 1.]], dtype=np.float32)
    return vertices, faces, colors


def test_smooth_normal_weighting_matches_original_and_indexed_views_are_cached():
    vertices, faces, colors = tetrahedron()
    vertices[0] *= [1., 2., .4]  # Unequal face areas catch unit-face weighting mistakes.
    original = gl.MeshData(vertexes=vertices, faces=faces, vertexColors=colors)
    cached = FixedTopologyMeshData(vertices, faces, colors)
    for indexed in (None, "faces"):
        np.testing.assert_allclose(cached.vertexNormals(indexed), original.vertexNormals(indexed), atol=3e-7)
        np.testing.assert_array_equal(cached.faceNormals(indexed), original.faceNormals(indexed))
        np.testing.assert_array_equal(cached.vertexColors(indexed), original.vertexColors(indexed))
    assert cached.vertexNormals("faces") is cached.vertexNormals("faces")
    assert cached.vertexes("faces") is cached.vertexes("faces")
    assert not cached.hasFaceIndexedData()


def test_updates_preserve_topology_adjacency_and_colors_but_replace_geometry():
    vertices, faces, colors = tetrahedron()
    cached = FixedTopologyMeshData(vertices, faces, colors)
    topology, adjacency = cached.faces(), cached.vertexFaces()
    cached_colors, face_colors = cached.vertexColors(), cached.vertexColors("faces")
    old_triangle_view, old_normals = cached.vertexes("faces"), cached.vertexNormals().copy()
    rotation = Rotation.from_euler("XYZ", [.3, -.8, .5])
    changed = rotation.apply(vertices) * [1., .5, 1.3]
    cached.update_vertices(changed)
    original = gl.MeshData(vertexes=changed, faces=faces, vertexColors=colors)
    assert cached.faces() is topology and cached.vertexFaces() is adjacency
    assert cached.vertexColors() is cached_colors and cached.vertexColors("faces") is face_colors
    assert cached.vertexes("faces") is not old_triangle_view
    assert not np.allclose(old_normals, cached.vertexNormals())
    np.testing.assert_allclose(cached.vertexNormals(), original.vertexNormals(), atol=3e-7)
    np.testing.assert_array_equal(cached.vertexes("faces"), original.vertexes("faces"))
    np.testing.assert_array_equal(cached.vertexColors(), colors)


def test_glmeshitem_retains_indexed_smooth_render_path_across_frames():
    vertices, faces, colors = tetrahedron()
    cached = FixedTopologyMeshData(vertices, faces, colors)
    cached.vertexNormals()  # Pre-querying normals must not expand canonical vertices.
    item = gl.GLMeshItem(meshdata=cached, smooth=True)
    item.parseMeshData()  # CPU-only; never call paint or require an OpenGL context.
    assert item.faces is cached.faces() and item.vertexes.shape == (4, 3)
    changed = vertices * [1., .4, 1.2]
    cached.update_vertices(changed)
    item.meshDataChanged()
    item.parseMeshData()
    assert item.faces is cached.faces() and item.colors is cached.vertexColors()
    np.testing.assert_allclose(item.normals, gl.MeshData(vertexes=changed, faces=faces).vertexNormals(), atol=3e-7)
    np.testing.assert_array_equal(item.vertexes, changed.astype(np.float32))


def test_partial_face_visibility_omits_unavailable_geometry_not_alpha_zero_or_neutral_pose():
    vertices, faces, colors = tetrahedron()
    cached = FixedTopologyMeshData(vertices, faces, colors)
    topology = cached.faces()
    item = JointVisibilityMeshItem(meshdata=cached, smooth=True)
    keep = np.asarray([True, False, True, False])
    item.set_face_visibility(keep)
    item.parseMeshData()
    np.testing.assert_array_equal(item.faces, faces[keep])
    assert cached.faces() is topology
    for scale in (1.2, 1.8, .6):
        cached.update_vertices(vertices * scale)
        item.meshDataChanged()
        item.parseMeshData()
        np.testing.assert_array_equal(item.faces, faces[keep])
        np.testing.assert_allclose(item.vertexes, vertices * scale)
    item.set_face_visibility([False] * len(faces))
    item.parseMeshData()
    assert item.faces.shape == (0, 3)
    item.set_face_visibility([True] * len(faces))
    item.parseMeshData()
    assert item.faces is topology


def test_isolated_and_zero_area_vertices_have_finite_zero_normals():
    vertices = np.array([[0., 0., 0.], [1., 0., 0.], [2., 0., 0.], [5., 5., 5.]])
    cached = FixedTopologyMeshData(vertices, np.array([[0, 1, 2]], dtype=int))
    np.testing.assert_array_equal(cached.vertexNormals(), np.zeros((4, 3)))
    assert np.isfinite(cached.vertexNormals()).all()
    empty = FixedTopologyMeshData(vertices, np.empty((0, 3), dtype=int))
    np.testing.assert_array_equal(empty.vertexNormals(), np.zeros((4, 3)))


@pytest.mark.parametrize("bad", [np.zeros((3, 3)), np.zeros((4, 2)), np.full((4, 3), np.nan), np.full((4, 3), 1e100)])
def test_invalid_vertex_update_is_atomic(bad):
    vertices, faces, colors = tetrahedron()
    cached = FixedTopologyMeshData(vertices, faces, colors)
    original_vertices, original_normals = cached.vertexes(), cached.vertexNormals()
    with pytest.raises(ValueError, match="vertexes"):
        cached.update_vertices(bad)
    assert cached.vertexes() is original_vertices and cached.vertexNormals() is original_normals


@pytest.mark.parametrize("faces", [np.array([[0, 1, -1]]), np.array([[0, 1, 4]]),
                                   np.array([[0., 1., 2.]]), np.array([[0, 1]])])
def test_invalid_topology_is_rejected(faces):
    with pytest.raises(ValueError, match="faces"):
        FixedTopologyMeshData(tetrahedron()[0], faces)


def test_caller_cannot_mutate_initial_inputs_or_change_fixed_topology():
    vertices, faces, colors = tetrahedron()
    cached = FixedTopologyMeshData(vertices, faces, colors)
    vertices[:] = 0; faces[:] = 0; colors[:] = 0
    expected_vertices, expected_faces, expected_colors = tetrahedron()
    np.testing.assert_array_equal(cached.vertexes(), expected_vertices)
    np.testing.assert_array_equal(cached.faces(), expected_faces)
    np.testing.assert_array_equal(cached.vertexColors(), expected_colors)
    with pytest.raises(ValueError):
        cached.faces()[0, 0] = 3
    with pytest.raises(ValueError, match="fixed"):
        cached.setFaces(expected_faces)
    with pytest.raises(ValueError, match="face-indexed"):
        cached.setVertexes(expected_vertices, indexed="faces")
    changed = expected_vertices * 2
    cached.setVertexes(changed, resetNormals=False)  # Must still invalidate stale normals.
    np.testing.assert_array_equal(cached.vertexes(), changed)


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_MANO_ASSETS"), reason="No explicit real MANO assets")
def test_real_mano_cpu_mesh_update_matches_reference_and_reports_cost(record_property):
    from vt_fsglove12.mano import ManoBackend

    backend = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    poses = []
    for bend in (0., .3, .7):
        angles = np.zeros((16, 3))
        for proximal in (1, 4, 7, 10):
            angles[proximal, 2] = bend
            angles[proximal + 1, 2] = bend * 1.2
            angles[proximal + 2, 2] = bend * .78
        angles[14, 2], angles[15, 2] = bend * .7, bend
        poses.append(backend.render(angles))
    vertices = [np.asarray(pose["vertices_m"]) for pose in poses]
    faces = np.asarray(poses[0]["faces"], dtype=np.uint32)
    assert vertices[0].shape == (778, 3) and faces.shape == (1538, 3)
    colors = np.random.default_rng(12).uniform(.2, 1., (778, 4)).astype(np.float32)
    cached = FixedTopologyMeshData(vertices[0], faces, colors)
    for positions in vertices:
        cached.update_vertices(positions)
        reference = gl.MeshData(vertexes=positions, faces=faces, vertexColors=colors)
        np.testing.assert_allclose(cached.vertexNormals(), reference.vertexNormals(), atol=2e-6)
        np.testing.assert_array_equal(cached.faceNormals(), reference.faceNormals())
    item = gl.GLMeshItem(meshdata=cached, smooth=True)
    reference_item = gl.GLMeshItem(vertexes=vertices[0], faces=faces, vertexColors=colors, smooth=True)
    slow, fast = [], []
    for index in range(24):
        positions = vertices[index % len(vertices)]
        started = time.perf_counter()
        reference_item.setMeshData(vertexes=positions, faces=faces, vertexColors=colors)
        reference_item.parseMeshData()
        slow.append((time.perf_counter() - started) * 1000.)
        started = time.perf_counter()
        cached.update_vertices(positions)
        item.meshDataChanged()
        item.parseMeshData()
        fast.append((time.perf_counter() - started) * 1000.)
    metrics = dict(cpu_only_no_glpaint=True, frames=24, vertices=778, faces=1538,
                   reference_mean_ms=float(np.mean(slow)), reference_p95_ms=float(np.percentile(slow, 95)),
                   cached_mean_ms=float(np.mean(fast)), cached_p95_ms=float(np.percentile(fast, 95)),
                   median_speedup=float(np.median(slow) / np.median(fast)))
    print("LIVE_MESH_METRICS=" + json.dumps(metrics, sort_keys=True))
    record_property("cpu_mesh_metrics_json", json.dumps(metrics, sort_keys=True))
    # Timing is reported, not made a machine-load-dependent CI correctness gate.
    assert cached.faces() is item.faces and item.vertexes.shape == (778, 3)
