"""No hardware/assets needed; optional MANO checks use explicitly supplied files."""
from copy import deepcopy
import json
import os
from pathlib import Path

import numpy as np
import pytest

from vt_fsglove12.tactile_surface import TactileSurfaceBinding


AXES = {"roll_axis": [0., 1., 0.], "pitch_axis": [-1., 0., 0.], "palm_up_axis": [0., 0., 1.]}


def fixture_geometry(*, palmar=True, dorsal=True):
    joints = np.zeros((21, 3))
    fingers = [(-.044, .027, [-.8, .6, 0.]), (-.028, .070, [0., 1., 0.]),
               (-.009, .075, [0., 1., 0.]), (.012, .070, [0., 1., 0.]),
               (.031, .058, [0., 1., 0.])]
    vertices, faces = [], []

    def strip(origin, direction, length, width, nrows):
        direction = np.asarray(direction)
        lateral = np.cross(direction, [0., 0., 1.])
        if lateral[0] > 0:
            lateral = -lateral
        for z, flip in ((-.004, False), (.004, True)):
            if (not palmar and z < 0) or (not dorsal and z > 0):
                continue
            start = len(vertices)
            for progress in np.linspace(0., length, nrows):
                for across in np.linspace(-width / 2, width / 2, 5):
                    vertices.append(np.asarray(origin) + direction * progress + lateral * across + [0., 0., z])
            for row in range(nrows - 1):
                for col in range(4):
                    a = start + row * 5 + col
                    for face in ([a, a + 1, a + 5], [a + 1, a + 6, a + 5]):
                        faces.append(face[::-1] if flip else face)

    strip([0., .006, 0.], [0., 1., 0.], .050, .077, 8)
    for finger, (x, y, direction) in enumerate(fingers):
        base = 1 + 4 * finger
        for offset, progress in enumerate((0., .020, .038, .053)):
            joints[base + offset] = [x, y, 0.] + np.asarray(direction) * progress
        strip([x, y, 0.], direction, .053, .014, 13)
    return {"vertices_m": np.asarray(vertices), "faces": np.asarray(faces, dtype=int),
            "joints_m": joints, "provenance": {"hand": "right"}}


@pytest.fixture(scope="module")
def sample():
    geometry = fixture_geometry()
    return geometry, TactileSurfaceBinding(geometry, AXES)


def test_candidate_addresses_anchors_and_convex_barycentric_bindings(sample):
    geometry, binding = sample
    assert binding.addresses.shape == (400, 2)
    np.testing.assert_array_equal(binding._raw_indices, np.arange(400))
    assert binding.metadata()["rows"] == binding.metadata()["cols"] == 20
    assert binding.metadata()["tactile_state_schema_version"] == 2
    np.testing.assert_array_equal(binding.addresses, [(r, c) for r in range(20) for c in range(20)])
    assert binding.verified_anchor.sum() == 9
    assert binding.valid.sum() > 300
    assert all(binding.metadata()["region_counts"].values())
    assert binding.metadata()["status"] == "candidate"
    assert binding.metadata()["verified_skin_coordinates"] is False
    assert np.all(binding.weights >= 0)
    np.testing.assert_allclose(binding.weights[binding.valid].sum(axis=1), 1, atol=1e-12)
    np.testing.assert_array_equal(binding.weights[~binding.valid], 0)
    positions = binding.positions(geometry["vertices_m"])
    assert np.isfinite(positions[binding.valid]).all()
    assert np.isnan(positions[~binding.valid]).all()
    assert np.all(positions[binding.valid, 2] < 0)


def test_fixed_binding_rigid_motion_and_independent_finger_movement(sample):
    geometry, binding = sample
    vertices = geometry["vertices_m"]
    original = binding.positions(vertices)
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    shift = np.array([.12, -.03, .2])
    moved = binding.positions(vertices @ rotation.T + shift)
    np.testing.assert_allclose(moved[binding.valid], original[binding.valid] @ rotation.T + shift, atol=1e-12)
    modified = vertices.copy()
    modified[binding.vertex_regions == 2] += [.04, -.03, .018]
    finger_moved = binding.positions(modified)
    expected = original.copy()
    expected[binding.regions == 2] += [.04, -.03, .018]
    np.testing.assert_allclose(finger_moved, expected, atol=1e-12, equal_nan=True)


def test_binding_and_color_support_never_cross_fingers_or_dorsal_surface(sample):
    geometry, binding = sample
    for index in np.flatnonzero(binding.valid):
        vertices = geometry["faces"][binding.face_ids[index]]
        assert np.all(binding.vertex_regions[vertices] == binding.regions[index])
        assert np.all(binding.palmar_vertices[vertices])
    base = np.tile([.4, .65, .55, 1.], (len(geometry["vertices_m"]), 1))
    delta = np.zeros((20, 20)); delta[8:20, 12:16] = 5000
    mask = np.ones((20, 20), dtype=bool)
    colored = binding.vertex_colors(delta, mask, base)
    changed = np.any(np.abs(colored - base) > 1e-6, axis=1)
    assert changed.any()
    assert np.all(binding.vertex_regions[changed] == 2)
    assert np.all(binding.palmar_vertices[changed])
    assert np.all(colored[changed, 0] > base[changed, 0])
    negative = binding.vertex_colors(-delta, mask, base)
    assert np.all(negative[changed, 2] > base[changed, 2])
    # Neutral or invalid compact-grid data restores the original skin.
    np.testing.assert_allclose(binding.vertex_colors(delta, ~mask, base), base, atol=1e-7)
    delta[:] = 200
    np.testing.assert_allclose(binding.vertex_colors(delta, mask, base), base, atol=1e-7)
    delta[:] = 0
    np.testing.assert_allclose(binding.vertex_colors(delta, mask, base), base, atol=1e-7)


def test_missing_palmar_faces_stays_unbound_instead_of_using_dorsal_faces():
    geometry = fixture_geometry(palmar=False)
    binding = TactileSurfaceBinding(geometry, AXES)
    assert not binding.valid.any()
    assert np.all(binding.face_ids == -1)
    assert np.isnan(binding.positions(geometry["vertices_m"])).all()
    assert not binding.vertex_weights.any()
    assert binding.metadata()["unbound_count"] == 400


def test_no_faces_and_degenerate_faces_do_not_create_binding():
    geometry = fixture_geometry()
    for faces in (np.empty((0, 3), dtype=int), np.zeros((2, 3), dtype=int)):
        geometry["faces"] = faces
        binding = TactileSurfaceBinding(geometry, AXES)
        assert not binding.valid.any()


def test_input_ownership_fingerprint_and_nonfinite_delta(sample):
    geometry, binding = sample
    original = deepcopy(geometry)
    base = np.tile([.4, .65, .55, .8], (len(geometry["vertices_m"]), 1))
    delta = np.full((20, 20), np.nan); delta[0, 0] = np.inf
    mask = np.ones((20, 20), dtype=bool)
    np.testing.assert_allclose(binding.vertex_colors(delta, mask, base), base, atol=1e-7)
    for name in ("vertices_m", "faces", "joints_m"):
        np.testing.assert_array_equal(geometry[name], original[name])
    assert np.isnan(delta[1, 1]) and np.isinf(delta[0, 0]) and mask.all()
    np.testing.assert_array_equal(base[:, 3], .8)
    copied = deepcopy(geometry)
    second = TactileSurfaceBinding(copied, AXES)
    copied["vertices_m"][:] = 0
    copied["faces"][:] = 0
    np.testing.assert_allclose(second.positions(geometry["vertices_m"]), binding.positions(geometry["vertices_m"]), equal_nan=True)
    assert second.topology_id == binding.topology_id
    changed = deepcopy(geometry); changed["faces"] = changed["faces"][::-1]
    assert TactileSurfaceBinding(changed, AXES).topology_id != binding.topology_id
    for value in (binding.addresses, binding.weights, binding.face_ids, binding.vertex_weights):
        assert not value.flags.writeable


@pytest.mark.parametrize("field", ["vertices_m", "joints_m"])
def test_bad_neutral_geometry_is_rejected(field):
    geometry = fixture_geometry()
    geometry[field][0, 0] = np.nan
    with pytest.raises(ValueError):
        TactileSurfaceBinding(geometry, AXES)


def test_bad_axes_faces_and_hand_are_rejected():
    geometry = fixture_geometry()
    for axes in ({**AXES, "palm_up_axis": [0, 0, 0]}, {**AXES, "roll_axis": [0, 0, 1]}):
        with pytest.raises(ValueError):
            TactileSurfaceBinding(geometry, axes)
    geometry["faces"][0, 0] = len(geometry["vertices_m"])
    with pytest.raises(ValueError):
        TactileSurfaceBinding(geometry, AXES)
    geometry = fixture_geometry(); geometry["provenance"]["hand"] = "left"
    with pytest.raises(ValueError, match="right hand"):
        TactileSurfaceBinding(geometry, AXES)


def left_layout_path():
    import vt_tactile_capture.hand_layout as layout_module
    return Path(layout_module.__file__).parent / "layouts/left_glove_unverified_candidate.json"


def left_geometry():
    geometry = fixture_geometry()
    # Synthetic fixture only: reflect geometry and reverse triangle winding so
    # normals stay anatomically palmar. This does not validate left hardware.
    geometry["vertices_m"][:, 0] *= -1
    geometry["joints_m"][:, 0] *= -1
    geometry["faces"] = geometry["faces"][:, ::-1].copy()
    geometry["provenance"]["hand"] = "left"
    return geometry, {**AXES, "pitch_axis": [1., 0., 0.]}


def test_left_binding_needs_explicit_left_layout_and_matching_neutral_hand():
    geometry, axes = left_geometry()
    with pytest.raises(ValueError, match="explicit left layout"):
        TactileSurfaceBinding(geometry, axes, hand="left")
    with pytest.raises(ValueError, match="left hand"):
        TactileSurfaceBinding(fixture_geometry(), AXES, hand="left", layout_path=left_layout_path())
    with pytest.raises(ValueError, match="does not match"):
        TactileSurfaceBinding(geometry, axes, hand="right", layout_path=left_layout_path())
    binding = TactileSurfaceBinding(geometry, axes, hand="left", layout_path=left_layout_path())
    assert binding.valid.sum() > 300
    assert binding.metadata()["hand"] == "left"
    assert binding.metadata()["observed_anchor_count"] == 0
    assert "未验证" in binding.metadata()["layout_note"]
    assert np.all(binding.positions(geometry["vertices_m"])[binding.valid, 2] < 0)


def test_left_surface_uses_selected_addresses_and_directions_not_right_finger_columns(tmp_path):
    geometry, axes = left_geometry()
    original = TactileSurfaceBinding(geometry, axes, hand="left", layout_path=left_layout_path())
    config = json.loads(left_layout_path().read_text())
    regions = {region["region"]: region for region in config["regions"]}
    regions["index"]["cols"], regions["little"]["cols"] = regions["little"]["cols"], regions["index"]["cols"]
    regions["index"]["row_direction"] = -1
    path = tmp_path / "independent_left.json"
    path.write_text(json.dumps(config))
    binding = TactileSurfaceBinding(geometry, axes, hand="left", layout_path=path)
    expected_regions = original.regions.reshape(20, 20).copy()
    expected_regions[8:20, 0:4] = 2
    expected_regions[8:20, 12:16] = 5
    np.testing.assert_array_equal(binding.regions.reshape(20, 20), expected_regions)
    np.testing.assert_array_equal(binding.addresses, original.addresses)
    positions = binding.positions(geometry["vertices_m"]).reshape(20, 20, 3)
    previous = original.positions(geometry["vertices_m"]).reshape(20, 20, 3)
    np.testing.assert_allclose(positions[8:20, 0:4], previous[8:20, 12:16][::-1], atol=1e-12, equal_nan=True)
    base = np.tile([.4, .65, .55, 1.], (len(geometry["vertices_m"]), 1))
    delta = np.zeros((20, 20)); delta[8:20, 0:4] = 5000
    colors = binding.vertex_colors(delta, np.ones((20, 20), bool), base)
    changed = np.any(np.abs(colors - base) > 1e-6, axis=1)
    assert changed.any() and np.all(binding.vertex_regions[changed] == 2)
@pytest.mark.parametrize("bad", [np.zeros((2, 3)), np.full((810, 3), np.nan)])
def test_bad_current_vertices_rejected_without_changing_binding(sample, bad):
    geometry, binding = sample
    previous = binding.positions(geometry["vertices_m"])
    with pytest.raises(ValueError):
        binding.positions(bad)
    np.testing.assert_array_equal(binding.positions(geometry["vertices_m"]), previous)


@pytest.mark.parametrize("limit,deadband", [(0, 0), (100, 100), (100, -1), (np.nan, 0), (3000, np.inf), (True, 0), (1j, 0)])
def test_invalid_display_limits_rejected(sample, limit, deadband):
    geometry, binding = sample
    with pytest.raises(ValueError):
        binding.vertex_colors(np.zeros((20, 20)), np.ones((20, 20), dtype=bool),
                              np.ones((len(geometry["vertices_m"]), 4)), limit, deadband)


def test_invalid_color_inputs_are_rejected(sample):
    geometry, binding = sample
    base = np.ones((len(geometry["vertices_m"]), 4))
    delta, mask = np.zeros((20, 20)), np.ones((20, 20), dtype=bool)
    for changes in ((delta[:19], mask, base), (delta, mask.astype(int), base),
                    (np.zeros((80, 80)), np.ones((80, 80), bool), base),
                    (delta, mask, base[:, :3]), (delta, mask, base * np.nan)):
        with pytest.raises(ValueError):
            binding.vertex_colors(*changes)


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_MANO_ASSETS"), reason="No explicit licensed MANO assets")
def test_real_mano_candidate_coverage_and_fixed_topology_deformation():
    from vt_fsglove12.guided_calibration import model_motion_axes
    from vt_fsglove12.mano import ManoBackend
    backend = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    neutral = backend.render(np.zeros((16, 3)))
    axes = model_motion_axes(neutral["joints_m"])
    binding = TactileSurfaceBinding(neutral, axes)
    assert binding.valid.sum() >= 360
    assert binding.valid[binding.verified_anchor].all()
    print("TACTILE_SURFACE_REAL_MODEL", binding.metadata())
    bent = np.zeros((16, 3)); bent[1:4, 2] = [.5, .8, .4]
    changed = backend.render(bent)
    positions = binding.positions(changed["vertices_m"])
    selected = np.asarray(changed["vertices_m"])[np.asarray(changed["faces"])[binding.face_ids[binding.valid]]]
    np.testing.assert_allclose(positions[binding.valid], np.einsum("ni,nij->nj", binding.weights[binding.valid], selected))
