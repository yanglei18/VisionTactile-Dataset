"""Procedural solids and physical-root rendering, without any hardware access."""

from copy import deepcopy
import math
import os

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.engine import demo_snapshot
from vt_glove_pose.kinematics import HandModel
from vt_glove_pose.mesh import (
    ESTIMATE_ACCENT, JOINT_INTERMEDIATE, JOINT_ROOT, JOINT_STALE, JOINT_UNAVAILABLE,
    NAIL, PALMAR_MATERIAL_MULTIPLIER, SKIN, STALE, UNCALIBRATED, UNCERTAIN,
    UNAVAILABLE_NAIL_MULTIPLIER, _Builder,
    build_hand_mesh, display_root_matrix, orientation_matrix,
)


def model_at(t=0.0, hand="right"):
    config = load_config()
    config["hand"] = hand
    return demo_snapshot(t, config)["model"]


def volume(vertices, faces):
    triangle = np.asarray(vertices)[np.asarray(faces)]
    return np.einsum("ij,ij->i", triangle[:, 0], np.cross(triangle[:, 1], triangle[:, 2])).sum()/6


def test_mesh_is_volumetric_finite_and_complete_without_mutating_model():
    model = model_at()
    original = deepcopy(model)
    mesh = build_hand_mesh(model)
    assert model == original
    assert mesh.vertices.shape[1] == 3
    assert mesh.normals.shape == mesh.vertices.shape
    assert mesh.triangles.shape[1] == 3
    assert 500 < len(mesh.triangles) < 4000
    assert len(mesh.colors) == len(mesh.estimated) == len(mesh.parts) == len(mesh.triangles)
    assert np.all(np.isfinite(mesh.vertices))
    assert np.all(np.isfinite(mesh.normals))
    assert np.all(mesh.triangles >= 0) and np.all(mesh.triangles < len(mesh.vertices))
    assert np.all((mesh.colors >= 0) & (mesh.colors <= 255))
    assert np.ptp(mesh.vertices[:, 2]) > 12
    for part in ["palm", "wrist_stub"] + [f"{name}:{detail}" for name in model["fingers"] for detail in ("knuckle", "tip", "nail")]:
        assert part in mesh.parts


def test_fresh_ivory_material_and_aqua_details_do_not_use_intersecting_thenar_shell():
    model = model_at()
    model["fingers"]["index"].update(valid=True, quality="ok")
    mesh = build_hand_mesh(model)
    assert np.min(SKIN) >= 225
    assert SKIN[2] >= SKIN[0]  # Neutral/cool ivory, not the former brown wood.
    assert NAIL[1] > NAIL[0]+50 and NAIL[2] > NAIL[0]+50
    assert np.mean(UNCERTAIN) > 220  # Gentle cream body, with explicit markers.
    assert "thenar_pad" not in mesh.parts
    index = np.asarray(mesh.parts) == "index:0"
    body = mesh.colors[index][24:48]  # Middle band remains the fresh hand material.
    np.testing.assert_allclose(body, np.broadcast_to(SKIN, body.shape))
    # Invalid states remain materially distinct from the valid ivory surface.
    assert np.linalg.norm(SKIN-UNCALIBRATED) > 50
    assert np.linalg.norm(SKIN-STALE) > 70


@pytest.mark.parametrize("hand", ["left", "right"])
@pytest.mark.parametrize("stale,valid,expected", [
    (False, True, SKIN), (False, False, UNCALIBRATED), (True, True, STALE),
])
def test_palm_and_dorsum_remain_distinct_in_every_quality_state(hand, stale, valid, expected):
    model = model_at(hand=hand)
    model["palm_valid"] = valid
    original = deepcopy(model)
    mesh = build_hand_mesh(model, stale=stale)
    assert model == original
    palm = np.asarray(mesh.parts) == "palm"
    face_height = mesh.vertices[mesh.triangles[palm], 2].mean(axis=1)
    colors = mesh.colors[palm]
    back, front = colors[face_height > 0], colors[face_height < 0]
    assert len(back) == len(front) > 0
    np.testing.assert_allclose(back, np.broadcast_to(expected, back.shape))
    np.testing.assert_allclose(front, np.broadcast_to(expected*PALMAR_MATERIAL_MULTIPLIER, front.shape))
    assert np.linalg.norm(back[0]-front[0]) > 40
    assert len(mesh.triangles) == 3864  # Material only; no new intersecting layers.
    if stale:
        assert np.all(front[:, 0] > front[:, 1])  # Still red, never valid aqua.
    elif not valid:
        assert np.all(front[:, 2] > front[:, 0])
        assert np.max(front) < np.min(SKIN)


@pytest.mark.parametrize("hand", ["left", "right"])
def test_flat_hand_nails_are_only_on_the_dorsal_side(hand):
    config = load_config()
    config["hand"] = hand
    model = HandModel(config).solve({}, set(), set(), False)
    mesh = build_hand_mesh(model)
    for name in model["fingers"]:
        faces = mesh.triangles[np.asarray(mesh.parts) == f"{name}:nail"]
        assert np.all(mesh.vertices[faces, 2] > 0.0)


@pytest.mark.parametrize("stale,valid,quality,expected", [
    (True, True, "ok", STALE), (False, False, "no_neutral", UNCALIBRATED),
    (False, True, "stale", STALE),
])
def test_unavailable_nails_keep_contrast_and_do_not_use_valid_or_estimated_colors(stale, valid, quality, expected):
    model = model_at()
    model["fingers"]["index"].update(valid=valid, quality=quality)
    mesh = build_hand_mesh(model, stale=stale)
    colors = mesh.colors[np.asarray(mesh.parts) == "index:nail"]
    np.testing.assert_allclose(colors, np.broadcast_to(expected*UNAVAILABLE_NAIL_MULTIPLIER, colors.shape))
    assert np.linalg.norm(colors[0]-expected) > 70


@pytest.mark.parametrize("hand", ["left", "right"])
def test_closed_primitives_have_outward_winding_for_both_hands(hand):
    mesh = build_hand_mesh(model_at(hand=hand))
    for part in ("palm", "wrist_stub", "index:knuckle", "index:tip", "index:nail", "thumb:knuckle"):
        faces = mesh.triangles[np.asarray(mesh.parts) == part]
        assert volume(mesh.vertices, faces) > 0, part
    triangles = mesh.vertices[mesh.triangles]
    geometry_normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    smooth_normal = mesh.normals[mesh.triangles].mean(axis=1)
    good = np.linalg.norm(geometry_normal, axis=1) > 1e-7
    assert np.all(np.einsum("ij,ij->i", geometry_normal[good], smooth_normal[good]) > 0)


def test_tube_and_ellipsoid_normals_point_away_from_interior():
    builder = _Builder()
    builder.tube(np.array([0., 0, 0]), np.array([0., 20, 0]), np.array([1., 0, 0]),
                 np.array([0., 0, 1]), 4, 3, [200, 150, 100], "tube")
    vertices, faces = np.asarray(builder.vertices), np.asarray(builder.faces)
    triangle = vertices[faces]
    normal = np.cross(triangle[:, 1]-triangle[:, 0], triangle[:, 2]-triangle[:, 0])
    radial = triangle.mean(axis=1)
    radial[:, 1] = 0
    assert np.all(np.einsum("ij,ij->i", normal, radial) > 0)
    sphere = _Builder()
    center = np.array([3., 4, 5])
    sphere.ellipsoid(center, np.eye(3), [3, 4, 2], [200, 150, 100], "ellipsoid")
    tri = np.asarray(sphere.vertices)[np.asarray(sphere.faces)]
    normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    nonzero = np.linalg.norm(normal, axis=1) > 1e-7
    assert np.all(np.einsum("ij,ij->i", normal[nonzero], tri.mean(axis=1)[nonzero]-center) > 0)


def test_estimated_distal_mesh_is_not_promoted_to_measured():
    mesh = build_hand_mesh(model_at())
    parts = np.asarray(mesh.parts)
    assert not np.any(mesh.estimated[np.char.startswith(parts, "thumb:")])
    for name in ("index", "middle", "ring", "little"):
        for segment in (f"{name}:2", f"{name}:tip", f"{name}:nail"):
            assert np.all(mesh.estimated[parts == segment])
        assert not np.any(mesh.estimated[parts == f"{name}:0"])


@pytest.mark.parametrize("stale,valid,quality,expected", [
    (True, True, "ok", STALE), (False, False, "no_neutral", UNCALIBRATED),
    (False, True, "clipped", UNCERTAIN), (False, True, "stale", STALE),
])
def test_invalid_stale_and_clipped_finger_colors_are_explicit(stale, valid, quality, expected):
    model = model_at()
    model["fingers"]["index"].update(valid=valid, quality=quality)
    mesh = build_hand_mesh(model, stale=stale)
    selected = np.asarray(mesh.parts) == "index:0"
    body = mesh.colors[selected][24:48]
    np.testing.assert_allclose(body, np.broadcast_to(expected, body.shape))


@pytest.mark.parametrize("quality", ["thumb_prediction_only", "thumb_uncertain", "thumb_reacquiring", "thumb_observed_after_loss"])
def test_finite_thumb_prediction_remains_visible_but_is_not_colored_as_confident(quality):
    model = model_at()
    before_mesh = build_hand_mesh(model)
    thumb = model["fingers"]["thumb"]
    thumb.update(valid=True, geometry_live=True, ip_confident=False, quality=quality, clipped_angles=[])
    original = deepcopy(model)
    mesh = build_hand_mesh(model)
    parts = np.asarray(mesh.parts)
    body = mesh.colors[parts == "thumb:0"][24:48]
    joints = mesh.colors[parts == "thumb:knuckle"]
    nails = mesh.colors[parts == "thumb:nail"]
    np.testing.assert_allclose(body, np.broadcast_to(UNCERTAIN, body.shape))
    np.testing.assert_allclose(joints, np.broadcast_to(ESTIMATE_ACCENT, joints.shape))
    np.testing.assert_allclose(nails, np.broadcast_to(NAIL*.85 + ESTIMATE_ACCENT*.15, nails.shape))
    # Mark uncertainty without hiding data, replacing the pose, or changing
    # hand style, topology, measured/inferred segment metadata, or other digits.
    np.testing.assert_array_equal(mesh.vertices, before_mesh.vertices)
    np.testing.assert_array_equal(mesh.triangles, before_mesh.triangles)
    np.testing.assert_array_equal(mesh.estimated, before_mesh.estimated)
    other = ~np.char.startswith(parts, "thumb:")
    np.testing.assert_array_equal(mesh.colors[other], before_mesh.colors[other])
    assert model == original and thumb["geometry_live"] and thumb["valid"]


@pytest.mark.parametrize("stale,valid,quality,body_color,joint_color", [
    (True, True, "thumb_prediction_only", STALE, JOINT_STALE),
    (False, True, "stale", STALE, JOINT_STALE),
    (False, False, "thumb_uninitialized", UNCALIBRATED, JOINT_UNAVAILABLE),
])
def test_stale_and_unavailable_override_thumb_prediction_markers(stale, valid, quality, body_color, joint_color):
    model = model_at()
    model["fingers"]["thumb"].update(valid=valid, ip_confident=False, quality=quality, clipped_angles=["ip_deg"])
    mesh = build_hand_mesh(model, stale=stale)
    parts = np.asarray(mesh.parts)
    body = mesh.colors[parts == "thumb:0"][24:48]
    joints = mesh.colors[parts == "thumb:knuckle"]
    nails = mesh.colors[parts == "thumb:nail"]
    np.testing.assert_allclose(body, np.broadcast_to(body_color, body.shape))
    np.testing.assert_allclose(joints, np.broadcast_to(joint_color, joints.shape))
    np.testing.assert_allclose(nails, np.broadcast_to(body_color*UNAVAILABLE_NAIL_MULTIPLIER, nails.shape))


def test_clipped_angle_markers_survive_more_specific_thumb_quality():
    model = model_at()
    # Deliberately no confidence warning: clipping itself must remain visible
    # even when a newer quality label replaces the old literal "clipped".
    model["fingers"]["thumb"].update(valid=True, ip_confident=True, quality="thumb_observed", clipped_angles=["ip_deg"])
    mesh = build_hand_mesh(model)
    parts = np.asarray(mesh.parts)
    body = mesh.colors[parts == "thumb:0"][24:48]
    joints = mesh.colors[parts == "thumb:knuckle"]
    np.testing.assert_allclose(body, np.broadcast_to(UNCERTAIN, body.shape))
    np.testing.assert_allclose(joints, np.broadcast_to(ESTIMATE_ACCENT, joints.shape))


def test_joint_positions_have_distinct_bands_without_adding_intersecting_shells():
    model = model_at()
    model["fingers"]["thumb"]["quality"] = "anatomical_provisional"
    mesh = build_hand_mesh(model)
    parts = np.asarray(mesh.parts)
    assert len(mesh.triangles) == 3864  # Recolored existing surfaces; no new ring mesh.
    for name in model["fingers"]:
        expected = [JOINT_ROOT, JOINT_INTERMEDIATE]
        if name != "thumb":
            expected.append(ESTIMATE_ACCENT)
        joints = mesh.colors[parts == f"{name}:knuckle"].reshape(len(expected), 96, 3)
        for joint, color in zip(joints, expected):
            np.testing.assert_allclose(joint, np.broadcast_to(color, joint.shape))
        for segment, color in enumerate(expected):
            tube = mesh.colors[parts == f"{name}:{segment}"]
            # Overlapping sphere and tubes share a material, avoiding the
            # white/teal triangle shards of differently colored intersections.
            np.testing.assert_allclose(tube[:24], np.broadcast_to(color, (24, 3)))
            if segment < len(expected)-1:
                np.testing.assert_allclose(tube[-24:], np.broadcast_to(expected[segment+1], (24, 3)))
    thumb = model["fingers"]["thumb"]
    assert thumb["quality"] == "anatomical_provisional"
    assert not np.any(mesh.estimated[parts == "thumb:knuckle"])


@pytest.mark.parametrize("source,valid,expected", [
    ("model_estimate", True, ESTIMATE_ACCENT),
    ("disabled", False, JOINT_UNAVAILABLE),
    ("model_estimate", False, JOINT_UNAVAILABLE),
])
def test_dip_band_uses_actual_estimate_availability_not_segment_list(source, valid, expected):
    model = model_at()
    model["fingers"]["index"].update(dip_source=source, dip_valid=valid, estimated_segments=[2])
    mesh = build_hand_mesh(model)
    selected = np.asarray(mesh.parts) == "index:knuckle"
    distal = mesh.colors[selected].reshape(3, 96, 3)[2]
    np.testing.assert_allclose(distal, np.broadcast_to(expected, distal.shape))
    inferred = mesh.estimated[selected].reshape(3, 96)[2]
    assert np.all(inferred == (source == "model_estimate" and valid))


@pytest.mark.parametrize("stale,valid,quality,expected", [
    (True, True, "ok", JOINT_STALE),
    (False, True, "stale", JOINT_STALE),
    (False, False, "no_neutral", JOINT_UNAVAILABLE),
    (False, True, "clipped", ESTIMATE_ACCENT),
])
def test_joint_marker_safety_states_override_position_colors(stale, valid, quality, expected):
    model = model_at()
    model["fingers"]["index"].update(valid=valid, quality=quality)
    mesh = build_hand_mesh(model, stale=stale)
    selected = np.asarray(mesh.parts) == "index:knuckle"
    np.testing.assert_allclose(mesh.colors[selected], np.broadcast_to(expected, mesh.colors[selected].shape))


def test_conservative_framing_radius_is_constant_through_flexion():
    meshes = [build_hand_mesh(model_at(t)) for t in np.linspace(0, 8, 12)]
    assert np.ptp([mesh.radius for mesh in meshes]) < 1e-8
    for mesh in meshes:
        np.testing.assert_allclose(mesh.center, meshes[0].center)
        assert np.max(np.linalg.norm(mesh.vertices-mesh.center, axis=1)) < mesh.radius


def test_physical_root_matrix_normalization_validity_and_local_view():
    q = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    rotation = orientation_matrix(q)
    np.testing.assert_allclose(rotation @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(orientation_matrix(np.array(q)*-3), rotation, atol=1e-12)
    model = {"root_orientation_valid": True, "root_orientation_wxyz": q, "hand": "left"}
    # No additional left-hand reflection is allowed in the renderer.
    np.testing.assert_allclose(display_root_matrix(model), rotation)
    np.testing.assert_allclose(display_root_matrix(model, follow=False), np.eye(3))
    model["root_orientation_valid"] = False
    np.testing.assert_allclose(display_root_matrix(model), np.eye(3))
    model.update(root_orientation_valid=True, root_orientation_wxyz=[float("nan"), 0, 0, 0])
    np.testing.assert_allclose(display_root_matrix(model), np.eye(3))


def test_bad_geometry_is_rejected_instead_of_becoming_a_fake_hand():
    model = model_at()
    model["fingers"]["index"]["points"][0][0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        build_hand_mesh(model)


def test_canvas_solid_skeleton_and_root_transform_offscreen():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PyQt5")
    from PyQt5 import QtWidgets
    from vt_glove_pose.canvas import HandCanvas
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["mesh-test"])
    canvas = HandCanvas()
    canvas.resize(850, 650)
    model = model_at()
    canvas.set_model(model, mode="demo", stale=False)
    solid = canvas.grab()
    assert not solid.isNull()
    background = solid.toImage().pixelColor(1, 1)
    # The user requested a fresh-looking hand, not a changed interface theme.
    assert max(background.red(), background.green(), background.blue()) < 75
    assert 100 < canvas.drawn_triangles < len(canvas._mesh.triangles)
    assert canvas.paint_elapsed_ms > 0
    assert canvas.render_mode == "solid" and canvas.follow_root
    point = np.asarray([[1., 2, 3]])
    expected = point @ orientation_matrix(model["root_orientation_wxyz"]).T @ canvas.view_matrix().T
    np.testing.assert_allclose(canvas.model_to_view(point), expected)
    canvas.set_root_follow(False)
    np.testing.assert_allclose(canvas.model_to_view(point), point @ canvas.view_matrix().T)
    canvas.set_render_mode("skeleton")
    skeleton = canvas.grab()
    assert not skeleton.isNull()
    assert canvas.drawn_triangles == 0
    assert skeleton.toImage() != solid.toImage()
    canvas.close()
    app.processEvents()


def test_disabled_or_stale_dip_never_receives_gold_estimate_overlay():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PyQt5")
    from PyQt5 import QtWidgets
    from vt_glove_pose.canvas import HandCanvas
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["overlay-test"])
    canvas = HandCanvas()
    model = model_at()
    for finger in model["fingers"].values():
        finger.update(dip_source="disabled", dip_valid=False)
    canvas.set_model(model, mode="demo", stale=False)

    class Painter:
        def __init__(self):
            self.lines = []

        def setPen(self, pen):
            pass

        def drawLine(self, start, end):
            self.lines.append((start, end))

    painter = Painter()
    project = lambda points: np.asarray(points)[:, :2]
    canvas._draw_estimated_overlay(painter, project)
    assert painter.lines == []
    model["fingers"]["index"].update(dip_source="model_estimate", dip_valid=True)
    canvas._draw_estimated_overlay(painter, project)
    assert len(painter.lines) == 1
    painter.lines.clear()
    model["fingers"]["index"]["quality"] = "stale"
    canvas._draw_estimated_overlay(painter, project)
    assert painter.lines == []
    canvas.close()
    app.processEvents()
