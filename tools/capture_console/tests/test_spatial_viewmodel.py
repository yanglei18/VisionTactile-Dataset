"""Spatial viewing keeps world coordinates, immutable freezes and identities."""
import math

from vt_capture_console.backend.scene_demo import demo_scene
from vt_capture_console.viewmodels.spatial import SpatialViewModel


def test_shared_world_and_immutable_freeze():
    frame = demo_scene(0, sequence=1)
    first, second = SpatialViewModel(), SpatialViewModel()
    first.update(frame)
    second.update(frame)
    before = first.frame["trackers"][0]["position_m"]
    assert before != (0, 0, 0)
    first.set_frozen(True)
    frame["trackers"][0]["position_m"][0] = 123
    newer = demo_scene(1, sequence=2)
    first.update(newer)
    second.update(newer)
    assert first.frame["sequence"] == 1
    assert first.frame["trackers"][0]["position_m"] == before
    assert second.frame["sequence"] == 2
    first.set_frozen(False)
    assert first.frame["sequence"] == 2


def test_clear_frame_discards_live_and_frozen_data_without_resetting_view_choices():
    model = SpatialViewModel()
    model.update(demo_scene(0, sequence=1))
    model.set_frozen(True)
    model.update(demo_scene(1, sequence=2))
    model.set_preset("tracker")
    model.layers.discard("grid")
    model.hidden_entities.add("torso")
    model.select("left_wrist")

    model.clear_frame()

    assert model.frame is None and model.frozen
    model.set_frozen(False)
    assert model.frame is None
    assert model.preset == "tracker" and "grid" not in model.layers
    assert model.hidden_entities == {"torso"} and model.selected_id == "left_wrist"


def test_presets_selection_and_camera_geometry():
    model = SpatialViewModel()
    model.update(demo_scene(0))
    assert len(model.entities()) == 6
    model.set_preset("tracker")
    assert len(model.entities()) == 3
    model.select("left_wrist")
    assert model.selected["role"] == "left_wrist"
    model.set_preset("camera")
    assert len(model.entities()) == 3
    assert all(len(e["frustum_vertices"]) == 8 for e in model.entities())
    assert all(math.isfinite(v) for e in model.entities() for v in e["position_m"])


def test_invalid_camera_never_drawn_at_origin_and_identity_change_clears_live_selection():
    model = SpatialViewModel()
    frame = demo_scene(0)
    frame["cameras"][0].update(state="identity_unverified", position_m=None)
    model.update(frame)
    assert len(model.entities()) == 5
    model.select("left_wrist")
    frame["manager_instance_id"] = "replacement-manager"
    model.update(frame)
    assert model.selected is None


def test_demo_cameras_keep_same_rigid_tracker_mount_during_rotation():
    from vt_capture_core.spatial.transforms import RigidTransform
    for seconds in (0., 1.7, 4.):
        frame = demo_scene(seconds)
        for tracker, camera in zip(frame["trackers"], frame["cameras"]):
            tracker_pose = RigidTransform(tuple(tracker["position_m"]), tuple(tracker["quaternion_xyzw"]))
            camera_pose = RigidTransform(tuple(camera["position_m"]), tuple(camera["quaternion_xyzw"]))
            relative = tracker_pose.inverse().compose(camera_pose)
            assert all(abs(a-b) < 1e-9 for a, b in zip(relative.translation_m, (0., -.05, -.04)))


def test_malformed_presentation_identity_is_omitted_and_optional_label_is_safe():
    model = SpatialViewModel()
    frame = demo_scene(0)
    frame["trackers"].append({})
    frame["cameras"][0]["label"] = None
    model.update(frame)
    assert len(model.all_entities()) == 6
    assert model.frame["cameras"][0]["label"] == "d405_1"
    assert any("无效实体" in str(note) for note in model.frame["diagnostics"])
