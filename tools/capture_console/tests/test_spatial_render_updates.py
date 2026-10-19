"""Spatial data continues updating while unnecessary GPU uploads are deferred."""
from copy import deepcopy

from vt_capture_console.backend.scene_demo import demo_scene
from vt_capture_console.ui.widgets.spatial_scene import SpatialScene
from vt_capture_console.viewmodels.spatial import SpatialViewModel


def _count_uploads(monkeypatch, scene):
    uploads = []
    original = scene.viewport.set_model

    def upload(model):
        uploads.append(model.frame)
        original(model)

    monkeypatch.setattr(scene.viewport, "set_model", upload)
    return uploads


def test_public_fit_reset_refits_the_next_frame_without_changing_view_layers(qapp, monkeypatch):
    scene, model = SpatialScene(), SpatialViewModel()
    fits = []
    monkeypatch.setattr(scene.viewport, "fit", lambda: fits.append(model.frame))
    try:
        model.update(demo_scene(0))
        scene.show()
        scene.set_model(model)
        qapp.processEvents()
        assert len(fits) == 1
        scene.set_model(model)
        assert len(fits) == 1
        layers = set(model.layers)

        scene.reset_view_fit()
        assert len(fits) == 1
        scene.set_model(model)

        assert len(fits) == 2 and model.layers == layers
    finally:
        scene.close()


def test_hidden_scene_retains_latest_model_and_uploads_once_on_show(qapp, monkeypatch):
    scene, model = SpatialScene(), SpatialViewModel()
    uploads = _count_uploads(monkeypatch, scene)
    try:
        for sequence in range(5):
            model.update(demo_scene(sequence, sequence=sequence))
            scene.set_model(model)
        assert uploads == []
        assert scene.model.frame["sequence"] == 4
        scene.show()
        qapp.processEvents()
        assert len(uploads) == 1 and uploads[0]["sequence"] == 4

        scene.hide()
        model.update(demo_scene(10, sequence=10))
        scene.set_model(model)
        assert len(uploads) == 1
        scene.show()
        qapp.processEvents()
        assert len(uploads) == 2 and uploads[-1]["sequence"] == 10
    finally:
        scene.close()


def test_scene_ages_do_not_upload_but_pose_and_state_changes_do(qapp, monkeypatch):
    scene, model = SpatialScene(), SpatialViewModel()
    uploads = _count_uploads(monkeypatch, scene)
    frame = demo_scene(0)
    try:
        scene.show()
        model.update(frame)
        scene.set_model(model)
        qapp.processEvents()
        assert len(uploads) == 1
        for sequence in range(1, 6):
            changed = deepcopy(frame)
            changed["sequence"] = sequence
            changed["generated_realtime_ns"] += sequence
            for entity in changed["trackers"] + changed["cameras"]:
                entity["age_s"] += sequence
            model.update(changed)
            scene.set_model(model)
        assert len(uploads) == 1
        assert model.frame["sequence"] == 5

        moved = deepcopy(frame)
        moved["trackers"][0]["position_m"][0] += .1
        model.update(moved)
        scene.set_model(model)
        assert len(uploads) == 2
        model.source_available = False
        scene.set_model(model)
        assert len(uploads) == 3
        model.selected_id = moved["trackers"][0]["id"]
        scene.set_model(model)
        assert len(uploads) == 4
    finally:
        scene.close()


def test_hidden_cloud_updates_are_coalesced_and_minimized_scene_defers(qapp, monkeypatch):
    scene, model = SpatialScene(), SpatialViewModel()
    calls = []
    monkeypatch.setattr(scene.viewport, "set_clouds", lambda *args: calls.append(args))
    try:
        model.update(demo_scene(0))
        scene.set_model(model)
        for size in range(1, 6):
            scene.set_clouds({}, point_size=size)
        assert not calls
        scene.show()
        qapp.processEvents()
        assert len(calls) == 1 and calls[-1][2] == 5
        scene.set_clouds({}, point_size=5)
        assert len(calls) == 1
        scene.showMinimized()
        qapp.processEvents()
        scene.set_clouds({}, point_size=6)
        assert len(calls) == 1
        scene.showNormal()
        qapp.processEvents()
        assert len(calls) == 2 and calls[-1][2] == 6
    finally:
        scene.close()


def test_mutable_cloud_transform_cannot_change_previous_signature(qapp, monkeypatch):
    scene, model = SpatialScene(), SpatialViewModel()
    calls = []
    monkeypatch.setattr(scene.viewport, "set_clouds", lambda *args: calls.append(args))
    cloud = {"header": {"worker_instance_id": "worker", "cloud_sequence": 1,
                        "world_from_color": {"translation_m": [0., 0., 0.],
                                             "quaternion_xyzw": [0., 0., 0., 1.]}}}
    try:
        model.update(demo_scene(0))
        scene.set_model(model)
        scene.show()
        qapp.processEvents()
        scene.set_clouds({"camera": cloud})
        assert len(calls) == 1
        cloud["header"]["world_from_color"]["translation_m"][0] = .1
        scene.set_clouds({"camera": cloud})
        assert len(calls) == 2
    finally:
        scene.close()
