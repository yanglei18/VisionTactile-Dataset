"""Calibration binding comes from workstation configuration, never UI serials."""
from types import SimpleNamespace

import pytest
import yaml

from vt_capture_manager.config import ManagerConfig, default_devices


def camera_file(tmp_path):
    path = tmp_path / "cameras.yaml"
    path.write_text(yaml.safe_dump(dict(cameras=[dict(name="d405_1", model_token="D405", serial="test-serial")])))
    return path


def test_catalog_uses_camera_configuration_and_monitored_tracker_roles(tmp_path):
    from vt_capture_manager.calibration_runtime import CalibrationBindings
    config = ManagerConfig(tuple(default_devices()))
    bindings = CalibrationBindings(config, default_camera_config=camera_file(tmp_path))
    assert bindings.catalog()["cameras"] == [dict(name="d405_1", model="D405", serial="test-serial")]
    assert bindings.cameras[0]["device_info_service"] == "/d405_1/device_info"
    assert {t["role"] for t in bindings.catalog()["trackers"]} == {"torso", "left_wrist", "right_wrist"}
    params = dict(camera_name="d405_1", tracker_role="torso", mode="guided", config_path="", board=dict(
        square_length_mm=40., marker_length_mm=30., squares_x=9, squares_y=6, dictionary="DICT_5X5_1000"))
    result = bindings.prepare(params, expected_tracker_id="a"*64)
    assert result["config"]["camera"]["serial"] == "test-serial"
    assert result["expected_tracker_id"] == "a"*64
    assert result["config"]["tracker"]["frame_id"] == "vive_tracker_torso"


def test_imported_configuration_cannot_change_bound_camera_identity(tmp_path):
    from vt_capture_manager.calibration_runtime import CalibrationBindings
    bindings = CalibrationBindings(ManagerConfig(tuple(default_devices())), default_camera_config=camera_file(tmp_path))
    params = dict(camera_name="d405_1", tracker_role="torso", mode="offline", config_path="", board=dict(
        square_length_mm=40., marker_length_mm=30., squares_x=9, squares_y=6, dictionary="DICT_5X5_1000"))
    doc = bindings.prepare(params)["config"]
    doc["camera"]["serial"] = "another-device"
    path = tmp_path / "import.yaml"
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match="identity|binding"):
        bindings.prepare(params | dict(config_path=str(path)))


def test_no_monitored_color_stream_means_no_guided_camera(tmp_path):
    from vt_capture_manager.calibration_runtime import CalibrationBindings
    bindings = CalibrationBindings(ManagerConfig(()), default_camera_config=camera_file(tmp_path))
    assert bindings.catalog()["cameras"] == []


def test_guided_start_checks_actual_camera_identity_before_recording(tmp_path):
    from vt_capture_manager.calibration_runtime import CalibrationBindings, prepare_session
    bindings = CalibrationBindings(ManagerConfig(tuple(default_devices())), default_camera_config=camera_file(tmp_path))
    params = dict(camera_name="d405_1", tracker_role="torso", mode="guided", config_path="", board=dict(
        square_length_mm=40., marker_length_mm=30., squares_x=9, squares_y=6, dictionary="DICT_5X5_1000"))
    adapter = SimpleNamespace(expected_tracker_id=lambda role: "a"*64,
                              identity=lambda document: (False, "camera_identity_unverified"))
    with pytest.raises(ValueError, match="camera_identity_unverified"):
        prepare_session(bindings, adapter, params)
    adapter.identity = lambda document: (True, "")
    assert prepare_session(bindings, adapter, params)["expected_tracker_id"] == "a"*64


def test_calibration_preserves_configured_tracker_frames_and_topics(tmp_path, monkeypatch):
    from vt_capture_manager.calibration_runtime import CalibrationBindings
    from vt_capture_manager.spatial.config import SceneConfig
    scene = SceneConfig(world_frame="custom_world", alignment_document=dict(trackers=[
        dict(role="torso", frame_id="custom_torso", sample_topic="/vive/torso/sample")]))
    monkeypatch.setattr("vt_capture_manager.spatial.config.load_scene_config", lambda path: scene)
    config = ManagerConfig(tuple(default_devices()), scene_config_path=str(tmp_path / "scene.yaml"))
    bindings = CalibrationBindings(config, default_camera_config=camera_file(tmp_path))
    tracker = next(t for t in bindings.trackers if t["role"] == "torso")
    assert tracker == dict(role="torso", frame_id="custom_torso", sample_topic="/vive/torso/sample")
    assert bindings.world_frame == "custom_world"


def test_conflicting_tracker_source_is_not_silently_rebound(tmp_path, monkeypatch):
    from vt_capture_manager.calibration_runtime import CalibrationBindings
    from vt_capture_manager.spatial.config import SceneConfig
    scene = SceneConfig(alignment_document=dict(trackers=[
        dict(role="torso", frame_id="custom_torso", sample_topic="/different/torso/sample")]))
    monkeypatch.setattr("vt_capture_manager.spatial.config.load_scene_config", lambda path: scene)
    config = ManagerConfig(tuple(default_devices()), scene_config_path=str(tmp_path / "scene.yaml"))
    with pytest.raises(ValueError, match="Tracker.*topic"):
        CalibrationBindings(config, default_camera_config=camera_file(tmp_path))
