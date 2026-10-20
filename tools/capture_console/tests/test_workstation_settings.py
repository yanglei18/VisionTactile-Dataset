from copy import deepcopy
import json
import os
from pathlib import Path

import pytest

from vt_capture_console.backend.workstation_settings import (
    WorkstationSettings, default_configuration, serial_ports, validate_configuration,
)
from vt_capture_core.workstation import parse_profiles


@pytest.fixture(autouse=True)
def isolated_home(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "home")
    for key in ("VT_DATA_ROOT", "VT_FSGLOVE_PYTHON", "VT_FSGLOVE_SOURCE", "VT_MANO_ASSETS",
                "FSGLOVE12_UPSTREAM_CHECKOUT", "FSGLOVE12_MANO_ASSETS", "FSGLOVE_SOURCE",
                "MANO_ASSETS", "VT_PYVUT_ROOT", "VT_PYVUT_VENV", "VT_PYVUT_SITE",
                "VT_BUNDLE", "VT_ROLE_MAP"):
        monkeypatch.delenv(key, raising=False)


def test_defaults_are_six_disabled_profiles_without_port_guessing(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "home")
    root = tmp_path / "repo"
    root.mkdir()
    value = default_configuration(root)
    profiles = value["process_groups"]
    assert len(profiles) == 6 and not any(group["enabled"] for group in profiles)
    assert {group["id"] for group in profiles} == {"cameras", "trackers", "right_glove", "left_glove", "right_tactile", "left_tactile"}
    assert profiles[0]["parameters"] == dict(config_path="", output_root=str(tmp_path / "home/visiontactile-data"))
    assert value["disk_path"] == profiles[0]["parameters"]["output_root"]
    assert not (tmp_path / "home/visiontactile-data").exists()
    assert all(group["parameters"].get("port", "") == "" for group in profiles)
    assert all(group["parameters"].get("fsglove_source", "") == "" for group in profiles)
    assert not (root / "ros2_ws").exists()


def test_only_existing_known_paths_are_suggested(tmp_path, monkeypatch):
    home = tmp_path / "home"
    output = home / "visiontactile-data"
    output.mkdir(parents=True)
    monkeypatch.setattr(Path, "home", lambda: home)
    root = tmp_path / "repo"
    camera = root / "ros2_ws/src/vt_realsense_capture/config/cameras.yaml"
    camera.parent.mkdir(parents=True)
    camera.write_text("# harmless test config\n")
    interpreter = root / ".venv-fsglove-120hz/bin/python"
    interpreter.parent.mkdir(parents=True)
    interpreter.touch()
    interpreter.chmod(0o700)
    value = default_configuration(root)
    assert value["disk_path"] == str(output)
    assert value["process_groups"][0]["parameters"] == dict(config_path=str(camera), output_root=str(output))
    assert value["process_groups"][2]["parameters"]["glove_python"] == str(interpreter)


def test_existing_tracker_only_scene_is_suggested(tmp_path):
    path = tmp_path / "ros2_ws/src/vt_capture_manager/config/scene.example.yaml"
    path.parent.mkdir(parents=True)
    path.write_text("schema_version: 1\n")
    assert default_configuration(tmp_path)["scene_config_path"] == str(path)


def test_developer_dated_artifacts_are_not_used_as_new_machine_defaults(tmp_path):
    baseline = tmp_path / "artifacts/glove_hand_pose_live/upstream_baselines_20260930"
    (baseline / "fsglove-a7988f0").mkdir(parents=True)
    (baseline / "mano-assets").mkdir()
    calibration = tmp_path / ".runtime/recovery/right_current_live_20261005.json"
    calibration.parent.mkdir(parents=True)
    calibration.write_text("old operator calibration")
    groups = {value["id"]: value for value in default_configuration(tmp_path)["process_groups"]}
    for hand in ("right", "left"):
        parameters = groups[hand + "_glove"]["parameters"]
        assert parameters["fsglove_source"] == parameters["mano_assets"] == ""
        assert parameters["config"] == parameters["port"] == ""
    assert groups["trackers"]["parameters"]["bundle_path"] == ""


def test_default_data_directory_is_created_only_when_settings_are_saved(tmp_path):
    settings = WorkstationSettings(tmp_path / "repo", tmp_path / "private")
    output = Path.home() / "visiontactile-data"
    assert not output.exists()
    settings.detect_environment(settings.configuration)
    assert not output.exists() and not settings.path.exists()
    settings.save(settings.configuration)
    assert output.is_dir() and not output.is_symlink()
    assert output.stat().st_mode & 0o777 == 0o700
    assert settings.configuration["disk_path"] == str(output)


def test_default_output_file_or_symlink_cannot_replace_saved_configuration(tmp_path):
    settings = WorkstationSettings(tmp_path / "repo", tmp_path / "private")
    settings.save(settings.configuration)
    before = settings.path.read_bytes()
    output = Path.home() / "visiontactile-data"
    output.rmdir()
    output.write_text("unrelated file")
    with pytest.raises((OSError, ValueError)):
        settings.save(settings.configuration)
    assert settings.path.read_bytes() == before and output.read_text() == "unrelated file"
    output.unlink()
    target = tmp_path / "target"
    target.mkdir()
    output.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match="数据目录"):
        settings.save(settings.configuration)
    assert settings.path.read_bytes() == before and list(target.iterdir()) == []


def test_custom_missing_output_is_not_created_by_default_directory_preparation(tmp_path):
    settings = WorkstationSettings(tmp_path / "repo", tmp_path / "private")
    value = settings.configuration
    custom = tmp_path / "operator-choice/output"
    value["process_groups"][0]["parameters"]["output_root"] = str(custom)
    settings.save(value)
    assert not custom.exists()


def test_data_root_override_is_created_on_save(tmp_path, monkeypatch):
    output = tmp_path / "data-volume/new-user/datasets"
    monkeypatch.setenv("VT_DATA_ROOT", str(output))
    settings = WorkstationSettings(tmp_path / "repo", tmp_path / "private")
    assert settings.configuration["process_groups"][0]["parameters"]["output_root"] == str(output)
    assert not output.exists()
    settings.save(settings.configuration)
    assert output.is_dir() and output.stat().st_mode & 0o777 == 0o700


def test_invalid_environment_default_does_not_block_explicit_configuration(tmp_path, monkeypatch):
    settings = WorkstationSettings(tmp_path / "repo", tmp_path / "private")
    output = tmp_path / "chosen"
    output.mkdir()
    value = settings.configuration
    value["process_groups"][0]["parameters"]["output_root"] = str(output)
    monkeypatch.setenv("VT_DATA_ROOT", "invalid-relative-default")
    settings.save(value)
    assert settings.configuration == value


def test_existing_settings_wait_for_explicit_environment_detection(tmp_path):
    root = tmp_path / "repo"
    settings = WorkstationSettings(root, tmp_path / "private")
    value = settings.configuration
    value["process_groups"][0]["parameters"]["config_path"] = ""
    settings.save(value)
    camera = Path.home() / "visiontactile-data/config/cameras.yaml"
    camera.parent.mkdir(parents=True)
    camera.write_text("# operator configuration\n")
    loaded = WorkstationSettings(root, settings.directory)
    before = loaded.path.read_bytes()
    assert loaded.configuration["process_groups"][0]["parameters"]["config_path"] == ""
    detected = loaded.detect_environment(loaded.configuration)
    assert detected.configuration["process_groups"][0]["parameters"]["config_path"] == str(camera)
    assert loaded.path.read_bytes() == before
    assert loaded.configuration["process_groups"][0]["parameters"]["config_path"] == ""


def test_private_atomic_save_roundtrip_and_copy_isolation(tmp_path):
    settings = WorkstationSettings(tmp_path, tmp_path / "private")
    value = settings.configuration
    value["process_groups"][0]["label"] = "本机配置"
    assert settings.configuration["process_groups"][0]["label"] != "本机配置"
    settings.save(value)
    assert settings.directory.stat().st_mode & 0o777 == 0o700
    assert settings.path.stat().st_mode & 0o777 == 0o600
    assert WorkstationSettings(tmp_path, settings.directory).configuration == settings.configuration
    assert json.loads(settings.path.read_text())["process_groups"][0]["label"] == "本机配置"
    assert not list(settings.directory.glob("*.tmp"))


@pytest.mark.parametrize("reliability", [None, "best_effort", "reliable"])
def test_image_reliability_survives_private_settings_roundtrip(tmp_path, reliability):
    settings = WorkstationSettings(tmp_path, tmp_path / "private")
    device = dict(id="camera", label="Camera", kind="camera", side="", topic="/custom/image")
    if reliability is not None:
        device["image_reliability"] = reliability
    value = settings.configuration
    value["devices"] = [device]
    settings.save(value)
    assert WorkstationSettings(tmp_path, settings.directory).configuration["devices"] == [device]


@pytest.mark.parametrize("change", [
    lambda d: d.update(image_reliability="auto"),
    lambda d: d.update(image_reliability=True),
    lambda d: d.update(kind="tracker", image_reliability="reliable"),
    lambda d: d.update(unknown="reliable"),
    lambda d: d.pop("topic"),
])
def test_invalid_image_subscription_cannot_replace_saved_configuration(tmp_path, change):
    settings = WorkstationSettings(tmp_path, tmp_path / "private")
    settings.save(settings.configuration)
    before = settings.path.read_bytes()
    device = dict(id="camera", label="Camera", kind="camera", side="", topic="/custom/image")
    change(device)
    value = settings.configuration
    value["devices"] = [device]
    with pytest.raises(ValueError):
        settings.save(value)
    assert settings.path.read_bytes() == before


@pytest.mark.parametrize("change", [
    lambda value: value.update(argv=["sh", "-c", "true"]),
    lambda value: value["process_groups"][0].update(argv=["arbitrary"]),
    lambda value: value["process_groups"][0]["parameters"].update(injected="true"),
    lambda value: value["process_groups"][0]["parameters"].update(config_path="relative.yaml"),
    lambda value: value["process_groups"][0].update(enabled=True),
    lambda value: value.update(stale_after_s=float("nan")),
    lambda value: value.update(quality_timeout_s=True),
])
def test_invalid_configuration_never_replaces_saved_file(tmp_path, change):
    settings = WorkstationSettings(tmp_path, tmp_path / "private")
    settings.save(settings.configuration)
    before = settings.path.read_bytes()
    value = settings.configuration
    # Ensure camera enable is invalid even on a developer's workstation.
    value["process_groups"][0]["parameters"].update(config_path="", output_root="")
    change(value)
    with pytest.raises(ValueError):
        settings.save(value)
    assert settings.path.read_bytes() == before


def test_shared_validator_normalizes_without_mutating_input(tmp_path):
    value = default_configuration(tmp_path)
    value["process_groups"][0].pop("startup_timeout_s")
    original = deepcopy(value)
    normalized = validate_configuration(value)
    assert value == original
    assert normalized["process_groups"][0]["startup_timeout_s"] == 30.
    assert list(parse_profiles(value["process_groups"])) == normalized["process_groups"]


def test_untrusted_permissions_and_symlinks_are_refused(tmp_path):
    public = tmp_path / "public"
    public.mkdir(mode=0o755)
    with pytest.raises(ValueError):
        WorkstationSettings(tmp_path, public)
    settings = WorkstationSettings(tmp_path, tmp_path / "private")
    other = tmp_path / "elsewhere.json"
    other.write_text("{}")
    settings.path.symlink_to(other)
    with pytest.raises(OSError):
        WorkstationSettings(tmp_path, settings.directory)


def test_serial_choices_enumerate_only_and_prefer_stable_identity(tmp_path, monkeypatch):
    dev = tmp_path / "dev"
    by_id = dev / "serial/by-id"
    by_path = dev / "serial/by-path"
    by_id.mkdir(parents=True)
    by_path.mkdir(parents=True)
    first, second = dev / "ttyACM0", dev / "ttyUSB1"
    first.touch()
    second.touch()
    (by_id / "usb-explicit-device").symlink_to(first)
    (by_path / "pci-same-device").symlink_to(first)
    (by_id / "unplugged").symlink_to(dev / "ttyUSB9")
    monkeypatch.setattr(os, "open", lambda *args, **kwargs: pytest.fail("enumeration opened a device"))
    choices = serial_ports(dev)
    assert [value["path"] for value in choices] == [str(by_id / "usb-explicit-device"), str(second)]
    assert "ttyACM0" in choices[0]["label"]
