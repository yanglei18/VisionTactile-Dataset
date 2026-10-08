from copy import deepcopy
import json
import os
from pathlib import Path

import pytest

from vt_capture_console.backend.workstation_settings import (
    WorkstationSettings, default_configuration, serial_ports, validate_configuration,
)
from vt_capture_core.workstation import parse_profiles


def test_defaults_are_six_disabled_profiles_without_port_guessing(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "home")
    root = tmp_path / "repo"
    root.mkdir()
    value = default_configuration(root)
    profiles = value["process_groups"]
    assert len(profiles) == 6 and not any(group["enabled"] for group in profiles)
    assert {group["id"] for group in profiles} == {"cameras", "trackers", "right_glove", "left_glove", "right_tactile", "left_tactile"}
    assert profiles[0]["parameters"] == dict(config_path="", output_root="")
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
    value = default_configuration(root)
    assert value["disk_path"] == str(output)
    assert value["process_groups"][0]["parameters"] == dict(config_path=str(camera), output_root=str(output))
    assert value["process_groups"][2]["parameters"]["glove_python"] == str(interpreter)


def test_known_assets_are_suggestions_only_and_right_calibration_is_not_copied_left(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "home")
    baseline = tmp_path / "artifacts/glove_hand_pose_live/upstream_baselines_20260930"
    source, assets = baseline / "fsglove-a7988f0", baseline / "mano-assets"
    source.mkdir(parents=True)
    assets.mkdir()
    calibration = tmp_path / ".runtime/recovery/right_current_live_20261005.json"
    calibration.parent.mkdir(parents=True)
    calibration.write_text("do not read or copy these calibration contents")
    role_map = tmp_path / "visiontactile-data/tracker-validation/roles.yaml"
    role_map.parent.mkdir(parents=True)
    role_map.write_text("do not read identifiers")
    groups = {value["id"]: value for value in default_configuration(tmp_path)["process_groups"]}
    for hand in ("right", "left"):
        value = groups[hand + "_glove"]
        assert value["parameters"]["fsglove_source"] == str(source)
        assert value["parameters"]["mano_assets"] == str(assets)
        assert value["parameters"]["port"] == "" and not value["enabled"]
    assert groups["right_glove"]["parameters"]["config"] == str(calibration)
    assert groups["left_glove"]["parameters"]["config"] == ""
    assert groups["trackers"]["parameters"]["role_map_path"] == str(role_map)
    assert groups["trackers"]["parameters"]["bundle_path"] == ""


def test_only_existing_explicit_tracker_environment_directories_are_suggested(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "home")
    local = tmp_path / ".local/share/visiontactile"
    source = local / "pyvut-src/.worktrees/capture-locked-live-bootstrap"
    site = local / "pyvut-venv/lib/python3.12/site-packages"
    before = next(profile for profile in default_configuration(tmp_path)["process_groups"] if profile["preset"] == "trackers")
    assert before["parameters"]["pyvut_source"] == before["parameters"]["pyvut_site_packages"] == ""
    source.mkdir(parents=True)
    site.mkdir(parents=True)
    profile = next(profile for profile in default_configuration(tmp_path)["process_groups"] if profile["preset"] == "trackers")
    assert profile["parameters"]["pyvut_source"] == str(source)
    assert profile["parameters"]["pyvut_site_packages"] == str(site)
    assert not profile["enabled"]


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
