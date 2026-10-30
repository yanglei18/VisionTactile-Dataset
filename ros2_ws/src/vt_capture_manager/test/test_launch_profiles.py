"""Profiles resolve argv only; none of these tests launches sensor processes."""
import json

import pytest

from vt_capture_manager.config import load_config
from vt_capture_manager.launch_profiles import build_specs, parse_profiles


def profile(preset="tactile", **changes):
    value = dict(id="right_tactile", label="右手触觉", preset=preset, enabled=True,
                 parameters=dict(hand="right", port="/dev/serial/by-id/test"))
    value.update(changes)
    return value


def test_default_config_does_not_enable_device_launch():
    assert load_config().process_groups == ()


def test_hub_profile_uses_same_topics_without_bootstrap_or_hid_python():
    p = dict(backend="vivehub", frame_id="vive_hub_map", hub_bindings_path="/private/roles.yaml",
             hub_reader_path="/runtime/vt-vivehub-reader", hub_socket_path="/tmp/vut.sock",
             pyvut_source="/old/source", bundle_path="/old/bundle.json")
    spec, = build_specs(parse_profiles([profile(preset="trackers", parameters=p)]))
    assert spec.argv[:4] == ("ros2", "launch", "vt_vive_tracker", "triple_tracker.launch.py")
    assert "backend:=vivehub" in spec.argv
    assert not any("bundle_path:=" in a or "PYTHONPATH=" in a for a in spec.argv)
    assert spec.topics == tuple(f"/vive/{r}/sample" for r in ("left_wrist", "right_wrist", "torso"))


@pytest.mark.parametrize("change", [{"frame_id": "vive_map"}, {"hub_bindings_path": ""},
                                    {"hub_reader_path": "relative"}, {"backend": "unknown"}])
def test_hub_rejects_legacy_frame_or_incomplete_configuration(change):
    p = dict(backend="vivehub", frame_id="vive_hub_map", hub_bindings_path="/private/roles.yaml",
             hub_reader_path="/runtime/vt-vivehub-reader", hub_socket_path="/tmp/vut.sock")
    p.update(change)
    with pytest.raises(ValueError):
        parse_profiles([profile(preset="trackers", parameters=p)])


def test_profiles_are_explicit_and_only_build_existing_launch_argv(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"process_groups": [profile()]}))
    spec, = build_specs(load_config(path).process_groups)
    assert spec.argv[:4] == ("ros2", "launch", "vt_tactile_capture", "tactile.launch.py")
    assert "gui:=false" in spec.argv and "namespace:=/gloves/right/tactile" in spec.argv
    assert spec.topics == ("/gloves/right/tactile/state",)
    assert spec.enabled


@pytest.mark.parametrize("change", [
    dict(argv=["touch", "/tmp/not-allowed"]), dict(preset="shell"), dict(enabled="true"),
    dict(parameters=dict(hand="right", port="relative")), dict(parameters=dict(hand="both")),
    dict(parameters=dict(hand="right", port="")), dict(parameters=dict(hand="right", port="/dev/test", gui="true")),
    dict(id="../../bad"), dict(label=""), dict(startup_timeout_s=float("nan")),
    dict(parameters=dict(hand="right", port="/dev/x\nother")),
])
def test_reject_ambiguous_or_unapproved_commands(change):
    with pytest.raises(ValueError):
        parse_profiles([profile(**change)])


def test_disabled_draft_can_omit_machine_paths():
    result = parse_profiles([profile(preset="cameras", parameters={}, enabled=False)])
    spec, = build_specs(result)
    assert not spec.enabled
    assert "/capture/status" in spec.topics
    assert "capture-controller" in spec.resources


def test_camera_profile_supplies_config_to_read_only_usb_guard():
    spec, = build_specs(parse_profiles([profile(preset="cameras", parameters=dict(
        config_path="/project/cameras.yaml", output_root="/capture/data"))]))
    assert "camera-config:/project/cameras.yaml" in spec.resources
    assert "config_path:=/project/cameras.yaml" in spec.argv


def test_glove_gui_requires_runtime_assets_but_bridge_only_does_not():
    with pytest.raises(ValueError):
        parse_profiles([profile(preset="glove", parameters=dict(hand="left"))])
    spec, = build_specs(parse_profiles([profile(preset="glove", parameters=dict(hand="left", launch_gui="false"))]))
    assert spec.topics == ("/gloves/left/state",)
    assert "launch_gui:=false" in spec.argv


def test_duplicate_ids_and_shared_serial_aliases(tmp_path):
    with pytest.raises(ValueError):
        parse_profiles([profile(), profile()])
    port = tmp_path / "port"
    port.touch()
    alias = tmp_path / "alias"
    alias.symlink_to(port)
    profiles = parse_profiles([
        profile(parameters=dict(hand="right", port=str(port))),
        profile(id="left_tactile", parameters=dict(hand="left", port=str(alias))),
    ])
    a, b = build_specs(profiles)
    assert set(a.resolved().resources) & set(b.resolved().resources) == {"serial:" + str(port)}


def test_tracker_python_paths_are_fixed_env_prefix_not_ros_arguments(monkeypatch):
    monkeypatch.setenv("PYTHONPATH", "/existing/ros:/existing/core")
    parameters = dict(bundle_path="/private/bundle.json", role_map_path="/private/roles.yaml",
                      pyvut_source="/approved/source with spaces", pyvut_site_packages="/approved/env/site-packages")
    spec, = build_specs(parse_profiles([profile(preset="trackers", parameters=parameters)]))
    assert spec.argv[:6] == ("/usr/bin/env", "PYTHONPATH=/approved/source with spaces:/approved/env/site-packages:/existing/ros:/existing/core",
                            "ros2", "launch", "vt_vive_tracker", "triple_tracker.launch.py")
    assert "bundle_path:=/private/bundle.json" in spec.argv
    assert "role_map_path:=/private/roles.yaml" in spec.argv
    assert not any(argument.startswith(("pyvut_source:=", "pyvut_site_packages:=")) for argument in spec.argv)
    assert parameters["pyvut_source"] == "/approved/source with spaces"


@pytest.mark.parametrize("parameters", [dict(), dict(pyvut_source="", pyvut_site_packages="")])
def test_tracker_legacy_profiles_keep_existing_environment(parameters, monkeypatch):
    monkeypatch.setenv("PYTHONPATH", "/legacy/approved/env")
    parameters.update(bundle_path="/private/bundle.json", role_map_path="/private/roles.yaml")
    spec, = build_specs(parse_profiles([profile(preset="trackers", parameters=parameters)]))
    assert spec.argv[:4] == ("ros2", "launch", "vt_vive_tracker", "triple_tracker.launch.py")


def test_tracker_source_only_prefix_without_inherited_path(monkeypatch):
    monkeypatch.delenv("PYTHONPATH", raising=False)
    parameters = dict(bundle_path="/private/bundle.json", role_map_path="/private/roles.yaml", pyvut_source="/approved/source")
    spec, = build_specs(parse_profiles([profile(preset="trackers", parameters=parameters)]))
    assert spec.argv[:2] == ("/usr/bin/env", "PYTHONPATH=/approved/source")


@pytest.mark.parametrize("key", ["pyvut_source", "pyvut_site_packages"])
@pytest.mark.parametrize("path", ["relative/path", "/approved:/other", "/approved\n/other"])
def test_tracker_python_paths_are_single_explicit_directories(key, path):
    parameters = dict(bundle_path="/private/bundle.json", role_map_path="/private/roles.yaml", **{key: path})
    with pytest.raises(ValueError):
        parse_profiles([profile(preset="trackers", parameters=parameters)])


def test_python_path_fields_are_not_general_environment_overrides():
    parameters = dict(hand="right", port="/dev/test", pyvut_source="/approved/source")
    with pytest.raises(ValueError):
        parse_profiles([profile(parameters=parameters)])
