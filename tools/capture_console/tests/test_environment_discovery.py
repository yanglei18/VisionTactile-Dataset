"""Filesystem-only deployment discovery; no device or dependency execution."""
from copy import deepcopy
from pathlib import Path

import pytest

from vt_capture_console.backend.environment_discovery import (
    default_data_root, discover_configuration,
)


SOURCE_MARKERS = (
    "src/hand_visualiser/modules/hand/calibration.py",
    "src/hand_visualiser/modules/utils.py",
    "src/hand_visualiser/modules/enumerates.py",
    "src/hand_visualiser/modules/datamodels.py",
)
TRACKER_MARKERS = (
    "pyvut/live_hid.py", "pyvut/live_probe.py", "pyvut/live_bootstrap_bundle.py", "pyvut/pose_decoder.py",
)


def files(root, names):
    for name in names:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture: discovery must not read this")
    return root


def interpreter(path):
    files(path.parent, [path.name])
    path.chmod(0o700)
    return path


def configuration():
    return {
        "quality_enabled": False,
        "custom": {"preserve": [1, 2]},
        "process_groups": [
            dict(id="my_cameras", preset="cameras", enabled=False,
                 parameters={"config_path": "", "output_root": ""}),
            dict(id="my_trackers", preset="trackers", enabled=False, parameters={}),
            dict(id="my_right", preset="glove", enabled=False,
                 parameters={"hand": "right", "config": "", "output_rate_hz": "120"}),
            dict(id="my_left", preset="glove", enabled=True, parameters={"hand": "left"}),
            dict(id="touch", preset="tactile", enabled=False, parameters={"hand": "right"}),
        ],
    }


def run(tmp_path, value=None, **kwargs):
    return discover_configuration(
        configuration() if value is None else value,
        repo_root=tmp_path / "repo", home=tmp_path / "home",
        environ=kwargs.pop("environ", {}), **kwargs,
    )


def parameters(result, group):
    return next(item["parameters"] for item in result.configuration["process_groups"]
                if item["id"] == group)


def finding(result, group, key):
    return next(item for item in result.findings if (item.group_id, item.key) == (group, key))


def test_new_machine_suggests_output_without_creating_it_and_preserves_structure(tmp_path):
    source = configuration()
    before = deepcopy(source)
    result = run(tmp_path, source)
    output = tmp_path / "home/visiontactile-data"
    assert parameters(result, "my_cameras")["output_root"] == str(output)
    assert finding(result, "my_cameras", "output_root").status == "default"
    assert ("my_cameras", "output_root") in result.updated_fields
    assert not output.exists()
    assert source == before
    assert result.configuration["custom"] == before["custom"]
    assert [(g["id"], g["enabled"]) for g in result.configuration["process_groups"]] == [
        (g["id"], g["enabled"]) for g in before["process_groups"]
    ]
    assert finding(result, "my_right", "port").status == "manual"
    assert finding(result, "touch", "port").status == "manual"
    assert not any(item.key == "config" for item in result.findings)


def test_data_root_accepts_explicit_absolute_path_and_tilde_without_shell_expansion(tmp_path):
    home = tmp_path / "home"
    assert default_data_root(home=home, environ={}) == home / "visiontactile-data"
    assert default_data_root(home=home, environ={"VT_DATA_ROOT": "~/measurements"}) == home / "measurements"
    assert default_data_root(home=home, environ={"VT_DATA_ROOT": str(tmp_path / "disk")}) == tmp_path / "disk"
    with pytest.raises(ValueError):
        default_data_root(home=home, environ={"VT_DATA_ROOT": "$HOME/measurements"})


def test_local_camera_and_saved_output_win_over_repository_and_environment(tmp_path):
    root, output = tmp_path / "repo", tmp_path / "chosen-data"
    files(root, ["ros2_ws/src/vt_realsense_capture/config/cameras.yaml"])
    files(output, ["config/cameras.yaml"])
    source = configuration()
    source["process_groups"][0]["parameters"]["output_root"] = str(output)
    result = run(tmp_path, source, environ={"VT_DATA_ROOT": str(tmp_path / "another")})
    assert parameters(result, "my_cameras") == {
        "output_root": str(output), "config_path": str(output / "config/cameras.yaml"),
    }
    result = run(tmp_path, source, environ={"VT_DATA_ROOT": "$invalid-relative"})
    assert parameters(result, "my_cameras")["output_root"] == str(output)


def test_repository_templates_and_scene_are_resolved_against_current_checkout(tmp_path):
    root = tmp_path / "repo"
    camera = "ros2_ws/src/vt_realsense_capture/config/cameras.yaml"
    scene = "ros2_ws/src/vt_capture_manager/config/scene.example.yaml"
    files(root, [camera, scene])
    result = run(tmp_path)
    assert parameters(result, "my_cameras")["config_path"] == str(root / camera)
    assert result.configuration["scene_config_path"] == str(root / scene)
    assert ("", "scene_config_path") in result.updated_fields


@pytest.mark.parametrize("group,key", [
    ("my_cameras", "config_path"), ("my_trackers", "bundle_path"),
    ("my_trackers", "role_map_path"), ("", "scene_config_path"),
])
def test_missing_explicit_hardware_configuration_is_never_replaced(tmp_path, group, key):
    source = configuration()
    target = source if not group else next(g["parameters"] for g in source["process_groups"] if g["id"] == group)
    target[key] = str(tmp_path / "old-computer" / key)
    files(tmp_path / "repo", [
        "ros2_ws/src/vt_realsense_capture/config/cameras.yaml",
        "ros2_ws/src/vt_capture_manager/config/scene.example.yaml",
    ])
    files(tmp_path / "home/visiontactile-data", ["tracker-validation/roles.yaml"])
    files(tmp_path, ["new-bundle.json", "new-roles.yaml"])
    result = run(tmp_path, source, replace_missing=True, environ={
        "VT_BUNDLE": str(tmp_path / "new-bundle.json"),
        "VT_ROLE_MAP": str(tmp_path / "new-roles.yaml"),
    })
    actual = result.configuration if not group else parameters(result, group)
    assert actual[key] == target[key]
    assert finding(result, group, key).status == "missing"
    assert (group, key) not in result.updated_fields


def test_valid_explicit_software_paths_preserved_over_discovery(tmp_path):
    chosen = interpreter(tmp_path / "custom/bin/python")
    interpreter(tmp_path / "repo/.venv-glove/bin/python")
    other = interpreter(tmp_path / "env/bin/python")
    source = configuration()
    source["process_groups"][2]["parameters"]["glove_python"] = str(chosen)
    result = run(tmp_path, source, replace_missing=True,
                 environ={"VT_FSGLOVE_PYTHON": str(other)})
    assert parameters(result, "my_right")["glove_python"] == str(chosen)
    assert finding(result, "my_right", "glove_python").status == "ready"


def test_software_repair_requires_explicit_replace_flag(tmp_path):
    candidate = interpreter(tmp_path / "repo/.venv-glove/bin/python")
    source = configuration()
    source["process_groups"][2]["parameters"]["glove_python"] = str(tmp_path / "old/bin/python")
    assert parameters(run(tmp_path, source), "my_right")["glove_python"] != str(candidate)
    result = run(tmp_path, source, replace_missing=True)
    assert parameters(result, "my_right")["glove_python"] == str(candidate)
    assert ("my_right", "glove_python") in result.updated_fields


def test_interpreters_must_be_executable_and_multiple_candidates_are_ambiguous(tmp_path):
    first = interpreter(tmp_path / "repo/.venv-glove/bin/python")
    second = interpreter(tmp_path / "home/.venvs/vt-fsglove/bin/python")
    files(tmp_path / "repo/.venv-fsglove/bin", ["python"])
    result = run(tmp_path)
    item = finding(result, "my_right", "glove_python")
    assert item.status == "ambiguous"
    assert set(item.candidates) == {str(first), str(second)}
    assert not parameters(result, "my_right").get("glove_python")


def test_symlink_aliases_of_same_candidate_are_not_ambiguous(tmp_path):
    first = interpreter(tmp_path / "repo/.venv-glove/bin/python")
    alias = tmp_path / "repo/.venv-fsglove"
    alias.symlink_to(first.parents[1], target_is_directory=True)
    assert finding(run(tmp_path), "my_right", "glove_python").status == "ready"


def test_distinct_virtualenvs_sharing_system_python_are_still_ambiguous(tmp_path):
    binary = interpreter(tmp_path / "system/bin/python")
    for relative in (".venv-glove/bin/python", ".venv-fsglove/bin/python"):
        path = tmp_path / "repo" / relative
        path.parent.mkdir(parents=True)
        path.symlink_to(binary)
    assert finding(run(tmp_path), "my_right", "glove_python").status == "ambiguous"


@pytest.mark.parametrize("variable", ["VT_FSGLOVE_PYTHON", "VT_FSGLOVE_SOURCE", "VT_MANO_ASSETS"])
def test_invalid_explicit_environment_does_not_fall_back(tmp_path, variable):
    interpreter(tmp_path / "repo/.venv-glove/bin/python")
    files(tmp_path / "repo/external/FSGlove", SOURCE_MARKERS)
    files(tmp_path / "repo/external/mano-assets", ["models/MANO_RIGHT.pkl"])
    result = run(tmp_path, environ={variable: str(tmp_path / "uninstalled")})
    key = {"VT_FSGLOVE_PYTHON": "glove_python", "VT_FSGLOVE_SOURCE": "fsglove_source",
           "VT_MANO_ASSETS": "mano_assets"}[variable]
    assert finding(result, "my_right", key).status == "missing"
    assert not parameters(result, "my_right").get(key)


def test_source_markers_and_mano_hand_are_checked_without_imports(tmp_path):
    source = files(tmp_path / "repo/external/FSGlove", SOURCE_MARKERS)
    assets = files(tmp_path / "repo/external/mano-assets", ["models/MANO_RIGHT.pkl"])
    result = run(tmp_path)
    assert parameters(result, "my_right")["fsglove_source"] == str(source)
    assert parameters(result, "my_right")["mano_assets"] == str(assets)
    assert finding(result, "my_left", "mano_assets").status == "missing"
    (source / SOURCE_MARKERS[0]).unlink()
    assert finding(run(tmp_path), "my_right", "fsglove_source").status == "missing"


def test_existing_upstream_environment_names_are_supported(tmp_path):
    source = files(tmp_path / "source", SOURCE_MARKERS)
    assets = files(tmp_path / "assets", ["models/MANO_RIGHT.pkl"])
    result = run(tmp_path, environ={
        "FSGLOVE12_UPSTREAM_CHECKOUT": str(source),
        "FSGLOVE12_MANO_ASSETS": str(assets),
    })
    assert parameters(result, "my_right")["fsglove_source"] == str(source)
    assert parameters(result, "my_right")["mano_assets"] == str(assets)


def test_tracker_requires_extended_source_and_supports_python_versions(tmp_path):
    source = files(tmp_path / "home/.local/share/visiontactile/pyvut-src", TRACKER_MARKERS)
    site = tmp_path / "home/.local/share/visiontactile/pyvut-venv/lib/python3.14/site-packages"
    site.mkdir(parents=True)
    result = run(tmp_path)
    assert parameters(result, "my_trackers")["pyvut_source"] == str(source)
    assert parameters(result, "my_trackers")["pyvut_site_packages"] == str(site)
    (source / TRACKER_MARKERS[0]).unlink()
    assert finding(run(tmp_path), "my_trackers", "pyvut_source").status == "missing"
    second = site.parents[1] / "python3.12/site-packages"
    second.mkdir(parents=True)
    assert finding(run(tmp_path), "my_trackers", "pyvut_site_packages").status == "ambiguous"


def test_explicit_tracker_paths_never_read_capture_or_private_file_contents(tmp_path, monkeypatch):
    source = files(tmp_path / "extended", TRACKER_MARKERS)
    site = tmp_path / "venv/lib/python3.13/site-packages"
    site.mkdir(parents=True)
    files(tmp_path, ["approved.json", "roles.yaml"])
    def forbidden(*args, **kwargs):
        pytest.fail("discovery attempted to read or execute")
    monkeypatch.setattr(Path, "read_text", forbidden)
    monkeypatch.setattr(Path, "read_bytes", forbidden)
    monkeypatch.setattr("subprocess.Popen", forbidden)
    monkeypatch.setattr("builtins.open", forbidden)
    monkeypatch.setattr("os.open", forbidden)
    result = run(tmp_path, environ={
        "VT_PYVUT_ROOT": str(source), "VT_PYVUT_VENV": str(tmp_path / "venv"),
        "VT_BUNDLE": str(tmp_path / "approved.json"),
        "VT_ROLE_MAP": str(tmp_path / "roles.yaml"),
    })
    assert parameters(result, "my_trackers")["bundle_path"] == str(tmp_path / "approved.json")
    assert parameters(result, "my_trackers")["role_map_path"] == str(tmp_path / "roles.yaml")
    assert parameters(result, "my_trackers")["pyvut_source"] == str(source)
    assert parameters(result, "my_trackers")["pyvut_site_packages"] == str(site)


def test_empty_optional_config_and_unknown_parameters_are_not_rewritten(tmp_path):
    source = configuration()
    source["process_groups"][2]["parameters"].update(socket_path="/unavailable/socket", launch_gui="false")
    result = run(tmp_path, source)
    assert parameters(result, "my_right")["socket_path"] == "/unavailable/socket"
    assert parameters(result, "my_right")["config"] == ""
    assert parameters(result, "my_right")["launch_gui"] == "false"
    assert not any(item.key in {"config", "socket_path"} for item in result.findings)


def test_detection_does_not_add_profiles_to_empty_configuration(tmp_path):
    result = run(tmp_path, {"process_groups": [], "quality_enabled": False})
    assert result.configuration["process_groups"] == []


def test_disk_path_defaults_to_camera_output_and_missing_value_requires_explicit_repair(tmp_path):
    source = configuration()
    output = tmp_path / "output"
    output.mkdir()
    source["process_groups"][0]["parameters"]["output_root"] = str(output)
    assert run(tmp_path, source).configuration["disk_path"] == str(output)
    source["disk_path"] = str(tmp_path / "previous-machine")
    assert run(tmp_path, source).configuration["disk_path"] == source["disk_path"]
    assert run(tmp_path, source, replace_missing=True).configuration["disk_path"] == str(output)
    other = tmp_path / "monitored"
    other.mkdir()
    source["disk_path"] = str(other)
    assert run(tmp_path, source, replace_missing=True).configuration["disk_path"] == str(other)


def test_documented_environment_aliases_and_standard_user_locations(tmp_path):
    source = files(tmp_path / "home/.local/share/visiontactile/fsglove", SOURCE_MARKERS)
    assets = files(tmp_path / "home/visiontactile-data/models/mano", ["models/MANO_RIGHT.pkl"])
    site = tmp_path / "dependency-site"
    site.mkdir()
    result = run(tmp_path)
    assert parameters(result, "my_right")["fsglove_source"] == str(source)
    assert parameters(result, "my_right")["mano_assets"] == str(assets)
    result = run(tmp_path, environ={
        "FSGLOVE_SOURCE": str(source), "MANO_ASSETS": str(assets), "VT_PYVUT_SITE": str(site),
    })
    assert parameters(result, "my_right")["fsglove_source"] == str(source)
    assert parameters(result, "my_right")["mano_assets"] == str(assets)
    assert parameters(result, "my_trackers")["pyvut_site_packages"] == str(site)


def test_missing_custom_output_stays_visible_and_is_not_silently_redirected(tmp_path):
    source = configuration()
    previous = str(tmp_path / "previous-computer/data")
    source["process_groups"][0]["parameters"]["output_root"] = previous
    result = run(tmp_path, source, replace_missing=True)
    assert parameters(result, "my_cameras")["output_root"] == previous
    assert finding(result, "my_cameras", "output_root").status == "missing"
    assert finding(result, "", "disk_path").status == "missing"
    assert ("my_cameras", "output_root") not in result.updated_fields


def test_custom_camera_output_does_not_redefine_the_creatable_default(tmp_path):
    value = configuration()
    output = tmp_path / "custom"
    output.mkdir()
    value["process_groups"][0]["parameters"]["output_root"] = str(output)
    value["disk_path"] = str(output / "not-created")
    result = run(tmp_path, value)
    assert finding(result, "", "disk_path").status == "missing"
    value["process_groups"].insert(0, dict(
        id="missing_camera", preset="cameras", enabled=False,
        parameters={"output_root": str(tmp_path / "missing"), "config_path": ""},
    ))
    result = run(tmp_path, value)
    assert finding(result, "missing_camera", "output_root").status == "missing"


def test_explicit_data_root_is_the_only_missing_output_reported_as_default(tmp_path):
    target = tmp_path / "new-target"
    result = run(tmp_path, environ={"VT_DATA_ROOT": str(target)})
    assert finding(result, "my_cameras", "output_root").status == "default"
    assert finding(result, "", "disk_path").status == "default"
    assert not target.exists()


@pytest.mark.parametrize("variable,key", [
    ("VT_PYVUT_ROOT", "pyvut_source"), ("VT_PYVUT_VENV", "pyvut_site_packages"),
    ("VT_PYVUT_SITE", "pyvut_site_packages"),
])
def test_invalid_tracker_environment_does_not_fall_back_to_standard_location(tmp_path, variable, key):
    files(tmp_path / "home/.local/share/visiontactile/pyvut-src", TRACKER_MARKERS)
    (tmp_path / "home/.local/share/visiontactile/pyvut-venv/lib/python3.14/site-packages").mkdir(parents=True)
    result = run(tmp_path, environ={variable: str(tmp_path / "missing")})
    assert finding(result, "my_trackers", key).status == "missing"
    assert not parameters(result, "my_trackers").get(key)


def test_explicit_bundle_in_checkout_is_not_suggested_as_usable(tmp_path):
    files(tmp_path / "repo", ["private.json"])
    result = run(tmp_path, environ={"VT_BUNDLE": str(tmp_path / "repo/private.json")})
    assert finding(result, "my_trackers", "bundle_path").status == "missing"
    assert not parameters(result, "my_trackers").get("bundle_path")


def test_unknown_group_and_optional_absent_configuration_remain_unchanged(tmp_path):
    value = configuration()
    extra = dict(id="future", preset="future_extension", parameters={"path": "/explicit"}, enabled=True)
    value["process_groups"].append(extra)
    assert run(tmp_path, value).configuration["process_groups"][-1] == extra


def test_permission_errors_are_reported_as_missing_candidates(tmp_path, monkeypatch):
    original = Path.is_file
    def inaccessible(path):
        if "cameras.yaml" == path.name:
            raise PermissionError("fixture")
        return original(path)
    monkeypatch.setattr(Path, "is_file", inaccessible)
    assert finding(run(tmp_path), "my_cameras", "config_path").status == "missing"
