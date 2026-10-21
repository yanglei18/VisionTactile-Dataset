"""Managed Tracker installations supply coherent defaults without hardware access."""
from copy import deepcopy
import builtins
import importlib
import json
import os
from pathlib import Path
import sys

import pytest

from vt_capture_console.backend.environment_discovery import (
    discover_bootstrap_configuration, discover_configuration,
)
from vt_capture_console.backend.tracker_settings import TrackerBootstrapSettings


@pytest.fixture
def machine(tmp_path, monkeypatch):
    home, root = tmp_path / "home", tmp_path / "repo"
    home.mkdir()
    root.mkdir()
    monkeypatch.setattr(Path, "home", lambda: home)
    for key in ("VT_PYVUT_ROOT", "VT_PYVUT_VENV", "VT_PYVUT_SITE",
                "VT_CAPTURE", "VT_CAPTURE_SHA256", "VT_BUNDLE", "VT_ROLE_MAP"):
        monkeypatch.delenv(key, raising=False)
    return home, root


def install(home, char):
    identity = char * 64
    base = home / ".local/share/visiontactile/tracker-deployment"
    version = base / "versions" / identity
    version.mkdir(parents=True)
    for directory in (base, base / "versions", version):
        directory.chmod(0o700)
    paths = dict(source="source", python="venv/bin/python",
                 site_packages="venv/lib/python3.12/site-packages",
                 bundle="inputs/bundle.json", roles="inputs/roles.yaml", capture="inputs/capture.pcapng")
    for relative in ("source/pyvut/live_hid.py", "source/pyvut/live_probe.py", "source/pyvut/live_bootstrap_bundle.py",
                     "source/pyvut/pose_decoder.py", "source/scripts/live_windows_bootstrap.py",
                     paths["bundle"], paths["roles"], paths["capture"]):
        path = version / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("must not read or execute")
        path.chmod(0o600)
    (version / "venv/bin").mkdir(parents=True)
    (version / paths["python"]).symlink_to(sys.executable)
    (version / paths["site_packages"]).mkdir(parents=True)
    descriptor = version / "installation.json"
    descriptor.write_text(json.dumps(dict(schema_version=1, installation_id=identity,
                                         capture_sha256=char * 64, paths=paths)))
    descriptor.chmod(0o600)
    current = base / "current.json"
    current.write_text(json.dumps(dict(schema_version=1, installation_id=identity)))
    current.chmod(0o600)
    return {key: str(version / relative) for key, relative in paths.items()} | {"digest": char * 64}


def config(parameters=None):
    return {"process_groups": [dict(id="operator_trackers", preset="trackers", enabled=False,
                                   parameters=parameters or {})]}


def discover(machine, value, **kwargs):
    home, root = machine
    return discover_configuration(value, repo_root=root, home=home, environ=kwargs.pop("environ", {}), **kwargs)


def test_fresh_workstation_uses_installed_bundle_role_and_software_as_one_group(machine):
    deployment = install(machine[0], "a")
    original = config()
    result = discover(machine, original)
    assert result.configuration["process_groups"][0]["parameters"] == {
        "pyvut_source": deployment["source"], "pyvut_site_packages": deployment["site_packages"],
        "bundle_path": deployment["bundle"], "role_map_path": deployment["roles"],
    }
    assert original == config()
    assert not result.configuration["process_groups"][0]["enabled"]


def test_installed_runtime_precedes_generic_candidates_but_keeps_manual_and_environment_choices(machine):
    deployment = install(machine[0], "a")
    manual = Path(deployment["source"]).parent / "manual-source"
    manual.mkdir()
    for name in ("live_hid.py", "live_probe.py", "live_bootstrap_bundle.py", "pose_decoder.py"):
        path = manual / "pyvut" / name
        path.parent.mkdir(exist_ok=True)
        path.touch()
    result = discover(machine, config({"pyvut_source": str(manual)}),
                       environ={"VT_PYVUT_ROOT": "/missing/explicit"})
    assert result.configuration["process_groups"][0]["parameters"]["pyvut_source"] == str(manual)
    result = discover(machine, config(), environ={"VT_PYVUT_ROOT": str(manual)})
    assert result.configuration["process_groups"][0]["parameters"]["pyvut_source"] == str(manual)
    result = discover(machine, config(), environ={"VT_PYVUT_ROOT": "/missing/explicit"})
    assert not result.configuration["process_groups"][0]["parameters"].get("pyvut_source")


def test_previous_managed_bundle_selects_previous_software_and_roles_after_activation_changes(machine):
    previous = install(machine[0], "a")
    current = install(machine[0], "b")
    result = discover(machine, config({"bundle_path": previous["bundle"]}))
    parameters = result.configuration["process_groups"][0]["parameters"]
    assert parameters["bundle_path"] == previous["bundle"]
    assert parameters["role_map_path"] == previous["roles"]
    assert parameters["pyvut_source"] == previous["source"] != current["source"]
    assert parameters["pyvut_site_packages"] == previous["site_packages"]


def test_explicit_bundle_environment_selects_its_own_installed_roles(machine):
    previous = install(machine[0], "a")
    install(machine[0], "b")
    result = discover(machine, config(), environ={"VT_BUNDLE": previous["bundle"]})
    parameters = result.configuration["process_groups"][0]["parameters"]
    assert parameters["bundle_path"] == previous["bundle"]
    assert parameters["role_map_path"] == previous["roles"]
    assert parameters["pyvut_source"] == previous["source"]


def test_missing_manual_hardware_paths_are_not_replaced_by_installed_inputs(machine):
    install(machine[0], "a")
    result = discover(machine, config({"bundle_path": "/old/manual/bundle.json",
                                      "role_map_path": "/old/manual/roles.yaml"}), replace_missing=True)
    parameters = result.configuration["process_groups"][0]["parameters"]
    assert parameters["bundle_path"] == "/old/manual/bundle.json"
    assert parameters["role_map_path"] == "/old/manual/roles.yaml"


def test_initializer_uses_complete_installed_group_without_private_input_reads(machine, monkeypatch):
    deployment = install(machine[0], "a")
    def forbidden(*_args, **_kwargs):
        pytest.fail("must not execute or read deployment inputs")
    monkeypatch.setattr(Path, "read_text", forbidden)
    monkeypatch.setattr(Path, "read_bytes", forbidden)
    monkeypatch.setattr("subprocess.Popen", forbidden)
    result = discover_bootstrap_configuration({}, repo_root=machine[1], home=machine[0], environ={})
    assert result.configuration == {
        "python": deployment["python"], "script": str(Path(deployment["source"]) / "scripts/live_windows_bootstrap.py"),
        "capture": deployment["capture"], "bundle": deployment["bundle"], "expected_sha256": deployment["digest"],
    }
    assert "approval" not in result.configuration


@pytest.mark.parametrize("override", [
    {"capture": "/manual/capture.pcapng"},
    {"bundle": "/manual/bundle.json"},
    {"expected_sha256": "c" * 64},
])
def test_initializer_never_combines_manual_private_input_with_other_installation_defaults(machine, override):
    install(machine[0], "a")
    result = discover_bootstrap_configuration(deepcopy(override), repo_root=machine[1],
                                             home=machine[0], environ={})
    for key, value in override.items():
        assert result.configuration[key] == value
    for key in {"capture", "bundle", "expected_sha256"} - override.keys():
        assert not result.configuration.get(key)


def test_workstation_bundle_binding_prevents_defaults_from_current_different_installation(machine):
    previous = install(machine[0], "a")
    install(machine[0], "b")
    settings = TrackerBootstrapSettings(machine[1], machine[0] / "settings")
    settings.directory.mkdir(mode=0o700)
    path = settings.directory / "workstation.json"
    value = config({"bundle_path": previous["bundle"]})
    value["process_groups"][0]["label"] = "Trackers"
    path.write_text(json.dumps(value))
    path.chmod(0o600)
    actual = settings.defaults()
    assert actual["bundle"] == previous["bundle"]
    assert actual["capture"] == previous["capture"]
    assert actual["expected_sha256"] == previous["digest"]
    assert actual["python"] == previous["python"]
    assert actual["script"] == str(Path(previous["source"]) / "scripts/live_windows_bootstrap.py")
    assert not settings.path.exists()


def test_saved_initializer_settings_remain_authoritative_and_environment_still_overrides(machine, monkeypatch):
    previous = install(machine[0], "a")
    install(machine[0], "b")
    settings = TrackerBootstrapSettings(machine[1], machine[0] / "settings")
    saved = dict(python=previous["python"], script=str(Path(previous["source"]) / "scripts/live_windows_bootstrap.py"),
                 bundle=previous["bundle"], capture=previous["capture"], expected_sha256=previous["digest"])
    settings.save(saved)
    before = settings.path.read_bytes()
    assert settings.defaults() == saved
    monkeypatch.setenv("VT_CAPTURE", "/explicit/capture.pcapng")
    monkeypatch.setenv("VT_CAPTURE_SHA256", "c" * 64)
    assert settings.defaults()["capture"] == "/explicit/capture.pcapng"
    assert settings.defaults()["expected_sha256"] == "c" * 64
    assert settings.path.read_bytes() == before


def test_explicit_workstation_managed_bundle_takes_precedence_over_previous_saved_initializer(machine):
    previous = install(machine[0], "a")
    current = install(machine[0], "b")
    settings = TrackerBootstrapSettings(machine[1], machine[0] / "settings")
    settings.save(dict(python=previous["python"],
                       script=str(Path(previous["source"]) / "scripts/live_windows_bootstrap.py"),
                       bundle=previous["bundle"], capture=previous["capture"], expected_sha256=previous["digest"]))
    before = settings.path.read_bytes()
    value = config({"bundle_path": current["bundle"],
                    "pyvut_source": previous["source"],
                    "pyvut_site_packages": previous["site_packages"]})
    value["process_groups"][0]["label"] = "Trackers"
    path = settings.directory / "workstation.json"
    path.write_text(json.dumps(value))
    path.chmod(0o600)
    actual = settings.defaults()
    assert actual == dict(python=current["python"],
                          script=str(Path(current["source"]) / "scripts/live_windows_bootstrap.py"),
                          bundle=current["bundle"], capture=current["capture"], expected_sha256=current["digest"])
    assert settings.path.read_bytes() == before


def test_manual_workstation_bundle_uses_its_selected_software_before_another_active_deployment(machine):
    previous = install(machine[0], "a")
    install(machine[0], "b")
    venv = Path(previous["python"]).parent.parent
    (venv / "pyvenv.cfg").touch()
    bundle = machine[0] / "manual-bundle.json"
    bundle.touch()
    settings = TrackerBootstrapSettings(machine[1], machine[0] / "settings")
    settings.directory.mkdir(mode=0o700)
    value = config({"bundle_path": str(bundle), "pyvut_source": previous["source"],
                    "pyvut_site_packages": previous["site_packages"]})
    value["process_groups"][0]["label"] = "Trackers"
    path = settings.directory / "workstation.json"
    path.write_text(json.dumps(value))
    path.chmod(0o600)
    actual = settings.defaults()
    assert actual["bundle"] == str(bundle)
    assert actual["python"] == previous["python"]
    assert actual["script"] == str(Path(previous["source"]) / "scripts/live_windows_bootstrap.py")
    assert actual["capture"] == actual["expected_sha256"] == ""
    assert not settings.path.exists()


def test_complete_manual_configuration_does_not_depend_on_broken_activation_metadata(machine):
    deployment = install(machine[0], "a")
    base = machine[0] / ".local/share/visiontactile/tracker-deployment"
    (base / "current.json").write_text("broken activation")
    parameters = dict(pyvut_source=deployment["source"], pyvut_site_packages=deployment["site_packages"],
                      bundle_path=deployment["bundle"], role_map_path=deployment["roles"])
    result = discover(machine, config(parameters))
    assert result.configuration["process_groups"][0]["parameters"] == parameters
    initializer = dict(python=deployment["python"],
                       script=str(Path(deployment["source"]) / "scripts/live_windows_bootstrap.py"),
                       bundle=deployment["bundle"], capture=deployment["capture"],
                       expected_sha256=deployment["digest"])
    assert discover_bootstrap_configuration(
        initializer, repo_root=machine[1], home=machine[0], environ={}).configuration == initializer


def test_saved_managed_bundle_remains_usable_if_current_pointer_is_damaged(machine):
    previous = install(machine[0], "a")
    install(machine[0], "b")
    (machine[0] / ".local/share/visiontactile/tracker-deployment/current.json").write_text("broken")
    result = discover(machine, config({"bundle_path": previous["bundle"]}))
    assert result.configuration["process_groups"][0]["parameters"]["role_map_path"] == previous["roles"]


def test_invalid_installation_never_produces_ready_defaults(machine):
    deployment = install(machine[0], "a")
    Path(deployment["bundle"]).chmod(0o644)
    result = discover(machine, config())
    parameters = result.configuration["process_groups"][0]["parameters"]
    assert not parameters.get("bundle_path")
    assert not parameters.get("role_map_path")
    assert any(item.key == "installation_manifest" and item.status == "missing"
               for item in result.findings)
    with pytest.raises(ValueError):
        discover_bootstrap_configuration({}, repo_root=machine[1], home=machine[0], environ={})


def test_damaged_installation_allows_camera_and_generic_software_discovery(machine):
    deployment = install(machine[0], "a")
    Path(deployment["bundle"]).chmod(0o644)
    home, root = machine
    source = root / "external/pyvut"
    for name in ("live_hid.py", "live_probe.py", "live_bootstrap_bundle.py", "pose_decoder.py"):
        path = source / "pyvut" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    roles = home / "visiontactile-data/tracker-validation/roles.yaml"
    roles.parent.mkdir(parents=True)
    roles.touch()
    value = config()
    value["process_groups"].insert(0, dict(id="camera", preset="cameras", parameters={}))
    result = discover(machine, value)
    camera, tracker = (group["parameters"] for group in result.configuration["process_groups"])
    assert camera["output_root"] == str(home / "visiontactile-data")
    assert tracker["pyvut_source"] == str(source)
    assert not tracker.get("bundle_path")
    assert not tracker.get("role_map_path")
    assert not value["process_groups"][1]["enabled"]


def test_discovery_needs_no_optional_deployment_package(machine, monkeypatch):
    original_import = builtins.__import__
    def without_package(name, *args, **kwargs):
        if name == "vt_tracker_deploy" or name.startswith("vt_tracker_deploy."):
            raise ModuleNotFoundError("No module named 'vt_tracker_deploy'", name="vt_tracker_deploy")
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", without_package)
    # Import the discovery module afresh: a cached import would hide an
    # accidental top-level dependency on the optional deployment package.
    name = "vt_capture_console.backend.environment_discovery"
    monkeypatch.delitem(sys.modules, name)
    module = importlib.import_module(name)
    result = module.discover_configuration(config(), repo_root=machine[1], home=machine[0], environ={})
    assert not result.configuration["process_groups"][0]["parameters"].get("bundle_path")
    assert discover_bootstrap_configuration({}, repo_root=machine[1], home=machine[0], environ={})


def test_no_deployment_does_not_require_posix_nofollow(machine, monkeypatch):
    for name in ("O_NOFOLLOW", "O_DIRECTORY", "getuid"):
        monkeypatch.delattr(os, name, raising=False)
    result = discover(machine, config())
    assert not result.configuration["process_groups"][0]["parameters"].get("bundle_path")


def test_source_without_live_probe_is_not_ready(machine):
    deployment = install(machine[0], "a")
    (Path(deployment["source"]) / "pyvut/live_probe.py").unlink()
    result = discover(machine, config())
    assert not result.configuration["process_groups"][0]["parameters"].get("pyvut_source")
