"""Private metadata-only preferences: no devices, child processes or credentials."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import stat
import subprocess
import sys

import pytest

from vt_capture_console.backend import tracker_settings as module
from vt_capture_console.backend.tracker_settings import TrackerBootstrapSettings
from vt_capture_console.backend.tracker_workflow import BootstrapConfig, detect_defaults
from vt_capture_console.backend.workstation_settings import default_configuration


@pytest.fixture(autouse=True)
def no_bootstrap_environment(monkeypatch):
    for name in ("VT_REPO", "VT_PYVUT_ROOT", "VT_PYVUT_VENV", "VT_CAPTURE", "VT_BUNDLE", "VT_CAPTURE_SHA256"):
        monkeypatch.delenv(name, raising=False)


@pytest.fixture
def settings(tmp_path, monkeypatch):
    home, root = tmp_path / "home", tmp_path / "repo"
    home.mkdir()
    root.mkdir()
    monkeypatch.setattr(Path, "home", lambda: home)
    return TrackerBootstrapSettings(root, tmp_path / "settings")


@pytest.fixture
def values(tmp_path):
    inputs = tmp_path / "inputs"
    inputs.mkdir()
    script = inputs / "live_windows_bootstrap.py"
    script.write_text("raise AssertionError('must never execute')\n")
    capture, bundle = inputs / "capture.pcapng", inputs / "bundle.json"
    capture.write_bytes(b"synthetic capture; must not read to compute a hash")
    bundle.write_text("credential fixture must never be persisted or opened")
    return asdict(BootstrapConfig(sys.executable, str(script), str(capture), str(bundle), "A" * 64))


def private_file(path, text):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    path.chmod(0o600)


def workstation(settings, bundle):
    value = default_configuration(settings.repo_root)
    tracker = next(group for group in value["process_groups"] if group["preset"] == "trackers")
    tracker["parameters"]["bundle_path"] = str(bundle)
    private_file(settings.directory / "workstation.json", json.dumps(value))


def test_missing_store_load_is_empty_and_readonly(settings, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("settings must not execute a process")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    assert settings.load() == {}
    assert not settings.directory.exists()
    assert set(settings.defaults()) == module.FIELDS
    assert not settings.directory.exists()


def test_save_is_private_atomic_metadata_only_and_reuses_argv(settings, values, monkeypatch):
    original = BootstrapConfig.argv
    calls = []

    def checked(config):
        calls.append(config)
        return original(config)

    def forbidden(*args, **kwargs):
        raise AssertionError("settings must not execute a process")

    monkeypatch.setattr(BootstrapConfig, "argv", checked)
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    assert settings.save(values) is None
    assert len(calls) == 1
    assert stat.S_IMODE(settings.directory.stat().st_mode) == 0o700
    assert stat.S_IMODE(settings.path.stat().st_mode) == 0o600
    document = json.loads(settings.path.read_text())
    assert set(document) == {"schema_version", "values"}
    assert document["schema_version"] == 1 and set(document["values"]) == module.FIELDS
    assert "credential fixture" not in settings.path.read_text()
    assert "approval" not in settings.path.read_text()
    assert settings.load() == dict(values, expected_sha256="a" * 64)
    loaded = settings.load()
    loaded["capture"] = "/changed"
    assert settings.load()["capture"] == values["capture"]
    assert not list(settings.directory.glob(".*.tmp"))


def test_saved_stale_paths_stay_visible_but_cannot_be_resaved(settings, values):
    settings.save(values)
    Path(values["capture"]).unlink()
    assert settings.load()["capture"] == values["capture"]
    assert settings.defaults()["capture"] == values["capture"]
    before = settings.path.read_bytes()
    with pytest.raises(ValueError, match="现有"):
        settings.save(values)
    assert settings.path.read_bytes() == before


@pytest.mark.parametrize("change", [lambda value: value.update(approval=True),
    lambda value: value.update(credentials={"secret": "must not persist"}),
    lambda value: value.pop("capture"), lambda value: value.update(expected_sha256=True),
    lambda value: value.update(expected_sha256="not-a-digest"), lambda value: value.update(capture="relative"),
    lambda value: value.update(bundle="/secret\nother"), lambda value: value.update(script="/unapproved.py")])
def test_only_five_structurally_valid_fields_can_be_saved(settings, values, change):
    change(values)
    with pytest.raises(ValueError):
        settings.save(values)
    assert not settings.path.exists()


@pytest.mark.parametrize("text", ["null", "{}", "not-json", "[]",
    '{"schema_version":true,"values":{}}', '{"schema_version":1,"schema_version":1,"values":{}}',
    '{"schema_version":1,"values":{},"approval":true}'])
def test_bad_existing_store_is_not_silently_overwritten(settings, values, text):
    private_file(settings.path, text)
    with pytest.raises(ValueError):
        settings.load()
    with pytest.raises(ValueError):
        settings.defaults()
    with pytest.raises(ValueError):
        settings.save(values)
    assert settings.path.read_text() == text


def test_public_store_and_directory_are_rejected_without_chmod(settings, values):
    settings.save(values)
    settings.path.chmod(0o644)
    with pytest.raises(ValueError, match="0600"):
        settings.load()
    with pytest.raises(ValueError, match="0600"):
        settings.save(values)
    assert stat.S_IMODE(settings.path.stat().st_mode) == 0o644
    settings.path.chmod(0o600)
    settings.directory.chmod(0o755)
    with pytest.raises(ValueError, match="0700"):
        settings.load()
    assert stat.S_IMODE(settings.directory.stat().st_mode) == 0o755


def test_foreign_owned_store_is_rejected(settings, values, monkeypatch):
    settings.save(values)
    monkeypatch.setattr(module.os, "getuid", lambda: settings.directory.stat().st_uid + 1)
    with pytest.raises(ValueError, match="当前用户"):
        settings.load()


def test_store_file_symlink_is_rejected_and_target_untouched(settings, values, tmp_path):
    target = tmp_path / "target.json"
    private_file(target, "must not overwrite")
    settings.directory.mkdir(mode=0o700)
    settings.path.symlink_to(target)
    for method in (settings.load, lambda: settings.save(values)):
        with pytest.raises(ValueError, match="链接"):
            method()
    assert target.read_text() == "must not overwrite" and settings.path.is_symlink()


@pytest.mark.parametrize("ancestor", [False, True])
def test_store_directory_or_ancestor_symlink_is_rejected(settings, tmp_path, ancestor):
    target = tmp_path / "real-directory"
    target.mkdir(mode=0o700)
    alias = tmp_path / "alias-directory"
    alias.symlink_to(target, target_is_directory=True)
    if ancestor:
        (target / "child").mkdir(mode=0o700)
        alias /= "child"
    linked = TrackerBootstrapSettings(settings.repo_root, alias)
    with pytest.raises(ValueError, match="链接"):
        linked.load()


def test_fifo_and_oversize_store_rejected_without_blocking(settings):
    settings.directory.mkdir(mode=0o700)
    os.mkfifo(settings.path, 0o600)
    with pytest.raises(ValueError, match="常规文件"):
        settings.load()
    settings.path.unlink()
    private_file(settings.path, "x" * (module.MAX_STORE_BYTES + 1))
    with pytest.raises(ValueError, match="64 KiB"):
        settings.load()


def test_failed_atomic_replace_preserves_old_store_and_removes_temp(settings, values, monkeypatch):
    settings.save(values)
    previous = settings.path.read_bytes()
    values["expected_sha256"] = "b" * 64

    def fail(*args, **kwargs):
        raise OSError("synthetic replace failure")

    monkeypatch.setattr(module.os, "replace", fail)
    with pytest.raises(OSError, match="synthetic"):
        settings.save(values)
    assert settings.path.read_bytes() == previous
    assert not list(settings.directory.glob(".*.tmp"))


def test_defaults_do_not_guess_dated_private_inputs(settings):
    private = Path.home() / "visiontactile-data/private/vut"
    capture = private / "20260731_session01/01_cold_reconnect.pcapng"
    capture.parent.mkdir(parents=True)
    capture.touch()
    old_bundle = settings.repo_root / "visiontactile-data/private/vut/live-bootstrap-2026-07-31.json"
    old_bundle.parent.mkdir(parents=True)
    old_bundle.touch()
    values = detect_defaults(settings.repo_root)
    assert values["capture"] == ""
    assert values["bundle"] == values["expected_sha256"] == ""
    external_bundle = private / "live-bootstrap-2026-07-31.json"
    external_bundle.touch()
    assert detect_defaults(settings.repo_root)["bundle"] == ""
    external_bundle.unlink()
    external_bundle.symlink_to(old_bundle)
    assert detect_defaults(settings.repo_root)["bundle"] == ""


def test_current_workstation_bundle_saved_settings_and_environment_priority(settings, values, monkeypatch, tmp_path):
    current_bundle = tmp_path / "configured-bundle.json"
    # Deliberately absent: an explicitly configured path remains editable.
    workstation(settings, current_bundle)
    assert settings.defaults()["bundle"] == str(current_bundle)
    settings.save(values)
    changed = settings.defaults()
    assert changed["bundle"] == str(current_bundle)
    assert changed["capture"] == changed["expected_sha256"] == ""
    assert changed["python"] == values["python"]
    assert changed["script"] == values["script"]
    monkeypatch.setenv("VT_CAPTURE", "/explicit/missing/capture.pcapng")
    monkeypatch.setenv("VT_BUNDLE", "/explicit/missing/bundle.json")
    monkeypatch.setenv("VT_CAPTURE_SHA256", "c" * 64)
    monkeypatch.setenv("VT_PYVUT_ROOT", "/explicit/pyvut")
    monkeypatch.setenv("VT_PYVUT_VENV", "/explicit/venv")
    assert settings.defaults() == dict(python="/explicit/venv/bin/python",
        script="/explicit/pyvut/scripts/live_windows_bootstrap.py", capture="/explicit/missing/capture.pcapng",
        bundle="/explicit/missing/bundle.json", expected_sha256="c" * 64)
    assert settings.load()["bundle"] == values["bundle"]  # Environment never writes the store.


def test_repository_bundle_in_workstation_is_not_an_automatic_suggestion(settings):
    path = settings.repo_root / "private-bundle.json"
    workstation(settings, path)
    assert settings.defaults()["bundle"] == ""


def test_defaults_do_not_hide_bad_workstation_settings(settings):
    private_file(settings.directory / "workstation.json", "corrupted")
    with pytest.raises(ValueError, match="损坏"):
        settings.defaults()
