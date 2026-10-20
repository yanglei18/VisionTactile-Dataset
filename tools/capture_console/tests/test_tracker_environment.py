"""Tracker publisher and initialization paths share read-only local discovery."""
from copy import deepcopy
from pathlib import Path

import pytest

from vt_capture_console.backend import environment_discovery
from vt_capture_console.backend.tracker_settings import TrackerBootstrapSettings
from vt_capture_console.backend.tracker_workflow import detect_defaults


MARKERS = (
    "pyvut/live_hid.py", "pyvut/live_bootstrap_bundle.py", "pyvut/pose_decoder.py",
)


def file(path, *, executable=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("fixture: never execute or read dependency content")
    if executable:
        path.chmod(0o700)
    return path


def source(root):
    for name in (*MARKERS, "scripts/live_windows_bootstrap.py"):
        file(root / name)
    return root


@pytest.fixture
def machine(tmp_path, monkeypatch):
    home, root = tmp_path / "home", tmp_path / "repo"
    home.mkdir()
    root.mkdir()
    monkeypatch.setattr(Path, "home", lambda: home)
    for name in ("VT_REPO", "VT_PYVUT_ROOT", "VT_PYVUT_VENV", "VT_CAPTURE",
                 "VT_BUNDLE", "VT_CAPTURE_SHA256", "VT_DATA_ROOT"):
        monkeypatch.delenv(name, raising=False)
    return home, root, TrackerBootstrapSettings(root, tmp_path / "settings")


def installed(home):
    root = source(home / ".local/share/visiontactile/pyvut-src")
    python = file(home / ".local/share/visiontactile/pyvut-venv/bin/python", executable=True)
    return python, root / "scripts/live_windows_bootstrap.py"


def saved(settings, tmp_path):
    values = {
        "python": str(file(tmp_path / "previous/bin/python", executable=True)),
        "script": str(file(tmp_path / "previous/scripts/live_windows_bootstrap.py")),
        "capture": str(file(tmp_path / "private/approved.pcapng")),
        "bundle": str(file(tmp_path / "private/approved.json")),
        "expected_sha256": "a" * 64,
    }
    settings.save(values)
    return values


def test_initialization_defaults_find_standard_home_software(machine):
    home, root, settings = machine
    python, script = installed(home)
    defaults = detect_defaults(root, include_environment=False)
    assert defaults["python"] == str(python)
    assert defaults["script"] == str(script)
    assert defaults["expected_sha256"] == ""
    assert not settings.directory.exists()


def test_publisher_and_initialization_find_the_same_external_source(machine):
    _, root, _ = machine
    checkout = source(root / "external/pyvut")
    publisher = environment_discovery.discover_configuration(
        {"process_groups": [{"id": "my_trackers", "preset": "trackers", "parameters": {}}]},
        repo_root=root, environ={},
    ).configuration["process_groups"][0]["parameters"]
    assert publisher["pyvut_source"] == str(checkout)
    assert detect_defaults(root, include_environment=False)["script"] == str(
        checkout / "scripts/live_windows_bootstrap.py")


def test_publisher_and_bootstrap_share_explicit_source_and_virtualenv(machine, tmp_path):
    home, root, _ = machine
    installed(home)
    checkout = source(tmp_path / "chosen/source")
    venv = tmp_path / "chosen/venv"
    python = file(venv / "bin/python", executable=True)
    site = venv / "lib/python3.14/site-packages"
    site.mkdir(parents=True)
    environment = {"VT_PYVUT_ROOT": str(checkout), "VT_PYVUT_VENV": str(venv)}
    publisher = environment_discovery.discover_configuration(
        {"process_groups": [{"id": "trackers", "preset": "trackers", "parameters": {}}]},
        repo_root=root, environ=environment,
    ).configuration["process_groups"][0]["parameters"]
    bootstrap = environment_discovery.discover_bootstrap_configuration(
        {}, repo_root=root, environ=environment).configuration
    assert publisher["pyvut_source"] == str(checkout)
    assert publisher["pyvut_site_packages"] == str(site)
    assert bootstrap == {"python": str(python),
                         "script": str(checkout / "scripts/live_windows_bootstrap.py")}
    invalid = environment_discovery.discover_bootstrap_configuration(
        {}, repo_root=root, environ={"VT_PYVUT_ROOT": "/missing", "VT_PYVUT_VENV": "/missing"})
    assert not invalid.configuration
    assert all(item.status == "missing" for item in invalid.findings)


def test_old_repository_worktree_is_not_a_software_default(machine):
    _, root, _ = machine
    source(root / ".local/share/visiontactile/pyvut-src/.worktrees/capture-locked-live-bootstrap")
    file(root / ".local/share/visiontactile/pyvut-venv/bin/python", executable=True)
    defaults = detect_defaults(root, include_environment=False)
    assert defaults["script"] == defaults["python"] == ""


def test_migrated_saved_software_is_repaired_without_changing_private_inputs_or_store(machine, tmp_path):
    home, _, settings = machine
    values = saved(settings, tmp_path)
    before = settings.path.read_bytes()
    for key in ("python", "script", "capture", "bundle"):
        Path(values[key]).unlink()
    python, script = installed(home)
    result = settings.defaults()
    assert result["python"] == str(python)
    assert result["script"] == str(script)
    assert {key: result[key] for key in ("capture", "bundle", "expected_sha256")} == {
        key: values[key] for key in ("capture", "bundle", "expected_sha256")
    }
    assert settings.path.read_bytes() == before
    assert settings.load() == values


def test_existing_saved_software_wins_over_standard_candidates(machine, tmp_path):
    home, _, settings = machine
    values = saved(settings, tmp_path)
    installed(home)
    assert settings.defaults() == values


def test_missing_saved_script_with_multiple_sources_stays_visible(machine, tmp_path):
    home, root, settings = machine
    values = saved(settings, tmp_path)
    Path(values["script"]).unlink()
    installed(home)
    source(root / "external/pyvut")
    assert settings.defaults()["script"] == values["script"]
    assert detect_defaults(root, include_environment=False)["script"] == ""


def test_explicit_environment_overrides_remain_visible_even_when_invalid(machine, tmp_path, monkeypatch):
    home, _, settings = machine
    values = saved(settings, tmp_path)
    installed(home)
    monkeypatch.setenv("VT_PYVUT_ROOT", "/missing/explicit/source")
    monkeypatch.setenv("VT_PYVUT_VENV", "/missing/explicit/venv")
    monkeypatch.setenv("VT_CAPTURE", "/missing/explicit/capture.pcapng")
    monkeypatch.setenv("VT_BUNDLE", "/missing/explicit/bundle.json")
    monkeypatch.setenv("VT_CAPTURE_SHA256", "b" * 64)
    assert settings.defaults() == {
        "python": "/missing/explicit/venv/bin/python",
        "script": "/missing/explicit/source/scripts/live_windows_bootstrap.py",
        "capture": "/missing/explicit/capture.pcapng",
        "bundle": "/missing/explicit/bundle.json",
        "expected_sha256": "b" * 64,
    }
    assert settings.load() == values


def test_discovered_scripts_require_extended_source_and_python_must_be_executable(machine):
    home, root, _ = machine
    python, script = installed(home)
    python.chmod(0o600)
    (script.parents[1] / MARKERS[0]).unlink()
    defaults = detect_defaults(root, include_environment=False)
    assert defaults["python"] == defaults["script"] == ""


def test_bootstrap_discovery_preserves_all_inputs_without_reading_or_executing(machine, monkeypatch):
    home, root, _ = machine
    python, script = installed(home)
    values = {"python": "", "script": "", "capture": "/private/missing.pcapng",
              "bundle": "/private/missing.json", "expected_sha256": "A" * 64,
              "future": {"preserve": [1, 2]}}
    before = deepcopy(values)

    def forbidden(*args, **kwargs):
        pytest.fail("discovery must use metadata only")

    monkeypatch.setattr(Path, "read_text", forbidden)
    monkeypatch.setattr(Path, "read_bytes", forbidden)
    monkeypatch.setattr("builtins.open", forbidden)
    monkeypatch.setattr("os.open", forbidden)
    monkeypatch.setattr("subprocess.Popen", forbidden)
    result = environment_discovery.discover_bootstrap_configuration(
        values, repo_root=root, environ={}, replace_missing=True)
    assert result.configuration == dict(before, python=str(python), script=str(script))
    assert values == before
    assert result.updated_fields == (("", "python"), ("", "script"))
    assert {item.key for item in result.findings} == {"python", "script"}


def test_default_discovery_keeps_dated_private_resource_choices_and_empty_digest(machine):
    home, root, _ = machine
    private = home / "visiontactile-data/private/vut"
    bundle = file(private / "live-bootstrap-2026-07-31.json")
    capture = file(private / "20260731_session01/01_cold_reconnect.pcapng")
    defaults = detect_defaults(root, include_environment=False)
    assert defaults["capture"] == str(capture)
    assert defaults["bundle"] == str(bundle)
    assert defaults["expected_sha256"] == ""
