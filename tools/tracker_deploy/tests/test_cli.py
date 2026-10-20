import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_tracker_deploy import cli


def _write(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document))
    path.chmod(0o600)


def _settings(home, *, initializer_bundle="/bundle.json"):
    directory = home / ".config/visiontactile/workbench"
    _write(directory / "workstation.json", {"process_groups": [
        {"preset": "trackers", "parameters": {
            "pyvut_source": "/source", "bundle_path": "/bundle.json", "role_map_path": "/roles.yaml",
        }},
    ]})
    _write(directory / "tracker-bootstrap.json", {"schema_version": 1, "values": {
        "bundle": initializer_bundle, "capture": "/capture.pcapng", "expected_sha256": "a" * 64,
    }})


def test_workstation_inputs_keep_publisher_and_initializer_together(tmp_path):
    _settings(tmp_path)
    values = cli.workstation_inputs(tmp_path)
    assert values["bundle"] == Path("/bundle.json")
    assert values["capture"] == Path("/capture.pcapng")
    assert values["capture_sha256"] == "a" * 64
    _settings(tmp_path, initializer_bundle="/other-bundle.json")
    with pytest.raises(ValueError, match="same bundle"):
        cli.workstation_inputs(tmp_path)


def test_workstation_inputs_reject_non_private_settings(tmp_path):
    _settings(tmp_path)
    (tmp_path / ".config/visiontactile/workbench/workstation.json").chmod(0o644)
    with pytest.raises(ValueError, match="unsafe"):
        cli.workstation_inputs(tmp_path)


def test_managed_bundle_uses_own_capture_without_reading_legacy_settings(tmp_path, monkeypatch):
    _settings(tmp_path, initializer_bundle="/obsolete.json")
    monkeypatch.setattr(cli, "for_bundle", lambda *a, **k: SimpleNamespace(
        capture=Path("/managed-capture.pcapng"), capture_sha256="b" * 64))
    values = cli.workstation_inputs(tmp_path)
    assert values["capture"] == Path("/managed-capture.pcapng")
    assert values["capture_sha256"] == "b" * 64


def test_cli_never_prints_private_exception_details(monkeypatch, capsys):
    def fail(*args):
        raise ValueError("PRIVATE_DEVICE_CAPTURE_CONTENT")
    monkeypatch.setattr(cli, "read_package", fail)
    assert cli.main(["inspect", "--package", "/bad.vttracker"]) == 1
    output = capsys.readouterr()
    assert "PRIVATE_DEVICE_CAPTURE_CONTENT" not in output.out + output.err
    assert "invalid_package_or_settings" in output.err


def test_build_disallows_mixing_settings_with_explicit_inputs():
    with pytest.raises(SystemExit) as error:
        cli.main(["build", "--from-workstation", "--source", "/source",
                  "--wheelhouse", "/wheels", "--output", "/package.vttracker"])
    assert error.value.code == 2


def test_unsupported_platform_reports_error_without_posix_io(monkeypatch, capsys):
    monkeypatch.setattr(cli.sys, "platform", "win32")
    monkeypatch.setattr(cli, "read_package", lambda *_args: pytest.fail("must not inspect on Windows"))
    assert cli.main(["inspect", "--package", "/package.vttracker"]) == 1
    assert "unsupported_platform" in capsys.readouterr().err


def test_cli_help_does_not_load_linux_installer(monkeypatch):
    import builtins
    original_import = builtins.__import__
    def guarded_import(name, *args, **kwargs):
        if name in {"fcntl", "installer", "vt_tracker_deploy.installer"}:
            pytest.fail("help must not import Linux installer")
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", guarded_import)
    with pytest.raises(SystemExit) as error:
        cli.main(["--help"])
    assert error.value.code == 0
