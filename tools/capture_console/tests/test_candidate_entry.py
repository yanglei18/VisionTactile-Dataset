"""Candidate startup checks the prepared deployment without re-importing settings."""
import json
from pathlib import Path

import pytest

from vt_capture_console import candidate


@pytest.fixture
def entry(tmp_path, monkeypatch):
    root, settings = tmp_path / "checkout", tmp_path / "candidate"
    root.mkdir()
    settings.mkdir()
    config = {"process_groups": []}
    checks, launches = [], []
    monkeypatch.setattr(candidate, "repository_root", lambda: root)
    monkeypatch.setattr(candidate, "default_candidate_settings", lambda actual: settings)
    monkeypatch.setattr(candidate, "load_candidate_binding", lambda path, **kw: (path, {"fixture": "identity"}))
    monkeypatch.setattr(candidate, "read_private_json", lambda path: config)
    monkeypatch.setattr(candidate, "validate_configuration", lambda value: value)
    monkeypatch.setattr(candidate, "verify_candidate_runtimes", lambda *args: checks.append(args))
    monkeypatch.setattr(candidate.os, "execv", lambda program, args: launches.append((program, args)))
    return root, settings, config, checks, launches


def test_candidate_check_only_preflights_without_launch(entry, capsys):
    root, settings, config, checks, launches = entry
    assert candidate.main(["--check-only"]) == 0
    assert checks == [(config, root)]
    assert not launches
    evidence = json.loads(capsys.readouterr().out)
    assert evidence["settings_dir"] == str(settings)
    assert evidence["hardware_accessed"] is False


def test_entry_uses_owned_checkout_and_keeps_paths_as_arguments(entry):
    root, settings, config, checks, launches = entry
    assert candidate.main(["--repo-root", str(root), "--language", "en", "--smoke-seconds", "2",
                           "--screenshot", str(settings / "screen shot.png")]) == 0
    program, argv = launches[0]
    assert program == "/bin/bash"
    assert argv[:4] == [program, str(root / "tools/capture_deploy/start_workbench.sh"),
                        "--candidate-settings", str(settings)]
    assert str(settings / "screen shot.png") in argv
    assert checks == [(config, root)]


def test_entry_rejects_interpreter_installed_from_another_checkout(entry, tmp_path):
    other = tmp_path / "other"
    other.mkdir()
    with pytest.raises(SystemExit) as error:
        candidate.main(["--repo-root", str(other)])
    assert error.value.code == 2
    assert not entry[3] and not entry[4]


@pytest.mark.parametrize("function", ["load_candidate_binding", "verify_candidate_runtimes"])
def test_failed_preflight_does_not_launch_or_fall_back_to_global(entry, monkeypatch, function):
    def fail(*args, **kwargs):
        raise ValueError("invalid candidate")
    monkeypatch.setattr(candidate, function, fail)
    with pytest.raises(SystemExit) as error:
        candidate.main([])
    assert error.value.code == 2
    assert not entry[4]
