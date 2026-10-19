"""All mutation tests use temporary files and fake udev/device operations."""

import importlib.util
import json
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "configure_realsense_usb", Path(__file__).resolve().parents[1] / "configure_realsense_usb.py"
)
usb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usb)


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    for name in ("RULE", "STATE", "LOCK", "MEMORY", "BOOT"):
        monkeypatch.setattr(usb, name, tmp_path / name)
    usb.MEMORY.write_text("16\n")
    usb.BOOT.write_text("fixture-boot\n")
    monkeypatch.setattr(usb, "trusted", lambda *args, **kwargs: None)
    monkeypatch.setattr(usb, "devices", lambda: [])
    monkeypatch.setattr(usb, "refresh", lambda current: None)
    monkeypatch.setattr(usb.grp, "getgrnam", lambda name: object())
    return tmp_path


def test_apply_and_rollback_restore_original_memory_and_rule(sandbox):
    assert usb.apply()["applied"]
    state = json.loads(usb.STATE.read_text())
    assert state["memory_mb"] == 16 and state["rule"] is None
    assert usb.RULE.read_text() == usb.RULE_TEXT
    assert int(usb.MEMORY.read_text()) == 64
    assert usb.STATE.stat().st_mode & 0o777 == 0o600
    # Repeated apply must preserve the first backup, not back up the new value.
    usb.apply()
    assert json.loads(usb.STATE.read_text())["memory_mb"] == 16
    assert usb.restore(usb.load_state())["restored"]
    assert int(usb.MEMORY.read_text()) == 16
    assert not usb.RULE.exists() and not usb.STATE.exists()


def test_foreign_rule_is_never_overwritten(sandbox):
    usb.RULE.write_text("# Administrator's other rule\n")
    with pytest.raises(RuntimeError, match="overwrite"):
        usb.apply()
    assert not usb.STATE.exists()
    assert int(usb.MEMORY.read_text()) == 16


def test_apply_failure_automatically_restores_original_settings(sandbox, monkeypatch):
    calls = []

    def fail_once(current):
        calls.append(current)
        if len(calls) == 1:
            raise RuntimeError("fixture udev failure")

    monkeypatch.setattr(usb, "refresh", fail_once)
    with pytest.raises(RuntimeError, match="original settings restored"):
        usb.apply()
    assert int(usb.MEMORY.read_text()) == 16
    assert not usb.RULE.exists() and not usb.STATE.exists()


def test_failed_rollback_preserves_backup(sandbox, monkeypatch):
    def unavailable(current):
        raise RuntimeError("fixture udev unavailable")

    monkeypatch.setattr(usb, "refresh", unavailable)
    with pytest.raises(RuntimeError, match="backup retained"):
        usb.apply()
    assert usb.STATE.exists()


@pytest.mark.parametrize("target", ["RULE", "MEMORY"])
def test_rollback_does_not_overwrite_external_changes(sandbox, target):
    usb.apply()
    path = getattr(usb, target)
    path.write_text("# external edit\n" if target == "RULE" else "128\n")
    with pytest.raises(RuntimeError, match="outside this helper"):
        usb.restore(usb.load_state())
    assert usb.STATE.exists()


def test_rollback_does_not_restore_previous_boot_memory(sandbox):
    usb.apply()
    usb.BOOT.write_text("another-boot\n")
    usb.MEMORY.write_text("32\n")
    assert not usb.restore(usb.load_state())["memory_restored"]
    assert int(usb.MEMORY.read_text()) == 32


def test_unprivileged_apply_fails_before_any_mutation(sandbox, monkeypatch, capsys):
    monkeypatch.setattr(usb.os, "geteuid", lambda: 1000)
    assert usb.main(["apply"]) == 1
    assert "Administrator authentication" in json.loads(capsys.readouterr().out)["error"]
    assert not usb.LOCK.exists() and not usb.STATE.exists() and not usb.RULE.exists()


@pytest.mark.parametrize("original", [0, 128])
def test_existing_larger_or_unlimited_budget_is_not_reduced(sandbox, original):
    usb.MEMORY.write_text(f"{original}\n")
    usb.apply()
    assert int(usb.MEMORY.read_text()) == original
    usb.restore(usb.load_state())
    assert int(usb.MEMORY.read_text()) == original


def test_unknown_new_device_prevents_false_complete_rollback(sandbox, monkeypatch):
    usb.apply()
    monkeypatch.setattr(usb, "devices", lambda: [{
        "product": "0b5b", "serial": "new-camera", "node": "/dev/fixture-not-opened",
    }])
    with pytest.raises(RuntimeError, match="unknown original permissions"):
        usb.restore(usb.load_state())
    assert usb.STATE.exists()
    assert not usb.RULE.exists()
    assert int(usb.MEMORY.read_text()) == 16


def test_trusted_path_rejects_symlinks_and_writable_parent(tmp_path, monkeypatch):
    target = tmp_path / "rule"
    actual_lstat = Path.lstat

    def root_owned(path):
        info = actual_lstat(path)
        values = list(info)
        values[4] = 0  # st_uid; isolate symlink protection from fixture ownership.
        if path != target:
            values[0] &= ~0o022
        return usb.os.stat_result(values)

    target.symlink_to(tmp_path / "unrelated")
    monkeypatch.setattr(Path, "lstat", root_owned)
    with pytest.raises(RuntimeError, match="Unsafe system path"):
        usb.trusted(target, missing=True)
    target.unlink()
    target.write_text("fixture")
    target.chmod(0o666)
    with pytest.raises(RuntimeError, match="Unsafe system path"):
        usb.trusted(target)
