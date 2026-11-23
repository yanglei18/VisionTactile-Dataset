"""Numerical preflight is isolated, bounded and bound to the selected checkout."""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from vt_capture_core import hand_runtime_probe as module


def test_probe_uses_isolated_interpreter_and_no_device_command(tmp_path, monkeypatch):
    root = tmp_path / "checkout"
    evidence = dict(schema_version=1, hand="left", source_root=str(root),
                    hardware_accessed=False, controller_started=False)
    def run(command, **kwargs):
        assert command[0] == sys.executable
        assert command[1:3] == ["-I", "-c"]
        assert command[-3:] == [str(root), str(tmp_path / "bundle"), "left"]
        assert "PYTHONPATH" not in kwargs["env"]
        assert kwargs["timeout"] == 45.
        assert kwargs["stdin"] == subprocess.DEVNULL
        kwargs["stdout"].write(json.dumps(evidence).encode())
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(module.subprocess, "run", run)
    assert module.probe_hand_runtime(sys.executable, tmp_path / "bundle", "left", root) == evidence


@pytest.mark.parametrize("mode", ["timeout", "failure", "wrong_hand", "wrong_root", "hardware", "oversized", "invalid"])
def test_probe_rejects_unconfirmed_or_foreign_runtime(tmp_path, monkeypatch, mode):
    evidence = dict(schema_version=1, hand="left", source_root=str(tmp_path),
                    hardware_accessed=False, controller_started=False)
    if mode == "wrong_hand": evidence["hand"] = "right"
    if mode == "wrong_root": evidence["source_root"] = "/another-checkout"
    if mode == "hardware": evidence["hardware_accessed"] = True
    def run(command, **kwargs):
        if mode == "timeout":
            raise subprocess.TimeoutExpired(command, 45)
        if mode == "failure":
            kwargs["stderr"].write(b"wrong source installed")
            return SimpleNamespace(returncode=1)
        payload = b"x" * (1024 * 1024 + 1) if mode == "oversized" else b"no json" if mode == "invalid" else json.dumps(evidence).encode()
        kwargs["stdout"].write(payload)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(module.subprocess, "run", run)
    with pytest.raises(ValueError):
        module.probe_hand_runtime(sys.executable, tmp_path / "bundle", "left", tmp_path)
