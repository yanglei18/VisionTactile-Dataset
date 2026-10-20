"""Verifier trust boundaries without importing private source or hardware."""
import json
from pathlib import Path

import pytest

from vt_tracker_deploy import verify_runtime


def test_verifier_requires_the_repository_owned_identity_and_role_policy(tmp_path, monkeypatch):
    location = tmp_path / "tools/tracker_deploy/src/vt_tracker_deploy/verify_runtime.py"
    location.parent.mkdir(parents=True)
    monkeypatch.setattr(verify_runtime, "__file__", str(location))
    monkeypatch.setattr(verify_runtime.sys, "argv", ["verify", str(tmp_path / "version"), "a" * 64])
    # No fallback to an unrelated installed vt_vive_tracker package or a package
    # supplied by the incoming runtime is permitted.
    with pytest.raises(ImportError, match="trusted checkout"):
        verify_runtime.verify(tmp_path / "version", "a" * 64)


@pytest.mark.parametrize("failure,code", [
    (ImportError("private module information"), "runtime_dependency"),
    (ValueError("private bundle contents"), "runtime_validation"),
])
def test_verifier_reports_only_public_error_codes(monkeypatch, capsys, failure, code):
    def fail(*_args):
        raise failure
    monkeypatch.setattr(verify_runtime, "verify", fail)
    monkeypatch.setattr(verify_runtime.sys, "argv", ["verify", "/private/path", "a" * 64])
    assert verify_runtime.main() == 1
    result = capsys.readouterr()
    assert json.loads(result.out) == {"ok": False, "code": code}
    assert "private" not in result.out + result.err
