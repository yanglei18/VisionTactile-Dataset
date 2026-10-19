"""Verify that the offline package keeps notices for its actual build inputs."""
import importlib.util
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location("tactile_windows_notices", Path(__file__).parents[1] / "collect_notices.py")
notices = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(notices)


def test_preserves_python_and_dependency_license_text(tmp_path, monkeypatch):
    python = tmp_path / "Python312"
    python.mkdir()
    (python / "LICENSE.txt").write_text("CPython license fixture", encoding="utf-8")
    monkeypatch.setattr(notices.sys, "base_prefix", str(python))
    package = tmp_path / "site-packages"
    license_file = package / "example-1.0.dist-info/licenses/THIRD_PARTY_NOTICES.md"
    license_file.parent.mkdir(parents=True)
    license_file.write_text("Dependency notices fixture", encoding="utf-8")
    distribution = SimpleNamespace(
        metadata={"Name": "example", "License-Expression": "MIT"}, version="1.0",
        files=[PurePosixPath("example-1.0.dist-info/licenses/THIRD_PARTY_NOTICES.md"),
               PurePosixPath("example/code.py")],
        locate_file=lambda path: package / path)
    monkeypatch.setattr(notices.importlib.metadata, "distributions", lambda: [distribution])
    target = tmp_path / "bundle/licenses"
    records = notices.collect(target)
    assert records[0]["name"] == "CPython"
    assert (target / records[0]["notice_files"][0]).read_text() == "CPython license fixture"
    assert records[1]["name"] == "example" and records[1]["license"] == "MIT"
    assert (target / records[1]["notice_files"][0]).read_text() == "Dependency notices fixture"


def test_missing_python_notice_stops_release(tmp_path, monkeypatch):
    monkeypatch.setattr(notices.sys, "base_prefix", str(tmp_path))
    with pytest.raises(FileNotFoundError, match="CPython"):
        notices.collect(tmp_path / "licenses")
