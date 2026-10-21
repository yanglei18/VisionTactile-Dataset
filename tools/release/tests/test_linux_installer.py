"""Build contracts with synthetic ELF bytes; no compiler or package install runs."""

import importlib.util
import json
from pathlib import Path
import struct
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "build_linux_installer.py"
SPEC = importlib.util.spec_from_file_location("linux_bootstrap_builder", SCRIPT)
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


def elf():
    return b"\x7fELF\x02\x01" + bytes(12) + struct.pack("<H", 62) + b"synthetic ELF fixture"


@pytest.fixture
def sources(tmp_path):
    root = tmp_path / "源码 $(not-a-command) with spaces"
    names = ["tools/capture_deploy/install_full.py", "tools/capture_deploy/oneclick/__init__.py",
             "tools/capture_deploy/oneclick/engine.py", "tools/capture_deploy/oneclick/gui.py",
             "tools/capture_deploy/oneclick/delivery.py", "tools/capture_deploy/oneclick/plan.py",
             "tools/tracker_deploy/src/vt_tracker_deploy/__init__.py",
             "tools/tracker_deploy/src/vt_tracker_deploy/archive.py", "docs/guide.md", "LICENSE"]
    for name in names:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("# Synthetic fixture\n")
    return root


@pytest.fixture
def fake_build(tmp_path, sources, monkeypatch):
    output = tmp_path / "export"
    calls = []
    monkeypatch.setattr(builder, "check_platform", lambda: None)
    monkeypatch.setattr(builder, "inspect_python", lambda python: {"python_full": "3.12.3", "packages": ["PyInstaller==6.22.3"]})
    def run(argv, **kwargs):
        calls.append((argv, kwargs))
        workspace = kwargs["cwd"]
        target = workspace / "dist" / builder.NAME
        target.parent.mkdir()
        target.write_bytes(elf())
        return subprocess.CompletedProcess(argv, 0, "")
    monkeypatch.setattr(builder, "run", run)
    def licenses(directory, analysis, probe, root):
        directory.mkdir()
        (directory / "Python.txt").write_text("Synthetic license")
        return [{"name": "libpython3.12.so", "package": "synthetic", "version": "1", "license": "LICENSES/Python.txt"}]
    monkeypatch.setattr(builder, "collect_licenses", licenses)
    return output, calls


def test_manifest_binds_all_runtime_sources_and_export_files(sources, fake_build):
    output, calls = fake_build
    report = builder.build(sources, Path("/build venv/bin/python"), output)
    manifest = json.loads((output / "BUILD_MANIFEST.json").read_text())
    names = {entry["path"] for entry in manifest["sources"]}
    assert "tools/capture_deploy/oneclick/plan.py" in names
    assert "tools/tracker_deploy/src/vt_tracker_deploy/archive.py" in names
    assert "docs/guide.md" not in names
    assert manifest["target"] == builder.TARGET
    assert manifest["kind"] == "visiontactile-linux-bootstrap"
    assert manifest["build_environment"]["packages"] == ["PyInstaller==6.22.3"]
    assert {item["path"] for item in manifest["files"]} == {builder.NAME, "LICENSES/Python.txt"}
    for item in manifest["files"]:
        assert item["sha256"] == builder.digest(output / item["path"])
    assert report["sha256"] == builder.digest(output / builder.NAME)
    assert calls[0][0][0] == "/build venv/bin/python"
    assert str(sources / "tools/capture_deploy/install_full.py") == calls[0][0][-1]
    assert (output / builder.NAME).stat().st_mode & 0o777 == 0o755
    assert not list(output.parent.glob(".vt-linux-bootstrap-*"))


def test_modified_source_invalidates_build_without_publication(sources, fake_build, monkeypatch):
    output, _ = fake_build
    original = builder.run
    def changing_source(*args, **kwargs):
        result = original(*args, **kwargs)
        with (sources / "tools/capture_deploy/oneclick/engine.py").open("a") as stream:
            stream.write("# change while compiling\n")
        return result
    monkeypatch.setattr(builder, "run", changing_source)
    with pytest.raises(builder.BuildError, match="changed"):
        builder.build(sources, Path("/python"), output)
    assert not output.exists()
    assert not list(output.parent.glob(".vt-linux-bootstrap-*"))


def test_failed_build_cleans_only_its_temporary_tree(sources, fake_build, monkeypatch):
    output, _ = fake_build
    unrelated = output.parent / "keep-me"
    unrelated.write_text("existing")
    def failed(*args, **kwargs):
        raise builder.BuildError("simulated compiler failure")
    monkeypatch.setattr(builder, "run", failed)
    with pytest.raises(builder.BuildError, match="simulated"):
        builder.build(sources, Path("/python"), output)
    assert unrelated.read_text() == "existing"
    assert not output.exists()
    assert not list(output.parent.glob(".vt-linux-bootstrap-*"))


def test_existing_output_is_never_overwritten(sources, fake_build):
    output, calls = fake_build
    output.mkdir()
    marker = output / "existing"
    marker.write_text("keep")
    with pytest.raises(builder.BuildError, match="already exists"):
        builder.build(sources, Path("/python"), output)
    assert marker.read_text() == "keep"
    assert not calls


def test_output_inside_source_tree_is_rejected(sources, fake_build):
    with pytest.raises(builder.BuildError, match="outside"):
        builder.build(sources, Path("/python"), sources / "built")


def test_build_platform_is_checked_before_commands(monkeypatch):
    monkeypatch.setattr(builder.platform, "system", lambda: "Linux")
    monkeypatch.setattr(builder.platform, "machine", lambda: "aarch64")
    monkeypatch.setattr(builder.platform, "freedesktop_os_release", lambda: {"ID": "ubuntu", "VERSION_ID": "24.04"})
    with pytest.raises(builder.BuildError, match="cross-compilation"):
        builder.check_platform()


def test_build_python_must_be_absolute():
    with pytest.raises(builder.BuildError, match="absolute"):
        builder.inspect_python(Path("venv/bin/python"))


def test_analysis_extracts_native_gui_dependencies(tmp_path):
    path = tmp_path / "Analysis.toc"
    path.write_text(repr((["ignored"], [("_tkinter.so", "/usr/lib/python3.12/_tkinter.so", "EXTENSION"),
                                      ("libtk.so", "/lib/libtk.so", "BINARY"),
                                      ("test", "/example.py", "PYMODULE")])))
    assert set(builder.binary_dependencies(path)) == {"_tkinter.so", "libtk.so"}


def test_missing_tk_binary_is_rejected(tmp_path):
    path = tmp_path / "Analysis.toc"
    path.write_text(repr([("libpython.so", "/lib/libpython.so", "BINARY")]))
    with pytest.raises(builder.BuildError, match="Tk GUI"):
        builder.binary_dependencies(path)


@pytest.mark.parametrize("payload", [b"#!/bin/bash\n", b"\x7fELF\x01\x01" + bytes(20),
                                     b"\x7fELF\x02\x01" + bytes(12) + struct.pack("<H", 183)])
def test_wrong_executable_architecture_is_rejected(tmp_path, payload):
    path = tmp_path / "installer"
    path.write_bytes(payload)
    with pytest.raises(builder.BuildError, match="ELF x86_64"):
        builder.verify_elf(path)


def test_unknown_runtime_library_license_owner_is_rejected(monkeypatch):
    monkeypatch.setattr(builder, "run", lambda argv, **kwargs: subprocess.CompletedProcess(argv, 1, ""))
    with pytest.raises(builder.BuildError, match="copyright owner"):
        builder._package_owner(Path("/opt/unknown/libcustom.so"))
