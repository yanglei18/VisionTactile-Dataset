"""Portable private snapshots: no source execution or hardware access."""
from hashlib import sha256
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import warnings
import zipfile

import pytest

from vt_tracker_deploy import archive


WHEELS = (
    ("hid", "1.0.9", "py3-none-any"),
    ("numpy", "1.26.4", "cp312-cp312-manylinux_2_17_x86_64"),
    ("scapy", "2.7.0", "py3-none-any"),
    ("PyYAML", "6.0.2", "cp312-cp312-manylinux_2_17_x86_64"),
)


def wheel_bytes(name, version, tag):
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_DEFLATED) as target:
        directory = f"{name}-{version}.dist-info"
        target.writestr(f"{directory}/METADATA", f"Metadata-Version: 2.1\nName: {name}\nVersion: {version}\n")
        target.writestr(f"{directory}/WHEEL", f"Wheel-Version: 1.0\nTag: {tag}\n")
    return stream.getvalue()


@pytest.fixture
def inputs(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    for name in archive.RUNTIME_SOURCE_FILES:
        path = source / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("# Fixture source; never execute.\n" if name.endswith(".py") else "Fixture license notice\n")
    subprocess.run(["git", "init", "-q", str(source)], check=True)
    subprocess.run(["git", "-C", str(source), "add", "."], check=True)
    subprocess.run(["git", "-C", str(source), "-c", "user.name=Fixture",
                    "-c", "user.email=fixture@example.invalid", "commit", "-qm", "source fixture"], check=True)
    (source / "pyvut/live_bootstrap_runner.py").write_text("# Current verified local correction.\n")
    (source / "pyvut/wifi_info.json").write_text('{"ssid":"must not export","pass":"must not export"}')
    (source / "pyvut/tracker_core.py").write_text("raise AssertionError('must not export')")
    wheelhouse = tmp_path / "wheels"
    wheelhouse.mkdir()
    for name, version, tag in WHEELS:
        (wheelhouse / f"{name}-{version}-{tag}.whl").write_bytes(wheel_bytes(name, version, tag))
    capture = tmp_path / "capture.pcapng"
    capture.write_bytes(b"synthetic approved capture")
    digest = sha256(capture.read_bytes()).hexdigest()
    bundle = tmp_path / "bundle.json"
    bundle.write_text(json.dumps({"schema_version": 3, "capture_sha256": digest}))
    roles = tmp_path / "roles.yaml"
    roles.write_text("schema_version: 1\nroles: {}\n")
    return dict(source=source, bundle=bundle, roles=roles, capture=capture,
                capture_sha256=digest, wheelhouse=wheelhouse, output=tmp_path / "private.vttracker")


def rewrite(package, destination, change):
    with zipfile.ZipFile(package) as source:
        entries = [(info, source.read(info)) for info in source.infolist()]
    entries = change(entries)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as target:
            for info, content in entries:
                target.writestr(info, content)
    return destination


def test_private_snapshot_roundtrip_preserves_current_source_and_excludes_unneeded_files(inputs):
    original = (inputs["source"] / "pyvut/live_bootstrap_runner.py").read_bytes()
    manifest = archive.build_package(**inputs)
    package = archive.read_package(inputs["output"])
    assert stat.S_IMODE(inputs["output"].stat().st_mode) == 0o600
    assert package.manifest == manifest
    assert package.package_id == sha256(inputs["output"].read_bytes()).hexdigest()
    assert manifest["source"]["modified_files"] == ["pyvut/live_bootstrap_runner.py"]
    assert "source/pyvut/wifi_info.json" not in package.files
    assert "source/pyvut/tracker_core.py" not in package.files
    copied = package.files["source/pyvut/live_bootstrap_runner.py"]
    assert original in copied and b"Modified" in copied
    assert original == (inputs["source"] / "pyvut/live_bootstrap_runner.py").read_bytes()
    assert sha256(original).hexdigest().encode() in package.files["source/DEPLOYMENT_CHANGES.md"]
    assert package.files["inputs/capture.pcapng"] == b"synthetic approved capture"
    inputs["output"].unlink()
    assert package.files["inputs/capture.pcapng"] == b"synthetic approved capture"
    with pytest.raises(TypeError):
        package.files["extra"] = b"bad"


def test_existing_output_is_never_overwritten(inputs):
    inputs["output"].write_bytes(b"previous package")
    with pytest.raises((ValueError, FileExistsError)):
        archive.build_package(**inputs)
    assert inputs["output"].read_bytes() == b"previous package"


@pytest.mark.parametrize("kind", ["source", "capture", "wheel"])
def test_symlink_input_is_not_followed(inputs, tmp_path, kind):
    path = (inputs["source"] / "pyvut/live_probe.py" if kind == "source" else
            inputs["capture"] if kind == "capture" else next(inputs["wheelhouse"].glob("*.whl")))
    real = tmp_path / "symlink-target"
    path.rename(real)
    path.symlink_to(real)
    with pytest.raises(ValueError):
        archive.build_package(**inputs)
    assert not inputs["output"].exists()


@pytest.mark.parametrize("kind", ["wrong_capture", "wrong_bundle", "missing_source",
                                   "wrong_wheel_tag", "missing_wheel", "extra_wheel"])
def test_invalid_input_is_rejected_before_output_creation(inputs, kind):
    if kind == "wrong_capture":
        inputs["capture"].write_bytes(b"unapproved capture")
    elif kind == "wrong_bundle":
        inputs["bundle"].write_text(json.dumps({"schema_version": 3, "capture_sha256": "0" * 64}))
    elif kind == "missing_source":
        (inputs["source"] / "pyvut/live_probe.py").unlink()
    elif kind == "wrong_wheel_tag":
        old = next(inputs["wheelhouse"].glob("numpy-*.whl"))
        old.rename(inputs["wheelhouse"] / old.name.replace("cp312-cp312", "cp311-cp311"))
    elif kind == "missing_wheel":
        next(inputs["wheelhouse"].glob("hid-*.whl")).unlink()
    else:
        (inputs["wheelhouse"] / "hid-1.0.9-1-py3-none-any.whl").write_bytes(wheel_bytes("hid", "1.0.9", "py3-none-any"))
    with pytest.raises(ValueError):
        archive.build_package(**inputs)
    assert not inputs["output"].exists()


def test_wheel_metadata_cannot_be_disguised_by_filename(inputs):
    target = next(inputs["wheelhouse"].glob("hid-*.whl"))
    target.write_bytes(wheel_bytes("other", "1.0.9", "py3-none-any"))
    with pytest.raises(ValueError):
        archive.build_package(**inputs)


def test_inner_wheel_expansion_is_bounded_before_installation(inputs, monkeypatch):
    target = next(inputs["wheelhouse"].glob("hid-*.whl"))
    with zipfile.ZipFile(target, "a", compression=zipfile.ZIP_DEFLATED) as wheel:
        wheel.writestr("large.bin", b"\0" * 8192)
    monkeypatch.setattr(archive, "MAX_WHEEL_EXPANDED_BYTES", 4096)
    with pytest.raises(ValueError, match="Expanded wheel"):
        archive.build_package(**inputs)
    assert not inputs["output"].exists()


def test_builder_bounds_cumulative_payload_size(inputs, monkeypatch):
    monkeypatch.setattr(archive, "MAX_TOTAL_BYTES", 512)
    with pytest.raises(ValueError, match="Expanded package"):
        archive.build_package(**inputs)
    assert not inputs["output"].exists()


@pytest.mark.parametrize("name", ["../escape", "/absolute", "source\\escape", "source/../escape",
                                 "source//escape", "C:/escape", "./source/escape"])
def test_unsafe_zip_names_are_rejected(inputs, tmp_path, name):
    archive.build_package(**inputs)
    bad = rewrite(inputs["output"], tmp_path / "bad.vttracker",
                  lambda entries: entries + [(name, b"bad")])
    with pytest.raises(ValueError):
        archive.read_package(bad)


@pytest.mark.parametrize("kind", ["duplicate", "symlink", "extra", "missing", "hash", "manifest"])
def test_manifest_and_zip_integrity_are_checked(inputs, tmp_path, kind):
    archive.build_package(**inputs)

    def change(entries):
        if kind == "duplicate":
            return entries + [entries[0]]
        if kind == "symlink":
            info, content = next((i, c) for i, c in entries if i.filename == "inputs/bundle.json")
            info.create_system = 3
            info.external_attr = (stat.S_IFLNK | 0o777) << 16
        if kind == "extra":
            return entries + [("source/unknown.py", b"raise AssertionError")]
        if kind == "missing":
            return [(i, c) for i, c in entries if i.filename != "source/pyvut/live_probe.py"]
        if kind == "hash":
            return [(i, b"tampered" if i.filename == "source/pyvut/live_probe.py" else c) for i, c in entries]
        if kind == "manifest":
            return [(i, b'{"schema_version":1,"schema_version":2}' if i.filename == "manifest.json" else c)
                    for i, c in entries]
        return entries

    bad = rewrite(inputs["output"], tmp_path / "bad.vttracker", change)
    with pytest.raises(ValueError):
        archive.read_package(bad)


@pytest.mark.parametrize("limit,value", [("MAX_FILES", 2), ("MAX_FILE_BYTES", 16),
                                        ("MAX_TOTAL_BYTES", 32), ("MAX_MANIFEST_BYTES", 16)])
def test_declared_zip_sizes_are_bounded_before_reading(inputs, monkeypatch, limit, value):
    archive.build_package(**inputs)
    monkeypatch.setattr(archive, limit, value)
    with pytest.raises(ValueError):
        archive.read_package(inputs["output"])


def test_failed_write_cleans_up_new_partial_output(inputs, monkeypatch):
    original = zipfile.ZipFile.writestr
    calls = 0

    def fail(self, *args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("synthetic full disk")
        return original(self, *args, **kwargs)

    monkeypatch.setattr(zipfile.ZipFile, "writestr", fail)
    with pytest.raises(OSError):
        archive.build_package(**inputs)
    assert not inputs["output"].exists()
