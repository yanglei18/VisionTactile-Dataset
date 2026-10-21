"""Delivery checks use synthetic bytes and never import device payload code."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
from types import SimpleNamespace

import pytest

from oneclick.delivery import DeliveryError, inspect_delivery, stage_delivery


REQUIRED_FILES = {
    "tools/capture_deploy/install.sh": b"#!/bin/sh\n# inert fixture\n",
    "tools/capture_deploy/install_system.py": b"# inert fixture\n",
    "tools/capture_deploy/start_workbench.sh": b"#!/bin/sh\n# inert fixture\n",
    "README.md": b"synthetic public source\n",
}


def digest(content):
    return hashlib.sha256(content).hexdigest()


@pytest.fixture
def synthetic_delivery(tmp_path, monkeypatch):
    directory = tmp_path / "download"
    source = directory / "Vision-Tactile-Dataset"
    source.mkdir(parents=True)
    for name, content in REQUIRED_FILES.items():
        target = source / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
    public = {
        "schema_version": 1,
        "files": [{"path": name, "bytes": len(content), "sha256": digest(content)}
                  for name, content in REQUIRED_FILES.items()],
    }
    raw = json.dumps(public).encode()
    (source / "PUBLIC_SOURCE_MANIFEST.json").write_bytes(raw)
    package = directory / "deployment/tracker.vttracker"
    package.parent.mkdir()
    package.write_bytes(b"synthetic non-device payload")
    manifest = {
        "schema_version": 1,
        "distribution": "restricted-device-delivery",
        "source": {"manifest_path": "Vision-Tactile-Dataset/PUBLIC_SOURCE_MANIFEST.json",
                   "manifest_sha256": digest(raw)},
        "tracker_package": {"path": "deployment/tracker.vttracker",
                            "bytes": package.stat().st_size, "sha256": digest(package.read_bytes())},
    }
    (directory / "DELIVERY_MANIFEST.json").write_text(json.dumps(manifest))
    calls = []

    def read_package(path):
        calls.append(Path(path))
        return SimpleNamespace(package_id=digest(Path(path).read_bytes()))

    monkeypatch.setattr("vt_tracker_deploy.archive.read_package", read_package)
    return SimpleNamespace(directory=directory, source=source, package=package,
                           manifest=manifest, public=public, calls=calls)


def rewrite_public(fixture):
    raw = json.dumps(fixture.public).encode()
    (fixture.source / "PUBLIC_SOURCE_MANIFEST.json").write_bytes(raw)
    fixture.manifest["source"]["manifest_sha256"] = digest(raw)
    (fixture.directory / "DELIVERY_MANIFEST.json").write_text(json.dumps(fixture.manifest))


def test_inspection_is_read_only_and_snapshots_all_verified_bytes(synthetic_delivery):
    fixture = synthetic_delivery
    before = {p.relative_to(fixture.directory): p.read_bytes()
              for p in fixture.directory.rglob("*") if p.is_file()}
    result = inspect_delivery(fixture.directory)
    assert result.files["README.md"] == REQUIRED_FILES["README.md"]
    assert result.package_bytes == b"synthetic non-device payload"
    assert result.tracker_sha256 == digest(result.package_bytes)
    assert fixture.calls == [fixture.package]
    with pytest.raises(TypeError):
        result.files["README.md"] = b"changed"
    after = {p.relative_to(fixture.directory): p.read_bytes()
             for p in fixture.directory.rglob("*") if p.is_file()}
    assert after == before


@pytest.mark.parametrize("member", ["README.md", "PUBLIC_SOURCE_MANIFEST.json"])
def test_changed_source_bytes_fail_before_device_package_validation(synthetic_delivery, member):
    fixture = synthetic_delivery
    (fixture.source / member).write_bytes(b"corrupted")
    with pytest.raises(DeliveryError):
        inspect_delivery(fixture.directory)
    assert not fixture.calls


def test_changed_package_bytes_fail_before_package_validation(synthetic_delivery):
    fixture = synthetic_delivery
    fixture.package.write_bytes(b"tampered device package")
    with pytest.raises(DeliveryError):
        inspect_delivery(fixture.directory)
    assert not fixture.calls


def test_package_replaced_during_validation_cannot_use_stale_snapshot(synthetic_delivery, monkeypatch):
    fixture = synthetic_delivery
    monkeypatch.setattr("vt_tracker_deploy.archive.read_package",
                        lambda _: SimpleNamespace(package_id="0" * 64))
    with pytest.raises(DeliveryError, match="发生变化"):
        inspect_delivery(fixture.directory)


@pytest.mark.parametrize("name", ["../outside", "/outside", "a/../b", "a//b", "a/./b", "a\\b", "a\nb"])
def test_manifest_rejects_paths_outside_its_source_tree(synthetic_delivery, name):
    fixture = synthetic_delivery
    fixture.public["files"][0]["path"] = name
    rewrite_public(fixture)
    with pytest.raises(DeliveryError, match="无效文件路径"):
        inspect_delivery(fixture.directory)
    assert not fixture.calls


@pytest.mark.parametrize("size", [True, -1, 5 * 1024 * 1024 + 1, "12"])
def test_manifest_rejects_invalid_file_sizes(synthetic_delivery, size):
    fixture = synthetic_delivery
    fixture.public["files"][0]["bytes"] = size
    rewrite_public(fixture)
    with pytest.raises(DeliveryError, match="摘要无效"):
        inspect_delivery(fixture.directory)
    assert not fixture.calls


def test_manifest_rejects_duplicate_file_entries(synthetic_delivery):
    fixture = synthetic_delivery
    fixture.public["files"].append(dict(fixture.public["files"][0]))
    rewrite_public(fixture)
    with pytest.raises(DeliveryError, match="重复"):
        inspect_delivery(fixture.directory)


def test_manifest_parse_error_does_not_disclose_input(synthetic_delivery):
    fixture = synthetic_delivery
    secret = "synthetic-private-credential-never-print"
    (fixture.directory / "DELIVERY_MANIFEST.json").write_text('{"secret": "' + secret)
    with pytest.raises(DeliveryError) as captured:
        inspect_delivery(fixture.directory)
    assert secret not in str(captured.value)


def test_duplicate_json_keys_are_rejected(synthetic_delivery):
    fixture = synthetic_delivery
    (fixture.directory / "DELIVERY_MANIFEST.json").write_text('{"schema_version": 1, "schema_version": 1}')
    with pytest.raises(DeliveryError):
        inspect_delivery(fixture.directory)


@pytest.mark.parametrize("member", ["README.md", "tools/capture_deploy"])
def test_symlink_inputs_are_rejected_even_inside_delivery(synthetic_delivery, member):
    fixture = synthetic_delivery
    target = fixture.source / member
    original = target.with_name(target.name + "-original")
    target.rename(original)
    target.symlink_to(original, target_is_directory=original.is_dir())
    with pytest.raises(DeliveryError, match="符号链接"):
        inspect_delivery(fixture.directory)
    assert not fixture.calls


def test_staged_installation_survives_deleting_download_and_keeps_package_private(synthetic_delivery, tmp_path):
    fixture = synthetic_delivery
    result = inspect_delivery(fixture.directory)
    home = tmp_path / "new user"
    shutil.rmtree(fixture.directory)
    source = stage_delivery(result, home)
    assert source.is_relative_to(home / ".local/share/visiontactile/application/versions")
    assert not source.is_relative_to(fixture.directory)
    assert (source / "README.md").read_bytes() == REQUIRED_FILES["README.md"]
    package = source.parent / "deployment/tracker.vttracker"
    assert package.read_bytes() == result.package_bytes
    assert stat.S_IMODE(package.stat().st_mode) == 0o600
    assert stat.S_IMODE(source.parent.stat().st_mode) == 0o700
    assert (source / "tools/capture_deploy/install.sh").stat().st_mode & stat.S_IXUSR
    assert stage_delivery(result, home) == source
    assert not list(source.parent.parent.glob(".staging-*"))


def test_staging_does_not_overwrite_changed_existing_installation(synthetic_delivery, tmp_path):
    result = inspect_delivery(synthetic_delivery.directory)
    home = tmp_path / "home"
    source = stage_delivery(result, home)
    modified = source / "README.md"
    modified.write_bytes(b"local modifications")
    with pytest.raises(DeliveryError, match="不能覆盖"):
        stage_delivery(result, home)
    assert modified.read_bytes() == b"local modifications"


def test_staging_rejects_symlink_installation_parent(synthetic_delivery, tmp_path):
    result = inspect_delivery(synthetic_delivery.directory)
    home = tmp_path / "home"
    home.mkdir(mode=0o700)
    other = tmp_path / "other"
    other.mkdir()
    (home / ".local").symlink_to(other, target_is_directory=True)
    with pytest.raises(DeliveryError, match="权限不安全"):
        stage_delivery(result, home)
    assert list(other.iterdir()) == []


def test_shared_group_umask_still_stages_private_directories_and_allows_retry(synthetic_delivery, tmp_path):
    result = inspect_delivery(synthetic_delivery.directory)
    home = tmp_path / "home"
    previous = os.umask(0o002)
    try:
        source = stage_delivery(result, home)
        directories = [home, *[p for p in home.rglob("*") if p.is_dir()]]
        assert directories
        assert all(stat.S_IMODE(p.stat().st_mode) == 0o700 for p in directories)
        assert stage_delivery(result, home) == source
        assert (source / "README.md").read_bytes() == REQUIRED_FILES["README.md"]
    finally:
        os.umask(previous)
