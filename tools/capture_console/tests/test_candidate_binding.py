"""Candidate startup binding is private, explicit, and tied to one source tree."""
from pathlib import Path
import tempfile

import pytest

from vt_capture_console.backend import candidate_binding as binding
from vt_capture_console.backend.workstation_settings import atomic_json, private_directory
from vt_capture_core.software_identity import ROOTS, runtime_identity


@pytest.fixture
def candidate(tmp_path, monkeypatch):
    root = tmp_path / "checkout"
    for name in ROOTS:
        (root / name).mkdir(parents=True)
    script = root / "tools/capture_deploy/start_manager.sh"
    script.parent.mkdir(parents=True)
    script.write_text("#!/bin/sh\nexit 0\n")
    (root / "tools/capture_console/src/example.py").write_text("VALUE = 'candidate'\n")
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "home")
    monkeypatch.setattr(binding, "repository_root", lambda: root)
    with tempfile.TemporaryDirectory(prefix="vt-binding-") as temporary:
        directory = private_directory(Path(temporary) / "candidate")
        manifest = dict(schema_version=1, kind="visiontactile_candidate_migration", repo_root=str(root.resolve()))
        atomic_json(directory / "migration.json", manifest)
        atomic_json(directory / "workstation.json", dict(process_groups=[]))
        yield root, directory, manifest


@pytest.mark.parametrize("explicit_root", [False, True])
def test_candidate_binding_returns_exact_code_and_configuration_identity(candidate, explicit_root):
    root, directory, _ = candidate
    before = {path.name: path.read_bytes() for path in directory.iterdir()}
    result, identity = binding.load_candidate_binding(directory, **(dict(repo_root=root) if explicit_root else {}))
    assert result == directory
    assert identity == runtime_identity(root, directory / "workstation.json")
    assert identity["source_root"] == str(root.resolve())
    assert identity["configuration_path"] == str(directory / "workstation.json")
    assert len(identity["source_sha256"]) == 64
    assert {path.name: path.read_bytes() for path in directory.iterdir()} == before


def test_candidate_binding_accepts_a_source_alias_only_after_canonical_root_check(candidate, tmp_path):
    root, directory, _ = candidate
    alias = tmp_path / "checkout-alias"
    alias.symlink_to(root, target_is_directory=True)
    _, identity = binding.load_candidate_binding(directory, repo_root=alias)
    assert identity["source_root"] == str(root.resolve())


def test_candidate_binding_rejects_other_source_tree_before_loading_runtime(candidate, tmp_path, monkeypatch):
    _, directory, _ = candidate
    other = tmp_path / "other-checkout"
    other.mkdir()
    monkeypatch.setattr(binding, "runtime_identity", lambda *a: pytest.fail("loaded another tree"))
    with pytest.raises(ValueError, match="代码目录不匹配"):
        binding.load_candidate_binding(directory, repo_root=other)


@pytest.mark.parametrize("change", [dict(schema_version=2), dict(kind="other"), dict(repo_root="/other/checkout")])
def test_candidate_binding_rejects_unrecognized_migration_identity(candidate, change):
    root, directory, manifest = candidate
    atomic_json(directory / "migration.json", manifest | change)
    with pytest.raises(ValueError, match="代码目录不匹配"):
        binding.load_candidate_binding(directory, repo_root=root)


def test_candidate_binding_refuses_unprepared_directory_without_creating_it(candidate):
    root, directory, _ = candidate
    missing = directory / "not-prepared"
    with pytest.raises(ValueError, match="尚未准备"):
        binding.load_candidate_binding(missing, repo_root=root)
    assert not missing.exists()


def test_candidate_binding_rejects_public_directory_without_changing_permissions(candidate):
    root, directory, _ = candidate
    directory.chmod(0o755)
    try:
        with pytest.raises(ValueError, match="私有目录"):
            binding.load_candidate_binding(directory, repo_root=root)
        assert directory.stat().st_mode & 0o777 == 0o755
    finally:
        directory.chmod(0o700)


def test_candidate_binding_rejects_directory_symlink(candidate):
    root, directory, _ = candidate
    alias = directory.parent / "alias"
    alias.symlink_to(directory, target_is_directory=True)
    with pytest.raises(ValueError, match="不能是链接"):
        binding.load_candidate_binding(alias, repo_root=root)


@pytest.mark.parametrize("name", ["migration.json", "workstation.json"])
@pytest.mark.parametrize("bad_kind", ["public", "symlink", "directory"])
def test_candidate_binding_requires_private_regular_documents(candidate, name, bad_kind):
    root, directory, _ = candidate
    path = directory / name
    if bad_kind == "public":
        path.chmod(0o644)
    elif bad_kind == "symlink":
        target = directory / (name + ".actual")
        path.rename(target)
        path.symlink_to(target)
    else:
        path.unlink()
        path.mkdir(mode=0o700)
    with pytest.raises((ValueError, OSError)):
        binding.load_candidate_binding(directory, repo_root=root)


def test_candidate_binding_rejects_global_workstation_directory_even_with_valid_manifest(candidate, monkeypatch):
    root, _, manifest = candidate
    directory = private_directory(Path.home() / ".config/visiontactile/workbench")
    atomic_json(directory / "migration.json", manifest)
    atomic_json(directory / "workstation.json", dict(process_groups=[]))
    before = {path.name: path.read_bytes() for path in directory.iterdir()}
    monkeypatch.setattr(binding, "runtime_identity", lambda *a: pytest.fail("bound global settings"))
    with pytest.raises(ValueError, match="不能复用全局"):
        binding.load_candidate_binding(directory, repo_root=root)
    assert {path.name: path.read_bytes() for path in directory.iterdir()} == before


def test_candidate_binding_rejects_overlong_unix_socket_directory(candidate):
    root, directory, manifest = candidate
    long_directory = private_directory(directory / ("x" * 100))
    atomic_json(long_directory / "migration.json", manifest)
    atomic_json(long_directory / "workstation.json", dict(process_groups=[]))
    with pytest.raises(ValueError, match="目录过长"):
        binding.load_candidate_binding(long_directory, repo_root=root)
