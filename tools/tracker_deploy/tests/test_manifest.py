"""Private installation metadata selects one coherent runtime without execution."""
from dataclasses import FrozenInstanceError
import json
import os
from pathlib import Path
import sys

import pytest

from vt_tracker_deploy.manifest import for_bundle, load_installation


def installation(home, identity="a" * 64, *, activate=True):
    base = home / ".local/share/visiontactile/tracker-deployment"
    version = base / "versions" / identity
    version.mkdir(parents=True)
    for path in (base, base / "versions", version):
        path.chmod(0o700)
    (version / "source").mkdir()
    (version / "venv/bin").mkdir(parents=True)
    (version / "venv/bin/python").symlink_to(sys.executable)
    (version / "venv/lib/python3.12/site-packages").mkdir(parents=True)
    (version / "inputs").mkdir()
    for name in ("bundle.json", "roles.yaml", "capture.pcapng"):
        path = version / "inputs" / name
        path.write_bytes(b"not metadata; must never read")
        path.chmod(0o600)
    document = dict(schema_version=1, installation_id=identity, capture_sha256="c" * 64, paths={
        "source": "source", "python": "venv/bin/python",
        "site_packages": "venv/lib/python3.12/site-packages",
        "bundle": "inputs/bundle.json", "roles": "inputs/roles.yaml",
        "capture": "inputs/capture.pcapng",
    })
    descriptor = version / "installation.json"
    descriptor.write_text(json.dumps(document))
    descriptor.chmod(0o600)
    if activate:
        current = base / "current.json"
        current.write_text(json.dumps(dict(schema_version=1, installation_id=identity)))
        current.chmod(0o600)
    return version, descriptor, document


def rewrite(path, document):
    path.write_text(json.dumps(document))


def test_missing_deployment_and_missing_version_are_optional(tmp_path):
    assert load_installation(home=tmp_path) is None
    version, _, _ = installation(tmp_path)
    assert load_installation(home=tmp_path, installation_id="b" * 64) is None
    assert for_bundle(version.parent / ("b" * 64) / "inputs/bundle.json", home=tmp_path) is None
    assert for_bundle(tmp_path / "manual/bundle.json", home=tmp_path) is None


def test_absent_installation_is_optional_without_posix_primitives(tmp_path, monkeypatch):
    monkeypatch.delattr(os, "O_NOFOLLOW")
    assert load_installation(home=tmp_path) is None


def test_existing_installation_without_posix_primitives_reports_unsupported(tmp_path, monkeypatch):
    installation(tmp_path)
    monkeypatch.delattr(os, "O_NOFOLLOW")
    with pytest.raises(ValueError, match="platform"):
        load_installation(home=tmp_path)


def test_loader_resolves_exact_paths_and_keeps_system_python_symlink(tmp_path, monkeypatch):
    version, _, _ = installation(tmp_path)
    original_open = os.open
    opened = []
    def guarded_open(path, flags, *args, **kwargs):
        opened.append(str(path))
        assert str(path) not in {"bundle.json", "roles.yaml", "capture.pcapng", "python"}
        return original_open(path, flags, *args, **kwargs)
    monkeypatch.setattr(os, "open", guarded_open)
    monkeypatch.setattr("subprocess.Popen", lambda *_a, **_k: pytest.fail("must not execute"))
    value = load_installation(home=tmp_path)
    assert value.installation_id == "a" * 64
    assert value.source == version / "source"
    assert value.python == version / "venv/bin/python"
    assert value.site_packages == version / "venv/lib/python3.12/site-packages"
    assert value.bundle == version / "inputs/bundle.json"
    assert value.roles == version / "inputs/roles.yaml"
    assert value.capture == version / "inputs/capture.pcapng"
    assert value.capture_sha256 == "c" * 64
    assert "current.json" in opened and "installation.json" in opened
    with pytest.raises(FrozenInstanceError):
        value.capture_sha256 = "d" * 64


def test_bundle_lookup_keeps_previous_version_after_current_changes(tmp_path):
    first, _, _ = installation(tmp_path)
    second, _, _ = installation(tmp_path, "b" * 64)
    assert load_installation(home=tmp_path).bundle == second / "inputs/bundle.json"
    assert for_bundle(first / "inputs/bundle.json", home=tmp_path).bundle == first / "inputs/bundle.json"
    assert for_bundle(first / "inputs/roles.yaml", home=tmp_path) is None


@pytest.mark.parametrize("change", [
    lambda d: d.update(schema_version=2),
    lambda d: d.update(schema_version=True),
    lambda d: d.update(installation_id="b" * 64),
    lambda d: d.update(capture_sha256="not-digest"),
    lambda d: d.update(capture_sha256="C" * 64),
    lambda d: d.update(unknown=True),
    lambda d: d["paths"].update(source="/outside"),
    lambda d: d["paths"].update(bundle="../bundle.json"),
    lambda d: d["paths"].update(python="venv/../python"),
    lambda d: d["paths"].update(site_packages="venv/lib/python3.12/../../outside"),
    lambda d: d["paths"].update(extra="inputs/bundle.json"),
])
def test_invalid_schema_and_path_mapping_are_refused(tmp_path, change):
    _, path, document = installation(tmp_path)
    change(document)
    rewrite(path, document)
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)


@pytest.mark.parametrize("target", ["current", "descriptor", "base", "versions", "version", "inputs"])
def test_unsafe_permissions_are_refused(tmp_path, target):
    version, descriptor, _ = installation(tmp_path)
    base = version.parent.parent
    paths = dict(current=base / "current.json", descriptor=descriptor, base=base,
                 versions=version.parent, version=version, inputs=version / "inputs/bundle.json")
    paths[target].chmod(0o644 if paths[target].is_file() else 0o755)
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)


@pytest.mark.parametrize("target", ["current", "descriptor", "source", "inputs", "venv"])
def test_symlink_metadata_or_directory_escape_is_refused(tmp_path, target):
    version, descriptor, _ = installation(tmp_path)
    paths = dict(current=version.parent.parent / "current.json", descriptor=descriptor,
                 source=version / "source", inputs=version / "inputs", venv=version / "venv")
    path = paths[target]
    moved = tmp_path / ("outside-" + target)
    path.rename(moved)
    path.symlink_to(moved, target_is_directory=moved.is_dir())
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)


@pytest.mark.parametrize("body", [
    '{"schema_version":1,"schema_version":1}',
    "not json",
    " " * (64 * 1024 + 1),
])
def test_malformed_duplicate_or_oversized_metadata_is_refused(tmp_path, body):
    _, path, _ = installation(tmp_path)
    path.write_text(body)
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)


def test_descriptor_for_missing_current_target_is_not_treated_as_a_complete_installation(tmp_path):
    version, descriptor, _ = installation(tmp_path)
    descriptor.unlink()
    assert load_installation(home=tmp_path) is None
    assert for_bundle(version / "inputs/bundle.json", home=tmp_path) is None


def test_nonexecutable_python_and_missing_input_are_refused(tmp_path):
    version, _, _ = installation(tmp_path)
    path = version / "venv/bin/python"
    path.unlink()
    path.write_text("not executable")
    path.chmod(0o600)
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)
    path.chmod(0o700)
    (version / "inputs/capture.pcapng").unlink()
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)


def test_private_input_file_symlink_is_refused(tmp_path):
    version, _, _ = installation(tmp_path)
    path = version / "inputs/bundle.json"
    target = tmp_path / "other-bundle.json"
    path.rename(target)
    path.symlink_to(target)
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)


@pytest.mark.parametrize("identity", ["../escape", "/absolute", "a" * 63, "A" * 64])
def test_unsafe_version_identifiers_are_refused_without_path_traversal(tmp_path, identity):
    version, _, _ = installation(tmp_path)
    current = version.parent.parent / "current.json"
    current.write_text(json.dumps(dict(schema_version=1, installation_id=identity)))
    with pytest.raises(ValueError):
        load_installation(home=tmp_path)
    with pytest.raises(ValueError):
        load_installation(home=tmp_path, installation_id=identity)
