"""Reject incomplete or escaped resource bundles before loading a model."""
import hashlib
import json

import pytest

from vt_hand_tactile_viewer.resources import MAX_JSON_BYTES, load_resources, read_json


REQUIRED = ("hand.json", "rest.json", "config.json", "mano/models/MANO_RIGHT.pkl", "default_response.json")


@pytest.fixture
def bundle(tmp_path):
    root = tmp_path / "runtime"
    root.mkdir()
    manifest = dict(schema_version=1, product="hand-motion-touch", files={})
    # Invalid model bytes are deliberate: the integrity checks must reject
    # each damaged bundle before any model or calibration parsing can happen.
    for name in REQUIRED:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"not a model or calibration document")
        manifest["files"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return root, manifest


def write_manifest(root, manifest):
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def test_missing_runtime_cannot_fall_back_to_machine_local_assets(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_resources(tmp_path / "absent-runtime")


@pytest.mark.parametrize("name", REQUIRED)
def test_required_file_missing_from_manifest_is_not_optional(bundle, name):
    root, manifest = bundle
    del manifest["files"][name]
    write_manifest(root, manifest)
    with pytest.raises(ValueError, match="Incomplete"):
        load_resources(root)


@pytest.mark.parametrize("name", REQUIRED)
def test_required_file_missing_on_disk_cannot_start(bundle, name):
    root, manifest = bundle
    (root / name).unlink()
    write_manifest(root, manifest)
    with pytest.raises(FileNotFoundError):
        load_resources(root)


@pytest.mark.parametrize("name", REQUIRED)
def test_changed_resource_bytes_fail_checksum_before_model_parsing(bundle, name):
    root, manifest = bundle
    (root / name).write_bytes(b"changed after packaging")
    write_manifest(root, manifest)
    with pytest.raises(ValueError, match="checksum mismatch"):
        load_resources(root)


@pytest.mark.parametrize("name", ["../outside", "/outside", "sub/../../outside", "..\\outside"])
def test_manifest_paths_cannot_escape_runtime(bundle, name):
    root, manifest = bundle
    manifest["files"][name] = "0" * 64
    write_manifest(root, manifest)
    with pytest.raises(ValueError, match="Invalid resource path"):
        load_resources(root)


def test_symlink_cannot_supply_an_external_private_model_even_with_correct_hash(bundle, tmp_path):
    root, manifest = bundle
    external = tmp_path / "external-model.pkl"
    external.write_bytes(b"external model must not be read")
    model = root / "mano/models/MANO_RIGHT.pkl"
    model.unlink()
    try:
        model.symlink_to(external)
    except OSError:
        pytest.skip("OS does not permit creating symlinks")
    manifest["files"]["mano/models/MANO_RIGHT.pkl"] = hashlib.sha256(external.read_bytes()).hexdigest()
    write_manifest(root, manifest)
    with pytest.raises(ValueError):
        load_resources(root)


@pytest.mark.parametrize("field, value", [("schema_version", 2), ("product", "tactile-3d"), ("files", [])])
def test_other_product_or_manifest_schema_is_rejected(bundle, field, value):
    root, manifest = bundle
    manifest[field] = value
    write_manifest(root, manifest)
    with pytest.raises(ValueError, match="Invalid Hand Motion"):
        load_resources(root)


def test_resource_document_size_is_bounded(tmp_path):
    path = tmp_path / "oversized.json"
    path.write_bytes(b" " * (MAX_JSON_BYTES + 1))
    with pytest.raises(ValueError, match="size limit"):
        read_json(path)
