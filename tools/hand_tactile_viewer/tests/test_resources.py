"""Reject incomplete or escaped resource bundles before loading a model."""
import hashlib
import json
from unittest.mock import Mock

import pytest

from vt_hand_tactile_viewer.resources import MAX_JSON_BYTES, load_resources, read_json


REQUIRED = ("hand.json", "rest.json", "config.json", "mano/models/MANO_RIGHT.pkl")


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


@pytest.fixture
def semantic_bundle(bundle, monkeypatch):
    """Isolate model semantics so optional tactile references can be verified."""
    root, manifest = bundle
    for name, value in (("config.json", {"hand": "right"}), ("rest.json", {})):
        path = root / name
        path.write_text(json.dumps(value))
        manifest["files"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    monkeypatch.setattr("vt_fsglove12.config.validate_config", lambda value: value)
    monkeypatch.setattr("vt_fsglove12.config.check_model_hand", lambda *args: None)
    monkeypatch.setattr("vt_fsglove12.calibration.validate_rest", lambda *args: None)
    monkeypatch.setattr("vt_fsglove12.upstream_core.inspect_upstream_sources", lambda *args: {})
    monkeypatch.setattr("vt_capture_console.ui.widgets.tactile_model_loader.load_bundle_file",
                        lambda *args: {})
    return root, manifest


def test_missing_reference_does_not_prevent_resource_loading(semantic_bundle):
    root, manifest = semantic_bundle
    write_manifest(root, manifest)
    assert load_resources(root)["response_profile"] is None


def test_unlisted_reference_is_not_loaded_from_adjacent_files(semantic_bundle):
    root, manifest = semantic_bundle
    (root / "default_response.json").write_text('{"protocol":"obsolete"}')
    write_manifest(root, manifest)
    assert load_resources(root)["response_profile"] is None


def test_declared_obsolete_reference_is_rejected(semantic_bundle):
    root, manifest = semantic_bundle
    path = root / "default_response.json"
    path.write_text('{"schema_version":2,"protocol":"vt_tactile_adc400_v1"}')
    manifest["files"][path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    write_manifest(root, manifest)
    with pytest.raises(ValueError, match="Unsupported"):
        load_resources(root)


def test_left_bundle_cannot_use_right_hand_or_baseline_layout(semantic_bundle):
    root, manifest = semantic_bundle
    write_manifest(root, manifest)
    # The wrong model/configuration must fail before a solver can be created.
    with pytest.raises(ValueError):
        load_resources(root, hand="left")


def test_left_bundle_requires_hashed_explicit_layout(tmp_path):
    root = tmp_path / "runtime"
    root.mkdir()
    manifest = {"schema_version": 1, "product": "hand-motion-touch", "files": {}}
    for name in ("hand.json", "rest.json", "config.json", "mano/models/MANO_LEFT.pkl"):
        target = root / name
        target.parent.mkdir(exist_ok=True, parents=True)
        target.write_bytes(b"{}")
        manifest["files"][name] = hashlib.sha256(target.read_bytes()).hexdigest()
    write_manifest(root, manifest)
    with pytest.raises(ValueError, match="layout"):
        load_resources(root, hand="left")


def test_preparing_right_resources_pins_the_historical_layout(tmp_path, monkeypatch):
    from vt_hand_tactile_viewer.prepare import prepare
    from vt_tactile_capture.hand_layout import DEFAULT_RIGHT_LAYOUT

    model_root = tmp_path / "mano"
    (model_root / "models").mkdir(parents=True)
    (model_root / "models/MANO_RIGHT.pkl").write_bytes(b"test")
    notice = tmp_path / "notice.txt"
    notice.write_text("Test fixture", encoding="utf-8")
    model = Mock()
    model.export_rest.return_value = {}
    model.render.return_value = {"joints_m": []}
    monkeypatch.setattr("vt_fsglove12.mano.ManoBackend", lambda *args, **kwargs: model)
    monkeypatch.setattr("vt_fsglove12.guided_calibration.model_motion_axes", lambda *args, **kwargs: {})
    monkeypatch.setattr("vt_fsglove12.upstream_core.inspect_upstream_sources", lambda *args: {"sources": {}})

    root = prepare(tmp_path / "runtime", mano_assets=model_root, fsglove_source=tmp_path,
                   config=None, license_file=notice)
    layout = (root / "tactile_layout.json").read_bytes()
    assert layout == DEFAULT_RIGHT_LAYOUT.read_bytes()
    manifest = read_json(root / "manifest.json")
    assert manifest["files"]["tactile_layout.json"] == hashlib.sha256(layout).hexdigest()
