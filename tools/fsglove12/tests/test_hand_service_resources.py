"""The headless and embedded UI use one bounded resource trust boundary."""
import hashlib
import json
import os
from pathlib import Path

import pytest

from vt_fsglove12 import __version__
from vt_fsglove12.hand_service_resources import MAX_JSON_BYTES, _read, validate_bundle_manifest, load_service_resources


def test_distribution_version_matches_public_runtime():
    metadata = (Path(__file__).parents[1] / "pyproject.toml").read_text()
    assert f'version = "{__version__}"' in metadata
    assert __version__ == "0.1.3"


def test_shared_manifest_json_read_is_bounded(tmp_path):
    (tmp_path / "manifest.json").write_bytes(b" " * (MAX_JSON_BYTES + 1))
    with pytest.raises(ValueError, match="size limit"):
        validate_bundle_manifest(tmp_path, hand="right")
    with pytest.raises(ValueError, match="size limit"):
        _read(tmp_path / "manifest.json")


@pytest.mark.parametrize("hand", ["left", "right"])
def test_manifest_streams_model_hash_and_requires_declared_hand(tmp_path, hand):
    names = ["hand.json", "config.json", "rest.json", f"mano/models/MANO_{hand.upper()}.pkl"]
    if hand == "left":
        names.append("tactile_layout.json")
    manifest = dict(schema_version=1, product="hand-motion-touch", files={})
    for name in names:
        path = tmp_path / name
        path.parent.mkdir(exist_ok=True, parents=True)
        path.write_bytes(b"opaque; validation does not deserialize a model")
        manifest["files"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    assert validate_bundle_manifest(tmp_path, hand=hand) == (tmp_path, manifest)
    (tmp_path / names[0]).write_bytes(b"altered")
    with pytest.raises(ValueError, match="checksum mismatch"):
        validate_bundle_manifest(tmp_path, hand=hand)


@pytest.mark.parametrize("missing", ["config", "rest", "mano_assets", "fsglove_source"])
def test_service_rejects_incomplete_explicit_selection_before_opening_resources(missing):
    paths = dict(config="/not-opened/config.json", rest="/not-opened/rest.json",
                 mano_assets="/not-opened/mano", fsglove_source="/not-opened/upstream")
    paths.pop(missing)
    with pytest.raises(ValueError, match="Explicit service resources require"):
        load_service_resources(hand="right", **paths)


def _real_bundle(hand):
    path = os.environ.get(f"VT_TEST_{hand.upper()}_RESOURCES")
    if not path:
        pytest.skip("requires explicitly selected real hand resource bundle")
    return Path(path)


@pytest.mark.parametrize("hand", ["left", "right"])
def test_explicit_and_bundle_resources_keep_identical_numerical_inputs(hand):
    root = _real_bundle(hand)
    bundle = load_service_resources(hand=hand, resources=root)
    explicit = load_service_resources(hand=hand, config=root / "config.json", rest=root / "rest.json",
        mano_assets=root / "mano", fsglove_source=root / "upstream")
    # Bundled display axes are optional metadata, never numerical solver input.
    for key in ("config", "rest", "mano_assets", "upstream_checkout", "hand"):
        assert explicit[key] == bundle[key]
    assert explicit["model_axes"] is None


@pytest.mark.parametrize("hand", ["left", "right"])
def test_explicit_rest_cannot_belong_to_other_hand(tmp_path, hand):
    root = _real_bundle(hand)
    rest = json.loads((root / "rest.json").read_text())
    rest.setdefault("provenance", {})["hand"] = "left" if hand == "right" else "right"
    path = tmp_path / "other-rest.json"
    path.write_text(json.dumps(rest))
    with pytest.raises(ValueError, match="does not match"):
        load_service_resources(hand=hand, config=root / "config.json", rest=path,
            mano_assets=root / "mano", fsglove_source=root / "upstream")


def test_left_explicit_rest_must_identify_hand(tmp_path):
    root = _real_bundle("left")
    rest = json.loads((root / "rest.json").read_text())
    rest.get("provenance", {}).pop("hand", None)
    path = tmp_path / "unknown-rest.json"
    path.write_text(json.dumps(rest))
    with pytest.raises(ValueError, match="explicitly identify"):
        load_service_resources(hand="left", config=root / "config.json", rest=path,
            mano_assets=root / "mano", fsglove_source=root / "upstream")


@pytest.mark.parametrize("hand", ["left", "right"])
def test_explicit_model_directory_must_contain_selected_hand(tmp_path, hand):
    root = _real_bundle(hand)
    with pytest.raises(ValueError, match="Missing .* MANO model"):
        load_service_resources(hand=hand, config=root / "config.json", rest=root / "rest.json",
            mano_assets=tmp_path, fsglove_source=root / "upstream")
