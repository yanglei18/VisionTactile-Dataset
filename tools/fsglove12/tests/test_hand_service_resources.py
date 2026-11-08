"""The headless and embedded UI use one bounded resource trust boundary."""
import hashlib
import json
from pathlib import Path

import pytest

from vt_fsglove12 import __version__
from vt_fsglove12.hand_service_resources import MAX_JSON_BYTES, _read, validate_bundle_manifest


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
