"""Portable checked resources. Model files are maintainer-supplied, never searched."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath


MAX_JSON_BYTES = 2 * 1024 * 1024


def read_json(path):
    with Path(path).open("rb") as stream:
        content = stream.read(MAX_JSON_BYTES + 1)
    if len(content) > MAX_JSON_BYTES:
        raise ValueError("Resource document exceeds its size limit")
    return json.loads(content.decode("utf-8"))


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def bundled_root():
    return Path(__file__).resolve().parent / "assets" / "runtime"


def load_resources(root=None):
    """Check bytes before any MANO pickle is opened or a compute child starts."""
    from vt_fsglove12.calibration import validate_rest
    from vt_fsglove12.config import validate_config, check_model_hand
    from vt_fsglove12.upstream_core import inspect_upstream_sources
    from vt_capture_console.ui.widgets.tactile_model_loader import load_bundle_file
    from vt_capture_console.backend.tactile_reference_store import read_reference_profile

    root = Path(root) if root is not None else bundled_root()
    root = root.expanduser().resolve(strict=True)
    manifest = read_json(root / "manifest.json")
    if (not isinstance(manifest, dict) or manifest.get("schema_version") != 1
            or manifest.get("product") != "hand-motion-touch"
            or not isinstance(manifest.get("files"), dict)):
        raise ValueError("Invalid Hand Motion & Touch resource manifest")
    required = {"hand.json", "rest.json", "config.json", "mano/models/MANO_RIGHT.pkl", "default_response.json"}
    if not required <= manifest["files"].keys():
        raise ValueError("Incomplete hand-motion resources")
    for name, expected in manifest["files"].items():
        relative = PurePosixPath(name)
        if (relative.is_absolute() or ".." in relative.parts or "\\" in name
                or not isinstance(expected, str) or len(expected) != 64):
            raise ValueError("Invalid resource path or checksum")
        path = (root / name).resolve(strict=True)
        path.relative_to(root)
        if not path.is_file() or sha256(path) != expected:
            raise ValueError(f"Resource checksum mismatch: {name}")
    config = validate_config(read_json(root / "config.json"))
    if config["hand"] != "right":
        raise ValueError("This application requires the right-hand configuration")
    rest = read_json(root / "rest.json")
    validate_rest(rest)
    check_model_hand(config, rest)
    bundle = load_bundle_file(root / "hand.json", "right")
    upstream = inspect_upstream_sources(root / "upstream")
    reference_path = root / "default_response.json"
    read_reference_profile(reference_path)
    config["pose_backend"] = "official_fsglove_sparse_adapter"
    config["upstream_provenance"] = upstream
    return dict(bundle, rest=rest, config=config, root=root,
                mano_assets=root / "mano", upstream_checkout=root / "upstream",
                response_profile=reference_path, manifest=manifest)
