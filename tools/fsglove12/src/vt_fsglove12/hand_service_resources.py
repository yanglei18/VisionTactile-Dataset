"""Resolve a hand service's explicit resources without importing any Qt code."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath

MAX_JSON_BYTES = 2 * 1024 * 1024


def _read(path):
    with Path(path).open("rb") as stream:
        content = stream.read(MAX_JSON_BYTES + 1)
    if len(content) > MAX_JSON_BYTES:
        raise ValueError("Resource JSON exceeds its size limit")
    return json.loads(content.decode("utf-8"))


def validate_bundle_manifest(root, *, hand):
    """Return (resolved root, verified manifest) for the shared hand bundle."""
    if hand not in ("left", "right"):
        raise ValueError("Invalid resource hand")
    root = Path(root).expanduser().resolve(strict=True)
    manifest = _read(root / "manifest.json")
    if (not isinstance(manifest, dict) or manifest.get("schema_version") != 1
            or manifest.get("product") != "hand-motion-touch"
            or not isinstance(manifest.get("files"), dict)):
        raise ValueError("Invalid Hand Motion & Touch resource manifest")
    model = f"mano/models/MANO_{hand.upper()}.pkl"
    required = {"config.json", "rest.json", "hand.json", model}
    if hand == "left":
        if "tactile_layout.json" not in manifest["files"]:
            raise ValueError("Left-hand resources require an explicit tactile layout")
        required.add("tactile_layout.json")
    if not required <= manifest["files"].keys():
        raise ValueError("Incomplete hand resource bundle")
    other = "LEFT" if hand == "right" else "RIGHT"
    if f"mano/models/MANO_{other}.pkl" in manifest["files"]:
        raise ValueError("Resource bundle includes the opposite hand model")
    for name, expected in manifest["files"].items():
        relative = PurePosixPath(name)
        if (relative.is_absolute() or ".." in relative.parts or "\\" in name
                or not isinstance(expected, str) or len(expected) != 64):
            raise ValueError("Invalid resource path or checksum")
        path = (root / name).resolve(strict=True)
        path.relative_to(root)
        checksum = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                checksum.update(block)
        if checksum.hexdigest() != expected:
            raise ValueError(f"Resource checksum mismatch: {name}")
    return root, manifest


def load_service_resources(*, hand, resources=None, config=None, rest=None,
                           mano_assets=None, fsglove_source=None, output_rate_hz=None):
    from .calibration import validate_rest
    from .config import check_model_hand, load_config, validate_config
    from .upstream_core import inspect_upstream_sources

    if resources is not None:
        if any(value is not None for value in (config, rest, mano_assets, fsglove_source)):
            raise ValueError("Use either a resource bundle or explicit model/configuration paths")
        root, _ = validate_bundle_manifest(resources, hand=hand)
        config, rest = root / "config.json", root / "rest.json"
        mano_assets, fsglove_source = root / "mano", root / "upstream"
        axes = _read(root / "hand.json")["model_axes"]
    else:
        if any(value is None for value in (config, rest, mano_assets, fsglove_source)):
            raise ValueError("Explicit service resources require config, rest, mano-assets and fsglove-source")
        # Geometry and model axes are computed in the spawned process. The
        # controller's optional axes value is not part of the numerical solver.
        axes = None
    configuration = load_config(config, hand=hand)
    if output_rate_hz is not None:
        configuration["output_rate_hz"] = output_rate_hz
        configuration = validate_config(configuration)
    reference = _read(rest)
    validate_rest(reference)
    check_model_hand(configuration, reference)
    source = inspect_upstream_sources(fsglove_source)
    configuration["pose_backend"] = "official_fsglove_sparse_adapter"
    configuration["upstream_provenance"] = source
    return dict(config=configuration, rest=reference, model_axes=axes,
                mano_assets=Path(mano_assets).resolve(), upstream_checkout=Path(fsglove_source).resolve(), hand=hand)
