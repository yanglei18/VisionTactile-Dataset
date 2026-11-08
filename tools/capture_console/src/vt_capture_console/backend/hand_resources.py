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


def load_resources(root, *, hand="right"):
    """Check bytes before any MANO pickle is opened or a compute child starts."""
    from vt_fsglove12.calibration import validate_rest
    from vt_fsglove12.config import validate_config, check_model_hand
    from vt_fsglove12.upstream_core import inspect_upstream_sources
    from vt_capture_console.ui.widgets.tactile_model_loader import load_bundle_file
    from vt_capture_console.backend.tactile_reference_store import read_reference_profile

    from vt_tactile_capture.hand_layout import load_layout

    if hand not in ("right", "left"):
        raise ValueError("Unknown hand for single-hand runtime")
    from vt_fsglove12.hand_service_resources import validate_bundle_manifest
    root, manifest = validate_bundle_manifest(root, hand=hand)
    config = validate_config(read_json(root / "config.json"))
    if config["hand"] != hand:
        raise ValueError(f"This application requires the {hand}-hand configuration")
    layout = root / "tactile_layout.json" if "tactile_layout.json" in manifest["files"] else None
    if layout is not None:
        read_json(layout)
        load_layout(layout, expected_hand=hand)
    rest = read_json(root / "rest.json")
    validate_rest(rest)
    check_model_hand(config, rest)
    if hand == "left" and rest.get("provenance", {}).get("hand") != hand:
        raise ValueError("Left-hand rest model must explicitly identify its hand")
    bundle = load_bundle_file(root / "hand.json", hand)
    upstream = inspect_upstream_sources(root / "upstream")
    reference_path = None
    if "default_response.json" in manifest["files"]:
        reference_path = root / "default_response.json"
        read_reference_profile(reference_path, expected_hand=hand)
    config["pose_backend"] = "official_fsglove_sparse_adapter"
    config["upstream_provenance"] = upstream
    return dict(bundle, rest=rest, config=config, root=root, hand=hand, layout_path=layout,
                mano_assets=root / "mano", upstream_checkout=root / "upstream",
                response_profile=reference_path, manifest=manifest)
