"""Relocatable, inexpensive bag/index binding (not a full bag-content hash)."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import warnings

import yaml


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def bag_identity(path: str | Path) -> dict[str, object]:
    """Hash metadata and inventory declared storage files; never hash multi-GB data.

    File names are bag-relative, so moving/copying the entire bag is supported.
    Metadata-only old/synthetic bags remain representable, explicitly weaker.
    """
    path = Path(path).resolve()
    metadata = path / "metadata.yaml"
    try:
        document = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError("bag metadata YAML is malformed") from error
    info = document.get("rosbag2_bagfile_information", {}) if isinstance(document, dict) else {}
    if not isinstance(info, dict):
        raise ValueError("bag metadata rosbag2_bagfile_information must be a mapping")
    names = info.get("relative_file_paths")
    inventory = None
    if names is not None:
        if not isinstance(names, list) or any(not isinstance(name, str) or not name for name in names):
            raise ValueError("bag metadata relative_file_paths must be a list of non-empty names")
        inventory = []
        for name in sorted(names):
            relative = Path(name)
            resolved = (path / relative).resolve()
            if relative.is_absolute() or ".." in relative.parts or not resolved.is_relative_to(path):
                raise ValueError("bag storage inventory contains a path outside the bag")
            if not resolved.is_file():
                raise FileNotFoundError(f"bag storage file is missing: {relative}")
            inventory.append({"relative_path": relative.as_posix(), "size_bytes": resolved.stat().st_size})
        if len({entry["relative_path"] for entry in inventory}) != len(inventory):
            raise ValueError("bag storage inventory contains duplicate paths")
    result: dict[str, object] = {
        "metadata_sha256": sha256_file(metadata),
        "identity_scope": "metadata_sha256_and_storage_file_names_sizes" if inventory else "metadata_sha256_only",
        "storage_content_hashed": False,
    }
    if inventory is not None:
        result["storage_files"] = inventory
    return result


def verify_alignment_source(
    bag_path: str | Path, aligned_frames: str | Path, *, require_provenance: bool = False,
) -> dict[str, object]:
    """Reject mismatches before export; warn explicitly for legacy unbound indices."""
    bag, frames = Path(bag_path).resolve(), Path(aligned_frames).resolve()
    if not bag.is_dir():
        raise FileNotFoundError(f"rosbag2 directory does not exist: {bag}")
    index = {"size_bytes": frames.stat().st_size, "sha256": sha256_file(frames)}
    source = bag_identity(bag) if (bag / "metadata.yaml").is_file() else None
    manifest_path = frames.parent / "manifest.json"
    limitation = "Storage contents are not hashed; same-size data replacement is not detected."
    if not manifest_path.is_file():
        if require_provenance:
            raise ValueError("alignment manifest is missing; cannot verify bag/index provenance")
        warning = "Alignment manifest is missing: bag/index provenance is UNVERIFIED."
        warnings.warn(warning, UserWarning, stacklevel=2)
        return {"status": "unverified_missing_manifest", "source_bag_identity": source,
                "aligned_frames": index, "warnings": [warning], "limitations": [limitation]}
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest["schema_version"] != 1 or manifest["tool"] != "vt-multisensor-alignment":
            raise ValueError("unsupported alignment manifest")
        expected_index = manifest["files"]["aligned_frames.jsonl"]
        expected_source = manifest["source_bag"]
        metadata_hash = expected_source["metadata_sha256"]
        if (not isinstance(metadata_hash, str) or len(metadata_hash) != 64
                or type(expected_index["size_bytes"]) is not int
                or not isinstance(expected_index["sha256"], str)):
            raise ValueError("malformed alignment provenance")
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError("alignment manifest provenance is malformed") from error
    if index != {key: expected_index[key] for key in ("size_bytes", "sha256")}:
        raise ValueError("aligned_frames integrity mismatch: size/SHA256 differs from alignment manifest")
    if source is None or source["metadata_sha256"] != metadata_hash:
        raise ValueError("source bag provenance mismatch: metadata SHA256 differs from alignment manifest")
    expected_inventory = expected_source.get("storage_files")
    if expected_inventory is not None and expected_inventory != source.get("storage_files"):
        raise ValueError("source bag provenance mismatch: storage file inventory differs from alignment manifest")
    verified_inventory = bool(expected_inventory)
    warning_list = [] if verified_inventory else ["Legacy alignment binds metadata only; storage file sizes were not bound."]
    for warning in warning_list:
        warnings.warn(warning, UserWarning, stacklevel=2)
    return {
        "status": "verified_metadata_and_storage_inventory" if verified_inventory else "verified_metadata_only",
        "source_bag_identity": source, "aligned_frames": index,
        "alignment_manifest_sha256": sha256_file(manifest_path),
        "alignment_tool_version": manifest.get("tool_version"),
        "warnings": warning_list, "limitations": [limitation],
    }
