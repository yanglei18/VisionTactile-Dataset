"""Versioned, validated settings without device-specific serial identifiers."""
from __future__ import annotations

import hashlib
import json
import math
from importlib.resources import files
from pathlib import Path

FINGERS = ("thumb", "index", "middle", "ring", "little")


def _number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be finite in [{low}, {high}]")


def validate_config(config: dict) -> dict:
    if not isinstance(config, dict) or type(config.get("schema_version")) is not int or config.get("schema_version") != 1:
        raise ValueError("Expected configuration schema_version=1")
    if config.get("hand") not in ("left", "right"):
        raise ValueError("hand must be left or right")
    for key, low, high in (("sample_rate_hz", 10, 2000), ("gyro_dps_per_lsb", 1e-6, 10),
                          ("accel_g_per_lsb", 1e-7, 1), ("stale_after_s", 0.05, 5),
                          ("dip_ratio", 0, 1)):
        _number(config.get(key), key, low, high)
    overrides = config.get("gyro_dps_per_lsb_by_node", {})
    if not isinstance(overrides, dict):
        raise ValueError("gyro_dps_per_lsb_by_node must be an object keyed by node IDs 1..12")
    for node, scale in overrides.items():
        if type(node) is not str or node not in {str(n) for n in range(1, 13)}:
            raise ValueError("gyro_dps_per_lsb_by_node keys must be canonical strings 1..12")
        _number(scale, f"gyro_dps_per_lsb_by_node.{node}", 1e-6, 10)
    for key in ("units_verified", "sample_rate_verified", "palm_mount_verified",
                "hand_side_confirmed", "estimate_dip"):
        if not isinstance(config.get(key), bool):
            raise ValueError(f"{key} must be boolean")
    fingers = config.get("fingers", {})
    if not isinstance(fingers, dict) or set(fingers) != set(FINGERS):
        raise ValueError("All five finger definitions are required")
    nodes = [config.get("palm_node"), config.get("auxiliary_node")]
    for name, finger in fingers.items():
        if not isinstance(finger, dict):
            raise ValueError(f"Invalid finger definition: {name}")
        nodes += [finger.get("proximal"), finger.get("distal")]
        lengths = finger.get("lengths_mm", [])
        if not isinstance(lengths, list) or len(lengths) != (2 if name == "thumb" else 3):
            raise ValueError(f"Wrong number of bone lengths for {name}")
        for length in lengths:
            _number(length, f"{name}.lengths_mm", 5, 120)
        base = finger.get("base_mm", [])
        if not isinstance(base, list) or len(base) != 3:
            raise ValueError(f"{name}.base_mm needs three coordinates")
        for coordinate in base:
            _number(coordinate, f"{name}.base_mm", -200, 200)
        _number(finger.get("neutral_splay_deg"), f"{name}.neutral_splay_deg", -90, 90)
        if not isinstance(finger.get("anatomy_verified"), bool):
            raise ValueError(f"{name}.anatomy_verified must be boolean")
    if any(type(node) is not int for node in nodes) or sorted(nodes) != list(range(1, 13)):
        raise ValueError("Finger, palm and auxiliary IDs must cover 1..12 exactly once")
    # Return an independent JSON-compatible copy; reject nonfinite extra fields as well.
    return json.loads(json.dumps(config, allow_nan=False))


def load_config(path: str | Path | None = None) -> dict:
    text = Path(path).read_text(encoding="utf-8") if path else files(__package__).joinpath("default_config.json").read_text(encoding="utf-8")
    return validate_config(json.loads(text))


def gyro_scale(config: dict, node_id: int) -> float:
    """Effective configured sensitivity, not an assertion of firmware range.

    Callers validate their configuration at the acquisition/import boundary;
    this lookup is deliberately cheap enough for the per-sample path.
    """
    if type(node_id) is not int or not 1 <= node_id <= 12:
        raise ValueError("node_id must be an integer in 1..12")
    return float(config.get("gyro_dps_per_lsb_by_node", {}).get(
        str(node_id), config["gyro_dps_per_lsb"]))


def pair_gyro_scales(config: dict, proximal: int, distal: int) -> tuple[float, float]:
    """Return effective sensitivities in the caller's proximal/distal order."""
    return gyro_scale(config, proximal), gyro_scale(config, distal)


def fingerprint(config: dict) -> str:
    relevant = validate_config(config)
    # Operator confirmations and prose can change without changing mounting math.
    for key in ("notes", "hand_side_confirmed", "palm_mount_verified", "units_verified", "sample_rate_verified"):
        relevant.pop(key, None)
    # No override, an empty map, and explicit copies of the fallback describe
    # the same conversion. Preserve pre-override calibration hashes exactly.
    overrides = {node: float(scale) for node, scale in
                 relevant.pop("gyro_dps_per_lsb_by_node", {}).items()
                 if scale != relevant["gyro_dps_per_lsb"]}
    if overrides:
        relevant["gyro_dps_per_lsb_by_node"] = overrides
    return hashlib.sha256(json.dumps(relevant, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def write_json_exclusive(path: str | Path, data: dict) -> None:
    """Never silently overwrite calibration or experiment evidence."""
    text = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    with Path(path).open("x", encoding="utf-8") as stream:
        stream.write(text)
