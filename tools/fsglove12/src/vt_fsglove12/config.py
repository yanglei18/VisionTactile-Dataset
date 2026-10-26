"""Explicit sensor-to-segment mappings; sensor IDs are NOT MANO joint IDs."""
from __future__ import annotations

import hashlib
import json
import math
from importlib.resources import files
from pathlib import Path

from .input_contract import validate_input_contract
from .input_guard import guard_settings
from .input_health import health_settings
from .vqf_settings import vqf_parameters

SLOT_NAMES = (
    "palm", "index_proximal", "index_middle", "index_distal",
    "middle_proximal", "middle_middle", "middle_distal",
    "little_proximal", "little_middle", "little_distal",
    "ring_proximal", "ring_middle", "ring_distal",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal",
)
PARENTS = (-1, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14)
FINGER_DIPS = {"index": 3, "middle": 6, "little": 9, "ring": 12}
_LIMIT_AXES = {str(slot): ("xyz" if slot == 13 else "yz" if slot in (1, 4, 7, 10, 14) else "z")
               for slot in range(1, 16)}


def _finite_number(value) -> bool:
    try:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        return False


def validate_solver_constraints(config: dict) -> None:
    """Validate optional overrides, also for direct SparseHandSolver callers.

    Bounds apply only to existing optimized DOFs; they do not add twist DOFs,
    constrain a missing CMC prior, or independently clamp a coupled missing DIP.
    A mapped but temporarily missing DIP still uses its configured coupling.
    """
    limits = config.get("joint_limits_rad", {})
    if not isinstance(limits, dict):
        raise ValueError("joint_limits_rad must be an object of slot/axis bounds in radians")
    mapped = set(config["node_to_slot"].values())
    conditional_slots = {*FINGER_DIPS.values(), 13}
    for slot, axes in limits.items():
        if slot not in _LIMIT_AXES or not isinstance(axes, dict) or not axes:
            raise ValueError("joint_limits_rad requires supported string slots 1..15 and nonempty axis objects")
        if int(slot) in conditional_slots and int(slot) not in mapped:
            raise ValueError(f"joint_limits_rad slot {slot} has no mapped sensor; its prior/coupling is not an optimized limit")
        for axis, bounds in axes.items():
            if not isinstance(axis, str) or len(axis) != 1 or axis not in _LIMIT_AXES[slot]:
                raise ValueError(f"joint_limits_rad slot {slot} does not support axis {axis!r}")
            if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                    or not all(_finite_number(v) for v in bounds) or bounds[0] >= bounds[1]):
                raise ValueError(f"joint_limits_rad slot {slot} axis {axis} requires two finite numbers with lower < upper")
    ratios = config.get("dip_ratio_by_finger", {})
    if not isinstance(ratios, dict):
        raise ValueError("dip_ratio_by_finger must be an object")
    for finger, ratio in ratios.items():
        if finger not in FINGER_DIPS or not _finite_number(ratio) or not 0 <= ratio <= 1:
            raise ValueError("dip_ratio_by_finger requires index/middle/ring/little ratios finite in [0, 1]")


def digest(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def file_digest(path) -> str:
    hasher = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def validate_config(config: dict) -> dict:
    if not isinstance(config, dict) or type(config.get("schema_version")) is not int or config["schema_version"] != 1:
        raise ValueError("configuration requires schema_version=1")
    if config.get("hand") not in ("right", "left"):
        raise ValueError("configuration hand must be right or left")
    mapping = config.get("node_to_slot")
    if not isinstance(mapping, dict) or not mapping:
        raise ValueError("node_to_slot must explicitly map sensor IDs to MANO slots")
    for node, slot in mapping.items():
        if node not in {str(i) for i in range(1, 13)} or type(slot) is not int or not 0 <= slot < 16:
            raise ValueError("node_to_slot requires string IDs 1..12 and integer slots 0..15")
    if len(set(mapping.values())) != len(mapping) or 0 not in mapping.values():
        raise ValueError("each mapped bone must be unique and a palm (slot 0) is required")
    for key, low, high in (("sample_rate_hz", 10, 2000), ("output_rate_hz", 1, 240),
                          ("gyro_dps_per_lsb", 1e-6, 10), ("accel_g_per_lsb", 1e-7, 1),
                          ("stale_after_s", .01, 5), ("dip_ratio", 0, 1)):
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be finite in [{low}, {high}]")
    if config["output_rate_hz"] > config["sample_rate_hz"]:
        raise ValueError("output_rate_hz cannot exceed the configured sample rate")
    reset_gap = config.get("filter_reset_gap_s", max(1.0, config["stale_after_s"]))
    if not _finite_number(reset_gap) or reset_gap < config["stale_after_s"]:
        raise ValueError("filter_reset_gap_s must be finite and >= stale_after_s")
    overrides = config.get("gyro_dps_per_lsb_by_node", {})
    if not isinstance(overrides, dict):
        raise ValueError("gyro_dps_per_lsb_by_node must be an object")
    for node, scale in overrides.items():
        if node not in {str(i) for i in range(1, 13)} or isinstance(scale, bool) or not isinstance(scale, (int, float)) or not math.isfinite(scale) or not 1e-6 <= scale <= 10:
            raise ValueError("invalid per-node gyroscope scale")
    accel_overrides = config.get("accel_g_per_lsb_by_node", {})
    if not isinstance(accel_overrides, dict):
        raise ValueError("accel_g_per_lsb_by_node must be an object")
    for node, scale in accel_overrides.items():
        if node not in {str(i) for i in range(1, 13)} or not _finite_number(scale) or not 1e-7 <= scale <= 1:
            raise ValueError("invalid per-node accelerometer scale")
    validate_input_contract(config)
    vqf_parameters(config)
    effective_guard = guard_settings(config)
    effective_health = health_settings(config)
    validate_solver_constraints(config)
    # A JSON round trip creates an independent, finite configuration.
    result = json.loads(json.dumps(config, allow_nan=False))
    # Persist the effective policy in every new session/profile binding. A
    # missing field must not silently mean 'reset at the display timeout'.
    result.setdefault("filter_reset_gap_s", float(reset_gap))
    if "input_filter" in config:
        result["input_filter"] = effective_guard
    if "input_health" in config:
        result["input_health"] = effective_health
    return result


def load_config(path: str | Path | None = None, *, hand: str | None = None) -> dict:
    """An explicit config owns its hand; an explicit conflicting flag is an error."""
    if hand is not None and hand not in ("right", "left"):
        raise ValueError("hand must be right or left")
    source = Path(path) if path else files(__package__).joinpath(f"configs/{hand or 'right'}_current.json")
    config = validate_config(json.loads(source.read_text(encoding="utf-8")))
    if hand is not None and config["hand"] != hand:
        raise ValueError(f"requested hand={hand} conflicts with configuration hand={config['hand']}")
    return config


def check_model_hand(config: dict, rest: dict) -> None:
    """Reject a known model/config side mismatch without relabelling old fixtures."""
    model_hand = rest.get("provenance", {}).get("hand")
    if model_hand is not None and model_hand != config["hand"]:
        raise ValueError(f"MANO model hand={model_hand} does not match configuration hand={config['hand']}")


def layout_report(config: dict) -> dict:
    config = validate_config(config)
    inverse = {slot: int(node) for node, slot in config["node_to_slot"].items()}
    return {
        "hand": config["hand"],
        "sensors_mapped": len(inverse),
        "sensors_unmapped": sorted(set(range(1, 13)) - set(inverse.values())),
        "segments": [{"slot": slot, "name": name, "sensor": inverse.get(slot),
                      "source": "configured_sensor" if slot in inverse else "requires_prior"}
                     for slot, name in enumerate(SLOT_NAMES)],
        "mounting_confirmed": bool(config.get("mounting_confirmed", False)),
        "note": "Configured mapping is not evidence of physical mounting or valid measurements.",
    }


def check_proposed_mount(config: dict) -> None:
    if config.get("layout") == "proposed_1_palm_3_thumb_8_other_finger" and config.get("mounting_confirmed") is not True:
        raise ValueError("Proposed thumb-3 layout is not physically confirmed: node 11 must actually be fixed to the first metacarpal before calibration/solve")


def write_json(path, payload) -> None:
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
