"""Frozen hinge units, including optional per-node gyro sensitivity.

Legacy settings retain their four fields. Comparisons use effective sensitivity
of the selected nodes, not a global fallback or dictionary representation.
"""
from __future__ import annotations

import math
from numbers import Real


REQUIRED_SETTINGS = {"node_ids", "sample_rate_hz", "gyro_dps_per_lsb", "accel_g_per_lsb"}
GYRO_OVERRIDES = "gyro_dps_per_lsb_by_node"


def _number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} is outside its supported range")
    return value


def gyro_overrides(value):
    """Validate all entries, even ones outside the current selected pair."""
    if not isinstance(value, dict):
        raise ValueError("gyro_dps_per_lsb_by_node must be a dictionary")
    result = {}
    for node, scale in value.items():
        if not isinstance(node, str) or node not in {str(n) for n in range(1, 13)}:
            raise ValueError("gyro_dps_per_lsb_by_node keys must be canonical node strings 1..12")
        result[node] = _number(scale, f"gyro_dps_per_lsb_by_node[{node}]", 1e-6, 10.)
    return result


def effective_gyro_scales(value, nodes):
    fallback = _number(value["gyro_dps_per_lsb"], "gyro_dps_per_lsb", 1e-6, 10.)
    overrides = gyro_overrides(value[GYRO_OVERRIDES]) if GYRO_OVERRIDES in value else {}
    return tuple(overrides.get(str(node), fallback) for node in nodes)


def normalize_hinge_settings(value):
    if (not isinstance(value, dict) or not REQUIRED_SETTINGS <= set(value)
            or set(value) - REQUIRED_SETTINGS - {GYRO_OVERRIDES}):
        raise ValueError("hinge settings must contain nodes, rate and units, with optional per-node gyro scales")
    nodes = value["node_ids"]
    if (not isinstance(nodes, list) or len(nodes) != 2
            or any(type(node) is not int or not 1 <= node <= 12 for node in nodes)
            or nodes[0] == nodes[1] or (nodes[0] - 1) // 6 != (nodes[1] - 1) // 6):
        raise ValueError("hinge settings node_ids must be distinct nodes in one wire group")
    result = dict(node_ids=list(nodes))
    for name, low, high in (("sample_rate_hz", 10., 2000.),
                            ("gyro_dps_per_lsb", 1e-6, 10.),
                            ("accel_g_per_lsb", 1e-7, 1.)):
        result[name] = _number(value[name], name, low, high)
    if GYRO_OVERRIDES in value:
        result[GYRO_OVERRIDES] = gyro_overrides(value[GYRO_OVERRIDES])
    return result


def hinge_settings_match(left, right):
    left, right = normalize_hinge_settings(left), normalize_hinge_settings(right)
    if left["node_ids"] != right["node_ids"]:
        return False
    if any(not math.isclose(left[key], right[key], rel_tol=1e-10)
           for key in ("sample_rate_hz", "accel_g_per_lsb")):
        return False
    return all(math.isclose(a, b, rel_tol=1e-10) for a, b in zip(
        effective_gyro_scales(left, left["node_ids"]), effective_gyro_scales(right, right["node_ids"])))


def settings_from_units(value, nodes, *, accel_g_per_lsb=None):
    settings = dict(node_ids=list(nodes), sample_rate_hz=value["sample_rate_hz"],
                    gyro_dps_per_lsb=value["gyro_dps_per_lsb"],
                    accel_g_per_lsb=value["accel_g_per_lsb"] if accel_g_per_lsb is None else accel_g_per_lsb)
    if GYRO_OVERRIDES in value:
        settings[GYRO_OVERRIDES] = value[GYRO_OVERRIDES]
    return normalize_hinge_settings(settings)
