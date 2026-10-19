"""Explicit STM32 input assumptions, distinct from verified device settings.

The published FSGlove core consumes quaternions. This module describes our
12-node raw-input adapter, not the upstream HI229 wire protocol. Nothing here
reads registers, identifies a device, changes firmware, or changes raw counts.
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np


_NODE_KEYS = tuple(str(n) for n in range(1, 13))
_DEFAULTS = {
    "schema_version": 1,
    "sensor_model": "unknown",
    "model_source": "undeclared",
    "transport": "stm32_usb_cdc",
    "encoding": "signed_int16_le",
    "field_order": ["gx", "gy", "gz", "ax", "ay", "az"],
    "frame_bytes": 84,
    "node_block_bytes": 14,
    "nodes_per_frame": 6,
    "second_byte_semantics": "unknown",
    "raw_value_source": "stm32_reported_counts",
    "scale_source": "configured_assumption",
    "host_timestamp_source": "host_read_completion",
    "sample_timing_assumption": "per_node_nominal_period",
    "device_timestamp_available": False,
}
_CHOICES = {
    "sensor_model": {"unknown", "LSM6DSRTR", "LSM6DSR"},
    "model_source": {"undeclared", "user_reported", "firmware_documented", "device_identified"},
    "scale_source": {"configured_assumption", "empirical_validation", "firmware_documented", "register_readback"},
}
# ST LSM6DSR datasheet sensitivities, NOT integer span / configured scale.
# A matching scale alone does not establish the actual FS register setting.
_GYRO_FS = {0.004375: 125, 0.00875: 250, 0.0175: 500, 0.035: 1000, 0.07: 2000, 0.14: 4000}
_ACCEL_FS = {0.000061: 2, 0.000122: 4, 0.000244: 8, 0.000488: 16}
_DATASHEET = "https://www.st.com/resource/en/datasheet/lsm6dsr.pdf"


def _positive(value, name):
    try:
        valid = not isinstance(value, bool) and isinstance(value, Real) and math.isfinite(value) and value > 0
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(f"{name} must be a finite positive number")
    return float(value)


def effective_input_scales(config):
    """Return per-node gyro (degrees/s/LSB) and accel (g/LSB) maps.

    Defaults preserve the existing live/replay contract, including callers
    supplying only a partial replay configuration. The input is not mutated.
    """
    if not isinstance(config, dict):
        raise ValueError("config must be a dictionary")
    result = []
    for field, default in (("gyro_dps_per_lsb", .00875), ("accel_g_per_lsb", .000122)):
        base = _positive(config.get(field, default), field)
        overrides = config.get(field + "_by_node", {})
        if not isinstance(overrides, dict):
            raise ValueError(f"{field}_by_node must be an object")
        scales = dict.fromkeys(_NODE_KEYS, base)
        for key, value in overrides.items():
            if key not in _NODE_KEYS:
                raise ValueError(f"{field}_by_node requires canonical string node IDs 1..12")
            scales[key] = _positive(value, f"{field} for node {key}")
        result.append(scales)
    return tuple(result)


def validate_input_contract(config):
    """Validate optional metadata against the parser actually implemented.

    This validates a declaration, not its truth on attached hardware. Existing
    configurations need no new fields and retain their original JSON/hash.
    """
    if not isinstance(config, dict):
        raise ValueError("config must be a dictionary")
    contract = config.get("input_contract", {})
    if not isinstance(contract, dict):
        raise ValueError("input_contract must be an object")
    for key, value in contract.items():
        if key not in _DEFAULTS:
            raise ValueError(f"unsupported input_contract field: {key}")
        if key in _CHOICES:
            if not isinstance(value, str) or value not in _CHOICES[key]:
                raise ValueError(f"unsupported input_contract {key}")
        elif type(value) is not type(_DEFAULTS[key]) or value != _DEFAULTS[key]:
            raise ValueError(f"input_contract {key} does not match the implemented STM32 parser")
    for flag in ("units_verified", "sample_rate_verified"):
        if flag in config and type(config[flag]) is not bool:
            raise ValueError(f"{flag} must be a boolean declaration")
    if contract.get("scale_source") == "register_readback":
        # A user may describe external verification, but a label cannot silently
        # upgrade the configured scale into a verified on-device register read.
        if config.get("units_verified") is not True:
            raise ValueError("register_readback scale_source requires units_verified=true")
    effective_input_scales(config)
    _positive(config.get("sample_rate_hz", 120.), "sample_rate_hz")


def _nominal_fs(scale, sensitivities):
    return next((full_scale for sensitivity, full_scale in sensitivities.items()
                 if math.isclose(scale, sensitivity, rel_tol=1e-9, abs_tol=0.)), None)


def describe_input_contract(config):
    """Return JSON-safe effective assumptions for UI/session metadata only."""
    validate_input_contract(config)
    # Lazy import avoids a config-validation import cycle.
    from .config import SLOT_NAMES

    contract = {**_DEFAULTS, **config.get("input_contract", {})}
    contract["field_order"] = list(contract["field_order"])
    gyros, accels = effective_input_scales(config)
    mapping = config.get("node_to_slot", {})
    rate = float(config.get("sample_rate_hz", 120.))
    lsm_declared = contract["sensor_model"] in {"LSM6DSRTR", "LSM6DSR"}
    nodes = {}
    for key in _NODE_KEYS:
        slot = mapping.get(key)
        role = SLOT_NAMES[slot] if type(slot) is int and 0 <= slot < len(SLOT_NAMES) else "unmapped_auxiliary"
        nodes[key] = dict(
            node_id=int(key), slot=slot, role=role,
            gyro_dps_per_lsb=gyros[key], accel_g_per_lsb=accels[key],
            nominal_gyro_full_scale_dps=_nominal_fs(gyros[key], _GYRO_FS) if lsm_declared else None,
            nominal_accel_full_scale_g=_nominal_fs(accels[key], _ACCEL_FS) if lsm_declared else None,
            full_scale_basis=("LSM6DSR_datasheet_sensitivity_match_not_register_readback" if lsm_declared
                              else "unavailable_sensor_model_undeclared"),
            physical_full_scale_verified=False,
        )
    contract.update(
        configuration_explicit="input_contract" in config,
        nodes=nodes,
        node_groups=[[1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12]],
        sample_timing=dict(nominal_rate_hz=rate, integration_period_s=1. / rate,
                           host_timestamp_source="host_read_completion",
                           device_timestamp_available=False, nominal_period_assumed=True),
        verification=dict(units_verified=config.get("units_verified", False),
                          sample_rate_verified=config.get("sample_rate_verified", False),
                          claims_source="configuration_not_runtime_measurement",
                          register_readback_performed_by_this_tool=False,
                          physical_full_scale_verified=False),
        datasheet_source=_DATASHEET,
        note="Model is declared, not device-detected. Scales are configured conversions of STM32-reported counts; "
             "matching a datasheet sensitivity does not verify register settings or absence of firmware rescaling. "
             "The second byte has unknown meaning. Host arrival times do not verify device sampling or synchronization.",
    )
    return contract


def raw_to_si(values, *, gyro_dps_per_lsb, accel_g_per_lsb):
    """Shared six-value conversion; no clipping, filtering, reordering, or dt.

    Callers validate wire int16 values at their input boundary. This function
    accepts numeric arrays too, allowing exact live/offline conversion parity.
    """
    raw = np.asarray(values, dtype=float)
    if raw.shape != (6,) or not np.all(np.isfinite(raw)):
        raise ValueError("raw_to_si requires six finite gyro-then-accel values")
    gyro_scale = _positive(gyro_dps_per_lsb, "gyro_dps_per_lsb")
    accel_scale = _positive(accel_g_per_lsb, "accel_g_per_lsb")
    gyro = raw[:3] * (gyro_scale * math.pi / 180.)
    accel = raw[3:] * (accel_scale * 9.80665)
    if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel)):
        raise ValueError("unit conversion overflow")
    return gyro, accel
