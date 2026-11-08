"""Dependency-free reference envelope shared by runtime and installer preflight.

These identifiers describe the 44-byte row protocol and the declared hand
layout. A reference from an earlier protocol must never be relabelled in place.
Numerical reference validation belongs to the acquisition display processor.
"""
from __future__ import annotations


PROTOCOL = "vt_tactile_row44_v1"
LAYOUT = "right_adc20x20_row44_v1"
MAX_PROFILE_BYTES = 256 * 1024


def layout_for_hand(hand):
    if hand not in ("right", "left"):
        raise ValueError("hand must be right or left")
    return f"{hand}_adc20x20_row44_v1"


def validate_envelope(value, *, expected_hand=None):
    """Validate the current device/layout scopes without importing GUI/numerics."""
    if expected_hand is not None:
        layout_for_hand(expected_hand)
    common = {"schema_version", "protocol", "hand", "layout", "reference",
              "settings", "captured_at_utc"}
    if (not isinstance(value, dict) or type(value.get("schema_version")) is not int
            or value.get("protocol") != PROTOCOL or value.get("hand") not in ("right", "left")
            or value.get("layout") != layout_for_hand(value["hand"])):
        raise ValueError("Unsupported tactile reference protocol or layout")
    if expected_hand is not None and value["hand"] != expected_hand:
        raise ValueError(f"Tactile reference hand does not match expected hand {expected_hand}")
    version = value["schema_version"]
    if version == 1:
        if (set(value) != common | {"device"} or not isinstance(value["device"], dict)
                or set(value["device"]) != {"vendor_id", "product_id", "serial_number"}):
            raise ValueError("Invalid tactile device reference envelope")
    elif version == 2:
        if set(value) != common | {"scope"} or value["scope"] != "layout":
            raise ValueError("Invalid tactile layout reference envelope")
    else:
        raise ValueError("Unsupported tactile reference schema")
    return version
