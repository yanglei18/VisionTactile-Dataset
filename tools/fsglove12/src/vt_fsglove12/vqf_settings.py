"""Explicit upstream VQF parameters shared by live input and raw replay.

Absent settings preserve the upstream defaults. ``biasClip`` is expressed in
degrees per second: it bounds both bias estimation and the absolute angular
rate accepted by VQF's rest detector. It does not rescale raw measurements.
"""
from __future__ import annotations

import math


def vqf_parameters(config):
    """Return validated constructor overrides without importing VQF."""
    if not isinstance(config, dict):
        raise ValueError("config must be a dictionary")
    value = config.get("vqf", {})
    if not isinstance(value, dict) or set(value) - {"biasClip"}:
        raise ValueError("vqf must be an object containing only biasClip")
    if "biasClip" in value:
        clip = value["biasClip"]
        try:
            valid = (not isinstance(clip, bool) and isinstance(clip, (int, float))
                     and math.isfinite(clip) and clip > 0)
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError("vqf.biasClip must be finite and positive in degrees per second")
    return dict(value)
