"""Tactile scalar coloring over the calibration tool's fixed surface binding.

This is display interpolation, not pressure estimation or pose reconstruction.
The original 400 samples and masks are never changed. Neutral skin is retained
outside the binding; missing samples are gray only on supported surface areas.
"""

from __future__ import annotations

import numpy as np


def surface_colors(binding, frame, base_colors, levels, heat_stops):
    colors = np.array(base_colors, dtype=np.float32, copy=True)
    if frame is None or levels is None:
        return colors
    values = np.asarray(frame["values"], dtype=float)
    mask = np.asarray(frame["valid_mask"], dtype=bool)
    if values.shape != (400,) or mask.shape != (400,):
        raise ValueError("3D tactile display requires 400 values and masks")
    lo, hi = levels
    if not np.isfinite([lo, hi]).all() or hi <= lo:
        raise ValueError("ADC color levels must be finite and increasing")
    indices = np.asarray([row * 20 + col for row, col in binding.addresses])
    usable = mask[indices] & binding.valid & np.isfinite(values[indices])
    weights = binding.vertex_weights
    # Reuse exactly the calibration surface's local, region-limited supports.
    # No extra nearest-neighbour projection and no coloring across fingers.
    vertex, address = np.nonzero(weights)
    supported = usable[address]
    weights_at = weights[vertex[supported], address[supported]]
    count = len(colors)
    total = np.bincount(vertex[supported], weights=weights_at, minlength=count)
    sums = np.bincount(vertex[supported],
                       weights=weights_at * values[indices[address[supported]]], minlength=count)
    valid = total > 1e-15
    candidate = weights.sum(axis=1) > 1e-15
    colors[candidate & ~valid, :3] = np.array([229, 232, 237]) / 255.
    means = np.divide(sums, total, out=np.zeros(count), where=valid)
    fraction = np.clip((means[valid] - lo) / (hi - lo), 0., 1.)
    stops, rgb = zip(*heat_stops)
    rgb = np.asarray(rgb, dtype=float) / 255.
    for axis in range(3):
        colors[valid, axis] = np.interp(fraction, stops, rgb[:, axis])
    return colors
