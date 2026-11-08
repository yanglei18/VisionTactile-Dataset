"""Tactile scalar coloring over the calibration tool's fixed surface binding.

This is display interpolation, not pressure estimation or pose reconstruction.
The original 400 samples and masks are never changed. Neutral skin is retained
outside the binding; only supported surface areas can receive a missing-data color.
"""

from __future__ import annotations

import numpy as np


def surface_colors(binding, frame, base_colors, levels, heat_stops, missing_rgb=(229, 232, 237), *, fade_base=True):
    """A palette starting above zero fades from each vertex's own material."""
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
    projection = getattr(binding, "projection", None)
    if projection is None:
        # Historical static bundles expose the same fixed support arrays. Use
        # the shared projection implementation, not a second coloring solver.
        from vt_fsglove12.tactile_surface import TactileSurfaceProjection
        projection = TactileSurfaceProjection(binding.addresses, binding.valid, binding.vertex_weights)
    means, valid = projection.vertex_means(values, mask)
    colors[projection.coverage & ~valid, :3] = np.asarray(missing_rgb) / 255.
    fraction = np.clip((means[valid] - lo) / (hi - lo), 0., 1.)
    stops, rgb = zip(*heat_stops)
    rgb = np.asarray(rgb, dtype=float) / 255.
    mapped = np.column_stack([np.interp(fraction, stops, channel) for channel in rgb.T])
    if fade_base and stops[0] > 0.:
        light = fraction < stops[0]
        base = colors[valid, :3][light]
        weight = fraction[light, None] / stops[0]
        mapped[light] = base + weight * (rgb[0] - base)
    colors[valid, :3] = mapped
    return colors


def raw_surface_colors(binding, frame, base_colors, levels, heat_stops):
    """Preserve the workbench's raw ADC palette over the shared projection."""
    return surface_colors(binding, frame, base_colors, levels, heat_stops, fade_base=False)
