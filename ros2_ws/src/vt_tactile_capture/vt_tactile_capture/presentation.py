"""Read-only display helpers for the candidate tactile hand layout.

The physical area is user-confirmed 20x20; anatomical boundaries remain inferred
display regions. Neither color deadband nor activity threshold
modifies, clips, filters, or replaces any ADC measurement.
"""

from __future__ import annotations

import numpy as np

from .grid import SHAPE


NEUTRAL = np.array((155, 174, 185, 255), dtype=np.float64)
NEGATIVE = np.array((49, 121, 192, 255), dtype=np.float64)
POSITIVE_LOW = np.array((253, 215, 92, 255), dtype=np.float64)
POSITIVE_HIGH = np.array((240, 92, 52, 255), dtype=np.float64)

# Every address in [0:20,0:20] occurs once; finger boundaries are candidates.
_REGIONS = (
    ("thumb", "拇指", (8, 20), (16, 20)),
    ("index", "食指", (8, 20), (12, 16)),
    ("middle", "中指", (8, 20), (8, 12)),
    ("ring", "无名指", (8, 20), (4, 8)),
    ("little", "小指", (8, 20), (0, 4)),
    ("palm_center", "掌心", (3, 8), (0, 20)),
    ("palm_root", "掌根", (0, 3), (0, 20)),
)


def _nonnegative_finite(value, name):
    if isinstance(value, (bool, np.bool_)) or not np.isscalar(value):
        raise ValueError(f"{name} must be a finite nonnegative number")
    try:
        value = float(value)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite nonnegative number") from exc
    if not np.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be a finite nonnegative number")
    return value


def delta_color_stops(levels: tuple[float, float], deadband: float = 200):
    """Return strictly increasing [0,1] positions and uint8 RGBA color stops.

    [-deadband,+deadband] is neutral gray-blue; negative deltas tend toward blue,
    positive deltas progress through yellow to orange-red.  The color deadband
    is just a visual convention. Ranges not containing zero are cropped from
    the same signed interpretation rather than recentered at their midpoint.
    """
    try:
        bounds = np.asarray(levels, dtype=np.float64)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError("levels must be two finite increasing numbers") from exc
    if bounds.shape != (2,) or not np.all(np.isfinite(bounds)) or bounds[0] >= bounds[1]:
        raise ValueError("levels must be two finite increasing numbers")
    lo, hi = float(bounds[0]), float(bounds[1])
    with np.errstate(over="ignore", invalid="ignore"):
        span = np.float64(hi) - np.float64(lo)
    if not np.isfinite(span):
        raise ValueError("levels span must be finite")
    deadband = _nonnegative_finite(deadband, "deadband")
    # A positive stop at 30% of the positive dynamic range provides yellow for
    # lower contact responses and orange-red at the displayed positive maximum.
    yellow_at = deadband + (hi - deadband) * .30 if hi > deadband else None
    candidates = [lo, hi]
    for value in (-deadband, deadband, yellow_at):
        if value is not None and lo < value < hi:
            candidates.append(value)
    points = sorted(set(candidates))
    positions = np.asarray([(value - lo) / span for value in points], dtype=np.float64)
    colors = []
    for value in points:
        if value < -deadband:
            fraction = (value + deadband) / (lo + deadband)
            color = NEUTRAL + fraction * (NEGATIVE - NEUTRAL)
        elif value > deadband:
            fraction = (value - deadband) / (hi - deadband)
            if fraction <= .30:
                color = NEUTRAL + (fraction / .30) * (POSITIVE_LOW - NEUTRAL)
            else:
                color = POSITIVE_LOW + ((fraction - .30) / .70) * (POSITIVE_HIGH - POSITIVE_LOW)
        else:
            color = NEUTRAL
        colors.append(np.rint(color).astype(np.uint8))
    rgba = np.asarray(colors, dtype=np.uint8)
    # Extremely asymmetric finite ranges can round neighboring normalized
    # points to the same float. Keep the first at zero and last at one; remove
    # unrepresentable interior distinctions rather than emit invalid LUT stops.
    keep = np.r_[True, np.diff(positions) > 0]
    if not keep[-1]:
        earlier_endpoint = np.flatnonzero(positions == 1.0)[0]
        keep[earlier_endpoint] = False
        keep[-1] = True
    return positions[keep], rgba[keep]


def region_summaries(values, valid_mask, threshold: float = 200, *, layout=None) -> list[dict]:
    """Summarize candidate regions without zero-filling or modifying inputs.

    Activity means abs(delta) > threshold (strictly), for display only. A peak
    retains the sign of the maximum absolute valid finite response. Invalid
    cells and NaN/Inf are not considered measurements; no valid cells yields
    None for every peak, not zero contact.
    """
    threshold = _nonnegative_finite(threshold, "threshold")
    try:
        values = np.asarray(values, dtype=np.float64)
        valid_mask = np.asarray(valid_mask)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError("values and valid_mask must be 20x20 arrays") from exc
    if values.shape != SHAPE or valid_mask.shape != SHAPE or valid_mask.dtype.kind != "b":
        raise ValueError("values must be 20x20 and valid_mask must be a 20x20 boolean array")
    selections = []
    if layout is None:
        for key, name, rows, cols in _REGIONS:
            selections.append((key, name, np.s_[rows[0]:rows[1], cols[0]:cols[1]]))
    else:
        names = dict(thumb="拇指", index="食指", middle="中指", ring="无名指", little="小指", palm="掌部")
        for key, name in names.items():
            mask = np.zeros(SHAPE, dtype=bool)
            for point in layout.points:
                if point.region == key:
                    mask[point.row, point.col] = True
            selections.append((key, name, mask))
    summaries = []
    for key, name, selection in selections:
        region = values[selection]
        mask = valid_mask[selection]
        finite = region[mask & np.isfinite(region)]
        positive = finite[finite > 0]
        negative = finite[finite < 0]
        summaries.append({
            "key": key,
            "name": name,
            "peak_delta": float(finite[np.argmax(np.abs(finite))]) if finite.size else None,
            "positive_peak": float(np.max(positive)) if positive.size else None,
            "negative_peak": float(np.min(negative)) if negative.size else None,
            "active_cells": int(np.count_nonzero(np.abs(finite) > threshold)),
            "valid_cells": int(finite.size),
            "candidate_cells": int(region.size),
            "boundary_status": "candidate_inferred",
        })
    return summaries
