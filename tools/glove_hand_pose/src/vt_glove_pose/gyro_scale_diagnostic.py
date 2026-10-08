"""Offline gyro/acceleration-direction consistency, never auto-calibration.

For gravity-dominated motion in a sensor frame, du/dt = -omega x u.
This diagnostic estimates a scalar discrepancy, not firmware register settings
or anatomical angles. Linear acceleration and timing errors remain confounders.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .config import gyro_scale, load_config, write_json_exclusive


def gyro_scale_diagnostic(gyro_dps, acceleration, host_times, *, neutral_gyro_dps,
                          window_samples=61):
    """Compare recorded gyro to acceleration direction using host elapsed time.

    Returned suggestions are diagnostic only. All input samples are retained by
    the caller; no unit configuration or calibration profile is changed here.
    """
    g, a, t, neutral = (np.asarray(v, dtype=float) for v in
                        (gyro_dps, acceleration, host_times, neutral_gyro_dps))
    if (g.ndim != 2 or g.shape[1:] != (3,) or a.shape != g.shape
            or t.shape != (len(g),) or neutral.ndim != 2 or neutral.shape[1:] != (3,)
            or len(neutral) < 2 or len(g) > 30000
            or any(not np.all(np.isfinite(v)) for v in (g, a, t, neutral))):
        raise ValueError("finite matching gyro/accel (N,3), time (N,), neutral (M,3) required")
    if (type(window_samples) is not int or window_samples < 5
            or window_samples % 2 != 1 or window_samples > 501):
        raise ValueError("window_samples must be an odd integer in [5,501]")
    result = dict(kind="gyro_acceleration_direction_scale_diagnostic", status="insufficient_information",
                  suggested_multiplier=None, vector_cosine=None, retained_samples=0,
                  input_samples=len(g), window_samples=window_samples,
                  hardware_units_verified=False, changes_configuration=False,
                  time_basis="host_observed_not_device_time",
                  limitations=["Acceleration direction is not independent angle truth.",
                               "Linear acceleration, sensor filtering and timing may affect the estimate.",
                               "A scale discrepancy does not identify the firmware register setting."])
    if len(g) < max(240, 3 * window_samples):
        result["reason"] = "too_few_motion_samples"
        return result
    dt = np.diff(t)
    if np.any(dt <= 0) or np.max(dt) > max(.1, 5 * float(np.median(dt))):
        result["reason"] = "host_time_not_continuous_and_strictly_increasing"
        return result
    kernel = np.ones(window_samples) / window_samples
    smooth = lambda v: np.column_stack([np.convolve(v[:, i], kernel, "valid") for i in range(3)])
    a_s = smooth(a)
    omega = smooth(np.deg2rad(g - np.mean(neutral, axis=0)))
    norms = np.linalg.norm(a_s, axis=1)
    if np.any(norms < 1e-12):
        result["reason"] = "undefined_acceleration_direction"
        return result
    u = a_s / norms[:, None]
    edge = window_samples // 2
    observed = np.gradient(u, t[edge:len(t)-edge], axis=0)
    predicted = -np.cross(omega, u)
    # No residual-based sample selection. Do not infer scale from a stationary
    # or gravity-parallel gyro signal with vanishing directional information.
    valid = np.linalg.norm(predicted, axis=1) > .05
    result["retained_samples"] = int(valid.sum())
    result["active_fraction"] = float(np.mean(valid))
    if valid.sum() < 120:
        result["reason"] = "insufficient_gravity_direction_motion"
        return result
    x, y = predicted[valid], observed[valid]
    xx, yy, xy = float(np.sum(x*x)), float(np.sum(y*y)), float(np.sum(x*y))
    if yy < 1e-12:
        result["reason"] = "acceleration_direction_does_not_move"
        return result
    gain, cosine = xy/xx, xy/np.sqrt(xx*yy)
    result.update(suggested_multiplier=float(gain), vector_cosine=float(cosine),
                  rms_direction_rate_error_configured=float(np.sqrt(np.mean(np.sum((y-x)**2, axis=1)))),
                  rms_direction_rate_error_fitted=float(np.sqrt(np.mean(np.sum((y-gain*x)**2, axis=1)))))
    if cosine < .95 or not .1 < gain < 10:
        result.update(status="inconclusive", reason="kinematic_direction_model_not_consistent")
    elif not .8 <= gain <= 1.25:
        result.update(status="possible_scale_mismatch", reason="configured_gyro_and_direction_dynamics_disagree")
    else:
        result.update(status="no_large_scale_discrepancy_detected",
                      reason="not_a_firmware_or_accuracy_certification")
    return result


def _read_nodes(path):
    """Bounded read of the existing raw CSV, never a serial device."""
    path = Path(path)
    if not path.is_file() or path.stat().st_size > 64 * 1024 * 1024:
        raise ValueError("raw CSV must be a regular file no larger than 64 MiB")
    columns = ("gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb")
    out = {}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not {"node_id", "host_parse_monotonic_s", *columns} <= set(reader.fieldnames or []):
            raise ValueError("expected pose-tool raw CSV with node IDs and six raw axes")
        for row in reader:
            node = int(row["node_id"])
            if not 1 <= node <= 12:
                raise ValueError("raw node must be 1..12")
            values = tuple(int(row[key]) for key in columns)
            if any(not -32768 <= value <= 32767 for value in values):
                raise ValueError("raw axes must be signed int16 values")
            rows = out.setdefault(node, [])
            if len(rows) >= 30000:
                raise ValueError("at most 30000 samples per node supported")
            rows.append((float(row["host_parse_monotonic_s"]), *values))
    return {node: np.asarray(rows, dtype=float) for node, rows in out.items()}


def diagnose_raw_files(neutral_path, motion_path, config):
    neutral, motion = _read_nodes(neutral_path), _read_nodes(motion_path)
    report = dict(kind="per_node_gyro_scale_diagnostics", changes_configuration=False,
                  neutral_source=str(neutral_path), motion_source=str(motion_path),
                  units_verified=False, nodes={})
    for node in sorted(motion):
        if node not in neutral:
            report["nodes"][str(node)] = dict(status="insufficient_information", reason="no_neutral_for_node")
            continue
        scale = gyro_scale(config, node)
        rows, still = motion[node], neutral[node]
        item = gyro_scale_diagnostic(rows[:, 1:4]*scale, rows[:, 4:7], rows[:, 0],
                                     neutral_gyro_dps=still[:, 1:4]*scale)
        item["configured_gyro_dps_per_lsb"] = scale
        report["nodes"][str(node)] = item
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("neutral", type=Path)
    parser.add_argument("motion", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path, help="New diagnostic JSON; never overwrite")
    args = parser.parse_args(argv)
    try:
        report = diagnose_raw_files(args.neutral, args.motion, load_config(args.config))
        if args.output:
            write_json_exclusive(args.output, report)
        else:
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
