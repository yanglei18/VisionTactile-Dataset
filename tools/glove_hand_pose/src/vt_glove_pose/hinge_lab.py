"""Offline experiments on existing paired IMU captures; never a live calibration.

The archive adapter also accepts the four historical thumb-A captures. Those
are *common-motion* experiments, not a labelled free-motion hinge benchmark.
Their replay must not be reported as a measurement of anatomical accuracy.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import re

import numpy as np


def _scalar(value, name, low, high):
    array = np.asarray(value)
    if array.shape != () or array.dtype.kind not in "iuf":
        raise ValueError(f"{name} must be a numeric scalar")
    number = float(array)
    if not math.isfinite(number) or not low <= number <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return number


def _vectors(value, name):
    array = np.array(value, dtype=float, copy=True)
    if array.ndim != 2 or array.shape[1] != 3 or not 1 <= len(array) <= 30000:
        raise ValueError(f"{name} must be (N, 3), with 1 <= N <= 30000")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains nonfinite values")
    return array


def load_pair_archive(path, *, prefix="", proximal=5, distal=6, max_pairs=10000):
    """Read an explicit capture from a small NPZ, without executing pickle data.

    Required fields: <prefix>prox_gyro_dps, <prefix>dist_gyro_dps,
    sample_rate_hz, node_ids. Raw int16 arrays are optional but their absence
    means saturation cannot be independently assessed. Historical trim masks
    are intentionally NOT applied: the complete selected capture is reported.
    """
    path = Path(path)
    if not isinstance(prefix, str) or not re.fullmatch(r"[A-Za-z0-9_]*", prefix):
        raise ValueError("archive prefix must contain only letters, digits or underscores")
    if type(max_pairs) is not int or not 1 <= max_pairs <= 30000:
        raise ValueError("max_pairs must be an integer in [1, 30000]")
    size = path.stat().st_size
    if size > 32 * 1024 * 1024:
        raise ValueError("pair archive exceeds the 32 MiB experiment limit")
    content = path.read_bytes()
    # Read exactly the bytes whose digest is reported, including for mutable paths.
    import io
    import zipfile
    try:
        with zipfile.ZipFile(io.BytesIO(content)) as archive:
            if sum(info.file_size for info in archive.infolist()) > 64 * 1024 * 1024:
                raise ValueError("uncompressed pair archive exceeds 64 MiB")
    except zipfile.BadZipFile as exc:
        raise ValueError("pair archive is not a valid NPZ/ZIP file") from exc
    with np.load(io.BytesIO(content), allow_pickle=False) as archive:
        required = {prefix + "prox_gyro_dps", prefix + "dist_gyro_dps", "sample_rate_hz", "node_ids"}
        if not required.issubset(archive.files):
            raise ValueError("archive is missing fields; select a capture with --archive-prefix (e.g. a1_)")
        nodes = np.asarray(archive["node_ids"])
        if nodes.shape != (2,) or nodes.dtype.kind not in "iu" or nodes.tolist() != [proximal, distal]:
            raise ValueError("archive node_ids do not match the requested proximal/distal pair")
        rate = _scalar(archive["sample_rate_hz"], "sample_rate_hz", 10, 2000)
        gp = _vectors(archive[prefix + "prox_gyro_dps"], "prox_gyro_dps")
        gd = _vectors(archive[prefix + "dist_gyro_dps"], "dist_gyro_dps")
        if len(gp) != len(gd):
            raise ValueError("archive gyro arrays must have equal length")
        count = min(len(gp), max_pairs)
        masks = []
        for label in ("prox", "dist"):
            key = prefix + "raw_" + label
            if key not in archive.files:
                masks = None
                break
            raw = np.asarray(archive[key])
            if (raw.shape != (len(gp), 6) or raw.dtype.kind not in "iu"
                    or np.any(raw < -32768) or np.any(raw > 32767)):
                raise ValueError(f"{key} must contain paired signed-int16-range (N, 6) data")
            masks.append(np.any(np.abs(raw[:count, :3].astype(np.int64)) >= 32700, axis=1))
    metadata = {
        "source": str(path), "format": "paired_npz", "archive_prefix": prefix,
        "source_sha256": hashlib.sha256(content).hexdigest(), "consumed_bytes": len(content),
        "source_pairs": len(gp), "selected_pairs": count, "node_ids": [proximal, distal],
        "sample_rate_hz_assumed": rate, "time_basis": "nominal_sample_order",
        "hardware_synchronization_verified": False, "units_verified": False,
        "bias_handling": "gyro arrays used exactly as stored; no extra subtraction",
        "selection": "complete capture prefix, historical trim_mask not applied",
        "stop_reason": "max_pairs" if count < len(gp) else "capture_complete",
        "saturation_assessed_from_raw": masks is not None,
    }
    return {"prox_gyro_dps": gp[:count], "dist_gyro_dps": gd[:count],
            "near_limit_mask": None if masks is None else np.column_stack(masks),
            "metadata": metadata}


def _bias(value, name):
    if value is None:
        return np.zeros(3)
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain three finite deg/s values")
    return array


def build_hinge_report(capture, *, sample_rate_hz, proximal_bias_dps=None, distal_bias_dps=None):
    """Run a candidate experiment and retain a clear physical-validation boundary."""
    from .hinge_axes import fit_hinge_axes

    rate = _scalar(sample_rate_hz, "sample_rate_hz", 10, 2000)
    gp = np.asarray(capture["prox_gyro_dps"], dtype=float)
    gd = np.asarray(capture["dist_gyro_dps"], dtype=float)
    if gp.ndim != 2 or gp.shape[1:] != (3,) or gp.shape != gd.shape:
        raise ValueError("paired gyro arrays must have matching (N, 3) shapes")
    if not np.all(np.isfinite(gp)) or not np.all(np.isfinite(gd)):
        raise ValueError("paired gyro arrays contain nonfinite values")
    bp = _bias(proximal_bias_dps, "proximal_bias_dps")
    bd = _bias(distal_bias_dps, "distal_bias_dps")
    mask = capture.get("near_limit_mask")
    if mask is not None:
        mask = np.asarray(mask)
        if mask.dtype.kind != "b" or mask.shape not in ((len(gp),), (len(gp), 2)):
            raise ValueError("near_limit_mask must be boolean (N,) or (N, 2)")
        rows = mask if mask.ndim == 1 else mask.any(axis=1)
        near_limit_count = int(rows.sum())
    else:
        near_limit_count = None
    if near_limit_count:
        result = {"candidate_status": "input_near_limit", "reason":
                  "Raw gyro reaches int16 limits; a new axis cannot recover clipped motion.",
                  "usable_as_live_calibration": False}
    elif len(gp) < max(120, math.ceil(2.0 * rate)):
        result = {"candidate_status": "insufficient_capture", "reason":
                  "At least two nominal seconds of paired observations are needed.",
                  "usable_as_live_calibration": False}
    else:
        result = fit_hinge_axes(gp - bp, gd - bd, sample_rate_hz=rate)
    return {
        "schema_version": 1, "experiment": "free_motion_hinge_axes_v1",
        "offline_only": True, "live_calibration_changed": False,
        "source": dict(capture["metadata"]), "paired_samples": len(gp),
        "nominal_duration_s": len(gp) / rate,
        "additional_bias_dps": {"proximal": bp.tolist(), "distal": bd.tolist()},
        "raw_near_limit_rows": near_limit_count, "result": result,
        "validation": {
            "anatomical_angle_accuracy": "not_evaluated_no_independent_reference",
            "return_latency": "not_evaluated_no_independent_reference",
            "axis_sign_and_neutral": "not_established_by_axis_line_fit",
            "production_acceptance": "not_evaluated",
        },
    }


def run_hinge_experiment(path, config, *, proximal=5, distal=6, archive_prefix="",
                         byte_limit=None, start_s=0., duration_s=None, max_pairs=10000,
                         proximal_bias_dps=None, distal_bias_dps=None):
    """CLI orchestration, without serial, VQF, Qt, or modifications to the source."""
    if type(max_pairs) is not int or not 1 <= max_pairs <= 30000:
        raise ValueError("max_pairs must be an integer in [1, 30000]")
    path = Path(path)
    if path.suffix.lower() == ".npz":
        if start_s != 0 or duration_s is not None or byte_limit is not None:
            raise ValueError("NPZ selects a stored capture; time/byte slicing is for CSV or JSONL")
        if proximal_bias_dps is not None or distal_bias_dps is not None:
            raise ValueError("NPZ gyro arrays are used as stored; extra bias could subtract it twice")
        capture = load_pair_archive(path, prefix=archive_prefix, proximal=proximal,
                                    distal=distal, max_pairs=max_pairs)
        rate = capture["metadata"]["sample_rate_hz_assumed"]
    else:
        from .pair_capture import load_pair_capture
        if archive_prefix:
            raise ValueError("--archive-prefix is only supported for NPZ captures")
        capture = load_pair_capture(path, proximal=proximal, distal=distal,
                                   sample_rate_hz=config["sample_rate_hz"],
                                   gyro_dps_per_lsb=config["gyro_dps_per_lsb"],
                                   gyro_dps_per_lsb_by_node=config.get("gyro_dps_per_lsb_by_node"),
                                   byte_limit=byte_limit, start_s=start_s,
                                   duration_s=duration_s, max_pairs=max_pairs)
        rate = config["sample_rate_hz"]
    return build_hinge_report(capture, sample_rate_hz=rate,
                              proximal_bias_dps=proximal_bias_dps,
                              distal_bias_dps=distal_bias_dps)
