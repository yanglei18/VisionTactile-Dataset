"""Reproducible offline comparison on a saved hinge experiment directory.

Does not connect hardware, change original files, install a candidate or relax
quality gates. An explicit --config reinterprets raw LSB values in the report
only; both recorded and evaluation configuration remain visible.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .config import load_config, validate_config, write_json_exclusive
from .hinge_frames import build_hinge_frames
from .hinge_joint_axes import evaluate_joint_hinge_axes, fit_joint_hinge_axes
from .hinge_replay import calibrate_hinge_pair
from .pair_capture import load_pair_capture


def compare_export(directory, config=None):
    directory = Path(directory).resolve()
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("export_complete") is not True:
        raise ValueError("requires a completed export")
    recorded = validate_config(manifest["config"])
    config = recorded if config is None else validate_config(config)
    session = manifest["session"]
    prox, dist = session["proximal"], session["distal"]
    inputs = dict(proximal=prox, distal=dist, sample_rate_hz=config["sample_rate_hz"],
                  gyro_dps_per_lsb=config["gyro_dps_per_lsb"],
                  gyro_dps_per_lsb_by_node=config.get("gyro_dps_per_lsb_by_node"), max_pairs=30000)
    def load(attempt):
        name = attempt["raw_file"]
        path = directory / name
        if Path(name).name != name or path.resolve().parent != directory:
            raise ValueError("raw file must stay inside the experiment directory")
        metadata = manifest["files"][name]
        if path.stat().st_size != metadata["bytes"] or path.stat().st_size > 64*1024*1024:
            raise ValueError(f"raw file size mismatch or too large: {name}")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != metadata["sha256"]:
            raise ValueError(f"raw file hash mismatch: {name}")
        capture = load_pair_capture(path, **inputs)
        if len(capture["prox_gyro_dps"]) != attempt["matched_pairs"]:
            raise ValueError(f"pair extraction count differs from export: {name}")
        return capture
    runs, arrays = [], []
    neutral = None
    for attempt in manifest["attempts"]:
        if attempt.get("status") != "completed":
            continue
        if attempt["kind"] == "neutral":
            neutral = attempt
            continue
        if attempt["kind"] != "motion":
            continue
        if neutral is None or neutral["connection"] != attempt["connection"]:
            raise ValueError("completed motion has no independent neutral in the same connection")
        if len(runs) >= 8:
            raise ValueError("comparison supports up to eight complete neutral/motion runs")
        n, m = load(neutral), load(attempt)
        baseline = calibrate_hinge_pair(n, m, accel_g_per_lsb=config["accel_g_per_lsb"])
        if baseline.get("neutral") is None or baseline["profile_status"] in {
                "neutral_invalid", "insufficient_neutral", "input_near_limit", "insufficient_motion"}:
            raise ValueError("capture did not pass basic neutral/motion checks; inspect baseline first")
        gp = m["prox_gyro_dps"] - np.asarray(baseline["neutral"]["prox_bias_dps"])
        gd = m["dist_gyro_dps"] - np.asarray(baseline["neutral"]["dist_bias_dps"])
        ap = m["prox_acc_raw"] * config["accel_g_per_lsb"]
        ad = m["dist_acc_raw"] * config["accel_g_per_lsb"]
        candidate = fit_joint_hinge_axes(gp, gd, ap, ad, sample_rate_hz=config["sample_rate_hz"])
        frames = None
        if candidate["axis_prox"] is not None:
            frames = build_hinge_frames(candidate["axis_prox"], candidate["axis_dist"], ap, ad,
                neutral_prox_acc_g=baseline["neutral"]["prox_acc_g"],
                neutral_dist_acc_g=baseline["neutral"]["dist_acc_g"])
        runs.append(dict(neutral_file=neutral["raw_file"], motion_file=attempt["raw_file"],
                         baseline=baseline, joint_candidate=candidate,
                         frame_diagnostics_on_candidate=frames,
                         candidate_installed=False, frames_do_not_override_axis_rejection=True))
        arrays.append((gp, gd, ap, ad))
    if not runs:
        raise ValueError("no complete neutral/motion pair in export")
    transfers = []
    for i, run in enumerate(runs):
        fit = run["joint_candidate"]
        if fit["axis_prox"] is None:
            continue
        for j, other in enumerate(runs):
            if i == j:
                continue
            metrics = evaluate_joint_hinge_axes(fit["axis_prox"], fit["axis_dist"], *arrays[j])
            transfers.append(dict(fit_motion=run["motion_file"], test_motion=other["motion_file"],
                                  metrics=metrics, mounting_unchanged_not_verified=True))
    differences = []
    for i in range(len(runs)):
        for j in range(i+1, len(runs)):
            one, two = runs[i]["joint_candidate"], runs[j]["joint_candidate"]
            if one["axis_prox"] is None or two["axis_prox"] is None:
                continue
            angles = {key: float(np.rad2deg(np.arccos(np.clip(abs(np.dot(one[key],two[key])),0,1))))
                      for key in ("axis_prox", "axis_dist")}
            differences.append(dict(motions=[runs[i]["motion_file"], runs[j]["motion_file"]],
                                    unoriented_line_difference_deg=angles))
    return dict(kind="offline_joint_hinge_comparison", source_directory=str(directory),
                source_manifest_sha256=hashlib.sha256((directory/"manifest.json").read_bytes()).hexdigest(),
                recorded_config=recorded, evaluation_config=config, raw_files_modified=False,
                configured_units_not_firmware_readback=True, live_calibration_changed=False,
                production_acceptance="not_evaluated", runs=runs,
                cross_recording_metrics=transfers, axis_differences=differences,
                limitation="Cross-recording axis comparison assumes unchanged mounting; no independent angle truth.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="New JSON report; never overwrite")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(args.output)
        report = compare_export(args.directory, load_config(args.config) if args.config else None)
        write_json_exclusive(args.output, report)
        print(f"Offline comparison saved: {args.output}; no live calibration changed")
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
