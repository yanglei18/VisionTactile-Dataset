"""Fit a conditional thumb-angle preview from a saved trial, never a device.

The native NumPy joint-axis solver is used with its unchanged defaults. Its
acceptance result is retained, even when a usable experimental curve exists
but the strict fixed-hinge hypothesis is not supported. This format is NOT a
PoseEngine calibration. The external holdout is never used to fit/select axes.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .config import validate_config, write_json_exclusive
from .hinge_frames import build_hinge_frames
from .hinge_joint_axes import evaluate_joint_hinge_axes, fit_joint_hinge_axes
from .pair_capture import load_pair_capture
from .stream import iter_csv


def _window(value, name):
    if (not isinstance(value, (tuple, list)) or len(value) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)
            or not 0 <= value[0] < value[1]):
        raise ValueError(f"{name} must be a finite, increasing pair of elapsed seconds")
    return [float(v) for v in value]


def load_trial(directory):
    """Verify a complete local export; preserve every original file unchanged."""
    directory = Path(directory).resolve()
    manifest_path = directory / "manifest.json"
    if manifest_path.stat().st_size > 1024 * 1024:
        raise ValueError("trial manifest is too large")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("export_complete") is not True or manifest.get("status") != "completed":
        raise ValueError("a completed, fully saved thumb trial is required")
    for name in ("metadata.json", "raw.csv", "online.jsonl", "report.json"):
        path = directory / name
        expected = manifest.get("files", {}).get(name, {})
        if path.resolve().parent != directory:
            raise ValueError(f"trial file must remain in its directory: {name}")
        size = path.stat().st_size
        if size != expected.get("bytes") or size > 64 * 1024 * 1024:
            raise ValueError(f"trial file size mismatch or limit exceeded: {name}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected.get("sha256"):
            raise ValueError(f"trial file hash mismatch: {name}")
    meta = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    if meta.get("kind") != "vt_glove_pose.thumb_trial" or meta.get("schema_version") != 1:
        raise ValueError("expected a schema 1 thumb trial")
    meta["config"] = validate_config(meta["config"])
    start = meta.get("start_s")
    if type(start) not in (int, float) or not math.isfinite(start):
        raise ValueError("trial start must be finite")
    finger = meta["config"]["fingers"]["thumb"]
    nodes = [finger["proximal"], finger["distal"]]
    if meta.get("node_ids") != nodes:
        raise ValueError("trial node IDs differ from the configured thumb")
    return directory, manifest, meta


def _summary(rows):
    angles = [r["angle_deg"] for r in rows if r.get("angle_deg") is not None]
    innovation = [r["innovation_deg"] for r in rows if r.get("innovation_deg") is not None]
    return dict(
        samples=len(rows), available_angles=len(angles),
        angle_range_deg=[min(angles), max(angles)] if angles else None,
        last_angle_deg=rows[-1].get("angle_deg") if rows else None,
        observation_fraction=(sum(bool(r.get("current_angle_observed")) for r in rows) / len(rows)
                              if rows else None),
        quality_counts=dict(Counter(r["quality"] for r in rows)),
        history_loss_samples=sum(bool(r.get("history_loss")) for r in rows),
        innovation_rms_deg=float(np.sqrt(np.mean(np.square(innovation)))) if innovation else None,
        innovation_abs_p95_deg=float(np.percentile(np.abs(innovation), 95)) if innovation else None,
        innovation_is_not_independent_angle_error=True)


def replay_candidate(directory, profile, *, start_s, stop_s=None):
    """Use the SAME per-sample path as live, without merging equal timestamps.

    All raw nodes are fed in recorded wire order, so partial pairs are visible
    to the runtime. There is no reset at the train/holdout boundary, no forced
    terminal zero and no smoothing using future measurements.
    """
    from .thumb_angle_stream import CandidateAngleProcessor
    directory, manifest, meta = load_trial(directory)
    start_s = float(start_s)
    if not math.isfinite(start_s) or start_s < 0:
        raise ValueError("replay start must be finite and nonnegative")
    if stop_s is not None and (not math.isfinite(stop_s) or stop_s <= start_s):
        raise ValueError("replay stop must be later than its start")
    processor = CandidateAngleProcessor(profile, config=meta["config"])
    rows = []
    for sample in iter_csv(directory / "raw.csv"):
        elapsed = sample.host_monotonic_s - meta["start_s"]
        if elapsed < start_s:
            continue
        if stop_s is not None and elapsed >= stop_s:
            break
        snapshot = processor.process(sample)
        if snapshot is not None:
            rows.append(dict(snapshot, time_s=float(elapsed)))
    if not rows:
        raise ValueError("replay interval contains no complete selected pair")
    return dict(kind="vt_glove_pose.thumb_angle_replay", schema_version=1,
        recorded_only=True, source_directory=str(directory), node_ids=profile["node_ids"],
        profile=profile, series=rows, summary=_summary(rows),
        replay_start_s=start_s, replay_stop_s=stop_s,
        time_basis="host_observed_plot_time_nominal_sample_integration",
        certified_calibration=False, anatomical_accuracy_verified=False,
        reference_not_reset_at_holdout=True, forced_return_zero=False,
        source_manifest_sha256=hashlib.sha256((directory / "manifest.json").read_bytes()).hexdigest())


def build_candidate(directory, *, neutral=(3., 8.), train=(12., 21.), holdout=(21., 30.),
                    frame_sign_strategy="whole_capture"):
    """Return an explicitly experimental profile and its causal raw replay."""
    from .thumb_angle_stream import validate_candidate
    if frame_sign_strategy not in ("whole_capture", "informative_blocks"):
        raise ValueError("frame_sign_strategy must be whole_capture or informative_blocks")
    neutral, train, holdout = [_window(v, name) for v, name in
                               ((neutral, "neutral"), (train, "train"), (holdout, "holdout"))]
    if neutral[1] > train[0] or train[1] > holdout[0]:
        raise ValueError("neutral, train and holdout must be ordered and disjoint")
    directory, manifest, meta = load_trial(directory)
    config = meta["config"]
    prox, dist = meta["node_ids"]
    capture = load_pair_capture(directory / "raw.csv", proximal=prox, distal=dist,
        sample_rate_hz=config["sample_rate_hz"], gyro_dps_per_lsb=config["gyro_dps_per_lsb"],
        gyro_dps_per_lsb_by_node=config.get("gyro_dps_per_lsb_by_node"), max_pairs=30000)
    if capture["metadata"]["stop_reason"] != "eof":
        raise ValueError("candidate fitting requires the complete bounded raw recording")
    times = capture["host_monotonic_s"][:, 0] - meta["start_s"]
    masks = {name: (times >= limits[0]) & (times < limits[1]) for name, limits in
             (("neutral", neutral), ("train", train), ("holdout", holdout))}
    if any(int(mask.sum()) < 60 for mask in masks.values()):
        raise ValueError("each explicit interval needs at least 60 paired samples")
    # No saturated samples may silently contribute to the fitted coordinate system.
    fitting = masks["neutral"] | masks["train"]
    if np.any(capture["near_limit_mask"][fitting]):
        raise ValueError("raw gyro near its limit in neutral/training interval")
    biases = [capture[k][masks["neutral"]].mean(axis=0) for k in ("prox_gyro_dps", "dist_gyro_dps")]
    gp, gd = [capture[k] - bias for k, bias in zip(("prox_gyro_dps", "dist_gyro_dps"), biases)]
    ap, ad = [capture[k] * config["accel_g_per_lsb"] for k in ("prox_acc_raw", "dist_acc_raw")]
    arrays = (gp, gd, ap, ad)
    fit = fit_joint_hinge_axes(*[a[masks["train"]] for a in arrays], sample_rate_hz=config["sample_rate_hz"])
    frames = build_hinge_frames(fit["axis_prox"], fit["axis_dist"], ap[masks["train"]], ad[masks["train"]],
        neutral_prox_acc_g=ap[masks["neutral"]].mean(axis=0),
        neutral_dist_acc_g=ad[masks["neutral"]].mean(axis=0),
        sign_strategy=frame_sign_strategy, sample_indices=np.flatnonzero(masks["train"]))
    if frames["status"] != "frames_ready":
        raise ValueError("cannot construct experimental functional frames: " + ", ".join(frames["reason_codes"]))
    validation = {name: evaluate_joint_hinge_axes(fit["axis_prox"], fit["axis_dist"],
                  *[a[masks[name]] for a in arrays]) for name in ("train", "holdout")}
    profile = validate_candidate(dict(
        kind="vt_glove_pose.thumb_angle_candidate", schema_version=1, scope="conditional_scalar_preview",
        certified_calibration=False, anatomical_accuracy_verified=False,
        config=config, node_ids=[prox, dist], bias_dps={str(n): b.tolist() for n, b in zip((prox, dist), biases)},
        frames=frames, axis_fit=fit, validation=validation,
        protocol=dict(neutral_s=neutral, train_s=train, holdout_s=holdout,
                      frame_sign_strategy=frame_sign_strategy,
                      holdout_used_for_fitting_or_selection=False,
                      bias_source="explicit_neutral_window_not_arbitrary_motion",
                      sample_counts={k: int(v.sum()) for k, v in masks.items()}),
        provenance=dict(source_directory=str(directory), source_files=manifest["files"],
                        method="native_numpy_joint_gyro_accel_unchanged_defaults_no_qmt_dependency"),
        limitations=["Conditional relative bend signal; not an anatomical angle certification.",
                     "A planned hold or small gyro does not prove physical stillness.",
                     "Fixed-hinge fit rejection remains visible; it is not waived by this preview.",
                     "Removing/repositioning the glove can invalidate frozen sensor frames.",
                     "Host times and nominal integration do not establish sensor synchronization.",
                     "No automatic straight-hand zero or independent angle truth."]))
    replay = replay_candidate(directory, profile, start_s=neutral[1])
    replay["interval_summaries"] = {name: _summary([r for r in replay["series"] if limits[0] <= r["time_s"] < limits[1]])
                                     for name, limits in (("train", train), ("holdout", holdout))}
    return profile, replay


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--neutral", nargs=2, type=float, required=True, metavar=("START", "END"))
    parser.add_argument("--train", nargs=2, type=float, required=True, metavar=("START", "END"))
    parser.add_argument("--holdout", nargs=2, type=float, required=True, metavar=("START", "END"))
    parser.add_argument("--frame-sign-strategy", choices=("whole_capture", "informative_blocks"),
                        default="whole_capture", help="experimental sign evidence strategy; does not certify the fitted axes")
    args = parser.parse_args(argv)
    try:
        if args.output_dir.exists():
            raise FileExistsError(f"choose a new output directory: {args.output_dir}")
        profile, replay = build_candidate(args.trial, neutral=args.neutral, train=args.train, holdout=args.holdout,
                                          frame_sign_strategy=args.frame_sign_strategy)
        args.output_dir.mkdir(parents=True, exist_ok=False)
        write_json_exclusive(args.output_dir / "candidate.json", profile)
        write_json_exclusive(args.output_dir / "replay.json", replay)
        print(f"Experimental candidate + raw replay saved: {args.output_dir}")
        print(f"Original axis-fit result preserved: {profile['axis_fit']['candidate_status']}; not certified calibration")
        print(json.dumps(replay["interval_summaries"]["holdout"], ensure_ascii=False))
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
