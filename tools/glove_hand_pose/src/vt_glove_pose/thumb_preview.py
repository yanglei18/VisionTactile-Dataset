"""Turn saved real IMU recordings into an independent-sensor orientation replay.

No joint axes or anatomical joint angles are estimated. Each motion recording
starts a NEW filter and display reference: the unrecorded time between neutral
and motion is deliberately not integrated or presented as observed motion.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .config import load_config, validate_config, write_json_exclusive
from .pair_capture import load_pair_capture
from .records import Sample


def export_captures(directory, config):
    """Read complete neutral/motion pairs without modifying the export."""
    directory = Path(directory).resolve()
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("export_complete") is not True:
        raise ValueError("A completed experiment export is required")
    recorded = validate_config(manifest["config"])
    config = validate_config(config)
    prox, dist = manifest["session"]["proximal"], manifest["session"]["distal"]
    def read(attempt, units):
        name = attempt["raw_file"]
        path = directory / name
        if Path(name).name != name or path.resolve().parent != directory:
            raise ValueError("Raw recording must be inside the export directory")
        metadata = manifest["files"][name]
        if path.stat().st_size != metadata["bytes"] or path.stat().st_size > 64*1024*1024:
            raise ValueError(f"Raw file size mismatch: {name}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != metadata["sha256"]:
            raise ValueError(f"Raw file hash mismatch: {name}")
        capture = load_pair_capture(path, proximal=prox, distal=dist,
            sample_rate_hz=units["sample_rate_hz"], gyro_dps_per_lsb=units["gyro_dps_per_lsb"],
            gyro_dps_per_lsb_by_node=units.get("gyro_dps_per_lsb_by_node"), max_pairs=30000)
        if len(capture["prox_gyro_raw"]) != attempt["matched_pairs"]:
            raise ValueError(f"Incomplete extraction: {name}")
        return capture
    neutral = None
    result = []
    for attempt in manifest["attempts"]:
        if attempt.get("status") != "completed":
            continue
        if attempt["kind"] == "neutral":
            neutral = attempt
        elif attempt["kind"] == "motion":
            if neutral is None or neutral["connection"] != attempt["connection"]:
                raise ValueError("Motion needs a separate neutral in the same connection")
            if len(result) >= 8:
                raise ValueError("At most eight complete motion recordings are supported")
            result.append(dict(name=attempt["raw_file"], neutral_file=neutral["raw_file"],
                neutral=read(neutral, config), motion=read(attempt, config),
                legacy_neutral=read(neutral, recorded), legacy_motion=read(attempt, recorded)))
    if not result:
        raise ValueError("No completed neutral/motion recording pair")
    return manifest, result


def snapshot_row(snapshot, time_s):
    """Small common wire format for recorded and live preview widgets."""
    ids = snapshot["node_ids"]
    row = dict(time_s=float(time_s))
    for label, node in zip(("prox", "dist"), ids):
        data = snapshot["nodes"][str(node)]
        row[f"quat_{label}"] = data["relative_quat_wxyz"]
        row[f"rotation_{label}_deg"] = data["rotation_magnitude_deg"]
        row[f"gravity_{label}_deg"] = data["gravity_direction_change_deg"]
    return row


def replay_orientation(neutral, motion, config, nodes):
    from .thumb_orientation import ThumbOrientationProcessor
    bias = {node: np.mean(neutral[f"{label}_gyro_dps"], axis=0)
            for node, label in zip(nodes, ("prox", "dist"))}
    processor = ThumbOrientationProcessor(config, node_ids=nodes, bias_dps=bias)
    times = motion["host_monotonic_s"]
    rows = []
    wire_order = sorted([(col, node, label) for col, (node, label)
                         in enumerate(zip(nodes, ("prox", "dist")))], key=lambda item: item[1])
    for i in range(len(times)):
        for col, node, label in wire_order:
            raw = np.r_[motion[f"{label}_gyro_raw"][i], motion[f"{label}_acc_raw"][i]]
            processor.process(Sample(node, tuple(int(x) for x in raw),
                                     times[i, col], motion["host_wall_s"][i, col]))
        if i == 0:
            processor.set_reference(source="first_motion_pair_not_neutral")
        snapshot = processor.snapshot(now=times[i, 1])
        if not snapshot["reference_ready"]:
            raise ValueError("Recording contains a gap/invalid input; split into continuous runs first")
        rows.append(snapshot_row(snapshot, times[i, 1] - times[0, 1]))
    return rows


def build_preview(directory, config):
    manifest, captures = export_captures(directory, config)
    nodes = [manifest["session"]["proximal"], manifest["session"]["distal"]]
    runs = []
    for capture in captures:
        current = replay_orientation(capture["neutral"], capture["motion"], config, nodes)
        legacy = replay_orientation(capture["legacy_neutral"], capture["legacy_motion"],
                                    manifest["config"], nodes)
        summary = dict(samples=len(current), host_span_s=current[-1]["time_s"],
                       reference="first_motion_sample_not_anatomical_zero",
                       neutral_used_for="gyro_bias_only",
                       recorded_gap_not_integrated=True, no_independent_angle_truth=True)
        for name in ("prox", "dist"):
            summary[f"rotation_{name}_range_deg"] = [
                float(min(row[f"rotation_{name}_deg"] for row in current)),
                float(max(row[f"rotation_{name}_deg"] for row in current))]
        runs.append(dict(name=capture["name"], neutral_file=capture["neutral_file"],
                         samples=current, legacy_samples=legacy, summary=summary))
    return dict(kind="vt_glove_pose.thumb_orientation_preview", schema_version=1,
                source_directory=str(Path(directory).resolve()), recorded_only=True,
                node_ids=nodes, config_provenance=dict(recorded=manifest["config"], evaluation=config),
                source_manifest_sha256=hashlib.sha256((Path(directory)/"manifest.json").read_bytes()).hexdigest(),
                filter="upstream VQF six-axis, nominal sample period, motion bias disabled",
                reference="Each sensor has its own first-motion-sample reference; no common calibrated heading",
                anatomical_joint_angle=False, production_acceptance="not_evaluated", runs=runs)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(args.output)
        report = build_preview(args.directory, load_config(args.config))
        write_json_exclusive(args.output, report)
        print(f"Real-data preview saved: {args.output}; {len(report['runs'])} independent recordings")
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
