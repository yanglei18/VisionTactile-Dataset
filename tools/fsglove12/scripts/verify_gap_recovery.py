#!/usr/bin/env python3
"""Bounded, offline raw-gap regression; never opens a serial port or old profile.

Replays exactly the same recorded rows under .15 s and 1 s reset thresholds.
Synthetic identity profiles test ONLY epoch rejection, not real calibration,
orientation accuracy, recovered missing motion, or sensor-to-photon latency.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import importlib.metadata
from itertools import groupby
import json
import math
from pathlib import Path

from vt_fsglove12.calibration import apply_calibration
from vt_fsglove12.config import load_config
from vt_fsglove12.live_stream import CSV_FIELDS, OnlineFusion, RawSample


def bounded_window(path, start, end, max_lines=150_000):
    """Read only the header and at most the final max_lines complete data rows."""
    if not math.isfinite(start) or not math.isfinite(end) or start >= end:
        raise ValueError("start/end must be finite with start < end")
    with path.open("rb") as stream:
        header = stream.readline()
        stream.seek(0, 2)
        size, chunks, newlines = stream.tell(), [], 0
        position = size
        while position > 0 and newlines <= max_lines:
            length = min(65536, position)
            position -= length
            stream.seek(position)
            block = stream.read(length)
            chunks.append(block)
            newlines += block.count(b"\n")
    raw = b"".join(reversed(chunks))
    lines = raw.splitlines(keepends=True)
    if position:
        lines = lines[1:]  # First tail line may be partial.
    elif lines:
        lines = lines[1:]  # Header.
    if lines and not lines[-1].endswith(b"\n"):
        lines.pop()  # A concurrently appended incomplete row is not a sample.
    lines = lines[-max_lines:]
    if not lines:
        raise ValueError("no complete rows in bounded tail")
    fields = next(csv.reader([header.decode("utf-8-sig").strip()]))
    if not set(CSV_FIELDS[:-1]) <= set(fields):
        raise ValueError("raw CSV lacks required fields")
    samples, selected = [], []
    first_tail_stamp = None
    last_tail_stamp = None
    for line in lines:
        row = dict(zip(fields, next(csv.reader([line.decode("utf-8").strip()]))))
        stamp = float(row[CSV_FIELDS[0]])
        first_tail_stamp = stamp if first_tail_stamp is None else first_tail_stamp
        last_tail_stamp = stamp
        if start <= stamp <= end:
            status = row.get("status", "").strip()
            samples.append(RawSample(int(row["node_id"]), tuple(int(row[key]) for key in CSV_FIELDS[3:9]),
                                     stamp, float(row[CSV_FIELDS[1]]), int(status) if status else None))
            selected.append(line)
    if first_tail_stamp > start or last_tail_stamp < end:
        raise ValueError(f"bounded tail does not cover [{start}, {end}]: [{first_tail_stamp}, {last_tail_stamp}]")
    if not samples:
        raise ValueError("selected window has no samples")
    return samples, header + b"".join(selected), dict(source_bytes_at_read=size,
        bounded_tail_bytes_read=len(raw), bounded_tail_rows_parsed=len(lines),
        requested_window_s=[start, end], first_sample_s=samples[0].host_monotonic_s,
        last_sample_s=samples[-1].host_monotonic_s)


def identity_profile(epochs, stale_after):
    return dict(kind="synthetic_identity_test_only", calibration_accuracy_verified=False,
                session_id="independent_offline_epoch_contract", stale_after_s=stale_after,
                nodes={node: dict(epoch=epoch, sensor_from_bone_xyzw=[0., 0., 0., 1.],
                                  world_alignment_xyzw=[0., 0., 0., 1.]) for node, epoch in epochs.items()})


def accepted(frame, profile):
    observations, rejected = apply_calibration(frame, profile)
    return dict(accepted_nodes=sorted(observations, key=int), rejected_nodes=rejected)


def compare_recorded(samples, config):
    stale = .15
    configs = {name: dict(config, stale_after_s=stale, filter_reset_gap_s=gap)
               for name, gap in (("old", .15), ("new", 1.))}
    fusions = {name: OnlineFusion(settings) for name, settings in configs.items()}
    for name, fusion in fusions.items():
        if fusion.settings.get("filter_reset_gap_s") != configs[name]["filter_reset_gap_s"]:
            raise RuntimeError("OnlineFusion must implement separate filter_reset_gap_s before this check can run")
    profile = identity_profile(dict(fusions["new"].epochs), stale)
    previous, gaps, last_stamp, probes = {}, {str(n): [] for n in range(1, 13)}, None, []
    for stamp, rows in groupby(samples, key=lambda sample: sample.host_monotonic_s):
        rows = list(rows)
        if last_stamp is not None and stamp < last_stamp:
            raise ValueError("this short-gap fixture must not contain backward host time")
        if last_stamp is not None and stale < stamp - last_stamp < 1.:
            probe_time = last_stamp + stale + (stamp - last_stamp - stale) / 2.
            probe = dict(before_s=last_stamp, resume_s=stamp, gap_s=stamp - last_stamp,
                         stale_probe_s=probe_time)
            for name, fusion in fusions.items():
                epochs, counts = dict(fusion.epochs), dict(fusion.counts)
                frame = fusion.snapshot(probe_time)
                if frame["nodes"] or epochs != fusion.epochs or counts != fusion.counts:
                    raise AssertionError("silent stale snapshot must hide nodes without resetting or updating filters")
                probe[name] = dict(visible_nodes=[], epochs_unchanged=True, sample_counts_unchanged=True)
            probes.append(probe)
        for sample in rows:
            node = str(sample.node_id)
            if node in previous:
                gaps[node].append(stamp - previous[node])
            previous[node] = stamp
        for fusion in fusions.values():
            fusion.update(rows)
        last_stamp = stamp
    if not probes:
        raise AssertionError("recorded window contains no .15–1.0 s host gap")
    counts = dict(sorted(Counter(str(s.node_id) for s in samples).items(), key=lambda item: int(item[0])))
    summaries = {}
    for name, fusion in fusions.items():
        frame = fusion.snapshot(last_stamp)
        if fusion.counts != counts:
            raise AssertionError(f"{name} did not consume exactly the selected recorded samples")
        summaries[name] = dict(filter_reset_gap_s=configs[name]["filter_reset_gap_s"],
                               stale_after_s=stale, sample_count=sum(fusion.counts.values()),
                               sample_counts_by_node=fusion.counts, final_epochs=fusion.epochs,
                               final_fresh_nodes=sorted(frame["nodes"], key=int),
                               synthetic_identity_epoch_contract=accepted(frame, profile))
    old_resets = [node for node, epoch in fusions["old"].epochs.items() if epoch > 0]
    new_resets = [node for node, epoch in fusions["new"].epochs.items() if epoch > 0]
    if len(old_resets) != 12 or new_resets:
        raise AssertionError(f"expected old all12 reset and new none, got old={old_resets}, new={new_resets}")
    if len(summaries["new"]["synthetic_identity_epoch_contract"]["accepted_nodes"]) != 12:
        raise AssertionError("new strategy must accept all12 nodes with its own offline epoch-bound test profile")
    return dict(strategies=summaries, per_node_max_gap_s={node: max(values) for node, values in gaps.items()},
                old_reset_nodes=sorted(old_resets, key=int), new_reset_nodes=new_resets,
                stale_gap_probes=probes, profile_kind=profile["kind"])


def synthetic_epoch_contract(config):
    """Real VQF, synthetic stationary inputs: short recovery and long-gap reset."""
    fusion = OnlineFusion(dict(config, stale_after_s=.15, filter_reset_gap_s=1.))

    def rows(stamp):
        return [RawSample(node, (0, 0, 0, 0, 0, 8197), stamp, 1000. + stamp) for node in range(1, 13)]

    fusion.update(rows(1.))
    profile = identity_profile(dict(fusion.epochs), .15)
    fresh = accepted(fusion.snapshot(1.), profile)
    stale = accepted(fusion.snapshot(1.151), profile)
    assert len(fresh["accepted_nodes"]) == 12 and not stale["accepted_nodes"]
    assert all(epoch == 0 for epoch in fusion.epochs.values())
    fusion.update(rows(1.164))
    resumed = accepted(fusion.snapshot(1.164), profile)
    assert len(resumed["accepted_nodes"]) == 12 and all(epoch == 0 for epoch in fusion.epochs.values())
    fusion.update(rows(2.2))  # 1.036 s, strictly above the new reset threshold.
    long_gap = accepted(fusion.snapshot(2.2), profile)
    assert all(epoch == 1 for epoch in fusion.epochs.values()) and not long_gap["accepted_nodes"]
    assert set(long_gap["rejected_nodes"].values()) == {"filter_epoch_changed_recalibration_required"}
    return dict(kind="synthetic_identity_test_only_real_vqf", fresh=fresh, stale_after_151_ms=stale,
                recovery_after_164_ms=resumed, reset_after_1036_ms=long_gap,
                final_epochs=fusion.epochs, samples_per_node=fusion.counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", required=True, type=Path)
    parser.add_argument("--start", required=True, type=float)
    parser.add_argument("--end", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path, help="NEW output directory; never overwritten")
    parser.add_argument("--config", type=Path, help="Defaults to raw.csv's session config.json, if present")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; choose a new directory")
    config_path = args.config or args.raw.parent / "config.json"
    config = load_config(config_path if config_path.exists() else None)
    samples, selected_bytes, source = bounded_window(args.raw, args.start, args.end)
    recorded = compare_recorded(samples, config)
    synthetic = synthetic_epoch_contract(config)
    report = dict(status="passed", created_utc=datetime.now(timezone.utc).isoformat(),
        hardware_accessed=False, old_live_profile_loaded=False, orientation_accuracy_verified=False,
        source_raw_path=str(args.raw.resolve()), configuration_path=str(config_path.resolve()),
        source_full_hash_computed=False, source=source,
        copied_window_sha256=hashlib.sha256(selected_bytes).hexdigest(),
        copied_window_file="recorded_window_raw.csv", recorded_window_comparison=recorded,
        synthetic_epoch_contract=synthetic, dependency_versions={name: importlib.metadata.version(name) for name in ("vqf", "numpy", "scipy")},
        limitations=["Offline filters are cold-started at the selected window, not resumed from the live filter state.",
                     "Identity profiles test only current offline epoch acceptance; no historical calibrated posture is claimed.",
                     "No missing IMU motion is reconstructed or integrated across the gap.",
                     "Host parse gaps are not verified device sampling gaps; this is not a physical accuracy or latency measurement."])
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output / "recorded_window_raw.csv").open("xb") as stream:
        stream.write(selected_bytes)
    with (args.output / "verification_report.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(status=report["status"], artifact_dir=str(args.output), recorded_samples=len(samples),
                         max_gap_s=max(recorded["per_node_max_gap_s"].values()),
                         old_reset_nodes=recorded["old_reset_nodes"], new_reset_nodes=recorded["new_reset_nodes"])))


if __name__ == "__main__":
    main()
