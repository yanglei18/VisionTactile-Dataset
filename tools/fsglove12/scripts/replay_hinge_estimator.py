#!/usr/bin/env python3
"""Replay saved, byte-bounded orientations through the current hinge estimator.

This never opens a device, rewrites source recordings, or treats unavailable
angles as successful jump removal. The saved official calibration is restored,
not refitted. Input prefixes are hashed and can be pinned with --source-manifest.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

import vt_fsglove12.hinge_estimator as implementation
from vt_fsglove12.heading_correction import CausalHeadingCorrector
from vt_fsglove12.hinge_estimator import FunctionalHingeEstimator, METHOD
from vt_fsglove12.upstream_backend import OfficialFSGloveBackend


ESTIMATOR_SOURCE = Path(implementation.__file__).resolve()
ESTIMATOR_SHA256 = hashlib.sha256(ESTIMATOR_SOURCE.read_bytes()).hexdigest()
SCRIPT_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


class PrefixReader:
    def __init__(self, path, pinned=None):
        self.path = path
        self.snapshot_bytes = path.stat().st_size
        self.limit = self.snapshot_bytes if pinned is None else pinned["prefix_bytes"]
        if self.limit > self.snapshot_bytes:
            raise ValueError(f"source shrank below pinned prefix: {path}")
        self.expected = None if pinned is None else pinned["prefix_sha256"]
        self.summary = None

    def until(self, end):
        checksum = hashlib.sha256()
        consumed = lines = 0
        with self.path.open("rb") as source:
            while consumed < self.limit:
                line = source.readline(self.limit - consumed)
                if not line or not line.endswith(b"\n"):
                    break  # Ignore only the uncommitted tail of an active log.
                checksum.update(line)
                consumed += len(line)
                lines += 1
                row = json.loads(line)
                if row["timestamp_s"] > end:
                    break
                yield row
        self.summary = dict(path=str(self.path), snapshot_bytes=self.snapshot_bytes,
                            prefix_bytes=consumed, prefix_sha256=checksum.hexdigest(),
                            complete_lines=lines)
        if self.expected is not None and self.expected != self.summary["prefix_sha256"]:
            raise ValueError(f"pinned source prefix hash changed: {self.path}")


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                    encoding="utf-8")


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_angle(pose, slot):
    angles = pose.get("anatomical_euler_rad")
    if angles is None:
        return None
    value = float(angles[slot][2])
    return math.degrees(value) if math.isfinite(value) else None


def old_record(pose, slots):
    adapter = pose.get("functional_adapter", {})
    return dict(frame_index=pose["frame_index"], timestamp_s=pose["timestamp_s"],
                angle_deg={str(slot): finite_angle(pose, slot) for slot in slots},
                little_heading=pose.get("little_relative_heading", {}),
                little_continuity=adapter.get("angle_continuity_by_slot", {}).get("8", {}),
                functional_enabled=adapter.get("enabled"),
                projection_deg=adapter.get("projection_deg_by_slot", {}),
                residual_deg=adapter.get("residual_deg_by_slot", {}))


def input_change(previous, current, dependencies):
    output = {}
    for node in dependencies:
        first, second = previous.get("nodes", {}).get(node), current.get("nodes", {}).get(node)
        if first is None or second is None:
            output[node] = {"missing": True}
            continue
        step = math.degrees(float((Rotation.from_quat(first["quat_xyzw"]).inv()
                                   * Rotation.from_quat(second["quat_xyzw"])).magnitude()))
        output[node] = dict(quaternion_step_deg=step,
            previous_sample_index=first.get("sample_index"), sample_index=second.get("sample_index"),
            previous_epoch=first.get("epoch"), epoch=second.get("epoch"),
            previous_orientation_valid=first.get("orientation_valid"),
            orientation_valid=second.get("orientation_valid"),
            previous_input_filter=first.get("input_filter"), input_filter=second.get("input_filter"),
            previous_gyro_rad_s=first.get("gyro_rad_s"), gyro_rad_s=second.get("gyro_rad_s"),
            age_s=second.get("age_s"))
    return output


def transition(previous, current, slot):
    key = str(slot)
    old_a, old_b = previous["recorded"]["angle_deg"][key], current["recorded"]["angle_deg"][key]
    new_a, new_b = previous["replayed"][key], current["replayed"][key]
    return dict(slot=slot, previous_frame_index=previous["frame_index"],
        frame_index=current["frame_index"], previous_timestamp_s=previous["timestamp_s"],
        timestamp_s=current["timestamp_s"], dt_s=current["timestamp_s"]-previous["timestamp_s"],
        source_frame_gap=current["frame_index"]-previous["frame_index"],
        recorded_previous_deg=old_a, recorded_deg=old_b,
        recorded_step_deg=None if old_a is None or old_b is None else old_b-old_a,
        replay_previous_deg=new_a["angle_deg"], replay_deg=new_b["angle_deg"],
        replay_step_deg=(None if new_a["angle_deg"] is None or new_b["angle_deg"] is None
                         else new_b["angle_deg"]-new_a["angle_deg"]),
        diagnostic_previous_deg=new_a["diagnostic_angle_deg"],
        diagnostic_deg=new_b["diagnostic_angle_deg"],
        diagnostic_step_deg=(None if new_a["diagnostic_angle_deg"] is None or new_b["diagnostic_angle_deg"] is None
                            else new_b["diagnostic_angle_deg"]-new_a["diagnostic_angle_deg"]),
        replay_previous_quality=new_a["quality"], replay_quality=new_b["quality"],
        replay_previous_detail=new_a["detail"], replay_detail=new_b["detail"],
        recorded_previous_heading=previous["recorded"]["little_heading"] if slot == 8 else None,
        recorded_heading=current["recorded"]["little_heading"] if slot == 8 else None,
        recorded_previous_continuity=previous["recorded"]["little_continuity"] if slot == 8 else None,
        recorded_continuity=current["recorded"]["little_continuity"] if slot == 8 else None)


def diagnostic_step(previous, current, frame, geometry, dependencies):
    """Decompose on the same current input, even when output is unavailable."""
    before, after = previous["diagnostic_angle_deg"], current["diagnostic_angle_deg"]
    delta = previous["detail"].get("delta_rad")
    if before is None or after is None or delta is None:
        return None
    hp, hc, rp, rc, axis = geometry
    first, second = (frame["nodes"][node] for node in dependencies)
    q1 = hp * Rotation.from_quat(first["quat_xyzw"]) * rp
    q2 = hc * Rotation.from_quat(second["quat_xyzw"]) * rc
    held, _ = FunctionalHingeEstimator._project(q1, q2, axis, delta)
    held = math.degrees(held)
    motion = (held-before+180.) % 360.-180.
    parameter = (after-held+180.) % 360.-180.
    return dict(input_projection_step_deg=motion, heading_parameter_step_deg=parameter,
                branch_step_deg=after-before-motion-parameter,
                total_diagnostic_step_deg=after-before)


def same_pair_counterfactual(event, frame, calibrator, binding, profile):
    """Separate current input motion from changing yaw on the same quaternion pair."""
    before, after = event["recorded_previous_heading"], event["recorded_heading"]
    if not before.get("applied") or not after.get("applied"):
        return None
    reference = profile["functional_fingers"]["little"]["slot_refs"]["8"]
    hp, hc, rp, rc, axis = CausalHeadingCorrector._geometry(calibrator, 7, 8, reference)
    nodes = [frame["nodes"][node] for node in binding["dependency_nodes_by_slot"]["8"]]
    q1 = hp * Rotation.from_quat(nodes[0]["quat_xyzw"]) * rp
    q2 = hc * Rotation.from_quat(nodes[1]["quat_xyzw"]) * rc
    held, held_residual = FunctionalHingeEstimator._project(q1, q2, axis, before["delta_rad"])
    held = math.degrees(held)
    held += round((event["recorded_previous_deg"]-held)/360.)*360.
    return dict(current_pair_previous_delta_angle_deg=held,
        current_pair_previous_delta_residual_deg=held_residual,
        input_motion_contribution_deg=held-event["recorded_previous_deg"],
        instantaneous_heading_change_contribution_deg=event["recorded_deg"]-held,
        delta_change_deg=math.degrees((after["delta_rad"]-before["delta_rad"]+math.pi) % math.tau-math.pi),
        scope="Algebraic same-input decomposition, not a physical-motion ground truth")


def replay(args):
    started = time.monotonic()
    session, output = args.session.resolve(), args.output.resolve()
    if output == session or session in output.parents:
        raise ValueError("output must be outside the original recording directory")
    output.mkdir(parents=True, exist_ok=True)
    pinned = json.loads(args.source_manifest.read_text()) if args.source_manifest else {}
    readers = {name: PrefixReader(session / name, pinned.get("sources", {}).get(name))
               for name in ("poses.jsonl", "orientations.jsonl")}
    print("Scanning frozen pose prefix", flush=True)
    selected = {}
    for pose in readers["poses.jsonl"].until(args.end):
        if (pose["timestamp_s"] >= args.start and pose.get("profile_version") == args.profile_version):
            index = pose["frame_index"]
            if index in selected:
                raise ValueError(f"duplicate saved pose frame_index {index}")
            selected[index] = old_record(pose, args.slots)
    if not selected:
        raise ValueError("no saved poses selected by profile and time bounds")
    filenames = ("config.json", "rest.json", f"profile.v{args.profile_version:03d}.json")
    frozen = {name: (session / name).read_bytes() for name in filenames}
    config, rest, profile = [json.loads(frozen[name]) for name in filenames]
    provenance = config["upstream_provenance"]
    backend = OfficialFSGloveBackend(config, rest, checkout=provenance["checkout"],
                                     mano_assets_root=provenance["mano_assets_root"])
    calibrator = backend._calibrator(profile)
    binding = backend._functional_binding(profile)
    estimator = FunctionalHingeEstimator()
    finger_names = {2: "index", 5: "middle", 8: "little", 11: "ring", 15: "thumb"}
    geometry = {slot: CausalHeadingCorrector._geometry(calibrator, slot-1, slot,
        profile["functional_fingers"][finger_names[slot]]["slot_refs"][str(slot)]) for slot in args.slots}
    counters = {str(slot): Counter() for slot in args.slots}
    statuses = {str(slot): Counter() for slot in args.slots}
    deltas = {str(slot): [] for slot in args.slots}
    missing_runs = {str(slot): dict(start=None, longest_s=0., longest_frames=0, frames=0, path=[]) for slot in args.slots}
    diagnostic_deltas = {str(slot): [] for slot in args.slots}
    old_jumps, new_jumps, recoveries = [], [], []
    previous = previous_frame = None
    last_available = {}
    matched = set()
    latencies = []
    print(f"Replaying {len(selected)} saved poses; slots {args.slots}", flush=True)
    with (output / "frame_comparison.jsonl").open("w", encoding="utf-8") as trace:
        for frame in readers["orientations.jsonl"].until(args.end):
            if frame["timestamp_s"] < args.start:
                continue
            index = frame["frame_index"]
            if index not in selected:
                continue
            if index in matched:
                raise ValueError(f"duplicate saved orientation frame_index {index}")
            recorded = selected[index]
            if abs(frame["timestamp_s"]-recorded["timestamp_s"]) > 1e-9:
                raise ValueError(f"pose/orientation time mismatch at frame {index}")
            matched.add(index)
            current = dict(frame_index=index, timestamp_s=frame["timestamp_s"],
                           recorded=recorded, replayed={})
            tick = time.perf_counter()
            for slot in args.slots:
                key = str(slot)
                angle, detail = estimator.apply(slot, frame, profile, calibrator, binding)
                quality = detail["quality"]
                degree = None if angle is None else math.degrees(angle)
                diagnostic_angle = detail.get("continuous_angle_rad", angle)
                diagnostic_degree = None if diagnostic_angle is None else math.degrees(diagnostic_angle)
                replayed = dict(angle_deg=degree, diagnostic_angle_deg=diagnostic_degree,
                                quality=quality, detail=detail)
                current["replayed"][key] = replayed
                decomposition = None if previous is None else diagnostic_step(
                    previous["replayed"][key], replayed, frame, geometry[slot],
                    binding["dependency_nodes_by_slot"][key])
                replayed["diagnostic_step_decomposition"] = decomposition
                if decomposition is not None:
                    diagnostic_deltas[key].append(abs(decomposition["total_diagnostic_step_deg"]))
                path_point = dict(frame_index=index, timestamp_s=frame["timestamp_s"],
                                  diagnostic_angle_deg=diagnostic_degree, decomposition=decomposition)
                counters[key][quality] += 1
                statuses[key][detail.get("reason", detail["status"])] += 1
                run = missing_runs[key]
                if degree is None:
                    run["start"] = frame["timestamp_s"] if run["start"] is None else run["start"]
                    run["frames"] += 1
                    run["path"].append(path_point)
                    run["longest_s"] = max(run["longest_s"], frame["timestamp_s"]-run["start"])
                    run["longest_frames"] = max(run["longest_frames"], run["frames"])
                else:
                    if run["start"] is not None:
                        last = last_available.get(key)
                        recoveries.append(dict(slot=slot, frame_index=index,
                            timestamp_s=frame["timestamp_s"], unavailable_frames=run["frames"],
                            angle_deg=degree, previous_available=last,
                            step_from_last_available_deg=None if last is None else degree-last["angle_deg"],
                            quality=quality, continuity=detail.get("continuity"),
                            unavailable_diagnostic_path=run["path"]+[path_point],
                            diagnostic_path_contributions_deg={name: sum(
                                item["decomposition"][name] for item in run["path"]+[path_point]
                                if item["decomposition"] is not None) for name in
                                ("input_projection_step_deg", "heading_parameter_step_deg", "branch_step_deg")},
                            diagnostic_path_incomplete=any(item["decomposition"] is None
                                for item in run["path"]+[path_point])))
                    run["start"], run["frames"] = None, 0
                    run["path"] = []
                    last_available[key] = dict(frame_index=index, timestamp_s=frame["timestamp_s"], angle_deg=degree)
                if previous is None:
                    continue
                event = transition(previous, current, slot)
                old_step, new_step = event["recorded_step_deg"], event["replay_step_deg"]
                if new_step is None:
                    counters[key]["unassessable_adjacent_transitions"] += 1
                else:
                    deltas[key].append(abs(new_step))
                    counters[key]["assessable_adjacent_transitions"] += 1
                    if abs(new_step) > args.jump_deg:
                        new_jumps.append(event)
                if old_step is not None and old_step < -args.jump_deg:
                    if slot == 8:
                        event["recorded_heading_application_changed"] = bool(
                            event["recorded_previous_heading"].get("applied")) != bool(
                            event["recorded_heading"].get("applied"))
                        event["same_pair_counterfactual"] = same_pair_counterfactual(
                            event, frame, calibrator, binding, profile)
                    event["input_change"] = input_change(previous_frame, frame,
                        binding["dependency_nodes_by_slot"][key])
                    event["replay_outcome"] = ("unassessable_unavailable" if new_step is None else
                        "negative_jump_remains" if new_step < -args.jump_deg else
                        "positive_jump_instead" if new_step > args.jump_deg else "no_threshold_jump")
                    old_jumps.append(event)
            latencies.append((time.perf_counter()-tick)*1000)
            trace.write(json.dumps(current, allow_nan=False, separators=(",", ":")) + "\n")
            previous, previous_frame = current, frame
            if len(matched) % 2000 == 0:
                print(f"Replayed {len(matched)}/{len(selected)}", flush=True)
    missing = sorted(set(selected)-matched)
    if missing:
        raise ValueError(f"{len(missing)} saved poses lack matched orientation frames: {missing[:10]}")
    manifest = dict(session=str(session), sources={key: reader.summary for key, reader in readers.items()},
        calibration_files={name: dict(sha256=hashlib.sha256(content).hexdigest(), bytes=len(content))
                           for name, content in frozen.items()})
    if pinned.get("calibration_files") and pinned["calibration_files"] != manifest["calibration_files"]:
        raise ValueError("pinned calibration inputs changed")
    summaries = {}
    for slot in args.slots:
        key = str(slot)
        old_events = [event for event in old_jumps if event["slot"] == slot]
        new_events = [event for event in new_jumps if event["slot"] == slot]
        summaries[key] = dict(frames=len(matched), counts=dict(counters[key]),
            coverage={quality: counters[key][quality]/len(matched)
                      for quality in ("valid", "degraded", "unavailable")},
            display_available_fraction=(counters[key]["valid"]+counters[key]["degraded"])/len(matched),
            status_counts=dict(statuses[key]),
            longest_unavailable_s=missing_runs[key]["longest_s"],
            longest_unavailable_frames=missing_runs[key]["longest_frames"],
            recorded_negative_jump_count=len(old_events),
            replay_negative_jump_count=sum(event["replay_step_deg"] < 0 for event in new_events),
            replay_positive_jump_count=sum(event["replay_step_deg"] > 0 for event in new_events),
            recorded_jump_outcomes=dict(Counter(event["replay_outcome"] for event in old_events)),
            max_assessable_step_deg=max(deltas[key], default=None),
            p99_assessable_step_deg=float(np.percentile(deltas[key], 99)) if deltas[key] else None,
            diagnostic_max_step_deg=max(diagnostic_deltas[key], default=None),
            diagnostic_over_threshold_step_count=sum(value > args.jump_deg for value in diagnostic_deltas[key]),
            diagnostic_note="Includes unavailable projections only for continuity analysis, never valid output coverage",
            recovery_count=sum(event["slot"] == slot for event in recoveries),
            large_recovery_change_count=sum(event["slot"] == slot
                and event["step_from_last_available_deg"] is not None
                and abs(event["step_from_last_available_deg"]) > args.jump_deg for event in recoveries))
    little_events = [event for event in old_jumps if event["slot"] == 8]
    focus_coverage = None
    if little_events:
        focus_start = little_events[0]["previous_timestamp_s"]
        focus_end = little_events[-1]["timestamp_s"]
        focus_counts = Counter()
        with (output / "frame_comparison.jsonl").open(encoding="utf-8") as trace:
            for line in trace:
                row = json.loads(line)
                if focus_start <= row["timestamp_s"] <= focus_end:
                    focus_counts[row["replayed"]["8"]["quality"]] += 1
        focus_total = sum(focus_counts.values())
        focus_coverage = dict(start_s=focus_start, end_s=focus_end, frames=focus_total,
            counts=dict(focus_counts), coverage={quality: focus_counts[quality]/focus_total
                for quality in ("valid", "degraded", "unavailable")},
            note="First-to-last recorded little-finger jump interval; avoids diluting active-period losses with later quiet frames")
    report = dict(method=METHOD, settings=estimator.settings, session=str(session),
        profile_version=args.profile_version, start_s=args.start, end_s=args.end,
        actual_start_s=min(row["timestamp_s"] for row in selected.values()),
        actual_end_s=max(row["timestamp_s"] for row in selected.values()),
        frames=len(matched), slots=summaries, jump_threshold_deg=args.jump_deg,
        little_recorded_switch_jump_count=sum(event["recorded_heading_application_changed"] for event in little_events),
        little_recorded_non_switch_jump_count=sum(not event["recorded_heading_application_changed"] for event in little_events),
        little_jump_interval_coverage=focus_coverage,
        source_manifest="source_manifest.json", frame_trace="frame_comparison.jsonl", jump_events="jump_events.json",
        provenance=dict(estimator_file=str(ESTIMATOR_SOURCE), estimator_sha256=ESTIMATOR_SHA256,
            estimator_file_changed_during_run=file_hash(ESTIMATOR_SOURCE) != ESTIMATOR_SHA256,
            replay_script_sha256=SCRIPT_SHA256,
            replay_script_changed_during_run=file_hash(Path(__file__)) != SCRIPT_SHA256,
            official_core=backend.core.provenance),
        estimator_frame_compute_ms=dict(median=float(np.median(latencies)), p95=float(np.percentile(latencies, 95)),
            note="All requested slots, diagnostic assembly included; not motion latency or a live benchmark"),
        elapsed_s=time.monotonic()-started,
        scope="Same recorded quaternion inputs and saved official calibration; not a refit, hardware test or anatomical validation",
        acceptance="Unavailable observations/transitions are reported as coverage losses, never counted as eliminated jumps; inspect recovery changes too")
    write_json(output / "source_manifest.json", manifest)
    write_json(output / "jump_events.json", dict(recorded_negative_jumps=old_jumps,
        replay_absolute_jumps=new_jumps, recoveries_after_unavailability=recoveries))
    write_json(output / "report.json", report)
    print(json.dumps(dict(report=str(output / "report.json"), frames=len(matched), slots=summaries), indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile-version", type=int, default=4)
    parser.add_argument("--start", type=float, default=102881.516)
    parser.add_argument("--end", type=float, default=103195.218)
    parser.add_argument("--slots", type=lambda text: [int(value) for value in text.split(",")], default=[2, 5, 8, 11, 15])
    parser.add_argument("--jump-deg", type=float, default=30.)
    parser.add_argument("--source-manifest", type=Path)
    args = parser.parse_args()
    if args.end <= args.start or args.jump_deg <= 0 or not args.slots or len(set(args.slots)) != len(args.slots):
        parser.error("invalid time range, jump threshold or slot selection")
    replay(args)


if __name__ == "__main__":
    main()
