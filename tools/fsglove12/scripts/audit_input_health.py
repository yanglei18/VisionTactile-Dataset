#!/usr/bin/env python3
"""Bounded, hardware-free raw replay at 30/120 Hz publication cadences.

Only recorded rows advance fusion. Receive timestamps, six raw counts, status,
ordering and the nominal VQF sample rate are preserved. A byte-offset window
starts with fresh fusion/health state; it does not reconstruct earlier history.
Input health is explicitly enabled for this audit by default, never written
back to the recording's configuration. No calibration, model or serial opens.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import csv
import hashlib
from itertools import groupby
import json
import math
from pathlib import Path

from vt_fsglove12.live_stream import CSV_FIELDS, OnlineFusion, RawSample


MAX_BYTES = 50_000_000
MAX_SAMPLES = 50_000


class BoundedRawWindow:
    """Stream complete recorded CSV rows from a fixed, bounded file window."""

    def __init__(self, path, *, start_byte=0, max_bytes=2_000_000,
                 max_samples=12_000, max_duration_s=10.):
        self.path = Path(path)
        if not self.path.is_file():
            raise ValueError("raw input must be a regular recorded file, never a device")
        if type(start_byte) is not int or start_byte < 0:
            raise ValueError("start_byte must be a nonnegative integer")
        if type(max_bytes) is not int or not 256 <= max_bytes <= MAX_BYTES:
            raise ValueError(f"max_bytes must be between 256 and {MAX_BYTES}")
        if type(max_samples) is not int or not 1 <= max_samples <= MAX_SAMPLES:
            raise ValueError(f"max_samples must be between 1 and {MAX_SAMPLES}")
        if isinstance(max_duration_s, bool) or not math.isfinite(max_duration_s) or not 0 < max_duration_s <= 60.:
            raise ValueError("max_duration_s must be finite, positive and at most 60")
        self.start_byte, self.max_bytes = start_byte, max_bytes
        self.max_samples, self.max_duration_s = max_samples, max_duration_s
        self.summary = dict(path=str(self.path.resolve()), snapshot_file_bytes=self.path.stat().st_size,
            requested_start_byte=start_byte, max_bytes=max_bytes, max_samples=max_samples,
            max_duration_s=max_duration_s, bytes_read=0, replayed_rows=0,
            skipped_partial_prefix_bytes=0, stop_reason=None)

    def __iter__(self):
        digest = hashlib.sha256()
        first = previous = None
        with self.path.open("rb") as stream:
            def read_line():
                remaining = min(self.max_bytes - self.summary["bytes_read"],
                    self.summary["snapshot_file_bytes"] - stream.tell())
                if remaining <= 0:
                    self.summary["stop_reason"] = ("byte_limit" if self.summary["bytes_read"] >= self.max_bytes
                                                   else "snapshot_eof")
                    return None
                line = stream.readline(remaining)
                self.summary["bytes_read"] += len(line)
                if not line or not line.endswith(b"\n"):
                    self.summary["stop_reason"] = "incomplete_or_bounded_tail"
                    return None
                return line

            header = read_line()
            if header is None:
                raise ValueError("recorded CSV has no complete header within the byte limit")
            fields = next(csv.reader([header.decode("utf-8-sig")], strict=True))
            required = set(CSV_FIELDS[:-1])
            if (len(fields) != len(set(fields)) or not required <= set(fields)
                    or set(fields) - required - {"status", "phase"}):
                raise ValueError("raw CSV requires receive timestamps, node ID, six counts and optional status/phase")
            self.summary["header_sha256"] = hashlib.sha256(header).hexdigest()
            if self.start_byte > stream.tell():
                # Seek one byte earlier to distinguish a row boundary from a
                # mid-row offset. Never scan the skipped prefix of a huge file.
                stream.seek(self.start_byte - 1)
                boundary = stream.read(1)
                self.summary["bytes_read"] += len(boundary)
                if boundary != b"\n":
                    discarded = read_line()
                    self.summary["skipped_partial_prefix_bytes"] = len(discarded or b"")
                    if discarded is None:
                        self.summary["replayed_rows_sha256"] = digest.hexdigest()
                        return
            self.summary["actual_start_byte"] = stream.tell()
            while self.summary["replayed_rows"] < self.max_samples:
                line = read_line()
                if line is None:
                    break
                values = next(csv.reader([line.decode("utf-8")], strict=True))
                if len(values) != len(fields):
                    raise ValueError(f"wrong raw CSV field count near byte {stream.tell()}")
                row = dict(zip(fields, values))
                status = row.get("status", "").strip()
                sample = RawSample(int(row["node_id"]), tuple(int(row[key]) for key in CSV_FIELDS[3:9]),
                    float(row[CSV_FIELDS[0]]), float(row[CSV_FIELDS[1]]), int(status) if status else None)
                stamp = sample.host_monotonic_s
                first = stamp if first is None else first
                if previous is not None and stamp < previous:
                    raise ValueError("recorded receive clock moves backward; select a monotonic window, never reorder rows")
                if stamp - first > self.max_duration_s:
                    self.summary["stop_reason"] = "duration_limit"
                    break
                previous = stamp
                digest.update(line)
                self.summary["replayed_rows"] += 1
                yield sample
            else:
                self.summary["stop_reason"] = "sample_limit"
            self.summary.update(replayed_rows_sha256=digest.hexdigest(),
                first_timestamp_s=first, last_timestamp_s=previous, stopped_at_byte=stream.tell())


def _comparable(frame):
    return dict(nodes={node: {key: value.get(key) for key in
                ("quat_xyzw", "epoch", "sample_index", "orientation_valid", "input_health")}
                for node, value in frame.get("nodes", {}).items()},
                input_health=frame.get("input_health"), input_observation=frame.get("input_observation"))


def _count_real_filter_updates(engine):
    """Wrap the existing factory: count real VQF calls, never replace its math."""
    factory, counts = engine._factory, dict(updates=0, instances=0)

    class CountedVQF:
        def __init__(self, period):
            self.original = factory(period)
            counts["instances"] += 1

        def update(self, *args, **kwargs):
            counts["updates"] += 1
            return self.original.update(*args, **kwargs)

        def __getattr__(self, name):
            return getattr(self.original, name)

    engine._factory = CountedVQF
    counts["health_transitions"] = []
    health = getattr(engine, "input_health", None)
    if health is not None:
        observe = health.observe

        def observe_transitions(*args, **kwargs):
            transitions = observe(*args, **kwargs)
            stamp = args[3] if len(args) > 3 else kwargs["stamp"]
            for event, node in transitions:
                status = health.node_status(node, stamp)
                counts["health_transitions"].append(dict(event=event, node=node, timestamp_s=stamp,
                    reasons=status["reasons"], observed_receive_rate_hz=status["observed_receive_rate_hz"]))
            return transitions

        health.observe = observe_transitions
    return counts


def audit_replay(raw_path, config, *, start_byte=0, max_bytes=2_000_000,
                 max_samples=12_000, max_duration_s=10., input_health="enabled"):
    """Replay one recorded stream into two independently initialized VQFs."""
    if input_health not in {"enabled", "disabled", "recorded"}:
        raise ValueError("input_health must be enabled, disabled or recorded")
    replay_config = deepcopy(config)
    if input_health != "recorded":
        replay_config["input_health"] = {**replay_config.get("input_health", {}),
                                         "enabled": input_health == "enabled"}
    engines = {rate: OnlineFusion({**replay_config, "output_rate_hz": float(rate)}) for rate in (30, 120)}
    filter_calls = {rate: _count_real_filter_updates(engine) for rate, engine in engines.items()}
    window = BoundedRawWindow(raw_path, start_byte=start_byte, max_bytes=max_bytes,
                             max_samples=max_samples, max_duration_s=max_duration_s)
    raw_nodes, snapshot_counts = {}, Counter()
    matched, mismatches, max_quat_difference = 0, [], 0.
    anchor = last_stamp = None
    tick = 0

    def compare(first, second, stamp):
        nonlocal matched, max_quat_difference
        matched += 1
        left, right = _comparable(first), _comparable(second)
        if left != right and len(mismatches) < 8:
            mismatches.append(dict(timestamp_s=stamp,
                changed_sections=[key for key in left if left[key] != right[key]]))
        for node in set(left["nodes"]) & set(right["nodes"]):
            a, b = left["nodes"][node]["quat_xyzw"], right["nodes"][node]["quat_xyzw"]
            if a is not None and b is not None:
                max_quat_difference = max(max_quat_difference, max(abs(x - y) for x, y in zip(a, b)))

    def publish(stamp):
        value = engines[120].snapshot(stamp)
        snapshot_counts[120] += 1
        if tick % 4 == 0:
            other = engines[30].snapshot(stamp)
            snapshot_counts[30] += 1
            compare(other, value, stamp)

    for stamp, batch_iter in groupby(window, key=lambda sample: sample.host_monotonic_s):
        batch = list(batch_iter)
        if anchor is None:
            anchor = stamp
        # Finish all earlier publication ticks, then update the entire group
        # at its original receive timestamp before publishing that same time.
        while anchor + tick / 120. < stamp:
            publish(anchor + tick / 120.)
            tick += 1
        for sample in batch:
            key = str(sample.node_id)
            state = raw_nodes.setdefault(key, dict(count=0, first=stamp, last=stamp,
                unique=set(), previous=None, repeated=0, run=0, max_run=0))
            signature = (*sample.values, sample.status)
            state["count"] += 1
            state["last"] = stamp
            state["unique"].add(signature)
            equal = signature == state["previous"]
            state["repeated"] += equal
            state["run"] = state["run"] + 1 if equal else 1
            state["max_run"] = max(state["max_run"], state["run"])
            state["previous"] = signature
        for engine in engines.values():
            engine.update(batch)
        last_stamp = stamp
    if last_stamp is None:
        raise ValueError("bounded raw window contains no complete replayable samples")
    while anchor + tick / 120. <= last_stamp:
        publish(anchor + tick / 120.)
        tick += 1
    final = {rate: engine.input_snapshot(last_stamp) for rate, engine in engines.items()}
    if input_health == "enabled" and not final[120].get("input_health", {}).get("enabled"):
        raise RuntimeError("fusion did not expose enabled input health; protection was not validated")
    compare(final[30], final[120], last_stamp)
    received = {node: state["count"] for node, state in raw_nodes.items()}
    receipt_counts_preserved = all(all(engine.counts[node] == count for node, count in received.items())
                                   for engine in engines.values())
    return dict(schema_version=1, kind="bounded_input_health_cadence_audit", hardware_accessed=False,
        source=window.summary, nominal_sample_rate_hz=config.get("sample_rate_hz"),
        configuration_overrides=dict(input_health_mode=input_health, snapshot_rates_hz=[30, 120],
            sample_rate_changed=False, receive_timestamps_changed=False, raw_values_changed=False),
        received_rows=sum(received.values()), synthetic_input_rows=0, receipt_counts_preserved=receipt_counts_preserved,
        raw_nodes={node: dict(received_samples=s["count"], receive_span_s=s["last"] - s["first"],
            observed_receive_rate_hz=((s["count"] - 1) / (s["last"] - s["first"])
                                      if s["last"] > s["first"] else None),
            unique_six_axis_and_status=len(s["unique"]), repeated_payload_samples=s["repeated"],
            longest_equal_payload_run=s["max_run"]) for node, s in raw_nodes.items()},
        cadence_comparison=dict(snapshot_counts=dict(snapshot_counts), matched_checks=matched,
            invariant=not mismatches, mismatch_examples=mismatches,
            max_quaternion_component_difference=max_quat_difference,
            compares=["quaternion", "epoch", "sample_index", "orientation_valid", "input_health", "receive_observation"],
            excludes=["snapshot_frame_index", "consumed_one_shot_warnings"]),
        final={str(rate): dict(received_counts=engine.counts, epochs=engine.epochs,
            input_health=final[rate].get("input_health"),
            actual_vqf_update_calls=filter_calls[rate]["updates"],
            actual_vqf_instances=filter_calls[rate]["instances"],
            health_transitions=filter_calls[rate]["health_transitions"],
            raw_rows_not_integrated=sum(received.values()) - filter_calls[rate]["updates"],
            stored_quaternions={key: value["quat_xyzw"] for key, value in engine.states.items()},
            valid_orientation_nodes=[key for key, value in final[rate].get("nodes", {}).items()
                                     if value.get("orientation_valid", True)],
            input_filter_rejected_samples=engine.input_guard_rejections)
            for rate, engine in engines.items()},
        limitations=["Observed host arrival cadence is not verified device ODR.",
            "An offset window starts fresh state; earlier calibration/fusion history is not replayed.",
            "Matching publication cadences proves snapshot invariance, not anatomical accuracy.",
            "Repeated decoded counts do not identify a specific hardware or firmware failure."])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("--config", type=Path, help="defaults to config.json beside raw.csv")
    parser.add_argument("--start-byte", type=int, default=0)
    parser.add_argument("--max-bytes", type=int, default=2_000_000)
    parser.add_argument("--max-samples", type=int, default=12_000)
    parser.add_argument("--max-duration-s", type=float, default=10.)
    parser.add_argument("--input-health", choices=("enabled", "disabled", "recorded"), default="enabled")
    args = parser.parse_args()
    config_path = args.config or args.raw.parent / "config.json"
    if not config_path.is_file() or config_path.stat().st_size > 1_000_000:
        parser.error("config must be a regular JSON file smaller than 1 MB")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    result = audit_replay(args.raw, config, start_byte=args.start_byte, max_bytes=args.max_bytes,
        max_samples=args.max_samples, max_duration_s=args.max_duration_s, input_health=args.input_health)
    result["configuration_source"] = str(config_path.resolve())
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))


if __name__ == "__main__":
    main()
