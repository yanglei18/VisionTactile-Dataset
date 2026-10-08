#!/usr/bin/env python3
"""Audit the latest host-time window of an existing raw IMU CSV, offline only.

Read only the file-size prefix present at open, excluding an incomplete final
line. Preserve selected CSV rows byte-for-byte. This is decoded CSV evidence,
not original serial bytes; no reconstruction, device access, or filtering.
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys


AXES = ("gx", "gy", "gz", "ax", "ay", "az")
TIME_FIELD = "host_parse_monotonic_s"
REQUIRED = (TIME_FIELD, "node_id", *(axis + "_lsb" for axis in AXES))
MAX_INPUT_BYTES = 64 * 1024 * 1024
MAX_SELECTED_ROWS = 1_000_000


def _csv_row(raw, line_number):
    try:
        return next(csv.reader([raw.decode("utf-8")], strict=True))
    except (UnicodeError, csv.Error, StopIteration) as exc:
        raise ValueError(f"invalid CSV at source line {line_number}: {exc}") from exc


def _snapshot_size(source):
    metadata = os.fstat(source.fileno())
    if not stat.S_ISREG(metadata.st_mode):
        raise ValueError("--raw-csv must be a regular saved file, not a device or pipe")
    if metadata.st_size > MAX_INPUT_BYTES:
        raise ValueError(f"source prefix exceeds {MAX_INPUT_BYTES} byte limit")
    return metadata.st_size


def _read_window(path, duration):
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError("--raw-csv must be a regular saved file, not a device or pipe")
    selected = deque()
    hasher = hashlib.sha256()
    complete_rows = partial_bytes = 0
    previous = first_stamp = None
    with path.open("rb") as source:
        prefix_size = remaining = _snapshot_size(source)
        header = source.readline(remaining)
        hasher.update(header)
        remaining -= len(header)
        if not header.endswith(b"\n"):
            raise ValueError("source requires a complete newline-terminated CSV header")
        fields = _csv_row(header, 1)
        if len(set(fields)) != len(fields) or any(name not in fields for name in REQUIRED):
            raise ValueError("CSV header requires unique names, host_parse_monotonic_s, node_id, and six *_lsb fields")
        while remaining:
            raw = source.readline(remaining)
            if not raw:
                raise ValueError("source was truncated while reading its initial-size prefix")
            hasher.update(raw)
            remaining -= len(raw)
            if not raw.endswith(b"\n"):
                partial_bytes = len(raw)
                break
            line_number = complete_rows + 2
            columns = _csv_row(raw, line_number)
            if len(columns) != len(fields):
                raise ValueError(f"wrong CSV field count at source line {line_number}")
            row = dict(zip(fields, columns))
            try:
                stamp = float(row[TIME_FIELD])
                if not math.isfinite(stamp):
                    raise ValueError("host monotonic timestamp must be finite")
                if "host_parse_wall_s" in row and not math.isfinite(float(row["host_parse_wall_s"])):
                    raise ValueError("host wall timestamp must be finite")
                node = int(row["node_id"])
                if not 1 <= node <= 12:
                    raise ValueError("node_id must be in 1..12")
                # Deliberately no int16 cast, signed-range rejection, or clamp.
                values = tuple(int(row[axis + "_lsb"]) for axis in AXES)
                if previous is not None and stamp < previous:
                    raise ValueError("host monotonic timestamps decreased")
            except ValueError as exc:
                raise ValueError(f"invalid CSV at source line {line_number}: {exc}") from exc
            complete_rows += 1
            first_stamp = stamp if first_stamp is None else first_stamp
            previous = stamp
            selected.append((raw, line_number, stamp, row[TIME_FIELD], node, values))
            while selected and selected[0][2] < stamp - duration:
                selected.popleft()
            if len(selected) > MAX_SELECTED_ROWS:
                raise ValueError(f"active selected time window exceeds {MAX_SELECTED_ROWS} row limit")
    if previous is None:
        raise ValueError("source has no complete data rows")
    return header, selected, dict(source_prefix_bytes=prefix_size, source_prefix_sha256=hasher.hexdigest(),
        source_complete_data_rows=complete_rows, source_incomplete_final_row_bytes_excluded=partial_bytes,
        source_first_complete_host_monotonic_s=first_stamp, source_last_complete_host_monotonic_s=previous,
        window_start_host_monotonic_s=previous - duration, window_end_host_monotonic_s=previous,
        malformed_complete_rows=0, timestamp_decreases=0)


def _new_axis(node, axis):
    return dict(node_id=node, axis=axis, samples=0, signed_min=None, signed_max=None, max_abs=None,
                exact_minus_32768_count=0, exact_plus_32767_count=0,
                near_boundary_abs_ge_32760_count=0, outside_signed16_count=0)


def analyze_csv(raw_csv, output_dir, *, last_seconds=30):
    if isinstance(last_seconds, bool) or not isinstance(last_seconds, (int, float)) or not math.isfinite(last_seconds) or last_seconds <= 0:
        raise ValueError("last_seconds must be finite and positive")
    raw_csv, output_dir = Path(raw_csv), Path(output_dir)
    header, selected, source_info = _read_window(raw_csv, last_seconds)
    output_dir.mkdir(exist_ok=False)
    stats = {(node, axis): _new_axis(node, axis) for node in range(1, 13) for axis in AXES}
    nodes = {str(node): dict(samples=0, first_host_monotonic_s=None, last_host_monotonic_s=None) for node in range(1, 13)}
    sample_hash = hashlib.sha256(header)
    events_count = 0
    event_fields = ("source_line_number", "selected_sample_sequence", TIME_FIELD, "node_id", "axis",
                    "value_lsb", "near_boundary_abs_ge_32760", "outside_signed16", "exact_signed_endpoint")
    with (output_dir / "samples.csv").open("xb") as samples_stream, \
         (output_dir / "boundary_events.csv").open("x", encoding="utf-8", newline="") as events_stream:
        samples_stream.write(header)
        writer = csv.DictWriter(events_stream, fieldnames=event_fields)
        writer.writeheader()
        for sequence, (raw, line, stamp, stamp_text, node, values) in enumerate(selected, 1):
            samples_stream.write(raw)
            sample_hash.update(raw)
            node_info = nodes[str(node)]
            node_info["samples"] += 1
            if node_info["first_host_monotonic_s"] is None:
                node_info["first_host_monotonic_s"] = stamp
            node_info["last_host_monotonic_s"] = stamp
            for axis, value in zip(AXES, values):
                info = stats[node, axis]
                info["samples"] += 1
                info["signed_min"] = value if info["signed_min"] is None else min(info["signed_min"], value)
                info["signed_max"] = value if info["signed_max"] is None else max(info["signed_max"], value)
                info["max_abs"] = abs(value) if info["max_abs"] is None else max(info["max_abs"], abs(value))
                info["exact_minus_32768_count"] += int(value == -32768)
                info["exact_plus_32767_count"] += int(value == 32767)
                near, outside = abs(value) >= 32760, not -32768 <= value <= 32767
                info["near_boundary_abs_ge_32760_count"] += int(near)
                info["outside_signed16_count"] += int(outside)
                if near or outside:
                    events_count += 1
                    writer.writerow(dict(source_line_number=line, selected_sample_sequence=sequence,
                        host_parse_monotonic_s=stamp_text, node_id=node, axis=axis, value_lsb=value,
                        near_boundary_abs_ge_32760=int(near), outside_signed16=int(outside),
                        exact_signed_endpoint=int(value in (-32768, 32767))))
    for node_info in nodes.values():
        count = node_info["samples"]
        span = node_info["last_host_monotonic_s"] - node_info["first_host_monotonic_s"] if count else None
        node_info["observed_receive_span_s"] = span
        node_info["observed_receive_rate_hz"] = (count - 1) / span if count > 1 and span > 0 else None
    summaries = list(stats.values())
    with (output_dir / "summary.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    count_keys = ("exact_minus_32768_count", "exact_plus_32767_count", "near_boundary_abs_ge_32760_count", "outside_signed16_count")
    report = dict(schema_version=1, kind="vt_fsglove12.offline_decoded_csv_signed16_audit",
        source_csv=str(raw_csv.resolve()), output_dir=str(output_dir.resolve()), last_seconds=last_seconds,
        source_prefix_policy="read only initial fstat size from the open file; later appends ignored; incomplete final newline-less row excluded; concurrent in-place rewrites are not an atomic snapshot",
        source_max_bytes=MAX_INPUT_BYTES, selected_max_rows=MAX_SELECTED_ROWS,
        samples_csv_sha256=sample_hash.hexdigest(), selected_rows=len(selected), selected_scalar_values=len(selected) * 6,
        source_complete_rows_outside_window=source_info["source_complete_data_rows"] - len(selected),
        first_selected_source_line=selected[0][1], last_selected_source_line=selected[-1][1],
        first_selected_host_monotonic_s=selected[0][2], last_selected_host_monotonic_s=selected[-1][2],
        actual_selected_host_span_s=selected[-1][2] - selected[0][2],
        window_selection="inclusive: last complete row host monotonic time minus last_seconds <= row time <= last complete row time",
        nodes=nodes, nodes_missing=[int(node) for node, info in nodes.items() if info["samples"] == 0],
        axis_summary=summaries, totals={key: sum(info[key] for info in summaries) for key in count_keys},
        boundary_event_count=events_count,
        signed_observed_min=min(info["signed_min"] for info in summaries if info["samples"]),
        signed_observed_max=max(info["signed_max"] for info in summaries if info["samples"]),
        theoretical_bounds=dict(signed_min=-32768, signed_max=32767),
        evidence_domain="decoded raw CSV samples recorded before fusion/input filtering; not original serial bytes",
        evidence_basis=[
            "tools/fsglove12/src/vt_fsglove12/live_stream.py: RawRecorder.write emits node and *sample.values directly.",
            "tools/fsglove12/src/vt_fsglove12/live_runtime.py: session.recorder.write(samples) is called before session.fusion.update(samples).",
        ],
        hardware_accessed=False, source_modified=False, wire_bytes_available=False, wire_reconstruction_performed=False,
        samples_preservation="exact original CSV header and selected complete row bytes, including their original line endings",
        malformed_row_policy="abort on any malformed complete source row or timestamp decrease, including outside selected window",
        value_parsing="Python int from CSV text; no int16 cast, range rejection, clamping, scaling, or filtering",
        receive_rate_basis="(samples - 1) / per-node host timestamp span; host receive/parse timing, not device ODR",
        interpretation=[
            "Outside-signed16 count is an actual check of CSV integers, not zero by construction; source production parser/RawSample uses int16 values.",
            "No observed out-of-range CSV values does not exclude firmware arithmetic or pre-pack overflow, nor certify original wire integrity.",
            "Endpoints and near-boundary values alone do not establish physical sensor saturation or firmware overflow.",
            "This analyzes an existing recording; it does not independently re-record serial bytes or guarantee a fresh device timestamp.",
        ], **source_info)
    with (output_dir / "report.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-csv", required=True)
    parser.add_argument("--last-seconds", type=float, default=30)
    parser.add_argument("--output-dir", required=True, help="new exclusive directory")
    args = parser.parse_args(argv)
    try:
        report = analyze_csv(args.raw_csv, args.output_dir, last_seconds=args.last_seconds)
    except (OSError, ValueError, TypeError) as exc:
        print(f"analyze_imu_csv: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({key: report[key] for key in (
        "output_dir", "source_prefix_bytes", "source_prefix_sha256", "selected_rows", "nodes_missing",
        "window_start_host_monotonic_s", "window_end_host_monotonic_s", "signed_observed_min", "signed_observed_max", "totals")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
