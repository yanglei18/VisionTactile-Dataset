"""Bounded, read-only extraction of ordered within-frame IMU pairs.

Wire order and equal host parse times are necessary software consistency checks,
not proof of simultaneous device sampling. No interpolation, bias estimation,
calibration, recording or device access is performed here.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import stat

import numpy as np

from .config import fingerprint, validate_config
from .hinge_settings import GYRO_OVERRIDES, effective_gyro_scales, gyro_overrides
from .diagnostics import MAX_BATCH_SAMPLES, MAX_LINE_BYTES, _bounded_json, _reject_constant, _unique_object
from .records import Sample
from .stream import CSV_FIELDS


DEFAULT_BYTE_LIMIT = 64 * 1024 * 1024
_SAMPLE_FIELDS = {"node_id", "values", "host_monotonic_s", "host_wall_s", "status"}


def _number(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
    return float(value)


class _Prefix:
    """Freeze file length at open; hash exactly the bytes actually read."""
    def __init__(self, stream, byte_limit):
        self.stream = stream
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise ValueError("pair capture source must be a regular file")
        self.size, self.inode = info.st_size, info.st_ino
        self.requested = byte_limit
        self.limit = min(self.size, DEFAULT_BYTE_LIMIT if byte_limit is None else byte_limit)
        self.read_bytes = self.complete_bytes = self.line = self.partial_bytes = 0
        self.digest = hashlib.sha256()
        self.stop_reason = None

    def __iter__(self):
        return self

    def __next__(self):
        remaining = self.limit - self.read_bytes
        if remaining <= 0:
            self.stop_reason = ("eof" if self.limit == self.size else
                                "default_byte_limit" if self.requested is None else "byte_limit")
            raise StopIteration
        data = self.stream.readline(min(MAX_LINE_BYTES + 1, remaining))
        if not data:
            raise ValueError("source was truncated during fixed-prefix extraction")
        self.read_bytes += len(data)
        self.digest.update(data)
        if len(data) > MAX_LINE_BYTES:
            raise ValueError(f"line {self.line + 1}: exceeds {MAX_LINE_BYTES} bytes")
        if not data.endswith(b"\n"):
            self.partial_bytes = len(data)
            if self.requested is None and self.limit == self.size:
                raise ValueError(f"line {self.line + 1}: truncated line (missing newline)")
            self.stop_reason = "partial_line_at_byte_limit"
            raise StopIteration
        self.complete_bytes = self.read_bytes
        self.line += 1
        try:
            return data.decode("utf-8-sig" if self.line == 1 else "utf-8")
        except UnicodeError as exc:
            raise ValueError(f"line {self.line}: invalid UTF-8") from exc


def _csv_samples(prefix, metadata):
    reader = csv.reader(prefix, strict=True)
    try:
        fields = next(reader, None)
        if fields is None:
            raise ValueError("CSV has no complete header")
        required = set(CSV_FIELDS[:-1])
        if len(fields) != len(set(fields)):
            raise ValueError("CSV header contains duplicate fields")
        missing, extra = required - set(fields), set(fields) - required - {"status", "phase"}
        if missing or extra:
            raise ValueError(f"CSV header: missing fields {sorted(missing)}; unexpected fields {sorted(extra)}")
        metadata["source_format"] = "csv"
        for values in reader:
            if not values:
                continue
            if len(values) != len(fields):
                raise ValueError("CSV row has a different field count from the header")
            row = dict(zip(fields, values))
            status = row.get("status", "").strip()
            yield Sample(int(row["node_id"]), tuple(int(row[k]) for k in CSV_FIELDS[3:9]),
                         float(row["host_parse_monotonic_s"]), float(row["host_parse_wall_s"]),
                         int(status) if status else None)
    except (ValueError, TypeError, csv.Error, OverflowError) as exc:
        raise ValueError(f"line {reader.line_num}: {exc}") from exc


def _jsonl_samples(prefix, metadata):
    sequence, ended = 0, False
    for line in prefix:
        try:
            event = json.loads(line, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
            _bounded_json(event)
            if type(event) is not dict or type(event.get("seq")) is not int or event["seq"] != sequence:
                raise ValueError("missing or discontinuous diagnostic event sequence")
            if ended:
                raise ValueError("unexpected records after diagnostic end")
            kind = event.get("type")
            metadata["last_event_seq"] = sequence
            sequence += 1
            if sequence == 1:
                if (kind != "header" or event.get("kind") != "vt_glove_pose.diagnostic_session"
                        or type(event.get("schema_version")) is not int or event["schema_version"] != 1
                        or event.get("time_basis") != "host_observed_not_sensor_latency"):
                    raise ValueError("unsupported diagnostic header")
                checkpoint = event.get("checkpoint")
                if type(checkpoint) is not dict:
                    raise ValueError("missing diagnostic checkpoint")
                config = validate_config(checkpoint.get("config"))
                if checkpoint.get("config_fingerprint") != fingerprint(config):
                    raise ValueError("diagnostic configuration fingerprint mismatch")
                metadata.update(source_format="diagnostic_jsonl", source_configuration=config,
                                source_config_fingerprint=fingerprint(config))
            elif kind == "batch":
                rows = event.get("samples")
                if type(rows) is not list or len(rows) > MAX_BATCH_SAMPLES:
                    raise ValueError("invalid or oversized diagnostic sample batch")
                if type(event.get("performance", {})) is not dict:
                    raise ValueError("invalid diagnostic performance observations")
                samples = []
                for row in rows:
                    if (type(row) is not dict or not set(row) <= _SAMPLE_FIELDS
                            or not _SAMPLE_FIELDS - {"status"} <= set(row)
                            or type(row.get("values")) is not list):
                        raise ValueError("invalid diagnostic sample fields")
                    samples.append(Sample(**row))
                # Validate the whole bounded event before yielding any sample.
                yield from samples
            elif kind == "end":
                if type(event.get("complete")) is not bool or type(event.get("reason")) is not str:
                    raise ValueError("invalid diagnostic end marker")
                ended = True
                metadata["observed_end_complete"] = event["complete"]
            elif kind in {"snapshot", "command", "checkpoint", "marker"}:
                # These events are structurally bounded, but not executed or
                # interpreted as angle truth. This is extraction, not replay.
                if kind == "snapshot":
                    _number(event.get("now_s"), "snapshot now_s", -1e12, 1e12)
                    if type(event.get("snapshot")) is not dict:
                        raise ValueError("invalid diagnostic snapshot")
                elif kind == "checkpoint":
                    checkpoint = event.get("checkpoint")
                    if (type(checkpoint) is not dict or checkpoint.get("config") != metadata["source_configuration"]
                            or checkpoint.get("config_fingerprint") != metadata["source_config_fingerprint"]):
                        raise ValueError("diagnostic checkpoint configuration changed")
                elif kind == "marker":
                    if event.get("label") not in ("rest", "moving", "returned"):
                        raise ValueError("invalid diagnostic marker")
                else:
                    name, args = event.get("name"), event.get("args")
                    valid = type(args) is dict and (
                        (name == "cancel" and not args)
                        or (name == "neutral" and set(args) == {"duration", "confirm_mount"}
                            and args["duration"] == 5 and type(args["confirm_mount"]) is bool)
                        or (name == "axis" and set(args) == {"duration", "target"}
                            and args["duration"] == 8 and args["target"] in
                            ("palm", "thumb", "index", "middle", "ring", "little", "thumb_common", "thumb_hinge"))
                        or (name == "axis" and set(args) == {"duration", "target", "calibration_method"}
                            and args["duration"] == 8 and args["target"] in ("thumb_common", "thumb_hinge")
                            and args["calibration_method"] == "robust_v2"))
                    if not valid:
                        raise ValueError("invalid diagnostic command")
            else:
                raise ValueError(f"unsupported diagnostic event type: {kind}")
        except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
            raise ValueError(f"line {prefix.line}: {exc}") from exc
    if sequence == 0:
        raise ValueError("diagnostic JSONL has no complete header")


def load_pair_capture(path, proximal=5, distal=6, sample_rate_hz=120.,
                      gyro_dps_per_lsb=.00875, byte_limit=None, start_s=0.,
                      duration_s=None, max_pairs=10000, *, gyro_dps_per_lsb_by_node=None):
    """Return bounded arrays of ordered same-wire-group pairs, without writes.

    ``start_s``/``duration_s`` select [start, end) host elapsed seconds relative
    to the first input sample, not physical sensor time. ``byte_limit`` is an
    absolute fixed file prefix; None applies a 64 MiB safety budget. The first
    reached byte, duration or pair limit stops extraction. Incomplete boundary
    lines are excluded and reported; malformed complete rows raise ValueError.

    Pairing requires consecutive node IDs from the smaller selected ID through
    the larger, plus identical host parse stamps. Both conditions are only
    software evidence. Cross-group pairing is explicitly unsupported. No data
    are interpolated, re-timed, bias-corrected or silently scale-corrected.
    Returned gyro is assumed deg/s (not SI rad/s), accelerometer remains LSB.
    Optional per-node scales are explicit assumptions; no recorded settings are
    inferred or changed. Effective scales are frozen into the capture metadata.
    """
    for name, value in (("proximal", proximal), ("distal", distal)):
        if type(value) is not int or not 1 <= value <= 12:
            raise ValueError(f"{name} must be an integer node ID in 1..12")
    if proximal == distal:
        raise ValueError("proximal and distal must be distinct nodes")
    if (proximal-1)//6 != (distal-1)//6:
        raise ValueError("cross-group pair has no physical pairing evidence; select nodes within one wire group")
    rate = _number(sample_rate_hz, "sample_rate_hz", 10, 2000)
    scale = _number(gyro_dps_per_lsb, "gyro_dps_per_lsb", 1e-6, 10)
    units = dict(gyro_dps_per_lsb=scale)
    if gyro_dps_per_lsb_by_node is not None:
        units[GYRO_OVERRIDES] = gyro_overrides(gyro_dps_per_lsb_by_node)
    prox_scale, dist_scale = effective_gyro_scales(units, (proximal, distal))
    start_s = _number(start_s, "start_s", 0, 1e12)
    duration = None if duration_s is None else _number(duration_s, "duration_s", 1e-12, 1e12)
    if byte_limit is not None and (type(byte_limit) is not int or not 1 <= byte_limit <= 2**63-1):
        raise ValueError("byte_limit must be a positive integer")
    if type(max_pairs) is not int or not 1 <= max_pairs <= 1_000_000:
        raise ValueError("max_pairs must be an integer in 1..1000000")
    path = Path(path)
    if path.suffix.lower() not in {".csv", ".jsonl", ".ndjson"}:
        raise ValueError("pair input must be .csv, .jsonl or .ndjson")
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError("pair capture source must be a regular file")
    metadata = dict(source=str(path), proximal=proximal, distal=distal,
                    sample_rate_hz=rate, gyro_dps_per_lsb=scale, start_s=start_s, duration_s=duration,
                    max_pairs=max_pairs, requested_byte_limit=byte_limit,
                    time_basis="host_observed_not_sensor_latency", sample_rate_verified=False,
                    units_verified=False, physical_synchronization_verified=False,
                    pairing_basis="consecutive_wire_node_order_and_equal_host_parse_time",
                    input_samples=0, candidate_starts=0, matched_pairs=0,
                    dropped_order_candidates=0, dropped_timestamp_candidates=0,
                    unmatched_high_records=0, unmatched_low_records_at_stop=0,
                    pairs_before_window=0, last_event_seq=None, observed_end_complete=None,
                    validation_scope="sample schema and event envelope, not full diagnostic replay or anatomical truth")
    if gyro_dps_per_lsb_by_node is not None:
        metadata[GYRO_OVERRIDES] = {str(proximal): prox_scale, str(distal): dist_scale}
    output, last_times, pending = [], {}, None
    low, high = sorted((proximal, distal))
    first_time = None
    stop_reason = None
    with path.open("rb") as stream:
        prefix = _Prefix(stream, byte_limit)
        samples = _csv_samples(prefix, metadata) if path.suffix.lower() == ".csv" else _jsonl_samples(prefix, metadata)
        try:
            for sample in samples:
                node, t = sample.node_id, sample.host_monotonic_s
                if node in last_times and t < last_times[node]:
                    raise ValueError(f"line {prefix.line}: nonmonotonic host timestamp for node {node}")
                last_times[node] = t
                metadata["input_samples"] += 1
                if first_time is None:
                    first_time = t
                if duration is not None and t >= first_time + start_s + duration:
                    stop_reason = "duration"
                    break
                if pending is not None:
                    first, previous = pending
                    if node != previous+1 or node > high:
                        metadata["dropped_order_candidates"] += 1
                        pending = None
                    else:
                        pending = (first, node)
                        if t != first.host_monotonic_s:
                            metadata["dropped_timestamp_candidates"] += 1
                            pending = None
                            continue
                        if node == high:
                            pending = None
                            metadata["matched_pairs"] += 1
                            if t < first_time + start_s:
                                metadata["pairs_before_window"] += 1
                                continue
                            output.append((first, sample) if proximal == low else (sample, first))
                            if len(output) == max_pairs:
                                stop_reason = "max_pairs"
                                break
                            continue
                if node == low:
                    pending = (sample, node)
                    metadata["candidate_starts"] += 1
                elif node == high:
                    metadata["unmatched_high_records"] += 1
        finally:
            samples.close()
        if pending is not None:
            metadata["unmatched_low_records_at_stop"] = 1
        metadata.update(source_size_bytes=prefix.size, source_inode=prefix.inode,
                        effective_byte_limit=prefix.limit, bytes_read=prefix.read_bytes,
                        complete_line_bytes=prefix.complete_bytes, partial_line_bytes_ignored=prefix.partial_bytes,
                        sha256_read_prefix=prefix.digest.hexdigest(), complete_lines=prefix.line,
                        first_input_host_monotonic_s=first_time,
                        stop_reason=stop_reason or prefix.stop_reason or "eof", pairs=len(output))
    raw = np.asarray([[p.values, d.values] for p,d in output], dtype=np.int16).reshape(-1,2,6)
    metadata["dropped_pair_candidates"] = (metadata["dropped_order_candidates"] +
                                           metadata["dropped_timestamp_candidates"] + metadata["unmatched_low_records_at_stop"])
    return dict(prox_gyro_raw=raw[:,0,:3].copy(), dist_gyro_raw=raw[:,1,:3].copy(),
                prox_gyro_dps=raw[:,0,:3].astype(float)*prox_scale, dist_gyro_dps=raw[:,1,:3].astype(float)*dist_scale,
                prox_acc_raw=raw[:,0,3:].copy(), dist_acc_raw=raw[:,1,3:].copy(),
                near_limit_mask=np.any(np.abs(raw[:,:,:3].astype(np.int32))>=32700,axis=2),
                host_monotonic_s=np.asarray([[p.host_monotonic_s,d.host_monotonic_s] for p,d in output],dtype=float).reshape(-1,2),
                host_wall_s=np.asarray([[p.host_wall_s,d.host_wall_s] for p,d in output],dtype=float).reshape(-1,2),
                status=np.asarray([[-1 if s.status is None else s.status for s in pair] for pair in output],dtype=np.int16).reshape(-1,2),
                metadata=metadata)
