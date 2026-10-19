#!/usr/bin/env python3
"""Offline, integer-only signed-16 audit of a saved twelve-node IMU wire file.

No serial dependencies, device discovery, hardware access, scaling, filtering,
or synthesized timestamps. Input is a regular file bounded to 4 MiB. Every saved
byte is examined, independently of the capture probe's parser sample limit.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import stat
import sys


FRAME_BYTES = 84
BLOCK_BYTES = 14
MAX_BYTES = 4 * 1024 * 1024
CHUNK_BYTES = 4096
AXES = ("gx", "gy", "gz", "ax", "ay", "az")
SIGNED_MIN = -32768
SIGNED_MAX = 32767
NEAR_ABS = 32760


def decode_word(low, high):
    """Manual little-endian decoding into unbounded Python integers."""
    if type(low) is not int or type(high) is not int or not (0 <= low <= 255 and 0 <= high <= 255):
        raise ValueError("each input byte must be an integer in [0, 255]")
    unsigned = low + high * 256
    signed = unsigned - 65536 if unsigned >= 32768 else unsigned
    return signed, unsigned


def iter_frames(chunks, accounting):
    """Yield (absolute byte offset, frame); chunk boundaries carry no meaning."""
    buffer = bytearray()
    base = 0
    accounting.update(bytes_discarded_during_framing=0, unparsed_tail_bytes=0)
    for chunk in chunks:
        buffer.extend(chunk)
        cursor = 0
        while len(buffer) - cursor >= FRAME_BYTES:
            first = buffer[cursor]
            if first not in (1, 7) or any(
                buffer[cursor + BLOCK_BYTES * index] != first + index for index in range(6)
            ):
                cursor += 1
                accounting["bytes_discarded_during_framing"] += 1
                continue
            yield base + cursor, bytes(buffer[cursor:cursor + FRAME_BYTES])
            cursor += FRAME_BYTES
        del buffer[:cursor]
        base += cursor
    accounting["unparsed_tail_bytes"] = len(buffer)
    accounting["unparsed_tail_wire_offset"] = base
    accounting["unparsed_tail_hex"] = buffer.hex()


def _new_axis(node, axis):
    return dict(node_id=node, axis=axis, samples=0, signed_min=None, signed_max=None,
                unsigned_min=None, unsigned_max=None, negative_count=0,
                exact_minus_32768_count=0, exact_plus_32767_count=0,
                near_boundary_abs_ge_32760_count=0, longest_consecutive_near_boundary_run=0,
                outside_signed16_count=0)


def _read_wire(wire_path):
    # A saved file only: reject devices/FIFOs, including symlinks resolving to them.
    if not stat.S_ISREG(wire_path.stat().st_mode):
        raise ValueError("--wire must be a saved regular file, not a device or pipe")
    with wire_path.open("rb") as source:
        data = source.read(MAX_BYTES + 1)
    if len(data) > MAX_BYTES:
        raise ValueError(f"wire file exceeds the offline audit limit of {MAX_BYTES} bytes")
    return data


def _crosscheck_probe(probe_path, digest, byte_count):
    if probe_path is None:
        return dict(requested=False, sha256_match=None)
    probe_path = Path(probe_path)
    with probe_path.open("r", encoding="utf-8") as source:
        report = json.load(source)
    expected = report.get("wire_sha256")
    if not isinstance(expected, str) or len(expected) != 64 or any(c not in "0123456789abcdefABCDEF" for c in expected):
        raise ValueError("probe report must contain a valid wire_sha256")
    if digest != expected.lower():
        raise ValueError("saved wire SHA256 does not match probe report; no analysis outputs created")
    expected_bytes = report.get("wire_bytes_saved")
    if expected_bytes is not None and (type(expected_bytes) is not int or expected_bytes != byte_count):
        raise ValueError("saved wire byte count does not match probe report")
    return dict(requested=True, report_path=str(probe_path.resolve()), sha256_match=True,
                wire_bytes_saved_match=(expected_bytes == byte_count) if expected_bytes is not None else None,
                probe_status=report.get("status"), probe_frames_decoded=report.get("frames_decoded"),
                probe_samples_decoded=report.get("samples_decoded"),
                probe_unparsed_tail_bytes=report.get("unparsed_tail_bytes"),
                probe_wire_complete=report.get("wire_complete"),
                parser_sample_limit_reused=False)


def analyze_wire(wire_path, output_dir=None, *, probe_report=None):
    """Create exclusive report/CSV files; never modify the input evidence."""
    wire_path = Path(wire_path)
    data = _read_wire(wire_path)
    digest = hashlib.sha256(data).hexdigest()
    crosscheck = _crosscheck_probe(probe_report, digest, len(data))
    output_dir = Path(output_dir) if output_dir is not None else wire_path.with_name(wire_path.name + ".analysis")
    output_dir.mkdir(exist_ok=False)
    stats = {(node, axis): _new_axis(node, axis) for node in range(1, 13) for axis in AXES}
    streaks = {key: 0 for key in stats}
    previous = {}
    node_counts = {str(node): 0 for node in range(1, 13)}
    statuses = {str(node): set() for node in range(1, 13)}
    bank_counts = {"1-6": 0, "7-12": 0}
    accounting = {}
    frames = samples = boundary_count = wrap_count = 0
    sample_fields = ["sample_sequence", "node_sequence", "frame_sequence", "block_index",
                     "frame_wire_offset", "block_wire_offset", "node_id", "status"]
    sample_fields += [f"{axis}_signed" for axis in AXES] + [f"{axis}_unsigned" for axis in AXES]
    event_fields = ["sample_sequence", "node_sequence", "frame_sequence", "frame_wire_offset",
                    "block_wire_offset", "axis_wire_offset", "node_id", "status", "axis",
                    "signed_value", "unsigned_value", "wire_bytes_hex_le", "exact_signed_endpoint"]
    wrap_fields = ["node_id", "axis", "previous_sample_sequence", "sample_sequence",
                   "previous_node_sequence", "node_sequence", "previous_frame_sequence", "frame_sequence",
                   "previous_axis_wire_offset", "axis_wire_offset", "previous_signed_value", "signed_value",
                   "raw_signed_jump", "absolute_raw_jump", "previous_wire_bytes_hex_le", "wire_bytes_hex_le",
                   "interpretation"]
    with (output_dir / "samples.csv").open("x", encoding="utf-8", newline="") as samples_stream, \
         (output_dir / "boundary_events.csv").open("x", encoding="utf-8", newline="") as events_stream, \
         (output_dir / "wrap_candidates.csv").open("x", encoding="utf-8", newline="") as wraps_stream:
        sample_writer = csv.DictWriter(samples_stream, fieldnames=sample_fields)
        event_writer = csv.DictWriter(events_stream, fieldnames=event_fields)
        wrap_writer = csv.DictWriter(wraps_stream, fieldnames=wrap_fields)
        for writer in (sample_writer, event_writer, wrap_writer):
            writer.writeheader()
        chunks = (data[offset:offset + CHUNK_BYTES] for offset in range(0, len(data), CHUNK_BYTES))
        for frame_offset, frame in iter_frames(chunks, accounting):
            frames += 1
            bank_counts["1-6" if frame[0] == 1 else "7-12"] += 1
            for block_index in range(6):
                relative = block_index * BLOCK_BYTES
                block_offset = frame_offset + relative
                node, status = frame[relative], frame[relative + 1]
                samples += 1
                node_counts[str(node)] += 1
                statuses[str(node)].add(status)
                node_sequence = node_counts[str(node)]
                sample = dict(sample_sequence=samples, node_sequence=node_sequence,
                              frame_sequence=frames, block_index=block_index,
                              frame_wire_offset=frame_offset, block_wire_offset=block_offset,
                              node_id=node, status=status)
                for index, axis in enumerate(AXES):
                    value_offset = relative + 2 + index * 2
                    signed, unsigned = decode_word(frame[value_offset], frame[value_offset + 1])
                    sample[f"{axis}_signed"], sample[f"{axis}_unsigned"] = signed, unsigned
                    key = node, axis
                    axis_stats = stats[key]
                    axis_stats["samples"] += 1
                    for name, value in (("signed", signed), ("unsigned", unsigned)):
                        low, high = name + "_min", name + "_max"
                        axis_stats[low] = value if axis_stats[low] is None else min(axis_stats[low], value)
                        axis_stats[high] = value if axis_stats[high] is None else max(axis_stats[high], value)
                    axis_stats["negative_count"] += int(signed < 0)
                    axis_stats["exact_minus_32768_count"] += int(signed == SIGNED_MIN)
                    axis_stats["exact_plus_32767_count"] += int(signed == SIGNED_MAX)
                    axis_stats["outside_signed16_count"] += int(not SIGNED_MIN <= signed <= SIGNED_MAX)
                    near = abs(signed) >= NEAR_ABS
                    axis_stats["near_boundary_abs_ge_32760_count"] += int(near)
                    streaks[key] = streaks[key] + 1 if near else 0
                    axis_stats["longest_consecutive_near_boundary_run"] = max(
                        axis_stats["longest_consecutive_near_boundary_run"], streaks[key])
                    axis_offset = frame_offset + value_offset
                    raw_hex = frame[value_offset:value_offset + 2].hex()
                    if near:
                        boundary_count += 1
                        event_writer.writerow(dict(sample_sequence=samples, node_sequence=node_sequence,
                            frame_sequence=frames, frame_wire_offset=frame_offset, block_wire_offset=block_offset,
                            axis_wire_offset=axis_offset, node_id=node, status=status, axis=axis,
                            signed_value=signed, unsigned_value=unsigned, wire_bytes_hex_le=raw_hex,
                            exact_signed_endpoint=int(signed in (SIGNED_MIN, SIGNED_MAX))))
                    prior = previous.get(key)
                    if prior is not None:
                        old = prior["value"]
                        jump = signed - old
                        if old * signed < 0 and abs(old) >= 30000 and abs(signed) >= 30000 and abs(jump) >= 60000:
                            wrap_count += 1
                            wrap_writer.writerow(dict(node_id=node, axis=axis,
                                previous_sample_sequence=prior["sample"], sample_sequence=samples,
                                previous_node_sequence=prior["node_sequence"], node_sequence=node_sequence,
                                previous_frame_sequence=prior["frame"], frame_sequence=frames,
                                previous_axis_wire_offset=prior["offset"], axis_wire_offset=axis_offset,
                                previous_signed_value=old, signed_value=signed, raw_signed_jump=jump,
                                absolute_raw_jump=abs(jump), previous_wire_bytes_hex_le=prior["hex"],
                                wire_bytes_hex_le=raw_hex, interpretation="suspicion_only_not_proof_of_wrap_or_overflow"))
                    previous[key] = dict(value=signed, sample=samples, node_sequence=node_sequence,
                                         frame=frames, offset=axis_offset, hex=raw_hex)
                sample_writer.writerow(sample)
    summaries = list(stats.values())
    with (output_dir / "summary.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    count_names = ("negative_count", "exact_minus_32768_count", "exact_plus_32767_count",
                   "near_boundary_abs_ge_32760_count", "outside_signed16_count")
    observed = [row for row in summaries if row["samples"]]
    report = dict(schema_version=1, kind="vt_fsglove12.offline_imu_signed16_audit",
        wire_path=str(wire_path.resolve()), output_dir=str(output_dir.resolve()), wire_sha256=digest,
        wire_bytes=len(data), input_max_bytes=MAX_BYTES, all_saved_bytes_examined=True,
        capture_parser_sample_limit_applied=False, probe_crosscheck=crosscheck,
        hardware_accessed=False, filtering_applied=False, physical_unit_conversion_applied=False,
        decoding="manual little-endian unsigned = low + 256*high; signed = unsigned - 65536 if unsigned >= 32768 else unsigned; Python integers",
        axis_order=list(AXES), frame_bytes=FRAME_BYTES, node_block_bytes=BLOCK_BYTES,
        framing_validation="six ordered node IDs 1..6 or 7..12 in 84 bytes; no verified CRC/checksum; false-positive framing remains possible",
        status_byte_semantics="unknown; preserved verbatim and never used to discard samples",
        theoretical_bounds=dict(signed_min=SIGNED_MIN, signed_max=SIGNED_MAX, unsigned_min=0, unsigned_max=65535),
        near_boundary_criterion="abs(signed Python integer) >= 32760", frames_decoded=frames,
        samples_decoded=samples, scalar_values_decoded=samples * 6, bank_frames=bank_counts,
        node_sample_counts=node_counts, nodes_missing=[int(node) for node, count in node_counts.items() if count == 0],
        node_status_values={node: sorted(values) for node, values in statuses.items()},
        signed_observed_min=min((row["signed_min"] for row in observed), default=None),
        signed_observed_max=max((row["signed_max"] for row in observed), default=None),
        unsigned_observed_min=min((row["unsigned_min"] for row in observed), default=None),
        unsigned_observed_max=max((row["unsigned_max"] for row in observed), default=None),
        totals={name: sum(row[name] for row in summaries) for name in count_names},
        boundary_event_count=boundary_count, wrap_candidate_count=wrap_count,
        sequence_basis="sample_sequence and frame_sequence are one-based offline decode order; node_sequence is one-based per-node order; block_index and all byte offsets are zero-based",
        timestamps_available=False, timestamp_basis="no timestamps are present in saved wire; none synthesized",
        consecutive_run_basis="adjacent accepted samples for the same node and axis, across intervening bank frames; missing device samples cannot be detected",
        wrap_candidate_criterion="adjacent accepted same-node-axis values have opposite signs, both abs >= 30000, and abs signed difference >= 60000; suspicion only, not proof",
        interpretation=[
            "Outside-signed16 count is zero by construction for any two-byte signed decoding; it does not demonstrate absence of firmware arithmetic or pre-pack overflow.",
            "Unsigned values above 32767 are the same bit patterns as negative signed values, not evidence of overflow.",
            "Exact endpoints, near-boundary runs, and wrap candidates alone do not establish physical sensor saturation or firmware overflow.",
            "Discarded bytes are framing rejects, not a measurement of device-dropped bytes; status and CRC semantics are unverified.",
        ], axis_summary=summaries, **accounting)
    report["byte_accounting_valid"] = (
        frames * FRAME_BYTES + accounting["bytes_discarded_during_framing"] + accounting["unparsed_tail_bytes"] == len(data))
    with (output_dir / "report.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wire", required=True, help="saved raw .bin regular file, at most 4 MiB")
    parser.add_argument("--output-dir", help="new exclusive directory; default: <wire filename>.analysis")
    parser.add_argument("--probe-report", help="optional capture JSON; require SHA256 and optional saved-byte-count match")
    args = parser.parse_args(argv)
    try:
        report = analyze_wire(args.wire, args.output_dir, probe_report=args.probe_report)
    except (OSError, ValueError, TypeError) as exc:
        print(f"analyze_imu_wire: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({key: report[key] for key in (
        "output_dir", "wire_sha256", "wire_bytes", "frames_decoded", "samples_decoded", "nodes_missing",
        "signed_observed_min", "signed_observed_max", "totals", "boundary_event_count", "wrap_candidate_count")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
