"""Streaming tactile reconstruction at an existing alignment's camera times."""

from __future__ import annotations

from collections.abc import Iterable, Iterator
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import warnings

import numpy as np

from .bag_reader import _deserializer, _message_types, _open_reader, stamp_to_nanoseconds
from .model import MessageRef
from .identity import stream_identity
from .export import TOOL_VERSION
from .provenance import verify_alignment_source
from .tactile_adapter import (
    COLS, ROWS, WIRE_COLS, WIRE_ROWS,
    DEFAULT_TACTILE_TOPIC, TACTILE_SAMPLE_TYPE, TactileContinuity,
    TactileEvent, TactileGrid,
)


def iter_tactile_events(
    bag_path: str | Path, *, topic: str = DEFAULT_TACTILE_TOPIC,
    max_clock_step_ns: int = 5_000_000,
    expected_hand: str | None = None, expected_frame_id: str | None = None,
) -> Iterator[TactileEvent]:
    """Read only sparse source batches, preserving bag order and message indices."""
    path = Path(bag_path).resolve()
    if not path.is_dir():
        raise FileNotFoundError(f"rosbag2 directory does not exist: {path}")
    reader = _open_reader(path)
    if _message_types(reader).get(topic) != TACTILE_SAMPLE_TYPE:
        raise ValueError(f"tactile source topic is missing or has the wrong type: {topic}")
    # Let storage skip camera payloads, not merely discard them after read_next.
    # Sequence below is still local to this topic, matching alignment MessageRef.
    import rosbag2_py
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    deserialize = _deserializer(TACTILE_SAMPLE_TYPE)
    hand, frame = stream_identity(topic, TACTILE_SAMPLE_TYPE, expected_hand=expected_hand,
                                  expected_frame_id=expected_frame_id)
    continuity = TactileContinuity(max_clock_step_ns=max_clock_step_ns,
                                  expected_hand=hand, expected_frame_id=frame)
    sequence = 0
    while reader.has_next():
        current_topic, payload, bag_timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize(payload)
        header_ns = stamp_to_nanoseconds(message.header.stamp)
        reference = MessageRef(topic, sequence, int(bag_timestamp_ns), header_ns)
        sequence += 1
        yield continuity.decode(message, reference, header_stamp_ns=header_ns)


def iter_alignment_targets(path: str | Path) -> Iterator[tuple[int, int]]:
    """Use reference host realtime, not camera hardware stamps or bag times."""
    previous_time = previous_index = -1
    with Path(path).open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                value = json.loads(line)
                index, timestamp = value["frame_index"], value["reference_time_ns"]
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f"invalid aligned frame on line {line_number}") from error
            if (type(index) is not int or type(timestamp) is not int
                    or index <= previous_index or timestamp <= previous_time):
                raise ValueError("aligned frame indices and reference times must be non-negative and strictly increasing")
            previous_index, previous_time = index, timestamp
            yield index, timestamp


def reconstruct_frames(
    events: Iterable[TactileEvent], targets: Iterable[tuple[int, int]], *, max_age_ns: int,
) -> Iterator[tuple[dict[str, np.ndarray], dict[str, object]]]:
    """Keep one batch of lookahead and one rolling grid, not a grid per message.

    If the next event reveals a gap, the unsupported open interval after the
    preceding source event is invalidated conservatively. Future ADC values
    are never used. At a resumed batch only that batch's blocks become valid.
    """
    source = iter(events)
    pending = next(source, None)
    grid = TactileGrid()
    previous_target = -1
    for frame_index, timestamp in targets:
        if type(timestamp) is not int or timestamp <= previous_target:
            raise ValueError("target timestamps must be non-negative and strictly increasing")
        previous_target = timestamp
        while pending is not None and pending.effective_ns <= timestamp:
            grid.apply(pending)
            pending = next(source, None)
        future_barrier = None
        if (pending is not None and pending.barriers and pending.boundary_ns is not None
                and timestamp > pending.boundary_ns):
            grid.clear()
            future_barrier = pending.metadata()
        arrays = grid.snapshot(timestamp, max_age_ns)
        metadata = {
            "frame_index": frame_index, "reference_time_ns": timestamp,
            "valid_cell_count": int(arrays["valid_mask"].sum()),
            "last_source_event": grid.last_event.metadata() if grid.last_event else None,
            "last_contributing_event": (grid.last_contributing_event.metadata()
                                        if grid.last_contributing_event else None),
            "barrier_count": grid.barrier_count,
            "last_barrier": grid.last_barrier,
            "pending_barrier": future_barrier,
        }
        yield arrays, metadata


def export_tactile(
    *, bag_path: str | Path, aligned_frames: str | Path,
    output_directory: str | Path, topic: str = DEFAULT_TACTILE_TOPIC,
    max_age_ms: float = 100.0, max_clock_step_ms: float = 5.0,
    require_provenance: bool = False,
    expected_hand: str | None = None, expected_frame_id: str | None = None,
) -> Path:
    """Write one compressed NPZ per reference frame with bounded working RAM."""
    for name, value in (("max_age_ms", max_age_ms), ("max_clock_step_ms", max_clock_step_ms)):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and non-negative")
    if not topic.startswith("/"):
        raise ValueError("tactile topic must be absolute")
    hand, frame = stream_identity(topic, TACTILE_SAMPLE_TYPE, expected_hand=expected_hand,
                                  expected_frame_id=expected_frame_id)
    target = Path(output_directory).resolve()
    if target.exists():
        raise FileExistsError(f"refusing to overwrite output directory: {target}")
    if not target.parent.is_dir():
        raise FileNotFoundError(f"output parent does not exist: {target.parent}")
    provenance = verify_alignment_source(bag_path, aligned_frames,
                                         require_provenance=require_provenance)
    temporary = Path(tempfile.mkdtemp(prefix=f".{target.name}.", dir=target.parent))
    temporary.chmod(0o700)
    try:
        (temporary / "frames").mkdir(mode=0o700)
        events = iter_tactile_events(bag_path, topic=topic,
                                    max_clock_step_ns=round(max_clock_step_ms * 1_000_000),
                                    expected_hand=hand, expected_frame_id=frame)
        targets = iter_alignment_targets(aligned_frames)
        frames = reconstruct_frames(events, targets, max_age_ns=round(max_age_ms * 1_000_000))
        quality = _QualityAccumulator(max_age_ns=round(max_age_ms * 1_000_000))
        with (temporary / "source_metadata.jsonl").open("w", encoding="utf-8") as output:
            for arrays, metadata in frames:
                name = f"frames/{metadata['frame_index']:09d}.npz"
                np.savez_compressed(temporary / name, **arrays)
                (temporary / name).chmod(0o600)
                metadata["npz"] = name
                output.write(json.dumps(metadata, allow_nan=False, separators=(",", ":")) + "\n")
                quality.add(arrays)
        if not quality.frame_count:
            raise ValueError("aligned_frames contains no reference frames")
        (temporary / "source_metadata.jsonl").chmod(0o600)
        quality_report = quality.report()
        for warning in quality_report["warnings"]:
            warnings.warn(warning, UserWarning, stacklevel=2)
        with (temporary / "quality_report.json").open("w", encoding="utf-8") as output:
            json.dump(quality_report, output, indent=2, allow_nan=False)
            output.write("\n")
        (temporary / "quality_report.json").chmod(0o600)
        manifest = {
            "schema_version": 2, "source_bag": str(Path(bag_path).resolve()),
            "tool": "vt-multisensor-alignment", "tool_version": TOOL_VERSION,
            "provenance": provenance, "quality_summary": quality_report,
            "aligned_frames": str(Path(aligned_frames).resolve()),
            "source_topic": topic, "source_type": TACTILE_SAMPLE_TYPE,
            "expected_hand": hand, "expected_frame_id": frame,
            "frame_count": quality.frame_count,
            "frames_with_valid_cells": quality.frame_count - quality.empty_frames,
            "rows": ROWS, "cols": COLS, "unit": "adc_counts",
            "layout_id": "row20_col_group_stride8_v2",
            "layout": "wire column=column_group+8*k; retain row<20 and column<20",
            "source_schema_version": 1,
            "wire_rows": WIRE_ROWS, "wire_cols": WIRE_COLS,
            "physical_roi": {"row_start": 0, "row_stop": ROWS,
                             "col_start": 0, "col_stop": COLS},
            "selection": "previous_per_block", "max_age_ms": max_age_ms,
            "max_clock_step_ms": max_clock_step_ms,
            "timestamp_clock": "host_realtime_ns", "timing_source": "host_receive",
            "hardware_synchronized": False,
            "notes": ["Rolling partial coverage, not a simultaneous scan or force/pressure.",
                      "Only the physical upper-left 20x20 is reconstructed; raw wire packets stay unchanged.",
                      "Out-of-ROI packets count for continuity but do not refresh physical cells or their clocks.",
                      "Equal batch clocks are host-read completion, not simultaneous hardware sampling.",
                      "ADC zero is a valid value; use valid_mask, never nonzero(adc).",
                      "age_ns=-1 means unobserved; expired observed cells retain their nonnegative age.",
                      "Block message sequence refers to the source topic's zero-based rosbag message index.",
                      "Discontinuity intervals are conservatively invalidated using one-event lookahead."],
        }
        with (temporary / "manifest.json").open("w", encoding="utf-8") as output:
            json.dump(manifest, output, indent=2, allow_nan=False)
            output.write("\n")
        (temporary / "manifest.json").chmod(0o600)
        os.rename(temporary, target)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return target


class _QualityAccumulator:
    """Constant-memory coverage and valid-cell age histogram (no frame buffering)."""

    _BINS = 2048

    def __init__(self, *, max_age_ns: int) -> None:
        self.frame_count = self.full_frames = self.partial_frames = self.empty_frames = 0
        self.valid_cells = 0
        self.coverage_min = ROWS * COLS
        self.coverage_max = 0
        self.max_age_ns = max_age_ns
        self.age_histogram = np.zeros(self._BINS, dtype=np.int64)
        self.age_min = None
        self.age_max = None
        self.age_sum = 0.0

    def add(self, arrays: dict[str, np.ndarray]) -> None:
        mask = arrays["valid_mask"]
        count = int(mask.sum())
        self.frame_count += 1
        self.full_frames += count == ROWS * COLS
        self.partial_frames += 0 < count < ROWS * COLS
        self.empty_frames += count == 0
        self.valid_cells += count
        self.coverage_min = min(self.coverage_min, count)
        self.coverage_max = max(self.coverage_max, count)
        ages = arrays["age_ns"][mask]
        if ages.size:
            low, high = int(ages.min()), int(ages.max())
            self.age_min = low if self.age_min is None else min(self.age_min, low)
            self.age_max = high if self.age_max is None else max(self.age_max, high)
            self.age_sum += float(ages.astype(np.float64).sum())
            bins = np.minimum((ages.astype(np.float64) / (self.max_age_ns + 1)
                               * self._BINS).astype(np.int64), self._BINS - 1)
            self.age_histogram += np.bincount(bins, minlength=self._BINS)

    def report(self) -> dict[str, object]:
        warning_list = []
        if not self.valid_cells:
            warning_list.append("No valid tactile cells in any exported frame; files are present but contain no usable tactile measurements.")
        elif self.empty_frames or self.partial_frames:
            warning_list.append("Tactile export contains partial or empty coverage; inspect valid_mask before using ADC values.")
        quantiles = {}
        if self.valid_cells:
            cumulative = self.age_histogram.cumsum()
            for name, q in (("p50_ns", .5), ("p95_ns", .95), ("p99_ns", .99)):
                index = int(np.searchsorted(cumulative, math.ceil(self.valid_cells * q)))
                midpoint = round((index + .5) * (self.max_age_ns + 1) / self._BINS)
                quantiles[name] = max(self.age_min, min(self.age_max, midpoint))
        else:
            quantiles = {name: None for name in ("p50_ns", "p95_ns", "p99_ns")}
        return {
            "schema_version": 1,
            "verdict": "WARNING" if warning_list else "COMPLETE",
            "frame_count": self.frame_count,
            "frames_with_valid_cells": self.frame_count - self.empty_frames,
            "full_frames": self.full_frames, "partial_frames": self.partial_frames,
            "empty_frames": self.empty_frames, "cells_per_frame": ROWS * COLS,
            "valid_cell_observations": self.valid_cells,
            "coverage_ratio": self.valid_cells / (self.frame_count * ROWS * COLS) if self.frame_count else 0.0,
            "min_valid_cells_per_frame": self.coverage_min if self.frame_count else 0,
            "max_valid_cells_per_frame": self.coverage_max,
            "valid_cell_age_ns": {
                "count": self.valid_cells, "min": self.age_min, "max": self.age_max,
                "mean": self.age_sum / self.valid_cells if self.valid_cells else None,
                **quantiles, "quantile_method": "fixed_width_histogram_midpoint",
                "quantile_bin_width_ns": (self.max_age_ns + 1) / self._BINS,
                "histogram_bins": self._BINS,
                "population": "valid cells at each exported target; excludes expired/unobserved cells",
            },
            "warnings": warning_list,
            "interpretation": "Rolling partial coverage; not simultaneous hardware sampling or proof of zero transport loss.",
        }
