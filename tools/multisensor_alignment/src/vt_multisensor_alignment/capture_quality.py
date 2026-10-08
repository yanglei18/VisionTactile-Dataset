"""Streaming, post-recording observations; never a recording acceptance gate.

Images and unknown types use bag metadata only. Compact source messages are
selected at the storage reader, so an image-heavy bag need not deserialize or
copy image payloads just to inspect tactile/pose timing. No sample histories are
retained. Rates are labelled by their clock and measurement basis: a batch, a
rolling state, and a physical scan are deliberately not interchangeable.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from numbers import Integral
import os
from pathlib import Path
import tempfile

import yaml

from .bag_reader import _deserializer, _open_reader, configured_topic_types
from .config import AlignmentConfig
from .model import MessageRef
from .tactile_adapter import TACTILE_SAMPLE_TYPE, TactileContinuity


GLOVE_TYPE = "vt_glove_msgs/msg/GloveState"
TRACKER_TYPE = "vt_tracker_msgs/msg/TrackerSample"
CAMERA_TYPE = "vt_camera_msgs/msg/CameraFrameTiming"
TACTILE_STATE_TYPE = "vt_tactile_msgs/msg/TactileState"
INSPECTED_TYPES = frozenset({
    GLOVE_TYPE, TRACKER_TYPE, CAMERA_TYPE, TACTILE_SAMPLE_TYPE, TACTILE_STATE_TYPE,
})
_BLOCKS = 160
_CELLS = 400
_GAP_NS = 100_000_000
_CLOCK_STEP_NS = 5_000_000


def _integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return int(value)


def _stamp(message: object) -> int:
    stamp = message.header.stamp
    seconds = _integer(stamp.sec, "header.stamp.sec")
    nanos = _integer(stamp.nanosec, "header.stamp.nanosec")
    if nanos >= 1_000_000_000:
        raise ValueError("header.stamp.nanosec is outside its range")
    return seconds * 1_000_000_000 + nanos


@dataclass
class _Values:
    count: int = 0
    total: int = 0
    minimum: int | None = None
    maximum: int | None = None

    def add(self, value: int, weight: int = 1) -> None:
        self.count += weight
        self.total += value * weight
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def document(self) -> dict:
        return {"count": self.count, "minimum": self.minimum,
                "maximum": self.maximum,
                "mean": self.total / self.count if self.count else None}


class IntervalAccumulator:
    """Fixed-memory intervals, segmented explicitly at source/reset boundaries."""

    def __init__(self, gap_threshold_ns: int = _GAP_NS) -> None:
        self.gap_threshold_ns = gap_threshold_ns
        self.previous: int | None = None
        self.observations = 0
        self.duplicates = 0
        self.regressions = 0
        self.gaps = 0
        self.intervals = _Values()

    def reset_boundary(self) -> None:
        self.previous = None

    def add(self, stamp_ns: int) -> None:
        self.observations += 1
        if self.previous is not None:
            delta = stamp_ns - self.previous
            if delta == 0:
                self.duplicates += 1
            elif delta < 0:
                self.regressions += 1
            else:
                self.intervals.add(delta)
                self.gaps += delta > self.gap_threshold_ns
        # Do not let a delayed/repeated timestamp lower the high-water mark.
        self.previous = stamp_ns if self.previous is None else max(self.previous, stamp_ns)

    def document(self) -> dict:
        return {
            "observations": self.observations,
            "positive_interval_rate_hz": (
                self.intervals.count * 1e9 / self.intervals.total
                if self.intervals.total else None),
            "rate_basis": "positive adjacent intervals within source segments; duplicates and regressions excluded",
            "interval_ns": self.intervals.document(),
            "duplicate_timestamps": self.duplicates,
            "timestamp_regressions": self.regressions,
            "gap_threshold_ns": self.gap_threshold_ns,
            "gaps_over_threshold": self.gaps,
        }


class _Sequence:
    def __init__(self) -> None:
        self.highwater: int | None = None
        self.missing = self.gaps = self.duplicates = self.regressions = 0

    def reset_boundary(self) -> None:
        self.highwater = None

    def add(self, value: int) -> None:
        if self.highwater is not None:
            if value == self.highwater:
                self.duplicates += 1
            elif value < self.highwater:
                self.regressions += 1
            elif value > self.highwater + 1:
                self.gaps += 1
                self.missing += value - self.highwater - 1
        self.highwater = value if self.highwater is None else max(self.highwater, value)

    def document(self) -> dict:
        return {"gap_events": self.gaps, "unobserved_index_values": self.missing,
                "duplicates": self.duplicates, "regressions": self.regressions,
                "basis": "software indices within session/epoch; not proof of hardware or recorder loss"}


class _Counter:
    def __init__(self) -> None:
        self.context: object = None
        self.highwater: int | None = None
        self.increases = self.initial_values_sum = self.regressions = 0

    def add(self, value: int, context: object) -> None:
        if self.context != context or self.highwater is None:
            self.context, self.highwater = context, value
            # A nonzero first observation may predate this bag: never silently
            # count it as loss that occurred during the recording.
            self.initial_values_sum += value
            return
        if value < self.highwater:
            self.regressions += 1
        else:
            self.increases += value - self.highwater
            self.highwater = value

    def document(self) -> dict:
        return {"observed_increase": self.increases,
                "initial_values_sum_not_attributed_to_recording": self.initial_values_sum,
                "counter_regressions": self.regressions}


class TopicQualityAccumulator:
    """Pure message accumulator; one instance per compact source topic."""

    def __init__(self, topic: str, type_name: str) -> None:
        if type_name not in INSPECTED_TYPES:
            raise ValueError(f"unsupported inspection type: {type_name}")
        self.topic, self.type_name = topic, type_name
        self.messages = self.decoded = self.malformed = 0
        self.first_error: str | None = None
        self.invalid_time = self.disconnected = self.invalid_pose = 0
        self.clock_steps = self.header_mismatches = self.boundaries = 0
        self.bag_receive = IntervalAccumulator()
        self.source_monotonic = IntervalAccumulator()
        self.source_realtime = IntervalAccumulator()
        self.receipt_minus_source = _Values()
        self.sequence = _Sequence()
        self.counters: dict[str, _Counter] = {}
        self.context: object = None
        self.previous_clocks: tuple[int, int] | None = None
        self.continuity = TactileContinuity()
        self.raw_blocks = self.usable_roi_blocks = self.outside_roi_blocks = 0
        self.empty_batches = self.unusable_batches = 0
        self.roi_seen = [False] * _BLOCKS
        self.block_timing = [IntervalAccumulator() for _ in range(_BLOCKS)]
        self.cycle_seen: set[int] = set()
        self.coverage_cycles = 0
        self.cycle_timing = IntervalAccumulator()
        self.valid_cells = _Values()
        self.valid_cell_age = _Values()
        self.negative_age_cells = 0

    def _begin(self, receipt_ns: int) -> None:
        self.messages += 1
        self.bag_receive.add(_integer(receipt_ns, "bag timestamp"))

    def decode_failure(self, receipt_ns: int, error: Exception) -> None:
        self._begin(receipt_ns)
        self._error(error)

    def _error(self, error: Exception) -> None:
        self.malformed += 1
        if self.first_error is None:
            self.first_error = f"{type(error).__name__}: {error}"[:400]
        self._break_coverage()

    def _boundary(self, context: object) -> None:
        if self.context == context:
            return
        self.boundaries += self.context is not None
        self.context = context
        self.previous_clocks = None
        self.source_monotonic.reset_boundary()
        self.source_realtime.reset_boundary()
        self.sequence.reset_boundary()
        self.continuity = TactileContinuity()
        self._break_coverage()

    def _break_coverage(self) -> None:
        self.cycle_seen.clear()
        self.cycle_timing.reset_boundary()
        for timing in self.block_timing:
            timing.reset_boundary()

    def add(self, message: object, receipt_ns: int) -> None:
        self._begin(receipt_ns)
        try:
            self._add(message, receipt_ns)
            self.decoded += 1
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self._error(error)

    def _add(self, message: object, receipt_ns: int) -> None:
        camera = self.type_name == CAMERA_TYPE
        if self.type_name == GLOVE_TYPE and _integer(message.schema_version, "schema_version") != 1:
            raise ValueError("unsupported glove state schema")
        if camera:
            context = (str(message.camera_name), str(message.serial_number), "CLOCK_MONOTONIC_RAW")
            realtime = int(message.group_host_realtime_ns)
            monotonic = _integer(message.group_host_monotonic_raw_ns, "group monotonic")
            flags = _integer(message.group_validity_flags, "group validity")
            time_valid = flags & 11 == 11
        else:
            session = getattr(message, "session_id", getattr(message, "tracker_id", ""))
            domain = getattr(message, "clock_domain", "CLOCK_MONOTONIC")
            epoch = getattr(message, "stream_epoch", None)
            context = (session, epoch, domain)
            realtime = _integer(message.host_realtime_ns, "host realtime")
            monotonic = _integer(message.host_monotonic_ns, "host monotonic")
            time_valid = bool(getattr(message, "time_valid", True))
            if self.type_name != TRACKER_TYPE:
                time_valid = (time_valid and domain == "CLOCK_MONOTONIC"
                              and message.timing_source == "host_receive")
            if _stamp(message) != realtime:
                self.header_mismatches += 1
                time_valid = False
        self._boundary(context)
        connected = bool(getattr(message, "connected", True))
        self.disconnected += not connected
        if self.type_name in {TRACKER_TYPE, GLOVE_TYPE}:
            pose_valid = bool(message.pose_valid)
            if self.type_name == TRACKER_TYPE:
                pose_valid = pose_valid and _integer(message.tracking_status, "tracking_status") & 15 == 2
            self.invalid_pose += not pose_valid
        time_valid = time_valid and realtime > 0 and monotonic > 0
        self.invalid_time += not time_valid
        if time_valid:
            self.source_monotonic.add(monotonic)
            self.source_realtime.add(realtime)
            if self.previous_clocks is not None:
                rt, mono = self.previous_clocks
                self.clock_steps += abs((realtime - rt) - (monotonic - mono)) > _CLOCK_STEP_NS
            self.previous_clocks = (realtime, monotonic)
            self.receipt_minus_source.add(receipt_ns - realtime)
        else:
            self.previous_clocks = None
            self.source_monotonic.reset_boundary()
            self.source_realtime.reset_boundary()

        sequence_field = ("sequence" if self.type_name == TACTILE_SAMPLE_TYPE
                          else "frame_index" if self.type_name == GLOVE_TYPE else None)
        if sequence_field is not None:
            self.sequence.add(_integer(getattr(message, sequence_field), sequence_field))
        for name in ("producer_dropped_total", "discarded_bytes_total", "malformed_packets_total",
                     "bridge_sequence_gaps_total", "bridge_rejected_total"):
            if hasattr(message, name):
                counter = self.counters.setdefault(name, _Counter())
                # These are session-cumulative, not epoch-cumulative counters.
                counter.add(_integer(getattr(message, name), name), context[0])

        if self.type_name == TACTILE_SAMPLE_TYPE:
            reference = MessageRef(self.topic, self.messages - 1, receipt_ns, realtime)
            event = self.continuity.decode(message, reference, header_stamp_ns=_stamp(message))
            self.raw_blocks += len(event.blocks)
            self.empty_batches += not event.blocks
            self.outside_roi_blocks += sum(block.row >= 20 for block in event.blocks)
            if event.barriers:
                self._break_coverage()
            if not event.usable:
                self.unusable_batches += 1
                return
            for block in event.blocks:
                if block.row >= 20:
                    continue
                index = block.row * 8 + block.column_group
                self.usable_roi_blocks += 1
                self.roi_seen[index] = True
                self.block_timing[index].add(monotonic)
                self.cycle_seen.add(index)
                if len(self.cycle_seen) == _BLOCKS:
                    self.coverage_cycles += 1
                    self.cycle_timing.add(monotonic)
                    self.cycle_seen.clear()
        elif self.type_name == TACTILE_STATE_TYPE:
            schema = _integer(message.schema_version, "schema_version")
            rows, cols = int(message.rows), int(message.cols)
            if (schema, rows, cols) not in {(1, 80, 80), (2, 20, 20)}:
                raise ValueError("unsupported tactile state grid")
            if len(message.valid_mask) != rows * cols or len(message.block_monotonic_ns) != rows * 8:
                raise ValueError("tactile state mask/clock length mismatch")
            valid = [bool(message.valid_mask[row * cols + col])
                     for row in range(20) for col in range(20)]
            self.valid_cells.add(sum(valid))
            snapshot = _integer(message.snapshot_monotonic_ns, "snapshot clock")
            for row in range(20):
                for col in range(20):
                    if not valid[row * 20 + col]:
                        continue
                    block_time = _integer(message.block_monotonic_ns[row * 8 + col % 8], "block clock")
                    age = snapshot - block_time
                    if block_time == 0 or age < 0:
                        self.negative_age_cells += 1
                    else:
                        self.valid_cell_age.add(age)

    def document(self) -> dict:
        result = {
            "inspection": "compact_messages", "observed_messages": self.messages,
            "decoded_messages": self.decoded, "malformed_messages": self.malformed,
            "first_decode_error": self.first_error,
            "source_context_changes": self.boundaries,
            "bag_receive_timing": self.bag_receive.document(),
            "source_monotonic_timing": self.source_monotonic.document(),
            "source_realtime_timing": self.source_realtime.document(),
            "clock_step_threshold_ns": _CLOCK_STEP_NS,
            "clock_step_events": self.clock_steps,
            "invalid_time_messages": self.invalid_time,
            "disconnected_messages": self.disconnected,
            "invalid_pose_messages": self.invalid_pose,
            "header_source_mismatches": self.header_mismatches,
            "bag_receive_minus_source_realtime_ns": self.receipt_minus_source.document(),
            "latency_basis": "bag timestamp minus host receive clock; meaningful only with the same realtime clock; not hardware latency",
            "cumulative_counters": {name: value.document() for name, value in self.counters.items()},
        }
        if self.type_name in {GLOVE_TYPE, TACTILE_SAMPLE_TYPE}:
            result["software_sequence"] = self.sequence.document()
        if self.type_name == TACTILE_SAMPLE_TYPE:
            cells_seen = sum((3 if index % 8 < 4 else 2)
                             for index, seen in enumerate(self.roi_seen) if seen)
            result["tactile"] = {
                "rows": 20, "cols": 20, "physical_cells": _CELLS,
                "raw_wire_blocks": self.raw_blocks,
                "outside_physical_roi_blocks": self.outside_roi_blocks,
                "usable_physical_blocks": self.usable_roi_blocks,
                "empty_status_batches": self.empty_batches,
                "unusable_batches": self.unusable_batches,
                "observed_at_least_once_cells": cells_seen,
                "observed_at_least_once_coverage": cells_seen / _CELLS,
                "coverage_basis": "union of valid physical addresses across the recording; not simultaneous freshness",
                "complete_observed_coverage_cycles": self.coverage_cycles,
                "coverage_cycle_timing": self.cycle_timing.document(),
                "cycle_basis": "all 160 physical stride-8 blocks observed since previous completion without continuity barriers; NOT hardware-synchronized scans",
                "per_physical_block": [
                    {"row": i // 8, "column_group": i % 8, "cell_count": 3 if i % 8 < 4 else 2,
                     **timing.document()}
                    for i, timing in enumerate(self.block_timing)
                ],
            }
        elif self.type_name == TACTILE_STATE_TYPE:
            result["tactile"] = {
                "rows": 20, "cols": 20, "physical_cells": _CELLS,
                "valid_cells_per_snapshot": self.valid_cells.document(),
                "valid_cell_age_ns": self.valid_cell_age.document(),
                "invalid_age_cells": self.negative_age_cells,
                "basis": "published rolling snapshot validity, not independent simultaneous scans; legacy 80x80 cropped to physical 20x20",
            }
        return result


def _metadata(path: Path) -> tuple[dict, dict[str, dict]]:
    document = yaml.safe_load((path / "metadata.yaml").read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("rosbag2_bagfile_information"), dict):
        raise ValueError("bag metadata is missing rosbag2_bagfile_information")
    info = document["rosbag2_bagfile_information"]
    duration = _integer(info["duration"]["nanoseconds"], "bag duration")
    start = _integer(info["starting_time"]["nanoseconds_since_epoch"], "bag start")
    topics = {}
    for entry in info.get("topics_with_message_count", []):
        topic = entry["topic_metadata"]["name"]
        type_name = entry["topic_metadata"]["type"]
        if not isinstance(topic, str) or not isinstance(type_name, str) or topic in topics:
            raise ValueError("bag metadata topic is malformed or duplicated")
        count = _integer(entry["message_count"], "message_count")
        topics[topic] = {
            "type": type_name, "recorded_message_count": count,
            "metadata_global_duration_rate_hz": count * 1e9 / duration if duration else None,
            "metadata_rate_basis": "message count / whole bag duration; NOT per-topic source frequency",
            "inspection": "metadata_only",
            "inspection_reason": "payload inspection is not supported or is unnecessary for this type",
        }
    return {"path": str(path), "storage_identifier": info.get("storage_identifier"),
            "duration_ns": duration, "start_realtime_ns": start,
            "metadata_message_count": _integer(info["message_count"], "bag message count")}, topics


def _set_filter(reader: object, topics: list[str]) -> None:
    import rosbag2_py
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))


def _write_report(path: Path, document: dict) -> None:
    """Publish a complete file atomically, refusing even raced-in destinations."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        # Same-directory hard link is atomic and does not overwrite existing
        # reports, unlike replace()/rename(). The temporary name is removed.
        os.link(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def generate_capture_quality(
    bag_path: str | Path,
    output_path: str | Path,
    config: AlignmentConfig | None = None,
) -> dict:
    """Read a closed bag and write a warning-only, no-overwrite JSON report.

    No extrinsics, strict whole-bag configuration, or online hardware access are
    required. Optional config only adds expected-topic/type observations. An
    unavailable ROS message type produces metadata-only coverage, not a false
    claim of a verified stream. Output must be outside the source bag directory.
    """
    bag, output = Path(bag_path).resolve(), Path(output_path).absolute()
    if not bag.is_dir():
        raise FileNotFoundError(f"rosbag2 directory does not exist: {bag}")
    if os.path.lexists(output):
        raise FileExistsError(f"quality report already exists: {output}")
    if output.resolve().is_relative_to(bag):
        raise ValueError("quality report must be outside the source bag directory")
    metadata, topics = _metadata(bag)
    warnings: list[dict] = []

    def warn(code: str, detail: str, topic: str | None = None) -> None:
        item = {"code": code, "detail": detail}
        if topic is not None:
            item["topic"] = topic
        warnings.append(item)

    if sum(value["recorded_message_count"] for value in topics.values()) != metadata["metadata_message_count"]:
        warn("metadata_count_mismatch", "per-topic counts differ from bag message_count")
    if not topics or metadata["metadata_message_count"] == 0:
        warn("empty_bag", "metadata contains no recorded messages")
    if config is not None:
        expected = configured_topic_types(config)
        for topic in config.required_topics:
            if topic not in topics or topics[topic]["recorded_message_count"] == 0:
                warn("expected_topic_missing", "required configured topic has no recorded messages", topic)
        for topic, type_name in expected.items():
            if topic in topics and topics[topic]["type"] != type_name:
                warn("configured_type_mismatch", f"expected {type_name}, got {topics[topic]['type']}", topic)

    deserializers = {}
    accumulators = {}
    for topic, entry in topics.items():
        if entry["recorded_message_count"] == 0:
            warn("empty_topic", "topic exists but contains no messages", topic)
        if entry["type"] not in INSPECTED_TYPES:
            continue
        try:
            deserializers[topic] = _deserializer(entry["type"])
        except (ImportError, AttributeError, RuntimeError, ValueError) as error:
            entry["inspection_reason"] = f"message type unavailable: {error}"[:400]
            warn("message_support_unavailable", entry["inspection_reason"], topic)
        else:
            accumulators[topic] = TopicQualityAccumulator(topic, entry["type"])
    if deserializers:
        reader = _open_reader(bag)
        # A failed/missing filter is not silently replaced by decoding the
        # entire image bag. Caller receives an actionable unsupported-reader error.
        try:
            _set_filter(reader, list(deserializers))
            while reader.has_next():
                topic, payload, receipt_ns = reader.read_next()
                if topic not in deserializers:
                    continue
                accumulator = accumulators[topic]
                try:
                    message = deserializers[topic](payload)
                except Exception as error:
                    accumulator.decode_failure(int(receipt_ns), error)
                    continue
                accumulator.add(message, int(receipt_ns))
        finally:
            close = getattr(reader, "close", None)
            if close is not None:
                close()
        for topic, accumulator in accumulators.items():
            entry = topics[topic]
            entry.pop("inspection_reason", None)
            entry.update(accumulator.document())
            if entry["observed_messages"] != entry["recorded_message_count"]:
                warn("observed_count_mismatch", "filtered payload count differs from bag metadata", topic)
            for key in ("malformed_messages", "invalid_time_messages", "disconnected_messages",
                        "invalid_pose_messages", "clock_step_events", "header_source_mismatches"):
                if entry[key]:
                    warn(key, f"observed {entry[key]}", topic)
            for key in ("timestamp_regressions", "gaps_over_threshold"):
                count = entry["source_monotonic_timing"][key]
                if count:
                    warn(f"source_{key}", f"observed {count}; see source timing threshold/basis", topic)
            sequence = entry.get("software_sequence", {})
            if any(sequence.get(key, 0) for key in ("gap_events", "duplicates", "regressions")):
                warn("software_sequence_discontinuity", "software indices are discontinuous; not a hardware loss estimate", topic)
            for name, values in entry["cumulative_counters"].items():
                if values["observed_increase"] or values["counter_regressions"]:
                    warn("source_counter_change", f"{name}: {values}", topic)
            tactile = entry.get("tactile", {})
            if "observed_at_least_once_cells" in tactile and tactile["observed_at_least_once_cells"] < _CELLS:
                warn("incomplete_tactile_address_coverage", f"observed {tactile['observed_at_least_once_cells']}/400 physical cells", topic)
            if "complete_observed_coverage_cycles" in tactile and not tactile["complete_observed_coverage_cycles"]:
                warn("no_complete_tactile_coverage_cycle", "no complete observed coverage within one continuity segment; this is not a hardware frame check", topic)
            if "valid_cells_per_snapshot" in tactile:
                minimum = tactile["valid_cells_per_snapshot"]["minimum"]
                if minimum is None or minimum < _CELLS:
                    warn("partial_tactile_snapshots", "one or more snapshots lack complete valid physical coverage", topic)

    inspected = len(accumulators)
    result = {
        "schema_version": 1,
        "report_kind": "post_record_capture_quality",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "warnings" if warnings else "ok",
        "bag": metadata,
        "summary": {"topic_count": len(topics), "inspected_topic_count": inspected,
                    "metadata_only_topic_count": len(topics) - inspected,
                    "warning_count": len(warnings)},
        "topics": topics, "warnings": warnings,
        "limitations": [
            "Post-record observations only; this report never gates or stops recording.",
            "Uninspected payloads, including images, have metadata counts only; no per-frame validity or source frequency claim.",
            "Host receive timestamps are not hardware sample timestamps or a guarantee of cross-device synchronization.",
            "Software sequence gaps and cumulative counters overlap and are never summed into a total hardware loss estimate.",
            "Per-block and coverage-cycle tactile rates use host read timestamps; a ROS batch or rolling snapshot is not a simultaneous whole-hand scan.",
            "Counter values present at the first observation may predate the bag; only subsequent high-water increases are attributed to this recording.",
            "The absence of warnings is not a certification of sensor accuracy, image completeness, or unobservable firmware loss.",
        ],
    }
    _write_report(output, result)
    return result
