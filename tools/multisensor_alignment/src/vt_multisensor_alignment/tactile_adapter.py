"""Loss-aware sparse tactile batches; never treat one packet as a whole hand."""

from __future__ import annotations

from dataclasses import dataclass, replace
from numbers import Integral

import numpy as np

from .model import ClockObservation, GenericSample, MessageRef
from .identity import check_identity


TACTILE_SAMPLE_TYPE = "vt_tactile_msgs/msg/TactileSampleArray"
DEFAULT_TACTILE_TOPIC = "/gloves/right/tactile/samples"
# The STM32 archive is unchanged: 80 rows, 8 groups and 10 values per packet.
# Only its upper-left 20x20 addresses are physical sensors on this glove.
WIRE_ROWS = WIRE_COLS = 80
ROWS = COLS = 20
GROUPS = 8


def _uint(value: object, name: str, bits: int = 64) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"TactileSampleArray {name} must be an unsigned integer")
    result = int(value)
    if not 0 <= result < 1 << bits:
        raise ValueError(f"TactileSampleArray {name} is out of range")
    return result


@dataclass(frozen=True)
class TactileBlock:
    row: int
    column_group: int
    status_raw: int
    adc: tuple[int, ...]


@dataclass(frozen=True)
class TactileEvent:
    reference: MessageRef
    hand: str
    session_id: str
    stream_epoch: int
    sequence: int
    first_packet_index: int
    realtime_ns: int
    monotonic_ns: int
    connected: bool
    time_valid: bool
    reason: str
    producer_dropped_total: int
    discarded_bytes_total: int
    malformed_packets_total: int
    blocks: tuple[TactileBlock, ...]
    usable: bool
    barriers: tuple[str, ...]
    # Scheduling uses source clock high-water, never rosbag receive time.
    effective_ns: int
    boundary_ns: int | None

    def metadata(self) -> dict[str, object]:
        return {
            "message": self.reference.as_document(),
            "hand": self.hand, "session_id": self.session_id,
            "stream_epoch": self.stream_epoch, "sequence": self.sequence,
            "first_packet_index": self.first_packet_index,
            "block_count": len(self.blocks),
            "host_realtime_ns": self.realtime_ns,
            "host_monotonic_ns": self.monotonic_ns,
            "connected": self.connected, "time_valid": self.time_valid,
            "reason": self.reason, "usable": self.usable,
            "barriers": list(self.barriers),
            "producer_dropped_total": self.producer_dropped_total,
            "discarded_bytes_total": self.discarded_bytes_total,
            "malformed_packets_total": self.malformed_packets_total,
        }


class TactileContinuity:
    """Validate the v1 contract and identify resets without retaining payloads."""

    def __init__(self, *, max_clock_step_ns: int = 5_000_000,
                 expected_hand: str | None = None, expected_frame_id: str | None = None) -> None:
        self.max_clock_step_ns = _uint(max_clock_step_ns, "max_clock_step_ns")
        self.expected_hand, self.expected_frame_id = expected_hand, expected_frame_id
        self.previous: TactileEvent | None = None
        self.highwater_ns = 0
        self.hand: str | None = None
        self._counter_session: str | None = None
        self._max_sequence = -1
        self._max_packet_end = 0
        self._max_epoch = 0

    def decode(self, message: object, reference: MessageRef, *, header_stamp_ns: int) -> TactileEvent:
        try:
            return self._decode(message, reference, header_stamp_ns=header_stamp_ns)
        except AttributeError as error:
            raise ValueError(f"TactileSampleArray is missing a contract field: {error}") from error

    def _decode(self, message: object, reference: MessageRef, *, header_stamp_ns: int) -> TactileEvent:
        check_identity(message, reference.topic, expected_hand=self.expected_hand,
                       expected_frame_id=self.expected_frame_id)
        if _uint(message.schema_version, "schema_version", 16) != 1:
            raise ValueError("unsupported TactileSampleArray schema_version")
        if message.hand not in {"left", "right"}:
            raise ValueError("TactileSampleArray hand must be left or right")
        if self.hand is not None and self.hand != message.hand:
            raise ValueError("TactileSampleArray hand changed on one topic")
        self.hand = message.hand
        for name in ("session_id", "clock_domain", "timing_source", "reason"):
            if not isinstance(getattr(message, name), str):
                raise ValueError(f"TactileSampleArray {name} must be a string")
        for name in ("connected", "time_valid"):
            if type(getattr(message, name)) is not bool:
                raise ValueError(f"TactileSampleArray {name} must be bool")
        integers = {name: _uint(getattr(message, name), name) for name in (
            "sequence", "first_packet_index", "host_realtime_ns", "host_monotonic_ns",
            "producer_dropped_total", "discarded_bytes_total", "malformed_packets_total",
        )}
        epoch = _uint(message.stream_epoch, "stream_epoch", 32)
        rt, mono = integers["host_realtime_ns"], integers["host_monotonic_ns"]
        if header_stamp_ns != rt or reference.source_timestamp_ns != rt:
            raise ValueError("TactileSampleArray source/header timestamp mismatch")
        blocks = []
        for block in message.blocks:
            row = _uint(block.row, "row", 16)
            group = _uint(block.column_group, "column_group", 8)
            adc = tuple(_uint(value, "adc", 16) for value in block.adc)
            if row >= WIRE_ROWS or group >= GROUPS or len(adc) != 10:
                raise ValueError("TactileSampleArray block must address 10 cells in the 80x80 stride-8 layout")
            blocks.append(TactileBlock(row, group, _uint(block.status_raw, "status_raw", 8), adc))
        barriers: list[str] = []
        timing_ok = (message.time_valid and rt > 0 and mono > 0
                     and message.clock_domain == "CLOCK_MONOTONIC"
                     and message.timing_source == "host_receive")
        if not timing_ok:
            barriers.append("invalid_clock")
        if not message.session_id:
            barriers.append("missing_session")
        if not message.connected:
            barriers.append("disconnected")
        if not blocks:
            barriers.append("status_event")
        previous = self.previous
        usable = bool(timing_ok and message.session_id and message.connected and blocks)
        if usable and rt <= self.highwater_ns:
            barriers.append("nonincreasing_realtime")
            usable = False
        if self._counter_session == message.session_id:
            if integers["sequence"] <= self._max_sequence:
                barriers.append("nonincreasing_sequence")
                usable = False
            if integers["first_packet_index"] < self._max_packet_end:
                barriers.append("packet_index_regression")
                usable = False
            if epoch < self._max_epoch:
                barriers.append("epoch_regression")
                usable = False
        else:
            self._counter_session = message.session_id
            self._max_sequence = -1
            self._max_packet_end = 0
            self._max_epoch = 0
        if previous is not None:
            same_session = previous.session_id == message.session_id
            same_context = same_session and previous.stream_epoch == epoch
            if not same_session:
                barriers.append("session_change")
            elif not same_context:
                barriers.append("epoch_change")
            if same_session:
                if integers["sequence"] != previous.sequence + 1:
                    barriers.append("sequence_gap")
                    if integers["sequence"] <= previous.sequence:
                        usable = False
                for name in ("producer_dropped_total", "discarded_bytes_total", "malformed_packets_total"):
                    if integers[name] != getattr(previous, name):
                        barriers.append(name)
                if integers["first_packet_index"] != previous.first_packet_index + len(previous.blocks):
                    barriers.append("packet_index_gap")
            if same_context and timing_ok and previous.time_valid and previous.monotonic_ns > 0:
                delta_mono = mono - previous.monotonic_ns
                if delta_mono <= 0:
                    barriers.append("nonincreasing_monotonic")
                    usable = False
                elif abs((rt - previous.realtime_ns) - delta_mono) > self.max_clock_step_ns:
                    barriers.append("clock_step")
                    usable = False
        event = TactileEvent(
            reference=reference, hand=message.hand, session_id=message.session_id,
            stream_epoch=epoch, sequence=integers["sequence"],
            first_packet_index=integers["first_packet_index"],
            realtime_ns=rt, monotonic_ns=mono, connected=message.connected,
            time_valid=message.time_valid, reason=message.reason,
            producer_dropped_total=integers["producer_dropped_total"],
            discarded_bytes_total=integers["discarded_bytes_total"],
            malformed_packets_total=integers["malformed_packets_total"],
            blocks=tuple(blocks), usable=usable, barriers=tuple(barriers),
            effective_ns=max(self.highwater_ns, rt),
            boundary_ns=self.highwater_ns if previous is not None else None,
        )
        self.highwater_ns = event.effective_ns
        # A rejected/repeated event must not lower these admission bounds and
        # make the next replay of an already-seen software index look new.
        self._max_sequence = max(self._max_sequence, event.sequence)
        self._max_packet_end = max(self._max_packet_end, event.first_packet_index + len(event.blocks))
        self._max_epoch = max(self._max_epoch, event.stream_epoch)
        self.previous = event
        return event


class TactileSampleAdapter:
    """Index valid batches with conservative bounds; do not retain dense grids."""

    def __init__(self, stream_name: str, *, max_clock_step_ns: int = 5_000_000,
                 expected_hand: str | None = None, expected_frame_id: str | None = None) -> None:
        self.stream_name = stream_name
        self.continuity = TactileContinuity(max_clock_step_ns=max_clock_step_ns,
                                          expected_hand=expected_hand,
                                          expected_frame_id=expected_frame_id)
        self.clock_observations: dict[str, list[ClockObservation]] = {}
        self._samples: list[GenericSample] = []
        self._run: list[GenericSample] = []

    def _close(self, barrier: bool) -> None:
        if self._run:
            lower = self._run[0].timestamp_ns
            upper = self._run[-1].timestamp_ns if barrier else None
            self._samples.extend(replace(sample, valid_from_ns=lower, valid_until_ns=upper)
                                 for sample in self._run)
            self._run.clear()

    def add(self, message: object, reference: MessageRef, *, header_stamp_ns: int) -> None:
        event = self.continuity.decode(message, reference, header_stamp_ns=header_stamp_ns)
        if event.time_valid and event.realtime_ns > 0 and event.monotonic_ns > 0 and event.session_id:
            key = f"tactile:{self.stream_name}:{event.session_id}:{event.stream_epoch}"
            self.clock_observations.setdefault(key, []).append(
                ClockObservation(event.realtime_ns, event.monotonic_ns))
        if event.barriers:
            self._close(True)
        if event.usable:
            self._run.append(GenericSample(self.stream_name, event.realtime_ns, reference))

    def finish(self) -> tuple[GenericSample, ...]:
        self._close(False)
        return tuple(self._samples)


class TactileGrid:
    """One physical 20x20 grid and 160 clocks; wire continuity stays unchanged."""

    def __init__(self) -> None:
        self.adc = np.zeros((ROWS, COLS), dtype=np.uint16)
        self.block_realtime_ns = np.zeros((ROWS, GROUPS), dtype=np.uint64)
        self.block_monotonic_ns = np.zeros((ROWS, GROUPS), dtype=np.uint64)
        self.block_message_sequence = np.full((ROWS, GROUPS), -1, dtype=np.int64)
        self.block_packet_index = np.full((ROWS, GROUPS), -1, dtype=np.int64)
        self.last_event: TactileEvent | None = None
        self.last_contributing_event: TactileEvent | None = None
        self.last_barrier: dict[str, object] | None = None
        self.barrier_count = 0

    def clear(self) -> None:
        self.adc.fill(0)
        self.block_realtime_ns.fill(0)
        self.block_monotonic_ns.fill(0)
        self.block_message_sequence.fill(-1)
        self.block_packet_index.fill(-1)
        self.last_contributing_event = None

    def apply(self, event: TactileEvent) -> None:
        if event.barriers:
            self.clear()
            self.barrier_count += 1
            self.last_barrier = event.metadata()
        self.last_event = event
        if not event.usable:
            return
        for index, block in enumerate(event.blocks):
            row, group = block.row, block.column_group
            if row >= ROWS:
                continue
            count = len(range(group, COLS, GROUPS))
            self.adc[row, group::GROUPS] = block.adc[:count]
            self.block_realtime_ns[row, group] = event.realtime_ns
            self.block_monotonic_ns[row, group] = event.monotonic_ns
            self.block_message_sequence[row, group] = event.reference.sequence
            self.block_packet_index[row, group] = event.first_packet_index + index
            self.last_contributing_event = event

    def snapshot(self, timestamp_ns: int, max_age_ns: int) -> dict[str, np.ndarray]:
        timestamp_ns = _uint(timestamp_ns, "timestamp_ns")
        max_age_ns = _uint(max_age_ns, "max_age_ns")
        clocks = self.block_realtime_ns[:, np.arange(COLS) % GROUPS]
        observed = (clocks > 0) & (clocks <= timestamp_ns)
        ages = np.full((ROWS, COLS), -1, dtype=np.int64)
        ages[observed] = timestamp_ns - clocks[observed]
        valid = observed & (ages <= max_age_ns)
        adc = self.adc.copy()
        adc[~valid] = 0
        return {
            "timestamp_ns": np.asarray(timestamp_ns, dtype=np.uint64),
            "adc": adc, "valid_mask": valid, "age_ns": ages,
            "block_realtime_ns": self.block_realtime_ns.copy(),
            "block_monotonic_ns": self.block_monotonic_ns.copy(),
            "block_message_sequence": self.block_message_sequence.copy(),
            "block_packet_index": self.block_packet_index.copy(),
        }
