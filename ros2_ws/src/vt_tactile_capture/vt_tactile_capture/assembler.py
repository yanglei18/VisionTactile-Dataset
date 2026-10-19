"""Conservative, rolling ADC snapshots; never a simultaneous hardware scan."""

from dataclasses import dataclass
from numbers import Integral

from .grid import BLOCK_COUNT, CELL_COUNT, COLS, COLUMN_GROUPS, LAYOUT_ID, ROWS, SCHEMA_VERSION
from .protocol import ROWS as WIRE_ROWS, VALUES_PER_BLOCK
from .serial_source import Batch, CLOCK_DOMAIN, TIMING_SOURCE


UNIT = "adc_counts"


@dataclass(frozen=True)
class State:
    """Fields of TactileState except its ROS Header; arrays are immutable."""

    schema_version: int
    hand: str
    session_id: str
    stream_epoch: int
    source_sequence: int
    host_realtime_ns: int
    host_monotonic_ns: int
    snapshot_monotonic_ns: int
    clock_domain: str
    timing_source: str
    layout_id: str
    unit: str
    rows: int
    cols: int
    connected: bool
    time_valid: bool
    reason: str
    adc: tuple[int, ...]
    valid_mask: tuple[bool, ...]
    updated_mask: tuple[bool, ...]
    block_realtime_ns: tuple[int, ...]
    block_monotonic_ns: tuple[int, ...]
    max_age_ns: int
    producer_dropped_total: int


class Assembler:
    """Apply loss-aware batches and derive age-masked row-major snapshots.

    Gaps, resets, corruption, clock regressions and fault events invalidate the
    old cache. A subsequent valid batch can populate only the two or three
    physically active cells in each raw ten-value block. Raw addresses outside
    [0, 20) remain part of continuity accounting but never enter the state.
    Zero ADC is a valid value, never a missing sentinel.
    """

    def __init__(self, hand: str = "right", *, max_clock_step_ns: int = 5_000_000):
        if hand not in ("left", "right"):
            raise ValueError("hand must be left or right")
        if (isinstance(max_clock_step_ns, bool) or not isinstance(max_clock_step_ns, Integral)
                or max_clock_step_ns < 0):
            raise ValueError("max_clock_step_ns must be a nonnegative integer")
        self.hand = hand
        self.max_clock_step_ns = int(max_clock_step_ns)
        self._previous = None
        self._session_id = ""
        self._stream_epoch = 0
        self._source_sequence = 0
        self._host_realtime_ns = 0
        self._host_monotonic_ns = 0
        self._clock_domain = CLOCK_DOMAIN
        self._timing_source = TIMING_SOURCE
        self._connected = False
        self._time_valid = False
        self._reason = "not_started"
        self._producer_dropped_total = 0
        self._last_snapshot_ns = None
        self._realtime_highwater_ns = 0
        self._counter_session = None
        self._max_sequence = -1
        self._max_packet_end = 0
        self._max_epoch = 0
        self._clear()

    def _clear(self) -> None:
        self._adc = [0] * CELL_COUNT
        self._block_realtime_ns = [0] * BLOCK_COUNT
        self._block_monotonic_ns = [0] * BLOCK_COUNT
        self._seen = [False] * BLOCK_COUNT
        self._updated = [False] * BLOCK_COUNT

    @staticmethod
    def _reset_reason(previous: Batch, batch: Batch) -> str:
        if batch.session_id != previous.session_id:
            return "session_changed"
        if batch.stream_epoch != previous.stream_epoch:
            return "epoch_changed"
        if batch.clock_domain != previous.clock_domain:
            return "clock_domain_changed"
        if batch.sequence != previous.sequence + 1:
            return "sequence_gap"
        if batch.first_packet_index != previous.first_packet_index + len(previous.blocks):
            return "packet_index_gap"
        if batch.producer_dropped_total != previous.producer_dropped_total:
            return "producer_drop"
        if (batch.discarded_bytes_total != previous.discarded_bytes_total
                or batch.malformed_packets_total != previous.malformed_packets_total):
            return "serial_corruption"
        return ""

    def apply(self, batch: Batch) -> None:
        contract_error = self._contract_error(batch)
        if contract_error:
            self._clear()
            self._time_valid = False
            self._reason = contract_error
            return
        previous = self._previous
        reset_reason = self._reset_reason(previous, batch) if previous is not None else ""
        same_stream = (previous is not None
                       and batch.session_id == previous.session_id
                       and batch.stream_epoch == previous.stream_epoch)
        nonincreasing_realtime = batch.host_realtime_ns <= self._realtime_highwater_ns
        self._realtime_highwater_ns = max(self._realtime_highwater_ns, batch.host_realtime_ns)
        regression = ""
        if self._counter_session == batch.session_id:
            if batch.sequence <= self._max_sequence:
                regression = "nonincreasing_sequence"
            elif batch.first_packet_index < self._max_packet_end:
                regression = "packet_index_regression"
            elif batch.stream_epoch < self._max_epoch:
                regression = "epoch_regression"
        else:
            self._counter_session = batch.session_id
            self._max_sequence = -1
            self._max_packet_end = 0
            self._max_epoch = 0
        # Rejected events may never lower admission bounds. Realtime retains
        # one global high-water even across sessions; a bag rewind uses a new
        # Assembler, while a new session may reset only its monotonic origin.
        self._max_sequence = max(self._max_sequence, batch.sequence)
        self._max_packet_end = max(self._max_packet_end, batch.first_packet_index + len(batch.blocks))
        self._max_epoch = max(self._max_epoch, batch.stream_epoch)
        if regression:
            self._clear()
            self._previous = batch
            self._time_valid = False
            self._reason = regression
            return
        if not same_stream:
            # A new session/epoch can legitimately start on another monotonic
            # origin (e.g. replay), independently of the previous refresh time.
            self._last_snapshot_ns = None
        check_clock = same_stream and previous.time_valid and batch.time_valid
        backwards = bool(check_clock and (
            batch.host_realtime_ns <= previous.host_realtime_ns
            or batch.host_monotonic_ns <= previous.host_monotonic_ns))
        clock_step = bool(check_clock and not backwards and abs(
            (batch.host_realtime_ns - previous.host_realtime_ns)
            - (batch.host_monotonic_ns - previous.host_monotonic_ns)) > self.max_clock_step_ns)
        domain_valid = batch.clock_domain == CLOCK_DOMAIN and batch.timing_source == TIMING_SOURCE
        time_valid = (batch.time_valid and domain_valid and batch.host_realtime_ns > 0
                      and batch.host_monotonic_ns > 0 and not backwards and not clock_step
                      and not nonincreasing_realtime)
        if reset_reason or not batch.connected or not time_valid or not batch.blocks:
            self._clear()
        self._previous = batch
        self._session_id = batch.session_id
        self._stream_epoch = batch.stream_epoch
        self._source_sequence = batch.sequence
        self._clock_domain = batch.clock_domain
        self._timing_source = batch.timing_source
        self._connected = batch.connected
        self._time_valid = bool(time_valid and batch.connected and batch.blocks)
        self._producer_dropped_total = batch.producer_dropped_total
        self._reason = ("clock_backwards" if backwards else "clock_step" if clock_step
                        else "nonincreasing_realtime" if nonincreasing_realtime and batch.blocks
                        else (reset_reason or batch.reason))
        # Out-of-area packets and health events must not restamp old physical
        # readings. Raw batch/packet continuity still includes every packet.
        contributes = any(isinstance(block.row, Integral) and not isinstance(block.row, bool)
                          and 0 <= block.row < ROWS for block in batch.blocks)
        if contributes:
            self._host_realtime_ns = batch.host_realtime_ns
            self._host_monotonic_ns = batch.host_monotonic_ns
        elif not same_stream:
            self._host_realtime_ns = self._host_monotonic_ns = 0
        if not self._time_valid:
            if not domain_valid:
                self._reason = ("invalid_clock_domain" if batch.clock_domain != CLOCK_DOMAIN
                                else "invalid_timing_source")
            elif (batch.blocks and not backwards and not clock_step
                  and not nonincreasing_realtime and not time_valid):
                self._reason = "invalid_time"
            return
        # Validate the whole batch before writing so malformed external input
        # cannot leave an apparently valid, partially applied snapshot.
        if any(not (all(isinstance(value, Integral) and not isinstance(value, bool)
                        for value in (block.row, block.column_group, block.status_raw))
                    and 0 <= block.row < WIRE_ROWS
                    and 0 <= block.column_group < COLUMN_GROUPS
                    and 0 <= block.status_raw <= 255
                    and len(block.adc) == VALUES_PER_BLOCK
                    and all(isinstance(value, Integral) and not isinstance(value, bool)
                            and 0 <= value <= 65535
                            for value in block.adc)) for block in batch.blocks):
            self._clear()
            self._time_valid = False
            self._reason = "invalid_block"
            return
        for block in batch.blocks:
            if block.row >= ROWS:
                continue
            index = block.row * COLUMN_GROUPS + block.column_group
            for k, value in enumerate(block.adc):
                col = block.column_group + COLUMN_GROUPS * k
                if col < COLS:
                    self._adc[block.row * COLS + col] = value
            self._block_realtime_ns[index] = batch.host_realtime_ns
            self._block_monotonic_ns[index] = batch.host_monotonic_ns
            self._seen[index] = True
            self._updated[index] = True

    def _contract_error(self, batch: Batch) -> str:
        if (isinstance(batch.schema_version, bool)
                or not isinstance(batch.schema_version, Integral) or batch.schema_version != 1):
            return "invalid_schema"
        if batch.hand != self.hand:
            return "invalid_hand"
        if not isinstance(batch.session_id, str) or not batch.session_id.strip():
            return "missing_session"
        if not isinstance(batch.clock_domain, str):
            return "invalid_clock_domain"
        if not isinstance(batch.timing_source, str):
            return "invalid_timing_source"
        if type(batch.connected) is not bool or type(batch.time_valid) is not bool:
            return "invalid_flags"
        for name in ("sequence", "first_packet_index", "host_realtime_ns", "host_monotonic_ns",
                     "producer_dropped_total", "discarded_bytes_total", "malformed_packets_total",
                     "stream_epoch"):
            value = getattr(batch, name)
            bits = 32 if name == "stream_epoch" else 64
            if isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value < 1 << bits:
                return "invalid_" + name
        return ""

    def snapshot(self, now_monotonic_ns: int, max_age_ns: int) -> State:
        """Return current validity and consume the 'updated since snapshot' mask.

        Invalid ADC placeholders are zero. A GUI retaining stale pixels must
        own a separate display cache and visibly distinguish them from valid
        samples. Snapshot time never replaces the last input's capture time.
        """
        if now_monotonic_ns < 0 or max_age_ns < 0:
            raise ValueError("snapshot clocks and max_age_ns must be nonnegative")
        if ((self._last_snapshot_ns is not None and now_monotonic_ns < self._last_snapshot_ns)
                or (self._time_valid and now_monotonic_ns < self._host_monotonic_ns)):
            self._clear()
            self._time_valid = False
            self._reason = "snapshot_clock_backwards"
        self._last_snapshot_ns = now_monotonic_ns
        adc = [0] * CELL_COUNT
        valid_mask = [False] * CELL_COUNT
        updated_mask = [False] * CELL_COUNT
        if self._connected and self._time_valid:
            for index, seen in enumerate(self._seen):
                if not seen:
                    continue
                age = now_monotonic_ns - self._block_monotonic_ns[index]
                if age < 0 or age > max_age_ns:
                    continue
                row, group = divmod(index, COLUMN_GROUPS)
                for col in range(group, COLS, COLUMN_GROUPS):
                    cell = row * COLS + col
                    adc[cell] = self._adc[cell]
                    valid_mask[cell] = True
                    updated_mask[cell] = self._updated[index]
        reason = self._reason
        if self._connected and self._time_valid and not any(valid_mask):
            reason = "stale"
        state = State(
            schema_version=SCHEMA_VERSION, hand=self.hand, session_id=self._session_id,
            stream_epoch=self._stream_epoch, source_sequence=self._source_sequence,
            host_realtime_ns=self._host_realtime_ns, host_monotonic_ns=self._host_monotonic_ns,
            snapshot_monotonic_ns=now_monotonic_ns, clock_domain=self._clock_domain,
            timing_source=self._timing_source, layout_id=LAYOUT_ID, unit=UNIT,
            rows=ROWS, cols=COLS, connected=self._connected, time_valid=self._time_valid,
            reason=reason, adc=tuple(adc), valid_mask=tuple(valid_mask),
            updated_mask=tuple(updated_mask), block_realtime_ns=tuple(self._block_realtime_ns),
            block_monotonic_ns=tuple(self._block_monotonic_ns), max_age_ns=max_age_ns,
            producer_dropped_total=self._producer_dropped_total,
        )
        self._updated = [False] * BLOCK_COUNT
        return state
