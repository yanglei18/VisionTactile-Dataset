"""Decode 44-byte, 20-column tactile rows without inventing CRCs.

There is no documented checksum or hardware clock/sequence in this protocol.
Only the header and address bounds can be checked. In particular, a corrupted
payload with a plausible address cannot be detected by this parser.
"""

from dataclasses import dataclass
from collections import deque
import struct


ROWS = COLS = 20
VALUES_PER_BLOCK = COLS
FRAME_SIZE = 44
HEADER = b"\xa5\x5a"
PROTOCOL_ID = "vt_tactile_row44_v1"


@dataclass(frozen=True)
class Block:
    """One complete row: ADC values at ``(row, column)`` in wire order."""

    row: int
    adc: tuple[int, ...]


def frame_key(buffer, offset=0):
    """Return a structurally valid complete frame's row, otherwise ``None``.

    Shared by acquisition and port identification. This checks framing and
    addresses, not payload integrity: the firmware provides no checksum.
    """
    if len(buffer) - offset < FRAME_SIZE or buffer[offset:offset + 2] != HEADER:
        return None
    row = (buffer[offset + 2] << 8) | buffer[offset + 3]
    return row if row < ROWS else None


class FramingEvidence:
    """Bounded evidence of adjacent complete row frames from at least two rows.

    A single plausible prefix is insufficient, since another protocol may
    share the header. Host read boundaries do not break a continuous run.
    """

    def __init__(self, required_frames=8):
        if type(required_frames) is not int or required_frames < 2:
            raise ValueError("required_frames must be an integer >= 2")
        self.required_frames = required_frames
        self._buffer = bytearray()
        self._count = 0
        self._rows = set()
        self.confirmed = False

    def feed(self, data):
        view = memoryview(data).cast("B")
        for start in range(0, len(view), 4096):
            self._buffer.extend(view[start:start + 4096])
            cursor = 0
            while cursor + FRAME_SIZE <= len(self._buffer):
                row = frame_key(self._buffer, cursor)
                if row is None:
                    self._count = 0
                    self._rows.clear()
                    cursor += 1
                    continue
                self._count = min(self._count + 1, self.required_frames)
                if len(self._rows) < 2:
                    self._rows.add(row)
                if self._count >= self.required_frames and len(self._rows) >= 2:
                    self.confirmed = True
                cursor += FRAME_SIZE
            del self._buffer[:cursor]
        return self.confirmed


class Parser:
    """Incremental, bounded-memory parser with sliding header resynchronization.

    Counters are lifetime totals, including bytes discarded on ``reset``.
    ``malformed_packets_total`` counts header candidates with invalid addresses,
    not all corrupt wire packets (which cannot be known without a checksum).
    """

    def __init__(self, max_buffer_bytes: int = 4096):
        if max_buffer_bytes < FRAME_SIZE:
            raise ValueError("max_buffer_bytes must be at least 44")
        self.max_buffer_bytes = int(max_buffer_bytes)
        self._buffer = bytearray()
        self.discarded_bytes_total = 0
        self.malformed_packets_total = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        """Discard a partial packet at a connection or stream boundary."""
        self.discarded_bytes_total += len(self._buffer)
        self._buffer.clear()

    def feed(self, data: bytes) -> tuple[Block, ...]:
        """Decode every complete packet; retain at most 43 pending bytes.

        Input may be arbitrarily split or contain many packets. Even a very
        large input is processed in bounded slices, not copied into one buffer.
        """
        view = memoryview(data).cast("B")
        offset = 0
        result = []
        while offset < len(view):
            count = min(self.max_buffer_bytes - len(self._buffer), len(view) - offset)
            self._buffer.extend(view[offset:offset + count])
            offset += count
            self._decode_available(result)
        return tuple(result)

    def _discard(self, count: int) -> None:
        self.discarded_bytes_total += count
        del self._buffer[:count]

    def _decode_available(self, result: list[Block]) -> None:
        while self._buffer:
            start = self._buffer.find(HEADER)
            if start < 0:
                # A trailing A5 may be the first byte of the next header.
                keep = int(self._buffer[-1] == HEADER[0])
                self._discard(len(self._buffer) - keep)
                return
            if start:
                self._discard(start)
            if len(self._buffer) < FRAME_SIZE:
                return
            row = (self._buffer[2] << 8) | self._buffer[3]
            if row >= ROWS:
                self.malformed_packets_total += 1
                # Do not skip all 44 bytes: another genuine header may overlap.
                self._discard(1)
                continue
            result.append(Block(row, struct.unpack_from(">20H", self._buffer, 4)))
            del self._buffer[:FRAME_SIZE]


class StreamParser:
    """Admit only confirmed, continuous row44 frames with original read tags.

    Confirmation requires eight adjacent complete frames from at least two
    rows. Thereafter each frame needs the next four-byte header/address as
    lookahead. A framing break immediately removes the lock. The final frame
    waits for that lookahead; a reset discards it rather than inventing proof.
    Byte segments retain caller-provided immutable read metadata, so delayed
    admission never changes a packet's completing read timestamp.
    """

    REQUIRED_FRAMES = 8

    def __init__(self, max_buffer_bytes=4096):
        if type(max_buffer_bytes) is not int or max_buffer_bytes < self.REQUIRED_FRAMES * FRAME_SIZE:
            raise ValueError("stream buffer must hold at least eight row frames")
        self.max_buffer_bytes = max_buffer_bytes
        self._buffer = bytearray()
        self._segments = deque()
        self._admitted = deque()
        self.confirmed = False
        self.discarded_bytes_total = 0
        self.malformed_packets_total = 0

    @property
    def buffered_bytes(self):
        return len(self._buffer)

    def reset(self):
        self.discarded_bytes_total += len(self._buffer) + len(self._admitted) * FRAME_SIZE
        self._buffer.clear()
        self._segments.clear()
        self._admitted.clear()
        self.confirmed = False

    def _consume(self, count):
        del self._buffer[:count]
        tag = None
        while count:
            size, tag = self._segments.popleft()
            take = min(size, count)
            count -= take
            if take < size:
                self._segments.appendleft((size - take, tag))
        return tag

    def _discard(self, count):
        self.discarded_bytes_total += count
        self._consume(count)

    def feed(self, data, read_tag):
        view = memoryview(data).cast("B")
        if len(view) > self.max_buffer_bytes:
            raise ValueError("stream reads must not exceed the configured byte cap")
        offset = 0
        result = []
        while offset < len(view):
            count = min(self.max_buffer_bytes - len(self._buffer), len(view) - offset)
            self._buffer.extend(view[offset:offset + count])
            self._segments.append((count, read_tag))
            offset += count
            self._decode_available(self._admitted)
        # Never split one original read across emitted batches with identical
        # clocks. Hold admitted packets until none of that read's bytes remain
        # awaiting lookahead or resynchronization.
        pending_tags = set()
        pending_bytes = 0
        for size, tag in self._segments:
            pending_bytes += size
            if pending_bytes >= FRAME_SIZE:
                pending_tags.add(tag)
        while self._admitted and self._admitted[0][1] not in pending_tags:
            result.append(self._admitted.popleft())
        return tuple(result)

    def _decode_available(self, result):
        while self._buffer:
            start = self._buffer.find(HEADER)
            if start != 0:
                self.confirmed = False
                if start < 0:
                    keep = int(self._buffer[-1] == HEADER[0])
                    self._discard(len(self._buffer) - keep)
                    return
                self._discard(start)
            if len(self._buffer) < FRAME_SIZE:
                return
            if not self.confirmed:
                rows = set()
                for index in range(self.REQUIRED_FRAMES):
                    offset = index * FRAME_SIZE
                    if len(self._buffer) < offset + FRAME_SIZE:
                        return
                    row = frame_key(self._buffer, offset)
                    if row is None:
                        self.malformed_packets_total += 1
                        self._discard(1)
                        break
                    rows.add(row)
                else:
                    if len(rows) < 2:
                        self._discard(FRAME_SIZE)
                        continue
                    self.confirmed = True
                if not self.confirmed:
                    continue
            if len(self._buffer) < FRAME_SIZE + 4:
                return
            row = frame_key(self._buffer)
            next_row = (self._buffer[FRAME_SIZE + 2] << 8) | self._buffer[FRAME_SIZE + 3]
            if (row is None or self._buffer[FRAME_SIZE:FRAME_SIZE + 2] != HEADER
                    or next_row >= ROWS):
                self.confirmed = False
                self.malformed_packets_total += 1
                self._discard(1)
                continue
            block = Block(row, struct.unpack_from(">20H", self._buffer, 4))
            tag = self._consume(FRAME_SIZE)
            result.append((block, tag))
