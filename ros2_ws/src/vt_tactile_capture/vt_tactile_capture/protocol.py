"""Decode the existing 26-byte glove tactile protocol, without inventing CRCs.

There is no documented checksum or hardware clock/sequence in this protocol.
Only the header and address bounds can be checked. In particular, a corrupted
payload with a plausible address cannot be detected by this parser.
"""

from dataclasses import dataclass
import struct


ROWS = 80
COLS = 80
COLUMN_GROUPS = 8
VALUES_PER_BLOCK = 10
FRAME_SIZE = 26
HEADER = b"\xa5\x5a"


@dataclass(frozen=True)
class Block:
    """ADC values at ``(row, column_group + 8*k)``, in wire order."""

    row: int
    column_group: int
    status_raw: int
    adc: tuple[int, ...]


class Parser:
    """Incremental, bounded-memory parser with sliding header resynchronization.

    Counters are lifetime totals, including bytes discarded on ``reset``.
    ``malformed_packets_total`` counts header candidates with invalid addresses,
    not all corrupt wire packets (which cannot be known without a checksum).
    """

    def __init__(self, max_buffer_bytes: int = 4096):
        if max_buffer_bytes < FRAME_SIZE:
            raise ValueError("max_buffer_bytes must be at least 26")
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
        """Decode every complete packet; retain at most 25 pending bytes.

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
            column_group = self._buffer[5]
            if row >= ROWS or column_group >= COLUMN_GROUPS:
                self.malformed_packets_total += 1
                # Do not skip all 26 bytes: another genuine header may overlap.
                self._discard(1)
                continue
            result.append(Block(row, column_group, self._buffer[4],
                                struct.unpack_from(">10H", self._buffer, 6)))
            del self._buffer[:FRAME_SIZE]
