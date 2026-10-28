"""Bounded, explicit serial protocol identification, independent of the UI.

The caller must exclude its active ports before scheduling a probe. The shared
serial transport also enforces exclusive ownership. No command is written;
opening a serial device can still have operating-system or firmware effects.

Identification is evidence of repeated supported framing, not a hardware model,
sensor layout, checksum, or device-identity verification. Neither wire protocol
provides a documented checksum. A silent or incomplete stream remains unknown.
"""
from __future__ import annotations

from dataclasses import dataclass
import errno
from numbers import Integral
import time

from vt_fsglove12.live_stream import FRAME_SIZE as IMU_FRAME_SIZE, NODE_BLOCK_SIZE, ImuParser
from vt_tactile_capture.protocol import (
    COLUMN_GROUPS, FRAME_SIZE as TACTILE_FRAME_SIZE, HEADER as TACTILE_HEADER,
    ROWS as TACTILE_ROWS, Parser as TactileParser,
)
from vt_tactile_capture.serial_access import PortInUseError


PROBE_SECONDS = .65
READ_TIMEOUT = .05
MAX_READ_BYTES = 4096
MAX_BYTES = 64 * 1024
MAX_READS = 128
MAX_BAUDRATES = 2


@dataclass(frozen=True)
class ProbeResult:
    role: str = "unknown"
    detail: str = ""


def _imu_frame_key(buffer, offset):
    first = buffer[offset]
    if first in (1, 7) and all(buffer[offset + i * NODE_BLOCK_SIZE] == first + i for i in range(6)):
        return first
    return None


def _tactile_frame_key(buffer, offset):
    if buffer[offset:offset + len(TACTILE_HEADER)] != TACTILE_HEADER:
        return None
    row = (buffer[offset + 2] << 8) | buffer[offset + 3]
    group = buffer[offset + 5]
    return (row, group) if row < TACTILE_ROWS and group < COLUMN_GROUPS else None


class _ConsecutiveFrames:
    """Require adjacent complete wire frames; never decode their payload.

    Existing parsers deliberately resynchronize after skipped bytes. Their
    lifetime packet counts alone can mistake prefixes of a different packet
    format for this protocol. This independent framing evidence requires an
    uninterrupted run at the protocol's exact frame size. Partial host reads
    retain the run; an intervening wire byte breaks it. Noise before/after a
    valid run does not erase a protocol already observed during this probe.
    """
    def __init__(self, frame_size, key, required):
        self._size, self._key, self._required = frame_size, key, required
        self._buffer = bytearray()
        self._count = 0
        self._keys = set()
        self.confirmed = False

    def feed(self, data):
        view = memoryview(data).cast("B")
        for start in range(0, len(view), MAX_READ_BYTES):
            self._buffer.extend(view[start:start + MAX_READ_BYTES])
            cursor = 0
            while cursor + self._size <= len(self._buffer):
                key = self._key(self._buffer, cursor)
                if key is None:
                    self._count = 0
                    self._keys.clear()
                    cursor += 1
                    continue
                self._count = min(self._count + 1, self._required)
                # Only two distinct addresses/banks are needed; keep bounded
                # state independently of any unusually large caller input.
                if len(self._keys) < 2:
                    self._keys.add(key)
                if self._count >= self._required and len(self._keys) >= 2:
                    self.confirmed = True
                cursor += self._size
            if cursor:
                del self._buffer[:cursor]


class _Evidence:
    def __init__(self):
        self.imu = ImuParser()
        self.tactile = TactileParser()
        self.imu_nodes = set()
        self.tactile_addresses = set()
        self.tactile_blocks = 0
        self._imu_frames = _ConsecutiveFrames(IMU_FRAME_SIZE, _imu_frame_key, 4)
        self._tactile_frames = _ConsecutiveFrames(TACTILE_FRAME_SIZE, _tactile_frame_key, 8)

    def feed(self, data):
        # Classification uses framing only; no filter, solver, clocks, or
        # physical-unit assumptions are needed for these parser inputs.
        self.imu_nodes.update(sample.node_id for sample in self.imu.feed(
            data, host_monotonic_s=0., host_wall_s=0.))
        blocks = self.tactile.feed(data)
        self.tactile_blocks += len(blocks)
        self.tactile_addresses.update((block.row, block.column_group) for block in blocks)
        self._imu_frames.feed(data)
        self._tactile_frames.feed(data)

    def result(self):
        imu = self.imu.frames_ok >= 4 and len(self.imu_nodes) == 12 and self._imu_frames.confirmed
        tactile = (self.tactile_blocks >= 8 and len(self.tactile_addresses) >= 2
                   and self._tactile_frames.confirmed)
        if imu and tactile:
            return ProbeResult("ambiguous", "Both supported wire protocols were observed.")
        if imu:
            return ProbeResult("imu")
        if tactile:
            return ProbeResult("tactile")
        return ProbeResult()


def _cancelled(cancel):
    return cancel is not None and cancel.is_set()


def _open(port, baudrate, **kwargs):
    from vt_tactile_capture.serial_access import open_serial
    return open_serial(port, baudrate, **kwargs)


def _close(device):
    from vt_tactile_capture.serial_access import close_serial
    close_serial(device)


def _failure(exc):
    # Windows access denied can mean a permissions problem as well as another
    # reader. Do not label that ambiguous failure as a confirmed busy port.
    busy = (getattr(exc, "errno", None) in {errno.EBUSY, errno.EAGAIN}
            or getattr(exc, "winerror", None) == 32
            or isinstance(exc, PortInUseError)
            or "exclusively lock" in str(exc).lower())
    return ProbeResult("busy" if busy else "error", str(exc)[:2048])


def _scan(port, baudrate, cancel, opener, closer, clock):
    device = None
    result = ProbeResult()
    try:
        deadline = clock() + PROBE_SECONDS
        device = opener(port, baudrate, read_timeout=READ_TIMEOUT, passive=True)
        evidence, received = _Evidence(), 0
        for _ in range(MAX_READS):
            if _cancelled(cancel) or clock() >= deadline or received >= MAX_BYTES:
                break
            data = device.read(min(MAX_READ_BYTES, MAX_BYTES - received))
            if _cancelled(cancel):
                break
            if not isinstance(data, (bytes, bytearray, memoryview)):
                raise ValueError("Serial read did not return bytes")
            if len(data) > min(MAX_READ_BYTES, MAX_BYTES - received):
                raise ValueError("Serial read exceeded its requested size")
            received += len(data)
            evidence.feed(data)
        result = evidence.result()
        if result.role == "unknown":
            result = ProbeResult("unknown", f"bytes_received={received}; "
                                 f"imu_frames={evidence.imu.frames_ok}; "
                                 f"tactile_blocks={evidence.tactile_blocks}")
    except (OSError, ValueError, RuntimeError) as exc:
        result = _failure(exc)
    finally:
        if device is not None:
            try:
                closer(device)
            except (OSError, ValueError, RuntimeError) as exc:
                result = _failure(exc)
    # A cancellation during open/read/close must never publish positive evidence.
    return ProbeResult("unknown", "cancelled") if _cancelled(cancel) else result


def probe_port(port, *, baudrates=(115200,), cancel=None, opener=None,
               closer=None, clock=time.monotonic):
    """Identify repeated IMU/tactile frames on one explicitly requested port.

    At most two distinct baud rates are attempted, each with its own 0.65-second,
    64-KiB and 128-read budget. Production reads have a 50-ms timeout; platform
    device-open calls cannot be preempted here. A second rate is attempted only
    when the first produced no identification. Evidence never crosses rates.

    ``cancel`` is an optional threading.Event. An injected opener must accept
    the shared serial transport arguments and provide bounded ``read(size)``;
    an injected closer owns its matching cleanup. No background worker starts
    here and no device is opened when cancellation is already requested.
    """
    if not isinstance(port, str) or not port.strip():
        raise ValueError("An explicit serial port is required")
    rates = []
    for value in baudrates:
        if isinstance(value, bool) or not isinstance(value, Integral) or not 1 <= value <= 4_000_000:
            raise ValueError("Baud rates must be integers from 1 to 4000000")
        if value not in rates:
            rates.append(int(value))
        if len(rates) > MAX_BAUDRATES:
            raise ValueError("At most two distinct baud rates may be probed")
    if not rates:
        raise ValueError("At least one baud rate is required")
    opener, closer = opener or _open, closer or _close
    result = ProbeResult()
    for rate in rates:
        if _cancelled(cancel):
            return ProbeResult("unknown", "cancelled")
        result = _scan(port, rate, cancel, opener, closer, clock)
        if result.role != "unknown" or _cancelled(cancel):
            break
    return result
