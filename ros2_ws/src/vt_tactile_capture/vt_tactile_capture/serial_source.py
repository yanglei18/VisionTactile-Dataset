"""A bounded, read-only serial producer, independent of ROS and any UI.

One worker thread owns the serial object for its entire lifetime. The queue
discards the oldest batch when full so health events cannot sit behind an
unbounded backlog. ``producer_dropped_total`` counts discarded *batches*, while
``first_packet_index`` counts all decoded packets, including dropped batches.

On Linux, ownership checks inspect accessible /proc file descriptors before
opening, then use pyserial's exclusive flock, a defensive flock and TIOCEXCL,
and check again after opening. This detects older non-exclusive Glove.py
readers owned by the same user. Privileged processes and inaccessible /proc
entries cannot be ruled out; root can also bypass kernel TIOCEXCL.
"""

from dataclasses import dataclass, replace
import errno
import math
import os
import queue
import stat
import threading
import time
import uuid
from typing import Callable, Optional
from numbers import Integral

from .protocol import Block, Parser


CLOCK_DOMAIN = "CLOCK_MONOTONIC"
TIMING_SOURCE = "host_receive"
MAX_READ_BYTES = 4096


@dataclass(frozen=True)
class Batch:
    """Fields of TactileSampleArray except its ROS Header."""

    schema_version: int = 1
    hand: str = "right"
    session_id: str = ""
    stream_epoch: int = 0
    sequence: int = 0
    first_packet_index: int = 0
    host_realtime_ns: int = 0
    host_monotonic_ns: int = 0
    clock_domain: str = CLOCK_DOMAIN
    timing_source: str = TIMING_SOURCE
    connected: bool = False
    time_valid: bool = False
    reason: str = "not_started"
    producer_dropped_total: int = 0
    discarded_bytes_total: int = 0
    malformed_packets_total: int = 0
    blocks: tuple[Block, ...] = ()


class PortInUseError(OSError):
    """The explicit port already has another visible owner."""


def _port_owners(port: str, exclude_fd: Optional[int] = None) -> list[str]:
    """Find existing descriptors, including non-cooperating legacy readers."""
    target = os.stat(port)
    if not stat.S_ISCHR(target.st_mode):
        raise ValueError("serial port must be a character device")
    owners = []
    own_pid = os.getpid()
    with os.scandir("/proc") as processes:
        for process in processes:
            if not process.name.isdecimal():
                continue
            try:
                with os.scandir(os.path.join(process.path, "fd")) as descriptors:
                    for descriptor in descriptors:
                        if (int(process.name) == own_pid
                                and descriptor.name == str(exclude_fd)):
                            continue
                        try:
                            info = os.stat(descriptor.path)
                        except (FileNotFoundError, PermissionError, ProcessLookupError):
                            continue
                        if stat.S_ISCHR(info.st_mode) and info.st_rdev == target.st_rdev:
                            owners.append(f"pid={process.name},fd={descriptor.name}")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
    return owners


def _assert_port_free(port: str, exclude_fd: Optional[int] = None) -> None:
    owners = _port_owners(port, exclude_fd)
    if owners:
        raise PortInUseError(errno.EBUSY, "serial port already in use: " + ", ".join(owners), port)


class SerialWorker:
    """Read only the explicitly configured port; never auto-select or write.

    ``serial_factory`` is a test seam. A supplied factory receives the pyserial
    keyword arguments and owns its own locking policy; production must leave
    it unset to enable the Linux ownership protections above.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        hand: str = "right",
        queue_capacity: int = 128,
        read_timeout: float = 0.05,
        no_data_timeout: float = 2.0,
        reconnect_interval_sec: float = 1.0,
        serial_factory: Optional[Callable] = None,
        max_clock_step_ns: int = 5_000_000,
        read_batch_bytes: int = 1,
    ):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("an explicit nonempty serial port is required")
        if baudrate <= 0 or queue_capacity < 1:
            raise ValueError("baudrate and queue_capacity must be positive")
        if hand not in ("left", "right"):
            raise ValueError("hand must be left or right")
        if (isinstance(read_batch_bytes, bool) or not isinstance(read_batch_bytes, Integral)
                or not 1 <= read_batch_bytes <= MAX_READ_BYTES):
            raise ValueError(f"read_batch_bytes must be an integer in [1, {MAX_READ_BYTES}]")
        if (isinstance(max_clock_step_ns, bool) or not isinstance(max_clock_step_ns, Integral)
                or max_clock_step_ns < 0):
            raise ValueError("max_clock_step_ns must be a nonnegative integer")
        for name, value in (("read_timeout", read_timeout),
                            ("no_data_timeout", no_data_timeout),
                            ("reconnect_interval_sec", reconnect_interval_sec)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.port = port
        self.baudrate = int(baudrate)
        self.hand = hand
        self.read_timeout = float(read_timeout)
        self.read_batch_bytes = int(read_batch_bytes)
        self.no_data_timeout = float(no_data_timeout)
        self.reconnect_interval_sec = float(reconnect_interval_sec)
        self.max_clock_step_ns = int(max_clock_step_ns)
        self.session_id = str(uuid.uuid4())
        self._factory = serial_factory
        self._queue = queue.Queue(maxsize=queue_capacity)
        self._stop = threading.Event()
        self._thread = None
        self._parser = Parser()
        self._stream_epoch = 0
        self._sequence = 0
        self._packet_index = 0
        self._producer_dropped_total = 0
        self._previous_clock = None
        self._pending_clock_fault = ""
        self._connected = False

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def producer_dropped_total(self) -> int:
        return self._producer_dropped_total

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("a SerialWorker can be started only once")
        self._thread = threading.Thread(target=self._run, name="tactile-serial", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> bool:
        """Request shutdown and join for at most timeout seconds; never hang."""
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("stop timeout must be finite and nonnegative")
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
        return not self.running

    def get_nowait(self) -> Batch:
        return self._queue.get_nowait()

    def drain(self, limit: int = 128) -> list[Batch]:
        if limit < 0:
            raise ValueError("drain limit must be nonnegative")
        batches = []
        for _ in range(limit):
            try:
                batches.append(self.get_nowait())
            except queue.Empty:
                break
        return batches

    def _clock(self) -> tuple[int, int, bool]:
        # These calls occur immediately after read() returns, never at publish.
        realtime_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()
        valid = realtime_ns > 0 and monotonic_ns > 0
        fault = "" if valid else "invalid_time"
        if self._previous_clock is not None:
            delta_real = realtime_ns - self._previous_clock[0]
            delta_mono = monotonic_ns - self._previous_clock[1]
            if delta_real <= 0 or delta_mono <= 0:
                valid, fault = False, "clock_backwards"
            elif abs(delta_real - delta_mono) > self.max_clock_step_ns:
                valid, fault = False, "clock_step"
        if fault:
            # A clock discontinuity during an incomplete/no-data read must not
            # disappear before the next decoded batch reaches the consumer.
            self._pending_clock_fault = fault
        self._previous_clock = (realtime_ns, monotonic_ns)
        return realtime_ns, monotonic_ns, valid

    def _emit(self, *, connected: bool, reason: str,
              blocks: tuple[Block, ...] = (), clocks=None) -> None:
        realtime_ns, monotonic_ns, clock_valid = clocks if clocks is not None else self._clock()
        if blocks and self._pending_clock_fault:
            clock_valid = False
            reason = self._pending_clock_fault
            self._pending_clock_fault = ""
        elif blocks and not clock_valid:
            reason = "invalid_time"
        batch = Batch(
            hand=self.hand, session_id=self.session_id, stream_epoch=self._stream_epoch,
            sequence=self._sequence, first_packet_index=self._packet_index,
            host_realtime_ns=max(0, realtime_ns), host_monotonic_ns=max(0, monotonic_ns),
            connected=connected, time_valid=bool(blocks) and clock_valid,
            reason=reason,
            producer_dropped_total=self._producer_dropped_total,
            discarded_bytes_total=self._parser.discarded_bytes_total,
            malformed_packets_total=self._parser.malformed_packets_total, blocks=blocks,
        )
        self._sequence += 1
        self._packet_index += len(blocks)
        self._connected = connected
        try:
            self._queue.put_nowait(batch)
        except queue.Full:
            # Single producer. The consumer may empty the queue between these
            # operations, so only count a drop when an item was really removed.
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            else:
                self._producer_dropped_total += 1
            self._queue.put_nowait(replace(
                batch, producer_dropped_total=self._producer_dropped_total))

    def _open_serial(self):
        options = dict(port=self.port, baudrate=self.baudrate,
                       timeout=self.read_timeout, write_timeout=self.read_timeout,
                       exclusive=True)
        if self._factory is not None:
            return self._factory(**options)
        import fcntl
        import serial
        import termios

        _assert_port_free(self.port)
        device = serial.Serial(**options)
        try:
            fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(device.fileno(), termios.TIOCEXCL)
            _assert_port_free(self.port, exclude_fd=device.fileno())
        except BaseException:
            self._close_serial(device)
            raise
        return device

    def _close_serial(self, device) -> None:
        if self._factory is None:
            import fcntl
            import termios
            try:
                fcntl.ioctl(device.fileno(), termios.TIOCNXCL)
            except (OSError, ValueError):
                pass
        try:
            device.close()
        except (OSError, ValueError):
            pass

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                device = None
                try:
                    device = self._open_serial()
                    self._stream_epoch += 1
                    self._parser.reset()
                    self._previous_clock = None
                    self._pending_clock_fault = ""
                    self._emit(connected=True, reason="connected_waiting_data")
                    last_packet_ns = time.monotonic_ns()
                    last_byte_ns = last_packet_ns
                    timeout_reported = False
                    while not self._stop.is_set():
                        # Ask pyserial to accumulate a real read batch, bounded
                        # by its timeout. Never merge already timestamped reads.
                        count = min(MAX_READ_BYTES, max(self.read_batch_bytes, int(device.in_waiting)))
                        data = device.read(count)
                        clocks = self._clock()
                        if self._stop.is_set():
                            break
                        if data:
                            last_byte_ns = clocks[1]
                        blocks = self._parser.feed(data)
                        if blocks:
                            last_packet_ns = clocks[1]
                            timeout_reported = False
                            self._emit(connected=True, reason="ok", blocks=blocks, clocks=clocks)
                        elif (not timeout_reported
                              and clocks[1] - last_packet_ns >= self.no_data_timeout * 1e9):
                            reason = ("no_data_timeout" if clocks[1] - last_byte_ns
                                      >= self.no_data_timeout * 1e9 else "no_valid_packets")
                            # An incomplete pre-fault packet must never be
                            # completed by bytes arriving after the boundary.
                            self._parser.reset()
                            self._emit(connected=True, reason=reason, clocks=clocks)
                            timeout_reported = True
                except Exception as exc:
                    self._parser.reset()
                    self._emit(connected=False, reason=f"serial_error: {type(exc).__name__}: {exc}")
                finally:
                    if device is not None:
                        self._close_serial(device)
                if not self._stop.is_set():
                    self._stop.wait(self.reconnect_interval_sec)
        finally:
            self._parser.reset()
            self._emit(connected=False, reason="stopped")
