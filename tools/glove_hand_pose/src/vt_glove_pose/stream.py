"""Bounded raw IMU parsing, exclusive serial access, and portable CSV replay.

The wire format has no checksum, sequence counter, or device timestamp. Frame
synchronization checks all six node IDs but cannot prove payload integrity.
No serial configuration commands are sent to the device.
"""

from __future__ import annotations

import csv
import math
import os
from pathlib import Path
import struct
import sys
import time
from typing import Iterable, Iterator, TextIO

from .records import Sample


FRAME_SIZE = 84
NODE_BLOCK_SIZE = 14
MAX_READ_BYTES = 4096
CSV_FIELDS = (
    "host_parse_monotonic_s", "host_parse_wall_s", "node_id",
    "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb", "status",
)
_REQUIRED_FIELDS = CSV_FIELDS[:-1]
_OPTIONAL_FIELDS = {"status", "phase"}


class StreamError(RuntimeError):
    """A controlled serial/dependency error suitable for displaying in a UI."""


class ImuParser:
    """Incrementally find 84-byte frames containing IDs 1..6 or 7..12.

    At most 83 bytes survive a call. Even a large input is scanned in bounded
    chunks; the returned sample list naturally scales with valid input size.
    All samples completed in a call share that call's host parse timestamps.
    """

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames_ok = 0
        self.bytes_discarded = 0

    def reset(self) -> None:
        self.buffer.clear()
        self.frames_ok = 0
        self.bytes_discarded = 0

    def feed(
        self,
        data: bytes | bytearray | memoryview,
        host_monotonic_s: float | None = None,
        host_wall_s: float | None = None,
    ) -> list[Sample]:
        monotonic = time.monotonic() if host_monotonic_s is None else host_monotonic_s
        wall = time.time() if host_wall_s is None else host_wall_s
        # Validate timestamp inputs even when no complete frame is received.
        timestamp_check = Sample(1, (0, 0, 0, 0, 0, 0), monotonic, wall)
        monotonic, wall = timestamp_check.host_monotonic_s, timestamp_check.host_wall_s
        view = memoryview(data).cast("B")
        samples: list[Sample] = []
        for chunk_start in range(0, len(view), MAX_READ_BYTES):
            self.buffer.extend(view[chunk_start:chunk_start + MAX_READ_BYTES])
            cursor = 0
            last_start = len(self.buffer) - FRAME_SIZE
            while cursor <= last_start:
                first = self.buffer.find(b"\x01", cursor, last_start + 1)
                second = self.buffer.find(b"\x07", cursor, last_start + 1)
                candidates = [index for index in (first, second) if index >= 0]
                if not candidates:
                    self.bytes_discarded += last_start + 1 - cursor
                    cursor = last_start + 1
                    break
                start = min(candidates)
                self.bytes_discarded += start - cursor
                first_id = self.buffer[start]
                if not all(
                    self.buffer[start + index * NODE_BLOCK_SIZE] == first_id + index
                    for index in range(6)
                ):
                    self.bytes_discarded += 1
                    cursor = start + 1
                    continue
                for index in range(6):
                    offset = start + index * NODE_BLOCK_SIZE
                    samples.append(Sample(
                        node_id=first_id + index,
                        values=struct.unpack_from("<6h", self.buffer, offset + 2),
                        host_monotonic_s=monotonic,
                        host_wall_s=wall,
                        status=self.buffer[offset + 1],
                    ))
                self.frames_ok += 1
                cursor = start + FRAME_SIZE
            if cursor:
                del self.buffer[:cursor]
        return samples


def iter_csv(path: str | os.PathLike[str]) -> Iterator[Sample]:
    """Stream raw CSV with strict row validation and per-node time ordering.

    ``status`` and ``phase`` are optional. Phase is recording metadata, not a
    sensor value. Equal monotonic timestamps are valid for batched host reads;
    decreasing per-node timestamps are rejected. Wall time may move backwards
    when the host clock is adjusted.
    """

    with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle, strict=True)
        try:
            fields = reader.fieldnames
        except csv.Error as exc:
            raise ValueError(f"{path}: invalid CSV header: {exc}") from exc
        if fields is None:
            raise ValueError(f"{path}: CSV has no header")
        if len(fields) != len(set(fields)):
            raise ValueError(f"{path}: CSV header contains duplicate fields")
        missing = set(_REQUIRED_FIELDS) - set(fields)
        unexpected = set(fields) - set(_REQUIRED_FIELDS) - _OPTIONAL_FIELDS
        if missing or unexpected:
            details = []
            if missing:
                details.append("missing fields: " + ", ".join(sorted(missing)))
            if unexpected:
                details.append("unexpected fields: " + ", ".join(sorted(unexpected)))
            raise ValueError(f"{path}: " + "; ".join(details))
        last_times: dict[int, float] = {}
        try:
            for row in reader:
                try:
                    if None in row or any(value is None for value in row.values()):
                        raise ValueError("row has a different field count from the header")
                    status_text = row.get("status", "").strip()
                    sample = Sample(
                        node_id=int(row["node_id"]),
                        values=tuple(int(row[name]) for name in CSV_FIELDS[3:9]),
                        host_monotonic_s=float(row["host_parse_monotonic_s"]),
                        host_wall_s=float(row["host_parse_wall_s"]),
                        status=int(status_text) if status_text else None,
                    )
                    previous = last_times.get(sample.node_id)
                    if previous is not None and sample.host_monotonic_s < previous:
                        raise ValueError(f"nonmonotonic host timestamp for node {sample.node_id}")
                    last_times[sample.node_id] = sample.host_monotonic_s
                except (ValueError, TypeError) as exc:
                    raise ValueError(f"{path}: line {reader.line_num}: {exc}") from exc
                yield sample
        except csv.Error as exc:
            raise ValueError(f"{path}: line {reader.line_num}: invalid CSV: {exc}") from exc


class RawRecorder:
    """Streaming CSV recorder; exclusive creation prevents accidental overwrite."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self.path = Path(path)
        self._handle: TextIO | None = None
        self._writer = None
        self._closed = False

    def open(self) -> RawRecorder:
        if self._closed:
            raise RuntimeError("RawRecorder is closed")
        if self._handle is None:
            self._handle = self.path.open("x", encoding="utf-8", newline="")
            try:
                self._writer = csv.writer(self._handle)
                self._writer.writerow(CSV_FIELDS)
                self._handle.flush()
            except BaseException:
                self.close()
                raise
        return self

    def __enter__(self) -> RawRecorder:
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def write(self, samples: Iterable[Sample]) -> None:
        if self._handle is None or self._writer is None:
            raise RuntimeError("RawRecorder is not open")
        for sample in samples:
            if not isinstance(sample, Sample):
                raise TypeError("RawRecorder.write expects Sample instances")
            self._writer.writerow((
                sample.host_monotonic_s, sample.host_wall_s, sample.node_id,
                *sample.values, "" if sample.status is None else sample.status,
            ))
        self._handle.flush()

    def close(self) -> None:
        if self._handle is not None:
            try:
                self._handle.close()
            finally:
                self._handle = None
                self._writer = None
        self._closed = True


def _serial_module():
    try:
        import serial
    except ImportError as exc:
        raise StreamError("Serial support requires pyserial; install the package's serial dependencies") from exc
    return serial


def list_serial_ports() -> list[tuple[str, str]]:
    """Return explicit choices; never guess which port is the IMU board."""
    _serial_module()
    from serial.tools import list_ports
    return sorted((item.device, item.description or "") for item in list_ports.comports())


def _find_serial_port_holders(
    port: str, *, proc_root: Path = Path("/proc")
) -> list[tuple[int, int]]:
    """Best-effort Linux preflight, including other FDs in this process.

    pyserial's ``exclusive=True`` uses an advisory lock that older readers may
    not acquire. Visible /proc descriptors catch such readers before we open
    the device. Permission-hidden processes and open/check races remain
    possible; this is not a claim of atomic system-wide ownership.
    """
    if not sys.platform.startswith("linux"):
        return []
    target = os.path.realpath(port)
    holders: list[tuple[int, int]] = []
    try:
        processes = list(proc_root.iterdir())
    except OSError:
        return holders
    for process in processes:
        if not process.name.isdecimal():
            continue
        try:
            descriptors = list((process / "fd").iterdir())
        except OSError:
            continue  # A process can exit or be invisible to our credentials.
        for descriptor in descriptors:
            if not descriptor.name.isdecimal():
                continue
            try:
                linked = os.readlink(descriptor)
                if linked.startswith("/") and os.path.realpath(linked) == target:
                    holders.append((int(process.name), int(descriptor.name)))
            except OSError:
                continue  # Descriptors may close while /proc is being scanned.
    return sorted(holders)


def _claim_tty_exclusive(port) -> bool:
    """Ask the POSIX tty driver to reject subsequent ordinary opens, if supported."""
    if os.name != "posix":
        return False
    import fcntl
    import termios
    command = getattr(termios, "TIOCEXCL", None)
    if command is None:
        return False
    fcntl.ioctl(port.fileno(), command)
    return True


def _release_tty_exclusive(port) -> None:
    import fcntl
    import termios
    command = getattr(termios, "TIOCNXCL", None)
    if command is not None:
        try:
            fcntl.ioctl(port.fileno(), command)
        except OSError:
            pass  # A disconnected tty still needs its file descriptor closed.


class SerialSource:
    """Read-only source with visible-holder checks and POSIX tty exclusivity.

    Linux /proc preflight detects non-cooperating existing readers. TIOCEXCL,
    when available, rejects later ordinary opens; pyserial also takes its
    advisory exclusive lock. None can exclude every privileged/hidden process
    or eliminate the interval between preflight and opening the device.
    """

    def __init__(self, port: str, baud: int = 115200) -> None:
        if not isinstance(port, str) or not port.strip():
            raise ValueError("Select an explicit serial port")
        if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
            raise ValueError("baud must be a positive integer")
        self.port = port
        self.baud = baud
        self.parser = ImuParser()
        self._serial = None
        self._tty_exclusive = False
        self._reset_performance()

    def _reset_performance(self) -> None:
        # Last observation and scalar counters only: no raw-data history.
        self._performance = dict(
            serial_backlog_before_bytes=None, serial_backlog_after_bytes=None,
            serial_backlog_peak_bytes=0, raw_read_elapsed_ms=0.0,
            parse_elapsed_ms=0.0, raw_read_bytes=0, raw_read_bytes_total=0,
            raw_read_calls=0, parser_frames_ok=0, parser_bytes_discarded=0,
        )

    @property
    def performance(self) -> dict:
        """Bounded host observations, not sensor age or end-to-end latency.

        Queue sizes describe bytes visible to this host serial descriptor.
        They cannot expose buffering in firmware, USB transport, or the sensor.
        Reading this property performs no serial I/O.
        """
        return dict(self._performance)

    def open(self) -> SerialSource:
        if self._serial is not None:
            return self
        serial = _serial_module()
        holders = _find_serial_port_holders(self.port)
        if holders:
            owners = ", ".join(f"PID {pid} (fd {fd})" for pid, fd in holders)
            raise StreamError(
                f"Serial port {self.port!r} is already open by visible Linux process(es): {owners}. "
                "Close the owning application first; no process was changed."
            )
        kwargs = {"timeout": 0.020, "write_timeout": 0.2}
        if os.name == "posix":
            kwargs["exclusive"] = True
        try:
            self._serial = serial.Serial(self.port, self.baud, **kwargs)
            self._tty_exclusive = _claim_tty_exclusive(self._serial)
        except (serial.SerialException, OSError) as exc:
            if self._serial is not None:
                opened, self._serial = self._serial, None
                opened.close()
            self._tty_exclusive = False
            raise StreamError(
                f"Cannot open serial port {self.port!r}: {exc}. "
                "Check the selected device and permissions; close any monitor already using it."
            ) from exc
        self.parser.reset()
        self._reset_performance()
        return self

    def __enter__(self) -> SerialSource:
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def read(self) -> list[Sample]:
        if self._serial is None:
            raise RuntimeError("SerialSource is not open")
        try:
            waiting = self._serial.in_waiting
            started = time.perf_counter()
            data = self._serial.read(min(MAX_READ_BYTES, max(1, waiting)))
            read_elapsed_ms = (time.perf_counter() - started) * 1000
        except (OSError, _serial_module().SerialException) as exc:
            raise StreamError(f"Serial read failed for {self.port!r}: {exc}") from exc
        try:
            after = self._serial.in_waiting
        except (OSError, _serial_module().SerialException):
            # An optional observation must not discard already-read samples.
            # The next real read will report a disconnection if it persists.
            after = None
        started = time.perf_counter()
        samples = self.parser.feed(data)
        metrics = self._performance
        metrics.update(
            serial_backlog_before_bytes=waiting, serial_backlog_after_bytes=after,
            serial_backlog_peak_bytes=max(metrics["serial_backlog_peak_bytes"], waiting, after or 0),
            raw_read_elapsed_ms=read_elapsed_ms,
            parse_elapsed_ms=(time.perf_counter() - started) * 1000,
            raw_read_bytes=len(data), raw_read_bytes_total=metrics["raw_read_bytes_total"] + len(data),
            raw_read_calls=metrics["raw_read_calls"] + 1,
            parser_frames_ok=self.parser.frames_ok, parser_bytes_discarded=self.parser.bytes_discarded,
        )
        return samples

    def close(self) -> None:
        if self._serial is not None:
            serial, self._serial = self._serial, None
            try:
                if self._tty_exclusive:
                    _release_tty_exclusive(serial)
            finally:
                self._tty_exclusive = False
                serial.close()


class ReplaySource:
    """Nonblocking, host-timestamp-paced replay with a bounded read backlog.

    ``read()`` returns an empty list when the next sample is not yet due. Call
    periodically until ``eof``; at most ``max_samples`` are returned per call.
    Replay preserves the original recorded host timestamps.
    """

    def __init__(
        self, path: str | os.PathLike[str], speed: float = 1.0, *, max_samples: int = 256
    ) -> None:
        if isinstance(speed, bool) or not isinstance(speed, (int, float)):
            raise ValueError("replay speed must be finite and positive")
        try:
            speed = float(speed)
        except (OverflowError, ValueError) as exc:
            raise ValueError("replay speed must be finite and positive") from exc
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("replay speed must be finite and positive")
        if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 1:
            raise ValueError("max_samples must be a positive integer")
        self.path = Path(path)
        self.speed = float(speed)
        self.max_samples = max_samples
        self.eof = False
        self._iterator: Iterator[Sample] | None = None
        self._next: Sample | None = None
        self._start_host_s = 0.0
        self._start_local_s = 0.0
        self._last_emitted_host_s: float | None = None

    @property
    def timeline_s(self) -> float | None:
        """Current playback position in the recording's host-monotonic domain.

        It advances through gaps even when ``read()`` returns no samples, so a
        viewer can expire freshness and interrupted calibration normally. At
        EOF it freezes at the last emitted timestamp for a historical snapshot.
        An unopened source or an empty recording has no presentation timestamp.
        """
        if self._iterator is None:
            return None
        if self.eof:
            return self._last_emitted_host_s
        elapsed = max(0.0, time.monotonic() - self._start_local_s)
        return self._start_host_s + elapsed * self.speed

    def open(self) -> ReplaySource:
        if self._iterator is not None:
            return self
        iterator = iter_csv(self.path)
        try:
            first = next(iterator, None)
        except BaseException:
            iterator.close()
            raise
        self._iterator = iterator
        self._next = first
        self.eof = first is None
        self._last_emitted_host_s = None
        self._start_host_s = first.host_monotonic_s if first is not None else 0.0
        self._start_local_s = time.monotonic()
        return self

    def __enter__(self) -> ReplaySource:
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def read(self) -> list[Sample]:
        if self._iterator is None:
            raise RuntimeError("ReplaySource is not open")
        elapsed = time.monotonic() - self._start_local_s
        result: list[Sample] = []
        while self._next is not None and len(result) < self.max_samples:
            due = (self._next.host_monotonic_s - self._start_host_s) / self.speed
            if due > elapsed:
                break
            result.append(self._next)
            stamp = self._next.host_monotonic_s
            self._last_emitted_host_s = stamp if self._last_emitted_host_s is None else max(stamp, self._last_emitted_host_s)
            self._next = next(self._iterator, None)
        self.eof = self._next is None
        return result

    def close(self) -> None:
        if self._iterator is not None:
            self._iterator.close()
            self._iterator = None
        self._next = None
