"""Bounded, display-only local tactile IPC; no ROS, serial, Qt, or pose imports.

Wire v2 is an eight-byte prefix, a bounded UTF-8 JSON header, then fixed-size
little-endian ADC/clock arrays and packed validity/update masks. No pickle is
used. Datagram delivery is best effort; this is never an archival data channel.
The binary format adds no claim of a hardware checksum or synchronous scan.
Legacy wire v1 states are decoded and explicitly cropped to the 20x20 area.
"""

from __future__ import annotations

import errno
import fcntl
import hashlib
import json
from numbers import Integral
import os
from pathlib import Path
import socket
import stat
import struct
from types import SimpleNamespace

import numpy as np

from .grid import BLOCK_COUNT, CELL_COUNT, COLS, ROWS, SCHEMA_VERSION, normalize_state


MAX_PACKET_BYTES = 32_768
MAX_HEADER_BYTES = 4096
MAX_DRAIN_PACKETS = 64
_PREFIX = struct.Struct("!4sHH")
_MAGIC = b"VTTD"
_VERSION = 2
_MASK_BYTES = CELL_COUNT // 8
_BODY_BYTES = CELL_COUNT * 2 + _MASK_BYTES * 2 + BLOCK_COUNT * 8 * 2
_UINT64_MAX = (1 << 64) - 1
_INTEGER_FIELDS = (
    "source_sequence", "host_realtime_ns", "host_monotonic_ns",
    "snapshot_monotonic_ns", "max_age_ns", "producer_dropped_total",
)


def default_socket_path(hand: str = "right", namespace: str | None = None) -> str:
    """Separate hand endpoints; custom ROS namespaces also get distinct paths."""
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    suffix = ""
    if namespace is not None:
        if not isinstance(namespace, str) or (namespace and not namespace.startswith("/")):
            raise ValueError("namespace must be an absolute ROS namespace")
        namespace = namespace.rstrip("/")
        if namespace not in ("", f"/gloves/{hand}/tactile"):
            suffix = "-" + hashlib.sha256(namespace.encode("utf-8")).hexdigest()[:12]
    return f"/tmp/vt-tactile-{os.getuid()}/{hand}{suffix}.sock"


def _uint(value, name: str, maximum: int = _UINT64_MAX) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an unsigned integer <= {maximum}")
    return int(value)


def _text(value, name: str, limit: int, *, empty: bool = False) -> str:
    if not isinstance(value, str) or (not value and not empty) or "\x00" in value:
        raise ValueError(f"invalid {name}")
    try:
        encoded = value.encode("utf-8")
    except UnicodeError as exc:
        raise ValueError(f"invalid UTF-8 {name}") from exc
    if len(encoded) > limit:
        raise ValueError(f"{name} exceeds {limit} UTF-8 bytes")
    return value


def _header(values: dict, *, wire_version: int = _VERSION) -> dict:
    """Validate both outgoing and incoming metadata using the same contract."""
    if not isinstance(values, dict):
        raise ValueError("display header must be an object")
    try:
        result = {
            "schema_version": _uint(values["schema_version"], "schema_version", 65535),
            "rows": _uint(values["rows"], "rows", 65535),
            "cols": _uint(values["cols"], "cols", 65535),
            "stream_epoch": _uint(values["stream_epoch"], "stream_epoch", (1 << 32) - 1),
        }
        expected = (SCHEMA_VERSION, ROWS, COLS) if wire_version == 2 else (1, 80, 80)
        if (result["schema_version"], result["rows"], result["cols"]) != expected:
            raise ValueError("unsupported tactile schema or grid shape")
        for name in _INTEGER_FIELDS:
            result[name] = _uint(values[name], name)
        for name, limit, empty in (
            ("hand", 8, False), ("session_id", 128, False),
            ("clock_domain", 64, False), ("timing_source", 64, False),
            ("layout_id", 128, True), ("unit", 32, False),
            ("reason", 1024, True), ("frame_id", 128, True),
        ):
            result[name] = _text(values[name], name, limit, empty=empty)
        if result["hand"] not in ("left", "right") or result["unit"] != "adc_counts":
            raise ValueError("unsupported hand or tactile unit")
        for name in ("connected", "time_valid"):
            if not isinstance(values[name], (bool, np.bool_)):
                raise ValueError(f"{name} must be boolean")
            result[name] = bool(values[name])
    except KeyError as exc:
        raise ValueError(f"missing display field {exc.args[0]}") from exc
    return result


def _integer_array(values, count: int, dtype: str, name: str) -> np.ndarray:
    try:
        result = np.asarray(values)
        limit = np.iinfo(dtype).max
        if (result.shape != (count,) or result.dtype.kind not in "iu"
                or np.any(result < 0) or np.any(result > limit)):
            raise ValueError(f"{name} must contain {count} unsigned {dtype} integers")
        return result.astype(dtype, copy=False)
    except (TypeError, OverflowError) as exc:
        raise ValueError(f"invalid {name} array") from exc


def _mask(values, name: str) -> np.ndarray:
    result = np.asarray(values)
    if (result.shape != (CELL_COUNT,) or result.dtype.kind not in "biu"
            or np.any((result != 0) & (result != 1))):
        raise ValueError(f"{name} must contain {CELL_COUNT} booleans")
    return np.packbits(result.astype(np.uint8), bitorder="little")


def encode_state(message) -> bytes:
    """Encode one full rolling state, retaining zeros and invalidity separately."""
    names = ("schema_version", "rows", "cols", "stream_epoch", "hand", "session_id",
             "clock_domain", "timing_source", "connected", "time_valid") + _INTEGER_FIELDS[:-1]
    try:
        message = normalize_state(message)
        values = {name: getattr(message, name) for name in names}
        values.update(
            producer_dropped_total=getattr(message, "producer_dropped_total", 0),
            layout_id=getattr(message, "layout_id", ""), unit=getattr(message, "unit", "adc_counts"),
            reason=getattr(message, "reason", ""),
            frame_id=getattr(getattr(message, "header", None), "frame_id", ""),
        )
        metadata = _header(values)
        header = json.dumps(metadata, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        if len(header) > MAX_HEADER_BYTES:
            raise ValueError("display JSON header is too large")
        adc = _integer_array(message.adc, CELL_COUNT, "<u2", "adc")
        valid = _mask(message.valid_mask, "valid_mask")
        updated = _mask(getattr(message, "updated_mask", np.zeros(CELL_COUNT, dtype=bool)), "updated_mask")
        mono = _integer_array(message.block_monotonic_ns, BLOCK_COUNT, "<u8", "block_monotonic_ns")
        real = _integer_array(getattr(message, "block_realtime_ns", np.zeros(BLOCK_COUNT, dtype=np.uint64)),
                              BLOCK_COUNT, "<u8", "block_realtime_ns")
    except (AttributeError, KeyError) as exc:
        raise ValueError("incomplete tactile display state") from exc
    return (_PREFIX.pack(_MAGIC, _VERSION, len(header)) + header + adc.tobytes()
            + valid.tobytes() + updated.tobytes() + mono.tobytes() + real.tobytes())


def decode_state(payload: bytes) -> SimpleNamespace:
    """Decode safely bounded bytes into a ROS/Qt-free state-shaped object.

    Arrays are independent writable copies. ``header.stamp`` is reconstructed
    from the preserved source host_realtime_ns, never from GUI receipt time.
    Block clocks retain their original values for cell-age calculation.
    """
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise ValueError("display datagram must be bytes")
    if not _PREFIX.size <= len(payload) <= MAX_PACKET_BYTES:
        raise ValueError("invalid display datagram size")
    magic, version, header_size = _PREFIX.unpack_from(payload)
    if magic != _MAGIC or version not in (1, _VERSION) or not 0 < header_size <= MAX_HEADER_BYTES:
        raise ValueError("unsupported display wire format or header size")
    body_start = _PREFIX.size + header_size
    cells, blocks = (CELL_COUNT, BLOCK_COUNT) if version == _VERSION else (6400, 640)
    mask_bytes = cells // 8
    body_bytes = cells * 2 + mask_bytes * 2 + blocks * 8 * 2
    if len(payload) != body_start + body_bytes:
        raise ValueError("truncated or oversized display arrays")
    try:
        metadata = _header(json.loads(bytes(payload[_PREFIX.size:body_start]).decode("utf-8")),
                           wire_version=version)
    except (ValueError, TypeError, UnicodeError, RecursionError) as exc:
        raise ValueError(f"invalid display JSON header: {exc}") from exc
    offset = body_start

    def array(dtype, count):
        nonlocal offset
        value = np.frombuffer(payload, dtype=dtype, count=count, offset=offset).copy()
        offset += np.dtype(dtype).itemsize * count
        return value

    adc = array("<u2", cells)
    valid = np.unpackbits(array("u1", mask_bytes), bitorder="little").astype(bool)
    updated = np.unpackbits(array("u1", mask_bytes), bitorder="little").astype(bool)
    mono = array("<u8", blocks)
    real = array("<u8", blocks)
    stamp_ns = metadata["host_realtime_ns"]
    result = normalize_state(SimpleNamespace(
        **metadata, adc=adc, valid_mask=valid, updated_mask=updated,
        block_monotonic_ns=mono, block_realtime_ns=real,
        header=SimpleNamespace(frame_id=metadata["frame_id"], stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
    ))
    result.frame_id = metadata["frame_id"]
    result.metadata = dict(metadata, schema_version=SCHEMA_VERSION, rows=ROWS, cols=COLS,
                           layout_id=result.layout_id)
    return result


def _socket_path(path) -> Path:
    path = Path(path)
    encoded = os.fsencode(path)
    if not path.is_absolute() or b"\x00" in encoded or len(encoded) > 107:
        raise ValueError("socket_path must be absolute and at most 107 bytes")
    return path


def _private_parent(path: Path, *, create: bool) -> bool:
    if create:
        path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    try:
        info = path.parent.lstat()
    except FileNotFoundError:
        return False
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077):
        raise ValueError("socket parent must be an owned private directory (chmod 700)")
    return True


class LocalTactileSender:
    """Nonblocking best-effort sender; absent/full/restarting GUI never blocks ROS."""

    def __init__(self, path=None, *, hand="right", namespace=None):
        self.path = _socket_path(default_socket_path(hand, namespace) if path is None else path)
        _private_parent(self.path, create=False)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.sent = self.dropped = self.rejected = 0
        self.last_error = ""

    def send(self, message) -> bool:
        try:
            payload = encode_state(message)
        except (ValueError, TypeError, OverflowError) as exc:
            self.rejected += 1
            self.last_error = str(exc)
            return False
        try:
            if self.socket is None:
                raise OSError(errno.EBADF, "display sender is closed")
            if not _private_parent(self.path, create=False):
                raise FileNotFoundError(errno.ENOENT, "display receiver directory does not exist")
            info = self.path.lstat()
            if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
                raise ValueError("display destination must be an owned private socket")
            if self.socket.sendto(payload, str(self.path)) != len(payload):
                raise OSError(errno.EIO, "incomplete display datagram")
        except (OSError, ValueError) as exc:
            self.dropped += 1
            self.last_error = str(exc)
            return False
        self.sent += 1
        self.last_error = ""
        return True

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None


class LocalTactileReceiver:
    """Own one private endpoint, preserving identity/health boundaries while draining.

    ``received`` counts accepted states, ``rejected`` malformed/duplicate/out-of-
    order states. A same-sequence health transition is accepted as an event,
    never claimed to be a new sensor sample. ``reset_generation`` increments on every observed identity
    change (including the first); ``health_generation`` increments on a healthy
    to unhealthy event. These counters also accompany the returned latest state
    so consumers cannot miss an interruption hidden inside one drained batch.
    """

    def __init__(self, path=None, *, hand="right", namespace=None):
        self.path = _socket_path(default_socket_path(hand, namespace) if path is None else path)
        self.socket = None
        self._lock_fd = None
        self._bound_identity = None
        self.received = self.rejected = self.reset_generation = self.health_generation = 0
        self.last_error = ""
        self._identity = None
        self._sequence = None
        self._healthy = None
        _private_parent(self.path, create=True)
        try:
            self._lock_fd = os.open(str(self.path) + ".lock",
                                    os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
            info = os.fstat(self._lock_fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid():
                raise ValueError("socket lock must be an owned regular file")
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._remove_stale_socket()
            self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.socket.setblocking(False)
            self.socket.bind(str(self.path))
            info = self.path.lstat()
            self._bound_identity = (info.st_dev, info.st_ino)
            os.chmod(self.path, 0o600)
        except BaseException:
            self.close()
            raise

    def _remove_stale_socket(self):
        try:
            info = self.path.lstat()
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
            raise ValueError("refusing to replace a non-socket or unowned socket path")
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            probe.connect(str(self.path))
        except OSError as exc:
            if exc.errno not in (errno.ECONNREFUSED, errno.ENOENT):
                raise
        else:
            raise ValueError("another receiver already owns this socket")
        finally:
            probe.close()
        try:
            current = self.path.lstat()
            if (current.st_dev, current.st_ino) != (info.st_dev, info.st_ino):
                raise ValueError("socket changed while checking its owner")
            self.path.unlink()
        except FileNotFoundError:
            pass

    def receive_latest(self):
        if self.socket is None:
            return None
        latest = None
        for _ in range(MAX_DRAIN_PACKETS):
            try:
                payload, _ancillary, flags, _address = self.socket.recvmsg(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            except OSError as exc:
                self.last_error = str(exc)
                break
            try:
                if flags & socket.MSG_TRUNC or len(payload) > MAX_PACKET_BYTES:
                    raise ValueError("display datagram exceeds 32768 bytes")
                message = decode_state(payload)
                identity = (message.hand, message.session_id, message.stream_epoch)
                healthy = message.connected and message.time_valid
                health_only = False
                if self._identity == identity:
                    if message.source_sequence < self._sequence:
                        raise ValueError("out-of-order tactile source_sequence")
                    if message.source_sequence == self._sequence:
                        if healthy == self._healthy:
                            raise ValueError("duplicate tactile source_sequence")
                        # Derived rolling states can report loss/recovery of
                        # time/connection without a new serial read. Preserve
                        # this boundary, but don't invent a new source sample.
                        health_only = True
                if identity != self._identity:
                    self.reset_generation += 1
                    self._healthy = None
                if self._healthy is True and not healthy:
                    self.health_generation += 1
                self._identity, self._sequence, self._healthy = identity, message.source_sequence, healthy
                self.received += 1
                self.last_error = ""
                message.transport_reset_generation = self.reset_generation
                message.transport_health_generation = self.health_generation
                message.transport_health_only = health_only
                latest = message
            except (ValueError, TypeError, OverflowError) as exc:
                self.rejected += 1
                self.last_error = str(exc)
        return latest

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        if self._bound_identity is not None:
            try:
                info = self.path.lstat()
                if (info.st_dev, info.st_ino) == self._bound_identity:
                    self.path.unlink()
            except FileNotFoundError:
                pass
            self._bound_identity = None
        if self._lock_fd is not None:
            os.close(self._lock_fd)
            self._lock_fd = None
