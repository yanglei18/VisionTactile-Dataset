"""Opt-in, bounded JSONL evidence recording without disk I/O on the producer.

``complete`` means application-ordered writes, flush and close succeeded. It is
not an fsync/power-loss guarantee, proof of loss-free sensors, or a sensor latency
measurement. A failed footer is removed best-effort; a completely failing file
system can also prevent that cleanup, so runtime errors must not be ignored.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import queue
import threading
import time

from .records import Sample


TIME_BASIS = "host_observed_not_sensor_latency"
MARKERS = frozenset({"rest", "moving", "returned"})
MAX_EVENT_BYTES = 2_000_000


def _bounded_copy(value, limit):
    """Copy JSON data with conservative encoded-size, shape and depth limits."""
    remaining = limit - 128  # Reserve the seq/type envelope and newline.

    def visit(item, depth=0):
        nonlocal remaining
        if depth > 24:
            raise ValueError("diagnostic event nesting is too deep")
        remaining -= 4
        if remaining < 0:
            raise ValueError("diagnostic event exceeds its byte limit")
        if item is None or type(item) is bool:
            remaining -= 5
            result = item
        elif type(item) is int:
            if item.bit_length() > 64:
                raise ValueError("diagnostic integer is too large")
            remaining -= 24
            result = item
        elif type(item) is float:
            if not math.isfinite(item):
                raise ValueError("diagnostic JSON requires finite numbers")
            remaining -= 32
            result = item
        elif type(item) is str:
            remaining -= 6 * len(item) + 2
            result = item
        elif type(item) in (list, tuple):
            if len(item) > 4096:
                raise ValueError("diagnostic array has too many entries")
            result = [visit(child, depth + 1) for child in item]
        elif type(item) is dict:
            if len(item) > 512 or any(type(key) is not str or len(key) > 256 for key in item):
                raise ValueError("diagnostic object has invalid keys or too many entries")
            result = {visit(key, depth + 1): visit(child, depth + 1) for key, child in item.items()}
        else:
            raise ValueError(f"unsupported diagnostic JSON value: {type(item).__name__}")
        if remaining < 0:
            raise ValueError("diagnostic event exceeds its byte limit")
        return result

    result = visit(value)
    return result, limit - remaining


class DiagnosticRecorder:
    """One daemon writer, one bounded queue, exclusive creation and no retries.

    Only ``start``/``offer``/``stop`` are called by the acquisition producer.
    They never perform file I/O or wait for the writer. Accepted payloads are
    independent copies. Queue exhaustion fails the session instead of silently
    dropping events. A blocked writer cannot be replaced until it has exited.
    """

    def __init__(self, path, *, max_queue_items=64, max_queue_bytes=8 * 1024 * 1024,
                 max_event_bytes=MAX_EVENT_BYTES):
        self.path = Path(path)
        for name, value in (("max_queue_items", max_queue_items),
                            ("max_queue_bytes", max_queue_bytes),
                            ("max_event_bytes", max_event_bytes)):
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if max_event_bytes > MAX_EVENT_BYTES:
            raise ValueError("diagnostic line limit cannot exceed 2,000,000 bytes")
        self._max_bytes, self._max_event = max_queue_bytes, max_event_bytes
        self._queue = queue.Queue(maxsize=max_queue_items)
        self._lock = threading.Lock()
        self._thread = None
        self._state, self._error, self._reason = "idle", "", "manual"
        self._stop = threading.Event()
        self._failed = threading.Event()
        self._next_seq = self._written = self._pending_bytes = self._peak_bytes = 0

    @property
    def is_alive(self):
        return self._thread is not None and self._thread.is_alive()

    def status(self):
        with self._lock:
            return dict(state=self._state, path=str(self.path), error=self._error,
                        accepted_events=self._next_seq, written_events=self._written,
                        pending_bytes=self._pending_bytes, peak_pending_bytes=self._peak_bytes,
                        queued_events=self._queue.qsize(), writer_alive=self.is_alive)

    def wait_closed(self, timeout=.5):
        """Bounded shutdown grace only; never used by ordinary stop or reads."""
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 <= timeout <= .5:
            raise ValueError("shutdown grace must be between zero and 0.5 seconds")
        if self._thread is not None:
            self._thread.join(timeout)
        return not self.is_alive

    def fail(self, message):
        message = str(message)[:2000]
        with self._lock:
            if self._state == "complete":
                return
            self._state = "failed"
            self._error = (self._error + "; " + message if self._error else message)[:4000]
        self._failed.set()
        self._stop.set()

    def start(self, checkpoint):
        with self._lock:
            if self._state != "idle":
                raise RuntimeError("DiagnosticRecorder cannot be restarted")
            self._state = "recording"
        if type(checkpoint) is not dict:
            self.fail("diagnostic header checkpoint must be an object")
            return False
        header = dict(kind="vt_glove_pose.diagnostic_session", schema_version=1,
                      checkpoint=checkpoint, time_basis=TIME_BASIS)
        if not self._enqueue("header", header):
            return False
        self._thread = threading.Thread(target=self._run, name="glove-diagnostic-writer", daemon=True)
        try:
            self._thread.start()
        except Exception as exc:
            self.fail(f"cannot start diagnostic writer: {exc}")
            return False
        return True

    def offer(self, event_type, **payload):
        with self._lock:
            if self._state != "recording":
                return False
        try:
            if event_type == "batch":
                samples = payload.get("samples")
                if type(samples) is not list or len(samples) > 4096:
                    raise ValueError("diagnostic batch must contain at most 4096 samples")
                for sample in samples:
                    if (type(sample) is not dict or type(sample.get("values")) is not list
                            or len(sample["values"]) != 6
                            or set(sample) != {"node_id", "values", "host_monotonic_s", "host_wall_s", "status"}):
                        raise ValueError("invalid diagnostic sample shape")
                    Sample(sample["node_id"], tuple(sample["values"]),
                           sample["host_monotonic_s"], sample["host_wall_s"], sample["status"])
                if type(payload.get("performance")) is not dict:
                    raise ValueError("diagnostic batch requires performance observations")
            elif event_type == "snapshot":
                if type(payload.get("now_s")) not in (float, int) or type(payload.get("snapshot")) is not dict:
                    raise ValueError("invalid diagnostic snapshot")
                if "runtime" in payload["snapshot"] or type(payload.get("performance")) is not dict:
                    raise ValueError("diagnostic snapshot must exclude UI runtime")
            elif event_type == "command":
                name, args = payload.get("name"), payload.get("args")
                valid = type(args) is dict and (
                    (name == "cancel" and not args)
                    or (name == "neutral" and set(args) == {"confirm_mount", "duration"}
                        and type(args["confirm_mount"]) is bool and args["duration"] == 5)
                    or (name == "axis" and set(args) == {"target", "duration"}
                        and args["target"] in {"palm", "thumb", "index", "middle", "ring", "little", "thumb_common", "thumb_hinge"}
                        and args["duration"] == 8)
                    or (name == "axis" and set(args) == {"target", "duration", "calibration_method"}
                        and args["target"] in {"thumb_common", "thumb_hinge"}
                        and args["duration"] == 8 and args["calibration_method"] == "robust_v2"))
                if not valid:
                    raise ValueError("invalid diagnostic calibration command")
            elif event_type == "marker":
                if payload.get("label") not in MARKERS:
                    raise ValueError("marker must be rest, moving, or returned")
            elif event_type == "checkpoint":
                if type(payload.get("checkpoint")) is not dict:
                    raise ValueError("diagnostic checkpoint must be an object")
            else:
                raise ValueError("unsupported diagnostic event type")
            return self._enqueue(event_type, payload)
        except Exception as exc:
            self.fail(f"diagnostic event rejected: {exc}")
            return False

    def _enqueue(self, event_type, payload):
        try:
            if "seq" in payload or "type" in payload:
                raise ValueError("diagnostic envelope fields are reserved")
            copied, size = _bounded_copy(payload, self._max_event)
            with self._lock:
                if self._state != "recording":
                    return False
                if self._pending_bytes + size > self._max_bytes:
                    raise ValueError("diagnostic byte queue is full; acquisition continues")
                event = dict(seq=self._next_seq, type=event_type, **copied)
                self._queue.put_nowait((event, size))
                self._next_seq += 1
                self._pending_bytes += size
                self._peak_bytes = max(self._peak_bytes, self._pending_bytes)
            return True
        except Exception as exc:
            self.fail(f"diagnostic enqueue failed: {type(exc).__name__}: {exc}")
            return False

    def stop(self, reason="manual"):
        with self._lock:
            if self._state != "recording":
                return
            self._reason = str(reason)[:256]
            self._state = "stopping"
        self._stop.set()

    def _open_stream(self):
        return self.path.open("xb")

    def _write(self, stream, event):
        # The writer checks the exact UTF-8 size too, including every envelope.
        data = (json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n").encode("utf-8")
        if len(data) > self._max_event:
            raise ValueError("encoded diagnostic event exceeds byte limit")
        if stream.write(data) != len(data):
            raise OSError("short diagnostic write")

    def _rollback_footer(self, offset, identity):
        # Only truncate a file this writer created, never an existing/replaced file.
        descriptor = os.open(self.path, os.O_WRONLY | getattr(os, "O_NOFOLLOW", 0))
        try:
            stat = os.fstat(descriptor)
            if (stat.st_dev, stat.st_ino) != identity:
                raise OSError("diagnostic file identity changed; footer rollback refused")
            os.ftruncate(descriptor, offset)
        finally:
            os.close(descriptor)

    def _run(self):
        stream, footer_offset, identity = None, None, None
        try:
            stream = self._open_stream()
            stat = os.fstat(stream.fileno())
            identity = (stat.st_dev, stat.st_ino)
            last_flush = time.monotonic()
            while not self._failed.is_set():
                try:
                    event, size = self._queue.get(timeout=.02)
                except queue.Empty:
                    if self._stop.is_set():
                        break
                    continue
                try:
                    self._write(stream, event)
                    with self._lock:
                        self._written += 1
                finally:
                    with self._lock:
                        self._pending_bytes -= size
                if time.monotonic() - last_flush >= .5:
                    stream.flush()
                    last_flush = time.monotonic()
            if not self._failed.is_set():
                stream.flush()
                footer_offset = stream.tell()
                footer = dict(seq=self._next_seq, type="end", reason=self._reason,
                              complete=True, events_before_end=self._written)
                self._write(stream, footer)
                stream.flush()
        except Exception as exc:
            self.fail(f"diagnostic write failed: {type(exc).__name__}: {exc}")
        finally:
            if stream is not None:
                try:
                    stream.close()
                except Exception as exc:
                    self.fail(f"diagnostic close failed: {type(exc).__name__}: {exc}")
            if self._failed.is_set() and footer_offset is not None:
                try:
                    self._rollback_footer(footer_offset, identity)
                except Exception as exc:
                    self.fail(f"cannot remove failed completion footer: {exc}")
            while True:
                try:
                    _, size = self._queue.get_nowait()
                except queue.Empty:
                    break
                with self._lock:
                    self._pending_bytes -= size
            with self._lock:
                if self._state != "failed":
                    self._state = "complete"
                    self._written += 1  # The footer, after successful close.
