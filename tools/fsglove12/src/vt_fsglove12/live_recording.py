"""Bounded result recording, outside the latency-sensitive solving thread.

One worker owns both JSONL streams. Accepted frame/pose pairs are read-only
after submission; there is no background mutation, lossy queue or retry that
could reorder records. An exhausted queue is an explicit recording failure.
"""
from __future__ import annotations

import json
import queue
import threading
import time

from . import host_clock


class AsyncResultRecorder:
    def __init__(self, streams, *, capacity=512, flush_interval_s=1.):
        if type(capacity) is not int or capacity < 1:
            raise ValueError("result recording capacity must be a positive integer")
        self.streams = {name: streams[name] for name in ("orientations", "poses")}
        self.capacity, self.flush_interval_s = capacity, flush_interval_s
        self._queue = queue.Queue(maxsize=capacity)
        self._stop, self._lock = threading.Event(), threading.Lock()
        self._accepted = self._written = self._failed = self._rejected = self._high_water = 0
        self._error = None
        self._aborted = self._timed_out = False
        self.thread = threading.Thread(target=self._run, name="fsglove12-result-recorder", daemon=True)
        self.thread.start()

    @property
    def worker(self):
        """Lifecycle handle shared with the optional process-isolated writer."""
        return self.thread

    def _fail(self, exc):
        with self._lock:
            if self._error is None:
                self._error = f"{type(exc).__name__}: {exc}"

    def check(self):
        with self._lock:
            error = self._error
        if error:
            raise OSError(f"result recording failed: {error}")

    def submit(self, frame, pose, *, flush=False):
        """Transfer read-only values; fail immediately instead of dropping silently."""
        with self._lock:
            if self._stop.is_set() or self._error:
                self._rejected += 1
                raise OSError(f"result recording is unavailable: {self._error or 'closing'}")
            try:
                self._queue.put_nowait((frame, pose, flush))
            except queue.Full as exc:
                self._rejected += 1
                self._error = "result recording queue full; result pair was not recorded"
                raise OSError(self._error) from exc
            self._accepted += 1
            self._high_water = max(self._high_water, self._queue.qsize())

    def snapshot(self):
        with self._lock:
            alive = self.thread.is_alive()
            pending = self._accepted - self._written - self._failed
            complete = (not alive and not self._error and not pending
                        and not self._failed and not self._timed_out)
            incomplete = bool(self._error or self._timed_out or (not alive and not complete))
            status = "complete" if complete else ("incomplete" if incomplete else
                     "draining" if self._stop.is_set() else "recording")
            return dict(mode="bounded_async", capacity_pairs=self.capacity,
                        accepted_pairs=self._accepted, written_pairs=self._written,
                        failed_pairs=self._failed, rejected_pairs=self._rejected,
                        pending_pairs=pending,
                        queue_high_water_pairs=self._high_water,
                        writer_alive=alive, error=self._error,
                        stop_requested=self._stop.is_set(), abort_requested=self._aborted,
                        recording_complete=complete, incomplete=incomplete, status=status)

    def request_stop(self):
        """Stop admissions now; drain accepted pairs without waiting on disk I/O.

        The owner must retain ``worker`` and eventually join/close it. This
        operation deliberately does not flush, close, or join on the caller.
        """
        with self._lock:
            self._stop.set()

    def abort(self, reason):
        """Nonblocking failed stop; preserve the first error and finite drain.

        Already accepted pairs may finish in order. Their accounting remains
        truthful, but successful writes cannot turn an aborted recording into
        a complete one. No worker is killed and no original error is cleared.
        """
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("recording abort requires a nonempty reason")
        with self._lock:
            self._error = self._error or f"result recording aborted: {reason}"
            self._aborted = True
            self._stop.set()

    def close(self, timeout=.75):
        """Drain accepted pairs and close owned streams; never kill a blocked writer."""
        self.request_stop()
        self.thread.join(timeout)
        done = not self.thread.is_alive()
        with self._lock:
            self._timed_out = not done
        return done

    def _flush(self):
        for stream in self.streams.values():
            stream.flush()

    def _run(self):
        last_flush = host_clock.monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    frame, pose, force_flush = self._queue.get(timeout=.05)
                except queue.Empty:
                    if host_clock.monotonic() - last_flush >= self.flush_interval_s:
                        self._flush()
                        last_flush = host_clock.monotonic()
                    continue
                try:
                    # Serialize both before writing either: malformed JSON cannot
                    # leave an orientation without its corresponding pose.
                    lines = [json.dumps(value, ensure_ascii=False, allow_nan=False,
                                        separators=(",", ":")) + "\n"
                             for value in (frame, pose)]
                    for stream, line in zip(self.streams.values(), lines):
                        stream.write(line)
                    with self._lock:
                        self._written += 1
                except Exception:
                    with self._lock:
                        self._failed += 1
                    raise
                finally:
                    self._queue.task_done()
                if force_flush or host_clock.monotonic() - last_flush >= self.flush_interval_s:
                    self._flush()
                    last_flush = host_clock.monotonic()
        except Exception as exc:
            self._fail(exc)
            # Every accepted-but-unwritten pair is accounted for, even after
            # serialization/disk failure prevents draining it successfully.
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
                with self._lock:
                    self._failed += 1
                self._queue.task_done()
        finally:
            for stream in self.streams.values():
                try:
                    stream.close()  # Text streams flush their buffers on close.
                except Exception as exc:
                    self._fail(exc)
