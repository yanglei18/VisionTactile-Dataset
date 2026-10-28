"""Optional spawn-isolated result JSONL recording; never opens an input source.

The parent only pickles each pair into a bounded FIFO. A small sender thread
transports those bytes, while one child owns both files and all JSON encoding.
Unlike multiprocessing.Queue's asynchronous pickler, admission cannot accept
an object which is then silently discarded by a feeder serialization error.
Pickle is private IPC between this process and its own trusted spawn child.
"""
from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import pickle
import queue
import threading
import time

from . import host_clock


def _open_stream(path):
    return Path(path).open("x", encoding="utf-8")


def _write_results(paths, incoming, status, written, flushed, flush_interval_s, stream_factory):
    """Spawn target: own, serialize, write, flush and close both result streams."""
    streams, error = [], None
    parent = mp.parent_process()

    def flush():
        for stream in streams:
            stream.flush()
        flushed.value = written.value

    try:
        for name in ("orientations", "poses"):
            streams.append(stream_factory(paths[name]))
        status.send(("ready", None))
        last_flush = host_clock.monotonic()
        while True:
            if not incoming.poll(.05):
                if parent is not None and not parent.is_alive():
                    raise OSError("result recording parent exited before drain completed")
                if host_clock.monotonic() - last_flush >= flush_interval_s:
                    flush()
                    last_flush = host_clock.monotonic()
                continue
            try:
                payload = incoming.recv_bytes()
            except EOFError as exc:
                raise OSError("result recording transport closed before drain completed") from exc
            if not payload:  # Only the sender emits this, after draining its FIFO.
                break
            frame, pose, force_flush = pickle.loads(payload)
            # Serialize both before writing either. Strict JSON rejection must
            # never leave an orientation row for an unserializable pose.
            lines = [json.dumps(value, ensure_ascii=False, allow_nan=False,
                                separators=(",", ":")) + "\n"
                     for value in (frame, pose)]
            for stream, line in zip(streams, lines):
                stream.write(line)
            written.value += 1
            if force_flush or host_clock.monotonic() - last_flush >= flush_interval_s:
                flush()
                last_flush = host_clock.monotonic()
        flush()
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        closed = True
        for stream in streams:
            try:
                stream.close()
            except Exception as exc:
                closed = False
                error = error or f"{type(exc).__name__}: {exc}"
        if closed and len(streams) == 2:
            flushed.value = written.value
        try:
            status.send(("finished", error))
        except (BrokenPipeError, EOFError, OSError):
            pass  # Parent death is not a successful drain acknowledgement.
        status.close()
        incoming.close()


class ProcessResultRecorder:
    """Bounded optional recorder with a process-compatible lifecycle handle.

    ``paths`` contains unopened ``orientations`` and ``poses`` paths. The
    constructor waits briefly for child-owned exclusive file creation; callers
    must assign/track ``worker`` and call ``check()`` before starting a source.
    Startup failures stay on this object so a timed-out opener is never orphaned.

    ``worker`` is this lifecycle handle, not just the child: ``join/is_alive``
    also account for the sender. ``close`` cooperatively drains and never kills
    either worker. ``written_pairs`` means both writes returned; ``flushed_pairs``
    distinguishes acknowledged flushing, not fsync durability after power loss.
    ``capacity`` bounds the parent FIFO, not all pending pairs: the sender, the
    bounded OS pipe and the writer can also have accepted pairs in flight.
    Accepted input values are serialized during submit and may then be mutated.
    ``_stream_factory`` is a spawn-picklable test seam, executed only in the child.
    """

    def __init__(self, paths, *, capacity=512, flush_interval_s=1., startup_timeout=2.,
                 _stream_factory=None):
        if type(capacity) is not int or capacity < 1:
            raise ValueError("result recording capacity must be a positive integer")
        for name, value in (("flush_interval_s", flush_interval_s), ("startup_timeout", startup_timeout)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        self.paths = {name: os.fspath(paths[name]) for name in ("orientations", "poses")}
        self.capacity, self.flush_interval_s = capacity, flush_interval_s
        self._queue = queue.Queue(maxsize=capacity)
        self._lock, self._stop = threading.RLock(), threading.Event()
        self._accepted = self._rejected = self._high_water = 0
        self._error, self._ready, self._finished = None, False, False
        self._status_closed, self._timed_out = False, False
        self._aborted = False
        context = mp.get_context("spawn")
        # Each counter has exactly one writer. No cross-process lock can be
        # stranded by a child crash; final counts are read after child exit.
        self._written = context.RawValue("Q", 0)
        self._flushed = context.RawValue("Q", 0)
        incoming, self._outgoing = context.Pipe(duplex=False)
        self._status, reporting = context.Pipe(duplex=False)
        self.process = context.Process(target=_write_results,
            args=(self.paths, incoming, reporting, self._written, self._flushed,
                  flush_interval_s, _stream_factory or _open_stream),
            name="fsglove12-result-recorder", daemon=False)
        try:
            self.process.start()
        except BaseException:
            incoming.close()
            reporting.close()
            self._outgoing.close()
            self._status.close()
            raise
        incoming.close()
        reporting.close()
        self._sender = threading.Thread(target=self._send, name="fsglove12-result-sender", daemon=True)
        self.worker = self
        self._sender_started = False
        try:
            self._sender.start()
            self._sender_started = True
        except Exception as exc:
            # A non-daemon child already exists: preserve this lifecycle
            # handle, and EOF lets that child close its own streams safely.
            self._error = f"result recording sender could not start: {type(exc).__name__}: {exc}"
            self._stop.set()
            self._outgoing.close()
        deadline = host_clock.monotonic() + startup_timeout
        while True:
            with self._lock:
                self._refresh()
                if self._ready or self._error:
                    break
            remaining = deadline - host_clock.monotonic()
            if remaining <= 0:
                with self._lock:
                    self._error = "result recording startup timed out before files were ready"
                    self._stop.set()
                break
            self._status.poll(min(.05, remaining))

    def _read_status(self):
        if not self._status_closed:
            while self._status.poll():
                try:
                    kind, error = self._status.recv()
                except (EOFError, OSError):
                    self._status.close()
                    self._status_closed = True
                    if not self._finished:
                        self._error = self._error or "result recording status channel closed before completion acknowledgement; recording incomplete"
                        self._stop.set()
                    break
                if kind == "ready":
                    self._ready = True
                elif kind == "finished":
                    self._finished = True
                    if not self._ready and not error:
                        error = "result recording ended before files were ready"
                    # A failed child closes the pipe, often causing a secondary
                    # sender error before its original disk/JSON error arrives.
                    if error and (not self._error or self._error.startswith("result recording transport failed:")):
                        self._error = error
                    self._stop.set()

    def _refresh(self):
        """Collect finite status messages while holding only the parent's lock."""
        self._read_status()
        if not self.process.is_alive():
            # Exit may race the first poll: after confirmed exit all messages
            # have been published, so recheck before declaring a missing ack.
            self._read_status()
            if not self._finished or self.process.exitcode != 0:
                self._error = self._error or f"result recording process exited unexpectedly (exitcode={self.process.exitcode}); recording incomplete"
                self._stop.set()

    def _send(self):
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    payload = self._queue.get(timeout=.05)
                except queue.Empty:
                    continue
                try:
                    self._outgoing.send_bytes(payload)
                finally:
                    self._queue.task_done()
            self._outgoing.send_bytes(b"")
        except Exception as exc:
            with self._lock:
                self._error = self._error or f"result recording transport failed: {type(exc).__name__}: {exc}"
                self._stop.set()
        finally:
            self._outgoing.close()
            # A failed pipe cannot drain. All remaining accepted records are
            # explicitly failed once the child has exited, never silently lost.
            while True:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
                self._queue.task_done()

    def check(self):
        with self._lock:
            self._refresh()
            if self._error:
                raise OSError(f"result recording failed: {self._error}")
            if not self._ready:
                raise OSError("result recording failed: files are not ready")

    def submit(self, frame, pose, *, flush=False):
        """Pickle before FIFO admission; never block on file I/O or JSON work."""
        with self._lock:
            self._refresh()
            if self._stop.is_set() or self._error:
                self._rejected += 1
                raise OSError(f"result recording is unavailable: {self._error or 'closing'}")
            try:
                payload = pickle.dumps((frame, pose, flush), protocol=pickle.HIGHEST_PROTOCOL)
                self._queue.put_nowait(payload)
            except Exception as exc:
                self._rejected += 1
                if isinstance(exc, queue.Full):
                    self._error = "result recording queue full; result pair was not recorded"
                else:
                    self._error = f"result recording serialization failed: {type(exc).__name__}: {exc}"
                self._stop.set()  # Already accepted records still drain in order.
                raise OSError(self._error) from exc
            self._accepted += 1
            self._high_water = max(self._high_water, self._queue.qsize())

    def is_alive(self):
        return self.process.is_alive() or self._sender.is_alive()

    def join(self, timeout=None):
        deadline = None if timeout is None else host_clock.monotonic() + max(0., timeout)
        for worker in (self.process, self._sender):
            if worker is self._sender and not self._sender_started:
                continue
            worker.join(None if deadline is None else max(0., deadline - host_clock.monotonic()))
        with self._lock:
            self._refresh()

    def close(self, timeout=.75):
        """Drain both workers; on timeout retain ownership and incomplete status."""
        self.request_stop()
        self.join(timeout)
        done = not self.is_alive()
        with self._lock:
            self._timed_out = not done
        return done

    def request_stop(self):
        """Stop admissions without joining, flushing, or closing caller-side I/O.

        The finite accepted FIFO and in-flight pipe still drain. The owner
        retains this lifecycle handle and joins it during eventual cleanup.
        """
        with self._lock:
            self._stop.set()

    def abort(self, reason):
        """Fail recording now, preserving errors and cooperatively draining.

        This never waits for the writer, force-kills it, or reopens files.
        Accepted pairs may still be written, but final status stays incomplete.
        """
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("recording abort requires a nonempty reason")
        with self._lock:
            self._refresh()  # Preserve an already-reported original child error.
            self._error = self._error or f"result recording aborted: {reason}"
            self._aborted = True
            self._stop.set()

    def snapshot(self):
        with self._lock:
            self._refresh()
            alive = self.is_alive()
            written = min(self._accepted, self._written.value)
            flushed = min(written, self._flushed.value)
            failed = self._accepted - written if not self.process.is_alive() else 0
            pending = self._accepted - written - failed
            if failed and not self._error:
                self._error = "result recording ended with unwritten accepted pairs; recording incomplete"
            complete = (not alive and self._finished and not self._error and not pending
                        and not failed and not self._timed_out)
            incomplete = bool(self._error or self._timed_out or (not alive and not complete))
            status = "complete" if complete else ("incomplete" if incomplete else
                     ("draining" if self._stop.is_set() else "recording"))
            return dict(mode="bounded_process", capacity_pairs=self.capacity,
                        capacity_scope="parent_fifo_pairs", queued_pairs=self._queue.qsize(),
                        accepted_pairs=self._accepted, written_pairs=written,
                        flushed_pairs=flushed, failed_pairs=failed,
                        rejected_pairs=self._rejected, pending_pairs=pending,
                        queue_high_water_pairs=self._high_water,
                        writer_alive=alive, writer_pid=self.process.pid, ready=self._ready,
                        stop_requested=self._stop.is_set(), abort_requested=self._aborted,
                        recording_complete=complete, incomplete=incomplete,
                        status=status, error=self._error)
