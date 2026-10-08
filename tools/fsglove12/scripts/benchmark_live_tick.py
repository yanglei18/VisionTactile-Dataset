#!/usr/bin/env python3
"""Offline capacity of the real LiveController._tick, never GUI/serial/ROS DDS.

Uses a bounded window of existing recorded orientations and its unchanged
profile/config/rest. Every input frame is visited once, in recorded order.
The first --warmup records initialize caches; subsequent distinct inputs execute the
real solver, MANO, display metadata, asynchronous JSONL and nonblocking local
datagram export. A separate process drains and decodes the local datagrams; it is NOT
a ROS/DDS subscriber. Optional 60 Hz pose-only snapshot reads model copy contention.
Publication mode also encodes the production envelope pickle, but does not transfer
it through the mailbox or perform GUI rendering.

The source clock is frozen at each original frame timestamp while _tick runs.
This avoids inventing 120 Hz source timestamps and does NOT test freshness
under real wall-time deadlines. Only perf_counter measures processing time.
No controller.start(), connect(), source.open() or GUI code is invoked.
"""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
from copy import deepcopy
import gc
import hashlib
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import pickle
import platform
import queue
import select
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

from benchmark_upstream_runtime import (THREAD_VARIABLES, implementation_provenance,
                                        latency_stats, load_recording, source_summary)


class LiveGcTrace:
    """Bounded, passive GC and thread-interval correlation on one monotonic clock.

    A wall-time overlap is not a measurement of GIL ownership or a causal
    attribution. In particular, writer_pair covers JSON encoding *and* writes.
    No profiler, global JSON monkeypatch, forced collection or GC tuning is used.
    """

    def __init__(self, *, clock=time.perf_counter_ns, max_events=4096, max_intervals=20000):
        self.clock, self.max_events, self.max_intervals = clock, max_events, max_intervals
        self.events, self.intervals, self.active, self.starts = [], [], {}, {}
        self.generations = {generation: [0, 0, 0] for generation in range(3)}
        self.dropped_events = self.dropped_intervals = 0
        self.running, self.report = False, None

    def start(self):
        if self.running or self.report is not None:
            raise RuntimeError("GC trace cannot be started twice")
        self.enabled_before, self.thresholds = gc.isenabled(), gc.get_threshold()
        self.counts_before, self.origin = gc.get_count(), self.clock()
        self.running = True
        gc.callbacks.append(self._callback)

    def begin(self, kind, frame_index=None, *, measured=False):
        if not self.running:
            return None
        start, thread_id = self.clock(), threading.get_ident()
        stack = self.active.setdefault(thread_id, [])
        if frame_index is None and stack:
            frame_index = stack[-1][1]
        token = (kind, frame_index, measured, start, thread_id, threading.current_thread().name)
        stack.append(token)
        return token

    def end(self, token, *, emitted=None):
        if token is None:
            return
        end = self.clock()
        stack = self.active[token[4]]
        if stack[-1] is not token:
            raise RuntimeError("Unbalanced diagnostic interval")
        stack.pop()
        if len(self.intervals) < self.max_intervals:
            self.intervals.append((*token, end, emitted))
        else:
            self.dropped_intervals += 1

    def _callback(self, phase, info):
        now, thread_id = self.clock(), threading.get_ident()
        generation = info["generation"]
        key = (thread_id, generation)
        if phase == "start":
            stack = self.active.get(thread_id, ())
            stage = stack[-1][:2] if stack else ("outside_traced_interval", None)
            self.starts[key] = (now, threading.current_thread().name, *stage)
        elif phase == "stop" and key in self.starts:
            start, thread_name, kind, frame = self.starts.pop(key)
            duration = now - start
            summary = self.generations[generation]
            summary[0] += 1
            summary[1] += duration
            summary[2] = max(summary[2], duration)
            if len(self.events) < self.max_events:
                self.events.append((generation, start, now, thread_id, thread_name, kind, frame,
                                    info.get("collected"), info.get("uncollectable")))
            else:
                self.dropped_events += 1

    def _interval(self, item):
        kind, frame, measured, start, thread_id, thread_name, end, emitted = item
        return dict(kind=kind, frame_index=frame, measured=measured, emitted=emitted,
                    thread_id=thread_id, thread_name=thread_name,
                    start_elapsed_ns=start - self.origin, duration_ns=end - start,
                    duration_ms=(end - start) / 1e6)

    @staticmethod
    def _overlap(start, end, other_start, other_end):
        return max(0, min(end, other_end) - max(start, other_start))

    def finish(self, *, writer_process=False):
        if self.report is not None:
            return self.report
        gc.callbacks.remove(self._callback)
        self.running = False
        events = []
        measured_ticks = [item for item in self.intervals if item[0] == "tick" and item[2]]
        for generation, start, end, thread_id, name, kind, frame, collected, uncollectable in self.events:
            overlaps = []
            for tick in measured_ticks:
                overlap = self._overlap(start, end, tick[3], tick[6])
                if overlap:
                    overlaps.append(dict(**self._interval(tick), overlap_ns=overlap))
            events.append(dict(generation=generation, start_elapsed_ns=start - self.origin,
                duration_ns=end - start, duration_ms=(end - start) / 1e6,
                collector_thread_id=thread_id, collector_thread_name=name,
                collector_interval=kind, collector_frame_index=frame,
                collected=collected, uncollectable=uncollectable,
                overlapping_measured_ticks=overlaps[:64], overlapping_measured_tick_count=len(overlaps)))

        def slowest(kind, count):
            candidates = [item for item in self.intervals if item[0] == kind
                          and (kind != "tick" or item[2])]
            output = []
            for item in sorted(candidates, key=lambda row: row[6] - row[3], reverse=True)[:count]:
                row = self._interval(item)
                overlaps = []
                counts = Counter()
                for other in self.intervals:
                    overlap = self._overlap(item[3], item[6], other[3], other[6])
                    if overlap and other is not item:
                        counts[other[0]] += 1
                        if counts[other[0]] <= 64:
                            overlaps.append(dict(**self._interval(other), overlap_ns=overlap))
                row["overlapping_intervals"] = overlaps
                row["overlapping_interval_counts"] = dict(counts)
                row["gc_events"] = [dict(event, overlap_ns=self._overlap(item[3], item[6],
                    self.origin + event["start_elapsed_ns"],
                    self.origin + event["start_elapsed_ns"] + event["duration_ns"]))
                    for event in events if self._overlap(item[3], item[6],
                        self.origin + event["start_elapsed_ns"],
                        self.origin + event["start_elapsed_ns"] + event["duration_ns"])]
                output.append(row)
            return output

        self.report = dict(enabled=True, clock="time.perf_counter_ns", origin_perf_counter_ns=self.origin,
            automatic_gc_enabled_before=self.enabled_before, automatic_gc_enabled_after=gc.isenabled(),
            thresholds_before=self.thresholds, thresholds_after=gc.get_threshold(),
            gc_policy_changed=(self.enabled_before != gc.isenabled() or self.thresholds != gc.get_threshold()),
            manual_collect_called=False, counts_before=self.counts_before, counts_after=gc.get_count(),
            by_generation={str(generation): dict(count=value[0], total_ms=value[1] / 1e6,
                                                max_ms=value[2] / 1e6)
                           for generation, value in self.generations.items()},
            events=events, interval_counts=dict(Counter(item[0] for item in self.intervals)),
            slowest_ticks=slowest("tick", 20), slowest_snapshots=slowest("snapshot", 5),
            slowest_writer_pairs=slowest("writer_pair", 5),
            dropped_events=self.dropped_events, dropped_intervals=self.dropped_intervals,
            event_limit=self.max_events, interval_limit=self.max_intervals,
            writer_process_gc_observed=False if writer_process else None,
            instrumentation="gc.callbacks; bounded benchmark-local wall-time wrappers; production GC policy untouched",
            limitations=["Tracing adds overhead and may shift automatic collection timing; use untraced runs for capacity.",
                "Overlap is timing correlation, not proof of GIL ownership, CPU consumption or causation.",
                "writer_pair spans dequeued pair processing (JSON encoding plus writes), not isolated JSON time.",
                "Process-mode child GC and child writer intervals are not observed; parent recorder_submit is observed.",
                "Event and interval storage are bounded; dropped counts and overlap counts expose truncation."])
        return self.report


class _TracedStream:
    def __init__(self, stream, trace):
        self.stream, self.trace = stream, trace

    def __getattr__(self, name):
        return getattr(self.stream, name)

    def _call(self, method, *args):
        token = self.trace.begin(f"writer_{method}")
        try:
            return getattr(self.stream, method)(*args)
        finally:
            self.trace.end(token)

    def write(self, value):
        return self._call("write", value)

    def flush(self):
        return self._call("flush")

    def close(self):
        return self._call("close")


class _TracedQueue:
    """Keep the real queue/worker; bracket dequeue-to-task_done pair processing."""

    def __init__(self, original, trace):
        self.original, self.trace, self.token = original, trace, None

    def __getattr__(self, name):
        return getattr(self.original, name)

    def get(self, *args, **kwargs):
        item = self.original.get(*args, **kwargs)
        self.token = self.trace.begin("writer_pair", item[0].get("frame_index"))
        return item

    def task_done(self):
        try:
            return self.original.task_done()
        finally:
            self.trace.end(self.token)
            self.token = None


def make_result_recorder(paths, *, mode="thread", trace=None):
    """Child-owned process streams; do not even open their files in the parent."""
    if mode == "process":
        from vt_fsglove12.live_recording_process import ProcessResultRecorder
        recorder = ProcessResultRecorder(paths)
        try:
            recorder.check()
        except Exception:
            recorder.close(5.)
            raise
        return (_TracedRecorderSubmit(recorder, trace) if trace else recorder), {}
    if mode != "thread":
        raise ValueError(f"Unknown result recorder: {mode}")
    from vt_fsglove12.live_recording import AsyncResultRecorder
    streams = {key: path.open("x", encoding="utf-8") for key, path in paths.items()}
    if trace is None:
        return AsyncResultRecorder(streams), streams

    class TracedRecorder(AsyncResultRecorder):
        def _run(self):
            self._queue = _TracedQueue(self._queue, trace)
            super()._run()

    wrapped = {key: _TracedStream(stream, trace) for key, stream in streams.items()}
    return _TracedRecorderSubmit(TracedRecorder(wrapped), trace), wrapped


class _TracedRecorderSubmit:
    def __init__(self, recorder, trace):
        self.recorder, self.trace = recorder, trace

    def __getattr__(self, name):
        return getattr(self.recorder, name)

    def submit(self, frame, pose, *, flush=False):
        token = self.trace.begin("recorder_submit", frame.get("frame_index"))
        try:
            return self.recorder.submit(frame, pose, flush=flush)
        finally:
            self.trace.end(token)


class FrameClock:
    def __init__(self, stamp):
        self.stamp = stamp

    def __call__(self):
        return self.stamp


class RecordedFusion:
    """Read-only injection of already fused data; never runs/reinitializes VQF."""
    def __init__(self, frame):
        self.frame = frame

    def snapshot(self, _now):
        return self.frame  # _tick owns its normal deep copy.

    input_snapshot = snapshot


def _drain_process(path, expected_sources, stop, ready, changed, failed, received, results):
    """Bridge-like separate interpreter: JSON decoding cannot hold the solver GIL."""
    endpoint = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    frames, errors, lengths, byte_count = [], [], Counter(), 0
    try:
        endpoint.bind(path)
        endpoint.setblocking(False)
        ready.set()
        while not stop.is_set():
            if not select.select([endpoint], [], [], .02)[0]:
                continue
            while True:
                try:
                    payload = endpoint.recv(60_001)
                except BlockingIOError:
                    break
                document = json.loads(payload)
                index = document["frame_index"]
                if (document.get("source_mode") != "replay"
                        or document.get("host_monotonic_ns") != expected_sources.get(index)):
                    raise ValueError("Datagram source identity changed or replay was labelled live")
                frames.append(index)
                byte_count += len(payload)
                lengths[len(document.get("joint_positions_m", []))] += 1
                received.value = len(frames)
                changed.set()
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
        failed.set()
    finally:
        endpoint.close()
        ready.set()
        changed.set()
        results.put(dict(received=len(frames), bytes=byte_count, errors=errors,
            frame_indices=frames, joint_array_lengths=dict(lengths), receiver_pid=os.getpid(),
            transport="AF_UNIX/SOCK_DGRAM; separate-process JSON decode; NOT ROS DDS",
            separate_receiver_process=True, original_palm_source_times_checked=True))


class DatagramDrain:
    def __init__(self, path, expected_sources):
        self.path, self._result = path, None
        context = mp.get_context("spawn")
        self.stop, self.ready, self.changed, self.failed = (context.Event() for _ in range(4))
        self.received, self.results = context.Value("Q", 0), context.Queue(maxsize=1)
        self.process = context.Process(target=_drain_process,
            args=(str(path), expected_sources, self.stop, self.ready, self.changed,
                  self.failed, self.received, self.results), name="offline-pose-datagram-drain", daemon=True)
        self.process.start()
        if not self.ready.wait(10.) or self.failed.is_set():
            self.finish(0)
            raise RuntimeError("Offline datagram receiver could not open its private endpoint")

    def finish(self, expected_count, *, timeout=2.):
        if self._result is not None:
            return self._result
        deadline = time.monotonic() + timeout
        while self.received.value < expected_count and not self.failed.is_set() and time.monotonic() < deadline:
            self.changed.wait(min(.05, max(0., deadline - time.monotonic())))
            self.changed.clear()
        self.stop.set()
        try:
            self._result = self.results.get(timeout=2.)
        except queue.Empty:
            self._result = dict(received=self.received.value, errors=["receiver did not return its counters"])
        self.process.join(2.)
        self._result["receiver_process_stopped"] = not self.process.is_alive()
        if self.process.is_alive():
            # This child owns only our temporary offline socket, never hardware.
            self.process.terminate()
            self.process.join(1.)
            self._result["errors"].append("offline receiver failed to stop and was terminated")
        self.results.close()
        self.results.join_thread()
        return self._result


class SnapshotReader:
    def __init__(self, controller, hz, *, trace=None, mode="copy"):
        self.controller, self.hz = controller, hz
        self.trace, self.mode = trace, mode
        if mode == "publication":
            self.read = getattr(controller, "_latest_pose_publication", None)
            if not callable(self.read):
                raise ValueError("publication snapshot mode requires _latest_pose_publication")
        elif mode == "copy":
            self.read = getattr(controller, "latest_pose_snapshot", None) or controller.latest_snapshot
        else:
            raise ValueError(f"Unknown snapshot mode: {mode}")
        self.read_method = getattr(self.read, "__name__", "unknown")
        self.stop, self.measure = threading.Event(), threading.Event()
        self.latencies, self.errors = [], []
        self.read_latencies, self.pickle_latencies, self.pickle_bytes = [], [], 0
        self.thread = None
        if hz:
            self.thread = threading.Thread(target=self._run, name="offline-snapshot-reader", daemon=True)
            self.thread.start()

    def _run(self):
        deadline = time.perf_counter()
        try:
            while not self.stop.is_set():
                start = time.perf_counter()
                token = self.trace.begin("snapshot", measured=self.measure.is_set()) if self.trace else None
                try:
                    read_start = time.perf_counter()
                    read_token = self.trace.begin("snapshot_read") if self.trace else None
                    try:
                        snapshot = self.read()
                    finally:
                        if self.trace:
                            self.trace.end(read_token)
                    read_ms = (time.perf_counter() - read_start) * 1000.
                    pickle_ms, payload_size = 0., 0
                    if self.mode == "publication":
                        pickle_start = time.perf_counter()
                        pickle_token = self.trace.begin("snapshot_pickle") if self.trace else None
                        try:
                            # Match live_process._publish's envelope and protocol;
                            # no shared-memory copy/GUI render is claimed here.
                            payload_size = len(pickle.dumps((time.monotonic(), 0, snapshot),
                                                          protocol=pickle.HIGHEST_PROTOCOL))
                        finally:
                            if self.trace:
                                self.trace.end(pickle_token)
                        pickle_ms = (time.perf_counter() - pickle_start) * 1000.
                finally:
                    if self.trace:
                        self.trace.end(token)
                if self.measure.is_set():
                    self.latencies.append((time.perf_counter() - start) * 1000.)
                    self.read_latencies.append(read_ms)
                    if self.mode == "publication":
                        self.pickle_latencies.append(pickle_ms)
                        self.pickle_bytes += payload_size
                deadline += 1. / self.hz
                self.stop.wait(max(0., deadline - time.perf_counter()))
                if deadline < time.perf_counter() - 1. / self.hz:
                    deadline = time.perf_counter()  # Never replay a historical read backlog.
        except Exception as exc:
            self.errors.append(f"{type(exc).__name__}: {exc}")

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(1.)
        return dict(enabled=bool(self.hz), requested_hz=self.hz, reads=len(self.latencies),
                    errors=list(self.errors), reader_stopped=self.thread is None or not self.thread.is_alive(),
                    latency_ms=latency_stats(self.latencies, deadline_ms=1000. / self.hz if self.hz else 0.),
                    method=self.read_method, mode=self.mode,
                    read_latency_ms=latency_stats(self.read_latencies,
                        deadline_ms=1000. / self.hz if self.hz else 0.),
                    pickle_latency_ms=latency_stats(self.pickle_latencies,
                        deadline_ms=1000. / self.hz if self.hz else 0.),
                    public_copy_calls=len(self.latencies) if self.mode == "copy" else 0,
                    borrowed_publication_calls=len(self.latencies) if self.mode == "publication" else 0,
                    pickle_encoded_snapshots=len(self.pickle_latencies), pickle_bytes=self.pickle_bytes,
                    scope=("same-process trusted publication plus real highest-protocol envelope pickle; "
                           "excludes mailbox/shared-memory copy, GUI, OpenGL" if self.mode == "publication" else
                           "same-process production pose snapshot deepcopy, not GUI/IPC/OpenGL"))


def semantic_digest(records):
    """Exact normalized JSON, not numeric rounding; absent fields remain absent."""
    excluded = {"solve_completed_monotonic_ns", "published_monotonic_s",
                "processing_latency_ms", "runtime_timing_ms"}
    hasher, present = hashlib.sha256(), Counter()
    for record in records:
        retained = {key: value for key, value in record.items() if key not in excluded}
        present.update(retained.keys())
        hasher.update(json.dumps(retained, sort_keys=True, ensure_ascii=False, allow_nan=False,
                                 separators=(",", ":")).encode("utf-8"))
        hasher.update(b"\n")
    return dict(sha256=hasher.hexdigest(), records=len(records),
                included_top_level_fields=sorted(present), field_presence_counts=dict(sorted(present.items())),
                excluded_top_level_fields=sorted(excluded),
                scope="All retained fields recursively, including pose numbers, quality/masks, source IDs and "
                      "source timestamps; only listed top-level performance fields excluded; no rounding; "
                      "missing and null remain distinct")


def verify_logs(paths, expected_indices):
    result = {}
    for name, path in paths.items():
        with path.open(encoding="utf-8") as stream:
            records = [json.loads(line) for line in stream]
        indices = [item["frame_index"] for item in records]
        if indices != expected_indices:
            raise RuntimeError(f"{name} JSONL lost, duplicated or reordered an accepted frame")
        result[name] = dict(records=len(records), bytes=path.stat().st_size,
                            ordered_frame_indices_match=True, semantic_digest=semantic_digest(records))
        if name == "poses":
            result[name]["source_fresh_results"] = sum(item.get("source_fresh") is True for item in records)
            result[name]["hardware_accessed"] = any(item.get("hardware_accessed") for item in records)
    return result


def run_tick_benchmark(recording, backend, geometry, *, warmup, target_hz, snapshot_hz,
                       trace_gc=False, result_recorder="thread", snapshot_mode="copy"):
    from vt_fsglove12.live_runtime import LiveController, _Session
    from vt_fsglove12.pose_export import PoseDatagramExporter

    frames, profile, config = recording["frames"], recording["profile"], recording["config"]
    root = next(node for node, slot in config["node_to_slot"].items() if slot == 0)
    expected_indices = []  # Only outputs actually emitted by production deduplication.
    expected_sources = {frame["frame_index"]: frame["nodes"][root]["host_monotonic_ns"] for frame in frames}
    clock, fusion = FrameClock(frames[0]["timestamp_s"]), RecordedFusion(frames[0])
    durations, duplicate_durations = [], []
    component_times = {key: [] for key in ("solve", "mesh", "calibration", "total")}
    pending, available, measured_start = [], 0, None
    trace = LiveGcTrace() if trace_gc else None
    with tempfile.TemporaryDirectory(prefix="fsglove-tick-") as temporary:
        directory = Path(temporary)
        paths = {key: directory / f"{key}.jsonl" for key in ("orientations", "poses")}
        receiver = DatagramDrain(directory / "result.sock", expected_sources)
        exporter = PoseDatagramExporter(receiver.path)
        writer, streams = make_result_recorder(paths, mode=result_recorder, trace=trace)
        controller = LiveController(config, recording["rest"], {}, output_root=directory,
            clock=clock, geometry_backend=geometry, pose_backend=backend, pose_exporter=exporter)
        source = SimpleNamespace(input_kind="recorded_orientation_tick_capacity",
                                 provenance={"source_session": str(recording["session"])})
        session = _Session(profile["session_id"], directory, "OFFLINE_NO_DEVICE", source, fusion,
                           None, clock(), streams=streams, result_recorder=writer)
        session.opened = session.metadata_written = True
        session.frames = frames[0]["frame_index"]
        controller._session, controller._profile = session, profile
        reader = SnapshotReader(controller, snapshot_hz, trace=trace, mode=snapshot_mode)
        if trace:
            trace.start()
        try:
            for index, frame in enumerate(frames):
                with session.lock:
                    clock.stamp, fusion.frame = frame["timestamp_s"], frame
                    session.last_sample = frame["timestamp_s"] - frame["nodes"][root]["age_s"]
                    # Preserve the original recorded frame identity even when a
                    # duplicate input correctly produces no output in between.
                    session.frames = frame["frame_index"]
                if index == warmup:
                    reader.measure.set()
                    measured_start = time.perf_counter()
                start = time.perf_counter()
                token = trace.begin("tick", frame["frame_index"], measured=index >= warmup) if trace else None
                emitted = None
                try:
                    emitted = controller._tick(session)
                finally:
                    if trace:
                        trace.end(token, emitted=emitted)
                elapsed_ms = (time.perf_counter() - start) * 1000.
                if not emitted:
                    if index >= warmup:
                        duplicate_durations.append(elapsed_ms)
                    continue  # Never count a suppressed duplicate as a fast new solve.
                expected_indices.append(frame["frame_index"])
                snapshot = controller._snapshot  # Read only; no extra per-frame deep copy in the driver.
                pose = snapshot["pose"]
                if (not pose.get("pose_available") or not pose.get("solver_success")
                        or not pose.get("upstream_fsglove_solver_executed") or not pose.get("source_fresh")):
                    raise RuntimeError(f"Frame {frame['frame_index']} did not produce a distinct official pose")
                if (snapshot.get("geometry_error")
                        or not (snapshot.get("geometry") or snapshot.get("display_geometry"))):
                    raise RuntimeError(f"Frame {frame['frame_index']} did not produce a current MANO mesh")
                if index >= warmup:
                    durations.append(elapsed_ms)
                    available += 1
                    for key in component_times:
                        component_times[key].append(snapshot["runtime"]["timing_ms"][key])
                    pending.append(writer.snapshot()["pending_pairs"])
            loop_finished = time.perf_counter()
            end_loop_recording = writer.snapshot()
            snapshot_stats = reader.close()
            drain_started = time.perf_counter()
            drain_done = writer.close(5.)
            drain_ms = (time.perf_counter() - drain_started) * 1000.
            gc_diagnostic = trace.finish(writer_process=result_recorder == "process") if trace else None
            finished_recording = writer.snapshot()
            exporter_stats = exporter.snapshot()
            transport = receiver.finish(exporter_stats["sent"])
            logs = verify_logs(paths, expected_indices) if drain_done else None
            if (not drain_done or finished_recording["error"] or finished_recording["rejected_pairs"]
                    or finished_recording["failed_pairs"] or finished_recording["pending_pairs"]
                    or finished_recording["written_pairs"] != finished_recording["accepted_pairs"]):
                raise RuntimeError(f"Incomplete result recording: {finished_recording}")
            if snapshot_stats["errors"] or not snapshot_stats["reader_stopped"]:
                raise RuntimeError(f"Snapshot reader failed: {snapshot_stats}")
            if transport["errors"] or transport["received"] != exporter_stats["sent"]:
                raise RuntimeError(f"Datagram receiver did not drain successful sends: {transport}")
            budget = 1000. / target_hz
            wall_s = loop_finished - measured_start
            result = dict(measured_frames=len(durations), measured_input_records=len(frames) - warmup,
                measured_duplicate_inputs_suppressed=len(duplicate_durations), warmup_frames=warmup,
                duplicate_tick_latency_ms=latency_stats(duplicate_durations, deadline_ms=budget),
                emitted_output_frames_including_warmup=len(expected_indices),
                available_official_results=available,
                tick_wall_latency_ms=latency_stats(durations, deadline_ms=budget),
                tick_components_ms={key: latency_stats(values, deadline_ms=budget)
                                    for key, values in component_times.items()},
                measured_loop_wall_s=wall_s, measured_loop_throughput_fps=len(durations) / wall_s,
                throughput_including_result_drain_fps=len(durations) / (wall_s + drain_ms / 1000.),
                recording=dict(at_loop_end=end_loop_recording, after_drain=finished_recording,
                    drain_done=drain_done, drain_ms=drain_ms, max_observed_pending_pairs=max(pending),
                    pending_pairs_after_each_measured_tick=pending, logs=logs,
                    real_temporary_files=True, temporary_files_removed_after_verification=True),
                exporter=exporter_stats, local_transport=transport, snapshot_reader=snapshot_stats,
                controller_threads_started=False, source_opened=False,
                frame_clock="frozen original timestamp_s per frame; perf_counter measures wall compute only",
                duplicate_inputs_skipped=session.duplicate_inputs_skipped,
                timestamps_rebased=False, config_modified=False, result_recorder=result_recorder)
            if gc_diagnostic is not None:
                result["gc_diagnostic"] = gc_diagnostic
            return result
        finally:
            reader.close()
            writer.close(5.)
            receiver.finish(exporter.sent)
            exporter.close()
            if trace and trace.running:
                trace.finish(writer_process=result_recorder == "process")


@contextmanager
def prepared_benchmark_inputs(args):
    """Matched startup ordering: static models, optional policy, dynamic recording.

    Both A/B paths initialize identical modules/assets before loading any saved
    profile or orientation records. In freeze mode, no historical frames are
    present in the permanent generation created by the startup helper.
    """
    policy = None
    try:
        if args.startup_gc_freeze:
            from vt_fsglove12.live_gc import StartupGcFreeze
            # This CLI is a fresh dedicated interpreter. Capture its bootstrap
            # permanent-generation baseline before torch/model imports; do not
            # silently grant this permission to generic in-process callers.
            policy = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        _configure_tensor_threads()
        from vt_fsglove12.calibration import validate_rest
        from vt_fsglove12.config import validate_config
        from vt_fsglove12.mano import ManoBackend
        from vt_fsglove12.upstream_backend import OfficialFSGloveBackend
        # Match the production startup's imports, without constructing a controller,
        # opening a source, starting a worker or loading any session profile/frames.
        from vt_fsglove12 import live_recording, live_runtime, pose_export  # noqa: F401
        if args.result_recorder == "process":
            from vt_fsglove12 import live_recording_process  # noqa: F401
        with (args.session / "config.json").open(encoding="utf-8") as stream:
            config = validate_config(json.load(stream))
        with (args.session / "rest.json").open(encoding="utf-8") as stream:
            rest = json.load(stream)
        validate_rest(rest)
        backend = OfficialFSGloveBackend(config, rest, checkout=args.fsglove_source,
                                        mano_assets_root=args.mano_assets)
        geometry = ManoBackend(args.mano_assets, hand=config["hand"],
            **({"compact_live_geometry": True} if args.compact_live_geometry else {}))
        geometry.export_rest()  # ManoBackend is lazy: assets must be materialized before freeze.
        if policy is not None:
            policy.start()
        recording = load_recording(args.session, count=args.frames + args.warmup,
                                   max_bytes=args.tail_mib * 1024 * 1024)
        if config != recording["config"] or rest != recording["rest"]:
            raise RuntimeError("Static config/rest changed while the benchmark was preparing its inputs")
        yield recording, backend, geometry, policy
    finally:
        if policy is not None:
            policy.close()


def _configure_tensor_threads():
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--fsglove-source", required=True, type=Path)
    parser.add_argument("--mano-assets", required=True, type=Path)
    parser.add_argument("--frames", type=int, default=200)
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--tail-mib", type=int, default=16, help="Explicit bounded input read budget")
    parser.add_argument("--target-hz", type=float, default=120.)
    parser.add_argument("--snapshot-hz", type=float, default=60.)
    parser.add_argument("--compact-live-geometry", action="store_true",
                        help="Use production compact array-backed geometry when supported by ManoBackend")
    parser.add_argument("--trace-gc", action="store_true",
                        help="Passively correlate parent GC with tick/snapshot/writer wall intervals; adds overhead")
    parser.add_argument("--result-recorder", choices=("thread", "process"), default="thread",
                        help="JSONL worker isolation; process mode opens its files only in the child")
    parser.add_argument("--snapshot-mode", choices=("copy", "publication"), default="copy",
                        help="Public pose deepcopy, or trusted publication plus real envelope pickle")
    parser.add_argument("--startup-gc-freeze", action="store_true",
                        help="EXPERIMENT: startup collect(2)/freeze of initialized static models before loading "
                             "any profile/frames; automatic GC stays enabled; unfreeze on exit")
    parser.add_argument("--output", required=True, type=Path, help="New, exclusive JSON report")
    args = parser.parse_args(argv)
    if (args.frames < 2 or args.warmup < 1 or args.tail_mib < 1
            or not math.isfinite(args.target_hz) or args.target_hz <= 0
            or not math.isfinite(args.snapshot_hz) or not 0 <= args.snapshot_hz <= 240):
        parser.error("Need frames>=2, warmup>=1, tail-mib>=1, positive target rate and snapshot-hz in [0,240]")
    if args.output.exists():
        parser.error(f"Refusing to overwrite existing output: {args.output}")
    for variable in THREAD_VARIABLES:
        os.environ[variable] = "1"
    with prepared_benchmark_inputs(args) as (recording, backend, geometry, startup_policy):
        profile_before = deepcopy(recording["profile"])
        result = run_tick_benchmark(recording, backend, geometry, warmup=args.warmup,
                                   target_hz=args.target_hz, snapshot_hz=args.snapshot_hz,
                                   trace_gc=args.trace_gc, result_recorder=args.result_recorder,
                                   snapshot_mode=args.snapshot_mode)
        if recording["profile"] != profile_before:
            raise RuntimeError("Supplied recorded profile was mutated")
    result["startup_gc_policy"] = dict(requested=args.startup_gc_freeze,
        startup_order=["optional early fresh-interpreter permanent-generation baseline token",
                       "torch thread setup and static config/rest only", "official core and MANO export_rest initialized",
                       "optional startup GC policy", "recording profile and orientation frames loaded",
                       "warmup and measured ticks", "policy cleanup"],
        recorded_frames_loaded_before_startup_policy=0,
        profile_loaded_before_startup_policy=False,
        policy_after_cleanup=startup_policy.snapshot() if startup_policy is not None else None,
        scope="Separate opt-in startup policy; --trace-gc itself remains passive and calls no collection")
    import torch  # Already initialized inside preparation; used here only for environment metadata.
    result.update(schema_version=1, benchmark="official_live_tick_offline_capacity",
        hardware_accessed=False, is_120hz_hardware_validation=False,
        target_compute_hz=args.target_hz, profile_unchanged=True,
        compact_live_geometry=args.compact_live_geometry,
        input=source_summary(recording["frames"][args.warmup:], recording["config"]),
        complete_window=source_summary(recording["frames"], recording["config"]),
        source_session=str(recording["session"]), source_profile=str(recording["profile_path"]),
        bounded_read=recording["tail"],
        upstream=backend.core.provenance, implementation=implementation_provenance(),
        runtime_implementation={name: hashlib.sha256(
            (Path(__file__).resolve().parents[1] / "src" / "vt_fsglove12" / f"{name}.py").read_bytes()).hexdigest()
            for name in ("live_runtime", "live_recording", "pose_export", "live_process", "mano",
                         *(("live_recording_process",) if args.result_recorder == "process" else ()),
                         *(("live_gc",) if args.startup_gc_freeze else ()))},
        benchmark_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        environment=dict(python=sys.version.split()[0], platform=platform.platform(),
            threads={key: os.environ[key] for key in THREAD_VARIABLES},
            torch_threads=torch.get_num_threads(), torch_interop_threads=torch.get_num_interop_threads()),
        inclusions=["real complete _tick", "official solver and five-finger functional layer",
                    "MANO every emitted frame", "display cache and snapshot publication",
                    "bounded asynchronous JSONL to temporary disk files",
                    "nonblocking local pose exporter with draining/JSON-decoding receiver",
                    "optional concurrent production snapshot reads (see snapshot_reader.mode)",
                    *(["real highest-protocol pose-envelope pickle"] if args.snapshot_mode == "publication" else [])],
        exclusions=["serial hardware and raw recorder", "VQF fusion", "paced _run scheduler",
                    "ROS DDS and rosbag", "GUI/OpenGL", "IPC shared memory", "model loading and warmup",
                    *(["IPC pickle"] if args.snapshot_mode == "copy" else [])],
        limitations=["This is compute capacity, not proof of sustained 120 Hz fresh hardware observations.",
                     "Recorded timestamps and sample identities remain unchanged; no synthetic 120 Hz trajectory.",
                     "Frozen per-frame source clock means real-time freshness/late-frame rejection is not measured.",
                     "Inspect deadline misses and queue growth, not only median compute time."])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
    print(json.dumps(dict(output=str(args.output), frames=result["measured_frames"],
        tick=result["tick_wall_latency_ms"], throughput_fps=result["measured_loop_throughput_fps"],
        max_pending=result["recording"]["max_observed_pending_pairs"],
        drained=result["recording"]["drain_done"], exported=result["exporter"],
        snapshot_reads=result["snapshot_reader"]["reads"], hardware_accessed=False), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
