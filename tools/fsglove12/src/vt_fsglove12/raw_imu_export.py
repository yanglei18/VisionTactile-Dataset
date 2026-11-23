"""Bounded original IMU export, independent of solve/display cadence and ROS.

The reader only hands off detached sample dictionaries. A dedicated worker does
JSON and nonblocking socket I/O. Missing/slow receivers never stop acquisition;
queue and transport losses are retained in counters and the next delivered batch.
"""
from collections import OrderedDict
import json
import os
from pathlib import Path
import queue
import socket
import threading

MAX_BATCH_SAMPLES = 64
QUEUE_CAPACITY = 64


def raw_socket_path(pose_socket):
    path = os.fspath(pose_socket) + ".imu"
    if not Path(path).is_absolute() or "\0" in path or len(os.fsencode(path)) > 107:
        raise ValueError("Raw IMU socket must be an absolute local path of at most 107 bytes")
    return path


def raw_sample_document(sample, sample_index, filter_epoch, protection, fusion_updated):
    real, mono = sample.host_realtime_ns, sample.host_monotonic_ns
    return dict(node_id=sample.node_id, sample_index=sample_index, filter_epoch=filter_epoch,
                host_realtime_ns=real if real is not None else 0,
                host_monotonic_ns=mono if mono is not None else 0,
                time_valid=bool(real is not None and mono is not None and real > 0 and mono > 0),
                raw_values=list(sample.values), status=sample.status if sample.status is not None else 0,
                status_known=sample.status is not None, gyro_rejected=bool(protection["gyro_rejected"]),
                accel_rejected=bool(protection["accel_rejected"]), fusion_updated=bool(fusion_updated))


class RawImuDatagramExporter:
    def __init__(self, pose_socket, *, capacity=QUEUE_CAPACITY):
        if type(capacity) is not int or not 1 <= capacity <= 4096:
            raise ValueError("raw IMU queue capacity must be in 1..4096")
        self.path = raw_socket_path(pose_socket)
        self._queue = queue.Queue(maxsize=capacity)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sequences = OrderedDict()
        self.sent_batches = self.sent_samples = self.dropped_batches = self.dropped_samples = 0
        self.last_error = None
        self._worker = threading.Thread(target=self._run, name="raw-imu-export", daemon=True)
        self._worker.start()

    def publish(self, samples, *, hand, session_id, config_sha256, source_mode):
        """Take ownership of detached rows; bounded enqueue only, never serialization."""
        for offset in range(0, len(samples), MAX_BATCH_SAMPLES):
            batch = samples[offset:offset + MAX_BATCH_SAMPLES]
            with self._lock:
                sequence = self._sequences.get(session_id, 0) + 1
                self._sequences[session_id] = sequence
                self._sequences.move_to_end(session_id)
                if len(self._sequences) > 64:
                    self._sequences.popitem(last=False)
                document = dict(kind="raw_imu_batch", schema_version=1, hand=hand,
                    session_id=session_id, config_sha256=config_sha256, source_mode=source_mode,
                    batch_sequence=sequence, clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
                    samples=batch)
                try:
                    if self._stop.is_set():
                        raise RuntimeError("raw IMU exporter is closed")
                    self._queue.put_nowait(document)
                except (queue.Full, RuntimeError) as exc:
                    self._record_error(exc, len(batch))

    def _record_error(self, exc, sample_count):
        self.dropped_batches += 1
        self.dropped_samples += sample_count
        self.last_error = f"{type(exc).__name__}: {exc}"

    def record_error(self, exc, sample_count):
        with self._lock:
            self._record_error(exc, sample_count)

    def _run(self):
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    document = self._queue.get(timeout=.05)
                except queue.Empty:
                    continue
                try:
                    with self._lock:
                        document.update(producer_dropped_batches_total=self.dropped_batches,
                                        producer_dropped_samples_total=self.dropped_samples)
                    payload = json.dumps(document, separators=(",", ":"), allow_nan=False).encode("utf-8")
                    if len(payload) > 60_000:
                        raise ValueError("raw IMU batch exceeds 60000 bytes")
                    self._socket.sendto(payload, self.path)
                except (OSError, ValueError, TypeError) as exc:
                    self.record_error(exc, len(document["samples"]))
                else:
                    with self._lock:
                        self.sent_batches += 1
                        self.sent_samples += len(document["samples"])
                finally:
                    self._queue.task_done()
        finally:
            self._socket.close()

    def snapshot(self):
        with self._lock:
            return dict(enabled=True, socket_path=self.path, queued_batches=self._queue.qsize(),
                        queue_capacity=self._queue.maxsize, sent_batches=self.sent_batches,
                        sent_samples=self.sent_samples, dropped_batches=self.dropped_batches,
                        dropped_samples=self.dropped_samples, last_error=self.last_error)

    def close(self):
        self._stop.set()
        self._worker.join(2.)
        return not self._worker.is_alive()
