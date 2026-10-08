"""Replay-clock and recorder-failure regressions; no real serial devices."""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from unittest.mock import Mock
from itertools import count

import pytest

pytest.importorskip("PyQt5")

from vt_glove_pose.config import load_config
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.gui import PoseWorker
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import RawRecorder, ReplaySource


def samples_at(stamp):
    return [Sample(node, (0, 0, 0, 0, 0, 8197), stamp, 1000 + stamp) for node in range(1, 13)]


def write_recording(path, samples):
    with RawRecorder(path) as recorder:
        recorder.write(samples)


def worker_with_source(source):
    worker = PoseWorker(load_config(), mode="replay")
    worker._engine = PoseEngine(load_config())
    worker._source = source
    return worker


def stop_after_publish(worker):
    """Run one normal worker-loop iteration synchronously without GUI timers."""
    snapshots = []
    original = worker._publish

    def publish(elapsed):
        original(elapsed)
        snapshots.append(worker.latest_snapshot())
        worker.stop()

    worker._publish = publish
    return snapshots


@pytest.mark.parametrize("speed", [0.5, 1.0, 2.0])
def test_replay_timeline_advances_through_no_sample_gap(monkeypatch, tmp_path, speed):
    path = tmp_path / "gap.csv"
    write_recording(path, samples_at(10) + samples_at(20))
    clock = Mock(return_value=100.0)
    monkeypatch.setattr("vt_glove_pose.stream.time.monotonic", clock)
    source = ReplaySource(path, speed=speed)
    assert source.timeline_s is None
    with source:
        assert source.timeline_s == 10
        assert len(source.read()) == 12
        clock.return_value = 101.0
        assert source.read() == []
        assert source.timeline_s == pytest.approx(10 + speed)
        assert not source.eof
    assert source.timeline_s is None


def test_long_replay_gap_expires_nodes_and_cancels_capture(monkeypatch, tmp_path):
    path = tmp_path / "gap.csv"
    write_recording(path, samples_at(10) + samples_at(20))
    clock = Mock(return_value=100.0)
    monkeypatch.setattr("vt_glove_pose.stream.time.monotonic", clock)
    with ReplaySource(path) as source:
        worker = worker_with_source(source)
        for sample in source.read():
            worker._engine.process(sample)
        worker._engine.begin_neutral(confirm_mount=True)
        worker._publish(0)
        assert all(node["fresh"] for node in worker.latest_snapshot()["nodes"].values())
        clock.return_value = 100.6
        assert source.read() == []
        worker._publish(0.6)
        snapshot = worker.latest_snapshot()
        assert all(not node["fresh"] for node in snapshot["nodes"].values())
        assert snapshot["nodes"]["1"]["age_s"] == pytest.approx(0.6)
        assert worker._engine.last_time == 10  # Arrival time is not rewritten.
        assert worker._engine.active is None
        assert snapshot["calibration"]["active"] is None


def test_replay_timeline_freezes_only_after_eof(monkeypatch, tmp_path):
    path = tmp_path / "short.csv"
    write_recording(path, samples_at(10) + samples_at(10.1))
    clock = Mock(return_value=100.0)
    monkeypatch.setattr("vt_glove_pose.stream.time.monotonic", clock)
    with ReplaySource(path) as source:
        worker = worker_with_source(source)
        for sample in source.read():
            worker._engine.process(sample)
        clock.return_value = 100.2
        for sample in source.read():
            worker._engine.process(sample)
        assert source.eof
        assert source.timeline_s == 10.1
        worker._eof = True
        clock.return_value = 500.0
        worker._publish(400)
        snapshot = worker.latest_snapshot()
        assert source.timeline_s == 10.1
        assert snapshot["nodes"]["1"]["age_s"] == 0
        assert snapshot["runtime"]["eof"]


def test_empty_replay_has_no_presentation_timestamp(tmp_path):
    path = tmp_path / "empty.csv"
    write_recording(path, [])
    with ReplaySource(path) as source:
        assert source.eof
        assert source.timeline_s is None
        assert source.read() == []


def test_worker_eof_cancels_unfinished_calibration():
    engine = PoseEngine(load_config())
    for sample in samples_at(10):
        engine.process(sample)
    engine.begin_neutral(confirm_mount=True)

    class EndingSource:
        eof = True
        timeline_s = 10.1
        closed = False

        def open(self):
            pass

        def read(self):
            return samples_at(10.1)

        def close(self):
            self.closed = True

    source = EndingSource()
    worker = PoseWorker(load_config(), mode="replay", engine_factory=lambda config: engine,
                        replay_factory=lambda path, speed: source)
    worker.submit("connect", source="mock-source-no-file")
    snapshots = stop_after_publish(worker)
    worker.run()
    assert engine.active is None
    assert snapshots[0]["calibration"]["active"] is None
    assert snapshots[0]["runtime"]["eof"]
    assert "未完成的标定已取消" in snapshots[0]["runtime"]["message"]
    assert source.closed


@pytest.mark.parametrize("close_fails", [False, True])
def test_recording_error_never_disconnects_acquisition(close_fails):
    class LiveSource:
        closes = 0

        def open(self):
            pass

        def read(self):
            return samples_at(10)

        def close(self):
            self.closes += 1

    class FailingRecorder:
        closes = 0

        def open(self):
            pass

        def write(self, samples):
            raise OSError("disk full during write")

        def close(self):
            self.closes += 1
            if close_fails:
                raise OSError("disk full during close")

    source, recorder = LiveSource(), FailingRecorder()
    worker = PoseWorker(load_config(), serial_factory=lambda path: source,
                        recorder_factory=lambda path: recorder)
    worker.submit("connect", source="mock-source-no-device")
    worker.submit("record_start", path="mock-recorder-no-file")
    snapshots = stop_after_publish(worker)
    worker.run()
    snapshot = snapshots[0]
    assert snapshot["runtime"]["connected"]  # Still connected before normal shutdown.
    assert snapshot["runtime"]["recording"] is None
    assert all(node["samples"] == 1 for node in snapshot["nodes"].values())
    assert "采集继续" in snapshot["runtime"]["error"]
    assert "disk full during write" in snapshot["runtime"]["error"]
    assert ("disk full during close" in snapshot["runtime"]["error"]) == close_fails
    assert "数据源已停止" not in snapshot["runtime"]["error"]
    assert source.closes == 1  # Only normal worker shutdown, never a read-error close.
    assert recorder.closes == 1


def test_worker_publishes_host_only_read_fusion_and_queue_observations(monkeypatch):
    class Source:
        performance = {
            "serial_backlog_before_bytes": 168,
            "serial_backlog_after_bytes": 0,
            "serial_backlog_peak_bytes": 168,
            "raw_read_elapsed_ms": 0.2,
        }

        def open(self):
            pass

        def read(self):
            return samples_at(10)

        def close(self):
            pass

    ticks = count(1.0, 0.001)
    monkeypatch.setattr("vt_glove_pose.gui.time.perf_counter", lambda: next(ticks))
    monkeypatch.setattr("vt_glove_pose.gui.time.monotonic", lambda: 12345.0)
    source = Source()
    worker = PoseWorker(load_config(), serial_factory=lambda path: source)
    worker.submit("connect", source="mock-only")
    snapshots = stop_after_publish(worker)
    worker.run()
    performance = snapshots[0]["runtime"]["performance"]
    assert performance["basis"] == "host_observed_not_sensor_latency"
    assert performance["batch_samples"] == 12
    assert performance["batch_samples_max"] == 12
    assert performance["total_samples"] == 12
    assert performance["read_batches"] == 1
    assert performance["source_read_elapsed_ms"] == pytest.approx(1)
    assert performance["fusion_elapsed_ms"] == pytest.approx(1)
    assert performance["fusion_elapsed_ms_max"] == pytest.approx(1)
    assert performance["record_elapsed_ms"] == 0
    assert performance["snapshot_build_elapsed_ms"] == pytest.approx(1)
    assert performance["snapshot_published_monotonic_s"] == 12345.0
    assert performance["serial_backlog_before_bytes"] == 168
    assert performance["serial_backlog_after_bytes"] == 0
    assert performance["raw_read_elapsed_ms"] == 0.2
    assert "sensor_latency_ms" not in performance
    assert all(not isinstance(value, (list, dict, tuple)) for value in performance.values())
    performance["total_samples"] = -1
    assert worker.latest_snapshot()["runtime"]["performance"]["total_samples"] == 12


def test_worker_empty_reads_preserve_peak_and_total_without_growing_history(monkeypatch):
    class Source:
        reads = 0

        def open(self):
            pass

        def read(self):
            self.reads += 1
            return samples_at(10) if self.reads == 1 else []

        def close(self):
            pass

    ticks = count(100.0, 0.1)
    monkeypatch.setattr("vt_glove_pose.gui.time.monotonic", lambda: next(ticks))
    worker = PoseWorker(load_config(), serial_factory=lambda path: Source())
    worker.submit("connect", source="mock-only")
    snapshots = []
    original = worker._publish

    def publish(elapsed):
        original(elapsed)
        snapshots.append(worker.latest_snapshot())
        if len(snapshots) == 2:
            worker.stop()

    worker._publish = publish
    worker.run()
    first = snapshots[0]["runtime"]["performance"]
    second = snapshots[1]["runtime"]["performance"]
    assert first["batch_samples"] == 12
    assert second["batch_samples"] == 0
    assert second["batch_samples_max"] == 12
    assert second["total_samples"] == 12
    assert second["read_batches"] == 2
    assert second["snapshot_published_monotonic_s"] > first["snapshot_published_monotonic_s"]
    assert set(first) == set(second)
    worker._reset_performance()
    assert worker._performance["total_samples"] == 0
    assert worker._performance["batch_samples_max"] == 0
