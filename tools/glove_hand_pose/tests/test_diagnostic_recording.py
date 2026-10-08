"""Diagnostic writer and worker contracts, entirely without hardware."""

import json
import os
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from vt_glove_pose.diagnostic_recording import DiagnosticRecorder, TIME_BASIS


def done(recorder):
    if recorder._thread is not None:
        recorder._thread.join(3)
    assert not recorder.is_alive
    return recorder.status()


def events(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def start(tmp_path, **kwargs):
    recorder = DiagnosticRecorder(tmp_path / "evidence.jsonl", **kwargs)
    assert recorder.start({"config": {}, "independent": [1]})
    return recorder


class StreamWrapper:
    def __init__(self, wrapped, *, write_error=False, flush_error=False, close_error=False,
                 entered=None, release=None):
        self.wrapped = wrapped
        self.write_error, self.flush_error, self.close_error = write_error, flush_error, close_error
        self.entered, self.release = entered, release

    def __getattr__(self, name):
        return getattr(self.wrapped, name)

    def write(self, data):
        if self.entered is not None:
            self.entered.set()
            assert self.release.wait(3), "test did not release slow writer"
        if self.write_error:
            raise OSError("disk full on write")
        return self.wrapped.write(data)

    def flush(self):
        if self.flush_error:
            raise OSError("disk full on flush")
        self.wrapped.flush()

    def close(self):
        self.wrapped.close()
        if self.close_error:
            raise OSError("disk full on close")


def test_ordered_exclusive_jsonl_and_independent_copies(tmp_path):
    path = tmp_path / "session.jsonl"
    checkpoint = {"config": {"hand": "left"}, "state": [1]}
    recorder = DiagnosticRecorder(path)
    assert recorder.start(checkpoint)
    checkpoint["state"][0] = 99
    performance = {"basis": TIME_BASIS, "batch_samples": 0}
    assert recorder.offer("batch", samples=[], performance=performance)
    performance["batch_samples"] = 999
    assert recorder.offer("command", name="neutral", args={"duration": 5, "confirm_mount": True})
    assert recorder.offer("marker", label="returned")
    assert recorder.offer("snapshot", now_s=1.5, snapshot={"model": {}}, performance={})
    assert recorder.offer("checkpoint", checkpoint={"config": {}, "state": 2})
    recorder.stop("manual")
    status = done(recorder)
    assert status["state"] == "complete"
    assert status["pending_bytes"] == 0
    rows = events(path)
    assert [row["seq"] for row in rows] == list(range(7))
    assert [row["type"] for row in rows] == ["header", "batch", "command", "marker", "snapshot", "checkpoint", "end"]
    assert rows[0]["kind"] == "vt_glove_pose.diagnostic_session"
    assert rows[0]["schema_version"] == 1
    assert rows[0]["time_basis"] == TIME_BASIS
    assert rows[0]["checkpoint"]["state"] == [1]
    assert rows[1]["performance"]["batch_samples"] == 0
    assert rows[-1]["complete"] is True
    assert rows[-1]["reason"] == "manual"


def test_existing_file_is_never_overwritten(tmp_path):
    path = tmp_path / "existing.jsonl"
    path.write_text("old evidence\n")
    recorder = DiagnosticRecorder(path)
    recorder.start({"config": {}})
    assert done(recorder)["state"] == "failed"
    assert path.read_text() == "old evidence\n"


@pytest.mark.parametrize("failure", ["write_error", "flush_error", "close_error"])
def test_disk_write_flush_and_close_failure_never_leaves_complete_footer(tmp_path, failure):
    recorder = DiagnosticRecorder(tmp_path / "failure.jsonl")
    original = recorder._open_stream
    recorder._open_stream = lambda: StreamWrapper(original(), **{failure: True})
    recorder.start({"config": {}})
    recorder.offer("marker", label="rest")
    recorder.stop()
    status = done(recorder)
    assert status["state"] == "failed"
    assert "disk full" in status["error"]
    assert not any(row.get("complete") for row in events(recorder.path))


def test_write_and_close_errors_are_aggregated(tmp_path):
    recorder = DiagnosticRecorder(tmp_path / "failure.jsonl")
    original = recorder._open_stream
    recorder._open_stream = lambda: StreamWrapper(original(), write_error=True, close_error=True)
    recorder.start({"config": {}})
    status = done(recorder)
    assert "on write" in status["error"] and "on close" in status["error"]


def test_slow_writer_stop_is_nonblocking_and_queue_exhaustion_is_explicit(tmp_path):
    entered, release = threading.Event(), threading.Event()
    recorder = DiagnosticRecorder(tmp_path / "slow.jsonl", max_queue_items=1)
    original = recorder._open_stream
    recorder._open_stream = lambda: StreamWrapper(original(), entered=entered, release=release)
    try:
        recorder.start({"config": {}})
        assert entered.wait(1)
        assert recorder.offer("marker", label="rest")
        before = time.perf_counter()
        assert not recorder.offer("marker", label="moving")
        recorder.stop()
        assert time.perf_counter() - before < .2
        assert recorder.is_alive
        assert recorder.status()["state"] == "failed"
        assert recorder.status()["queued_events"] <= 1
    finally:
        release.set()
        done(recorder)
    assert not any(row.get("complete") for row in events(recorder.path))


def test_pending_byte_budget_includes_inflight_payload(tmp_path):
    entered, release = threading.Event(), threading.Event()
    recorder = DiagnosticRecorder(tmp_path / "bytes.jsonl", max_queue_bytes=2000)
    original = recorder._open_stream
    recorder._open_stream = lambda: StreamWrapper(original(), entered=entered, release=release)
    try:
        recorder.start({"config": {}})
        assert entered.wait(1)
        assert recorder.status()["pending_bytes"] > 0
        assert not recorder.offer("checkpoint", checkpoint={"large": "x" * 1000})
        assert recorder.status()["peak_pending_bytes"] <= 2000
    finally:
        release.set()
        done(recorder)


@pytest.mark.parametrize("kind,payload", [
    ("marker", {"label": "not-an-action"}),
    ("command", {"name": "load", "args": {"path": "external-file"}}),
    ("command", {"name": "neutral", "args": {"duration": 4, "confirm_mount": True}}),
    ("batch", {"samples": [{"values": [1]}], "performance": {}}),
    ("batch", {"samples": [], "performance": {"x": float("nan")}}),
    ("snapshot", {"now_s": 1, "snapshot": {"runtime": {}}, "performance": {}}),
    ("checkpoint", {"checkpoint": {"x": "a" * 2_000_001}}),
    ("checkpoint", {"checkpoint": {"x": [0] * 4097}}),
    ("checkpoint", {"checkpoint": {"x": object()}}),
    ("checkpoint", {"checkpoint": {}, "seq": 99}),
])
def test_invalid_or_oversized_events_fail_bounded_session(tmp_path, kind, payload):
    recorder = start(tmp_path)
    assert not recorder.offer(kind, **payload)
    assert done(recorder)["state"] == "failed"
    assert not any(row.get("complete") for row in events(recorder.path))


def test_header_and_footer_obey_line_limit(tmp_path):
    recorder = DiagnosticRecorder(tmp_path / "header.jsonl", max_event_bytes=200)
    assert not recorder.start({"config": {}, "large": "a" * 100})
    assert recorder.status()["state"] == "failed"
    assert not recorder.path.exists()
    recorder = DiagnosticRecorder(tmp_path / "footer.jsonl", max_event_bytes=1500)
    assert recorder.start({"config": {}})
    recorder.stop("\x00" * 256)
    assert done(recorder)["state"] == "failed"
    assert all(len(line) <= 1500 for line in recorder.path.read_bytes().splitlines(keepends=True))
    assert not any(row["type"] == "end" for row in events(recorder.path))


def test_stop_prevents_more_records_and_recorder_restart(tmp_path):
    recorder = start(tmp_path)
    recorder.stop()
    assert not recorder.offer("marker", label="rest")
    assert done(recorder)["state"] == "complete"
    assert not recorder.offer("bogus", invalid=True)
    assert recorder.status()["state"] == "complete"
    with pytest.raises(RuntimeError):
        recorder.start({})


class FakeEngine:
    def __init__(self, config):
        self.config, self.active, self.count = config, None, 0
        self.commands, self.now_values = [], []

    def export_diagnostic_checkpoint(self):
        return {"config": self.config, "count": self.count}

    def process(self, sample):
        self.count += 1

    def get_snapshot(self, now=None):
        self.now_values.append(now)
        return {"model": {}, "nodes": {}, "count": self.count}

    def begin_neutral(self, *, duration, confirm_mount):
        if not confirm_mount:
            raise ValueError("confirm required")
        self.active = {"kind": "neutral"}
        self.commands.append(("neutral", duration))

    def begin_axis(self, *, target, duration):
        self.active = {"kind": "axis"}
        self.commands.append((target, duration))

    def cancel_calibration(self):
        self.active = None

    def load_calibration(self, path):
        self.count = 99

    def save_calibration(self, path):
        pass


def worker(tmp_path, *, factory=None):
    pytest.importorskip("PyQt5")
    from vt_glove_pose.config import load_config
    from vt_glove_pose.gui import PoseWorker
    from vt_glove_pose.records import Sample

    class Source:
        closed = False
        def open(self):
            pass
        def close(self):
            self.closed = True
        def read(self):
            return [Sample(1, (0, 0, 0, 0, 0, 8197), 1.0, 10.0, 7)]

    source = Source()
    result = PoseWorker(load_config(), engine_factory=FakeEngine,
                        serial_factory=lambda path: source, diagnostic_factory=factory)
    result._engine = FakeEngine(load_config())
    result._command("connect", {"source": "fake-no-device"})
    return result, source, tmp_path / "worker.jsonl"


def test_worker_only_successful_commands_and_loaded_checkpoint_in_order(tmp_path):
    result, source, path = worker(tmp_path)
    result._command("diagnostic_start", {"path": path})
    with pytest.raises(ValueError):
        result._command("neutral", {"confirm_mount": False})
    result._command("neutral", {"confirm_mount": True})
    result._command("cancel", {})
    result._command("axis", {"target": "index"})
    result._command("cancel", {})
    result._command("load", {"path": "fake-external-path-not-opened"})
    result._command("save", {"path": tmp_path / "not-written.json"})
    result._command("diagnostic_marker", {"label": "returned"})
    result._diagnostic_batch([])
    result._publish(0)
    result._command("diagnostic_stop", {})
    assert done(result._diagnostic)["state"] == "complete"
    rows = events(path)
    assert [row["type"] for row in rows] == ["header", "command", "command", "command", "command", "checkpoint", "marker", "batch", "snapshot", "end"]
    assert rows[5]["checkpoint"]["count"] == 99
    assert "path" not in rows[5]
    assert rows[7]["samples"] == []
    assert rows[8]["now_s"] == result._engine.now_values[-1]
    assert "runtime" not in rows[8]["snapshot"]
    assert result.latest_snapshot()["runtime"]["diagnostic"]["state"] == "complete"
    assert not source.closed


def test_worker_start_is_live_manual_connected_and_not_mid_capture(tmp_path):
    result, source, path = worker(tmp_path)
    assert result.latest_snapshot()["runtime"]["diagnostic"]["state"] == "idle"
    assert not path.exists()
    result._engine.active = {"kind": "neutral"}
    with pytest.raises(ValueError, match="标定"):
        result._command("diagnostic_start", {"path": path})
    assert result._engine.active is not None
    result._engine.active = None
    for mode in ("demo", "replay"):
        result.mode = mode
        with pytest.raises(ValueError):
            result._command("diagnostic_start", {"path": path})
    result.mode = "live"
    result._source = None
    with pytest.raises(ValueError):
        result._command("diagnostic_start", {"path": path})
    assert not path.exists()


def test_worker_disconnect_stops_only_its_session(tmp_path):
    result, source, path = worker(tmp_path)
    result._command("diagnostic_start", {"path": path})
    result._command("disconnect", {})
    assert source.closed
    assert done(result._diagnostic)["state"] == "complete"
    assert events(path)[-1]["reason"] == "disconnect"


def test_worker_no_new_thread_while_previous_writer_is_blocked(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def factory(path):
        recorder = DiagnosticRecorder(path)
        original = recorder._open_stream
        recorder._open_stream = lambda: StreamWrapper(original(), entered=entered, release=release)
        return recorder
    result, source, path = worker(tmp_path, factory=factory)
    try:
        result._command("diagnostic_start", {"path": path})
        assert entered.wait(1)
        result._command("diagnostic_stop", {})
        assert result._diagnostic_status()["state"] == "stopping"
        with pytest.raises(ValueError, match="线程"):
            result._command("diagnostic_start", {"path": tmp_path / "another.jsonl"})
        result._fail_diagnostic("test failure while blocked")
        with pytest.raises(ValueError, match="线程"):
            result._command("diagnostic_start", {"path": tmp_path / "another.jsonl"})
        assert not source.closed
    finally:
        release.set()
        done(result._diagnostic)


def run_one_publish(result):
    snapshots = []
    publish = result._publish
    def once(elapsed):
        publish(elapsed)
        snapshots.append(result.latest_snapshot())
        result.stop()
    result._publish = once
    result.run()
    return snapshots[0]


def test_worker_jsonl_failure_does_not_stop_batch_fusion(tmp_path):
    def factory(path):
        recorder = DiagnosticRecorder(path)
        original = recorder._open_stream
        recorder._open_stream = lambda: StreamWrapper(original(), write_error=True, close_error=True)
        return recorder
    result, source, path = worker(tmp_path, factory=factory)
    result.submit("diagnostic_start", path=path)
    snapshot = run_one_publish(result)
    done(result._diagnostic)
    assert snapshot["runtime"]["connected"]
    assert result._engine.count == 1
    assert result._diagnostic_status()["state"] == "failed"
    assert "数据源已停止" not in result.latest_snapshot()["runtime"]["error"]


def test_worker_batch_snapshot_order_and_shutdown_end(tmp_path):
    result, source, path = worker(tmp_path)
    result.submit("diagnostic_start", path=path)
    run_one_publish(result)
    assert done(result._diagnostic)["state"] == "complete"
    rows = events(path)
    assert [row["type"] for row in rows] == ["header", "batch", "snapshot", "end"]
    assert rows[1]["samples"][0] == dict(node_id=1, values=[0, 0, 0, 0, 0, 8197], host_monotonic_s=1.0, host_wall_s=10.0, status=7)
    assert rows[2]["snapshot"]["count"] == 1
    assert rows[-1]["reason"] == "worker_shutdown"


def test_partial_fusion_batch_marks_diagnostic_incomplete(tmp_path):
    result, source, path = worker(tmp_path)
    engine = FakeEngine(result.config)
    def process(sample):
        engine.count += 1
        raise ValueError("partial fusion")
    engine.process = process
    result._engine_factory = lambda config: engine
    result.submit("diagnostic_start", path=path)
    run_one_publish(result)
    assert done(result._diagnostic)["state"] == "failed"
    rows = events(path)
    assert not any(row["type"] in {"end", "batch"} for row in rows)


def test_worker_source_error_ends_existing_evidence_without_failed_batch(tmp_path):
    result, source, path = worker(tmp_path)
    def fail_read():
        raise OSError("fake disconnect")
    source.read = fail_read
    result.submit("diagnostic_start", path=path)
    run_one_publish(result)
    assert done(result._diagnostic)["state"] == "complete"
    rows = events(path)
    assert rows[-1]["reason"] == "source_error"
    assert not any(row["type"] == "batch" for row in rows)


def test_real_worker_recorder_engine_session_reconstructs_exactly(tmp_path, monkeypatch):
    from vt_glove_pose.diagnostics import diagnose_session
    from vt_glove_pose.engine import PoseEngine
    from vt_glove_pose.records import Sample

    result, source, path = worker(tmp_path)
    result._engine = PoseEngine(result.config)
    current = [100.0]
    monkeypatch.setattr("vt_glove_pose.gui.time.monotonic", lambda: current[0])

    def batch(stamp, gyro=0):
        return [Sample(node, (gyro, 0, 0, 0, 0, 8197), stamp, 1000 + stamp, node)
                for node in range(1, 13)]

    for sample in batch(100.0):
        result._engine.process(sample)
    result._engine.begin_neutral(confirm_mount=True)
    for step in range(1, 607):
        for sample in batch(100 + step / 120):
            result._engine.process(sample)
    assert result._engine.neutral_ready
    calibration = tmp_path / "calibration.json"
    result._engine.save_calibration(calibration)
    before = result._engine.export_diagnostic_checkpoint()
    result._command("diagnostic_start", {"path": path})
    assert result._engine.export_diagnostic_checkpoint() == before

    def process_and_record(stamp, gyro=0):
        current[0] = stamp
        samples = batch(stamp, gyro)
        for sample in samples:
            result._engine.process(sample)
        result._diagnostic_batch(samples)
        result._publish(0)

    result._command("diagnostic_marker", {"label": "rest"})
    process_and_record(105.06)
    result._command("neutral", {"confirm_mount": True})
    process_and_record(105.07, gyro=20)
    result._command("cancel", {})
    result._command("load", {"path": calibration})
    result._publish(0)  # Loaded filters and previous displayed quat legitimately differ.
    result._command("diagnostic_marker", {"label": "moving"})
    process_and_record(105.08, gyro=100)
    result._command("neutral", {"confirm_mount": True})
    current[0] = 106.0
    result._diagnostic_batch([])
    result._publish(0)  # Snapshot itself cancels capture after a data gap.
    assert result._engine.active is None
    result._command("diagnostic_marker", {"label": "returned"})
    result._command("diagnostic_stop", {})
    assert done(result._diagnostic)["state"] == "complete"
    report = diagnose_session(path)
    assert report["session_complete"], report
    assert report["reconstruction"]["status"] == "match", report["reconstruction"]
    assert not source.closed


def test_worker_shutdown_wait_is_bounded_and_timeout_fails_session(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def factory(path):
        recorder = DiagnosticRecorder(path)
        original = recorder._open_stream
        recorder._open_stream = lambda: StreamWrapper(original(), entered=entered, release=release)
        return recorder
    result, source, path = worker(tmp_path, factory=factory)
    try:
        result.submit("diagnostic_start", path=path)
        before = time.perf_counter()
        run_one_publish(result)
        assert time.perf_counter() - before < 1.0
        assert result._diagnostic_status()["state"] == "failed"
        assert "超时" in result._diagnostic_status()["error"]
        assert source.closed
    finally:
        release.set()
        done(result._diagnostic)
    assert not any(row.get("complete") for row in events(path))
