"""Optional isolated result writer contracts; no hardware or performance tests."""
from functools import partial
import json
import multiprocessing as mp
from pathlib import Path
import time

import pytest

from vt_fsglove12 import live_recording_process
from vt_fsglove12.live_recording_process import ProcessResultRecorder


def _paths(root):
    return {name: root / f"{name}.jsonl" for name in ("orientations", "poses")}


def _wait(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "timed out waiting for recorder test condition"
        time.sleep(.005)


class _GatedStream:
    def __init__(self, path, entered, release, *, fail_write=False, fail_close=False):
        self.stream = Path(path).open("x", encoding="utf-8")
        self.gated = Path(path).stem == "orientations"
        self.entered, self.release = entered, release
        self.fail_write, self.fail_close = fail_write, fail_close

    def write(self, value):
        if self.gated:
            self.entered.set()
            if not self.release.wait(5.):
                raise OSError("test gate timeout")
            if self.fail_write:
                raise OSError("test disk full")
        return self.stream.write(value)

    def flush(self):
        self.stream.flush()

    def close(self):
        self.stream.close()
        if self.fail_close:
            raise OSError("test close failure")


class _FailPoseStream(_GatedStream):
    def write(self, value):
        if not self.gated:
            raise OSError("test pose disk full")
        return super().write(value)


def _gated_open(path, *, entered, release):
    entered.set()
    if not release.wait(5.):
        raise OSError("test open gate timeout")
    return Path(path).open("x", encoding="utf-8")


def _status_eof_before_ready(paths, incoming, status, written, flushed, interval, stream_factory):
    # Deterministically close the status channel while the child is still
    # alive, before readiness or a clean-shutdown acknowledgement exists.
    status.close()
    stream_factory(paths["orientations"]).close()
    incoming.close()


def _gates():
    context = mp.get_context("spawn")
    return context.Event(), context.Event()


def test_process_recorder_drains_preserves_order_and_owns_exclusive_files(tmp_path, monkeypatch):
    paths = _paths(tmp_path)
    writer = ProcessResultRecorder(paths)
    try:
        writer.check()
        assert writer.snapshot()["ready"]
        assert writer.process.pid != mp.current_process().pid
        assert not writer.process.daemon

        # No parent JSON serialization, and submit takes an owned pickle copy.
        def no_parent_json(*args, **kwargs):
            raise AssertionError("JSON serialization belongs to recorder child")

        monkeypatch.setattr(json, "dumps", no_parent_json)
        for index in range(25):
            frame, pose = dict(frame=index, text="完成"), dict(pose=index)
            writer.submit(frame, pose, flush=index == 24)
            frame["frame"] = pose["pose"] = -1
    finally:
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["mode"] == "bounded_process"
    assert status["accepted_pairs"] == status["written_pairs"] == status["flushed_pairs"] == 25
    assert status["pending_pairs"] == status["failed_pairs"] == status["rejected_pairs"] == 0
    assert not status["writer_alive"] and status["error"] is None
    assert status["recording_complete"] and not status["incomplete"]
    assert status["status"] == "complete"
    for name, path in paths.items():
        field = "frame" if name == "orientations" else "pose"
        assert [json.loads(line)[field] for line in path.read_text().splitlines()] == list(range(25))
    assert "完成" in paths["orientations"].read_text()
    assert writer.close(0.)
    with pytest.raises(OSError, match="closing"):
        writer.submit({}, {})
    assert writer.snapshot()["rejected_pairs"] == 1


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf"), object()])
def test_process_recorder_strict_json_rejects_pair_before_either_line(tmp_path, value):
    writer = ProcessResultRecorder(_paths(tmp_path))
    writer.check()
    writer.submit({"valid": True}, {"invalid": value})
    assert writer.close(3.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["failed_pairs"] == 1
    assert status["written_pairs"] == status["pending_pairs"] == 0
    assert status["incomplete"] and not status["recording_complete"]
    for path in _paths(tmp_path).values():
        assert path.read_text() == ""
    with pytest.raises(OSError, match="TypeError" if type(value) is object else "ValueError"):
        writer.check()


def test_pickle_failure_is_rejected_before_acceptance_and_earlier_pair_drains(tmp_path):
    writer = ProcessResultRecorder(_paths(tmp_path))
    try:
        writer.check()
        writer.submit({"index": 0}, {"index": 0})
        with pytest.raises(OSError, match="serialization failed"):
            writer.submit({"cannot_pickle": lambda: None}, {})
    finally:
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["written_pairs"] == 1
    assert status["rejected_pairs"] == 1
    assert status["pending_pairs"] == status["failed_pairs"] == 0
    assert status["incomplete"]


def test_saturation_drains_all_accepted_pairs_and_timeout_retains_ownership(tmp_path):
    entered, release = _gates()
    writer = ProcessResultRecorder(_paths(tmp_path), capacity=1,
        _stream_factory=partial(_GatedStream, entered=entered, release=release))
    accepted = 0
    try:
        writer.check()
        writer.submit({"index": accepted}, {"index": accepted})
        accepted += 1
        assert entered.wait(2.)
        # Large bytes guarantee the sender blocks in its pipe once the child
        # is gated. The bounded queue then deterministically rejects a pair.
        for _ in range(4):
            try:
                writer.submit({"index": accepted, "padding": "x" * 1024 * 1024}, {"index": accepted})
            except OSError as exc:
                assert "queue full" in str(exc)
                break
            accepted += 1
        else:
            pytest.fail("bounded result queue did not saturate")
        status = writer.snapshot()
        assert status["accepted_pairs"] == accepted
        assert status["rejected_pairs"] == status["queue_high_water_pairs"] == 1
        assert not writer.close(0.)
        assert writer.worker.is_alive() and writer.process.is_alive()
        assert writer.snapshot()["incomplete"]
    finally:
        release.set()
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["written_pairs"] == accepted
    assert status["failed_pairs"] == status["pending_pairs"] == 0
    with pytest.raises(OSError, match="queue full"):
        writer.check()
    for path in _paths(tmp_path).values():
        assert [json.loads(line)["index"] for line in path.read_text().splitlines()] == list(range(accepted))


def test_disk_error_accounts_for_every_accepted_pair(tmp_path):
    entered, release = _gates()
    writer = ProcessResultRecorder(_paths(tmp_path),
        _stream_factory=partial(_GatedStream, entered=entered, release=release, fail_write=True))
    try:
        writer.check()
        writer.submit({"index": 0}, {"index": 0})
        assert entered.wait(2.)
        for index in range(1, 5):
            writer.submit({"index": index}, {"index": index})
    finally:
        release.set()
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["failed_pairs"] == 5
    assert status["written_pairs"] == status["pending_pairs"] == 0
    assert status["incomplete"]
    with pytest.raises(OSError, match="test disk full"):
        writer.check()


def test_second_file_write_failure_reports_partial_pair_as_incomplete(tmp_path):
    entered, release = _gates()
    writer = ProcessResultRecorder(_paths(tmp_path),
        _stream_factory=partial(_FailPoseStream, entered=entered, release=release))
    try:
        writer.check()
        writer.submit({"index": 0}, {"index": 0})
        assert entered.wait(2.)
        writer.submit({"index": 1}, {"index": 1})
    finally:
        release.set()
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["failed_pairs"] == 2
    assert status["written_pairs"] == status["pending_pairs"] == 0
    assert status["incomplete"] and not status["recording_complete"]
    assert len(_paths(tmp_path)["orientations"].read_text().splitlines()) == 1
    assert _paths(tmp_path)["poses"].read_text() == ""
    with pytest.raises(OSError, match="test pose disk full"):
        writer.check()


def test_force_flush_is_acknowledged_before_close(tmp_path):
    writer = ProcessResultRecorder(_paths(tmp_path), flush_interval_s=60.)
    try:
        writer.check()
        writer.submit({"index": 1}, {"index": 1}, flush=True)
        _wait(lambda: writer.snapshot()["flushed_pairs"] == 1)
        for path in _paths(tmp_path).values():
            assert json.loads(path.read_text()) == {"index": 1}
    finally:
        assert writer.close(3.)


def test_close_timeout_can_later_finish_without_killing_stream_owner(tmp_path):
    entered, release = _gates()
    writer = ProcessResultRecorder(_paths(tmp_path),
        _stream_factory=partial(_GatedStream, entered=entered, release=release))
    try:
        writer.check()
        writer.submit({}, {})
        assert entered.wait(2.)
        assert not writer.close(0.)
        status = writer.snapshot()
        assert status["incomplete"] and status["pending_pairs"] == 1
        assert status["error"] is None and status["writer_alive"]
    finally:
        release.set()
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["recording_complete"] and not status["incomplete"]
    assert status["accepted_pairs"] == status["written_pairs"] == 1


def test_close_error_keeps_written_pair_but_marks_recording_incomplete(tmp_path):
    entered, release = _gates()
    release.set()
    writer = ProcessResultRecorder(_paths(tmp_path),
        _stream_factory=partial(_GatedStream, entered=entered, release=release, fail_close=True))
    writer.check()
    writer.submit({}, {})
    assert writer.close(3.)
    status = writer.snapshot()
    assert status["written_pairs"] == 1 and status["failed_pairs"] == 0
    assert status["incomplete"] and not status["recording_complete"]
    with pytest.raises(OSError, match="test close failure"):
        writer.check()


def test_existing_file_is_not_clobbered_and_open_failure_is_visible(tmp_path):
    paths = _paths(tmp_path)
    paths["poses"].write_text("old pose\n")
    writer = ProcessResultRecorder(paths)
    try:
        with pytest.raises(OSError, match="FileExistsError"):
            writer.check()
    finally:
        assert writer.close(3.)
    assert paths["poses"].read_text() == "old pose\n"
    assert writer.snapshot()["incomplete"]


def test_startup_timeout_retains_worker_until_child_open_completes(tmp_path):
    entered, release = _gates()
    writer = ProcessResultRecorder(_paths(tmp_path), startup_timeout=.02,
        _stream_factory=partial(_gated_open, entered=entered, release=release))
    try:
        with pytest.raises(OSError, match="startup timed out"):
            writer.check()
        assert not writer.close(0.)
        assert writer.worker.is_alive()
        assert writer.snapshot()["incomplete"]
    finally:
        release.set()
        assert writer.close(3.)


def test_pre_ready_status_eof_retains_live_child_ownership(tmp_path, monkeypatch):
    entered, release = _gates()
    monkeypatch.setattr(live_recording_process, "_write_results", _status_eof_before_ready)
    writer = None
    try:
        writer = ProcessResultRecorder(_paths(tmp_path),
            _stream_factory=partial(_gated_open, entered=entered, release=release))
        assert entered.wait(2.)
        with pytest.raises(OSError, match="status channel closed"):
            writer.check()
        assert writer.worker.is_alive() and not writer.snapshot()["ready"]
        assert not writer.close(0.)
    finally:
        release.set()
        if writer is not None:
            assert writer.close(3.)
    assert writer.snapshot()["incomplete"]


def test_sender_start_failure_retains_recorder_for_cooperative_close(tmp_path, monkeypatch):
    def failed_start(_thread):
        raise RuntimeError("test sender thread unavailable")

    monkeypatch.setattr(live_recording_process.threading.Thread, "start", failed_start)
    writer = ProcessResultRecorder(_paths(tmp_path))
    try:
        with pytest.raises(OSError, match="test sender thread unavailable"):
            writer.check()
    finally:
        assert writer.close(3.)
    assert writer.snapshot()["incomplete"]
    assert not writer.worker.is_alive()


def test_crashed_child_and_broken_transport_never_report_complete(tmp_path):
    entered, release = _gates()
    writer = ProcessResultRecorder(_paths(tmp_path),
        _stream_factory=partial(_GatedStream, entered=entered, release=release))
    try:
        writer.check()
        writer.submit({"index": 0}, {"index": 0})
        assert entered.wait(2.)
        writer.submit({"padding": "x" * 1024 * 1024}, {})
        writer.submit({"index": 2}, {"index": 2})
        # This test terminates only the dedicated recorder child it created.
        writer.process.terminate()
        writer.process.join(2.)
        assert not writer.process.is_alive()
    finally:
        # The terminated waiter cannot acknowledge multiprocessing.Event's
        # notify handshake. Do not signal that test-only event after its death.
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["failed_pairs"] == 3
    assert status["pending_pairs"] == status["written_pairs"] == 0
    assert status["incomplete"] and not status["recording_complete"]
    assert not status["writer_alive"] and status["error"]
    with pytest.raises(OSError):
        writer.check()
    with pytest.raises(OSError):
        writer.submit({}, {})
    assert writer.snapshot()["rejected_pairs"] == 1


def test_crash_after_flushed_pairs_still_requires_clean_shutdown_ack(tmp_path):
    writer = ProcessResultRecorder(_paths(tmp_path))
    try:
        writer.check()
        writer.submit({}, {}, flush=True)
        _wait(lambda: writer.snapshot()["flushed_pairs"] == 1)
        writer.process.terminate()  # Only this test's dedicated recorder child.
        writer.process.join(2.)
        assert not writer.process.is_alive()
    finally:
        assert writer.close(3.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["written_pairs"] == status["flushed_pairs"] == 1
    assert status["pending_pairs"] == status["failed_pairs"] == 0
    assert status["incomplete"] and not status["recording_complete"]
    assert status["error"]


def _nested_recorder(root, result):
    # This mirrors the non-daemon compute-child -> writer-grandchild topology.
    writer = ProcessResultRecorder(_paths(Path(root)))
    try:
        writer.check()
        writer.submit({"frame": 1}, {"pose": 1})
    finally:
        closed = writer.close(3.)
    import sys
    result.send((closed, writer.snapshot(), "serial" in sys.modules,
                 "vt_fsglove12.live_stream" in sys.modules))
    result.close()


def test_non_daemon_compute_child_can_spawn_recorder_without_device_import(tmp_path):
    context = mp.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    process = context.Process(target=_nested_recorder, args=(str(tmp_path), sender), daemon=False)
    process.start()
    sender.close()
    try:
        assert receiver.poll(5.)
        closed, status, serial_imported, source_imported = receiver.recv()
        assert closed and status["recording_complete"]
        assert not serial_imported and not source_imported
    finally:
        receiver.close()
        process.join(5.)
        assert not process.is_alive()
        assert process.exitcode == 0


@pytest.mark.parametrize("capacity", [0, -1, 1.5, True])
def test_invalid_capacity_is_rejected_without_creating_files(tmp_path, capacity):
    with pytest.raises(ValueError, match="positive integer"):
        ProcessResultRecorder(_paths(tmp_path), capacity=capacity)
    assert list(tmp_path.iterdir()) == []
