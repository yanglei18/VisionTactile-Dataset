"""Real recorder failure contracts; only temporary files and fake sources."""

from concurrent.futures import Future
import sys
from types import SimpleNamespace

import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose.thumb_preview_live import OrientationWorker
from vt_glove_pose import thumb_trial


class FakeSource:
    def __init__(self, port):
        self.port, self.closed = port, False

    def open(self):
        return self

    def close(self):
        self.closed = True


def _frame(stamp):
    return [Sample(node, (0, 0, 0, 0, 0, 8197), stamp, 1000+stamp) for node in range(1, 13)]


def _ready_trial(tmp_path):
    now = [100.]
    worker = OrientationWorker(load_config(), source_factory=FakeSource, clock=lambda: now[0])
    worker.command("connect", {"port": "fake-no-hardware"})
    worker.feed_samples(_frame(now[0]))
    worker.command("reference", {})
    reference = worker.processor.snapshot()["reference_host_s"]
    worker.command("start_trial", {"output_root": str(tmp_path)})
    now[0] += .01
    worker.feed_samples(_frame(now[0]))
    return worker, now, reference


def _analysis_stub(monkeypatch, *, fail=False):
    def analyze(*args, **kwargs):
        if fail:
            raise RuntimeError("analysis unavailable")
        return {"summary": "test fixture, not product accuracy"}
    monkeypatch.setitem(sys.modules, "vt_glove_pose.thumb_trial_analysis", SimpleNamespace(analyze_trial=analyze))


@pytest.mark.parametrize("failure", ["analysis", "report", "manifest", "hash", "close"])
def test_real_finish_failure_never_reports_saved_and_keeps_raw_reference_and_stream(tmp_path, monkeypatch, failure):
    _analysis_stub(monkeypatch, fail=failure == "analysis")
    worker, now, reference = _ready_trial(tmp_path)
    trial = worker.trial
    folder = trial.directory
    original_raw = (folder/"raw.csv").read_bytes()
    original_online = (folder/"online.jsonl").read_bytes()
    assert original_raw and original_online
    source = worker.source
    if failure in {"report", "manifest"}:
        real_write = thumb_trial.write_json_exclusive
        def write(path, data):
            if path.name == failure+".json":
                raise OSError("simulated storage full")
            return real_write(path, data)
        monkeypatch.setattr(thumb_trial, "write_json_exclusive", write)
    elif failure == "hash":
        real_hash = thumb_trial._file_info
        def file_info(path):
            if path.name == "raw.csv":
                raise OSError("simulated hash read failure")
            return real_hash(path)
        monkeypatch.setattr(thumb_trial, "_file_info", file_info)
    elif failure == "close":
        real_close = trial._raw.close
        def close():
            real_close()
            raise OSError("simulated close failure")
        monkeypatch.setattr(trial._raw, "close", close)
    try:
        now[0] = 131.
        worker.finish_trial("completed")
        worker.save_pool.shutdown(wait=True)
        worker.poll_trial_save()
        result = worker.last_trial
        assert result["status"] == "save_failed"
        assert result["export_complete"] is False
        assert result["errors"]
        assert result["directory"] == str(folder)
        assert "保存未完成" in worker.error
        assert "测试数据已保存" not in worker.notice
        assert (folder/"raw.csv").read_bytes() == original_raw
        assert (folder/"online.jsonl").read_bytes() == original_online
        assert worker.source is source and not source.closed
        # Do not call snapshot(now=131): that would deliberately expire a
        # fixture with no intervening samples, unrelated to file I/O behavior.
        assert worker.processor.snapshot()["reference_host_s"] == reference
    finally:
        worker.disconnect()
        worker.save_pool.shutdown(wait=True)


def test_returned_completed_with_null_report_and_errors_exposes_actual_storage_error():
    worker = OrientationWorker(load_config())
    result = dict(status="completed", directory="/fake/no-files", export_complete=False,
                  errors=["analysis_or_report_error: storage unavailable"], report=None)
    future = Future()
    future.set_result(result)
    worker.trial_future = future
    try:
        worker.poll_trial_save()
        assert worker.last_trial["status"] == "save_failed"
        assert worker.last_trial["directory"] == "/fake/no-files"
        assert "storage unavailable" in worker.error
        assert "NoneType" not in worker.error
    finally:
        worker.save_pool.shutdown(wait=True)


def test_successful_real_finish_preserves_files_and_copies_analysis_summary(tmp_path, monkeypatch):
    _analysis_stub(monkeypatch)
    worker, now, reference = _ready_trial(tmp_path)
    folder = worker.trial.directory
    try:
        now[0] = 131.
        worker.finish_trial("completed")
        worker.save_pool.shutdown(wait=True)
        worker.poll_trial_save()
        assert worker.last_trial["status"] == "completed"
        assert worker.last_trial["export_complete"] is True
        assert worker.last_trial["errors"] == []
        assert worker.last_trial["summary"] == "test fixture, not product accuracy"
        assert worker.processor.snapshot()["reference_host_s"] == reference
        assert all((folder/name).is_file() for name in ("raw.csv", "online.jsonl", "report.json", "manifest.json"))
    finally:
        worker.disconnect()
        worker.save_pool.shutdown(wait=True)
