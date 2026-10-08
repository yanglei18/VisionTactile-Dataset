"""Guided capture integration; synthetic input is not a hardware acceptance."""
import json
import threading
import time

import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose.thumb_preview_live import OrientationWorker


class FakeSource:
    def __init__(self, port): self.port=port;self.closed=False
    def open(self): return self
    def close(self): self.closed=True


def feed(worker, stamp):
    worker.feed_samples([Sample(node,(0,0,0,0,0,8197),stamp,1000+stamp) for node in range(1,13)])


def ready_worker():
    now=[100.]
    worker=OrientationWorker(load_config(),source_factory=FakeSource,clock=lambda:now[0])
    worker.command("connect",{"port":"test"})
    feed(worker,now[0])
    worker.command("reference",{})
    return worker,now


def finish_save(worker):
    worker.save_pool.shutdown(wait=True)
    worker.poll_trial_save()


def test_guided_capture_is_manual_preserves_reference_and_saves_complete_data(tmp_path):
    worker,now=ready_worker()
    reference=worker.processor.snapshot()["reference_host_s"]
    assert list(tmp_path.iterdir())==[]
    worker.command("start_trial",{"output_root":str(tmp_path)})
    folder=worker.trial.directory
    try:
        with pytest.raises(ValueError,match="测试期间"):
            worker.command("reference",{})
        for i in range(1,3722):
            now[0]=100+i/120
            feed(worker,now[0])
            if i%4==0:worker.publish(now[0])
        worker.publish(now[0])
        finish_save(worker)
        assert worker.last_trial["status"]=="completed"
        assert worker.processor.snapshot()["reference_host_s"]==reference
        assert worker.source is not None and not worker.source.closed
        assert (folder/"raw.csv").exists()
        assert (folder/"online.jsonl").exists()
        assert (folder/"report.json").exists()
        assert (folder/"manifest.json").exists()
        first=json.loads((folder/"online.jsonl").read_text().splitlines()[0])
        assert first["pair_count"]==2
    finally:
        worker.disconnect()
        worker.save_pool.shutdown(wait=True)


def test_disconnect_retains_partial_capture_and_releases_port(tmp_path):
    worker,now=ready_worker()
    source=worker.source
    worker.command("start_trial",{"output_root":str(tmp_path)})
    folder=worker.trial.directory
    now[0]+=.01;feed(worker,now[0])
    worker.command("disconnect",{})
    assert source.closed and worker.source is None
    finish_save(worker)
    assert worker.last_trial["status"]!="completed"
    assert (folder/"raw.csv").exists()
    assert (folder/"manifest.json").exists()


def test_data_gap_aborts_trial_without_reopening_or_rezeroing(tmp_path):
    worker,now=ready_worker()
    worker.command("start_trial",{"output_root":str(tmp_path)})
    now[0]+=1.
    worker.publish(now[0])
    assert worker.trial is None and worker.source is not None
    finish_save(worker)
    assert worker.last_trial["status"]!="completed"
    assert not worker.processor.snapshot()["reference_ready"]
    worker.disconnect()


def test_background_save_does_not_block_feed_or_change_reference(tmp_path):
    worker,now=ready_worker()
    entered,released=threading.Event(),threading.Event()
    class SlowTrial:
        directory=tmp_path
        def finish(self,now_s,reason):
            entered.set();released.wait(3)
            return dict(status="completed",directory=str(tmp_path),summary="saved",export_complete=True)
    worker.trial=SlowTrial()
    worker.finish_trial()
    try:
        assert entered.wait(1)
        before=worker.processor.pair_count
        for i in range(10):
            now[0]+=.01;feed(worker,now[0])
        assert worker.processor.pair_count==before+10
        assert worker.processor.snapshot()["reference_ready"]
        assert worker.source is not None and not worker.source.closed
    finally:
        released.set();finish_save(worker);worker.disconnect()


def test_write_failure_keeps_preview_running_and_reports_save_failure(tmp_path):
    worker,now=ready_worker()
    class FailingTrial:
        directory=tmp_path
        def feed_raw(self,samples): raise OSError("storage unavailable")
        def finish(self,now_s,reason): raise OSError("cannot finish report")
    worker.trial=FailingTrial()
    before=worker.processor.pair_count
    now[0]+=.01;feed(worker,now[0])
    finish_save(worker)
    assert worker.processor.pair_count==before+1
    assert worker.source is not None
    assert worker.last_trial["status"]=="save_failed"
    assert worker.last_trial["directory"]==str(tmp_path)
    worker.disconnect()


def test_port_close_error_does_not_lose_pending_trial_save(tmp_path):
    worker,now=ready_worker()
    worker.command("start_trial",{"output_root":str(tmp_path)})
    folder=worker.trial.directory
    now[0]+=.01;feed(worker,now[0])
    def fail_close(): raise OSError("disconnected device")
    worker.source.close=fail_close
    worker.disconnect()
    finish_save(worker)
    assert worker.source is None
    assert worker.last_trial["export_complete"] is True
    assert worker.last_trial["status"]!="completed"
    assert (folder/"manifest.json").exists()
