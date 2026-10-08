"""Stopping local logs must not wait on their disk owner or falsify completion."""
from functools import partial
import json
import multiprocessing as mp
import threading
import time

import pytest

from test_live_recording_process import _GatedStream, _paths, _wait
from vt_fsglove12.live_recording import AsyncResultRecorder
from vt_fsglove12.live_recording_process import ProcessResultRecorder


@pytest.fixture(params=["async", "process"])
def gated_writer(request, tmp_path):
    kind = request.param
    context = mp.get_context("spawn") if kind == "process" else threading
    entered, release = context.Event(), context.Event()
    made = []

    def create(**stream_options):
        factory = partial(_GatedStream, entered=entered, release=release, **stream_options)
        if kind == "process":
            recorder = ProcessResultRecorder(_paths(tmp_path), _stream_factory=factory)
        else:
            recorder = AsyncResultRecorder({name: factory(path) for name, path in _paths(tmp_path).items()})
        made.append(recorder)
        recorder.check()
        return recorder, entered, release, _paths(tmp_path)

    yield create
    release.set()
    for recorder in made:
        assert recorder.close(3.)


def _start_blocked(create, **stream_options):
    recorder, entered, release, paths = create(**stream_options)
    recorder.submit({"index": 0}, {"index": 0})
    assert entered.wait(2.)
    for index in range(1, 6):
        recorder.submit({"index": index}, {"index": index})
    return recorder, release, paths


def _accounted(snapshot):
    assert snapshot["accepted_pairs"] == (snapshot["written_pairs"]
        + snapshot["failed_pairs"] + snapshot["pending_pairs"])


def test_request_stop_is_nonblocking_idempotent_and_drains_all_accepted(gated_writer):
    recorder, release, paths = _start_blocked(gated_writer)
    start = time.perf_counter()
    recorder.request_stop()
    recorder.request_stop()
    assert time.perf_counter() - start < .2
    pending = recorder.snapshot()
    _accounted(pending)
    assert pending["writer_alive"] and pending["stop_requested"]
    assert pending["status"] == "draining"
    assert not pending["abort_requested"] and pending["error"] is None
    assert not pending["recording_complete"] and not pending["incomplete"]
    with pytest.raises(OSError, match="closing"):
        recorder.submit({"index": 99}, {})
    release.set()
    recorder.worker.join(3.)
    final = recorder.snapshot()
    _accounted(final)
    assert not final["writer_alive"] and final["recording_complete"]
    assert final["status"] == "complete" and not final["incomplete"]
    assert final["accepted_pairs"] == final["written_pairs"] == 6
    assert final["failed_pairs"] == final["pending_pairs"] == 0
    for path in paths.values():
        assert [json.loads(line)["index"] for line in path.read_text().splitlines()] == list(range(6))


def test_abort_returns_while_disk_is_blocked_and_never_claims_complete(gated_writer):
    recorder, release, _ = _start_blocked(gated_writer)
    start = time.perf_counter()
    recorder.abort("storage guard disabled local logging")
    assert time.perf_counter() - start < .2
    assert recorder.worker.is_alive()
    pending = recorder.snapshot()
    _accounted(pending)
    assert pending["abort_requested"] and pending["stop_requested"]
    assert pending["incomplete"] and pending["status"] == "incomplete"
    with pytest.raises(OSError, match="storage guard disabled local logging"):
        recorder.check()
    with pytest.raises(OSError, match="storage guard disabled local logging"):
        recorder.submit({}, {})
    recorder.abort("secondary reason must not overwrite the original")
    recorder.request_stop()
    release.set()
    recorder.worker.join(3.)
    final = recorder.snapshot()
    _accounted(final)
    assert not final["writer_alive"]
    assert final["written_pairs"] == final["accepted_pairs"] == 6  # finite cooperative drain
    assert final["pending_pairs"] == final["failed_pairs"] == 0
    assert final["incomplete"] and not final["recording_complete"]
    assert final["error"] == pending["error"]


def test_original_disk_error_survives_abort_and_every_pair_is_accounted(gated_writer):
    recorder, release, _ = _start_blocked(gated_writer, fail_write=True)
    release.set()
    # The process owner polls the child status pipe via its public snapshot.
    _wait(lambda: not recorder.snapshot()["writer_alive"])
    original = recorder.snapshot()["error"]
    assert "test disk full" in original
    recorder.abort("runtime is isolating the local recorder")
    status = recorder.snapshot()
    _accounted(status)
    assert status["error"] == original
    assert status["accepted_pairs"] == status["failed_pairs"] == 6
    assert status["written_pairs"] == status["pending_pairs"] == 0
    assert status["incomplete"] and not status["recording_complete"]
    with pytest.raises(OSError, match="test disk full"):
        recorder.check()


def test_close_error_after_requested_drain_is_not_complete(gated_writer):
    recorder, entered, release, _ = gated_writer(fail_close=True)
    release.set()
    recorder.submit({"i": 1}, {"i": 1})
    recorder.request_stop()
    recorder.worker.join(3.)
    status = recorder.snapshot()
    _accounted(status)
    assert not status["writer_alive"]
    assert status["accepted_pairs"] == status["written_pairs"] == 1
    assert status["incomplete"] and not status["recording_complete"]
    assert "test close failure" in status["error"]


def test_compact_json_retains_entire_replay_structure(gated_writer):
    recorder, _, release, paths = gated_writer()
    release.set()
    frame = dict(nodes={"12": {"q": [0., 0., 0., 1.], "nested": {"source": "完整 data"}}},
                 timestamp=1.25, optional=None)
    pose = dict(joints=[[1, 2, 3]], diagnostic={"quality": "degraded", "available": True})
    recorder.submit(frame, pose)
    recorder.request_stop()
    recorder.worker.join(3.)
    for name, expected in (("orientations", frame), ("poses", pose)):
        content = paths[name].read_text()
        assert json.loads(content) == expected
        assert content == json.dumps(expected, ensure_ascii=False, allow_nan=False,
                                     separators=(",", ":")) + "\n"


@pytest.mark.parametrize("reason", [None, "", "   ", 3])
def test_invalid_abort_reason_does_not_stop_healthy_writer(gated_writer, reason):
    recorder, _, release, _ = gated_writer()
    with pytest.raises(ValueError, match="nonempty reason"):
        recorder.abort(reason)
    assert not recorder.snapshot()["stop_requested"]
    assert recorder.snapshot()["error"] is None
    release.set()
