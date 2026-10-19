"""Both decoder adapters obey cancellation, correlation and ownership rules."""
import json
import subprocess
import sys
import threading
import time

import pytest

from vt_capture_console.backend.scene_demo import demo_scene
from vt_capture_console.backend.spatial_replay import BagSubprocessSource, SpatialReplayBackend
from vt_capture_console.backend.topic_replay import TopicBagSource


@pytest.fixture
def decoder(monkeypatch):
    processes = []
    popen = subprocess.Popen

    def install(script):
        def launch(*args, **kwargs):
            process = popen([sys.executable, "-u", "-c", script], **kwargs)
            processes.append(process)
            return process
        monkeypatch.setattr(subprocess, "Popen", launch)
        return processes

    yield install
    for process in processes:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=2)


def metadata(kind):
    if kind == "spatial":
        return dict(frame_count=3, note="synthetic")
    return dict(start_ns=0, end_ns=3, topics=[
        dict(name="/x", type="custom", count=3, kind="message")], warnings=[])


def source_type(kind):
    return BagSubprocessSource if kind == "spatial" else TopicBagSource


def frame(source, kind, index):
    return source.frame(index) if kind == "spatial" else source.frame("/x", index)


@pytest.mark.parametrize("kind", ["spatial", "topic"])
@pytest.mark.parametrize("late_ok", [True, False])
@pytest.mark.parametrize("interruption", ["cancel", "timeout"])
def test_late_reply_cannot_be_published_as_new_selection(decoder, monkeypatch, kind, late_ok, interruption):
    script = "\n".join([
        "import json,sys",
        "print(" + repr(json.dumps(dict(ok=True, metadata=metadata(kind)))) + ",flush=True)",
        "scene=" + repr(demo_scene(0, sequence=1)),
        "first=None",
        "def reply(request, ok=True):",
        "    index=request.get('index',request.get('time_ns'))",
        "    scene['scene_time_ns']=index",
        "    print(json.dumps(dict(ok=ok,error='old request failed',request_id=request.get('request_id'),index=index,",
        "        scene=scene,clouds=[],frame=dict(topic='/x',time_ns=index))),flush=True)",
        "for line in sys.stdin:",
        "    request=json.loads(line)",
        "    if first is None: first=request; continue",
        "    reply(first," + repr(late_ok) + ")",
        "    reply(request)",
    ])
    decoder(script)
    cancelled = threading.Event()
    source = source_type(kind)("unused", cancelled.is_set)
    timer = threading.Timer(.05, cancelled.set)
    try:
        if interruption == "timeout":
            from vt_capture_console.backend import replay_transport
            monkeypatch.setattr(replay_transport, "FRAME_TIMEOUT_S", .05)
        else:
            timer.start()
        with pytest.raises(TimeoutError if interruption == "timeout" else InterruptedError):
            frame(source, kind, 1)
        cancelled.clear()
        result = frame(source, kind, 2)
        assert result["scene_time_ns" if kind == "spatial" else "time_ns"] == 2
    finally:
        timer.cancel()
        source.close()


@pytest.mark.parametrize("kind", ["spatial", "topic"])
def test_invalid_initial_metadata_reaps_decoder(decoder, kind):
    processes = decoder("import json,time; print(json.dumps(dict(ok=True,metadata={})),flush=True); time.sleep(10)")
    with pytest.raises(ValueError):
        source_type(kind)("unused")
    assert processes[0].wait(timeout=2) is not None


@pytest.mark.parametrize("kind", ["spatial", "topic"])
@pytest.mark.parametrize("interruption", ["cancel", "timeout"])
def test_interrupted_initial_metadata_reaps_decoder(decoder, monkeypatch, kind, interruption):
    from vt_capture_console.backend import replay_transport
    processes = decoder("import time; time.sleep(10)")
    cancelled = threading.Event()
    timer = threading.Timer(.03, cancelled.set)
    if interruption == "timeout":
        monkeypatch.setattr(replay_transport, "METADATA_TIMEOUT_S", .03)
    else:
        timer.start()
    try:
        with pytest.raises(TimeoutError if interruption == "timeout" else InterruptedError):
            source_type(kind)("unused", cancelled.is_set)
        assert processes[0].wait(timeout=2) is not None
    finally:
        timer.cancel()


def test_first_spatial_frame_failure_reaps_decoder_before_backend_close(decoder):
    script = "\n".join([
        "import json,sys",
        "print(" + repr(json.dumps(dict(ok=True, metadata=metadata("spatial")))) + ",flush=True)",
        "for line in sys.stdin:",
        "    request=json.loads(line)",
        "    print(json.dumps(dict(ok=False,request_id=request['request_id'],error='bad initial frame')),flush=True)",
    ])
    processes = decoder(script)
    backend = SpatialReplayBackend(loader=BagSubprocessSource)
    try:
        backend.load("unused")
        deadline = time.monotonic() + 2
        while backend.status()["state"] != "error" and time.monotonic() < deadline:
            time.sleep(.005)
        assert backend.status()["state"] == "error"
        assert "bad initial frame" in backend.status()["message"]
        assert processes[0].wait(timeout=2) is not None
    finally:
        backend.close()


@pytest.mark.parametrize("kind", ["spatial", "topic"])
@pytest.mark.parametrize("bad_id", [None, True, 0, 2])
def test_missing_invalid_or_future_reply_identifier_is_rejected(decoder, kind, bad_id):
    script = "\n".join([
        "import json,sys",
        "print(" + repr(json.dumps(dict(ok=True, metadata=metadata(kind)))) + ",flush=True)",
        "for line in sys.stdin:",
        "    print(" + repr(json.dumps(dict(ok=True, request_id=bad_id))) + ",flush=True)",
    ])
    decoder(script)
    source = source_type(kind)("unused")
    try:
        with pytest.raises(ValueError, match="request identifier"):
            frame(source, kind, 1)
    finally:
        source.close()


def test_shared_transport_keeps_existing_topic_request_size_support(decoder):
    script = "\n".join([
        "import json,sys",
        "print(" + repr(json.dumps(dict(ok=True, metadata=metadata("topic")))) + ",flush=True)",
        "for line in sys.stdin:",
        "    request=json.loads(line)",
        "    print(json.dumps(dict(ok=True,request_id=request['request_id'],",
        "        frame=dict(topic=request['topic'],time_ns=request['time_ns']))),flush=True)",
    ])
    decoder(script)
    source = TopicBagSource("unused")
    topic = "/" + "设备" * 500  # Within the existing 1024-character inventory bound.
    try:
        assert source.frame(topic, 1)["topic"] == topic
    finally:
        source.close()


def test_cancelled_partial_write_invalidates_decoder_instead_of_reusing_broken_json(decoder, monkeypatch):
    from vt_capture_console.backend import replay_transport
    processes = decoder("import json,time; print(json.dumps(dict(ok=True,metadata={})),flush=True); time.sleep(10)")
    cancelled = threading.Event()
    transport = replay_transport.ReplayDecoder("topic_replay_worker", "unused", cancelled.is_set)
    transport.metadata()
    original_write = replay_transport.os.write

    def partial_write(fd, data):
        written = original_write(fd, data[:10])
        cancelled.set()
        return written

    monkeypatch.setattr(replay_transport.os, "write", partial_write)
    try:
        with pytest.raises(InterruptedError):
            transport.request(dict(command="frame", topic="/x", time_ns=1))
        assert processes[0].wait(timeout=.5) is not None
    finally:
        transport.close()


def test_draining_late_replies_does_not_extend_the_current_request_deadline(decoder, monkeypatch):
    from vt_capture_console.backend import replay_transport
    script = "\n".join([
        "import json,sys,time",
        "print(json.dumps(dict(ok=True,metadata={})),flush=True)",
        "for line in sys.stdin:",
        "    request=json.loads(line)",
        "    if request['request_id']==1: continue",
        "    deadline=time.monotonic()+1",
        "    while time.monotonic()<deadline:",
        "        print(json.dumps(dict(ok=True,request_id=1)),flush=True)",
    ])
    decoder(script)
    transport = replay_transport.ReplayDecoder("topic_replay_worker", "unused")
    transport.metadata()
    monkeypatch.setattr(replay_transport, "FRAME_TIMEOUT_S", .03)
    try:
        with pytest.raises(TimeoutError):
            transport.request(dict(command="frame", topic="/x", time_ns=1))
        started = time.monotonic()
        with pytest.raises(TimeoutError):
            transport.request(dict(command="frame", topic="/x", time_ns=2))
        assert time.monotonic() - started < .5
    finally:
        transport.close()
