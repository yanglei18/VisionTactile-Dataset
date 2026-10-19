"""Partial recordings remain playable without a live manager or scene geometry."""
import json
import threading
import time
import subprocess
import sys

import pytest

from vt_capture_console.backend.topic_replay import TopicReplayBackend, validate_metadata


def ready(backend, predicate=lambda: True):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if backend.status()["state"] == "ready" and predicate():
            return backend.latest()
        time.sleep(.005)
    pytest.fail(str(backend.status()))


class Source:
    def __init__(self, path, cancelled):
        self.closed = False
        self.metadata = dict(path=str(path), start_ns=1_000_000_000, end_ns=2_000_000_000,
            topics=[dict(name="/tracker", type="Tracker", count=0, kind="tracker"),
                    dict(name="/rgb", type="Image", count=3, kind="camera"),
                    dict(name="/custom", type="Custom", count=1, kind="message")],
            warnings=["/tracker: no recorded messages"])

    def frame(self, topic, time_ns):
        return dict(topic=topic, time_ns=time_ns, sample_ns=time_ns,
                    device=None, details={"value": 42}, warnings=[])

    def close(self):
        self.closed = True


def test_camera_only_recording_selects_nonempty_topic_and_keeps_warnings():
    backend = TopicReplayBackend(loader=Source)
    try:
        backend.load("camera-bag")
        frame = ready(backend)
        assert frame["topic"] == "/rgb"
        assert backend.metadata()["warnings"]
        backend.seek(1_500_000_000)
        ready(backend, lambda: backend.timeline()["time_ns"] == 1_500_000_000)
        backend.select_topic("/custom")
        frame = ready(backend, lambda: backend.latest()["topic"] == "/custom")
        assert frame["time_ns"] == 1_500_000_000
        assert frame["details"]["value"] == 42
    finally:
        backend.close()


def test_play_pause_end_and_seek_replay_use_bag_time():
    backend = TopicReplayBackend(loader=Source)
    try:
        backend.load("partial-bag")
        ready(backend)
        backend.seek(1_900_000_000)
        ready(backend, lambda: backend.timeline()["time_ns"] == 1_900_000_000)
        backend.set_rate(4.)
        backend.play()
        ready(backend, lambda: backend.timeline()["time_ns"] == 2_000_000_000)
        assert not backend.timeline()["playing"]
        backend.play()
        ready(backend, lambda: backend.timeline()["time_ns"] < 2_000_000_000)
        backend.pause()
        assert not backend.timeline()["playing"]
        with pytest.raises(ValueError):
            backend.set_rate(0.)
    finally:
        backend.close()


def test_replaced_load_is_cancelled_and_owned_sources_are_closed():
    started, release = threading.Event(), threading.Event()
    sources = []

    def loader(path, cancelled):
        source = Source(path, cancelled)
        sources.append(source)
        if path == "old":
            started.set()
            release.wait(2)
            assert cancelled()
        return source

    backend = TopicReplayBackend(loader=loader)
    try:
        backend.load("old")
        assert started.wait(1)
        backend.load("new")
        release.set()
        ready(backend, lambda: backend.timeline()["path"] == "new")
        assert sources[0].closed
    finally:
        backend.close()
    assert all(source.closed for source in sources)


def test_frame_failure_does_not_prevent_selecting_other_topic():
    class PartialSource(Source):
        def frame(self, topic, time_ns):
            if topic == "/custom":
                raise ValueError("broken topic")
            return super().frame(topic, time_ns)

    backend = TopicReplayBackend(loader=PartialSource)
    try:
        backend.load("partial")
        ready(backend)
        backend.select_topic("/custom")
        frame = ready(backend, lambda: backend.latest()["topic"] == "/custom")
        assert "broken topic" in frame["warnings"]
        assert backend.snapshot()["frames"]["/rgb"]["sample_ns"] is not None
        backend.select_topic("/rgb")
        assert ready(backend)["topic"] == "/rgb"
    finally:
        backend.close()


@pytest.mark.parametrize("initial_frame", [False, True])
def test_failed_obsolete_frame_does_not_discard_new_topic_selection(initial_frame):
    started, release = threading.Event(), threading.Event()
    bad_topic = "/rgb" if initial_frame else "/custom"
    good_topic = "/custom" if initial_frame else "/rgb"

    class SlowFailureSource(Source):
        def frame(self, topic, time_ns):
            if topic == bad_topic:
                started.set()
                assert release.wait(2), "test did not release the obsolete frame"
                raise ValueError("obsolete topic failed")
            return super().frame(topic, time_ns)

    backend = TopicReplayBackend(loader=SlowFailureSource)
    try:
        backend.load("partial")
        if not initial_frame:
            ready(backend)
            backend.select_topic(bad_topic)
        assert started.wait(1)
        backend.select_topic(good_topic)
        release.set()
        assert ready(backend)["topic"] == good_topic
    finally:
        release.set()
        backend.close()


def test_metadata_rejects_unbounded_or_ambiguous_inventory():
    metadata = Source("x", None).metadata
    validate_metadata(metadata)
    metadata["topics"].append(dict(metadata["topics"][0]))
    with pytest.raises(ValueError):
        validate_metadata(metadata)


def test_clear_removes_previous_trial_and_closes_decoder():
    sources = []
    def loader(path, cancelled):
        source = Source(path, cancelled)
        sources.append(source)
        return source
    backend = TopicReplayBackend(loader=loader)
    try:
        backend.load("old-trial")
        ready(backend)
        backend.clear()
        assert backend.latest() is None
        assert not backend.metadata()
        assert backend.status()["state"] == "waiting"
        deadline = time.monotonic() + 1
        while not sources[0].closed and time.monotonic() < deadline:
            time.sleep(.005)
        assert sources[0].closed
    finally:
        backend.close()


def test_decoder_errors_are_visible_and_process_is_reaped(monkeypatch):
    from vt_capture_console.backend import topic_replay
    popen = subprocess.Popen
    processes = []
    def failed_decoder(*args, **kwargs):
        process = popen([sys.executable, "-c",
                        "import sys; print('missing message package', file=sys.stderr); sys.exit(2)"],
                       **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(subprocess, "Popen", failed_decoder)
    with pytest.raises(RuntimeError, match="missing message package"):
        topic_replay.TopicBagSource("unused")
    assert processes[0].wait(timeout=1) == 2


def test_decoder_protocol_delivers_requested_topic_and_time(monkeypatch):
    from vt_capture_console.backend import topic_replay
    popen = subprocess.Popen
    metadata = Source("fake", None).metadata
    script = "\n".join([
        "import json, sys",
        "print(" + repr(json.dumps({"ok": True, "metadata": metadata})) + ", flush=True)",
        "for line in sys.stdin:",
        "    request=json.loads(line)",
        "    frame=dict(topic=request['topic'],time_ns=request['time_ns'],sample_ns=None,device=None,details={},warnings=['empty'])",
        "    print(json.dumps(dict(ok=True,frame=frame,request_id=request['request_id'])),flush=True)",
    ])
    processes = []
    def decoder(*args, **kwargs):
        process = popen([sys.executable, "-c", script], **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(subprocess, "Popen", decoder)
    source = topic_replay.TopicBagSource("unused")
    try:
        assert source.frame("/tracker", 1_500_000_000)["warnings"] == ["empty"]
    finally:
        source.close()
    assert processes[0].wait(timeout=2) is not None


class MultiSource(Source):
    def __init__(self, path, cancelled):
        super().__init__(path, cancelled)
        self.path = path
        self.calls = []
        self.metadata['topics'] = [
            dict(name='/rgb', type='Image', count=3, kind='camera'),
            dict(name='/depth', type='Image', count=3, kind='camera'),
            dict(name='/tracker', type='Tracker', count=2, kind='tracker'),
            dict(name='/custom', type='Custom', count=1, kind='message')]

    def frame(self, topic, time_ns):
        self.calls.append((topic, time_ns))
        return dict(topic=topic, time_ns=time_ns, sample_ns=time_ns-(20 if topic == '/tracker' else 10),
                    device=None, details={'path': self.path}, warnings=[])


def test_all_visual_topics_share_a_playhead_and_retain_individual_sample_times():
    backend = TopicReplayBackend(loader=MultiSource)
    try:
        backend.load('multi')
        ready(backend)
        backend.seek(1_500_000_000)
        ready(backend)
        snapshot = backend.snapshot()
        assert set(snapshot['frames']) == {'/rgb', '/depth', '/tracker'}
        assert {frame['time_ns'] for frame in snapshot['frames'].values()} == {snapshot['time_ns']}
        assert snapshot['time_ns'] == 1_500_000_000
        assert snapshot['frames']['/tracker']['sample_ns'] != snapshot['frames']['/rgb']['sample_ns']
        snapshot['frames']['/rgb']['details']['path'] = 'mutated consumer copy'
        assert backend.snapshot()['frames']['/rgb']['details']['path'] == 'multi'
    finally:
        backend.close()


def test_one_visual_decode_failure_does_not_stop_other_frames_or_playback():
    class BrokenDepth(MultiSource):
        def frame(self, topic, stamp):
            if topic == '/depth':
                raise ValueError('depth unavailable')
            return super().frame(topic, stamp)
    backend = TopicReplayBackend(loader=BrokenDepth)
    try:
        backend.load('partial')
        ready(backend)
        snapshot = backend.snapshot()
        assert snapshot['frames']['/depth']['error'] == 'depth unavailable'
        assert snapshot['frames']['/rgb']['sample_ns'] is not None
        backend.seek(1_950_000_000)
        ready(backend)
        backend.set_rate(4.)
        backend.play()
        ready(backend, lambda: backend.snapshot()['time_ns'] == 2_000_000_000)
        assert backend.snapshot()['frames']['/rgb']['sample_ns'] is not None
    finally:
        backend.close()


def test_selecting_message_topic_keeps_existing_visual_snapshot_and_seek_revision():
    started, release = threading.Event(), threading.Event()
    sources = []
    class SlowDetails(MultiSource):
        def __init__(self, path, cancelled):
            super().__init__(path, cancelled)
            sources.append(self)
        def frame(self, topic, stamp):
            if topic == '/custom':
                started.set()
                assert release.wait(2)
            return super().frame(topic, stamp)
    backend = TopicReplayBackend(loader=SlowDetails)
    try:
        backend.load('multi')
        ready(backend)
        before = backend.snapshot()
        calls = len(sources[0].calls)
        backend.select_topic('/custom')
        assert started.wait(1)
        assert backend.snapshot() == before
        release.set()
        assert ready(backend)['topic'] == '/custom'
        after = backend.snapshot()
        assert after['frames'] == before['frames']
        assert after['seek_revision'] == before['seek_revision']
        assert len(sources[0].calls) == calls + 1
    finally:
        release.set()
        backend.close()


def test_seek_during_multi_frame_build_never_publishes_mixed_or_obsolete_time():
    started, release = threading.Event(), threading.Event()
    class SlowDepth(MultiSource):
        def frame(self, topic, stamp):
            if topic == '/depth' and stamp == 1_100_000_000:
                started.set()
                assert release.wait(2)
            return super().frame(topic, stamp)
    backend = TopicReplayBackend(loader=SlowDepth)
    try:
        backend.load('multi')
        ready(backend)
        previous_seek = backend.snapshot()['seek_revision']
        backend.seek(1_100_000_000)
        assert started.wait(1)
        backend.seek(1_900_000_000)
        assert backend.snapshot()['frames'] == {}
        release.set()
        ready(backend)
        snapshot = backend.snapshot()
        assert snapshot['time_ns'] == 1_900_000_000
        assert snapshot['seek_revision'] == previous_seek + 2
        assert {frame['time_ns'] for frame in snapshot['frames'].values()} == {1_900_000_000}
    finally:
        release.set()
        backend.close()


def test_replaced_load_during_snapshot_read_never_publishes_old_recording():
    started, release = threading.Event(), threading.Event()
    class SlowRecording(MultiSource):
        def frame(self, topic, stamp):
            if self.path == 'old' and topic == '/depth':
                started.set()
                assert release.wait(2)
            return super().frame(topic, stamp)
    backend = TopicReplayBackend(loader=SlowRecording)
    try:
        backend.load('old')
        assert started.wait(1)
        backend.load('new')
        assert backend.snapshot()['frames'] == {}
        release.set()
        ready(backend)
        snapshot = backend.snapshot()
        assert snapshot['path'] == 'new'
        assert {frame['details']['path'] for frame in snapshot['frames'].values()} == {'new'}
    finally:
        release.set()
        backend.close()


def test_visual_limit_does_not_hide_omitted_topic_from_individual_browser():
    backend = TopicReplayBackend(loader=MultiSource, max_visual_topics=2)
    try:
        backend.load('multi')
        ready(backend)
        assert set(backend.snapshot()['frames']) == {'/rgb', '/depth'}
        assert backend.metadata()['visual_topics_omitted'] == ['/tracker']
        assert backend.snapshot()['warnings']
        backend.select_topic('/tracker')
        assert ready(backend)['topic'] == '/tracker'
        assert set(backend.snapshot()['frames']) == {'/rgb', '/depth'}
    finally:
        backend.close()
