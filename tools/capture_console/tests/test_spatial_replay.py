"""Replay is asynchronous and never substitutes current calibration."""
import hashlib
import json
import threading
import time

import pytest

from vt_capture_console.backend.scene_demo import demo_scene
from vt_capture_console.backend.spatial_replay import SpatialReplayBackend, load_replay_source


def wait_ready(backend, index=0):
    deadline = time.monotonic()+2
    while time.monotonic() < deadline:
        frame = backend.latest()
        if backend.status()["state"] == "ready" and backend.timeline()["index"] == index:
            return frame
        time.sleep(.005)
    pytest.fail(str(backend.status()))


def test_archive_seek_has_new_epoch_and_preserves_world_positions(tmp_path):
    path = tmp_path / "scene.json"
    frames = [demo_scene(0, sequence=1), demo_scene(1, sequence=2)]
    frames[0]['preview_notice'] = '左腕·试用候选（三轮一致性未通过）'
    path.write_text(json.dumps(dict(schema_version=1, kind="vt.spatial.archive", frames=frames)))
    backend = SpatialReplayBackend()
    try:
        backend.load(path)
        first = wait_ready(backend)
        assert first["mode"] == "replay"
        assert first['preview_notice'] == frames[0]['preview_notice']
        backend.seek(1)
        second = wait_ready(backend, 1)
        assert first["scene_epoch"] != second["scene_epoch"]
        assert second["trackers"][0]["position_m"] == frames[1]["trackers"][0]["position_m"]
        assert not second.get('preview_notice')
    finally:
        backend.close()


def test_alignment_replay_checks_hash_and_reports_missing_pixels(tmp_path):
    pose = {"translation_m": [1., 2., 3.], "quaternion_xyzw": [0., 0., 0., 1.]}
    record = {"frame_index": 0, "reference_time_ns": 100, "reference_camera": "d405_1",
              "trackers": {"left_wrist": {"tracker_id": "original", "timestamp_ns": 100, "world_from_tracker": pose}},
              "cameras": {"d405_1": {"host_realtime_ns": 100, "world_from_camera": pose}}, "quality_flags": []}
    data = (json.dumps(record)+"\n").encode()
    path = tmp_path / "aligned_frames.jsonl"
    path.write_bytes(data)
    manifest = {"schema_version": 1, "tool": "vt-multisensor-alignment", "configuration": {"world_frame": "vive_map"},
                "files": {path.name: {"size_bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}}}
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    source = load_replay_source(path)
    frame = source.frame(0)
    assert frame["trackers"][0]["position_m"] == [1., 2., 3.]
    assert "点云不可用" in source.note
    path.write_bytes(data+b" ")
    with pytest.raises(ValueError, match="完整性"):
        load_replay_source(path)


@pytest.mark.parametrize("failure", ["frame", "scene", "cloud"])
def test_failed_first_frame_closes_unpublished_source(failure):
    closed = threading.Event()

    class Source:
        frame_count = 1
        note = "synthetic"
        last_clouds = ["broken cloud"] if failure == "cloud" else []

        def frame(self, index):
            if failure == "frame":
                raise ValueError("first frame failed")
            return {} if failure == "scene" else demo_scene(0, sequence=1)

        def close(self):
            closed.set()

    backend = SpatialReplayBackend(loader=lambda *args: Source())
    try:
        backend.load("synthetic")
        assert closed.wait(1), "failed initial frame left the temporary source open"
        assert backend.status()["state"] == "error"
        assert backend.timeline()["frame_count"] == 0
    finally:
        backend.close()


def test_replaced_load_cancels_first_frame_and_closes_temporary_source():
    entered, cancelled, closed = threading.Event(), threading.Event(), threading.Event()

    class Source:
        frame_count = 1
        note = "synthetic"

        def __init__(self, path, cancellation):
            self.path, self.cancelled = path, cancellation

        def set_cancelled(self, callback):
            self.cancelled = callback

        def frame(self, index):
            if self.path == "old":
                entered.set()
                deadline = time.monotonic() + 1
                while not self.cancelled() and time.monotonic() < deadline:
                    time.sleep(.005)
                if self.cancelled():
                    cancelled.set()
                    raise InterruptedError("old load cancelled")
            return demo_scene(0, sequence=1)

        def close(self):
            if self.path == "old":
                closed.set()

    backend = SpatialReplayBackend(loader=Source)
    try:
        backend.load("old")
        assert entered.wait(1)
        backend.load("new")
        assert cancelled.wait(.5), "frame cancellation ignored a new load generation"
        wait_ready(backend)
        assert closed.is_set()
        assert backend.timeline()["path"] == "new"
    finally:
        backend.close()
