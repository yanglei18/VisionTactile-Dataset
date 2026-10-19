"""Archive space and topic details retain one historical playback session."""
from copy import deepcopy

from vt_capture_console.ui.widgets.topic_replay import TopicReplayPanel
from test_preview_widgets import jpeg_preview
from test_topic_replay_panel import Backend


def topic(name, kind="camera", count=2):
    return dict(name=name, kind=kind, type="Image" if kind == "camera" else kind, count=count)


def camera_frame(name, sample_ns, *, time_ns=2_000_000_000, width=96):
    return dict(topic=name, time_ns=time_ns, sample_ns=sample_ns, warnings=[], details={},
                device=dict(id=name, label=name, kind="camera", status="ready",
                            preview=jpeg_preview(width, 54)))


def tracker_frame(sample_ns, position, *, time_ns=2_000_000_000):
    return dict(topic="/tracker", time_ns=time_ns, sample_ns=sample_ns, warnings=[], details={},
                device=dict(id="/tracker", label="Tracker", kind="tracker", status="ready", frame_id="recorded_map",
                            preview=dict(valid=True, position=position,
                                         orientation=[0., 0., 0., 1.])))


def snapshot(*frames, time_ns=2_000_000_000, generation=1, seek_revision=0, sequence=1):
    return dict(time_ns=time_ns, generation=generation, seek_revision=seek_revision,
                sequence=sequence, frames={frame["topic"]: frame for frame in frames}, warnings=[])


class SnapshotBackend(Backend):
    def __init__(self):
        super().__init__()
        self.stamp = 2_000_000_000
        self.saved = snapshot(camera_frame("/rgb", 1_000_000_000),
                              camera_frame("/depth", 1_500_000_000),
                              tracker_frame(1_500_000_000, [1., 0., 0.]))
        self.close_count = 0
        self.failed = False

    def metadata(self):
        return dict(topics=[topic("/rgb"), topic("/depth"), topic("/tracker", "tracker")], warnings=[])

    def snapshot(self):
        return deepcopy(self.saved)

    def latest(self):
        return deepcopy(self.saved["frames"].get(self.topic))

    def timeline(self):
        return dict(path=self.path, topic=self.topic, start_ns=0, end_ns=3_000_000_000,
                    time_ns=self.stamp, playing=self.playing, rate=1.)

    def status(self):
        return dict(state="error" if self.failed else "ready", message="Read failed" if self.failed else "Ready")

    def close(self):
        self.close_count += 1
        super().close()


def test_topic_details_and_space_keep_one_playhead_and_hiding_panel_pauses(qapp):
    backend = SnapshotBackend()
    panel = TopicReplayPanel(backend=backend)
    panel.resize(1100, 760)
    panel.show()
    try:
        panel.load("historical-bag")
        panel.timer.stop()
        panel.play_button.click()
        assert backend.playing
        stamp = backend.stamp
        panel.views.setCurrentWidget(panel.topics.parentWidget())
        qapp.processEvents()
        assert backend.playing and backend.stamp == stamp
        assert panel.spatial.backend is panel.scene_backend
        assert len(panel.spatial.model.entities()) == 1
        assert not panel.spatial.replay_bar.isVisible()
        assert not panel.spatial.expand_button.isVisible()
        panel.spatial.open_detail()
        assert panel.spatial.detail_window is None
        panel.topics.selectRow(1)
        assert backend.topic == "/depth"
        assert panel.preview is not None
        assert "0.500" in panel.sample_time.text()
        panel.views.setCurrentWidget(panel.spatial)
        assert backend.playing and backend.stamp == stamp
        panel.hide()
        qapp.processEvents()
        assert not backend.playing
    finally:
        panel.close()
        panel.close()
    assert backend.close_count == 1


def test_global_decoder_error_clears_all_visible_historical_samples(qapp):
    backend = SnapshotBackend()
    panel = TopicReplayPanel(backend=backend)
    try:
        panel.load("historical-bag")
        panel.timer.stop()
        assert panel.preview._image is not None
        assert len(panel.spatial.model.entities()) == 1
        backend.failed = True
        backend.saved = snapshot(sequence=2)
        panel.refresh()
        assert not panel.play_button.isEnabled()
        assert panel.preview is None
        assert panel.preview_stack.currentWidget() is panel.empty
        assert all(preview._frame is None for preview in panel._previews.values())
        assert not panel.spatial.model.entities()
        assert "Read failed" in panel.status.text()
    finally:
        panel.close()


def test_slow_decode_keeps_timeline_and_views_at_published_snapshot(qapp):
    backend = SnapshotBackend()
    panel = TopicReplayPanel(backend=backend)
    try:
        panel.load("historical-bag")
        backend.stamp = 2_750_000_000  # Requested next time is still decoding.
        panel.refresh()
        assert panel.time_label.text() == "00:02.0 / 00:03.0"
        assert panel.timeline.value() == 6666
        assert "1.000" in panel.sample_time.text()
        assert panel.preview._frame["width"] == 96
        assert panel.scene_backend.latest()["scene_time_ns"] == 2_000_000_000
    finally:
        panel.close()
