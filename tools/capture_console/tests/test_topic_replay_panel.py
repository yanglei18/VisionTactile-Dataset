from copy import deepcopy

from PySide6.QtWidgets import QDialog

from vt_capture_console.ui.widgets.topic_replay import TopicReplayPanel, trial_replay_allowed


class Backend:
    def __init__(self):
        self.path = ""
        self.topic = "/rgb"
        self.closed = False
        self.playing = False
        self.stamp = 100
        self.frame = None
    def load(self, path): self.path = str(path)
    def clear(self): self.path = ""; self.frame = None
    def metadata(self):
        return dict(topics=[dict(name="/rgb", count=2, type="Image", kind="camera"),
                            dict(name="/tracker", count=0, type="Tracker", kind="tracker")],
                    warnings=["Tracker not recorded"], start_ns=100, end_ns=200) if self.path else {}
    def status(self): return dict(state="ready" if self.path else "waiting", message="Partial bag")
    def timeline(self):
        return dict(path=self.path, topic=self.topic, start_ns=100, end_ns=200,
                    time_ns=self.stamp, playing=self.playing, rate=1.)
    def select_topic(self, topic): self.topic = topic; self.frame = None
    def seek(self, stamp): self.stamp = stamp
    def play(self): self.playing = True
    def pause(self): self.playing = False
    def set_rate(self, rate): pass
    def latest(self): return deepcopy(self.frame)
    def close(self): self.closed = True


def test_warning_trial_is_playable_but_active_or_demo_is_not():
    trial = dict(state="COMPLETE", quality="WARNINGS", output_path="/missing/bag")
    assert trial_replay_allowed(trial)
    for state in ("RECORDING", "STARTING", "STOPPING", "UNKNOWN", "FINALIZING"):
        assert not trial_replay_allowed(dict(trial, state=state))
    assert not trial_replay_allowed(trial, demo=True)


def test_replay_opens_space_without_a_separate_preview_page(qapp):
    panel = TopicReplayPanel(backend=Backend())
    try:
        panel.load("partial-bag")
        assert panel.views.count() == 3
        assert panel.views.currentWidget() is panel.spatial
        assert [panel.views.tabText(index) for index in range(2)] == ["空间", "话题详情"]
        assert panel.views.tabText(panel.views.indexOf(panel.warnings)) == "缺失与限制（1）"
        assert "Tracker not recorded" in panel.warnings.toPlainText()
        assert panel.play_button.isEnabled()
    finally:
        panel.close()


def test_panel_is_inline_and_displays_empty_topics_without_blocking_play(qapp):
    backend = Backend()
    panel = TopicReplayPanel(backend=backend)
    try:
        assert not isinstance(panel, QDialog)
        panel.load("partial-bag")
        panel.refresh()
        assert panel.topics.rowCount() == 2
        assert "Tracker not recorded" in panel.warnings.toPlainText()
        assert panel.play_button.isEnabled()
        panel.play_button.click()
        assert backend.playing
        panel.topics.selectRow(1)
        assert backend.topic == "/tracker"
    finally:
        panel.close()
    assert backend.closed


def test_changing_topic_clears_previous_sample(qapp):
    backend = Backend()
    panel = TopicReplayPanel(backend=backend)
    try:
        panel.load("partial-bag")
        backend.frame = dict(topic="/rgb", time_ns=100, sample_ns=100, device=None,
                             details={"marker": "old sample"}, warnings=[])
        panel.refresh()
        assert "old sample" in panel.details.toPlainText()
        panel.topics.selectRow(1)
        panel.refresh()
        assert "old sample" not in panel.details.toPlainText()
        panel.clear()
        assert panel.topics.rowCount() == 0
    finally:
        panel.close()


def test_external_path_guard_cannot_load_active_recording(qapp):
    backend = Backend()
    panel = TopicReplayPanel(backend=backend, path_blocker=lambda path: True)
    try:
        panel.load("active-bag")
        assert not backend.path
        assert not panel.play_button.isEnabled()
    finally:
        panel.close()


def test_continuous_tracker_frames_keep_motion_origin_but_seek_clears_it(qapp):
    backend = Backend()
    panel = TopicReplayPanel(backend=backend)
    try:
        panel.load("partial-bag")
        backend.select_topic("/tracker")
        for stamp, x in ((100, 1.), (150, 2.)):
            backend.frame = dict(topic="/tracker", time_ns=stamp, sample_ns=stamp,
                device=dict(id="/tracker", kind="tracker", status="ready",
                    preview=dict(valid=True, position=[x, 0., 0.], orientation=[0., 0., 0., 1.])),
                details={}, warnings=[])
            panel.refresh()
        assert panel.preview._tracker_origin == (1., 0., 0.)
        assert len(panel.preview._trail) == 2
        assert panel.preview._mode == "replay"
        assert "mode" not in backend.frame["device"]["preview"]
        panel._seek()
        assert panel.preview is None
        assert panel._previews["tracker"]._tracker_origin is None
    finally:
        panel.close()


def test_archive_and_trial_details_share_embedded_player(qapp, monkeypatch):
    from test_trial_details import DetailsBackend
    from vt_capture_console.backend import topic_replay
    from vt_capture_console.ui.main_window import MainWindow
    replay = Backend()
    monkeypatch.setattr(topic_replay, "TopicReplayBackend", lambda: replay)
    capture = DetailsBackend()
    window = MainWindow(capture)
    try:
        window.trial_table.selectRow(0)
        window.replay_trial_button.click()
        assert window.pages.currentWidget() is window.archive_page
        assert window.archive_page.isAncestorOf(window.archive_replay)
        assert replay.path == window._trial_rows[0]["output_path"]
        window.refresh()
        assert replay.path
        window.inspect_trial_button.click()
        assert window.details_dialog.replay_button.isEnabled()
        window.details_dialog.replay_button.click()
        assert not window.details_dialog.isVisible()
        assert window.archive_replay.backend is replay
        assert not capture.commands
        window.trial_table.clearSelection()
        window.refresh()
        assert not replay.path
        assert not window.replay_trial_button.isEnabled()
        active = dict(window._trial_rows[0], state="RECORDING")
        window._trial_rows = [active]
        assert window._archive_path_active(active["output_path"])
        assert window._archive_path_active(str(__import__('pathlib').Path(active["output_path"]).parent))
    finally:
        window.close()
    assert replay.closed


def test_external_recording_clears_trial_binding_and_survives_refresh(qapp, monkeypatch):
    from test_trial_details import DetailsBackend
    from vt_capture_console.backend import topic_replay
    from vt_capture_console.ui.main_window import MainWindow
    replay = Backend()
    monkeypatch.setattr(topic_replay, "TopicReplayBackend", lambda: replay)
    monkeypatch.setattr("PySide6.QtWidgets.QFileDialog.getExistingDirectory", lambda *_: "/external/bag")
    window = MainWindow(DetailsBackend())
    try:
        window.trial_table.selectRow(0)
        window._choose_archive_bag()
        window.refresh()
        assert replay.path == "/external/bag"
        assert window.trial_table.currentRow() == -1
        assert not window.replay_trial_button.isEnabled()
        assert not window.trial_table.property("selectedTrialId")
        assert window.trial_details.text() == "/external/bag"
    finally:
        window.close()
