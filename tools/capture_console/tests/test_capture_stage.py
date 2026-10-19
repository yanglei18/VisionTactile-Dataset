"""Focused preview follows existing stream caches without more acquisition."""
from types import SimpleNamespace

import pytest
from PySide6 import QtCore, QtGui, QtTest
from shiboken6 import isValid

from vt_capture_console.ui.widgets.capture_stage import CaptureStage
from vt_capture_console.ui.widgets.preview import PreviewWidget
from test_preview_widgets import device, jpeg_preview
from test_glove_hand_preview import glove_device
from test_tactile_3d_preview import device as tactile_device


@pytest.fixture
def stage(qapp):
    widget = CaptureStage()
    widget.resize(850, 480)
    widget.show()
    yield widget
    widget.close()
    widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    qapp.processEvents()


@pytest.fixture
def source(qapp):
    widgets = []

    def create(kind="camera", value=None):
        widget = PreviewWidget(kind)
        widget.set_device(value or device(kind, jpeg_preview() if kind == "camera" else {
            "demo": True, "phase": .3}, receive_hz=30.0))
        widgets.append(widget)
        return widget

    yield create
    for widget in widgets:
        if not isValid(widget):
            continue
        for name in ("_glove_3d", "_tactile_3d"):
            renderer = getattr(widget, name)
            if renderer is not None:
                renderer.dispose()
                setattr(widget, name, None)
        widget.close()
        widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    qapp.processEvents()


def test_empty_selection_then_expand_and_clear_only_change_stage(stage, source):
    assert stage.preview is None and stage.canvas.currentWidget() is stage.empty
    assert not stage.expand_button.isEnabled()
    selected = source()
    before = selected._frame
    stage.set_source(selected, selected._device)
    requested = QtTest.QSignalSpy(stage.expand_requested)
    stage.expand_button.click()
    assert requested.count() == 1
    assert stage.minimumHeight() == 220
    assert stage.preview.sizePolicy().verticalPolicy().name == "Expanding"
    assert not stage.glove_controls.isVisible() and not stage.tactile_controls.isVisible()
    assert stage.findChildren(QtCore.QTimer) == []
    stage.clear_source()
    assert stage.preview is None and stage.source_preview is None
    assert not stage.expand_button.isEnabled()
    assert selected._frame is before and isValid(selected)
    stage.clear_source()


def test_camera_shared_pixels_no_decode_or_extra_preparation(stage, source, monkeypatch):
    selected = source()

    def forbidden(*args, **kwargs):
        raise AssertionError("stage must reuse the source's prepared cache")

    monkeypatch.setattr(PreviewWidget, "_prepare", forbidden)
    monkeypatch.setattr(PreviewWidget, "_prepare_image", forbidden)
    stage.set_source(selected, selected._device)
    preview = stage.preview
    stage.refresh_source()
    stage.set_source(selected, selected._device)
    assert stage.preview is preview
    assert preview._image.cacheKey() == selected._image.cacheKey()
    assert preview._image is not selected._image
    assert preview._encoded_image is selected._encoded_image
    original = selected._image.pixelColor(0, 0)
    preview._image.setPixelColor(0, 0, QtGui.QColor("red"))
    assert selected._image.pixelColor(0, 0) == original


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
def test_offline_and_replay_markers_follow_source_without_invented_live_data(stage, source, kind):
    selected = source(kind)
    stage.set_source(selected, selected._device)
    frame = dict(selected._frame)
    selected.set_device({**selected._device, "status": "offline", "preview": {}})
    stage.refresh_source()
    assert stage.preview._held and stage.preview._frame == frame
    assert "离线" in stage.status.text() and "保持上一画面" in stage.status.text()
    assert stage.preview._mode == selected._mode
    if kind == "camera":
        selected.set_device({**selected._device, "status": "online", "preview": {**jpeg_preview(), "replay": True}})
        stage.refresh_source()
        assert stage.preview._mode == "replay" and "回放" in stage.status.text()


def test_tracker_rotation_and_zoom_survive_refresh_and_do_not_mutate_thumbnail(stage, source, qapp):
    selected = source("tracker")
    stage.set_source(selected, selected._device)
    qapp.processEvents()
    stage.preview._yaw, stage.preview._pitch, stage.preview._zoom = .9, -.2, 1.6
    selected.set_device({**selected._device, "preview": {"demo": True, "phase": .8}})
    stage.refresh_source()
    assert (stage.preview._yaw, stage.preview._pitch, stage.preview._zoom) == (.9, -.2, 1.6)
    assert (selected._yaw, selected._pitch, selected._zoom) == (-.24, .32, 1.)
    assert list(stage.preview._trail) == list(selected._trail)
    assert stage.preview._trail is not selected._trail


def test_new_stream_and_hand_cannot_reuse_previous_frame(stage, source):
    camera = source()
    stage.set_source(camera, camera._device)
    first = stage.preview
    tactile = source("tactile", tactile_device())
    stage.set_source(tactile, tactile._device)
    assert stage.preview is not first and stage.preview.kind == "tactile"
    assert stage.preview._frame == tactile._frame
    tactile.set_device(tactile_device("left", preview={}))
    stage.refresh_source()
    assert stage.preview._identity == ("left_tactile", "left")
    assert stage.preview._frame is None
    assert stage.preview.tactile_view == "matrix"
    assert not stage.tactile_controls.hand_button.isEnabled()
    assert not stage.tactile_controls.three_d_button.isEnabled()


def test_tactile_selectors_follow_source_both_directions_and_preserve_values(stage, source):
    selected = source("tactile", tactile_device())
    stage.set_source(selected, selected._device)
    frame = selected._frame
    stage.tactile_controls.matrix_button.click()
    assert stage.preview.tactile_view == selected.tactile_view == "matrix"
    selected.set_tactile_view("hand")
    assert stage.preview.tactile_view == "hand" and stage.tactile_controls.hand_button.isChecked()
    assert selected._frame is frame and stage.preview._frame == frame
    assert stage.preview._levels is selected._levels
    assert stage.preview._cell_colors is selected._cell_colors


def test_glove_mano_worker_shared_with_card_and_deduplicates_request(stage, source, monkeypatch):
    from vt_capture_console.backend.pose_preview import PosePreview
    from vt_capture_console.ui.widgets import glove_3d

    worker = PosePreview("right", configuration_provider=lambda: {})
    monkeypatch.setattr(worker._settings, "_configuration", lambda hand: ("test-python", "test-assets"))
    starts = []
    monkeypatch.setattr(worker, "_start", lambda configuration: starts.append(configuration))
    monkeypatch.setattr(glove_3d, "pose_preview", lambda hand: worker)
    selected = source("glove", glove_device())
    selected.set_glove_view("3d")
    stage.set_source(selected, selected._device)
    assert stage.preview._glove_3d.worker is selected._glove_3d.worker is worker
    assert starts == [("test-python", "test-assets")]
    assert stage.preview._frame == selected._frame
    stage.glove_controls.skeleton_button.click()
    assert stage.preview.glove_view == selected.glove_view == "skeleton"
    selected.set_glove_view("3d")
    assert stage.preview.glove_view == "3d"
    assert starts == [("test-python", "test-assets")]
    stage_renderer, source_renderer = stage.preview._glove_3d, selected._glove_3d
    stage.clear_source()
    assert stage_renderer._disposed and not source_renderer._disposed
    assert worker._desired_key is not None  # Clearing a view never clears the shared pose.
    worker.shutdown()


def test_tactile_3d_reuses_pending_loader_and_disposes_only_stage(stage, source, monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    from vt_capture_console.ui.widgets.tactile_model_loader import TactileModelLoader

    loader = TactileModelLoader(configuration_provider=lambda: {})
    key = ("test-python", "test-assets")
    monkeypatch.setattr(loader, "_key", lambda hand: key)
    pending = SimpleNamespace(key=key)
    loader._jobs["right"] = pending
    monkeypatch.setattr(tactile_3d, "model_loader", lambda: loader)
    selected = source("tactile", tactile_device())
    selected.set_tactile_view("3d")
    stage.set_source(selected, selected._device)
    assert stage.preview._tactile_3d.loader is selected._tactile_3d.loader is loader
    assert loader._jobs["right"] is pending
    assert loader.findChildren(QtCore.QProcess) == []
    renderer = stage.preview._tactile_3d
    stage.clear_source()
    assert renderer._disposed and not selected._tactile_3d._disposed
    loader._jobs.clear()
    loader.shutdown()


def test_glove_calibration_has_selected_hand_and_respects_disabled_state(stage, source):
    selected = source("glove")
    stage.set_source(selected, selected._device)
    requested = QtTest.QSignalSpy(stage.calibration_requested)
    stage.glove_controls.calibration_button.click()
    assert requested.count() == 1 and requested.at(0) == ["right"]
    selected.set_device({**selected._device, "glove_action_enabled": False})
    stage.refresh_source()
    stage.glove_controls.calibration_button.click()
    assert requested.count() == 1


def test_replay_stage_reports_sample_age_and_timestamps_without_live_rates(stage, source):
    selected = source("camera", device("camera", {**jpeg_preview(), "replay": True},
                                      mode="replay", status="ready", replay_available=True,
                                      receive_hz=30.0, sample_age_s=.125,
                                      bag_timestamp_ns=123456789, source_realtime_ns=987654321))
    stage.set_source(selected, selected._device)
    assert "回放" in stage.status.text() and "历史样本" in stage.status.text()
    assert "0.125" in stage.stream_info.text()
    assert "Hz" not in stage.stream_info.text()
    assert "123456789 ns" in stage.stream_info.toolTip()
    assert "987654321 ns" in stage.stream_info.toolTip()
    assert not stage.metric_panel.isVisible() and not stage.hero.isVisible()
    assert stage.canvas.currentWidget() is stage.preview


def test_empty_replay_uses_device_mode_and_explains_missing_topic(stage, source):
    selected = source("camera", device("camera", mode="replay", status="unknown",
                                      replay_available=False, detail="/camera/rgb 未录制数据"))
    assert selected.mode == "replay" and not selected.has_frame
    stage.set_source(selected, selected._device)
    assert "回放" in stage.status.text() and "该时刻无样本" in stage.status.text()
    assert stage.hero_title.text() == "该时刻无样本"
    assert stage.hero_lead.text() == "/camera/rgb 未录制数据"
    assert "历史回放" in stage.hero_kicker.text()
    assert stage.canvas.currentWidget() is stage.empty
    assert "接收" not in stage.stream_info.text()
    assert not stage.metric_panel.isVisible()


def test_unavailable_replay_hides_retained_picture_and_recovers_with_next_sample(stage, source):
    selected = source("camera", device("camera", {**jpeg_preview(), "replay": True},
                                      mode="replay", status="ready", replay_available=True))
    stage.set_source(selected, selected._device)
    previous = selected._frame
    selected.set_device({**selected._device, "preview": {}, "replay_available": False})
    stage.refresh_source()
    assert selected._frame is previous  # The stage never edits the source cache.
    assert stage.canvas.currentWidget() is stage.empty
    assert "该时刻无样本" in stage.status.text()
    selected.set_device({**selected._device, "preview": {**jpeg_preview(), "replay": True},
                         "replay_available": True})
    stage.refresh_source()
    assert stage.canvas.currentWidget() is stage.preview
    assert "历史样本" in stage.status.text() and not stage.hero.isVisible()


@pytest.mark.parametrize("replay_mark", ["device", "preview"])
def test_replay_glove_never_emits_calibration_even_when_called_directly(stage, source, replay_mark):
    value = glove_device()
    if replay_mark == "device":
        value.update(mode="replay", preview={}, replay_available=False)
    else:
        value["preview"] = {**value["preview"], "replay": True}
    selected = source("glove", value)
    stage.set_source(selected, selected._device)
    requested = QtTest.QSignalSpy(stage.calibration_requested)
    assert not stage.glove_controls.calibration_button.isEnabled()
    stage.glove_controls.calibration_button.click()
    stage._request_calibration()
    assert requested.count() == 0


def test_live_stage_recovers_live_labels_and_calibration_after_replay(stage, source):
    value = glove_device(mode="replay", receive_hz=60.0)
    value["preview"] = {**value["preview"], "replay": True}
    selected = source("glove", value)
    stage.set_source(selected, selected._device)
    selected.set_device(glove_device(receive_hz=60.0))
    stage.refresh_source()
    assert "实时" in stage.status.text() and "接收 60.0 Hz" in stage.stream_info.text()
    assert stage.glove_controls.calibration_button.isEnabled()
    requested = QtTest.QSignalSpy(stage.calibration_requested)
    stage._request_calibration()
    assert requested.count() == 1


def test_removed_source_clears_selection_and_old_source_cannot_change_current_stage(stage, source, qapp):
    first = source("tactile", tactile_device())
    stage.set_source(first, first._device)
    second = source()
    stage.set_source(second, second._device)
    first.set_tactile_view("matrix")
    assert stage.source_preview is second and stage.preview.kind == "camera"
    second.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    qapp.processEvents()
    assert stage.preview is None and stage.source_preview is None


def test_reject_wrong_cache_identity_and_handle_plain_text_and_invalid_rate(stage, source):
    selected = source()
    with pytest.raises(ValueError, match="同一设备"):
        stage.set_source(selected, {**selected._device, "id": "different"})
    with pytest.raises(ValueError, match="同一设备"):
        stage.set_source(selected, {**selected._device, "kind": "tracker"})
    selected.set_device({**selected._device, "label": "<b>Camera</b>", "receive_hz": float("nan")})
    stage.set_source(selected, selected._device)
    assert stage.title.text() == "<b>Camera</b>"
    assert stage.title.textFormat() == QtCore.Qt.TextFormat.PlainText
    assert "接收 — Hz" in stage.stream_info.text()


def test_reduced_motion_freezes_only_synthetic_decoration_and_live_frames_continue(stage, source):
    from test_preview_widgets import render, pixels

    selected = source("camera", device("camera", {"demo": True, "phase": .3}))
    stage.set_source(selected, selected._device)
    stage.set_motion_enabled(False)
    before = pixels(render(stage.preview))
    selected.set_device({**selected._device, "preview": {"demo": True, "phase": 1.8}})
    stage.refresh_source()
    assert stage.preview._frame["phase"] == 1.8
    assert pixels(render(stage.preview)) == before
    stage.set_motion_enabled(True)
    assert pixels(render(stage.preview)) != before

    stage.set_motion_enabled(False)
    selected.set_device({**selected._device, "preview": jpeg_preview(96, 54)})
    stage.refresh_source()
    first_key = stage.preview._image.cacheKey()
    selected.set_device({**selected._device, "preview": jpeg_preview(80, 45)})
    stage.refresh_source()
    assert stage.preview._image.cacheKey() != first_key
    assert stage.preview._image.cacheKey() == selected._image.cacheKey()
    assert stage.preview._frame == {"width": 80, "height": 45}
    assert stage.findChildren(QtCore.QTimer) == []


def test_intro_and_large_metrics_never_overlay_live_replayed_or_held_pixels(stage, source, qapp):
    from test_preview_widgets import render

    stage.set_motion_enabled(False)
    selected = source("camera", device("camera", {"demo": True, "phase": .3}))
    stage.set_source(selected, selected._device)
    qapp.processEvents()
    assert stage.hero.isVisible() and stage.metric_panel.isVisible()
    assert "合成" in stage.hero_lead.text()
    assert not render(stage).isNull()
    for payload in (jpeg_preview(), {**jpeg_preview(), "replay": True}, {}):
        selected.set_device({**selected._device, "preview": payload})
        stage.refresh_source()
        qapp.processEvents()
        assert not stage.hero.isVisible() and not stage.metric_panel.isVisible()
        assert stage.title.isVisible()
        canvas = stage.preview._canvas_rect()
        assert canvas.left() == 0 and canvas.width() == stage.preview.width()
        assert not render(stage).isNull()
    assert stage.preview._held


def test_empty_stage_stays_opaque_when_source_disappears_during_entry(stage, source):
    assert stage._hero_opacity.opacity() == 1.
    assert stage.source_preview is None and stage.preview is None
    selected = source("camera", device("camera", {"demo": True, "phase": .3}))
    stage.set_source(selected, selected._device)
    stage._hero_animation.setCurrentTime(80)
    assert stage._hero_opacity.opacity() < 1.
    frame = selected._frame
    for _ in range(6):
        stage.clear_source()
        assert stage._hero_opacity.opacity() == 1.
        assert stage._hero_animation.state() == QtCore.QAbstractAnimation.State.Stopped
        assert stage.source_preview is None and stage.preview is None
        assert stage.canvas.currentWidget() is stage.empty
        assert "等待选择" in stage.stream_info.text()
    assert selected._frame is frame


class PaintCounter(QtCore.QObject):
    def __init__(self):
        super().__init__()
        self.count = 0

    def eventFilter(self, watched, event):
        if event.type() == QtCore.QEvent.Type.Paint:
            self.count += 1
        return False


def test_idle_camera_age_updates_do_not_repaint_but_new_image_does(stage, source, qapp):
    stage.set_motion_enabled(False)
    selected = source("camera", device("camera", status="offline", receive_hz=0.0))
    stage.set_source(selected, selected._device)
    for _ in range(3):
        qapp.processEvents()
    paints = PaintCounter()
    stage.preview.installEventFilter(paints)
    for age in range(10):
        selected.set_device({**selected._device, "receive_age_s": age})
        stage.refresh_source()
        qapp.processEvents()
    assert paints.count == 0
    assert stage.preview._device["receive_age_s"] == 9

    selected.set_device({**selected._device, "status": "online", "receive_hz": 30.0,
                         "preview": jpeg_preview(80, 45)})
    stage.refresh_source()
    qapp.processEvents()
    assert paints.count > 0
    assert stage.preview._image.cacheKey() == selected._image.cacheKey()
    assert "接收 30.0 Hz" in stage.stream_info.text()


def test_repeated_empty_selection_does_not_repaint_hero(stage, qapp):
    stage.set_motion_enabled(False)
    for _ in range(3):
        qapp.processEvents()
    paints = PaintCounter()
    stage.hero.installEventFilter(paints)
    for _ in range(10):
        stage.clear_source()
        qapp.processEvents()
    assert paints.count == 0


def test_small_tracker_cache_change_cannot_mutate_previous_signature(stage, source):
    selected = source("tracker")
    stage.set_source(selected, selected._device)
    changed_position = tuple(value + .1 for value in selected._frame["position"])
    selected._frame["position"] = changed_position
    stage.refresh_source()
    assert stage.preview._frame["position"] == changed_position
