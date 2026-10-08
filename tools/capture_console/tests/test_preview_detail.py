"""Camera/Tracker detail windows reuse one cache, not another data consumer."""
import subprocess

import pytest
from PySide6 import QtCore, QtGui, QtWidgets, QtTest
from shiboken6 import isValid

from vt_capture_console.ui.widgets.preview import MAX_TRAIL_POINTS, PreviewWidget
from vt_capture_console.ui.widgets.preview_detail import PreviewDetailDialog
from test_preview_widgets import device, jpeg_preview


def tracker_payload(x=0.):
    return dict(position=[x, .02, .03], orientation=[0., 0., 0., 1.], valid=True)


def sample(kind, payload=None, **changes):
    if payload is None:
        payload = jpeg_preview() if kind == "camera" else tracker_payload()
    return device(kind, payload, **{**dict(topic=f"/{kind}/sample", receive_hz=30., valid_hz=29.), **changes})


@pytest.fixture
def previews(qapp):
    objects = []

    def create(kind, value=None):
        source = PreviewWidget(kind)
        source.resize(300, 210)
        source.set_device(value or sample(kind))
        objects.append(source)
        detail = PreviewDetailDialog(source, source._device)
        objects.append(detail)
        return source, detail

    yield create
    for obj in reversed(objects):
        if isValid(obj):
            obj.close()
            obj.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    qapp.processEvents()


@pytest.mark.parametrize("kind", ["camera", "tracker"])
def test_detail_native_nonmodal_large_window_and_no_timer(previews, kind):
    source, detail = previews(kind)
    assert detail.identity == source._identity and detail.kind == kind
    assert detail.size() == QtCore.QSize(1000, 720)
    assert detail.testAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
    assert detail.windowFlags() & QtCore.Qt.WindowType.WindowMaximizeButtonHint
    assert not detail.windowFlags() & QtCore.Qt.WindowType.FramelessWindowHint
    assert not detail.isModal()
    assert detail.windowModality() == QtCore.Qt.WindowModality.NonModal
    assert detail.findChildren(QtCore.QTimer) == []
    assert detail.preview._frame == source._frame
    assert detail.preview._frame is not source._frame
    assert "30.0 Hz" in detail.status.text()
    assert "关闭窗口不停止采集或录制" in detail.readonly_note.text()
    if kind == "camera":
        assert "降采样 JPEG" in detail.source_note.text()
        assert "并非相机原始分辨率" in detail.source_note.text()
    else:
        assert "拖动旋转" in detail.source_note.text() and "双击复位" in detail.source_note.text()


def test_camera_copies_existing_decoded_qimage_without_decode_or_process(qapp, monkeypatch):
    source = PreviewWidget("camera")
    source.set_device(sample("camera"))

    def forbidden(*args, **kwargs):
        raise AssertionError("detail must not decode, prepare data or start a process")

    monkeypatch.setattr(subprocess, "Popen", forbidden)
    monkeypatch.setattr(PreviewWidget, "_prepare", forbidden)
    monkeypatch.setattr(PreviewWidget, "_prepare_image", forbidden)
    detail = PreviewDetailDialog(source, source._device)
    try:
        assert detail.preview._image is not source._image
        assert detail.preview._image.cacheKey() == source._image.cacheKey()
        assert detail.preview._encoded_image is source._encoded_image
        detail.update_preview(source, source._device)
        assert detail.preview._image.cacheKey() == source._image.cacheKey()
        # Qt's implicit sharing detaches on mutation, so receivers cannot
        # corrupt the source image even if a future UI extension changes pixels.
        original = source._image.pixelColor(0, 0)
        detail.preview._image.setPixelColor(0, 0, QtGui.QColor("red"))
        assert source._image.pixelColor(0, 0) == original
    finally:
        detail.close()
        source.close()


@pytest.mark.parametrize("kind", ["camera", "tracker"])
@pytest.mark.parametrize("mode", ["live", "demo", "replay"])
def test_opening_held_view_keeps_mode_errors_and_history(previews, kind, mode):
    payload = jpeg_preview() if kind == "camera" else tracker_payload()
    if mode == "demo":
        payload = {"demo": True, "phase": .4}
    elif mode == "replay":
        payload["replay"] = True
    source, detail = previews(kind, sample(kind, payload))
    before = dict(source._frame)
    source.set_device(sample(kind, {}, status="offline"))
    detail.update_preview(source, source._device)
    assert detail.preview._mode == source._mode == mode
    assert detail.preview._held and detail.preview._frame == before
    assert detail.preview._frame == source._frame
    source.set_device(sample(kind, payload))
    detail.update_preview(source, source._device)
    assert not detail.preview._held


@pytest.mark.parametrize("kind", ["camera", "tracker"])
def test_invalid_frames_retain_good_cache_and_recover(previews, kind):
    source, detail = previews(kind)
    before = dict(source._frame)
    bad = dict(format="jpeg", data_b64="not-an-image") if kind == "camera" else dict(
        position=[0, 0, 0], orientation=[0, 0, 0, 0], valid=True)
    source.set_device(sample(kind, bad))
    detail.update_preview(source, source._device)
    assert detail.preview._frame == before and detail.preview._held
    assert detail.preview._preview_error == source._preview_error != ""
    assert detail.status.toolTip() == source._preview_error
    source.set_device(sample(kind))
    detail.update_preview(source, source._device)
    assert not detail.preview._held and not detail.preview._preview_error
    assert not detail.status.toolTip()


def test_tracker_trail_copy_is_bounded_independent_and_does_not_reset_view(previews):
    source, detail = previews("tracker")
    detail.preview._yaw, detail.preview._pitch, detail.preview._zoom = .7, -.5, 1.8
    for i in range(150):
        source.set_device(sample("tracker", tracker_payload(i * .001)))
        detail.update_preview(source, source._device)
    assert list(detail.preview._trail) == list(source._trail)
    assert detail.preview._trail is not source._trail
    assert len(detail.preview._trail) == MAX_TRAIL_POINTS
    assert detail.preview._tracker_origin == source._tracker_origin
    assert detail.preview._tracker_span == source._tracker_span
    assert (detail.preview._yaw, detail.preview._pitch, detail.preview._zoom) == (.7, -.5, 1.8)
    detail.preview._trail.append((999, 999, 999))
    assert source._trail[-1] != (999, 999, 999)
    detail.update_preview(source, source._device)
    assert list(detail.preview._trail) == list(source._trail)


def test_tracker_drag_wheel_and_double_click_affect_only_detail(previews, qapp):
    source, detail = previews("tracker")
    detail.show()
    qapp.processEvents()
    canvas = detail.preview
    baseline = (source._yaw, source._pitch, source._zoom)
    QtTest.QTest.mousePress(canvas, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(150, 150))
    QtTest.QTest.mouseMove(canvas, QtCore.QPoint(200, 180))
    QtTest.QTest.mouseRelease(canvas, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(200, 180))
    assert (canvas._yaw, canvas._pitch) != baseline[:2]
    wheel = QtGui.QWheelEvent(QtCore.QPointF(150, 150), QtCore.QPointF(150, 150), QtCore.QPoint(),
        QtCore.QPoint(0, 120), QtCore.Qt.MouseButton.NoButton, QtCore.Qt.KeyboardModifier.NoModifier,
        QtCore.Qt.ScrollPhase.NoScrollPhase, False)
    QtWidgets.QApplication.sendEvent(canvas, wheel)
    assert canvas._zoom > 1.
    assert (source._yaw, source._pitch, source._zoom) == baseline
    source.set_device(sample("tracker", tracker_payload(.1)))
    held_angle = (canvas._yaw, canvas._pitch, canvas._zoom)
    detail.update_preview(source, source._device)
    assert (canvas._yaw, canvas._pitch, canvas._zoom) == held_angle
    QtTest.QTest.mouseDClick(canvas, QtCore.Qt.MouseButton.LeftButton, pos=QtCore.QPoint(150, 150))
    assert (canvas._yaw, canvas._pitch, canvas._zoom) == (-.24, .32, 1.)


@pytest.mark.parametrize("kind", ["camera", "tracker"])
def test_maximize_restore_and_close_only_destroy_preview(previews, qapp, kind):
    source, detail = previews(kind)
    frame = source._frame
    detail.show()
    qapp.processEvents()
    detail.maximize_button.click()
    qapp.processEvents()
    assert detail.isMaximized() and detail.maximize_button.text() == "恢复窗口"
    detail.maximize_button.click()
    qapp.processEvents()
    assert not detail.isMaximized() and detail.maximize_button.text() == "最大化"
    detail.close_button.click()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    qapp.processEvents()
    assert not isValid(detail) and isValid(source)
    assert source._frame is frame
    source.set_device(sample(kind))
    assert source._frame is not None


@pytest.mark.parametrize("kind", ["camera", "tracker"])
def test_wrong_source_identity_kind_or_device_cannot_mix_cache(previews, kind):
    source, detail = previews(kind)
    old = dict(detail.preview._frame)
    with pytest.raises(ValueError, match="不能更换"):
        detail.update_preview(source, {**source._device, "id": "other"})
    source.set_device(sample(kind, id="other"))
    with pytest.raises(ValueError, match="同一设备"):
        detail.update_preview(source, {**source._device, "id": detail.identity[0]})
    other = PreviewWidget("tracker" if kind == "camera" else "camera")
    try:
        with pytest.raises(ValueError):
            detail.preview.copy_camera_tracker_state_from(other)
        with pytest.raises(ValueError, match="same device"):
            detail.preview.copy_camera_tracker_state_from(source)
    finally:
        other.close()
    assert detail.preview._frame == old


@pytest.mark.parametrize("kind", ["glove", "tactile"])
def test_specific_hand_dialogs_are_not_replaced(qapp, kind):
    source = PreviewWidget(kind)
    try:
        with pytest.raises(ValueError, match="仅支持"):
            PreviewDetailDialog(source, {"id": "hand", "kind": kind})
        with pytest.raises(ValueError, match="matching preview kinds"):
            source.copy_camera_tracker_state_from(source)
    finally:
        source.close()


def test_label_markup_is_plain_text_and_updates_without_recreating_window(previews):
    source, detail = previews("camera")
    source.set_device(sample("camera", label="<b>Camera</b>", receive_hz=float("nan")))
    detail.update_preview(source, source._device)
    assert detail.title.text() == "<b>Camera</b>"
    assert detail.title.textFormat() == QtCore.Qt.TextFormat.PlainText
    assert "接收 — Hz" in detail.status.text()
