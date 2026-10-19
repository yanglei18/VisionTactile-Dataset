"""Public preview updates preserve validated caches and device boundaries."""

import pytest
from PySide6 import QtCore

from vt_capture_console.ui.widgets.preview import PreviewWidget
from test_preview_widgets import device, jpeg_preview


@pytest.fixture
def preview(qapp):
    widget = PreviewWidget("camera")
    widget.set_device(device("camera", jpeg_preview(), details={"tags": ["original"]}))
    yield widget
    widget.close()
    widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)


def test_device_snapshot_isolates_nested_metadata_and_input(preview):
    value = preview.device_snapshot
    value["details"]["tags"].append("caller")
    value["preview"]["width"] = 1
    assert preview.device_snapshot["details"]["tags"] == ["original"]
    assert preview.device_snapshot["preview"]["width"] == 64
    preview.set_device(value)
    value["details"]["tags"].append("later")
    assert preview.device_snapshot["details"]["tags"] == ["original", "caller"]


def test_metadata_refresh_keeps_cache_and_does_not_decode_or_repaint(preview, monkeypatch):
    image, frame = preview._image, preview._frame

    def forbidden(*args, **kwargs):
        raise AssertionError("metadata refresh must preserve the rendered sample")

    monkeypatch.setattr(preview, "_prepare", forbidden)
    monkeypatch.setattr(preview, "update", forbidden)
    value = preview.device_snapshot
    value.update(receive_age_s=2.5, sample_age_s=.25)
    preview.refresh_metadata(value)
    value["details"]["tags"].append("caller")
    assert preview._image is image and preview._frame is frame
    assert preview.device_snapshot["sample_age_s"] == .25
    assert preview.device_snapshot["details"]["tags"] == ["original"]


@pytest.mark.parametrize("change", [
    {"id": "other"}, {"side": "left"}, {"kind": "tracker"},
    {"mode": "replay"}, {"status": "offline"},
    {"preview": {"format": "jpeg", "data_b64": "invalid"}},
])
def test_metadata_refresh_rejects_data_or_identity_changes_atomically(preview, change):
    before = preview.device_snapshot
    with pytest.raises(ValueError):
        preview.refresh_metadata({**before, **change})
    assert preview.device_snapshot == before
    assert preview.has_frame and not preview.held and not preview.preview_error


def test_clear_frame_removes_held_error_and_renderer_cache(preview, monkeypatch):
    preview.set_device({**preview.device_snapshot, "preview": {"format": "jpeg", "data_b64": "bad"}})
    assert preview.held and preview.preview_error
    refreshed = []
    for name in ("_refresh_camera_gpu", "_refresh_tactile_3d", "_refresh_glove_3d", "update"):
        monkeypatch.setattr(preview, name, lambda name=name: refreshed.append(name))
    identity, mode = preview.identity, preview.mode
    preview.clear_frame()
    assert not preview.has_frame and not preview.held and not preview.preview_error
    assert preview._image is None and preview._encoded_image is None
    assert not preview._trail and preview._tracker_origin is None
    assert preview.identity == identity and preview.mode == mode
    assert set(refreshed) == {"_refresh_camera_gpu", "_refresh_tactile_3d", "_refresh_glove_3d", "update"}


def test_empty_replay_context_is_explicit_and_drops_old_live_frame(preview):
    preview.set_device(device("camera", {}, mode="replay", status="ready", recorded_count=0))
    assert preview.mode == "replay" and not preview.has_frame and not preview.held
    assert "未录制" in preview._footer_text()
    preview.set_device(device("camera", jpeg_preview()))
    assert preview.mode == "live" and preview.has_frame


def test_view_copy_preserves_data_and_rejects_other_identity(preview, qapp):
    other = PreviewWidget("camera")
    try:
        other.set_device(preview.device_snapshot)
        other._yaw, other._pitch, other._zoom = .7, -.2, 1.4
        frame = preview._frame
        preview.copy_view_state_from(other)
        assert (preview._yaw, preview._pitch, preview._zoom) == (.7, -.2, 1.4)
        assert preview._frame is frame
        other.set_device({**other.device_snapshot, "id": "other"})
        with pytest.raises(ValueError):
            preview.copy_view_state_from(other)
        assert (preview._yaw, preview._pitch, preview._zoom) == (.7, -.2, 1.4)
    finally:
        other.close()
        other.deleteLater()


def test_same_camera_display_copy_refreshes_metadata_without_decode_or_repaint(preview, monkeypatch):
    other = PreviewWidget("camera")
    try:
        other.copy_camera_tracker_state_from(preview)
        preview.set_device({**preview.device_snapshot, "receive_age_s": 4.0})

        def forbidden(*args, **kwargs):
            raise AssertionError("unchanged display must not repaint or decode")

        monkeypatch.setattr(other, "update", forbidden)
        monkeypatch.setattr(other, "_prepare", forbidden)
        other.copy_camera_tracker_state_from(preview)
        assert other.device_snapshot["receive_age_s"] == 4.0
        assert other._image.cacheKey() == preview._image.cacheKey()
    finally:
        other.close()
        other.deleteLater()
