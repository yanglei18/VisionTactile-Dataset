"""Dense and focused presentations preserve measured pixels and state labels."""
import pytest
from PySide6 import QtCore

from vt_capture_console.ui.widgets.preview import PreviewWidget
from test_preview_widgets import device, jpeg_preview, render


@pytest.mark.parametrize("presentation", ["thumbnail", "stage", "standard"])
def test_presentation_does_not_redecode_recolor_or_replace_camera_frame(qapp, monkeypatch, presentation):
    preview = PreviewWidget("camera")
    try:
        preview.set_device(device("camera", jpeg_preview()))
        frame, key = preview._frame, preview._image.cacheKey()
        expected_pixel = preview._image.pixelColor(32, 18)

        def forbidden(*args, **kwargs):
            pytest.fail("presentation changes must not prepare sensor data again")

        monkeypatch.setattr(preview, "_prepare_image", forbidden)
        preview.set_presentation(presentation)
        preview.resize(204, 104 if presentation == "thumbnail" else 300)
        assert preview._frame is frame and preview._image.cacheKey() == key
        image = render(preview)
        center = preview._canvas_rect().center().toPoint()
        assert image.pixelColor(center) == expected_pixel
        if presentation == "thumbnail":
            assert preview.height() == 104 and preview._canvas_rect().height() >= 80
    finally:
        preview.close()


@pytest.mark.parametrize("presentation", ["thumbnail", "stage"])
@pytest.mark.parametrize("mode", ["demo", "replay"])
def test_compact_chrome_keeps_mode_history_and_invalid_data_visible(qapp, presentation, mode):
    preview = PreviewWidget("camera")
    try:
        payload = {"demo": True, "phase": .2} if mode == "demo" else {**jpeg_preview(), "replay": True}
        selected = device("camera", payload)
        preview.set_device(selected)
        preview.set_presentation(presentation)
        assert ("演示" if mode == "demo" else "回放") in preview._footer_text()
        preview.set_device({**selected, "status": "offline", "preview": {}})
        assert preview._held and "历史保持" in preview._footer_text()
        assert not render(preview).isNull()
        preview.set_device({**selected, "preview": {"format": "jpeg", "data_b64": "invalid"}})
        assert preview._preview_error and "预览" in preview._footer_text()
        assert "无效" in preview._footer_text() or "不可用" in preview._footer_text()
    finally:
        preview.close()


def test_view_angles_and_tracker_cache_survive_switching_density(qapp):
    preview = PreviewWidget("tracker")
    try:
        preview.set_device(device("tracker", {"demo": True, "phase": .3}))
        preview._yaw, preview._pitch, preview._zoom = .7, -.5, 1.6
        frame, trail = preview._frame, preview._trail
        for presentation in ("thumbnail", "stage", "standard"):
            preview.set_presentation(presentation)
            assert preview._frame is frame and preview._trail is trail
            assert (preview._yaw, preview._pitch, preview._zoom) == (.7, -.5, 1.6)
        assert not preview.findChildren(QtCore.QTimer)
    finally:
        preview.close()
