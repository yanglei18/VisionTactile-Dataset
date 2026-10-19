"""Status polling must not trigger Qt style changes for unchanged colours."""
import pytest
from PySide6.QtCore import QEvent, QObject

from vt_capture_console.ui.main_window import DeviceCard


class StyleChanges(QObject):
    def __init__(self):
        super().__init__()
        self.count = 0

    def eventFilter(self, watched, event):
        if event.type() == QEvent.Type.StyleChange:
            self.count += 1
        return False


@pytest.fixture
def status_card(qapp):
    device = dict(id="camera", kind="camera", label="Camera", status="offline",
                  receive_hz=0., preview={})
    card = DeviceCard(device)
    card.state.ensurePolished()
    qapp.processEvents()
    changes = StyleChanges()
    card.state.installEventFilter(changes)
    yield card, device, changes
    card.close()
    card.deleteLater()
    qapp.processEvents()


def test_unchanged_status_refreshes_telemetry_without_qt_style_changes(status_card):
    card, device, changes = status_card
    for rate in range(30):
        card.update_device(dict(device, receive_hz=float(rate)))
    assert changes.count == 0
    assert "29.0 Hz" in card.metrics.text()
    assert card.state.text() == "● 离线"


def test_status_transition_restyles_once_and_repeated_refresh_does_not(status_card):
    card, device, changes = status_card
    for status, text, expected in (("online", "在线", 1), ("stale", "数据陈旧", 2), ("offline", "离线", 3)):
        for _ in range(10):
            card.update_device(dict(device, status=status))
        assert changes.count == expected
        assert card.state.text() == "● " + text


def test_distinct_unknown_status_values_share_the_same_style(status_card):
    card, device, changes = status_card
    for status in ("unknown", "unrecognized", "unknown"):
        card.update_device(dict(device, status=status))
    assert changes.count == 1
    assert card.state.text() == "● 等待数据"


def test_embedded_card_keeps_replay_context_for_empty_inputs(qapp):
    device = dict(id="camera", kind="camera", label="Camera", status="unknown",
                  receive_hz=0., preview={})
    card = DeviceCard(device, embedded_replay=True)
    try:
        assert card.preview.mode == "replay"
        card.update_device(dict(device, recorded_count=0))
        card.update_metadata(dict(device, recorded_count=0, sample_age_s=1.))
        assert card.preview.mode == "replay"
        assert card.device_snapshot["mode"] == "replay"
        assert "mode" not in device
    finally:
        card.close()


def test_card_metadata_rejects_a_different_sample_without_partial_update(qapp):
    device = dict(id="camera", kind="camera", label="Camera", status="unknown",
                  receive_hz=0., preview={})
    card = DeviceCard(device)
    before = card.device_snapshot
    try:
        with pytest.raises(ValueError):
            card.update_metadata(dict(device, id="another-camera", label="Another camera"))
        assert card.device_snapshot == before
        assert card.preview.device_snapshot == before
        copied = card.device_snapshot
        copied["preview"]["demo"] = True
        assert card.device_snapshot == before
    finally:
        card.close()
