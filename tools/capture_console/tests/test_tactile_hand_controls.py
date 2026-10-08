"""Offscreen hand-view controls reuse IPC display state, never live hardware."""

import pytest
from PySide6.QtCore import QCoreApplication, QEvent, QPoint, QRect, QTimer, Qt
from PySide6.QtGui import QFont
from PySide6.QtTest import QTest
from shiboken6 import isValid

from vt_capture_console.ui.main_window import DeviceCard


def tactile_device(side="right", value=100., **changes):
    result = {
        "id": f"{side}_tactile", "kind": "tactile", "side": side,
        "label": "右手 · 触觉" if side == "right" else "左手 · 触觉",
        "topic": f"/gloves/{side}/tactile/state", "status": "online",
        "receive_hz": 30., "valid_hz": None, "age_s": .01,
        "preview": {"rows": 20, "cols": 20, "values": [value] * 400,
                    "valid_mask": [True] * 400, "levels": [0., 1000.]},
    }
    result.update(changes)
    return result


@pytest.fixture
def cards(qapp):
    created = []

    def create(device):
        card = DeviceCard(device)
        card.resize(260, card.sizeHint().height())
        card.show()
        created.append(card)
        qapp.processEvents()
        return card

    yield create
    for card in created:
        card.close_tactile_detail()
        card.close()
        card.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


def click(button):
    QTest.mouseClick(button, Qt.MouseButton.LeftButton)


def test_tactile_card_defaults_and_view_switch_do_not_change_values(cards):
    right = cards(tactile_device())
    left = cards(tactile_device("left"))
    assert right.preview.tactile_view == "hand"
    assert right.tactile_controls.hand_button.isChecked()
    assert right.preview.height() == 256
    assert "候选映射" in right.tactile_note.text()
    assert "非压力" in right.tactile_note.text()
    assert left.preview.tactile_view == "matrix"
    assert not left.tactile_controls.hand_button.isEnabled()
    assert left.tactile_controls.matrix_button.isChecked()
    assert "未映射" in left.tactile_controls.hand_button.toolTip()

    original = right.preview._frame
    for control, mode in ((right.tactile_controls.matrix_button, "matrix"),
                          (right.tactile_controls.hand_button, "hand")):
        click(control)
        assert right.preview.tactile_view == mode
        assert right.preview._frame is original
        assert left.preview.tactile_view == "matrix"


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove"])
def test_non_tactile_cards_keep_their_existing_layout(cards, kind):
    card = cards(tactile_device(kind=kind, preview={}))
    assert card.tactile_controls is None
    assert card.preview.height() == (256 if kind == "glove" else 166)
    card.open_tactile_detail()
    assert card.tactile_dialog is None


def test_detail_is_singleton_and_switches_stay_in_sync(cards):
    card = cards(tactile_device())
    click(card.tactile_controls.expand_button)
    dialog = card.tactile_dialog
    assert dialog is not None and dialog.isVisible()
    assert dialog.preview.tactile_view == "hand"
    assert dialog.preview._frame == card.preview._frame
    assert dialog.preview._frame is not card.preview._frame
    assert "候选位置" in dialog.layout_note.text()
    assert "尚非逐点标定" in dialog.layout_note.text()
    assert dialog.layout_note.toolTip() == card.preview.tactile_layout_note
    assert dialog.width() >= 700 and dialog.height() >= 650
    click(card.tactile_controls.expand_button)
    assert card.tactile_dialog is dialog
    click(dialog.controls.matrix_button)
    assert dialog.preview.tactile_view == card.preview.tactile_view == "matrix"
    assert card.tactile_controls.matrix_button.isChecked()
    click(card.tactile_controls.hand_button)
    assert dialog.preview.tactile_view == card.preview.tactile_view == "hand"
    assert dialog.controls.hand_button.isChecked()
    assert dialog.findChildren(QTimer) == []


def test_real_theme_compact_controls_and_enlarged_canvas_fit(cards, qapp):
    from vt_capture_console.theme import stylesheet

    previous_style, previous_font = qapp.styleSheet(), QFont(qapp.font())
    try:
        qapp.setStyleSheet(stylesheet())
        font = QFont()
        font.setFamilies(["Inter", "Noto Sans", "Noto Sans CJK SC"])
        font.setPointSize(10)
        qapp.setFont(font)
        card = cards(tactile_device())
        card.resize(210, card.sizeHint().height())
        qapp.processEvents()
        for button in (card.tactile_controls.hand_button, card.tactile_controls.matrix_button,
                       card.tactile_controls.expand_button):
            bounds = QRect(button.mapTo(card, QPoint(0, 0)), button.size())
            assert card.rect().contains(bounds)
            assert button.width() >= button.fontMetrics().horizontalAdvance(button.text()) + 10
        card.open_tactile_detail()
        qapp.processEvents()
        dialog = card.tactile_dialog
        assert dialog.preview.width() >= 650 and dialog.preview.height() >= 400
        assert dialog.layout_note.height() >= dialog.layout_note.fontMetrics().height()
        assert dialog.preview.geometry().bottom() < dialog.layout_note.geometry().top()
    finally:
        qapp.setStyleSheet(previous_style)
        qapp.setFont(previous_font)


@pytest.mark.parametrize("mode", ["live", "demo", "replay"])
def test_opening_detail_offline_retains_history_mode_and_levels(cards, mode):
    device = tactile_device()
    if mode == "demo":
        device["preview"] = {"demo": True, "phase": .3}
    elif mode == "replay":
        device["preview"]["replay"] = True
    card = cards(device)
    original = card.preview._frame
    card.update_device({**device, "status": "offline", "preview": {}, "receive_hz": 0.})
    assert card.preview._held
    card.open_tactile_detail()
    dialog = card.tactile_dialog
    assert dialog.preview._frame == original
    assert dialog.preview._held
    assert dialog.preview._mode == mode
    assert dialog.preview._levels == card.preview._levels
    assert dialog.preview._cell_colors == card.preview._cell_colors
    click(dialog.controls.matrix_button)
    assert dialog.preview._held and card.preview._held
    card.update_device(device)
    assert not dialog.preview._held and not card.preview._held
    assert dialog.preview.tactile_view == card.preview.tactile_view == "matrix"


def test_empty_invalid_and_recovery_remain_consistent_in_detail(cards):
    card = cards(tactile_device(status="unknown", preview={}))
    card.open_tactile_detail()
    dialog = card.tactile_dialog
    assert dialog.preview._frame is None
    card.update_device(tactile_device(value=99.))
    original = card.preview._frame
    assert dialog.preview._frame == original
    card.update_device(tactile_device(preview={"values": [float("nan")] * 400,
                                                "valid_mask": [True] * 400}))
    assert dialog.preview._held and card.preview._held
    assert dialog.preview._frame == original
    assert dialog.preview._preview_error == card.preview._preview_error
    card.update_device(tactile_device(value=750.))
    assert not dialog.preview._held
    assert dialog.preview._frame["values"] == (750.,) * 400


def test_detail_cannot_mix_hands_and_identity_change_closes_it(cards):
    right = cards(tactile_device())
    left = cards(tactile_device("left", value=800.))
    right.open_tactile_detail()
    left.open_tactile_detail()
    dialog = right.tactile_dialog
    assert left.tactile_dialog.preview.tactile_view == "matrix"
    assert not left.tactile_dialog.controls.hand_button.isEnabled()
    assert "左手未映射" in left.tactile_dialog.layout_note.text()
    assert left.tactile_dialog.layout_note.toolTip() == left.preview.tactile_layout_note
    with pytest.raises(ValueError, match="设备或左右手"):
        dialog.update_preview(left.preview, left._device)
    assert dialog.preview._frame["values"] == (100.,) * 400
    assert left.tactile_dialog.preview._frame["values"] == (800.,) * 400
    right.update_device(tactile_device("left", id="right_tactile", preview={}))
    assert right.tactile_dialog is None
    assert right.preview._frame is None
    assert right.preview.tactile_view == "matrix"


def test_close_releases_detail_and_reopen_copies_latest_state(cards, qapp):
    card = cards(tactile_device())
    card.open_tactile_detail()
    old_dialog = card.tactile_dialog
    old_dialog.close()
    assert card.tactile_dialog is None
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()
    assert not isValid(old_dialog)
    card.update_device(tactile_device(value=600.))
    card.open_tactile_detail()
    assert card.tactile_dialog is not old_dialog
    assert card.tactile_dialog.preview._frame["values"] == (600.,) * 400


def test_filtered_detail_uses_single_existing_snapshot_and_removal_closes_it(window, monkeypatch):
    window.timer.stop()
    card = window.cards["right_tactile"]
    card.open_tactile_detail()
    dialog = card.tactile_dialog
    window.filter.setCurrentIndex(1)  # Camera cards only; keep detail open.
    window.backend.clock = lambda: window.backend.origin + 2.
    snapshot = window.backend.snapshot
    calls = []

    def counted_snapshot():
        calls.append(True)
        return snapshot()

    monkeypatch.setattr(window.backend, "snapshot", counted_snapshot)
    window.refresh()
    assert len(calls) == 1
    assert not card.isVisible() and dialog.isVisible()
    assert card.preview._device["preview"]["phase"] == 2.
    assert dialog.preview._frame == card.preview._frame
    window.backend.scenario = "stale"
    window.refresh()
    assert len(calls) == 2
    assert dialog.preview._held
    signature = window.grid_signature
    window.backend.hands = "none"
    window.refresh()
    assert window.grid_signature == signature
    assert "right_tactile" not in window.cards
    assert card.tactile_dialog is None
    assert not dialog.isVisible()


def test_main_window_closes_all_tactile_detail_windows(window):
    window.timer.stop()
    right, left = window.cards["right_tactile"], window.cards["left_tactile"]
    right.open_tactile_detail()
    left.open_tactile_detail()
    window.close()
    assert right.tactile_dialog is left.tactile_dialog is None
