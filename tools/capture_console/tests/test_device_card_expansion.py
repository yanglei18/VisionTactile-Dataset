"""All card expansion routes share existing demo snapshots, never devices."""
from copy import deepcopy

import pytest
from PySide6.QtCore import QCoreApplication, Qt, QTimer
from PySide6.QtTest import QTest


CASES = (
    ("camera", "d405_1", "preview_dialog", "PreviewDetailDialog"),
    ("tracker", "left_wrist", "preview_dialog", "PreviewDetailDialog"),
    ("glove", "right_glove", "glove_dialog", "GloveDetailDialog"),
    ("tactile", "right_tactile", "tactile_dialog", "TactileDetailDialog"),
)


@pytest.fixture
def display_window(window, monkeypatch, qapp):
    # The existing fixture supplies the in-memory DemoBackend, not live IPC.
    window.timer.stop()
    window.motion_toggle.setChecked(False)
    assert not window.scroll.isVisible()
    window.streams_toggle.click()
    qapp.processEvents()
    assert window.scroll.isVisible()
    assert window.backend.mode == "demo"
    commands = []

    def command(*args, **kwargs):
        commands.append((args, kwargs))
        pytest.fail("Opening/closing preview windows must not submit backend commands")

    monkeypatch.setattr(window.backend, "submit", command)
    window._expansion_test_commands = commands
    return window


def click(button):
    QTest.mouseClick(button, Qt.MouseButton.LeftButton)


def open_case(window, case):
    kind, identifier, attribute, expected = case
    card = window.cards[identifier]
    window.scroll.ensureWidgetVisible(card)
    QCoreApplication.processEvents()
    click(card.expand_button)
    dialog = getattr(card, attribute)
    assert dialog is not None and type(dialog).__name__ == expected
    assert dialog.isVisible() and dialog.preview.kind == kind
    return card, dialog


@pytest.mark.parametrize("case", CASES, ids=lambda case: case[0])
def test_title_expand_routes_each_kind_to_its_existing_or_generic_detail(display_window, case):
    card, dialog = open_case(display_window, case)
    assert card.expand_button.isVisible() and card.expand_button.isEnabled()
    assert card.expand_button.accessibleName()
    assert dialog.preview._frame == card.preview._frame
    assert dialog.preview._mode == card.preview._mode == "demo"
    active_slots = [getattr(card, name, None) for name in ("preview_dialog", "glove_dialog", "tactile_dialog")]
    assert sum(value is not None for value in active_slots) == 1
    if case[0] == "glove":
        assert card.expand_button is card.glove_controls.expand_button
    elif case[0] == "tactile":
        assert card.expand_button is card.tactile_controls.expand_button
    assert not display_window._expansion_test_commands


@pytest.mark.parametrize("case", CASES, ids=lambda case: case[0])
def test_repeated_expand_reuses_single_dialog_and_latest_snapshot(display_window, case):
    card, dialog = open_case(display_window, case)
    click(card.expand_button)
    card.open_detail()
    assert getattr(card, case[2]) is dialog
    assert dialog.findChildren(QTimer) == []
    before = deepcopy(dialog.preview._frame)
    display_window.backend.clock = lambda: display_window.backend.origin + 2.
    display_window.refresh()
    assert getattr(card, case[2]) is dialog
    assert dialog.preview._frame == card.preview._frame
    assert dialog.preview._frame != before
    assert dialog.preview._device["id"] == card._device["id"]


@pytest.mark.parametrize("case", CASES, ids=lambda case: case[0])
def test_closing_large_preview_sends_no_command_and_refresh_does_not_reopen(display_window, qapp, case):
    card, dialog = open_case(display_window, case)
    dialog.close()
    assert getattr(card, case[2]) is None
    qapp.processEvents()
    display_window.backend.clock = lambda: display_window.backend.origin + 4.
    display_window.refresh()
    display_window.refresh()
    assert getattr(card, case[2]) is None
    assert not display_window._expansion_test_commands
    click(card.expand_button)
    replacement = getattr(card, case[2])
    assert replacement is not None and replacement is not dialog
    assert replacement.preview._frame == card.preview._frame


@pytest.mark.parametrize("case", CASES, ids=lambda case: case[0])
def test_filtered_card_detail_keeps_single_snapshot_updates_without_recreation(display_window, monkeypatch, case):
    card, dialog = open_case(display_window, case)
    display_window.filter.setCurrentIndex(3 if case[0] == "camera" else 1)
    display_window.refresh()
    assert not card.isVisible() and dialog.isVisible()
    calls = []
    snapshot = display_window.backend.snapshot

    def counted():
        calls.append(True)
        return snapshot()

    monkeypatch.setattr(display_window.backend, "snapshot", counted)
    display_window.backend.clock = lambda: display_window.backend.origin + 3.
    display_window.refresh()
    assert len(calls) == 1
    assert getattr(card, case[2]) is dialog
    assert dialog.preview._frame == card.preview._frame
    assert card.preview._device["preview"]["phase"] == 3.
    display_window.grid_signature = None  # Force a grid rebuild, not new data acquisition.
    display_window.refresh()
    assert len(calls) == 2
    assert display_window.cards[case[1]] is card and getattr(card, case[2]) is dialog
    assert dialog.isVisible()


@pytest.mark.parametrize("case", CASES, ids=lambda case: case[0])
def test_device_removal_closes_correct_dialog_even_while_card_filtered_out(display_window, monkeypatch, case):
    card, dialog = open_case(display_window, case)
    display_window.filter.setCurrentIndex(3 if case[0] == "camera" else 1)
    snapshot = display_window.backend.snapshot

    def removed():
        value = snapshot()
        value["devices"] = [device for device in value["devices"] if device["id"] != case[1]]
        return value

    monkeypatch.setattr(display_window.backend, "snapshot", removed)
    display_window.refresh()
    assert case[1] not in display_window.cards
    assert getattr(card, case[2]) is None
    assert not dialog.isVisible()
    assert not display_window._expansion_test_commands


@pytest.mark.parametrize("case", CASES, ids=lambda case: case[0])
def test_identity_replacement_closes_old_detail_without_automatic_reopen(display_window, case):
    card, dialog = open_case(display_window, case)
    replaced = deepcopy(card._device)
    replaced["id"] = case[1] + "-new-source"
    card.update_device(replaced)
    assert getattr(card, case[2]) is None and not dialog.isVisible()
    assert card.preview._device["id"] == replaced["id"]
    assert not display_window._expansion_test_commands


def test_close_details_and_main_window_close_clean_all_three_dialog_slots(display_window):
    opened = [open_case(display_window, case) for case in CASES]
    first, dialog = opened[0]
    first.close_details()
    assert first.preview_dialog is None and not dialog.isVisible()
    first.close_details()  # Safe when already closed; no replacement window.
    display_window.close()
    assert not display_window.timer.isActive()
    for card, dialog in opened:
        assert card.preview_dialog is card.glove_dialog is card.tactile_dialog is None
    assert not display_window._expansion_test_commands


def test_tracker_double_click_keeps_reset_semantics_and_does_not_expand(display_window):
    card = display_window.cards["left_wrist"]
    card.preview._yaw, card.preview._pitch, card.preview._zoom = 1.1, -.6, 1.7
    QTest.mouseDClick(card.preview, Qt.MouseButton.LeftButton)
    assert (card.preview._yaw, card.preview._pitch, card.preview._zoom) == (-.24, .32, 1.)
    assert card.preview_dialog is None
    click(card.expand_button)
    dialog = card.preview_dialog
    dialog.preview._yaw, dialog.preview._pitch, dialog.preview._zoom = .8, -.8, 1.5
    QTest.mouseDClick(dialog.preview, Qt.MouseButton.LeftButton)
    assert (dialog.preview._yaw, dialog.preview._pitch, dialog.preview._zoom) == (-.24, .32, 1.)
    assert card.preview_dialog is dialog
    assert not display_window._expansion_test_commands


def test_camera_double_click_opens_same_dialog_as_explicit_title_button(display_window):
    card = display_window.cards["d405_1"]
    QTest.mouseDClick(card.preview, Qt.MouseButton.LeftButton)
    first = card.preview_dialog
    assert first is not None and first.preview.kind == "camera"
    click(card.expand_button)
    assert card.preview_dialog is first
    assert not display_window._expansion_test_commands
