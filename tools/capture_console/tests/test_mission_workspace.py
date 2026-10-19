"""Workspace selection shares card caches and preserves inspection state."""

from copy import deepcopy

import pytest
from PySide6.QtCore import QCoreApplication, QEvent, Qt
from PySide6.QtTest import QTest


def test_filter_selects_visible_stage_source_with_one_snapshot_per_refresh(window, monkeypatch):
    window.timer.stop()
    assert not window.scroll.isVisible()
    # Clicking the already selected category must also reveal its streams.
    window.segments.button(0).click()
    assert window.scroll.isVisible()
    snapshot = window.backend.snapshot
    polls = []

    def counted_snapshot():
        polls.append(True)
        return snapshot()

    monkeypatch.setattr(window.backend, "snapshot", counted_snapshot)
    for count, kind in enumerate(("tracker", "glove"), 1):
        window.segments.button(window.filter.findData(kind)).click()
        assert len(polls) == count - 1
        window.refresh()
        assert len(polls) == count
        selected = window.cards[window.selected_id]
        assert selected.isVisible() and selected._device["kind"] == kind
        assert any(window.grid.itemAt(index).widget() is selected
                   for index in range(window.grid.count()))
        assert window.capture_stage.source_preview is selected.preview
        assert window.capture_stage.preview._identity == selected.preview._identity
        assert window.capture_stage.preview._frame == selected.preview._frame


def test_clicking_another_stream_changes_stage_but_refresh_preserves_inspection(window, qapp):
    window.timer.stop()
    window.motion_toggle.setChecked(False)
    window.streams_toggle.click()
    window.filter.setCurrentIndex(window.filter.findData("tracker"))
    window.refresh()
    previous_source = window.capture_stage.source_preview
    target = next(window.grid.itemAt(index).widget() for index in range(window.grid.count())
                  if window.grid.itemAt(index).widget().identifier != window.selected_id)
    window.scroll.ensureWidgetVisible(target)
    qapp.processEvents()
    QTest.mouseClick(target.preview, Qt.MouseButton.LeftButton)
    assert window.selected_id == target.identifier
    assert window.capture_stage.source_preview is target.preview
    assert window.capture_stage.source_preview is not previous_source

    preview = window.capture_stage.preview
    before = deepcopy(preview._frame)
    preview._yaw, preview._pitch, preview._zoom = .85, -.2, 1.6
    window.backend.clock = lambda: window.backend.origin + 4.
    window.refresh()
    assert window.capture_stage.preview is preview
    assert window.capture_stage.source_preview is target.preview
    assert (preview._yaw, preview._pitch, preview._zoom) == (.85, -.2, 1.6)
    assert preview._frame == target.preview._frame and preview._frame != before


def test_removing_selected_stream_reselects_and_empty_devices_clear_stage(window, qapp, monkeypatch):
    window.timer.stop()
    window._select("d405_2")
    removed_id = window.selected_id
    old_preview = window.capture_stage.preview
    snapshot = window.backend.snapshot
    remove_all = False

    def changed_snapshot():
        value = snapshot()
        value["devices"] = ([] if remove_all else
                            [device for device in value["devices"] if device["id"] != removed_id])
        return value

    monkeypatch.setattr(window.backend, "snapshot", changed_snapshot)
    window.refresh()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()
    assert removed_id not in window.cards
    assert window.selected_id in window.devices and window.selected_id != removed_id
    selected = window.cards[window.selected_id]
    assert window.capture_stage.source_preview is selected.preview
    assert window.capture_stage.preview is not old_preview
    assert window.capture_stage.preview._identity == selected.preview._identity

    remove_all = True
    window.refresh()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()
    assert window.selected_id is None and not window.cards
    assert window.capture_stage.source_preview is None
    assert window.capture_stage.preview is None
    assert window.capture_stage.canvas.currentWidget() is window.capture_stage.empty
    assert not window.capture_stage.expand_button.isEnabled()


@pytest.mark.parametrize("identifier,attribute", [
    ("d405_1", "preview_dialog"),
    ("left_wrist", "preview_dialog"),
    ("right_glove", "glove_dialog"),
    ("right_tactile", "tactile_dialog"),
])
def test_stage_expansion_uses_existing_card_dialog_without_backend_commands(
        window, monkeypatch, identifier, attribute):
    window.timer.stop()

    def forbidden_command(*args, **kwargs):
        pytest.fail("Opening a stage preview must not submit backend commands")

    monkeypatch.setattr(window.backend, "submit", forbidden_command)
    window._select(identifier)
    card = window.cards[identifier]
    window.capture_stage.expand_button.click()
    dialog = getattr(card, attribute)
    assert dialog is not None and dialog.isVisible()
    assert dialog.preview._identity == card.preview._identity
    window.capture_stage.expand_requested.emit()
    assert getattr(card, attribute) is dialog
    dialog.close()
    assert getattr(card, attribute) is None
