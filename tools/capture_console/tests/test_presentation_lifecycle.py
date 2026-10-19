"""User motion preferences survive restarts and background presentation pauses."""

import pytest
from PySide6.QtCore import QCoreApplication, QEvent
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QWidget

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.main_window import MainWindow


def test_new_window_defaults_to_quiet_presentation(window):
    assert not window.motion_toggle.isChecked()
    assert not window.ambient._timer.isActive()
    assert window.timer.isActive()


def test_motion_choice_survives_close_and_restart(qapp):
    first = MainWindow(DemoBackend())
    first.show()
    qapp.processEvents()
    first.motion_toggle.setChecked(True)
    first.close()
    qapp.processEvents()

    second = MainWindow(DemoBackend())
    try:
        assert second.motion_toggle.isChecked()
        second.show()
        qapp.processEvents()
        assert second.ambient._timer.isActive()
        second.motion_toggle.setChecked(False)
    finally:
        second.close()
    third = MainWindow(DemoBackend())
    try:
        assert not third.motion_toggle.isChecked()
    finally:
        third.close()


def test_background_preserves_recording_and_motion_preference(window, qapp):
    window.motion_toggle.setChecked(True)
    window.navigation.button(4).click()
    window.start_button.click()
    assert window.backend.state == "RECORDING"
    other = QWidget()
    try:
        other.show()
        other.activateWindow()
        qapp.processEvents()
        assert window.motion_toggle.isChecked()
        assert not window.ambient._timer.isActive()
        assert not window.spatial_page.scene._motion_enabled
        assert window.timer.isActive() and window.timer.interval() >= 500
        assert window.backend.state == "RECORDING"
        window.backend.clock = lambda: window.backend.origin + 2.
        window.refresh()
        assert window.last_snapshot["recording"]["state"] == "RECORDING"
        window.activateWindow()
        qapp.processEvents()
        assert window.motion_toggle.isChecked() and window.ambient._timer.isActive()
        assert window.timer.interval() == 100
        assert window.backend.state == "RECORDING"
    finally:
        other.close()
        window.stop_button.click()


def test_minimize_and_restore_never_reenable_disabled_motion(window, qapp):
    window.motion_toggle.setChecked(False)
    window.showMinimized()
    qapp.processEvents()
    assert not window.ambient._timer.isActive()
    assert window.timer.isActive() and window.timer.interval() >= 500
    window.showNormal()
    window.activateWindow()
    qapp.processEvents()
    assert not window.motion_toggle.isChecked()
    assert not window.ambient._timer.isActive()
    assert window.timer.interval() == 100


def _settle_windows(qapp):
    qapp.processEvents()
    # Window events coalesce a cadence update for the next event-loop turn.
    QTest.qWait(1)


def _open_data_window(window, kind):
    if kind == "spatial":
        window.spatial_page.open_detail()
        return window.spatial_page.detail_window
    card = next(card for card in window.cards.values() if card._device["kind"] == kind)
    card.open_detail()
    return card.preview_dialog or card.glove_dialog or card.tactile_dialog


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile", "spatial"])
def test_active_detail_keeps_shared_data_cadence_and_main_motion_paused(window, qapp, kind):
    window.motion_toggle.setChecked(True)
    window.backend.start_recording(300)
    detail = _open_data_window(window, kind)
    _settle_windows(qapp)
    assert qapp.activeWindow() is detail
    assert window.timer.interval() == 100
    assert not window.ambient._timer.isActive()
    assert not window.spatial_page.scene._motion_enabled
    assert window.motion_toggle.isChecked()

    # Verify real timer delivery, beyond merely inspecting the configured value.
    ticks = []
    window.timer.timeout.connect(lambda: ticks.append(True))
    QTest.qWait(450)
    assert len(ticks) >= 2
    assert window.backend.state == "RECORDING"

    other = QWidget()
    try:
        other.show()
        other.activateWindow()
        _settle_windows(qapp)
        assert window.timer.interval() >= 500
        assert not window.ambient._timer.isActive()
        detail.activateWindow()
        _settle_windows(qapp)
        assert window.timer.interval() == 100
        detail.showMinimized()
        _settle_windows(qapp)
        assert window.timer.interval() >= 500
        detail.showNormal()
        detail.activateWindow()
        _settle_windows(qapp)
        assert window.timer.interval() == 100
        assert not window.ambient._timer.isActive()
        assert window.backend.state == "RECORDING"
    finally:
        other.close()
        detail.close()
        window.backend.stop_recording()
    window.activateWindow()
    _settle_windows(qapp)
    assert window.timer.interval() == 100
    assert window.ambient._timer.isActive()


def test_switching_and_hiding_details_releases_fast_cadence(window, qapp):
    camera = _open_data_window(window, "camera")
    tracker = _open_data_window(window, "tracker")
    other = QWidget()
    try:
        _settle_windows(qapp)
        assert window.timer.interval() == 100
        camera.activateWindow()
        _settle_windows(qapp)
        assert window.timer.interval() == 100
        # A visible but unfocused detail must not defeat background throttling.
        other.show()
        other.activateWindow()
        _settle_windows(qapp)
        assert camera.isVisible() and tracker.isVisible()
        assert window.timer.interval() >= 500
        camera.close()
        tracker.hide()
        _settle_windows(qapp)
        assert window.timer.interval() >= 500
        tracker.show()
        tracker.activateWindow()
        _settle_windows(qapp)
        assert window.timer.interval() == 100
    finally:
        tracker.close()
        other.close()


def test_unrelated_child_dialog_does_not_keep_live_polling_fast(window, qapp):
    from PySide6.QtWidgets import QDialog
    settings = QDialog(window)
    try:
        settings.show()
        settings.activateWindow()
        _settle_windows(qapp)
        assert window.timer.interval() >= 500
    finally:
        settings.close()


def test_close_removes_application_event_filter_and_pending_sync(qapp, monkeypatch):
    from shiboken6 import isValid
    first = MainWindow(DemoBackend())
    first.show()
    detail = _open_data_window(first, "camera")
    _settle_windows(qapp)
    first.close()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    assert not isValid(detail)
    assert not first._presentation_sync_timer.isActive()
    assert not first.timer.isActive()

    # Keep the closed Python object alive; no application-wide filter or queued
    # callback should reach it when a new console's windows gain/lose focus.
    calls = []
    monkeypatch.setattr(first, "_sync_presentation", lambda: calls.append(True))
    events = []
    original_filter = first.eventFilter
    monkeypatch.setattr(first, "eventFilter", lambda watched, event: (
        events.append(event.type()), original_filter(watched, event))[1])
    second = MainWindow(DemoBackend())
    try:
        second.show()
        _open_data_window(second, "tracker")
        _settle_windows(qapp)
        assert second.timer.interval() == 100
        assert not calls and not events
    finally:
        second.close()
