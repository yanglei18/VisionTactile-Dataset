"""Work themes keep actions local while global recording status remains truthful."""

from copy import deepcopy

import pytest
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QFrame

from test_managed_workflow import ManagedBackend
from vt_capture_console.ui.main_window import MainWindow


@pytest.fixture
def managed_window(qapp):
    backend = ManagedBackend()
    window = MainWindow(backend)
    window.motion_toggle.setChecked(False)
    window.timer.stop()
    window.show()
    qapp.processEvents()
    yield window, backend
    window.close()
    qapp.processEvents()


def test_form_actions_and_archive_have_distinct_page_ownership(window):
    assert window.findChild(QFrame, "recordingBar") is None
    assert window.pages.widget(0) is window.preview_page
    assert window.pages.widget(1) is window.archive_page
    assert window.pages.widget(2) is window.device_page
    assert window.pages.widget(3) is window.spatial_page
    assert window.pages.widget(4) is window.recording_page
    for control in (window.batch_selector, window.batch_name, window.create_batch_button,
                    window.trial_name, window.task_name, window.operator_name,
                    window.duration, window.start_button, window.stop_button):
        assert window.recording_page.isAncestorOf(control)
        assert not window.archive_page.isAncestorOf(control)
    for control in (window.trial_table, window.trial_details, window.inspect_trial_button):
        assert window.archive_page.isAncestorOf(control)
        assert not window.recording_page.isAncestorOf(control)


def test_recording_indicator_tracks_backend_across_pages_and_returns_to_stop(window, qapp):
    window.motion_toggle.setChecked(False)
    window.timer.stop()
    window.navigation.button(4).click()
    window.duration.setValue(60)
    qapp.processEvents()
    buttons = window.navigation.buttons()
    titles = [button.text() for button in buttons]
    geometry = [button.geometry() for button in buttons]
    QTest.mouseClick(window.start_button, Qt.MouseButton.LeftButton)
    identifier = window.backend.trials[0]["id"]
    window.backend.clock = lambda: window.backend.started + 12.
    for index in (2, 0, 3, 1):
        QTest.mouseClick(window.navigation.button(index), Qt.MouseButton.LeftButton)
        window.refresh()
        qapp.processEvents()
        assert window.backend.state == "RECORDING"
        assert window.backend.trials[0]["id"] == identifier
        assert not window.start_button.isVisible() and not window.stop_button.isVisible()
        assert not window.operation_notice.isVisible()
        assert "00:12" in window.recording_nav_status.text()
        assert "录制中" in window.navigation.button(4).toolTip()
        assert [button.text() for button in buttons] == titles
        assert [button.geometry() for button in buttons] == geometry
    QTest.mouseClick(window.navigation.button(4), Qt.MouseButton.LeftButton)
    assert window.stop_button.isVisible() and window.stop_button.isEnabled()
    QTest.mouseClick(window.stop_button, Qt.MouseButton.LeftButton)
    assert window.backend.state == "COMPLETE"
    assert "00:12" not in window.recording_nav_status.text()
    qapp.processEvents()
    assert [button.text() for button in buttons] == titles
    assert [button.geometry() for button in buttons] == geometry
    window.navigation.button(1).click()
    assert window.trial_table.rowCount() == 1
    assert window.trial_table.isVisible() and not window.batch_selector.isVisible()


def test_queued_start_indicator_is_not_recording_confirmation(managed_window):
    window, backend = managed_window
    window.navigation.button(4).click()
    QTest.mouseClick(window.start_button, Qt.MouseButton.LeftButton)
    request = window.pending_request
    assert request and backend.value["recording"]["state"] != "RECORDING"
    assert "开始中" in window.recording_nav_status.text()
    for index in (0, 3, 1, 2, 4):
        window.navigation.button(index).click()
        window.refresh()
        assert window.pending_request == request
        assert len(backend.commands) == 1
        assert "00:" not in window.recording_nav_status.text()
        assert not window.start_button.isEnabled()
    backend.confirm_recording()
    window.refresh()
    assert not window.pending_request
    assert "00:01" in window.recording_nav_status.text()
    assert window.stop_button.isEnabled()


def test_stop_indicator_waits_for_confirmation_while_other_pages_are_open(managed_window):
    window, backend = managed_window
    backend.latest_operation = dict(request_id="old", state="accepted", code="started", trial_id="t1", detail="")
    backend.confirm_recording()
    window.refresh()
    window.navigation.button(4).click()
    QTest.mouseClick(window.stop_button, Qt.MouseButton.LeftButton)
    assert backend.commands[-1][1] == "capture.stop"
    request = window.pending_request
    assert request
    window.navigation.button(0).click()
    window.refresh()
    assert "结束中" in window.recording_nav_status.text()
    assert "已结束" not in window.navigation.button(4).toolTip()
    assert backend.value["recording"]["state"] == "RECORDING"
    assert not window.start_button.isEnabled() and not window.stop_button.isEnabled()

    backend.value["recording"].update(state="COMPLETE")
    backend.value["management"]["trials"][0].update(state="COMPLETE")
    backend.value["management"]["controller"].update(state=5, recorder_alive=False)
    backend.latest_operation.update(state="accepted", code="stopped", trial_id="t1")
    window.refresh()
    assert not window.pending_request
    assert "结束中" not in window.recording_nav_status.text()
    assert "已结束" in window.navigation.button(4).toolTip()
    assert window.pages.currentWidget() is window.preview_page
    assert len(backend.commands) == 1


def test_unavailable_recording_status_never_claims_completion_or_sends_commands(managed_window):
    window, backend = managed_window
    backend.latest_operation = dict(request_id="old", state="accepted", code="started", trial_id="t1", detail="")
    backend.confirm_recording()
    window.refresh()
    saved_trials = deepcopy(backend.value["management"]["trials"])
    backend.value["recording"].update(state="UNAVAILABLE", detail="连接中断，录制状态待核实")
    backend.value["capabilities"]["record"] = False
    backend.value["management"]["controller"] = None
    for index in (2, 0, 3, 1, 4):
        window.navigation.button(index).click()
        window.refresh()
        assert "待核实" in window.recording_nav_status.text()
        assert "已结束" not in window.navigation.button(4).toolTip()
        assert not window.start_button.isEnabled() and not window.stop_button.isEnabled()
    assert backend.commands == []
    assert backend.value["management"]["trials"] == saved_trials


def test_idle_clock_previews_duration_without_overwriting_confirmed_session(window):
    window.timer.stop()
    window.navigation.button(4).click()
    window.duration.setValue(300)
    window.refresh()
    assert window.elapsed.text() == "00:00 / 05:00"
    assert window.backend.duration == 30.  # Editing presentation does not start a session.
    window.start_button.click()
    window.backend.clock = lambda: window.backend.started + 12.
    window.duration.setValue(10)  # Even an external UI edit cannot rewrite the active session.
    window.refresh()
    assert window.elapsed.text() == "00:12 / 05:00"
    assert window.backend.duration == 300.
