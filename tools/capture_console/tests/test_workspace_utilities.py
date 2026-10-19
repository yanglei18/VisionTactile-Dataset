"""Secondary help/demo utilities leave the acquisition workspace usable."""

from copy import deepcopy

import pytest
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest


def forbid_commands(monkeypatch, backend):
    def forbidden(*args, **kwargs):
        pytest.fail("Help and demo presentation choices must not submit backend commands")

    monkeypatch.setattr(backend, "submit", forbidden)


def test_primary_navigation_has_one_label_per_work_theme(window):
    buttons = sorted(window.navigation.buttons(), key=lambda button: button.x())
    assert [button.text() for button in buttons] == ["设备", "预览", "空间", "录制", "档案"]
    assert [window.navigation.id(button) for button in buttons] == [2, 0, 3, 4, 1]
    assert window.pages.count() == 5
    assert window.pages.currentWidget() is window.preview_page
    assert window.help_button.isVisible()


def test_help_is_reused_and_does_not_interrupt_recording_or_navigation(window, qapp, monkeypatch):
    window.motion_toggle.setChecked(False)
    window.navigation.button(4).click()
    window.start_button.click()
    assert window.backend.state == "RECORDING"
    window.navigation.button(3).click()
    before = deepcopy(window.backend.trials)
    forbid_commands(monkeypatch, window.backend)

    QTest.mouseClick(window.help_button, Qt.MouseButton.LeftButton)
    qapp.processEvents()
    dialog = window.help_dialog
    assert dialog is not None and dialog.isVisible() and not dialog.isModal()
    assert window.pages.currentWidget() is window.spatial_page
    assert window.backend.state == "RECORDING" and window.backend.trials == before
    assert window.stop_button.isEnabled() and window.timer.isActive()

    window.help_button.click()
    assert window.help_dialog is dialog
    dialog.close()
    qapp.processEvents()
    assert not dialog.isVisible()
    window.help_button.click()
    qapp.processEvents()
    assert window.help_dialog is dialog and dialog.isVisible()
    assert window.pages.currentWidget() is window.spatial_page
    assert window.backend.state == "RECORDING" and window.backend.trials == before

    window.close()
    qapp.processEvents()
    assert not dialog.isVisible()
    assert window.backend.state == "RECORDING"


def test_demo_scenario_menu_refreshes_stream_and_spatial_states_without_commands(window, monkeypatch):
    window.timer.stop()
    window.motion_toggle.setChecked(False)
    forbid_commands(monkeypatch, window.backend)
    assert window.demo_menu is not None
    assert window.mode_badge.menu() is window.demo_menu
    assert set(window.scenario_actions) == {"normal", "stale", "offline"}
    assert all(action in window.demo_menu.actions() for action in window.scenario_actions.values())
    assert window.scenario_actions["normal"].isChecked()
    window._select("right_tactile")

    window.scenario_actions["stale"].trigger()
    assert window.backend.scenario == "stale"
    assert window.last_snapshot["devices"]
    assert window.devices["right_tactile"]["status"] == "stale"
    assert "陈旧" in window.cards["right_tactile"].state.text()
    assert window.capture_stage.source_preview is window.cards["right_tactile"].preview

    window.scenario_actions["offline"].trigger()
    assert window.backend.scenario == "offline"
    assert window.devices["d436"]["status"] == "offline"
    assert "离线" in window.cards["d436"].state.text()
    assert window.scene_backend.scenario == "offline"
    assert any(entity["state"] == "offline" for entity in window.spatial_page.model.frame["cameras"])
    assert sum(action.isChecked() for action in window.scenario_actions.values()) == 1

    window.scenario_actions["normal"].trigger()
    assert window.backend.scenario == "normal"
    assert all(device["status"] == "online" for device in window.devices.values())
    assert all(entity["state"] == "fresh" for entity in window.spatial_page.model.frame["cameras"])
    assert window.scenario_actions["normal"].isChecked()


@pytest.mark.parametrize("hands", ["right", "left", "none", "both"])
def test_demo_hands_menu_refreshes_streams_and_retains_recording(window, monkeypatch, hands):
    window.timer.stop()
    window.motion_toggle.setChecked(False)
    window.navigation.button(4).click()
    window.start_button.click()
    assert window.backend.state == "RECORDING"
    before = deepcopy(window.backend.trials)
    forbid_commands(monkeypatch, window.backend)
    window._select("right_glove")
    assert set(window.hands_actions) == {"both", "right", "left", "none"}
    assert all(action in window.demo_menu.actions() for action in window.hands_actions.values())

    window.hands_actions[hands].trigger()
    expected_sides = {"both": {"left", "right"}, "right": {"right"}, "left": {"left"}, "none": set()}[hands]
    assert window.backend.hands == hands
    actual_sides = {device["side"] for device in window.devices.values()
                    if device["kind"] in {"glove", "tactile"}}
    assert actual_sides == expected_sides
    assert len(window.devices) == len(window.cards) == 6 + 2 * len(expected_sides)
    assert window.selected_id in window.devices
    assert window.capture_stage.source_preview is window.cards[window.selected_id].preview
    assert window.backend.state == "RECORDING" and window.backend.trials == before
    assert window.stop_button.isEnabled()
    assert window.hands_actions[hands].isChecked()
    assert sum(action.isChecked() for action in window.hands_actions.values()) == 1


def test_live_workspace_has_help_without_demo_menu(qapp, monkeypatch):
    from vt_capture_console.backend.demo_backend import DemoBackend
    from vt_capture_console.backend.scene_demo import DemoSceneBackend
    from vt_capture_console.ui.main_window import MainWindow

    class LivePresentationBackend(DemoBackend):
        mode = "live"
        path = "/nonexistent/test-only/control.sock"

        def snapshot(self):
            value = super().snapshot()
            value["mode"] = "live"
            return value

    backend = LivePresentationBackend(clock=lambda: 12.)
    forbid_commands(monkeypatch, backend)
    window = MainWindow(backend, scene_backend=DemoSceneBackend())
    window.motion_toggle.setChecked(False)
    window.show()
    qapp.processEvents()
    try:
        assert not window.demo
        assert window.demo_menu is None
        assert window.pages.count() == 5
        window.help_button.click()
        qapp.processEvents()
        assert window.help_dialog.isVisible() and not window.help_dialog.isModal()
        assert window.pages.currentIndex() == 0
    finally:
        window.close()
        qapp.processEvents()
