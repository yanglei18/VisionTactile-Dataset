"""Offscreen workbench routing, with in-memory manager and mocked focus only."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import subprocess
import time

import pytest
from PySide6.QtCore import QCoreApplication, QEvent

from vt_capture_console import i18n
from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui import main_window as module
from vt_capture_console.ui.desktop_tools import DesktopTools
from vt_capture_console.ui.main_window import MainWindow


class Backend:
    mode = "live"
    path = "/test-only/never-connected.sock"
    supports_device_control = True

    def __init__(self, tmp_path):
        demo = DemoBackend()
        self.value = demo.snapshot()
        self.value.update(mode="live", instance_id="manager-test")
        self.value["capabilities"]["device_control"] = True
        self.value["process_control"] = demo.process_control
        self.value["management"] = dict(enabled=False, controller=None, batches=[], trials=[], operation=None)
        self.commands, self.latest_operation, self.closed = [], None, False
        profiles = []
        for group in self.value["process_control"]["groups"]:
            if group["kind"] != "glove":
                continue
            hand = "left" if "/gloves/left/state" in group["topics"] else "right"
            group.update(state="STOPPED", owned=False, pid=None, can_start=True, can_stop=False, can_restart=False)
            profiles.append(dict(id=group["id"], preset="glove", enabled=True,
                                 parameters=dict(hand=hand, launch_gui="true")))
        for device in self.value["devices"]:
            if device["kind"] == "glove":
                device.update(status="offline", receive_hz=0., valid_hz=None, age_s=None, preview={})
        self.runtime = SimpleNamespace(configuration=dict(process_groups=profiles), status=dict(state="ready", detail="synthetic"),
            repo_root=Path(__file__).resolve().parents[3], state_dir=tmp_path, log_path=None)

    def snapshot(self):
        return deepcopy(self.value)

    def submit(self, command, params):
        request = f"test-device-{len(self.commands)}"
        self.commands.append((command, deepcopy(params)))
        self.latest_operation = dict(request_id=request, state="unknown", code="QUEUED", detail="synthetic queue", trial_id="")
        return request

    def close(self):
        self.closed = True


def group(backend, hand="right"):
    return next(g for g in backend.value["process_control"]["groups"]
                if g["kind"] == "glove" and f"/gloves/{hand}/state" in g["topics"])


def stream(backend, hand="right"):
    return next(d for d in backend.value["devices"] if d["kind"] == "glove" and d["side"] == hand)


def card(window, backend, hand="right"):
    return window.cards[stream(backend, hand)["id"]]


@pytest.fixture
def workbench(qapp, tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Hand preview must not spawn or open hardware")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    backend = Backend(tmp_path)
    backend.value["hand_sessions"] = {
        hand: dict(hand=hand, group_id=group(backend, hand)["id"], resources="",
                   control_socket="", tactile_socket="", service_instance_id="", detail="Managed hand")
        for hand in ("right", "left")}
    backend.operation_result = lambda request: backend.latest_operation
    window = MainWindow(backend)
    window.show()
    window.refresh()
    qapp.processEvents()
    yield window, backend
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


@pytest.mark.parametrize("hand", ["right", "left"])
def test_calibration_opens_embedded_panel_without_starting_hardware(workbench, hand):
    window, backend = workbench
    pages = window.pages.count()
    card(window, backend, hand).glove_controls.calibration_button.click()
    pane = window.hand_panes[hand]
    assert window.preview_stack.currentWidget() is pane
    assert pane._calibration_visible
    assert window.pages.count() == pages
    assert not pane.isWindow()
    assert not backend.commands
    assert "设备配置" in pane.notice.text()


def test_hand_selection_and_expand_share_one_panel(workbench):
    window, backend = workbench
    target = card(window, backend)
    window._select(target.identifier)
    first = window.hand_panes["right"]
    target.open_detail()
    window.open_glove_calibration("right")
    assert window.hand_panes["right"] is first
    assert target.glove_dialog is None
    assert backend.commands == []


def test_explicit_start_uses_manager_profile_and_instance(workbench):
    window, backend = workbench
    window.open_glove_calibration("left")
    pane = window.hand_panes["left"]
    pane._toggle_device("imu")
    assert backend.commands == [("device.start", dict(group_id=group(backend, "left")["id"],
                                                        expected_manager_instance_id="manager-test"))]
    with pytest.raises(RuntimeError):
        pane._toggle_device("imu")
    assert len(backend.commands) == 1


def test_recording_guard_and_independent_hand_lifecycle(workbench):
    window, backend = workbench
    window.open_glove_calibration("left")
    window.open_glove_calibration("right")
    group(backend, "right").update(can_start=False, can_stop=False)
    window.refresh()
    with pytest.raises(RuntimeError):
        window.hand_panes["right"]._toggle_device("imu")
    window.hand_panes["left"]._toggle_device("imu")
    assert backend.commands[0][1]["group_id"] == group(backend, "left")["id"]


def test_switching_to_camera_restores_existing_preview(workbench):
    window, backend = workbench
    window.open_glove_calibration("left")
    camera = next(d for d in backend.value["devices"] if d["kind"] == "camera")
    window._select(camera["id"])
    assert window.preview_stack.currentWidget() is window.capture_stage
    assert window.capture_stage._device["id"] == camera["id"]
    assert not backend.commands


def test_close_detaches_both_hands_without_device_stop(workbench):
    window, backend = workbench
    for hand in ("left", "right"):
        window.open_glove_calibration(hand)
    window.close()
    assert all(pane._closed for pane in window.hand_panes.values())
    assert backend.closed and not backend.commands


def test_tools_menu_routes_to_embedded_hand_panel(workbench):
    window, backend = workbench
    tools = DesktopTools(window, backend, backend.runtime)
    tools.left_glove_action.trigger()
    assert window.preview_stack.currentWidget() is window.hand_panes["left"]
    tools.right_glove_action.trigger()
    assert window.preview_stack.currentWidget() is window.hand_panes["right"]
    assert not backend.commands


def test_demo_does_not_create_remote_hand_clients(qapp, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Demo must not launch hardware tooling")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    window = MainWindow(DemoBackend())
    try:
        window.open_glove_calibration("right")
        assert window.hand_panes == {}
    finally:
        window.close()
