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
        pytest.fail("Workbench action tests must not spawn any process or access hardware")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    focuses = []
    def focus(value, hand):
        focuses.append((deepcopy(value), hand))
        return True, "synthetic focus request; no actual desktop operation"
    monkeypatch.setattr(module, "focus_owned_calibration", focus)
    backend = Backend(tmp_path)
    window = MainWindow(backend)
    window.show()
    window.refresh()
    qapp.processEvents()
    yield window, backend, focuses
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


@pytest.mark.parametrize("hand", ["right", "left"])
def test_stopped_card_action_uses_device_controls_and_current_instance(workbench, hand):
    window, backend, focuses = workbench
    target = group(backend, hand)
    card(window, backend, hand).glove_controls.calibration_button.click()
    assert backend.commands == [("device.start", dict(group_id=target["id"], expected_manager_instance_id="manager-test"))]
    assert window.device_controls.selected_id == target["id"]
    assert window.device_controls.pending_request == "test-device-0"
    assert not focuses
    assert "已请求" in window.statusBar().currentMessage()


@pytest.mark.parametrize("state", ["STARTING", "RUNNING", "DEGRADED"])
def test_running_group_focuses_existing_window_without_duplicate_start(workbench, state):
    window, backend, focuses = workbench
    group(backend).update(state=state, owned=True, pid=1234, can_start=False)
    window.refresh()
    window.open_glove_calibration("right")
    window.open_glove_calibration("right")
    assert backend.commands == []
    assert len(focuses) == 2 and all(hand == "right" and g["pid"] == 1234 for g, hand in focuses)
    assert "synthetic focus" in window.statusBar().currentMessage()


def test_focus_unavailable_never_falls_back_to_new_acquisition(workbench, monkeypatch):
    window, backend, _ = workbench
    group(backend).update(state="DEGRADED", owned=True, pid=1234, can_start=False)
    monkeypatch.setattr(module, "focus_owned_calibration", lambda *_: (False, "请用 Alt+Tab 切换现有窗口"))
    window.open_glove_calibration("right")
    assert backend.commands == []
    assert "Alt+Tab" in window.statusBar().currentMessage()


def test_pending_device_operation_blocks_duplicate_and_other_hand(workbench):
    window, backend, focuses = workbench
    window.open_glove_calibration("right")
    first = window.device_controls.pending_request
    window.open_glove_calibration("right")
    window.open_glove_calibration("left")
    assert len(backend.commands) == 1 and window.device_controls.pending_request == first
    assert not focuses
    assert "已请求打开" not in window.statusBar().currentMessage()


def test_preexisting_pending_does_not_retarget_or_submit(workbench):
    window, backend, focuses = workbench
    controls = window.device_controls
    controls.pending_request = "different-device-operation"
    controls.pending_instance_id = backend.value["instance_id"]
    controls._submitted_at = time.monotonic()
    window.open_glove_calibration("right")
    assert backend.commands == [] and not focuses
    assert controls.pending_request == "different-device-operation"
    assert "已请求打开" not in window.statusBar().currentMessage()


@pytest.mark.parametrize("case", ["readonly", "disabled", "external", "unknown", "unowned_pid", "manager_disabled"])
def test_non_actionable_states_never_start_or_focus(workbench, case):
    window, backend, focuses = workbench
    target = group(backend)
    if case == "readonly":
        backend.value["capabilities"]["device_control"] = False
        backend.supports_device_control = False
    elif case == "manager_disabled":
        backend.value["process_control"]["enabled"] = False
    elif case == "unowned_pid":
        target.update(state="RUNNING", owned=False, pid=999, can_start=True)
    else:
        target.update(state={"disabled": "DISABLED", "external": "EXTERNAL", "unknown": "UNKNOWN"}[case], can_start=True)
    window.refresh()
    assert not card(window, backend).glove_controls.calibration_button.isEnabled()
    window.open_glove_calibration("right")  # Direct menu call must have the same guards.
    assert backend.commands == [] and not focuses


@pytest.mark.parametrize("language", ["zh_CN", "en"])
def test_blocked_glove_start_shows_serial_diagnostic_in_menu_and_card(workbench, language):
    window, backend, focuses = workbench
    path = "/dev/serial/by-id/用户设备-{right}"
    detail = f"串口 {path} 不存在；请检查 USB 连接、设备枚举和配置端口。"
    expected = detail if language == "zh_CN" else (
        f"Serial port {path} does not exist; check the USB connection, detected devices and configured port.")
    group(backend).update(state="FAILED", can_start=False, detail=detail)
    previous = i18n.language()
    try:
        i18n.set_language(language, persist=False)
        window.refresh()
        target = card(window, backend)
        assert not target.glove_controls.calibration_button.isEnabled()
        assert target.glove_note.toolTip().splitlines()[0] == expected
        window.open_glove_calibration("right")
        assert window.statusBar().currentMessage() == expected
        assert backend.commands == [] and not focuses
        assert group(backend)["detail"] == detail
    finally:
        i18n.set_language(previous, persist=False)


def test_disconnected_backend_property_blocks_cached_true_start_flags(workbench):
    window, backend, focuses = workbench
    backend.supports_device_control = False  # Cached snapshot still contains old true flags.
    window.open_glove_calibration("right")
    assert backend.commands == [] and not focuses
    assert "已请求打开" not in window.statusBar().currentMessage()


@pytest.mark.parametrize("configuration", [None, [], [dict(id="different", parameters=dict(hand="right", launch_gui="true"))],
    [dict(id="glove_right", parameters=dict(hand="left", launch_gui="true"))]])
def test_missing_or_wrong_hand_profile_does_not_launch(workbench, configuration):
    window, backend, focuses = workbench
    if configuration is None:
        del backend.runtime
    else:
        backend.runtime.configuration["process_groups"] = configuration
    window.open_glove_calibration("right")
    assert backend.commands == [] and not focuses


@pytest.mark.parametrize("launch_gui", ["false", False, "invalid"])
def test_bridge_only_or_invalid_gui_profile_never_starts_second_gui(workbench, launch_gui):
    window, backend, focuses = workbench
    profile = next(p for p in backend.runtime.configuration["process_groups"] if p["id"] == group(backend)["id"])
    profile["parameters"]["launch_gui"] = launch_gui
    window.open_glove_calibration("right")
    assert backend.commands == [] and not focuses
    assert "启用标定窗口" in window.statusBar().currentMessage()


@pytest.mark.parametrize("state,device_changes,label_part,hint_part", [
    ("STOPPED", {}, "未启动", "进程已停止"),
    ("DISABLED", {}, "未启用", "未启用"),
    ("DEGRADED", {}, "已启动", "尚无姿态数据"),
    ("RUNNING", dict(status="online", age_s=.02, receive_hz=30., valid_hz=0., preview=dict(valid=False)), "姿态待就绪", "当前姿态尚未有效"),
    ("DEGRADED", dict(status="stale", age_s=5.), "姿态断流", "消息已陈旧"),
])
def test_device_card_hint_reflects_process_and_observed_data(workbench, state, device_changes, label_part, hint_part):
    window, backend, _ = workbench
    active = state in {"RUNNING", "DEGRADED"}
    group(backend).update(state=state, owned=active, pid=1234 if active else None, can_start=state == "STOPPED")
    stream(backend).update(device_changes)
    window.refresh()
    target = card(window, backend)
    assert label_part in target.state.text()
    assert hint_part in target.glove_note.text()
    assert target.glove_note.toolTip().startswith(target.glove_note.text())


def test_enlarged_glove_view_forwards_same_hand_action_without_new_process(workbench, qapp):
    window, backend, focuses = workbench
    target = card(window, backend, "left")
    target.open_glove_detail()
    qapp.processEvents()
    dialog = target.glove_dialog
    assert dialog is not None
    dialog.controls.calibration_button.click()
    assert backend.commands == [("device.start", dict(group_id=group(backend, "left")["id"], expected_manager_instance_id="manager-test"))]
    assert not focuses
    dialog.close()


def test_tools_menu_routes_each_hand_through_same_guarded_method(workbench):
    window, backend, focuses = workbench
    tools = DesktopTools(window, backend, backend.runtime)
    tools.left_glove_action.trigger()
    assert backend.commands[0][1]["group_id"] == group(backend, "left")["id"]
    assert not focuses
    tools.right_glove_action.trigger()
    assert len(backend.commands) == 1  # Still pending: no independent launcher bypass.


def test_demo_method_never_spawns_or_focuses(qapp, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Demo must not launch or focus hardware tooling")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    monkeypatch.setattr(module, "focus_owned_calibration", forbidden)
    window = MainWindow(DemoBackend())
    try:
        window.open_glove_calibration("right")
        assert "演示模式" in window.statusBar().currentMessage()
    finally:
        window.close()
