"""Window lifecycle and explicit actions, with no real serial ports or OpenGL."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
from PySide6 import QtWidgets
import pytest

from vt_hand_tactile_viewer import window as module
from vt_hand_tactile_viewer.composition import CombinedFrame
from vt_hand_tactile_viewer.serial_discovery import PortClassification, PortSnapshot, SerialPortDiscovery
from vt_hand_tactile_viewer.translations import TEXT
from vt_capture_console.backend.tactile_source import TactileDisplayState, TactileTiming
from vt_tactile_capture.display_response import ResponseSettings, ResponseStatus


class Settings:
    language = "zh_CN"

    def __init__(self, root):
        self.data_root, self.values = root, {}

    def get(self, key, default=None):
        return self.values.get(key, default)

    def set(self, key, value):
        self.values[key] = value

    def save_language(self, value):
        self.language = value


class Pose:
    def __init__(self):
        self.commands, self.started, self.stopped = [], 0, 0
        self.done, self.accept, self.polls = True, True, 0
        self.join_times = []
        self.snapshot = {"runtime": {"connected": False, "error": None},
                         "pose": {}, "calibration": {"active": None, "captured_steps": []}}

    def start(self):
        self.started += 1

    def submit(self, name, **args):
        if not self.accept:
            return False
        self.commands.append((name, args))
        return True

    def latest_snapshot(self):
        self.polls += 1
        return deepcopy(self.snapshot)

    def stop(self):
        self.stopped += 1

    def join(self, timeout=None):
        self.join_times.append(timeout)
        return self.done


class Touch:
    def __init__(self):
        self.port = ""
        self.connection_state = "disconnected"
        self.response_settings = ResponseSettings()
        self.response_status = ResponseStatus()
        self.reference_persistence_error = ""
        self.state = TactileDisplayState(None, "未连接", "")
        self.calls, self.polls, self.zeroes = [], 0, 0
        self.done = True

    @property
    def is_connected(self):
        return self.connection_state == "connected"

    @property
    def can_connect(self):
        return self.connection_state == "disconnected"

    @property
    def can_disconnect(self):
        return not self.can_connect

    def start(self):
        self.calls.append(("start",))

    def connect(self, port, baudrate):
        self.port, self.connection_state = port, "connected"
        self.calls.append(("connect", port, baudrate))

    def disconnect(self):
        self.calls.append(("disconnect",))
        self.connection_state = "disconnected" if self.done else "disconnecting"

    def poll(self):
        self.polls += 1
        if self.connection_state == "disconnecting" and self.done:
            self.connection_state = "disconnected"
        return self.state

    def close(self):
        self.calls.append(("close",))
        return True

    def start_reference(self):
        self.zeroes += 1
        self.response_status = ResponseStatus(phase="collecting", progress=0.)

    def configure_response(self, *, display_range_adc, direction):
        self.response_settings = ResponseSettings(display_range_adc, direction)
        self.calls.append(("configure", display_range_adc, direction))


class View(QtWidgets.QWidget):
    def __init__(self, geometry, axes, parent=None):
        super().__init__(parent)
        self.colors = np.ones((3, 4))
        self.geometry = geometry
        self.surface_colors = None
        self.hand_visible = True
        self.geometry_updates = 0

    def set_show_joints(self, value):
        self.show_joints = value

    def reset_camera(self, palmar=False):
        self.palmar = palmar

    def set_geometry(self, value):
        self.geometry = value
        self.hand_visible = True
        self.geometry_updates += 1

    def set_hand_visible(self, value):
        self.hand_visible = value

    def set_surface_colors(self, value):
        self.surface_colors = value


class Composer:
    def __init__(self):
        self.result = CombinedFrame(None, None, False, "waiting", "等待手势标定")

    def compose(self, *_args, **_kwargs):
        return self.result


class Discovery:
    def __init__(self):
        self.snapshot = None
        self.started = self.requests = self.stopped = 0
        self.identifications, self.join_times = [], []
        self.done = True

    def start(self):
        self.started += 1

    def refresh(self):
        self.requests += 1

    def latest(self):
        return self.snapshot

    def stop(self):
        self.stopped += 1

    def identify(self, *, excluded=(), baudrates=(115200,)):
        if not self.started or self.stopped or (self.snapshot and self.snapshot.identifying):
            return False
        self.identifications.append((tuple(excluded), tuple(baudrates)))
        self.snapshot = replace(self.snapshot or PortSnapshot(), identifying=True)
        return True

    def join(self, timeout=0):
        self.join_times.append(timeout)
        return self.done


@pytest.fixture(scope="session")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def viewer(qapp, tmp_path, monkeypatch):
    monkeypatch.setattr(module, "TactileSurfaceBinding", lambda *args, **kwargs: object())
    monkeypatch.setattr(module, "surface_colors", lambda *args: np.full((3, 4), .5))
    settings, pose, touch, composer = Settings(tmp_path), Pose(), Touch(), Composer()
    resources = {"neutral_geometry": {"example": True}, "model_axes": {}, "rest": {}}
    window = module.HandTactileWindow(settings, resources, pose, touch, composer=composer,
                                     view_factory=View, port_discovery=Discovery())
    yield window, pose, touch, composer, settings
    pose.done = touch.done = True
    if isinstance(window.port_discovery, Discovery):
        window.port_discovery.done = True
    window.shutdown()
    window.close()
    window.deleteLater()
    qapp.processEvents()


def connected_pose(pose):
    pose.snapshot["runtime"].update(connected=True, port="COM3", session_id="test", ipc_sequence=1)


def test_start_and_refresh_never_connect_or_calibrate(viewer):
    window, pose, touch, *_ = viewer
    assert pose.started == 0 and not pose.commands and touch.calls == []
    window.start()
    window.start()
    window.refresh()
    assert pose.started == 1 and pose.commands == []
    assert touch.calls == [("start",)] and touch.zeroes == 0
    assert window.timer.interval() == 16
    assert window.port_discovery.started == 1


def test_background_discovery_updates_both_lists_without_clicking_refresh(viewer, qapp):
    import threading
    import time

    window, pose, touch, *_ = viewer
    devices, caller_threads = [], []

    def provider():
        caller_threads.append(threading.get_ident())
        return list(devices)

    window.port_discovery = SerialPortDiscovery(provider, interval_s=.02)
    window.start()

    def await_port(path, present):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            qapp.processEvents()
            if all((combo.findData(path) >= 0) == present for combo in window.port_combos.values()):
                return
            time.sleep(.005)
        pytest.fail(f"Port {path} was not automatically updated")

    devices.append(dict(path="COM3", label="COM3 · Motion"))
    await_port("COM3", True)
    assert all(window.selected_port(kind) == "" for kind in window.port_combos)
    devices[:] = [dict(path="COM9", label="COM9 · Touch")]
    await_port("COM9", True)
    await_port("COM3", False)
    assert caller_threads and threading.get_ident() not in caller_threads
    assert not pose.commands and touch.calls == [("start",)]


def test_list_changes_preserve_owned_port_and_manual_path_without_switching_sources(viewer):
    window, pose, touch, *_ = viewer
    discovery = window.port_discovery
    discovery.snapshot = PortSnapshot((("COM3", "COM3 · Motion"), ("COM9", "COM9 · Touch")))
    window.refresh()
    window.port_combos["imu"].setCurrentIndex(window.port_combos["imu"].findData("COM3"))
    window.toggle_connection("imu")
    window.port_combos["tactile"].setEditText("/custom/touch")
    discovery.snapshot = PortSnapshot((("COM9", "COM9 · Touch"), ("COM11", "COM11 · Motion")))
    window.refresh()
    assert window.selected_port("imu") == window._imu_port_owned == "COM3"
    assert window.port_combos["imu"].findData("COM3") == -1
    assert window.selected_port("tactile") == "/custom/touch"
    assert pose.commands == [("connect", {"port": "COM3"})]
    assert not touch.calls


def test_unchanged_scan_never_rebuilds_combo_models_and_scan_error_recovers(viewer):
    window, *_ = viewer
    discovery = window.port_discovery
    ports = (("COM3", "COM3 · Motion"),)
    discovery.snapshot = PortSnapshot(ports)
    window.refresh()
    resets = []
    for combo in window.port_combos.values():
        combo.model().modelReset.connect(lambda: resets.append(True))
    for _ in range(5):
        discovery.snapshot = PortSnapshot(ports)
        window.refresh()
    discovery.snapshot = PortSnapshot(ports, "enumeration temporarily unavailable")
    window.refresh()
    assert "串口列表暂时无法更新" in window.notice.text()
    assert not resets
    window.set_language("en")
    assert "port list could not be updated" in window.notice.text()
    # Type prefixes are translated too; their deliberate rebuild is allowed.
    assert window.port_combos["imu"].itemText(1).startswith("Unidentified ·")
    resets.clear()
    discovery.snapshot = PortSnapshot(ports)
    window.refresh()
    assert not window.notice.text() and not resets


def test_port_type_labels_are_bilingual_and_translation_preserves_selected_paths(viewer):
    window, pose, touch, *_ = viewer
    discovery = window.port_discovery
    discovery.snapshot = PortSnapshot(
        (("COM3", "COM3 · STM32"), ("COM9", "COM9 · STM32"), ("COM11", "COM11")),
        classifications=(PortClassification("COM3", "imu"), PortClassification("COM9", "tactile")),
        identities=(("COM3", "usb-imu"), ("COM9", "usb-touch"), ("COM11", None)))
    window.refresh()
    for kind, port in (("imu", "COM3"), ("tactile", "COM9")):
        combo = window.port_combos[kind]
        combo.setCurrentIndex(combo.findData(port))
        assert combo.itemText(combo.findData("COM3")) == "动捕 · COM3 · STM32"
        assert combo.itemText(combo.findData("COM9")) == "触觉 · COM9 · STM32"
        assert combo.itemText(combo.findData("COM11")) == "未识别 · COM11"
    polls = pose.polls, touch.polls
    window.set_language("en")
    for kind, port in (("imu", "COM3"), ("tactile", "COM9")):
        combo = window.port_combos[kind]
        assert window.selected_port(kind) == port
        assert combo.itemText(combo.findData("COM3")) == "Motion · COM3 · STM32"
        assert combo.itemText(combo.findData("COM9")) == "Touch · COM9 · STM32"
        assert combo.itemText(combo.findData("COM11")) == "Unidentified · COM11"
    assert polls == (pose.polls, touch.polls)
    assert not pose.commands and not touch.calls and not discovery.identifications


@pytest.mark.parametrize("role,label", [
    ("identifying", "识别中"), ("ambiguous", "类型不确定"),
    ("busy", "已占用 · 未识别"), ("error", "识别失败"),
])
def test_incomplete_classification_does_not_pretend_device_type(viewer, role, label):
    window, *_ = viewer
    window.port_discovery.snapshot = PortSnapshot(
        (("COM3", "COM3"),), classifications=(PortClassification("COM3", role),))
    window.refresh()
    combo = window.port_combos["imu"]
    assert combo.itemText(combo.findData("COM3")) == label + " · COM3"
    assert window.selected_port("imu") == ""  # classification never auto-selects


def test_identification_button_excludes_connected_and_pending_ports_without_connecting(viewer):
    window, pose, touch, *_ = viewer
    window.start()
    window.port_combos["imu"].setEditText("COM3")
    window.toggle_connection("imu")  # connection request accepted, reader not open yet
    window.port_combos["tactile"].setEditText("COM9")
    window.toggle_connection("tactile")
    commands, calls = list(pose.commands), list(touch.calls)
    window.identify_ports_button.click()
    excluded, rates = window.port_discovery.identifications[0]
    assert set(excluded) == {"COM3", "COM9"} and rates == (115200,)
    assert pose.commands == commands and touch.calls == calls
    assert window.identify_ports_button.text() == "正在识别…"
    assert not window.identify_ports_button.isEnabled()
    assert not window.tactile_baudrate.isEnabled()


def test_identification_uses_configured_tactile_baudrate_without_switching_inputs(viewer):
    window, pose, touch, *_ = viewer
    window.start()
    window.tactile_baudrate.setValue(57600)
    window.identify_ports_button.click()
    _, rates = window.port_discovery.identifications[0]
    assert rates == (115200, 57600)
    assert not pose.commands and touch.calls == [("start",)]
    assert all(window.selected_port(kind) == "" for kind in window.port_combos)


def test_identification_blocks_new_connects_in_ui_and_direct_calls_until_complete(viewer):
    window, pose, touch, *_ = viewer
    discovery = window.port_discovery
    discovery.snapshot = PortSnapshot(identifying=True)
    window.port_combos["imu"].setEditText("COM3")
    window.port_combos["tactile"].setEditText("COM9")
    window.refresh()
    assert all(not button.isEnabled() for button in window.connection_buttons.values())
    window.toggle_connection("imu")
    window.toggle_connection("tactile")
    assert not pose.commands and not touch.calls
    assert "正在识别串口类型" in window.notice.text()
    discovery.snapshot = PortSnapshot()
    window.refresh()
    assert all(button.isEnabled() for button in window.connection_buttons.values())
    window.toggle_connection("imu")
    assert pose.commands == [("connect", {"port": "COM3"})]


def test_identification_keeps_existing_disconnect_actions_available(viewer):
    window, pose, touch, *_ = viewer
    connected_pose(pose)
    touch.connection_state, touch.port = "connected", "COM9"
    window.port_discovery.snapshot = PortSnapshot(identifying=True)
    window.refresh()
    assert all(button.isEnabled() for button in window.connection_buttons.values())
    window.connection_buttons["imu"].click()
    window.connection_buttons["tactile"].click()
    assert pose.commands == [("disconnect", {})]
    assert touch.calls == [("disconnect",)]


@pytest.mark.parametrize("kind,actual", [("imu", "tactile"), ("tactile", "imu")])
def test_wrong_classified_type_cannot_be_opened_even_through_port_alias(viewer, kind, actual):
    window, pose, touch, *_ = viewer
    window.port_discovery.snapshot = PortSnapshot(
        (("COM3", "COM3"),), classifications=(PortClassification("COM3", actual),),
        identities=(("COM3", "usb-device"),))
    window.refresh()
    window.port_combos[kind].setEditText(r"\\.\COM03")
    window.toggle_connection(kind)
    assert not pose.commands and not touch.calls
    expected = "下方“触觉串口”" if kind == "imu" else "上方“姿态串口”"
    assert expected in window.notice.text()


@pytest.mark.parametrize("kind,role,zh,en", [
    ("imu", "tactile", "触觉", "Touch"),
    ("tactile", "imu", "动捕", "Motion"),
])
def test_unidentified_hardware_uses_temporary_type_label_without_blocking_explicit_choice(
        viewer, kind, role, zh, en):
    window, pose, touch, *_ = viewer
    window.port_discovery.snapshot = PortSnapshot(
        (("COM3", "COM3"),), classifications=(PortClassification("COM3", role),),
        identities=(("COM3", None),))
    window.refresh()
    combo = window.port_combos[kind]
    combo.setCurrentIndex(combo.findData("COM3"))
    assert combo.currentText() == f"{zh}（本次识别） · COM3"
    window.set_language("en")
    assert combo.currentText() == f"{en} (this scan) · COM3"
    assert window.selected_port(kind) == "COM3"
    window.toggle_connection(kind)
    if kind == "imu":
        assert pose.commands == [("connect", {"port": "COM3"})] and not touch.calls
    else:
        assert touch.calls == [("connect", "COM3", 115200)] and not pose.commands
    assert not window.notice.text()


def fresh_touch_state():
    timing = TactileTiming("session", 0, 1, 1, 1, (1,) * 160, 100_000_000)
    return TactileDisplayState({"values": [0.] * 400}, "实时", "", age_ms=10,
                               valid_cells=400, timing=timing)


@pytest.mark.parametrize("kind", ["imu", "tactile"])
@pytest.mark.parametrize("probe_role,label", [(None, "未识别"), ("imu", "动捕"), ("tactile", "触觉")])
def test_selection_connection_and_live_snapshots_do_not_assign_or_override_protocol_type(
        viewer, kind, probe_role, label):
    window, pose, touch, *_ = viewer
    discovery = window.port_discovery
    discovery.snapshot = PortSnapshot((("COM3", "COM3"),), identities=(("COM3", "usb-device"),))
    window.refresh()
    combo = window.port_combos[kind]
    combo.setCurrentIndex(combo.findData("COM3"))
    window.toggle_connection(kind)
    assert not discovery.latest().classifications  # Selecting/opening is not protocol evidence.
    if kind == "imu":
        connected_pose(pose)
        pose.snapshot["runtime"].update(received_samples=24, last_sample_age_s=.01)
    else:
        touch.state = fresh_touch_state()
    classifications = () if probe_role is None else (PortClassification("COM3", probe_role),)
    before = discovery.snapshot = replace(discovery.snapshot, classifications=classifications)
    for _ in range(3):
        window.refresh()
    assert discovery.latest() is before
    assert combo.itemText(combo.findData("COM3")) == label + " · COM3"
    assert not discovery.identifications and not window.notice.text()


def test_close_waits_asynchronously_for_probe_to_release_serial_handle(viewer, qapp):
    window, pose, touch, *_ = viewer
    discovery = window.port_discovery
    discovery.done = False
    discovery.snapshot = PortSnapshot(identifying=True)
    window.show()
    window.close()
    assert window.isVisible() and window._closing and not window._closed
    assert discovery.stopped == 1 and discovery.join_times == [0]
    window.refresh()
    assert discovery.join_times == [0, 0]
    discovery.done = True
    window.refresh()
    assert window._closed and not window.isVisible()
    window.identify_ports()
    assert not discovery.identifications


def test_open_port_popup_defers_changes_until_selection_finishes(viewer, qapp):
    window, *_ = viewer
    discovery = window.port_discovery
    discovery.snapshot = PortSnapshot((("COM3", "COM3"),))
    window.refresh()
    window.show()
    qapp.processEvents()
    combo = window.port_combos["imu"]
    combo.showPopup()
    qapp.processEvents()
    assert combo.view().isVisible()
    discovery.snapshot = PortSnapshot((("COM9", "COM9"),))
    window.refresh()
    assert combo.findData("COM3") >= 0 and combo.findData("COM9") == -1
    combo.hidePopup()
    window.refresh()
    assert combo.findData("COM9") >= 0 and combo.findData("COM3") == -1


def test_typing_is_not_interrupted_and_update_resumes_on_edit_finished(viewer, qapp):
    window, *_ = viewer
    discovery = window.port_discovery
    discovery.snapshot = PortSnapshot((("COM3", "COM3"),))
    window.refresh()
    window.show()
    combo = window.port_combos["tactile"]
    editor = combo.lineEdit()
    editor.setFocus()
    qapp.processEvents()
    editor.setText("/custom/port")
    editor.setCursorPosition(4)
    editor.setModified(True)
    assert editor.hasFocus()
    discovery.snapshot = PortSnapshot((("COM9", "COM9"),))
    window.refresh()
    assert combo.findData("COM3") >= 0 and combo.findData("COM9") == -1
    assert editor.text() == "/custom/port" and editor.cursorPosition() == 4
    editor.editingFinished.emit()
    window.refresh()
    assert combo.findData("COM9") >= 0 and combo.findData("COM3") == -1
    assert window.selected_port("tactile") == "/custom/port" and editor.cursorPosition() == 4


def test_manual_refresh_requests_same_scanner_and_close_stops_discovery(viewer):
    window, *_ = viewer
    discovery = window.port_discovery
    window.start()
    window.refresh_ports_button.click()
    assert discovery.requests == 1 and discovery.started == 1
    window.close()
    window.refresh_ports()
    assert discovery.stopped == 1 and discovery.requests == 1


@pytest.mark.parametrize("one,two", [("COM3", "com3"), ("COM03", r"\\.\COM3")])
def test_two_inputs_cannot_open_same_windows_port(viewer, one, two):
    window, pose, touch, *_ = viewer
    window.port_combos["imu"].setEditText(one)
    window.toggle_connection("imu")
    window.port_combos["tactile"].setEditText(two)
    window.toggle_connection("tactile")
    assert pose.commands == [("connect", {"port": one})]
    assert not any(item[0] == "connect" for item in touch.calls)
    assert "同一个串口" in window.notice.text()




def test_same_linux_port_alias_is_rejected(viewer, tmp_path):
    window, pose, touch, *_ = viewer
    device = tmp_path / "ttyUSB0"
    device.touch()
    alias = tmp_path / "by-id"
    alias.symlink_to(device)
    window.port_combos["tactile"].setEditText(str(alias))
    window.toggle_connection("tactile")
    window.port_combos["imu"].setEditText(str(device))
    window.toggle_connection("imu")
    assert not pose.commands
    assert "同一个串口" in window.notice.text()


def test_explicit_ports_are_saved_but_queue_rejection_does_not_claim_connection(viewer):
    window, pose, touch, _, settings = viewer
    pose.accept = False
    window.port_combos["imu"].setEditText("COM9")
    window.toggle_connection("imu")
    assert window._imu_port_owned == "" and "imu_port" not in settings.values
    assert "未提交" in window.notice.text()
    window.port_combos["tactile"].setEditText("COM12")
    window.toggle_connection("tactile")
    assert touch.calls == [("connect", "COM12", 115200)]
    assert settings.get("tactile_port") == "COM12"


def test_language_switch_reuses_views_and_does_not_poll_sources(viewer):
    window, pose, touch, _, settings = viewer
    window.refresh()
    counts = pose.polls, touch.polls, list(pose.commands), list(touch.calls)
    view = window.view
    window.set_language("en")
    assert window.windowTitle() == "Hand Motion & Touch"
    assert window.calibration_toggle.text() == "Motion calibration"
    assert window.view is view and settings.language == "en"
    assert (pose.polls, touch.polls, pose.commands, touch.calls) == counts
    assert "Fixed" not in window.mode_label.text()


def test_capture_receipt_is_pending_until_backend_starts_and_completes(viewer):
    window, pose, touch, *_ = viewer
    connected_pose(pose)
    window.refresh()
    window.capture_button.click()
    assert pose.commands[-1] == ("capture", {"step": "neutral"})
    assert "等待后台" in window.calibration_status.text()
    assert not window.capture_button.isEnabled()
    assert not pose.snapshot["calibration"]["captured_steps"]
    pose.snapshot["calibration"]["active"] = {"step": "neutral", "phase": "collecting", "remaining_s": 1.5}
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="capture", last_command_error=None)
    window.refresh()
    assert "剩余 2 秒" in window.calibration_status.text()
    pose.snapshot["calibration"].update(active=None, captured_steps=["neutral"])
    window.refresh()
    assert "1/3" in window.calibration_status.text()
    assert window.step_buttons["neutral"].text().startswith("✓")
    assert touch.zeroes == 0


def test_functional_result_reports_partial_not_success(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["calibration"].update(calibrated_nodes=[str(x) for x in range(1, 13)])
    window.refresh()
    window.functional_button.click()
    assert pose.commands[-1] == ("capture_functional_hand", {})
    assert "等待后台" in window.calibration_status.text()
    pose.snapshot["calibration"].update(functional_hand_result={"status": "partial", "errors": {"thumb": "inadequate movement"}})
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="capture_functional_hand", last_command_error=None)
    window.refresh()
    assert "部分手指" in window.functional_status.text()
    assert "已完成" not in window.functional_status.text()


def test_reference_origin_and_optional_zero_are_independent_of_motion(viewer):
    window, pose, touch, *_ = viewer
    touch.connection_state = "connected"
    touch.state = TactileDisplayState({"values": [0.] * 400}, "实时", "", valid_cells=400)
    touch.response_status = ResponseStatus(phase="ready", reference_cells=400, reference_source="default")
    window.refresh()
    assert window.reference_status.text() == "通用参考 400/400" and window.reference_button.isEnabled()
    window.reference_button.click()
    assert touch.zeroes == 1 and not pose.commands
    assert not window.reference_progress.isHidden()


def test_profile_range_sync_preserves_unsubmitted_range_edit(viewer):
    window, _, touch, *_ = viewer
    window.refresh()
    window.response_range.setValue(750)
    window.refresh()
    assert window.response_range.value() == 750
    window.configure_response()
    assert touch.response_settings.display_range_adc == 750


def test_renderer_hides_missing_pose_and_only_uses_composed_touch(viewer):
    window, pose, _, composer, _ = viewer
    connected_pose(pose)
    composer.result = CombinedFrame({"pose": 1}, {"levels": [0., 1000.]}, True, "live", "实时手势与触觉")
    window.refresh()
    assert window.view.hand_visible and window.view.surface_colors is not None
    assert window.mode_label.text() == "实时手势与触觉"
    composer.result = CombinedFrame(None, None, False, "waiting", "手势数据已过期")
    window.refresh()
    assert not window.view.hand_visible and window.view.surface_colors is None
    composer.result = CombinedFrame({"neutral": True}, None, False, "reference", "固定参考手形 · 等待连接")
    window.refresh()
    assert window.view.hand_visible and "非实时手势" in window.mode_label.text()


def test_shutdown_polls_without_blocking_or_killing_and_releases_both(viewer, qapp):
    window, pose, touch, *_ = viewer
    pose.done = touch.done = False
    touch.connection_state = "connected"
    window.show()
    window.close()
    assert window.isVisible() and window._closing and not window._closed
    assert pose.stopped == 1 and pose.join_times == [0]
    window.refresh()
    assert pose.join_times == [0, 0]
    pose.done = touch.done = True
    window.refresh()
    assert window._closed and not window.isVisible()
    assert pose.stopped == 1 and touch.calls[-1] == ("close",)


def test_vocabulary_is_bilingual_without_chinese_in_english_labels():
    assert all(len(pair) == 2 and all(pair) for pair in TEXT.values())
    assert all(not any('\u4e00' <= char <= '\u9fff' for char in pair[1]) for pair in TEXT.values())


def test_unrelated_snapshot_change_does_not_acknowledge_capture(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    window.refresh()
    window.capture_button.click()
    pose.snapshot["calibration"]["input_scale_check"] = {"status": "changed"}
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="unrelated")
    window.refresh()
    assert window._pending_calibration is not None
    assert "等待后台" in window.calibration_status.text()
    pose.snapshot["runtime"].update(command_rejected_count=1, last_rejected_command_name="capture",
                                    last_rejected_command_error="queue full")
    window.refresh()
    assert window._pending_calibration is None
    assert window.capture_button.isEnabled()
    assert "未完成" in window.notice.text()
    assert "queue full" in window.details.toPlainText()


def test_cancel_before_capture_starts_releases_pending_with_execution_ack(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    window.refresh()
    window.capture_button.click()
    window.cancel_button.click()
    pose.snapshot["runtime"].update(command_completed_generation=2, last_command_name="cancel_capture",
                                    last_command_error=None)
    window.refresh()
    assert window._pending_calibration is None
    assert window.capture_button.isEnabled()
    assert "已生效" not in window.calibration_status.text()


def test_disconnect_keeps_port_reserved_until_worker_execution_finishes(viewer):
    window, pose, touch, *_ = viewer
    connected_pose(pose)
    window.refresh()
    window.toggle_connection("imu")
    pose.snapshot["runtime"]["connected"] = False
    pose.snapshot["pose"]["reason"] = "disconnect_requested"
    window.refresh()
    window.port_combos["tactile"].setEditText("COM3")
    window.toggle_connection("tactile")
    assert not any(item[0] == "connect" for item in touch.calls)
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="disconnect", last_command_error=None)
    window.refresh()
    assert window._imu_port_owned == ""
    window.toggle_connection("tactile")
    assert ("connect", "COM3", 115200) in touch.calls


def test_old_connection_error_does_not_release_new_connection_request(viewer):
    window, pose, *_ = viewer
    pose.snapshot["runtime"].update(error="previous error", command_completed_generation=1,
                                    last_command_name="connect", last_command_error="previous error")
    window.refresh()
    window.port_combos["imu"].setEditText("COM7")
    window.toggle_connection("imu")
    assert window._pose_request == "connect" and window._imu_port_owned == "COM7"
    pose.snapshot["runtime"].update(command_completed_generation=2, last_command_name="connect", last_command_error="new error")
    window.refresh()
    assert window._pose_request is None and window._imu_port_owned == ""
