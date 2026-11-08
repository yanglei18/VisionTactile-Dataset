"""Window lifecycle and explicit actions, with no real serial ports or OpenGL."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
from PySide6 import QtWidgets
import pytest

from vt_capture_console.ui.hand import window as module
from vt_hand_tactile_viewer.composition import CombinedFrame
from vt_hand_tactile_viewer.serial_discovery import PortClassification, PortSnapshot, SerialPortDiscovery
from vt_hand_tactile_viewer.translations import TEXT
from vt_capture_console.backend.tactile_source import TactileDisplayState, TactileTiming
from vt_tactile_capture.display_response import ResponseSettings, ResponseStatus
from vt_fsglove12.tactile_surface import TactileSurfaceProjection


def test_raw_diagnostics_remain_available_without_reference_and_clear_on_outage(viewer):
    window, _, touch, *_ = viewer
    values = [18195] * 400
    values[0] = 0
    valid = [True] * 400
    valid[1] = False
    touch.state = TactileDisplayState(
        None, "部分有效", "原始 ADC", valid_cells=399, response=ResponseStatus(),
        raw_frame=dict(values=values, valid_mask=valid, levels=[0, 65535], value_kind="adc_counts"))
    window.refresh()
    details = window._tactile_diagnostics()
    assert details["valid_cells"] == 399
    assert details["adc_range"] == [0, 18195]
    assert details["adc_rows_0_to_19"][0].split()[:3] == ["0", "—", "18195"]
    assert len(details["adc_rows_0_to_19"]) == 20
    assert details["reference_circuit_resistance_ohm_range"][0] > 24000
    touch.state = TactileDisplayState(None, "数据过期", "没有新数据")
    window.refresh()
    assert "adc_rows_0_to_19" not in window._tactile_diagnostics()


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
        self.surface_colors = self.colors
        self.hand_visible = True
        self.geometry_updates = 0

    def set_show_joints(self, value):
        self.show_joints = value

    def reset_camera(self, palmar=False):
        self.palmar = palmar

    def setCameraPosition(self, **position):
        self.camera_position = position

    def set_step(self, step, *, hold=False):
        self.step = step
        self.hold = hold

    def set_geometry(self, value):
        self.geometry = value
        self.hand_visible = True
        self.geometry_updates += 1

    def set_hand_visible(self, value):
        self.hand_visible = value

    def set_surface_colors(self, value):
        self.surface_colors = self.colors if value is None else value


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
                                     view_factory=View, port_discovery=Discovery(), demo_factory=View)
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


def test_live_joint_loss_and_required_realignment_are_visible_and_clear_on_recovery(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["pose"]["calibration_rejections"] = {"4": "missing_or_stale_node"}
    window.refresh()
    assert "4" in window.notice.text() and "等待" in window.notice.text()
    pose.snapshot["pose"]["calibration_rejections"]["4"] = "filter_epoch_changed_recalibration_required"
    window.refresh()
    assert "4" in window.notice.text() and "标定" in window.notice.text()
    pose.snapshot["pose"]["calibration_rejections"].clear()
    window.refresh()
    assert not window.notice.text()


@pytest.mark.parametrize("guides_visible", [True, False])
def test_transient_joint_warning_keeps_both_viewports_in_place(viewer, qapp, guides_visible):
    from vt_capture_console.theme import stylesheet

    window, pose, *_ = viewer
    connected_pose(pose)
    window.setStyleSheet(stylesheet())
    window.resize(1100, 1000)
    window.calibration_toggle.setChecked(guides_visible)
    window.show()
    window.refresh()
    qapp.processEvents()

    def viewports():
        return [(widget.pos(), widget.size()) for widget in (window.view, window.demo) if widget.isVisible()]

    baseline = viewports()
    for rejections in ({"7": "node_not_calibrated", "9": "missing_or_stale_node"}, {}):
        pose.snapshot["pose"]["calibration_rejections"] = rejections
        window.refresh()
        qapp.processEvents()
        assert window.notice.isVisible() == bool(rejections)
        assert viewports() == baseline


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
        assert combo.itemText(combo.findData("COM9")) == "触觉 · 20×20 · COM9 · STM32"
        assert combo.itemText(combo.findData("COM11")) == "未识别 · COM11"
    polls = pose.polls, touch.polls
    window.set_language("en")
    for kind, port in (("imu", "COM3"), ("tactile", "COM9")):
        combo = window.port_combos[kind]
        assert window.selected_port(kind) == port
        assert combo.itemText(combo.findData("COM3")) == "Motion · COM3 · STM32"
        assert combo.itemText(combo.findData("COM9")) == "Touch · 20×20 · COM9 · STM32"
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
    ("imu", "tactile", "触觉 · 20×20", "Touch · 20×20"),
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
    timing = TactileTiming("session", 0, 1, 1, 1, (1,) * 20, 100_000_000)
    return TactileDisplayState({"values": [0.] * 400}, "实时", "", age_ms=10,
                               valid_cells=400, timing=timing)


@pytest.mark.parametrize("kind", ["imu", "tactile"])
@pytest.mark.parametrize("probe_role,label", [(None, "未识别"), ("imu", "动捕"), ("tactile", "触觉 · 20×20")])
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
    assert window.windowTitle() == "右手三维展示"
    window.set_language("en")
    assert window.windowTitle() == "Right Hand 3D Display"
    assert window.calibration_toggle.text() == "Motion calibration"
    assert window.view is view and settings.language == "en"
    assert (pose.polls, touch.polls, pose.commands, touch.calls) == counts
    assert window.mode_label.text() == "Fixed reference hand · Motion calibration required"


@pytest.mark.parametrize("step", ["neutral", "fingers_down", "thumb_up"])
def test_one_click_capture_tracks_its_backend_phase_and_receipt(viewer, step):
    window, pose, touch, *_ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(received_samples=24, last_sample_age_s=.01)
    calibration = pose.snapshot["calibration"]
    window.refresh()
    button = window.step_buttons[step]
    button.click()
    assert pose.commands == [("capture", {"step": step})]
    assert window.calibration_toggle.isChecked() and window.demo.step == step
    assert "等待后台" in window.calibration_status.text()
    assert button.progress_bar.minimum() == button.progress_bar.maximum() == 0
    assert all(not control.isEnabled() for control in window.step_buttons.values())
    assert not calibration["captured_steps"]
    calibration.update(active=dict(step=step, kind="mounting", phase="preparing",
                                   remaining_s=2.25, phase_progress=.25), profile_status="collecting")
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="capture", last_command_error=None)
    window.refresh()
    assert button.progress_bar.value() == 250 and "25%" in button.text()
    assert window.calibration_progress.value() == 250 and "25%" in window.calibration_progress.format()
    calibration["active"].update(phase="collecting", remaining_s=1.5, phase_progress=.4)
    window.refresh()
    assert "剩余 2 秒" in window.calibration_status.text()
    assert button.progress_bar.value() == 400
    assert window.demo.hold and window.calibration_progress.value() == 400
    calibration["active"].update(remaining_s=0., phase_progress=1.)
    window.refresh()
    assert button.progress_bar.value() < button.progress_bar.maximum()
    assert not button.text().startswith("✓")
    calibration.update(active=None, captured_steps=[step], profile_status="steps_pending")
    window.refresh()
    assert "1/3" in window.calibration_status.text()
    assert button.text().startswith("✓") and button.progress_bar.value() == 1000
    assert "1/3" in window.calibration_progress.format()
    assert touch.zeroes == 0


def test_mounting_input_must_arrive_before_capture(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(received_samples=0, last_sample_age_s=None)
    window.refresh()
    for button in window.step_buttons.values():
        assert not button.isEnabled()
        button.click()
    assert pose.commands == []
    pose.snapshot["runtime"].update(received_samples=12, last_sample_age_s=.01)
    window.refresh()
    assert all(button.isEnabled() for button in window.step_buttons.values())


def test_missing_progress_and_fitting_wait_for_backend_outcome(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(received_samples=24, last_sample_age_s=.01)
    calibration = pose.snapshot["calibration"]
    window.refresh()
    button = window.step_buttons["fingers_down"]
    button.click()
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="capture",
                                    last_command_error=None)
    calibration.update(active=dict(step="fingers_down", kind="mounting", phase="collecting",
                                   remaining_s=1.), captured_steps=["neutral", "thumb_up"])
    window.refresh()
    assert button.progress_bar.minimum() == button.progress_bar.maximum() == 0
    calibration.update(active=None, profile_status="fitting", captured_steps=list(window.step_buttons))
    window.refresh()
    assert button.progress_bar.minimum() == button.progress_bar.maximum() == 0
    assert not button.text().startswith("✓")
    calibration.update(profile_status="fit_failed", retry_steps=["fingers_down"])
    window.refresh()
    assert button.text().startswith("↻") and button.progress_bar.value() == 0
    pose.snapshot["runtime"].update(connected=False, session_id=None)
    window.refresh()
    assert all(control.progress_bar.value() == 0 and not control.isEnabled()
               for control in window.step_buttons.values())
    pose.snapshot["runtime"].update(connected=True, session_id="reconnected", received_samples=0)
    pose.snapshot["calibration"] = dict(captured_steps=[], active=None, profile_status="not_calibrated")
    window.refresh()
    assert all(control.progress_bar.value() == 0 and not control.text().startswith("✓")
               for control in window.step_buttons.values())


@pytest.mark.parametrize("phase,keyword", [
    ("moving_fingers", "fingers"), ("preparing_thumb", "thumb"), ("moving_thumb", "thumb")])
def test_whole_hand_capture_progress_belongs_to_its_action(viewer, phase, keyword):
    window, pose, *_ = viewer
    connected_pose(pose)
    window.set_language("en")
    calibration = pose.snapshot["calibration"]
    calibration.update(profile_status="collecting_functional_hand", calibrated_nodes=["11"],
                       active=dict(kind="functional_hand", step="neutral", phase="collecting_neutral",
                                   reference_waiting=True, remaining_s=8., phase_progress=None))
    window.refresh()
    bar = window.functional_button.progress_bar
    assert bar.minimum() == bar.maximum() == 0
    assert window.demo.step == "neutral" and window.demo.hold
    assert "stable" in window.calibration_status.text().lower()
    calibration["active"].update(phase=phase, reference_waiting=False, phase_progress=.5)
    window.refresh()
    assert bar.value() == 500 and keyword in window.calibration_status.text().lower()
    expected_demo = {"moving_fingers": "flex_fingers", "preparing_thumb": "flex_thumb", "moving_thumb": "flex_thumb"}
    assert window.demo.step == expected_demo[phase] and not window.demo.hold
    assert window.step_buttons["neutral"].progress_bar.value() == 0
    calibration.update(active=None, profile_status="fitting_functional_hand")
    window.refresh()
    assert bar.minimum() == bar.maximum() == 0
    assert window.demo.step == "neutral" and window.demo.hold
    calibration.update(profile_status="ready", functional_hand_result={"status": "complete"})
    window.refresh()
    assert bar.value() == bar.maximum() == 1000
    assert "complete" in window.instructions.text().lower()


def test_flat_hand_correction_has_its_own_action_guide_progress_and_result(viewer):
    window, pose, touch, *_ = viewer
    connected_pose(pose)
    state, runtime = pose.snapshot["calibration"], pose.snapshot["runtime"]
    state.update(profile_status="ready", captured_steps=list(window.step_buttons),
                 calibrated_nodes=["11"], functional_hand_result={"status": "complete"})
    runtime.update(received_samples=60, last_sample_age_s=.01)
    window.refresh()
    button = window.heading_button
    assert button.isHidden()  # Older workers cannot accept this command.
    state["heading_reference_available"] = True
    runtime["last_sample_age_s"] = 1.
    window.refresh()
    assert not button.isHidden() and not button.isEnabled()
    runtime["last_sample_age_s"] = .01
    window.refresh()
    button.click()
    assert pose.commands == [("capture_heading_reference", {})] and touch.zeroes == 0
    assert not button.isEnabled() and button.progress_bar.maximum() == 0
    assert "五指自然伸直" in window.instructions.text() and window.demo.step == "neutral"
    runtime.update(command_completed_generation=1, last_command_name="capture_heading_reference")
    state.update(active=dict(kind="heading_reference", step="neutral", phase="collecting",
                             remaining_s=1., phase_progress=.5))
    window.refresh()
    assert window.calibration_progress.value() == button.progress_bar.value() == 500
    assert window.functional_button.progress_bar.value() == window.step_buttons["neutral"].progress_bar.value() == 1000
    state.update(active=None, heading_reference_status=dict(status="rejected", message="采集期间移动"))
    window.refresh()
    assert button.isEnabled() and button.progress_bar.value() == 0
    assert "已保留原标定" in window.heading_status.text() and "采集期间移动" in window.heading_status.text()
    state["heading_reference_status"] = dict(status="updated")
    window.refresh()
    assert "校正已生效" in window.heading_status.text()
    window.set_language("en")
    assert "applied" in window.heading_status.text() and "flat hand" in button.text()


def test_functional_result_reports_partial_not_success(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(received_samples=24, last_sample_age_s=.01)
    pose.snapshot["calibration"].update(calibrated_nodes=[str(x) for x in range(1, 13)])
    window.refresh()
    assert window.functional_button.isEnabled() and not pose.commands
    assert "4. 开始手指弯曲标定" in window.instructions.text()
    assert window.demo.step == "flex_fingers"
    window.functional_button.click()
    assert pose.commands[-1] == ("capture_functional_hand", {})
    assert "等待后台" in window.calibration_status.text()
    pose.snapshot["calibration"].update(functional_hand_result={"status": "partial", "errors": {"thumb": "inadequate movement"}})
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="capture_functional_hand", last_command_error=None)
    window.refresh()
    assert "部分手指" in window.functional_status.text()
    assert "拇指" in window.functional_status.text()
    assert "已完成" not in window.functional_status.text()
    assert window.functional_button.progress_bar.value() == 0


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


def test_view_switches_between_fixed_hand_and_live_motion_without_retaining_old_colors(viewer):
    window, pose, _, composer, _ = viewer
    connected_pose(pose)
    neutral = window.resources["neutral_geometry"]
    composer.result = CombinedFrame(None, None, False, "waiting", "等待 IMU 数据")
    window.refresh()
    reference_colors = window.view.surface_colors.copy()
    assert window.view.hand_visible and window.view.geometry is neutral
    assert window.mode_label.text() == "固定参考手形 · 等待 IMU 数据"
    updates = window.view.geometry_updates
    composer.result = CombinedFrame(None, None, False, "waiting", "等待手势标定")
    pose.snapshot["runtime"]["ipc_sequence"] += 1
    window.refresh()
    assert window.view.geometry_updates == updates
    assert window.mode_label.text() == "固定参考手形 · 等待手势标定"
    frame = {"levels": [0., 1000.], "values": [0.] * 399 + [250.], "valid_mask": [True] * 399 + [False]}
    composer.result = replace(composer.result, tactile_frame=frame, tactile_age_ms=15.)
    window.refresh()
    assert window.view.geometry is neutral and window.view.geometry_updates == updates
    assert not np.array_equal(window.view.surface_colors, reference_colors)
    assert window.mode_label.text() == "固定参考手形 · 实时触觉 · 等待手势标定"
    assert "有效读数 399/400 · 有响应 0 点 · 峰值 0.0 ΔADC" in window.metrics.text()
    composer.result = replace(composer.result, tactile_frame=None, tactile_age_ms=None)
    window.refresh()
    assert window.view.geometry is neutral
    np.testing.assert_array_equal(window.view.surface_colors, reference_colors)
    assert window.mode_label.text() == "固定参考手形 · 等待手势标定"
    live = CombinedFrame({"pose": 1}, dict(frame, valid_mask=[True] * 400), True, "live", "实时手势与触觉")
    composer.result = live
    window.refresh()
    assert window.view.hand_visible
    assert not np.array_equal(window.view.surface_colors, reference_colors)
    assert window.view.geometry is live.geometry
    assert window.mode_label.text() == "实时手势与触觉"
    assert "有效读数 400/400 · 有响应 1 点 · 峰值 250.0 ΔADC" in window.metrics.text()
    composer.result = CombinedFrame(None, None, False, "waiting", "手势数据已过期")
    window.refresh()
    assert window.view.hand_visible and window.view.geometry is neutral
    np.testing.assert_array_equal(window.view.surface_colors, reference_colors)
    assert window.mode_label.text() == "固定参考手形 · 手势数据已过期"
    assert "ΔADC" not in window.metrics.text()
    composer.result = live
    window.refresh()
    assert window.view.geometry is live.geometry
    assert not np.array_equal(window.view.surface_colors, reference_colors)
    composer.result = CombinedFrame(neutral, None, False, "reference", "固定参考手形 · 等待连接")
    window.refresh()
    assert window.view.hand_visible and "非实时手势" in window.mode_label.text()


def test_brief_pose_gap_keeps_geometry_material_and_guide_without_enabling_capture(viewer):
    window, pose, _, composer, _ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(received_samples=24, last_sample_age_s=.01)
    pose.snapshot["calibration"].update(calibrated_nodes=["11", "3", "4"], profile_status="ready")
    live = CombinedFrame({"pose": 1}, None, True, "live", "实时手势 · 等待触觉",
                         geometry_key=("test", 1, 10))
    composer.result = live
    window.refresh()
    colors = window.view.surface_colors.copy()
    updates = window.view.geometry_updates
    assert window.demo.step == "flex_fingers" and window.functional_button.isEnabled()
    progress = window.calibration_progress.format()
    pose.snapshot["runtime"].update(ipc_sequence=2, last_sample_age_s=.17)
    composer.result = replace(live, pose_current=False, mode="held",
                              status="保留上一帧手形 · 等待姿态更新", pose_age_ms=170.)
    for _ in range(2):
        window.refresh()
        assert window.view.geometry is live.geometry and window.view.geometry_updates == updates
        np.testing.assert_array_equal(window.view.surface_colors, colors)
        assert "上一帧" in window.live_title.text() and "等待姿态更新" in window.mode_label.text()
        assert "170 ms" in window.metrics.text()
        assert window.demo.step == "flex_fingers" and window.calibration_progress.format() == progress
        assert not window.functional_button.isEnabled()
    composer.result = CombinedFrame(None, None, False, "waiting", "手势数据已过期")
    window.refresh()
    assert window.view.geometry is window.resources["neutral_geometry"]
    assert not np.array_equal(window.view.surface_colors, colors)
    assert "固定参考" in window.live_title.text() and window.demo.step == "neutral"


@pytest.fixture
def left_surface_viewer(qapp, tmp_path, monkeypatch):
    # Three patches in rest coordinates: palm (-Z), dorsum (+Z) and edge (+X).
    # One palm vertex and one edge vertex have independent tactile readings.
    vertices = [[0., 0., 0.], [0., .01, 0.], [.01, 0., 0.],
                [0., 0., .003], [.01, 0., .003], [0., .01, .003],
                [.01, 0., 0.], [.01, .01, 0.], [.01, 0., .003]]
    weights = np.zeros((len(vertices), 2))
    weights[0, 0] = weights[6, 1] = 1.
    binding = SimpleNamespace(projection=TactileSurfaceProjection([(0, 0), (0, 1)], [True, True], weights))
    monkeypatch.setattr(module, "TactileSurfaceBinding", lambda *args, **kwargs: binding)
    geometry = dict(vertices_m=vertices, faces=[[0, 1, 2], [3, 4, 5], [6, 7, 8]],
                    joints_m=np.zeros((21, 3)), provenance={"hand": "left"})
    axes = dict(roll_axis=[0., 1., 0.], pitch_axis=[1., 0., 0.], palm_up_axis=[0., 0., 1.], hand="left")
    pose, composer = Pose(), Composer()
    window = module.HandTactileWindow(Settings(tmp_path), dict(hand="left", neutral_geometry=geometry, model_axes=axes),
                                      pose, Touch(), composer=composer, port_discovery=Discovery(), demo_factory=View)
    yield window, pose, composer
    window.shutdown()
    window.close()
    window.deleteLater()
    qapp.processEvents()


def test_reference_hand_colors_distinguish_contact_release_and_missing_readings(left_surface_viewer):
    window, _, composer = left_surface_viewer
    window.refresh()
    mesh = window.view.mesh_data
    base = mesh.vertexColors().copy()
    neutral, yellow, orange, red = [np.array(rgb) for rgb in (
        [184, 184, 184], [251, 232, 165], [244, 146, 64], [195, 42, 42])]
    np.testing.assert_allclose(base[:, :3] * 255, np.tile(neutral, (len(base), 1)), atol=1e-4)
    frame = dict(values=np.zeros(400), valid_mask=np.ones(400, dtype=bool), levels=[0., 3000.])
    composer.result = replace(composer.result, tactile_frame=frame)
    for adc, expected in ((0, neutral), (330, (neutral + yellow) / 2), (660, yellow),
                          (1950, orange), (3000, red), (0, neutral)):
        frame["values"][0] = adc
        window.refresh()
        colors = mesh.vertexColors()
        np.testing.assert_allclose(colors[0, :3] * 255, expected, atol=1e-4)
        np.testing.assert_array_equal(colors[1:], base[1:])
    frame["valid_mask"][0] = False
    window.refresh()
    np.testing.assert_allclose(mesh.vertexColors()[0, :3] * 255, [229, 232, 237], atol=1e-4)
    composer.result = replace(composer.result, tactile_frame=None)
    window.refresh()
    np.testing.assert_array_equal(mesh.vertexColors(), base)


def test_measured_palm_and_dorsum_keep_their_colors_through_motion_and_touch(left_surface_viewer):
    window, pose, composer = left_surface_viewer
    window.refresh()
    mesh = window.view.mesh_data
    reference = mesh.vertexColors().copy()
    connected_pose(pose)
    pose.snapshot["calibration"].update(profile_status="ready", calibrated_nodes=["11", "8", "7"])
    composer.result = CombinedFrame(window.resources["neutral_geometry"], None, True, "live", "实时手势 · 等待触觉")
    window.refresh()
    skin = mesh.vertexColors().copy()
    assert np.all(skin[:, 1] > skin[:, 0]) and np.all(skin[:, 2] > skin[:, 0])
    palm, dorsum = skin[0, :3], skin[3, :3]
    assert np.all(palm > dorsum)
    assert np.ptp(palm) < np.ptp(dorsum)

    # Turning the wrist must not swap the anatomical material.
    geometry = dict(composer.result.geometry)
    geometry["vertices_m"] = np.asarray(geometry["vertices_m"]) * [1., -1., -1.]
    composer.result = replace(composer.result, geometry=geometry)
    pose.snapshot["runtime"]["ipc_sequence"] += 1
    window.refresh()
    np.testing.assert_allclose(mesh.vertexColors(), skin)

    frame = dict(values=np.zeros(400), valid_mask=np.ones(400, dtype=bool), levels=[0., 3000.])
    composer.result = replace(composer.result, tactile_frame=frame)
    window.refresh()
    np.testing.assert_allclose(mesh.vertexColors(), skin)
    # Halfway to the yellow stop, blend each contact from its own rest color.
    frame["values"][:2] = 330.
    window.refresh()
    expected = (skin[[0, 6], :3] * 255 + np.array([251, 232, 165])) / 2.
    np.testing.assert_allclose(mesh.vertexColors()[[0, 6], :3] * 255, expected, atol=1e-4)
    frame["values"][1] = 0.
    for adc, expected in ((660, [251, 232, 165]), (1950, [244, 146, 64]), (3000, [195, 42, 42])):
        frame["values"][0] = adc
        window.refresh()
        np.testing.assert_allclose(mesh.vertexColors()[0, :3] * 255, expected, atol=1e-4)
        np.testing.assert_allclose(mesh.vertexColors()[1:], skin[1:])

    frame["valid_mask"][0] = False
    window.refresh()
    missing = mesh.vertexColors()[0, :3]
    assert missing[2] > missing[1] > missing[0]
    np.testing.assert_allclose(mesh.vertexColors()[1:], skin[1:])
    frame["valid_mask"][0] = True
    frame["values"][0] = 0
    window.refresh()
    np.testing.assert_allclose(mesh.vertexColors(), skin)
    frame["values"][0] = 3000
    window.refresh()
    composer.result = replace(composer.result, tactile_frame=None)
    window.refresh()
    np.testing.assert_allclose(mesh.vertexColors(), skin)

    pose.snapshot["calibration"].update(profile_status="not_calibrated", calibrated_nodes=[])
    composer.result = CombinedFrame(None, None, False, "waiting", "等待手势标定")
    window.refresh()
    np.testing.assert_array_equal(mesh.vertexColors(), reference)


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
    pose.snapshot["runtime"].update(received_samples=24, last_sample_age_s=.01)
    window.refresh()
    window.step_buttons["neutral"].click()
    pose.snapshot["calibration"]["input_scale_check"] = {"status": "changed"}
    pose.snapshot["runtime"].update(command_completed_generation=1, last_command_name="unrelated")
    window.refresh()
    assert window._pending_calibration is not None
    assert "等待后台" in window.calibration_status.text()
    pose.snapshot["runtime"].update(command_rejected_count=1, last_rejected_command_name="capture",
                                    last_rejected_command_error="queue full")
    window.refresh()
    assert window._pending_calibration is None
    assert window.step_buttons["neutral"].isEnabled()
    assert "未完成" in window.notice.text()
    assert "queue full" in window.details.toPlainText()


def test_cancel_before_capture_starts_releases_pending_with_execution_ack(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(received_samples=24, last_sample_age_s=.01)
    window.refresh()
    window.step_buttons["neutral"].click()
    window.cancel_button.click()
    pose.snapshot["runtime"].update(command_completed_generation=2, last_command_name="cancel_capture",
                                    last_command_error=None)
    window.refresh()
    assert window._pending_calibration is None
    assert window.step_buttons["neutral"].isEnabled()
    assert window.step_buttons["neutral"].progress_bar.value() == 0
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


@pytest.mark.parametrize("partial", [False, True])
def test_mounting_shows_the_retry_step_and_partial_nodes(viewer, partial):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["calibration"].update(
        profile_status="ready" if partial else "fit_failed",
        calibrated_nodes=["11", "4"] if partial else [],
        rejected_nodes={"3": "inconsistent"} if partial else {},
        retry_steps=["fingers_down"])
    window.refresh()
    text = window.calibration_status.text()
    assert "2. 指尖上倾" in text
    assert window.step_buttons["fingers_down"].text().startswith("↻")
    assert not window.functional_button.isEnabled()
    assert window.demo.step == "fingers_down"
    assert "3. 指尖下倾" not in text
    if partial:
        assert "3" in text


@pytest.mark.parametrize("slots, can_start", [
    ({"index": [1, 2], "middle": [4, 5], "ring": [10, 11], "little": [], "thumb": []}, True),
    ({"index": [1]}, True),
    ({}, False),
])
def test_partial_mounting_uses_published_joint_coverage_for_stage_four(viewer, slots, can_start):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(received_samples=348, last_sample_age_s=.01)
    calibration = pose.snapshot["calibration"]
    calibration.update(profile_status="official_core_sparse_adapter_not_accuracy_validated",
                       captured_steps=["neutral", "fingers_down", "thumb_up"],
                       calibrated_nodes=["1", "3", "4", "5", "6", "7", "8", "10", "11"],
                       rejected_nodes={"2": "inconsistent", "9": "inconsistent"},
                       retry_steps=["fingers_down", "thumb_up"], functional_slots_by_finger=slots)
    window.refresh()
    assert window.functional_button.isEnabled() is can_start
    assert window.step_buttons["fingers_down"].text().startswith("↻")
    assert "拇指" in window.calibration_status.text() and "小指" in window.calibration_status.text()
    if can_start:
        assert "4. 开始手指弯曲标定" in window.instructions.text()
        assert "可继续采集：食指" in window.calibration_status.text()
        assert "要补齐" in window.calibration_status.text()
        assert "已完成" not in window.calibration_status.text()
        window.functional_button.click()
        assert pose.commands == [("capture_functional_hand", {})]
    else:
        assert window.demo.step == "fingers_down"
        window.functional_button.click()
        assert not pose.commands


@pytest.mark.parametrize(("runtime", "status"), [
    ({"received_samples": 0}, "ready"),
    ({"last_sample_age_s": .5}, "ready"),
    ({"clear_pending": True}, "ready"),
    ({}, "fit_failed"),
])
def test_published_coverage_does_not_admit_invalid_input_or_profile(viewer, runtime, status):
    window, pose, *_ = viewer
    connected_pose(pose)
    pose.snapshot["runtime"].update(runtime)
    pose.snapshot["calibration"].update(profile_status=status, calibrated_nodes=["11", "8", "7"],
                                        functional_slots_by_finger={"index": [1, 2]})
    window.refresh()
    assert not window.functional_button.isEnabled()
    window.functional_button.click()
    assert not pose.commands


def test_retry_keeps_mounting_coverage_and_explains_why_in_both_languages(viewer):
    window, pose, *_ = viewer
    connected_pose(pose)
    calibration = pose.snapshot["calibration"]
    calibration.update(profile_status="ready", calibrated_nodes=["11", "8", "7"],
                       functional_slots_by_finger={"index": [1, 2]},
                       mounting_retry_feedback={"kept_nodes": 3, "candidate_nodes": 1})
    window.refresh()
    assert window.calibration_status.text().startswith("本次补采未改善结果，已保留上次通过标定的 3 个节点。")
    assert window.functional_button.isEnabled()
    window.set_language("en")
    assert "previous calibration for 3 sensors is still active" in window.calibration_status.text()
    calibration.pop("mounting_retry_feedback")
    window.refresh()
    assert "previous calibration" not in window.calibration_status.text()


@pytest.mark.parametrize("legacy", [True, False])
def test_partial_mounting_names_the_measured_sensor_for_each_tilt(viewer, legacy):
    window, pose, *_ = viewer
    connected_pose(pose)
    window.resources["config"] = {"node_to_slot": {"2": 7, "9": 15, "5": 5}}
    calibration = pose.snapshot["calibration"]
    residuals = {"7": (8.8, 17.6), "15": (16.1, 8.1), "5": (99., 99.)}
    diagnostics = {
        "max_rigid_residual_deg": 15.,
        "by_slot": {s: {"rigid_residual_deg": {"z_rot_pose_1": up, "z_rot_pose_2": down}}
                    for s, (up, down) in residuals.items()},
    }
    if not legacy:
        diagnostics["rigid_rejections"] = {"2": ["z_rot_pose_2"], "9": ["z_rot_pose_1"]}
    calibration.update(profile_status="ready", calibrated_nodes=["11", "8", "7"],
                       functional_slots_by_finger={"index": [1, 2]},
                       retry_steps=["fingers_down", "thumb_up"], motion_diagnostics=diagnostics,
                       rejected_nodes={"2": "节点 2 的倾转与手掌不一致（最大差 17.6°）",
                                       "9": "节点 9 的倾转与手掌不一致（最大差 16.1°）",
                                       "5": "insufficient samples"})
    window.refresh()
    label = window.calibration_status.text()
    assert "2. 指尖上倾（拇指 IMU 9）" in label
    assert "3. 指尖下倾（小指 IMU 2）" in label
    assert "中指 IMU 5" not in label  # Nonmeasured slot has a synthetic high residual.
    assert window.functional_button.isEnabled()


@pytest.mark.parametrize(("hand", "title", "english", "thumb_x"), [
    ("left", "左手三维展示", "Left Hand 3D Display", 1),
    ("right", "右手三维展示", "Right Hand 3D Display", -1),
])
def test_window_matches_its_hardware_layout_and_frames_both_views_from_thumb_side(
        hand, title, english, thumb_x, qapp, tmp_path, monkeypatch):
    bindings = []
    def bind(geometry, axes, *, hand, layout_path):
        bindings.append((hand, layout_path))
        return object()
    monkeypatch.setattr(module, "TactileSurfaceBinding", bind)
    resources = dict(neutral_geometry={}, model_axes={}, hand=hand, layout_path=tmp_path / f"{hand}.json")
    window = module.HandTactileWindow(Settings(tmp_path), resources, Pose(), Touch(),
                                     composer=Composer(), view_factory=View, port_discovery=Discovery(), demo_factory=View)
    try:
        assert bindings == [(hand, resources["layout_path"])]
        assert window.windowTitle() == window.title.text() == title
        camera = window.view.camera_position
        assert camera == window.demo.camera_position
        azimuth = np.deg2rad(camera["azimuth"])
        assert thumb_x * np.cos(azimuth) > 0 and np.sin(azimuth) < 0
        assert camera["elevation"] > 0
        window.set_language("en")
        assert window.windowTitle() == window.title.text() == english
    finally:
        window.shutdown()
        window.close()
        window.deleteLater()
        qapp.processEvents()


def test_embedded_panel_detaches_without_stopping_owned_acquisition(viewer):
    window, pose, touch, *_ = viewer
    detached = []
    window.owns_session = False
    pose.close = lambda: detached.append("pose-client")
    window.start()
    window.shutdown()
    assert detached == ["pose-client"]
    assert pose.stopped == 0 and not pose.join_times
    assert ("disconnect",) not in touch.calls
    assert ("close",) in touch.calls
    assert not window.timer.isActive()


@pytest.mark.parametrize("hand", ["left", "right"])
def test_touch_only_shared_panel_has_static_geometry_and_no_motion_lifecycle(hand, qapp, tmp_path, monkeypatch):
    bindings, commands = [], []
    def bind(geometry, axes, *, hand, layout_path):
        bindings.append((hand, layout_path))
        return object()
    monkeypatch.setattr(module, "TactileSurfaceBinding", bind)
    monkeypatch.setattr(module, "surface_colors", lambda *args: np.full((3, 4), .5))
    resources = dict(neutral_geometry={"static": hand}, model_axes={}, hand=hand,
                     layout_path=tmp_path / f"{hand}.json")
    touch, composer = Touch(), Composer()
    composer.result = CombinedFrame(None, dict(values=[1, 0], valid_mask=[True, True], levels=[0, 1]),
                                    False, "waiting", "waiting_pose", tactile_age_ms=20.)
    panel = module.HandPanel(Settings(tmp_path), resources, None, touch, composer=composer,
        view_factory=View, demo_factory=lambda *a, **kw: pytest.fail("created a motion demo"),
        port_discovery=Discovery(), connection_request=commands.append,
        connection_status=lambda kind: dict(enabled=kind == "tactile", label="启动", state="STOPPED", port="/touch"))
    try:
        panel.start()
        panel.show()
        qapp.processEvents()
        assert panel.pose_controller is None and panel.demo is None and panel.palm_guide is None
        assert not panel.motion_available and panel._snapshot == {}
        assert all(widget.isHidden() for widget in panel.connection_rows["imu"])
        assert panel.calibration_toggle.isHidden() and panel.calibration_panel.isHidden()
        assert panel.demo_card.isHidden() and panel.live_title.isHidden()
        assert panel.view.geometry == resources["neutral_geometry"] and panel.view.hand_visible
        assert not panel._combined.pose_current and not panel._combined.uses_tracked_geometry
        assert bindings == [(hand, resources["layout_path"])]
        assert "纯触觉" in panel.mode_label.text() and "等待" not in panel.mode_label.text()
        assert "20 ms" in panel.metrics.text() and "不含动捕姿态" in panel.metrics.text()
        assert np.all(panel.view.surface_colors == .5)
        panel.toggle_connection("imu")
        panel.toggle_connection("tactile")
        assert commands == ["tactile"]
        panel.start_reference()
        assert touch.zeroes == 1
        panel.response_range.setValue(1750.)
        panel.configure_response()
        assert touch.response_settings.display_range_adc == 1750.
        panel._reset_views(True)
        panel.set_language("en")
        assert "Touch only" in panel.mode_label.text()
        panel.shutdown()
        assert ("disconnect",) not in touch.calls and ("close",) in touch.calls
        assert not panel.timer.isActive() and commands == ["tactile"]
    finally:
        panel.shutdown()
        panel.deleteLater()
        qapp.processEvents()
