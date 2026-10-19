"""v4 process controls, using memory-only adapters and offscreen Qt."""

from copy import deepcopy
import subprocess

import pytest
from PySide6.QtCore import QPoint, Qt
from PySide6.QtGui import QFont

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.main_window import MainWindow
from vt_capture_core.protocol import decode_message, encode_message


class DeviceBackend:
    mode = "live"
    path = "/test-only/never-opened.sock"
    supports_device_control = True

    def __init__(self):
        demo = DemoBackend()
        self.value = demo.snapshot()
        self.value.update(mode="live", instance_id="manager-test")
        self.value["capabilities"]["device_control"] = True
        self.value["process_control"] = demo.process_control
        self.value["management"] = {"enabled": False, "controller": None,
                                    "batches": [], "trials": [], "operation": None}
        self.commands = []
        self.latest_operation = None
        self.closed = False

    def snapshot(self):
        return deepcopy(self.value)

    def submit(self, command, params):
        request_id = f"device-request-{len(self.commands)}"
        self.commands.append((command, deepcopy(params)))
        self.latest_operation = {"request_id": request_id, "state": "unknown", "code": "QUEUED",
                                 "detail": "queued", "trial_id": ""}
        return request_id

    def close(self):
        self.closed = True


@pytest.fixture
def device_window(qapp):
    backend = DeviceBackend()
    window = MainWindow(backend)
    window.show()
    window.navigation.button(2).click()
    qapp.processEvents()
    window.refresh()
    yield window, backend, window.device_controls
    window.close()
    qapp.processEvents()


def test_demo_groups_preserve_v1_schema_and_have_no_actual_pid_or_log():
    backend = DemoBackend()
    snapshot = backend.snapshot()
    message = dict(version=1, request_id="v1", ok=True, snapshot=snapshot)
    assert decode_message(encode_message(message)) == message
    assert "process_control" not in snapshot
    assert snapshot["capabilities"]["device_control"] is False
    assert backend.supports_device_control
    assert len(backend.process_control["groups"]) == 6
    assert all(g["pid"] is None and g["log_path"] == "" for g in backend.process_control["groups"])


def test_demo_device_actions_are_idempotent_and_memory_only(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    def forbidden(*args, **kwargs):
        raise AssertionError("DEMO must not launch processes")

    monkeypatch.setattr(subprocess, "Popen", forbidden)
    backend = DemoBackend()
    params = {"group_id": "cameras", "expected_manager_instance_id": "demo-in-memory"}
    backend.submit("device.stop", params, "stop-1")
    assert backend.process_control["groups"][0]["state"] == "STOPPED"
    assert all(d["status"] == "offline" for d in backend.snapshot()["devices"] if d["kind"] == "camera")
    event_count = len(backend.events)
    backend.submit("device.stop", params, "stop-1")
    assert len(backend.events) == event_count
    with pytest.raises(ValueError, match="冲突"):
        backend.submit("device.start", params, "stop-1")
    backend.submit("device.start", params)
    backend.submit("device.restart", params)
    assert backend.process_control["groups"][0]["state"] == "RUNNING"
    assert backend.latest_operation["state"] == "accepted"
    assert list(tmp_path.iterdir()) == []


def test_demo_rejects_stale_instance_and_recording_conflicts():
    backend = DemoBackend()
    with pytest.raises(ValueError, match="实例"):
        backend.submit("device.stop", {"group_id": "cameras", "expected_manager_instance_id": "old"})
    backend.start_recording(30)
    assert not any(g["can_stop"] or g["can_restart"] for g in backend.process_control["groups"])
    with pytest.raises(ValueError, match="不允许"):
        backend.submit("device.stop", {"group_id": "cameras", "expected_manager_instance_id": "demo-in-memory"})
    backend.close()
    assert backend.state == "RECORDING"
    assert all(g["state"] == "RUNNING" for g in backend.process_control["groups"])


def test_controls_independent_of_recording_enable_and_bound_to_instance(device_window):
    window, backend, controls = device_window
    assert not window.management["enabled"]
    assert not window.start_button.isEnabled()
    assert controls.stop_button.isEnabled()
    controls.stop_button.click()
    assert backend.commands == [("device.stop", {"group_id": "cameras",
                                                "expected_manager_instance_id": "manager-test"})]
    assert controls.pending_request
    assert not controls.stop_button.isEnabled()
    window.refresh()
    controls.stop_button.click()
    assert len(backend.commands) == 1
    assert controls.pending_request  # QUEUED is not success.
    backend.latest_operation.update(state="accepted", code="queued", detail="等待进程状态")
    window.refresh()
    assert controls.pending_request
    assert not controls.stop_button.isEnabled()
    backend.value["management"]["operation"] = {**backend.latest_operation, "code": "stopped"}
    window.refresh()
    assert controls.pending_request is None
    assert "以进程组状态为准" in controls.operation_notice.text()
    assert controls.table.item(0, 2).text() == "运行中"  # No fabricated STOPPED.


def test_missing_controller_shows_next_step_without_hiding_unresolved_trials(device_window):
    window, backend, _ = device_window
    backend.value["management"]["enabled"] = True
    backend.value["recording"]["state"] = "UNAVAILABLE"
    backend.value["recording"]["detail"] = "No fresh CaptureStatus received."
    window.refresh()
    assert window.record_state.text() == "录制 · 等待控制器"
    assert "设备" in window.record_detail.text()
    assert "录制控制器" in window.record_detail.text()
    backend.value["management"]["trials"] = [{"id": "unresolved", "state": "UNKNOWN"}]
    # The focused rendering path uses the same state guard; provide a complete
    # trial row from the demo manager to exercise the full refresh safely.
    demo = DemoBackend()
    demo.start_recording(30)
    row = deepcopy(demo.management["trials"][0])
    row["state"] = "UNKNOWN"
    backend.value["management"]["trials"] = [row]
    window.refresh()
    assert "等待控制器" not in window.record_state.text()
    assert window.record_detail.text() == "No fresh CaptureStatus received."


@pytest.mark.parametrize("action", ["start", "stop", "restart"])
def test_authoritative_flags_disable_actions(device_window, action):
    window, backend, controls = device_window
    group = backend.value["process_control"]["groups"][0]
    group.update(state="STOPPED" if action == "start" else "RUNNING", owned=True)
    group["can_" + action] = False
    window.refresh()
    assert not getattr(controls, action + "_button").isEnabled()
    controls.submit(action)
    assert backend.commands == []


@pytest.mark.parametrize("action", ["start", "stop", "restart"])
def test_external_group_cannot_be_controlled_even_with_bad_flags(device_window, action):
    window, backend, controls = device_window
    group = backend.value["process_control"]["groups"][0]
    group.update(state="EXTERNAL", owned=False, pid=None, can_start=True, can_stop=True, can_restart=True)
    window.refresh()
    controls.submit(action)
    assert backend.commands == []
    assert not getattr(controls, action + "_button").isEnabled()
    assert "外部" in controls.table.item(0, 3).text()


@pytest.mark.parametrize("action", ["stop", "restart"])
def test_non_owned_running_process_cannot_be_stopped(device_window, action):
    window, backend, controls = device_window
    backend.value["process_control"]["groups"][0].update(owned=False, can_stop=True, can_restart=True)
    window.refresh()
    controls.submit(action)
    assert backend.commands == []


@pytest.mark.parametrize("refresh_first", [False, True])
def test_disconnect_or_staleness_disables_even_cached_true_flags(device_window, refresh_first):
    window, backend, controls = device_window
    assert controls.stop_button.isEnabled()
    backend.supports_device_control = False
    if refresh_first:
        window.refresh()
    controls.submit("stop")
    assert backend.commands == []
    assert not controls.stop_button.isEnabled()
    assert controls.table.rowCount() == 6  # Historical groups remain inspectable.


def test_pending_command_disconnect_or_manager_change_is_never_retried(device_window):
    window, backend, controls = device_window
    controls.restart_button.click()
    backend.value["instance_id"] = "manager-restarted"
    for _ in range(3):
        window.refresh()
    assert len(backend.commands) == 1
    assert controls.pending_request is None
    assert "待核实" in controls.operation_notice.text()


def test_superseded_receipt_does_not_leave_pending_forever(device_window, monkeypatch):
    from vt_capture_console.ui.widgets import device_controls
    window, backend, controls = device_window
    controls.stop_button.click()
    backend.latest_operation["request_id"] = "some-other-command"
    monkeypatch.setattr(device_controls.time, "monotonic", lambda: controls._submitted_at + 16)
    window.refresh()
    assert controls.pending_request is None
    assert len(backend.commands) == 1
    assert "不会自动重试" in controls.operation_notice.text()


def test_selection_follows_id_not_row_and_disappearance_never_retargets(device_window):
    window, backend, controls = device_window
    controls.table.selectRow(3)
    selected = controls.selected_id
    groups = backend.value["process_control"]["groups"]
    groups.reverse()
    window.refresh()
    assert controls.selected_id == selected
    item = controls.table.item(controls.table.currentRow(), 0)
    assert item.data(Qt.ItemDataRole.UserRole) == selected
    removed = next(g for g in groups if g["id"] == selected)
    groups.remove(removed)
    window.refresh()
    assert controls.selected_id == selected
    assert controls.table.currentRow() == -1
    assert not controls.stop_button.isEnabled()
    groups.append(removed)
    window.refresh()
    assert controls.table.currentRow() == len(groups) - 1


def test_log_path_is_literal_readonly_and_all_streams_still_displayed(device_window):
    window, backend, controls = device_window
    path = "/test-only/日志/<not-markup>; do-not-run.log"
    backend.value["process_control"]["groups"][0]["log_path"] = path
    # Production's 13-stream set is not constrained by the six process groups.
    for i in range(3):
        device = deepcopy(backend.value["devices"][i])
        device.update(id=f"depth-{i}", label=f"深度 {i}", topic=f"/camera_{i}/depth/image_rect_raw")
        backend.value["devices"].append(device)
    window.refresh()
    assert controls.log_path.isReadOnly()
    assert controls.log_path.text() == path
    assert window.device_table.rowCount() == 13
    assert backend.commands == []


def test_close_disconnects_without_stopping_devices_or_recording(device_window):
    window, backend, controls = device_window
    assert controls.stop_button.isEnabled()
    window.close()
    assert backend.closed
    assert backend.commands == []


@pytest.mark.parametrize("size", [(1080, 720), (1366, 768), (1440, 900)])
def test_themed_small_screen_preserves_tables_and_controls(device_window, qapp, size):
    from vt_capture_console.theme import stylesheet
    window, _, controls = device_window
    old_font, old_style = QFont(qapp.font()), qapp.styleSheet()
    font = QFont()
    font.setFamilies(["Inter", "Noto Sans", "Noto Sans CJK SC"])
    font.setPointSize(10)
    qapp.setFont(font)
    qapp.setStyleSheet(stylesheet())
    try:
        window.resize(*size)
        qapp.processEvents()
        window.refresh()
        qapp.processEvents()
        assert (window.width(), window.height()) == size
        assert window.minimumSizeHint().height() <= size[1]
        assert window.device_tabs.currentIndex() == 0
        assert not window.device_table.isVisible()
        for widget in (controls.table, controls.start_button, controls.stop_button,
                       controls.restart_button, controls.log_path, window.footer):
            assert widget.isVisible()
            bottom_right = widget.mapTo(window, QPoint(widget.width() - 1, widget.height() - 1))
            assert window.rect().contains(bottom_right)
        assert controls.table.viewport().height() >= 25

        window.device_tabs.setCurrentIndex(1)
        qapp.processEvents()
        assert not controls.table.isVisible()
        for widget in (window.device_table, window.footer):
            assert widget.isVisible()
            bottom_right = widget.mapTo(window, QPoint(widget.width() - 1, widget.height() - 1))
            assert window.rect().contains(bottom_right)
        assert window.device_table.viewport().height() >= 25
    finally:
        qapp.setFont(old_font)
        qapp.setStyleSheet(old_style)
