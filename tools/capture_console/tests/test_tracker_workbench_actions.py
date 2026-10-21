"""Tracker handoff uses only mocked commands; no bootstrap or hardware access."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import subprocess
import time

import pytest
from PySide6.QtCore import QCoreApplication, QEvent, Signal
from PySide6.QtWidgets import QDialog

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.desktop_tools import DesktopTools
from vt_capture_console.ui.main_window import MainWindow


class Backend:
    mode = "live"
    path = "/test-only/no-manager.sock"
    supports_device_control = True

    def __init__(self, tmp_path):
        demo = DemoBackend()
        self.value = demo.snapshot()
        self.value.update(mode="live", instance_id="manager-test", process_control=demo.process_control)
        self.value["capabilities"]["device_control"] = True
        self.value["management"] = dict(enabled=False, controller=None, batches=[], trials=[], operation=None)
        self.target = next(g for g in self.value["process_control"]["groups"] if g["kind"] == "tracker")
        self.target.update(state="STOPPED", owned=False, pid=None, can_start=True, can_stop=False, can_restart=False)
        self.commands, self.latest_operation = [], None
        self.submit_error = None
        self.runtime = SimpleNamespace(
            configuration=dict(process_groups=[dict(id=self.target["id"], preset="trackers", enabled=True,
                                parameters=dict(bundle_path=str(tmp_path / "approved-bundle.json")))]),
            status=dict(state="ready", detail="synthetic"), log_path=None,
            repo_root=Path(__file__).resolve().parents[3], state_dir=tmp_path,
            settings_path=tmp_path / "workstation.json", admission_path=tmp_path / "gate",
            fresh_snapshot=self.snapshot,
        )

    def snapshot(self):
        return deepcopy(self.value)

    def submit(self, command, params):
        if self.submit_error:
            raise self.submit_error
        request = f"tracker-start-{len(self.commands)}"
        self.commands.append((command, deepcopy(params)))
        self.latest_operation = dict(request_id=request, state="unknown", code="QUEUED", detail="queued")
        return request

    def close(self):
        pass


class CompletedDialog(QDialog):
    reading_requested = Signal()

    def __init__(self, bundle, parent):
        super().__init__(parent)
        self.workflow_state = "COMPLETE"
        self.workflow = SimpleNamespace(snapshot=lambda: dict(state=self.workflow_state))
        self.started_config = SimpleNamespace(bundle=bundle)
        self.feedback = ""
        self.pending = False
        self.accepted_count = 0

    def set_reading_feedback(self, text, *, pending=False):
        self.feedback, self.pending = text, pending

    def accept(self):
        self.accepted_count += 1
        super().accept()


@pytest.fixture
def workbench(qapp, tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Tracker GUI tests must not create any hardware process")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    backend = Backend(tmp_path)
    window = MainWindow(backend)
    tools = DesktopTools(window, backend, backend.runtime)
    window.timer.stop()
    tools.timer.stop()
    dialog = CompletedDialog(str(tmp_path / "approved-bundle.json"), window)
    tools.tracker_dialog = dialog
    dialog.reading_requested.connect(tools._start_tracker_reading)
    dialog.finished.connect(tools._tracker_closed)
    window.show()
    window._navigate(2)
    window.navigation.button(2).setChecked(True)
    dialog.show()
    qapp.processEvents()
    yield window, tools, backend, dialog
    dialog.workflow_state = "COMPLETE"
    if tools.tracker_dialog is not None:
        tools.tracker_dialog.reject()
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


@pytest.mark.parametrize("state", ["STOPPED", "FAILED"])
def test_completed_bootstrap_submits_existing_device_command_then_waits_for_receipt(workbench, state):
    window, tools, backend, dialog = workbench
    backend.target["state"] = state
    dialog.reading_requested.emit()
    assert backend.commands == [("device.start", dict(group_id=backend.target["id"], expected_manager_instance_id="manager-test"))]
    assert dialog.isVisible() and dialog.accepted_count == 0 and dialog.pending
    assert window.pages.currentIndex() == 2
    assert "等待后台回执" in dialog.feedback and "验收" in dialog.feedback
    backend.latest_operation.update(state="accepted", code="started", detail="spawned")
    tools._poll_tools()
    assert tools.tracker_dialog is None and dialog.accepted_count == 1
    assert window.pages.currentIndex() == 0 and window.navigation.button(0).isChecked()
    assert window.filter.currentData() == "tracker"
    assert "不等于姿态验收" in window.statusBar().currentMessage()


def test_duplicate_clicks_and_server_queued_receipt_do_not_repeat_start(workbench):
    _, tools, backend, dialog = workbench
    dialog.reading_requested.emit()
    backend.latest_operation.update(state="accepted", code="queued")
    dialog.reading_requested.emit()
    tools._poll_tools()
    assert len(backend.commands) == 1 and dialog.pending and dialog.accepted_count == 0


@pytest.mark.parametrize("state", ["RUNNING", "STARTING", "DEGRADED"])
def test_owned_existing_reader_only_navigates(workbench, state):
    window, tools, backend, dialog = workbench
    backend.target.update(state=state, owned=True, pid=999, can_start=False)
    dialog.reading_requested.emit()
    assert backend.commands == [] and tools.tracker_dialog is None
    assert window.filter.currentData() == "tracker"
    assert "不重复启动" in window.statusBar().currentMessage()


def test_external_reader_is_never_taken_over_even_if_can_start_is_true(workbench):
    window, tools, backend, dialog = workbench
    backend.target.update(state="EXTERNAL", owned=False, pid=None, can_start=True)
    dialog.reading_requested.emit()
    assert backend.commands == [] and tools.tracker_dialog is None
    assert window.filter.currentData() == "tracker"
    assert "外部" in window.statusBar().currentMessage()
    assert "不接管" in window.statusBar().currentMessage()


@pytest.mark.parametrize("state", ["IDLE", "RUNNING", "WAITING", "FAILED", "CANCELLED"])
def test_incomplete_initialization_cannot_handoff(workbench, state):
    window, tools, backend, dialog = workbench
    dialog.workflow_state = state
    dialog.reading_requested.emit()
    assert not backend.commands and tools.tracker_dialog is dialog and dialog.isVisible()
    assert window.pages.currentIndex() == 2 and "未完整完成" in dialog.feedback


@pytest.mark.parametrize("case", ["demo", "snapshot_demo", "instance", "control", "capability", "disconnected",
                                  "disabled", "unknown", "stopping", "owned", "pid", "cannot_start",
                                  "no_group", "ambiguous_group"])
def test_unavailable_or_unsafe_snapshot_never_starts_reader(workbench, case):
    window, tools, backend, dialog = workbench
    if case == "demo":
        backend.mode = "demo"
    elif case == "snapshot_demo":
        backend.value["mode"] = "demo"
    elif case == "instance":
        backend.value["instance_id"] = ""
    elif case == "control":
        backend.value["process_control"]["enabled"] = False
    elif case == "capability":
        backend.value["capabilities"]["device_control"] = False
    elif case == "disconnected":
        backend.supports_device_control = False
    elif case in {"disabled", "unknown", "stopping"}:
        backend.target["state"] = case.upper()
    elif case == "owned":
        backend.target["owned"] = True
    elif case == "pid":
        backend.target["pid"] = 123
    elif case == "cannot_start":
        backend.target["can_start"] = False
    elif case == "no_group":
        backend.value["process_control"]["groups"].remove(backend.target)
    else:
        backend.value["process_control"]["groups"].append(deepcopy(backend.target))
    dialog.reading_requested.emit()
    assert not backend.commands and tools.tracker_dialog is dialog and dialog.isVisible()
    assert window.pages.currentIndex() == 2 and dialog.feedback


@pytest.mark.parametrize("case", ["mismatch", "no_started_config", "no_profile", "missing_bundle"])
def test_bootstrap_bundle_must_match_reader_profile(workbench, case):
    _, tools, backend, dialog = workbench
    if case == "mismatch":
        dialog.started_config.bundle += ".different"
    elif case == "no_started_config":
        dialog.started_config = None
    elif case == "no_profile":
        backend.runtime.configuration["process_groups"] = []
    else:
        backend.runtime.configuration["process_groups"][0]["parameters"].pop("bundle_path")
    dialog.reading_requested.emit()
    assert not backend.commands and tools.tracker_dialog is dialog
    assert "启动包" in dialog.feedback and not dialog.pending


def test_unrelated_pending_command_does_not_get_retargeted(workbench):
    window, _, backend, dialog = workbench
    controls = window.device_controls
    controls.pending_request, controls.pending_instance_id = "other-request", "manager-test"
    controls._submitted_at = time.monotonic()
    selected = controls.selected_id
    dialog.reading_requested.emit()
    assert not backend.commands and controls.selected_id == selected
    assert controls.pending_request == "other-request" and "已有设备命令" in dialog.feedback


def test_synchronous_submit_failure_keeps_dialog_open(workbench):
    _, tools, backend, dialog = workbench
    backend.submit_error = OSError("synthetic unavailable")
    dialog.reading_requested.emit()
    assert not backend.commands and tools.tracker_dialog is dialog and not dialog.pending
    assert "synthetic unavailable" in dialog.feedback


@pytest.mark.parametrize("source", ["latest", "snapshot"])
def test_asynchronous_rejection_keeps_completed_dialog_for_feedback(workbench, source):
    window, tools, backend, dialog = workbench
    dialog.reading_requested.emit()
    rejected = dict(request_id=tools._tracker_read_request, state="rejected", code="recorder_not_idle", detail="blocked")
    if source == "latest":
        backend.latest_operation = rejected
    else:
        backend.value["management"]["operation"] = rejected
    tools._poll_tools()
    assert tools.tracker_dialog is dialog and dialog.isVisible() and not dialog.pending
    assert "尚未确认录制空闲，暂不能操作设备。" in dialog.feedback and "不会自动重试" in dialog.feedback
    assert window.pages.currentIndex() == 2 and len(backend.commands) == 1


@pytest.mark.parametrize("case", ["manager_changed", "disconnect", "timeout"])
def test_unknown_result_never_retries_or_claims_success(workbench, case):
    _, tools, backend, dialog = workbench
    dialog.reading_requested.emit()
    if case == "manager_changed":
        backend.value["instance_id"] = "replacement-manager"
        backend.latest_operation.update(state="accepted", code="started")
    elif case == "disconnect":
        backend.supports_device_control = False
    else:
        tools._tracker_read_started -= 16
    tools._poll_tools()
    assert tools.tracker_dialog is dialog and dialog.isVisible() and not dialog.pending
    assert len(backend.commands) == 1 and "不会自动重试" in dialog.feedback


def test_unrelated_receipt_cannot_close_tracker_dialog(workbench):
    _, tools, backend, dialog = workbench
    dialog.reading_requested.emit()
    backend.latest_operation = dict(request_id="unrelated", state="accepted", code="started", detail="other device")
    tools._poll_tools()
    assert dialog.pending and tools.tracker_dialog is dialog and dialog.accepted_count == 0


def test_navigation_is_read_only_and_checks_sidebar_and_filter(workbench):
    window, _, backend, _ = workbench
    window.show_tracker_preview("只查看")
    assert not backend.commands and window.pages.currentIndex() == 0
    assert window.navigation.button(0).isChecked() and window.filter.currentData() == "tracker"
    assert window.statusBar().currentMessage() == "只查看"


def test_existing_tracker_dialog_can_be_raised_without_reloading_during_runtime_operation(workbench):
    _, tools, backend, dialog = workbench
    backend.runtime.status["state"] = "applying"
    dialog.hide()
    tools.open_tracker()
    assert tools.tracker_dialog is dialog and dialog.isVisible()
    assert not backend.commands


def test_real_dialog_signal_and_runtime_local_settings_are_connected(workbench, monkeypatch):
    window, tools, backend, previous = workbench
    previous.reject()
    tools.open_tracker()
    dialog = tools.tracker_dialog
    assert dialog.settings.repo_root == backend.runtime.repo_root
    assert dialog.settings.path.parent == backend.runtime.settings_path.parent
    assert not dialog.settings.path.exists()  # Opening the dialog does not save or initialize.
    assert not dialog.reading_button.isVisible()
    monkeypatch.setattr(dialog.workflow, "snapshot", lambda: dict(state="COMPLETE", detail="synthetic completion",
                                                                 prompt="", logs=[]))
    dialog.started_config = SimpleNamespace(bundle=backend.runtime.configuration["process_groups"][0]["parameters"]["bundle_path"])
    dialog.refresh()
    assert dialog.reading_button.isVisible() and dialog.reading_button.isEnabled()
    dialog.reading_button.click()
    assert len(backend.commands) == 1 and not dialog.reading_button.isEnabled()
    assert "等待后台回执" in dialog.reading_note.text()
    backend.latest_operation.update(state="rejected", code="synthetic_refusal", detail="no hardware")
    tools._poll_tools()
    assert tools.tracker_dialog is dialog and dialog.isVisible() and dialog.reading_button.isEnabled()
    assert "synthetic_refusal" in dialog.reading_note.text()
    assert window.pages.currentIndex() == 2
