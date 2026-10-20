"""Private package import with fake tasks; no deployment or hardware actions."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
from PySide6.QtTest import QSignalSpy

from test_workstation_settings_ui import Runtime, dispose
from vt_capture_console import i18n
from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.main_window import MainWindow
from vt_capture_console.ui.widgets.tracker_deployment import TrackerDeploymentDialog
from vt_capture_console.ui.widgets.workstation_settings import WorkstationSettingsDialog


def installation():
    return SimpleNamespace(installation_id="fixture", source=Path("/private/fixture/source"),
                           site_packages=Path("/private/fixture/venv/lib/python3.12/site-packages"),
                           python=Path("/private/fixture/venv/bin/python"),
                           bundle=Path("/private/fixture/device/bundle.json"),
                           roles=Path("/private/fixture/device/roles.yaml"),
                           capture=Path("/private/fixture/device/capture.pcapng"), capture_sha256="a" * 64)


class Task:
    def __init__(self):
        self.status = dict(state="IDLE", stage="", error="", installation=None)
        self.starts = []
        self.cancels = 0

    @property
    def running(self):
        return self.status["state"] in {"RUNNING", "CANCELLING"}

    def start(self, package):
        self.starts.append(package)
        self.status.update(state="RUNNING", stage="verify")

    def cancel(self):
        self.cancels += 1
        self.status["state"] = "CANCELLING"


def test_dialog_open_is_inert_and_success_emits_once(qapp):
    task = Task()
    dialog = TrackerDeploymentDialog(task=task)
    delivered = QSignalSpy(dialog.installed)
    try:
        dialog.show()
        assert not task.starts and not dialog.install_button.isEnabled()
        dialog.package.setText("/local/交付.vttracker")
        dialog.refresh()
        dialog.install_button.click()
        assert task.starts == ["/local/交付.vttracker"]
        assert not dialog.browse_button.isEnabled()
        assert not dialog.install_button.isEnabled()
        assert "校验" in dialog.status_label.text()
        task.status.update(state="COMPLETE", installation=installation())
        dialog.refresh()
        dialog.refresh()
        assert delivered.count() == 1 and "安装完成" in dialog.status_label.text()
    finally:
        dispose(dialog, qapp)


def test_selecting_another_package_does_not_redeliver_previous_installation(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import tracker_deployment as module
    task = Task()
    dialog = TrackerDeploymentDialog(task=task)
    delivered = QSignalSpy(dialog.installed)
    try:
        dialog.package.setText("/local/first.vttracker")
        dialog.start()
        task.status.update(state="COMPLETE", installation=installation())
        dialog.refresh()
        assert delivered.count() == 1
        monkeypatch.setattr(module.QFileDialog, "getOpenFileName",
                            lambda *args: ("/local/second.vttracker", ""))
        dialog._browse()
        assert delivered.count() == 1
        assert "安装完成" not in dialog.status_label.text()
        dialog.start()
        assert task.starts == ["/local/first.vttracker", "/local/second.vttracker"]
    finally:
        task.status["state"] = "CANCELLED"
        dispose(dialog, qapp)


def test_dialog_close_cancels_and_waits_without_delivering_result(qapp):
    task = Task()
    dialog = TrackerDeploymentDialog(task=task)
    delivered, closed = QSignalSpy(dialog.installed), QSignalSpy(dialog.closed)
    dialog.show()
    dialog.package.setText("/local/package.vttracker")
    dialog.start()
    dialog.close()
    assert task.cancels == 1 and dialog.isVisible()
    assert closed.count() == 0 and dialog.timer.isActive()
    task.status.update(state="COMPLETE", installation=installation())
    dialog.refresh()
    assert closed.count() == 1 and not dialog.isVisible()
    assert delivered.count() == 0 and not dialog.timer.isActive()
    dispose(dialog, qapp)


def test_tracker_import_fills_only_draft_and_does_not_enable_or_save(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import workstation_settings as settings_module
    task = Task()
    monkeypatch.setattr(settings_module, "TrackerDeploymentDialog",
                        lambda parent, **kwargs: TrackerDeploymentDialog(parent, task=task, **kwargs))
    runtime = Runtime()
    runtime.configuration["process_groups"][1]["id"] = "custom_trackers"
    original = deepcopy(runtime.configuration)
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    try:
        assert not dialog.enabled_checks["trackers"].isChecked()
        assert dialog.import_tracker_button.isEnabled()
        dialog.import_tracker_button.click()
        child = dialog.deployment_dialog
        child.package.setText("/local/package.vttracker")
        child.start()
        dialog.refresh_status()
        assert not dialog.apply_button.isEnabled()
        assert not dialog.detect_environment_button.isEnabled()
        dialog.apply()
        assert not runtime.applications
        result = installation()
        task.status.update(state="COMPLETE", installation=result)
        child.refresh()
        draft = dialog.configuration()
        tracker = draft["process_groups"][1]
        assert tracker["id"] == "custom_trackers" and not tracker["enabled"]
        assert tracker["parameters"]["bundle_path"] == str(result.bundle)
        assert tracker["parameters"]["role_map_path"] == str(result.roles)
        assert tracker["parameters"]["pyvut_source"] == str(result.source)
        assert tracker["parameters"]["pyvut_site_packages"] == str(result.site_packages)
        assert tracker["parameters"]["frame_id"] == "custom_tracker_frame"
        assert draft["metadata_refs"] == original["metadata_refs"]
        assert runtime.configuration == original and not runtime.applications and runtime.starts == 0
        assert "保存并应用" in dialog.status_label.text()
        dialog.cancel_button.click()
        assert runtime.configuration == original and not runtime.applications
    finally:
        dispose(dialog, qapp)


@pytest.mark.parametrize("state,recording", [("starting", False), ("applying", False), ("ready", True)])
def test_import_is_blocked_during_backend_operations_or_recording(qapp, state, recording):
    runtime = Runtime()
    runtime.status["state"] = state
    snapshot = {"recording": {"state": "RECORDING" if recording else "IDLE"}}
    dialog = WorkstationSettingsDialog(runtime, lambda: snapshot)
    try:
        assert not dialog.import_tracker_button.isEnabled()
        dialog._open_tracker_deployment()
        assert dialog.deployment_dialog is None
        assert not runtime.applications
    finally:
        dispose(dialog, qapp)


def test_settings_close_waits_for_deployment_cleanup_without_saving(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import workstation_settings as settings_module
    task = Task()
    monkeypatch.setattr(settings_module, "TrackerDeploymentDialog",
                        lambda parent, **kwargs: TrackerDeploymentDialog(parent, task=task, **kwargs))
    runtime = Runtime()
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    finished = QSignalSpy(dialog.finished)
    dialog.show()
    dialog._open_tracker_deployment()
    child = dialog.deployment_dialog
    child.package.setText("/local/package.vttracker")
    child.start()
    dialog.cancel_button.click()
    assert dialog.deployment_running and finished.count() == 0
    assert task.cancels == 1
    task.status.update(state="CANCELLED")
    child.refresh()
    assert finished.count() == 1 and not dialog.isVisible()
    assert not runtime.applications
    dispose(dialog, qapp)


def test_main_window_close_waits_for_owned_installation_cleanup(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import workstation_settings as settings_module
    task = Task()
    monkeypatch.setattr(settings_module, "TrackerDeploymentDialog",
                        lambda parent, **kwargs: TrackerDeploymentDialog(parent, task=task, **kwargs))
    backend = DemoBackend()
    backend.runtime = Runtime()
    backend_closed = []
    monkeypatch.setattr(backend, "close", lambda: backend_closed.append(True))
    window = MainWindow(backend)
    try:
        window.show()
        window._open_workstation_settings()
        settings = window.settings_dialog
        settings._open_tracker_deployment()
        child = settings.deployment_dialog
        child.package.setText("/local/package.vttracker")
        child.start()
        window.close()
        assert window.isVisible() and window.timer.isActive()
        assert not backend_closed and task.cancels == 1
        task.status.update(state="CANCELLED")
        child.refresh()
        qapp.processEvents()
        assert not window.isVisible() and backend_closed == [True]
        assert not backend.runtime.applications
    finally:
        task.status["state"] = "CANCELLED"
        dispose(window, qapp)


def test_import_dialog_retranslates_without_restarting_task(qapp):
    previous = i18n.language()
    task = Task()
    dialog = TrackerDeploymentDialog(task=task)
    try:
        dialog.package.setText("/local/私有包.vttracker")
        dialog.start()
        task.status["stage"] = "environment"
        dialog.refresh()
        i18n.set_language("en", persist=False)
        assert "isolated Python" in dialog.status_label.text()
        assert dialog.package.text() == "/local/私有包.vttracker"
        assert task.starts == ["/local/私有包.vttracker"]
        assert dialog.close_button.text() == "Cancel Installation"
    finally:
        task.status["state"] = "CANCELLED"
        dispose(dialog, qapp)
        i18n.set_language(previous, persist=False)


@pytest.mark.parametrize("error,expected", [
    ("runtime_dependency", "libhidapi-hidraw0"),
    ("venv_failed", "python3-venv"),
    ("unsupported_python", "Python 3.12"),
    ("runtime_validation", "角色"),
])
def test_install_failure_shows_actionable_safe_message(qapp, error, expected):
    task = Task()
    task.status.update(state="FAILED", error=error)
    dialog = TrackerDeploymentDialog(task=task)
    try:
        assert expected in dialog.status_label.text()
    finally:
        dispose(dialog, qapp)


def test_missing_installation_metadata_has_local_hint_without_blocking_other_modules(qapp):
    runtime = Runtime()
    runtime.discovery = SimpleNamespace(
        configuration=deepcopy(runtime.configuration), updated_fields=(),
        findings=(SimpleNamespace(group_id="trackers", key="installation_manifest",
                                  status="missing", value="/private/current.json", candidates=()),))
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    try:
        dialog.detect_environment()
        assert "Tracker 安装清单" in dialog.import_tracker_button.toolTip()
        assert "/private/current.json" in dialog.import_tracker_button.toolTip()
        assert dialog.import_tracker_button.isEnabled() and dialog.apply_button.isEnabled()
        assert "必需配置待设置" not in dialog.status_label.text()
        assert not runtime.applications
    finally:
        dispose(dialog, qapp)
