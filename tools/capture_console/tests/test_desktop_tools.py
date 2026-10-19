from pathlib import Path
from types import SimpleNamespace

from PySide6.QtCore import QCoreApplication, QEvent

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.desktop_tools import DesktopTools
from vt_capture_console.ui.main_window import MainWindow


def test_menu_construction_and_tracker_reopen_do_not_start_processes(qapp, tmp_path, monkeypatch):
    def no_spawn(*args, **kwargs):
        raise AssertionError("Opening a tool must not initialize hardware")
    monkeypatch.setattr("subprocess.Popen", no_spawn)
    backend = DemoBackend()
    runtime = SimpleNamespace(repo_root=Path(__file__).resolve().parents[3],
                              state_dir=tmp_path, admission_path=tmp_path / "gate",
                              settings_path=tmp_path / "workstation.json",
                              fresh_snapshot=backend.snapshot)
    window = MainWindow(backend)
    tools = DesktopTools(window, backend, runtime)
    try:
        assert len(tools.button.menu().actions()) == 7
        tools.open_tracker()
        first = tools.tracker_dialog
        assert first.isVisible() and not first.start_button.isEnabled()
        first.reject()
        assert tools.tracker_dialog is None
        tools.open_tracker()
        assert tools.tracker_dialog is not first
        assert tools.tracker_dialog.timer.isActive()
        tools.tracker_dialog.reject()
    finally:
        window.close()
        window.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        qapp.processEvents()


def test_camera_calibration_uses_embedded_workspace_and_preserves_page_indices(qapp, tmp_path, monkeypatch):
    def no_spawn(*args, **kwargs):
        raise AssertionError("The calibration entry must not launch a standalone window")
    monkeypatch.setattr("subprocess.Popen", no_spawn)
    backend = DemoBackend()
    runtime = SimpleNamespace(repo_root=Path(__file__).resolve().parents[3], state_dir=tmp_path)
    window = MainWindow(backend)
    tools = DesktopTools(window, backend, runtime)
    try:
        assert tools.button.text() == "标定与离线工具"
        assert tools.calibration_action.text() == "相机–追踪器标定"
        assert window.calibration_workspace is None
        before = {widget for widget in qapp.topLevelWidgets() if widget.isVisible()}
        tools.open_calibration()
        first = window.calibration_workspace
        assert window.pages.currentIndex() == 2
        assert window.device_workspace.currentWidget() is first
        assert not first.isWindow()
        assert {widget for widget in qapp.topLevelWidgets() if widget.isVisible()} == before
        tools.open_calibration()
        assert window.calibration_workspace is first
        first.back_button.click()
        assert window.device_workspace.currentIndex() == 0
        assert window.pages.count() == 5
    finally:
        window.close()
        window.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        qapp.processEvents()


def test_parent_close_waits_for_owned_offline_task_without_stopping_sensors(qapp, tmp_path):
    backend = DemoBackend()
    runtime = SimpleNamespace(repo_root=Path(__file__).resolve().parents[3], state_dir=tmp_path)
    window = MainWindow(backend)
    tools = DesktopTools(window, backend, runtime)
    calls = []
    task_dialog = SimpleNamespace(running=True, close=lambda: calls.append("cancel tool"))
    tools.alignment_dialog = task_dialog
    window.show()
    qapp.processEvents()
    try:
        window.close()
        assert window.isVisible() and tools._closing
        assert calls == ["cancel tool"]
        task_dialog.running = False
        tools._poll_tools()
        assert not window.isVisible()
    finally:
        tools.alignment_dialog = None
        window.close()
        window.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        qapp.processEvents()
