"""Manual diagnostic controls use snapshot-only fakes: no serial or files open."""

from copy import deepcopy
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5")
from PyQt5 import QtWidgets

from vt_glove_pose import gui


@pytest.fixture(scope="module")
def app():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["diagnostic-gui-tests"])
    application.setQuitOnLastWindowClosed(False)
    return application


class SnapshotWorker:
    def __init__(self):
        self.snapshot = {
            "runtime": {"connected": True, "running": True, "error": "", "message": "采集中"},
            "calibration": {"active": None, "neutral_ready": True},
            "model": {"fingers": {}}, "nodes": {},
        }
        self.commands = []
        self.started = self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def wait(self, timeout):
        return True

    def latest_snapshot(self):
        return deepcopy(self.snapshot)

    def submit(self, command, **arguments):
        self.commands.append((command, arguments))
        return True


@pytest.fixture
def make_window(app, monkeypatch):
    monkeypatch.setattr(gui, "list_serial_ports", lambda: [])
    windows = []

    def create(mode="live"):
        worker = SnapshotWorker()
        window = gui.HandPoseWindow({}, mode=mode, worker=worker)
        windows.append(window)
        return window, worker

    yield create
    for window in windows:
        window.close()
    app.processEvents()


def diagnostic(worker, state="idle", **extra):
    worker.snapshot["runtime"]["diagnostic"] = {"state": state, "path": None, "error": "", **extra}


def test_diagnostic_controls_are_manual_and_compatible_with_absent_runtime_field(make_window):
    window, worker = make_window()
    assert worker.started and worker.commands == []
    assert not window.diagnostic_controls.isHidden()
    assert window.diagnostic_start_button.isEnabled()
    assert not window.diagnostic_stop_button.isEnabled()
    assert not window.diagnostic_marker_button.isEnabled()
    assert "未开始" in window.diagnostic_label.text()
    assert window.nodes_table.rowCount() == 12
    assert window.angles_table.rowCount() == 5
    assert window.styleSheet() == ""


@pytest.mark.parametrize("mode", ["demo", "replay"])
def test_non_live_modes_hide_diagnostics_and_reject_direct_actions(make_window, mode):
    window, worker = make_window(mode)
    diagnostic(worker, "recording")
    window._refresh_snapshot()
    assert window.diagnostic_controls.isHidden()
    assert not window.diagnostic_start_button.isEnabled()
    assert not window.diagnostic_stop_button.isEnabled()
    assert not window.diagnostic_marker_button.isEnabled()
    window._start_diagnostic()
    window._stop_diagnostic()
    window._mark_diagnostic()
    assert worker.commands == []


@pytest.mark.parametrize("connected,running,active,state,allowed", [
    (True, True, None, "idle", True),
    (True, True, None, "complete", True),
    (True, True, None, "failed", True),
    (False, True, None, "idle", False),
    (True, False, None, "idle", False),
    (True, None, None, "idle", False),
    (True, True, {"kind": "neutral"}, "idle", False),
    (True, True, {"kind": "axis"}, "idle", False),
    (True, True, None, "recording", False),
    (True, True, None, "stopping", False),
])
def test_start_gates_use_current_connection_worker_calibration_and_writer_state(
        make_window, monkeypatch, connected, running, active, state, allowed):
    window, worker = make_window()
    worker.snapshot["runtime"].update(connected=connected, running=running)
    worker.snapshot["calibration"]["active"] = active
    diagnostic(worker, state)
    window._refresh_snapshot()
    assert window.diagnostic_start_button.isEnabled() is allowed
    dialogs = []
    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", lambda *args, **kwargs: (dialogs.append(True) or "", ""))
    window._start_diagnostic()
    assert bool(dialogs) is allowed
    assert worker.commands == []


def test_new_jsonl_is_only_queued_after_manual_file_choice(make_window, monkeypatch, tmp_path):
    window, worker = make_window()
    target = tmp_path / "session"
    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", lambda *args, **kwargs: (str(target), "JSONL"))
    window.diagnostic_start_button.click()
    assert worker.commands == [("diagnostic_start", {"path": str(target.with_suffix(".jsonl"))})]
    assert list(tmp_path.iterdir()) == []  # Only the worker may open files.
    assert not any(command == "connect" for command, _ in worker.commands)


@pytest.mark.parametrize("kind", ["existing", "broken_symlink", "wrong_suffix"])
def test_start_rejects_existing_paths_and_other_formats_without_overwrite(make_window, monkeypatch, tmp_path, kind):
    window, worker = make_window()
    target = tmp_path / ("existing.csv" if kind == "wrong_suffix" else "existing.jsonl")
    if kind == "existing":
        target.write_text("keep", encoding="utf-8")
    elif kind == "broken_symlink":
        target.symlink_to(tmp_path / "absent")
    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", lambda *args, **kwargs: (str(target), "JSONL"))
    window._start_diagnostic()
    assert worker.commands == []
    assert "后缀" in window.status_label.text() if kind == "wrong_suffix" else "不会覆盖" in window.status_label.text()
    if kind == "existing":
        assert target.read_text(encoding="utf-8") == "keep"


def test_state_is_rechecked_after_native_file_dialog(make_window, monkeypatch, tmp_path):
    window, worker = make_window()

    def choose(*args, **kwargs):
        worker.snapshot["runtime"]["connected"] = False
        return str(tmp_path / "new.jsonl"), "JSONL"

    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", choose)
    window._start_diagnostic()
    assert worker.commands == []


def test_markers_and_stop_only_operate_during_recording(make_window):
    window, worker = make_window()
    diagnostic(worker, "recording", path="/fake/capture.jsonl")
    window._refresh_snapshot()
    assert window.diagnostic_stop_button.isEnabled()
    assert window.diagnostic_marker_button.isEnabled()
    for index, value in enumerate(("rest", "moving", "returned")):
        window.diagnostic_marker_combo.setCurrentIndex(index)
        window.diagnostic_marker_button.click()
        assert worker.commands[-1] == ("diagnostic_marker", {"label": value})
    window.diagnostic_stop_button.click()
    assert worker.commands[-1] == ("diagnostic_stop", {})
    diagnostic(worker, "stopping", path="/fake/capture.jsonl")
    window._refresh_snapshot()
    assert "停止中" in window.diagnostic_label.text()
    assert "尚未完成" in window.diagnostic_label.text()
    assert not window.diagnostic_start_button.isEnabled()
    assert not window.diagnostic_stop_button.isEnabled()
    assert not window.diagnostic_marker_combo.isEnabled()
    previous = list(worker.commands)
    window._stop_diagnostic()
    window._mark_diagnostic()
    assert worker.commands == previous
    diagnostic(worker, "complete", path="/fake/capture.jsonl")
    window._refresh_snapshot()
    assert "已完成" in window.diagnostic_label.text()
    assert window.diagnostic_start_button.isEnabled()


def test_diagnostic_failure_is_visible_but_keeps_acquisition_controls_operable(make_window):
    window, worker = make_window()
    diagnostic(worker, "failed", path="/fake/capture.jsonl", error="ENOSPC: no room for trace")
    window._refresh_snapshot()
    assert "失败" in window.diagnostic_label.text()
    assert "ENOSPC" in window.diagnostic_label.text()
    assert "/fake/capture.jsonl" in window.diagnostic_label.toolTip()
    assert worker.snapshot["runtime"]["connected"]
    assert window.disconnect_button.isEnabled()
    assert window.record_button.isEnabled()
    assert window.diagnostic_start_button.isEnabled()
    assert worker.commands == []


def test_fatal_worker_disables_all_diagnostic_actions(make_window):
    window, worker = make_window()
    diagnostic(worker, "recording")
    worker.snapshot["runtime"].update(running=False, connected=False, error="fatal")
    window._refresh_snapshot()
    for widget in (window.diagnostic_start_button, window.diagnostic_stop_button,
                   window.diagnostic_marker_combo, window.diagnostic_marker_button):
        assert not widget.isEnabled()
    window._start_diagnostic()
    window._stop_diagnostic()
    window._mark_diagnostic()
    assert worker.commands == []


def test_failed_but_still_blocked_writer_cannot_be_replaced(make_window):
    window, worker = make_window()
    diagnostic(worker, "failed", writer_alive=True, error="blocked file system")
    window._refresh_snapshot()
    assert not window.diagnostic_start_button.isEnabled()
    assert window.disconnect_button.isEnabled()
    window._start_diagnostic()
    assert worker.commands == []
