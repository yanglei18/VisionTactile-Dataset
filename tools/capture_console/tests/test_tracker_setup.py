"""Offscreen Qt with synthetic child scripts; never start an approved live script."""
from pathlib import Path
import subprocess
import time

import pytest
from PySide6.QtTest import QSignalSpy

from vt_capture_console.backend.tracker_workflow import PROMPTS
from vt_capture_console.ui.widgets import tracker_setup as module
from vt_capture_console.ui.widgets.tracker_setup import TrackerSetupDialog

from test_tracker_workflow import idle_snapshot, make_config


def pump(qapp, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        qapp.processEvents()
        if predicate():
            return
        time.sleep(.005)
    assert predicate()


def fill(dialog, config):
    for name, edit in dialog.fields.items():
        edit.setText(getattr(config, name))


def test_open_is_readonly_and_demo_cannot_start(qapp, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("Opening wizard must never launch a process")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    snapshot = idle_snapshot()
    snapshot["mode"] = "demo"
    dialog = TrackerSetupDialog(lambda: snapshot)
    dialog.show()
    dialog.approval.setChecked(True)
    dialog.refresh()
    assert not dialog.start_button.isEnabled()
    assert not dialog.confirm_button.isEnabled()
    assert dialog.workflow.snapshot()["state"] == "IDLE"
    assert dialog.log.toPlainText() == ""
    dialog.close()


def test_digest_is_async_not_approval_and_file_change_revokes(qapp, tmp_path):
    config = make_config(tmp_path)
    dialog = TrackerSetupDialog(idle_snapshot)
    fill(dialog, config)
    dialog.fields["expected_sha256"].clear()
    dialog.approval.setChecked(True)
    dialog.hash_button.click()
    pump(qapp, lambda: len(dialog.fields["expected_sha256"].text()) == 64)
    assert not dialog.approval.isChecked()
    assert not dialog.start_button.isEnabled()
    assert "不是批准" in dialog.hash_note.text()
    dialog.approval.setChecked(True)
    assert dialog.start_button.isEnabled()
    dialog.fields["bundle"].setText(str(tmp_path / "different-bundle"))
    assert not dialog.approval.isChecked()
    dialog.close()


def test_gui_exact_prompts_complete_without_cli(qapp, tmp_path):
    config = make_config(tmp_path)
    dialog = TrackerSetupDialog(idle_snapshot)
    fill(dialog, config)
    dialog.approval.setChecked(True)
    dialog.start_button.click()
    try:
        for prompt in PROMPTS:
            pump(qapp, lambda: dialog.workflow.snapshot()["prompt"] == prompt)
            dialog.refresh()
            assert dialog.confirm_button.isEnabled()
            assert not dialog.start_button.isEnabled()
            assert not dialog.fields["script"].isEnabled()
            dialog.confirm_button.click()
        pump(qapp, lambda: dialog.workflow.snapshot()["state"] == "COMPLETE")
        dialog.refresh()
        assert dialog.stage.text() == "初始化完成"
        assert "手动启动只读发布器" in dialog.instruction.text()
        assert not dialog.confirm_button.isEnabled()
        assert len(list(tmp_path.glob("*.pcapng"))) == 1  # Only fixture input, no recorder.
    finally:
        dialog.close()
        dialog.workflow.wait(3)


def test_closing_active_dialog_only_cancels_its_child(qapp, tmp_path):
    dialog = TrackerSetupDialog(idle_snapshot)
    fill(dialog, make_config(tmp_path))
    dialog.show()
    dialog.approval.setChecked(True)
    dialog.start_button.click()
    pump(qapp, lambda: dialog.workflow.snapshot()["state"] == "WAITING")
    dialog.close()
    pump(qapp, lambda: dialog.workflow.snapshot()["state"] == "CANCELLED")
    pump(qapp, lambda: not dialog.isVisible())
    assert dialog.workflow.wait(1)


def test_fresh_guard_at_click_blocks_stale_cached_enable(qapp, tmp_path):
    snapshot = idle_snapshot()
    dialog = TrackerSetupDialog(lambda: snapshot)
    fill(dialog, make_config(tmp_path))
    dialog.approval.setChecked(True)
    assert dialog.start_button.isEnabled()
    snapshot["management"]["trials"] = [dict(state="UNKNOWN")]
    dialog.start_button.click()
    assert dialog.workflow.snapshot()["state"] == "IDLE"
    assert "未启动" in dialog.instruction.text()
    dialog.close()


@pytest.fixture
def retry_dialog(qapp, monkeypatch):
    class FailedWorkflow:
        state = "FAILED"
        detail = "此前初始化未完成，旧流程已经结束。"

        def __init__(self):
            self.starts = []

        def snapshot(self):
            return dict(state=self.state, detail=self.detail, prompt="", logs=[])

        def start(self, config):
            self.starts.append(config)
            self.state, self.detail = "RUNNING", "新流程已开始，等待脚本提示。"

        def cancel(self):
            self.state = "CANCELLED"

    workflow = FailedWorkflow()
    monkeypatch.setattr(module, "TrackerWorkflow", lambda *args, **kwargs: workflow)
    monkeypatch.setattr(module, "detect_defaults", lambda: {})
    dialog = TrackerSetupDialog(idle_snapshot)
    yield dialog, workflow
    dialog.close()
    dialog.deleteLater()


def test_invalid_retry_error_survives_refresh_of_previous_failed_attempt(retry_dialog, tmp_path):
    dialog, workflow = retry_dialog
    fill(dialog, make_config(tmp_path))
    dialog.fields["capture"].setText(str(tmp_path / "missing-capture.pcapng"))
    dialog.approval.setChecked(True)
    dialog.start_button.click()
    error = dialog.instruction.text()
    assert "未启动" in error and "必须是现有的绝对文件路径" in error
    for _ in range(3):
        dialog.refresh()
        assert dialog.instruction.text() == error
    assert workflow.state == "FAILED" and workflow.starts == []


@pytest.mark.parametrize("repair", ["edit_input", "restore_file"])
def test_retry_error_clears_when_input_changes_or_new_attempt_starts(retry_dialog, tmp_path, repair):
    dialog, workflow = retry_dialog
    config = make_config(tmp_path)
    fill(dialog, config)
    missing_capture = tmp_path / "missing-capture.pcapng"
    dialog.fields["capture"].setText(str(missing_capture))
    dialog.approval.setChecked(True)
    dialog.start_button.click()
    assert "未启动" in dialog.instruction.text()
    if repair == "edit_input":
        dialog.fields["capture"].setText(config.capture)
        assert not dialog.approval.isChecked()
        assert dialog.instruction.text() == workflow.detail
        dialog.approval.setChecked(True)
    else:
        # The operator may restore a missing file without changing its path.
        missing_capture.write_bytes(b"synthetic fixture, no device access")
    dialog.start_button.click()
    dialog.refresh()
    assert len(workflow.starts) == 1
    assert dialog.instruction.text() == workflow.detail
    assert "未启动" not in dialog.instruction.text()


def test_close_does_not_finish_or_destroy_until_owned_child_termination(qapp):
    class ControlledWorkflow:
        def __init__(self):
            self.state = "WAITING"
            self.cancel_count = 0
        def snapshot(self):
            return dict(state=self.state, prompt="", logs=[], detail="synthetic process")
        def cancel(self):
            self.cancel_count += 1
            self.state = "CANCELLING"
    dialog = TrackerSetupDialog(idle_snapshot)
    controlled = ControlledWorkflow()
    dialog.workflow = controlled
    dialog.show()
    finished = QSignalSpy(dialog.finished)
    dialog.close()
    qapp.processEvents()
    assert controlled.cancel_count == 1
    assert dialog.isVisible() and finished.count() == 0
    assert dialog.timer.isActive()
    controlled.state = "CANCELLED"  # Emulates the worker after owned child reaping.
    dialog.refresh()
    qapp.processEvents()
    assert not dialog.isVisible() and finished.count() == 1
    assert not dialog.timer.isActive()
