"""Wizard persistence and read-after-completion UI; no real settings or HID."""
from copy import deepcopy
from dataclasses import asdict
import subprocess

import pytest
from PySide6.QtCore import QCoreApplication, QEvent
from PySide6.QtTest import QSignalSpy

from vt_capture_console.backend.tracker_workflow import PROMPTS
from vt_capture_console.ui.widgets import tracker_setup as module
from test_tracker_setup import pump
from test_tracker_workflow import idle_snapshot, make_config


FIELDS = {"python", "script", "capture", "bundle", "expected_sha256"}


class FakeSettings:
    """An isolated preference store with atomic failure behavior."""
    def __init__(self, defaults, events=None, *, read_error=None, save_error=None):
        self.stored = deepcopy(defaults)
        self.events = events if events is not None else []
        self.read_error, self.save_error = read_error, save_error
        self.reads = 0
        self.saves = []

    def defaults(self):
        self.reads += 1
        if self.read_error is not None:
            raise self.read_error
        return deepcopy(self.stored)

    def save(self, values):
        clean = deepcopy(values)
        self.events.append(("save", clean))
        self.saves.append(clean)
        if self.save_error is not None:
            raise self.save_error
        self.stored = clean


class FakeWorkflow:
    def __init__(self, events):
        self.events = events
        self.state, self.prompt = "IDLE", ""
        self.detail, self.logs = "尚未启动", []
        self.starts = []
        self.cancel_count = 0

    def snapshot(self):
        return dict(state=self.state, prompt=self.prompt, detail=self.detail, logs=list(self.logs))

    def start(self, config):
        self.events.append(("start", config))
        self.starts.append(config)
        self.state = "RUNNING"
        self.detail = "synthetic workflow started"

    def cancel(self):
        self.cancel_count += 1
        self.state = "CANCELLED"

    def answer(self, prompt):
        self.events.append(("answer", prompt))


@pytest.fixture
def dialogs(qapp, monkeypatch):
    created = []

    def forbidden(*args, **kwargs):
        pytest.fail("Fake-settings UI tests must not access defaults, hash files, or launch a child")

    monkeypatch.setattr(module, "detect_defaults", forbidden)
    monkeypatch.setattr(module, "capture_sha256", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)

    def create(settings):
        workflow = FakeWorkflow(settings.events)
        monkeypatch.setattr(module, "TrackerWorkflow", lambda *args, **kwargs: workflow)
        dialog = module.TrackerSetupDialog(idle_snapshot, settings=settings)
        dialog.show()
        created.append(dialog)
        qapp.processEvents()
        return dialog, workflow

    yield create
    for dialog in created:
        dialog.close()
        dialog.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


def fill(dialog, values):
    for name, value in values.items():
        dialog.fields[name].setText(value)


def test_opening_and_closing_reads_defaults_but_never_saves_or_starts(dialogs, tmp_path):
    values = asdict(make_config(tmp_path))
    settings = FakeSettings(values)
    dialog, workflow = dialogs(settings)
    assert settings.reads == 1 and settings.saves == []
    assert dialog._values() == values
    assert not dialog.approval.isChecked() and not dialog.start_button.isEnabled()
    assert workflow.starts == [] and workflow.cancel_count == 0
    assert dialog.started_config is None
    dialog.close()
    assert settings.stored == values and settings.events == []
    assert workflow.starts == []


def test_save_persists_exactly_five_trimmed_fields_not_execution_approval(dialogs, tmp_path):
    values = asdict(make_config(tmp_path))
    settings = FakeSettings(values)
    dialog, workflow = dialogs(settings)
    modified = {**values, "expected_sha256": "b" * 64}
    fill(dialog, {key: "  " + value + "  " for key, value in modified.items()})
    dialog.approval.setChecked(True)
    dialog.save_button.click()
    assert settings.saves == [modified] and settings.stored == modified
    assert set(settings.stored) == FIELDS
    assert all("approval" not in key for key in settings.stored)
    assert workflow.starts == []
    assert "已保存" in dialog.save_note.text()


def test_reopening_uses_saved_defaults_but_consent_is_always_unchecked(dialogs, tmp_path):
    settings = FakeSettings(asdict(make_config(tmp_path)))
    first, _ = dialogs(settings)
    first.fields["expected_sha256"].setText("c" * 64)
    first.approval.setChecked(True)
    first.save_button.click()
    first.close()
    second, workflow = dialogs(settings)
    assert settings.reads == 2
    assert second._values() == settings.stored
    assert second.fields["expected_sha256"].text() == "c" * 64
    assert not second.approval.isChecked() and not second.start_button.isEnabled()
    assert workflow.starts == [] and len(settings.saves) == 1


@pytest.mark.parametrize("name", sorted(FIELDS))
def test_each_field_change_revokes_consent_without_saving(dialogs, tmp_path, name):
    settings = FakeSettings(asdict(make_config(tmp_path)))
    dialog, workflow = dialogs(settings)
    dialog.approval.setChecked(True)
    assert dialog.start_button.isEnabled()
    dialog.fields[name].setText(dialog.fields[name].text() + "x")
    assert not dialog.approval.isChecked()
    assert not dialog.start_button.isEnabled()
    assert "配置已修改" in dialog.save_note.text()
    assert settings.saves == [] and workflow.starts == []


def test_start_requires_fresh_explicit_consent_then_saves_before_workflow_start(dialogs, tmp_path):
    values = asdict(make_config(tmp_path))
    settings = FakeSettings(values)
    dialog, workflow = dialogs(settings)
    dialog._start()  # Direct invocation must not bypass the unchecked consent.
    assert settings.events == [] and workflow.starts == []
    dialog.approval.setChecked(True)
    dialog.start_button.click()
    assert [name for name, value in settings.events] == ["save", "start"]
    assert settings.events[0][1] == values
    assert asdict(workflow.starts[0]) == values
    assert dialog.started_config is workflow.starts[0]
    assert workflow.state == "RUNNING"
    assert not dialog.save_button.isEnabled() and not dialog.start_button.isEnabled()


@pytest.mark.parametrize("error", [ValueError("invalid preferences"), OSError("write refused"), TypeError("invalid value")])
def test_save_failure_preserves_original_preferences_and_never_starts(dialogs, tmp_path, error):
    original = asdict(make_config(tmp_path))
    settings = FakeSettings(original, save_error=error)
    dialog, workflow = dialogs(settings)
    dialog.fields["expected_sha256"].setText("d" * 64)
    dialog.approval.setChecked(True)
    dialog._start()
    assert settings.stored == original and len(settings.saves) == 1
    assert workflow.starts == [] and dialog.started_config is None
    assert "原文件保留" in dialog.save_note.text()
    assert "初始化未启动" in dialog.instruction.text()
    assert [name for name, value in settings.events] == ["save"]


@pytest.mark.parametrize("state", ["RUNNING", "WAITING", "CANCELLING"])
def test_active_workflow_disallows_save_even_when_slot_called_directly(dialogs, tmp_path, state):
    settings = FakeSettings(asdict(make_config(tmp_path)))
    dialog, workflow = dialogs(settings)
    workflow.state = state
    dialog.refresh()
    assert not dialog.save_button.isEnabled()
    assert all(not edit.isEnabled() for edit in dialog.fields.values())
    assert dialog._save_settings() is False
    assert settings.saves == [] and workflow.starts == []


@pytest.mark.parametrize("state", ["IDLE", "RUNNING", "WAITING", "CANCELLING", "CANCELLED", "FAILED"])
def test_reading_request_is_forbidden_until_explicit_complete(dialogs, tmp_path, state):
    settings = FakeSettings(asdict(make_config(tmp_path)))
    dialog, workflow = dialogs(settings)
    workflow.state = state
    requested = QSignalSpy(dialog.reading_requested)
    dialog.refresh()
    assert dialog.reading_button.isHidden() and not dialog.reading_button.isEnabled()
    dialog._request_reading()
    assert requested.count() == 0


def test_complete_only_emits_read_request_and_pending_blocks_duplicate_actions(dialogs, tmp_path):
    settings = FakeSettings(asdict(make_config(tmp_path)))
    dialog, workflow = dialogs(settings)
    workflow.state, workflow.detail = "COMPLETE", "初始化合成完成；尚未验证位姿"
    requested = QSignalSpy(dialog.reading_requested)
    dialog.refresh()
    assert dialog.reading_button.isVisible() and dialog.reading_button.isEnabled()
    dialog.reading_button.click()
    assert requested.count() == 1
    assert workflow.starts == [] and settings.saves == []
    dialog.set_reading_feedback("正在提交读取请求", pending=True)
    assert dialog._reading_pending and not dialog.reading_button.isEnabled()
    dialog._request_reading()
    dialog.reading_button.click()
    assert requested.count() == 1
    dialog.set_reading_feedback("读取请求失败，可重试", pending=False)
    assert dialog.reading_button.isEnabled()
    dialog.reading_button.click()
    assert requested.count() == 2


@pytest.mark.parametrize("pending", [False, True])
def test_reading_feedback_does_not_replace_initialization_stage_or_close_dialog(dialogs, tmp_path, pending):
    settings = FakeSettings(asdict(make_config(tmp_path)))
    dialog, workflow = dialogs(settings)
    workflow.state, workflow.detail = "COMPLETE", "初始化已经完成，但不是位姿验收"
    dialog.refresh()
    stage, instruction = dialog.stage.text(), dialog.instruction.text()
    finished = QSignalSpy(dialog.finished)
    dialog.set_reading_feedback("读取失败：另有发布者；不自动重试", pending=pending)
    assert dialog.stage.text() == stage and dialog.instruction.text() == instruction
    assert workflow.state == "COMPLETE" and workflow.starts == []
    assert dialog.isVisible() and finished.count() == 0
    assert dialog.reading_note.text() == "读取失败：另有发布者；不自动重试"
    assert dialog.reading_note.isVisible()
    dialog.set_reading_feedback("")
    assert dialog.reading_note.isHidden() and dialog.stage.text() == stage


@pytest.mark.parametrize("error", [ValueError("损坏的设置"), OSError("配置不可读"), TypeError("字段类型错误")])
def test_corrupt_settings_are_explained_and_never_automatically_overwritten(dialogs, tmp_path, error):
    original = asdict(make_config(tmp_path))
    settings = FakeSettings(original, read_error=error)
    dialog, workflow = dialogs(settings)
    assert all(edit.text() == "" for edit in dialog.fields.values())
    assert "读取失败" in dialog.save_note.text() and "未覆盖原文件" in dialog.save_note.text()
    assert not dialog.save_button.isEnabled()
    assert settings.saves == []
    fill(dialog, original)
    dialog.approval.setChecked(True)
    assert dialog._save_settings() is False
    dialog._start()
    assert settings.saves == [] and settings.stored == original
    assert workflow.starts == [] and dialog.started_config is None
    assert "初始化未启动" in dialog.instruction.text()


def test_synthetic_child_completion_exposes_read_button_but_never_starts_reader(qapp, tmp_path):
    """Only the existing generated fixture script runs; no approved live code."""
    config = make_config(tmp_path)
    settings = FakeSettings(asdict(config))
    dialog = module.TrackerSetupDialog(idle_snapshot, settings=settings)
    dialog.show()
    requested = QSignalSpy(dialog.reading_requested)
    try:
        assert settings.saves == [] and dialog.workflow.snapshot()["state"] == "IDLE"
        dialog.approval.setChecked(True)
        dialog.start_button.click()
        assert settings.saves == [asdict(config)]
        for prompt in PROMPTS:
            pump(qapp, lambda: dialog.workflow.snapshot()["prompt"] == prompt)
            dialog.refresh()
            dialog.confirm_button.click()
        pump(qapp, lambda: dialog.workflow.snapshot()["state"] == "COMPLETE")
        dialog.refresh()
        assert dialog.started_config == config
        assert dialog.reading_button.isVisible() and requested.count() == 0
        dialog.reading_button.click()
        assert requested.count() == 1
        assert dialog.isVisible() and dialog.workflow.snapshot()["state"] == "COMPLETE"
        assert len(settings.saves) == 1
    finally:
        dialog.close()
        assert dialog.workflow.wait(3)
        dialog.deleteLater()
        qapp.processEvents()
