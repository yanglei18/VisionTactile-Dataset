"""Defaults and reload use synthetic files and never initialize hardware."""
from dataclasses import asdict
from types import SimpleNamespace

import pytest

from vt_capture_console import i18n
from vt_capture_console.ui.widgets import tracker_setup as module
from vt_capture_console.ui.widgets.tracker_setup import TrackerSetupDialog
from test_tracker_workflow import idle_snapshot, make_config


class Settings:
    def __init__(self, values):
        self.values = values
        self.reads = 0
        self.error = False

    def defaults(self):
        self.reads += 1
        if self.error:
            raise ValueError("Synthetic settings error")
        return dict(self.values)

    def save(self, values):
        pytest.fail("Default filling must not persist settings")


@pytest.fixture
def wizard(qapp, tmp_path, monkeypatch):
    settings = Settings(asdict(make_config(tmp_path)))
    def forbidden(*args, **kwargs):
        pytest.fail("Reading defaults must not hash, initialize, or start a process")
    monkeypatch.setattr(module, "capture_sha256", forbidden)
    monkeypatch.setattr(module.TrackerWorkflow, "start", forbidden)
    monkeypatch.setattr("subprocess.Popen", forbidden)
    dialog = TrackerSetupDialog(idle_snapshot, settings=settings)
    yield dialog, settings
    dialog.close()
    dialog.deleteLater()
    qapp.processEvents()


def test_open_fills_saved_inputs_without_consent_or_side_effects(wizard):
    dialog, settings = wizard
    assert dialog._values() == settings.values
    assert settings.reads == 1
    assert "已自动填入" in dialog.save_note.text()
    assert not dialog.approval.isChecked()
    assert dialog.workflow.snapshot()["state"] == "IDLE"


def test_incomplete_or_missing_paths_are_reported_instead_of_claiming_ready(wizard):
    dialog, settings = wizard
    settings.values["capture"] = "/missing/approved.pcapng"
    settings.values["expected_sha256"] = ""
    dialog.reload_button.click()
    assert "请补全或检查" in dialog.save_note.text()
    assert "抓包文件" in dialog.save_note.text()
    assert "抓包指纹" in dialog.save_note.text()
    assert "已自动填入" not in dialog.save_note.text()


def test_reload_replaces_unsaved_inputs_and_revokes_consent(wizard):
    dialog, settings = wizard
    dialog.fields["script"].setText("/unsaved/live_windows_bootstrap.py")
    dialog.approval.setChecked(True)
    settings.values["expected_sha256"] = "b" * 64
    # Simulate a completed hash result awaiting the Qt timer. Even with the
    # same capture path, it cannot override the reloaded approved fingerprint.
    dialog._hash_results.put((settings.values["capture"], "c" * 64, ""))
    dialog.reload_button.click()
    dialog.refresh()
    assert settings.reads == 2
    assert dialog._values() == settings.values
    assert not dialog.approval.isChecked()
    assert dialog.workflow.snapshot()["state"] == "IDLE"


def test_reload_read_failure_preserves_visible_inputs_and_can_recover(wizard):
    dialog, settings = wizard
    before = dialog._values()
    settings.error = True
    dialog.approval.setChecked(True)
    dialog.reload_button.click()
    assert dialog._values() == before
    assert "读取失败" in dialog.save_note.text()
    assert not dialog.save_button.isEnabled()
    assert not dialog.approval.isChecked()
    settings.error = False
    dialog.reload_button.click()
    assert dialog.save_button.isEnabled()
    assert "已自动填入" in dialog.save_note.text()


@pytest.mark.parametrize("busy", ["RUNNING", "WAITING", "CANCELLING", "hashing"])
def test_reload_does_not_replace_inputs_during_owned_work(wizard, monkeypatch, busy):
    dialog, settings = wizard
    if busy == "hashing":
        dialog._hash_thread = SimpleNamespace(is_alive=lambda: True)
    else:
        original = dialog.workflow.snapshot()
        monkeypatch.setattr(dialog.workflow, "snapshot", lambda: dict(original, state=busy))
    dialog.refresh()
    assert not dialog.reload_button.isEnabled()
    dialog._reload_defaults()
    assert settings.reads == 1
    if busy != "hashing":
        monkeypatch.setattr(dialog.workflow, "snapshot", lambda: original)


def test_new_status_and_reload_action_switch_language(wizard):
    dialog, settings = wizard
    settings.values["capture"] = ""
    dialog.reload_button.click()
    try:
        i18n.set_language("en")
        assert dialog.reload_button.text() == "Reload Configuration"
        assert "Complete or check: Capture file" == dialog.save_note.text()
        assert "抓包" not in dialog.save_note.text()
    finally:
        i18n.set_language("zh_CN")
