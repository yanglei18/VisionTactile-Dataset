"""Only synthetic short-lived children; no rosbag data, sensors or algorithms."""
from pathlib import Path
import subprocess
import sys
import time

import pytest
from PySide6.QtCore import QEvent, QTimer

from vt_capture_console.backend import alignment_task
from vt_capture_console.backend.alignment_task import AlignmentTask, MAX_LOG_BYTES, build_alignment_command
from vt_capture_console.ui.widgets.offline_alignment import OfflineAlignmentDialog


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def paths(tmp_path):
    bag = tmp_path / "synthetic-bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("# metadata-only test, never read by real algorithms\n")
    config = tmp_path / "alignment.yaml"
    config.write_text("# test\n")
    extrinsics = tmp_path / "extrinsics"
    extrinsics.mkdir()
    return dict(bag=str(bag), config=str(config), extrinsics=str(extrinsics), output=str(tmp_path / "new-result"))


def wait_until(predicate, qapp=None, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if qapp:
            qapp.processEvents()
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("background action did not settle")


def fake_command(monkeypatch, script, output=""):
    monkeypatch.setattr(alignment_task, "build_alignment_command",
                        lambda *args, **kwargs: ([sys.executable, "-u", "-c", script], output))


def test_fixed_command_uses_argument_vector_and_existing_cli(paths):
    argv, result = build_alignment_command(ROOT, "align", **paths)
    assert argv[:3] == ["/bin/bash", str(ROOT / "tools/capture_deploy/run_alignment.sh"), "align"]
    assert argv[3:] == ["--bag", paths["bag"], "--config", paths["config"],
                       "--extrinsics", paths["extrinsics"], "--output", paths["output"]]
    assert result == paths["output"]
    assert not Path(result).exists()


def test_inspect_and_validate_only_have_required_readonly_arguments(paths):
    argv, result = build_alignment_command(ROOT, "inspect", **paths)
    assert "--output" not in argv and "--extrinsics" not in argv and result == ""
    Path(paths["output"]).mkdir()
    argv, result = build_alignment_command(ROOT, "validate", output=paths["output"])
    assert argv[3:] == ["--output", paths["output"]]
    assert result == paths["output"]


def test_never_overwrites_existing_result_or_writes_inside_bag(paths, tmp_path):
    Path(paths["output"]).mkdir()
    with pytest.raises(ValueError, match="已存在"):
        build_alignment_command(ROOT, "align", **paths)
    inside = {**paths, "output": str(Path(paths["bag"]) / "result")}
    with pytest.raises(ValueError, match="源 bag"):
        build_alignment_command(ROOT, "align", **inside)
    link = tmp_path / "bag-link"
    link.symlink_to(paths["bag"], target_is_directory=True)
    with pytest.raises(ValueError, match="源 bag"):
        build_alignment_command(ROOT, "align", **{**paths, "output": str(link / "result")})
    dangling = tmp_path / "dangling-result"
    dangling.symlink_to(tmp_path / "missing")
    with pytest.raises(ValueError, match="已存在"):
        build_alignment_command(ROOT, "align", **{**paths, "output": str(dangling)})


@pytest.mark.parametrize("action", ["export-tactile", "report-capture", "rm -rf", ""])
def test_arbitrary_operations_are_not_exposed(paths, action):
    with pytest.raises(ValueError):
        build_alignment_command(ROOT, action, **paths)


def test_required_file_errors_do_not_create_output(paths):
    with pytest.raises(ValueError, match="配置文件不存在"):
        build_alignment_command(ROOT, "align", **{**paths, "config": "/nonexistent/never.yaml"})
    assert not Path(paths["output"]).exists()


def test_recording_guard_rechecks_in_worker_before_child(monkeypatch, paths):
    calls = []
    snapshots = iter([{"recording": {"state": "IDLE"}}, {"recording": {"state": "RECORDING"}}])
    monkeypatch.setattr(alignment_task.subprocess, "Popen", lambda *a, **k: calls.append(a))
    task = AlignmentTask(ROOT, snapshot_provider=lambda: next(snapshots))
    task.start("inspect", **paths)
    wait_until(lambda: not task.running)
    assert calls == []
    assert task.status["state"] == "FAILED"
    assert "录制" in task.status["detail"]


def test_fake_child_output_is_bounded_and_quality_exit_two_is_not_success(monkeypatch):
    fake_command(monkeypatch, "import sys; print('x' * 400000); print('TAIL'); sys.exit(2)")
    task = AlignmentTask(ROOT)
    task.start("inspect")
    wait_until(lambda: not task.running)
    result = task.status
    assert result["state"] == "FAILED" and result["exit_code"] == 2
    assert "未通过" in result["detail"]
    assert "TAIL" in result["log"]
    assert len(result["log"].encode()) <= MAX_LOG_BYTES + 200


def test_cancel_escalates_only_owned_fake_child_and_leaves_other_process(monkeypatch):
    other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"])
    try:
        fake_command(monkeypatch, "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); print('READY'); time.sleep(30)")
        task = AlignmentTask(ROOT)
        task.start("inspect")
        wait_until(lambda: "READY" in task.status["log"])
        task.cancel()
        wait_until(lambda: not task.running)
        assert task.status["state"] == "CANCELLED"
        assert other.poll() is None
    finally:
        other.terminate()
        other.wait(timeout=3)


def test_already_completed_task_cannot_become_stuck_cancelling(monkeypatch):
    fake_command(monkeypatch, "print('DONE')")
    task = AlignmentTask(ROOT)
    task.start("inspect")
    wait_until(lambda: not task.running)
    task.cancel()
    assert task.status["state"] == "COMPLETE"


@pytest.fixture
def dialog(qapp):
    value = OfflineAlignmentDialog(ROOT)
    value.show()
    qapp.processEvents()
    yield value
    if value.running:
        value.cancel()
        wait_until(lambda: not value.running, qapp)
    value.close()
    value.deleteLater()
    qapp.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


def test_qt_remains_responsive_and_result_directory_opens_only_after_finish(dialog, qapp, monkeypatch, tmp_path):
    result = tmp_path / "result"
    result.mkdir()
    fake_command(monkeypatch, "import time; print('RUNNING'); time.sleep(.15); print('DONE')", str(result))
    dialog.start()
    assert dialog.running and not dialog.start_button.isEnabled()
    assert not dialog.open_button.isEnabled()
    alive = []
    QTimer.singleShot(0, lambda: alive.append(True))
    qapp.processEvents()
    assert alive == [True]
    wait_until(lambda: not dialog.running, qapp)
    dialog.refresh()
    assert dialog.start_button.isEnabled() and dialog.open_button.isEnabled()
    assert "DONE" in dialog.log.toPlainText()


def test_dialog_close_cancels_waits_and_can_be_reopened(dialog, qapp, monkeypatch):
    fake_command(monkeypatch, "import time; print('READY'); time.sleep(30)")
    dialog.start()
    wait_until(lambda: "READY" in dialog.task.status["log"], qapp)
    dialog.close()
    assert dialog.isVisible()  # Close is delayed until the owned child exits.
    wait_until(lambda: not dialog.isVisible(), qapp)
    assert not dialog.running
    assert not dialog.timer.isActive()
    dialog.show()
    qapp.processEvents()
    assert dialog.timer.isActive() and not dialog._closing
    fake_command(monkeypatch, "print('REOPENED')")
    dialog.start()
    wait_until(lambda: not dialog.running, qapp)
    assert dialog.task.status["state"] == "COMPLETE"


def test_recording_disables_start_without_launch(dialog, monkeypatch):
    dialog.snapshot_provider = lambda: {"recording": {"state": "RECORDING"}}
    dialog.refresh()
    assert not dialog.start_button.isEnabled()
    dialog.start()
    assert not dialog.running
    assert "停止录制" in dialog.status.text()


def test_missing_paths_report_without_losing_user_inputs(dialog, paths):
    dialog.fields["bag"].setText(paths["bag"])
    dialog.start()
    assert not dialog.running
    assert "配置" in dialog.status.text()
    assert dialog.fields["bag"].text() == paths["bag"]


def test_module_entrypoint_and_wrapper_shell_syntax_are_readonly():
    import os
    environment = {**os.environ, "PYTHONPATH": str(ROOT / "tools/multisensor_alignment/src")}
    run = subprocess.run([sys.executable, "-m", "vt_multisensor_alignment", "--version"],
                         env=environment, capture_output=True, text=True, timeout=5)
    assert run.returncode == 0 and "vt-multisensor-alignment" in run.stdout
    run = subprocess.run(["/bin/bash", "-n", str(ROOT / "tools/capture_deploy/run_alignment.sh")], timeout=5)
    assert run.returncode == 0
