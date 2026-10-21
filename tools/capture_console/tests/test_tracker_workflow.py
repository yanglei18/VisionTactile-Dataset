"""Synthetic prompt subprocesses only: no PyVUT imports, USB or hardware."""
from copy import deepcopy
from dataclasses import asdict
import fcntl
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

from vt_capture_console.backend import tracker_workflow as module
from vt_capture_console.backend.tracker_workflow import (
    ACTIONS, BootstrapConfig, COMPLETE_LINE, PROMPTS, TrackerWorkflow,
    capture_sha256, initialization_blocker,
)


def idle_snapshot():
    return dict(mode="live", instance_id="manager-test", recording=dict(state="UNAVAILABLE"),
        management=dict(controller=None, trials=[]), capabilities=dict(device_control=True),
        process_control=dict(enabled=True, groups=[dict(kind="tracker", state="STOPPED", owned=False,
                                                       pid=None, can_start=True)]), devices=[])


def make_config(tmp_path, body=None):
    script = tmp_path / "live_windows_bootstrap.py"
    if body is None:
        body = f'''
import sys
print({PROMPTS[0]!r}, flush=True)
assert input() == 'yes'
print({PROMPTS[1]!r}, flush=True)
assert input() == 'yes'
print({ACTIONS[0]!r}, flush=True)
print({PROMPTS[2]!r}, flush=True)
assert input() == 'yes'
print({ACTIONS[1]!r}, flush=True)
print({COMPLETE_LINE!r}, flush=True)
'''
    script.write_text(body)
    capture = tmp_path / "capture ; inert.pcapng"
    capture.write_bytes(b"synthetic capture, not a USB recording")
    bundle = tmp_path / "bundle.json"
    bundle.write_text("synthetic fixture; not real credentials")
    return BootstrapConfig(sys.executable, str(script), str(capture), str(bundle), "a" * 64)


def until(workflow, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = workflow.snapshot()
        if predicate(snapshot):
            return snapshot
        time.sleep(.005)
    pytest.fail(f"Synthetic workflow did not converge: {workflow.snapshot()}")


def finish_prompts(workflow):
    for prompt in PROMPTS:
        until(workflow, lambda value: value["prompt"] == prompt)
        workflow.answer(prompt)
    assert workflow.wait(3)
    return workflow.snapshot()


def test_fixed_argv_and_exact_interactive_completion(tmp_path):
    calls = []
    def factory(argv, **kwargs):
        assert kwargs["shell"] is False and kwargs["start_new_session"] is True
        assert kwargs["stdin"] == subprocess.PIPE and kwargs["stderr"] == subprocess.STDOUT
        calls.append(argv)
        return subprocess.Popen(argv, **kwargs)
    config = make_config(tmp_path)
    workflow = TrackerWorkflow(idle_snapshot, process_factory=factory)
    try:
        workflow.start(config)
        assert finish_prompts(workflow)["state"] == "COMPLETE"
        assert calls == [config.argv()]
        assert calls[0][-3:] == ["--startup-mode", "host-then-clients", "--execute-feature-writes"]
        assert calls[0][calls[0].index("--capture") + 1] == config.capture
        assert workflow.snapshot()["logs"][-1] == COMPLETE_LINE
    finally:
        workflow.cancel()
        workflow.wait(3)


@pytest.mark.parametrize("mutate", [
    lambda s: s.update(mode="demo"), lambda s: s.update(instance_id=""),
    lambda s: s["recording"].update(state="RECORDING"),
    lambda s: s["management"].update(controller=dict(state=3, recorder_alive=True)),
    lambda s: s["management"].update(trials=[dict(state="UNKNOWN")]),
    lambda s: s["capabilities"].update(device_control=False),
    lambda s: s["process_control"].update(enabled=False),
    lambda s: s["process_control"]["groups"][0].update(state="EXTERNAL"),
    lambda s: s["process_control"]["groups"][0].update(owned=True),
    lambda s: s["process_control"]["groups"][0].update(pid=1234),
    lambda s: s["process_control"]["groups"][0].update(can_start=False),
    lambda s: s.update(devices=[dict(kind="tracker", status="online", receive_hz=1.)]),
])
def test_unsafe_snapshot_never_launches(tmp_path, mutate):
    snapshot = idle_snapshot()
    mutate(snapshot)
    assert initialization_blocker(snapshot)
    def forbidden(*args, **kwargs):
        pytest.fail("Unsafe snapshot must not launch even a fake process")
    workflow = TrackerWorkflow(lambda: snapshot, process_factory=forbidden)
    with pytest.raises(ValueError):
        workflow.start(make_config(tmp_path))


@pytest.mark.parametrize("body", [
    "print('status=COMPLETE')", "print('status=PARTIAL_BOOTSTRAP failure=ack_timeout')",
    f"print({COMPLETE_LINE!r})", f"print({PROMPTS[1]!r})",
    "print('confirm=unexpected type=yes')", "print('hello')",
    "print('x' * 5000)",
])
def test_no_exit_or_unknown_prompt_can_fake_success(tmp_path, body):
    workflow = TrackerWorkflow(idle_snapshot)
    workflow.start(make_config(tmp_path, body))
    assert workflow.wait(3)
    assert workflow.snapshot()["state"] == "FAILED"


def test_correct_complete_with_nonzero_exit_is_not_success(tmp_path):
    config = make_config(tmp_path)
    script = Path(config.script)
    script.write_text(script.read_text() + "\nraise SystemExit(2)\n")
    workflow = TrackerWorkflow(idle_snapshot)
    workflow.start(config)
    assert finish_prompts(workflow)["state"] == "FAILED"


def test_only_current_recognized_prompt_is_answered_and_cancel_reaps(tmp_path):
    workflow = TrackerWorkflow(idle_snapshot)
    workflow.start(make_config(tmp_path))
    try:
        until(workflow, lambda value: value["prompt"] == PROMPTS[0])
        with pytest.raises(ValueError):
            workflow.answer(PROMPTS[1])
        assert workflow.snapshot()["prompt"] == PROMPTS[0]
        with pytest.raises(ValueError):
            workflow.start(make_config(tmp_path))
        workflow.cancel()
        assert workflow.wait(3)
        assert workflow.snapshot()["state"] == "CANCELLED"
        with pytest.raises(ValueError):
            workflow.answer(PROMPTS[0])
    finally:
        workflow.cancel()
        workflow.wait(3)


def test_changed_guard_before_answer_never_continues(tmp_path):
    snapshot = idle_snapshot()
    workflow = TrackerWorkflow(lambda: deepcopy(snapshot))
    workflow.start(make_config(tmp_path))
    until(workflow, lambda value: value["prompt"] == PROMPTS[0])
    snapshot["instance_id"] = "new-manager"
    with pytest.raises(ValueError):
        workflow.answer(PROMPTS[0])
    assert workflow.wait(3)
    assert workflow.snapshot()["state"] == "FAILED"


def test_timeout_and_output_bounds_reap_only_owned_child(tmp_path):
    children = []
    def factory(*args, **kwargs):
        process = subprocess.Popen(*args, **kwargs)
        children.append(process)
        return process
    workflow = TrackerWorkflow(idle_snapshot, process_factory=factory, timeout_s=.05)
    workflow.start(make_config(tmp_path, "import time\ntime.sleep(10)"))
    assert workflow.wait(3)
    assert workflow.snapshot()["state"] == "FAILED" and "超时" in workflow.snapshot()["detail"]
    assert len(children) == 1 and children[0].poll() is not None


def test_unknown_output_is_not_a_private_terminal_transcript(tmp_path):
    config = make_config(tmp_path)
    script = Path(config.script)
    script.write_text("print('secret-fixture-must-not-display')\n" + script.read_text())
    workflow = TrackerWorkflow(idle_snapshot)
    workflow.start(config)
    result = finish_prompts(workflow)
    assert result["state"] == "COMPLETE"
    assert "secret-fixture" not in str(result)


def test_explicit_digest_reads_fixture_without_approval_or_execution(tmp_path):
    config = make_config(tmp_path)
    assert capture_sha256(config.capture) == hashlib.sha256(Path(config.capture).read_bytes()).hexdigest()
    with pytest.raises(ValueError, match="取消"):
        capture_sha256(config.capture, cancel=lambda: True)
    assert module.detect_defaults().keys() == {"python", "script", "capture", "bundle", "expected_sha256"}


def test_complete_is_visible_only_after_admission_release(tmp_path, monkeypatch):
    lock = tmp_path / ".desktop-apply.lock"
    workflow = TrackerWorkflow(idle_snapshot, admission_path=lock)
    at_release, allow_release = threading.Event(), threading.Event()
    real_close = os.close

    def controlled_close(fd):
        try:
            is_admission = os.fstat(fd).st_ino == lock.stat().st_ino
        except (OSError, FileNotFoundError):
            is_admission = False
        if is_admission:
            at_release.set()
            assert allow_release.wait(3)
        real_close(fd)

    monkeypatch.setattr(module.os, "close", controlled_close)
    try:
        workflow.start(make_config(tmp_path))
        for prompt in PROMPTS:
            until(workflow, lambda value: value["prompt"] == prompt)
            workflow.answer(prompt)
        assert at_release.wait(3)
        assert workflow.snapshot()["state"] != "COMPLETE"
        allow_release.set()
        assert workflow.wait(3)
        assert workflow.snapshot()["state"] == "COMPLETE"
        with lock.open("rb") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(stream, fcntl.LOCK_UN)
    finally:
        allow_release.set()
        workflow.cancel()
        workflow.wait(3)


def test_invalid_config_rejected_before_execution(tmp_path):
    config = make_config(tmp_path)
    with pytest.raises(ValueError, match="SHA-256"):
        BootstrapConfig(config.python, config.script, config.capture, config.bundle, "").argv()
    with pytest.raises(ValueError, match="绝对"):
        BootstrapConfig(config.python, "live_windows_bootstrap.py", config.capture, config.bundle, "a" * 64).argv()


def test_bootstrap_metadata_issues_have_stable_field_order_and_share_argv_validation(tmp_path):
    assert module.bootstrap_input_issues({}) == ("python", "script", "capture", "bundle", "expected_sha256")
    values = asdict(make_config(tmp_path))
    assert module.bootstrap_input_issues(values) == ()
    wrong_script = tmp_path / "different.py"
    wrong_script.touch()
    non_executable = tmp_path / "python"
    non_executable.touch()
    for field, invalid in (("python", str(non_executable)), ("script", str(wrong_script)),
                           ("capture", str(tmp_path)), ("bundle", "relative"),
                           ("expected_sha256", "invalid")):
        changed = dict(values, **{field: invalid})
        assert module.bootstrap_input_issues(changed) == (field,)
        with pytest.raises(ValueError):
            BootstrapConfig(**changed).argv()


def test_bootstrap_metadata_issues_never_read_hash_or_start_inputs(tmp_path, monkeypatch):
    values = asdict(make_config(tmp_path))
    def forbidden(*_args, **_kwargs):
        pytest.fail("metadata validation must not read, hash or execute")
    monkeypatch.setattr(Path, "read_bytes", forbidden)
    monkeypatch.setattr(Path, "read_text", forbidden)
    monkeypatch.setattr(os, "open", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    monkeypatch.setattr(module, "capture_sha256", forbidden)
    assert module.bootstrap_input_issues(values) == ()


def test_admission_excludes_other_process_for_whole_child_and_is_released(tmp_path):
    lock = tmp_path / ".desktop-apply.lock"
    children = []
    def factory(argv, **kwargs):
        assert len(kwargs["pass_fds"]) == 1  # Owned child retains lock on GUI death.
        child = subprocess.Popen(argv, **kwargs)
        children.append(child)
        return child
    workflow = TrackerWorkflow(idle_snapshot, admission_path=lock, process_factory=factory)
    workflow.start(make_config(tmp_path))
    try:
        until(workflow, lambda view: view["state"] == "WAITING")
        inode = lock.stat().st_ino
        # A separate ordinary Python process uses the exact same flock contract.
        probe = """import fcntl, os, sys
fd = os.open(sys.argv[1], os.O_RDWR)
try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise SystemExit(42)
"""
        assert subprocess.run([sys.executable, "-c", probe, str(lock)], check=False).returncode == 42
        workflow.cancel()
        assert workflow.wait(3)
        assert children[0].poll() is not None
        assert subprocess.run([sys.executable, "-c", probe, str(lock)], check=False).returncode == 0
        assert lock.stat().st_ino == inode  # Never unlink/replace a shared lock.
        assert lock.stat().st_mode & 0o777 == 0o600
    finally:
        workflow.cancel()
        workflow.wait(3)


def test_busy_admission_never_launches_and_no_local_owner_leak(tmp_path):
    lock = tmp_path / ".desktop-apply.lock"
    fd = os.open(lock, os.O_RDWR | os.O_CREAT, 0o600)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    calls = []
    workflow = TrackerWorkflow(idle_snapshot, admission_path=lock, process_factory=lambda *a, **k: calls.append(a))
    try:
        with pytest.raises(ValueError, match="其他设备"):
            workflow.start(make_config(tmp_path))
        assert not calls
    finally:
        os.close(fd)
    # Failure must release the per-application guard as well.
    second = TrackerWorkflow(idle_snapshot, admission_path=lock)
    second.start(make_config(tmp_path))
    second.cancel()
    assert second.wait(3)


def test_fresh_snapshot_is_rechecked_after_exclusive_admission(tmp_path):
    lock = tmp_path / ".desktop-apply.lock"
    observations = []
    def fresh_provider():
        snapshot = idle_snapshot()
        descriptor = os.open(lock, os.O_RDWR)
        try:
            with pytest.raises(BlockingIOError):
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            snapshot["recording"]["state"] = "RECORDING"
        finally:
            os.close(descriptor)
        observations.append(threading.get_ident())
        return snapshot
    calls = []
    workflow = TrackerWorkflow(idle_snapshot, admission_path=lock, fresh_snapshot_provider=fresh_provider,
                               process_factory=lambda *a, **k: calls.append(a))
    workflow.start(make_config(tmp_path))
    assert workflow.wait(3)
    assert workflow.snapshot()["state"] == "FAILED" and "录制" in workflow.snapshot()["detail"]
    assert len(observations) == 1 and observations[0] != threading.get_ident() and not calls
    descriptor = os.open(lock, os.O_RDWR)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        os.close(descriptor)


def test_admission_rejects_nonprivate_or_symlink_lock(tmp_path):
    lock = tmp_path / ".desktop-apply.lock"
    lock.write_text("")
    lock.chmod(0o644)
    workflow = TrackerWorkflow(idle_snapshot, admission_path=lock)
    with pytest.raises(ValueError, match="私有"):
        workflow.start(make_config(tmp_path))
    lock.unlink()
    lock.symlink_to(tmp_path / "another.lock")
    with pytest.raises(OSError):
        workflow.start(make_config(tmp_path))
