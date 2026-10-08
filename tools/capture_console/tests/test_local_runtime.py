"""Desktop lifecycle integration using a tiny harmless Unix-socket child only."""
from copy import deepcopy
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

from vt_capture_console.backend import local_runtime
from vt_capture_console.backend.local_runtime import DesktopRuntime, _identity, _snapshot_problem
from vt_capture_console.backend.workstation_settings import atomic_json
from vt_capture_core import (ControllerSnapshot, DeviceControlSnapshot, ManagementSnapshot,
                             ProcessControlSnapshot, ProcessGroupSnapshot)


FAKE_BACKEND = r'''
import json, os, signal, socket, sys
from pathlib import Path
from vt_capture_core import encode_message
path, source = Path(sys.argv[1]), Path(sys.argv[2])
done = False
def stop(*args):
    global done
    done = True
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
server.bind(str(path))
server.listen(8)
server.settimeout(.04)
print('synthetic manager ready', flush=True)
try:
    while not done:
        try:
            connection, _ = server.accept()
        except socket.timeout:
            continue
        with connection:
            connection.settimeout(.3)
            try:
                data = bytearray()
                while b'\n' not in data:
                    chunk = connection.recv(65536)
                    if not chunk:
                        break
                    data.extend(chunk)
                if not data:
                    continue
                request = json.loads(bytes(data))
                snapshot = json.loads(source.read_text())
                snapshot['instance_id'] = 'synthetic-' + str(os.getpid())
                connection.sendall(encode_message(dict(version=4, request_id=request['request_id'], ok=True, snapshot=snapshot)))
            except (OSError, ValueError):
                pass
finally:
    server.close()
    path.unlink(missing_ok=True)
'''


def idle_snapshot():
    return DeviceControlSnapshot("live", "synthetic-unbound", management=ManagementSnapshot(
        enabled=True, controller=ControllerSnapshot("controller")),
        process_control=ProcessControlSnapshot(enabled=True)).to_dict()


def eventually(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.01)
    raise AssertionError("timed out waiting for desktop runtime")


@pytest.fixture
def runtime_factory(tmp_path, monkeypatch):
    runtimes, children = [], []
    original_popen = subprocess.Popen

    def popen(*args, **kwargs):
        result = original_popen(*args, **kwargs)
        children.append(result)
        return result

    monkeypatch.setattr(local_runtime.subprocess, "Popen", popen)

    def factory(*, directory=None, start=True, source=None, child_code=None):
        index = len(runtimes)
        directory = directory or tmp_path / str(index)
        source = source or tmp_path / f"snapshot-{index}.json"
        if not source.exists():
            atomic_json(source, idle_snapshot())
        runtime = DesktopRuntime(tmp_path, directory)
        # Keep Unix paths below the kernel limit despite descriptive pytest ids.
        runtime.socket_path = tmp_path / (str(index) + ".sock")
        if directory in [value.settings.directory for value in runtimes]:
            runtime.socket_path = next(value.socket_path for value in runtimes if value.settings.directory == directory)
        runtime.STARTUP_TIMEOUT_S = 2.
        runtime.STOP_TIMEOUT_S = .6
        runtime.POLL_INTERVAL_S = .04
        monkeypatch.setattr(runtime, "_session_status", lambda: dict(
            gid=None, account_has_group=False, process_has_group=False, needs_refresh=False))
        monkeypatch.setattr(runtime, "_manager_argv", lambda: (sys.executable, "-u", "-c", child_code or FAKE_BACKEND, str(runtime.socket_path), str(source)))
        runtime.source = source
        runtimes.append(runtime)
        if start:
            runtime.start()
        return runtime

    factory.children = children
    yield factory
    for runtime in runtimes:
        runtime.close()
        if runtime._thread:
            runtime._thread.join(timeout=3.)
    for process in children:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=2.)


def test_start_only_fixed_manager_recipe_and_no_sensor_commands(tmp_path):
    script = tmp_path / "tools/capture_deploy/start_manager.sh"
    script.parent.mkdir(parents=True)
    script.touch()
    runtime = DesktopRuntime(tmp_path, tmp_path / "private")
    argv = runtime._manager_argv()
    assert argv[:2] == ("/bin/bash", str(script))
    assert "--enable-device-control" in argv and "--enable-recording" in argv
    assert argv[argv.index("--config") + 1] == str(runtime.settings_path)
    assert argv[argv.index("--desktop-apply-lock") + 1] == str(runtime.admission_path)
    assert not any("device.start" in arg or ";" in arg for arg in argv)
    assert all(not profile["enabled"] for profile in runtime.configuration["process_groups"])
    runtime.close()


def test_start_is_nonblocking_and_close_detaches(runtime_factory):
    runtime = runtime_factory(start=False)
    before = time.monotonic()
    runtime.start()
    assert time.monotonic() - before < .1
    eventually(lambda: runtime.status["state"] == "ready")
    assert len(runtime_factory.children) == 1
    process = runtime_factory.children[0]
    assert "synthetic manager ready" in runtime.log_path.read_text()
    assert runtime.log_path.stat().st_mode & 0o777 == 0o600
    runtime.close()
    assert runtime.status["state"] == "closed"
    assert process.poll() is None


def test_reopening_window_reuses_prior_owned_backend_without_duplicate(runtime_factory):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    first.close()
    replacement = runtime_factory(directory=first.settings.directory, source=first.source)
    eventually(lambda: replacement.status["state"] == "ready")
    assert len(runtime_factory.children) == 1
    assert replacement._process is None  # Identity registry, not Popen adoption.
    assert replacement._probe()[1] == runtime_factory.children[0].pid


def serial_session(*, stale=False):
    return dict(gid=20, account_has_group=True, process_has_group=not stale,
                needs_refresh=stale)


def test_process_groups_contains_effective_and_supplementary_groups():
    assert local_runtime._process_groups(os.getpid()) == set(os.getgroups()) | {os.getegid()}


def test_stale_desktop_refuses_launch_without_pending_registry(runtime_factory, monkeypatch):
    runtime = runtime_factory(start=False)
    monkeypatch.setattr(runtime, "_session_status", lambda: serial_session(stale=True))
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert "桌面快捷方式" in runtime.status["detail"]
    assert not runtime_factory.children
    assert not runtime._registry_path.exists()


def test_stale_desktop_apply_preserves_manager_and_saved_configuration(runtime_factory, monkeypatch):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    before = runtime.settings_path.read_bytes()
    configuration = runtime.configuration
    configuration["quality_enabled"] = not configuration["quality_enabled"]
    monkeypatch.setattr(runtime, "_session_status", lambda: serial_session(stale=True))
    assert runtime.apply(configuration, runtime._probe()[0])
    eventually(lambda: runtime.status["last_apply"]["state"] == "error")
    assert "串口权限" in runtime.status["detail"]
    assert runtime.settings_path.read_bytes() == before
    assert len(runtime_factory.children) == 1
    assert runtime_factory.children[0].poll() is None


@pytest.mark.parametrize("initial_probe_fails", [False, True])
def test_refreshed_desktop_replaces_only_idle_owned_stale_manager(runtime_factory, monkeypatch, initial_probe_fails):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    first.close()
    old = runtime_factory.children[0]
    replacement = runtime_factory(directory=first.settings.directory, source=first.source, start=False)
    monkeypatch.setattr(replacement, "_session_status", serial_session)
    monkeypatch.setattr(local_runtime, "_process_groups", lambda pid: set() if pid == old.pid else {20})
    if initial_probe_fails:
        probe = replacement._probe
        calls = []

        def fail_once():
            calls.append(True)
            if len(calls) == 1:
                raise ConnectionError("transient socket error")
            return probe()

        monkeypatch.setattr(replacement, "_probe", fail_once)
    before = replacement.settings_path.read_bytes()
    replacement.start()
    eventually(lambda: replacement.status["state"] == "ready")
    assert "串口权限" in replacement.status["detail"]
    assert old.poll() is not None
    assert len(runtime_factory.children) == 2
    assert replacement._probe()[1] == runtime_factory.children[1].pid
    assert replacement.settings_path.read_bytes() == before


@pytest.mark.parametrize("active", ["device", "recording"])
def test_stale_manager_repair_waits_for_idle_then_explicit_retry(runtime_factory, monkeypatch, active):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    first.close()
    old = runtime_factory.children[0]
    snapshot = idle_snapshot()
    if active == "device":
        snapshot["process_control"]["groups"] = [ProcessGroupSnapshot(
            "right_tactile", "Tactile", "tactile", state="DEGRADED", owned=True, pid=123).to_dict()]
    else:
        snapshot["management"]["controller"]["recorder_alive"] = True
    atomic_json(first.source, snapshot)
    replacement = runtime_factory(directory=first.settings.directory, source=first.source, start=False)
    monkeypatch.setattr(replacement, "_session_status", serial_session)
    monkeypatch.setattr(local_runtime, "_process_groups", lambda pid: set() if pid == old.pid else {20})
    replacement.start()
    eventually(lambda: replacement.status["state"] == "permission_required")
    assert old.poll() is None
    assert len(runtime_factory.children) == 1
    atomic_json(first.source, idle_snapshot())
    replacement.start()
    eventually(lambda: replacement.status["state"] == "ready")
    assert old.poll() is not None
    assert len(runtime_factory.children) == 2


def test_old_desktop_never_recycles_stale_backend(runtime_factory, monkeypatch):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    first.close()
    replacement = runtime_factory(directory=first.settings.directory, source=first.source, start=False)
    monkeypatch.setattr(replacement, "_session_status", lambda: serial_session(stale=True))
    monkeypatch.setattr(local_runtime, "_process_groups", lambda pid: set())
    replacement.start()
    eventually(lambda: replacement.status["state"] == "permission_required")
    assert len(runtime_factory.children) == 1
    assert runtime_factory.children[0].poll() is None


def test_fresh_backend_is_reused_even_if_reopened_gui_is_stale(runtime_factory, monkeypatch):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    first.close()
    replacement = runtime_factory(directory=first.settings.directory, source=first.source, start=False)
    monkeypatch.setattr(replacement, "_session_status", lambda: serial_session(stale=True))
    monkeypatch.setattr(local_runtime, "_process_groups", lambda pid: {20})
    replacement.start()
    eventually(lambda: replacement.status["state"] == "ready")
    assert len(runtime_factory.children) == 1
    assert runtime_factory.children[0].poll() is None


def test_safe_apply_after_window_reopen_restarts_only_saved_owned_manager(runtime_factory):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    first.close()
    replacement = runtime_factory(directory=first.settings.directory, source=first.source)
    eventually(lambda: replacement.status["state"] == "ready")
    before = runtime_factory.children[0]
    configuration = replacement.configuration
    configuration["process_groups"][0]["label"] = "新配置"
    assert replacement.apply(configuration, replacement._probe()[0])
    eventually(lambda: replacement.status["last_apply"]["state"] == "applied")
    assert before.poll() is not None
    assert len(runtime_factory.children) == 2
    assert replacement.configuration["process_groups"][0]["label"] == "新配置"
    assert all(not value["enabled"] for value in replacement.configuration["process_groups"])


def test_apply_is_async_and_holds_shared_admission_lock(runtime_factory, monkeypatch):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    entered, release = threading.Event(), threading.Event()
    original = runtime._stop_owned

    def stop(record):
        entered.set()
        assert release.wait(2.)
        return original(record)

    monkeypatch.setattr(runtime, "_stop_owned", stop)
    started = time.monotonic()
    assert runtime.apply(runtime.configuration, runtime._probe()[0])
    assert time.monotonic() - started < .15
    assert entered.wait(2.)
    fd = os.open(runtime.admission_path, os.O_RDWR)
    try:
        with pytest.raises(BlockingIOError):
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert not runtime.apply(runtime.configuration, {})
        assert runtime.status["state"] == "applying"
    finally:
        os.close(fd)
        release.set()
    eventually(lambda: runtime.status["last_apply"]["state"] == "applied")


@pytest.mark.parametrize("state,owned", [("RUNNING", True), ("DEGRADED", True), ("STARTING", False), ("STOPPING", False)])
def test_active_or_pending_device_groups_reject_without_saving(runtime_factory, state, owned):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    snapshot = runtime._probe()[0]
    snapshot["process_control"]["groups"] = [ProcessGroupSnapshot("camera", "Camera", "camera", state=state, owned=owned, pid=123 if owned else None).to_dict()]
    configuration = runtime.configuration
    configuration["quality_enabled"] = False
    before = runtime.settings_path.read_bytes()
    assert not runtime.apply(configuration, snapshot)
    assert runtime.status["last_apply"]["state"] == "rejected"
    assert runtime.settings_path.read_bytes() == before
    assert len(runtime_factory.children) == 1


def test_fresh_worker_snapshot_blocks_recording_started_since_ui_snapshot(runtime_factory):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    stale_ui = runtime._probe()[0]
    active = idle_snapshot()
    active["management"]["controller"]["recorder_alive"] = True
    active["management"]["controller"]["state"] = 3
    atomic_json(runtime.source, active)
    before = runtime.settings_path.read_bytes()
    assert runtime.apply(runtime.configuration, stale_ui)
    eventually(lambda: runtime.status["last_apply"]["state"] == "error")
    assert "录制" in runtime.status["detail"]
    assert runtime.settings_path.read_bytes() == before
    assert len(runtime_factory.children) == 1
    assert runtime_factory.children[0].poll() is None


def test_unresolved_trials_block_configuration_apply():
    snapshot = idle_snapshot()
    snapshot["management"]["trials"] = [dict(state="UNKNOWN")]
    assert "试次" in _snapshot_problem(snapshot)


def test_reachable_external_backend_is_reused_but_never_adopted(runtime_factory):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    first.close()
    first._registry_path.unlink()
    replacement = runtime_factory(directory=first.settings.directory, source=first.source)
    eventually(lambda: replacement.status["state"] == "external")
    before = replacement.settings_path.read_bytes()
    assert replacement.apply(replacement.configuration, replacement._probe()[0])
    eventually(lambda: replacement.status["last_apply"]["state"] == "error")
    assert "不属于" in replacement.status["detail"]
    assert len(runtime_factory.children) == 1
    assert runtime_factory.children[0].poll() is None
    assert replacement.settings_path.read_bytes() == before


def test_backend_crash_is_reported_without_automatic_retry(runtime_factory):
    runtime = runtime_factory(child_code="raise SystemExit(7)")
    eventually(lambda: runtime.status["state"] == "error")
    assert len(runtime_factory.children) == 1
    time.sleep(.15)
    assert len(runtime_factory.children) == 1
    assert "后台" in runtime.status["detail"]


def test_explicit_start_retry_recovers_after_exited_backend(runtime_factory, monkeypatch):
    runtime = runtime_factory(child_code="raise SystemExit(7)")
    eventually(lambda: runtime.status["state"] == "error")
    monkeypatch.setattr(runtime, "_manager_argv", lambda: (sys.executable, "-u", "-c", FAKE_BACKEND, str(runtime.socket_path), str(runtime.source)))
    runtime.start()
    eventually(lambda: runtime.status["state"] == "ready")
    assert len(runtime_factory.children) == 2
    assert runtime.fresh_snapshot()["instance_id"] == "synthetic-" + str(runtime_factory.children[-1].pid)


def test_ownership_birth_identity_mismatch_never_signals(runtime_factory, monkeypatch):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    record = runtime._record()
    real_identity = _identity

    def reused(pid):
        identity = real_identity(pid)
        if identity and pid == record["identity"]["pid"]:
            identity["start_time"] += 1
        return identity

    monkeypatch.setattr(local_runtime, "_identity", reused)
    monkeypatch.setattr(signal, "pidfd_send_signal", lambda *args, **kwargs: pytest.fail("reused PID signalled"))
    runtime._signal_owned(record, signal.SIGTERM)
    assert runtime_factory.children[0].poll() is None


def test_start_failure_does_not_overwrite_unowned_socket_path(runtime_factory):
    runtime = runtime_factory(start=False)
    runtime.socket_path.write_text("must be preserved")
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert not runtime_factory.children
    assert runtime.socket_path.read_text() == "must be preserved"


def test_invalid_configuration_does_not_enqueue_or_overwrite(runtime_factory):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    configuration = runtime.configuration
    configuration["argv"] = ["arbitrary", "command"]
    before = runtime.settings_path.read_bytes()
    assert not runtime.apply(configuration, runtime._probe()[0])
    assert runtime.status["last_apply"]["state"] == "rejected"
    assert runtime.settings_path.read_bytes() == before


def test_existing_admission_operation_blocks_apply_without_restart(runtime_factory):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    fd = os.open(runtime.admission_path, os.O_CREAT | os.O_RDWR, 0o600)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    try:
        assert runtime.apply(runtime.configuration, runtime._probe()[0])
        eventually(lambda: runtime.status["last_apply"]["state"] == "error")
        assert "另一个工作台" in runtime.status["detail"]
        assert len(runtime_factory.children) == 1
    finally:
        os.close(fd)


def test_fresh_snapshot_refuses_unowned_backend(runtime_factory):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    runtime._registry_path.unlink()
    with pytest.raises(RuntimeError, match="不属于"):
        runtime.fresh_snapshot()


def test_unknown_interrupted_launch_is_not_retried_or_adopted(runtime_factory):
    runtime = runtime_factory(start=False)
    atomic_json(runtime._registry_path, dict(version=1, boot_id=runtime._boot_id,
                socket_path=str(runtime.socket_path), identity=None, pending_launch=True))
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert "结果不明确" in runtime.status["detail"]
    assert not runtime_factory.children


def test_failed_local_log_setup_does_not_claim_a_process_was_launched(runtime_factory):
    runtime = runtime_factory(start=False)
    runtime.log_path.write_text("not private")
    runtime.log_path.chmod(0o644)
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert not runtime_factory.children
    assert not runtime._registry_path.exists()
    runtime.log_path.chmod(0o600)
    runtime.start()
    eventually(lambda: runtime.status["state"] == "ready")
    assert len(runtime_factory.children) == 1
