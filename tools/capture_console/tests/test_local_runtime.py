"""Desktop lifecycle integration using a tiny harmless Unix-socket child only."""
from copy import deepcopy
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

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


@pytest.fixture(autouse=True)
def isolated_output(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path / "home")
    monkeypatch.delenv("VT_DATA_ROOT", raising=False)


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


def test_environment_detection_is_a_draft_without_starting_or_saving(runtime_factory, tmp_path):
    runtime = runtime_factory(start=False)
    original, status = runtime.configuration, runtime.status
    camera = tmp_path / "ros2_ws/src/vt_realsense_capture/config/cameras.yaml"
    camera.parent.mkdir(parents=True)
    camera.write_text("# test template\n")
    result = runtime.detect_environment(original)
    assert result.configuration["process_groups"][0]["parameters"]["config_path"] == str(camera)
    assert runtime.configuration == original and runtime.status == status
    assert not runtime.settings_path.exists()
    assert not (Path.home() / "visiontactile-data").exists()
    assert not runtime_factory.children


def test_new_backend_automatically_prepares_saved_empty_paths_before_launch(runtime_factory, tmp_path):
    runtime = runtime_factory(start=False)
    runtime.settings.save(runtime.configuration)
    camera = tmp_path / "ros2_ws/src/vt_realsense_capture/config/cameras.yaml"
    camera.parent.mkdir(parents=True)
    camera.write_text("# synthetic template\n")
    runtime.start()
    eventually(lambda: runtime.status["state"] == "ready")
    output = Path.home() / "visiontactile-data"
    persisted = json.loads(runtime.settings_path.read_text())
    assert persisted == runtime.configuration
    assert persisted["process_groups"][0]["parameters"] == {
        "config_path": str(camera), "output_root": str(output),
    }
    assert output.is_dir()
    assert not any(group["enabled"] for group in persisted["process_groups"])
    assert len(runtime_factory.children) == 1
    assert runtime.status["detail"] == "本地路径已自动准备，后台已就绪；可启动需要的设备。"


def test_slow_startup_discovery_keeps_status_and_complete_configuration_readable(
        runtime_factory, monkeypatch):
    runtime = runtime_factory(start=False)
    before = runtime.configuration
    entered, release, read_done = threading.Event(), threading.Event(), threading.Event()
    observed = {}
    original = runtime.settings.detect_environment
    def detect(configuration):
        entered.set()
        if not release.wait(5.):
            raise RuntimeError("synthetic discovery timed out")
        return original(configuration)
    def read_state():
        observed["status"] = runtime.status
        observed["configuration"] = runtime.configuration
        read_done.set()
    monkeypatch.setattr(runtime.settings, "detect_environment", detect)
    runtime.start()
    assert entered.wait(2.)
    reader = threading.Thread(target=read_state)
    reader.start()
    try:
        assert read_done.wait(.5), "UI state reads blocked behind filesystem discovery"
        assert observed["status"]["state"] == "starting"
        assert observed["configuration"] == before
        assert not runtime.settings_path.exists()
    finally:
        release.set()
        reader.join(timeout=2.)
    eventually(lambda: runtime.status["state"] == "ready")
    assert runtime.configuration["process_groups"][0]["parameters"]["output_root"]
    assert runtime.configuration != before
    assert json.loads(runtime.settings_path.read_text()) == runtime.configuration


@pytest.mark.parametrize("saved", [False, True])
def test_invalid_environment_is_reported_by_worker_without_launch_or_overwrite(
        runtime_factory, monkeypatch, saved):
    monkeypatch.setenv("VT_DATA_ROOT", "relative-data-root")
    runtime = runtime_factory(start=False)
    if saved:
        runtime.settings.save(runtime.configuration)
    before = runtime.settings_path.read_bytes() if saved else None
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert "VT_DATA_ROOT" in runtime.status["detail"]
    assert not runtime_factory.children
    assert (runtime.settings_path.read_bytes() if runtime.settings_path.exists() else None) == before


def test_default_output_preparation_failure_does_not_launch_or_replace_configuration(runtime_factory):
    runtime = runtime_factory(start=False)
    runtime.settings.save(runtime.configuration)
    before = runtime.settings_path.read_bytes()
    output = Path.home() / "visiontactile-data"
    output.parent.mkdir(parents=True)
    output.write_text("keep")
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert "默认数据目录" in runtime.status["detail"]
    assert runtime.settings_path.read_bytes() == before
    assert output.read_text() == "keep"
    assert not runtime_factory.children


@pytest.mark.parametrize("external", [False, True])
def test_reusing_backend_never_discovers_or_saves_new_environment_candidates(
        runtime_factory, tmp_path, monkeypatch, external):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    before = first.settings_path.read_bytes()
    first.close()
    if external:
        first._registry_path.unlink()
    camera = tmp_path / "ros2_ws/src/vt_realsense_capture/config/cameras.yaml"
    camera.parent.mkdir(parents=True)
    camera.write_text("# appeared after backend startup\n")
    replacement = runtime_factory(directory=first.settings.directory, source=first.source, start=False)
    def forbidden():
        pytest.fail("an existing backend must not trigger startup preparation")
    monkeypatch.setattr(replacement.settings, "prepare_for_startup", forbidden)
    replacement.start()
    eventually(lambda: replacement.status["state"] == ("external" if external else "ready"))
    assert replacement.settings_path.read_bytes() == before
    assert len(runtime_factory.children) == 1
    assert replacement.configuration["process_groups"][0]["parameters"]["config_path"] == ""


def test_alive_backend_with_initial_probe_failure_does_not_prepare_again(
        runtime_factory, monkeypatch):
    first = runtime_factory()
    eventually(lambda: first.status["state"] == "ready")
    before = first.settings_path.read_bytes()
    first.close()
    replacement = runtime_factory(directory=first.settings.directory, source=first.source, start=False)
    original = replacement._probe
    calls = 0
    def probe():
        nonlocal calls
        calls += 1
        if calls == 1:
            raise OSError("synthetic initial unavailability")
        return original()
    def forbidden():
        pytest.fail("a still-alive backend must not trigger startup preparation")
    monkeypatch.setattr(replacement, "_probe", probe)
    monkeypatch.setattr(replacement.settings, "prepare_for_startup", forbidden)
    replacement.start()
    eventually(lambda: replacement.status["state"] == "ready")
    assert calls >= 2
    assert replacement.settings_path.read_bytes() == before
    assert len(runtime_factory.children) == 1


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


def test_explicit_glove_config_is_preserved_without_composition(runtime_factory, tmp_path):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    path = tmp_path / "historical-right.json"
    original = b'{"sample_rate_hz":120,"gyro_dps_per_lsb":0.00875,"node_to_slot":{"12":0}}\n'
    path.write_bytes(original)
    assert not (runtime.settings.directory / "glove-configs").exists()
    draft = runtime.configuration
    right = next(g for g in draft["process_groups"] if g["id"] == "right_glove")
    right["parameters"].update(config=str(path), output_rate_hz="119")
    assert runtime.apply(draft, runtime._probe()[0])
    eventually(lambda: runtime.status["last_apply"]["state"] == "applied")
    right = next(g for g in runtime.configuration["process_groups"] if g["id"] == "right_glove")
    assert right["parameters"]["config"] == str(path)
    assert path.read_bytes() == original
    assert right["parameters"]["output_rate_hz"] == "119"
    assert not (runtime.settings.directory / "glove-configs").exists()
    assert len(runtime_factory.children) == 2


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


def test_apply_retains_selected_identity_if_tty_numbers_change_during_restart(
        runtime_factory, monkeypatch, tmp_path):
    from vt_capture_console.backend import serial_ports

    root = tmp_path / "dev"
    aliases = root / "serial/by-id"
    aliases.mkdir(parents=True)
    old, new = root / "ttyACM0", root / "ttyACM1"
    old.touch()
    new.touch()
    chosen, other = aliases / "chosen-glove", aliases / "other-device"
    chosen.symlink_to(old)
    other.symlink_to(new)
    monkeypatch.setattr(serial_ports, "DEV_ROOT", root)
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    configuration = runtime.configuration
    selected = next(g for g in configuration["process_groups"] if g["id"] == "right_glove")
    selected["parameters"]["port"] = str(old)
    stop = runtime._stop_owned

    def swap_numbering(record):
        chosen.unlink()
        other.unlink()
        chosen.symlink_to(new)
        other.symlink_to(old)
        return stop(record)

    monkeypatch.setattr(runtime, "_stop_owned", swap_numbering)
    assert runtime.apply(configuration, runtime._probe()[0])
    eventually(lambda: runtime.status["last_apply"]["state"] == "applied")
    saved = json.loads(runtime.settings_path.read_text())
    right = next(g for g in saved["process_groups"] if g["id"] == "right_glove")
    assert right["parameters"]["port"] == str(chosen)
    assert Path(right["parameters"]["port"]).resolve() == new


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


def previous_boot_socket(runtime):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(runtime.socket_path))
    record = dict(version=1, boot_id=str(uuid.uuid4()), socket_path=str(runtime.socket_path),
                  identity=dict(pid=1234, pgid=1234, session=1234, start_time=1),
                  pending_launch=False, socket_inode=runtime.socket_path.lstat().st_ino)
    atomic_json(runtime._registry_path, record)
    return record


def test_start_after_reboot_recovers_only_registered_stale_socket(runtime_factory):
    runtime = runtime_factory(start=False)
    previous_boot_socket(runtime)
    runtime.start()
    eventually(lambda: runtime.status["state"] == "ready")
    assert len(runtime_factory.children) == 1
    assert runtime._record()["boot_id"] == runtime._boot_id
    assert runtime.fresh_snapshot()["mode"] == "live"


def test_previous_boot_record_never_checks_or_signals_reused_pid(runtime_factory, monkeypatch):
    runtime = runtime_factory(start=False)
    previous_boot_socket(runtime)
    record = runtime._record()
    monkeypatch.setattr(local_runtime, "_identity", lambda pid: pytest.fail("previous-boot PID inspected"))
    monkeypatch.setattr(os, "pidfd_open", lambda pid: pytest.fail("previous-boot PID opened"))
    assert not runtime._record_alive(record)
    runtime._signal_owned(record, signal.SIGTERM)
    runtime._clean_owned_socket(record)
    assert not runtime.socket_path.exists()


@pytest.mark.parametrize("change", [
    {"identity": {"pid": 1234}}, {"boot_id": "invalid"}, {"boot_id": None},
    {"socket_path": "/other.sock"}, {"socket_inode": True}, {"socket_inode": 0},
])
def test_previous_boot_registry_validation_precedes_cleanup(runtime_factory, change):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    record.update(change)
    atomic_json(runtime._registry_path, record)
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert runtime.socket_path.exists()
    assert not runtime_factory.children


@pytest.mark.parametrize("change", [{"socket_inode": 1}, {"socket_inode": None}])
def test_previous_boot_missing_or_changed_inode_is_preserved(runtime_factory, change):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    if change["socket_inode"] is None:
        record.pop("socket_inode")
    else:
        record.update(change)
    atomic_json(runtime._registry_path, record)
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert runtime.socket_path.exists()
    assert not runtime_factory.children


@pytest.mark.parametrize("kind", ["file", "symlink"])
def test_previous_boot_ordinary_file_or_symlink_is_preserved(runtime_factory, kind):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    runtime.socket_path.unlink()
    if kind == "file":
        runtime.socket_path.write_text("preserve me")
    else:
        runtime.socket_path.symlink_to(runtime.source)
    record["socket_inode"] = runtime.socket_path.lstat().st_ino
    atomic_json(runtime._registry_path, record)
    runtime.start()
    eventually(lambda: runtime.status["state"] == "error")
    assert runtime.socket_path.exists()
    assert not runtime_factory.children


@pytest.mark.parametrize("current_boot", [False, True])
def test_live_listener_is_preserved_even_when_snapshot_fails(runtime_factory, current_boot):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    if current_boot:
        record["boot_id"] = runtime._boot_id
    runtime.socket_path.unlink()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(runtime.socket_path))
        listener.listen(16)
        record["socket_inode"] = runtime.socket_path.lstat().st_ino
        atomic_json(runtime._registry_path, record)
        runtime.start()
        eventually(lambda: runtime.status["state"] == "error")
        assert "监听" in runtime.status["detail"]
        assert runtime.socket_path.exists()
        assert not runtime_factory.children


def test_previous_boot_socket_with_different_owner_is_preserved(runtime_factory, monkeypatch):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    real_uid = os.getuid()
    monkeypatch.setattr(local_runtime.os, "getuid", lambda: real_uid + 1)
    with pytest.raises(RuntimeError, match="安全清理"):
        runtime._clean_owned_socket(record)
    assert runtime.socket_path.exists()


@pytest.mark.parametrize("error", [TimeoutError(), PermissionError()])
def test_socket_cleanup_requires_explicit_connection_refusal(runtime_factory, monkeypatch, error):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)

    class FailedConnection:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def settimeout(self, timeout):
            pass

        def connect(self, path):
            raise error

    monkeypatch.setattr(local_runtime.socket, "socket", lambda *args: FailedConnection())
    with pytest.raises(RuntimeError, match="停止监听"):
        runtime._clean_owned_socket(runtime._record())
    assert runtime.socket_path.lstat().st_ino == record["socket_inode"]


def test_socket_replaced_during_cleanup_is_preserved(runtime_factory, monkeypatch):
    runtime = runtime_factory(start=False)
    previous_boot_socket(runtime)
    record = runtime._record()
    original_lstat = Path.lstat
    calls = []

    def replaced(path, *args, **kwargs):
        if path == runtime.socket_path:
            calls.append(True)
            if len(calls) == 2:
                path.unlink()
                path.write_text("replacement")
        return original_lstat(path, *args, **kwargs)

    monkeypatch.setattr(Path, "lstat", replaced)
    with pytest.raises(RuntimeError, match="身份已变化"):
        runtime._clean_owned_socket(record)
    assert runtime.socket_path.read_text() == "replacement"


def stale_auxiliary_sockets(runtime):
    paths = [runtime.socket_path.with_name(name) for name in runtime.AUXILIARY_SOCKET_NAMES]
    for path in paths:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(path))
    return paths


@pytest.mark.parametrize("registered", [False, True])
def test_reboot_recovers_fixed_auxiliary_endpoints(runtime_factory, registered):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    paths = stale_auxiliary_sockets(runtime)
    unrelated = runtime.socket_path.with_name("unrelated.sock")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(unrelated))
    if registered:
        record["auxiliary_socket_inodes"] = {path.name: path.lstat().st_ino for path in paths}
        atomic_json(runtime._registry_path, record)
    else:
        # Model an old release's files surviving a later system boot.
        runtime._boot_started_ns = time.time_ns() + 1_000_000_000
    runtime.start()
    eventually(lambda: runtime.status["state"] == "ready")
    assert not any(path.exists() for path in paths)
    assert unrelated.exists()


def test_legacy_auxiliary_recovery_also_handles_failed_new_boot_launch(runtime_factory):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    record["boot_id"] = runtime._boot_id
    atomic_json(runtime._registry_path, record)
    paths = stale_auxiliary_sockets(runtime)
    runtime._boot_started_ns = time.time_ns() + 1_000_000_000
    runtime._clean_owned_socket(runtime._record())
    assert not any(path.exists() for path in paths)


@pytest.mark.parametrize("ctime_is_old", [False, True])
def test_unregistered_auxiliary_requires_both_timestamps_before_boot(runtime_factory, monkeypatch, ctime_is_old):
    runtime = runtime_factory(start=False)
    previous_boot_socket(runtime)
    paths = stale_auxiliary_sockets(runtime)
    original = Path.lstat

    def timestamps(path, *args, **kwargs):
        info = original(path, *args, **kwargs)
        if path not in paths:
            return info
        from types import SimpleNamespace
        values = {name: getattr(info, name) for name in dir(info) if name.startswith("st_")}
        values["st_ctime_ns" if ctime_is_old else "st_mtime_ns"] = runtime._boot_started_ns - 1
        return SimpleNamespace(**values)

    monkeypatch.setattr(Path, "lstat", timestamps)
    with pytest.raises(RuntimeError, match="缺少所有权"):
        runtime._clean_owned_socket(runtime._record())
    assert all(path.exists() for path in paths)


def test_legacy_auxiliary_live_listener_is_never_removed(runtime_factory):
    runtime = runtime_factory(start=False)
    previous_boot_socket(runtime)
    path = runtime.socket_path.with_name("manager-scene.sock")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        listener.listen(8)
        runtime._boot_started_ns = time.time_ns() + 1_000_000_000
        with pytest.raises(RuntimeError, match="仍有程序监听"):
            runtime._clean_owned_socket(runtime._record())
        assert path.exists()


@pytest.mark.parametrize("problem", ["unknown_name", "changed_inode", "public_parent"])
def test_auxiliary_recovery_rejects_unproven_ownership(runtime_factory, problem):
    runtime = runtime_factory(start=False)
    record = previous_boot_socket(runtime)
    paths = stale_auxiliary_sockets(runtime)
    record["auxiliary_socket_inodes"] = {path.name: path.lstat().st_ino for path in paths}
    if problem == "unknown_name":
        record["auxiliary_socket_inodes"]["../other.sock"] = 1
    elif problem == "changed_inode":
        record["auxiliary_socket_inodes"][paths[0].name] += 1
    else:
        runtime.socket_path.parent.chmod(0o755)
    atomic_json(runtime._registry_path, record)
    try:
        with pytest.raises((ValueError, RuntimeError)):
            runtime._clean_owned_socket(runtime._record())
        assert all(path.exists() for path in paths)
    finally:
        runtime.socket_path.parent.chmod(0o700)


def test_alive_manager_prevents_any_companion_socket_cleanup(runtime_factory):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    paths = stale_auxiliary_sockets(runtime)
    record = runtime._record()
    record["auxiliary_socket_inodes"] = {path.name: path.lstat().st_ino for path in paths}
    with pytest.raises(RuntimeError, match="安全清理"):
        runtime._clean_owned_socket(record)
    assert all(path.exists() for path in paths)
    assert runtime_factory.children[0].poll() is None


def test_late_manager_endpoints_are_authenticated_and_recorded(runtime_factory):
    code = FAKE_BACKEND.replace("server = socket.socket", '''
import threading, time
auxiliary = []
def create_auxiliary():
    time.sleep(.25)
    for name in ('manager-scene.sock', 'manager-cloud.sock'):
        endpoint = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        endpoint.bind(str(path.with_name(name)))
        endpoint.listen(8)
        auxiliary.append(endpoint)
threading.Thread(target=create_auxiliary, daemon=True).start()
server = socket.socket''')
    runtime = runtime_factory(child_code=code)
    eventually(lambda: runtime.status["state"] == "ready")
    eventually(lambda: len(runtime._record().get("auxiliary_socket_inodes", {})) == 2)
    assert runtime._record()["auxiliary_socket_inodes"] == {
        name: runtime.socket_path.with_name(name).lstat().st_ino for name in runtime.AUXILIARY_SOCKET_NAMES}


def test_unrelated_auxiliary_listener_is_not_claimed_by_manager(runtime_factory):
    runtime = runtime_factory()
    eventually(lambda: runtime.status["state"] == "ready")
    path = runtime.socket_path.with_name("manager-scene.sock")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        listener.listen(16)
        assert runtime._auxiliary_socket_inodes(runtime._record()) == {}


def test_fixed_manager_recipe_delegates_log_retention_to_backend(tmp_path):
    script = tmp_path / 'tools/capture_deploy/start_manager.sh'
    script.parent.mkdir(parents=True)
    script.touch()
    runtime = DesktopRuntime(tmp_path, tmp_path / 'private')
    try:
        argv = runtime._manager_argv()
        assert argv[argv.index('--log-path') + 1] == str(runtime.log_path)
    finally:
        runtime.close()


def test_backend_retains_logs_after_gui_detach(runtime_factory):
    # This child adopts the same output inode as the real manager entry point.
    code = FAKE_BACKEND.replace('done = False', '''
from vt_capture_core.process_log import ProcessLogRetention
import threading, time
retention = ProcessLogRetention(Path(source).parent / '0' / 'manager.log', output_fd=1,
                               limit_bytes=8192, keep_bytes=4096, interval_s=.01).start()
def noisy_backend():
    while not Path(source).with_suffix('.emit').exists():
        time.sleep(.01)
    print('bounded backend log ' * 8192, flush=True)
    Path(source).with_suffix('.emitted').touch()
threading.Thread(target=noisy_backend, daemon=True).start()
done = False''')
    runtime = runtime_factory(child_code=code)
    eventually(lambda: runtime.status['state'] == 'ready')
    runtime.close()
    runtime._thread.join(timeout=2)
    runtime.source.with_suffix('.emit').touch()
    eventually(runtime.source.with_suffix('.emitted').exists)
    eventually(lambda: runtime.log_path.stat().st_size == 4096)
    assert len(runtime_factory.children) == 1
    assert runtime_factory.children[0].poll() is None
