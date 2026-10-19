"""Hardware-free lifecycle tests. Only short-lived synthetic Python children run."""
from dataclasses import FrozenInstanceError
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

from vt_capture_manager import device_processes as devices
from vt_capture_manager.device_processes import ProcessSpec, ProcessSupervisor


def spec(identifier="camera", *, code="import time; time.sleep(120)", **changes):
    value = dict(id=identifier, label=identifier.title(), kind="camera",
                 argv=(sys.executable, "-u", "-c", code), topics=("/synthetic/" + identifier,),
                 resources=(identifier,), startup_timeout_s=.5)
    value.update(changes)
    return ProcessSpec(**value)


def eventually(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.01)
    raise AssertionError("timed out waiting for synthetic process state")


def group(supervisor, identifier="camera"):
    return next(value for value in supervisor.snapshot()["groups"] if value["id"] == identifier)


def command(supervisor, action, request_id=None, identifier="camera", **changes):
    params = dict(group_id=identifier, expected_manager_instance_id=supervisor.instance_id)
    params.update(changes)
    return supervisor.command("device." + action, params, request_id or action + "-" + identifier)


@pytest.fixture
def make_supervisor(tmp_path):
    supervisors = []

    def make(specs=None, *, start=True, state_dir=None, allowed=None, observe=None, **kwargs):
        specs = tuple(specs or (spec(),))
        samples = {topic: dict(publishers=0, fresh=False) for item in specs for topic in item.topics}
        events = []
        value = ProcessSupervisor(specs, instance_id="manager-" + str(len(supervisors)),
                                  state_dir=state_dir or tmp_path / str(len(supervisors)),
                                  observe=observe or (lambda: samples), allowed=allowed or (lambda action: (True, "idle")),
                                  event=lambda *args, **kwargs: events.append((args, kwargs)),
                                  poll_interval_s=.01, interrupt_timeout_s=.12, terminate_timeout_s=.12,
                                  kill_timeout_s=.3, **kwargs)
        value.samples = samples
        value.events = events
        supervisors.append(value)
        if start:
            value.start()
        return value

    yield make
    for supervisor in reversed(supervisors):
        supervisor.close()
        for runtime in supervisor._groups.values():
            # Test teardown is deliberately separate from supervisor.close(),
            # whose production contract is to leave device processes running.
            if runtime.identity and devices._session_members(runtime.identity):
                try:
                    os.killpg(runtime.identity["pgid"], signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if runtime.process is not None:
                try:
                    runtime.process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    pytest.fail("synthetic test child did not exit during teardown")


def test_frozen_spec_and_argv_validation():
    value = spec()
    with pytest.raises(FrozenInstanceError):
        value.argv = ("unsafe",)
    with pytest.raises(ValueError):
        spec(argv=["not-a-tuple"])
    with pytest.raises(ValueError):
        spec("../escape")
    with pytest.raises(ValueError):
        spec(startup_timeout_s=float("nan"))


def test_start_only_monitors_and_close_does_not_autolaunch(make_supervisor):
    supervisor = make_supervisor()
    assert group(supervisor)["state"] == "STOPPED"
    assert group(supervisor)["can_start"]
    assert not supervisor.busy
    time.sleep(.04)
    assert supervisor._groups["camera"].process is None
    supervisor.close()
    assert not group(supervisor)["can_start"]
    with pytest.raises(RuntimeError):
        supervisor.start()


def test_usb_failure_blocks_launch_and_snapshot_explains_recovery(make_supervisor):
    supervisor = make_supervisor()
    problem = ("usb_controller_failed", "USB 控制器失效（HC died），当前未枚举到配置的相机。")
    supervisor._camera_start_problem = lambda resources, **kwargs: problem
    state = group(supervisor)
    assert state["state"] == "FAILED" and not state["can_start"]
    assert "HC died" in state["detail"]
    assert command(supervisor, "start")["code"] == "usb_controller_failed"
    assert supervisor._groups["camera"].process is None
    supervisor._camera_start_problem = lambda resources, **kwargs: None
    assert group(supervisor)["can_start"]
    assert group(supervisor)["state"] == "STOPPED"


def test_usb_guard_does_not_remove_stop_from_an_owned_process(make_supervisor):
    supervisor = make_supervisor()
    assert command(supervisor, "start")["state"] == "accepted"
    eventually(lambda: group(supervisor)["owned"] and not supervisor.busy)
    supervisor._camera_start_problem = lambda resources, **kwargs: ("usb_controller_failed", "HC died")
    assert group(supervisor)["can_stop"]
    assert command(supervisor, "stop")["state"] == "accepted"
    eventually(lambda: supervisor.operation("stop-camera")["code"] == "stopped")


def test_usb_guard_rechecks_after_queued_admission_without_launch(make_supervisor, monkeypatch):
    supervisor = make_supervisor(start=False)
    # Queue admission can precede new hardware failure; the worker must check
    # again at execution rather than relying on the earlier snapshot.
    checks = []
    def guard(resources, *, refresh=False):
        checks.append(refresh)
        return ("usb_controller_failed", "HC died") if refresh else None
    supervisor._camera_start_problem = guard
    supervisor._thread = object()
    assert command(supervisor, "start")["state"] == "accepted"
    supervisor._thread = None
    monkeypatch.setattr(supervisor, "_launch", lambda _: pytest.fail("launch reached after controller failure"))
    supervisor._execute("device.start", "camera", "start-camera")
    assert supervisor.operation("start-camera")["code"] == "usb_controller_failed"
    assert checks == [False, True]


def test_readiness_degradation_and_explicit_stop(make_supervisor):
    supervisor = make_supervisor()
    assert command(supervisor, "start")["state"] == "accepted"
    eventually(lambda: group(supervisor)["owned"])
    assert group(supervisor)["state"] == "STARTING"
    eventually(lambda: not supervisor.busy)  # Readiness does not gate recording.
    supervisor.samples["/synthetic/camera"] = dict(publishers=1, fresh=True)
    eventually(lambda: group(supervisor)["state"] == "RUNNING")
    eventually(lambda: not supervisor.busy)
    pid = group(supervisor)["pid"]
    assert pid and pid != os.getpid()
    supervisor.samples["/synthetic/camera"]["fresh"] = False
    assert group(supervisor)["state"] == "DEGRADED"
    assert group(supervisor)["can_stop"]
    assert command(supervisor, "stop")["state"] == "accepted"
    supervisor.samples["/synthetic/camera"]["publishers"] = 0
    eventually(lambda: group(supervisor)["state"] == "STOPPED")
    eventually(lambda: not supervisor.busy)
    assert command(supervisor, "stop")["code"] == "stopped"
    assert not group(supervisor)["owned"]


def test_startup_timeout_degrades_without_killing_or_retrying(make_supervisor):
    supervisor = make_supervisor((spec(startup_timeout_s=.06),))
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["state"] == "DEGRADED")
    process = supervisor._groups["camera"].process
    assert process.poll() is None
    assert group(supervisor)["owned"]
    assert not supervisor.busy
    time.sleep(.08)
    assert supervisor._groups["camera"].process is process
    assert any("DEGRADED" in args[1] for args, _ in supervisor.events)


def test_all_topics_must_be_fresh(make_supervisor):
    supervisor = make_supervisor((spec(topics=("/synthetic/one", "/synthetic/two")),))
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["owned"])
    eventually(lambda: not supervisor.busy)
    supervisor.samples["/synthetic/one"] = dict(publishers=1, fresh=True)
    assert group(supervisor)["state"] == "STARTING"
    supervisor.samples["/synthetic/two"] = dict(publishers=1, fresh=True)
    assert group(supervisor)["state"] == "RUNNING"


def test_external_publishers_are_never_started_or_stopped(make_supervisor, monkeypatch):
    supervisor = make_supervisor()
    supervisor.samples["/synthetic/camera"] = dict(publishers=1, fresh=False)
    monkeypatch.setattr(devices.os, "killpg", lambda *args: pytest.fail("external group was signalled"))
    value = group(supervisor)
    assert value["state"] == "EXTERNAL" and not value["owned"]
    assert not value["can_start"] and not value["can_stop"] and not value["can_restart"]
    assert command(supervisor, "start")["code"] == "external_publishers"
    assert command(supervisor, "stop")["code"] == "not_owned"
    assert command(supervisor, "restart")["code"] == "not_owned"


def test_missing_or_failed_observations_fail_closed(make_supervisor):
    supervisor = make_supervisor(observe=lambda: {})
    assert group(supervisor)["state"] == "UNKNOWN"
    assert command(supervisor, "start")["code"] == "observations_unavailable"
    supervisor._observe = lambda: (_ for _ in ()).throw(RuntimeError("graph unavailable"))
    assert not group(supervisor)["can_start"]
    assert command(supervisor, "start", "new-id")["state"] == "rejected"


def test_disabled_stale_instance_and_payload_commands_rejected(make_supervisor):
    supervisor = make_supervisor((spec(enabled=False),))
    assert group(supervisor)["state"] == "DISABLED"
    assert command(supervisor, "start")["code"] == "device_disabled"
    assert command(supervisor, "start", "stale", expected_manager_instance_id="old")["code"] == "manager_instance_mismatch"
    assert command(supervisor, "start", "injected", argv=["sh", "-c", "false"])["code"] == "invalid_request"
    assert command(supervisor, "launch", "unknown")["code"] == "unsupported_command"


def test_duplicate_ids_do_not_launch_twice_and_conflicts_reject(make_supervisor, monkeypatch):
    launches = []
    original = devices.subprocess.Popen

    def launch(*args, **kwargs):
        launches.append((args, kwargs))
        return original(*args, **kwargs)

    monkeypatch.setattr(devices.subprocess, "Popen", launch)
    supervisor = make_supervisor()
    assert command(supervisor, "start", "same")["state"] == "accepted"
    eventually(lambda: command(supervisor, "start", "same")["code"] == "started")
    assert len(launches) == 1
    assert launches[0][0][0] == spec().argv
    assert launches[0][1]["start_new_session"] is True
    assert "shell" not in launches[0][1]
    assert command(supervisor, "stop", "same")["code"] == "request_id_conflict"


def test_resource_conflict_includes_queued_and_live_groups(make_supervisor):
    supervisor = make_supervisor((spec(resources=("usb-shared",)), spec("tracker", resources=("usb-shared",))))
    assert command(supervisor, "start")["state"] == "accepted"
    assert command(supervisor, "start", identifier="tracker")["code"] == "resource_conflict"
    eventually(lambda: group(supervisor)["owned"])
    assert command(supervisor, "start", "another", identifier="tracker")["code"] == "resource_conflict"
    assert supervisor._groups["tracker"].process is None


def test_guard_is_rechecked_before_dispatch(make_supervisor):
    actions = []

    def allowed(action):
        actions.append((threading.current_thread().name, action))
        return threading.current_thread().name != "device-process-control", "recording became active"

    supervisor = make_supervisor(allowed=allowed)
    assert command(supervisor, "start")["state"] == "accepted"
    eventually(lambda: not supervisor.busy)
    assert command(supervisor, "start")["code"] == "recorder_not_idle"
    assert supervisor._groups["camera"].process is None
    assert ("device-process-control", "device.start") in actions


def test_recorder_guard_rejects_lifecycle_changes(make_supervisor):
    supervisor = make_supervisor(allowed=lambda action: (False, "recorder state is stale"))
    assert command(supervisor, "start")["code"] == "recorder_not_idle"
    assert not group(supervisor)["can_start"]


def test_crash_is_failed_and_never_restarted(make_supervisor):
    supervisor = make_supervisor((spec(code="raise SystemExit(7)"),))
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["state"] == "FAILED")
    process = supervisor._groups["camera"].process
    assert process.returncode == 7
    assert not group(supervisor)["owned"]
    assert group(supervisor)["can_start"]
    time.sleep(.06)
    assert supervisor._groups["camera"].process is process


def test_close_detaches_durable_logs_and_new_manager_never_adopts(make_supervisor, tmp_path):
    directory = tmp_path / "shared"
    supervisor = make_supervisor((spec(code="import time; print('alive', flush=True); time.sleep(120)"),), state_dir=directory)
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["owned"])
    process = supervisor._groups["camera"].process
    log_path = Path(group(supervisor)["log_path"])
    eventually(lambda: log_path.read_text().startswith("alive"))
    supervisor.close()
    assert process.poll() is None
    registry = json.loads((directory / "device-processes.json").read_text())
    assert registry["groups"]["camera"]["identity"]["pid"] == process.pid
    replacement = make_supervisor(state_dir=directory)
    value = group(replacement)
    assert value["state"] == "UNKNOWN" and not value["owned"]
    assert value["pid"] == process.pid
    assert command(replacement, "start")["code"] == "ownership_unknown"
    assert command(replacement, "stop")["code"] == "not_owned"
    assert command(replacement, "restart")["code"] == "not_owned"
    replacement.samples["/synthetic/camera"] = dict(publishers=1, fresh=True)
    assert group(replacement)["state"] == "EXTERNAL"
    replacement.close()
    assert process.poll() is None


def test_same_state_directory_is_exclusively_locked(make_supervisor, tmp_path):
    directory = tmp_path / "locked"
    first = make_supervisor(state_dir=directory)
    with pytest.raises(RuntimeError, match="Another device supervisor"):
        make_supervisor(state_dir=directory)
    first.close()
    second = make_supervisor(state_dir=directory)
    assert group(second)["state"] == "STOPPED"


def test_live_leader_monitoring_does_not_enumerate_proc(make_supervisor, monkeypatch):
    supervisor = make_supervisor()
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["owned"])
    runtime = supervisor._groups["camera"]
    with monkeypatch.context() as context:
        context.setattr(devices.os, "scandir", lambda *args: pytest.fail("live leader caused full /proc scan"))
        members = devices._session_members(runtime.identity)
        assert len(members) == 1 and members[0]["pid"] == runtime.process.pid
        assert group(supervisor)["owned"]


def test_stop_signals_descendants_after_launcher_exited(make_supervisor, monkeypatch):
    child_code = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(120)"
    launcher_code = f"import subprocess,sys; p=subprocess.Popen([sys.executable,'-c',{child_code!r}]); print(p.pid,flush=True)"
    supervisor = make_supervisor((spec(code=launcher_code),))
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["state"] == "FAILED" and group(supervisor)["owned"])
    runtime = supervisor._groups["camera"]
    assert runtime.process.poll() == 0
    child_pid = int(Path(runtime.log_path).read_text().strip())
    assert devices._proc_info(child_pid)["state"] != "Z"
    scans = []
    original_scan = devices.os.scandir

    def scan(path):
        scans.append(path)
        return original_scan(path)

    monkeypatch.setattr(devices.os, "scandir", scan)
    members = devices._session_members(runtime.identity)
    assert any(member["pid"] == child_pid for member in members)
    assert "/proc" in scans  # Leader is gone: descendants require a full scan.
    assert command(supervisor, "stop")["state"] == "accepted"
    eventually(lambda: group(supervisor)["state"] == "STOPPED")
    assert devices._session_members(runtime.identity) == ()


def test_stop_escalates_to_sigkill_for_unresponsive_owned_group(make_supervisor):
    code = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); print('ready',flush=True); time.sleep(120)"
    supervisor = make_supervisor((spec(code=code),))
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["owned"])
    runtime = supervisor._groups["camera"]
    eventually(lambda: "ready" in Path(runtime.log_path).read_text())
    assert command(supervisor, "stop")["state"] == "accepted"
    eventually(lambda: runtime.process.poll() is not None)
    assert runtime.process.returncode == -signal.SIGKILL
    eventually(lambda: command(supervisor, "stop")["code"] == "stopped")


def test_restart_stops_then_launches_one_new_process(make_supervisor):
    supervisor = make_supervisor()
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["owned"])
    eventually(lambda: not supervisor._groups["camera"].pending)
    original = supervisor._groups["camera"].process
    assert command(supervisor, "restart")["state"] == "accepted"
    eventually(lambda: command(supervisor, "restart")["code"] == "started")
    replacement = supervisor._groups["camera"].process
    assert replacement.pid != original.pid and original.poll() is not None
    assert replacement.poll() is None


def test_pid_reuse_is_never_signalled(make_supervisor, monkeypatch):
    supervisor = make_supervisor()
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["owned"])
    original = devices._proc_info
    pid = supervisor._groups["camera"].process.pid

    def reused(value):
        info = original(value)
        if value == pid and info is not None:
            info["start_time"] += 1
        return info

    with monkeypatch.context() as context:
        context.setattr(devices, "_proc_info", reused)
        context.setattr(devices.os, "killpg", lambda *args: pytest.fail("reused PID was signalled"))
        assert group(supervisor)["state"] == "UNKNOWN"
        assert command(supervisor, "stop")["code"] == "not_owned"
        assert command(supervisor, "start", "another")["code"] == "ownership_unknown"


def test_attached_logs_are_bounded_and_private(make_supervisor):
    supervisor = make_supervisor((spec(code="import time; print('x'*100000,flush=True); time.sleep(120)"),))
    supervisor.LOG_LIMIT_BYTES = 8192
    supervisor.LOG_KEEP_BYTES = 4096
    command(supervisor, "start")
    eventually(lambda: group(supervisor)["owned"])
    path = Path(group(supervisor)["log_path"])
    eventually(lambda: path.stat().st_size > 0)
    eventually(lambda: path.stat().st_size <= supervisor.LOG_LIMIT_BYTES)
    assert path.stat().st_mode & 0o777 == 0o600


def test_incomplete_persisted_launch_blocks_relaunch_without_adoption(make_supervisor, tmp_path):
    directory = tmp_path / "intent"
    supervisor = make_supervisor(state_dir=directory)
    with supervisor._mutex:
        supervisor._registry["camera"] = dict(resources=["camera"], identity=None, pending_launch=True, log_path="")
        supervisor._save_registry()
    supervisor.close()
    replacement = make_supervisor(state_dir=directory)
    assert group(replacement)["state"] == "UNKNOWN"
    assert command(replacement, "start")["code"] == "ownership_unknown"


def test_missing_executable_is_persistently_failed_with_receipt(make_supervisor):
    supervisor = make_supervisor((spec(argv=("/nonexistent/synthetic-device",)),))
    assert supervisor.operation("start-camera") is None
    assert command(supervisor, "start")["code"] == "queued"
    eventually(lambda: not supervisor.busy)
    assert group(supervisor)["state"] == "FAILED"
    assert "/nonexistent/synthetic-device" in group(supervisor)["detail"]
    operation = supervisor.operation("start-camera")
    assert operation["code"] == "process_error"
    operation["code"] = "mutated"
    assert supervisor.operation("start-camera")["code"] == "process_error"
    time.sleep(.04)
    assert group(supervisor)["state"] == "FAILED"


def test_snapshot_conforms_to_device_protocol_and_receipt_cache_is_bounded(make_supervisor):
    from vt_capture_core.validation import validate_process_control

    supervisor = make_supervisor((spec(enabled=False),))
    validate_process_control(supervisor.snapshot())
    for index in range(supervisor.RECEIPT_LIMIT + 4):
        assert command(supervisor, "start", f"rejected-{index}")["state"] == "rejected"
    assert len(supervisor._receipts) == supervisor.RECEIPT_LIMIT
    assert supervisor.operation("rejected-0") is None


def test_external_publishers_appearing_after_admission_block_queued_start(make_supervisor):
    holder = {}

    def allowed(action):
        if threading.current_thread().name == "device-process-control":
            holder["supervisor"].samples["/synthetic/camera"] = dict(publishers=1, fresh=True)
        return True, "idle"

    supervisor = make_supervisor(allowed=allowed)
    holder["supervisor"] = supervisor
    assert command(supervisor, "start")["state"] == "accepted"
    eventually(lambda: not supervisor.busy)
    assert supervisor.operation("start-camera")["code"] == "external_publishers"
    assert supervisor._groups["camera"].process is None
    assert group(supervisor)["state"] == "EXTERNAL"


def tactile_profiles(*ports):
    from vt_capture_manager.launch_profiles import build_specs, parse_profiles
    return build_specs(parse_profiles([dict(id=hand + '_tactile', label=hand, preset='tactile', enabled=True, parameters=dict(hand=hand, port=str(port)))
                                      for hand, port in ports]))


def synthetic_launch(monkeypatch):
    """Exercise the real supervisor; replace ROS argv with our harmless child."""
    real_popen, launches = subprocess.Popen, []
    def launch(argv, **kwargs):
        launches.append(argv)
        return real_popen((sys.executable, '-c', 'import time; time.sleep(120)'), **kwargs)
    monkeypatch.setattr(devices.subprocess, 'Popen', launch)
    return launches


def test_serial_replug_rechecks_current_alias_and_freezes_launched_port(make_supervisor, tmp_path, monkeypatch):
    old, current, alias = (tmp_path / name for name in ('old', 'current', 'by-id'))
    old.touch()
    current.touch()
    alias.symlink_to(old)
    specs = tactile_profiles(('left', alias))
    service = make_supervisor(specs)
    alias.unlink()
    alias.symlink_to(current)
    old.unlink()
    launches = synthetic_launch(monkeypatch)
    result = command(service, 'start', identifier='left_tactile')
    assert result['state'] == 'accepted'
    eventually(lambda: not service.busy)
    assert 'port:=' + str(current) in launches[0]
    hint = json.loads(service._registry_path.read_text())['groups']['left_tactile']
    assert 'serial:' + str(current) in hint['resources']


def test_serial_alias_retarget_to_reserved_port_is_rejected(make_supervisor, tmp_path, monkeypatch):
    old, current, alias = (tmp_path / name for name in ('old', 'current', 'by-id'))
    old.touch()
    current.touch()
    alias.symlink_to(old)
    service = make_supervisor(tactile_profiles(('left', alias), ('right', current)))
    launches = synthetic_launch(monkeypatch)
    assert command(service, 'start', identifier='right_tactile')['state'] == 'accepted'
    eventually(lambda: not service.busy)
    alias.unlink()
    alias.symlink_to(current)
    assert command(service, 'start', identifier='left_tactile')['code'] == 'resource_conflict'
    assert len(launches) == 1


def test_running_serial_reservation_does_not_follow_later_alias_changes(make_supervisor, tmp_path, monkeypatch):
    old, current, alias = (tmp_path / name for name in ('old', 'current', 'by-id'))
    old.touch()
    current.touch()
    alias.symlink_to(old)
    service = make_supervisor(tactile_profiles(('left', alias), ('right', old)))
    launches = synthetic_launch(monkeypatch)
    assert command(service, 'start', identifier='left_tactile')['state'] == 'accepted'
    eventually(lambda: not service.busy)
    alias.unlink()
    alias.symlink_to(current)
    assert command(service, 'start', identifier='right_tactile')['code'] == 'resource_conflict'
    assert len(launches) == 1


def test_failed_log_creation_never_persists_an_uncertain_launch(make_supervisor, tmp_path, monkeypatch):
    directory = tmp_path / 'shared-log-failure'
    service = make_supervisor(state_dir=directory)
    real_open = os.open
    def denied(path, flags, *args, **kwargs):
        if Path(path).parent == service.logs:
            raise PermissionError('synthetic log create failure')
        return real_open(path, flags, *args, **kwargs)
    with monkeypatch.context() as context:
        context.setattr(devices.os, 'open', denied)
        command(service, 'start')
        eventually(lambda: not service.busy)
    assert service._groups['camera'].process is None
    service.close()
    replacement = make_supervisor(state_dir=directory)
    assert group(replacement)['can_start']
    assert replacement._registry == {}
