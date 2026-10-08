"""Serial admission tests use simulated metadata, never actual serial devices."""
from pathlib import Path
import os
import signal
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from vt_capture_manager import device_processes as devices
from vt_capture_manager import serial_access


PORT = "/dev/ttyACM-test"


@pytest.fixture
def port(monkeypatch):
    state = SimpleNamespace(exists=True, accessible=True, account_groups={1000, 20},
                            process_groups={1000}, effective_gid=1000, calls=[])

    def stat(path):
        state.calls.append(("stat", path))
        if not state.exists:
            raise FileNotFoundError(path)
        return SimpleNamespace(st_gid=20)

    def access(path, mode, *, effective_ids):
        state.calls.append(("access", path, mode, effective_ids))
        return state.accessible

    def getgrouplist(name, gid):
        state.calls.append(("getgrouplist", name, gid))
        return state.account_groups

    # Replace this module's OS facade, not the shared os module used by pytest
    # or the supervisor's ownership checks. No simulated interface can open a
    # port, change a group, or write a device's permissions.
    monkeypatch.setattr(serial_access, "os", SimpleNamespace(
        stat=stat, access=access, R_OK=os.R_OK, W_OK=os.W_OK,
        geteuid=lambda: 1000, getegid=lambda: state.effective_gid,
        getgroups=lambda: state.process_groups, getgrouplist=getgrouplist))
    monkeypatch.setattr(serial_access, "pwd", SimpleNamespace(
        getpwuid=lambda uid: SimpleNamespace(pw_name="operator", pw_gid=1000)))
    monkeypatch.setattr(serial_access, "grp", SimpleNamespace(
        getgrgid=lambda gid: SimpleNamespace(gr_name="dialout", gr_gid=20),
        getgrnam=lambda name: SimpleNamespace(gr_name="dialout", gr_gid=20)))
    monkeypatch.setattr(serial_access, "open", lambda *a, **k: pytest.fail("serial device opened"), raising=False)
    return state


def check():
    return serial_access.serial_start_problem(("serial:" + PORT,))


def test_only_explicit_serial_resources_are_checked(port):
    assert serial_access.serial_start_problem(("camera", "/dev/ttyACM-test", "usb:glove")) is None
    assert port.calls == []


def test_access_checks_effective_read_and_write_without_account_lookup(port):
    assert check() is None
    assert port.calls == [("stat", PORT), ("access", PORT, os.R_OK | os.W_OK, True)]


def test_missing_serial_is_not_reported_as_permission_denied(port):
    port.exists = False
    code, detail = check()
    assert code == "serial_unavailable"
    assert PORT in detail and "不存在" in detail
    assert port.calls == [("stat", PORT)]


def test_unplug_during_access_check_is_reported_as_missing(port, monkeypatch):
    def unplug(path, mode, *, effective_ids):
        port.exists = False
        return False

    monkeypatch.setattr(serial_access.os, "access", unplug)
    code, detail = check()
    assert code == "serial_unavailable"
    assert "断开" in detail


def test_account_authorized_but_process_stale_gets_refresh_advice(port):
    port.accessible = False
    code, detail = check()
    assert code == "serial_permission_denied"
    assert PORT in detail and "dialout" in detail
    assert "已获" in detail and "旧会话" in detail and "刷新并重启采集后台" in detail
    assert "无需重复添加用户组" in detail
    assert "尚未获得" not in detail


def test_ungranted_account_gets_deployment_advice(port):
    port.accessible = False
    port.account_groups = {1000}
    code, detail = check()
    assert code == "serial_permission_denied"
    assert "尚未获得" in detail and "权限部署" in detail
    assert "旧会话" not in detail


@pytest.mark.parametrize("primary", [False, True])
def test_effective_primary_and_supplemental_group_both_count(port, primary):
    port.accessible = False
    if primary:
        port.effective_gid = 20
    else:
        port.process_groups.add(20)
    code, detail = check()
    assert code == "serial_permission_denied"
    assert "后台进程已包含 dialout" in detail and "ACL" in detail
    assert "旧会话" not in detail


def test_incomplete_account_database_keeps_actionable_denial(port, monkeypatch):
    port.accessible = False

    def missing_account(uid):
        raise KeyError(uid)

    monkeypatch.setattr(serial_access.pwd, "getpwuid", missing_account)
    code, detail = check()
    assert code == "serial_permission_denied"
    assert "无法完整读取账户权限信息" in detail


def test_stat_permission_failure_is_not_mistaken_for_missing_port(port, monkeypatch):
    def inaccessible(path):
        raise PermissionError(path)

    monkeypatch.setattr(serial_access.os, "stat", inaccessible)
    code, detail = check()
    assert code == "serial_permission_denied" and "旧会话" in detail


def test_effective_identity_check_unavailable_fails_closed(port, monkeypatch):
    def unsupported(path, mode, *, effective_ids):
        raise NotImplementedError

    monkeypatch.setattr(serial_access.os, "access", unsupported)
    assert check()[0] == "serial_unavailable"


def test_all_serial_ports_are_checked(port, monkeypatch):
    calls = []

    def access(path, mode, *, effective_ids):
        calls.append(path)
        return path != "/dev/ttyACM-second"

    monkeypatch.setattr(serial_access.os, "access", access)
    code, detail = serial_access.serial_start_problem(("serial:" + PORT, "serial:/dev/ttyACM-second"))
    assert code == "serial_permission_denied" and "/dev/ttyACM-second" in detail
    assert calls == [PORT, "/dev/ttyACM-second"]


def eventually(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.01)
    raise AssertionError("serial admission state did not settle")


@pytest.fixture
def supervisor(tmp_path, port):
    spec = devices.ProcessSpec(
        id="glove", label="Test glove", kind="glove", resources=("serial:" + PORT,),
        argv=(sys.executable, "-u", "-c", "import time; time.sleep(120)"),
        topics=("/test/glove",), startup_timeout_s=.5)
    value = devices.ProcessSupervisor(
        (spec,), instance_id="test-manager", state_dir=tmp_path,
        observe=lambda: {"/test/glove": dict(publishers=0, fresh=False)},
        allowed=lambda action: (True, "idle"), event=lambda *args, **kwargs: None,
        poll_interval_s=.01, interrupt_timeout_s=.1, terminate_timeout_s=.1, kill_timeout_s=.3).start()
    yield value
    value.close()
    for runtime in value._groups.values():
        if runtime.identity and devices._session_members(runtime.identity):
            try:
                os.killpg(runtime.identity["pgid"], signal.SIGKILL)
            except ProcessLookupError:
                pass
        if runtime.process is not None:
            runtime.process.wait(timeout=2.)


def status(supervisor):
    return supervisor.snapshot()["groups"][0]


def command(supervisor, action, request_id=None):
    return supervisor.command("device." + action,
                              dict(group_id="glove", expected_manager_instance_id="test-manager"),
                              request_id or action)


@pytest.mark.parametrize("missing", [False, True])
def test_snapshot_and_admission_explain_serial_failure_without_launch(supervisor, port, monkeypatch, missing):
    port.exists = not missing
    port.accessible = False
    monkeypatch.setattr(devices.subprocess, "Popen", lambda *a, **k: pytest.fail("blocked process launched"))
    expected = "serial_unavailable" if missing else "serial_permission_denied"
    current = status(supervisor)
    assert current["state"] == "FAILED" and not current["can_start"]
    assert PORT in current["detail"]
    assert command(supervisor, "start")["code"] == expected
    assert supervisor._groups["glove"].process is None
    assert supervisor._registry == {}
    assert not list(Path(supervisor.logs).iterdir())
    port.exists = port.accessible = True
    assert status(supervisor)["can_start"]
    assert status(supervisor)["state"] == "STOPPED"


def test_worker_rechecks_permissions_changed_after_admission(supervisor, port, monkeypatch):
    def allowed(action):
        if threading.current_thread().name == "device-process-control":
            port.accessible = False
        return True, "idle"

    supervisor._allowed = allowed
    monkeypatch.setattr(devices.subprocess, "Popen", lambda *a, **k: pytest.fail("stale queued launch dispatched"))
    assert command(supervisor, "start")["code"] == "queued"
    eventually(lambda: not supervisor.busy)
    assert supervisor.operation("start")["code"] == "serial_permission_denied"
    assert status(supervisor)["state"] == "FAILED"
    assert supervisor._registry == {}


def test_worker_rechecks_port_removed_after_admission(supervisor, port, monkeypatch):
    def allowed(action):
        if threading.current_thread().name == "device-process-control":
            port.exists = False
        return True, "idle"

    supervisor._allowed = allowed
    monkeypatch.setattr(devices.subprocess, "Popen", lambda *a, **k: pytest.fail("unplugged device launched"))
    assert command(supervisor, "start")["code"] == "queued"
    eventually(lambda: not supervisor.busy)
    assert supervisor.operation("start")["code"] == "serial_unavailable"


def test_stop_is_still_available_after_access_is_lost(supervisor, port):
    assert command(supervisor, "start")["code"] == "queued"
    eventually(lambda: supervisor.operation("start")["code"] == "started")
    process = supervisor._groups["glove"].process
    port.exists = port.accessible = False
    current = status(supervisor)
    assert current["owned"] and current["can_stop"] and current["can_restart"]
    assert command(supervisor, "stop")["code"] == "queued"
    eventually(lambda: supervisor.operation("stop")["code"] == "stopped")
    assert process.poll() is not None


def test_restart_stops_owned_process_before_permission_recheck(supervisor, port):
    assert command(supervisor, "start")["code"] == "queued"
    eventually(lambda: supervisor.operation("start")["code"] == "started")
    process = supervisor._groups["glove"].process
    port.accessible = False
    assert status(supervisor)["can_restart"]
    assert command(supervisor, "restart")["code"] == "queued"
    eventually(lambda: not supervisor.busy)
    assert process.poll() is not None
    assert supervisor._groups["glove"].process is process
    assert supervisor.operation("restart")["code"] == "serial_permission_denied"
    assert status(supervisor)["state"] == "FAILED"
    assert supervisor._registry == {}


def test_accessible_serial_still_launches_synthetic_child(supervisor):
    assert command(supervisor, "start")["code"] == "queued"
    eventually(lambda: supervisor.operation("start")["code"] == "started")
    assert status(supervisor)["owned"]
