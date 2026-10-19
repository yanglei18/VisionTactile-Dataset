"""Session launcher tests use inert fixtures, never serial devices or ROS nodes."""

import importlib.util
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest


DEPLOY = Path(__file__).resolve().parents[1]
HELPER = DEPLOY / "serial_session.py"
spec = importlib.util.spec_from_file_location("capture_serial_session", HELPER)
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


def groups(monkeypatch, *, authorized=True, active=False, primary=False):
    monkeypatch.setattr(session.pwd, "getpwuid", lambda uid: SimpleNamespace(pw_name="fixture", pw_gid=1000))
    monkeypatch.setattr(session.grp, "getgrnam", lambda name: SimpleNamespace(gr_gid=20))
    monkeypatch.setattr(session.os, "getgrouplist", lambda name, gid: [1000, 20] if authorized else [1000])
    monkeypatch.setattr(session.os, "getgroups", lambda: [1000, 20] if active else [1000])
    monkeypatch.setattr(session.os, "getegid", lambda: 20 if primary else 1000)


def test_account_authorization_is_not_confused_with_inherited_groups(monkeypatch):
    groups(monkeypatch)
    status = session.session_status()
    assert status["account_has_group"] and not status["process_has_group"]
    assert status["needs_refresh"] and status["gid"] == 20
    assert "重新打开" in status["detail"]
    with pytest.raises(RuntimeError, match="当前桌面进程尚未继承"):
        session.require_manager_session()


@pytest.mark.parametrize("primary", [False, True])
def test_supplementary_or_effective_primary_group_is_sufficient(monkeypatch, primary):
    groups(monkeypatch, active=not primary, primary=primary)
    status = session.require_manager_session()
    assert status["process_has_group"] and not status["needs_refresh"]


@pytest.mark.parametrize("missing_group", [False, True])
def test_no_account_authorization_never_requests_password_or_blocks_other_devices(monkeypatch, missing_group):
    groups(monkeypatch, authorized=False)
    if missing_group:
        def missing(name):
            raise KeyError(name)
        monkeypatch.setattr(session.grp, "getgrnam", missing)
    monkeypatch.setattr(session, "_run_refreshed", lambda *args: pytest.fail("sg attempted without authorization"))
    launched = []
    monkeypatch.setattr(session, "_exec_command", lambda args: launched.append(args))
    assert session.launch_workbench(["inert", "argument"]) == 0
    assert launched == [["inert", "argument"]]
    status = session.require_manager_session()
    assert not status["needs_refresh"]
    assert "仍可使用" in status["detail"]


def test_group_database_failure_does_not_attempt_refresh(monkeypatch):
    groups(monkeypatch)
    def unavailable(*args):
        raise OSError("fixture NSS unavailable")
    monkeypatch.setattr(session.os, "getgrouplist", unavailable)
    assert not session.require_manager_session()["needs_refresh"]


def test_check_command_is_machine_readable_and_does_not_launch(monkeypatch, capsys):
    groups(monkeypatch)
    monkeypatch.setattr(session.subprocess, "Popen", lambda *args, **kwargs: pytest.fail("unexpected process"))
    assert session.main(["check"]) == 0
    assert json.loads(capsys.readouterr().out)["needs_refresh"]
    assert session.main(["require"]) == 1
    assert "注销桌面会话" in capsys.readouterr().err


def test_refreshed_command_verifies_groups_without_recursive_retry(monkeypatch):
    groups(monkeypatch)
    monkeypatch.setattr(session, "_run_refreshed", lambda *args: pytest.fail("recursive refresh"))
    monkeypatch.setattr(session, "_exec_command", lambda *args: pytest.fail("unauthorized exec"))
    with pytest.raises(RuntimeError, match="刷新未生效"):
        session._refreshed(-1, 1000, ["inert"])


def test_refreshed_command_acknowledges_before_exec(monkeypatch):
    groups(monkeypatch, active=True)
    actions = []
    monkeypatch.setattr(session.os, "write", lambda fd, data: actions.append(("write", fd, data)))
    monkeypatch.setattr(session.os, "close", lambda fd: actions.append(("close", fd)))
    monkeypatch.setattr(session, "_exec_command", lambda args: actions.append(("exec", args)))
    monkeypatch.setattr(session.os, "getgid", lambda: 1000)
    session._refreshed(73, 1000, ["inert", "two words"])
    assert actions == [("write", 73, b"ready\n"), ("close", 73), ("exec", ["inert", "two words"])]


@pytest.mark.parametrize("real_gid,effective_gid", [(20, 20), (20, 1000), (1000, 20)])
def test_refreshed_command_refuses_wrong_primary_group(monkeypatch, real_gid, effective_gid):
    groups(monkeypatch, active=True)
    monkeypatch.setattr(session.os, "getgid", lambda: real_gid)
    monkeypatch.setattr(session.os, "getegid", lambda: effective_gid)
    monkeypatch.setattr(session, "_exec_command", lambda *args: pytest.fail("primary group not restored"))
    with pytest.raises(RuntimeError, match="主组恢复失败"):
        session._refreshed(-1, 1000, ["inert"])


def test_unknown_original_primary_group_fails_before_any_refresh(monkeypatch):
    def missing(gid):
        raise KeyError(gid)
    monkeypatch.setattr(session.grp, "getgrgid", missing)
    monkeypatch.setattr(session.subprocess, "Popen", lambda *args, **kwargs: pytest.fail("unrestorable group"))
    with pytest.raises(RuntimeError, match="无法确认工作台的原主组"):
        session._run_refreshed(["inert"])


def fixture_driver(tmp_path, *, status=None, fake_sg=None, timeout=.4):
    """Run the real launcher with an inert sg replacement and synthetic NSS."""
    driver = tmp_path / "driver with 空格.py"
    state = status or dict(needs_refresh=True)
    driver.write_text(
        "import importlib.util, sys\n"
        f"spec = importlib.util.spec_from_file_location('session', {str(HELPER)!r})\n"
        "session = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(session)\n"
        f"session.session_status = lambda: {state!r}\n"
        "original_refresh = session._run_refreshed\n"
        f"session._run_refreshed = lambda argv: original_refresh(argv, sg_path={str(fake_sg)!r}, timeout={timeout!r})\n"
        "raise SystemExit(session.main(['launch', '--', *sys.argv[1:]]))\n"
    )
    return driver


def fixture_sg(tmp_path, body):
    launcher = tmp_path / "fake sg 'quoted'.py"
    launcher.write_text(f"#!{sys.executable}\nimport os, sys, shlex, time\n" + body)
    launcher.chmod(0o700)
    return launcher


SG_ARGS = (
    "assert sys.argv[1:3] == ['dialout', '-c']\n"
    "restore = shlex.split(sys.argv[3])\n"
    "assert restore[:2] == ['exec', sys.argv[0]] and restore[3] == '-c'\n"
    "import grp\n"
    "assert restore[2] == grp.getgrgid(os.getegid()).gr_name\n"
    "parts = shlex.split(restore[4])\n"
    "assert parts[0] == 'exec' and parts[2] == '-I'\n"
    "assert int(parts[parts.index('--primary-gid') + 1]) == os.getegid()\n"
    "fd = int(parts[parts.index('--ready-fd') + 1])\n"
    "command = parts[parts.index('--') + 1:]\n"
)


def test_refresh_quotes_every_argument_and_has_no_stdin_or_controlling_terminal(tmp_path):
    fake_sg = fixture_sg(tmp_path, SG_ARGS +
        "assert sys.stdin.read() == ''\n"
        "assert os.getsid(0) == os.getpid()\n"
        "try:\n"
        "    os.close(os.open('/dev/tty', os.O_RDONLY))\n"
        "except OSError:\n"
        "    pass\n"
        "else:\n"
        "    raise AssertionError('unexpected controlling terminal')\n"
        "os.write(fd, b'ready\\n')\n"
        "os.close(fd)\n"
        "os.execv(command[0], command)\n")
    driver = fixture_driver(tmp_path, fake_sg=fake_sg)
    code = "import json, sys; print(json.dumps(sys.argv[1:], ensure_ascii=False))"
    arguments = ["two words", "单引号'双引号\"", "$(touch SHOULD_NOT_EXIST)", "semi;colon", "", "--option"]
    result = subprocess.run([sys.executable, str(driver), sys.executable, "-I", "-c", code, *arguments],
                            cwd=tmp_path, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == arguments
    assert not (tmp_path / "SHOULD_NOT_EXIST").exists()


@pytest.mark.parametrize("restore_stalls", [False, True])
def test_both_group_hops_share_one_bounded_handshake(tmp_path, restore_stalls):
    calls = tmp_path / "group-hops.jsonl"
    fake_sg = fixture_sg(tmp_path,
        "import json\n"
        f"with open({str(calls)!r}, 'a') as stream: stream.write(json.dumps(sys.argv[1]) + '\\n')\n"
        "parts = shlex.split(sys.argv[3])\n"
        "assert parts[0] == 'exec'\n"
        "if sys.argv[1] == 'dialout':\n"
        "    assert parts[1] == sys.argv[0]\n"
        "    os.execv(parts[1], parts[1:])\n"
        f"if {restore_stalls!r}: time.sleep(20)\n"
        "fd = int(parts[parts.index('--ready-fd') + 1])\n"
        "command = parts[parts.index('--') + 1:]\n"
        "os.write(fd, b'ready\\n')\n"
        "os.close(fd)\n"
        "os.execv(command[0], command)\n")
    driver = fixture_driver(tmp_path, fake_sg=fake_sg, timeout=.2)
    result = subprocess.run([sys.executable, str(driver), sys.executable, "-c", "print('fixture restored')"],
                            capture_output=True, text=True, timeout=5)
    assert [json.loads(value) for value in calls.read_text().splitlines()] == [
        "dialout", session.grp.getgrgid(os.getegid()).gr_name,
    ]
    if restore_stalls:
        assert result.returncode == 1 and "超时" in result.stderr and not result.stdout
    else:
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == "fixture restored"


def test_current_session_launch_is_exec_and_preserves_original_pid(tmp_path):
    driver = fixture_driver(tmp_path, status=dict(needs_refresh=False))
    process = subprocess.Popen(
        [sys.executable, str(driver), sys.executable, "-c", "import os; print(os.getpid())"],
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    out, err = process.communicate(timeout=5)
    assert process.returncode == 0, err
    assert int(out) == process.pid


@pytest.mark.parametrize("body", ["raise SystemExit(0)\n", SG_ARGS + "os.write(fd, b'bad\\n')\ntime.sleep(20)\n"])
def test_failed_handshake_never_starts_the_requested_command(tmp_path, body):
    driver = fixture_driver(tmp_path, fake_sg=fixture_sg(tmp_path, body))
    result = subprocess.run([sys.executable, str(driver), sys.executable, "-c", "print('must not launch')"],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 1
    assert "刷新失败或超时" in result.stderr
    assert result.stdout == ""


def test_handshake_timeout_is_bounded_and_own_fixture_process_is_reaped(tmp_path):
    pidfile = tmp_path / "fixture.pid"
    fake_sg = fixture_sg(tmp_path,
        f"with open({str(pidfile)!r}, 'w') as stream: stream.write(str(os.getpid()))\n"
        "time.sleep(20)\n")
    driver = fixture_driver(tmp_path, fake_sg=fake_sg, timeout=.2)
    started = time.monotonic()
    result = subprocess.run([sys.executable, str(driver), "/bin/true"], capture_output=True, text=True, timeout=5)
    assert time.monotonic() - started < 4
    assert result.returncode == 1 and "超时" in result.stderr
    with pytest.raises(ProcessLookupError):
        os.kill(int(pidfile.read_text()), 0)


def test_missing_sg_has_actionable_noninteractive_failure(tmp_path):
    driver = fixture_driver(tmp_path, fake_sg=tmp_path / "missing sg")
    result = subprocess.run([sys.executable, str(driver), "/bin/true"], capture_output=True, text=True, timeout=5)
    assert result.returncode == 1
    assert "注销桌面会话后重新登录" in result.stderr


def test_launcher_forwards_termination_to_only_its_own_fixture_session(tmp_path):
    fake_sg = fixture_sg(tmp_path, SG_ARGS +
        "os.write(fd, b'ready\\n')\n"
        "os.close(fd)\n"
        "print(os.getpid(), flush=True)\n"
        "time.sleep(20)\n")
    driver = fixture_driver(tmp_path, fake_sg=fake_sg)
    process = subprocess.Popen([sys.executable, str(driver), "/bin/true"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        fixture_pid = int(process.stdout.readline())
        process.send_signal(signal.SIGTERM)
        process.communicate(timeout=5)
        assert process.returncode == 128 + signal.SIGTERM
        with pytest.raises(ProcessLookupError):
            os.kill(fixture_pid, 0)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)


def test_workbench_shell_preserves_paths_and_user_arguments(tmp_path):
    root = tmp_path / "工作台 with space ' quote"
    deploy = root / "tools/capture_deploy"
    deploy.mkdir(parents=True)
    (deploy / "start_workbench.sh").write_text((DEPLOY / "start_workbench.sh").read_text())
    (deploy / "serial_session.py").write_text(
        "import os, sys\n"
        "assert sys.argv[1:3] == ['launch', '--']\n"
        "os.execv(sys.argv[3], sys.argv[3:])\n"
    )
    (deploy / "start_console.sh").write_text("#!/bin/bash\nprintf '%s\\0' \"$@\"\n")
    arguments = ["--config", "带 空格'与\"符号", "$(no command)", ""]
    result = subprocess.run(["/bin/bash", str(deploy / "start_workbench.sh"), *arguments],
                            capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.decode().split("\0") == ["--desktop", *arguments, ""]


def test_manager_checks_only_device_control_launches_before_sourcing_ros():
    content = (DEPLOY / "start_manager.sh").read_text()
    assert 'if [[ "$arg" == --enable-device-control ]]' in content
    assert content.index('serial_session.py" require') < content.index("source /opt/ros")
    assert 'exec /usr/bin/python3 -m vt_capture_manager.node "$@"' in content
    assert "sg dialout" not in content


@pytest.mark.parametrize("device_control,stale", [(False, True), (True, False), (True, True)])
def test_manager_shell_preserves_pid_and_fails_before_ros_in_stale_session(tmp_path, device_control, stale):
    root = tmp_path / "inert manager 空格 ' path"
    deploy = root / "tools/capture_deploy"
    deploy.mkdir(parents=True)
    ros_fixture = root / "ros setup.bash"
    sourced = root / "ros-was-sourced"
    ros_fixture.write_text(f"touch {shlex.quote(str(sourced))}\n")
    manager_script = (DEPLOY / "start_manager.sh").read_text()
    # Substitute only the ROS installation location; never source real ROS or
    # run a real capture manager in this fixture.
    manager_script = manager_script.replace("/opt/ros/jazzy/setup.bash", shlex.quote(str(ros_fixture)))
    (deploy / "start_manager.sh").write_text(manager_script)
    (deploy / "serial_session.py").write_text(
        "import sys\n"
        "assert sys.argv[1:] == ['require']\n"
        f"print('fixture stale session', file=sys.stderr) if {stale!r} else None\n"
        f"raise SystemExit({1 if stale else 0})\n"
    )
    install = root / "ros2_ws/install"
    install.mkdir(parents=True)
    (install / "setup.bash").write_text("# inert fixture\n")
    package = root / "ros2_ws/src/vt_capture_manager/vt_capture_manager"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("")
    (package / "node.py").write_text("import json, os, sys; print(json.dumps([os.getpid(), sys.argv[1:]]))\n")
    arguments = ["--config", "路径 with 空格 ' and $()"]
    if device_control:
        arguments.append("--enable-device-control")
    process = subprocess.Popen(["/bin/bash", str(deploy / "start_manager.sh"), *arguments],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = process.communicate(timeout=5)
    if device_control and stale:
        assert process.returncode == 1 and "fixture stale session" in err
        assert not sourced.exists() and not out
    else:
        assert process.returncode == 0, err
        assert sourced.exists()
        assert json.loads(out) == [process.pid, arguments]


def test_termination_during_popen_handoff_reaps_owned_session(tmp_path):
    """The child may be ready before Popen has returned to the launcher."""
    pidfile = tmp_path/'handoff.pid'
    fake_sg = fixture_sg(tmp_path, SG_ARGS+
        "os.write(fd, b'ready\\n')\nos.close(fd)\n"+
        f"with open({str(pidfile)!r}, 'w') as stream: stream.write(str(os.getpid()))\n"+
        "time.sleep(20)\n")
    driver = fixture_driver(tmp_path, fake_sg=fake_sg)
    injection = (
        "import os, signal, time\n"
        "real_popen = session.subprocess.Popen\n"
        "def interrupted_popen(*args, **kwargs):\n"
        "    child = real_popen(*args, **kwargs)\n"
        "    deadline = time.monotonic()+2\n"
        f"    while not os.path.exists({str(pidfile)!r}) and time.monotonic()<deadline: time.sleep(.001)\n"
        "    os.kill(os.getpid(), signal.SIGTERM)\n"
        "    return child\n"
        "session.subprocess.Popen = interrupted_popen\n"
    )
    driver.write_text(driver.read_text().replace('raise SystemExit(session.main', injection+'raise SystemExit(session.main'))
    process = subprocess.Popen([sys.executable, str(driver), '/bin/true'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        process.communicate(timeout=4)
        assert process.returncode == 128+signal.SIGTERM
        with pytest.raises(ProcessLookupError):
            os.kill(int(pidfile.read_text()), 0)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2)
        if pidfile.exists():
            try:
                os.killpg(int(pidfile.read_text()), signal.SIGKILL)
            except ProcessLookupError:
                pass
