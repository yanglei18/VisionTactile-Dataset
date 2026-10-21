"""Exercise transaction order and cancellation with inert local processes only."""

import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from oneclick import engine, plan
from oneclick import delivery as delivery_module


class FakeRunner:
    def __init__(self, calls, *, sdk_installed=True, fail=None):
        self.calls, self.sdk_installed, self.fail = calls, sdk_installed, fail

    def run(self, argv, **kwargs):
        argv = list(map(str, argv))
        self.calls.append(("user", argv, kwargs))
        if self.fail and self.fail(argv):
            raise engine.InstallationError("injected failure")
        output, code = "", 0
        if argv[:3] == ["/usr/bin/python3", "-I", "-c"]:
            output = "3.12\n"
        if argv[:3] == ["/usr/bin/pkg-config", "--modversion", "realsense2"]:
            output, code = ("2.58.1\n", 0) if self.sdk_installed else ("", 1)
        if argv[:2] == ["/usr/bin/git", "clone"]:
            target = Path(argv[-1])
            target.mkdir(parents=True)
            (target / ".git").mkdir()
        if argv[0] == "/usr/bin/git" and "rev-parse" in argv:
            output = (plan.SDK.commit if "librealsense" in argv[2] else plan.WRAPPER.commit) + "\n"
        return subprocess.CompletedProcess(argv, code, output)


@pytest.fixture
def transaction(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir(mode=0o700)
    root = home / ".local/share/visiontactile/application/versions/fixture/source"
    root.mkdir(parents=True)
    (root / "ros2_ws").mkdir()
    package = root.parent / "deployment/tracker.vttracker"
    package.parent.mkdir()
    package.write_bytes(b"staged fixture")
    supplied = SimpleNamespace(source_root=tmp_path / "download/source",
                               tracker_package=tmp_path / "download/private.vttracker",
                               installation_id="fixture")
    monkeypatch.setattr(engine, "validate_host", lambda home: None)
    monkeypatch.setattr(engine, "assert_workbench_closed", lambda: None)
    monkeypatch.setattr(delivery_module, "inspect_delivery", lambda path: supplied)
    monkeypatch.setattr(delivery_module, "stage_delivery", lambda delivery, home: root)
    calls, events = [], []
    installer = engine.Installer(tmp_path / "download", home=home, notify=events.append)
    runner = FakeRunner(calls)
    installer.runner = runner

    class Broker:
        def __init__(self, *args):
            calls.append(("authenticate",))

        def request(self, action, **kwargs):
            calls.append(("system", action, kwargs))
            if action == "sdk_install":
                runner.sdk_installed = True

        def close(self):
            calls.append(("close",))

    installer.broker_factory = Broker
    return SimpleNamespace(installer=installer, runner=runner, calls=calls, events=events,
                           root=root, home=home, package=package)


def test_success_uses_one_broker_and_switches_desktop_only_after_validation(transaction):
    case = transaction
    result = case.installer.run()
    assert result["source_root"] == str(case.root)
    assert result["hardware_started"] is False
    assert sum(call[0] == "authenticate" for call in case.calls) == 1
    commands = [call[1] for call in case.calls if call[0] == "user"]
    install_index = next(i for i, cmd in enumerate(commands) if "--no-desktop" in cmd)
    tracker_index = next(i for i, cmd in enumerate(commands) if "--package" in cmd)
    desktop_index = next(i for i, cmd in enumerate(commands) if "vt_capture_console.desktop_entry" in cmd)
    assert install_index < tracker_index < desktop_index == len(commands) - 1
    assert commands[tracker_index][-1] == str(case.package)
    assert commands[install_index][-1] == "--no-desktop"
    assert case.calls[-1] == ("close",)
    report = json.loads((case.home / ".local/share/visiontactile/installation/last-success.json").read_text())
    assert report["wrapper_commit"] == plan.WRAPPER.commit


@pytest.mark.parametrize("failure", ["runtime", "qt", "tracker"])
def test_failure_never_registers_new_desktop_and_closes_broker(transaction, failure):
    case = transaction
    checks = {
        "runtime": lambda args: plan.RUNTIME_CHECK in args,
        "qt": lambda args: any("Qt runtime ready" in arg for arg in args),
        "tracker": lambda args: "--package" in args,
    }
    case.runner.fail = checks[failure]
    with pytest.raises(engine.InstallationError, match="injected"):
        case.installer.run()
    commands = [call[1] for call in case.calls if call[0] == "user"]
    assert not any("vt_capture_console.desktop_entry" in command for command in commands)
    if failure in {"runtime", "qt"}:
        assert not any("--package" in command for command in commands)
    assert case.calls[-1] == ("close",)
    assert not (case.home / ".local/share/visiontactile/installation/last-success.json").exists()


def test_missing_sdk_builds_exact_release_before_system_install(transaction):
    case = transaction
    case.runner.sdk_installed = False
    case.installer.run()
    sdk_install = next(i for i, call in enumerate(case.calls) if call[:2] == ("system", "sdk_install"))
    previous = case.calls[sdk_install - 1]
    assert previous[1][-2:] == ["--parallel", "2"]
    source = plan.Paths(case.home, case.root).sdk
    assert source.is_dir()
    assert any(call[0] == "user" and plan.SDK.url in call[1] for call in case.calls)


def test_wrong_git_tag_stops_before_build_or_desktop(transaction):
    case = transaction
    original = case.runner.run
    def moved_tag(argv, **kwargs):
        result = original(argv, **kwargs)
        if "rev-parse" in argv:
            result.stdout = "0" * 40
        return result
    case.runner.run = moved_tag
    with pytest.raises(engine.InstallationError, match="标签"):
        case.installer.run()
    assert not plan.Paths(case.home, case.root).wrapper.exists()
    assert not any(call[0] == "user" and "vt_capture_console.desktop_entry" in call[1] for call in case.calls)


def test_cancel_before_run_does_not_authenticate_or_install(transaction):
    case = transaction
    case.installer.cancel = lambda: True
    with pytest.raises(engine.InstallationCancelled):
        case.installer.run()
    assert not case.calls


def test_cancellation_after_tracker_activation_finishes_launcher_commit(transaction):
    case = transaction
    cancel = threading.Event()
    case.installer.cancel = cancel.is_set
    original = case.runner.run
    def activate_then_stop(argv, **kwargs):
        result = original(argv, **kwargs)
        if "--package" in argv:
            cancel.set()
        return result
    case.runner.run = activate_then_stop
    result = case.installer.run()
    assert result["source_root"] == str(case.root)
    assert any(call[0] == "user" and "vt_capture_console.desktop_entry" in call[1] for call in case.calls)


def test_command_paths_are_literal_shell_arguments(tmp_path):
    location = tmp_path / "配置 $(touch SHOULD_NOT_EXIST); ' spaces"
    args = plan.ros_command(["printf", "%s", str(location)], underlays=(location,))
    assert str(location) not in args[4]
    assert args[-1] == str(location)
    assert args[7] == str(location)


def test_environment_discards_frozen_python_and_developer_overrides(tmp_path, monkeypatch):
    for name in ("LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV",
                 "CONDA_PREFIX", "ROS_DISTRO", "AMENT_PREFIX_PATH", "VT_CONSOLE_PYTHON"):
        monkeypatch.setenv(name, "/untrusted")
    monkeypatch.setenv("DISPLAY", ":1")
    env = engine.clean_environment(tmp_path)
    assert env["DISPLAY"] == ":1"
    assert env["HOME"] == str(tmp_path)
    assert all(name not in env for name in ("LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH",
                                           "PYTHONHOME", "VIRTUAL_ENV", "CONDA_PREFIX",
                                           "ROS_DISTRO", "AMENT_PREFIX_PATH", "VT_CONSOLE_PYTHON"))


@pytest.mark.parametrize("command", [b"python\0-m\0vt_capture_manager.node\0",
                                      b"python\0-I\0-m\0vt_capture_console\0--desktop\0",
                                      b"/opt/ros/lib/vt_capture_manager/vt_capture_manager\0",
                                      b"/opt/ros/lib/vt_vive_tracker/vt_vive_tracker_node\0"])
def test_running_workbench_is_rejected_without_signalling(tmp_path, command):
    process = tmp_path / "999999"
    process.mkdir()
    (process / "cmdline").write_bytes(command)
    with pytest.raises(engine.InstallationError, match="关闭工作台"):
        engine.assert_workbench_closed(tmp_path)


def test_runner_cancellation_stops_only_its_owned_process_group(tmp_path):
    stop = threading.Event()
    pidfile = tmp_path / "child.pid"
    ready = threading.Event()
    def notify(event):
        if event.get("text") == "ready":
            ready.set()
            stop.set()
    runner = engine.Runner(tmp_path, notify, stop)
    code = ("import subprocess,sys,time,pathlib; "
            "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)']); "
            "pathlib.Path(sys.argv[1]).write_text(str(p.pid)); "
            "print('ready',flush=True); time.sleep(30)")
    started = time.monotonic()
    with pytest.raises(engine.InstallationCancelled):
        runner.run([sys.executable, "-I", "-c", code, str(pidfile)], timeout=10)
    assert ready.is_set()
    assert time.monotonic() - started < 6
    child = int(pidfile.read_text())
    status = Path(f"/proc/{child}/stat")
    assert not status.exists() or status.read_text().split()[2] == "Z"


def test_concurrent_installer_is_rejected(tmp_path):
    with engine.installation_lock(tmp_path):
        with pytest.raises(engine.InstallationError, match="另一个安装器"):
            with engine.installation_lock(tmp_path):
                pass


def test_system_cancellation_waits_for_result_without_killing_transaction(tmp_path, monkeypatch):
    script = tmp_path / "inert_broker.py"
    script.write_text(
        "import json,sys,time\n"
        "for line in sys.stdin:\n"
        "    action=json.loads(line)['action']\n"
        "    print(json.dumps({'kind':'log','text':'transaction started'}),flush=True)\n"
        "    time.sleep(0.1)\n"
        "    print(json.dumps({'kind':'result','action':action,'ok':True}),flush=True)\n"
    )
    original = subprocess.Popen
    processes, arguments, events = [], [], []
    def inert(argv, **kwargs):
        arguments.append(argv)
        process = original([sys.executable, "-I", str(script)], **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(engine.subprocess, "Popen", inert)
    cancel = threading.Event()
    def notify(event):
        events.append(event)
        if event.get("text") == "transaction started":
            cancel.set()
    broker = engine.SystemBroker(tmp_path, tmp_path, notify, cancel.is_set)
    try:
        with pytest.raises(engine.InstallationCancelled):
            broker.request("base")
        assert processes[0].poll() is None
    finally:
        broker.close()
    assert processes[0].returncode == 0
    assert len(arguments) == 1
    assert arguments[0][:3] == ["/usr/bin/pkexec", "/usr/bin/python3", "-I"]
    assert any("安全结束" in event.get("text", "") for event in events)
