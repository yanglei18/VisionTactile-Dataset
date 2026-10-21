"""Privilege-boundary tests; all installation commands are fakes.

Only the pure usbfs script is exercised against temporary ordinary files.
No package, group, device, udev, ROS, or systemd action is executed.
"""
from __future__ import annotations

import importlib.util
import io
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = "vt_oneclick_system_tests"
if PACKAGE not in sys.modules:
    spec = importlib.util.spec_from_file_location(
        PACKAGE, ROOT / "oneclick/__init__.py",
        submodule_search_locations=[str(ROOT / "oneclick")],
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[PACKAGE] = module
    spec.loader.exec_module(module)
spec = importlib.util.spec_from_file_location(PACKAGE + ".system", ROOT / "oneclick/system.py")
system = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = system
spec.loader.exec_module(system)


class FakeRunner:
    def __init__(self):
        self.calls = []
        self.responses = {}

    def run(self, arguments, **options):
        self.calls.append((arguments, options))
        if arguments[-1] == "check":
            return self.responses.get("check", (0, '{"rule_conflict":false}'))
        return self.responses.get(tuple(arguments), (0, ""))


@pytest.fixture
def installation(tmp_path):
    previous_umask = os.umask(0o022)
    home = tmp_path / "user"
    home.mkdir()
    repo = home / "source"
    (repo / "ros2_ws/src").mkdir(parents=True)
    fake_system = tmp_path / "system"
    fake_system.mkdir()
    account = SimpleNamespace(pw_dir=str(home), pw_uid=os.getuid(), pw_name="test-user")
    runner = FakeRunner()
    actions = system.SystemActions(account, repo, runner,
                                   system_root=fake_system, system_uid=os.getuid())
    try:
        yield actions, runner
    finally:
        os.umask(previous_umask)


@pytest.mark.parametrize("payload", [
    {"action": "exec", "command": "anything"},
    {"action": "base", "command": ["anything"]},
    {"action": "sdk_install", "path": "/etc"},
    {"action": "deps", "target": "../../elsewhere"},
    {"action": ["base"]},
    ["base"],
    {"action": "close", "unexpected": True},
])
def test_protocol_rejects_unknown_commands_and_arguments(installation, payload):
    actions, runner = installation
    events = []
    system.serve(actions, io.StringIO(json.dumps(payload) + "\n"), events.append)
    assert events[-1]["ok"] is False
    assert not runner.calls


def test_protocol_recovers_from_bad_json_and_closes(installation):
    actions, runner = installation
    events = []
    system.serve(actions, io.StringIO('bad\n{"action":"close"}\n{"action":"base"}\n'), events.append)
    assert [event["ok"] for event in events] == [False, True]
    assert not runner.calls


def test_protocol_does_not_publish_exception_private_contents(installation):
    actions, _ = installation
    actions.base = lambda: (_ for _ in ()).throw(OSError("SECRET_BUNDLE_BODY"))
    events = []
    system.serve(actions, io.StringIO('{"action":"base"}\n'), events.append)
    assert "SECRET" not in json.dumps(events)
    assert events[-1]["errorcode"] == "system_error"


def test_identity_requires_matching_pkexec_user(monkeypatch, installation):
    actions, _ = installation
    monkeypatch.setattr(system.os, "geteuid", lambda: 0)
    monkeypatch.setenv("PKEXEC_UID", "1234")
    with pytest.raises(system.SystemError, match="图形安装器"):
        system.validate_identity(5678, actions.repo)


def test_identity_rejects_root_even_with_matching_environment(monkeypatch, installation):
    actions, _ = installation
    monkeypatch.setattr(system.os, "geteuid", lambda: 0)
    monkeypatch.setenv("PKEXEC_UID", "0")
    with pytest.raises(system.SystemError):
        system.validate_identity(0, actions.repo)


def test_environment_does_not_inherit_credentials_or_python_paths(monkeypatch):
    for name in ("GH_TOKEN", "LD_PRELOAD", "PYTHONPATH", "PIP_INDEX_URL", "HTTP_PROXY"):
        monkeypatch.setenv(name, "PRIVATE")
    env = system.system_environment()
    assert "PRIVATE" not in env.values()
    assert env["PATH"] == "/usr/sbin:/usr/bin:/sbin:/bin"
    assert env["HOME"] == system.pwd.getpwuid(0).pw_dir


def fake_command(monkeypatch):
    read_fd, write_fd = os.pipe()
    os.write(write_fd, b"package transaction finished\n")
    os.close(write_fd)
    waited = []
    process = SimpleNamespace(stdout=os.fdopen(read_fd, "rb"),
                              wait=lambda: waited.append(True) or 0)
    monkeypatch.setattr(system.subprocess, "Popen", lambda *a, **k: process)
    return waited


def test_command_timeout_waits_for_transaction_exit_before_error(monkeypatch):
    waited = fake_command(monkeypatch)
    clock = iter([0, 10, 11, 12])
    monkeypatch.setattr(system.time, "monotonic", lambda: next(clock))
    events = []
    with pytest.raises(system.SystemError) as error:
        system.CommandRunner(events.append).run(["fake-package-command"], timeout=1)
    assert error.value.code == "command_timeout"
    assert waited == [True]
    assert any("package transaction finished" in event["text"] for event in events)


def test_command_cancellation_waits_for_transaction_exit(monkeypatch):
    waited = fake_command(monkeypatch)
    events = []
    runner = system.CommandRunner(events.append, cancelled=lambda: bool(waited))
    with pytest.raises(system.SystemError) as error:
        runner.run(["fake-package-command"])
    assert error.value.code == "cancelled"
    assert waited == [True]


def test_base_only_installs_fixed_packages_without_removals(installation):
    actions, runner = installation
    actions.base()
    commands = [command for command, _ in runner.calls]
    assert all(isinstance(command, list) for command in commands)
    assert all("--no-remove" in command for command in commands if "install" in command)
    assert not any("pip" in part or "colcon" == part for command in commands for part in command)


def test_ros_source_digest_checked_before_dpkg(installation, monkeypatch):
    actions, runner = installation
    monkeypatch.setattr(system.urllib.request, "urlopen", lambda *a, **k: io.BytesIO(b"wrong package"))
    with pytest.raises(system.SystemError) as error:
        actions.ros()
    assert error.value.code == "source_digest_invalid"
    assert not any(command[0] == "/usr/bin/dpkg" for command, _ in runner.calls)


def test_ros_existing_official_source_is_reused(installation, monkeypatch):
    actions, runner = installation
    runner.responses[("/usr/bin/dpkg-query", "-W", "-f=${Status}", "ros2-apt-source")] = (
        0, "install ok installed")
    monkeypatch.setattr(system.urllib.request, "urlopen", lambda *a, **k: pytest.fail("no download"))
    actions.ros()
    assert any(command[:2] == ["/usr/bin/rosdep", "update"] for command, _ in runner.calls)
    assert not any(command[0] == "/usr/bin/dpkg" for command, _ in runner.calls)


def test_ros_unknown_source_is_not_overwritten(installation):
    actions, runner = installation
    source = actions.path("/etc/apt/sources.list.d/ros2.sources")
    source.parent.mkdir(parents=True)
    source.write_text("custom")
    with pytest.raises(system.SystemError) as error:
        actions.ros()
    assert error.value.code == "ros_source_conflict"
    assert source.read_text() == "custom"


def dependency_responses(actions, runner, keys, resolved):
    key_command = ("/usr/bin/rosdep", "keys", "--from-paths", str(actions.repo / "ros2_ws/src"),
                   "--ignore-src", "--rosdistro", "jazzy")
    runner.responses[key_command] = (0, keys)
    included = sorted(set(keys.split()) - {"librealsense2", "realsense2_camera", "realsense2_camera_msgs"})
    runner.responses[("/usr/bin/rosdep", "resolve", *included, "--rosdistro", "jazzy")] = (0, resolved)


def test_dependencies_only_install_apt_packages_and_skip_source_driver(installation):
    actions, runner = installation
    dependency_responses(actions, runner, "librealsense2\npython3-yaml\n", "#apt\npython3-yaml\n")
    actions.deps("main")
    commands = [command for command, _ in runner.calls]
    assert commands[-1][-1] == "python3-yaml"
    assert commands[-1][0] == "/usr/bin/apt-get"
    assert "librealsense2" not in commands[-2]


def test_dependencies_parse_real_rosdep_multiple_key_groups(installation):
    actions, runner = installation
    # Output shape verified with the Ubuntu rosdep CLI, including its headings.
    resolved = "#ROSDEP[python3-numpy]\n#apt\npython3-numpy\n#ROSDEP[python3-yaml]\n#apt\npython3-yaml\n"
    dependency_responses(actions, runner, "python3-numpy\npython3-yaml\n", resolved)
    actions.deps("main")
    assert runner.calls[-1][0][-2:] == ["python3-numpy", "python3-yaml"]


@pytest.mark.parametrize("resolved", [
    "#ROSDEP[other]\n#apt\npython3-numpy\n",
    "#ROSDEP[python3-numpy]\n#apt\npython3-numpy\n",
    "#ROSDEP[python3-numpy]\n#pip\nnumpy\n#ROSDEP[python3-yaml]\n#apt\npython3-yaml\n",
])
def test_dependencies_reject_unknown_incomplete_or_non_apt_groups(installation, resolved):
    actions, runner = installation
    dependency_responses(actions, runner, "python3-numpy\npython3-yaml\n", resolved)
    with pytest.raises(system.SystemError):
        actions.deps("main")
    assert not any(command[0] == "/usr/bin/apt-get" for command, _ in runner.calls)


@pytest.mark.parametrize("resolved", ["#pip\nnumpy\n", "#apt\n--reinstall\n", "rm -rf /\n"])
def test_dependencies_reject_non_apt_or_command_like_output(installation, resolved):
    actions, runner = installation
    dependency_responses(actions, runner, "numpy", resolved)
    with pytest.raises(system.SystemError):
        actions.deps("main")
    assert not any(command[0] == "/usr/bin/apt-get" for command, _ in runner.calls)


def test_build_symlink_is_not_used_with_root(installation, tmp_path):
    actions, _ = installation
    actions.sdk.mkdir(parents=True)
    (actions.sdk / "build").symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(system.SystemError):
        actions.user_directory(actions.sdk / "build")


def test_sdk_version_conflict_never_installs(installation):
    actions, runner = installation
    runner.responses[("/usr/bin/pkg-config", "--modversion", "realsense2")] = (0, "2.57.0\n")
    with pytest.raises(system.SystemError) as error:
        actions.sdk_install()
    assert error.value.code == "sdk_conflict"
    assert not any(command[0] == "/usr/bin/cmake" for command, _ in runner.calls)


def test_matching_sdk_does_not_need_reinstall(installation):
    actions, runner = installation
    runner.responses[("/usr/bin/pkg-config", "--modversion", "realsense2")] = (0, "2.58.1\n")
    actions.sdk_install()
    assert len(runner.calls) == 1


def test_existing_unknown_permission_rule_prevents_all_mutation(installation):
    actions, runner = installation
    rule = actions.path("/etc/udev/rules.d/70-visiontactile-tracker-hid.rules")
    rule.parent.mkdir(parents=True)
    rule.write_text("custom rule\n")
    with pytest.raises(system.SystemError) as error:
        actions.permissions()
    assert error.value.code == "configuration_conflict"
    assert not runner.calls
    assert rule.read_text() == "custom rule\n"


def test_foreign_realsense_rule_prevents_all_mutation(installation):
    actions, runner = installation
    rule = actions.path("/etc/udev/rules.d/99-external.rules")
    rule.parent.mkdir(parents=True)
    rule.write_text('ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5b", MODE="0666"\n')
    with pytest.raises(system.SystemError) as error:
        actions.permissions()
    assert error.value.code == "udev_conflict"
    assert not runner.calls


@pytest.mark.parametrize("text,conflicts", [
    ('# ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5b"', False),
    ('ATTR{idVendor}=="0bb4", MODE="0666"', True),
    ('ATTR{idVendor}=="8086", ATTR{idProduct}=="1234"', False),
    ('ATTR{idVendor}=="8086", ATTR{idProduct}=="0b*"', True),
])
def test_udev_conflict_recognizes_vendor_wide_access(text, conflicts):
    assert system.conflicts_with_devices(text) is conflicts


def test_tracker_rules_match_existing_dongle_and_direct_tracker_support():
    canonical = ROOT.parents[1] / "tools/vut_validation/config/70-vive-ultimate-tracker.rules"
    rules = system.TRACKER_RULE.partition("\n")[2]
    assert rules == canonical.read_text()
    for product in ("0350", "06a3"):
        assert system.conflicts_with_devices(
            f'ATTRS{{idVendor}}=="0bb4", ATTRS{{idProduct}}=="{product}"')
    assert not system.conflicts_with_devices(
        'ATTRS{idVendor}=="0bb4", ATTRS{idProduct}=="9999"')


def test_permissions_are_idempotent_and_reuse_existing_usb_helper(installation):
    actions, runner = installation
    actions.permissions()
    actions.permissions()
    helper = str(actions.repo / "tools/capture_deploy/configure_realsense_usb.py")
    assert ["/usr/bin/python3", "-I", helper, "apply"] in [c for c, _ in runner.calls]
    assert actions.path("/etc/udev/rules.d/70-visiontactile-tracker-hid.rules").read_text() == system.TRACKER_RULE
    assert actions.path("/etc/systemd/system/visiontactile-usbfs-memory.service").read_text() == system.USBFS_SERVICE
    assert not any("chmod" in command[0] for command, _ in runner.calls)


@pytest.mark.parametrize("original,expected", [(0, 0), (16, 64), (64, 64), (512, 512)])
def test_boot_usbfs_helper_never_reduces_existing_limit(tmp_path, monkeypatch, original, expected):
    memory = tmp_path / "usbfs_memory_mb"
    memory.write_text(str(original) + "\n")
    # Substitute only the sysfs path, exercising the shipped helper logic.
    source = system.USBFS_HELPER.replace(
        'Path("/sys/module/usbcore/parameters/usbfs_memory_mb")', f"Path({str(memory)!r})")
    exec(compile(source, "usbfs-test", "exec"), {})
    assert int(memory.read_text()) == expected
