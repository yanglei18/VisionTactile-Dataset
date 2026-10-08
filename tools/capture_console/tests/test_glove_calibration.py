"""State-only and mocked-X11 tests: never launch, focus, signal or open a device."""
from copy import deepcopy
import os
from pathlib import Path
import subprocess

import pytest

from vt_capture_console.backend import glove_calibration as module
from vt_capture_console.backend.glove_calibration import glove_group, inspect_status, focus_owned_calibration


def group(hand="right", **changes):
    value = dict(id=f"{hand}_glove", label="arbitrary label", kind="glove", state="RUNNING", owned=True,
                 pid=100, topics=[f"/gloves/{hand}/state"], can_start=False)
    value.update(changes)
    return value


def snapshot(hand="right", **changes):
    result = dict(mode="live", instance_id="manager", capabilities=dict(device_control=True),
                  process_control=dict(enabled=True, groups=[group(hand)]), devices=[])
    result.update(changes)
    return result


def device(hand="right", **changes):
    result = dict(kind="glove", side=hand, topic=f"/gloves/{hand}/state", status="online", age_s=.01,
                  receive_hz=30., valid_hz=30., preview=dict(valid=True))
    result.update(changes)
    return result


@pytest.fixture(autouse=True)
def no_process_or_actual_x11(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("Tests must not spawn a process or open real X11")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    monkeypatch.setattr(module, "_X11", forbidden)
    monkeypatch.setenv("DISPLAY", ":fake-test")
    monkeypatch.setenv("XDG_SESSION_TYPE", "x11")
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)


def test_group_selection_uses_hand_topic_not_row_label_or_id():
    value = snapshot()
    value["process_control"]["groups"] = [group("left", id="right_glove"), group("right", id="custom")]
    selected = glove_group(value, "right")
    assert selected["id"] == "custom"
    selected["id"] = "changed copy"
    assert value["process_control"]["groups"][1]["id"] == "custom"
    value["process_control"]["groups"].append(group("right"))
    assert glove_group(value, "right") is None  # Never retarget ambiguous ownership.


def test_mixed_hand_or_tactile_group_is_not_pose_owner():
    value = snapshot()
    value["process_control"]["groups"] = [group(topics=["/gloves/right/state", "/gloves/left/state"]),
                                           group(kind="tactile", topics=["/gloves/right/tactile/state"])]
    assert glove_group(value, "right") is None
    assert inspect_status(value, "right")["state"] == "UNCONFIGURED"


@pytest.mark.parametrize("changes", [dict(mode="demo"), dict(instance_id=""),
    dict(capabilities=dict(device_control=False)), dict(process_control=dict(enabled=False, groups=[group()]))])
def test_unavailable_backend_never_authorizes_cached_flags(changes):
    status = inspect_status(snapshot(**changes), "right")
    assert status["state"] == "UNAVAILABLE" and not status["can_start"] and not status["can_focus"]


@pytest.mark.parametrize("process_state,allowed,expected", [
    ("STOPPED", True, "STARTABLE"), ("FAILED", True, "STARTABLE"), ("STOPPED", False, "STOPPED"),
    ("DISABLED", True, "DISABLED"), ("EXTERNAL", True, "EXTERNAL"), ("UNKNOWN", True, "UNKNOWN"),
    ("STOPPING", True, "STOPPING"),
])
def test_process_state_keeps_start_and_focus_separate(process_state, allowed, expected):
    value = snapshot(process_control=dict(enabled=True, groups=[group(state=process_state, owned=False, pid=None, can_start=allowed)]))
    status = inspect_status(value, "right")
    assert status["state"] == expected
    assert status["can_start"] == (expected == "STARTABLE")
    assert not status["can_focus"]


@pytest.mark.parametrize("process_state", ["STARTING", "RUNNING", "DEGRADED"])
def test_running_without_data_does_not_invent_calibration_stage(process_state):
    value = snapshot(process_control=dict(enabled=True, groups=[group(state=process_state)]))
    status = inspect_status(value, "right")
    assert status["state"] == "RUNNING_NO_DATA" and status["can_focus"] and not status["can_start"]
    assert "尚无姿态数据" in status["message"]
    assert "检查 IMU 连接与标定状态" in status["message"]


@pytest.mark.parametrize("stream,expected", [
    (device(), "VALID_DATA"), (device(preview=dict(valid=False), valid_hz=0.), "INVALID_DATA"),
    (device(status="stale", age_s=3.), "STALE_DATA"), (device(status="offline", age_s=None), "RUNNING_NO_DATA"),
])
def test_pose_observations_do_not_mix_other_hand_or_tactile(stream, expected):
    value = snapshot(devices=[device("left"), {**device(), "kind": "tactile"}, stream])
    status = inspect_status(value, "right")
    assert status["state"] == expected and status["can_focus"]


def processes(hand="right"):
    return {
        100: dict(pid=100, ppid=1, pgid=100, session=100, start_time=1000,
                  argv=("/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "vt_glove_bridge", "glove.launch.py", f"hand:={hand}")),
        101: dict(pid=101, ppid=100, pgid=100, session=100, start_time=1001, argv=("launch-helper",)),
        102: dict(pid=102, ppid=101, pgid=102, session=100, start_time=1002,
                  argv=("/approved/python", "-I", "-m", "vt_fsglove12", "upstream-live", "--hand", hand, "--port", "/dev/never-open")),
    }


class FakeX11:
    def __init__(self, windows=None):
        self.clients = windows or [(500, 102)]
        self.activated = []
        self.closed = False
        self.accept = True
    def windows(self):
        return self.clients
    def window_pid(self, window):
        return dict(self.clients).get(window)
    def activate(self, window):
        self.activated.append(window)
        return self.accept
    def close(self):
        self.closed = True


def setup_focus(monkeypatch, table=None, connection=None):
    table = processes() if table is None else table
    connection = FakeX11() if connection is None else connection
    monkeypatch.setattr(module, "_read_process", lambda pid: deepcopy(table.get(pid)))
    monkeypatch.setattr(module, "_X11", lambda: connection)
    return table, connection


def test_focus_only_exact_owned_descendant_and_hand(monkeypatch):
    _, connection = setup_focus(monkeypatch)
    sent, message = focus_owned_calibration(group(), "right")
    assert sent and "已请求" in message and "未启动新采集" in message
    assert connection.activated == [500] and connection.closed


@pytest.mark.parametrize("changes", [dict(owned=False), dict(pid=None), dict(state="EXTERNAL"),
    dict(state="STOPPED"), dict(kind="tactile"), dict(topics=["/gloves/left/state"]), dict(pid=True)])
def test_unowned_or_wrong_hand_group_never_opens_x11(changes):
    assert focus_owned_calibration(group(**changes), "right")[0] is False


@pytest.mark.parametrize("change", [
    lambda p: p[102].update(ppid=1),
    lambda p: p[102].update(argv=("python", "-m", "vt_fsglove12", "upstream-live", "--hand", "left")),
    lambda p: p[102].update(argv=("python", "untrusted.py", "vt_fsglove12", "--hand", "right")),
    lambda p: p[102].update(argv=("python", "-m", "vt_fsglove12", "upstream-live", "--hand", "right", "--hand", "left")),
    lambda p: p[101].update(ppid=102),
    lambda p: p[100].update(argv=("another-process",)),
    lambda p: p[100].update(session=1),
])
def test_unrelated_reused_or_wrong_command_pid_never_receives_focus(monkeypatch, change):
    table = processes()
    change(table)
    _, connection = setup_focus(monkeypatch, table)
    sent, message = focus_owned_calibration(group(), "right")
    assert not sent and "Alt+Tab" in message and connection.activated == []


def test_process_birth_identity_rechecked_before_activation(monkeypatch):
    table, connection = setup_focus(monkeypatch)
    reads = []
    def read(pid):
        value = deepcopy(table.get(pid))
        reads.append(pid)
        if pid == 102 and reads.count(102) > 1:
            value["start_time"] += 1
        return value
    monkeypatch.setattr(module, "_read_process", read)
    assert not focus_owned_calibration(group(), "right")[0]
    assert connection.activated == [] and connection.closed


def test_window_pid_rechecked_before_activation(monkeypatch):
    _, connection = setup_focus(monkeypatch)
    connection.window_pid = lambda _window: 999
    assert not focus_owned_calibration(group(), "right")[0]
    assert not connection.activated


@pytest.mark.parametrize("wayland", [True, False])
def test_wayland_or_missing_display_returns_manual_switch_without_x11(monkeypatch, wayland):
    if wayland:
        monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    else:
        monkeypatch.delenv("DISPLAY")
    sent, message = focus_owned_calibration(group(), "right")
    assert not sent and "Alt+Tab" in message


def test_missing_x11_library_is_best_effort_not_crash(monkeypatch):
    setup_focus(monkeypatch)
    def unavailable():
        raise OSError("libX11 missing")
    monkeypatch.setattr(module, "_X11", unavailable)
    assert focus_owned_calibration(group(), "right")[0] is False


def test_declined_focus_does_not_claim_window_is_front(monkeypatch):
    _, connection = setup_focus(monkeypatch)
    connection.accept = False
    sent, message = focus_owned_calibration(group(), "right")
    assert not sent and "Alt+Tab" in message


def test_read_process_parses_bounded_proc_fixture_with_spaced_name(tmp_path, monkeypatch):
    directory = tmp_path / "102"
    directory.mkdir()
    fields = ["S", "100", "100", "100"] + ["0"] * 15 + ["1002"]
    (directory / "stat").write_text("102 (name with spaces) " + " ".join(fields))
    (directory / "cmdline").write_bytes(b"python\0-m\0vt_fsglove12\0upstream-live\0--hand\0right\0")
    monkeypatch.setattr(module, "Path", lambda value: tmp_path if value == "/proc" else Path(value))
    value = module._read_process(102)
    assert value["ppid"] == 100 and value["start_time"] == 1002
    assert module._calibration_command(value["argv"], "right")
    (directory / "cmdline").write_bytes(b"x" * 32769)
    assert module._read_process(102) is None
