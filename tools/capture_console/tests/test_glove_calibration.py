"""Managed process and data status tests: never launch, focus, signal or open a device."""
from copy import deepcopy
import os
from pathlib import Path
import subprocess

import pytest

from vt_capture_console.backend import glove_calibration as module
from vt_capture_console.backend.glove_calibration import glove_group, inspect_status


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
    assert status["state"] == "UNAVAILABLE" and not status["can_start"] and not status["can_open"]


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
    assert not status["can_open"]


@pytest.mark.parametrize("process_state", ["STOPPED", "FAILED"])
def test_blocked_start_preserves_only_selected_hand_diagnostic(process_state):
    detail = "串口 /dev/serial/by-id/test-right 不存在；请检查 USB 连接、设备枚举和配置端口。"
    value = snapshot(process_control=dict(enabled=True, groups=[
        group("left", state="FAILED", owned=False, pid=None, detail="unrelated hand detail"),
        group(state=process_state, owned=False, pid=None, detail=detail, log_path="unrelated log path"),
    ]))
    before = deepcopy(value)
    status = inspect_status(value, "right")
    assert status["message"] == detail
    assert status["state"] == "STOPPED"
    assert not status["can_start"] and not status["can_open"]
    assert value == before


@pytest.mark.parametrize("detail", [None, "", "   ", {}, ["unexpected data"], 123])
def test_blocked_start_without_text_detail_keeps_fallback(detail):
    value = snapshot(process_control=dict(enabled=True, groups=[
        group(state="FAILED", owned=False, pid=None, detail=detail),
    ]))
    status = inspect_status(value, "right")
    assert "后台当前不允许启动" in status["message"]
    assert not status["can_start"] and not status["can_open"]


def test_startable_group_does_not_keep_previous_failure_message():
    value = snapshot(process_control=dict(enabled=True, groups=[
        group(state="FAILED", owned=False, pid=None, can_start=True, detail="previous failure"),
    ]))
    status = inspect_status(value, "right")
    assert status["can_start"]
    assert "可通过托管启动" in status["message"]
    assert "previous failure" not in status["message"]


def test_external_publisher_does_not_imply_a_window_exists():
    value = snapshot(process_control=dict(enabled=True, groups=[
        group(state="EXTERNAL", owned=False, pid=None, can_start=True),
    ]))
    status = inspect_status(value, "right")
    assert "窗口关闭后后台仍可能运行" in status["message"]
    assert "请退出该外部程序后重试" in status["message"]
    assert not status["can_start"] and not status["can_open"]


@pytest.mark.parametrize("process_state", ["STARTING", "RUNNING", "DEGRADED"])
def test_running_without_data_does_not_invent_calibration_stage(process_state):
    value = snapshot(process_control=dict(enabled=True, groups=[group(state=process_state)]))
    status = inspect_status(value, "right")
    assert status["state"] == "RUNNING_NO_DATA" and status["can_open"] and not status["can_start"]
    assert "尚无姿态数据" in status["message"]
    assert "检查 IMU 连接与标定状态" in status["message"]


@pytest.mark.parametrize("stream,expected", [
    (device(), "VALID_DATA"), (device(preview=dict(valid=False), valid_hz=0.), "INVALID_DATA"),
    (device(status="stale", age_s=3.), "STALE_DATA"), (device(status="offline", age_s=None), "RUNNING_NO_DATA"),
])
def test_pose_observations_do_not_mix_other_hand_or_tactile(stream, expected):
    value = snapshot(devices=[device("left"), {**device(), "kind": "tactile"}, stream])
    status = inspect_status(value, "right")
    assert status["state"] == expected and status["can_open"]
