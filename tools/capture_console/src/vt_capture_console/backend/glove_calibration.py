"""Read-only managed hand status; presentation is embedded in the workbench."""
from __future__ import annotations

from copy import deepcopy

_HANDS = {"left", "right"}
_ACTIVE_STATES = {"STARTING", "RUNNING", "DEGRADED"}


def glove_group(snapshot, hand):
    """Select by exact typed-stream topic, never by label or row position."""
    if hand not in _HANDS:
        raise ValueError("hand must be left or right")
    groups = (snapshot or {}).get("process_control", {}).get("groups", [])
    expected = f"/gloves/{hand}/state"
    matches = [g for g in groups if g.get("kind") == "glove" and expected in g.get("topics", [])
               and f"/gloves/{'left' if hand == 'right' else 'right'}/state" not in g.get("topics", [])]
    return deepcopy(matches[0]) if len(matches) == 1 else None


def inspect_status(snapshot, hand):
    """Describe process and data observations without sending any commands."""
    group = glove_group(snapshot, hand)
    result = dict(state="UNAVAILABLE", message="后台状态不可确认；不会启动或接管手套进程。",
                  group=group, can_start=False, can_open=False)
    if (not snapshot or snapshot.get("mode") != "live" or not snapshot.get("instance_id")
            or not snapshot.get("process_control", {}).get("enabled")
            or not snapshot.get("capabilities", {}).get("device_control")):
        return result
    if group is None:
        result.update(state="UNCONFIGURED", message="没有唯一匹配此手的姿态进程组；请先配置对应手模块。")
        return result
    state = group.get("state", "UNKNOWN")
    owned_pid = group.get("owned") is True and type(group.get("pid")) is int and group["pid"] > 0
    result["can_open"] = owned_pid and state in _ACTIVE_STATES
    if state == "EXTERNAL":
        result.update(state="EXTERNAL", message="此手已有外部发布程序；窗口关闭后后台仍可能运行。请退出该外部程序后重试。")
    elif state == "DISABLED":
        result.update(state="DISABLED", message="此手姿态模块未启用；请先在配置设备中启用。")
    elif state in {"STOPPED", "FAILED"} and not group.get("owned") and group.get("pid") is None:
        allowed = group.get("can_start") is True
        detail = group.get("detail")
        # Keep the manager's admission diagnostic intact for the existing UI
        # translation boundary. Never stringify arbitrary snapshot fields.
        blocked = detail if isinstance(detail, str) and detail.strip() else "此手进程已停止，但后台当前不允许启动；请核对录制与设备状态。"
        result.update(state="STARTABLE" if allowed else "STOPPED", can_start=allowed,
                      message="此手进程已停止；可通过托管启动连接 IMU，再在预览页标定。" if allowed else blocked)
    elif state == "STOPPING":
        result.update(state="STOPPING", message="此手进程正在停止；等待退出，不会重复启动。")
    elif not owned_pid or state not in _ACTIVE_STATES:
        result.update(state="UNKNOWN", message="此手进程归属或状态待核实；不会重复启动。")
    else:
        devices = [d for d in snapshot.get("devices", []) if d.get("kind") == "glove"
                   and d.get("side") == hand and d.get("topic") == f"/gloves/{hand}/state"]
        device = devices[0] if len(devices) == 1 else None
        if device and device.get("status") == "online":
            if device.get("preview", {}).get("valid") is True:
                result.update(state="VALID_DATA", message="有效手势正在更新；点击“标定”可展开手部面板。")
            else:
                result.update(state="INVALID_DATA", message="已收到状态，当前姿态尚未有效；点击“标定”检查输入和标定状态。")
        elif device and device.get("age_s") is not None:
            result.update(state="STALE_DATA", message="此手进程仍存在，但姿态消息已陈旧；可展开手部面板检查，不会重启采集。")
        else:
            result.update(state="RUNNING_NO_DATA", message="节点已启动，尚无姿态数据。点击“标定”，在手部面板检查 IMU 连接与标定状态。")
    return result


calibration_action_state = inspect_status
