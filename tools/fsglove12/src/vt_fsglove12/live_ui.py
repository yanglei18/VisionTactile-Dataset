"""Explicit-connect Qt/MANO preview with guided pose and touch calibration.

Qt/GL and model imports are reached only by the live CLI. The runtime owns the
device; this module never reads serial, performs fusion, or restores mounting
calibration. Personalized shape loading is an explicit, separate user action.
"""
from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .guided_calibration import STEPS, model_motion_axes
from .hand_view import HandView
from .input_contract import describe_input_contract
from .health_contract import blocked_input_nodes, calibration_input_ready
from .joint_quality import joint_quality_state
from .live_display import (JOINT_LABELS, JointQualityPresentation, StableDisplayWarning,
                           demonstration_root, input_display_snapshot,
                           joint_angle_text, joint_quality_message, measured_pose_visible,
                           unavailable_pose_message)
from .live_process import ProcessLiveController
from .live_runtime import FINGER_NAMES, OFFICIAL_FUNCTIONAL_FINGERS
from .live_shape_ui import ShapeCalibrationPanel, shape_busy, workflow_status_text
from .live_stream import list_serial_ports
from .mano import ManoBackend


_ROLE_NAMES = {"palm": "掌参考", "unmapped_auxiliary": "辅助·未映射",
               "thumb_metacarpal": "拇指·掌骨"}
_GYRO_AXES = ("x", "y", "z")
_GYRO_CONTINUITY_REVISION = "per_axis_gyro_continuity_v2"
_INPUT_SCALE_GUIDANCE = (
    "沿用现有三步：五指伸直，平掌 → 整掌指尖上倾 → 整掌指尖下倾。"
    "无需手长、尺子或精确角度；姿态间尽量在 20 秒内完成，不要只绕竖直轴转动。")
for _finger, _label in (("index", "食指"), ("middle", "中指"),
                        ("ring", "无名"), ("little", "小指"), ("thumb", "拇指")):
    for _part, _suffix in (("proximal", "近"), ("middle", "中"), ("distal", "远")):
        _ROLE_NAMES[f"{_finger}_{_part}"] = f"{_label}·{_suffix}"
_FUNCTIONAL_HAND_PHASES = {
    "preparing": (3., "neutral", "全手校准 1/5：准备 3 秒。五指自然伸直、手掌停稳；随后先静止，再四指一起弯伸，最后只弯伸拇指 IP。全程约 24 秒。",
                  "1/5 全手准备，暂未采集"),
    "collecting_neutral": (2., "neutral", "全手校准 2/5：保持五指伸直静止 2 秒，正在采集中立参考。先不要弯曲，等待“现在四指弯伸”。",
                           "2/5 中立采集，请保持不动"),
    "moving_fingers": (8., "functional_hand_fingers", "全手校准 3/5：现在四指弯伸。食指、中指、无名指、小指同步从伸直到舒适弯曲再伸直，8 秒内慢做 2–3 次；拇指放松，手掌尽量不动。",
                       "3/5 四指同步弯伸，正在采集运动"),
    "preparing_thumb": (3., "neutral", "全手校准 4/5：四指弯伸结束。全手重新伸直，准备 3 秒；拇指近节尽量不动，等待下一段提示后只弯伸末节 IP。此段不采集运动。",
                        "4/5 全手伸直，准备拇指，暂未采集运动"),
    "moving_thumb": (8., "functional_hand_thumb", "全手校准 5/5：现在只弯伸拇指末节 IP，再伸直，8 秒内慢做 2–3 次。拇指近节尽量不动，四指放松伸直；不校正 MCP / CMC，不必用力到极限。",
                     "5/5 拇指 IP 弯伸，正在采集运动"),
}
_STYLE = """
QMainWindow, QWidget#body { background: #f2f5f7; color: #20323b; }
QGroupBox { background: white; border: 1px solid #d5e1e5; border-radius: 8px;
            margin-top: 12px; padding: 12px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
QPushButton { background: white; border: 1px solid #bad1d0; border-radius: 6px; padding: 9px; }
QPushButton:hover { background: #e6f5f0; border-color: #209178; }
QPushButton:disabled { color: #99a6ad; background: #edf1f4; border-color: #dde4e8; }
QPushButton#primary { color: white; background: #187965; border: 0; font-weight: bold; }
QComboBox, QPlainTextEdit { background: white; border: 1px solid #cbdadf; border-radius: 5px; padding: 5px; }
QProgressBar { border: 1px solid #cfdfdf; border-radius: 5px; background: white; text-align: center; min-height: 24px; }
QProgressBar::chunk { background: #57b89d; border-radius: 4px; }
"""


def compact_view_status(text, *, replay=False):
    """Summarize existing display evidence without changing validity or timing.

    Full diagnostics remain in the detail window. The overview deliberately
    omits changing ages/counts and preserves demo, frozen and partial states.
    """
    heading = text.splitlines()[0] if text else ""
    if heading.startswith("正在清除"):
        summary = "正在清除标定"
    elif heading.startswith("动作示意"):
        summary = "动作示意 · 非实时手势"
    elif "标定失效" in text:
        summary = "标定已失效 · 请查看详情"
    elif "渲染" in heading and ("失败" in heading or "冻结" in heading):
        summary = "渲染失败 · 非当前测量"
    elif heading.startswith("已断开"):
        summary = "已断开 · 保留画面非实时"
    elif heading.startswith("输入异常保护"):
        summary = "输入保护 · 画面非实时"
    elif heading.startswith("输入中断"):
        summary = "输入中断 · 保留画面非实时"
    elif heading.startswith("解算延迟"):
        summary = "等待更新 · 保留画面非实时"
    elif heading.startswith("连续显示参考"):
        summary = "部分关节暂估 / 保持 · 非完整有效测量"
    elif heading.startswith("等待"):
        summary = "等待可用姿态"
    elif heading.startswith(("官方 FSGlove", "实时重建", "回放重建")):
        summary = ("回放手势" if replay else "实时手势") + " · 含推算关节"
    else:
        summary = "状态待确认 · 请查看详情"
    if any(marker in text for marker in ("偏轴残差提示", "标定方向需核对", "姿态触及软件限位")):
        summary += " · 有提示"
    return summary


def input_quality_text(frame, now, *, connected):
    """Detailed raw evidence; the main view uses solver_input_status instead."""
    text, current = _input_boundary_quality_text(frame, now, connected=connected)
    protection = input_filter_summary(frame, now, connected=connected)
    if protection:
        text = text.replace("不丢弃或截断数据。", "原始记录不丢弃或截断；解算输入按保护策略处理。")
        text += "\n" + protection
    return text, current


def _gyro_axis_actions(status):
    actions = status.get("gyro_action_by_axis")
    if (isinstance(actions, dict) and set(actions) == set(_GYRO_AXES)
            and all(isinstance(action, str) and action in {"pass", "held", "zero_motion"}
                    for action in actions.values())):
        return actions
    return None


def _integrated_gyro_boundary_axes(status, raw, boundary):
    """Validate the new explicit handling contract; never guess for old data."""
    actions = _gyro_axis_actions(status)
    observed = [axis for axis, value in zip(_GYRO_AXES, raw[:3]) if abs(int(value)) >= boundary]
    integrated = [axis for axis in observed if actions and actions[axis] == "pass"]
    aggregate = ("zero_motion" if actions and "zero_motion" in actions.values() else
                 "held" if actions and "held" in actions.values() else "pass")
    if (status.get("revision") == _GYRO_CONTINUITY_REVISION and actions is not None
            and status.get("gyro_action") == aggregate
            and status.get("gyro_rejected") is (aggregate != "pass")
            and status.get("boundary_is_physical_saturation_proof") is False
            and status.get("gyro_boundary_axes") == observed
            and status.get("gyro_boundary_integrated_axes") == integrated):
        return integrated
    return None


def _axis_count_text(status, key):
    counts = status.get(key)
    if not isinstance(counts, dict):
        return ""
    return " / ".join(f"{axis.upper()} {counts[axis]}" for axis in _GYRO_AXES
                      if type(counts.get(axis)) is int and counts[axis] > 0)


def solver_input_status(frame, now, *, connected):
    """Main-view handling outcome, not an alarm for an already rejected raw value."""
    health = frame.get("input_health") or {}
    blocked = blocked_input_nodes(frame)
    if connected and health.get("enabled") and not health.get("faulted_nodes") and any(
            item.get("state") == "warming_up" for item in health.get("nodes", {}).values()):
        return "输入完整性保护 · 正在确认各通道接收节奏；请稍候再开始标定。", []
    if connected and blocked:
        return ("输入完整性保护 · 检测到数据冻结、接收速率异常或正在恢复："
                + "、".join(node + "号" for node in blocked)
                + "。\n受影响通道暂停姿态积分；当前先恢复输入，不要重复标定。"
                  "原始数据仍完整记录，冻结手形不是新测量。"), blocked
    protection = frame.get("input_filter", {})
    if not protection.get("enabled"):
        return input_quality_text(frame, now, connected=connected)
    count = protection.get("rejected_samples", 0)
    heading = f"异常过滤已启用 · 累计已拦截 {count} 条"
    stamp = frame.get("timestamp_s")
    if not connected:
        return heading + "\n未连接，等待新数据。", []
    if not isinstance(stamp, (int, float)) or not 0 <= now - stamp <= .25:
        return heading + "\n数据已过期，等待更新。", []
    measured = frame.get("nodes", {})
    if not measured:
        return heading + "\n等待新鲜输入。", []
    snapshot = f"快照 #{frame.get('frame_index', '未知')} @ {stamp:.6f}s"
    active, degraded, unverified, integrated_boundary, boundary_history = [], [], [], [], []
    statuses = protection.get("nodes", {})
    boundary = protection.get("settings", {}).get("boundary_lsb", 32760)
    for node, data in sorted(measured.items(), key=lambda item: int(item[0])):
        status = data.get("input_filter") or statuses.get(node, {})
        raw = data.get("raw_values")
        gyro_blocked = status.get("gyro_rejected") is True and status.get("gyro_action") in {"held", "zero_motion"}
        accel_blocked = status.get("accel_rejected") is True and status.get("accel_action") == "skip_accel"
        if (not status or status.get("enabled") is False
                or status.get("gyro_action") not in {"pass", "held", "zero_motion"}
                or status.get("accel_action") not in {"pass", "skip_accel"}
                or not isinstance(raw, (list, tuple)) or len(raw) != 6):
            unverified.append(node)
            continue
        integrated = _integrated_gyro_boundary_axes(status, raw, boundary)
        counts = _axis_count_text(status, "gyro_boundary_samples_by_axis")
        if counts:
            boundary_history.append(f"{node}号({counts})")
        # Do not suppress a genuine mismatch merely because global filtering is on.
        if ((any(abs(int(v)) >= boundary for v in raw[:3]) and integrated is None
             and (not gyro_blocked or status.get("revision") == _GYRO_CONTINUITY_REVISION))
                or (any(abs(int(v)) >= boundary for v in raw[3:]) and not accel_blocked)
                or (status.get("gyro_action") != "pass" and not gyro_blocked)
                or (status.get("accel_action") != "pass" and not accel_blocked)):
            unverified.append(node)
            continue
        if integrated:
            integrated_boundary.append(f"{node}号({'/'.join(axis.upper() for axis in integrated)})")
        if status.get("orientation_valid") is False or data.get("orientation_valid") is False:
            degraded.append(node)
        if gyro_blocked or accel_blocked:
            active.append(node)
    warning_nodes = sorted(set(degraded + unverified), key=int)
    lines = [snapshot, heading]
    if degraded:
        lines.append("输入持续异常或有效参考不足，相关姿态暂不可用：" + "、".join(n + "号" for n in degraded) + "。")
    if unverified:
        lines.append("需要关注：尚未确认这些通道的过滤处理结果：" + "、".join(n + "号" for n in unverified) + "。")
    if active:
        lines.append("异常输入已拦截：" + "、".join(n + "号" for n in active)
                     + "；解算使用保护后的输入，当前保护样本不用于标定。")
    elif not warning_nodes and not integrated_boundary:
        lines.append("当前输入已通过过滤检查，无需拦截。")
    if integrated_boundary:
        lines.append("边界报告值已积分：" + "、".join(integrated_boundary) + "；是否饱和未确认。")
    if boundary_history:
        lines.append("编码边界累计（按轴）：" + "、".join(boundary_history) + "；历史不代表当前故障，是否饱和未确认。")
    lines.append("原始值与历史事件保留在“12 路输入配置 / 实时值”中；累计数不是当前故障。")
    return "\n".join(lines), warning_nodes


def input_filter_node_text(status):
    """Describe solver actions without replacing the separately displayed raw."""
    if not isinstance(status, dict) or not status:
        return "—"
    labels = {"pass": "透传", "held": "保持上次有效输入", "zero_motion": "零运动（偏置回退）"}
    gyro = labels.get(status.get("gyro_action"), "未知")
    actions = _gyro_axis_actions(status)
    if actions:
        groups = {}
        for axis in _GYRO_AXES:
            groups.setdefault(actions[axis], []).append(axis.upper())
        gyro = "；".join("/".join(axes) + " " + labels[action] for action, axes in groups.items())
    accel = {"pass": "透传", "skip_accel": "跳过更新"}.get(status.get("accel_action"), "未知")
    text = f"陀螺：{gyro}\n加速度：{accel}"
    if (status.get("revision") == _GYRO_CONTINUITY_REVISION
            and status.get("boundary_is_physical_saturation_proof") is False and actions):
        integrated = status.get("gyro_boundary_integrated_axes", [])
        if (isinstance(integrated, list) and integrated
                and all(axis in _GYRO_AXES and actions[axis] == "pass" for axis in integrated)):
            text += "\n边界报告值已积分：" + "/".join(axis.upper() for axis in integrated) + "；是否饱和未确认"
    for key, label in (("gyro_boundary_samples_by_axis", "编码边界累计"),
                       ("gyro_held_samples_by_axis", "累计保持"),
                       ("gyro_zero_motion_samples_by_axis", "累计偏置回退")):
        counts = _axis_count_text(status, key)
        if counts:
            text += f"\n{label}：{counts}（历史，非当前故障）"
    if status.get("orientation_valid") is False:
        text += "\n姿态降级：不可用"
    return text


def input_filter_summary(frame, now, *, connected):
    """Fresh per-node protection actions and session totals, never stale motion."""
    protection = frame.get("input_filter", {})
    if not protection.get("enabled"):
        return ""
    total = protection.get("rejected_samples", 0)
    heading = f"解算输入保护：已启用；本会话累计拦截 {total} 条（任一六轴分量被保护的样本）。"
    stamp = frame.get("timestamp_s")
    if not connected:
        return heading + "未连接；不显示当前保护节点。原始记录保持不变。"
    if not isinstance(stamp, (int, float)) or not 0 <= now - stamp <= .25:
        return heading + "数据已过期；等待新鲜保护状态。原始记录保持不变。"
    measured = frame.get("nodes", {})
    if not measured:
        return heading + "等待新鲜原始数据；不判断当前保护节点。原始记录保持不变。"
    statuses = protection.get("nodes", {})
    active, degraded = [], []
    for node, data in sorted(measured.items(), key=lambda item: int(item[0])):
        status = data.get("input_filter") or statuses.get(node, {})
        types = []
        if status.get("gyro_rejected") or status.get("gyro_action") in ("held", "zero_motion"):
            actions = _gyro_axis_actions(status)
            axes = "/".join(axis.upper() for axis in _GYRO_AXES if actions and actions[axis] != "pass")
            types.append("陀螺" + (" " + axes if axes else ""))
        if status.get("accel_rejected") or status.get("accel_action") == "skip_accel":
            types.append("加速度")
        if types:
            active.append(f"{node}号({'/'.join(types)})")
        if status.get("orientation_valid") is False or data.get("orientation_valid") is False:
            degraded.append(f"{node}号")
    current = "当前保护节点：" + "、".join(active) + "。" if active else "当前无拦截动作（不代表姿态准确）。"
    if degraded:
        current += "姿态降级/不可用：" + "、".join(degraded) + "。"
    return heading + current + "原始值和编码边界告警保留；不能恢复被拦截期间的真实运动。"


def _input_boundary_quality_text(frame, now, *, connected):
    """Current-frame highlights, explicitly separated from historical events."""
    rail = frame.get("sensor_quality", {}).get("gyro_near_rail")
    if not connected:
        return "编码边界观察：未连接；等待新鲜数据。硬件饱和未确认。", []
    if not rail:
        return "编码边界观察：等待统计；硬件饱和未确认。", []
    nodes = rail.get("nodes", {})
    total = sum(item.get("near_rail_samples", 0) for item in nodes.values())
    stamp = frame.get("timestamp_s")
    if not isinstance(stamp, (int, float)) or not 0 <= now - stamp <= .25:
        return f"编码边界观察：数据已过期，等待更新；累计 {total} 条。硬件饱和未确认。", []
    # Fusion can publish a fresh snapshot even while the source is silent.
    # An empty set of fresh raw-derived nodes is not a clean-input result.
    if not frame.get("nodes"):
        return f"编码边界观察：等待新鲜原始数据；本会话累计 {total} 条。硬件饱和未确认。", []
    threshold = rail.get("threshold_lsb", 32760)
    current, current_labels = [], []
    raw_available = False
    for node, data in sorted(frame["nodes"].items(), key=lambda item: int(item[0])):
        raw = data.get("raw_values")
        if not isinstance(raw, (list, tuple)) or len(raw) != 6:
            continue
        raw_available = True
        axes = [axis for axis, value in zip("XYZ", raw[:3]) if abs(int(value)) >= threshold]
        if axes:
            current.append(node)
            current_labels.append(f"{node}号({'/'.join(axes)}) #{data.get('sample_index', '未知')} "
                                  + " / ".join(map(str, raw[:3])))
    frame_label = f"快照 #{frame.get('frame_index', '未知')} @ {stamp:.6f}s"
    if current:
        current_text = "当前帧编码边界提示：" + "；".join(current_labels)
    else:
        current_text = "当前原始值未命中编码边界" if raw_available else "当前原始值不可用，不能判定当前命中"
    active_history = list(rail.get("active_nodes", []))
    if active_history:
        labels = []
        for node in active_history:
            entry = nodes.get(node, {})
            axes = "/".join(str(axis).upper() for axis in entry.get("axes", []))
            prefix = "至少 " if entry.get("recent_counts_complete") is False else ""
            labels.append(f"{node}号({axes}) {prefix}{entry.get('recent_near_rail_samples', 0)}条")
        history_text = (f"历史事件（近 {rail.get('recent_window_s', 2):g} 秒，不是当前读数）："
                        + "、".join(labels) + "。触发原值、时间、样本号见输入详情。")
    else:
        history_text = f"近 {rail.get('recent_window_s', 2):g} 秒无编码边界事件；本会话累计 {total} 条。"
    return (f"{frame_label}\n{current_text}（判定阈值 |raw| ≥ {threshold}）。\n{history_text}\n"
            "硬件饱和未确认；不丢弃或截断数据。无此事件不代表无漂移或姿态准确。"), current


def input_boundary_event_text(entry, now):
    """Exact saved trigger data; never infer a historical peak from a count."""
    event = entry.get("last_boundary_event")
    if not isinstance(event, dict):
        return "旧记录未保存触发原值" if entry.get("near_rail_samples", 0) else "无"
    stamp, raw = event.get("host_monotonic_s"), event.get("gyro_lsb")
    if (not isinstance(stamp, (int, float)) or not np.isfinite(stamp)
            or not isinstance(raw, (list, tuple)) or len(raw) != 3):
        return "事件证据不完整"
    age = now - stamp
    age_text = f"{age * 1000:.0f} ms 前" if age >= 0 else "时间待核对"
    return (f"#{event.get('sample_index', '未知')} @ {stamp:.6f}s\n"
            + " / ".join(map(str, raw)) + f"\n{age_text} · 历史，不是当前值")


def input_contract_summary(contract):
    """Explain software assumptions separately from hardware measurements."""
    timing, verification = contract["sample_timing"], contract["verification"]
    units = "配置声明已核实" if verification["units_verified"] else "未核实"
    rate = "配置声明已核实" if verification["sample_rate_verified"] else "未核实"
    model_source = {"user_reported": "用户提供，不是设备自动识别",
                    "firmware_documented": "配置声明来源为固件文档，本工具未自动识别",
                    "device_identified": "配置声明曾识别设备，本工具未进行读回"}.get(
                        contract.get("model_source"), "配置未声明，不推断旧记录的传感器型号")
    return (f"传感器型号：{contract['sensor_model']}（{model_source}）。\n"
            "当前软件解释：STM32 USB CDC；有符号小端 16 位；字段 gx / gy / gz / ax / ay / az。"
            "第二字节含义未知；结构可解码不等于固件协议已验证。\n"
            f"单位系数：{units}。名义逐路采样率：{timing['nominal_rate_hz']:g} Hz（{rate}）；"
            f"VQF 固定积分步长：{timing['integration_period_s'] * 1000:g} ms。\n"
            "物理量程：未读回，不能仅由换算系数确认。接收 Hz 来自主机接收时间，"
            "不是芯片 ODR；回放时也不是硬件测量。\n"
            "六轴 IMU 没有绝对航向观测；各路独立 VQF 的相对航向漂移，不能靠三姿势标定持续消除。"
            "静止与偏置是算法估计，不是硬件真值。\n"
            "下表来自本次运行配置与当前快照。只读查看，不修改参数、不读取设备寄存器。")


def input_fusion_text(state):
    """Human-readable VQF estimates, not independent motion/bias measurements."""
    if not isinstance(state, dict) or not state:
        return "—"
    rest = state.get("rest_detected")
    label = "判为静止" if rest is True else "未判静止" if rest is False else "静止未知"
    bias = state.get("gyro_bias_rad_s")
    if (isinstance(bias, (list, tuple)) and len(bias) == 3
            and all(isinstance(value, (int, float)) and not isinstance(value, bool)
                    and np.isfinite(value) for value in bias)):
        bias_text = " / ".join(f"{np.rad2deg(value):+.3f}" for value in bias)
        return f"{label}\n偏置 {bias_text} °/s"
    return f"{label}\n偏置未知"


def input_contract_rows(contract, frame, now, *, connected):
    """Twelve configured channels plus fresh values, never fabricated readback."""
    stamp = frame.get("timestamp_s")
    fresh_frame = (connected and isinstance(stamp, (float, int))
                   and 0 <= now - stamp <= .25)
    measured = frame.get("nodes", {}) if fresh_frame else {}
    observed = frame.get("input_observation", {}).get("nodes", {}) if fresh_frame else {}
    rows = []
    for node in range(1, 13):
        key = str(node)
        item = contract["nodes"][key]
        role = _ROLE_NAMES.get(item["role"], item["role"])
        data = measured.get(key, {})
        rate = observed.get(key, {}).get("observed_receive_rate_hz") if data else None
        rate_text = f"{rate:.2f}" if isinstance(rate, (int, float)) and np.isfinite(rate) else "—"
        raw = data.get("raw_values")
        gyro = accel = "—"
        if isinstance(raw, (list, tuple)) and len(raw) == 6:
            gyro, accel = " / ".join(map(str, raw[:3])), " / ".join(map(str, raw[3:]))
        rail_entry = frame.get("sensor_quality", {}).get("gyro_near_rail", {}).get("nodes", {}).get(key, {})
        protection = frame.get("input_filter", {})
        protection_status = (data.get("input_filter") or protection.get("nodes", {}).get(key, {})) if data else {}
        protection_text = (input_filter_node_text(protection_status) if protection.get("enabled") else "未启用") if data else "—"
        rows.append([f"{node:02d}", role, f"{item['gyro_dps_per_lsb']:g}",
                     f"{item['accel_g_per_lsb']:g}", "未读回 / 未确认", rate_text, gyro, accel,
                     input_fusion_text(data.get("fusion_state")),
                     str(data.get("sample_index", "—")),
                     input_boundary_event_text(rail_entry, now) if fresh_frame else "—",
                     protection_text])
    return rows


def input_scale_check_text(check, *, detailed=False):
    """Report backend evidence only; never fit or accept a scale in the GUI."""
    check = check if isinstance(check, dict) else {}
    labels = {"waiting": "等待三姿态采集", "collecting": "采集中", "running": "检查中",
              "completed": "检查完成 · 尚未采用", "failed": "检查未完成",
              "applied": "候选已采用 · 请重新连接并标定"}
    lines = [labels.get(check.get("status", "waiting"), "检查状态待确认")]
    names = (("supported_nodes", "证据支持"), ("recommended_nodes", "建议调整"),
             ("inconclusive_nodes", "证据不足"))
    counts = [f"{label} {len(check.get(key) or [])} 路" for key, label in names]
    if check.get("status") in {"completed", "applied"}:
        lines.append(" · ".join(counts))
    if check.get("message"):
        lines.append(str(check["message"]) if detailed else str(check["message"]).splitlines()[0])
    if not detailed:
        return "\n".join(lines)
    lines.extend(["", _INPUT_SCALE_GUIDANCE,
        "检查只比较本次原始输入与重力变化的一致性，不是芯片寄存器/物理量程读回，也不是人体精度验收。",
        "证据不足的通道不得视为已通过；不会自动采用候选或自动重新连接。", ""])
    for node, item in sorted(check.get("nodes", {}).items(), key=lambda pair: str(pair[0]).zfill(3)):
        lines.append(f"节点 {node}：{item.get('status', '未知')}")
        for key, label in (("nominal_gyro_dps_per_lsb", "当前系数"),
                           ("candidate_gyro_dps_per_lsb", "候选系数")):
            value = item.get(key)
            if isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value):
                lines.append(f"  {label}：{value:g} °/s/LSB")
        reasons = item.get("reasons") or []
        lines.extend("  " + str(reason) for reason in (reasons if isinstance(reasons, list) else [reasons]))
        if item.get("reason"):
            lines.append("  原因：" + str(item['reason']))
        if item.get("reason_codes"):
            lines.append("  诊断代码：" + str(item['reason_codes']))
    for key, label in (("config_sha256", "检查所用配置 SHA256"),
                       ("report_path", "报告文件"), ("candidate_path", "候选配置")):
        if check.get(key):
            lines.append(f"{label}：{check[key]}")
    # Preserve any additional backend acceptance diagnostics without inventing
    # UI conclusions from them; the full report is also available on disk.
    lines.extend(["", "后台结果（只读）：", json.dumps(check, ensure_ascii=False, indent=2)])
    return "\n".join(lines)


class LiveWindow(QtWidgets.QMainWindow):
    def __init__(self, config, backend, *, output_root, port="", replay_raw=None,
                 replay_speed=1., controller=None, upstream_checkout=None, ros_socket=None,
                 tactile_socket=None, tactile_layout=None):
        super().__init__()
        self.hand = config["hand"]
        if getattr(backend, "hand", self.hand) != self.hand:
            raise ValueError("MANO backend hand does not match configuration hand")
        hand_label = "右手" if self.hand == "right" else "左手"
        self.official = upstream_checkout is not None
        self._steps = tuple({**step, "instruction": step["instruction"].replace("右手", hand_label)} for step in STEPS) if not self.official else (
            {"key": "neutral", "title": "1. 官方基准 · 平掌", "instruction": f"{hand_label}掌心朝下、五指伸直；保持静止 2 秒。这是官方 base_pose。"},
            {"key": "fingers_down", "title": "2. 官方轴标定 · 指尖上倾", "instruction": "五指保持伸直，转动整掌使指尖向上倾约 50°，停稳 2 秒；不要弯曲手指或同时侧翻，不要勉强到疼痛。"},
            {"key": "thumb_up", "title": "3. 官方轴标定 · 指尖下倾", "instruction": "五指保持伸直，沿同一手腕转轴使指尖向下倾约 50°，停稳 2 秒；不是拇指侧上抬。上下总角差建议至少 90°，不要勉强到疼痛。"},
        )
        self.setWindowTitle(("FSGlove 官方核心 · 12 路适配" if self.official else
                            "12 路 IMU → MANO · 旧实验求解器") + f" · {hand_label}" + (" · ROS 2 结果发布" if ros_socket else ""))
        self.resize(1220, 860)
        self.backend = backend
        self.config = deepcopy(config)
        self.rest = backend.export_rest()
        self.neutral = backend.render(np.zeros((16, 3)))
        self.axes = model_motion_axes(self.neutral["joints_m"], hand=self.hand)
        self.replay_raw = replay_raw
        self.controller = controller if controller is not None else ProcessLiveController(
            config, self.rest, self.axes, output_root=output_root,
            mano_assets=backend.assets_root, hand=backend.hand,
            replay_raw=replay_raw, replay_speed=replay_speed, upstream_checkout=upstream_checkout,
            ros_socket=ros_socket)
        self.input_contract = describe_input_contract(getattr(self.controller, "config", config))
        self._gyro_scale_text = "本窗口陀螺换算：" + " / ".join(
            f"{node}号={self.input_contract['nodes'][node]['gyro_dps_per_lsb']:g}"
            for node in ("5", "6")) + " °/s/LSB；不是固件量程读回。其余通道见“12 路输入配置 / 实时值”。"
        self.input_contract_dialog = None
        self.input_scale_dialog = None
        self._input_scale_details_snapshot = None
        self._input_scale_apply_pending_generation = None
        self._local_recording_warning = ""
        # Instructions are static. Compute them before acquisition starts, not
        # inside a paint timer while the numerical solver is running.
        self._demonstrations = {step["key"]: (
            self.neutral if step["key"] == "neutral" else backend.render(
                np.zeros((16, 3)), demonstration_root(step["key"], self.axes))) for step in STEPS}
        if self.official:
            from scipy.spatial.transform import Rotation
            for key, degrees in (("fingers_down", -50.), ("thumb_up", 50.)):
                side = 1 if self.hand == "right" else -1
                q = Rotation.from_rotvec(side * np.asarray(self.axes["pitch_axis"]) * np.deg2rad(degrees)).as_quat()
                self._demonstrations[key] = backend.render(np.zeros((16, 3)), q)
        functional_slots = dict(thumb=(14, 15), little=(7, 8))
        if self.official:
            functional_slots.update(index=(1, 2), middle=(4, 5), ring=(10, 11))
        for finger, slots in functional_slots.items():
            angles = np.zeros((16, 3))
            angles[slots[0], 2], angles[slots[1], 2] = .6, .8
            if finger != "thumb":
                angles[slots[1] + 1, 2] = .52
            elif self.official:
                angles[14, 2] = 0.  # Official thumb correction is IP-only.
            self._demonstrations["functional_" + finger] = backend.render(angles)
        if self.official:
            angles = np.zeros((16, 3))
            for mcp, pip in ((1, 2), (4, 5), (7, 8), (10, 11)):
                angles[mcp, 2], angles[pip, 2], angles[pip + 1, 2] = .6, .8, .52
            self._demonstrations["functional_hand_fingers"] = backend.render(angles)
            self._demonstrations["functional_hand_thumb"] = self._demonstrations["functional_thumb"]
        self._last_pose = None
        self._joint_quality_presentation = JointQualityPresentation()
        self._display_warning = StableDisplayWarning()
        self._has_display_cache = False
        self._display_source_timestamp = None
        self._display_metadata = {}
        self._display_updated_at = None
        self._display_session = None
        self._display_profile = None
        self._input_contract_refresh_at = None
        self._demo_key = None
        self._step = "neutral"
        self._render_error = ""
        self._render_times = []
        self._refresh_ms = 0.
        self._pose_age_ms = None
        self._ui_values = {}
        self._clear_pending = False
        self._clear_generation_before = 0
        self._clear_session = None
        self._empty_calibration_marker = None
        self._capture_session = None
        self._capture_context = None
        self._capture_completion = None
        self._single_capture_feedback = None
        self.visual_details_dialog = None
        self._closing = False
        self.tactile_controller = None
        self.tactile_timer = None
        self._tactile_socket = tactile_socket
        self._tactile_pose_current = False
        self._build_ui(port)
        if tactile_socket:
            try:
                # The optional pure-Python transport keeps rclpy and ROS library
                # paths out of this GUI's independent numerical environment.
                from .tactile_overlay import TactileOverlayController
                from .tactile_surface import TactileSurfaceBinding
                if replay_raw:
                    raise ValueError("历史姿态回放不能与实时触觉混合；请使用实时手势窗口")
                self.view.tactile_binding = TactileSurfaceBinding(
                    self.neutral, self.axes, hand=self.hand, layout_path=tactile_layout)
                mapping = self.view.tactile_binding.metadata()
                anchors = mapping.get("observed_anchor_count", 0)
                self.tactile_mapping_note.setText(
                    "蓝=负变化 · 本色=±200 · 黄→红=正变化\n"
                    f"ΔADC 不是压力；当前手别映射有 {anchors} 个已核对锚点，其余位置为候选。")
                self.tactile_controller = TactileOverlayController(
                    tactile_socket, expected_hand=self.hand, layout_path=tactile_layout)
                self.tactile_enabled.setEnabled(True)
                self.tactile_enabled.setChecked(True)
                self.tactile_reference.setEnabled(True)
                self.tactile_timer = QtCore.QTimer(self)
                self.tactile_timer.timeout.connect(self.refresh_tactile)
                self.tactile_timer.start(33)
            except (ImportError, OSError, ValueError, RuntimeError) as exc:
                self.tactile_status.setText("触觉叠加未启动（不影响手势）：" + str(exc))
        self.controller.start()
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.timeout.connect(self.refresh)
        # Poll latest-only output faster than its 30 Hz production cadence.
        # This does not generate or interpolate extra poses; it reduces phase
        # delay and missed updates between two independent 30 Hz clocks.
        self.timer.start(16)
        self.refresh()

    @staticmethod
    def _label(text="", *, wrap=True):
        label = QtWidgets.QLabel(text)
        label.setWordWrap(wrap)
        label.setTextFormat(QtCore.Qt.PlainText)
        return label

    def _build_ui(self, port):
        body = QtWidgets.QWidget(objectName="body")
        self.setCentralWidget(body)
        outer = QtWidgets.QVBoxLayout(body)
        title = self._label("12 IMU → MANO    " + ("右手" if self.hand == "right" else "左手"))
        title.setStyleSheet("font-size: 22px; font-weight: 700;")
        outer.addWidget(title)
        mode = "CSV 回放模式：非实时硬件" if self.replay_raw else "独立实时原型 · 手动连接 · 不沿用旧安装标定"
        mode += (" · 官方 Raw2MANO 原始核心 / 12 路显式缺测适配" if self.official else
                 " · 旧版自写实验算法（非官方 FSGlove）")
        if self.hand == "left" and not self.config.get("mounting_confirmed", False):
            mode += " · 左手候选映射未实测：请确认每路安装及输入单位"
        self.mode = self._label(mode)
        outer.addWidget(self.mode)
        row = QtWidgets.QHBoxLayout()
        self.ports = QtWidgets.QComboBox()
        self.ports.setEditable(True)
        self.ports.setMinimumWidth(420)
        self.scan = QtWidgets.QPushButton("刷新串口")
        self.scan.clicked.connect(self.refresh_ports)
        self.connect_button = QtWidgets.QPushButton("开始回放" if self.replay_raw else "连接")
        self.connect_button.setObjectName("primary")
        self.connect_button.clicked.connect(self.toggle_connection)
        row.addWidget(self.ports, 1)
        row.addWidget(self.scan)
        row.addWidget(self.connect_button)
        outer.addLayout(row)

        split = QtWidgets.QHBoxLayout()
        left = QtWidgets.QVBoxLayout()
        sensors = QtWidgets.QGroupBox("12 路数据 · 绿=新鲜 · 蓝=已过滤 · 橙=需关注")
        grid = QtWidgets.QGridLayout(sensors)
        self.badges = {}
        for node in range(1, 13):
            role = self.input_contract["nodes"][str(node)]["role"]
            badge = self._label(f"{node:02d}  {_ROLE_NAMES.get(role, role)}")
            self.badges[node] = badge
            grid.addWidget(badge, (node - 1) // 3, (node - 1) % 3)
        left.addWidget(sensors)
        self.input_contract_button = QtWidgets.QPushButton("12 路输入配置 / 实时值")
        self.input_contract_button.setToolTip("查看每路映射、换算系数、接收频率和原始六轴值；只读，不向设备发送命令。")
        self.input_contract_button.clicked.connect(self.show_input_contract)
        left.addWidget(self.input_contract_button)
        tactile_group = QtWidgets.QGroupBox("3D 手表面触觉 · 只读显示")
        tactile_layout = QtWidgets.QVBoxLayout(tactile_group)
        tactile_row = QtWidgets.QHBoxLayout()
        self.tactile_enabled = QtWidgets.QCheckBox("叠加触觉")
        self.tactile_enabled.setEnabled(False)
        self.tactile_enabled.toggled.connect(self.refresh_tactile)
        tactile_row.addWidget(self.tactile_enabled)
        self.tactile_range = QtWidgets.QComboBox()
        self.tactile_range.addItem("轻触 ±3000 ADC", 3000.)
        self.tactile_range.addItem("强触 ±16000 ADC", 16000.)
        self.tactile_range.currentIndexChanged.connect(self.refresh_tactile)
        tactile_row.addWidget(self.tactile_range)
        tactile_layout.addLayout(tactile_row)
        self.tactile_reference = QtWidgets.QPushButton("采集触觉无载参考（3 秒）")
        self.tactile_reference.setEnabled(False)
        self.tactile_reference.setToolTip("松开全部触觉区域；仅设置显示用 ADC 零点，不清除或改变 IMU 标定。")
        self.tactile_reference.clicked.connect(self.start_tactile_reference)
        tactile_layout.addWidget(self.tactile_reference)
        mapping_note = ("左手 3D 需显式左手映射；原始二维触觉矩阵可单独使用。" if self.hand == "left" else
                        "映射核对信息在启用叠加后显示；其余位置为候选。")
        self.tactile_mapping_note = self._label("蓝=负变化 · 本色=±200 · 黄→红=正变化\nΔADC 不是压力；" + mapping_note)
        tactile_layout.addWidget(self.tactile_mapping_note)
        self.tactile_status = self._label("未启用；ROS 联合启动增加 tactile_overlay:=true。")
        self.tactile_status.setMinimumHeight(54)
        tactile_layout.addWidget(self.tactile_status)
        left.addWidget(tactile_group)
        self.workflow_panel = QtWidgets.QGroupBox("标定流程 / 尚未解决的项目")
        workflow_layout = QtWidgets.QVBoxLayout(self.workflow_panel)
        self.workflow_status = self._label()
        workflow_layout.addWidget(self.workflow_status)
        self.shape_navigation_button = QtWidgets.QPushButton("前往个性化手形 / 触指标定")
        self.shape_navigation_button.setToolTip("滚动到触指标定面板；不会连接、采集或采用任何手形。")
        self.shape_navigation_button.clicked.connect(
            lambda: self.sidebar_scroll.ensureWidgetVisible(self.shape_panel))
        workflow_layout.addWidget(self.shape_navigation_button)
        left.addWidget(self.workflow_panel)
        if not self.official:
            self.workflow_panel.hide()
        guide = QtWidgets.QGroupBox("三步标定 · 每一步可单独重采")
        steps = QtWidgets.QVBoxLayout(guide)
        self.step_buttons = {}
        for item in self._steps:
            button = QtWidgets.QPushButton(item["title"] + "  ·  采集")
            button.clicked.connect(lambda checked=False, key=item["key"]: self.capture(key))
            self.step_buttons[item["key"]] = button
            steps.addWidget(button)
        self.instructions = self._label(self._steps[0]["instruction"])
        self.instructions.setMinimumHeight(80)
        self.instructions.setStyleSheet("font-size: 14px; color: #254e52;")
        steps.addWidget(self.instructions)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setValue(0)
        self.progress.setFormat("点击步骤后：3 秒准备 → 2 秒保持采集")
        steps.addWidget(self.progress)
        self.cancel_capture = QtWidgets.QPushButton("取消当前采集")
        self.cancel_capture.clicked.connect(lambda: self.send("cancel_capture"))
        steps.addWidget(self.cancel_capture)
        self.input_scale_panel = QtWidgets.QGroupBox("三姿态输入尺度检查 · 自动随采集执行")
        scale_layout = QtWidgets.QVBoxLayout(self.input_scale_panel)
        self.input_scale_summary = self._label()
        scale_layout.addWidget(self.input_scale_summary)
        self.input_scale_guidance = self._label(_INPUT_SCALE_GUIDANCE)
        scale_layout.addWidget(self.input_scale_guidance)
        scale_actions = QtWidgets.QHBoxLayout()
        self.input_scale_details_button = QtWidgets.QPushButton("查看逐路结果")
        self.input_scale_details_button.clicked.connect(self.show_input_scale_check)
        scale_actions.addWidget(self.input_scale_details_button)
        self.input_scale_report_button = QtWidgets.QPushButton("打开报告目录")
        self.input_scale_report_button.clicked.connect(self.open_input_scale_report_directory)
        scale_actions.addWidget(self.input_scale_report_button)
        scale_layout.addLayout(scale_actions)
        self.input_scale_apply_button = QtWidgets.QPushButton("采用候选并断开（重新连接标定）")
        self.input_scale_apply_button.setToolTip("仅显式采用后台验证通过的候选；正常断开当前采集后生效，不自动连接。旧姿态标定不可沿用。")
        self.input_scale_apply_button.clicked.connect(self.use_input_scale_candidate)
        scale_layout.addWidget(self.input_scale_apply_button)
        steps.addWidget(self.input_scale_panel)
        if not self.official:
            self.input_scale_panel.hide()
        self.reanchor = QtWidgets.QPushButton("仅重设伸直参考（保留安装轴）")
        self.reanchor.setToolTip("三步标定后使用：五指自然伸直，3 秒准备、2 秒保持；不重启滤波器、不重置手掌朝向，也不校正持续漂移。")
        self.reanchor.clicked.connect(lambda: self.send("reanchor_neutral"))
        steps.addWidget(self.reanchor)
        functional_row = QtWidgets.QHBoxLayout()
        self.functional_buttons = {}
        for finger, name in (("thumb", "拇指"), ("little", "小指")):
            button = QtWidgets.QPushButton(name + "弯曲方向校准")
            button.setToolTip("先张手保持，再按提示只弯伸这根手指；学习运动轴与正方向，不自动拉伸角度或认定人体极限。")
            button.clicked.connect(lambda checked=False, value=finger: self.send("capture_functional", finger=value))
            self.functional_buttons[finger] = button
            functional_row.addWidget(button)
        steps.addLayout(functional_row)
        if self.official:
            self.reanchor.hide()
            for button in self.functional_buttons.values():
                button.hide()
        self.functional_hand_capture = QtWidgets.QPushButton("一键全手弯伸校准")
        self.functional_hand_capture.setToolTip(
            "先完成官方三姿势。3 秒准备 + 2 秒静止 + 8 秒四指同步弯伸 + 3 秒伸直准备 + 8 秒拇指 IP，共 24 秒。可取消；结果逐指保存，不保证每指都通过。")
        self.functional_hand_capture.clicked.connect(lambda: self.send("capture_functional_hand"))
        steps.addWidget(self.functional_hand_capture)
        self.functional_hand_feedback = self._label()
        steps.addWidget(self.functional_hand_feedback)
        self.functional_selector = QtWidgets.QComboBox()
        for finger in OFFICIAL_FUNCTIONAL_FINGERS:
            self.functional_selector.addItem("拇指 IP（不改 MCP/CMC）" if finger == "thumb" else FINGER_NAMES[finger], finger)
        self.functional_capture = QtWidgets.QPushButton("校准所选手指")
        self.functional_capture.setToolTip("3 秒准备 → 2 秒伸直静止 → 8 秒慢弯伸。失败或取消保留原结果；不是人体精度标定。")
        self.functional_capture.clicked.connect(lambda: self.send("capture_functional", finger=self.functional_selector.currentData()))
        official_functional_row = QtWidgets.QHBoxLayout()
        official_functional_row.addWidget(self.functional_selector, 1)
        official_functional_row.addWidget(self.functional_capture)
        steps.addLayout(official_functional_row)
        self.functional_enabled = QtWidgets.QCheckBox("启用单指轴校正 + 关节约束（非实测精度）")
        self.functional_enabled.setToolTip("关闭以对比官方原始输出；不删除已采参数、不修改官方基础标定，也不自动重采。")
        self.functional_enabled.toggled.connect(lambda enabled: self.send("set_functional_enabled", enabled=bool(enabled)))
        steps.addWidget(self.functional_enabled)
        if not self.official:
            self.functional_hand_capture.hide()
            self.functional_hand_feedback.hide()
            self.functional_selector.hide()
            self.functional_capture.hide()
            self.functional_enabled.hide()
        self.clear = QtWidgets.QPushButton("清除本次标定 / 重新开始")
        self.clear.clicked.connect(self.clear_calibration)
        steps.addWidget(self.clear)
        left.addWidget(guide)
        self.shape_panel = ShapeCalibrationPanel(self)
        left.addWidget(self.shape_panel)
        self.status = self._label()
        self.status.setMinimumHeight(65)
        left.addWidget(self.status)
        self.local_recording_status = self._label()
        self.local_recording_status.setStyleSheet("color: #8a4e00;")
        self.local_recording_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.local_recording_status.hide()
        left.addWidget(self.local_recording_status)
        self.details = QtWidgets.QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setMaximumHeight(120)
        self.details.setPlaceholderText("标定反馈与局部缺测说明")
        left.addWidget(self.details)
        left.addStretch()
        left_widget = QtWidgets.QWidget()
        left_widget.setLayout(left)
        left_widget.setFixedWidth(385)
        # Larger desktop fonts and long instructions must not force a window
        # taller than the screen. Only the controls scroll; the hand stays put.
        self.sidebar_scroll = QtWidgets.QScrollArea()
        self.sidebar_scroll.setWidgetResizable(True)
        self.sidebar_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.sidebar_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.sidebar_scroll.setFixedWidth(405)
        self.sidebar_scroll.setWidget(left_widget)
        split.addWidget(self.sidebar_scroll)
        right = QtWidgets.QVBoxLayout()
        # Keep long and changing diagnostics out of the 3D layout. They remain
        # live/readable in an optional nonmodal detail window, not discarded.
        self.visual_details_content = QtWidgets.QWidget(self)
        visual_details_layout = QtWidgets.QVBoxLayout(self.visual_details_content)
        visual_details_layout.addWidget(self._label("画面状态"))
        self.view_state = self._label()
        self.view_state.setStyleSheet("background: #fff3d9; padding: 10px; font-weight: bold;")
        self.view_state.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        visual_details_layout.addWidget(self.view_state)
        visual_details_layout.addWidget(self._label("输入状态"))
        self.input_status = self._label()
        self.input_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        visual_details_layout.addWidget(self.input_status)
        visual_details_layout.addStretch()
        self.visual_details_content.hide()
        self.visual_status_bar = QtWidgets.QWidget()
        self.visual_status_bar.setFixedHeight(40)
        compact_layout = QtWidgets.QHBoxLayout(self.visual_status_bar)
        compact_layout.setContentsMargins(0, 0, 0, 0)
        self.visual_status = self._label(wrap=False)
        self.visual_status.setMinimumWidth(0)
        self.visual_status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        compact_layout.addWidget(self.visual_status, 1)
        self.visual_details_button = QtWidgets.QPushButton("状态详情")
        self.visual_details_button.setToolTip("查看完整画面与输入诊断；不影响采集、标定或 ROS 输出。")
        self.visual_details_button.clicked.connect(self.show_visual_details)
        compact_layout.addWidget(self.visual_details_button)
        right.addWidget(self.visual_status_bar)
        self.shape_view_status = self._label(wrap=False)
        self.shape_view_status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.shape_view_status.setStyleSheet("color: #52636c; background: #f2f5f7; padding: 3px;")
        if not self.official:
            self.shape_view_status.hide()
        self.view = HandView(self.neutral, self.axes)
        # A compact in-view source label preserves the existing 1080p minimum
        # height; additional wizard controls live only in the scrollable side.
        shape_overlay = QtWidgets.QVBoxLayout(self.view)
        shape_overlay.setContentsMargins(8, 8, 8, 8)
        shape_overlay.addWidget(self.shape_view_status, 0, QtCore.Qt.AlignTop)
        shape_overlay.addStretch()
        right.addWidget(self.view, 1)
        view_controls = QtWidgets.QHBoxLayout()
        for text, palmar in (("看手背", False), ("看掌面", True)):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda checked=False, palm=palmar: self.view.reset_camera(palm))
            view_controls.addWidget(button)
        joints = QtWidgets.QCheckBox("关节透视标记")
        joints.setChecked(True)
        joints.toggled.connect(self.toggle_joints)
        view_controls.addWidget(joints)
        right.addLayout(view_controls)
        right.addWidget(self._label("浅青 = 掌面    深青 = 手背    鼠标拖动旋转 / 滚轮缩放"))
        self.joint_status = self._label("关节角与软件限位：完成标定后显示（不是人体极限测量）。")
        self.joint_status.setMinimumHeight(65)
        right.addWidget(self.joint_status)
        self.stats = self._label()
        right.addWidget(self.stats)
        split.addLayout(right, 1)
        outer.addLayout(split, 1)
        self.output = self._label("连接后创建独立记录目录；不覆盖旧文件。")
        self.output.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        outer.addWidget(self.output)
        outer.addWidget(self._label("边界：远节含推断，拇指第一掌骨未测；六轴相对航向漂移尚未校正。画面流畅不代表人体精度已验收。"))
        self.setStyleSheet(_STYLE)
        if self.replay_raw:
            self.ports.setCurrentText(str(Path(self.replay_raw).resolve()))
            self.ports.setEnabled(False)
            self.scan.setEnabled(False)
        else:
            self.refresh_ports()
            if port:
                self.ports.setCurrentText(port)

    def start_tactile_reference(self):
        if self.tactile_controller is not None:
            self.tactile_controller.poll()
            self.tactile_controller.start_reference()
            self.refresh_tactile()

    def refresh_tactile(self, *_):
        """Bounded read-only 30 Hz display path; no solver or serial commands."""
        if self.tactile_controller is None:
            return
        try:
            self.tactile_controller.poll()
            collecting = self.tactile_controller.baseline.collecting
            self.tactile_reference.setEnabled(not collecting)
            self._set_changed(self.tactile_reference, "setText",
                "正在采集：全部触觉区域保持无载…" if collecting else "采集触觉无载参考（3 秒）")
            frame = self.tactile_controller.frame(
                self._display_source_timestamp,
                pose_current=self._tactile_pose_current,
                metadata=self._display_metadata)
            enabled = self.tactile_enabled.isChecked()
            self.view.set_tactile(frame.delta if enabled else None,
                                  frame.valid_mask if enabled else None,
                                  limit=float(self.tactile_range.currentData()))
            text = frame.status if enabled else "叠加已关闭；触觉采集 / ROS / 手势解算继续。"
            receiver = getattr(self.tactile_controller, "receiver", None)
            if enabled and receiver is not None and receiver.received == 0:
                text = "等待 ROS 触觉输入；请先启动触觉采集和显示桥接，不需要清除 IMU 标定。"
            if frame.delta is not None and enabled:
                binding = self.view.tactile_binding
                addresses = binding.addresses
                active = frame.valid_mask[addresses[:, 0], addresses[:, 1]] & binding.valid
                text += f"\n有效表面点 {int(active.sum())}/{len(addresses)}（候选位置）"
                text += f" · 源年龄 {frame.age_ms:.0f} ms · 姿态差 {frame.skew_ms:.0f} ms"
            self._set_changed(self.tactile_status, "setText", text)
        except (OSError, ValueError, RuntimeError) as exc:
            # A failed optional display must not stop the pose event loop or
            # leave a previously red surface pretending to be current contact.
            self.view.set_tactile()
            self._set_changed(self.tactile_status, "setText", "触觉叠加暂不可用（手势继续）：" + str(exc))

    def refresh_ports(self):
        previous = self.ports.currentText()
        self.ports.clear()
        try:
            for item in list_serial_ports():
                self.ports.addItem(item[0] if isinstance(item, (tuple, list)) else str(item))
        except (RuntimeError, ImportError, OSError) as exc:
            self.status.setText(str(exc))
        if previous:
            self.ports.setCurrentText(previous)

    def show_visual_details(self):
        """Read-only details do not pause capture or change the main layout."""
        if self.visual_details_dialog is None:
            dialog = QtWidgets.QDialog(self)
            dialog.setWindowTitle("画面与输入状态详情 · 只读")
            dialog.resize(700, 460)
            layout = QtWidgets.QVBoxLayout(dialog)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            scroll.setWidget(self.visual_details_content)
            self.visual_details_content.show()
            layout.addWidget(scroll, 1)
            close = QtWidgets.QPushButton("关闭详情")
            close.clicked.connect(dialog.hide)
            layout.addWidget(close)
            self.visual_details_dialog = dialog
        self.visual_details_dialog.show()
        self.visual_details_dialog.raise_()

    def show_input_contract(self):
        snapshot = self.controller.latest_snapshot()
        self._sync_active_input_contract(snapshot)
        if self.input_contract_dialog is None:
            dialog = QtWidgets.QDialog(self)
            dialog.setWindowTitle("12 路 IMU 输入配置与实时值 · 只读")
            dialog.resize(1120, 650)
            layout = QtWidgets.QVBoxLayout(dialog)
            summary = self._label(input_contract_summary(self.input_contract))
            summary.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            layout.addWidget(summary)
            table = QtWidgets.QTableWidget(12, 12)
            table.setHorizontalHeaderLabels([
                "IMU", "配置骨段", "陀螺 °/s/LSB", "加速度 g/LSB",
                "实际物理量程", "接收 Hz\n非芯片 ODR", "当前原始 gx / gy / gz\n输入快照 · 10 Hz", "当前原始 ax / ay / az",
                "VQF 静止 / 偏置估计", "当前样本号", "最近边界事件（历史）\n原值 / 时间 / 样本号",
                "解算输入保护\n原始值不变"])
            table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            table.verticalHeader().hide()
            table.verticalHeader().setDefaultSectionSize(66)
            table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
            table.horizontalHeader().setStretchLastSection(True)
            # Keep the disputed current/trigger pair visible together before
            # less frequently used configuration columns. Logical indices stay
            # stable for updates and tooltips.
            header = table.horizontalHeader()
            for visual, logical in enumerate((0, 1, 9, 6, 10, 7, 11, 5, 2, 3, 4, 8)):
                header.moveSection(header.visualIndex(logical), visual)
            layout.addWidget(table, 1)
            status = self._label()
            layout.addWidget(status)
            close = QtWidgets.QPushButton("关闭详情（不停止采集）")
            close.clicked.connect(dialog.hide)
            layout.addWidget(close)
            self.input_contract_dialog = dialog
            self.input_contract_summary_label = summary
            self.input_contract_table = table
            self.input_contract_status = status
        self._refresh_input_contract(snapshot, time.monotonic(), force=True)
        self.input_contract_dialog.show()
        self.input_contract_dialog.raise_()

    def _refresh_input_contract(self, snapshot, now, *, force=False):
        self._sync_active_input_contract(snapshot)
        if self.input_contract_dialog is None:
            return
        if not force and not self.input_contract_dialog.isVisible():
            return
        if (not force and self._input_contract_refresh_at is not None
                and 0 <= now - self._input_contract_refresh_at < .1):
            return
        self._input_contract_refresh_at = now
        snapshot = input_display_snapshot(snapshot)
        frame = snapshot.get("orientation") or {}
        connected = bool(snapshot.get("runtime", {}).get("connected"))
        for row, values in enumerate(input_contract_rows(self.input_contract, frame, now, connected=connected)):
            for column, text in enumerate(values):
                item = self.input_contract_table.item(row, column)
                if item is None:
                    item = QtWidgets.QTableWidgetItem(text)
                    self.input_contract_table.setItem(row, column, item)
                elif item.text() != text:
                    item.setText(text)
                if column == 4:
                    node = self.input_contract["nodes"][str(row + 1)]
                    gyro = node.get("nominal_gyro_full_scale_dps")
                    accel = node.get("nominal_accel_full_scale_g")
                    nominal = (f"系数对应数据手册档位：陀螺 ±{gyro:g} °/s、加速度 ±{accel:g} g。"
                               if gyro is not None and accel is not None else "没有可确认的名义档位。")
                    self._set_item_tooltip(item, nominal + "这不是设备配置读回；实际物理量程仍未确认。")
                elif column == 8:
                    tip = "VQF 估计值，不是硬件偏置真值；未判静止也不代表手一定在运动。六轴输入没有绝对航向观测。"
                    state = frame.get("nodes", {}).get(str(row + 1), {}).get("fusion_state", {})
                    sigma = state.get("bias_sigma_rad_s") if text != "—" else None
                    if isinstance(sigma, (int, float)) and np.isfinite(sigma):
                        tip += f"\n偏置不确定性估计 σ：{np.rad2deg(sigma):.4f} °/s。"
                    self._set_item_tooltip(item, tip)
                elif column == 10:
                    self._set_item_tooltip(item, "来自同一读取线程实际命中的原始样本；不是把当前值换算或推算成峰值。"
                                    "样本号为该 IMU 本会话累计输入编号；历史事件不会把当前小值标成当前命中。")
                elif column == 11:
                    self._set_item_tooltip(item, "仅表示送入 VQF 的输入动作；当前原始值、原始记录和编码边界事件不改写。"
                                    "陀螺按轴处理：异常分量先短暂保持，再用该轴偏置回退；正常分量原样透传。"
                                    "边界报告值遵循同一连续性检查，积分不证明物理饱和或真实转速已确认。异常加速度跳过更新。"
                                    "不能由保护后的稳定画面推断被拦截期间的真实运动。\n\n" + text)
        message = input_quality_text(frame, now, connected=connected)[0]
        if self.replay_raw:
            message = "CSV 回放，不代表正在读取硬件。\n" + message
        self._set_changed(self.input_contract_status, "setText", message)

    def _sync_active_input_contract(self, snapshot):
        """Use the worker's committed runtime contract, never proxy config."""
        contract = snapshot.get("runtime", {}).get("active_input_contract")
        check = snapshot.get("calibration", {}).get("input_scale_check") or {}
        if contract is None and check.get("status") == "applied":
            contract = check.get("applied_input_contract")
        if not isinstance(contract, dict) or contract == self.input_contract:
            return
        try:
            summary = input_contract_summary(contract)
            input_contract_rows(contract, {}, 0., connected=False)
        except (KeyError, TypeError, ValueError):
            return  # A partial worker update must not erase a known contract.
        self.input_contract = deepcopy(contract)
        self._gyro_scale_text = "本窗口陀螺换算：" + " / ".join(
            f"{node}号={contract['nodes'][node]['gyro_dps_per_lsb']:g}" for node in ("5", "6")) + (
            " °/s/LSB；不是固件量程读回。其余通道见“12 路输入配置 / 实时值”。")
        for node, badge in self.badges.items():
            role = contract['nodes'][str(node)]['role']
            self._set_changed(badge, "setText", f"{node:02d}  {_ROLE_NAMES.get(role, role)}")
        self._input_contract_refresh_at = None
        if self.input_contract_dialog is not None:
            self._set_changed(self.input_contract_summary_label, "setText", summary)

    def _input_scale_snapshot(self):
        snapshot, clearing = self._calibration_display_snapshot(self.controller.latest_snapshot())
        return snapshot, clearing

    def _input_scale_can_apply(self, snapshot, clearing):
        calibration = snapshot.get("calibration", {})
        check = calibration.get("input_scale_check") or {}
        return bool(self.official and not clearing and not self._clear_pending
                    and self._input_scale_apply_pending_generation is None
                    and check.get("status") == "completed" and check.get("candidate_available") is True
                    and calibration.get("active") is None
                    and not shape_busy(calibration)
                    and not str(calibration.get("profile_status", "")).startswith("fitting")
                    and calibration.get("profile_status") != "clearing")

    @staticmethod
    def _input_scale_ack_count(runtime):
        # Execution completion and pre-execution child queue rejection are
        # independent acknowledgements. Neither triggers an automatic retry.
        values = (runtime.get('input_scale_generation', 0), runtime.get('input_scale_rejected_count', 0))
        return sum(value for value in values if type(value) is int and value >= 0)

    def _refresh_input_scale_check(self, snapshot, clearing):
        check = snapshot.get("calibration", {}).get("input_scale_check") or {}
        runtime = snapshot.get("runtime", {})
        generation = self._input_scale_ack_count(runtime)
        if (clearing or generation != self._input_scale_apply_pending_generation
                or check.get("status") in {"waiting", "collecting", "failed", "applied"}):
            self._input_scale_apply_pending_generation = None
        summary = input_scale_check_text(check)
        if self._input_scale_apply_pending_generation is not None:
            summary += "\n正在请求采用并正常断开；不会自动连接。"
        if not clearing and runtime.get('input_scale_command_error'):
            summary += "\n后台反馈：" + str(runtime['input_scale_command_error'])
        self._set_changed(self.input_scale_summary, "setText", summary)
        self.input_scale_apply_button.setEnabled(self._input_scale_can_apply(snapshot, clearing))
        self.input_scale_report_button.setEnabled(not clearing and bool(check.get("report_path")))
        if self.input_scale_dialog is not None and check != self._input_scale_details_snapshot:
            self._set_changed(self.input_scale_details, "setPlainText", input_scale_check_text(check, detailed=True))
            self._input_scale_details_snapshot = deepcopy(check)

    def show_input_scale_check(self):
        if self.input_scale_dialog is None:
            dialog = QtWidgets.QDialog(self)
            dialog.setWindowTitle("三姿态输入尺度检查 · 只读结果")
            dialog.resize(760, 570)
            layout = QtWidgets.QVBoxLayout(dialog)
            self.input_scale_details = QtWidgets.QPlainTextEdit()
            self.input_scale_details.setReadOnly(True)
            layout.addWidget(self.input_scale_details)
            close = QtWidgets.QPushButton("关闭详情（不停止采集）")
            close.clicked.connect(dialog.hide)
            layout.addWidget(close)
            self.input_scale_dialog = dialog
        snapshot, clearing = self._input_scale_snapshot()
        self._refresh_input_scale_check(snapshot, clearing)
        self.input_scale_dialog.show()
        self.input_scale_dialog.raise_()

    def open_input_scale_report_directory(self):
        snapshot, clearing = self._input_scale_snapshot()
        check = snapshot.get("calibration", {}).get("input_scale_check") or {}
        path = check.get("report_path")
        if clearing or not isinstance(path, str) or not path:
            return
        directory = Path(path).expanduser().resolve().parent
        if directory.is_dir():
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(directory)))
        else:
            self.status.setText("报告目录当前不可用；请在逐路结果中查看报告路径。")

    def use_input_scale_candidate(self):
        snapshot, clearing = self._input_scale_snapshot()
        if not self._input_scale_can_apply(snapshot, clearing):
            return
        if self.send("use_input_scale_candidate"):
            self._input_scale_apply_pending_generation = self._input_scale_ack_count(snapshot.get("runtime", {}))
            self._refresh_input_scale_check(snapshot, clearing)

    @staticmethod
    def _set_item_tooltip(item, value):
        if item.toolTip() != value:
            item.setToolTip(value)

    def send(self, name, **args):
        if not self.controller.submit(name, **args):
            self.status.setText("操作队列正忙，请稍后再点击；不会丢弃原始采样来执行界面操作。")
            return False
        return True

    def _reset_calibration_display(self):
        """Drop presentation caches, never sensor/filter state or source data."""
        self._step = "neutral"
        self._capture_context = self._capture_completion = self._single_capture_feedback = None
        self._clear_display_cache()
        self._render_times.clear()
        self._pose_age_ms = None
        self._ui_values.clear()
        self.view.set_hand_visible(False)
        self.functional_selector.setCurrentIndex(0)
        self.shape_panel.reset()
        self._set_changed(self.instructions, "setText", "请从第1步开始：" + self._steps[0]["instruction"])
        self._set_changed(self.instructions, "setStyleSheet", "font-size: 14px; color: #254e52;")

    def _clear_display_cache(self):
        """Session/reference changes and demonstrations never reuse an old hand."""
        self._last_pose = self._demo_key = None
        self._has_display_cache = False
        self._display_source_timestamp = None
        self._display_metadata = {}
        self._display_updated_at = None
        self._render_error = ""
        self._pose_age_ms = None
        self._joint_quality_presentation.reset()
        self._display_warning.reset()
        self.view.set_hand_visible(False)

    def clear_calibration(self):
        snapshot = self.controller.latest_snapshot()
        runtime, calibration = snapshot.get("runtime", {}), snapshot.get("calibration", {})
        if (not runtime.get("connected") or self._clear_pending
                or calibration.get("profile_status") == "clearing"):
            return
        generation = calibration.get("clear_generation", 0)
        session = runtime.get("session_id")
        if not self.send("clear_calibration"):
            return
        self._clear_pending = True
        self._clear_generation_before = generation if type(generation) is int else 0
        self._clear_session = session
        self._reset_calibration_display()
        # A slow worker can still expose its previous successful snapshot.
        # Hide it immediately, before the next timer refresh/worker response.
        self.refresh()

    def _calibration_display_snapshot(self, snapshot):
        """Project an in-flight clear into empty UI state without editing data."""
        runtime, calibration = snapshot.get("runtime", {}), snapshot.get("calibration", {})
        empty = (calibration.get("active") is None and not calibration.get("captured_steps")
                 and not calibration.get("calibrated_nodes"))
        if self._clear_pending:
            generation = calibration.get("clear_generation")
            acknowledged = (empty and calibration.get("profile_status") == "not_calibrated"
                            and type(generation) is int and generation > self._clear_generation_before)
            disconnected = (not runtime.get("connected")
                            or runtime.get("session_id") != self._clear_session)
            if acknowledged or disconnected:
                self._clear_pending = False
        clearing = self._clear_pending or calibration.get("profile_status") == "clearing"
        if clearing:
            calibration = dict(active=None, captured_steps=[], calibrated_nodes=[],
                functional_fingers=[], functional_enabled=True, functional_hand_result=None,
                functional_diagnostics={}, profile_status="clearing", motion_diagnostics={},
                rejected_nodes={}, warnings=[], clear_generation=calibration.get("clear_generation", 0))
            runtime = {**runtime, "message": "正在清除本次标定；完成后请从第1步开始。", "error": None}
            snapshot = {**snapshot, "runtime": runtime, "calibration": calibration,
                        "pose": None, "geometry": None, "geometry_error": "",
                        "display_pose": None, "display_geometry": None}
            empty = True
        if empty and calibration.get("profile_status") in {"not_calibrated", "clearing"}:
            marker = (runtime.get("session_id"), calibration.get("clear_generation"),
                      calibration.get("profile_status"))
            if marker != self._empty_calibration_marker:
                self._reset_calibration_display()
                self._empty_calibration_marker = marker
            # Even an inconsistent late snapshot cannot make an empty
            # calibration display a previously successful measured pose.
            calibration = dict(active=None, captured_steps=[], calibrated_nodes=[],
                functional_fingers=[], functional_enabled=True, functional_hand_result=None,
                functional_diagnostics={}, profile_status=calibration["profile_status"],
                motion_diagnostics={}, rejected_nodes={}, warnings=[],
                clear_generation=calibration.get("clear_generation", 0),
                input_scale_check=calibration.get("input_scale_check", {}),
                shape_calibration=calibration.get("shape_calibration", {}))
            snapshot = {**snapshot, "calibration": calibration,
                        "pose": None, "geometry": None, "geometry_error": "",
                        "display_pose": None, "display_geometry": None}
        else:
            self._empty_calibration_marker = None
        return snapshot, clearing

    def toggle_connection(self):
        connected = self.controller.latest_snapshot().get("runtime", {}).get("connected")
        if connected:
            self.send("disconnect")
        else:
            port = self.ports.currentText().strip()
            if not port:
                self.status.setText("请先选择或输入 IMU 串口；不会自动猜测设备。")
                return
            self.send("connect", port=port)

    def capture(self, key):
        self._step = key
        self._set_changed(self.instructions, "setText", next(item["instruction"] for item in self._steps if item["key"] == key))
        self.send("capture", step=key)

    def toggle_joints(self, enabled):
        self.view.show_joints = enabled
        self.view.joints.setVisible(enabled and self.view.mesh.visible())

    def _set_changed(self, widget, method, value):
        """Avoid reparsing Qt styles and rebuilding text documents every tick."""
        key = (widget, method)
        if self._ui_values.get(key) != value:
            getattr(widget, method)(value)
            self._ui_values[key] = value

    def _refresh_local_recording(self, runtime):
        """Persistent storage warning, independent of input health and pose.

        A missing/transient IPC field cannot clear a known recording failure.
        The next explicitly enabled recording state clears it. Do not include
        changing sample counters here or restyle the IMU health indicator.
        """
        recording = runtime.get("local_recording")
        if isinstance(recording, dict):
            if recording.get("enabled") is True:
                self._local_recording_warning = ""
            elif recording.get("enabled") is False:
                warning = recording.get("warning")
                if isinstance(warning, str) and warning.strip():
                    self._local_recording_warning = warning
                elif not self._local_recording_warning:
                    self._local_recording_warning = (
                        "本地记录已暂停；暂停后的数据未写入本地文件。"
                        "本地记录状态不代表传感器输入或 ROS 发布状态。")
                if recording.get("profile_saved") is False:
                    note = "当前标定未保存到本地文件；请勿把内存中可用的标定当作已落盘。"
                    if note not in self._local_recording_warning:
                        self._local_recording_warning += "\n" + note
        text = self._local_recording_warning
        self._set_changed(self.local_recording_status, "setText", text)
        self._set_changed(self.local_recording_status, "setVisible", bool(text))

    @staticmethod
    def _functional_hand_result_text(result):
        """Describe only this run, never infer success from saved old entries."""
        if not isinstance(result, dict):
            return "全手流程约 24 秒；也可用下方逐指补采。已存参数不代表本次采集结果。"
        status = result.get("status")
        titles = {"complete": "最近一次全手采集结束（非人体精度验收）",
                  "partial": "最近一次全手仅部分更新，可逐指补采",
                  "failed": "最近一次全手未更新，原有参数保留",
                  "cancelled": "最近一次全手采集已取消，未更新参数"}
        lines = [titles.get(status, "最近一次全手结果待确认")]
        if status == "cancelled":
            return lines[0]
        fingers = result.get("fingers", {})
        for finger in OFFICIAL_FUNCTIONAL_FINGERS:
            item = fingers.get(finger, {})
            label = {"updated": "已更新", "partial": "部分更新", "failed": "未更新"}.get(
                item.get("status"), "本次未更新")
            name = "拇指 IP" if finger == "thumb" else FINGER_NAMES[finger]
            lines.append(f"{name}：{label}")
            if item.get("error"):
                lines.append(str(item["error"]))
            for slot, reason in item.get("rejected_slots", {}).items():
                lines.append(f"关节 {slot} 未更新：{reason}")
        return "\n".join(lines)

    def _capture_completion_text(self, capture, runtime, calibration):
        """End an observed capture without treating previously saved axes as success."""
        kind = capture.get("kind", "mounting")
        if kind == "functional_hand" and isinstance(calibration.get("functional_hand_result"), dict):
            return (self._functional_hand_result_text(calibration["functional_hand_result"]).splitlines()[0]
                    + "。请查看下方逐指结果；可重复全手采集，也可只补采未更新的手指，不必清除已存参数。")
        if kind == "functional":
            finger = capture.get("finger")
            name = "拇指 IP" if self.official and finger == "thumb" else FINGER_NAMES.get(finger, "所选手指")
            title = f"{name}本次采集已结束。"
            guidance = "可单独补采该指，不必重做全手；已存参数不代表本次全部更新。"
        elif kind == "neutral_reference":
            title = "伸直参考采集已结束。"
            guidance = "请查看更新结果；需要时可只重采伸直参考。"
        else:
            title = "本步骤采集已结束。"
            guidance = "已采步骤保留；可继续下一步，也可单独重采。"
        lines = [title, str(runtime.get("message") or "请查看本次采集反馈。")]
        if runtime.get("error"):
            lines.append("未完成原因：" + str(runtime["error"]))
        elif kind == "functional":
            # These are explicitly current stored states, not proof that every
            # joint was updated in this run. A failed run preserves old entries.
            states = {"fitted": "已拟合", "retained_previous": "保留上次", "rejected": "未更新"}
            for slot, item in calibration.get("functional_diagnostics", {}).get(capture.get("finger"), {}).items():
                if isinstance(item, dict) and item.get("status") in states:
                    lines.append(f"当前参数：关节 {slot} {states[item['status']]}")
                    if item.get("reason"):
                        lines.append(str(item["reason"]))
        return "\n".join([*lines, guidance])

    def refresh(self):
        started = time.perf_counter()
        snapshot = self.controller.latest_snapshot()
        snapshot, clearing = self._calibration_display_snapshot(snapshot)
        now = time.monotonic()
        runtime, calibration = snapshot.get("runtime", {}), snapshot.get("calibration", {})
        input_snapshot = input_display_snapshot(snapshot)
        input_runtime = input_snapshot.get("runtime", {})
        active = calibration.get("active")
        connected = input_runtime.get("connected", False)
        session = input_runtime.get("session_id") or runtime.get("session_id")
        base_fitting = calibration.get("profile_status") == "fitting"
        if session != self._capture_session:
            self._capture_context = self._capture_completion = self._single_capture_feedback = None
            self._capture_session = session
        if active:
            self._capture_context = {key: active[key] for key in ("kind", "step", "finger") if key in active}
            self._capture_completion = self._single_capture_feedback = None
        elif self._capture_context is not None and not base_fitting and not clearing:
            if connected:
                self._capture_completion = self._capture_completion_text(self._capture_context, runtime, calibration)
                if self._capture_context.get("kind") == "functional":
                    self._single_capture_feedback = self._capture_completion
            else:
                self._capture_completion = "连接已结束，本次采集已停止；重新连接后请以新会话的标定状态为准。"
            self._capture_context = None
        profile = (snapshot.get("pose") or {}).get("profile_version")
        if ((session is not None and session != self._display_session)
                or (profile is not None and self._display_profile is not None and profile != self._display_profile)):
            self._clear_display_cache()
        if session is not None:
            self._display_session = session
        if profile is not None:
            self._display_profile = profile
        self.connect_button.setText("断开" if connected else ("开始回放" if self.replay_raw else "连接"))
        self.ports.setEnabled(not connected and not self.replay_raw)
        self.scan.setEnabled(not connected and not self.replay_raw)
        frame = input_snapshot.get("orientation") or {}
        health_blocked = blocked_input_nodes(frame, self.config["node_to_slot"])
        health = frame.get("input_health") or {}
        health_faulted = (set(health.get("faulted_nodes", ()))
                          | set(health.get("recovering_nodes", ()))) & set(self.config["node_to_slot"])
        calibration_ready = calibration_input_ready(frame, self.config)
        self._refresh_input_scale_check(snapshot, clearing)
        self.shape_panel.refresh(snapshot, clearing=clearing)
        self._set_changed(self.workflow_status, "setText", workflow_status_text(calibration))
        shape_active = shape_busy(calibration)
        self._refresh_input_contract(snapshot, now)
        input_text, warning_nodes = solver_input_status(frame, now, connected=connected)
        self._set_changed(self.input_status, "setText", input_text)
        self._set_changed(self.input_status, "setStyleSheet",
                          "background: #fff3d9; padding: 6px; color: #8a4e00;" if warning_nodes else
                          "padding: 6px; color: #52636c;")
        fresh = frame.get("nodes", {})
        published = input_runtime.get("published_monotonic_s", 0.)
        if not isinstance(published, (float, int)) or now - published > .25:
            fresh = {}
        for node, badge in self.badges.items():
            color = "#d6f3e7" if str(node) in fresh and connected else "#e6eaed"
            status = fresh.get(str(node), {}).get("input_filter", {})
            if (str(node) in fresh and connected and frame.get("input_filter", {}).get("enabled")
                    and (status.get("gyro_rejected") or status.get("accel_rejected"))):
                color = "#dceefe"
            if connected and str(node) in warning_nodes:
                color = "#ffe0a3"
            self._set_changed(badge, "setStyleSheet", f"background: {color}; border-radius: 4px; padding: 5px; font-size: 11px;")
        captured = calibration.get("captured_steps", [])
        for item in self._steps:
            key = item["key"]
            self.step_buttons[key].setText(item["title"] + ("  ✓  重采" if key in captured else "  ·  采集"))
            self.step_buttons[key].setEnabled(bool(connected) and active is None and not base_fitting and not clearing and not shape_active and calibration_ready)
        self.clear.setEnabled(bool(connected) and not clearing)
        self.cancel_capture.setEnabled(bool(connected) and active is not None and not clearing)
        self.reanchor.setEnabled(not self.official and bool(connected) and active is None and not base_fitting and bool(calibration.get("calibrated_nodes")) and calibration_ready)
        for finger, button in self.functional_buttons.items():
            button.setEnabled(self.reanchor.isEnabled())
            name = "拇指" if finger == "thumb" else "小指"
            button.setText(name + "弯曲方向" + (" ✓ 重采" if finger in calibration.get("functional_fingers", []) else "校准"))
        functional_ready = self.official and bool(connected) and active is None and not base_fitting and not clearing and not shape_active and bool(calibration.get("calibrated_nodes")) and calibration_ready
        self.functional_hand_capture.setEnabled(functional_ready and all(item["key"] in captured for item in self._steps))
        self.functional_selector.setEnabled(functional_ready)
        self.functional_capture.setEnabled(functional_ready)
        self.functional_enabled.setEnabled(functional_ready)
        blocked = self.functional_enabled.blockSignals(True)
        self.functional_enabled.setChecked(bool(calibration.get("functional_enabled", True)))
        self.functional_enabled.blockSignals(blocked)
        for index, finger in enumerate(OFFICIAL_FUNCTIONAL_FINGERS):
            label = "拇指 IP（不改 MCP/CMC）" if finger == "thumb" else FINGER_NAMES[finger]
            self.functional_selector.setItemText(index, label + ("（已存参数）" if finger in calibration.get("functional_fingers", []) else ""))
        hand_fitting = bool(active and active.get("kind") == "functional_hand"
                            and calibration.get("profile_status") == "fitting_functional_hand")
        hand_feedback = ("动作采集已结束，正在计算全手结果；请放松，无需继续弯伸，可取消本次更新。" if hand_fitting else
                         "本次全手采集进行中；结束后逐指报告更新或未更新，不沿用旧参数标记判断成功。"
                         if active and active.get("kind") == "functional_hand" else
                         self._functional_hand_result_text(calibration.get("functional_hand_result")))
        if active and active.get("kind") == "functional":
            name = "拇指 IP" if active.get("finger") == "thumb" else FINGER_NAMES.get(active.get("finger"), "所选手指")
            hand_feedback = f"本次{name}采集进行中；结束后显示本次反馈，不用旧参数判断成功。"
        elif self._single_capture_feedback:
            hand_feedback = self._single_capture_feedback
            if isinstance(calibration.get("functional_hand_result"), dict):
                hand_feedback += ("\n\n以下为此前全手结果，不包含后续单指补采：\n"
                                  + self._functional_hand_result_text(calibration["functional_hand_result"]))
        self._set_changed(self.functional_hand_feedback, "setText", hand_feedback)
        if active:
            self._step = active.get("step", "neutral")
            instruction = ("五指自然伸直并停稳；只更新手指伸直参考，保留手掌朝向和安装轴。不要握拳，不必重新做三步。"
                           if active.get("kind") == "neutral_reference"
                           else next(item["instruction"] for item in self._steps if item["key"] == self._step))
            hand_phase = (_FUNCTIONAL_HAND_PHASES.get(active.get("phase"))
                          if active.get("kind") == "functional_hand" else None)
            if hand_phase:
                instruction = hand_phase[2]
            if hand_fitting:
                instruction = "全手动作采集结束，正在逐指计算并保存结果。请放松，无需继续弯伸；完成后查看各指反馈，失败部分可单独补采。"
            if active.get("kind") == "functional":
                name = FINGER_NAMES[active["finger"]]
                instruction = (f"现在弯伸{name}：手掌尽量不动，其他手指放松；从伸直到舒适弯曲，再伸直，慢慢重复 2–3 次。不必用力到极限。"
                               if active.get("phase") == "moving" else
                               f"先不要弯{name}：五指自然伸直并保持，等提示变为“现在弯伸”再动。")
                if self.official and active["finger"] == "thumb":
                    instruction = ("现在弯伸拇指 IP：近节尽量不动，只弯伸末节，再伸直；慢慢重复 2–3 次。不校正 MCP / CMC，不必用力到极限。"
                                   if active.get("phase") == "moving" else
                                   "拇指 IP 校准：先自然伸直并停稳；等提示变为“现在弯伸”再动。仅校正末节 IP，不改变 MCP / CMC。")
            self._set_changed(self.instructions, "setText", instruction)
            self._set_changed(self.instructions, "setStyleSheet", "font-size: 14px; font-weight: bold; color: #254e52;")
            remaining = max(0., active.get("remaining_s", 0.))
            preparing = active.get("phase") == "preparing"
            moving = active.get("phase") == "moving"
            duration = hand_phase[0] if hand_phase else 3. if preparing else 8. if moving else 2.
            self.progress.setValue(round(100 * max(0., min(1., 1. - remaining / duration))))
            progress_label = (hand_phase[3] if hand_phase else
                              "准备动作，暂未采集" if preparing else "现在弯伸，正在采集运动" if moving else "正在采集，请保持不动")
            self.progress.setFormat("正在计算全手结果，请放松；可取消" if hand_fitting else
                                    progress_label + f"  {remaining:.1f} 秒")
        else:
            self.progress.setValue(round(100 * len(captured) / 3))
            self.progress.setFormat("正在清除标定，请稍候" if clearing else
                                    "正在计算基础标定，请放松" if base_fitting else
                                    f"已采集 {len(captured)}/3 步；可单独重采")
            if clearing or calibration.get("profile_status") == "not_calibrated" and not captured:
                instruction = "请从第1步开始：" + self._steps[0]["instruction"]
            elif base_fitting:
                instruction = "三步动作采集已结束，正在计算基础标定。请放松，无需继续保持姿势；完成后查看结果。"
            elif self._capture_completion:
                instruction = self._capture_completion
            elif calibration.get("calibrated_nodes"):
                instruction = "基础标定已执行（非实物精度验收）；可按需重复采集任一步，或进行功能轴校准。"
            else:
                instruction = next(item["instruction"] for item in self._steps if item["key"] == self._step)
            self._set_changed(self.instructions, "setText", instruction)
            self._set_changed(self.instructions, "setStyleSheet", "font-size: 14px; color: #254e52;")
        motion = calibration.get("motion_diagnostics", {})
        motion_warnings = motion.get("warnings", []) if self.official else []
        status_message = runtime.get("message", "等待手动连接")
        pose_reason = (snapshot.get("pose") or {}).get("reason")
        input_age = input_runtime.get("last_sample_age_s")
        input_healthy = (connected and not health_blocked and bool(fresh) and isinstance(input_age, (int, float))
                         and not isinstance(input_age, bool) and np.isfinite(input_age)
                         and 0 <= input_age <= .25)
        automatic_stale = str(status_message).startswith(("手掌数据缺失或已过期", "计算进程暂未响应"))
        if (not runtime.get("error") and input_healthy and automatic_stale
                and pose_reason in {"missing_or_stale_root", "process_unresponsive"}):
            # A stale solved frame is not evidence that the independent serial
            # acquisition stopped. Explicit I/O errors/calibration feedback win.
            status_message = "输入正常，解算 / 显示等待更新；采集仍在继续。"
        if health_faulted:
            status_message = "输入异常保护中；请先恢复实时数据，暂不重新标定。"
        self._set_changed(self.status, "setText", runtime.get("error") or status_message)
        self._set_changed(self.status, "setStyleSheet", "color: #b33d30;" if runtime.get("error") else
                          "color: #8a4e00;" if health_faulted else "color: #206b5b;")
        self._refresh_local_recording(runtime)
        lines = [self._gyro_scale_text, *calibration.get("warnings", [])]
        gaps = frame.get("gap_diagnostics", {})
        if gaps:
            preserved = sum(item.get("short_gaps", 0) for item in gaps.values())
            resets = sum(item.get("filter_resets", 0) for item in gaps.values())
            lines.insert(0, f"接收间隔记录（按节点计）：短间隔保留滤波 {preserved} 次，长间隔重置 {resets} 次。"
                         "保留状态不等于证明未丢样或姿态准确。")
        for node, reason in calibration.get("rejected_nodes", {}).items():
            lines.append(f"节点 {node}: {reason}")
        rejection_labels = {
            "input_health_invalid": "输入冻结 / 接收时序异常，姿态积分已暂停；先恢复输入",
            "filter_epoch_changed_recalibration_required": "滤波器因长时中断或时钟异常重置；旧标定不再匹配，不能只重设伸直参考",
            "missing_or_stale": "当前缺少新鲜数据", "stale": "当前数据已过期",
        }
        for node, reason in (snapshot.get("pose") or {}).get("calibration_rejections", {}).items():
            lines.append(f"节点 {node}: {rejection_labels.get(reason, reason)}")
        if calibration.get("functional_fingers"):
            if self.official:
                lines.append("单指轴校正 + 关节约束，非实测精度。" if calibration.get("functional_enabled", True) else
                             "单指校正已关闭：显示官方原始输出；校准参数仍保留。")
            else:
                lines.append("已启用功能轴重定向：角度幅度不缩放，拟合残差属于重定向后的模型域，不是人体角度误差。")
        for finger, diagnostic in calibration.get("functional_diagnostics", {}).items():
            for slot, item in diagnostic.items():
                if not isinstance(item, dict):
                    continue
                label = {"fitted": "已拟合", "retained_previous": "保留上次", "rejected": "未更新"}.get(item.get("status"), item.get("status", ""))
                residual = item.get("holdout_p95_residual_deg")
                suffix = f"；留出偏轴残差 p95 {residual:.1f}°（非真值误差）" if isinstance(residual, (int, float)) else ""
                lines.append(f"{FINGER_NAMES.get(finger, finger)}关节 {slot}：{label}{suffix}")
                if item.get("reason"):
                    lines.append(str(item["reason"]))
                lines.extend(item.get("warnings", []))
        functional_adapter = (snapshot.get("pose") or {}).get("functional_adapter", {})
        coverage = (snapshot.get("pose") or {}).get("functional_calibration_coverage", {})
        if coverage:
            total = len(coverage.get("expected_slots", []))
            saved = sum(value is True for value in coverage.get("reference_present_mask", []))
            trusted = sum(value is True for value in coverage.get("correction_valid_mask", []))
            suffix = "（校正已关闭）" if not coverage.get("enabled", True) else ""
            lines.extend([
                f"功能覆盖：已存 {saved}/{total}；当前可信校正 {trusted}/{total}{suffix}。",
                "功能校正与覆盖标记不等于独立人体精度验收。",
                "航向等待激励窗口不代表输入断流；实时采集是否正常请看输入状态。",
            ])
        lines.extend(functional_adapter.get("warnings", []))
        for slot, reason in functional_adapter.get("skipped_slots", {}).items():
            lines.append(f"单指校正关节 {slot} 暂未应用：{reason}")
        calibrated = calibration.get("calibrated_nodes", [])
        if calibrated:
            lines.insert(0, f"本次标定 {len(calibrated)} 个实测骨段；非人体精度认证。")
        if self._render_error:
            lines.insert(0, "渲染更新失败；如有已展示手形则保留冻结参照：" + self._render_error)
        self._set_changed(self.details, "setPlainText", "\n".join(lines))
        visible = measured_pose_visible(snapshot, now) and not health_blocked
        quality_pose = snapshot.get("pose") or {}
        joint_style = self._joint_quality_presentation.update(quality_pose, now, session=session)
        joint_message = joint_quality_message(quality_pose, joint_style)
        limits = (snapshot.get("pose") or {}).get("active_joint_limits", []) if visible and not active else []
        if visible and not active:
            pose = snapshot["pose"]
            parts = []
            for slot, name in ((14, "拇指近节*"), (15, "拇指末节"), (7, "小指根部"), (8, "小指中节")):
                limit = next((item for item in limits if item["slot"] == slot and item["axis"] == "z"), None)
                marker = (" [软件下限]" if limit["side"] == "lower" else " [软件上限]") if limit else ""
                parts.append(f"{name} {joint_angle_text(pose, slot, marker=marker)}")
            text = "  ·  ".join(parts[:2]) + "\n" + "  ·  ".join(parts[2:])
            if self.official:
                coupled = snapshot["pose"].get("missing_slot_policy") == "pip_coupled_dip_with_neutral_other_missing"
                text += ("\n*四指缺测末节按中节联动推算，非实测；拇指根部仍为中立先验。" if coupled else
                         "\n*官方核心 + 缺测适配；未测末节/拇指根部保持局部中立先验，不是传感器实测。")
                if coupled:
                    parts = []
                    for slot, name in ((3, "食指"), (6, "中指"), (12, "无名"), (9, "小指")):
                        parts.append(f"{name} {joint_angle_text(pose, slot)}")
                    text += "\n末节推算：" + "  ·  ".join(parts)
            else:
                text += "\n*含未测根部的分解假设；小指末节由耦合推断，非独立实测。"
            self._set_changed(self.joint_status, "setText", text)
        else:
            self._set_changed(self.joint_status, "setText", "关节角与软件限位：等待新鲜的已标定姿态（不是人体极限测量）。")
        if clearing:
            self._clear_display_cache()
            self.view.set_hand_visible(False)
            self.view_state.setText("正在清除本次标定 · 已隐藏模型；完成后请从第1步开始")
            self._set_changed(self.view_state, "setStyleSheet", "background: #fff3d9; padding: 10px; font-weight: bold;")
        elif active or not calibrated and not self._has_display_cache:
            # Instruction geometry is never admitted into the measurement /
            # continuity cache, even if it happens to look like the last hand.
            if self._has_display_cache:
                self._clear_display_cache()
            key = ("functional_" + active["finger"] if active and active.get("kind") == "functional"
                   and active.get("phase") == "moving" else self._step)
            if active and active.get("kind") == "functional_hand":
                key = "neutral" if hand_fitting else _FUNCTIONAL_HAND_PHASES.get(active.get("phase"), (0., "neutral"))[1]
            if key != self._demo_key:
                self.view.set_hand_visible(False)
                try:
                    self.view.set_geometry(self._demonstrations[key])
                    self._demo_key, self._last_pose, self._render_error = key, None, ""
                except (KeyError, ValueError, RuntimeError, OSError) as exc:
                    self._render_error = str(exc)
            self.view_state.setText("动作示意 · 人工指定模型姿态，不是你的实时手势")
            self._set_changed(self.view_state, "setStyleSheet", "background: #fff3d9; padding: 10px; font-weight: bold;")
        else:
            new_contract = "display_pose" in snapshot or "display_geometry" in snapshot
            candidate = snapshot.get("display_pose") if new_contract else quality_pose if visible else None
            if health_blocked:
                candidate = None  # Keep the last image, never advance from a rejected input.
            # Old injected controllers have no compositor. Never render their
            # unavailable zero placeholders; keep the already displayed hand.
            if not new_contract and not all(joint_quality_state(quality_pose)[1]):
                candidate = None
            if isinstance(candidate, dict):
                stamp = candidate.get("source_timestamp_s", candidate.get("timestamp_s"))
                candidate_session = candidate.get("session_id", session)
                candidate_profile = candidate.get("profile_version")
                token = (candidate_session, stamp, candidate.get("timestamp_s"),
                         candidate.get("frame_index"), candidate_profile,
                         candidate.get("shape_preview_mode"), candidate.get("shape_preview_revision"),
                         (calibration.get("shape_calibration") or {}).get("preview_revision"))
                context_matches = (candidate_session in (None, session)
                    and (candidate_profile is None or self._display_profile is None
                         or candidate_profile == self._display_profile))
                if context_matches and token != self._last_pose:
                    try:
                        if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                                or not np.isfinite(stamp) or stamp < 0 or stamp > time.monotonic()):
                            raise ValueError("显示源时间不可用，保留上一幅展示")
                        if new_contract or getattr(self.controller, "renders_geometry", False):
                            geometry = snapshot.get("display_geometry" if new_contract else "geometry")
                            if geometry is None:
                                raise RuntimeError(snapshot.get("display_geometry_error")
                                    or snapshot.get("geometry_error") or "计算进程尚未返回新的显示网格")
                        else:
                            geometry = (self.backend.render_mano(candidate["mano_pose_rotvec"], candidate.get("shape_params"))
                                if self.official else self.backend.render(candidate["anatomical_euler_rad"], candidate["root_quat_xyzw"]))
                        self.view.set_geometry(geometry)
                        self._last_pose, self._demo_key, self._render_error = token, None, ""
                        self._has_display_cache = True
                        self._display_source_timestamp = float(stamp)
                        self._display_metadata = dict(candidate)
                        # These fields identify the geometry actually returned
                        # by the worker. A requested mode alone cannot relabel
                        # a retained/current mesh after a failed preview.
                        self._display_metadata["shape_preview_revision"] = candidate.get("shape_preview_revision")
                        self._display_metadata["shape_preview_mode"] = candidate.get("shape_preview_mode", "current")
                        self._display_updated_at = time.monotonic()
                        self._render_times.append(self._display_updated_at)
                        self._pose_age_ms = (self._display_updated_at - stamp) * 1000.
                    except (KeyError, TypeError, ValueError, RuntimeError, OSError) as exc:
                        self._render_error = str(exc)

            display_now = time.monotonic()
            current_measurement = measured_pose_visible(snapshot, display_now) and not health_blocked
            if not current_measurement:
                self._set_changed(self.joint_status, "setText", "当前测量角度不可用 / 已过期 · 保留图像不代表新的测量，不显示历史角度。")
            age = (max(0., display_now - self._display_source_timestamp)
                   if self._display_source_timestamp is not None else None)
            input_age = input_runtime.get("last_sample_age_s")
            interrupted = (isinstance(input_age, (int, float)) and not isinstance(input_age, bool)
                           and np.isfinite(input_age) and input_age > .25)
            held = self._display_metadata.get("held_slots", [])
            unknown = self._display_metadata.get("unknown_slots", [])
            provisional = self._display_metadata.get("provisional_slots", [])
            issue = (not self._has_display_cache or not connected or interrupted or not current_measurement
                     or age is None or age > .25 or not all(joint_quality_state(quality_pose)[0])
                     or bool(held or unknown or provisional or self._render_error))
            pending = self._display_warning.update(issue, display_now)
            if self._has_display_cache:
                self.view.set_hand_visible(True)
                age_text = f"源年龄 {age * 1000:.0f} ms"
                if self._render_error:
                    heading = "模型渲染更新失败 · 已冻结最近展示手形，非当前测量"
                elif not connected:
                    heading = "已断开 · 已冻结最近展示手形，非当前测量"
                elif health_faulted:
                    heading = "输入异常保护 · 已冻结最近展示手形，非当前测量；先恢复输入，再重新标定"
                elif interrupted:
                    heading = "输入中断 · 已冻结最近展示手形，非当前测量；等待新输入，暂不重做标定"
                elif age > .25:
                    heading = "解算延迟 / 显示等待更新 · 显示冻结，保持上次显示姿态，非新测量"
                elif not current_measurement:
                    heading = "解算延迟 / 显示等待更新 · 保持完整手形，仅供显示，非当前测量"
                elif pending and not issue:
                    # Recovery hysteresis applies to the wording category as
                    # well as its color, never flashing back to "live" for one
                    # good frame between two delayed results.
                    heading = "解算延迟 / 显示等待更新 · 结果已恢复，稳定观察中；完整手形仅供显示"
                elif held or unknown or provisional or joint_message:
                    heading = "连续显示参考 · 部分关节暂估 / 保持，非完整有效测量"
                else:
                    heading = (("回放重建" if self.replay_raw else "实时重建") + " · 有缺测推断的实验结果")
                    if self.official:
                        heading = ("官方 FSGlove + 单指轴校正 + 关节约束 · 非实测精度"
                                   if functional_adapter.get("applied_slots") else
                                   "官方 FSGlove 核心 · 12 路缺测适配 · 非完整 16 路观测")
                if quality_pose.get("reason") == "calibration_epoch_changed":
                    heading += " · 标定失效，需查看节点反馈"
                messages = [heading + " · " + age_text]
                if held:
                    names = "、".join(JOINT_LABELS.get(slot, str(slot)) for slot in held)
                    messages.append("局部保持：" + names + "；保持上次显示姿态，非新测量")
                    source_times = self._display_metadata.get("source_timestamp_by_slot", {})
                    source_ages = self._display_metadata.get("source_age_by_slot", {})
                    times = [source_times.get(str(slot), source_times.get(slot)) for slot in held]
                    times = [value for value in times if isinstance(value, (int, float)) and np.isfinite(value)]
                    values = [max(0., display_now - value) for value in times]
                    if not values:
                        values = [source_ages.get(str(slot), source_ages.get(slot)) for slot in held]
                        elapsed = max(0., display_now - self._display_source_timestamp)
                        values = [value + elapsed for value in values
                                  if isinstance(value, (int, float)) and np.isfinite(value)]
                    values = [value for value in values if isinstance(value, (int, float)) and np.isfinite(value)]
                    if values:
                        messages[-1] += f" · 最旧局部参考 {max(values) * 1000:.0f} ms"
                if unknown:
                    messages.append("无历史参考：" + "、".join(JOINT_LABELS.get(slot, str(slot)) for slot in unknown)
                                    + "；中立轮廓不是测量值")
                if joint_message:
                    messages.append(joint_message)
                elif provisional:
                    messages.append("部分关节为当前暂估；完整手形仅供显示，不代表有效测量")
                if pending and not issue:
                    messages.append("数据已恢复 · 稳定观察 200 ms 后恢复正常提示")
            else:
                self.view.set_hand_visible(False)
                messages = ["等待首个可展示完整手形 · " + ("渲染失败：" + self._render_error
                            if self._render_error else unavailable_pose_message(snapshot))]
            display_warnings = []
            if functional_adapter.get("warnings"):
                display_warnings.append("单指轴校正存在偏轴残差提示 · 仍继续显示；残差不等于人体角度误差")
            if motion_warnings:
                root_quality = motion.get("root", {})
                total = root_quality.get("up_to_down_deg")
                axis_error = root_quality.get("opposite_axis_error_deg")
                values = []
                if isinstance(total, (int, float)):
                    values.append(f"上下转角 {total:.1f}°")
                if isinstance(axis_error, (int, float)):
                    values.append(f"同轴偏差 {axis_error:.1f}°")
                display_warnings.append("标定方向需核对 · " + " / ".join(values) + " · 可能出现侧偏，可单独重采")
            if limits:
                display_warnings.append("姿态触及软件限位 · 可能出现动作死区；先检查伸直参考和运动轴，不代表真实极限")
            if display_warnings:
                messages.extend(display_warnings)
            self._set_changed(self.view_state, "setText", "\n".join(messages))
            self._set_changed(self.view_state, "setStyleSheet", "background: "
                + ("#fff3d9" if pending or display_warnings else "#d6f3e7") + "; padding: 10px; font-weight: bold;")
        summary = compact_view_status(self.view_state.text(), replay=bool(self.replay_raw))
        if warning_nodes:
            summary += " · 输入有提示"
        self._set_changed(self.visual_status, "setText", summary)
        # Preserve existing warning/recovery colors, but only in a single-line
        # badge. Numbers and per-joint details cannot resize this fixed row.
        background = "#fff3d9" if warning_nodes or "#fff3d9" in self.view_state.styleSheet() else "#d6f3e7"
        self._set_changed(self.visual_status, "setStyleSheet",
            f"background: {background}; padding: 6px; border-radius: 5px;")
        self._set_changed(self.visual_status, "setToolTip", summary + "；完整信息见右侧“状态详情”。")
        shape_state = calibration.get("shape_calibration") or {}
        preview_mode = shape_state.get("preview_mode", "current")
        shown_mode = self._display_metadata.get("shape_preview_mode", "current")
        shown_name = {"current": "当前已采用手形", "candidate": "候选手形预览", "baseline": "基准手形对比"}.get(shown_mode, "手形来源待确认")
        shape_text = "显示：" + shown_name
        if shown_mode != "current":
            shape_text += " · 仅显示，不改变解算 / ROS"
        if (preview_mode != shown_mode or self._display_metadata.get("shape_preview_revision") != shape_state.get("preview_revision")):
            shape_text = "显示：" + shown_name + " · 新预览等待有效网格"
        if self._demo_key is not None or not self._has_display_cache:
            shape_text = "无实时手形预览；示意 / 保留图像不证明接触"
        self._set_changed(self.shape_view_status, "setText", shape_text)
        self._render_times = [stamp for stamp in self._render_times if now - stamp < 1.]
        age = input_runtime.get("last_sample_age_s")
        age_text = f"{age * 1000:.0f} ms" if isinstance(age, (float, int)) and np.isfinite(age) else "—"
        self._refresh_ms = (time.perf_counter() - started) * 1000.
        timing = runtime.get("timing_ms") or {}
        timing_text = "  ·  ".join(f"{label} {timing[key]:.1f} ms" for key, label in (
            ("solve", "求解"), ("mesh", "网格")) if isinstance(timing.get(key), (int, float)))
        processing = runtime.get("processing") or {}
        processing_text = ""
        if processing:
            target = processing.get("target_rate_hz", 0.)
            actual = processing.get("solved_rate_hz", 0.)
            processing_text = (f"\n新输入解算 {actual:.1f} Hz / 目标 {target:g} Hz"
                               f"  ·  完整周期 {processing.get('cycle_ms', 0.):.1f} ms"
                               f"  ·  超期 {processing.get('overrun_cycles', 0):,} 次"
                               "（不含无效心跳或重复输入）")
        pose_age = f"{self._pose_age_ms:.0f} ms" if self._pose_age_ms is not None else "—"
        receive_text = ""
        receive = input_runtime.get("input_diagnostics") or {}
        if receive.get("source_kind") == "raw_serial":
            receive_text = (f"\n串口接收 {receive.get('bytes_received', 0):,} 字节  ·  "
                            f"解码 IMU 帧 {receive.get('frames_decoded', 0):,}  ·  "
                            f"解析丢弃 {receive.get('bytes_discarded', 0):,} 字节"
                            "（计数不代表 CRC 校验通过）")
        export = runtime.get("ros_export") or {}
        ros_text = ""
        if export.get("enabled"):
            ros_text = (f"\nROS 结果出口：已发送 {export.get('sent', 0):,} 帧，"
                        f"未送达 {export.get('dropped', 0):,} 帧（非 bag 接收确认）")
            if export.get("last_error"):
                ros_text += " · 桥接暂不可用，手套采集与标定继续"
        self.stats.setText(f"新鲜通道 {len(fresh)}/12  ·  累计样本 {input_runtime.get('received_samples', 0):,}  ·  "
                           f"最新数据年龄 {age_text}\n重建画面更新 {len(self._render_times)} Hz（非硬件采样率）"
                           f"  ·  提交网格时姿态年龄 {pose_age}\n"
                           f"{timing_text}  ·  界面更新 {self._refresh_ms:.1f} ms（不含 GPU 显示延迟）"
                           f"{processing_text}{receive_text}{ros_text}")
        if runtime.get("output_dir"):
            self.output.setText("本次记录：" + str(runtime["output_dir"]))
        self._tactile_pose_current = bool(
            not clearing and not active and not self.replay_raw and not health_blocked
            and self._demo_key is None and self._has_display_cache
            and not self._render_error and measured_pose_visible(snapshot, time.monotonic()))
        if not self._tactile_pose_current and self.tactile_controller is not None:
            self.view.set_tactile()  # Never paint live contact onto a calibration demonstration.

    def closeEvent(self, event):
        self.timer.stop()
        self.controller.stop()
        if self.controller.join(3.):
            if self.tactile_timer is not None:
                self.tactile_timer.stop()
            if self.tactile_controller is not None:
                self.tactile_controller.close()
            if self.input_contract_dialog is not None:
                self.input_contract_dialog.hide()
            if self.input_scale_dialog is not None:
                self.input_scale_dialog.hide()
            if self.visual_details_dialog is not None:
                self.visual_details_dialog.hide()
            event.accept()
        else:
            self.status.setText("正在关闭串口读取线程，请稍候再关闭窗口；未强制终止或抢占设备。")
            self.timer.start(100)
            event.ignore()


def run_live(config, mano_assets, *, output_root, port="", replay_raw=None, replay_speed=1., upstream_checkout=None,
             ros_socket=None, tactile_socket=None, tactile_layout=None, hand=None):
    from .config import validate_config
    config = validate_config(config)
    if hand is not None and hand != config["hand"]:
        raise ValueError("requested hand conflicts with configuration hand")
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise RuntimeError("实时窗口需要 Linux 桌面会话（DISPLAY/WAYLAND_DISPLAY）；不会打开串口。")
    if replay_raw is not None and not Path(replay_raw).is_file():
        raise ValueError("回放 raw.csv 不存在；不会连接任何串口。")
    if not np.isfinite(replay_speed) or replay_speed <= 0:
        raise ValueError("回放速度必须大于零")
    backend = ManoBackend(mano_assets, hand=config["hand"])
    backend.export_rest()  # Fail clearly before opening any GUI or device.
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setFont(QtGui.QFont("Noto Sans CJK SC", 10))
    window = LiveWindow(config, backend, output_root=output_root, port=port,
                        replay_raw=replay_raw, replay_speed=replay_speed, upstream_checkout=upstream_checkout,
                        ros_socket=ros_socket, tactile_socket=tactile_socket, tactile_layout=tactile_layout)
    window.show()
    return app.exec_()


run_live_gui = run_live
