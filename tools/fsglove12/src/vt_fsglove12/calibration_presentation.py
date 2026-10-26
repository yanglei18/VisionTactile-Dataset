"""Read-only calibration facts shared by every part of the live workspace.

The runtime remains the owner of calibration, acquisition and file writes.
These functions project one snapshot into immutable presentation values; they
never retain sensor state, submit commands, read a clock or perform I/O.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math


_UNSET = object()
_STEPS = frozenset({"neutral", "fingers_down", "thumb_up"})
_INVALID_REASONS = frozenset({"calibration_epoch_changed", "calibration_session_mismatch"})
_INVALID_NODE_REASONS = frozenset({"filter_epoch_changed_recalibration_required"})


def _mapping(value):
    return value if isinstance(value, Mapping) else {}


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _node_order(value):
    value = str(value)
    return (0, int(value)) if value.isdecimal() else (1, value)


def _text(value):
    return value.strip() if isinstance(value, str) else ""


def shape_busy(calibration):
    state = _mapping(_mapping(calibration).get("shape_calibration"))
    return bool(state.get("busy") or state.get("status") in {"capturing", "optimizing"})


@dataclass(frozen=True)
class CalibrationStatus:
    state: str
    has_profile: bool
    saved: bool | None
    invalid_nodes: tuple[str, ...]
    current_pose: bool
    mounting_text: str

    @property
    def requires_recalibration(self):
        return self.state in {"invalid", "partial"}

    @property
    def summary(self):
        headings = {
            "valid": "基础标定已生效",
            "invalid": "标定已失效，请重新标定",
            "partial": self.mounting_text + "，见诊断",
            "waiting": "标定参数已保留 · 等待有效输入",
            "retained": "本次标定结果已保留",
            "absent": "尚未完成基础标定",
            "failed": "基础标定拟合未通过，可单步重采",
            "collecting": "正在采集 · 完成后更新结果",
            "fitting": "正在计算标定结果",
            "clearing": "正在清除标定，请稍候",
        }
        text = headings[self.state]
        if self.has_profile and self.state not in {"collecting", "fitting", "clearing"}:
            text += (" · 已保存" if self.saved is True else
                     " · 未保存" if self.saved is False else " · 保存状态待确认")
        return text


def calibration_status(snapshot, now=None, stale_after_s=.25, *, profile_saved=_UNSET):
    """Separate reference validity, current output and persistence evidence.

    ``now`` is supplied by the caller, so a refresh uses one consistent clock.
    If omitted, the caller is responsible for snapshot freshness. A retained or
    expired pose never establishes current validity when ``now`` is supplied.
    ``profile_saved`` lets the window supply its context-bound sparse-IPC cache;
    omission reads the explicit backend field, without guessing from file paths.
    """
    runtime = _mapping(snapshot.get("runtime"))
    calibration = _mapping(snapshot.get("calibration"))
    pose = _mapping(snapshot.get("pose"))
    has_profile = bool(calibration.get("calibrated_nodes"))
    if profile_saved is _UNSET:
        profile_saved = _mapping(runtime.get("local_recording")).get("profile_saved")
    saved = profile_saved if type(profile_saved) is bool else None
    connected = runtime.get("connected") is True
    same_session = not (runtime.get("session_id") is not None and pose.get("session_id") is not None
                        and runtime["session_id"] != pose["session_id"])
    fresh = True
    if now is not None:
        values = (now, stale_after_s, pose.get("timestamp_s"), runtime.get("published_monotonic_s"),
                  runtime.get("last_sample_age_s"))
        fresh = all(_number(value) for value in values) and stale_after_s >= 0
        if fresh:
            fresh = (0 <= now - pose["timestamp_s"] <= stale_after_s
                     and 0 <= now - runtime["published_monotonic_s"] <= stale_after_s
                     and 0 <= runtime["last_sample_age_s"] <= stale_after_s)
    current_pose = bool(connected and same_session and fresh
                        and pose.get("pose_available") is True and pose.get("solver_success") is True)
    rejections = _mapping(pose.get("calibration_rejections")) if same_session else {}
    invalid_nodes = tuple(sorted((str(node) for node, reason in rejections.items()
                                  if reason in _INVALID_NODE_REASONS), key=_node_order))
    uncalibrated_nodes = {str(node) for node, reason in rejections.items() if reason == "node_not_calibrated"}
    uncalibrated_nodes.update(map(str, _mapping(calibration.get("rejected_nodes"))))
    profile_status = str(calibration.get("profile_status", ""))
    captured = set(calibration.get("captured_steps") or ()) & _STEPS
    if profile_status == "clearing":
        state, mounting = "clearing", "正在清除"
    elif profile_status.startswith("fitting"):
        state, mounting = "fitting", "正在计算"
    elif calibration.get("active") is not None:
        state, mounting = "collecting", f"采集中 · 已采集 {len(captured)}/3 步"
    elif not has_profile:
        state = "failed" if profile_status == "fit_failed" else "absent"
        mounting = ("拟合未通过，可单步重采" if state == "failed" else
                    "已采集，等待结果" if len(captured) == 3 else f"已采集 {len(captured)}/3 步")
    elif same_session and pose.get("reason") in _INVALID_REASONS:
        state, mounting = "invalid", "已失效，需重新标定"
    elif invalid_nodes:
        state, mounting = "partial", "部分骨段标定失效"
    elif not connected:
        state, mounting = "retained", "参数已保留，设备未连接"
    elif not current_pose:
        state, mounting = "waiting", "参数已保留，等待有效输入"
    elif uncalibrated_nodes:
        state, mounting = "partial", "部分骨段未完成标定"
    else:
        state, mounting = "valid", "已生效"
    return CalibrationStatus(state, has_profile, saved, invalid_nodes,
                             current_pose and has_profile and state not in {"invalid", "clearing"}, mounting)


def workflow_status_text(snapshot, status=None, *, now=None, stale_after_s=.25):
    """Use the same reference conclusion as the global result summary."""
    calibration = _mapping(snapshot.get("calibration"))
    status = status if status is not None else calibration_status(snapshot, now, stale_after_s)
    fingers = set(calibration.get("functional_fingers") or ())
    functional = f"已有 {len(fingers)}/5 指参数" if fingers else "待采集"
    if calibration.get("profile_status") == "fitting_functional_hand":
        functional = "正在计算"
    if fingers and not calibration.get("functional_enabled", True):
        functional += " · 当前关闭"
    lines = [f"基础标定：{status.mounting_text}", f"弯伸校准：{functional}"]
    check = _mapping(calibration.get("input_scale_check"))
    unresolved = check.get("inconclusive_nodes") or []
    if unresolved:
        lines.append("输入尺度：" + "、".join(map(str, unresolved)) + "号待核对，查看诊断")
    elif check.get("status") == "applied":
        lines.append("输入配置已更新，请重新连接标定")
    elif check.get("status") == "completed" and check.get("recommended_nodes"):
        lines.append("输入尺度有待核对候选，查看诊断")
    return "\n".join(lines)


@dataclass(frozen=True)
class StorageIssue:
    source: str
    label: str
    error: str


@dataclass(frozen=True)
class StorageStatus:
    issues: tuple[StorageIssue, ...]
    messages: tuple[str, ...]
    can_retry: bool
    warnings: tuple[str, ...] = ()

    @property
    def has_errors(self):
        return bool(self.issues)

    @property
    def has_notices(self):
        return bool(self.issues or self.warnings)

    @property
    def error_text(self):
        return "\n".join(f"{issue.label}保存失败：{issue.error}" for issue in self.issues)

    @property
    def message(self):
        return "\n".join(self.messages)

    @property
    def notice_text(self):
        return "\n".join(text for text in (self.error_text, *self.warnings) if text)


def storage_status(snapshot):
    """Keep file and touch-checkpoint failures independent, including recovery."""
    runtime = _mapping(snapshot.get("runtime"))
    calibration = _mapping(snapshot.get("calibration"))
    files = _mapping(runtime.get("calibration_storage"))
    shape = _mapping(calibration.get("shape_calibration"))
    issues = []
    for source, label, value in (("calibration_files", "标定文件", files.get("error")),
                                 ("touch_task", "触指任务", shape.get("storage_error"))):
        error = _text(value)
        if error:
            issues.append(StorageIssue(source, label, error))
    messages = tuple(dict.fromkeys(text for text in
        (_text(files.get("warning")), _text(shape.get("storage_message"))) if text))
    evicted = files.get("evicted_count")
    warnings = ((f"{evicted} 份较早的标定文件未保留用于重试；当前内存标定仍保留。",)
                if type(evicted) is int and evicted > 0 else ())
    return StorageStatus(tuple(issues), messages,
                         files.get("retry_available") is True or shape.get("can_retry_save") is True,
                         warnings)


def save_actions_allowed(snapshot, storage, *, clearing=False, pending=False, official=True):
    """Shared UI eligibility; the runtime still rechecks every submitted action."""
    calibration = _mapping(snapshot.get("calibration"))
    status = str(calibration.get("profile_status", ""))
    busy = (clearing or pending or calibration.get("active") is not None
            or status == "clearing" or status.startswith("fitting") or shape_busy(calibration))
    return bool(official and storage.has_errors and storage.can_retry and not busy)
