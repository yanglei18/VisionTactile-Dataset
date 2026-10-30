"""Qt-independent calibration presentation; manager snapshots remain authoritative."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from ..i18n import tr


COLLECTING_STATES = {"PREPARING", "COLLECTING", "STOPPING", "SOLVING", "VERIFYING", "COMPARING", "CANCELLING"}
RESULT_STATES = {"READY", "APPLYING", "WAITING_TO_APPLY", "CANDIDATE", "COMPLETE",
                 "APPLIED_PENDING_PUBLICATION", "FAILED", "CANCELLED", "INTERRUPTED"}
TERMINAL_STATES = {"CANDIDATE", "COMPLETE", "FAILED", "CANCELLED", "INTERRUPTED"}

HINTS = {
    "WAITING_INPUT": tr('等待相机和追踪器数据'),
    "SHOW_BOARD": tr('让标定板完整出现在画面中'),
    "HOLD_STILL": tr('保持静止，等待自动采样'),
    "MOVE_TO_NEW_POSE": tr('移动相机与追踪器整体，换一个角度后停稳'),
    "INCREASE_TRANSLATION": tr('增大相机位置变化，保持标定板可见'),
    "INCREASE_ROTATION": tr('增加相机的俯仰或偏转角度'),
    "ROTATE_SECOND_AXIS": tr('换一个旋转方向，补充不同角度'),
    "COLLECT_VALIDATION": tr('继续移动到新姿态，采集独立验证数据'),
    "ROUND_COMPLETE": tr('本轮采集完成，正在检查数据'),
    "STOPPING_RECORDING": tr('正在保存本轮数据'),
    "SOLVING": tr('正在计算标定结果'),
    "VERIFYING": tr('正在使用独立数据验证结果'),
    "COMPARING": tr('正在检查三轮结果的一致性'),
    "resuming_saved_rounds": tr('正在恢复已完成轮次，仅继续未完成轮次'),
}
STATE_LABELS = {
    "PREPARING": tr('正在准备标定'), "COLLECTING": tr('引导采集'),
    "STOPPING": tr('正在保存数据'), "SOLVING": tr('正在求解'),
    "VERIFYING": tr('正在验证'), "COMPARING": tr('正在检查重复性'),
    "READY": tr('结果已验证'), "APPLYING": tr('正在应用标定'),
    "WAITING_TO_APPLY": tr('结果已保存，等待应用条件满足'),
    "CANDIDATE": tr('候选结果已保存，尚未应用'),
    "COMPLETE": tr('标定已应用并发布'),
    "APPLIED_PENDING_PUBLICATION": tr('标定已应用，发布尚未完成'),
    "FAILED": tr('标定未完成，请查看详情后重试'),
    "CANCELLED": tr('标定已取消'), "INTERRUPTED": tr('标定已中断，已完成轮次保留'),
    "CANCELLING": tr('正在取消并保存已采集的数据'),
}

TRACKER_ROLES = {
    "torso": tr('躯干'), "left_wrist": tr('左腕'), "right_wrist": tr('右腕'),
}


def tracker_role_label(role: str) -> str:
    """Translate known physical roles, preserving unknown/custom identifiers."""
    return TRACKER_ROLES.get(role, role)


def can_resume(snapshot):
    view = calibration_view(snapshot)
    session = (snapshot.get('calibration') or {}).get('session') or {}
    return bool(view.available and not view.busy and session.get('mode') == 'guided'
                and session.get('state') in {'FAILED', 'CANCELLED', 'INTERRUPTED'})


def _count(value, default=0):
    return max(0, value) if type(value) is int else default


@dataclass(frozen=True)
class CalibrationView:
    available: bool
    busy: bool
    stage: str
    state: str
    session_id: str
    instruction: str
    round: int
    rounds: int
    accepted: int
    target: int
    can_cancel: bool
    can_start: bool
    can_retry_publish: bool
    published: bool
    quality_summary: str
    active_revision: str
    topic: str


def restorable_revision(snapshot: Mapping) -> tuple[str, str, str] | None:
    """Return camera, previous and current revisions for an explicit restore."""
    view = calibration_view(snapshot)
    if not view.available or view.busy:
        return None
    service = snapshot.get("calibration") or {}
    camera_name = (service.get("session") or {}).get("camera_name")
    active = next((entry for entry in service.get("active", ())
                   if camera_name and entry.get("camera_name") == camera_name), {})
    previous, current = active.get("previous_revision"), active.get("revision")
    if not all(isinstance(value, str) and value for value in (camera_name, previous, current)):
        return None
    return camera_name, previous, current


def calibration_view(snapshot: Mapping) -> CalibrationView:
    service = snapshot.get("calibration") or {}
    session = service.get("session") or {}
    state = session.get("state", "")
    available = bool(snapshot.get("mode") == "live" and snapshot.get("instance_id")
                     and snapshot.get("capabilities", {}).get("calibration")
                     and service.get("enabled"))
    busy = bool(service.get("busy"))
    stage = "collect" if state in COLLECTING_STATES else "result" if state else "prepare"
    instruction = (STATE_LABELS.get(state, tr('正在等待后台状态')) if state in RESULT_STATES else
                   HINTS.get(session.get("hint_code"), STATE_LABELS.get(state, tr('选择设备并确认标定板参数'))))
    if state == 'FAILED' and session.get('hint_code') == 'REPEATABILITY_FAILED':
        instruction = tr('单轮验证已通过，三轮一致性未通过；可选择单轮重拍')
    active = next((entry for entry in service.get("active", ())
                   if entry.get("source_session_id") == session.get("session_id")
                   and entry.get("camera_name") == session.get("camera_name")), {})
    result = session.get("result") or {}
    grade = active.get("quality_grade") or result.get("quality_grade")
    quality = {"TARGET": tr('质量达到目标要求'),
               "ACCEPTABLE": tr('质量满足最低有效要求')}.get(grade, "")
    published = bool(available and active.get("published"))
    if state == "COMPLETE":
        if not active:
            instruction = tr('标定已完成，当前发布状态待核实')
        elif not active.get("identity_valid"):
            instruction = tr('标定已保存，当前设备身份尚未核验')
        elif not published:
            instruction = tr('标定已应用，发布尚未完成')
    if not available:
        instruction = tr('连接支持标定的真实后台后即可开始')
    return CalibrationView(
        available=available, busy=busy, stage=stage, state=state,
        session_id=str(session.get("session_id", "")), instruction=instruction,
        round=max(1, _count(session.get("round"), 1)), rounds=max(1, _count(session.get("rounds"), 3)),
        accepted=_count(session.get("accepted")), target=max(1, _count(session.get("target"), 50)),
        can_cancel=available and busy and state not in {"APPLYING", "COMPLETE", "APPLIED_PENDING_PUBLICATION", "CANCELLING"},
        can_start=available and not busy and (not state or state in TERMINAL_STATES),
        can_retry_publish=available and state == "APPLIED_PENDING_PUBLICATION", published=published,
        quality_summary=quality, active_revision=str(active.get("revision", "")), topic=str(active.get("topic", "")),
    )


def preview_device(session: Mapping, *, connected: bool) -> dict:
    """Adapt the worker's same-frame JPEG to the public shared-preview contract."""
    preview = session.get("preview") or {}
    encoded = preview.get("jpeg_base64")
    return dict(id="calibration:" + str(session.get("session_id", "")), kind="camera",
                label=tr('标定画面'), mode="live", status="online" if encoded and connected else "offline",
                preview=dict(format="jpeg", data_b64=encoded) if encoded else {})
