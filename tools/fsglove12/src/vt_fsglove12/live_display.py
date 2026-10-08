"""Pure live-view presentation rules, also testable without Qt/OpenGL."""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

from .config import PARENTS
from .joint_quality import joint_quality_state


JOINT_LABELS = {0: "掌根", 1: "食指根部", 2: "食指中节", 3: "食指末节",
                4: "中指根部", 5: "中指中节", 6: "中指末节", 7: "小指根部",
                8: "小指中节", 9: "小指末节", 10: "无名指根部", 11: "无名指中节",
                12: "无名指末节", 13: "拇指根部", 14: "拇指近节", 15: "拇指末节"}


def input_display_snapshot(snapshot):
    """The two input views share the independent acquisition snapshot.

    Absence of this field is the legacy injected-controller compatibility seam;
    an explicitly missing new snapshot must not reuse an old solver frame.
    """
    if "input_snapshot" not in snapshot:
        return snapshot
    value = snapshot["input_snapshot"]
    return value if isinstance(value, dict) else {"runtime": {}, "orientation": None}


class StableDisplayWarning:
    """Warning immediately; normal style only after a stable 200 ms recovery."""
    def __init__(self, recovery_s=.2):
        self.recovery_s = recovery_s
        self.reset()

    def reset(self):
        self.warning = False
        self._recovery_since = None

    def update(self, issue, now):
        if issue:
            self.warning, self._recovery_since = True, None
        elif self.warning:
            if self._recovery_since is None or now < self._recovery_since:
                self._recovery_since = now
            elif now - self._recovery_since >= self.recovery_s:
                self.warning, self._recovery_since = False, None
        return self.warning


def downstream_display_mask(mask):
    """Global geometry depends on ancestors, unlike local joint validity.

    This display-only expansion prevents a valid relative PIP from floating
    beyond an unavailable MCP. Never feed it back into measured masks.
    """
    output = np.asarray(mask, dtype=bool).copy()
    if output.shape != (16,):
        raise ValueError("display mask must have 16 entries")
    for slot, parent in enumerate(PARENTS[1:], start=1):
        output[slot] |= output[parent]
    return output


class JointQualityPresentation:
    """Immediate downgrade and bounded stable recovery, ONLY for visual style.

    There is no pose/angle storage here. Every displayed position still comes
    from the current result; neither this timer nor its output enters exports.
    """
    def __init__(self, recovery_s=.2):
        if not math.isfinite(recovery_s) or not 0 <= recovery_s <= .5:
            raise ValueError("visual recovery must be bounded to 0..0.5 seconds")
        self.recovery_s = recovery_s
        self.reset()

    def reset(self):
        self._session = None
        self._provisional = [False] * 16
        self._recovery_since = [None] * 16
        self._hidden = [False] * 16
        self._available_since = [None] * 16

    @property
    def hidden(self):
        return tuple(self._hidden)

    def update(self, pose, now, *, session=None):
        if session != self._session:
            self.reset()
            self._session = session
        valid, available, _ = joint_quality_state(pose)
        for slot, current_valid in enumerate(valid):
            if not available[slot]:
                self._hidden[slot] = True
                self._available_since[slot] = None
            elif self._hidden[slot]:
                since = self._available_since[slot]
                if since is None or now < since:
                    self._available_since[slot] = now
                elif now - since >= self.recovery_s:
                    self._hidden[slot] = False
                    self._available_since[slot] = None
            if not current_valid:
                self._provisional[slot] = True
                self._recovery_since[slot] = None
            elif self._provisional[slot]:
                since = self._recovery_since[slot]
                if since is None or now < since:
                    self._recovery_since[slot] = now
                elif now - since >= self.recovery_s:
                    self._provisional[slot] = False
                    self._recovery_since[slot] = None
        return tuple(self._provisional)


def joint_quality_message(pose, provisional, hidden=()):
    """Stable text only: quality never chops or recolors the complete hand.

    The latched slot set deliberately does not alternate wording every time a
    residual crosses a threshold. Actual numbers still obey immediate masks.
    """
    valid, _, _ = joint_quality_state(pose)
    names = [JOINT_LABELS[slot] for slot in range(16) if provisional[slot] or not valid[slot]]
    return ("关节质量提示：" + "、".join(names)
            + " · 暂估 / 不可用 / 恢复观察；完整手形仅供显示，不代表有效测量") if names else ""


def joint_angle_text(pose, slot, *, marker=""):
    """Unavailable never becomes a measured zero or a last-good numeric value."""
    _, available, states = joint_quality_state(pose)
    if not available[slot]:
        return "— [不可用]"
    try:
        angle = float(pose["anatomical_euler_rad"][slot][2])
    except (KeyError, IndexError, TypeError, ValueError):
        return "— [不可用]"
    if not math.isfinite(angle):
        return "— [不可用]"
    suffix = " [暂估]" if states[slot] == "degraded" else ""
    return f"{np.degrees(angle):.1f}°{suffix}{marker}"


def neutral_geometry_slot_map(neutral_geometry):
    """Assign surface vertices to nearest neutral skeletal segment once.

    This is a display annotation, not MANO skin weights or a new estimator.
    MANO16 ordering differs from the wrist/thumb/index/... MANO21 joints.
    Each wrist-to-MCP segment remains the palm's slot 0, so a bad PIP never
    recolors its MCP or the complete hand. Tip markers follow the distal slot.
    """
    vertices = np.asarray(neutral_geometry["vertices_m"], dtype=float)
    joints = np.asarray(neutral_geometry["joints_m"], dtype=float)
    if joints.shape != (21, 3) or not np.isfinite(joints).all():
        raise ValueError("neutral joints must be a finite (21, 3) array")
    segments, slots = [], []
    joint_slots = np.zeros(21, dtype=int)
    for base, first_slot in ((1, 13), (5, 1), (9, 4), (13, 10), (17, 7)):
        segments.append((0, base))
        slots.append(0)
        for offset in range(3):
            segments.append((base + offset, base + offset + 1))
            slots.append(first_slot + offset)
            joint_slots[base + offset] = first_slot + offset
        joint_slots[base + 3] = first_slot + 2
    edges = np.asarray(segments)
    starts, ends = joints[edges[:, 0]], joints[edges[:, 1]]
    directions = ends - starts
    offsets = vertices[:, None, :] - starts[None, :, :]
    lengths2 = np.sum(directions * directions, axis=1)
    projections = np.sum(offsets * directions[None, :, :], axis=2) / np.maximum(lengths2, 1e-15)
    projections = np.clip(projections, 0., 1.)
    distance2 = np.sum((offsets - projections[..., None] * directions[None, :, :]) ** 2, axis=2)
    return np.asarray(slots)[np.argmin(distance2, axis=1)], joint_slots


def measured_pose_visible(snapshot, now, *, timeout_s=.25):
    runtime = snapshot.get("runtime", {})
    pose = snapshot.get("pose")
    if not runtime.get("connected") or not isinstance(pose, dict):
        return False
    age = runtime.get("last_sample_age_s")
    published = runtime.get("published_monotonic_s")
    stamp = pose.get("timestamp_s")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
           for x in (now, age, published, stamp)):
        return False
    if not (0 <= age <= timeout_s and 0 <= now - published <= timeout_s and 0 <= now - stamp <= timeout_s):
        return False
    quality_visible = pose.get("quality") in {"experimental", "experimental_partial"}
    if (pose.get("algorithm") in {"official_fsglove_raw2mano_with_explicit_sparse_adapter_v1",
                                  "official_fsglove_raw2mano_with_explicit_sparse_adapter_v2",
                                  "official_fsglove_with_qmt_functional_adapter_v1"}
            and pose.get("upstream_fsglove_solver_executed") is True):
        quality_visible = pose.get("quality") == "not_accuracy_validated"
    return pose.get("pose_available") is True and pose.get("solver_success") is True and quality_visible


def unavailable_pose_message(snapshot):
    """Distinguish stale output from a reset/uncalibrated/disconnected session.

    Presentation only: never extends freshness, resets filters or connects.
    """
    runtime, pose = snapshot.get("runtime", {}), snapshot.get("pose") or {}
    reason = pose.get("reason")
    if reason in {"process_exited", "process_unresponsive", "stopping"}:
        return "计算进程暂不可用 · 已隐藏模型，请查看连接状态；不要反复归零"
    if not runtime.get("connected"):
        return "当前未连接 · 已隐藏实时模型；重新连接属于新会话"
    if reason == "calibration_required":
        return "本次连接尚未标定 · 数据到达不等于已有手势，请完成本次三步标定"
    rejections = pose.get("calibration_rejections", {})
    if reason == "calibration_epoch_changed" or "filter_epoch_changed_recalibration_required" in rejections.values():
        return "滤波状态已重置，相关标定失效 · 已隐藏模型；仅重设伸直参考不能恢复，请查看节点反馈"
    if reason == "missing_or_stale_root":
        return "手掌数据暂时过期或缺失 · 已隐藏模型，等待新数据；暂不重做标定"
    if pose.get("solver_success") is False and reason != "missing_or_invalid_root":
        return "当前姿态求解不可用 · 已隐藏模型，请查看求解 / 标定反馈"
    return "当前姿态结果不可用或已过期 · 已隐藏模型，等待最新结果；暂不重做标定"


def demonstration_root(step, model_axes):
    """Explicit static instruction pose, never a substitute for measured data."""
    if step == "neutral":
        return Rotation.identity().as_quat().tolist()
    name = {"fingers_down": "pitch_axis", "thumb_up": "roll_axis"}[step]
    axis = np.asarray(model_axes[name], dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
        raise ValueError("Invalid model motion axis")
    side = 1 if model_axes.get("hand", "right") == "right" else -1
    return Rotation.from_rotvec(side * axis / np.linalg.norm(axis) * np.deg2rad(40)).as_quat().tolist()


def display_basis(model_axes):
    """Proper rotation: forward→+Y, dorsal→+Z, thumb→-X (right)/+X (left).

    The same basis applies to every frame, never cancelling measured root pose.
    """
    forward = np.asarray(model_axes["roll_axis"], dtype=float)
    thumbward = np.asarray(model_axes["pitch_axis"], dtype=float)
    side = 1 if model_axes.get("hand", "right") == "right" else -1
    up = side * np.cross(forward, thumbward)
    # Reflecting anatomy changes the cross-product sign; flip X and Z together
    # to keep this a proper rotation, not a mirrored display of measured data.
    return np.stack((-side * thumbward, forward, up))
