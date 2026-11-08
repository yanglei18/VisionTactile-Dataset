"""One-shot relative heading observation from an explicitly confirmed straight hand.

Gravity does not identify finger spread about the vertical. Only a user-declared
neutral shape supplies the missing observation. Between such captures the offsets
are fixed, including during a real spread, a wrist turn or another still pose.
"""
from __future__ import annotations

from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import quat
from .functional_quality import FUNCTIONAL_REFERENCE_P95_MAX_DEG
from .upstream_functional import FINGER_SLOTS, KIND, _same_binding


METHOD = "confirmed_neutral_relative_world_z_v1"
_VERTICAL = np.array([0., 0., 1.])
_MIN_SAMPLES = 8
_MIN_SPAN_S = .75
_MAX_SKEW_S = .03
_MAX_GAP_S = .25
# A yaw correction cannot remove this tilt error. A pronounced bend should
# fail the entire observation rather than become one finger's heading zero.
_MAX_TILT_ERROR_DEG = 20.


def _finite(v):
    return type(v) in (int, float) and math.isfinite(v)


def _binding(profile):
    return dict(session_id=profile["session_id"], config_sha256=profile["config_sha256"],
                rest_sha256=profile["rest_sha256"], upstream_state_sha256=profile["upstream_state_sha256"],
                nodes_epochs={n: v["epoch"] for n, v in profile["nodes"].items()})


def _capture_samples(rows, profile, nodes, stale_after):
    samples, times = [], []
    last_ids = np.full(len(nodes), -1)
    for frame in rows:
        if frame.get("session_id") != profile["session_id"]:
            raise ValueError("伸直参考不是本次会话的采样。")
        stamp = frame.get("timestamp_s")
        if not _finite(stamp):
            raise ValueError("伸直参考缺少采样时间。")
        rotations, ids, ages = [], [], []
        for node in nodes:
            data = frame.get("nodes", {}).get(node)
            if not data:
                continue
            if data.get("epoch") != profile["nodes"][node]["epoch"]:
                raise ValueError(f"节点 {node} 的滤波参考已重建，请重新完成基础标定。")
            index, age = data.get("sample_index"), data.get("age_s")
            guard = data.get("input_filter") or {}
            if (type(index) is int and index > 0 and _finite(age) and 0 <= age <= stale_after
                    and data.get("orientation_valid") is not False
                    and data.get("input_health", {}).get("healthy") is not False
                    and not guard.get("gyro_rejected") and not guard.get("accel_rejected")):
                rotations.append(quat(data.get("quat_xyzw"), f"节点 {node}").as_quat())
                ids.append(index)
                ages.append(age)
        if len(ids) != len(nodes) or np.ptp(ages) > _MAX_SKEW_S or np.any(ids <= last_ids):
            continue
        times.append(stamp - max(ages))
        samples.append(rotations)
        last_ids = np.asarray(ids)
    intervals = np.diff(times)
    runs = np.split(np.asarray(times), np.flatnonzero(intervals > _MAX_GAP_S) + 1)
    if (np.any(intervals <= 0) or not any(len(run) >= _MIN_SAMPLES and run[-1] - run[0] >= _MIN_SPAN_S
                                           for run in runs)):
        raise ValueError("伸直参考缺少连续、同步的新样本。请保持五指自然伸直，再试一次。")
    return np.array(samples), times[-1] - times[0]


def fit_confirmed_heading(rows, profile, calibrator, binding, stale_after):
    nodes = tuple(profile["nodes"])
    slots = [profile["nodes"][n]["slot"] for n in nodes]
    if 0 not in slots or len(nodes) < 2:
        raise ValueError("伸直参考需要有效的手掌和手指标定。")
    samples, duration = _capture_samples(rows, profile, nodes, stale_after)
    means = []
    for index, node in enumerate(nodes):
        rotations = Rotation.from_quat(samples[:, index])
        mean = rotations.mean()
        excursion = np.rad2deg((mean.inv() * rotations).magnitude())
        if np.percentile(excursion, 95) > FUNCTIONAL_REFERENCE_P95_MAX_DEG:
            raise ValueError(f"节点 {node} 在采样期间仍在运动。请保持五指自然伸直。")
        means.append(mean.as_quat())
    measured = Rotation.from_quat(means)
    root = slots.index(0)
    base = Rotation.from_quat(calibrator._base_quat)
    installation = Rotation.from_rotvec(calibrator._global_installation_error) * base.inv()
    local = Rotation.from_rotvec(calibrator._local_installation_error)
    rest = Rotation.from_rotvec(calibrator._delta_rot_fa_rotvec)
    # In the official common frame world vertical is +Y. Every participating
    # sensor must map gravity to the same direction for a fixed raw-Z offset.
    verticals = installation[slots].apply(_VERTICAL)
    if np.max(np.linalg.norm(verticals - verticals[root], axis=1)) > 1e-4:
        raise ValueError("安装参考的竖直轴不一致，无法只校正航向。")
    zero = [Rotation.identity() for _ in range(16)]
    entries = profile.get("functional_fingers", {}) if profile.get("functional_enabled", True) else {}
    for finger, entry in entries.items():
        for slot in FINGER_SLOTS.get(finger, ()):
            ref = entry.get("slot_refs", {}).get(str(slot))
            if ref and entry.get("kind") == KIND and _same_binding(entry.get("binding"), binding, slot):
                zero[slot] = quat(ref["neutral_local_xyzw"], "已标定的伸直参考")
    target = [None] * 16
    target[0] = installation[0] * measured[root] * local[0].inv()
    for slot in calibrator._fk_seq:
        target[slot] = target[calibrator._joint_fa[slot]] * rest[slot] * zero[slot]
    desired = installation[slots].inv() * Rotation.from_quat([target[s].as_quat() for s in slots]) * local[slots]
    # Closest rotation *about world Z* to desired * measured^-1. Projecting
    # the relative quaternion onto (w,z) minimizes geodesic error over SO(2).
    relative = desired * measured.inv()
    tilt_error = np.rad2deg(np.arccos(np.clip(relative.apply(_VERTICAL)[:, 2], -1., 1.)))
    worst = np.argmax(tilt_error)
    if tilt_error[worst] > _MAX_TILT_ERROR_DEG:
        raise ValueError(f"节点 {nodes[worst]} 相对伸直姿势的倾角仍差 {tilt_error[worst]:.0f}°。请伸直五指后再试。")
    q = relative.as_quat()
    offsets = np.angle((q[:, 3] + 1j * q[:, 2]) ** 2)
    offsets[root] = 0.
    anchor = dict(method=METHOD, binding=_binding(profile), offsets_rad_by_node=dict(zip(nodes, offsets.tolist())),
                  sample_count=len(samples), duration_s=float(duration), max_tilt_error_deg=float(tilt_error.max()))
    return {**deepcopy(profile), "neutral_heading": anchor}


def apply_heading(frame, profile):
    """Return a shallow, corrected solver input; untouched raw streams remain owned by acquisition."""
    anchor = profile.get("neutral_heading")
    if anchor is None:
        return frame
    values = anchor.get("offsets_rad_by_node", {}) if isinstance(anchor, dict) else {}
    if (not isinstance(anchor, dict) or anchor.get("method") != METHOD or anchor.get("binding") != _binding(profile)
            or not isinstance(values, dict) or values.keys() != profile["nodes"].keys()
            or any(not _finite(v) or abs(v) > math.pi or (profile["nodes"][n]["slot"] == 0 and v != 0.)
                   for n, v in values.items())):
        raise ValueError("已保存的伸直航向参考与当前标定不匹配。")
    patched = dict(frame.get("nodes", {}))
    for node, angle in values.items():
        data = patched.get(node)
        if not angle or not isinstance(data, dict) or data.get("epoch") != profile["nodes"][node]["epoch"]:
            continue
        raw = data.get("quat_xyzw")
        if not isinstance(raw, (tuple, list, np.ndarray)):
            continue  # Preserve the original input for existing per-node rejection.
        try:
            if np.shape(raw) != (4,):
                continue
            x, y, z, w = map(float, raw)
        except (TypeError, ValueError, OverflowError):
            continue
        # Hamilton product Rz(angle) * raw in xyzw. Keeping the source norm
        # intact also keeps the downstream orientation-quality checks intact.
        c, s = math.cos(angle / 2.), math.sin(angle / 2.)
        patched[node] = {**data, "quat_xyzw": [c * x - s * y, c * y + s * x,
                                               c * z + s * w, c * w - s * z]}
    return {**frame, "nodes": patched}
