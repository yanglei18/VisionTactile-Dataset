"""Explicit functional-axis adaptation AFTER original FSGlove inference.

This does not estimate anatomical angle truth, alter the official installation
calibrator, or replace its pose solver. A manually declared neutral/flexion
capture defines a functional direction. SciPy/NumPy estimate its principal axis;
the actual MIT-licensed qmt.quatProject supplies hinge projection and residuals.
Only measured two-segment joints can be adapted. Root, thumb MCP/CMC, missing
slots, unselected fingers, raw IMU records and the official result stay intact.

A single flexion axis cannot identify rotation of the remaining coordinate
basis around that axis. MCP uses a minimum-rotation basis convention, preserves
spread/bend and suppresses twist; that convention is not anatomical calibration.
No gain, clipping, time filter, online learning, or automatic recentering occurs.
"""
from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import importlib
import importlib.metadata
import inspect
import math
from pathlib import Path
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from .functional_quality import (FUNCTIONAL_REFERENCE_P95_MAX_DEG,
                                 functional_fit_acceptance)


KIND = "qmt_functional_axis_adapter_v1"
SUPPRESSED_ROTATION_WARNING_DEG = 10.
FINGER_SLOTS = {"index": (1, 2), "middle": (4, 5), "ring": (10, 11),
                "little": (7, 8), "thumb": (15,)}
_MCP = {1, 4, 7, 10}
_ANCHORS = ("session_id", "upstream_state_sha256", "config_sha256", "rest_sha256")


class FunctionalDependencyError(RuntimeError):
    """The optional, real QMT projection dependency cannot be used."""


class FunctionalCalibrationRejected(ValueError):
    """No candidate can be installed; preserve inspectable rejection evidence."""

    def __init__(self, message, *, diagnostics, rejected_slots):
        super().__init__(message)
        self.diagnostics = deepcopy(diagnostics)
        self.rejected_slots = deepcopy(rejected_slots)


def _qmt():
    try:
        module = importlib.import_module("qmt")
        if not callable(getattr(module, "quatProject", None)):
            raise AttributeError("qmt.quatProject is unavailable")
        return module
    except (ImportError, AttributeError) as exc:
        raise FunctionalDependencyError(
            "功能轴适配需要独立环境中的 qmt.quatProject；不会改用自写投影或旧求解器。"
        ) from exc


def _project(rotations, axis):
    # QMT is scalar-first wxyz; the rest of this adapter uses SciPy xyzw.
    quaternion = rotations.as_quat()[..., [3, 0, 1, 2]]
    direction = np.asarray(axis, dtype=float)
    direction = direction / np.linalg.norm(direction)
    # At a 180-degree rotation perpendicular to the target axis every hinge
    # angle is equally distant. Do not present atan2(0, 0) as a measured angle.
    identifiable = np.hypot(quaternion[..., 0], np.sum(quaternion[..., 1:] * direction, axis=-1))
    if np.any(identifiable < 1e-10):
        raise ValueError("接近 180° 正交旋转，铰链投影不唯一；保留未适配结果")
    result = _qmt().quatProject(quaternion, axis,
                               debug=False, plot=False)
    angle = np.asarray(result["projAngle"], dtype=float)
    residual = np.abs(np.rad2deg(np.asarray(result["resAngle"], dtype=float)))
    if not np.isfinite(angle).all() or not np.isfinite(residual).all():
        raise ValueError("QMT 投影返回非有限角度，保留未适配结果")
    return angle, residual


def _qmt_provenance():
    function = _qmt().quatProject
    source = inspect.getsourcefile(function)
    try:
        version = importlib.metadata.version("qmt")
    except importlib.metadata.PackageNotFoundError:
        version = None
    return dict(package="qmt", version=version, function="qmt.quatProject",
                source_file=source,
                source_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest() if source else None,
                scope="MIT quaternion projection, not jointAxisEstHingeOlsson")


def _checked_binding(binding):
    if not isinstance(binding, dict) or any(not isinstance(binding.get(k), str) or not binding[k] for k in _ANCHORS):
        raise ValueError("功能轴必须绑定当前 session、官方安装状态、config 和 MANO rest 摘要")
    if not isinstance(binding.get("nodes_epochs"), dict) or not isinstance(binding.get("dependency_nodes_by_slot"), dict):
        raise ValueError("功能轴 binding 必须包含 nodes_epochs 和 dependency_nodes_by_slot")
    return deepcopy(binding)


def _dependencies(binding, slot):
    values = binding["dependency_nodes_by_slot"].get(str(slot))
    if not isinstance(values, (list, tuple)) or len(values) != 2 or len(set(values)) != 2:
        raise ValueError(f"关节 {slot} 必须有两个不同的父/子依赖节点，不能借用缺测 CMC")
    if any(not isinstance(node, str) or type(binding["nodes_epochs"].get(node)) is not int
           or binding["nodes_epochs"][node] < 0 for node in values):
        raise ValueError(f"关节 {slot} 的依赖节点或滤波 epoch 缺失")
    return list(values)


def _same_binding(saved, current, slot):
    try:
        if any(saved.get(key) != current.get(key) for key in _ANCHORS):
            return False
        old, new = _dependencies(saved, slot), _dependencies(current, slot)
        return old == new and all(saved["nodes_epochs"][n] == current["nodes_epochs"][n] for n in old)
    except (ValueError, TypeError, KeyError, AttributeError):
        return False


def _array(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} 必须是有限的 {shape} 数组")
    return array


def _rotation(value, name):
    array = _array(value, (4,), name)
    if not np.isclose(np.linalg.norm(array), 1., atol=1e-6):
        raise ValueError(f"{name} 必须是单位 xyzw quaternion")
    return Rotation.from_quat(array)


@lru_cache(maxsize=128)
def _reference_rotations(neutral_xyzw, basis_xyzw):
    """Content-keyed immutable references; in-place profile edits cannot go stale.

    The bounded cache stores only validated rotations, never caller-owned
    dictionaries or measured values. Keep multiplication order unchanged.
    """
    neutral = _rotation(neutral_xyzw, "功能轴中立参考")
    basis = _rotation(basis_xyzw, "功能轴基底")
    return neutral.inv(), basis, basis.inv()


def _collect(results, slot, dependencies, seen):
    values, rejected = [], 0
    for result in results:
        try:
            stamp = result.get("timestamp_s")
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                    or not math.isfinite(stamp) or stamp in seen):
                raise ValueError("无效或重复时间戳")
            if result.get("pose_available", True) is not True:
                raise ValueError("官方姿态不可用")
            mask = np.asarray(result.get("joint_observed_mask"))
            if mask.shape != (16,) or mask.dtype != np.bool_ or not mask[slot]:
                raise ValueError("缺少实测父/子骨段")
            rejections = result.get("calibration_rejections", {})
            if any(node in rejections for node in dependencies):
                raise ValueError("依赖节点已被基础标定拒绝")
            angles = _array(result["anatomical_euler_rad"], (16, 3), "官方局部解剖角")
            values.append(Rotation.from_euler("XYZ", angles[slot]).as_quat())
            seen.add(stamp)
        except (ValueError, KeyError, TypeError, AttributeError):
            rejected += 1
    if len(values) < 3:
        raise ValueError(f"关节 {slot} 有效独立样本仅 {len(values)} 个（跳过 {rejected}）；至少需要 3 个实测父/子样本")
    return Rotation.from_quat(values), rejected


def _principal_axis(vectors):
    _, singular, right = np.linalg.svd(vectors, full_matrices=False)
    energy = float(singular[0] ** 2 / np.sum(singular ** 2))
    return right[0], energy


def _orient_flexion_axis(axis, vectors):
    """Orient the fitted line using the whole declared neutral-to-flexion capture.

    SVD estimates an unsigned line. A small first backwards excursion must not
    invert every subsequent joint estimate. The dominant squared projection
    determines the positive side, without changing the fitted line or angular
    gain. Equally supported opposite excursions do not identify a direction.
    This is a capture convention, not an anatomical measurement.
    """
    projections = np.asarray(vectors) @ axis
    positive = float(np.sum(np.square(projections[projections > 0.])))
    negative = float(np.sum(np.square(projections[projections < 0.])))
    total = positive + negative
    if total <= 0. or abs(positive - negative) <= total * 1e-9:
        raise ValueError("弯曲正方向不明确：伸直参考两侧动作支持相同；请从自然伸直到弯曲再回位")
    if negative > positive:
        axis = -axis
    return axis, dict(positive_energy_fraction=max(positive, negative) / total,
                      opposite_energy_fraction=min(positive, negative) / total,
                      samples=len(projections), uses_first_sample_only=False)


def _minimum_basis(axis):
    # SciPy's one-vector alignment is the minimum rotation mapping +Z to axis.
    # It deliberately does not claim to measure the unobservable basis twist.
    basis, _ = Rotation.align_vectors(axis, [0., 0., 1.])
    return basis


def fit_functional_finger(neutral_results, motion_results, *, finger, binding, previous=None):
    """Fit eligible joints independently; reject only a failing joint.

    Results must be unadapted original-core outputs, with fresh/epoch-checked
    observations supplied by the caller. Timestamp deduplication here is an
    additional safeguard, not a replacement for input sample identity checks.
    A fully failed new capture raises ValueError; it never overwrites previous
    calibration. Partial success can retain a previous same-binding joint.
    """
    if finger not in FINGER_SLOTS:
        raise ValueError("finger 必须是 index/middle/ring/little/thumb")
    binding = _checked_binding(binding)
    if not isinstance(neutral_results, (list, tuple)) or not isinstance(motion_results, (list, tuple)):
        raise ValueError("请提供明确采集的静止与单指弯伸官方结果列表")
    entry = dict(kind=KIND, finger=finger, binding=binding, slot_refs={}, diagnostics={},
                 rejected_slots={}, warnings=[], qmt=None,
                 anatomical_accuracy_verified=False, relative_yaw_drift_corrected=False,
                 basis_twist_observed=False, depends_on_previous_frame=False,
                 angle_gain=1., thumb_cmc_reconstructed=False,
                 basis_convention="minimum_rotation_from_model_positive_z_to_fitted_flexion_axis")
    fitted = 0
    for slot in FINGER_SLOTS[finger]:
        candidate_diagnostic = None
        try:
            dependencies = _dependencies(binding, slot)
            seen = set()
            neutral, skipped_neutral = _collect(neutral_results, slot, dependencies, seen)
            motion, skipped_motion = _collect(motion_results, slot, dependencies, seen)
            reference = neutral.mean()
            neutral_p95 = float(np.percentile(np.rad2deg((reference.inv() * neutral).magnitude()), 95))
            if neutral_p95 > FUNCTIONAL_REFERENCE_P95_MAX_DEG:
                raise ValueError(f"关节 {slot} 静止参考不稳定（p95 {neutral_p95:.1f}° > "
                                 f"{FUNCTIONAL_REFERENCE_P95_MAX_DEG:g}°）；请保持静止重采")
            delta = reference.inv() * motion
            angles = np.rad2deg(delta.magnitude())
            if np.any(angles > 170.):
                raise ValueError(f"关节 {slot} 存在超过 170° 的相对转动，接近 π 的方向有歧义；请缩小动作重采")
            moving = delta.as_rotvec()[angles >= 8.]
            if len(moving) < 3:
                raise ValueError(f"关节 {slot} 超过 8° 的有效弯伸只有 {len(moving)} 个样本；请保持掌部稳定并多弯伸几次")
            _, overall_energy = _principal_axis(moving)
            train = np.arange(len(moving)) % 3 != 2
            axis, train_energy = _principal_axis(moving[train])
            if min(overall_energy, train_energy) < .75:
                energy = min(overall_energy, train_energy)
                raise ValueError(f"关节 {slot} 混合转轴较强（主轴能量 {energy:.6f}，低于 0.75 要求）；请只做该指屈伸")
            # The interleaved training set spans the full capture. Use its
            # directional support, not an arbitrary initial sample. Held-out
            # samples determine neither the fitted line nor its sign.
            axis, direction_support = _orient_flexion_axis(axis, moving[train])
            basis = _minimum_basis(axis)
            _, all_residual = _project(delta, axis)
            _, holdout_residual = _project(Rotation.from_rotvec(moving[~train]), axis)
            holdout_p95 = float(np.percentile(holdout_residual, 95))
            moving_residual_p95 = float(np.percentile(all_residual[angles >= 8.], 95))
            split_angle = None
            if len(moving) >= 6:
                a1, _ = _principal_axis(moving[:len(moving)//2])
                a2, _ = _principal_axis(moving[len(moving)//2:])
                split_angle = float(np.rad2deg(np.arccos(np.clip(abs(np.dot(a1, a2)), 0., 1.))))
            messages = []
            if max(holdout_p95, moving_residual_p95) > SUPPRESSED_ROTATION_WARNING_DEG:
                messages.append(f"关节 {slot} 偏轴残差较大（留出 p95 {holdout_p95:.1f}°）；不能据此认定安装或滑动已修复")
            if split_angle is not None and split_angle > 15.:
                messages.append(f"关节 {slot} 前后动作轴相差 {split_angle:.1f}°；可能含非铰链动作、滑动或滤波/同步误差")
            mode = "mcp_two_dof" if slot in _MCP else "hinge_projection"
            acceptance = functional_fit_acceptance(mode, holdout_p95_deg=holdout_p95,
                                                   moving_p95_deg=moving_residual_p95)
            candidate_diagnostic = dict(
                status="candidate", mode=mode, neutral_samples=len(neutral), motion_samples=len(motion),
                skipped_neutral=skipped_neutral, skipped_motion=skipped_motion,
                neutral_p95_deg=neutral_p95, moving_samples=len(moving),
                principal_axis_energy=overall_energy, train_principal_axis_energy=train_energy,
                train_samples=int(np.sum(train)), holdout_samples=int(np.sum(~train)),
                direction_support=direction_support,
                holdout_p95_residual_deg=holdout_p95, moving_p95_residual_deg=moving_residual_p95,
                split_half_axis_line_angle_deg=split_angle,
                observed_range_deg=[float(angles.min()), float(angles.max())], warnings=messages,
                holdout_used_for_axis_fitting=False,
                validation_scope="interleaved_samples_same_capture_not_independent_session",
                basis_twist_observed=False, anatomical_angle_ground_truth_available=False,
                acceptance=acceptance)
            if acceptance["status"] != "accepted":
                raise ValueError(
                    f"关节 {slot} 弯曲轴未通过模型验收（留出 p95 {holdout_p95:.1f}°，"
                    f"运动 p95 {moving_residual_p95:.1f}°，允许 {acceptance['residual_limit_deg']:g}°）；"
                    "本次不替换该关节参数。请单指补采，从伸直到舒适弯曲再回位，减少同时侧摆。")
            entry["slot_refs"][str(slot)] = dict(
                neutral_local_xyzw=reference.as_quat().tolist(), axis_local=axis.tolist(),
                axis_basis_xyzw=basis.as_quat().tolist(), mode=mode, dependency_nodes=dependencies,
                sign_convention="dominant_capture_projection_energy_positive",
                source="official_observed_parent_child_functional_motion_not_anatomical_truth")
            entry["diagnostics"][str(slot)] = {**candidate_diagnostic, "status": "fitted"}
            entry["warnings"].extend(messages)
            fitted += 1
        except (ValueError, KeyError, TypeError, np.linalg.LinAlgError) as exc:
            entry["rejected_slots"][str(slot)] = str(exc)
            old = (previous or {}).get("slot_refs", {}).get(str(slot))
            if (old and previous.get("kind") == KIND and previous.get("finger") == finger
                    and _same_binding(previous.get("binding", {}), binding, slot)):
                entry["slot_refs"][str(slot)] = deepcopy(old)
                entry["diagnostics"][str(slot)] = dict(status="retained_previous", reason=str(exc))
            else:
                entry["diagnostics"][str(slot)] = dict(status="rejected", reason=str(exc))
            if candidate_diagnostic is not None:
                entry["diagnostics"][str(slot)]["candidate"] = candidate_diagnostic
    if not fitted:
        raise FunctionalCalibrationRejected(
            "本次没有可更新的功能轴；原标定保持不变。" + "；".join(entry["rejected_slots"].values()),
            diagnostics=entry["diagnostics"], rejected_slots=entry["rejected_slots"])
    entry["qmt"] = _qmt_provenance()
    return entry


def apply_functional_fingers(anatomical_euler_rad, entries, joint_observed_mask, *, binding, slots=None):
    """Return a copy of current local angles and explicit adaptation diagnostics.

    ``entries`` maps finger names to entries returned by fit_functional_finger.
    Unavailable joints pass through unchanged, not a previous adapted value.
    The caller retains original official angles and handles MANO composition.
    """
    original = _array(anatomical_euler_rad, (16, 3), "官方局部解剖角")
    output = original.copy()
    mask = np.asarray(joint_observed_mask)
    if mask.shape != (16,) or mask.dtype != np.bool_:
        raise ValueError("joint_observed_mask 必须包含 16 个布尔值")
    binding = _checked_binding(binding)
    if not isinstance(entries, dict):
        raise ValueError("entries 必须按 finger 名称存储功能轴条目")
    diagnostic = dict(kind=KIND, applied_slots=[], skipped_slots={}, residual_deg_by_slot={},
                      warnings=[], projection_deg_by_slot={},
                      anatomical_accuracy_verified=False, relative_yaw_drift_corrected=False,
                      residual_is_anatomical_error=False, depends_on_previous_frame=False,
                      basis_twist_observed=False, angle_gain=1.)
    pending = []
    for finger, entry in entries.items():
        if (finger not in FINGER_SLOTS or not isinstance(entry, dict)
                or entry.get("kind") != KIND or entry.get("finger") != finger):
            diagnostic["skipped_slots"][str(finger)] = "unsupported_or_invalid_functional_entry"
            continue
        for slot in FINGER_SLOTS[finger]:
            if slots is not None and slot not in slots:
                continue
            key = str(slot)
            reference = entry.get("slot_refs", {}).get(key)
            if reference is None:
                continue
            if not mask[slot]:
                diagnostic["skipped_slots"][key] = "parent_or_child_not_currently_observed"
                continue
            if not _same_binding(entry.get("binding", {}), binding, slot):
                diagnostic["skipped_slots"][key] = "session_installation_config_rest_or_dependency_epoch_mismatch"
                continue
            try:
                expected_mode = "mcp_two_dof" if slot in _MCP else "hinge_projection"
                if (reference.get("mode") != expected_mode
                        or reference.get("dependency_nodes") != _dependencies(binding, slot)):
                    raise ValueError("功能轴条目关节模式或依赖不匹配")
                # Validate shape/finiteness every call, then use a bounded
                # content-keyed cache for static rotation construction/inverses.
                neutral_key = tuple(_array(reference["neutral_local_xyzw"], (4,), "功能轴中立参考"))
                basis_key = tuple(_array(reference["axis_basis_xyzw"], (4,), "功能轴基底"))
                neutral_inverse, basis, basis_inverse = _reference_rotations(neutral_key, basis_key)
                relative = neutral_inverse * Rotation.from_euler("XYZ", original[slot])
                retargeted = basis_inverse * relative * basis
                quaternion = retargeted.as_quat()
                # The projection axis is Z for every retargeted reference.
                # Isolate ambiguity before batching so a bad slot cannot veto
                # another joint's projection or cause unnecessary scalar work.
                if np.hypot(quaternion[3], quaternion[2]) < 1e-10:
                    raise ValueError("接近 180° 正交旋转，铰链投影不唯一；保留未适配结果")
                pending.append((slot, key, expected_mode, retargeted, quaternion))
            except (ValueError, KeyError, TypeError) as exc:
                diagnostic["skipped_slots"][key] = str(exc)

    projections = None
    if pending:
        try:
            # The exact installed MIT QMT implementation already accepts a
            # quaternion stack. Share its validation/vectorization overhead;
            # this is not a replacement formula or a change of angle convention.
            projections = _project(Rotation.from_quat(np.stack([item[4] for item in pending])), [0., 0., 1.])
            if any(values.shape != (len(pending),) for values in projections):
                projections = None
        except (ValueError, KeyError, TypeError):
            # A malformed/nonfinite batch result must retain the original
            # per-slot rejection behavior, including success of healthy slots.
            pass
    for index, (slot, key, expected_mode, retargeted, _) in enumerate(pending):
        try:
            projection, off_axis = (_project(retargeted, [0., 0., 1.]) if projections is None
                                    else (projections[0][index], projections[1][index]))
            if expected_mode == "hinge_projection":
                adjusted = np.array([0., 0., float(projection)])
                residual = float(off_axis)
            else:
                with warnings.catch_warnings(record=True) as caught:
                    warnings.simplefilter("always", UserWarning)
                    adjusted = retargeted.as_euler("XYZ")
                if caught:
                    diagnostic["warnings"].append(f"关节 {slot} 接近欧拉角奇异位置；分轴角属于坐标约定")
                adjusted[0] = 0.
                residual = float(np.rad2deg((Rotation.from_euler("XYZ", adjusted).inv() * retargeted).magnitude()))
            output[slot] = adjusted
            diagnostic["applied_slots"].append(slot)
            diagnostic["residual_deg_by_slot"][key] = residual
            diagnostic["projection_deg_by_slot"][key] = float(np.rad2deg(projection))
            if residual > SUPPRESSED_ROTATION_WARNING_DEG:
                diagnostic["warnings"].append(
                    f"关节 {slot} 被抑制旋转残差 {residual:.1f}°；可能是非铰链动作、安装变化或其他误差，不能认定已修复")
        except (ValueError, KeyError, TypeError) as exc:
            diagnostic["skipped_slots"][key] = str(exc)
    diagnostic["applied_slots"].sort()
    return output, diagnostic
