"""Explicit motion-axis retargeting, NOT recovered anatomical world rotations.

Conjugation preserves rotation magnitudes relative to an explicitly captured
neutral. There is no gain, temporal filter, true CMC decomposition, or drift
correction. The user must perform the instructed straight-to-bent movement;
its direction and observed range are not anatomical ground truth.
"""
from __future__ import annotations

from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import apply_calibration, quat, validate_rest
from .config import PARENTS, digest

_JOINTS = {"thumb": (14, 15), "little": (7, 8)}
_KIND = "functional_axis_retarget_not_anatomical_world_pose"


def _locals(rest):
    rotations = Rotation.from_quat(validate_rest(rest))
    return [rotations[0]] + [rotations[PARENTS[i]].inv() * rotations[i] for i in range(1, 16)]


def _parent(slot, rotations, local, prior, cmc_mapped):
    parent = PARENTS[slot]
    if parent in rotations:
        return rotations[parent]
    if slot == 14 and not cmc_mapped and 0 in rotations:
        return rotations[0] * local[13] * Rotation.from_euler("XYZ", prior)
    raise ValueError(f"关节 {slot} 缺少父骨段 {parent}，不能构造独立相对运动")


def _collect(frames, profile, slot, required, local, prior, cmc_mapped, seen):
    values = []
    for frame in frames:
        nodes = frame.get("nodes", {})
        usable = {}
        for node in required:
            data = nodes.get(node)
            if not isinstance(data, dict):
                break
            if data.get("epoch") != profile["nodes"][node]["epoch"]:
                raise ValueError(f"节点 {node} 滤波 epoch 已改变，请重新采集")
            age, index = data.get("age_s"), data.get("sample_index")
            if (isinstance(age, bool) or not isinstance(age, (int, float))
                    or not math.isfinite(age) or not 0 <= age <= profile["stale_after_s"]
                    or type(index) is not int or index < 1 or index in seen[node]):
                break
            try:
                quat(data.get("quat_xyzw"))
            except (ValueError, TypeError):
                break
            usable[node] = data
        if len(usable) != len(required):
            continue
        observations, _ = apply_calibration({"nodes": usable}, profile)
        rotations = {profile["nodes"][node]["slot"]: quat(value) for node, value in observations.items()}
        parent = _parent(slot, rotations, local, prior, cmc_mapped)
        values.append((local[slot].inv() * parent.inv() * rotations[slot]).as_quat())
        for node, data in usable.items():
            seen[node].add(data["sample_index"])
    if len(values) < 3:
        raise ValueError(f"关节 {slot} 不足 3 个新鲜独立父/子样本（有效 {len(values)}）；重复或冻结帧不计数")
    return Rotation.from_quat(values)


def fit_functional_finger(neutral_frames, motion_frames, profile, rest, config, *, finger, session_id):
    """Fit each requested joint independently; no successful joint raises ValueError.

    A failed joint retains an old reference only from the same session, rest,
    and dependency epochs. The caller owns capture timing and profile versioning.
    """
    if finger not in _JOINTS:
        raise ValueError("功能轴只支持 thumb 或 little")
    if not session_id or profile.get("session_id") != session_id:
        raise ValueError("功能轴必须使用当前会话的基础标定")
    if not isinstance(neutral_frames, (list, tuple)) or not isinstance(motion_frames, (list, tuple)):
        raise ValueError("请先采集中立姿态与单指弯曲动作")
    if any(not isinstance(frame, dict) or not isinstance(frame.get("nodes"), dict)
           for frame in (*neutral_frames, *motion_frames)):
        raise ValueError("功能轴采集帧必须包含 nodes 对象")
    local = _locals(rest)
    rest_hash = digest(rest)
    if profile.get("rest_sha256", rest_hash) != rest_hash:
        raise ValueError("功能轴与基础标定的模型 rest 不一致")
    inverse = {slot: node for node, slot in config["node_to_slot"].items()}
    prior = np.asarray(config.get("thumb_cmc_prior_rad", [0., 0., 0.]), dtype=float)
    if prior.shape != (3,) or not np.isfinite(prior).all():
        raise ValueError("thumb_cmc_prior_rad 必须是三个有限角度")
    entry = dict(kind=_KIND, finger=finger, session_id=session_id, rest_sha256=rest_hash,
                 slot_refs={}, nodes_epochs={}, rejected_slots={}, diagnostics={},
                 thumb_cmc_prior_rad=prior.tolist(), cmc_mapped=13 in inverse,
                 observed_range_is_anatomical_limit=False)
    old = profile.get("functional_fingers", {}).get(finger, {})
    for slot in _JOINTS[finger]:
        try:
            parent_slot = 0 if slot == 14 and 13 not in inverse else PARENTS[slot]
            required = [inverse[parent_slot], inverse[slot]]
            for node in required:
                if node not in profile["nodes"] or profile["nodes"][node]["slot"] != config["node_to_slot"][node]:
                    raise ValueError(f"节点 {node} 没有有效基础标定")
            seen = {node: set() for node in required}
            neutral = _collect(neutral_frames, profile, slot, required, local, prior, 13 in inverse, seen)
            motion = _collect(motion_frames, profile, slot, required, local, prior, 13 in inverse, seen)
            reference = neutral.mean()
            neutral_p95 = float(np.rad2deg(np.percentile((reference.inv() * neutral).magnitude(), 95)))
            if neutral_p95 > 5.:
                raise ValueError(f"关节 {slot} 中立姿态不稳定（p95 {neutral_p95:.1f}°），请保持静止重采")
            vectors = (reference.inv() * motion).as_rotvec()
            magnitudes = np.linalg.norm(vectors, axis=1)
            moving = vectors[magnitudes >= np.deg2rad(10.)]
            if len(moving) < 3:
                raise ValueError(f"关节 {slot} 有效相对弯曲不足 10° 或独立运动样本不足 3 个")
            _, singular, right = np.linalg.svd(moving, full_matrices=False)
            energy = float(singular[0] ** 2 / np.sum(singular ** 2))
            if energy < .8:
                raise ValueError(f"关节 {slot} 运动轴不一致（主轴能量 {energy:.2f} < 0.80），请单向弯曲重采")
            axis = right[0]
            projection = moving @ axis
            weighted_sign = float(np.sum(projection * np.abs(projection)))
            if weighted_sign < 0:
                axis = -axis
                projection = -projection
            positive_fraction = float(np.sum(np.maximum(projection, 0.) ** 2) / np.sum(projection ** 2))
            if positive_fraction < .8:
                raise ValueError(f"关节 {slot} 正反向运动混合，无法确认伸直到弯曲的方向")
            # Minimum rotation maps model +Z to the measured motion axis. The
            # remaining basis twist is a convention, not an extra observation.
            z = np.array([0., 0., 1.])
            cross = np.cross(z, axis)
            if axis[2] < -1. + 1e-10:
                basis = Rotation.from_rotvec([np.pi, 0., 0.])
            else:
                basis_q = np.array([*cross, 1. + axis[2]])
                basis = quat(basis_q / np.linalg.norm(basis_q))
            entry["slot_refs"][str(slot)] = dict(
                neutral_local_xyzw=reference.as_quat().tolist(), axis_basis_xyzw=basis.as_quat().tolist(),
                source="inferred_cmc_prior_parent" if slot == 14 and 13 not in inverse else "observed_parent_child_relative_motion",
                dependency_nodes=required,
            )
            entry["nodes_epochs"].update({node: profile["nodes"][node]["epoch"] for node in required})
            entry["diagnostics"][str(slot)] = dict(status="fitted", neutral_samples=len(neutral), motion_samples=len(motion),
                moving_samples=len(moving), neutral_p95_deg=neutral_p95, principal_axis_energy=energy,
                positive_projection_energy=positive_fraction,
                observed_motion_range_deg=[float(np.rad2deg(magnitudes.min())), float(np.rad2deg(magnitudes.max()))])
        except (ValueError, KeyError) as exc:
            entry["rejected_slots"][str(slot)] = str(exc)
            previous = old.get("slot_refs", {}).get(str(slot))
            dependencies = previous.get("dependency_nodes", []) if previous else []
            if (previous and old.get("session_id") == session_id and old.get("rest_sha256") == rest_hash
                    and dependencies and all(node in profile["nodes"] and old.get("nodes_epochs", {}).get(node) == profile["nodes"][node]["epoch"] for node in dependencies)):
                entry["slot_refs"][str(slot)] = deepcopy(previous)
                entry["nodes_epochs"].update({node: old["nodes_epochs"][node] for node in dependencies})
                entry["diagnostics"][str(slot)] = dict(status="retained_previous", reason=str(exc))
    if not any(value.get("status") == "fitted" for value in entry["diagnostics"].values()):
        raise ValueError("功能轴没有可更新的关节；保留原配置。" + "；".join(entry["rejected_slots"].values()))
    return entry


def apply_functional_axes(observations, profile, rest):
    """Retarget only existing observations, preserving unmodified child relations.

    Call after apply_calibration; raw frame freshness is enforced there. This
    API has no sample metadata: the runtime additionally owns application epoch
    and session guards. Here profile/session/rest/dependency bindings are checked.
    """
    output = deepcopy(observations)
    diagnostic = dict(kind=_KIND, applied_slots=[], propagated_slots=[], skipped_slots={},
                      residual_domain="calibrated_anatomical_observations", anatomical_world_pose=False)
    entries = profile.get("functional_fingers", {})
    if not entries:
        return output, diagnostic
    local, rest_hash = _locals(rest), digest(rest)
    inverse = {params["slot"]: node for node, params in profile["nodes"].items()}
    original = {slot: quat(observations[node]) for slot, node in inverse.items() if node in observations}
    if 0 not in original:
        diagnostic["skipped_slots"]["root"] = "missing_root"
        return output, diagnostic
    target = dict(original)
    for finger, entry in entries.items():
        if finger not in _JOINTS:
            continue
        if (entry.get("session_id") != profile.get("session_id") or entry.get("rest_sha256") != rest_hash
                or any(node not in profile["nodes"] or profile["nodes"][node]["epoch"] != epoch
                       for node, epoch in entry.get("nodes_epochs", {}).items())):
            diagnostic["skipped_slots"][finger] = "profile_session_rest_or_epoch_mismatch"
            continue
        prior, cmc_mapped = entry["thumb_cmc_prior_rad"], entry["cmc_mapped"]
        for slot in ((14, 15) if finger == "thumb" else (7, 8, 9)):
            if slot not in original:
                diagnostic["skipped_slots"][str(slot)] = "missing_child_observation"
                continue
            try:
                parent = _parent(slot, original, local, prior, cmc_mapped)
                new_parent = _parent(slot, target, local, prior, cmc_mapped)
            except ValueError as exc:
                diagnostic["skipped_slots"][str(slot)] = str(exc)
                continue
            delta = local[slot].inv() * parent.inv() * original[slot]
            reference = entry["slot_refs"].get(str(slot)) if slot in _JOINTS[finger] else None
            if reference:
                basis = quat(reference["axis_basis_xyzw"])
                delta = basis.inv() * quat(reference["neutral_local_xyzw"]).inv() * delta * basis
                diagnostic["applied_slots"].append(slot)
            elif PARENTS[slot] in diagnostic["applied_slots"] or PARENTS[slot] in diagnostic["propagated_slots"]:
                diagnostic["propagated_slots"].append(slot)
            else:
                continue
            target[slot] = new_parent * local[slot] * delta
            output[inverse[slot]] = target[slot].as_quat().tolist()
    if diagnostic["applied_slots"]:
        diagnostic["residual_domain"] = "functional_axis_retargeted_observations"
    return output, diagnostic
