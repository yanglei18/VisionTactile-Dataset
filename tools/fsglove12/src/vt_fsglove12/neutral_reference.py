"""Explicit, session-local open-hand reference; never an automatic drift reset.

Keep the fitted mounting rotations and the palm's world orientation. Re-anchor
only fresh finger observations to the neutral model relative to that palm. This
removes a constant reference offset, not a bad axis, gyro scale, or future drift.
"""
from __future__ import annotations

from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import quat, validate_rest
from .config import digest


def _held_rotation(rows, node, params, stale_after):
    values, seen = [], set()
    for frame in rows:
        data = frame.get("nodes", {}).get(node)
        if not isinstance(data, dict):
            continue
        age = data.get("age_s")
        if (isinstance(age, bool) or not isinstance(age, (float, int))
                or not math.isfinite(age) or not 0 <= age <= stale_after):
            continue
        if data.get("epoch") != params["epoch"]:
            raise ValueError("滤波器已重启，不能仅重设伸直参考；需要重新做三步标定。")
        index = data.get("sample_index")
        if type(index) is not int or index < 1 or index in seen:
            continue
        try:
            rotation = quat(data.get("quat_xyzw"), f"node {node}")
        except (ValueError, TypeError):
            continue
        seen.add(index)
        values.append(rotation.as_quat())
    if len(values) < 3:
        raise ValueError("不足 3 个新鲜独立样本；保留该节点原参考。")
    rotations = Rotation.from_quat(values)
    mean = rotations.mean()
    p95 = float(np.percentile(np.rad2deg((mean.inv() * rotations).magnitude()), 95))
    if p95 > 5.:
        raise ValueError(f"采集期间运动较大（P95 {p95:.1f}°）；请伸直停稳后再点一次。")
    return mean, len(values), p95


def reanchor_neutral_reference(rows, profile, rest, *, session_id):
    """Return a new profile, leaving inputs/palm/mounts/filter epochs untouched.

The user must explicitly hold a naturally open hand. Sensors cannot verify that
the fingers are straight. Missing/moving fingers retain their previous reference
and are reported individually; a missing/changed palm rejects the operation.
    """
    if not session_id or profile.get("session_id") != session_id:
        raise ValueError("伸直参考只能用于当前连续连接会话。")
    rest_rotations = Rotation.from_quat(validate_rest(rest))
    if profile.get("rest_sha256") != digest(rest):
        raise ValueError("伸直参考与当前模型不匹配。")
    if not isinstance(rows, (list, tuple)) or not rows:
        raise ValueError("伸直参考采集没有新样本，原标定保留。")
    nodes = profile["nodes"]
    palm_node = next((node for node, params in nodes.items() if params["slot"] == 0), None)
    if palm_node is None:
        raise ValueError("没有已标定的手掌节点。")
    palm_params = nodes[palm_node]
    palm_sensor, _, _ = _held_rotation(rows, palm_node, palm_params, profile["stale_after_s"])
    palm = (quat(palm_params["world_alignment_xyzw"]) * palm_sensor
            * quat(palm_params["sensor_from_bone_xyzw"]))
    root_delta = palm * rest_rotations[0].inv()
    result = deepcopy(profile)
    updated, rejected = {}, {}
    for node, params in nodes.items():
        if node == palm_node:
            continue
        try:
            sensor, count, p95 = _held_rotation(rows, node, params, profile["stale_after_s"])
        except ValueError as exc:
            rejected[node] = str(exc)
            continue
        mount = quat(params["sensor_from_bone_xyzw"])
        target = root_delta * rest_rotations[params["slot"]]
        alignment = target * (sensor * mount).inv()
        delta = float(np.rad2deg((quat(params["world_alignment_xyzw"]).inv() * alignment).magnitude()))
        result["nodes"][node].update(world_alignment_xyzw=alignment.as_quat().tolist(),
                                      neutral_sensor_xyzw=sensor.as_quat().tolist(),
                                      neutral_target_xyzw=target.as_quat().tolist(),
                                      label_fit_scope="before_manual_neutral_reference")
        updated[node] = dict(frames=count, within_pose_p95_deg=p95, alignment_change_deg=delta)
    if not updated:
        raise ValueError("没有可更新的手指参考，原标定保留。" + "；".join(f"{node}: {reason}" for node, reason in rejected.items()))
    result["manual_neutral_reference"] = dict(
        method="explicit_open_hand_palm_preserving_v1", session_id=session_id,
        previous_profile_version=profile.get("profile_version"),
        timestamp_s=float(rows[-1]["timestamp_s"]), updated_nodes=updated, rejected_nodes=rejected,
        palm_orientation_preserved=True, mounting_axes_refitted=False,
        anatomical_straightness_verified=False, yaw_drift_corrected=False,
        note="User-labelled open hand, not a verified zero, functional axis or range calibration.")
    result["neutral_reference"] = dict(kind="explicit_open_hand_reference",
                                       target="model_rest_relative_to_unchanged_palm",
                                       automatically_updated=False, mounting_fit_preserved=True,
                                       updated_nodes=sorted(updated, key=int))
    result["anatomical_accuracy_verified"] = False
    result["yaw_drift_corrected"] = False
    # Functional references were fitted in the previous calibrated coordinates.
    # Never reuse them silently after changing those coordinates.
    if result.pop("functional_fingers", None):
        result["manual_neutral_reference"]["functional_axes_cleared"] = True
    return result
