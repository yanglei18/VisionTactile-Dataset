"""Three held poses for session-bound, experimental IMU mounting calibration.

The palm measures the actual rotation magnitudes; no exact 90-degree pose is
assumed. Directions are instructions, not measured ground truth. In particular,
reversing both instructed motions has an unobservable 180-degree heading
ambiguity. This adapter neither uses prior failed profiles nor corrects yaw drift.
The explicitly captured neutral mean is anchored to model rest after mounting
fit; this reference choice is not a personal range-of-motion calibration.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import _fit, quat, validate_rest
from .config import check_model_hand, check_proposed_mount, digest, validate_config


STEPS = (
    {"key": "neutral", "title": "1. 平掌静止", "duration_s": 2.0,
     "instruction": "右手掌心朝下，五指伸直，手掌大致水平；保持静止 2 秒。"},
    {"key": "fingers_down", "title": "2. 指尖下倾", "duration_s": 2.0,
     "instruction": "五指保持伸直，转动手腕让整个手掌的指尖向下倾约 30–60°，然后静止 2 秒；不要握拳或只向下平移。"},
    {"key": "thumb_up", "title": "3. 拇指侧上抬", "duration_s": 2.0,
     "instruction": "先回到初始平掌，再转动整个手掌，使拇指侧向上、小指侧向下约 30–60°，然后静止 2 秒；不要只动拇指。"},
)
_KEYS = tuple(step["key"] for step in STEPS)
_TITLES = {step["key"]: step["title"] for step in STEPS}


class GuidedCalibrationError(ValueError):
    """Actionable failure; the caller can retain other captured steps."""

    def __init__(self, message, *, redo_steps=()):
        super().__init__(message)
        self.redo_steps = tuple(redo_steps)


def _unit(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} 必须是有限的三维向量")
    norm = np.linalg.norm(vector)
    if not math.isfinite(float(norm)) or norm < 1e-9:
        raise ValueError(f"{name} 方向退化，无法确定模型运动轴")
    return vector / norm


def model_motion_axes(joints_m, hand="right"):
    """Derive geometric forward/thumbward/dorsal axes from neutral MANO geometry.

    Joint order is wrist/thumb/index/middle/ring/little. For the right hand,
    positive rotation about thumbward lowers fingertips and about forward lifts
    the thumb. Left-hand anatomical motions reverse these rotation-axis signs;
    the geometric axes stay thumbward/forward. Dorsal also reverses the cross.
    Right-hand signs have opt-in real-MANO tests. Left-hand reflection is a
    geometric convention, not physical validation of a device or mounting.
    """
    if hand not in ("right", "left"):
        raise ValueError("hand must be right or left")
    joints = np.asarray(joints_m, dtype=float)
    if joints.shape != (21, 3) or not np.isfinite(joints).all():
        raise ValueError("中立 MANO joints_m 必须为有限的 (21, 3) 数组")
    forward = _unit(joints[9] - joints[0], "腕到中指根部")
    thumbward = joints[5] - joints[17]
    thumbward = _unit(thumbward - np.dot(thumbward, forward) * forward, "小指侧到食指侧")
    return {"pitch_axis": thumbward.tolist(), "roll_axis": forward.tolist(),
            "palm_up_axis": ((1 if hand == "right" else -1) * np.cross(forward, thumbward)).tolist(),
            "source": f"{hand}_neutral_mano_21_joints", "hand": hand}


def _axes(model_axes):
    if not isinstance(model_axes, dict):
        raise ValueError("model_axes 必须来自中立 MANO 模型")
    pitch = _unit(model_axes.get("pitch_axis"), "模型指尖下倾轴")
    roll = _unit(model_axes.get("roll_axis"), "模型拇指上抬轴")
    if abs(float(np.dot(pitch, roll))) > 1e-5:
        raise ValueError("模型指尖下倾轴和拇指上抬轴必须正交")
    side = 1 if model_axes.get("hand", "right") == "right" else -1
    up = _unit(side * np.cross(roll, pitch), "模型手背向上轴")
    if "palm_up_axis" in model_axes and np.dot(up, _unit(model_axes["palm_up_axis"], "模型手背向上轴")) < 1 - 1e-5:
        raise ValueError("模型掌面方向与运动轴的右手坐标约定不一致")
    # Rotation axes are axial vectors: reflected anatomy reverses their sign.
    return side * np.stack((pitch, roll)), up


def _collect_node(captures, node, stale_after):
    means, counts, deviations, diagnostics = [], [], [], {}
    epochs, seen, bad_steps = set(), set(), []
    for key in _KEYS:
        values = []
        stats = dict(fresh_unique=0, duplicate=0, stale_or_missing=0, invalid=0)
        for frame in captures[key]:
            data = frame["nodes"].get(node)
            if not isinstance(data, dict):
                stats["stale_or_missing"] += 1
                continue
            protection = data.get("input_filter", {})
            if (data.get("orientation_valid") is False or protection.get("gyro_rejected")
                    or protection.get("accel_rejected")):
                stats["invalid"] += 1
                continue
            age = data.get("age_s")
            if (isinstance(age, bool) or not isinstance(age, (int, float))
                    or not math.isfinite(age) or not 0 <= age <= stale_after):
                stats["stale_or_missing"] += 1
                continue
            epoch, index = data.get("epoch"), data.get("sample_index")
            if type(epoch) is not int or epoch < 0 or type(index) is not int or index < 1:
                stats["invalid"] += 1
                continue
            epochs.add(epoch)
            token = (epoch, index)
            if token in seen:
                stats["duplicate"] += 1
                continue
            seen.add(token)
            try:
                values.append(quat(data.get("quat_xyzw"), f"node {node}").as_quat())
            except (ValueError, TypeError):
                stats["invalid"] += 1
        stats["fresh_unique"] = len(values)
        diagnostics[key] = stats
        counts.append(len(values))
        if len(values) < 3:
            bad_steps.append(key)
            continue
        samples = Rotation.from_quat(values)
        mean = samples.mean()
        means.append(mean.as_quat())
        deviations.append(float(np.percentile(np.rad2deg((mean.inv() * samples).magnitude()), 95)))
    if len(epochs) > 1:
        raise GuidedCalibrationError(f"节点 {node} 的姿态滤波器在标定中重启，三步必须在同一次连续连接中重采。", redo_steps=_KEYS)
    if bad_steps:
        names = "、".join(_TITLES[key] for key in bad_steps)
        raise GuidedCalibrationError(
            f"节点 {node} 在 {names} 中不足 3 个新鲜独立样本（各步 {counts}）；请检查连接后只重采这些步骤。冻结或重复帧不计数。",
            redo_steps=bad_steps,
        )
    return Rotation.from_quat(means), next(iter(epochs)), counts, deviations, diagnostics


def fit_guided_calibration(captures, rest, config, *, session_id: str, model_axes: dict):
    """Fit current-layout nodes, returning an ``apply_calibration`` profile.

    Capture snapshots may contain only nodes with new sample_index values.
    Identity indices are per node and epoch, not host timestamps. The GUI owns
    the two-second capture clock; here at least three independent fresh samples
    are required per node per pose. Missing finger nodes are rejected locally.
    A missing/restarted palm or unobservable palm motions stop this fit, with
    ``GuidedCalibrationError.redo_steps`` for targeted recapture.
    Mounts retain the three-pose fit, but world alignment exactly maps each
    neutral sensor mean to its neutral target. Both pre-anchor and resulting
    three-pose errors are retained: zero neutral error is a reference choice,
    never evidence that inconsistent motion or subsequent drift was corrected.
    """
    config = validate_config(config)
    check_model_hand(config, rest)
    if model_axes.get("hand", "right") != config["hand"]:
        raise ValueError("model motion axes hand does not match configuration hand")
    check_proposed_mount(config)
    rest_rotations = Rotation.from_quat(validate_rest(rest))
    axes, model_up = _axes(model_axes)
    if not isinstance(session_id, str) or not session_id.strip():
        raise ValueError("session_id 必须标识当前连续采集会话，不能使用旧会话标定")
    if not isinstance(captures, dict):
        raise GuidedCalibrationError("请依次采集三种姿态。", redo_steps=_KEYS)
    for key in _KEYS:
        if not isinstance(captures.get(key), (list, tuple)) or not captures[key]:
            raise GuidedCalibrationError(f"尚未采集 {_TITLES[key]}。", redo_steps=(key,))
        for frame in captures[key]:
            if not isinstance(frame, dict) or not isinstance(frame.get("nodes"), dict):
                raise GuidedCalibrationError(f"{_TITLES[key]} 数据格式错误，请重新采集该步骤。", redo_steps=(key,))
            stamp = frame.get("timestamp_s")
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise GuidedCalibrationError(f"{_TITLES[key]} 时间戳无效。", redo_steps=(key,))
    palm_node = next(node for node, slot in config["node_to_slot"].items() if slot == 0)
    palm, epoch, counts, deviations, diagnostics = _collect_node(captures, palm_node, config["stale_after_s"])
    relative = (palm[0].inv() * palm[1:]).as_rotvec()
    magnitudes = np.linalg.norm(relative, axis=1)
    small = [key for key, angle in zip(_KEYS[1:], magnitudes) if angle < np.deg2rad(10)]
    if small:
        names = "、".join(_TITLES[key] for key in small)
        raise GuidedCalibrationError(f"{names} 的整掌转动不足 10°；五指保持伸直，转动手腕约 30–60°，不要只平移或弯手指。", redo_steps=small)
    sensor_axes = relative / magnitudes[:, None]
    separation_sin = float(np.linalg.norm(np.cross(sensor_axes[0], sensor_axes[1])))
    if separation_sin <= .3:
        raise GuidedCalibrationError("两次整掌转动方向太相近，无法区分安装轴；第二步指尖向下，第三步回平掌后拇指侧向上。", redo_steps=_KEYS[1:])
    sensor_from_model, _ = Rotation.align_vectors(sensor_axes, axes)
    world_alignment = (palm[0] * sensor_from_model).inv()
    up_cos = float(np.dot(world_alignment.apply([0., 0., 1.]), model_up))
    if up_cos <= 0:
        raise GuidedCalibrationError(
            "动作方向与掌心朝下不一致：请确认初始掌心朝下，第二步指尖下倾、第三步拇指侧上抬。"
            "至少一个方向可能反了，仅凭当前数据无法确定是哪一步；请核对后重采相应动作。",
            redo_steps=_KEYS[1:],
        )
    warnings = []
    tilt_deg = float(np.rad2deg(np.arccos(np.clip(up_cos, -1., 1.))))
    if tilt_deg > 15:
        warnings.append(f"初始掌面或动作轴偏离掌心朝下约 {tilt_deg:.1f}°；本次保留，请检查动作示意，标定不代表精度通过。")
    alignment_errors = np.rad2deg(np.arccos(np.clip(np.sum(sensor_from_model.apply(axes) * sensor_axes, axis=1), -1., 1.)))
    if float(alignment_errors.max()) > 5:
        warnings.append("两次整掌运动并非理想正交轴，轴拟合偏差超过 5°；保留结果，请参考模型检查后续手势。")
    root_deltas = world_alignment * palm * sensor_from_model
    nodes, rejected, capture_diagnostics = {}, {}, {palm_node: diagnostics}
    for node, slot in config["node_to_slot"].items():
        try:
            target = root_deltas * rest_rotations[slot]
            if node == palm_node:
                means, node_epoch, node_counts, within = palm, epoch, counts, deviations
                mount, alignment = sensor_from_model * rest_rotations[0], world_alignment
                errors = np.zeros(3)
            else:
                means, node_epoch, node_counts, within, node_diagnostics = _collect_node(captures, node, config["stale_after_s"])
                capture_diagnostics[node] = node_diagnostics
                mount, alignment, errors = _fit(means, target)
            pre_alignment, pre_errors = alignment, errors
            pre_rms = float(np.sqrt(np.mean(pre_errors ** 2)))
            # Do not compromise the explicit zero pose to reduce the other two
            # residuals. The fitted mount is unchanged; only the constant world
            # alignment is re-anchored to the recorded neutral mean.
            alignment = target[0] * (means[0] * mount).inv()
            errors = np.rad2deg((target.inv() * alignment * means * mount).magnitude())
            rms = float(np.sqrt(np.mean(errors ** 2)))
            sensor_motion = np.rad2deg((means[0].inv() * means[1:]).magnitude())
            magnitude_error = sensor_motion - np.rad2deg(magnitudes)
            if rms > 5 or pre_rms > 5 or max(within) > 5:
                warnings.append(f"节点 {node} 的姿态拟合或静止波动超过 5°；本次保留，但不视为准确度验收。")
            if float(np.max(np.abs(magnitude_error))) > 5:
                warnings.append(f"节点 {node} 的整掌旋转幅度与手掌最大相差 {np.max(np.abs(magnitude_error)):.1f}°；"
                                "恒定安装旋转不能消除幅度差，本次保留；可能涉及相对运动或惯性输入响应，不据此判定原因。")
            nodes[node] = dict(slot=slot, epoch=node_epoch,
                               sensor_from_bone_xyzw=mount.as_quat().tolist(),
                               world_alignment_xyzw=alignment.as_quat().tolist(),
                               neutral_sensor_xyzw=means[0].as_quat().tolist(),
                               neutral_target_xyzw=target[0].as_quat().tolist(),
                               label_fit_rms_deg=rms, label_fit_per_pose_deg=errors.tolist(),
                               pre_anchor_world_alignment_xyzw=pre_alignment.as_quat().tolist(),
                               pre_anchor_label_fit_rms_deg=pre_rms,
                               pre_anchor_label_fit_per_pose_deg=pre_errors.tolist(),
                               sensor_motion_deg=sensor_motion.tolist(),
                               rigid_motion_magnitude_error_deg=magnitude_error.tolist(),
                               within_pose_p95_deg=within, frames_per_pose=node_counts)
        except (GuidedCalibrationError, ValueError) as exc:
            rejected[node] = str(exc)
    return dict(
        schema_version=1, method="guided_rigid_pose_hand_eye_neutral_anchor_v2",
        status="provisional_not_accuracy_validated", session_id=session_id,
        binding="current_live_session_and_node_epochs", hand=config["hand"],
        config_sha256=digest(config), rest_sha256=digest(rest),
        stale_after_s=config["stale_after_s"], nodes=nodes, rejected_nodes=rejected,
        warnings=warnings, capture_diagnostics=capture_diagnostics,
        palm_node=palm_node, model_axes={"pitch_axis": list(model_axes["pitch_axis"]),
                                        "roll_axis": list(model_axes["roll_axis"]),
                                        "palm_up_axis": model_up.tolist(), "hand": config["hand"]},
        palm_motion=dict(rotation_deg=np.rad2deg(magnitudes).tolist(), axis_separation_sin=separation_sin,
                         axis_fit_error_deg=alignment_errors.tolist(), neutral_up_tilt_deg=tilt_deg),
        pose_root_quat_xyzw={key: root.as_quat().tolist() for key, root in zip(_KEYS, root_deltas)},
        neutral_reference=dict(kind="explicit_neutral_pose_mean", step="neutral",
                               target="model_rest_global", mounting_fit_preserved=True,
                               automatically_updated=False),
        anatomical_accuracy_verified=False, yaw_drift_corrected=False, personal_rom_calibrated=False,
        limitations=["Guided pose directions and a rigid neutral hand are assumptions, not measured anatomical ground truth.",
                     "The palm supplies actual motion magnitudes; no exact 90-degree poses are assumed.",
                     "Reversing both instructed motions has a 180-degree heading ambiguity that this calibration cannot detect.",
                     "Gravity-up consistency cannot identify which individual instruction was reversed.",
                     "Initial heading alignment does not correct subsequent six-axis relative yaw drift.",
                     "Neutral sensor means are explicitly anchored to model rest; zero neutral residual is imposed, not an accuracy result.",
                     "Pre-anchor fit and post-anchor pose residuals are both retained; incompatible motions are not repaired by re-anchoring.",
                     "The neutral reference does not measure personal anatomical joint zero or range of motion.",
                     "Valid only in this continuous live session and matching per-node filter epochs.",
                     "Missing finger observations are omitted; unmeasured bones are not calibrated."])
