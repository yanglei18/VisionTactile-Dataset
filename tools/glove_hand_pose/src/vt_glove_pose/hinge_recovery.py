"""Pure, reason-code-based recovery advice for experimental hinge failures.

Advice is not a new acceptance threshold or a diagnosis of physical root cause.
In particular a reference/estimated-axis mismatch does not identify which of
the reference, mounting or estimated axis is wrong. Unknown failures preserve
evidence for review instead of asking for the same motion again.
"""
from __future__ import annotations

import math
from numbers import Real


_AXIS_EXCITATION = {
    "too_few_samples_in_fixed_split", "training_motion_below_noise",
    "training_compatible_with_common_rigid_motion", "training_tangent_jacobian_rank_deficient",
    "training_tangent_sensitivity_too_small", "training_tangent_jacobian_ill_conditioned",
    "too_few_training_residual_inliers", "too_few_active_holdout_samples",
}
_AXIS_RESIDUAL = {"train_constraint_residual_inconsistent", "holdout_constraint_residual_inconsistent"}


def _acceleration_variation_with_plausible_norms(metrics):
    if not isinstance(metrics, dict):
        return False
    arrays = []
    for key in ("mean_accel_norm_g", "accel_vector_std_g"):
        values = metrics.get(key)
        if (not isinstance(values, (list, tuple)) or len(values) != 2
                or any(isinstance(value, bool) or not isinstance(value, Real)
                       or not math.isfinite(value) or value < 0 for value in values)):
            return False
        arrays.append(values)
    norms, variation = arrays
    # These are the existing neutral-check bounds, not new acceptance rules.
    return all(.85 <= value <= 1.15 for value in norms) and max(variation) > .03


def classify_hinge_failure(profile_status, reason_codes, *, metrics=None):
    """Return independent JSON-safe advice, never mutate a profile or hardware.

    ``profile_status`` is a nonempty string; ``reason_codes`` is a list/tuple of
    strings. Specific reason codes take precedence over broad status categories.
    Only explicit known status aliases are used when a legacy report omits codes;
    substring searches such as ``'neutral' in status`` are deliberately absent.
    Optional neutral ``metrics`` distinguish measured variation with in-range
    average acceleration norms from an unresolved norm/unit problem. They do
    not certify units or accept the failed capture.
    """
    if not isinstance(profile_status, str) or not profile_status.strip():
        raise ValueError("profile_status must be a nonempty string")
    if not isinstance(reason_codes, (list, tuple)) or any(not isinstance(code, str) for code in reason_codes):
        raise ValueError("reason_codes must be a list/tuple of strings")
    codes = list(reason_codes)
    reasons = set(codes)

    def result(stage, title, instruction, detail, action, *, neutral=False, illustration="export", review=False):
        return dict(stage=stage, profile_status=profile_status, reason_codes=list(codes),
                    title=title, instruction=instruction, detail=detail, suggested_action=action,
                    requires_new_neutral=neutral, illustration=illustration, needs_review=review)

    if "explicit_neutral_acceleration_norm_inconsistent" in reasons:
        return result("neutral", "两路参考加速度幅值不一致",
                      "先导出本次实验，检查两路单位、固定与参考采集条件，再决定如何重新采参考。",
                      "这不等于操作不够认真；重复同样姿势或只重采运动不能保证解决。旧参考暂不继续使用。",
                      "export", neutral=True, review=True)
    if ("explicit_neutral_gravity_nearly_parallel_to_hinge" in reasons
            or profile_status == "frames_neutral_unobservable"):
        return result("neutral", "参考姿态需要改变整手倾斜",
                      "保持手指形状不变，先改变整手倾斜；停稳后点击重新采参考。",
                      "当前重力方向与估计转轴太接近。不要在原来的整手朝向重复静止，也不需要强行伸直手指。",
                      "neutral", neutral=True, illustration="rotate")
    if "explicit_neutral_axis_gravity_mismatch" in reasons:
        return result("frames", "参考与估计转轴不一致",
                      "先导出本次实验，保留静止参考和运动数据一起检查。",
                      "问题可能来自运动轴估计、传感器固定或参考数据；当前不能断定必须重采参考，也不能保证只重采运动就解决。",
                      "export", review=True)
    if reasons & {"neutral_acceleration_not_stable_gravity_like", "neutral_not_quiet_or_gravity_like"}:
        if ("neutral_raw_near_limit" not in reasons
                and _acceleration_variation_with_plausible_norms(metrics)):
            return result("neutral", "静止参考中的加速度有波动",
                          "把手掌和手指一起放稳，暂停转掌和屈伸后重新采参考。",
                          "两路平均幅值在当前单路检查范围内，但波动超限。准备结束后完整保持 5 秒；若反复失败，再导出检查。",
                          "neutral", neutral=True, illustration="still")
        return result("neutral", "参考数据尚不能作为稳定重力参考",
                      "先导出记录，检查是否有加速度波动、幅值或单位异常；放稳并确认条件后再采参考。",
                      "当前报告可能涉及波动或幅值，不把它一律归因于手没有保持好；重复动作不是确定的解决办法。",
                      "export", neutral=True, review=True)
    if reasons & {"insufficient_norm_consistent_acceleration", "paired_axis_gravity_components_inconsistent"}:
        return result("frames", "两路运动加速度一致性不足",
                      "先导出本次实验，检查运动加速度、单位、固定与估计转轴。",
                      "这不是简单的运动次数不足；不要仅增加同样动作。现有信息不能区分加速度影响、固定误差或轴估计问题。",
                      "export", review=True)
    if "neutral_raw_near_limit" in reasons:
        return result("neutral", "静止阶段陀螺读数接近量程",
                      "先导出并检查原始读数与量程配置，不要继续重复静止动作。",
                      "停止快速动作；若静止时仍近量程，需要检查采集数据或配置后再重新采参考。",
                      "export", neutral=True, review=True)
    if "neutral_gyro_not_quiet" in reasons:
        return result("neutral", "静止参考中的陀螺数据不够平稳",
                      "将手掌和手指一起放稳，准备好后重新保持 5 秒。",
                      "先停止转掌与屈伸；若放稳后仍反复失败，导出记录检查数据，不必不断重试。",
                      "neutral", neutral=True, illustration="still")
    if reasons & {"neutral_pair_count_insufficient", "neutral_host_span_insufficient",
                  "declare_at_least_two_nominal_seconds_of_neutral", "neutral_host_gap"}:
        return result("neutral", "参考采集的数据数量或连续性不足",
                      "先等两路数据连续恢复，再重新采满参考窗口。",
                      "这不是要求更用力地保持；检查连接，等准备结束后完整记录 5 秒。",
                      "neutral", neutral=True, illustration="still")
    if ("insufficient_parallel_gravity_component_to_resolve_axis_sign" in reasons
            or profile_status == "frames_sign_unobservable"):
        return result("frames", "运动中的轴正负信息不足",
                      "保留静止参考；屈伸时向不同方向轻缓改变整手倾斜，再采自然运动。",
                      "不要始终保持同一掌向；需要改变轴相对重力的方向。这不是静止参考自动失效。",
                      "motion", illustration="rotate")
    if (reasons & _AXIS_EXCITATION and reasons <= _AXIS_EXCITATION | _AXIS_RESIDUAL
            or not reasons and profile_status in {"axis_insufficient_excitation", "axes_insufficient_excitation"}):
        return result("axes", "屈伸与转掌的方向信息不足",
                      "保留静止参考；让两段发生相对屈伸，同时分别朝不同方向轻缓转掌，再采 20 秒。",
                      "例如先轻轻左右倾掌，再前后倾掌；不要只整手共转、只绕同一方向或固定一段不动。必要时导出检查。",
                      "motion", illustration="flex")
    if reasons & {"too_few_motion_pairs", "motion_pair_count_insufficient", "too_few_paired_gravity_observations"}:
        return result("motion", "本次运动数据数量不足",
                      "保留静止参考；检查连续数据后，重新完整采集 20 秒自然运动。",
                      "等准备倒计时结束后再开始；这只表示数据量不足，不是动作精度判定。",
                      "motion", illustration="flex")
    if "motion_raw_near_limit" in reasons or not reasons and profile_status == "input_near_limit":
        return result("motion", "运动陀螺读数接近量程",
                      "保留静止参考；放慢屈伸和转掌，避免猛甩，再采自然运动。",
                      "若慢动作或静止仍发生，请先导出检查量程与单位配置，不要一直重复。",
                      "motion", illustration="flex")
    if reasons & (_AXIS_RESIDUAL | {"optimizer_not_converged"}):
        return result("axes", "当前运动数据未形成可信的转轴候选",
                      "先导出静止与运动记录，检查拟合结果、固定和配对。",
                      "约束残差或计算收敛未满足条件，不能直接认定是手动操作错误，也不能靠重复同样动作保证解决。",
                      "export", review=True)
    return result("unknown", "本次失败需要保留记录检查",
                  "先导出本次实验与错误信息，不要继续重复同样动作。",
                  "当前原因未被可靠分类，可能涉及配置或数据；需检查具体记录后再决定重采哪一步。",
                  "export", neutral=profile_status in {"neutral_invalid", "insufficient_neutral"}, review=True)
