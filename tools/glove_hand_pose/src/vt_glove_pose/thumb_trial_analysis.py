"""Describe a guided sensor-orientation trial, never certify joint accuracy.

Quaternion differences are measured on each sensor separately. There is no
subtraction of scalar rotation magnitudes and no cross-sensor joint angle.
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np

from .math3d import mean_quaternion


_STAGES = {"initial_hold": (3., 8.), "final_hold": (23., 31.)}
_MAX_HOST_GAP_S = .25
_MIN_WINDOW_SAMPLES = 10


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _vector(value, size, *, quaternion=False):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or raw.shape != (size,):
            return None
        array = raw.astype(float)
    except (TypeError, ValueError, OverflowError):
        return None
    if not np.all(np.isfinite(array)):
        return None
    if quaternion:
        norm = np.linalg.norm(array)
        if not .999 <= norm <= 1.001:
            return None
        array = array / norm
    return array


def _angle(a, b):
    return float(np.degrees(2*np.arccos(np.clip(abs(np.dot(a, b)), 0., 1.))))


def _direction_angle(a, b):
    denominator = np.linalg.norm(a)*np.linalg.norm(b)
    if denominator < 1e-12:
        return None
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b)/denominator, -1., 1.))))


def _node_observation(row, node):
    reasons = []
    data = row.get("nodes", {}).get(str(node), {}) if isinstance(row.get("nodes"), dict) else {}
    if not isinstance(data, dict):
        data = {}
    # The trial starts after an explicit display reference. Its validity flag
    # also captures resets/discontinuities that quaternion values alone hide.
    if row.get("valid") is not True:
        reasons.append("invalid_paired_snapshot")
    if data.get("fresh") is not True:
        reasons.append("sensor_not_fresh")
    values = {}
    for key, size in (("sensor_quat_wxyz", 4), ("gyro_dps", 3), ("accel_g", 3)):
        values[key] = _vector(data.get(key), size, quaternion=size == 4)
        if values[key] is None:
            reasons.append("invalid_"+key)
    if values["accel_g"] is not None and np.linalg.norm(values["accel_g"]) < 1e-12:
        reasons.append("undefined_acceleration_direction")
    return values, reasons


def _assess(rows, node, left, right, *, malformed_rows=False):
    chosen = [(t, row) for t, row in rows if left <= t < right]
    reasons = ["malformed_or_missing_timestamp"] if malformed_rows else []
    if len(chosen) < _MIN_WINDOW_SAMPLES:
        reasons.append("insufficient_samples")
    if not chosen or chosen[0][0]-left > _MAX_HOST_GAP_S+1e-9 or right-chosen[-1][0] > _MAX_HOST_GAP_S+1e-9:
        reasons.append("incomplete_window_coverage")
    observations = []
    previous = None
    for timestamp, row in chosen:
        count = row.get("pair_count")
        if type(count) is not int or count < 1:
            reasons.append("invalid_pair_count")
        if previous is not None:
            dt = timestamp-previous[0]
            if dt < 0:
                reasons.append("backward_host_time")
            if dt > _MAX_HOST_GAP_S+1e-9:
                reasons.append("host_gap")
            if type(count) is int and type(previous[1]) is int and count != previous[1]+1:
                reasons.append("missing_or_repeated_online_pair")
        previous = timestamp, count
        observation, invalid = _node_observation(row, node)
        reasons.extend(invalid)
        observations.append(observation)
    reasons = list(dict.fromkeys(reasons))
    quality = dict(status="available" if not reasons else "unavailable", reasons=reasons,
                   samples=len(chosen), window_host_s=[left, right])
    if reasons:
        return quality, None
    quats = np.array([r["sensor_quat_wxyz"] for r in observations])
    gyro = np.array([r["gyro_dps"] for r in observations])
    accel = np.array([r["accel_g"] for r in observations])
    statistics = dict(quat=mean_quaternion(quats), accel_mean=accel.mean(axis=0),
                       gyro_mean_dps=gyro.mean(axis=0).tolist(),
                       gyro_rms_dps=float(np.sqrt(np.mean(np.sum(gyro*gyro, axis=1)))),
                       gyro_variation_rms_dps=float(np.sqrt(np.mean(np.sum((gyro-gyro.mean(axis=0))**2, axis=1)))),
                       accel_variation_rms_g=float(np.sqrt(np.mean(np.sum((accel-accel.mean(axis=0))**2, axis=1)))))
    return quality, statistics


def _hold(rows, node, start_s, stage, malformed):
    left, right = (start_s+offset for offset in _STAGES[stage])
    quality, all_stats = _assess(rows, node, left, right, malformed_rows=malformed)
    first_quality, first = _assess(rows, node, left, left+1, malformed_rows=malformed)
    last_quality, last = _assess(rows, node, right-1, right, malformed_rows=malformed)
    reasons = list(dict.fromkeys(quality["reasons"]+first_quality["reasons"]+last_quality["reasons"]))
    result = dict(status="unavailable" if reasons else "available", reasons=reasons,
                  samples=quality["samples"], window_relative_s=list(_STAGES[stage]),
                  first_window_relative_s=[left-start_s, left+1-start_s],
                  last_window_relative_s=[right-1-start_s, right-start_s],
                  first_window_samples=first_quality["samples"], last_window_samples=last_quality["samples"],
                  orientation_change_deg=None, acceleration_direction_change_deg=None,
                  gyro_rms_dps=None, gyro_variation_rms_dps=None, gyro_mean_dps=None,
                  accel_variation_rms_g=None)
    if not reasons:
        result.update(orientation_change_deg=_angle(first["quat"], last["quat"]),
                      acceleration_direction_change_deg=_direction_angle(first["accel_mean"], last["accel_mean"]),
                      **{key: all_stats[key] for key in ("gyro_rms_dps", "gyro_variation_rms_dps",
                                                        "gyro_mean_dps", "accel_variation_rms_g")})
    return result, first, last


def analyze_trial(rows, start_s, node_ids, completion_reason="completed"):
    """Analyze the 31-second plan using one-second mean sensor poses.

    ``rows`` contains every online paired snapshot, including invalid ones;
    do not replace it with the 30 Hz GUI history. Equal host timestamps are
    retained; pair counters must be consecutive. A cancelled trial may retain
    measurements from fully captured hold stages but has no final return metric.
    Return comparison uses the settled last second of each hold: [7,8) versus
    [30,31). Within-hold change still compares that hold's first/last seconds.
    """
    if not _finite(start_s):
        raise ValueError("start_s must be finite")
    if (not isinstance(node_ids, (list, tuple)) or len(node_ids) != 2
            or any(type(n) is not int or not 1 <= n <= 12 for n in node_ids) or node_ids[0] == node_ids[1]):
        raise ValueError("node_ids must contain two distinct node IDs 1..12")
    if not isinstance(completion_reason, str) or not completion_reason or len(completion_reason) > 256:
        raise ValueError("completion_reason must be a nonempty short string")
    source = list(rows)
    if len(source) > 30000:
        raise ValueError("at most 30000 paired snapshots supported")
    timed = []
    malformed = False
    for row in source:
        if not isinstance(row, dict) or not _finite(row.get("host_monotonic_s")):
            malformed = True
            continue
        timed.append((float(row["host_monotonic_s"]), row))
    nodes = {}
    summary_parts = []
    for node in node_ids:
        initial, _, first = _hold(timed, node, start_s, "initial_hold", malformed)
        final, _, last = _hold(timed, node, start_s, "final_hold", malformed)
        across, _ = _assess(timed, node, start_s+3, start_s+31, malformed_rows=malformed)
        return_reasons = list(dict.fromkeys(initial["reasons"]+final["reasons"]+across["reasons"]))
        if completion_reason != "completed":
            return_reasons.append("trial_not_completed")
        difference = accel_difference = None
        if not return_reasons:
            difference = _angle(first["quat"], last["quat"])
            accel_difference = _direction_angle(first["accel_mean"], last["accel_mean"])
        nodes[str(node)] = dict(initial_hold=initial, final_hold=final,
            return_orientation_difference_deg=difference,
            return_acceleration_direction_difference_deg=accel_difference,
            return_status="unavailable" if return_reasons else "available", return_reasons=return_reasons,
            return_window_basis=dict(initial_relative_s=[7., 8.], final_relative_s=[30., 31.]))
        fmt = lambda value: "不可计算" if value is None else f"{value:.2f}°"
        summary_parts.append(f"{node}号：起始计划保持段变化 {fmt(initial['orientation_change_deg'])}，"
                             f"末尾计划保持段变化 {fmt(final['orientation_change_deg'])}，"
                             f"首末姿态差 {fmt(difference)}")
    available = sum(value["return_status"] == "available" for value in nodes.values())
    status = "available" if available == len(node_ids) else "partial" if any(
        value[stage]["status"] == "available" for value in nodes.values() for stage in _STAGES) else "unavailable"
    summary = ("；".join(summary_parts)+
               "。保持段按引导时间划分，不证明实际静止；上述是各传感器姿态变化，不是关节误差或合格判定。")
    if completion_reason != "completed":
        summary = "本次未完成，仅保留已完整采集阶段的测量。"+summary
    return dict(kind="independent_sensor_orientation_trial_analysis", schema_version=1,
                status=status, completion_reason=completion_reason, start_s=float(start_s),
                duration_s=31., node_ids=list(node_ids), input_rows=len(source), nodes=nodes,
                summary=summary, acceptance_evaluated=False, anatomical_joint_angles=False,
                actual_static_holds_verified=False,
                time_basis="host_observed_windows_not_device_time",
                measurement_policy=dict(window_mean="Markley quaternion mean, sign invariant",
                    orientation_difference="2 acos(abs(dot(mean_q_start, mean_q_end)))",
                    max_host_gap_s=_MAX_HOST_GAP_S, minimum_samples_per_window=_MIN_WINDOW_SAMPLES,
                    require_consecutive_online_pair_counts=True),
                limitations=["无法区分真实物理姿态变化与估计误差；需要独立真值才能判定精度。",
                             "引导用户静止不等于实际静止；角速度和加速度统计仅辅助观察。",
                             "加速度方向包含线性加速度影响，六轴航向仍可能漂移。",
                             "配对计数仅检查程序内记录连续性，不能证明传感器硬件未丢样。",
                             "每个传感器独立计算，未建立两传感器共同航向或解剖关节角。"])
