"""Cheap, observational feedback; never accepts a calibration or changes data.

This is a trailing 0.5-second indicator, not the whole-capture acceptance test.
Same packet arrival stamps do not establish physical sensor synchronization.
"""
from __future__ import annotations

import math

import numpy as np

from .config import gyro_scale


def thumb_capture_feedback(capture, config, bias):
    stage = "A" if capture["target"] == "thumb_common" else "B"
    pair = config["fingers"]["thumb"]
    p, d = pair["proximal"], pair["distal"]
    left, right = (capture["rows"].get(n, []) for n in (p, d))
    n = min(len(left), len(right))
    window = max(6, math.ceil(.5 * config["sample_rate_hz"]))
    tolerance = 0. if (p - 1) // 6 == (d - 1) // 6 else max(.02, 2 / config["sample_rate_hz"])
    pairs = [(a, b) for a, b in zip(left[max(0, n-window):n], right[max(0, n-window):n])
             if abs(a[0] - b[0]) <= tolerance]
    result = dict(stage=stage, status="waiting", paired_samples=n,
                  paired_window_samples=len(pairs), advisory_only=True,
                  message="正在接收配对数据；此提示不是最终标定结果。")
    if len(pairs) < 6:
        return result
    gp = np.asarray([a[1] for a, _ in pairs]) - bias[p]
    gd = np.asarray([b[1] for _, b in pairs]) - bias[d]
    sp, sd = np.linalg.norm(gp, axis=1), np.linalg.norm(gd, axis=1)
    result.update(proximal_speed_p95_dps=float(np.percentile(sp, 95)),
                  distal_speed_p95_dps=float(np.percentile(sd, 95)))
    near_limit = any(np.any(np.abs(row[1] / gyro_scale(config, node)) >= 32700)
                     for pair_rows in pairs for node, row in zip((p, d), pair_rows))
    if near_limit:
        result.update(status="needs_attention", message="检测到接近陀螺仪量程的数据；请减慢动作，量程仍需与固件核实。")
        return result
    if stage == "A":
        moving = np.maximum(sp, sd) > 8
        count = int(moving.sum())
        result["moving_window_samples"] = count
        if count < 6:
            result["message"] = "等待共同转动：保持拇指形状不变，轻缓地上下翻掌、左右转掌，不需要用力。"
            return result
        comparable = moving & (np.minimum(sp, sd) >= 3)
        good = np.zeros(len(sp), dtype=bool)
        good[comparable] = np.abs(sp[comparable] / sd[comparable] - 1) <= .15
        fraction = float(np.count_nonzero(good) / count)
        result["amplitude_consistent_fraction"] = fraction
        result.update(status="collecting" if fraction >= .8 else "needs_attention",
            message=(f"近 0.5 秒两路速度相近样本 {fraction:.0%}；保持拇指形状，完成两个方向的轻缓转动。"
                     if fraction >= .8 else
                     f"近 0.5 秒两路速度相近样本仅 {fraction:.0%}；可放慢并保持形状。时序或安装差异也可能造成不一致，并非一定是动作问题。"))
    else:
        stable = float(np.percentile(sp, 95)) <= 2 and float(np.max(sp)) <= 5
        result["proximal_stable_window"] = bool(stable)
        result.update(status="collecting" if stable else "needs_attention",
            message=("近节目前稳定；仅末节先弯再伸，不转掌。" if stable else
                     f"近节正在移动（近 0.5 秒 P95={np.percentile(sp, 95):.1f}°/s，目标≤2）；扶稳近节。末节已经做过的动作不会被自动重新归零。"))
    return result
