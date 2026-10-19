"""Conservative two-stage calibration of a *shared* thumb IP hinge axis.

A: keep the thumb shape fixed while rotating the hand about multiple axes.
B: hold the proximal segment still; flex the distal segment first, then extend.
This is not a calibration of arbitrary thumb CMC motion or anatomical accuracy.
The caller must supply paired, bias-corrected gyro in deg/s and acceleration in g.
No timestamp synchronization, zero pose, device state or prior mounts is inferred.
All functions are pure: failure raises and cannot partially replace old mounts.
"""

from __future__ import annotations

import math
import numpy as np

from .math3d import from_matrix


class ThumbCalibrationError(ValueError):
    """The supplied captures do not support the constrained calibration."""

    def __init__(self, message, *, details=None):
        super().__init__(message)
        self.details = {} if details is None else details


def _positive(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ThumbCalibrationError(f"{name} must be finite in [{low}, {high}]")
    return float(value)


def _vectors(value, name, *, one=False):
    try:
        result = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError) as exc:
        raise ThumbCalibrationError(f"{name} must contain numeric vectors") from exc
    shape_ok = result.shape == (3,) if one else result.ndim == 2 and result.shape[1] == 3 and len(result) <= 30000
    if not shape_ok or not np.all(np.isfinite(result)):
        raise ThumbCalibrationError(f"{name} needs finite {'(3,)' if one else '(N,3), N<=30000'} values")
    return result


def _capture(gp, gd, ap, ad, rate, saturation, saturated, stage):
    values = [_vectors(v, f"{stage}.{name}") for v, name in zip((gp,gd,ap,ad), ("prox_gyro","dist_gyro","prox_acc","dist_acc"))]
    n = len(values[0])
    if any(len(v) != n for v in values) or n < max(90, math.ceil(1.5 * rate)):
        raise ThumbCalibrationError(f"{stage} needs paired captures lasting at least 1.5 seconds")
    if saturated is not None:
        flags = np.asarray(saturated)
        if flags.dtype.kind != "b" or flags.shape not in ((n,), (n,2)):
            raise ThumbCalibrationError(f"{stage} saturation mask must be boolean (N,) or (N,2)")
        if flags.any():
            raise ThumbCalibrationError(f"{stage} contains saturated raw gyro samples; repeat slowly")
    if any(np.any(np.abs(g) >= saturation) for g in values[:2]):
        raise ThumbCalibrationError(f"{stage} contains near-limit gyro samples; repeat slowly")
    norms = [np.linalg.norm(a,axis=1) for a in values[2:]]
    if any(np.any((norm < .3) | (norm > 2.0)) for norm in norms):
        raise ThumbCalibrationError(f"{stage} acceleration is not a reliable gravity observation")
    return (*values, *norms)


def _angle_degrees(a, b):
    return np.degrees(np.arccos(np.clip(np.sum(a*b,axis=-1),-1,1)))


def _robust_failure(code, message, details, action):
    info = dict(method="robust_v2", stage="A", failure_code=code, **details)
    raise ThumbCalibrationError(
        f"拇指联合 A 未通过：{message}。{action}。"
        "不一致可能涉及配对时序、比例、安装或相对运动，当前不能确定物理原因；不要覆盖旧安装轴。",
        details=info)


def _continuous_mask(mask, minimum):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    starts, stops = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    result = np.zeros(len(mask), dtype=bool)
    spans = []
    for start, stop in zip(starts, stops):
        if stop - start >= minimum:
            result[start:stop] = True
            spans.append((int(start), int(stop)))
    return result, spans


def _weighted_rotation(gp, gd, weights):
    u, _, vt = np.linalg.svd(gd.T @ (gp * weights[:, None]))
    return vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T


def _excitation(values):
    singular = np.linalg.svd(values, compute_uv=False)
    return float(singular[1] / singular[0]) if len(singular) >= 2 and singular[0] > 1e-12 else 0.


def _fit_thumb_common_rotation_robust(gp, gd, ap, ad, *, rate, limit, saturated):
    """Train-only robust fit, followed by untouched alternating-block validation.

    Raw saturation/nonfinite inputs still fail the entire capture. Other short
    disturbances may be excluded, but support must cover >=80% both globally
    and in each preassigned subset. Final physical residual bounds are the
    strict-v1 bounds. Holdout data never select or refit the rotation.
    """
    try:
        values = [_vectors(v, "A." + name) for v, name in
                  zip((gp, gd, ap, ad), ("prox_gyro", "dist_gyro", "prox_acc", "dist_acc"))]
    except ThumbCalibrationError as exc:
        _robust_failure("input_invalid", f"输入向量格式或有限数值不合法：{exc}",
                        {"sample_rate_hz": rate}, "检查成对三轴输入，不插补损坏记录")
    gp, gd, ap, ad = values
    n = len(gp)
    details = {"total_samples": n, "sample_rate_hz": rate,
               "minimum_effective_fraction": .80, "minimum_segment_s": .25,
               "holdout_block_s": .5}
    if any(len(v) != n for v in values) or n < max(90, math.ceil(1.5 * rate)):
        _robust_failure("capture_length", f"配对样本数 {n}，需要两路等长且至少 {max(90, math.ceil(1.5 * rate))} 个",
                        details, "检查采集配对并保留完整连续记录")
    raw_saturated = np.zeros(n, dtype=bool)
    if saturated is not None:
        flags = np.asarray(saturated)
        if flags.dtype.kind != "b" or flags.shape not in ((n,), (n, 2)):
            _robust_failure("saturation_mask", "饱和标志必须为布尔 (N,) 或 (N,2)", details, "检查输入格式")
        raw_saturated = flags if flags.ndim == 1 else np.any(flags, axis=1)
    saturated_rows = raw_saturated | np.any(np.abs(gp) >= limit, axis=1) | np.any(np.abs(gd) >= limit, axis=1)
    if saturated_rows.any():
        details["saturated_rows"] = int(saturated_rows.sum())
        _robust_failure("saturation", f"{int(saturated_rows.sum())} 个样本触及饱和/近量程界限 {limit:g} °/s",
                        details, "减慢动作并核实量程；鲁棒拟合不恢复削顶数据")
    sp, sd = np.linalg.norm(gp, axis=1), np.linalg.norm(gd, axis=1)
    np_, nd = np.linalg.norm(ap, axis=1), np.linalg.norm(ad, axis=1)
    up, ud = ap / np.maximum(np_, 1e-12)[:, None], ad / np.maximum(nd, 1e-12)[:, None]
    moving = (sp > 8.) | (sd > 8.)
    movement_count = int(moving.sum())
    details["moving_samples_total"] = movement_count
    if movement_count < max(60, math.ceil(.5 * rate)):
        _robust_failure("motion_insufficient", f"有效运动候选仅 {movement_count} 个",
                        details, "保持拇指形状，轻缓做两个不同方向的整手旋转")
    ratios = sp / np.maximum(sd, 1e-12)
    ratio_error = np.abs(ratios - 1.)
    details["all_moving_amplitude_ratio_p95_error"] = float(np.percentile(ratio_error[moving], 95))
    details["all_moving_amplitude_ratio_median"] = float(np.median(ratios[moving]))
    amplitude_ok = ~moving | ((np.minimum(sp, sd) >= 3.) & (ratio_error <= .15))
    gravity_ok = (np_ >= .8) & (np_ <= 1.2) & (nd >= .8) & (nd <= 1.2) & (np.abs(np_ - nd) <= .10)
    pointwise = amplitude_ok & gravity_ok
    # Full-timeline continuity is an ACCEPTANCE check only. In particular, a
    # short train interval must not enter/leave the fit depending on whether
    # adjacent holdout samples make it reach the .25-second minimum.
    eligible, _ = _continuous_mask(pointwise, math.ceil(.25 * rate))
    blocks = np.arange(n) // max(1, round(.5 * rate))
    train, holdout = blocks % 2 == 0, blocks % 2 == 1

    def coverage(mask, subset):
        denominator = int(np.count_nonzero(moving & subset))
        return float(np.count_nonzero(mask & moving & subset) / denominator) if denominator else 0.

    initial_fractions = {"prefilter_motion_fraction": coverage(eligible, np.ones(n, bool)),
                         "prefilter_train_motion_fraction": coverage(eligible, train),
                         "prefilter_holdout_motion_fraction": coverage(eligible, holdout)}
    details.update(initial_fractions)
    if min(initial_fractions.values()) < .80 or np.mean(eligible) < .80:
        _robust_failure("reliable_segments_insufficient",
                        f"连续可靠运动保留 {initial_fractions['prefilter_motion_fraction']:.1%}，"
                        f"训练/独立验证分别 {initial_fractions['prefilter_train_motion_fraction']:.1%}/"
                        f"{initial_fractions['prefilter_holdout_motion_fraction']:.1%}；"
                        f"全运动幅比偏差 p95={details['all_moving_amplitude_ratio_p95_error']:.3f}，"
                        "需要各部分至少 80% 且逐样本幅比偏差不超过 0.15",
                        details, "先核对两路采样配对和比例，再检查小板是否相对滑动；不要只增加动作幅度")
    # Every fit/IRLS/refit input is determined solely by train rows and fixed
    # indices. Full-timeline segments can reject a capture, never select its
    # fitted training vectors. This also avoids artificial boundary trimming.
    fitting = pointwise & moving & train
    details["train_fitting_samples"] = int(fitting.sum())
    for label, subset in (("train", train), ("holdout", holdout)):
        count = int(np.count_nonzero(eligible & moving & subset))
        details[label + "_candidate_samples"] = count
        if count < max(30, math.ceil(.25 * rate)):
            _robust_failure("split_motion_insufficient", f"{label} 连续运动只有 {count} 个样本",
                            details, "把两个方向的缓慢转动分布在整段采集中，不要只在开头动一下")
    excitation = [_excitation(v[fitting]) for v in (gp, gd)]
    details["train_initial_excitation_second_to_first"] = excitation
    if min(excitation) < .20:
        _robust_failure("train_excitation", f"训练段第二/第一方向激励比 {min(excitation):.3f} < 0.20",
                        details, "保持拇指形状，分次上下翻掌及左右转掌，而非只沿一条轴旋转")
    p, d = gp[fitting], gd[fitting]
    # Balanced initial weights prevent a few faster rows dominating a rotation.
    # Huber reweighting uses TRAIN residuals only; no lag search or holdout tuning.
    base = 1. / np.maximum(np.maximum(sp[fitting], sd[fitting]), 8.) ** 2
    rotation = _weighted_rotation(p, d, base)
    for _ in range(30):
        residual = np.linalg.norm(d @ rotation.T - p, axis=1)
        weights = base * np.minimum(1., 1. / np.maximum(residual, 1e-12))
        updated = _weighted_rotation(p, d, weights)
        if np.max(np.abs(updated - rotation)) < 1e-11:
            rotation = updated
            break
        rotation = updated
    # A single train-only inlier refit removes remaining bias from transients.
    r = np.linalg.norm(gd @ rotation.T - gp, axis=1)
    relative = r / np.maximum(np.maximum(sp, sd), 1e-12)
    acc_angle = _angle_degrees(up, ud @ rotation.T)
    refine = fitting & (r <= 4.) & (relative <= .12) & (acc_angle <= 20.)
    if coverage(refine, train) < .80:
        details["train_refit_motion_fraction"] = coverage(refine, train)
        _robust_failure("train_constant_rotation", f"同一旋转能解释的训练运动仅 {coverage(refine, train):.1%} < 80%",
                        details, "核对时序、比例和安装是否稳定；当前数据不支持同一固定相对旋转")
    rotation = _weighted_rotation(gp[refine], gd[refine], np.ones(int(refine.sum())))

    r = np.linalg.norm(gd @ rotation.T - gp, axis=1)
    relative = r / np.maximum(np.maximum(sp, sd), 1e-12)
    acc_angle = _angle_degrees(up, ud @ rotation.T)
    consistent = eligible & ((~moving) | ((r <= 4.) & (relative <= .12))) & (acc_angle <= 20.)
    retained, spans = _continuous_mask(consistent, math.ceil(.25 * rate))
    details.update(effective_fraction=float(np.mean(retained)),
                   retained_motion_fraction=coverage(retained, np.ones(n, bool)),
                   train_motion_fraction=coverage(retained, train),
                   holdout_motion_fraction=coverage(retained, holdout),
                   retained_segment_count=len(spans),
                   longest_retained_segment_s=max((b-a for a,b in spans), default=0) / rate,
                   # At most 12 spans keep both success and failure details compact.
                   retained_segment_ranges_s=[[round(a/rate, 4), round(b/rate, 4)] for a,b in spans[:12]],
                   retained_segment_ranges_truncated=len(spans) > 12)
    if min(details[key] for key in ("effective_fraction", "retained_motion_fraction", "train_motion_fraction", "holdout_motion_fraction")) < .80:
        _robust_failure("validation_coverage", f"最终全段/训练/独立验证运动保留 {details['retained_motion_fraction']:.1%}/"
                        f"{details['train_motion_fraction']:.1%}/{details['holdout_motion_fraction']:.1%}，需要各部分至少 80%",
                        details, "记录中不一致超出少量瞬态，不能靠筛选大半数据建立标定")

    def metrics(subset):
        active = retained & moving & subset
        observable = retained & subset
        exc = [_excitation(v[active]) for v in (gp, gd)]
        return dict(moving_samples=int(active.sum()), excitation_second_to_first=exc,
                    gyro_amplitude_ratio_median=float(np.median(ratios[active])),
                    gyro_amplitude_ratio_p95_error=float(np.percentile(ratio_error[active], 95)),
                    gyro_residual_rms_dps=float(np.sqrt(np.mean(r[active] ** 2))),
                    gyro_residual_p95_dps=float(np.percentile(r[active], 95)),
                    gyro_relative_residual_p95=float(np.percentile(relative[active], 95)),
                    gravity_direction_p95_deg=float(np.percentile(acc_angle[observable], 95)),
                    gravity_direction_max_deg=float(np.max(acc_angle[observable])))

    scores = {"train": metrics(train), "holdout": metrics(holdout)}
    details.update(scores)
    for name, score in scores.items():
        if min(score["excitation_second_to_first"]) < .20:
            _robust_failure("validation_excitation", f"{name} 留存数据方向激励比 {min(score['excitation_second_to_first']):.3f} < 0.20",
                            details, "在整段内重复两个不同旋转方向；不能用单方向验证多方向标定")
        if (not .90 <= score["gyro_amplitude_ratio_median"] <= 1.10
                or score["gyro_amplitude_ratio_p95_error"] > .15
                or score["gyro_residual_rms_dps"] > 2.
                or score["gyro_residual_p95_dps"] > 4.
                or score["gyro_relative_residual_p95"] > .12
                or score["gravity_direction_p95_deg"] > 8.
                or score["gravity_direction_max_deg"] > 20.):
            _robust_failure("validation_residual", f"{name} 残差 RMS={score['gyro_residual_rms_dps']:.3f} °/s、"
                            f"p95={score['gyro_residual_p95_dps']:.3f} °/s、相对 p95={score['gyro_relative_residual_p95']:.3f}、"
                            f"重力 p95={score['gravity_direction_p95_deg']:.2f}°；门限仍为 2/4/0.12/8°",
                            details, "先核对两路时间配对、比例及固定相对姿态，误差未通过独立验证")
    quality = metrics(np.ones(n, bool))
    quality.update(details, algorithm="paired_thumb_common_rotation_robust_v2", method="robust_v2",
                   gravity_reliable_fraction=float(np.mean(gravity_ok)),
                   fit_rows_source="even chronological 0.5-second blocks only",
                   holdout_rows_source="odd chronological 0.5-second blocks; never used to fit rotation",
                   saturation_policy="reject entire capture", physical_cause_determined=False)
    return dict(rotation=rotation.tolist(), quality=quality)


def fit_thumb_common_rotation(prox_gyro_dps, dist_gyro_dps, prox_acc_g, dist_acc_g, *,
                              sample_rate_hz=120.0, gyro_saturation_dps=286.0,
                              saturated=None, method="strict_v1"):
    """Fit a proper dist-sensor -> prox-sensor rotation from rigid co-motion.

    Two substantially non-collinear angular-velocity directions are required.
    A can hold any fixed IP flexion: rotation about the hinge leaves that axis
    invariant. Holding an arbitrary *changing* thumb shape is not permitted.
    Supply raw-code saturation flags when subtracting bias could hide clipping.
    """
    rate = _positive(sample_rate_hz,"sample_rate_hz",10,2000)
    limit = _positive(gyro_saturation_dps,"gyro_saturation_dps",5,10000)
    if method == "robust_v2":
        return _fit_thumb_common_rotation_robust(prox_gyro_dps, dist_gyro_dps, prox_acc_g, dist_acc_g,
                                                  rate=rate, limit=limit, saturated=saturated)
    if method != "strict_v1":
        raise ThumbCalibrationError("未知标定 method；请选择 strict_v1 或显式 robust_v2")
    gp,gd,ap,ad,np_,nd = _capture(prox_gyro_dps,dist_gyro_dps,prox_acc_g,dist_acc_g,
                                rate,limit,saturated,"common-motion stage A")
    sp,sd = np.linalg.norm(gp,axis=1),np.linalg.norm(gd,axis=1)
    moving = (sp > 8) | (sd > 8)
    if np.count_nonzero(moving) < max(60,math.ceil(.5*rate)):
        raise ThumbCalibrationError("stage A has insufficient common rotation")
    if np.any(np.minimum(sp[moving],sd[moving]) < 3):
        raise ThumbCalibrationError("stage A gyro amplitude mismatch: thumb shape was not fixed")
    ratio = sp[moving]/sd[moving]
    ratio_median, ratio_error = float(np.median(ratio)),float(np.percentile(np.abs(ratio-1),95))
    if not .90 <= ratio_median <= 1.10 or ratio_error > .15:
        raise ThumbCalibrationError("stage A gyro amplitude mismatch; check units, pairing and fixed thumb shape")
    excitation=[]
    for values in (gp[moving],gd[moving]):
        singular=np.linalg.svd(values,compute_uv=False)
        ratio_2=float(singular[1]/singular[0])
        excitation.append(ratio_2)
        if ratio_2 < .20:
            raise ThumbCalibrationError("stage A needs at least two non-collinear rotations, not one-axis motion")
    u,_,vt=np.linalg.svd(gd[moving].T@gp[moving])
    correction=np.diag([1.,1.,np.linalg.det(vt.T@u.T)])
    rotation=vt.T@correction@u.T
    residual=np.linalg.norm(gd[moving]@rotation.T-gp[moving],axis=1)
    rms=float(np.sqrt(np.mean(residual**2)))
    p95=float(np.percentile(residual,95))
    relative=float(np.percentile(residual/np.maximum(sp[moving],sd[moving]),95))
    if rms > 2 or p95 > 4 or relative > .12:
        raise ThumbCalibrationError("stage A has no constant relative rotation; keep thumb shape fixed")
    reliable=(np_>=.8)&(np_<=1.2)&(nd>=.8)&(nd<=1.2)
    if np.mean(reliable)<.90 or np.percentile(np.abs(np_-nd),95)>.10:
        raise ThumbCalibrationError("stage A acceleration magnitudes disagree or contain too much dynamic acceleration")
    acc_angle=_angle_degrees(ap[reliable]/np_[reliable,None], (ad[reliable]/nd[reliable,None])@rotation.T)
    acc_p95,acc_max=float(np.percentile(acc_angle,95)),float(np.max(acc_angle))
    if acc_p95>8 or acc_max>20:
        raise ThumbCalibrationError("stage A gravity directions disagree; check synchronization and fixed thumb shape")
    return dict(rotation=rotation.tolist(), quality=dict(
        moving_samples=int(moving.sum()), excitation_second_to_first=excitation,
        gyro_amplitude_ratio_median=ratio_median,gyro_amplitude_ratio_p95_error=ratio_error,
        gyro_residual_rms_dps=rms,gyro_residual_p95_dps=p95,gyro_relative_residual_p95=relative,
        gravity_direction_p95_deg=acc_p95,gravity_direction_max_deg=acc_max,
        gravity_reliable_fraction=float(np.mean(reliable))))


def _first_flexion_sign(gyro, axis, rate):
    """Use the first coherent untrimmed bout, never a later extension."""
    kind,bout=None,[]
    minimum=max(6,math.ceil(.06*rate))
    def finish():
        if not bout: return None
        excursion=sum(abs(p) for p,s in bout)/rate
        disturbance=sum(s for p,s in bout)/rate
        if kind in (-1,1) and len(bout)>=minimum and excursion>=.75: return kind
        if len(bout)<=2 and disturbance<=.5: return None
        raise ThumbCalibrationError("stage B first flexion is too short or off-axis; do not infer sign from later extension")
    for vector in gyro:
        speed=float(np.linalg.norm(vector)); projection=float(np.dot(vector,axis))
        state=None if speed<=5 else (1 if projection>0 else -1) if abs(projection)>=.9*speed else "ambiguous"
        if state!=kind:
            sign=finish()
            if sign is not None: return sign
            kind,bout=state,[]
        if state is not None: bout.append((projection,speed))
    sign=finish()
    if sign is None:
        raise ThumbCalibrationError("stage B has no coherent first instructed flexion")
    return sign


def _mount(axis, neutral):
    up=neutral/np.linalg.norm(neutral)
    x=-axis  # The first instructed positive flexion is rotation about -x.
    z=up-np.dot(up,x)*x
    projection=float(np.linalg.norm(z))
    if projection<.55:
        raise ThumbCalibrationError("shared hinge is too close to gravity; neutral flexion is unobservable")
    z/=projection
    y=np.cross(z,x)
    return from_matrix(np.stack((x,y,z))).tolist(),projection


def calibrate_thumb_pair(common_prox_gyro_dps, common_dist_gyro_dps,
                         common_prox_acc_g, common_dist_acc_g,
                         flex_prox_gyro_dps, flex_dist_gyro_dps,
                         flex_prox_acc_g, flex_dist_acc_g,
                         neutral_prox_acc_g, neutral_dist_acc_g, *,
                         sample_rate_hz=120.0, gyro_saturation_dps=286.0,
                         common_saturated=None, flex_saturated=None, method="strict_v1"):
    """Return shared mounts and quality; never mutate input captures or engine.

    Stage B requires isolated IP motion with the proximal segment fixed. The
    first coherent action must be the instructed flexion, followed by extension.
    Existing explicit neutral gravity establishes the frame, not a guessed
    straight-finger state. No current pose is silently assigned zero.

    ``robust_v2`` robustifies A only and reports numerical Chinese feedback;
    B retains every strict hinge/sign gate and never trims its first motion.
    """
    rate=_positive(sample_rate_hz,"sample_rate_hz",10,2000)
    limit=_positive(gyro_saturation_dps,"gyro_saturation_dps",5,10000)
    common=fit_thumb_common_rotation(common_prox_gyro_dps,common_dist_gyro_dps,
        common_prox_acc_g,common_dist_acc_g,sample_rate_hz=rate,
        gyro_saturation_dps=limit,saturated=common_saturated,method=method)
    rotation=np.asarray(common["rotation"])
    def fail(old_message, code, chinese, numbers, action):
        if method == "strict_v1":
            raise ThumbCalibrationError(old_message)
        raise ThumbCalibrationError(
            f"拇指联合 B 未通过：{chinese}。{action}。"
            "当前不能确定是配对、比例、安装还是相对运动导致不一致；旧安装轴应保留。",
            details=dict(method="robust_v2", stage="B", failure_code=code, **numbers))
    try:
        gp,gd,ap,ad,np_,nd=_capture(flex_prox_gyro_dps,flex_dist_gyro_dps,
            flex_prox_acc_g,flex_dist_acc_g,rate,limit,flex_saturated,"isolated-IP stage B")
    except ThumbCalibrationError as exc:
        if method == "strict_v1":
            raise
        fail(str(exc), "capture_invalid", f"输入/配对或量程检查未通过（{exc}）",
             {"sample_rate_hz": rate, "gyro_saturation_dps": limit}, "检查两路记录长度与码值；出现削顶时先核实量程并减慢动作")
    sp,sd=np.linalg.norm(gp,axis=1),np.linalg.norm(gd,axis=1)
    prox_p95,prox_max=float(np.percentile(sp,95)),float(np.max(sp))
    if prox_p95>2 or prox_max>5:
        fail("stage B proximal segment moved; hold it fixed and move distal IP only", "proximal_motion",
             f"近节角速度 p95={prox_p95:.3f}、最大={prox_max:.3f} °/s，门限分别 2/5 °/s",
             {"proximal_gyro_p95_dps": prox_p95, "proximal_gyro_max_dps": prox_max},
             "检查近节固定及偏置；本步骤只支持近节稳定、末节屈伸")
    prox_up=ap/np_[:,None]
    center=np.median(prox_up,axis=0)
    if method == "robust_v2" and np.linalg.norm(center) < 1e-6:
        fail("stage B proximal gravity direction is degenerate", "proximal_gravity_degenerate",
             f"近节重力方向中位向量模长 {np.linalg.norm(center):.6g}，无法形成稳定参考",
             {"proximal_gravity_median_norm": float(np.linalg.norm(center))},
             "检查方向突变、安装和原始数据；不要把相反方向平均成可靠重力")
    center/=np.linalg.norm(center)
    prox_acc_angle=_angle_degrees(prox_up,center)
    if np.percentile(prox_acc_angle,95)>3 or np.max(prox_acc_angle)>8:
        p95, maximum = float(np.percentile(prox_acc_angle,95)), float(np.max(prox_acc_angle))
        fail("stage B proximal gravity direction moved; proximal fixation is not reliable", "proximal_gravity",
             f"近节重力方向变化 p95={p95:.2f}°、最大={maximum:.2f}°，门限 3°/8°",
             {"proximal_gravity_p95_deg": p95, "proximal_gravity_max_deg": maximum},
             "检查近节小板是否移动及动态加速度，不把这一段当作固定近节")
    moving=sd>5
    if np.count_nonzero(moving)<max(30,math.ceil(.5*rate)):
        count = int(np.count_nonzero(moving))
        fail("stage B has insufficient isolated distal motion", "distal_motion_insufficient",
             f"末节运动样本 {count} 个，需要至少 {max(30,math.ceil(.5*rate))} 个",
             {"distal_moving_samples": count}, "确认节点对应后，仅让末节缓慢完成屈曲和伸展")
    _,singular,vt=np.linalg.svd(gd[moving],full_matrices=False)
    linearity=float(singular[0]**2/np.sum(singular**2))
    if linearity<.97:
        fail("stage B distal motion is not a single hinge; arbitrary CMC motion is unsupported", "distal_not_hinge",
             f"末节单轴能量占比 {linearity:.4f} < 0.97", {"distal_linearity": linearity},
             "检查安装和实际运动关节；本模型不支持任意多自由度对掌动作")
    try:
        sign = _first_flexion_sign(gd,vt[0],rate)
    except ThumbCalibrationError as exc:
        if method == "strict_v1":
            raise
        first = int(np.flatnonzero(moving)[0])
        fail(str(exc), "first_flexion_ambiguous", f"首段屈曲正负不可确定；首个运动位于 {first/rate:.3f} s，"
             f"速度 {sd[first]:.2f} °/s；需连续至少 {max(6,math.ceil(.06*rate))} 个同向样本和 0.75° 转角",
             {"first_moving_sample": first, "first_moving_speed_dps": float(sd[first]),
              "first_motion_min_samples": max(6,math.ceil(.06*rate)), "first_motion_min_excursion_deg": .75},
             "从静止开始，先缓慢屈曲再伸展；不能删掉首段或用后续伸展猜测符号")
    axis_d=vt[0]*sign
    axis_p=rotation@axis_d
    axis_p/=np.linalg.norm(axis_p)
    along=gd@axis_d
    positive=float(np.maximum(along,0).sum()/rate)
    negative=float(-np.minimum(along,0).sum()/rate)
    if min(positive,negative)<10:
        fail("stage B needs clear flexion and extension, at least 10 degrees each", "excursion_insufficient",
             f"屈曲/伸展累计转角 {positive:.2f}°/{negative:.2f}°，分别需要至少 10°",
             {"positive_excursion_deg": positive, "negative_excursion_deg": negative},
             "轻缓完成两个方向的末节动作，不靠增大整手摆动补足")
    neutral_p=_vectors(neutral_prox_acc_g,"neutral_prox_acc_g",one=True)
    neutral_d=_vectors(neutral_dist_acc_g,"neutral_dist_acc_g",one=True)
    if any(not .75<=np.linalg.norm(v)<=1.25 for v in (neutral_p,neutral_d)):
        norms = [float(np.linalg.norm(v)) for v in (neutral_p, neutral_d)]
        fail("neutral acceleration must be approximately 1g", "neutral_acc_norm",
             f"中立参考加速度模长 {norms[0]:.3f}/{norms[1]:.3f} g，不在 0.75–1.25 g",
             {"neutral_acc_norm_g": norms}, "核实加速度比例和中立参考；不自动用当前姿势替代零位")
    neutral_dot_p=float(np.dot(neutral_p/np.linalg.norm(neutral_p),axis_p))
    neutral_dot_d=float(np.dot(neutral_d/np.linalg.norm(neutral_d),axis_d))
    neutral_difference=neutral_dot_d-neutral_dot_p
    if abs(neutral_difference)>.08:
        fail("shared-axis neutral gravity components disagree; do not force inconsistent axes to match", "neutral_axis_mismatch",
             f"中立重力沿共同轴分量差 {neutral_difference:.4f}，绝对值门限 0.08",
             {"neutral_axis_gravity_prox": neutral_dot_p, "neutral_axis_gravity_dist": neutral_dot_d,
              "neutral_axis_gravity_difference": neutral_difference},
             "检查两块板安装和中立参考一致性，不减去差值强行配平")
    reliable=(np_>=.85)&(np_<=1.15)&(nd>=.85)&(nd<=1.15)
    if np.mean(reliable)<.80:
        fraction = float(np.mean(reliable))
        fail("stage B contains too much dynamic acceleration; flex slowly", "gravity_fraction",
             f"约 1g 的可靠重力样本占比 {fraction:.1%} < 80%",
             {"gravity_reliable_fraction": fraction}, "减慢末节动作并检查加速度比例及小板固定")
    gravity_difference=(ad[reliable]/nd[reliable,None])@axis_d-prox_up[reliable]@axis_p
    gravity_p95=float(np.percentile(np.abs(gravity_difference),95))
    if gravity_p95>.10:
        fail("stage B shared-axis gravity components disagree; motion or axis model is not a hinge", "hinge_gravity_mismatch",
             f"运动时共同轴重力分量差 p95={gravity_p95:.4f} > 0.10",
             {"hinge_gravity_difference_p95": gravity_p95}, "检查共同轴、采样配对与安装；此段不能作为单铰链标定")
    try:
        mount_p,projection_p=_mount(axis_p,neutral_p)
        mount_d,projection_d=_mount(axis_d,neutral_d)
    except ThumbCalibrationError as exc:
        if method == "strict_v1":
            raise
        projections = [float(np.sqrt(max(0., 1.-v*v))) for v in (neutral_dot_p, neutral_dot_d)]
        fail(str(exc), "neutral_unobservable", f"中立重力在轴垂直面投影比例 {projections[0]:.3f}/{projections[1]:.3f}，需要至少 0.55",
             {"neutral_yz_projection_fraction": projections}, "此参考姿态使屈曲难以被重力观测；需重新选择明确的中立参考，不强行算轴")
    result = dict(mounts=dict(proximal=mount_p,distal=mount_d),
        dist_to_prox_rotation=rotation.tolist(),shared_axis_prox=axis_p.tolist(),shared_axis_dist=axis_d.tolist(),
        quality=dict(algorithm="paired_thumb_ip_fixed_proximal_v1",common_motion=common["quality"],
            distal_linearity=linearity,distal_moving_samples=int(moving.sum()),
            proximal_gyro_p95_dps=prox_p95,proximal_gyro_max_dps=prox_max,
            proximal_gravity_p95_deg=float(np.percentile(prox_acc_angle,95)),
            positive_excursion_deg=positive,negative_excursion_deg=negative,
            neutral_axis_gravity_prox=neutral_dot_p,neutral_axis_gravity_dist=neutral_dot_d,
            neutral_axis_gravity_difference=neutral_difference,
            hinge_gravity_difference_p95=gravity_p95,
            neutral_yz_projection_fraction=[projection_p,projection_d],
            sign_basis="first coherent untrimmed instructed distal flexion",axis_sign_algorithm_version=2,
            anatomical_accuracy_verified=False,hardware_time_synchronization_verified=False,
            applicability="fixed proximal, isolated distal IP hinge; not arbitrary thumb CMC motion"))
    if method == "robust_v2":
        result["quality"].update(algorithm="paired_thumb_ip_fixed_proximal_robust_a_v2",
                                 method="robust_v2", stage_b_method="strict_hinge_v1_no_trimming")
    return result
