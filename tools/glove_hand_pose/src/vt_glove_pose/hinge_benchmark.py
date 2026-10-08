"""Deterministic analytic two-link scenarios, explicitly NOT real hand accuracy.

The world/body rotations and their derivatives generate the measurements. Sensor
axes, scale and nominal sampling are ideal; disturbances are deliberately added
and recorded. No measurement is fabricated from an observer's own output. Every
sample, including uncertain predictions and clipped input, enters error metrics.
"""
from __future__ import annotations

from collections import Counter
import math

import numpy as np


CASE_NAMES = (
    "twenty_cycles_bias", "bent_hold_bias", "common_rotation",
    "observation_dropout", "linear_acceleration_burst", "saturation_recovery",
    "unit_norm_direction_jump", "gyro_constraint_only", "unobservable_hold",
)


def _rotate(values, angles, axis):
    """Active axis rotation of row vectors; independent of estimator helpers."""
    values = np.asarray(values, dtype=float)
    c, s = np.cos(angles), np.sin(angles)
    out = values.copy()
    a, b = ((1, 2), (2, 0), (0, 1))[axis]
    out[:, a] = c*values[:, a] - s*values[:, b]
    out[:, b] = s*values[:, a] + c*values[:, b]
    return out


def _step(t, start, duration):
    u = np.clip((t-start)/duration, 0., 1.)
    value = 10*u**3 - 15*u**4 + 6*u**5
    derivative = 30*u**2*(1-u)**2/duration
    return value, derivative


def make_scenario(name, *, sample_rate_hz=120., seed=7):
    """Return paired functional-frame input and analytic truth for one scenario."""
    if name not in CASE_NAMES:
        raise ValueError(f"unknown benchmark scenario {name!r}")
    if (isinstance(sample_rate_hz, bool) or not isinstance(sample_rate_hz, (int, float))
            or not math.isfinite(sample_rate_hz) or not 30 <= sample_rate_hz <= 240):
        raise ValueError("benchmark sample_rate_hz must be in [30, 240]")
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("benchmark seed must be a nonnegative uint32")
    durations = dict(twenty_cycles_bias=42., bent_hold_bias=60., common_rotation=20.,
                     observation_dropout=22., linear_acceleration_burst=15.,
                     saturation_recovery=14., unit_norm_direction_jump=12.,
                     gyro_constraint_only=20., unobservable_hold=20.)
    duration = durations[name]
    t = np.arange(round(duration*sample_rate_hz)+1)/sample_rate_hz
    n = len(t)
    theta = np.full(n, 35.)
    theta_rate = np.zeros(n)
    roll = np.zeros(n); pitch = np.zeros(n); yaw = np.zeros(n)
    rd = np.zeros(n); pd = np.zeros(n); yd = np.zeros(n)
    bias = np.zeros(n)
    recovery_events = []
    crossing = None
    disturbance = "none"
    description = name
    if name == "twenty_cycles_bias":
        u = np.clip(t-2., 0., 40.)
        theta = 30.*(1-np.cos(np.pi*u))
        theta_rate = 30.*np.pi*np.sin(np.pi*u)
        bias[:] = 1.2
        roll, rd = .2*np.sin(.8*t), .16*np.cos(.8*t)
        pitch, pd = .3*np.sin(.6*t), .18*np.cos(.6*t)
        yaw, yd = .25*np.sin(.9*t), .225*np.cos(.9*t)
        crossing = dict(start_s=2., end_s=3., angle_deg=30.)
        description = "20 smooth 0-60 degree cycles with three-axis common rotation and 1.2 dps residual relative bias"
    elif name == "bent_hold_bias":
        theta[:] = 45.
        bias[:] = .8
        description = "Hold a bent 45 degree joint for 60 s with 0.8 dps residual bias; quiet must not mean straight"
    elif name in ("common_rotation", "gyro_constraint_only"):
        roll, rd = .4*np.sin(.8*t), .32*np.cos(.8*t)
        pitch, pd = .6*np.sin(.6*t), .36*np.cos(.6*t)
        yaw, yd = .5*np.sin(.9*t), .45*np.cos(.9*t)
        if name == "gyro_constraint_only":
            theta = 35.+20.*np.sin(1.2*t)
            theta_rate = 24.*np.cos(1.2*t)
            bias = 1.2*_step(t, 4., 1.)[0]
            disturbance = "common_acceleration_norm"
            recovery_events = [dict(name="return_to_one_g", time_s=15.)]
            description = "During 4-15 s accelerometer magnitude is 1.8 g; off-axis common gyro motion remains informative"
        else:
            description = "Fixed 35 degree joint under analytic three-axis whole-pair rotation"
    elif name in ("observation_dropout", "unobservable_hold"):
        rise, rise_rate = _step(t, 2., 1.5)
        fall, fall_rate = _step(t, 12., 1.5)
        if name == "unobservable_hold":
            fall, fall_rate = np.zeros(n), np.zeros(n)
        pitch = np.pi/2*(rise-fall)
        pd = np.pi/2*(rise_rate-fall_rate)
        bias = .1 + 1.8*_step(t, 6., 1.)[0]
        if name == "observation_dropout":
            theta = 35.+15.*np.sin(1.4*t)
            theta_rate = 21.*np.cos(1.4*t)
            recovery_events = [dict(name="gravity_return", time_s=13.5)]
        description = "Hinge becomes vertical and common transverse rotation stops; residual bias changes during unobservable interval"
    elif name == "linear_acceleration_burst":
        theta = 35.+20.*np.sin(1.3*t)
        theta_rate = 26.*np.cos(1.3*t)
        disturbance = "distal_acceleration"
        description = "Unmodelled distal linear acceleration pulse during ordinary flexion, 5-7 s"
    elif name == "saturation_recovery":
        bend, bend_rate = _step(t, 3., .12)
        straight, straight_rate = _step(t, 9., 1.)
        theta = 80.*(bend-straight)
        theta_rate = 80.*(bend_rate-straight_rate)
        disturbance = "clip_gyro"
        recovery_events = [dict(name="clipped_bend_ends", time_s=3.12),
                           dict(name="return_to_reference", time_s=10.)]
        description = "80 degree bend in 120 ms clipped at 286.7 dps, then bent hold and slow return; history must remain lost"
    elif name == "unit_norm_direction_jump":
        theta[:] = 40.
        disturbance = "unit_norm_jump"
        description = "Distal apparent-gravity direction jumps 35 degrees for 0.15 s without gyro support, preserving its 1 g norm"
    # Rp = Rz(yaw) Ry(pitch) Rx(roll); body rates from its analytic derivative.
    zrate = np.column_stack((np.zeros((n, 2)), yd))
    yrate = np.column_stack((np.zeros(n), pd, np.zeros(n)))
    wp = (_rotate(_rotate(zrate, -pitch, 1), -roll, 0)
          + _rotate(yrate, -roll, 0) + np.column_stack((rd, np.zeros((n, 2)))))
    gp = np.rad2deg(wp)
    # Rd = Rp Rx(-theta): distal body rates and specific force at joint origin.
    gd = _rotate(gp, np.deg2rad(theta), 0)
    gd[:, 0] -= theta_rate
    gravity = np.tile([0., 0., 1.], (n, 1))
    ap = _rotate(_rotate(_rotate(gravity, -yaw, 2), -pitch, 1), -roll, 0)
    ad = _rotate(ap, np.deg2rad(theta), 0)
    if disturbance == "common_acceleration_norm":
        mask = (t >= 4.) & (t < 15.)
        ap[mask] *= 1.8; ad[mask] *= 1.8
    elif disturbance == "distal_acceleration":
        mask = (t >= 5.) & (t < 7.)
        ad[mask] += np.array([.12, .55, .25])
    elif disturbance == "unit_norm_jump":
        mask = (t >= 5.) & (t < 5.15)
        ad[mask] = _rotate(ad[mask], np.full(mask.sum(), np.deg2rad(35.)), 0)
    # Fixed random generator independent of either observer's parameters/state.
    rng = np.random.default_rng(seed + CASE_NAMES.index(name))
    gp[:, 0] += .4*bias; gd[:, 0] -= .6*bias
    gp += rng.normal(0., .08, gp.shape); gd += rng.normal(0., .08, gd.shape)
    ap += rng.normal(0., .0015, ap.shape); ad += rng.normal(0., .0015, ad.shape)
    clipped = np.zeros((n, 2), bool)
    if disturbance == "clip_gyro":
        clipped = np.column_stack((np.any(np.abs(gp) >= 286.7, axis=1), np.any(np.abs(gd) >= 286.7, axis=1)))
        gp = np.clip(gp, -286.7, 286.7); gd = np.clip(gd, -286.7, 286.7)
    return dict(name=name, description=description, time_s=t, truth_angle_deg=theta,
                truth_rate_dps=theta_rate, relative_bias_dps=bias,
                prox_gyro_dps=gp, dist_gyro_dps=gd, prox_acc_g=ap, dist_acc_g=ad,
                near_limit_mask=clipped, neutral_prox_acc_g=[0.,0.,1.],
                neutral_dist_acc_g=[0.,0.,1.], sample_rate_hz=float(sample_rate_hz),
                recovery_events=recovery_events, rising_crossing=crossing)


def observer_factory(name, sample_rate_hz):
    if name == "baseline":
        from .thumb_flexion import ThumbFlexionEstimator
        return ThumbFlexionEstimator(sample_rate_hz)
    if name == "constrained":
        from .hinge_observer import ConstrainedHingeObserver
        return ConstrainedHingeObserver(sample_rate_hz)
    raise ValueError("observer must be baseline or constrained")


def _run_trace(case, name):
    observer = observer_factory(name, case["sample_rate_hz"])
    state = observer.reset(case["neutral_prox_acc_g"], case["neutral_dist_acc_g"],
                           current_proximal_acc=case["prox_acc_g"][0],
                           current_distal_acc=case["dist_acc_g"][0])
    trace = [state]
    for i in range(1, len(case["time_s"])):
        trace.append(observer.update(case["prox_gyro_dps"][i], case["prox_acc_g"][i],
                                     case["dist_gyro_dps"][i], case["dist_acc_g"][i],
                                     proximal_saturated=bool(case["near_limit_mask"][i, 0]),
                                     distal_saturated=bool(case["near_limit_mask"][i, 1])))
    return trace


def _crossing_time(times, angles, event):
    indices = np.flatnonzero((times >= event["start_s"]) & (times <= event["end_s"]))
    for i in indices:
        if i and angles[i-1] < event["angle_deg"] <= angles[i]:
            fraction = (event["angle_deg"]-angles[i-1]) / (angles[i]-angles[i-1])
            return float(times[i-1]+fraction*(times[i]-times[i-1]))
    return None


def _metrics(case, trace):
    angles = np.array([row["angle_deg"] for row in trace])
    error = angles-case["truth_angle_deg"]
    t = case["time_s"]
    result = dict(samples=len(t), error_rms_deg=float(np.sqrt(np.mean(error**2))),
                  error_p95_abs_deg=float(np.percentile(abs(error), 95)),
                  error_max_abs_deg=float(np.max(abs(error))), final_error_deg=float(error[-1]),
                  last_two_seconds_mean_abs_error_deg=float(np.mean(abs(error[t >= t[-1]-2.]))),
                  quality_counts=dict(Counter(row["quality"] for row in trace)),
                  observation_source_counts=dict(Counter(source for row in trace for source in row.get("observation_sources", []))),
                  uncertain_fraction=float(np.mean([row["uncertain"] for row in trace])),
                  history_loss=trace[-1]["history_loss"],
                  final_relative_bias_dps=trace[-1].get("relative_bias_dps"),
                  final_model_angle_std_deg=trace[-1].get("angle_std_deg"), recovery={})
    # Find first full 0.5 s interval within 2 degrees, starting at the event.
    width = max(1, round(.5*case["sample_rate_hz"]))
    good = np.abs(error) <= 2.
    window_count = np.convolve(good.astype(int), np.ones(width, int), mode="valid")
    for event in case["recovery_events"]:
        candidates = np.flatnonzero((t[:len(window_count)] >= event["time_s"])
                                   & (window_count == width))
        result["recovery"][event["name"]] = dict(event_time_s=event["time_s"],
            within_two_deg_for_half_second_after_s=None if not len(candidates)
            else float(t[candidates[0]]-event["time_s"]))
    event = case["rising_crossing"]
    result["rising_crossing_difference_s"] = None
    if event is not None:
        estimated = _crossing_time(t, angles, event)
        reference = _crossing_time(t, case["truth_angle_deg"], event)
        if estimated is not None and reference is not None:
            result["rising_crossing_difference_s"] = estimated-reference
    return result


def benchmark_observers(*, cases=None, sample_rate_hz=120., seed=7, include_series=False):
    """Compare both algorithms on identical synthetic inputs, no pass/fail claim."""
    cases = list(CASE_NAMES if cases is None else cases)
    if not cases or len(set(cases)) != len(cases) or any(name not in CASE_NAMES for name in cases):
        raise ValueError("select nonempty, unique named benchmark cases")
    result = dict(kind="vt_glove_pose.synthetic_observer_benchmark", schema_version=1,
                  source="analytic_synthetic_not_hardware", sample_rate_hz=sample_rate_hz, seed=seed,
                  hardware_accuracy_verified=False, production_acceptance="not_evaluated",
                  assumptions=["Exactly calibrated, synchronized ideal hinge axes and nominal sample timing.",
                               "Known analytic world/body motion, additive measurement noise and scripted disturbances.",
                               "No anatomical error, mounting slip, packet timing or end-to-end hardware latency is modelled.",
                               "All samples, including uncertain states, enter error statistics.",
                               "Recovery and crossing are software scenario metrics, not observed hardware latency.",
                               "Model covariance is not an independently validated error bound."], cases={})
    for name in cases:
        case = make_scenario(name, sample_rate_hz=sample_rate_hz, seed=seed)
        item = dict(description=case["description"], samples=len(case["time_s"]),
                    nominal_duration_s=float(case["time_s"][-1]),
                    raw_clipped_pairs=int(case["near_limit_mask"].any(axis=1).sum()), observers={})
        for observer in ("baseline", "constrained"):
            trace = _run_trace(case, observer)
            item["observers"][observer] = _metrics(case, trace)
            if include_series:
                item["observers"][observer]["series"] = trace
        if include_series:
            item["time_s"] = case["time_s"].tolist()
            item["truth_angle_deg"] = case["truth_angle_deg"].tolist()
        result["cases"][name] = item
    return result
