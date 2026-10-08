"""Natural-motion calibration and angle replay, restricted to offline captures.

This joins the experimental gyro-axis solver to the existing relative-gravity
observer. It does not install live mounts, learn an anatomical flexion sign,
change the observer gains, or claim that quiet accelerometers prove a true zero.
The caller supplies an entire independent, explicitly declared neutral capture.
"""
from __future__ import annotations

from collections import Counter
import math
from numbers import Real

import numpy as np

from .hinge_axes import fit_hinge_axes
from .hinge_frames import build_hinge_frames
from .hinge_settings import (effective_gyro_scales, hinge_settings_match,
                             normalize_hinge_settings, settings_from_units)
from .gyro_scale_diagnostic import gyro_scale_diagnostic
from .thumb_flexion import ThumbFlexionEstimator


_KIND = "vt_glove_pose.offline_hinge_profile"


def _number(value, name, low, high):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return value


def _array(value, name, shape=None):
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf":
        raise ValueError(f"{name} must be real numeric data")
    result = raw.astype(float)
    if (shape is not None and result.shape != shape) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} has invalid shape or nonfinite values")
    return result


def _capture(capture, accel_scale):
    """Use the pair reader's data contract, with explicit configured units."""
    metadata = capture["metadata"]
    rate = _number(metadata["sample_rate_hz"], "sample_rate_hz", 10, 2000)
    gyro_scale = _number(metadata["gyro_dps_per_lsb"], "gyro_dps_per_lsb", 1e-6, 10)
    nodes = [metadata["proximal"], metadata["distal"]]
    if any(type(n) is not int or not 1 <= n <= 12 for n in nodes) or nodes[0] == nodes[1]:
        raise ValueError("invalid proximal/distal node IDs")
    if (nodes[0]-1)//6 != (nodes[1]-1)//6:
        raise ValueError("cross-group pairing is not supported")
    gp = _array(capture["prox_gyro_dps"], "prox_gyro_dps")
    if gp.ndim != 2 or gp.shape[1] != 3 or len(gp) > 30000:
        raise ValueError("paired data must have shape (N, 3), N <= 30000")
    gd = _array(capture["dist_gyro_dps"], "dist_gyro_dps", gp.shape)
    ap = _array(capture["prox_acc_raw"], "prox_acc_raw", gp.shape) * accel_scale
    ad = _array(capture["dist_acc_raw"], "dist_acc_raw", gp.shape) * accel_scale
    near = np.asarray(capture["near_limit_mask"])
    if near.shape != (len(gp), 2) or near.dtype.kind != "b":
        raise ValueError("near_limit_mask must be boolean (N, 2)")
    times = _array(capture["host_monotonic_s"], "host_monotonic_s", (len(gp), 2))
    if np.any(np.diff(times, axis=0) < 0) or np.any(times[:, 0] != times[:, 1]):
        raise ValueError("pair host timestamps are not ordered and matched")
    settings = settings_from_units(metadata, nodes, accel_g_per_lsb=accel_scale)
    source_config = metadata.get("source_configuration")
    if source_config is not None:
        if not hinge_settings_match(settings, settings_from_units(source_config, nodes)):
            raise ValueError("explicit configuration differs from recorded nodes, rate or effective units")
    return gp, gd, ap, ad, near, times[:, 0], settings


def calibrate_hinge_pair(neutral_capture, motion_capture, *, accel_g_per_lsb):
    """Create an offline profile from declared neutral + natural hinge motion.

    Quiet-input checks are experimental, not proof of anatomical straightness,
    no linear acceleration, firmware scale, physical timing or rigid mounting.
    A rejected candidate is returned as a report, not as a fake calibration.
    """
    scale = _number(accel_g_per_lsb, "accel_g_per_lsb", 1e-7, 1.)
    npg, ndg, npa, nda, nnear, nt, settings = _capture(neutral_capture, scale)
    pg, dg, pa, da, near, times, motion_settings = _capture(motion_capture, scale)
    if not hinge_settings_match(settings, motion_settings):
        raise ValueError("neutral and motion must use the same nodes, rate and units")
    result = dict(kind=_KIND, schema_version=1, offline_only=True,
                  profile_status="insufficient_neutral", usable_as_live_calibration=False,
                  settings=settings, source={"neutral": dict(neutral_capture["metadata"]),
                                             "motion": dict(motion_capture["metadata"])},
                  neutral=None, axis_fit=None, frames=None, reason_codes=[],
                  anatomical_flexion_sign_verified=False, anatomical_zero_verified=False,
                  hardware_synchronization_verified=False, production_acceptance="not_evaluated")
    rate = settings["sample_rate_hz"]
    if len(npg) < max(120, math.ceil(2 * rate)):
        result["reason_codes"] = ["declare_at_least_two_nominal_seconds_of_neutral"]
        return result
    bp, bd = npg.mean(axis=0), ndg.mean(axis=0)
    ap0, ad0 = npa.mean(axis=0), nda.mean(axis=0)
    gyro_std = [float(np.linalg.norm(g.std(axis=0))) for g in (npg, ndg)]
    acc_std = [float(np.linalg.norm(a.std(axis=0))) for a in (npa, nda)]
    mean_norm = [float(np.linalg.norm(a)) for a in (ap0, ad0)]
    result["neutral"] = dict(prox_bias_dps=bp.tolist(), dist_bias_dps=bd.tolist(),
                             prox_acc_g=ap0.tolist(), dist_acc_g=ad0.tolist(),
                             sample_count=len(npg), gyro_vector_std_dps=gyro_std,
                             accel_vector_std_g=acc_std, mean_accel_norm_g=mean_norm,
                             reference_scope="explicit_user_declared_pose_not_anatomical_zero")
    reasons = []
    if nnear.any():
        reasons.append("neutral_raw_near_limit")
    if max(gyro_std) > 1. or max(acc_std) > .03 or any(not .85 <= n <= 1.15 for n in mean_norm):
        reasons.append("neutral_not_quiet_or_gravity_like")
    if np.any(np.diff(nt) > max(.1, 5./rate)):
        reasons.append("neutral_host_gap")
    if reasons:
        result.update(profile_status="neutral_invalid", reason_codes=reasons)
        return result
    # This independent consistency check is evidence only. It neither corrects
    # configured sensitivity nor accepts/rejects the axis/frame candidate.
    result["gyro_scale_diagnostics"] = {}
    for role, node, gyro, acceleration, neutral_gyro, sensitivity in zip(
            ("proximal", "distal"), settings["node_ids"], (pg, dg), (pa, da), (npg, ndg),
            effective_gyro_scales(settings, settings["node_ids"])):
        diagnostic = gyro_scale_diagnostic(gyro, acceleration, times, neutral_gyro_dps=neutral_gyro)
        diagnostic.update(node_id=node, configured_gyro_dps_per_lsb=sensitivity)
        result["gyro_scale_diagnostics"][role] = diagnostic
    if len(pg) < max(240, math.ceil(2 * rate)):
        result.update(profile_status="insufficient_motion", reason_codes=["too_few_motion_pairs"])
        return result
    if near.any():
        result.update(profile_status="input_near_limit", reason_codes=["motion_raw_near_limit"])
        return result
    axes = fit_hinge_axes(pg-bp, dg-bd, sample_rate_hz=rate)
    result["axis_fit"] = axes
    if axes["candidate_status"] != "candidate_supported":
        result.update(profile_status="axis_" + axes["candidate_status"], reason_codes=axes["reason_codes"])
        return result
    frames = build_hinge_frames(axes["axis_prox"], axes["axis_dist"], pa, da,
                               neutral_prox_acc_g=ap0, neutral_dist_acc_g=ad0)
    result["frames"] = frames
    if frames["status"] != "frames_ready":
        result.update(profile_status="frames_"+frames["status"], reason_codes=frames["reason_codes"])
        return result
    result.update(profile_status="offline_candidate_ready",
                  reason_codes=["experimental_profile_requires_independent_motion_validation"])
    return result


def _profile(profile):
    if (not isinstance(profile, dict) or profile.get("kind") != _KIND
            or profile.get("schema_version") != 1 or profile.get("offline_only") is not True
            or profile.get("usable_as_live_calibration") is not False
            or profile.get("profile_status") != "offline_candidate_ready"):
        raise ValueError("replay requires a ready offline hinge profile, not a live calibration or rejected candidate")
    try:
        settings, frames, neutral = profile["settings"], profile["frames"], profile["neutral"]
        settings = normalize_hinge_settings(settings)
        nodes = settings["node_ids"]
        if (not isinstance(nodes, list) or len(nodes) != 2
                or any(type(n) is not int or not 1 <= n <= 12 for n in nodes)
                or nodes[0] == nodes[1] or (nodes[0]-1)//6 != (nodes[1]-1)//6):
            raise ValueError("offline profile node IDs are invalid")
        if frames["status"] != "frames_ready" or profile["axis_fit"]["candidate_status"] != "candidate_supported":
            raise ValueError("profile has no supported axis/frame candidate")
        matrices = [_array(frames[name], name, (3, 3)) for name in
                    ("prox_sensor_to_functional", "dist_sensor_to_functional")]
        for matrix in matrices:
            if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-7, rtol=0) or not math.isclose(np.linalg.det(matrix), 1., abs_tol=1e-7):
                raise ValueError("profile functional frames must be proper rotations")
        vectors = [_array(neutral[name], name, (3,)) for name in
                   ("prox_bias_dps", "dist_bias_dps", "prox_acc_g", "dist_acc_g")]
        _number(settings["sample_rate_hz"], "sample_rate_hz", 10, 2000)
        _number(settings["gyro_dps_per_lsb"], "gyro_dps_per_lsb", 1e-6, 10.)
        _number(settings["accel_g_per_lsb"], "accel_g_per_lsb", 1e-7, 1.)
        return settings, matrices, vectors
    except (KeyError, TypeError) as exc:
        raise ValueError("offline hinge profile is incomplete") from exc


def replay_hinge_pair(profile, capture, *, observer="baseline"):
    """Produce mathematical relative-angle samples, no live state changes.

    The default observer is unchanged. The explicitly selected constrained
    candidate estimates relative bias and uses hinge observations; neither
    route constitutes a claim that true-glove drift/latency is already fixed.
    Host gaps are flagged, not interpreted as hardware sample intervals.
    """
    settings, (rp, rd), (bp, bd, ap0, ad0) = _profile(profile)
    pg, dg, pa, da, near, times, actual = _capture(capture, settings["accel_g_per_lsb"])
    if not hinge_settings_match(settings, actual):
        raise ValueError("replay capture nodes, rate or units differ from the offline profile")
    if observer == "baseline":
        observer_kind = "existing_thumb_paired_gyro_gravity_v1_unchanged"
        observer = ThumbFlexionEstimator(settings["sample_rate_hz"])
    elif observer == "constrained":
        from .hinge_observer import ConstrainedHingeObserver
        observer_kind = "constrained_hinge_angle_bias_v1_experimental"
        observer = ConstrainedHingeObserver(settings["sample_rate_hz"])
    else:
        raise ValueError("observer must be baseline or constrained")
    result = dict(kind="vt_glove_pose.offline_hinge_replay", schema_version=1,
                  offline_only=True, live_calibration_changed=False,
                  replay_status="empty_capture", settings=settings,
                  source=dict(capture["metadata"]), series=[], summary={},
                  time_basis="nominal_sample_order_not_device_timestamps",
                  angle_convention="mathematical_positive_about_canonical_proximal_axis_relative_to_declared_neutral",
                  anatomical_flexion_sign_verified=False, production_acceptance="not_evaluated",
                  observer=observer_kind,
                  independent_angle_truth_available=False,
                  warnings=["Host times do not prove synchronization or loss-free sampling.",
                            "No anatomical angle error or end-to-end latency is evaluated.",
                            "Profile and test must come from separate motions for independent validation."])
    if not len(pg):
        return result
    pg, dg = (pg-bp) @ rp.T, (dg-bd) @ rd.T
    pa, da = pa @ rp.T, da @ rd.T
    try:
        state = observer.reset(rp @ ap0, rd @ ad0,
                               current_proximal_acc=pa[0], current_distal_acc=da[0])
    except ValueError as exc:
        result.update(replay_status="initial_gravity_unobservable", reason=str(exc))
        return result
    rate = settings["sample_rate_hz"]
    dt = 1./rate
    gaps = np.r_[False, np.diff(times) > max(.1, 5.*dt)]
    # The extractor cannot locate discarded candidates after returning only
    # matched pairs. Flag uncertain history conservatively from the beginning.
    extraction_loss = capture["metadata"].get("dropped_pair_candidates", 0) > 0
    if extraction_loss:
        state = observer.update(None, None, None, None, pair_valid=False)
        result["warnings"].append("Input extraction discarded pair candidates; loss location is unknown.")
    counts = Counter()
    for i in range(len(pg)):
        if gaps[i]:
            observer.update(None, None, None, None, pair_valid=False)
        if i or near[i].any():
            # The first orientation came from first-sample gravity; do not
            # integrate its rate once more, which would shift angle by one dt.
            if i:
                state = observer.update(pg[i], pa[i], dg[i], da[i],
                                        proximal_saturated=bool(near[i, 0]), distal_saturated=bool(near[i, 1]))
            else:
                state = observer.update(None, None, None, None, pair_valid=False)
        counts[state["quality"]] += 1
        result["series"].append(dict(sample_index=i, nominal_time_s=i*dt,
                                     host_elapsed_s=float(times[i]-times[0]),
                                     host_gap_before=bool(gaps[i]), raw_near_limit=bool(near[i].any()),
                                     **state))
    result.update(replay_status="completed", summary={
        "samples": len(pg), "nominal_span_s": (len(pg)-1)*dt,
        "quality_counts": dict(counts), "host_gap_events": int(gaps.sum()),
        "raw_near_limit_rows": int(near.any(axis=1).sum()),
        "history_loss": state["history_loss"], "final_angle_deg": state["angle_deg"],
        "final_relative_bias_dps": state.get("relative_bias_dps"),
        "final_model_angle_std_deg": state.get("angle_std_deg"),
        "observation_source_counts": dict(Counter(source for row in result["series"]
                                                  for source in row.get("observation_sources", []))),
        "angle_range_deg": [min(s["angle_deg"] for s in result["series"]),
                            max(s["angle_deg"] for s in result["series"])],
    })
    return result


def compare_hinge_pair(profile, capture):
    """Replay exactly the same data/calibration through old and new observers.

    Agreement/disagreement is not anatomical accuracy: this function has no
    independent angle reference and deliberately never declares a winner.
    """
    baseline = replay_hinge_pair(profile, capture, observer="baseline")
    candidate = replay_hinge_pair(profile, capture, observer="constrained")
    comparison = dict(kind="vt_glove_pose.offline_hinge_comparison", schema_version=1,
                      offline_only=True, live_calibration_changed=False,
                      independent_angle_truth_available=False, production_acceptance="not_evaluated",
                      source=dict(capture["metadata"]), baseline=baseline, constrained=candidate,
                      disagreement=None,
                      interpretation="Differences compare algorithms, not accuracy. Neither near-zero output nor agreement proves a correct hand pose.")
    if baseline["replay_status"] == candidate["replay_status"] == "completed":
        a = np.array([s["angle_deg"] for s in baseline["series"]])
        b = np.array([s["angle_deg"] for s in candidate["series"]])
        diff = b-a
        comparison["disagreement"] = dict(samples=len(diff), candidate_minus_baseline_mean_deg=float(diff.mean()),
                                          rms_deg=float(np.sqrt(np.mean(diff**2))),
                                          p95_abs_deg=float(np.percentile(abs(diff), 95)))
    return comparison
