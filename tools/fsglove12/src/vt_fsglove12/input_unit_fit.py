"""Offline gyro sensitivity evidence from labelled stationary gravity endpoints.

Only a candidate configuration is returned. Raw counts, source recordings,
calibration profiles and live hardware are never changed. Gravity constrains
tilt, not absolute heading, and the fitted gain is conditional on nominal dt.
"""
from __future__ import annotations

import copy
import csv
import hashlib
import math
from pathlib import Path

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation

from .input_contract import describe_input_contract, effective_input_scales


_LIMITS = dict(min_stationary_samples=12, min_stationary_duration_s=.2,
               max_gyro_rms_variation_dps=3., max_stationary_gyro_median_dps=5.,
               max_accel_rms_variation_g=.05, min_accel_norm_g=.75, max_accel_norm_g=1.25,
               max_bias_gyro_rms_variation_dps=.15, max_bias_accel_rms_variation_g=.015,
               max_endpoint_bias_difference_dps=.3,
               min_tilt_change_deg=15., max_transition_duration_s=30.,
               max_receive_gap_s=.1, max_rate_relative_error=.15,
               candidate_residual_max_deg=3., min_improvement_deg=3., min_improvement_fraction=.5,
               dyadic_relative_tolerance=.10, near_boundary_integer=32760)


def _windows(values, *, minimum=3):
    if not isinstance(values, list) or len(values) < minimum:
        raise ValueError(f"at least {minimum} labelled stationary windows are required")
    result = []
    for value in values:
        if not isinstance(value, dict):
            raise ValueError("each stationary window must be an object")
        start, end = value.get("start_s"), value.get("end_s")
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in (start, end)):
            raise ValueError("window start_s/end_s must be finite numbers")
        if start < 0 or end <= start or result and start <= result[-1]["end_s"]:
            raise ValueError("stationary windows must be nonnegative, ordered and nonoverlapping")
        label = value.get("label")
        if not isinstance(label, str) or not label.strip():
            raise ValueError("each stationary window requires a nonempty label")
        basis = value.get("time_basis", "relative_to_recording_start")
        if basis not in ("relative_to_recording_start", "absolute_monotonic"):
            raise ValueError("unsupported stationary window time_basis")
        if result and result[0]["time_basis"] != basis:
            raise ValueError("stationary windows must use one time_basis")
        result.append(dict(start_s=float(start), end_s=float(end), label=label, time_basis=basis))
    return result


def _read_prefix(raw_path, windows, max_samples, bias_window):
    from .replay import _REQUIRED, _raw_rows
    hasher = hashlib.sha256()
    consumed = rows = kept = 0
    origin = None
    normalized_bias = None
    stop = "eof"
    nodes = {str(n): [] for n in range(1, 13)}
    previous, rollbacks = {}, {str(n): 0 for n in range(1, 13)}
    # Do not retain minutes of unrelated motion separating labelled trials.
    spans = [(w["start_s"], w["end_s"]) for w in windows]
    final_end = max(end for _, end in spans)
    spans += [(a["start_s"], b["end_s"]) for a, b in zip(windows, windows[1:])
              if (b["start_s"] + b["end_s"] - a["start_s"] - a["end_s"]) / 2 <= _LIMITS["max_transition_duration_s"]]
    with Path(raw_path).open("rb") as stream:
        def lines():
            nonlocal consumed
            for index, raw in enumerate(stream):
                if not raw.endswith(b"\n"):
                    raise ValueError("raw CSV prefix ends with an incomplete line")
                hasher.update(raw)
                consumed += len(raw)
                yield raw.decode("utf-8-sig" if index == 0 else "utf-8")
        reader = csv.DictReader(lines(), strict=True)
        header = reader.fieldnames
        if (header is None or len(header) != len(set(header)) or not _REQUIRED <= set(header)
                or set(header) - _REQUIRED - {"status", "phase"}):
            raise ValueError("raw CSV must contain timestamps, node_id and six raw IMU fields")
        for stamp, node, values in _raw_rows(reader):
            rows += 1
            key = str(node)
            if origin is None:
                origin = stamp
                if bias_window is not None:
                    shift = (0. if bias_window["time_basis"] == windows[0]["time_basis"]
                             else origin if windows[0]["time_basis"] == "absolute_monotonic" else -origin)
                    normalized_bias = {**bias_window, "start_s": bias_window["start_s"] + shift,
                                       "end_s": bias_window["end_s"] + shift, "time_basis": windows[0]["time_basis"]}
                    spans.append((normalized_bias["start_s"], normalized_bias["end_s"]))
                    final_end = max(final_end, normalized_bias["end_s"])
            if key in previous and stamp < previous[key]:
                rollbacks[key] += 1
            previous[key] = stamp
            when = stamp if windows[0]["time_basis"] == "absolute_monotonic" else stamp - origin
            if when > final_end:
                stop = "window_end_lookahead"
                break
            if any(start <= when <= end for start, end in spans):
                nodes[key].append((when, *values))
                kept += 1
            if max_samples is not None and rows >= max_samples:
                stop = "sample_limit"
                break
    return nodes, dict(source=str(Path(raw_path).resolve()), source_prefix_bytes=consumed,
        source_prefix_sha256=hasher.hexdigest(), input_samples=rows, retained_samples=kept,
        source_time_origin_s=origin, stop_reason=stop, per_node_clock_rollbacks=rollbacks,
        normalized_bias_window=normalized_bias,
        prefix_includes_end_lookahead=stop == "window_end_lookahead")


def _angle(a, b):
    return float(np.rad2deg(np.arccos(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1., 1.))))


def _stationary(data, window, gyro_scale, accel_scale, rate):
    selected = data[(data[:, 0] >= window["start_s"]) & (data[:, 0] <= window["end_s"])]
    result = dict(label=window["label"], start_s=window["start_s"], end_s=window["end_s"],
                  samples=len(selected), accepted=False, reasons=[])
    minimum = max(_LIMITS["min_stationary_samples"], math.ceil(rate * _LIMITS["min_stationary_duration_s"]))
    if len(selected) < minimum:
        result["reasons"].append("insufficient_stationary_samples")
        return result, None
    stamps = selected[:, 0]
    if stamps[-1] - stamps[0] < _LIMITS["min_stationary_duration_s"] * .9:
        result["reasons"].append("insufficient_stationary_span")
    if np.max(np.diff(stamps)) > _LIMITS["max_receive_gap_s"]:
        result["reasons"].append("stationary_receive_gap")
    if np.any(np.abs(selected[:, 1:]) >= _LIMITS["near_boundary_integer"]):
        result["reasons"].append("encoding_boundary_event")
    gyro, accel = selected[:, 1:4] * gyro_scale, selected[:, 4:7] * accel_scale
    gm, am = np.median(gyro, axis=0), np.median(accel, axis=0)
    gv = float(np.sqrt(np.mean(np.sum((gyro - gm) ** 2, axis=1))))
    av = float(np.sqrt(np.mean(np.sum((accel - am) ** 2, axis=1))))
    result.update(gyro_median_dps=gm.tolist(), gyro_rms_variation_dps=gv,
                  accel_median_g=am.tolist(), accel_rms_variation_g=av, accel_norm_g=float(np.linalg.norm(am)))
    if gv > _LIMITS["max_gyro_rms_variation_dps"] or np.linalg.norm(gm) > _LIMITS["max_stationary_gyro_median_dps"]:
        result["reasons"].append("gyro_not_stationary")
    if av > _LIMITS["max_accel_rms_variation_g"]:
        result["reasons"].append("accel_not_stationary")
    if not _LIMITS["min_accel_norm_g"] <= np.linalg.norm(am) <= _LIMITS["max_accel_norm_g"]:
        result["reasons"].append("gravity_magnitude_inconsistent_with_config")
    result["accepted"] = not result["reasons"]
    center = (window["start_s"] + window["end_s"]) / 2
    return result, dict(time=center, gravity=am, bias_lsb=np.median(selected[:, 1:4], axis=0))


def _transition(data, start, end, left, right, rate, gyro_scale, independent_bias):
    result = dict(from_label=left["label"], to_label=right["label"],
                  start_center_s=(left["start_s"] + left["end_s"]) / 2,
                  end_center_s=(right["start_s"] + right["end_s"]) / 2, accepted=False, reasons=[])
    if not left["accepted"] or not right["accepted"]:
        result["reasons"] = ["stationary_endpoint_rejected"]
        return result, None
    span = end["time"] - start["time"]
    result["duration_s"] = span
    if span > _LIMITS["max_transition_duration_s"]:
        result["reasons"] = ["transition_too_long"]
        return result, None
    # Acceleration anchors describe stationary window centres; integrate only
    # the corresponding centre-to-centre sample intervals, not full windows.
    selected = data[(data[:, 0] > start["time"]) & (data[:, 0] <= end["time"])]
    result["samples"] = len(selected)
    tilt = _angle(start["gravity"], end["gravity"])
    result["endpoint_tilt_change_deg"] = tilt
    if tilt < _LIMITS["min_tilt_change_deg"]:
        result["reasons"].append("insufficient_tilt_excitation")
    if not len(selected):
        result["reasons"].append("no_transition_samples")
        return result, None
    gaps = np.diff(np.r_[start["time"], selected[:, 0], end["time"]])
    result["max_receive_gap_s"] = float(np.max(gaps))
    result["nominal_duration_to_host_span_ratio"] = len(selected) / rate / span
    if np.min(gaps) < 0:
        result["reasons"].append("clock_rollback")
    if np.max(gaps) > _LIMITS["max_receive_gap_s"]:
        result["reasons"].append("transition_receive_gap")
    if abs(len(selected) / rate / span - 1) > _LIMITS["max_rate_relative_error"]:
        result["reasons"].append("nominal_sample_count_inconsistent_with_host_span")
    if np.any(np.abs(selected[:, 1:]) >= _LIMITS["near_boundary_integer"]):
        result["reasons"].append("encoding_boundary_event")
    if result["reasons"]:
        return result, None
    if independent_bias is None:
        if (max(left["gyro_rms_variation_dps"], right["gyro_rms_variation_dps"])
                > _LIMITS["max_bias_gyro_rms_variation_dps"]
                or max(left["accel_rms_variation_g"], right["accel_rms_variation_g"])
                > _LIMITS["max_bias_accel_rms_variation_g"]
                or np.linalg.norm((start["bias_lsb"] - end["bias_lsb"]) * gyro_scale)
                > _LIMITS["max_endpoint_bias_difference_dps"]):
            result["reasons"] = ["endpoint_motion_cannot_establish_gyro_bias_provide_independent_bias_window"]
            return result, None
        fraction = ((selected[:, 0] - start["time"]) / span)[:, None]
        bias = start["bias_lsb"] * (1 - fraction) + end["bias_lsb"] * fraction
        result["bias_source"] = "strict_stationary_endpoints_linear_interpolation"
    else:
        bias = independent_bias
        result["bias_source"] = "explicit_independent_stationary_window"
    rotvec = -np.deg2rad((selected[:, 1:4] - bias) * gyro_scale) / rate
    g0, g1 = start["gravity"], end["gravity"]
    def residual(gain):
        vector = g0.copy()
        for step in Rotation.from_rotvec(rotvec * gain):
            vector = step.apply(vector)
        return _angle(vector, g1)
    result.update(accepted=True, nominal_residual_deg=residual(1.),
                  gyro_bias_start_lsb=start["bias_lsb"].tolist(), gyro_bias_end_lsb=end["bias_lsb"].tolist())
    return result, residual


def fit_input_units(raw_path, config, stationary_windows, *, max_samples=None, bias_window=None):
    """Fit one transition, validate later transitions; return candidates only.

    Window times default to seconds from the recording's first host timestamp;
    set every window's ``time_basis='absolute_monotonic'`` for absolute stamps.
    At least three ordered, nonoverlapping, explicitly labelled windows are
    required. Bad channels/transitions are reported, not silently repaired.
    An optional independent ``bias_window`` uses the same labelled dictionary
    with either supported time basis; the source origin resolves mixed bases.
    Without it endpoint windows must meet stricter bias tests.
    """
    if max_samples is not None and (type(max_samples) is not int or max_samples <= 0):
        raise ValueError("max_samples must be a positive integer")
    contract = describe_input_contract(config)
    windows = _windows(stationary_windows)
    if bias_window is not None:
        bias_window = _windows([bias_window], minimum=1)[0]
    scales, accels = effective_input_scales(config)
    rate = float(config.get("sample_rate_hz", 120.))
    raw, source = _read_prefix(raw_path, windows, max_samples, bias_window)
    reports, recommended, supported = {}, [], []
    candidate = copy.deepcopy(config)
    for key in map(str, range(1, 13)):
        result = dict(status="rejected", reasons=[], nominal_gyro_dps_per_lsb=scales[key],
                      candidate_gyro_dps_per_lsb=None, continuous_gain=None, recommended_gain=None,
                      nominal_sample_rate_hz=rate, samples=len(raw[key]), stationary_windows=[], transitions=[])
        reports[key] = result
        data = np.asarray(raw[key], dtype=float).reshape(-1, 7)
        if not len(data):
            result["reasons"] = ["missing_node_samples"]
            continue
        if source["per_node_clock_rollbacks"][key] or np.any(np.diff(data[:, 0]) < 0):
            result["reasons"] = ["clock_rollback"]
            continue
        independent_bias = None
        if bias_window is not None:
            bias_stats, bias_anchor = _stationary(data, source["normalized_bias_window"], scales[key], accels[key], rate)
            bias_stats["purpose"] = "gyro_bias_estimation_stricter_stationarity"
            result["bias_window"] = bias_stats
            if bias_stats["accepted"]:
                if bias_stats["gyro_rms_variation_dps"] > _LIMITS["max_bias_gyro_rms_variation_dps"]:
                    bias_stats["reasons"].append("gyro_bias_window_variation_too_large")
                if bias_stats["accel_rms_variation_g"] > _LIMITS["max_bias_accel_rms_variation_g"]:
                    bias_stats["reasons"].append("accel_bias_window_variation_too_large")
                bias_stats["accepted"] = not bias_stats["reasons"]
            if not bias_stats["accepted"]:
                result["reasons"] = ["independent_bias_window_not_stationary"]
                continue
            independent_bias = bias_anchor["bias_lsb"]
        anchors = []
        for window in windows:
            stats, anchor = _stationary(data, window, scales[key], accels[key], rate)
            result["stationary_windows"].append(stats)
            anchors.append(anchor)
        usable = []
        for i in range(len(windows) - 1):
            stats, residual = _transition(data, anchors[i], anchors[i+1],
                                         result["stationary_windows"][i], result["stationary_windows"][i+1], rate, scales[key], independent_bias)
            result["transitions"].append(stats)
            if residual is not None:
                usable.append((stats, residual))
        if len(usable) < 2:
            result["reasons"] = ["need_fit_and_independent_heldout_transition"]
            continue
        fit = minimize_scalar(lambda gain: usable[0][1](gain) ** 2, bounds=(.35, 2.5), method="bounded",
                              options=dict(xatol=1e-5, maxiter=80))
        gain = float(fit.x)
        result["continuous_gain"] = gain
        dyadic = min((.5, 1., 2.), key=lambda value: abs(gain - value) / value)
        result["tested_dyadic_gain"] = dyadic
        for index, (stats, residual) in enumerate(usable):
            stats.update(role="fit" if index == 0 else "heldout", continuous_fit_residual_deg=residual(gain),
                         candidate_residual_deg=residual(dyadic))
        if not fit.success or abs(gain / dyadic - 1) > _LIMITS["dyadic_relative_tolerance"]:
            result["reasons"] = ["continuous_gain_not_supported_dyadic_candidate"]
            continue
        if any(stats["candidate_residual_deg"] >= _LIMITS["candidate_residual_max_deg"] for stats, _ in usable):
            result["reasons"] = ["candidate_fails_fit_or_heldout_gravity_residual"]
            continue
        if dyadic == 1.:
            result.update(status="nominal_supported", recommended_gain=1., candidate_gyro_dps_per_lsb=scales[key])
            supported.append(key)
            continue
        if any(stats["nominal_residual_deg"] - stats["candidate_residual_deg"] < max(
                _LIMITS["min_improvement_deg"], stats["nominal_residual_deg"] * _LIMITS["min_improvement_fraction"])
               for stats, _ in usable):
            result["reasons"] = ["insufficient_independent_improvement"]
            continue
        result.update(status="recommended", recommended_gain=dyadic, candidate_gyro_dps_per_lsb=scales[key] * dyadic)
        recommended.append(key)
        candidate.setdefault("gyro_dps_per_lsb_by_node", {})[key] = scales[key] * dyadic
    if recommended:
        candidate["units_verified"] = False
        if "input_contract" in candidate:
            candidate["input_contract"]["scale_source"] = "empirical_validation"
        candidate["input_unit_fit_evidence"] = dict(source_prefix_sha256=source["source_prefix_sha256"],
            source_prefix_bytes=source["source_prefix_bytes"], source=source["source"],
            recommended_nodes=recommended.copy(), nominal_sample_rate_hz=rate,
            stationary_windows=copy.deepcopy(windows), bias_window=copy.deepcopy(bias_window),
            normalized_bias_window=copy.deepcopy(source["normalized_bias_window"]),
            original_provenance=copy.deepcopy(config.get("provenance")),
            method="stationary_gravity_endpoints_first_transition_fit_independent_heldout",
            physical_range_verified=False)
        applied = ", ".join(f"node {key}={reports[key]['candidate_gyro_dps_per_lsb']:g} dps/LSB" for key in recommended)
        candidate["provenance"] = (
            "Offline candidate per-node gyro sensitivity correction: " + applied + ". "
            "Supported by labelled stationary gravity endpoints with independent held-out transitions; "
            f"conditional on configured nominal dt=1/{rate:g} s, not sensor full-scale register readback. "
            "Unlisted per-node sensitivities are unchanged; this is not verification of all 12 IMUs, "
            "hardware sampling or pose accuracy. Source provenance is retained in input_unit_fit_evidence. "
            "Applying this candidate requires explicit review and a new calibration.")
    return dict(schema_version=1, kind="vt_fsglove12.input_unit_fit", status="completed", **source,
        stationary_windows=windows, bias_window=bias_window, input_contract=contract, thresholds=dict(_LIMITS), nodes=reports,
        recommended_nodes=recommended, nominal_supported_nodes=supported,
        rejected_nodes=[key for key, item in reports.items() if item["status"] == "rejected"],
        candidate_config=candidate, raw_values_modified=False, hardware_accessed=False,
        automatic_config_changes=False, physical_range_verified=False, pose_accuracy_verified=False,
        note="Candidate effective gyro sensitivity is conditional on configured per-node nominal period and labelled "
             "stationary gravity endpoints; it does not identify sensor FS registers, firmware rescaling, absolute "
             "heading, or distinguish every sampling-timing error. Candidates require explicit review and a new calibration.")
