"""Offline automatic stationary-window selection inside three completed holds.

No hand dimensions, known rotation angles, fused orientations or hardware are
used. Selection precedes and is independent of the existing gyro/gravity fit.
Results are conditional evidence, never permission to change live units.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np

from .config import digest, file_digest, validate_config, write_json
from .input_contract import effective_input_scales
from . import input_unit_fit
from .live_stream import RawRecorder, RawSample


_WINDOW_S = .5
_STEP_S = .05
_HOLD_S = 2.
_NODES = tuple(map(str, range(1, 13)))
_NOTE = ("无需手部尺寸或精确转角；结果以配置名义 dt 为条件，只检验陀螺积分与静止重力端点的相容性。"
         "不验证传感器量程、实际 ODR、绝对航向或手部姿态准确性；候选配置不会自动应用。")
_REASONS = {
    "missing_node_samples": "该路没有原始样本",
    "insufficient_stationary_samples": "静止窗口样本不足",
    "insufficient_stationary_span": "静止数据覆盖时长不足",
    "stationary_receive_gap": "静止窗口接收间隔过大",
    "stationary_window_incomplete_coverage": "原始数据未充分覆盖所选静止窗口",
    "stationary_sample_count_inconsistent_with_nominal_dt": "静止窗口样本数与名义周期不相容",
    "encoding_boundary_event": "原始编码接近边界，现有离线拟合不对此给出尺度结论",
    "gyro_not_stationary": "陀螺显示窗口内仍有运动",
    "accel_not_stationary": "加速度波动过大",
    "gravity_magnitude_inconsistent_with_config": "静止加速度模长与当前换算不相容",
    "gyro_bias_window_variation_too_large": "陀螺微动过大，不能可靠估计静止偏置",
    "accel_bias_window_variation_too_large": "加速度微动过大，不能可靠估计静止偏置",
    "clock_rollback": "原始时间戳出现倒退",
    "stationary_endpoint_rejected": "至少一个静止端点不合格",
    "transition_too_long": "两端点相隔过长，现有拟合不支持",
    "insufficient_tilt_excitation": "重力方向变化不足；纯航向转动无法用于此检验",
    "no_transition_samples": "缺少两端点之间的原始运动样本",
    "transition_receive_gap": "运动段缺样或接收间隔过大",
    "nominal_sample_count_inconsistent_with_host_span": "运动段样本数与名义周期不相容",
    "endpoint_motion_cannot_establish_gyro_bias_provide_independent_bias_window": "端点偏置不稳定，需更可靠的静止数据",
    "need_fit_and_independent_heldout_transition": "缺少两段合格的独立倾转，无法完成拟合及留出验证",
    "continuous_gain_not_supported_dyadic_candidate": "拟合增益不支持现有 0.5/1/2 倍候选",
    "candidate_fails_fit_or_heldout_gravity_residual": "候选未通过拟合段或独立留出段验证",
    "insufficient_independent_improvement": "候选相对原尺度的改善证据不足",
    "automatic_stationary_selection_rejected": "该路未通过三个所选窗口的严格静止检查",
    "cancelled": "检查已取消；未提供可应用候选",
}


class _Cancelled(Exception):
    pass


def _check_cancelled(cancelled):
    if cancelled is not None and cancelled():
        raise _Cancelled


def _holds(holds):
    if not isinstance(holds, (list, tuple)) or len(holds) != 3:
        raise ValueError("exactly three completed 2 s holds are required")
    if any(not isinstance(hold, dict) or hold.get("time_basis") != "absolute_monotonic" for hold in holds):
        raise ValueError("completed holds require time_basis=absolute_monotonic")
    result = input_unit_fit._windows(copy.deepcopy(list(holds)))
    if any(not math.isclose(hold["end_s"] - hold["start_s"], _HOLD_S, abs_tol=1e-6, rel_tol=0.)
           for hold in result):
        raise ValueError("each completed hold must span exactly 2 s; longer windows are not guessed")
    if len({hold["label"] for hold in result}) != 3:
        raise ValueError("the three completed holds require distinct labels")
    return result


def _strict_stationary(data, window, gyro_scale, accel_scale, rate):
    stats, _ = input_unit_fit._stationary(data, window, gyro_scale, accel_scale, rate)
    limits = input_unit_fit._LIMITS
    if stats.get("gyro_rms_variation_dps", 0.) > limits["max_bias_gyro_rms_variation_dps"]:
        stats["reasons"].append("gyro_bias_window_variation_too_large")
    if stats.get("accel_rms_variation_g", 0.) > limits["max_bias_accel_rms_variation_g"]:
        stats["reasons"].append("accel_bias_window_variation_too_large")
    selected = data[(data[:, 0] >= window["start_s"]) & (data[:, 0] <= window["end_s"])]
    if len(selected):
        stamps = selected[:, 0]
        if np.any(np.diff(stamps) < 0):
            stats["reasons"].append("clock_rollback")
        if (stamps[0] - window["start_s"] > limits["max_receive_gap_s"]
                or window["end_s"] - stamps[-1] > limits["max_receive_gap_s"]):
            stats["reasons"].append("stationary_window_incomplete_coverage")
        if abs(len(selected) / rate / _WINDOW_S - 1.) > limits["max_rate_relative_error"]:
            stats["reasons"].append("stationary_sample_count_inconsistent_with_nominal_dt")
    stats["accepted"] = not stats["reasons"]
    return stats


def _select(samples, config, holds, cancelled):
    arrays = {key: [] for key in _NODES}
    for index, sample in enumerate(samples):
        if index % 1024 == 0:
            _check_cancelled(cancelled)
        arrays[str(sample.node_id)].append((sample.host_monotonic_s, *sample.values))
    arrays = {key: np.asarray(rows, dtype=float).reshape(-1, 7) for key, rows in arrays.items()}
    gyro, accel = effective_input_scales(config)
    rate = float(config["sample_rate_hz"])
    selection = []
    for hold in holds:
        _check_cancelled(cancelled)
        # Only this hold's raw counts influence its choice. In particular,
        # neither transition motion nor fit residuals are examined here.
        inside = {key: data[(data[:, 0] >= hold["start_s"]) & (data[:, 0] <= hold["end_s"])]
                  for key, data in arrays.items()}
        best = None
        candidates = []
        for index in range(round((_HOLD_S - _WINDOW_S) / _STEP_S) + 1):
            _check_cancelled(cancelled)
            start = hold["start_s"] + index * _STEP_S
            end = min(start + _WINDOW_S, hold["end_s"])
            window = dict(label=hold["label"], start_s=start, end_s=end, time_basis="absolute_monotonic")
            nodes = {key: _strict_stationary(data, window, gyro[key], accel[key], rate)
                     for key, data in inside.items()}
            accepted = [key for key, node in nodes.items() if node["accepted"]]
            noise = (float(np.mean([
                (nodes[key]["gyro_rms_variation_dps"] / input_unit_fit._LIMITS["max_bias_gyro_rms_variation_dps"]) ** 2
                + (nodes[key]["accel_rms_variation_g"] / input_unit_fit._LIMITS["max_bias_accel_rms_variation_g"]) ** 2
                for key in accepted])) if accepted else None)
            candidate = dict(window=window, qualified_nodes=accepted, qualified_count=len(accepted), noise_score=noise)
            candidates.append(candidate)
            rank = (-len(accepted), math.inf if noise is None else noise,
                    abs((start + end) / 2 - (hold["start_s"] + hold["end_s"]) / 2), start)
            if best is None or rank < best[0]:
                best = rank, candidate, nodes
        selection.append(dict(hold=hold, selected_window=best[1]["window"],
                              qualified_nodes=best[1]["qualified_nodes"],
                              selection_accepted=bool(best[1]["qualified_nodes"]),
                              nodes=best[2], candidates=candidates))
    return dict(method="max_qualified_nodes_then_min_strict_stationary_noise",
                window_s=_WINDOW_S, step_s=_STEP_S, tie_break="nearest_hold_center_then_earliest",
                fit_residual_used=False, fused_orientation_used=False,
                thresholds=copy.deepcopy(input_unit_fit._LIMITS), holds=selection)


def _empty(config, reason):
    gyro, _ = effective_input_scales(config)
    return dict(schema_version=1, kind="vt_fsglove12.input_scale_check", status="completed",
                nodes={key: dict(status="rejected", reasons=[reason], nominal_gyro_dps_per_lsb=gyro[key],
                                candidate_gyro_dps_per_lsb=None, recommended_gain=None) for key in _NODES},
                nominal_supported_nodes=[], recommended_nodes=[], rejected_nodes=list(_NODES),
                candidate_available=False, candidate_path=None, candidate_config=None, report_path=None)


def _selection_review(report, selection, config):
    qualified = set.intersection(*(set(hold["qualified_nodes"]) for hold in selection["holds"]))
    for key, node in report["nodes"].items():
        if key not in qualified:
            node["fit_status_before_selection_review"] = node["status"]
            node["status"] = "rejected"
            node["reasons"] = list(dict.fromkeys([*node["reasons"], "automatic_stationary_selection_rejected"]))
            node["candidate_gyro_dps_per_lsb"] = node["recommended_gain"] = None
        node["automatic_stationary_reasons"] = list(dict.fromkeys(
            reason for hold in selection["holds"] for reason in hold["nodes"][key]["reasons"]))
    report["recommended_nodes"] = [key for key in _NODES if report["nodes"][key]["status"] == "recommended"]
    report["nominal_supported_nodes"] = [key for key in _NODES if report["nodes"][key]["status"] == "nominal_supported"]
    report["rejected_nodes"] = [key for key in _NODES if report["nodes"][key]["status"] == "rejected"]
    candidate = report["candidate_config"]
    if not report["recommended_nodes"]:
        report["candidate_config"] = None
        return None
    # Preserve every source override, then apply only recommendations that
    # also survived the stricter automatic selection review.
    candidate["gyro_dps_per_lsb_by_node"] = copy.deepcopy(config.get("gyro_dps_per_lsb_by_node", {}))
    for key in report["recommended_nodes"]:
        candidate["gyro_dps_per_lsb_by_node"][key] = report["nodes"][key]["candidate_gyro_dps_per_lsb"]
    candidate["input_unit_fit_evidence"]["recommended_nodes"] = report["recommended_nodes"].copy()
    candidate["provenance"] = ("Offline gyro-scale candidate from automatically selected strict stationary "
        "windows within completed holds; first transition fit and independent held-out verification. "
        f"Recommended nodes: {', '.join(report['recommended_nodes'])}. Conditional on nominal "
        f"dt=1/{config['sample_rate_hz']:g} s; not verification of sensor FS, ODR, absolute heading or pose accuracy. "
        "Original provenance and raw prefix binding are retained in input_unit_fit_evidence. Explicit review "
        "and a new installation calibration are required before use.")
    return candidate


def run_input_scale_check(samples, config, holds, output_dir, *, cancelled=None):
    """Write new offline evidence from immutable, already-recorded RawSamples.

``output_dir`` must not exist. ``cancelled`` is an optional zero-argument
callable. The unchanged fitter has no cancellation hook: cancellation during
its computation is handled immediately on return, before any candidate save.
No subprocess, device, source file or runtime configuration is owned here.
"""
    if not isinstance(samples, tuple) or any(not isinstance(sample, RawSample) for sample in samples):
        raise ValueError("samples must be a tuple of RawSample instances")
    source_config = copy.deepcopy(config)
    validate_config(source_config)
    holds = _holds(holds)
    if cancelled is not None and not callable(cancelled):
        raise ValueError("cancelled must be a zero-argument callable")
    directory = Path(output_dir)
    report = _empty(source_config, "missing_node_samples")
    report.update(raw_values_modified=False, hardware_accessed=False, automatic_config_changes=False,
                  physical_range_verified=False, sample_rate_verified=False, pose_accuracy_verified=False,
                  note=_NOTE, nominal_sample_rate_hz=source_config["sample_rate_hz"],
                  nominal_integration_period_s=1. / source_config["sample_rate_hz"])
    binding = dict(source_config_sha256=digest(source_config), holds_sha256=digest(holds), holds=holds,
                   algorithm="vt_fsglove12.input_unit_fit.fit_input_units",
                   algorithm_source_sha256=file_digest(input_unit_fit.__file__),
                   selector_source_sha256=file_digest(__file__),
                   supplied_samples=len(samples), written_samples=0)
    report["source_binding"] = binding
    created = False
    selection = None
    try:
        _check_cancelled(cancelled)
        directory.mkdir(parents=True, exist_ok=False)
        created = True
        raw_path = directory / "raw.csv"
        recorder = RawRecorder(raw_path).open()
        try:
            for start in range(0, len(samples), 1024):
                _check_cancelled(cancelled)
                chunk = samples[start:start + 1024]
                recorder.write(chunk)
                binding["written_samples"] += len(chunk)
        finally:
            recorder.close()
        binding.update(raw_path=str(raw_path.resolve()), raw_sha256=file_digest(raw_path))
        selection = _select(samples, source_config, holds, cancelled)
        windows = [hold["selected_window"] for hold in selection["holds"]]
        binding["selected_windows_sha256"] = digest(windows)
        _check_cancelled(cancelled)
        fitted = input_unit_fit.fit_input_units(raw_path, source_config, windows)
        _check_cancelled(cancelled)
        candidate = _selection_review(fitted, selection, source_config)
        fitted.pop("kind", None)
        report.update(fitted, selection=selection, source_binding=binding, note=_NOTE)
        report["report_path"] = str((directory / "report.json").resolve())
        if candidate is not None:
            candidate["input_unit_fit_evidence"]["automatic_selection"] = copy.deepcopy(binding)
            report["candidate_config"] = candidate
            pending = directory / "config.candidate.pending.json"
            write_json(pending, candidate)
            _check_cancelled(cancelled)
            pending.rename(directory / "config.candidate.json")
            report.update(candidate_available=True, candidate_path=str((directory / "config.candidate.json").resolve()))
        _check_cancelled(cancelled)
        write_json(directory / "report.pending.json", report)
        _check_cancelled(cancelled)
        (directory / "report.pending.json").rename(directory / "report.json")
        _check_cancelled(cancelled)
        return report
    except _Cancelled:
        report.update(_empty(source_config, "cancelled"), status="cancelled", source_binding=binding)
        if selection is not None:
            report["selection"] = selection
        if created:
            # Only files inside this newly created evidence directory are ours.
            for name in ("config.candidate.json", "config.candidate.pending.json", "report.pending.json"):
                (directory / name).unlink(missing_ok=True)
            report["report_path"] = str((directory / "report.json").resolve())
            write_json(directory / "report.pending.json", report)
            (directory / "report.pending.json").replace(directory / "report.json")
        return report


def scale_check_summary(report):
    """Detached compact UI payload; never promote inconclusive to bad input."""
    nodes = {}
    for key in _NODES:
        node = report.get("nodes", {}).get(key, {})
        status = node.get("status", "rejected")
        codes = list(dict.fromkeys([*node.get("reasons", []), *node.get("automatic_stationary_reasons", []),
            *(reason for phase in ("stationary_windows", "transitions")
              for item in node.get(phase, []) for reason in item.get("reasons", []))]))
        if status == "nominal_supported":
            reason = "当前尺度通过本次倾转与留出检验（以名义 dt 为条件）"
        elif status == "recommended":
            reason = f"建议离线审阅 {node['recommended_gain']:g} 倍有效陀螺增益；未自动应用"
        else:
            reason = "；".join(_REASONS.get(code, "未能建立尺度结论：" + code) for code in codes) or "证据不足，未建立尺度结论"
        nodes[key] = dict(status=status, reason=reason, reason_codes=codes,
                          nominal_gyro_dps_per_lsb=node.get("nominal_gyro_dps_per_lsb"),
                          candidate_gyro_dps_per_lsb=node.get("candidate_gyro_dps_per_lsb"),
                          recommended_gain=node.get("recommended_gain"))
    return dict(status=report.get("status", "completed"), supported_nodes=list(report.get("nominal_supported_nodes", [])),
                recommended_nodes=list(report.get("recommended_nodes", [])),
                inconclusive_nodes=[key for key in _NODES if nodes[key]["status"] not in {"nominal_supported", "recommended"}],
                nodes=nodes, report_path=report.get("report_path"), candidate_path=report.get("candidate_path"),
                candidate_available=bool(report.get("candidate_available", False)), note=_NOTE)
