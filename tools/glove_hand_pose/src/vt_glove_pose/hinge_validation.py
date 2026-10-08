"""Offline, operator-labelled hinge repeatability/stability statistics.

These are experimental candidate criteria, not measured anatomical accuracy or
product acceptance. Windows use the replay's ``host_elapsed_s`` clock and are
half-open [start_s, end_s). The caller must align marker and CSV origins before
calling. Host parse time is not device sample time or end-to-end latency.

Every finite angle is included, including prediction-only and uncertain rows.
Quality only determines whether a window has enough support for a conclusion;
it never removes inconvenient angles from a metric. No external reference is
consumed in this version. A declared pose is not verified physical ground truth.
"""
from __future__ import annotations

import html
import math
from numbers import Real

import numpy as np


_KINDS = {"reference", "return", "hold", "common_rotation"}
_MARKER_STATUSES = {"completed", "interrupted", "cancelled"}
_MIN_WINDOW_S = .5
_MIN_COVERAGE = .9
_MIN_OBSERVED = .5
_LIMITS = {"return": 5., "hold": 3., "common_rotation": 5.}
_LIMITATIONS = [
    "这些阈值是实验候选配置，不是产业精度指标或产品验收认证。",
    "所有有限角度均参与统计，包括预测、低质量和历史缺失帧；质量标志不是真值。",
    "同姿态参考与动作均由操作者声明，代码不证明真的回到同一姿态、保持静止或完成整手共转。",
    "回位重复性、保持变化及共转稳定性不等于绝对解剖角度精度；中立也不强制等于伸直。",
    "时间窗基于主机解析时间；样本率是配置值，覆盖率不能证明硬件同步、无丢帧或端到端延迟。",
    "历史缺失是锁存标志；当前可观察与过去完整性分别报告，恢复观测不会补回过去轨迹。",
    "本版本未接入独立角度真值或物理延迟测量，即使提供 reference_source 也不据此判定精度。",
    "校准与验证应使用独立动作；短窗口仅满足统计计算下限，不代表产品场景充分。",
    "常值或冻结输出也可能看似稳定；需结合实际屈伸幅度和独立参考。算法输出的变化不是独立动作真值。",
]


def _number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite real number")
    return value


def _flag(row, name, default=False):
    value = row.get(name, default)
    if not isinstance(value, (bool, np.bool_)):
        raise ValueError(f"series {name} must be boolean")
    return bool(value)


def _replay_arrays(report):
    if not isinstance(report, dict) or not isinstance(report.get("settings"), dict):
        raise ValueError("replay_report must contain settings and series")
    rate = _number(report["settings"].get("sample_rate_hz"), "sample_rate_hz")
    if not 10 <= rate <= 2000:
        raise ValueError("sample_rate_hz must be in [10, 2000]")
    rows = report.get("series")
    if not isinstance(rows, (list, tuple)) or len(rows) > 30000:
        raise ValueError("replay series must contain at most 30000 rows")
    values = {key: [] for key in ("time", "angle", "valid", "observed", "prediction",
                                  "history_loss", "near_limit", "host_gap")}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("replay series rows must be dictionaries")
        values["time"].append(_number(row.get("host_elapsed_s"), "host_elapsed_s"))
        angle = row.get("angle_deg")
        if angle is None:
            angle = float("nan")
        elif isinstance(angle, (bool, np.bool_)) or not isinstance(angle, Real):
            raise ValueError("angle_deg must be real or None")
        # Nonfinite angles are counted as missing rather than discarded silently.
        values["angle"].append(float(angle))
        observed = (_flag(row, "current_angle_observed")
                    if "current_angle_observed" in row else
                    _flag(row, "acc_observable") and _flag(row, "correction_active"))
        values["observed"].append(observed)
        values["prediction"].append(not observed)
        for key, source in (("valid", "valid"), ("history_loss", "history_loss"),
                            ("near_limit", "raw_near_limit"), ("host_gap", "host_gap_before")):
            values[key].append(_flag(row, source))
    arrays = {key: np.asarray(value, dtype=float if key in ("time", "angle") else bool)
              for key, value in values.items()}
    if np.any(np.diff(arrays["time"]) < 0) or np.any(arrays["time"] < 0):
        raise ValueError("host_elapsed_s must be nonnegative and ordered (ties are allowed)")
    return rate, arrays


def _windows(windows):
    if not isinstance(windows, (list, tuple)) or len(windows) > 256:
        raise ValueError("windows must contain at most 256 entries")
    result = []
    for index, window in enumerate(windows):
        if (not isinstance(window, dict) or not isinstance(window.get("kind"), str)
                or window.get("kind") not in _KINDS):
            raise ValueError("window kind must be reference, return, hold or common_rotation")
        pose = window.get("pose_id")
        if not isinstance(pose, str) or not pose.strip():
            raise ValueError("window pose_id must be a nonempty string")
        start = _number(window.get("start_s"), "start_s")
        end = _number(window.get("end_s"), "end_s")
        if end <= start or not math.isfinite(end - start):
            raise ValueError("window end_s must be greater than start_s")
        status = window.get("status", "completed")
        if not isinstance(status, str) or status not in _MARKER_STATUSES:
            raise ValueError("window status must be completed, interrupted or cancelled")
        item = dict(index=index, kind=window["kind"], pose_id=pose,
                    start_s=start, end_s=end, marker_status=status)
        for name in ("window_id", "label", "note"):
            if name in window:
                if not (isinstance(window[name], str) or
                        (name == "window_id" and type(window[name]) is int)):
                    raise ValueError(f"window {name} must be a string (window_id may also be an integer)")
                item[name] = window[name]
        result.append(item)
    return result


def _finite_float(value):
    """Keep reports JSON-safe even for extreme, invalid estimator magnitudes."""
    value = float(value)
    return value if math.isfinite(value) else None


def _angles(values):
    if not len(values):
        return {name: None for name in ("median_deg", "min_deg", "max_deg",
                                        "range_deg", "p95_range_deg")}
    # Long double prevents subtraction/quantile overflow for finite bad inputs.
    v = values.astype(np.longdouble)
    lo, hi = np.quantile(v, [.025, .975])
    return dict(median_deg=_finite_float(np.median(v)), min_deg=float(v.min()),
                max_deg=float(v.max()), range_deg=_finite_float(v.max() - v.min()),
                p95_range_deg=_finite_float(hi - lo))


def _window_report(window, arrays, rate, completed):
    start, end = window["start_s"], window["end_s"]
    indexes = np.flatnonzero((arrays["time"] >= start) & (arrays["time"] < end))
    times = arrays["time"][indexes]
    raw = arrays["angle"][indexes]
    finite = np.isfinite(raw)
    angles = raw[finite]
    n = len(indexes)
    duration = end - start
    # A half-open window normally ends one nominal sample after its final row.
    tolerance = max(.02, 2. / rate)
    span = float(times[-1] - times[0]) if n else 0.
    coverage = min(1., (span + 1. / rate) / duration) if n else 0.
    expected = duration * rate
    sample_coverage = min(1., n / expected) if math.isfinite(expected) else 0.
    interior_gaps = int(np.count_nonzero((np.diff(times) > max(.1, 5. / rate)) |
                                        arrays["host_gap"][indexes[1:]])) if n > 1 else 0
    quality = {f"{name}_fraction": float(arrays[name][indexes].mean()) if n else 0.
               for name in ("valid", "observed", "prediction", "history_loss", "near_limit")}
    quality["finite_fraction"] = float(finite.mean()) if n else 0.
    reasons = []
    if not completed:
        reasons.append("replay_not_completed")
    if window["marker_status"] != "completed":
        reasons.append("marker_not_completed")
    if duration < _MIN_WINDOW_S - 1e-9:
        reasons.append("window_shorter_than_0_5s")
    if window["kind"] == "hold" and duration < 60. - 1e-9:
        reasons.append("hold_shorter_than_60s")
    if n < max(3, math.ceil(_MIN_WINDOW_S * rate * _MIN_COVERAGE)):
        reasons.append("too_few_samples")
    if not n or times[0] - start > tolerance + 1e-9 or end - times[-1] > tolerance + 1e-9:
        reasons.append("window_endpoints_not_covered")
    if coverage < _MIN_COVERAGE:
        reasons.append("insufficient_host_span_coverage")
    if sample_coverage < _MIN_COVERAGE:
        reasons.append("insufficient_nominal_sample_coverage")
    if int(finite.sum()) != n or not n:
        reasons.append("missing_or_nonfinite_angles")
    if quality["observed_fraction"] < _MIN_OBSERVED or quality["valid_fraction"] < _MIN_OBSERVED:
        reasons.append("insufficient_current_observation_support")
    if interior_gaps:
        reasons.append("interior_host_gap")
    if quality["near_limit_fraction"]:
        reasons.append("raw_near_limit_in_window")
    stats = _angles(angles)
    if len(angles) and any(value is None for value in stats.values()):
        reasons.append("angle_statistics_overflow")
    report = dict(window, status="insufficient_data" if reasons else "sufficient_data",
                  data_sufficient=not reasons, reasons=reasons,
                  sample_count=n, finite_angle_count=int(finite.sum()),
                  angle=stats, quality=quality,
                  coverage=dict(requested_duration_s=duration, host_span_s=span,
                                first_host_elapsed_s=float(times[0]) if n else None,
                                last_host_elapsed_s=float(times[-1]) if n else None,
                                host_span_fraction=coverage,
                                expected_nominal_samples=_finite_float(expected),
                                nominal_sample_fraction=sample_coverage,
                                interior_host_gap_count=interior_gaps,
                                endpoint_tolerance_s=tolerance))
    return report, indexes, angles


def _deviations(angles, baseline):
    if baseline is None or not len(angles):
        return dict(median_deviation_deg=None, max_abs_deviation_deg=None,
                    p95_abs_deviation_deg=None)
    delta = angles.astype(np.longdouble) - np.longdouble(baseline)
    return dict(median_deviation_deg=_finite_float(np.median(delta)),
                max_abs_deviation_deg=_finite_float(np.abs(delta).max()),
                p95_abs_deviation_deg=_finite_float(np.quantile(np.abs(delta), .95)))


def _edge_median(arrays, indexes, boundary, *, first):
    t, a = arrays["time"][indexes], arrays["angle"][indexes]
    selected = ((t < boundary + 1.) if first else (t >= boundary - 1.)) & np.isfinite(a)
    return _finite_float(np.median(a[selected].astype(np.longdouble))) if selected.any() else None


def _criterion(windows, *, minimum_count, limit):
    adequate = sum(window["data_sufficient"] for window in windows)
    violations = sum(window.get("comparison", {}).get("within_candidate_limit") is False
                     for window in windows)
    if adequate < minimum_count or adequate != len(windows):
        status = "insufficient_data"
    else:
        status = "fail" if violations else "pass"
    maxima = [w["comparison"]["max_abs_deviation_deg"] for w in windows
              if w.get("comparison", {}).get("max_abs_deviation_deg") is not None]
    return dict(status=status, candidate_limit_deg=limit, minimum_window_count=minimum_count,
                window_count=len(windows), adequate_window_count=adequate,
                numeric_violation_window_count=violations,
                worst_max_abs_deviation_deg=max(maxima) if maxima else None)


def analyze_hinge_validation(replay_report, windows, *, reference_source=None):
    """Assess labelled *repeatability/stability*, never absolute accuracy.

    ``windows`` is a sequence of dictionaries with kind, pose_id, start_s and
    end_s. Optional status is completed (default), interrupted or cancelled;
    incomplete windows still receive metrics but cannot support a pass. Finite
    negative starts are permitted so missing recording heads remain visible.

    ``reference`` is an operator-declared pose, not independent truth. The first
    chronological reference of each pose is used, never a later better one.
    At least 20 nonoverlapping, adequately covered returns are required. Holds
    require 60 seconds. A common_rotation window without a preceding reference
    evaluates only change from its own first second. Actions are not recognized
    or verified. ``reference_source`` is reserved; supplied data is not consumed.

    Malformed inputs raise ValueError; inadequate data produces a report.
    There is no IO, observer update, calibration change or hardware operation.
    """
    rate, arrays = _replay_arrays(replay_report)
    declared = _windows(windows)
    reports, samples, values = [], [], []
    for window in declared:
        item, indexes, angles = _window_report(window, arrays, rate,
                                              replay_report.get("replay_status") == "completed")
        reports.append(item)
        samples.append(indexes)
        values.append(angles)
    references = {}
    for item in sorted(reports, key=lambda w: (w["start_s"], w["index"])):
        if item["kind"] == "reference":
            references.setdefault(item["pose_id"], item)
    returns = [item for item in reports if item["kind"] == "return"]
    # Mark both sides of an overlap: never select a favorable duplicate.
    for position, item in enumerate(returns):
        if any(max(item["start_s"], other["start_s"]) < min(item["end_s"], other["end_s"])
               for j, other in enumerate(returns) if j != position):
            item["reasons"].append("overlapping_return_windows")
            item["data_sufficient"] = False
    for item, indexes, angles in zip(reports, samples, values):
        kind = item["kind"]
        if kind == "reference":
            item["status"] = "reference_ready" if item["data_sufficient"] else "insufficient_data"
            continue
        reference = references.get(item["pose_id"])
        prior = reference is not None and reference["end_s"] <= item["start_s"]
        baseline = None
        baseline_source = None
        reference_index = None
        if kind == "return" or (kind == "common_rotation" and prior):
            if not prior:
                item["reasons"].append("missing_prior_same_pose_reference")
            else:
                baseline = reference["angle"]["median_deg"]
                baseline_source, reference_index = "first_same_pose_reference", reference["index"]
                if not reference["data_sufficient"]:
                    item["reasons"].append("first_same_pose_reference_insufficient")
        else:
            baseline = _edge_median(arrays, indexes, item["start_s"], first=True)
            baseline_source = "window_first_second"
        comparison = dict(baseline_deg=baseline, baseline_source=baseline_source,
                          reference_window_index=reference_index,
                          candidate_limit_deg=_LIMITS[kind], **_deviations(angles, baseline))
        if kind == "hold":
            final = _edge_median(arrays, indexes, item["end_s"], first=False)
            change = _finite_float(np.longdouble(final) - np.longdouble(baseline)) \
                if final is not None and baseline is not None else None
            comparison.update(first_second_median_deg=baseline, last_second_median_deg=final,
                              first_to_last_change_deg=change,
                              peak_to_peak_change_deg=item["angle"]["range_deg"])
        if kind == "common_rotation":
            comparison["physical_common_rotation_verified"] = False
            comparison["scope"] = ("operator_declared_same_pose_relative_to_reference" if prior
                                   else "within_window_stability_only")
        maximum = comparison["max_abs_deviation_deg"]
        comparison["within_candidate_limit"] = (maximum <= _LIMITS[kind] if maximum is not None else None)
        if baseline is None or maximum is None:
            item["reasons"].append("comparison_unavailable")
        item["data_sufficient"] = not item["reasons"]
        item["comparison"] = comparison
        item["status"] = ("pass" if comparison["within_candidate_limit"] else "fail") \
            if item["data_sufficient"] else "insufficient_data"
    excursions = 0
    for item in returns:
        reference = references.get(item["pose_id"])
        baseline = item["comparison"]["baseline_deg"]
        interval_start, excursion, count = None, None, 0
        if reference is not None and reference["end_s"] <= item["start_s"] and baseline is not None:
            previous_ends = [other["end_s"] for other in returns
                             if other is not item and other["pose_id"] == item["pose_id"]
                             and other["end_s"] <= item["start_s"]]
            interval_start = max([reference["end_s"], *previous_ends])
            select = ((arrays["time"] >= interval_start) & (arrays["time"] < item["start_s"])
                      & np.isfinite(arrays["angle"]))
            count = int(select.sum())
            excursion = _deviations(arrays["angle"][select], baseline)["max_abs_deviation_deg"]
        item["comparison"].update(excursion_interval_start_s=interval_start,
                                  excursion_interval_end_s=item["start_s"],
                                  excursion_finite_sample_count=count,
                                  excursion_from_reference_max_abs_deg=excursion)
        excursions += int(excursion is not None and excursion >= 10.)
    criteria = {
        "return_repeatability": _criterion(returns, minimum_count=20, limit=5.),
        "hold_stability": _criterion([w for w in reports if w["kind"] == "hold"],
                                     minimum_count=1, limit=3.),
        "common_rotation": _criterion([w for w in reports if w["kind"] == "common_rotation"],
                                      minimum_count=1, limit=5.),
    }
    statuses = [criterion["status"] for criterion in criteria.values()]
    candidate_status = ("fail" if "fail" in statuses else "pass" if all(s == "pass" for s in statuses)
                        else "insufficient_data")
    warnings = []
    if returns and excursions < len(returns):
        warnings.append("some_return_intervals_lack_10deg_output_excursion_evidence_not_physical_truth")
    return dict(kind="vt_glove_pose.offline_hinge_validation", schema_version=1,
                offline_only=True, candidate_status=candidate_status,
                production_acceptance="not_evaluated", observer=replay_report.get("observer"),
                time_basis="host_parse_elapsed_seconds_aligned_by_caller",
                angle_basis="reported_mathematical_angle_no_anatomical_truth",
                sample_rate_hz=rate, replay_sample_count=len(arrays["time"]),
                replay_status=replay_report.get("replay_status"),
                windows=reports, criteria=criteria,
                summary=dict(return_intervals_with_output_excursion=excursions,
                             output_excursion_display_threshold_deg=10.,
                             output_excursion_is_independent_truth=False), warnings=warnings,
                absolute_accuracy={"status": "not_evaluated", "reason": "independent_angle_truth_not_integrated"},
                end_to_end_latency={"status": "not_evaluated", "reason": "no_physical_input_output_timing_reference"},
                reference_source_provided=reference_source is not None,
                external_reference_processing="not_implemented",
                experimental_thresholds=dict(min_window_s=.5, min_hold_s=60., min_return_count=20,
                                             min_coverage_fraction=.9, min_current_observed_fraction=.5,
                                             min_valid_fraction=.5, return_limit_deg=5., hold_limit_deg=3.,
                                             common_rotation_limit_deg=5.),
                metric_definitions=dict(p95_range_deg="Q97.5(angle)-Q2.5(angle), all finite rows",
                                        pass_metric="maximum absolute deviation of every finite angle",
                                        history_loss="latched past loss, not a mask or current-observation flag",
                                        nominal_sample_fraction="observed row count / (declared duration * configured rate), capped at 1",
                                        host_span_fraction="(last-first host timestamp + one nominal period) / duration, capped at 1"),
                limitations=list(_LIMITATIONS))


def render_validation_html(report):
    """Return a self-contained, escaped Chinese HTML report (no IO/network)."""
    if not isinstance(report, dict) or report.get("kind") != "vt_glove_pose.offline_hinge_validation":
        raise ValueError("expected an offline hinge validation report")
    def esc(value):
        return html.escape(str(value), quote=True)

    def number(value):
        return "—" if value is None else esc(f"{value:.3f}")

    statuses = {"pass": "满足实验候选指标", "fail": "未满足候选指标", "insufficient_data": "数据不足，不能判定",
                "reference_ready": "参考数据足够", "not_evaluated": "未评估"}
    names = {"reference": "姿态参考", "return": "回位", "hold": "保持", "common_rotation": "整手共转"}
    criterion_names = {"return_repeatability": "至少 20 次回位重复性", "hold_stability": "至少 60 秒保持稳定性",
                       "common_rotation": "声明同姿态整手共转稳定性"}
    criterion_rows = []
    for name, metric in report["criteria"].items():
        criterion_rows.append("<tr>" + "".join(f"<td>{cell}</td>" for cell in (
            esc(criterion_names.get(name, name)), esc(statuses.get(metric["status"], metric["status"])),
            esc(f'{metric["adequate_window_count"]}/{metric["window_count"]}'),
            number(metric["worst_max_abs_deviation_deg"]), number(metric["candidate_limit_deg"]))) + "</tr>")
    rows = []
    for window in report["windows"]:
        angle, quality = window["angle"], window["quality"]
        comparison = window.get("comparison", {})
        cells = (esc(window.get("label", window.get("window_id", window["index"]))),
                 esc(names[window["kind"]]), esc(window["pose_id"]),
                 f'{number(window["start_s"])}–{number(window["end_s"])}',
                 esc(window["sample_count"]), number(angle["median_deg"]),
                 f'{number(angle["min_deg"])} / {number(angle["max_deg"])}', number(angle["p95_range_deg"]),
                 number(comparison.get("max_abs_deviation_deg")),
                 number(comparison.get("excursion_from_reference_max_abs_deg")),
                 number(comparison.get("first_to_last_change_deg")),
                 " / ".join(number(100 * quality[key]) for key in
                            ("finite_fraction", "prediction_fraction", "valid_fraction", "history_loss_fraction")),
                 number(100 * window["coverage"]["nominal_sample_fraction"]),
                 number(100 * window["coverage"]["host_span_fraction"]),
                 esc(statuses.get(window["status"], window["status"])),
                 esc(window["marker_status"]), esc(", ".join(window["reasons"])))
        rows.append("<tr>" + "".join(f"<td>{cell}</td>" for cell in cells) + "</tr>")
    limits = "".join(f"<li>{esc(item)}</li>" for item in report["limitations"])
    excursions = esc(report.get("summary", {}).get("return_intervals_with_output_excursion", 0))
    provenance = report.get("provenance", {})
    source = {"online": "实际在线逐配对输出（保留录制当时状态，不重新运行算法）",
              "replay": "离线重算（重新初始化观察器，不等同当时在线表现）"}.get(
                  provenance.get("evaluation_source"), "调用者提供的角度序列；来源以 JSON 为准")
    return """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>双 IMU 铰链离线验证</title>
<style>body{font-family:system-ui,sans-serif;margin:2rem;color:#20303a;background:#fafcfd}
table{border-collapse:collapse;width:100%;font-size:.88rem}th,td{border:1px solid #c7d5dc;padding:.45rem;text-align:left}
th{background:#e7eff4}section{overflow:auto}li{margin:.35rem 0}.notice{padding:1rem;background:#fff1d2}</style>
</head><body><h1>双 IMU 铰链离线验证</h1><p class="notice">实验候选结果：""" + esc(
        statuses.get(report["candidate_status"], report["candidate_status"])) + """。
不是产品合格证明。绝对角度精度：未评估；端到端延迟：未评估。</p>
<p>评估来源：""" + esc(source) + """</p>
<p>时间采用已对齐的主机解析相对秒，不是设备时间。角度单位为度。所有有限角度（含预测帧）参与统计。
P95 范围 = Q97.5 − Q2.5；完整最小/最大值同时报告。候选判据采用完整最大偏差，不按 P95 隐藏尖峰。</p>
<p>回位前间隔中，算法输出相对参考变化 ≥10° 的间隔数：""" + excursions + """。
这只是算法输出的运动迹象，不是独立动作真值，不参与上述候选判据。常值或冻结输出也可能看似稳定，
应核对真实屈伸幅度并使用独立参考验证。</p>
<h2>候选指标</h2><section><table><thead><tr><th>指标</th><th>结论</th><th>充分窗口/声明窗口</th>
<th>完整最大绝对偏差 °</th><th>候选上限 °</th></tr></thead><tbody>""" + "".join(criterion_rows) + """</tbody></table></section>
<h2>全部标记窗口</h2><section><table><thead><tr><th>标记</th><th>类型</th><th>姿态</th><th>时间 s</th>
<th>样本</th><th>中位数 °</th><th>最小/最大 °</th><th>P95 范围 °</th><th>最大偏差 °</th>
<th>回位前间隔最大输出幅度 °</th><th>保持首末变化 °</th>
<th>有限/预测/有效/历史缺失 %</th><th>名义样本覆盖 %</th><th>主机跨度覆盖 %</th><th>结论</th><th>原标记状态</th><th>不足原因代码</th>
</tr></thead><tbody>""" + "".join(rows) + """</tbody></table></section><h2>边界与使用限制</h2><ul>""" + limits + "</ul></body></html>"
