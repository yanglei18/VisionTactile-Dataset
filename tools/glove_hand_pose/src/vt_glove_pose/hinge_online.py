"""Join recorded online observer outputs to raw pairs without running a filter.

Matching uses wire order AND the complete raw payload/timestamps. Host parse
times alone are not unique: a USB read may contain several different pairs.
Missing outputs remain missing, rather than being replaced by replayed angles.
"""
from __future__ import annotations

from collections import Counter, defaultdict, deque
from copy import deepcopy
import math
from numbers import Real

import numpy as np

from .hinge_settings import (hinge_settings_match, normalize_hinge_settings,
                             settings_from_units)


_TRUNCATED = {"max_pairs", "byte_limit", "default_byte_limit", "partial_line_at_byte_limit"}


def _number(value, name, *, optional=False):
    if value is None and optional:
        return None
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number" + (" or None" if optional else ""))
    return float(value)


def _settings(value):
    return normalize_hinge_settings(value)


def _raw_array(value, shape, name):
    array = np.asarray(value)
    if (array.shape != shape or array.dtype.kind not in "iu"
            or np.any(array < -32768) or np.any(array > 32767)):
        raise ValueError(f"{name} must contain signed 16-bit raw integers of shape {shape}")
    return array


def _capture(capture, settings):
    try:
        metadata = capture["metadata"]
        nodes = [metadata["proximal"], metadata["distal"]]
        actual = settings_from_units(metadata, nodes, accel_g_per_lsb=settings["accel_g_per_lsb"])
        if not hinge_settings_match(actual, settings):
            raise ValueError("capture settings differ from frozen online settings")
        recorded = metadata.get("source_configuration")
        if recorded is not None:
            if not hinge_settings_match(settings_from_units(recorded, nodes), settings):
                raise ValueError("capture source configuration differs for nodes, rate or effective units")
        times = np.asarray(capture["host_monotonic_s"], dtype=float)
        walls = np.asarray(capture["host_wall_s"], dtype=float)
        n = len(times)
        if (n > 30000 or times.shape != (n, 2) or walls.shape != (n, 2)
                or not np.isfinite(times).all() or not np.isfinite(walls).all()
                or np.any(np.diff(times, axis=0) < 0) or np.any(times[:, 0] != times[:, 1])):
            raise ValueError("capture timestamps must be ordered finite paired arrays, N <= 30000")
        gp = _raw_array(capture["prox_gyro_raw"], (n, 3), "prox_gyro_raw")
        gd = _raw_array(capture["dist_gyro_raw"], (n, 3), "dist_gyro_raw")
        ap = _raw_array(capture["prox_acc_raw"], (n, 3), "prox_acc_raw")
        ad = _raw_array(capture["dist_acc_raw"], (n, 3), "dist_acc_raw")
        raw_p, raw_d = np.column_stack((gp, ap)), np.column_stack((gd, ad))
        return metadata, times, walls, raw_p, raw_d
    except (KeyError, TypeError) as exc:
        raise ValueError("capture is missing raw arrays, timestamps or settings") from exc


def _trace_rows(trace, settings):
    if (not isinstance(trace, dict) or trace.get("kind") != "vt_glove_pose.hinge_online_trace"
            or type(trace.get("schema_version")) is not int or trace["schema_version"] != 1):
        raise ValueError("expected a schema_version 1 hinge_online_trace")
    if not hinge_settings_match(_settings(trace.get("settings")), settings):
        raise ValueError("trace settings differ from frozen profile settings")
    discarded = trace.get("discarded_rows")
    if type(discarded) is not int or discarded < 0:
        raise ValueError("discarded_rows must be a nonnegative integer")
    rows = trace.get("series")
    if not isinstance(rows, list):
        raise ValueError("online trace series must be a list")
    previous = 0
    previous_time = -math.inf
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("online trace rows must be dictionaries")
        seq = row.get("pair_sequence")
        if type(seq) is not int or seq <= previous:
            raise ValueError("pair_sequence must be positive and strictly increasing")
        previous = seq  # The first pair may be well beyond 1 in a live session.
        connection = row.get("connection_id")
        if type(connection) is not int or connection <= 0:
            raise ValueError("connection_id must be a positive integer")
        t = _number(row.get("host_monotonic_s"), "host_monotonic_s")
        if t < previous_time:
            raise ValueError("online trace host times must be ordered")
        previous_time = t
        _number(row.get("host_wall_s"), "host_wall_s")
        for name in ("proximal_raw", "distal_raw"):
            _raw_array(row.get(name), (6,), name)
        for name in ("valid", "current_angle_observed", "history_loss", "raw_near_limit", "host_gap_before"):
            if type(row.get(name)) is not bool:
                raise ValueError(f"online trace {name} must be boolean")
        for name in ("angle_deg", "relative_bias_dps", "angle_std_deg", "relative_bias_std_dps"):
            _number(row.get(name), name, optional=True)
        if "angle_deg" not in row or (row["angle_deg"] is None and (row["valid"] or row["current_angle_observed"])):
            raise ValueError("absent angle must be explicit, invalid and unobserved")
        if not isinstance(row.get("quality"), str):
            raise ValueError("online trace quality must be a string")
        for name in ("reason_codes", "observation_sources"):
            if name in row and (not isinstance(row[name], list) or any(not isinstance(s, str) for s in row[name])):
                raise ValueError(f"online trace {name} must be a list of strings")
    return rows, discarded


def _signature(time, wall_p, wall_d, prox, dist):
    return (float(time), float(wall_p), float(wall_d), tuple(int(v) for v in prox), tuple(int(v) for v in dist))


def online_hinge_report(trace, capture, settings):
    """Report actual online angles; never initialize/update/replay an observer.

    Ordered alignment can recover from missing output rows but cannot recover
    their angles. Unmatched outputs are counted separately. Unknown trace loss
    blocks all validation conclusions; identifiable missing rows block their
    own windows through explicit None angles. A bounded capture prefix may
    ignore later trace rows, but its truncation remains visible in the report.
    """
    settings = _settings(settings)
    rows, discarded = _trace_rows(trace, settings)
    metadata, times, walls, raw_p, raw_d = _capture(capture, settings)
    n = len(times)
    rate = settings["sample_rate_hz"]
    positions = defaultdict(deque)
    signatures = []
    for i in range(n):
        key = _signature(times[i, 0], walls[i, 0], walls[i, 1], raw_p[i], raw_d[i])
        signatures.append(key)
        positions[key].append(i)
    # Full signatures index *positions*, never replace/combine equal-time rows.
    raw_counts = Counter(signatures)
    matched_counts = Counter()
    matches = {}
    cursor = unmatched = ignored = hidden_sequence_gaps = 0
    truncated = metadata.get("stop_reason") in _TRUNCATED
    previous_match = None
    for row in rows:
        if truncated and (cursor == n or (n and row["host_monotonic_s"] > times[-1, 0])):
            ignored += 1
            continue
        key = _signature(row["host_monotonic_s"], row["host_wall_s"], row["host_wall_s"],
                         row["proximal_raw"], row["distal_raw"])
        candidates = positions.get(key)
        while candidates and candidates[0] < cursor:
            candidates.popleft()
        if not candidates:
            unmatched += 1
            continue
        index = candidates.popleft()
        if previous_match is not None:
            old_index, old_sequence = previous_match
            hidden_sequence_gaps += max(0, row["pair_sequence"] - old_sequence - (index - old_index))
        previous_match = (index, row["pair_sequence"])
        matches[index] = row
        matched_counts[key] += 1
        cursor = index + 1
    ambiguous = sum(count for key, count in raw_counts.items()
                    if count > 1 and 0 < matched_counts[key] < count)
    series = []
    gaps = np.r_[False, np.diff(times[:, 0]) > max(.1, 5. / rate)] if n else np.zeros(0, bool)
    for i in range(n):
        if i in matches:
            row = deepcopy(matches[i])
        else:
            row = dict(angle_deg=None, valid=False, current_angle_observed=False,
                       quality="missing_online_output", history_loss=False, uncertain=True,
                       relative_bias_dps=None, angle_std_deg=None, observation_sources=[],
                       reason_codes=["missing_online_output"], pair_sequence=None, connection_id=None,
                       host_monotonic_s=float(times[i, 0]), host_wall_s=float(walls[i, 0]),
                       proximal_raw=raw_p[i].tolist(), distal_raw=raw_d[i].tolist(),
                       raw_near_limit=bool(np.any(np.abs(raw_p[i, :3].astype(np.int64)) >= 32700)
                                           or np.any(np.abs(raw_d[i, :3].astype(np.int64)) >= 32700)),
                       host_gap_before=bool(gaps[i]))
        row.update(sample_index=i, nominal_time_s=i / rate,
                   host_elapsed_s=float(times[i, 0] - times[0, 0]))
        series.append(row)
    reasons = []
    for condition, reason in ((discarded, "discarded_online_rows_location_unknown"),
                              (unmatched, "unmatched_online_outputs"),
                              (ambiguous, "identical_payload_missing_output_position_ambiguous"),
                              (hidden_sequence_gaps, "pair_sequence_gaps_not_represented_in_raw_capture"),
                              (metadata.get("dropped_pair_candidates", 0), "raw_pair_extraction_loss_location_unknown")):
        if condition:
            reasons.append(reason)
    status = "incomplete_online_trace" if reasons else "completed" if n else "empty_capture"
    finite_angles = [row["angle_deg"] for row in series if row["angle_deg"] is not None]
    final = series[-1] if series else {}
    summary = dict(samples=n, nominal_span_s=(n - 1) / rate if n else 0.,
                   quality_counts=dict(Counter(row["quality"] for row in series)),
                   host_gap_events=sum(row["host_gap_before"] for row in series),
                   raw_near_limit_rows=sum(row["raw_near_limit"] for row in series),
                   history_loss=any(row["history_loss"] for row in series),
                   final_angle_deg=final.get("angle_deg"),
                   final_relative_bias_dps=final.get("relative_bias_dps"),
                   final_model_angle_std_deg=final.get("angle_std_deg"),
                   observation_source_counts=dict(Counter(source for row in series
                                                          for source in row.get("observation_sources", []))),
                   angle_range_deg=[min(finite_angles), max(finite_angles)] if finite_angles else [None, None],
                   matched_online_rows=len(matches), missing_online_rows=n - len(matches),
                   unmatched_online_rows=unmatched, discarded_online_rows=discarded,
                   ignored_trace_rows_after_capture=ignored, capture_truncated=truncated,
                   ambiguous_raw_rows=ambiguous, unrepresented_sequence_gaps=hidden_sequence_gaps)
    return dict(kind="vt_glove_pose.online_hinge_report", schema_version=1,
                offline_only=False, live_calibration_changed=False, settings=deepcopy(settings),
                replay_status=status, source=deepcopy(metadata), series=series, summary=summary,
                reason_codes=reasons, observer="recorded_actual_online_observer_outputs",
                time_basis="host_parse_time_for_windows_actual_online_observer_outputs",
                angle_convention="mathematical_positive_about_canonical_proximal_axis_relative_to_declared_neutral",
                anatomical_flexion_sign_verified=False, independent_angle_truth_available=False,
                production_acceptance="not_evaluated", warnings=[
                    "Angles are recorded online outputs, not offline recomputation or a fresh zero.",
                    "Missing outputs stay absent; unknown trace loss makes validation insufficient.",
                    "Host parse timestamps do not prove device synchronization or end-to-end latency.",
                    "Capture is a bounded prefix; later online rows were excluded." if truncated else
                    "Agreement with raw input provenance is not independent angle truth."])
