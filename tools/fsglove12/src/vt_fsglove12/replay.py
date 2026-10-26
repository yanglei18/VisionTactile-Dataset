"""Offline raw-IMU replay using upstream VQF, without a hand/joint solver.

Each board has its own unobserved initial heading. Host read timestamps only
schedule output and detect visible discontinuities; every input sample advances
its board's filter by the configured nominal period, including batched reads.
"""

from __future__ import annotations

import csv
from itertools import groupby
import json
import math
from numbers import Real
from pathlib import Path

import numpy as np

from .sensor_quality import GyroNearRailMonitor, quality_warnings
from .input_contract import describe_input_contract, effective_input_scales, raw_to_si, validate_input_contract
from .input_diagnostics import InputObservationMonitor, describe_filter_state
from .input_guard import ImuInputGuard, REVISION as INPUT_GUARD_REVISION
from .vqf_settings import vqf_parameters


_TIME = "host_parse_monotonic_s"
_WALL = "host_parse_wall_s"
_VALUES = ("gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb")
_REQUIRED = {_TIME, _WALL, "node_id", *_VALUES}
_WARNINGS = ["independent_initial_heading", "nominal_sample_period_assumed",
             "host_timestamps_do_not_verify_sensor_synchronization"]


def _number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{name} must be finite" + (" and positive" if positive else ""))
    return value


def _node_key(value):
    if not isinstance(value, str) or value not in {str(n) for n in range(1, 13)}:
        raise ValueError("node IDs must be canonical strings '1' through '12'")
    return int(value)


def _configuration(config):
    if not isinstance(config, dict):
        raise ValueError("config must be a dictionary")
    defaults = dict(sample_rate_hz=120., gyro_dps_per_lsb=.00875,
                    accel_g_per_lsb=.000122, stale_after_s=.1, output_rate_hz=30.)
    settings = {key: _number(config.get(key, default), key, positive=True)
                for key, default in defaults.items()}
    settings["filter_reset_gap_s"] = _number(
        config.get("filter_reset_gap_s", max(1., settings["stale_after_s"])),
        "filter_reset_gap_s", positive=True)
    if settings["filter_reset_gap_s"] < settings["stale_after_s"]:
        raise ValueError("filter_reset_gap_s must be >= stale_after_s")
    overrides = config.get("gyro_dps_per_lsb_by_node", {})
    if not isinstance(overrides, dict):
        raise ValueError("gyro_dps_per_lsb_by_node must be a dictionary")
    scales = {str(n): settings["gyro_dps_per_lsb"] for n in range(1, 13)}
    for key, value in overrides.items():
        _node_key(key)
        scales[key] = _number(value, f"gyro scale for node {key}", positive=True)
    return settings, scales


def _record_gap(diagnostics, node, gap, stamp, *, reset):
    """Persist forward-gap decisions; clock rollback is tracked separately.

    Call only for gaps above the display-staleness threshold. Counts describe
    host arrival intervals, not verified device loss; no samples are fabricated.
    ``filter_resets`` counts gap-triggered resets, not global clock rollbacks.
    """
    item = diagnostics.setdefault(node, dict(short_gaps=0, filter_resets=0))
    item["filter_resets" if reset else "short_gaps"] += 1
    item.update(last_gap_s=gap, last_gap_timestamp_s=stamp,
                last_action="filter_reset" if reset else "filter_preserved")


def _raw_rows(reader):
    for line, row in enumerate(reader, 2):
        try:
            if None in row or any(value is None for value in row.values()):
                raise ValueError("wrong CSV column count")
            node = _node_key(row["node_id"])
            stamp, wall = float(row[_TIME]), float(row[_WALL])
            if not math.isfinite(stamp) or stamp < 0 or not math.isfinite(wall):
                raise ValueError("nonfinite or negative monotonic timestamp")
            values = [int(row[key]) for key in _VALUES]
            if any(not -32768 <= value <= 32767 for value in values):
                raise ValueError("raw values must fit signed 16-bit integers")
            if row.get("status", "") != "" and not 0 <= int(row["status"]) <= 255:
                raise ValueError("status must fit an unsigned byte")
        except (ValueError, TypeError) as exc:
            raise ValueError(f"invalid raw CSV row {line}: {exc}") from exc
        yield stamp, node, values


def replay_raw(raw_path, config: dict, output_path, *, max_frames=None) -> dict:
    """Write a new JSONL file of fresh independent six-axis orientations.

    Output is exclusive: existing paths are never replaced. Invalid data raises
    an exception; an already-created partial output is retained, never reported
    as a completed replay. ``sample_index`` is a cumulative 1-based per-board
    count. ``epoch`` changes whenever that board's VQF is restarted. Visibility
    uses ``stale_after_s``; only arrivals exceeding ``filter_reset_gap_s`` reset
    VQF. Short gaps preserve nominal-sample integration, not a claim of verified
    continuity. Host intervals cannot detect hidden USB/firmware sample loss.
    """
    validate_input_contract(config)
    parameters = vqf_parameters(config)
    settings, scales = _configuration(config)
    scales, accel_scales = effective_input_scales(config)
    if max_frames is not None and (type(max_frames) is not int or max_frames <= 0):
        raise ValueError("max_frames must be a positive integer or None")
    source, target = Path(raw_path), Path(output_path)
    if source.resolve() == target.resolve():
        raise ValueError("output must not replace the raw input")
    # Lazy optional import: reading exported JSONL needs no VQF installation.
    from vqf import VQF

    period = 1. / settings["sample_rate_hz"]
    output_period = 1. / settings["output_rate_hz"]
    stale_after = settings["stale_after_s"]
    counts = {str(n): dict(samples=0, gaps=0, short_gaps=0, clock_resets=0,
                          stale_frames=0, missing_frames=0, epochs=0)
              for n in range(1, 13)}
    states = {}
    epochs = {str(n): 0 for n in range(1, 13)}
    last_input = last_output = None
    frames = rows = resets = 0
    stop_reason = "eof"
    pending_warnings = set()
    gap_diagnostics = {}
    quality_monitor = GyroNearRailMonitor()
    input_observation = InputObservationMonitor(settings["sample_rate_hz"])
    input_guard = ImuInputGuard(config)
    input_guard_rejections = 0

    with source.open("r", encoding="utf-8-sig", newline="") as raw:
        reader = csv.DictReader(raw, strict=True)
        header = reader.fieldnames
        if (header is None or len(header) != len(set(header))
                or not _REQUIRED <= set(header)
                or set(header) - _REQUIRED - {"status", "phase"}):
            raise ValueError("raw CSV must contain timestamps, node_id and six raw IMU fields")
        with target.open("x", encoding="utf-8", newline="\n") as output:
            # All samples in the same host read are applied before output. They
            # are NOT deduplicated: several physical frames may share a stamp.
            for stamp, batch in groupby(_raw_rows(reader), key=lambda row: row[0]):
                clock_reset = last_input is not None and stamp < last_input
                if clock_reset:
                    resets += 1
                    pending_warnings.add("host_clock_reset_filters_reinitialized")
                    for key in epochs:
                        epochs[key] += 1
                        if key in states:
                            counts[key]["clock_resets"] += 1
                    states.clear()
                    input_guard.reset()
                for _, node, values in batch:
                    rows += 1
                    key = str(node)
                    previous = states.get(key)
                    if previous is not None and stamp - previous["stamp"] > stale_after:
                        gap = stamp - previous["stamp"]
                        reset = gap > settings["filter_reset_gap_s"]
                        _record_gap(gap_diagnostics, key, gap, stamp, reset=reset)
                        if reset:
                            epochs[key] += 1
                            counts[key]["gaps"] += 1
                            pending_warnings.add(f"node_{node}_gap_filter_reinitialized")
                            previous = None
                            input_guard.reset(key)
                        else:
                            counts[key]["short_gaps"] += 1
                            pending_warnings.add(f"node_{node}_short_gap_filter_preserved")
                    if previous is None:
                        previous = dict(filter=VQF(period, **parameters), stamp=stamp)
                        states[key] = previous
                    gyro, accel = raw_to_si(values, gyro_dps_per_lsb=scales[key],
                                           accel_g_per_lsb=accel_scales[key])
                    used_gyro, used_accel, protection = input_guard.process(
                        key, values, gyro, accel, gyro_bias_rad_s=previous["filter"].getBiasEstimate()[0])
                    if protection["gyro_rejected"] or protection["accel_rejected"]:
                        input_guard_rejections += 1
                    previous["filter"].update(used_gyro, used_accel)
                    q = np.asarray(previous["filter"].getQuat6D(), dtype=float)
                    if q.shape != (4,) or not np.all(np.isfinite(q)) or not np.isclose(np.linalg.norm(q), 1.):
                        raise ValueError(f"VQF returned an invalid quaternion for node {node}")
                    quality_monitor.observe(key, values[:3], stamp)
                    input_observation.observe(key, values, stamp)
                    counts[key]["samples"] += 1
                    counts[key]["epochs"] = epochs[key] + 1
                    previous.update(stamp=stamp, quat_xyzw=q[[1, 2, 3, 0]].tolist(),
                                    raw_values=list(values), gyro_rad_s=gyro.tolist(), accel_m_s2=accel.tolist())
                    if protection["enabled"]:
                        previous.update(input_filter=protection,
                                        fusion_gyro_rad_s=used_gyro.tolist(),
                                        fusion_accel_m_s2=used_accel.tolist())
                last_input = stamp
                if (last_output is not None and not clock_reset
                        and stamp - last_output < output_period - 1e-9):
                    continue
                nodes = {}
                quality = quality_monitor.snapshot(stamp)
                frame_warnings = list(_WARNINGS) + sorted(pending_warnings) + quality_warnings(quality)
                for key in counts:
                    state = states.get(key)
                    if state is None:
                        counts[key]["missing_frames"] += 1
                        continue
                    age = stamp - state["stamp"]
                    if age > stale_after:
                        counts[key]["stale_frames"] += 1
                        frame_warnings.append(f"node_{key}_stale")
                        continue
                    nodes[key] = dict(quat_xyzw=state["quat_xyzw"], age_s=age,
                                      raw_values=state["raw_values"], gyro_rad_s=state["gyro_rad_s"],
                                      accel_m_s2=state["accel_m_s2"],
                                      fusion_state=describe_filter_state(state["filter"]),
                                      sample_index=counts[key]["samples"], epoch=epochs[key])
                    if "input_filter" in state:
                        nodes[key].update(input_filter=dict(state["input_filter"]),
                            orientation_valid=state["input_filter"]["orientation_valid"],
                            fusion_gyro_rad_s=list(state["fusion_gyro_rad_s"]),
                            fusion_accel_m_s2=list(state["fusion_accel_m_s2"]))
                frame = dict(schema_version=1, timestamp_s=stamp, frame_index=frames,
                             orientation_source="vqf_6d",
                             orientation_frame="independent_initial_heading",
                             nodes=nodes, warnings=frame_warnings, gap_diagnostics=gap_diagnostics,
                             sensor_quality=quality, input_observation=input_observation.snapshot(detailed=False))
                if input_guard.settings["enabled"]:
                    frame["input_filter"] = dict(enabled=True, revision=INPUT_GUARD_REVISION,
                        settings=dict(input_guard.settings),
                        rejected_samples=input_guard_rejections,
                        nodes={key: dict(value["input_filter"]) for key, value in nodes.items()},
                        active_nodes=[key for key, value in nodes.items()
                                      if value["input_filter"]["gyro_rejected"] or value["input_filter"]["accel_rejected"]])
                output.write(json.dumps(frame, allow_nan=False, separators=(",", ":")) + "\n")
                frames += 1
                last_output = stamp
                pending_warnings.clear()
                if max_frames is not None and frames >= max_frames:
                    stop_reason = "max_frames"
                    break
    quality = quality_monitor.snapshot(last_input if last_input is not None else 0.)
    return dict(schema_version=1, kind="vt_fsglove12.orientation_replay", status="completed",
                stop_reason=stop_reason, raw_path=str(source.resolve()),
                output_path=str(target.resolve()), input_samples=rows, frames=frames,
                nodes=counts, host_clock_resets=resets, settings=settings,
                gap_diagnostics=gap_diagnostics,
                sensor_quality=quality,
                input_filter=dict(enabled=input_guard.settings["enabled"], revision=INPUT_GUARD_REVISION,
                    settings=dict(input_guard.settings),
                                  rejected_samples=input_guard_rejections),
                input_contract=describe_input_contract(config), input_observation=input_observation.snapshot(),
                used_gyro_dps_per_lsb_by_node=scales,
                used_accel_g_per_lsb_by_node=accel_scales,
                used_accel_g_per_lsb=settings["accel_g_per_lsb"],
                nominal_time_assumption="each per-node sample advances by 1/sample_rate_hz, including equal host stamps",
                hardware_synchronization_verified=False, common_heading_established=False,
                units_firmware_verified=False, warnings=list(_WARNINGS) + quality_warnings(quality))


def iter_orientation_frames(path):
    """Read and validate replay frames without inventing missing node poses."""
    expected_index = 0
    with Path(path).open("r", encoding="utf-8") as source:
        for line, text in enumerate(source, 1):
            if not text.strip():
                continue
            try:
                frame = json.loads(text)
                if (not isinstance(frame, dict) or type(frame.get("schema_version")) is not int
                        or frame["schema_version"] != 1
                        or type(frame.get("frame_index")) is not int
                        or frame["frame_index"] != expected_index
                        or frame.get("orientation_source") != "vqf_6d"
                        or frame.get("orientation_frame") != "independent_initial_heading"):
                    raise ValueError("unsupported frame schema, source or index")
                if _number(frame.get("timestamp_s"), "timestamp_s") < 0:
                    raise ValueError("timestamp_s must not be negative")
                if not isinstance(frame.get("nodes"), dict):
                    raise ValueError("nodes must be a dictionary")
                if not isinstance(frame.get("warnings"), list) or any(
                        not isinstance(warning, str) for warning in frame["warnings"]):
                    raise ValueError("warnings must be a list of strings")
                for key, node in frame["nodes"].items():
                    _node_key(key)
                    if not isinstance(node, dict):
                        raise ValueError("node must be a dictionary")
                    quat = node.get("quat_xyzw")
                    if not isinstance(quat, list) or len(quat) != 4:
                        raise ValueError("quat_xyzw must have four components")
                    q = [_number(value, "quaternion component") for value in quat]
                    if not math.isclose(sum(value * value for value in q), 1., abs_tol=1e-5):
                        raise ValueError("quat_xyzw must have unit norm")
                    if _number(node.get("age_s"), "age_s") < 0:
                        raise ValueError("age_s must not be negative")
                    for name, minimum in (("sample_index", 1), ("epoch", 0)):
                        if type(node.get(name)) is not int or node[name] < minimum:
                            raise ValueError(f"{name} must be an integer >= {minimum}")
            except (ValueError, TypeError, KeyError) as exc:
                raise ValueError(f"invalid orientation frame on line {line}: {exc}") from exc
            expected_index += 1
            yield frame
