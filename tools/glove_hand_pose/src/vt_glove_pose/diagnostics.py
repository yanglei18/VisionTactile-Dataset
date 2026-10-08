"""Bounded, read-only analysis of explicitly recorded diagnostic sessions.

Reproduction is a software consistency check, never a measurement of anatomical
accuracy, physical return latency, loss-free transport, or absolute heading.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .config import FINGERS, fingerprint, gyro_scale, validate_config
from .records import Sample

MAX_LINE_BYTES = 2_000_000
MAX_BATCH_SAMPLES = 4096
COMPARISON_TOLERANCE = 1e-7


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"Nonfinite JSON number: {value}")


def _bounded_json(value, depth=0):
    if depth > 32:
        raise ValueError("JSON nesting exceeds 32 levels")
    if isinstance(value, dict):
        if len(value) > 4096:
            raise ValueError("JSON object is too large")
        for child in value.values():
            _bounded_json(child, depth + 1)
    elif isinstance(value, list):
        if len(value) > 65536:
            raise ValueError("JSON array is too large")
        for child in value:
            _bounded_json(child, depth + 1)
    elif isinstance(value, (float, int)) and not isinstance(value, bool):
        if not math.isfinite(value) or abs(value) > 1e30:
            raise ValueError("JSON number is nonfinite or out of bounds")


def iter_session(path):
    """Yield strictly ordered events without loading an entire recording."""
    with Path(path).open("rb") as stream:
        sequence = 0
        while True:
            line = stream.readline(MAX_LINE_BYTES + 1)
            if not line:
                return
            if len(line) > MAX_LINE_BYTES:
                raise ValueError(f"Event {sequence}: exceeds {MAX_LINE_BYTES} bytes")
            if not line.endswith(b"\n"):
                raise ValueError(f"Event {sequence}: truncated line (missing newline)")
            try:
                event = json.loads(line, object_pairs_hook=_unique_object,
                                   parse_constant=_reject_constant)
                _bounded_json(event)
            except (ValueError, UnicodeError, RecursionError, OverflowError) as exc:
                raise ValueError(f"Event {sequence}: invalid JSON: {exc}") from exc
            if not isinstance(event, dict) or type(event.get("seq")) is not int or event["seq"] != sequence:
                raise ValueError(f"Event {sequence}: missing or discontinuous sequence number")
            if not isinstance(event.get("type"), str):
                raise ValueError(f"Event {sequence}: missing event type")
            yield event
            sequence += 1


def _finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


class _RawStatistics:
    def __init__(self, config):
        self.config = config
        self.nodes = {}
        self.count = 0

    def add(self, sample):
        node, t = str(sample.node_id), sample.host_monotonic_s
        state = self.nodes.setdefault(node, dict(count=0, first=t, last=t, sum=np.zeros(6),
                                                square=np.zeros(6), acc_sum=0., near_limit=0,
                                                equal_times=0, backwards=0, gaps=0, max_gap=0.))
        if state["count"]:
            gap = t - state["last"]
            state["equal_times"] += int(gap == 0)
            state["backwards"] += int(gap < 0)
            state["gaps"] += int(gap > self.config["stale_after_s"])
            state["max_gap"] = max(state["max_gap"], gap)
        v = np.asarray(sample.values, dtype=float)
        state["count"] += 1
        state["last"] = t
        state["sum"] += v
        state["square"] += v * v
        state["acc_sum"] += float(np.linalg.norm(v[3:]) * self.config["accel_g_per_lsb"])
        state["near_limit"] += int(np.any(np.abs(v[:3]) >= 32700))
        self.count += 1

    def report(self):
        result = {}
        for n in range(1, 13):
            state = self.nodes.get(str(n))
            if state is None:
                result[str(n)] = {"records": 0, "received_rate_hz": None}
                continue
            count = state["count"]
            mean = state["sum"] / count
            span = state["last"] - state["first"]
            rate = (count - 1) / span if span > 0 and not state["backwards"] else None
            result[str(n)] = dict(
                records=count, host_duration_s=span, received_rate_hz=rate,
                raw_mean=mean.tolist(),
                raw_std=np.sqrt(np.maximum(0, state["square"] / count - mean**2)).tolist(),
                mean_accel_norm_g_assumed=state["acc_sum"] / count,
                near_int16_limit_gyro_records=state["near_limit"],
                equal_host_timestamp_records=state["equal_times"],
                backwards_host_timestamp_records=state["backwards"],
                host_gaps_over_stale_threshold=state["gaps"], max_host_gap_s=state["max_gap"])
            if self.config.get("gyro_dps_per_lsb_by_node"):
                result[str(n)]["gyro_dps_per_lsb_assumed"] = gyro_scale(self.config, n)
        return result


def _compare(expected, actual, path="", differences=None):
    """Bound the stored mismatch examples, but count every mismatching leaf."""
    if differences is None:
        differences = {"count": 0, "examples": [], "max_numeric_absolute_error": 0.}
    same = False
    if isinstance(expected, dict) and isinstance(actual, dict):
        for key in sorted(expected.keys() | actual.keys()):
            if key not in expected or key not in actual:
                _compare("<missing key>", "<present key>", path + "." + key, differences)
            else:
                _compare(expected[key], actual[key], path + "." + key, differences)
        return differences
    if isinstance(expected, list) and isinstance(actual, list) and len(expected) == len(actual):
        for i, (left, right) in enumerate(zip(expected, actual)):
            _compare(left, right, f"{path}[{i}]", differences)
        return differences
    if type(expected) in (int, float) and type(actual) in (int, float):
        error = abs(expected - actual)
        differences["max_numeric_absolute_error"] = max(differences["max_numeric_absolute_error"], error)
        same = error <= COMPARISON_TOLERANCE
    else:
        same = type(expected) is type(actual) and expected == actual
    if not same:
        differences["count"] += 1
        if len(differences["examples"]) < 12:
            differences["examples"].append(path)
    return differences


def _pose_fields(snapshot):
    if not isinstance(snapshot, dict):
        raise ValueError("snapshot must be an object")
    # Human-facing save paths/messages do not affect fusion and are not replayed.
    required = ("model", "nodes", "calibration", "gap_count", "invalid_count", "timing_basis")
    if any(key not in snapshot for key in required):
        raise ValueError("Incomplete engine snapshot")
    result = {key: snapshot[key] for key in required}
    if not isinstance(snapshot["calibration"], dict):
        raise ValueError("Invalid calibration snapshot")
    result["calibration"] = {k: v for k, v in snapshot["calibration"].items() if k != "message"}
    return result


def diagnose_session(path, *, raw_only=False):
    """Return a bounded prefix report even for interrupted or damaged sessions.

    Invalid/absent headers raise ValueError. A damaged later event is reported as
    incomplete. Version-incompatible checkpoints disable reconstruction, while
    raw observations remain available. Never opens a device or writes files.
    """
    events = iter_session(path)
    try:
        header = next(events)
        if (header["type"] != "header" or header.get("kind") != "vt_glove_pose.diagnostic_session"
                or type(header.get("schema_version")) is not int or header["schema_version"] != 1
                or header.get("time_basis") != "host_observed_not_sensor_latency"):
            raise ValueError("Unsupported diagnostic session header")
        checkpoint = header.get("checkpoint")
        if not isinstance(checkpoint, dict):
            raise ValueError("Missing initial engine checkpoint")
        config = validate_config(checkpoint.get("config"))
        if checkpoint.get("config_fingerprint") != fingerprint(config):
            raise ValueError("Checkpoint configuration fingerprint does not match")
    except StopIteration as exc:
        raise ValueError("Empty diagnostic session") from exc
    except BaseException:
        events.close()
        raise

    engine = None
    checkpoint_schema = checkpoint.get("schema_version")
    algorithm_mode = checkpoint.get("pose_algorithm", "legacy_quaternion_v1" if checkpoint_schema == 1 else "unknown")
    replay_error = "Disabled by --raw-only" if raw_only else None
    if not raw_only:
        try:
            from .engine import PoseEngine
            engine = PoseEngine.from_offline_checkpoint(checkpoint)
        except (ValueError, TypeError, RuntimeError, ImportError, KeyError) as exc:
            replay_error = f"Initial checkpoint cannot be restored: {exc}"
    raw = _RawStatistics(config)
    counts = dict(events=1, batches=0, snapshots=0, commands=0, checkpoints=0, markers=0)
    peaks, markers, marker_omitted = {}, [], 0
    fingers = {name: dict(snapshot_count=0, valid_snapshots=0, clipped_snapshots=0,
                          raw_angle_range_deg={}) for name in FINGERS}
    comparison = dict(compared_snapshots=0, mismatched_snapshots=0,
                      max_numeric_absolute_error=0., examples=[])
    end = None
    format_error = None

    def metrics(value):
        if not isinstance(value, dict) or len(value) > 128:
            raise ValueError("Invalid host performance metrics")
        for key, v in value.items():
            if type(v) in (int, float) and (key in peaks or len(peaks) < 128):
                peaks[key] = max(peaks.get(key, v), v)

    try:
        for event in events:
            if end is not None:
                raise ValueError("Unexpected records after end marker")
            kind = event["type"]
            if kind == "batch":
                rows = event.get("samples")
                if not isinstance(rows, list) or len(rows) > MAX_BATCH_SAMPLES:
                    raise ValueError("Invalid or oversized sample batch")
                samples = []
                for row in rows:
                    if not isinstance(row, dict):
                        raise ValueError("Sample must be an object")
                    samples.append(Sample(**row))
                metrics(event.get("performance", {}))
                for sample in samples:
                    raw.add(sample)
                    if engine is not None:
                        engine.process(sample)
                counts["batches"] += 1
            elif kind == "snapshot":
                now = _finite(event.get("now_s"), "snapshot now_s")
                saved = _pose_fields(event.get("snapshot"))
                model = saved["model"]
                if not isinstance(model, dict) or not isinstance(model.get("fingers"), dict) or set(model["fingers"]) != set(FINGERS):
                    raise ValueError("Snapshot needs five finger records")
                for name, finger in model["fingers"].items():
                    if not isinstance(finger, dict):
                        raise ValueError("Invalid finger snapshot")
                    summary = fingers[name]
                    summary["snapshot_count"] += 1
                    summary["valid_snapshots"] += int(finger.get("valid") is True)
                    clipped = finger.get("clipped_angles", [])
                    if not isinstance(clipped, list):
                        raise ValueError("Invalid clipped_angles")
                    summary["clipped_snapshots"] += int(bool(clipped))
                    angles = finger.get("raw_angles_deg", {})
                    if not isinstance(angles, dict) or len(angles) > 5:
                        raise ValueError("Invalid raw_angles_deg")
                    for key, angle in angles.items():
                        if key not in {"mcp_deg", "pip_deg", "ip_deg", "dip_deg", "splay_deg"}:
                            raise ValueError("Unknown angle name")
                        if angle is not None:
                            angle = _finite(angle, "raw angle")
                            bounds = summary["raw_angle_range_deg"].setdefault(key, [angle, angle])
                            bounds[0], bounds[1] = min(bounds[0], angle), max(bounds[1], angle)
                metrics(event.get("performance", {}))
                if engine is not None:
                    actual = _pose_fields(engine.get_snapshot(now=now))
                    diff = _compare(saved, actual)
                    comparison["compared_snapshots"] += 1
                    comparison["mismatched_snapshots"] += int(diff["count"] > 0)
                    comparison["max_numeric_absolute_error"] = max(comparison["max_numeric_absolute_error"], diff["max_numeric_absolute_error"])
                    if diff["count"] and len(comparison["examples"]) < 5:
                        comparison["examples"].append(dict(seq=event["seq"], paths=diff["examples"]))
                counts["snapshots"] += 1
            elif kind == "command":
                name, args = event.get("name"), event.get("args")
                if not isinstance(args, dict):
                    raise ValueError("Invalid command arguments")
                if name == "neutral":
                    if set(args) != {"duration", "confirm_mount"} or args["duration"] != 5 or type(args["confirm_mount"]) is not bool:
                        raise ValueError("Invalid neutral command")
                    if engine is not None:
                        engine.begin_neutral(**args)
                elif name == "axis":
                    targets = (*FINGERS, "palm", "thumb_common", "thumb_hinge") if algorithm_mode != "legacy_quaternion_v1" else (*FINGERS, "palm")
                    fields = {"duration", "target"}
                    if "calibration_method" in args:
                        if (args["calibration_method"] != "robust_v2" or
                                args.get("target") not in {"thumb_common", "thumb_hinge"} or
                                algorithm_mode == "legacy_quaternion_v1"):
                            raise ValueError("Invalid axis calibration method")
                        fields.add("calibration_method")
                    if set(args) != fields or args["duration"] != 8 or args["target"] not in targets:
                        raise ValueError("Invalid axis command")
                    if engine is not None:
                        engine.begin_axis(**args)
                elif name == "cancel" and not args:
                    if engine is not None:
                        engine.cancel_calibration()
                else:
                    raise ValueError("Unsupported replay command (no external operations allowed)")
                counts["commands"] += 1
            elif kind == "checkpoint":
                state = event.get("checkpoint")
                if not isinstance(state, dict) or state.get("config_fingerprint") != fingerprint(config) or state.get("config") != config:
                    raise ValueError("Session checkpoint changed configuration")
                if (state.get("schema_version") != checkpoint_schema or
                        state.get("pose_algorithm", "legacy_quaternion_v1" if state.get("schema_version") == 1 else "unknown") != algorithm_mode):
                    raise ValueError("Session checkpoint changed pose algorithm/schema")
                if engine is not None:
                    try:
                        engine = PoseEngine.from_offline_checkpoint(state)
                    except (ValueError, TypeError, RuntimeError, KeyError) as exc:
                        engine, replay_error = None, f"Checkpoint at event {event['seq']} cannot be restored: {exc}"
                counts["checkpoints"] += 1
            elif kind == "marker":
                if event.get("label") not in {"rest", "moving", "returned"}:
                    raise ValueError("Unsupported operator marker")
                counts["markers"] += 1
                if len(markers) < 128:
                    markers.append(dict(seq=event["seq"], label=event["label"], after_sample=raw.count))
                else:
                    marker_omitted += 1
            elif kind == "end":
                if type(event.get("complete")) is not bool or not isinstance(event.get("reason"), str):
                    raise ValueError("Invalid session end marker")
                end = event
            else:
                raise ValueError(f"Unsupported event type: {kind}")
            counts["events"] += 1
    except (ValueError, TypeError, KeyError, RuntimeError, OverflowError) as exc:
        format_error = f"Stopped at event {counts['events']}: {exc}"
    finally:
        events.close()

    complete = end is not None and end["complete"] and format_error is None
    reproducible = (complete and replay_error is None and raw.count > 0 and counts["snapshots"] > 0
                    and comparison["compared_snapshots"] == counts["snapshots"]
                    and comparison["mismatched_snapshots"] == 0)
    comparison.update(status="match" if reproducible else "mismatch" if comparison["mismatched_snapshots"] else "unverified",
                      algorithm_mode=algorithm_mode, checkpoint_schema_version=checkpoint_schema,
                      reproducible=reproducible, blocked_reason=replay_error,
                      numeric_absolute_tolerance=COMPARISON_TOLERANCE,
                      scope="model, nodes, calibration state, counters; excludes prose/runtime")
    warnings = [
        "This is software reproduction, not hand-pose accuracy or physical return-latency validation.",
        "Host parse timestamps and backlog exclude unknown sensor/USB upstream latency; equal times in a batch are legitimate.",
        "complete is an application end marker, not a checksum, loss-free transport guarantee or power-loss durability guarantee.",
        "Gyro/accel conversion and nominal sample rate are configuration assumptions unless independently verified.",
        "Manual rest/moving/returned markers are operator annotations, not synchronized ground truth.",
        "Reported pose ranges/clipping describe recorded estimates, not independently measured anatomy.",
    ]
    if not complete:
        warnings.append("Session is incomplete or malformed; only the readable prefix is summarized.")
    if not raw.count or not counts["snapshots"]:
        warnings.append("Raw samples and pose snapshots are both required for a reproduction verdict.")
    node_report = raw.report()
    if any(n.get("near_int16_limit_gyro_records", 0) for n in node_report.values()):
        warnings.append("Near-int16-limit gyro values were observed; this alone cannot distinguish real motion, range saturation or corrupted payloads.")
    if any(f["clipped_snapshots"] for f in fingers.values()):
        warnings.append("Display angle clipping was observed; inspect raw-angle ranges before attributing delayed return to rendering.")
    return dict(schema_version=1, source=str(path), time_basis=header["time_basis"],
                session_complete=complete, end_reason=end["reason"] if end else None,
                format_error=format_error, counts=counts, total_records=raw.count,
                configuration=dict(fingerprint=fingerprint(config), sample_rate_hz=config["sample_rate_hz"],
                                   gyro_dps_per_lsb=config["gyro_dps_per_lsb"], accel_g_per_lsb=config["accel_g_per_lsb"],
                                   **({"gyro_dps_per_lsb_by_node": dict(config["gyro_dps_per_lsb_by_node"])}
                                      if "gyro_dps_per_lsb_by_node" in config else {}),
                                   units_verified=config["units_verified"], sample_rate_verified=config["sample_rate_verified"]),
                nodes=node_report, recorded_pose_statistics=fingers, host_metric_peaks=peaks,
                markers=markers, omitted_marker_details=marker_omitted,
                reconstruction=comparison, warnings=warnings)
