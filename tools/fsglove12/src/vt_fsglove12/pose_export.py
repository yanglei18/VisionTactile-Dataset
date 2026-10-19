"""Optional, results-only local ROS bridge transport (no ROS dependencies).

Called by the computation owner, never by the GUI timer or serial reader. One
nonblocking datagram carries one completed result. Missing/slow subscribers may
lose results, counted explicitly; they cannot reset acquisition or calibration.
No raw IMU values, mesh vertices, pickle, or device commands cross this boundary.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import socket

from .joint_quality import has_joint_quality, joint_quality_state
from .health_contract import blocked_input_nodes, result_input_health


MAX_DATAGRAM_BYTES = 60_000


def _json(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def validate_socket_path(path):
    path = os.fspath(path)
    if not Path(path).is_absolute() or "\0" in path or len(os.fsencode(path)) > 100:
        raise ValueError("ROS socket must be an absolute local path of at most 100 bytes")
    return path


def _flat(value, rows, columns):
    if not isinstance(value, (list, tuple)) or len(value) != rows:
        return []
    result = []
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) != columns:
            return []
        for v in row:
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return []
            result.append(float(v))
    return result


def _vector(value, size):
    return _flat([value], 1, size)


def _positive_ns(value):
    return type(value) is int and 0 < value < 2**64


def _results_only_sensor_quality(quality):
    """Keep event provenance/counts, never local raw-value evidence."""
    exported = deepcopy(quality)
    report = exported.get("gyro_near_rail") if isinstance(exported, dict) else None
    nodes = report.get("nodes") if isinstance(report, dict) else None
    if isinstance(nodes, dict):
        for node in nodes.values():
            event = node.get("last_boundary_event") if isinstance(node, dict) else None
            if isinstance(event, dict):
                event.pop("gyro_lsb", None)
    return exported


def build_state(frame, pose, geometry, config, *, root_node, solve_completed_monotonic_ns):
    """Map the numerical measurement result, never a held display pose.

    Header reference is the palm sample used for this result, not the tick,
    render, IPC or ROS publish time. Preserve other input ages explicitly.
    Missing integer source times make a result time-invalid, never substituted
    by wall-clock now or a historical CSV's rounded float timestamp.
    """
    if (pose.get("display_only") or (geometry or {}).get("display_only")
            or (geometry or {}).get("provenance", {}).get("display_only")):
        raise ValueError("display-only geometry or pose cannot be exported as a measurement")
    hand = config["hand"]
    if hand not in ("right", "left"):
        raise ValueError("hand must be right or left")
    configured_root = next(node for node, slot in config["node_to_slot"].items() if slot == 0)
    if str(root_node) != configured_root:
        raise ValueError("reference node does not match the configured palm")
    for value in (pose, (geometry or {}).get("provenance", {})):
        if value.get("hand", hand) != hand:
            raise ValueError("pose/model hand does not match the configured hand")
    nodes = frame.get("nodes", {})
    root = nodes.get(str(root_node), {})
    time_valid = (_positive_ns(root.get("host_realtime_ns"))
                  and _positive_ns(root.get("host_monotonic_ns")))
    source_nodes = [(int(key), node) for key, node in nodes.items()
                    if key in config["node_to_slot"] and _positive_ns(node.get("host_realtime_ns"))
                    and _positive_ns(node.get("host_monotonic_ns"))]
    source_nodes.sort(key=lambda item: item[0])
    mano = _flat(pose.get("mano_pose_rotvec"), 16, 3)
    root_quat = _vector(pose.get("root_quat_xyzw"), 4)
    shape = _vector(pose.get("shape_params"), 10)
    representation_available = bool(time_valid and pose.get("pose_available") and pose.get("solver_success")
                                    and mano and root_quat and shape)
    joint_valid, joint_available, joint_quality = joint_quality_state(pose)
    validity_present = has_joint_quality(pose)
    unhealthy = blocked_input_nodes(frame, config["node_to_slot"])
    valid = representation_available and (not validity_present or all(joint_valid)) and not unhealthy
    reason = str(pose.get("reason", ""))
    if pose.get("pose_available") and not representation_available:
        reason = "source_time_unavailable" if not time_valid else "pose_representation_unavailable"
    elif representation_available and not valid:
        reason = ("joint_estimation_unavailable" if "unavailable" in joint_quality
                  else "joint_estimation_degraded")
    if unhealthy:
        reason = "input_health_invalid"
    joints = _flat((geometry or {}).get("joints_m"), 21, 3) if representation_available else []
    quality_fields = {}
    if validity_present:
        quality_fields.update(joint_valid_mask=joint_valid, joint_display_available_mask=joint_available,
                              joint_quality=joint_quality, joint_estimation=pose.get("joint_estimation", {}))
    # Functional coverage is distinct from base-result validity. Missing or
    # disabled functional references do not erase an otherwise usable base
    # pose; downstream consumers can explicitly require correction coverage.
    for key in ("joint_calibration_status", "functional_calibration_coverage", "joint_output_provenance"):
        if key in pose:
            quality_fields[key] = deepcopy(pose[key])
    # These are output provenance, not sensor samples. Preserve unverified units.
    metadata = dict(
        node_to_slot=config["node_to_slot"],
        gyro_dps_per_lsb=config["gyro_dps_per_lsb"],
        gyro_dps_per_lsb_by_node=config.get("gyro_dps_per_lsb_by_node", {}),
        accel_g_per_lsb=config["accel_g_per_lsb"], sample_rate_hz=config["sample_rate_hz"],
        accel_g_per_lsb_by_node=config.get("accel_g_per_lsb_by_node", {}),
        input_contract=config.get("input_contract", {}),
        input_filter=config.get("input_filter", {"enabled": False}),
        input_health=config.get("input_health", {"enabled": False}),
        relative_heading_correction=config.get("relative_heading_correction", {"enabled": False}),
        upstream_commit=pose.get("upstream_commit"),
        upstream_state_sha256=pose.get("upstream_state_sha256"),
        missing_slot_policy=pose.get("missing_slot_policy"),
        pose_postprocessing=pose.get("pose_postprocessing"),
        adapter_execution=pose.get("adapter_execution"),
        source_fresh=False if unhealthy else pose.get("source_fresh"),
        sparse_prior=pose.get("sparse_prior"),
        joint_order="wrist,thumb[1:4],index[1:4],middle[1:4],ring[1:4],little[1:4]",
        coordinate_convention="MANO calibration-reference axes; wrist origin; metres; root rotation already applied",
        world_position_measured=False,
    )
    if validity_present:
        metadata["joint_validity_contract"] = {
            "version": 1, "order": "MANO16", "source": "quality_json",
            "pose_valid_requires_all_joints_valid": True,
            "partial_arrays": "current provisional/placeholder geometry, NOT valid measurement; consult per-joint masks",
            "observed_masks": "sensor provenance only, not estimator validity",
            "display_hysteresis_exported": False,
        }
    result = dict(
        schema_version=1, kind="glove_state", hand=hand, frame_id=f"glove_{hand}_reference",
        source_mode="live" if pose.get("input_kind") == "raw_serial" else "replay",
        session_id=str(pose.get("session_id", "")), frame_index=int(pose.get("frame_index", 0)),
        calibration_version=int(pose.get("profile_version") or 0),
        config_sha256=str(pose.get("config_sha256", "")),
        model_rest_sha256=str(pose.get("model_rest_sha256", "")),
        algorithm=str(pose.get("algorithm", "official_fsglove_sparse_adapter")),
        quality=str(pose.get("quality", "unavailable")), reason=reason,
        pose_valid=valid, time_valid=bool(time_valid),
        host_realtime_ns=root["host_realtime_ns"] if time_valid else 0,
        host_monotonic_ns=root["host_monotonic_ns"] if time_valid else 0,
        reference_node_id=int(root_node),
        solve_completed_monotonic_ns=int(solve_completed_monotonic_ns),
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        source_node_ids=[key for key, _ in source_nodes],
        source_realtime_ns=[node["host_realtime_ns"] for _, node in source_nodes],
        source_monotonic_ns=[node["host_monotonic_ns"] for _, node in source_nodes],
        source_sample_indices=[int(node["sample_index"]) for _, node in source_nodes],
        source_filter_epochs=[int(node["epoch"]) for _, node in source_nodes],
        mano_pose_rotvec=mano if representation_available else [], root_quat_xyzw=root_quat if representation_available else [],
        shape_params=shape if representation_available else [], joint_positions_m=joints,
        units_verified=bool(config["units_verified"]),
        sample_rate_verified=bool(config["sample_rate_verified"]),
        anatomical_accuracy_verified=False, yaw_drift_corrected=False,
        quality_json=_json(dict(sensor_quality=_results_only_sensor_quality(pose.get("sensor_quality", {})),
                               input_health=result_input_health(frame),
                               **quality_fields,
                               input_warnings=pose.get("input_warnings", []),
                               relative_heading_correction=pose.get("relative_heading_correction"),
                               **({"little_relative_heading": pose["little_relative_heading"]}
                                  if "little_relative_heading" in pose else {}),
                               **({"pose_processing_revision": pose["pose_processing_revision"]}
                                  if "pose_processing_revision" in pose else {}),
                               **({"input_filter": pose["input_filter"]} if "input_filter" in pose else {}))),
        metadata_json=_json(metadata),
    )
    for key in ("segment_observed_mask", "joint_observed_mask", "inferred_mask"):
        values = pose.get(key, [False] * 16)
        result[key] = list(values) if len(values) == 16 and representation_available else [False] * 16
    result["source_labels"] = list(pose.get("source_labels", ["unavailable"] * 16))
    return result


class PoseDatagramExporter:
    """Best-effort sender with bounded work, reconnect-by-next-result semantics."""

    def __init__(self, path):
        self.path = validate_socket_path(path)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self.sent = self.dropped = 0
        self.last_error = None

    def publish(self, document):
        try:
            data = _json({**document, "producer_dropped_total": self.dropped}).encode("utf-8")
            if len(data) > MAX_DATAGRAM_BYTES:
                raise ValueError("pose datagram exceeds 60000-byte limit")
            self._socket.sendto(data, self.path)
        except (OSError, ValueError, TypeError) as exc:
            self.dropped += 1
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False
        self.sent += 1
        self.last_error = None
        return True

    def snapshot(self):
        return dict(enabled=True, socket_path=self.path, sent=self.sent,
                    dropped=self.dropped, last_error=self.last_error)

    def record_error(self, exc):
        self.dropped += 1
        self.last_error = f"{type(exc).__name__}: {exc}"

    def close(self):
        self._socket.close()
