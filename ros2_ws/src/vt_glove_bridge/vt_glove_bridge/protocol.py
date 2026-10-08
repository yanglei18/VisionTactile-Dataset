"""Validate the bounded JSON ABI before assigning generated ROS message fields."""
from __future__ import annotations

from collections import OrderedDict
import json
import math

from .transport import MAX_PACKET_BYTES


STRINGS = (
    "hand", "frame_id", "source_mode", "session_id", "config_sha256",
    "model_rest_sha256", "algorithm", "quality", "reason", "clock_domain",
    "timing_source", "quality_json", "metadata_json",
)
BOOLS = (
    "pose_valid", "time_valid", "units_verified", "sample_rate_verified",
    "anatomical_accuracy_verified", "yaw_drift_corrected",
)
UINTS = {
    "schema_version": 16, "frame_index": 64, "calibration_version": 32,
    "host_realtime_ns": 64, "host_monotonic_ns": 64, "reference_node_id": 8,
    "solve_completed_monotonic_ns": 64, "producer_dropped_total": 64,
}
SOURCE_ARRAYS = {
    "source_node_ids": 8, "source_realtime_ns": 64, "source_monotonic_ns": 64,
    "source_sample_indices": 64, "source_filter_epochs": 32,
}
POSE_ARRAYS = {"mano_pose_rotvec": 48, "root_quat_xyzw": 4, "shape_params": 10,
               "joint_positions_m": 63}
MASKS = ("segment_observed_mask", "joint_observed_mask", "inferred_mask")
MESSAGE_FIELDS = (*STRINGS, *BOOLS, *UINTS, *SOURCE_ARRAYS, *POSE_ARRAYS, *MASKS,
                  "source_labels")


def _uint(value, bits, name):
    if type(value) is not int or not 0 <= value < (1 << bits):
        raise ValueError(f"{name} must be uint{bits}")


def _reject_constant(value):
    raise ValueError(f"non-finite JSON number: {value}")


def _unique_pairs(pairs):
    output = {}
    for key, value in pairs:
        if key in output:
            raise ValueError(f"duplicate JSON key: {key}")
        output[key] = value
    return output


def _validate_joint_quality(quality, *, pose_valid):
    """Validate optional per-joint extensions without changing the ROS ABI."""
    keys = ("joint_valid_mask", "joint_display_available_mask", "joint_quality")
    supplied = [key in quality for key in keys]
    if any(supplied):
        if not all(supplied):
            raise ValueError("joint quality requires both masks and joint_quality")
        for key in keys[:2]:
            values = quality[key]
            if (not isinstance(values, list) or len(values) != 16
                    or any(type(item) is not bool for item in values)):
                raise ValueError(f"{key} must contain 16 bool values")
        values = quality["joint_quality"]
        if (not isinstance(values, list) or len(values) != 16
                or any(item not in ("valid", "degraded", "unavailable") for item in values)):
            raise ValueError("joint_quality must contain 16 valid/degraded/unavailable values")
        expected = {"valid": (True, True), "degraded": (False, True),
                    "unavailable": (False, False)}
        for index, value in enumerate(values):
            actual = (quality["joint_valid_mask"][index],
                      quality["joint_display_available_mask"][index])
            if actual != expected[value]:
                raise ValueError(f"joint quality masks disagree at slot {index}")
        if pose_valid and any(value != "valid" for value in values):
            raise ValueError("valid pose requires all joint_quality entries to be valid")
    if "joint_estimation" in quality and not isinstance(quality["joint_estimation"], dict):
        raise ValueError("joint_estimation must be an object")


def decode_state(payload: bytes, *, allow_replay=False, expected_hand=None,
                 reference_node_id=None) -> dict:
    if expected_hand is not None and expected_hand not in ("left", "right"):
        raise ValueError("expected_hand must be left or right")
    if reference_node_id is not None and (type(reference_node_id) is not int
                                          or not 1 <= reference_node_id <= 12):
        raise ValueError("reference_node_id must be in 1..12")
    if not payload or len(payload) > MAX_PACKET_BYTES:
        raise ValueError("empty or oversized glove packet")
    try:
        state = json.loads(payload, parse_constant=_reject_constant,
                           object_pairs_hook=_unique_pairs)
    except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("malformed glove JSON") from exc
    if not isinstance(state, dict) or state.get("kind") != "glove_state":
        raise ValueError("expected a glove_state object")
    for key in MESSAGE_FIELDS:
        if key not in state:
            raise ValueError(f"missing field: {key}")
    for key in STRINGS:
        if not isinstance(state[key], str):
            raise ValueError(f"{key} must be a string")
    for key in BOOLS:
        if type(state[key]) is not bool:
            raise ValueError(f"{key} must be bool")
    for key, bits in UINTS.items():
        _uint(state[key], bits, key)
    if state["schema_version"] != 1 or state["hand"] not in ("left", "right"):
        raise ValueError("only schema version 1 and left/right hands are supported")
    if expected_hand is not None and state["hand"] != expected_hand:
        raise ValueError("glove packet hand does not match this bridge instance")
    if state["frame_id"] != f"glove_{state['hand']}_reference":
        raise ValueError("unexpected coordinate frame; world pose is not supported")
    if state["clock_domain"] != "CLOCK_MONOTONIC" or state["timing_source"] != "host_receive":
        raise ValueError("unsupported source clock domain")
    if not state["session_id"] or not state["algorithm"]:
        raise ValueError("session_id and algorithm are required")
    if state["source_mode"] not in ("live", "replay"):
        raise ValueError("unknown source_mode")
    if state["source_mode"] == "replay" and not allow_replay:
        raise ValueError("replay input disabled; explicitly enable allow_replay for testing")
    if state["anatomical_accuracy_verified"] or state["yaw_drift_corrected"]:
        raise ValueError("schema v1 does not claim anatomical validation or corrected yaw drift")
    for key in ("quality_json", "metadata_json"):
        try:
            nested = json.loads(state[key], parse_constant=_reject_constant,
                                object_pairs_hook=_unique_pairs)
        except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
            raise ValueError(f"{key} is not valid JSON") from exc
        if not isinstance(nested, dict):
            raise ValueError(f"{key} must encode an object")
        if key == "quality_json":
            _validate_joint_quality(nested, pose_valid=state["pose_valid"])
    size = None
    for key, bits in SOURCE_ARRAYS.items():
        values = state[key]
        if not isinstance(values, list) or len(values) > 12:
            raise ValueError(f"{key} must contain at most 12 entries")
        if size is None:
            size = len(values)
        if len(values) != size:
            raise ValueError("source arrays have different lengths")
        for value in values:
            _uint(value, bits, key)
    ids = state["source_node_ids"]
    if len(set(ids)) != len(ids) or any(not 1 <= item <= 12 for item in ids):
        raise ValueError("source node IDs must be unique and in 1..12")
    palm = state["reference_node_id"]
    if not 1 <= palm <= 12:
        raise ValueError("palm reference node must be in 1..12")
    if reference_node_id is not None and palm != reference_node_id:
        raise ValueError("palm reference node does not match configured mapping")
    for key, count in POSE_ARRAYS.items():
        values = state[key]
        allowed = (0, count) if key == "joint_positions_m" or not state["pose_valid"] else (count,)
        if not isinstance(values, list) or len(values) not in allowed:
            raise ValueError(f"unexpected {key} length")
        if any(type(item) not in (int, float) or not math.isfinite(item) for item in values):
            raise ValueError(f"{key} must contain finite numbers")
        state[key] = [float(item) for item in values]
    quat = state["root_quat_xyzw"]
    if quat and abs(sum(item * item for item in quat) - 1.0) > 0.002:
        raise ValueError("root quaternion must be unit length")
    for key in MASKS:
        values = state[key]
        if not isinstance(values, list) or len(values) != 16 or any(type(item) is not bool for item in values):
            raise ValueError(f"{key} must contain 16 bool values")
    if (not isinstance(state["source_labels"], list) or len(state["source_labels"]) != 16
            or any(not isinstance(item, str) for item in state["source_labels"])):
        raise ValueError("source_labels must contain 16 strings")
    if state["pose_valid"] and not state["time_valid"]:
        raise ValueError("valid pose requires valid input time")
    if state["time_valid"]:
        if (not state["host_realtime_ns"] or not state["host_monotonic_ns"]
                or palm not in ids):
            raise ValueError("valid time requires the palm source")
        reference = ids.index(palm)
        if (state["host_realtime_ns"] != state["source_realtime_ns"][reference]
                or state["host_monotonic_ns"] != state["source_monotonic_ns"][reference]):
            raise ValueError("reference stamp does not equal the palm input stamp")
    if state["host_realtime_ns"] // 1_000_000_000 >= (1 << 31):
        raise ValueError("input time exceeds ROS builtin Time sec range")
    return state


class SequenceTracker:
    """Reject duplicate/out-of-order results without accumulating unbounded session state."""

    def __init__(self):
        self.sessions = OrderedDict()
        self.gaps_total = 0

    def accept(self, state):
        session, index = (state["hand"], state["session_id"]), state["frame_index"]
        previous = self.sessions.get(session)
        if previous is not None and index <= previous:
            raise ValueError("duplicate or out-of-order solved frame")
        if previous is not None:
            self.gaps_total += index - previous - 1
        self.sessions[session] = index
        self.sessions.move_to_end(session)
        if len(self.sessions) > 64:
            self.sessions.popitem(last=False)


def populate_message(message, state, *, realtime_ns, monotonic_ns, gaps_total=0,
                     rejected_total=0, max_input_age_ns=150_000_000):
    """Keep all source timestamps. Stale/future input is invalid, never restamped."""
    for field in MESSAGE_FIELDS:
        if field != "frame_id":
            setattr(message, field, state[field])
    message.header.frame_id = state["frame_id"]
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(state["host_realtime_ns"], 1_000_000_000)
    message.bridge_received_realtime_ns = realtime_ns
    message.bridge_received_monotonic_ns = monotonic_ns
    message.bridge_sequence_gaps_total = gaps_total
    message.bridge_rejected_total = rejected_total
    if state["time_valid"]:
        ages = [monotonic_ns - value for value in state["source_monotonic_ns"]]
        if any(age < 0 for age in ages):
            message.pose_valid = False
            message.time_valid = False
            message.reason = "bridge_input_clock_mismatch"
        elif any(age > max_input_age_ns for age in ages):
            message.pose_valid = False
            message.reason = "bridge_input_stale"
    return message
