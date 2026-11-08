"""Workbench hand control, independent of Qt, ROS and the numerical runtime.

Device ownership stays with the process manager. Hand commands cannot select a
serial port or start another reader. High-rate geometry is not a manager field.
"""
from __future__ import annotations

import json

from .validation import (HAND_PROTOCOL_VERSION, ProtocolError, mapping, limited_text, boolean,
                         bounded_integer, bounded_number, choice)

HAND_COMMANDS = {"hand.command"}
NO_ARGS = {"capture_functional_hand", "capture_heading_reference", "cancel_capture",
           "clear_calibration", "use_input_scale_candidate", "cancel_shape",
           "optimize_shape", "apply_shape_candidate", "revert_shape",
           "retry_calibration_save"}


def validate_hand_command(command, params):
    choice(command, HAND_COMMANDS, "hand command")
    mapping(params, {"hand", "expected_manager_instance_id", "expected_service_instance_id",
                     "expected_session_id", "expected_profile_version", "name", "args"}, "hand params")
    choice(params["hand"], {"left", "right"}, "hand")
    for key in ("expected_manager_instance_id", "expected_service_instance_id"):
        limited_text(params[key], key, nonempty=True)
    if params["expected_session_id"] is not None:
        limited_text(params["expected_session_id"], "expected_session_id", maximum=256)
    if params["expected_profile_version"] is not None:
        bounded_integer(params["expected_profile_version"], "expected_profile_version", maximum=2**32-1)
    name, args = params["name"], params["args"]
    limited_text(name, "hand command name", maximum=64, nonempty=True)
    specifications = {**{n: set() for n in NO_ARGS}, "capture": {"step"},
        "capture_functional": {"finger"}, "set_functional_enabled": {"enabled"},
        "capture_shape_contact": {"category", "contact_confirmed"},
        "preview_shape": {"mode"}, "load_shape_profile": {"path"},
        "save_shape_profile": {"path"}, "load_shape_task": {"path"},
        "save_shape_task": {"path"}, "retry_calibration_save_to": {"directory"}}
    if name not in specifications:
        raise ProtocolError("unsupported hand operation; device connection belongs to the manager")
    mapping(args, specifications[name], "hand operation arguments")
    if name == "capture":
        choice(args["step"], {"neutral", "fingers_down", "thumb_up"}, "capture step")
    if name == "capture_functional":
        choice(args["finger"], {"thumb", "index", "middle", "ring", "little"}, "finger")
    if "enabled" in args:
        boolean(args["enabled"], "enabled")
    if "category" in args:
        bounded_integer(args["category"], "category", minimum=2, maximum=5)
        if args["contact_confirmed"] is not True:
            raise ProtocolError("shape contact requires an explicit contact label")
    if "mode" in args:
        choice(args["mode"], {"current", "candidate", "baseline"}, "preview mode")
    for key in ("path", "directory"):
        if key in args:
            limited_text(args[key], key, maximum=4096, nonempty=True)
    if len(json.dumps(params, ensure_ascii=False).encode()) > 16384:
        raise ProtocolError("hand command exceeds 16 KiB")


def validate_hand_sessions(value):
    if type(value) is not dict or set(value) - {"left", "right"}:
        raise ProtocolError("hand_sessions must contain only left and right")
    fields = {"hand", "group_id", "control_socket", "tactile_socket", "resources", "port", "glove_python", "tactile_port",
              "service_instance_id", "service_state", "connected", "raw_fresh",
              "calibration_ready", "pose_valid", "session_id", "profile_version",
              "busy", "age_s", "detail"}
    for hand, row in value.items():
        mapping(row, fields, "hand session")
        if row["hand"] != hand:
            raise ProtocolError("hand session identity mismatch")
        for key in ("group_id", "service_instance_id", "session_id"):
            limited_text(row[key], key, maximum=256)
        for key in ("control_socket", "tactile_socket", "resources", "port", "glove_python", "tactile_port", "detail"):
            limited_text(row[key], key, maximum=4096)
        choice(row["service_state"], {"unavailable", "starting", "ready", "failed", "stopping"}, "service_state")
        for key in ("connected", "raw_fresh", "calibration_ready", "pose_valid", "busy"):
            boolean(row[key], key)
        bounded_integer(row["profile_version"], "profile_version", maximum=2**32-1)
        if row["age_s"] is not None:
            bounded_number(row["age_s"], "age_s")
