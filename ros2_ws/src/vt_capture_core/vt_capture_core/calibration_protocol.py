"""Bounded calibration IPC contract; no ROS, numerical or GUI dependencies."""
from pathlib import PurePosixPath
import json
import re

from .validation import (ProtocolError, boolean, bounded_integer, bounded_number,
                         choice, limited_text, mapping, json_value)

CALIBRATION_COMMANDS = frozenset({"calibration.start", "calibration.cancel", "calibration.retry_publish", "calibration.restore"})
CALIBRATION_STATES = frozenset({
    "PREPARING", "COLLECTING", "STOPPING", "SOLVING", "VERIFYING", "COMPARING", "READY",
    "CANDIDATE", "APPLYING", "COMPLETE", "APPLIED_PENDING_PUBLICATION", "PENDING_IDLE",
    "FAILED", "CANCELLED", "INTERRUPTED", "WAITING_TO_APPLY", "CANCELLING",
})


def empty_calibration():
    return dict(enabled=False, busy=False, session=None, active=[], cameras=[], trackers=[])


def _path(value, context, *, optional=False):
    limited_text(value, context, maximum=4096, nonempty=not optional)
    if value and (not PurePosixPath(value).is_absolute() or ".." in PurePosixPath(value).parts):
        raise ProtocolError(f"{context} must be an absolute path without parent traversal")


def validate_calibration_command(command, params):
    choice(command, CALIBRATION_COMMANDS, "command")
    common = {"expected_manager_instance_id"}
    if command == "calibration.restore":
        mapping(params, common | {"camera_name", "revision", "expected_active_revision"}, "calibration params")
        limited_text(params["camera_name"], "camera_name", maximum=128, nonempty=True)
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", params["camera_name"]):
            raise ProtocolError("camera_name must be a ROS name token")
        for field in ("revision", "expected_active_revision"):
            if type(params[field]) is not str or not re.fullmatch(r"[0-9a-f]{32}", params[field]):
                raise ProtocolError(f"{field} must be a calibration revision")
    elif command != "calibration.start":
        mapping(params, common | {"session_id"}, "calibration params")
        limited_text(params["session_id"], "session_id", nonempty=True)
    else:
        mapping(params, common | {"mode", "camera_name", "tracker_role", "config_path", "board", "bag_paths"}, "calibration params")
        choice(params["mode"], {"guided", "offline"}, "mode")
        limited_text(params["camera_name"], "camera_name", maximum=128, nonempty=True)
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", params["camera_name"]):
            raise ProtocolError("camera_name must be a ROS name token")
        choice(params["tracker_role"], {"torso", "left_wrist", "right_wrist"}, "tracker_role")
        _path(params["config_path"], "config_path", optional=True)
        board = params["board"]
        mapping(board, {"square_length_mm", "marker_length_mm", "squares_x", "squares_y", "dictionary"}, "board")
        for name in ("squares_x", "squares_y"):
            bounded_integer(board[name], name, minimum=3, maximum=100)
        for name in ("square_length_mm", "marker_length_mm"):
            bounded_number(board[name], name, minimum=.001, maximum=1000.)
        if board["marker_length_mm"] >= board["square_length_mm"]:
            raise ProtocolError("marker length must be smaller than square length")
        limited_text(board["dictionary"], "dictionary", maximum=64, nonempty=True)
        paths = params["bag_paths"]
        if type(paths) is not list or len(paths) not in ({0} if params["mode"] == "guided" else {1, 3}):
            raise ProtocolError("guided requires no bags; offline requires one or three independent bags")
        for path in paths:
            _path(path, "bag path")
        if len(set(paths)) != len(paths):
            raise ProtocolError("bag paths must be distinct")
    limited_text(params["expected_manager_instance_id"], "expected_manager_instance_id", nonempty=True)


def validate_calibration_snapshot(value):
    mapping(value, {"enabled", "busy", "session", "active"}, "calibration",
            optional={"cameras", "trackers", "detail"})
    boolean(value["enabled"], "calibration.enabled")
    boolean(value["busy"], "calibration.busy")
    for key in ("active", "cameras", "trackers"):
        items = value.get(key, [])
        if type(items) is not list or len(items) > 64 or any(type(item) is not dict for item in items):
            raise ProtocolError(f"calibration.{key} must be a bounded array of objects")
    for camera in value.get("cameras", []):
        mapping(camera, {"name", "model", "serial"}, "calibration.camera")
        for key in ("name", "model", "serial"):
            limited_text(camera[key], f"calibration.camera.{key}", nonempty=True)
    for tracker in value.get("trackers", []):
        mapping(tracker, {"role"}, "calibration.tracker")
        choice(tracker["role"], {"torso", "left_wrist", "right_wrist"}, "calibration.tracker.role")
    session = value["session"]
    if session is not None:
        mapping(session, {"session_id", "state"}, "calibration.session", optional={
            "schema_version", "camera_name", "tracker_role", "mode", "round", "rounds", "accepted",
            "target", "hint_code", "detail", "metrics", "preview", "result", "updated_ns", "created_ns",
        })
        limited_text(session["session_id"], "calibration.session_id", nonempty=True)
        choice(session["state"], CALIBRATION_STATES, "calibration.state")
        for name in ("round", "rounds", "accepted", "target", "updated_ns", "created_ns"):
            if name in session:
                bounded_integer(session[name], f"calibration.{name}", maximum=(1 << 64)-1)
        for name in ("detail", "hint_code", "camera_name", "tracker_role", "mode"):
            if name in session:
                limited_text(session[name], f"calibration.{name}", maximum=4096)
        if session.get("metrics") is not None and type(session["metrics"]) is not dict:
            raise ProtocolError("calibration metrics must be an object")
        if session.get("result") is not None and type(session["result"]) is not dict:
            raise ProtocolError("calibration result must be an object or null")
        if session.get("preview") is not None:
            preview = session["preview"]
            mapping(preview, {"jpeg_base64", "source_stamp_ns"}, "calibration.preview", optional={"width", "height"})
            limited_text(preview["jpeg_base64"], "preview.jpeg_base64", maximum=512*1024)
            bounded_integer(preview["source_stamp_ns"], "preview.source_stamp_ns", maximum=(1 << 64)-1)
    json_value(value)
    if len(json.dumps(value, ensure_ascii=False, allow_nan=False).encode()) > 1024*1024:
        raise ProtocolError("calibration snapshot exceeds 1 MiB")
