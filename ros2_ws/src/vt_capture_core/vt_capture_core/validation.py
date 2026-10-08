"""Strict, versioned JSON schemas without ROS or an external schema runtime."""

from __future__ import annotations

import math
import json
from datetime import datetime


PROTOCOL_VERSION = 1
CONTROL_PROTOCOL_VERSION = 2
DETAIL_PROTOCOL_VERSION = 3
DEVICE_PROTOCOL_VERSION = 4
MAX_MESSAGE_BYTES = 2 * 1024 * 1024  # Includes the terminating LF on the wire.
MAX_JSON_DEPTH = 32
MAX_RECORDS = 200
MAX_PROCESS_GROUPS = 32
MAX_DETAIL_BYTES = 512 * 1024
MAX_DETAIL_DEPTH = 12
MAX_DETAIL_STRING_BYTES = 8192
UINT64_MAX = (1 << 64) - 1
UINT32_MAX = (1 << 32) - 1
TRIAL_STATES = {"STARTING", "RECORDING", "STOPPING", "COMPLETE", "FAILED", "UNKNOWN"}
PROCESS_STATES = {"DISABLED", "STOPPED", "STARTING", "RUNNING", "DEGRADED", "STOPPING", "FAILED", "EXTERNAL", "UNKNOWN"}


class ProtocolError(ValueError):
    """Malformed, unsupported or oversized workbench protocol data."""


def mapping(value, required, context, optional=()):
    if type(value) is not dict:
        raise ProtocolError(f"{context} must be an object")
    keys = set(value)
    if not set(required) <= keys or keys - set(required) - set(optional):
        raise ProtocolError(f"{context} fields do not match the schema")
    return value


def text(value, context, *, nonempty=False):
    if type(value) is not str:
        raise ProtocolError(f"{context} must be a string")
    if nonempty and (not value or not value.strip()):
        raise ProtocolError(f"{context} must not be empty")
    try:
        value.encode("utf-8", errors="strict")
    except UnicodeError as error:
        raise ProtocolError(f"{context} must be valid UTF-8 text") from error
    return value


def integer(value, context, *, minimum=0):
    if type(value) is not int or value < minimum:
        raise ProtocolError(f"{context} must be an integer >= {minimum}")
    return value


def number(value, context, *, minimum=0):
    if type(value) not in (int, float):
        raise ProtocolError(f"{context} must be a finite number")
    try:
        valid = math.isfinite(value)
    except (ValueError, OverflowError):
        valid = False
    if not valid or value < minimum:
        raise ProtocolError(f"{context} must be a finite number >= {minimum}")
    return value


def boolean(value, context):
    if type(value) is not bool:
        raise ProtocolError(f"{context} must be bool")
    return value


def choice(value, choices, context):
    text(value, context)
    if value not in choices:
        raise ProtocolError(f"{context} must be one of {', '.join(sorted(choices))}")
    return value


def json_value(value, context="JSON", *, depth=0):
    """Reject non-JSON Python objects and non-finite values, including previews."""
    if depth > MAX_JSON_DEPTH:
        raise ProtocolError(f"{context} exceeds maximum JSON nesting depth")
    if value is None or type(value) in (bool, int):
        return
    if type(value) is float:
        if not math.isfinite(value):
            raise ProtocolError(f"{context} contains a non-finite number")
        return
    if type(value) is str:
        text(value, context)
        return
    if type(value) is list:
        for item in value:
            json_value(item, context, depth=depth + 1)
        return
    if type(value) is dict:
        for key, item in value.items():
            text(key, f"{context} key")
            json_value(item, context, depth=depth + 1)
        return
    raise ProtocolError(f"{context} contains a non-JSON value")


def validate_recording(value):
    mapping(value, {"state", "session_id", "elapsed_s", "duration_s", "detail"}, "recording")
    choice(value["state"], {"UNAVAILABLE", "IDLE", "RECORDING", "COMPLETE"}, "recording.state")
    text(value["session_id"], "recording.session_id")
    number(value["elapsed_s"], "recording.elapsed_s")
    number(value["duration_s"], "recording.duration_s")
    text(value["detail"], "recording.detail")


def validate_device(value):
    mapping(value, {"id", "label", "kind", "side", "status", "topic", "receive_hz",
                    "valid_hz", "age_s", "detail", "preview"}, "device")
    for key in ("id", "label", "side", "topic", "detail"):
        text(value[key], f"device.{key}", nonempty=key in {"id", "label"})
    choice(value["kind"], {"camera", "tracker", "glove", "tactile"}, "device.kind")
    choice(value["status"], {"unknown", "online", "stale", "offline"}, "device.status")
    number(value["receive_hz"], "device.receive_hz")
    for key in ("valid_hz", "age_s"):
        if value[key] is not None:
            number(value[key], f"device.{key}")
    if type(value["preview"]) is not dict:
        raise ProtocolError("device.preview must be an object")
    json_value(value["preview"], "device.preview")


def validate_event(value):
    mapping(value, {"id", "level", "message"}, "event")
    text(value["id"], "event.id", nonempty=True)
    choice(value["level"], {"info", "warning", "error"}, "event.level")
    text(value["message"], "event.message")


def validate_resources(value):
    mapping(value, {"disk_free_bytes"}, "resources")
    if value["disk_free_bytes"] is not None:
        integer(value["disk_free_bytes"], "resources.disk_free_bytes")


def validate_capabilities(value):
    mapping(value, {"record", "device_control"}, "capabilities")
    boolean(value["record"], "capabilities.record")
    boolean(value["device_control"], "capabilities.device_control")


def validate_snapshot(value, *, version=PROTOCOL_VERSION):
    required = {"mode", "instance_id", "sequence", "recording", "devices", "events", "resources", "capabilities"}
    if version in (CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION):
        required.add("management")
        if version == DEVICE_PROTOCOL_VERSION:
            required.add("process_control")
    elif version != PROTOCOL_VERSION:
        raise ProtocolError("unsupported snapshot version")
    mapping(value, required, "snapshot")
    choice(value["mode"], {"live", "demo"}, "snapshot.mode")
    text(value["instance_id"], "snapshot.instance_id", nonempty=True)
    integer(value["sequence"], "snapshot.sequence")
    validate_recording(value["recording"])
    for field, validator in (("devices", validate_device), ("events", validate_event)):
        if type(value[field]) is not list:
            raise ProtocolError(f"snapshot.{field} must be an array")
        ids = set()
        for item in value[field]:
            validator(item)
            if item["id"] in ids:
                raise ProtocolError(f"snapshot.{field} contains duplicate id: {item['id']}")
            ids.add(item["id"])
    validate_resources(value["resources"])
    validate_capabilities(value["capabilities"])
    if version in (CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION):
        limited_text(value["instance_id"], "snapshot.instance_id", nonempty=True)
        bounded_integer(value["sequence"], "snapshot.sequence", maximum=UINT64_MAX)
        for field in ("devices", "events"):
            bounded_array(value[field], f"snapshot.{field}")
        limited_text(value["recording"]["session_id"], "recording.session_id")
        limited_text(value["recording"]["detail"], "recording.detail", maximum=4096)
        for field in ("elapsed_s", "duration_s"):
            bounded_number(value["recording"][field], f"recording.{field}")
        for device in value["devices"]:
            for field in ("id", "label", "side", "topic", "detail"):
                maximum = {"id": 256, "label": 512, "side": 16, "topic": 512, "detail": 4096}[field]
                limited_text(device[field], f"device.{field}", maximum=maximum)
            for field in ("receive_hz", "valid_hz", "age_s"):
                if device[field] is not None:
                    bounded_number(device[field], f"device.{field}")
        for event in value["events"]:
            limited_text(event["id"], "event.id")
            limited_text(event["message"], "event.message", maximum=4096)
        if value["resources"]["disk_free_bytes"] is not None:
            bounded_integer(value["resources"]["disk_free_bytes"], "resources.disk_free_bytes", maximum=UINT64_MAX)
        validate_management(value["management"])
    if version == DEVICE_PROTOCOL_VERSION:
        validate_process_control(value["process_control"])


def limited_text(value, context, *, maximum=256, nonempty=False):
    text(value, context, nonempty=nonempty)
    if len(value.encode("utf-8")) > maximum or "\x00" in value:
        raise ProtocolError(f"{context} exceeds {maximum} UTF-8 bytes or contains NUL")
    return value


def bounded_integer(value, context, *, minimum=0, maximum=UINT32_MAX):
    integer(value, context, minimum=minimum)
    if value > maximum:
        raise ProtocolError(f"{context} must be <= {maximum}")
    return value


def bounded_number(value, context, *, minimum=0, maximum=1e12):
    number(value, context, minimum=minimum)
    if value > maximum:
        raise ProtocolError(f"{context} must be <= {maximum}")
    return value


def bounded_array(value, context):
    if type(value) is not list or len(value) > MAX_RECORDS:
        raise ProtocolError(f"{context} must be an array of at most {MAX_RECORDS} records")
    return value


def timestamp(value, context):
    limited_text(value, context, maximum=64, nonempty=True)
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if "T" not in value or parsed.utcoffset() is None:
            raise ValueError("timezone required")
    except (ValueError, OverflowError) as error:
        raise ProtocolError(f"{context} must be an ISO datetime with timezone") from error


def validate_controller(value):
    fields = {"controller_instance_id", "state_revision", "state", "session_id", "start_request_id",
              "session_label", "output_path", "recorder_alive", "elapsed_sec", "planned_duration_sec",
              "topics", "max_duration_sec", "detail"}
    mapping(value, fields, "controller")
    for key in ("controller_instance_id", "session_id", "start_request_id"):
        limited_text(value[key], f"controller.{key}", nonempty=key == "controller_instance_id")
    limited_text(value["session_label"], "controller.session_label", maximum=512)
    for key in ("output_path", "detail"):
        limited_text(value[key], f"controller.{key}", maximum=4096)
    bounded_integer(value["state_revision"], "controller.state_revision", maximum=UINT64_MAX)
    bounded_integer(value["state"], "controller.state", maximum=255)
    boolean(value["recorder_alive"], "controller.recorder_alive")
    bounded_number(value["elapsed_sec"], "controller.elapsed_sec")
    for key in ("planned_duration_sec", "max_duration_sec"):
        bounded_integer(value[key], f"controller.{key}")
    topics = bounded_array(value["topics"], "controller.topics")
    for topic in topics:
        limited_text(topic, "controller topic", maximum=512, nonempty=True)
    if len(set(topics)) != len(topics):
        raise ProtocolError("controller.topics contains duplicates")


def validate_operation(value):
    mapping(value, {"request_id", "state", "code", "detail", "trial_id"}, "operation")
    limited_text(value["request_id"], "operation.request_id", nonempty=True)
    choice(value["state"], {"accepted", "rejected", "unknown"}, "operation.state")
    limited_text(value["code"], "operation.code", maximum=64, nonempty=True)
    limited_text(value["detail"], "operation.detail", maximum=4096)
    limited_text(value["trial_id"], "operation.trial_id")


def validate_batch(value):
    mapping(value, {"id", "label", "created_at"}, "batch")
    limited_text(value["id"], "batch.id", nonempty=True)
    limited_text(value["label"], "batch.label", maximum=512, nonempty=True)
    timestamp(value["created_at"], "batch.created_at")


def validate_trial(value):
    mapping(value, {"id", "batch_id", "label", "state", "session_id", "output_path", "duration_s",
                    "elapsed_s", "quality", "detail", "created_at"}, "trial")
    for key in ("id", "batch_id", "session_id"):
        limited_text(value[key], f"trial.{key}", nonempty=key != "session_id")
    limited_text(value["label"], "trial.label", maximum=512, nonempty=True)
    choice(value["state"], TRIAL_STATES, "trial.state")
    for key in ("output_path", "detail"):
        limited_text(value[key], f"trial.{key}", maximum=4096)
    limited_text(value["quality"], "trial.quality", maximum=64, nonempty=True)
    bounded_number(value["duration_s"], "trial.duration_s", maximum=300)
    bounded_number(value["elapsed_s"], "trial.elapsed_s")
    timestamp(value["created_at"], "trial.created_at")


def validate_management(value):
    mapping(value, {"enabled", "controller", "batches", "trials", "operation"}, "management")
    boolean(value["enabled"], "management.enabled")
    if value["controller"] is not None:
        validate_controller(value["controller"])
    if value["operation"] is not None:
        validate_operation(value["operation"])
    for key, validator in (("batches", validate_batch), ("trials", validate_trial)):
        ids = set()
        for item in bounded_array(value[key], f"management.{key}"):
            validator(item)
            if item["id"] in ids:
                raise ProtocolError(f"management.{key} contains duplicate id")
            ids.add(item["id"])


def validate_process_group(value):
    mapping(value, {"id", "label", "kind", "state", "owned", "pid", "detail", "log_path",
                    "can_start", "can_stop", "can_restart", "topics"}, "process_group")
    limited_text(value["id"], "process_group.id", nonempty=True)
    limited_text(value["label"], "process_group.label", maximum=512, nonempty=True)
    choice(value["kind"], {"camera", "tracker", "glove", "tactile"}, "process_group.kind")
    choice(value["state"], PROCESS_STATES, "process_group.state")
    for key in ("owned", "can_start", "can_stop", "can_restart"):
        boolean(value[key], f"process_group.{key}")
    if value["pid"] is not None:
        bounded_integer(value["pid"], "process_group.pid", minimum=1)
    for key in ("detail", "log_path"):
        limited_text(value[key], f"process_group.{key}", maximum=4096)
    topics = bounded_array(value["topics"], "process_group.topics")
    for topic in topics:
        limited_text(topic, "process_group topic", maximum=512, nonempty=True)
    if len(set(topics)) != len(topics):
        raise ProtocolError("process_group.topics contains duplicates")


def validate_process_control(value):
    mapping(value, {"enabled", "groups"}, "process_control")
    boolean(value["enabled"], "process_control.enabled")
    groups = value["groups"]
    if type(groups) is not list or len(groups) > MAX_PROCESS_GROUPS:
        raise ProtocolError(f"process_control.groups must be an array of at most {MAX_PROCESS_GROUPS} records")
    ids = set()
    for group in groups:
        validate_process_group(group)
        if group["id"] in ids:
            raise ProtocolError("process_control.groups contains duplicate id")
        ids.add(group["id"])


def _detail_json(value, context, *, depth=0):
    """Bounded evidence/report JSON, not an arbitrary file or command payload."""
    if depth > MAX_DETAIL_DEPTH:
        raise ProtocolError(f"{context} exceeds maximum details nesting depth")
    if value is None or type(value) is bool:
        return
    if type(value) in (int, float):
        try:
            finite = math.isfinite(value)
        except (OverflowError, ValueError):
            finite = False
        if not finite:
            raise ProtocolError(f"{context} contains a non-finite number")
        return
    if type(value) is str:
        limited_text(value, context, maximum=MAX_DETAIL_STRING_BYTES)
        return
    if type(value) is list:
        for item in bounded_array(value, context):
            _detail_json(item, context, depth=depth + 1)
        return
    if type(value) is dict:
        if len(value) > MAX_RECORDS:
            raise ProtocolError(f"{context} exceeds {MAX_RECORDS} object fields")
        for key, item in value.items():
            limited_text(key, f"{context} key", maximum=MAX_DETAIL_STRING_BYTES)
            _detail_json(item, context, depth=depth + 1)
        return
    raise ProtocolError(f"{context} contains a non-JSON value")


def validate_trial_details(value):
    mapping(value, {"trial_id", "metadata", "quality", "artifacts"}, "trial_details")
    limited_text(value["trial_id"], "trial_details.trial_id", nonempty=True)
    mapping(value["metadata"], {"start", "finish"}, "trial_details.metadata")
    quality = value["quality"]
    mapping(quality, {"state", "verdict", "detail", "attempt", "report_path", "report"}, "trial_details.quality")
    choice(quality["state"], {"NOT_ASSESSED", "QUEUED", "RUNNING", "COMPLETED", "ERROR", "INTERRUPTED"}, "quality.state")
    choice(quality["verdict"], {"", "ok", "warnings"}, "quality.verdict")
    bounded_integer(quality["attempt"], "quality.attempt")
    for field in ("detail", "report_path"):
        limited_text(quality[field], f"quality.{field}", maximum=4096)
    for name, evidence in (("metadata.start", value["metadata"]["start"]),
                           ("metadata.finish", value["metadata"]["finish"]), ("quality.report", quality["report"])):
        if evidence is not None:
            if type(evidence) is not dict:
                raise ProtocolError(f"{name} must be an object or null")
            _detail_json(evidence, name)
    mapping(value["artifacts"], {"start_metadata_path", "finish_metadata_path"}, "trial_details.artifacts")
    for key, path in value["artifacts"].items():
        limited_text(path, f"artifacts.{key}", maximum=4096)
    if len(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")) > MAX_DETAIL_BYTES:
        raise ProtocolError(f"trial_details exceeds {MAX_DETAIL_BYTES} bytes")


def validate_control_request(value):
    mapping(value, {"version", "request_id", "command"}, "control request", optional={"params"})
    commands = {"snapshot", "ping", "batch.create", "capture.start", "capture.stop"}
    detailed = value["version"] in (DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION)
    if detailed:
        commands |= {"trial.details", "quality.retry"}
    if value["version"] == DEVICE_PROTOCOL_VERSION:
        commands |= {"device.start", "device.stop", "device.restart"}
    choice(value["command"], commands, "command")
    params = value.get("params", {})
    required = {"snapshot": set(), "ping": set(), "batch.create": {"label"},
                "capture.start": {"batch_id", "label", "duration_s", "expected_controller_instance_id", "expected_state_revision"},
                "capture.stop": {"trial_id", "expected_controller_instance_id", "expected_session_id"},
                "trial.details": {"trial_id"}, "quality.retry": {"trial_id"},
                "device.start": {"group_id", "expected_manager_instance_id"},
                "device.stop": {"group_id", "expected_manager_instance_id"},
                "device.restart": {"group_id", "expected_manager_instance_id"}}[value["command"]]
    optional = {"task_name", "operator_name"} if detailed and value["command"] == "capture.start" else set()
    mapping(params, required, "params", optional=optional)
    for key in optional & set(params):
        limited_text(params[key], f"params.{key}", maximum=512)
    for key in required - {"duration_s", "expected_state_revision"}:
        limited_text(params[key], f"params.{key}", maximum=512 if key == "label" else 256, nonempty=True)
    if "duration_s" in required:
        bounded_integer(params["duration_s"], "params.duration_s", minimum=1, maximum=300)
    if "expected_state_revision" in required:
        bounded_integer(params["expected_state_revision"], "params.expected_state_revision", maximum=UINT64_MAX)


def validate_message(value):
    if type(value) is not dict:
        raise ProtocolError("message must be an object")
    if type(value.get("version")) is not int or value["version"] not in (PROTOCOL_VERSION, CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION):
        raise ProtocolError("unsupported protocol version; expected integer 1, 2, 3 or 4")
    text(value.get("request_id"), "request_id", nonempty=True)
    version = value["version"]
    if version in (CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION):
        limited_text(value["request_id"], "request_id", nonempty=True)
    if "command" in value:
        if version == PROTOCOL_VERSION:
            mapping(value, {"version", "request_id", "command"}, "request")
            choice(value["command"], {"snapshot", "ping"}, "command")
        else:
            validate_control_request(value)
    else:
        optional = {"error"} if version == PROTOCOL_VERSION else {"error", "operation"}
        if version in (DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION):
            optional.add("trial_details")
        mapping(value, {"version", "request_id", "ok", "snapshot"}, "response", optional=optional)
        boolean(value["ok"], "ok")
        validate_snapshot(value["snapshot"], version=version)
        if "error" in value:
            text(value["error"], "error")
            if version in (CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION):
                limited_text(value["error"], "error", maximum=4096)
        if version in (CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION) and value.get("operation") is not None:
            validate_operation(value["operation"])
            if value["operation"]["request_id"] != value["request_id"]:
                raise ProtocolError("response operation request_id mismatch")
        if version in (DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION) and value.get("trial_details") is not None:
            validate_trial_details(value["trial_details"])
    json_value(value)
    return value
