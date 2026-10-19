import copy
import json

import pytest

from vt_capture_core import (
    DEVICE_PROTOCOL_VERSION, DeviceControlRequest, DeviceControlResponse,
    DeviceControlSnapshot, ProcessControlSnapshot, ProcessGroupSnapshot,
    OperationSnapshot, ProtocolError, decode_message, encode_message,
)
from vt_capture_core.protocol import DEVICE_PROTOCOL_VERSION as WIRE_VERSION
from vt_capture_core.validation import PROCESS_STATES


@pytest.fixture
def reply():
    return DeviceControlResponse(
        "request", DeviceControlSnapshot("live", "manager", process_control=ProcessControlSnapshot(
            enabled=True, groups=[ProcessGroupSnapshot(
                "cameras", "相机组", "camera", state="RUNNING", owned=True, pid=123,
                detail="Process alive; waiting for samples", log_path="/logs/cameras.log",
                can_stop=True, can_restart=True, topics=["/camera/color/image_raw"])])),
        operation=OperationSnapshot("request", "accepted", "QUEUED"),
    ).to_dict()


def test_v4_defaults_exports_and_roundtrip(reply):
    assert DEVICE_PROTOCOL_VERSION == WIRE_VERSION == 4
    assert DeviceControlRequest("r").to_dict() == {"version": 4, "request_id": "r", "command": "snapshot", "params": {}}
    default = DeviceControlResponse("r", DeviceControlSnapshot("live", "manager")).to_dict()
    assert default["snapshot"]["process_control"] == {"enabled": False, "groups": []}
    assert "trial_details" not in default
    assert decode_message(encode_message(reply)) == reply
    assert decode_message(encode_message(default)) == default
    assert ProcessControlSnapshot().to_dict() == {"enabled": False, "groups": []}
    assert ProcessGroupSnapshot("g", "Gloves", "glove").to_dict()["state"] == "DISABLED"


@pytest.mark.parametrize("command", ["device.start", "device.stop", "device.restart"])
def test_device_commands_are_v4_only_with_exact_targets(command):
    params = {"group_id": "cameras", "expected_manager_instance_id": "manager"}
    request = DeviceControlRequest("r", command, params)
    assert decode_message(encode_message(request)) == request.to_dict()
    for version in (1, 2, 3):
        with pytest.raises(ProtocolError):
            encode_message({**request.to_dict(), "version": version})
    for invalid in ({}, {"group_id": "cameras"}, {"expected_manager_instance_id": "manager"},
                    {**params, "argv": ["echo"]}, {**params, "pid": 123},
                    {**params, "path": "/arbitrary"}, {**params, "shell": "anything"}):
        with pytest.raises(ProtocolError):
            encode_message(DeviceControlRequest("r", command, invalid))
    for key in params:
        for value in ("", "  ", "x" * 257, "界" * 86, "nul\x00", None, 1, False):
            with pytest.raises(ProtocolError):
                encode_message(DeviceControlRequest("r", command, {**params, key: value}))


@pytest.mark.parametrize("version", [1, 2, 3])
def test_old_snapshots_reject_new_process_control(reply, version):
    reply["version"] = version
    if version == 1:
        reply.pop("operation")
        reply["snapshot"].pop("management")
    with pytest.raises(ProtocolError):
        encode_message(reply)
    reply["snapshot"].pop("process_control")
    assert decode_message(encode_message(reply)) == reply


def test_v4_process_control_is_required_and_exact(reply):
    original = copy.deepcopy(reply)
    reply["snapshot"].pop("process_control")
    with pytest.raises(ProtocolError):
        encode_message(reply)
    for section in (original["snapshot"]["process_control"], original["snapshot"]["process_control"]["groups"][0]):
        for key in list(section):
            item = section.pop(key)
            with pytest.raises(ProtocolError):
                encode_message(original)
            section[key] = item
        section["arbitrary"] = "untrusted"
        with pytest.raises(ProtocolError):
            encode_message(original)
        section.pop("arbitrary")


@pytest.mark.parametrize("key,value", [
    ("id", ""), ("id", "x" * 257), ("id", "界" * 86), ("id", "nul\x00"),
    ("label", ""), ("label", "x" * 513), ("kind", "shell"), ("state", "ACTIVE"),
    ("owned", 1), ("can_start", "true"), ("can_stop", 0), ("can_restart", None),
    ("pid", True), ("pid", 0), ("pid", -1), ("pid", 1.5), ("pid", 1 << 32),
    ("detail", "x" * 4097), ("detail", "nul\x00"), ("log_path", "x" * 4097),
    ("log_path", "界" * 1366), ("topics", {"topic": "/topic"}), ("topics", [""]),
    ("topics", [" "]), ("topics", [True]), ("topics", ["/same", "/same"]),
    ("topics", ["x" * 513]), ("topics", [f"/topic{i}" for i in range(201)]),
])
def test_process_group_bounded_types(reply, key, value):
    reply["snapshot"]["process_control"]["groups"][0][key] = value
    with pytest.raises(ProtocolError):
        encode_message(reply)
    with pytest.raises(ProtocolError):
        decode_message(json.dumps(reply))


@pytest.mark.parametrize("state", sorted(PROCESS_STATES))
@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
def test_group_states_kinds_and_pid_null_roundtrip(reply, state, kind):
    reply["snapshot"]["process_control"]["groups"][0].update(state=state, kind=kind, pid=None)
    assert decode_message(encode_message(reply)) == reply


def test_process_group_boundaries_and_duplicate_ids(reply):
    control = reply["snapshot"]["process_control"]
    group = control["groups"][0]
    group.update(id="i" * 256, label="l" * 512, detail="d" * 4096,
                 log_path="p" * 4096, pid=(1 << 32) - 1, topics=[f"/{i}/" + "t" * 500 for i in range(200)])
    assert decode_message(encode_message(reply)) == reply
    group.update(topics=[])
    control["groups"] = [dict(group, id=f"group-{i}") for i in range(32)]
    encode_message(reply)
    control["groups"].append(dict(group, id="extra"))
    with pytest.raises(ProtocolError, match="32"):
        encode_message(reply)
    control["groups"] = [group, group]
    with pytest.raises(ProtocolError, match="duplicate"):
        encode_message(reply)
    for bad in (None, {}, "groups"):
        control["groups"] = bad
        with pytest.raises(ProtocolError):
            encode_message(reply)
    control["groups"] = []
    control["enabled"] = 1
    with pytest.raises(ProtocolError):
        encode_message(reply)


@pytest.mark.parametrize("command", ["trial.details", "quality.retry"])
def test_v4_preserves_detail_commands(command):
    request = DeviceControlRequest("r", command, {"trial_id": "t"})
    assert decode_message(encode_message(request)) == request.to_dict()


def test_v4_preserves_optional_metadata_and_detail_response(reply):
    params = {"batch_id": "b", "label": "t", "duration_s": 1,
              "expected_controller_instance_id": "c", "expected_state_revision": 0,
              "task_name": "task", "operator_name": "operator"}
    request = DeviceControlRequest("r", "capture.start", params)
    assert decode_message(encode_message(request)) == request.to_dict()
    reply["trial_details"] = {"trial_id": "t", "metadata": {"start": {"task_name": "task"}, "finish": None},
        "quality": {"state": "NOT_ASSESSED", "verdict": "", "detail": "", "attempt": 0,
                    "report_path": "", "report": None},
        "artifacts": {"start_metadata_path": "", "finish_metadata_path": ""}}
    assert decode_message(encode_message(reply)) == reply
    reply["operation"]["request_id"] = "another"
    with pytest.raises(ProtocolError, match="request_id mismatch"):
        encode_message(reply)
