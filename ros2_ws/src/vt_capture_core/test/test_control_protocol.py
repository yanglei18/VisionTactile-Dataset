import copy
import json

import pytest

from vt_capture_core import (
    CONTROL_PROTOCOL_VERSION, PROTOCOL_VERSION, ControlRequest, ControlResponse,
    ControlSnapshot, ControllerSnapshot, ManagementSnapshot, OperationSnapshot,
    ProtocolError, decode_message, encode_message,
)


def managed_snapshot():
    value = ControlSnapshot("live", "manager", management=ManagementSnapshot(
        enabled=True, controller=ControllerSnapshot("controller"))).to_dict()
    value["management"]["batches"] = [{"id": "batch", "label": "动作一", "created_at": "2026-10-07T12:30:00Z"}]
    value["management"]["trials"] = [{"id": "trial", "batch_id": "batch", "label": "trial one",
        "state": "STARTING", "session_id": "", "output_path": "", "duration_s": 30.,
        "elapsed_s": 0., "quality": "NOT_ASSESSED", "detail": "", "created_at": "2026-10-07T12:30:00+00:00"}]
    return value


@pytest.fixture
def reply():
    return {"version": 2, "request_id": "request", "ok": True, "snapshot": managed_snapshot(),
            "operation": OperationSnapshot("request", "accepted", "QUEUED", trial_id="trial").to_dict()}


@pytest.mark.parametrize("command,params", [
    ("snapshot", {}), ("ping", {}), ("batch.create", {"label": "一批"}),
    ("capture.start", {"batch_id": "batch", "label": "trial", "duration_s": 1,
        "expected_controller_instance_id": "controller", "expected_state_revision": 0}),
    ("capture.start", {"batch_id": "batch", "label": "trial", "duration_s": 300,
        "expected_controller_instance_id": "controller", "expected_state_revision": (1 << 64) - 1}),
    ("capture.stop", {"trial_id": "trial", "expected_controller_instance_id": "controller", "expected_session_id": "session"}),
])
def test_v2_command_roundtrip_and_v1_remains_read_only(command, params):
    request = ControlRequest("request", command, params)
    assert decode_message(encode_message(request)) == request.to_dict()
    assert (PROTOCOL_VERSION, CONTROL_PROTOCOL_VERSION) == (1, 2)
    if command not in {"snapshot", "ping"}:
        with pytest.raises(ProtocolError):
            encode_message({"version": 1, "request_id": "request", "command": command})


def test_v2_default_snapshot_params_and_models(reply):
    assert decode_message(encode_message({"version": 2, "request_id": "x", "command": "snapshot"}))["command"] == "snapshot"
    assert decode_message(encode_message(reply)) == reply
    response = ControlResponse("request", ControlSnapshot("live", "instance"))
    assert decode_message(encode_message(response))["snapshot"]["management"]["enabled"] is False
    assert response.to_dict()["operation"] is None
    plain = copy.deepcopy(reply)
    del plain["operation"]
    assert decode_message(encode_message(plain)) == plain


@pytest.mark.parametrize("path,value", [
    (("request_id",), "x" * 257),
    (("operation", "request_id"), "somebody-else"),
    (("operation", "state"), "executing"),
    (("operation", "code"), "x" * 65),
    (("operation", "detail"), "界" * 1500),
    (("snapshot", "recording", "detail"), "x" * 4097),
    (("snapshot", "recording", "elapsed_s"), 1e20),
    (("snapshot", "resources", "disk_free_bytes"), 1 << 64),
    (("snapshot", "management", "enabled"), 1),
    (("snapshot", "management", "controller", "controller_instance_id"), ""),
    (("snapshot", "management", "controller", "state_revision"), True),
    (("snapshot", "management", "controller", "state_revision"), 1 << 64),
    (("snapshot", "management", "controller", "state"), 256),
    (("snapshot", "management", "controller", "recorder_alive"), "false"),
    (("snapshot", "management", "controller", "elapsed_sec"), float("nan")),
    (("snapshot", "management", "controller", "planned_duration_sec"), -1),
    (("snapshot", "management", "controller", "planned_duration_sec"), 1.5),
    (("snapshot", "management", "controller", "max_duration_sec"), 1 << 32),
    (("snapshot", "management", "controller", "topics"), ["/image"] * 2),
    (("snapshot", "management", "controller", "topics"), [f"/image/{n}" for n in range(201)]),
    (("snapshot", "management", "controller", "topics"), ["x" * 513]),
    (("snapshot", "management", "controller", "output_path"), "x" * 4097),
    (("snapshot", "management", "trials", 0, "state"), "FINALIZING"),
    (("snapshot", "management", "trials", 0, "elapsed_s"), float("inf")),
    (("snapshot", "management", "trials", 0, "duration_s"), 301),
    (("snapshot", "management", "trials", 0, "quality"), ""),
    (("snapshot", "management", "trials", 0, "created_at"), "yesterday"),
    (("snapshot", "management", "trials", 0, "created_at"), "2026-10-07T12:30:00"),
    (("snapshot", "management", "batches", 0, "label"), "hello\x00world"),
    (("snapshot", "management", "batches", 0, "created_at"), "2026-10-07"),
])
def test_v2_nested_bounds(reply, path, value):
    target = reply
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ProtocolError):
        encode_message(reply)
    with pytest.raises(ProtocolError):
        decode_message(json.dumps(reply))


@pytest.mark.parametrize("change", [
    {"duration_s": 0}, {"duration_s": 301}, {"duration_s": 5.0}, {"duration_s": True},
    {"expected_state_revision": -1}, {"expected_state_revision": 1 << 64},
    {"expected_controller_instance_id": ""}, {"batch_id": ""}, {"label": "界" * 200},
    {"shell": "anything"},
])
def test_start_params_bounds(change):
    params = {"batch_id": "b", "label": "t", "duration_s": 30,
              "expected_controller_instance_id": "c", "expected_state_revision": 3}
    params.update(change)
    with pytest.raises(ProtocolError):
        encode_message(ControlRequest("request", "capture.start", params))


@pytest.mark.parametrize("command,params", [
    ("exec", {}), ("shell", {"cmd": "echo"}), ("capture.start", {}),
    ("snapshot", {"anything": 1}), ("batch.create", {"label": ""}),
    ("capture.stop", {"trial_id": "trial", "expected_controller_instance_id": "c"}),
])
def test_no_unknown_commands_or_implicit_targets(command, params):
    with pytest.raises(ProtocolError):
        encode_message(ControlRequest("request", command, params))


@pytest.mark.parametrize("field", ["batches", "trials"])
def test_management_record_limits_and_duplicate_ids(reply, field):
    records = reply["snapshot"]["management"][field]
    sample = records[0]
    records[:] = [dict(sample, id=f"id-{index}") for index in range(200)]
    encode_message(reply)
    records.append(dict(sample, id="last"))
    with pytest.raises(ProtocolError, match="200"):
        encode_message(reply)
    records[:] = [sample, sample]
    with pytest.raises(ProtocolError, match="duplicate"):
        encode_message(reply)


def test_v1_cannot_carry_management_or_control_operation(reply):
    reply["version"] = 1
    with pytest.raises(ProtocolError):
        encode_message(reply)
    del reply["operation"]
    with pytest.raises(ProtocolError):
        encode_message(reply)


@pytest.mark.parametrize("state", ["STARTING", "RECORDING", "STOPPING", "COMPLETE", "FAILED", "UNKNOWN"])
def test_trial_six_states(reply, state):
    reply["snapshot"]["management"]["trials"][0]["state"] = state
    assert decode_message(encode_message(reply)) == reply
