import copy
import json

import pytest

from vt_capture_core import (
    DETAIL_PROTOCOL_VERSION, ControlRequest, ControlResponse, ControlSnapshot,
    DetailRequest, DetailResponse, ProtocolError, decode_message, encode_message,
)


def details():
    return {"trial_id": "trial", "metadata": {"start": {"task_name": "抓取", "devices": ["left", "right"]}, "finish": None},
            "quality": {"state": "NOT_ASSESSED", "verdict": "", "detail": "", "attempt": 0, "report_path": "", "report": None},
            "artifacts": {"start_metadata_path": "/manager/trials/trial.start.json", "finish_metadata_path": ""}}


@pytest.fixture
def reply():
    return DetailResponse("request", ControlSnapshot("live", "manager"), trial_details=details()).to_dict()


def test_v3_defaults_and_snapshot_remain_v2_shape():
    assert DETAIL_PROTOCOL_VERSION == 3
    assert DetailRequest("r").to_dict() == {"version": 3, "request_id": "r", "command": "snapshot", "params": {}}
    snapshot = ControlSnapshot("live", "manager")
    response = DetailResponse("r", snapshot).to_dict()
    assert "trial_details" not in response
    assert response["snapshot"] == ControlResponse("r", snapshot).to_dict()["snapshot"]
    assert decode_message(encode_message(response)) == response


@pytest.mark.parametrize("command", ["trial.details", "quality.retry"])
def test_v3_detail_commands_not_available_to_v1_v2(command):
    assert decode_message(encode_message(DetailRequest("r", command, {"trial_id": "trial"})))["command"] == command
    for version in (1, 2):
        with pytest.raises(ProtocolError):
            encode_message({"version": version, "request_id": "r", "command": command, "params": {"trial_id": "trial"}})
    for params in ({}, {"trial_id": ""}, {"trial_id": "trial", "path": "/arbitrary"}):
        with pytest.raises(ProtocolError):
            encode_message(DetailRequest("r", command, params))


def test_v3_optional_start_metadata_is_not_silently_accepted_by_v2():
    base = {"batch_id": "b", "label": "t", "duration_s": 30, "expected_controller_instance_id": "c", "expected_state_revision": 2}
    for extra in ({}, {"task_name": "", "operator_name": ""}, {"task_name": "a" * 512, "operator_name": "操作者"}):
        request = DetailRequest("r", "capture.start", {**base, **extra})
        assert decode_message(encode_message(request)) == request.to_dict()
        if extra:
            with pytest.raises(ProtocolError):
                encode_message(ControlRequest("r", "capture.start", {**base, **extra}))
    for extra in ({"task_name": "a" * 513}, {"operator_name": "操" * 171}, {"task_name": None}, {"operator_name": True}):
        with pytest.raises(ProtocolError):
            encode_message(DetailRequest("r", "capture.start", {**base, **extra}))


def test_details_and_null_roundtrip_and_old_responses_remain_strict(reply):
    assert decode_message(encode_message(reply)) == reply
    reply["trial_details"] = None
    assert decode_message(encode_message(reply)) == reply
    for version in (1, 2):
        old = copy.deepcopy(reply)
        old["version"] = version
        if version == 1:
            del old["snapshot"]["management"]
            del old["operation"]
        with pytest.raises(ProtocolError):
            encode_message(old)


@pytest.mark.parametrize("path,value", [
    (("trial_id",), ""), (("trial_id",), "x" * 257),
    (("metadata", "start"), []), (("metadata", "finish"), "not an object"),
    (("quality", "state"), "PASS"), (("quality", "verdict"), "fail"),
    (("quality", "attempt"), -1), (("quality", "attempt"), True), (("quality", "attempt"), 1 << 32),
    (("quality", "detail"), "x" * 4097), (("quality", "report_path"), "x" * 4097),
    (("quality", "report"), []), (("quality", "report"), {"number": float("nan")}),
    (("metadata", "start"), {"value": float("inf")}),
    (("metadata", "start"), {"value": 10 ** 500}),
    (("metadata", "start"), {"field": "x" * 8193}),
    (("metadata", "start"), {"field": "操" * 2731}),
    (("metadata", "start"), {"field": "embedded\x00nul"}),
    (("metadata", "start"), {"values": [None] * 201}),
    (("metadata", "start"), {f"key{i}": None for i in range(201)}),
    (("artifacts", "start_metadata_path"), "x" * 4097),
])
def test_bounded_details_reject_bad_fields(reply, path, value):
    target = reply["trial_details"]
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ProtocolError):
        encode_message(reply)
    with pytest.raises(ProtocolError):
        decode_message(json.dumps(reply))


def test_details_depth_size_and_unknown_fields(reply):
    nested = {"number": -1.5}
    for _ in range(13):
        nested = {"child": nested}
    reply["trial_details"]["metadata"]["start"] = nested
    with pytest.raises(ProtocolError, match="depth"):
        encode_message(reply)
    reply["trial_details"]["metadata"]["start"] = {str(i): "x" * 8192 for i in range(65)}
    with pytest.raises(ProtocolError, match="524288"):
        encode_message(reply)
    reply["trial_details"]["metadata"]["start"] = {"unknown_future_evidence": True, "signed": -1.5}
    encode_message(reply)  # Evidence keys are extensible; envelope keys are not.
    reply["trial_details"]["quality"]["command"] = "shell"
    with pytest.raises(ProtocolError):
        encode_message(reply)


@pytest.mark.parametrize("state", ["NOT_ASSESSED", "QUEUED", "RUNNING", "COMPLETED", "ERROR", "INTERRUPTED"])
@pytest.mark.parametrize("verdict", ["", "ok", "warnings"])
def test_quality_execution_state_and_data_verdict_are_distinct(reply, state, verdict):
    reply["trial_details"]["quality"].update(state=state, verdict=verdict)
    assert decode_message(encode_message(reply))["trial_details"]["quality"]["verdict"] == verdict


def test_quality_details_are_not_allowed_inside_v3_snapshot(reply):
    reply["snapshot"]["management"]["trial_details"] = details()
    with pytest.raises(ProtocolError):
        encode_message(reply)
