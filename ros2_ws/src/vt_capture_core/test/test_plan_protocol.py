"""A plan-aware start must never silently acquire legacy start semantics."""

import copy

import pytest

from vt_capture_core import (
    PLAN_PROTOCOL_VERSION, PlanCapabilities, PlanControlRequest,
    PlanControlResponse, PlanControlSnapshot, ProtocolError,
    decode_message, encode_message,
)
from vt_capture_core.recipes import Recipe


def planned_start():
    return {
        "batch_id": "batch", "label": "trial", "duration_s": 30,
        "expected_controller_instance_id": "controller", "expected_state_revision": 1,
        "recipe": Recipe("standard", duration_s=30).to_dict(),
        "expected_topics_fingerprint": "a" * 64,
    }


def test_plan_dtos_are_explicit_and_roundtrip():
    assert PLAN_PROTOCOL_VERSION == 5
    assert PlanControlRequest("r").to_dict()["version"] == 5
    snapshot = PlanControlSnapshot("live", "manager", capabilities=PlanCapabilities(capture_plan=True))
    response = PlanControlResponse("r", snapshot)
    result = decode_message(encode_message(response))
    assert result["snapshot"]["capabilities"] == {
        "record": False, "device_control": False, "capture_plan": True,
    }
    assert result["snapshot"]["process_control"] == {"enabled": False, "groups": []}
    assert "trial_details" not in result
    request = PlanControlRequest("r", "capture.start", planned_start())
    assert decode_message(encode_message(request)) == request.to_dict()


@pytest.mark.parametrize("version", [1, 2, 3, 4])
def test_legacy_schemas_reject_plan_fields(version):
    request = {"version": version, "request_id": "r", "command": "capture.start", "params": planned_start()}
    with pytest.raises(ProtocolError):
        encode_message(request)
    response = PlanControlResponse("r", PlanControlSnapshot("live", "manager")).to_dict()
    response["version"] = version
    if version < 4:
        response["snapshot"].pop("process_control")
    if version == 1:
        response["snapshot"].pop("management")
        response.pop("operation")
    with pytest.raises(ProtocolError):
        encode_message(response)
    response["snapshot"]["capabilities"].pop("capture_plan")
    assert decode_message(encode_message(response)) == response


@pytest.mark.parametrize("field", ["recipe", "expected_topics_fingerprint"])
def test_plan_start_requires_both_guards(field):
    params = planned_start()
    params.pop(field)
    with pytest.raises(ProtocolError):
        encode_message(PlanControlRequest("r", "capture.start", params))


@pytest.mark.parametrize("fingerprint", [None, 1, "", "a" * 63, "a" * 65, "A" * 64, "g" * 64])
def test_plan_fingerprint_is_exact_sha256(fingerprint):
    params = dict(planned_start(), expected_topics_fingerprint=fingerprint)
    with pytest.raises(ProtocolError):
        encode_message(PlanControlRequest("r", "capture.start", params))


@pytest.mark.parametrize("change", [
    {"duration_s": 29}, {"mode": "preview", "duration_s": 0},
    {"cameras": [], "trackers": []}, {"version": 3}, {"name": "x" * 513}, {"arbitrary": True},
])
def test_plan_recipe_is_valid_record_recipe_with_matching_duration(change):
    params = planned_start()
    params["recipe"].update(change)
    original = copy.deepcopy(params)
    with pytest.raises(ProtocolError):
        encode_message(PlanControlRequest("r", "capture.start", params))
    assert params == original


@pytest.mark.parametrize("recipe", [
    Recipe("Camera only", duration_s=30, cameras=("d436",), trackers=()),
    Recipe("Tracker only", duration_s=30, cameras=(), trackers=("torso",)),
])
def test_plan_start_accepts_nonempty_device_subsets(recipe):
    params = dict(planned_start(), recipe=recipe.to_dict())
    request = PlanControlRequest("r", "capture.start", params)
    assert decode_message(encode_message(request)) == request.to_dict()


@pytest.mark.parametrize("capability", [None, "true", 1])
def test_plan_capability_is_required_boolean(capability):
    response = PlanControlResponse("r", PlanControlSnapshot("live", "manager")).to_dict()
    if capability is None:
        response["snapshot"]["capabilities"].pop("capture_plan")
    else:
        response["snapshot"]["capabilities"]["capture_plan"] = capability
    with pytest.raises(ProtocolError):
        encode_message(response)


@pytest.mark.parametrize("command,params", [
    ("device.start", {"group_id": "cameras", "expected_manager_instance_id": "manager"}),
    ("trial.details", {"trial_id": "trial"}),
    ("quality.retry", {"trial_id": "trial"}),
    ("capture.stop", {"trial_id": "trial", "expected_controller_instance_id": "controller", "expected_session_id": "session"}),
])
def test_v5_preserves_other_commands(command, params):
    request = PlanControlRequest("r", command, params)
    assert decode_message(encode_message(request)) == request.to_dict()
