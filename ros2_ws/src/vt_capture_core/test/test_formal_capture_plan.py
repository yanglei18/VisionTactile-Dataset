"""Formal intent cannot be weakened by missing sources or legacy recipe fields."""
from copy import deepcopy
import hashlib
import json

import pytest

from vt_capture_core import encode_message, decode_message, PlanControlRequest, ProtocolError
from vt_capture_core.capture_plan import (
    CapturePlanError, recipe_for_topics, require_available_streams,
    resolve_capture_plan, validate_capture_plan,
)
from vt_capture_core.recipes import Recipe
from test_plan_protocol import planned_start


@pytest.mark.parametrize("hands", [("left",), ("right",), ("left", "right")])
def test_formal_pose_has_matching_raw_and_all_streams_required(hands):
    recipe = Recipe("Formal", cameras=(), trackers=(), pose_hands=hands, version=3)
    plan = resolve_capture_plan(recipe, admission_policy="formal")
    assert plan.version == 4
    assert plan.required_topics() == recipe.record_topics()
    assert not plan.optional_topics()
    for hand in hands:
        assert plan.topic_types[f"/gloves/{hand}/imu/raw"] == "vt_glove_msgs/msg/RawImuBatch"
        assert plan.topic_types[f"/gloves/{hand}/state"] == "vt_glove_msgs/msg/GloveState"
    assert validate_capture_plan(plan.to_dict()) == plan
    assert recipe_for_topics(plan.record_topics(), name=recipe.name, duration_s=recipe.duration_s) == recipe
    with pytest.raises(CapturePlanError):
        require_available_streams(plan, [])


def test_formal_pose_cannot_silently_use_legacy_result_only_recipe():
    with pytest.raises(CapturePlanError, match="raw IMU"):
        resolve_capture_plan(Recipe("Missing raw", pose_hands=("left",)), admission_policy="formal")


def test_formal_tactile_only_requires_neither_pose_raw_nor_calibration():
    plan = resolve_capture_plan(Recipe("Touch", cameras=(), trackers=(), tactile_hands=("left", "right")),
                                admission_policy="formal")
    assert plan.required_topics() == ("/gloves/left/tactile/samples", "/gloves/right/tactile/samples")


def test_debug_preserves_available_subset_but_policy_is_frozen():
    recipe = Recipe("Debug", cameras=(), trackers=(), pose_hands=("left",), version=3)
    plan = require_available_streams(resolve_capture_plan(recipe, admission_policy="debug"),
                                     ["/gloves/left/state"])
    assert plan.optional_topics() == ("/gloves/left/imu/raw",)
    assert validate_capture_plan(plan.to_dict()) == plan
    changed = deepcopy(plan.to_dict())
    changed["admission_policy"] = "formal"
    content = {key: value for key, value in changed.items() if key != "fingerprint"}
    changed["fingerprint"] = hashlib.sha256(json.dumps(content, ensure_ascii=False,
        sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    with pytest.raises(CapturePlanError):
        validate_capture_plan(changed)


def test_v3_historical_document_keeps_its_exact_fingerprint_and_fields():
    plan = resolve_capture_plan(Recipe("Historical", pose_hands=("left",)))
    before = plan.to_dict()
    assert before["version"] == 3 and "admission_policy" not in before
    assert validate_capture_plan(before).to_dict() == before
    assert "/gloves/left/imu/raw" not in plan.record_topics()


@pytest.mark.parametrize("policy", ["formal", "debug"])
def test_policy_roundtrips_without_changing_legacy_request(policy):
    original = planned_start()
    values = dict(original, admission_policy=policy)
    decoded = decode_message(encode_message(policy_request(values)))
    assert decoded["params"]["admission_policy"] == policy
    assert "admission_policy" not in original


@pytest.mark.parametrize("policy", ["", "available", True, None])
def test_unrecognized_policy_never_falls_back_to_debug(policy):
    with pytest.raises(ProtocolError):
        encode_message(policy_request(dict(planned_start(), admission_policy=policy)))


def test_partial_raw_allowlist_is_reported_as_missing_without_dropping_hand():
    recipe = Recipe("Both", cameras=(), trackers=(), pose_hands=("left", "right"), version=3)
    topics = tuple(topic for topic in recipe.record_topics() if topic != "/gloves/right/imu/raw")
    with pytest.raises(CapturePlanError) as caught:
        recipe_for_topics(topics)
    assert caught.value.details["missing_topics"] == ["/gloves/right/imu/raw"]


def policy_request(values):
    return dict(version=9, request_id="request", command="capture.start", params=values,
                expected_software=dict(source_root="/workspace", source_revision="",
                    source_sha256="a" * 64, configuration_path=""))


@pytest.mark.parametrize("version", [5, 6, 7, 8])
def test_old_wire_versions_cannot_claim_new_admission_policy(version):
    request = policy_request(dict(planned_start(), admission_policy="formal"))
    request["version"] = version
    if version < 8:
        request.pop("expected_software")
    with pytest.raises(ProtocolError):
        encode_message(request)
