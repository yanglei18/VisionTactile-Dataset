"""Immutable recording intent, separate from device observations and bag quality."""

from dataclasses import FrozenInstanceError
import hashlib
import json
from itertools import product

import pytest

from vt_capture_core.capture_plan import (
    CapturePlanError,
    PlannedStream,
    compare_controller_topics,
    recipe_for_topics,
    resolve_capture_plan,
    topics_fingerprint,
    validate_capture_plan,
)
from vt_capture_core.recipes import Recipe


def test_base_plan_describes_exact_recorder_contract():
    plan = resolve_capture_plan(Recipe("Standard", duration_s=30))
    assert plan.record_topics() == plan.recipe.record_topics()
    assert len(plan.streams) == 21
    assert plan.schema_version == "unified-dataset-v2"
    assert plan.version == 2
    assert plan.streams[0] == PlannedStream(
        "/d405_1/color/image_raw", "sensor_msgs/msg/Image", "camera:d405_1", False
    )
    assert plan.topic_types["/d405_1/depth/image_rect_raw"] == "sensor_msgs/msg/Image"
    assert plan.topic_types["/d405_1/color/camera_info"] == "sensor_msgs/msg/CameraInfo"
    assert plan.topic_types["/d405_1/frame_timing"] == "vt_camera_msgs/msg/CameraFrameTiming"
    assert plan.streams[-1] == PlannedStream(
        "/vive/torso/sample", "vt_tracker_msgs/msg/TrackerSample", "tracker:torso", False
    )
    assert plan.device_ids == plan.recipe.device_ids


@pytest.mark.parametrize("enabled", list(product([False, True], repeat=4)))
def test_only_selected_optional_devices_belong_to_plan(enabled):
    pose = tuple(hand for hand, selected in zip(("left", "right"), enabled[:2]) if selected)
    tactile = tuple(hand for hand, selected in zip(("left", "right"), enabled[2:]) if selected)
    plan = resolve_capture_plan(Recipe("Hands", pose_hands=pose, tactile_hands=tactile))
    assert len(plan.streams) == 21 + len(pose) + len(tactile)
    for hand in ("left", "right"):
        pose_stream = PlannedStream(
            f"/gloves/{hand}/state", "vt_glove_msgs/msg/GloveState", f"glove:{hand}", False
        )
        tactile_stream = PlannedStream(
            f"/gloves/{hand}/tactile/samples", "vt_tactile_msgs/msg/TactileSampleArray",
            f"tactile:{hand}", False,
        )
        assert (pose_stream in plan.streams) == (hand in pose)
        assert (tactile_stream in plan.streams) == (hand in tactile)


def test_canonical_roundtrip_and_fingerprint():
    plan = resolve_capture_plan(Recipe("采集", pose_hands=("right", "left")))
    document = plan.to_dict()
    assert validate_capture_plan(json.loads(json.dumps(document))) == plan
    assert plan.recipe.pose_hands == ("left", "right")
    fingerprint = document.pop("fingerprint")
    canonical = json.dumps(document, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    assert fingerprint == hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    assert resolve_capture_plan(Recipe("采集", pose_hands=("left", "right"))).fingerprint == fingerprint
    assert resolve_capture_plan(Recipe("采集", duration_s=299, pose_hands=("left", "right"))).fingerprint != fingerprint


def test_plan_is_detached_and_immutable():
    recipe = Recipe("Detached", pose_hands=["left"])
    plan = resolve_capture_plan(recipe)
    recipe.pose_hands.append("right")
    assert plan.recipe.pose_hands == ("left",)
    document = plan.to_dict()
    document["streams"][0]["topic"] = "/changed"
    document["recipe"]["pose_hands"].clear()
    topic_types = plan.topic_types
    topic_types.clear()
    assert len(plan.topic_types) == 22
    assert plan.recipe.pose_hands == ("left",)
    with pytest.raises(FrozenInstanceError):
        plan.version = 2
    with pytest.raises(FrozenInstanceError):
        plan.streams[0].required = False


@pytest.mark.parametrize("recipe", [
    Recipe("Preview", mode="preview", duration_s=0),
    Recipe("Empty", cameras=(), trackers=()),
    Recipe("Too long", duration_s=301),
])
def test_plan_cannot_weaken_existing_recording_contract(recipe):
    with pytest.raises(CapturePlanError):
        resolve_capture_plan(recipe)


@pytest.mark.parametrize("mutation", [
    lambda value: value.update(version=True),
    lambda value: value.update(version=3),
    lambda value: value.update(schema_version="another-dataset"),
    lambda value: value.update(extra=True),
    lambda value: value.pop("fingerprint"),
    lambda value: value.update(fingerprint="0" * 64),
    lambda value: value["recipe"].update(duration_s=20),
    lambda value: value["streams"].pop(),
    lambda value: value["streams"].append(value["streams"][0]),
    lambda value: value["streams"][0].update(type_name="sensor_msgs/msg/CameraInfo"),
    lambda value: value["streams"][0].update(device_id="camera:d405_2"),
    lambda value: value["streams"][0].update(required=True),
    lambda value: value["streams"][0].update(required=1),
    lambda value: value["streams"][0].update(extra=True),
])
def test_untrusted_documents_reject_missing_forged_or_inconsistent_intent(mutation):
    document = resolve_capture_plan(Recipe("Test")).to_dict()
    mutation(document)
    with pytest.raises(CapturePlanError):
        validate_capture_plan(document)


def test_topic_check_is_order_independent_and_reports_each_difference():
    plan = resolve_capture_plan(Recipe("Standard"))
    assert compare_controller_topics(plan, list(reversed(plan.record_topics()))).matches
    configured = (*plan.record_topics()[1:], "/gloves/left/state", "/vive/torso/sample")
    result = compare_controller_topics(plan, configured)
    assert not result.matches
    assert result.missing_topics == ("/d405_1/color/image_raw",)
    assert result.unexpected_topics == ("/gloves/left/state",)
    assert result.duplicate_topics == ("/vive/torso/sample",)
    assert result.to_dict() == {
        "matches": False, "missing_topics": ["/d405_1/color/image_raw"],
        "unexpected_topics": ["/gloves/left/state"], "duplicate_topics": ["/vive/torso/sample"],
    }


def test_topic_check_reports_all_missing_when_a_finish_allowlist_is_empty():
    plan = resolve_capture_plan(Recipe("Standard"))
    result = compare_controller_topics(plan, [])
    assert not result.matches
    assert result.missing_topics == plan.record_topics()
    assert not result.unexpected_topics and not result.duplicate_topics


@pytest.mark.parametrize("configured", ["/camera/topic", None, {}, [None], ["relative/topic"]])
def test_topic_check_rejects_malformed_controller_evidence(configured):
    with pytest.raises(CapturePlanError):
        compare_controller_topics(resolve_capture_plan(Recipe("Standard")), configured)


@pytest.mark.parametrize("enabled", list(product([False, True], repeat=4)))
def test_recipe_from_controller_allowlist_preserves_every_selected_stream(enabled):
    pose = tuple(hand for hand, selected in zip(("left", "right"), enabled[:2]) if selected)
    tactile = tuple(hand for hand, selected in zip(("left", "right"), enabled[2:]) if selected)
    original = Recipe("From controller", duration_s=45, pose_hands=pose, tactile_hands=tactile)
    recipe = recipe_for_topics(tuple(reversed(original.record_topics())), name=original.name, duration_s=45)
    assert recipe == original
    assert compare_controller_topics(resolve_capture_plan(recipe), original.record_topics()).matches


def test_controller_topic_fingerprint_uses_canonical_sorted_names_only():
    topics = Recipe("Standard").record_topics()
    canonical = json.dumps(sorted(topics), ensure_ascii=False, separators=(",", ":"))
    expected = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    assert topics_fingerprint(topics) == topics_fingerprint(list(reversed(topics))) == expected
    assert topics_fingerprint((*topics, "/gloves/left/state")) != expected
    assert recipe_for_topics(topics).name == "controller-config"
    assert recipe_for_topics(topics).duration_s == 30


@pytest.mark.parametrize("configured", [
    [], "/camera/topic", None, {}, [None], ["relative/topic"], ["/has space"],
    ["/duplicate", "/duplicate"], ["/" + "x" * 512],
    [f"/topic_{number}" for number in range(201)],
])
def test_topic_fingerprint_rejects_unbounded_or_invalid_controller_allowlists(configured):
    with pytest.raises(CapturePlanError):
        topics_fingerprint(configured)
    with pytest.raises(CapturePlanError):
        recipe_for_topics(configured)


@pytest.mark.parametrize("configured", [
    Recipe("Base").record_topics()[1:],
    (*Recipe("Base").record_topics(), "/unrecognized/topic"),
    (*Recipe("Base").record_topics(), "/gloves/third/state"),
])
def test_controller_recipe_rejects_missing_core_or_unrecognized_extras(configured):
    with pytest.raises(CapturePlanError):
        recipe_for_topics(configured)


@pytest.mark.parametrize("duration", [0, 301, True])
def test_controller_recipe_preserves_v1_duration_validation(duration):
    with pytest.raises(CapturePlanError):
        recipe_for_topics(Recipe("Base").record_topics(), duration_s=duration)


def test_controller_recipe_failure_exposes_localizable_structured_evidence():
    configured = (*Recipe("Base").record_topics()[1:], "/unknown/samples")
    with pytest.raises(CapturePlanError) as caught:
        recipe_for_topics(configured)
    assert caught.value.code == "unsupported_controller_topics"
    assert caught.value.details == {
        "missing_topics": ["/d405_1/color/image_raw"],
        "unexpected_topics": ["/unknown/samples"],
    }
    assert str(caught.value)


@pytest.mark.parametrize("configured,code,details", [
    ("/topic", "invalid_controller_topics", {}),
    ([], "controller_topics_size", {}),
    (["relative/topic"], "invalid_controller_topic", {}),
    (["/" + "x" * 512], "controller_topic_too_long", {}),
    (["/duplicate", "/duplicate"], "duplicate_controller_topics", {"duplicate_topics": ["/duplicate"]}),
])
def test_malformed_controller_evidence_exposes_stable_error_codes(configured, code, details):
    with pytest.raises(CapturePlanError) as caught:
        topics_fingerprint(configured)
    assert caught.value.code == code
    assert caught.value.details == details


@pytest.mark.parametrize("recipe", [
    Recipe("Camera only", cameras=("d436",), trackers=()),
    Recipe("Tracker only", cameras=(), trackers=("torso",)),
    Recipe("Hands only", cameras=(), trackers=(), pose_hands=("right",), tactile_hands=("left",)),
])
def test_controller_recipe_supports_nonempty_inventory_subsets(recipe):
    resolved = recipe_for_topics(tuple(reversed(recipe.record_topics())), name=recipe.name,
                                 duration_s=recipe.duration_s)
    assert resolved == recipe
    assert validate_capture_plan(resolve_capture_plan(recipe).to_dict()).recipe == recipe


def test_available_streams_are_frozen_without_changing_candidate_allowlist():
    from vt_capture_core.capture_plan import require_available_streams

    candidate = resolve_capture_plan(Recipe("Available", pose_hands=("left",)))
    available = ["/d405_1/color/image_raw", "/vive/torso/sample"]
    frozen = require_available_streams(candidate, available)
    available.clear()
    assert not candidate.required_topics()
    assert candidate.optional_topics() == candidate.record_topics()
    assert frozen.required_topics() == ("/d405_1/color/image_raw", "/vive/torso/sample")
    assert set(frozen.optional_topics()) == set(candidate.record_topics()) - set(frozen.required_topics())
    assert frozen.record_topics() == candidate.record_topics()
    assert frozen.fingerprint != candidate.fingerprint
    assert validate_capture_plan(frozen.to_dict()) == frozen
    assert require_available_streams(frozen, []).required_topics() == ()
    assert compare_controller_topics(frozen, candidate.record_topics()).matches


@pytest.mark.parametrize("topics", [["/unknown/topic"], ["/vive/torso/sample"] * 2,
                                   "/vive/torso/sample", None, [True]])
def test_availability_must_be_an_explicit_unique_candidate_subset(topics):
    from vt_capture_core.capture_plan import require_available_streams

    with pytest.raises(CapturePlanError):
        require_available_streams(resolve_capture_plan(Recipe("Test")), topics)


def _refingerprint(document):
    content = {key: value for key, value in document.items() if key != "fingerprint"}
    document["fingerprint"] = hashlib.sha256(json.dumps(
        content, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return document


def _legacy_plan_document():
    document = resolve_capture_plan(Recipe("Historical", pose_hands=("left",), version=1)).to_dict()
    document["version"] = 1
    for stream in document["streams"]:
        stream["required"] = True
    return _refingerprint(document)


def test_historical_v1_plan_keeps_required_contract_and_original_fingerprint():
    document = _legacy_plan_document()
    legacy = validate_capture_plan(document)
    assert legacy.version == 1
    assert legacy.to_dict() == document
    assert legacy.required_topics() == legacy.record_topics()
    assert not legacy.optional_topics()
    from vt_capture_core.capture_plan import require_available_streams

    with pytest.raises(CapturePlanError):
        require_available_streams(legacy, [])


@pytest.mark.parametrize("change", ["optional_stream", "missing_trackers", "missing_cameras"])
def test_v1_contract_cannot_be_weakened_even_with_recomputed_fingerprint(change):
    document = _legacy_plan_document()
    if change == "optional_stream":
        document["streams"][0]["required"] = False
    elif change == "missing_trackers":
        document["recipe"]["trackers"] = []
        document["streams"] = [stream for stream in document["streams"]
                               if not stream["device_id"].startswith("tracker:")]
    else:
        document["recipe"]["cameras"] = ["d436"]
        document["streams"] = [stream for stream in document["streams"]
                               if stream["device_id"] not in {"camera:d405_1", "camera:d405_2"}]
    with pytest.raises(CapturePlanError):
        validate_capture_plan(_refingerprint(document))


@pytest.mark.parametrize("plan_version,expected_fingerprint", [
    (1, "547985a3ab46fe35b87f20c4c9dd08aee8d789bd5e1a068fdfac24a8833af0f5"),
    (2, "221275c5a0a8fbe2ba84b544d0a4bfdc811eaa0625f2b9f4be37e5b371f0a747"),
])
def test_legacy_frozen_plans_preserve_camera_bundle_and_original_fingerprints(
    plan_version, expected_fingerprint,
):
    document = resolve_capture_plan(Recipe("Historical", pose_hands=("left",), version=1)).to_dict()
    document["version"] = plan_version
    for stream in document["streams"]:
        stream["required"] = plan_version == 1
    _refingerprint(document)
    assert document["fingerprint"] == expected_fingerprint
    plan = validate_capture_plan(document)
    assert plan.schema_version == "unified-dataset-v1"
    assert len(plan.streams) == 16
    assert plan.to_dict() == document


@pytest.mark.parametrize("version", [1, 2])
def test_controller_recipe_infers_version_from_complete_camera_bundle(version):
    original = Recipe("Camera only", version=version, cameras=("d405_2",), trackers=())
    assert recipe_for_topics(original.record_topics(), name=original.name,
                             duration_s=original.duration_s) == original


def test_controller_recipe_rejects_mixed_or_partial_calibration_bundles():
    legacy = Recipe("Old", version=1, cameras=("d405_1",), trackers=()).record_topics()
    current = Recipe("New", cameras=("d405_2",), trackers=()).record_topics()
    for topics in ((*legacy, *current), (*legacy, "/d405_1/depth/camera_info")):
        with pytest.raises(CapturePlanError, match="supported recipe"):
            recipe_for_topics(topics)


@pytest.mark.parametrize("recipe_version,schema,plan_version", [
    (1, "unified-dataset-v2", 2), (2, "unified-dataset-v1", 2),
    (2, "unified-dataset-v2", 1),
])
def test_schema_recipe_and_historical_plan_versions_must_agree(recipe_version, schema, plan_version):
    document = resolve_capture_plan(Recipe("Mismatch", version=recipe_version)).to_dict()
    document["version"] = plan_version
    document["schema_version"] = schema
    for stream in document["streams"]:
        stream["required"] = plan_version == 1
    with pytest.raises(CapturePlanError):
        validate_capture_plan(_refingerprint(document))


def test_current_plan_records_both_intrinsics_and_factory_extrinsics_without_tracker():
    plan = resolve_capture_plan(Recipe("Camera calibration", cameras=("d436",), trackers=()))
    assert len(plan.streams) == 6
    assert plan.topic_types["/d436/depth/camera_info"] == "sensor_msgs/msg/CameraInfo"
    assert plan.topic_types["/d436/extrinsics/depth_to_color"] == "realsense2_camera_msgs/msg/Extrinsics"
    assert not plan.required_topics()
