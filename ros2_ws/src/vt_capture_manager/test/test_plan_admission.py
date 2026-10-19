"""Plan admission uses fake observations and recorders, never hardware."""
from copy import deepcopy

import pytest

from vt_capture_core.capture_plan import resolve_capture_plan, topics_fingerprint, validate_capture_plan
from vt_capture_core.recipes import Recipe
from vt_capture_manager.capture_preflight import PlanPreflightError, check_capture_plan
from test_management import managed, create_batch, execute, start, stop


def params(recipe, context, batch_id="batch"):
    return dict(batch_id=batch_id, label="planned trial", duration_s=int(recipe.duration_s),
                expected_controller_instance_id=context["controller_instance_id"],
                expected_state_revision=context["state_revision"], recipe=recipe.to_dict(),
                expected_topics_fingerprint=topics_fingerprint(context["topics"]))


def ready_topics(recipe):
    return {stream.topic: True for stream in resolve_capture_plan(recipe).streams
            if stream.type_name not in {"sensor_msgs/msg/CameraInfo", "vt_camera_msgs/msg/CameraFrameTiming", "realsense2_camera_msgs/msg/Extrinsics"}}


@pytest.fixture
def planned(managed, monkeypatch):
    service, adapter = managed
    recipe = Recipe("standard", duration_s=30)
    adapter.context["topics"] = list(recipe.record_topics())
    service.poll_once()
    health = ready_topics(recipe)
    monkeypatch.setattr(service.model, "topic_activity", lambda: dict(health))
    return service, adapter, recipe, health


def planned_start(planned, *, identifier="planned-start", changes=None):
    service, adapter, recipe, _ = planned
    value = params(recipe, adapter.context, create_batch(service))
    value.update(changes or {})
    return service.dispatch(dict(version=5, request_id=identifier, command="capture.start", params=value))


def test_preflight_records_unobserved_auxiliary_streams_without_inventing_readiness(planned):
    _, adapter, recipe, health = planned
    plan, observation = check_capture_plan(params(recipe, adapter.context), adapter.context, health)
    assert plan.record_topics() == recipe.record_topics()
    assert len(observation["unobserved_topics"]) == 12
    assert len(observation["observations"]) == 21
    assert observation["message_type_verification"] == "not_observed"


@pytest.mark.parametrize("failure,code", [
    ("changed", "capture_topics_changed"),
    ("mismatch", "capture_plan_mismatch"),
    ("silent", "capture_no_streams_available"),
    ("unmonitored", "capture_no_streams_available"),
])
def test_preflight_rejects_changed_or_unready_inputs_without_mutating_them(planned, failure, code):
    _, adapter, recipe, health = planned
    value = params(recipe, adapter.context)
    if failure in {"changed", "mismatch"}:
        adapter.context["topics"].append("/gloves/right/state")
        if failure == "mismatch":
            value["expected_topics_fingerprint"] = topics_fingerprint(adapter.context["topics"])
    elif failure == "silent":
        health.update({topic: False for topic in health})
    else:
        health.clear()
    before = deepcopy((value, adapter.context, health))
    with pytest.raises(PlanPreflightError) as caught:
        check_capture_plan(value, adapter.context, health)
    assert caught.value.code == code
    assert (value, adapter.context, health) == before


def test_tactile_state_does_not_claim_raw_sample_availability(planned):
    _, adapter, _, _ = planned
    recipe = Recipe("touch", duration_s=30, tactile_hands=("right",))
    adapter.context["topics"] = list(recipe.record_topics())
    health = ready_topics(recipe)
    health.pop("/gloves/right/tactile/samples")
    health["/gloves/right/tactile/state"] = True
    plan, observed = check_capture_plan(params(recipe, adapter.context), adapter.context, health)
    item = next(item for item in observed["observations"] if item["topic"].endswith("/tactile/samples"))
    assert item["state"] == "not_observed"
    assert item["observation_topic"] is None
    assert "/gloves/right/tactile/samples" in observed["unobserved_topics"]
    assert "/gloves/right/tactile/samples" in plan.optional_topics()


def test_tactile_state_heartbeat_alone_cannot_start_a_recording(planned):
    _, adapter, _, _ = planned
    recipe = Recipe("touch", duration_s=30, tactile_hands=("right",))
    adapter.context["topics"] = list(recipe.record_topics())
    with pytest.raises(PlanPreflightError) as caught:
        check_capture_plan(params(recipe, adapter.context), adapter.context,
                           {"/gloves/right/tactile/state": True})
    assert caught.value.code == "capture_no_streams_available"


def test_tactile_raw_interruption_is_rejected_even_with_state_heartbeats(planned):
    service, adapter, _, activity = planned
    recipe = Recipe("touch", duration_s=30, tactile_hands=("right",))
    adapter.context["topics"] = list(recipe.record_topics())
    service.poll_once()
    activity.clear()
    activity["/gloves/right/tactile/samples"] = True
    request = dict(version=5, request_id="tactile-once", command="capture.start",
                   params=params(recipe, adapter.context, create_batch(service)))
    result = service.dispatch(request)
    assert result["ok"]
    activity.clear()
    activity["/gloves/right/tactile/state"] = True
    execute(service)
    assert not adapter.commands
    assert service.store.operation("tactile-once")["code"] == "capture_streams_not_ready"
    assert service.store.trial(result["operation"]["trial_id"])["state"] == "FAILED"


def test_v5_start_freezes_the_plan_and_preserves_the_ros_command_contract(planned):
    service, adapter, recipe, _ = planned
    result = planned_start(planned)
    assert result["ok"]
    trial_id = result["operation"]["trial_id"]
    metadata = service.store.details(trial_id)["metadata"]["start"]
    assert validate_capture_plan(metadata["capture_plan"]).required_topics() == recipe.record_topics()
    assert metadata["capture_preflight"]["plan_fingerprint"] == metadata["capture_plan"]["fingerprint"]
    assert service.snapshot(5)["capabilities"]["capture_plan"]
    assert "capture_plan" not in service.snapshot(4)["capabilities"]
    execute(service)
    assert service.store.trial(trial_id)["state"] == "RECORDING"
    assert len(adapter.commands) == 1
    assert "recipe" not in adapter.commands[0] and "expected_topics_fingerprint" not in adapter.commands[0]


def test_rejected_plan_does_not_create_trial_or_send_start(planned):
    service, adapter, _, health = planned
    health.clear()
    result = planned_start(planned)
    assert not result["ok"] and result["operation"]["code"] == "capture_no_streams_available"
    assert not service.store.active_trials() and service._commands.empty() and not adapter.commands


def test_plan_survives_optional_metadata_collection_failure(planned, monkeypatch):
    service, _, recipe, _ = planned

    def unavailable(*args):
        raise OSError("optional provenance unavailable")

    monkeypatch.setattr(service._metadata, "start_document", unavailable)
    result = planned_start(planned)
    assert result["ok"]
    metadata = service.store.details(result["operation"]["trial_id"])["metadata"]["start"]
    assert metadata["metadata_status"] == "unknown"
    assert validate_capture_plan(metadata["capture_plan"]).required_topics() == recipe.record_topics()
    assert metadata["capture_preflight"]["plan_fingerprint"] == metadata["capture_plan"]["fingerprint"]
    execute(service)
    assert service.store.trial(result["operation"]["trial_id"])["state"] == "RECORDING"


def test_duplicate_planned_request_returns_original_receipt_after_health_changes(planned):
    service, adapter, recipe, health = planned
    request = dict(version=5, request_id="planned-once", command="capture.start",
                   params=params(recipe, adapter.context, create_batch(service)))
    first = service.dispatch(request)
    assert first["ok"]
    execute(service)
    original = service.store.receipt(service.store.operation("planned-once"))
    health.clear()
    repeated = service.dispatch(request)
    assert repeated["operation"] == original
    assert len(adapter.commands) == 1 and service._commands.empty()


@pytest.mark.parametrize("topics,code", [
    ([], "capture_plan_invalid"),
    (["relative/topic"], "capture_plan_invalid"),
    (["/duplicate", "/duplicate"], "controller_unavailable"),
])
def test_invalid_controller_inventory_is_a_known_unsent_rejection(planned, topics, code):
    service, adapter, recipe, _ = planned
    value = params(recipe, adapter.context, create_batch(service))
    adapter.context["topics"] = topics
    service.poll_once()
    result = service.dispatch(dict(version=5, request_id="invalid-topics", command="capture.start", params=value))
    assert result["operation"]["state"] == "rejected"
    assert result["operation"]["code"] == code
    assert not service.store.active_trials() and service._commands.empty() and not adapter.commands


@pytest.mark.parametrize("change", ["topics", "health", "controller", "unavailable"])
def test_queued_plan_is_checked_again_before_any_recorder_side_effect(planned, change):
    service, adapter, _, health = planned
    result = planned_start(planned)
    trial_id = result["operation"]["trial_id"]
    if change == "topics":
        adapter.context["topics"].append("/gloves/left/state")
    elif change == "health":
        health["/vive/torso/sample"] = False
    elif change == "controller":
        adapter.context["controller_instance_id"] = "new-controller"
    else:
        adapter.available = False
    execute(service)
    assert not adapter.commands
    assert service.store.trial(trial_id)["state"] == "FAILED"
    assert service.store.operation("planned-start")["state"] == "rejected"
    assert not service.store.active_trials()


def test_legacy_start_keeps_its_existing_semantics(planned):
    service, adapter, _, health = planned
    health.clear()
    result = start(service, adapter)
    assert result["ok"]
    assert "capture_plan" not in service.store.details(result["operation"]["trial_id"])["metadata"]["start"]
    execute(service)
    assert len(adapter.commands) == 1


@pytest.mark.parametrize("final_topics", [["/changed/only"], []])
def test_quality_uses_frozen_plan_even_if_finish_reports_a_changed_allowlist(planned, final_topics):
    from test_quality_management import FakeQuality
    from vt_capture_manager.quality_scheduler import QualityScheduler

    service, adapter, recipe, _ = planned
    result = planned_start(planned)
    trial_id = result["operation"]["trial_id"]
    execute(service)
    # Report a different final allowlist without altering any frozen artifact.
    adapter.context["topics"] = final_topics
    stop(service, adapter, trial_id)
    execute(service)
    with service.store.transaction() as db:
        service.store.retry_quality(db, trial_id)
    runner = FakeQuality()
    scheduler = QualityScheduler(service.store, runner, can_run=lambda: True, event=lambda *a, **k: None)
    assert scheduler.run_once()
    assert runner.calls, service.store.quality_job(trial_id)
    assert runner.calls[0]["expected_topics"] == list(recipe.record_topics())
    assert runner.calls[0]["expected_image_rates_hz"] == {
        f"/{camera}/{suffix}": 30. for camera in recipe.cameras
        for suffix in ("color/image_raw", "depth/image_rect_raw")}
    assert any(item["code"] == "recorded_allowlist_changed" for item in runner.calls[0]["advisories"])


@pytest.mark.parametrize("availability", [False, None])
def test_absent_trackers_do_not_block_and_are_optional_in_frozen_plan(planned, availability):
    service, adapter, recipe, activity = planned
    for topic in list(activity):
        if topic.startswith("/vive/"):
            if availability is None:
                activity.pop(topic)
            else:
                activity[topic] = availability
    result = planned_start(planned)
    assert result["ok"]
    details = service.store.details(result["operation"]["trial_id"])
    plan = validate_capture_plan(details["metadata"]["start"]["capture_plan"])
    assert plan.version == 2
    assert plan.optional_topics() == tuple(topic for topic in recipe.record_topics() if topic.startswith("/vive/"))
    assert len(plan.required_topics()) == 18
    execute(service)
    assert len(adapter.commands) == 1


def test_one_receiving_topic_is_enough_and_new_topics_are_still_subscribed(planned):
    service, adapter, recipe, activity = planned
    activity.clear()
    activity["/d436/color/image_raw"] = True
    result = planned_start(planned)
    assert result["ok"]
    plan = validate_capture_plan(service.store.details(result["operation"]["trial_id"])["metadata"]["start"]["capture_plan"])
    assert set(plan.required_topics()) == {"/d436/color/image_raw", "/d436/color/camera_info", "/d436/frame_timing",
                                           "/d436/depth/camera_info", "/d436/extrinsics/depth_to_color"}
    assert plan.record_topics() == recipe.record_topics()
    # A late Tracker is not excluded from the configured Recorder subscription.
    activity["/vive/torso/sample"] = True
    execute(service)
    assert len(adapter.commands) == 1
    assert adapter.context["topics"] == list(recipe.record_topics())


def test_quality_separates_available_sources_from_optional_missing_sources(planned):
    from test_quality_management import FakeQuality
    from vt_capture_manager.quality_scheduler import QualityScheduler

    service, adapter, recipe, activity = planned
    activity.clear()
    activity["/d436/color/image_raw"] = True
    result = planned_start(planned)
    trial_id = result["operation"]["trial_id"]
    execute(service)
    stop(service, adapter, trial_id)
    execute(service)
    with service.store.transaction() as db:
        service.store.retry_quality(db, trial_id)
    runner = FakeQuality()
    scheduler = QualityScheduler(service.store, runner, can_run=lambda: True, event=lambda *a, **k: None)
    assert scheduler.run_once()
    assert runner.calls
    call = runner.calls[0]
    assert set(call["expected_topics"]) == {"/d436/color/image_raw", "/d436/color/camera_info", "/d436/frame_timing",
                                           "/d436/depth/camera_info", "/d436/extrinsics/depth_to_color"}
    assert set(call["optional_topics"]) == set(recipe.record_topics()) - set(call["expected_topics"])
    assert len(call["expected_image_rates_hz"]) == 6  # Late image streams still receive rate checks.


@pytest.mark.parametrize("recipe_version", [1, 2])
def test_camera_calibration_contract_is_frozen_without_tracker_or_live_auxiliary_observers(planned, recipe_version):
    service, adapter, _, activity = planned
    recipe = Recipe("calibrated-camera", duration_s=30, version=recipe_version)
    adapter.context["topics"] = list(recipe.record_topics())
    activity.clear()
    activity["/d405_1/color/image_raw"] = True
    service.poll_once()
    result = planned_start((service, adapter, recipe, activity))
    assert result["ok"]
    plan = validate_capture_plan(service.store.details(result["operation"]["trial_id"])["metadata"]["start"]["capture_plan"])
    expected = {"/d405_1/color/image_raw", "/d405_1/color/camera_info", "/d405_1/frame_timing"}
    if recipe_version == 2:
        expected |= {"/d405_1/depth/camera_info", "/d405_1/extrinsics/depth_to_color"}
    assert set(plan.required_topics()) == expected
    assert plan.schema_version == f"unified-dataset-v{recipe_version}"
    execute(service)
    assert len(adapter.commands) == 1
    # Required calibration is checked after recording, not used as a live gate.


def test_calibration_alone_cannot_start_a_camera_recording(planned):
    _, adapter, recipe, activity = planned
    activity.clear()
    activity["/d405_1/extrinsics/depth_to_color"] = True
    with pytest.raises(PlanPreflightError) as caught:
        check_capture_plan(params(recipe, adapter.context), adapter.context, activity)
    assert caught.value.code == "capture_no_streams_available"
