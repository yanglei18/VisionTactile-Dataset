"""Formal requests survive UI/queue boundaries without silently losing streams."""
import json
import hashlib
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from vt_capture_core.capture_plan import resolve_capture_plan, validate_capture_plan
from vt_capture_core.recipes import Recipe
from vt_capture_manager.capture_preflight import PlanPreflightError, check_capture_plan
from vt_capture_manager.quality_runner import QualityRunner
from test_management import managed, create_batch, execute
from test_plan_admission import params
from test_model import make, message
from test_quality_plan_compatibility import FrozenQualityStore, evaluate


def configure(service, adapter, monkeypatch, *, tactile_only=False):
    recipe = Recipe("Formal", duration_s=30, cameras=(), trackers=(), version=3,
                    pose_hands=() if tactile_only else ("left", "right"), tactile_hands=("left", "right"))
    adapter.context["topics"] = list(recipe.record_topics())
    service.poll_once()
    activity = {topic: True for topic in recipe.record_topics()}
    healthy = dict(activity)
    monkeypatch.setattr(service.model, "topic_activity", lambda: dict(activity))
    monkeypatch.setattr(service.model, "topic_health", lambda: dict(healthy))
    def cached(phase, hands):
        documents = {name: {"fixture": name} for name in ("config", "rest", "profile")}
        artifacts = {name: dict(document=document, hash_semantics="canonical_json_sha256",
            sha256=hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":")).encode()).hexdigest())
            for name, document in documents.items()}
        return dict(schema_version=1, observation_phase=phase, boundary_exact=False, hands={hand:dict(
            hand=hand, status="verified_snapshot", service_instance_id="service-"+hand, session_id="source-"+hand,
            profile_version=1, source_mode="live", config_sha256=artifacts["config"]["sha256"],
            rest_sha256=artifacts["rest"]["sha256"], profile_sha256=artifacts["profile"]["sha256"],
            artifacts=deepcopy(artifacts) if phase == "before_start" else {}) for hand in hands})
    service.hands = NS(busy=False, snapshot=lambda: {}, close=lambda: None, trial_evidence=cached)
    value = dict(params(recipe, adapter.context, create_batch(service)), admission_policy="formal")
    return recipe, value, activity, healthy


@pytest.mark.parametrize("failure", ["absent", "invalid"])
def test_formal_missing_or_invalid_hand_source_rejects_before_trial(managed, monkeypatch, failure):
    service, adapter = managed
    recipe, values, activity, healthy = configure(service, adapter, monkeypatch)
    topic = "/gloves/left/imu/raw" if failure == "absent" else "/gloves/right/state"
    (activity if failure == "absent" else healthy)[topic] = False
    result = service.dispatch(dict(version=9, request_id="formal", command="capture.start", params=values, expected_software=service.software))
    assert not result["ok"]
    assert result["operation"]["code"] == ("capture_streams_not_ready" if failure == "absent" else "capture_streams_invalid")
    assert topic in result["operation"]["detail"]
    assert not service.store.active_trials() and not adapter.commands


def test_formal_freezes_all_streams_and_rechecks_latest_validity_before_command(managed, monkeypatch):
    service, adapter = managed
    recipe, values, activity, healthy = configure(service, adapter, monkeypatch)
    result = service.dispatch(dict(version=9, request_id="formal", command="capture.start", params=values, expected_software=service.software))
    assert result["ok"]
    metadata = service.store.details(result["operation"]["trial_id"])["metadata"]["start"]
    plan = validate_capture_plan(metadata["capture_plan"])
    assert plan.admission_policy == "formal"
    assert plan.required_topics() == recipe.record_topics() and not plan.optional_topics()
    assert metadata["capture_preflight"]["raw_imu_coverage"] == "required"
    healthy["/gloves/left/state"] = False
    execute(service)
    assert not adapter.commands
    assert service.store.operation("formal")["code"] == "capture_streams_invalid"


def test_formal_tactile_only_dispatches_without_hand_pose_or_calibration(managed, monkeypatch):
    service, adapter = managed
    recipe, values, _, _ = configure(service, adapter, monkeypatch, tactile_only=True)
    service.hands.trial_evidence = lambda *args: pytest.fail("Touch-only must not query hand provenance")
    result = service.dispatch(dict(version=9, request_id="touch", command="capture.start", params=values, expected_software=service.software))
    assert result["ok"]
    execute(service)
    assert len(adapter.commands) == 1
    assert set(recipe.record_topics()) == {"/gloves/left/tactile/samples", "/gloves/right/tactile/samples"}


def test_formal_request_reports_exact_configured_missing_raw(managed, monkeypatch):
    service, adapter = managed
    recipe, values, activity, healthy = configure(service, adapter, monkeypatch)
    adapter.context["topics"].remove("/gloves/left/imu/raw")
    from vt_capture_core.capture_plan import topics_fingerprint
    values["expected_topics_fingerprint"] = topics_fingerprint(adapter.context["topics"])
    with pytest.raises(PlanPreflightError) as caught:
        check_capture_plan(values, adapter.context, activity, topic_health=healthy)
    assert caught.value.code == "capture_plan_mismatch"
    assert "/gloves/left/imu/raw" in str(caught.value)


def raw_batch(clock, **changes):
    values = dict(schema_version=1, hand="right", session_id="one", config_sha256="a", source_mode="live",
                  clock_domain="CLOCK_MONOTONIC", timing_source="host_receive", batch_sequence=1,
                  samples=[NS(node_id=12, sample_index=1, time_valid=True, host_monotonic_ns=clock.ns, host_realtime_ns=clock.ns,
                              raw_values=[0]*6)])
    values.update(changes)
    return NS(**values)


def test_raw_readiness_needs_actual_samples_and_source_freshness_not_pose_heartbeat():
    model, clock, identity = make("glove")
    topic = "/test/imu/raw"
    model.observe(identity, message("glove", clock))
    assert model.topic_health()["/test/state"] and not model.topic_health()[topic]
    model.observe_imu_activity(topic, raw_batch(clock))
    assert model.topic_activity()[topic] and model.topic_health()[topic]
    stale = raw_batch(clock)
    clock.ns += int((model.config.stale_after_s + 1) * 1e9)
    model.observe_imu_activity(topic, stale)
    assert model.topic_activity()[topic] and not model.topic_health()[topic]
    model.observe_imu_activity(topic, raw_batch(clock, samples=[]))
    assert not model.topic_health()[topic]


@pytest.mark.parametrize("changes", [
    {"hand": "left"}, {"session_id": "another-session"}, {"config_sha256": "different-config"},
    {"source_mode": "replay"}, {"config_sha256": ""}, {"clock_domain": "unknown"}, {"batch_sequence": 0},
])
def test_formal_raw_and_pose_must_share_live_hand_session_and_configuration(changes):
    model, clock, identity = make("glove")
    model.observe(identity, message("glove", clock))
    model.observe_imu_activity("/test/imu/raw", raw_batch(clock))
    assert model.recording_health()["/test/state"] and model.recording_health()["/test/imu/raw"]
    model.observe_imu_activity("/test/imu/raw", raw_batch(clock, **changes))
    assert model.topic_activity()["/test/imu/raw"]  # Debug can still record the evidence.
    assert not model.recording_health()["/test/imu/raw"]
    assert not model.recording_health()["/test/state"]
    assert model.topic_health()["/test/state"]  # Preview health stays independent.


@pytest.mark.parametrize("mode", ["replay", "", "legacy"])
def test_fresh_legacy_or_replay_pose_cannot_enable_formal_live_capture(mode):
    model, clock, identity = make("glove")
    model.observe(identity, message("glove", clock, source_mode=mode))
    model.observe_imu_activity("/test/imu/raw", raw_batch(clock))
    assert model.topic_health()["/test/state"]
    assert not model.recording_health()["/test/state"]


def test_oversized_raw_batch_is_not_formally_ready():
    model, clock, _ = make("glove")
    batch = raw_batch(clock)
    batch.samples *= 65
    model.observe_imu_activity("/test/imu/raw", batch)
    assert model.topic_activity()["/test/imu/raw"]
    assert not model.topic_health()["/test/imu/raw"]


def test_tactile_status_or_grid_cannot_masquerade_as_actual_rows():
    model, clock, identity = make("tactile")
    topic = "/test/samples"
    model.observe(identity, message("tactile", clock))
    assert not model.topic_health()[topic]
    values = dict(schema_version=2, hand="right", connected=True, time_valid=True,
                  host_realtime_ns=clock.ns, blocks=[NS(row=0, adc=[0]*20)])
    model.observe_topic_activity(topic, message=NS(**values))
    assert model.topic_health()[topic]
    model.observe_topic_activity(topic, message=NS(**dict(values, connected=False, blocks=[])))
    assert model.topic_activity()[topic] and not model.topic_health()[topic]


def test_formal_final_quality_cannot_downgrade_missing_required_raw_to_optional(tmp_path):
    recipe = Recipe("Formal", cameras=(), trackers=(), pose_hands=("left",), version=3)
    plan = resolve_capture_plan(recipe, admission_policy="formal")
    store = FrozenQualityStore(tmp_path, plan, counts={"/gloves/left/state": 1},
                               finish_topics=["/gloves/left/state"])
    result = evaluate(store, QualityRunner(timeout_s=10))
    full = json.loads((tmp_path / "quality.json").read_text())
    assert result["verdict"] == "warnings"
    assert full["runner_provenance"]["optional_topics"] == []
    assert "/gloves/left/imu/raw" in full["runner_provenance"]["expected_topics"]
    assert any(w["code"] == "expected_topic_missing" and w.get("topic") == "/gloves/left/imu/raw"
               for w in full["warnings"])

def test_frozen_hand_source_must_match_the_pose_on_the_ros_topic():
    model, clock, identity = make("glove")
    pose = message("glove", clock)
    model.observe(identity, pose)
    model.observe_imu_activity("/test/imu/raw", raw_batch(clock))
    producer = dict(status="verified_snapshot", source_mode="live", session_id=pose.session_id,
                    config_sha256=pose.config_sha256, rest_sha256=pose.model_rest_sha256,
                    profile_version=pose.calibration_version)
    evidence = dict(hands={pose.hand: producer})
    assert model.recording_health(evidence)["/test/state"]
    producer["profile_version"] += 1
    assert not model.recording_health(evidence)["/test/state"]
    assert not model.recording_health(evidence)["/test/imu/raw"]
    assert model.topic_health()["/test/state"]


def test_formal_waits_for_provenance_without_waiting_on_socket_and_debug_can_record(managed, monkeypatch):
    service, adapter = managed
    _, values, _, _ = configure(service, adapter, monkeypatch)
    service.hands.trial_evidence = lambda phase, hands: dict(schema_version=1, observation_phase=phase,
        boundary_exact=False, hands={hand:dict(status="unavailable", problems=["尚未取得配置与标定原件"])
                                     for hand in hands})
    request = dict(version=9, request_id="missing-originals", command="capture.start", params=values,
                   expected_software=service.software)
    result = service.dispatch(request)
    assert not result["ok"] and result["operation"]["code"] == "hand_provenance_not_ready"
    assert "调试录制" in result["operation"]["detail"]
    assert not adapter.commands and not service.store.active_trials()
    request["request_id"] = "debug-originals-missing"
    request["params"] = dict(values, admission_policy="debug")
    result = service.dispatch(request)
    assert result["ok"]
    execute(service)
    assert len(adapter.commands) == 1


def test_repeated_start_does_not_reobserve_sources_and_queued_profile_change_rejects(managed, monkeypatch):
    service, adapter = managed
    _, values, _, _ = configure(service, adapter, monkeypatch)
    original = service.hands.trial_evidence
    calls = []
    def observed(phase, hands):
        calls.append((phase, hands))
        return original(phase, hands)
    service.hands.trial_evidence = observed
    request = dict(version=9, request_id="single-observation", command="capture.start", params=values,
                   expected_software=service.software)
    first = service.dispatch(request)
    assert first["ok"]
    count = len(calls)
    assert service.dispatch(request)["ok"] and len(calls) == count
    def changed(phase, hands):
        value = original(phase, hands)
        value["hands"]["left"]["profile_version"] += 1
        return value
    service.hands.trial_evidence = changed
    execute(service)
    assert not adapter.commands
    assert service.store.operation("single-observation")["code"] == "hand_provenance_not_ready"
