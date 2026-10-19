"""Frozen quality intent survives plan version changes and finish allowlist drift."""

from copy import deepcopy
from dataclasses import replace
import json

import yaml
import pytest

from vt_capture_core.capture_plan import (
    ResolvedCapturePlan, require_available_streams, resolve_capture_plan,
)
from vt_capture_core.recipes import Recipe
from vt_capture_manager.quality_runner import QualityRunner
from vt_capture_manager.quality_scheduler import QualityScheduler

from test_quality_management import FakeQuality


class FrozenQualityStore:
    """Only the storage boundary used by one scheduler attempt."""

    def __init__(self, tmp_path, plan, *, counts=None, finish_topics=None):
        self.job = dict(trial_id="frozen-trial", output_path=str(tmp_path / "bag"),
                        report_path=str(tmp_path / "quality.json"))
        self.claimed = False
        self.results = []
        self.manifests = []
        configured = list(plan.record_topics()) if plan is not None else ["/legacy/stream"]
        start = {} if plan is None else dict(capture_plan=plan.to_dict())
        self.document = dict(metadata=dict(start=start, finish=dict(controller=dict(
            recorder_alive=False, topics=configured if finish_topics is None else finish_topics))))
        if counts is not None:
            bag = tmp_path / "bag"
            bag.mkdir()
            # These tests verify scheduling/provenance using metadata only, not
            # calibration payload correctness. Nonempty fixture data is opaque.
            entries = [dict(topic_metadata=dict(name=stream.topic,
                            type="unknown/msg/Opaque" if counts.get(stream.topic, 0) else stream.type_name),
                            message_count=counts.get(stream.topic, 0)) for stream in plan.streams]
            info = dict(storage_identifier="mcap", duration=dict(nanoseconds=1_000_000_000),
                        starting_time=dict(nanoseconds_since_epoch=1_000_000_000),
                        message_count=sum(counts.values()), topics_with_message_count=entries,
                        relative_file_paths=[])
            (bag / "metadata.yaml").write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=info)))

    def claim_quality(self):
        if self.claimed:
            return None
        self.claimed = True
        return self.job

    def write_manifest(self, trial_id):
        self.manifests.append(trial_id)

    def details(self, trial_id):
        assert trial_id == self.job["trial_id"]
        return deepcopy(self.document)

    def alignment_snapshot(self, trial_id):
        return None

    def finish_quality(self, job, **result):
        assert job == self.job
        self.results.append(result)


def evaluate(store, runner):
    events = []
    scheduler = QualityScheduler(store, runner, can_run=lambda: True,
                                 event=lambda *args, **kwargs: events.append((args, kwargs)))
    assert scheduler.run_once()
    assert not scheduler.run_once()
    assert not events
    assert len(store.results) == 1
    assert store.results[0]["state"] == "COMPLETED"
    return store.results[0]


def test_no_frozen_plan_preserves_legacy_required_topics_without_invented_image_rates(tmp_path):
    store = FrozenQualityStore(tmp_path, None, finish_topics=["/legacy/image", "/legacy/tracker"])
    runner = FakeQuality()
    evaluate(store, runner)
    assert runner.calls[0]["expected_topics"] == ["/legacy/image", "/legacy/tracker"]
    assert runner.calls[0]["optional_topics"] == []
    assert runner.calls[0]["expected_image_rates_hz"] == {}


def test_v1_frozen_plan_keeps_every_stream_required_despite_reduced_finish_inventory(tmp_path):
    candidate = resolve_capture_plan(Recipe("Historical", pose_hands=("left",), version=1))
    legacy = ResolvedCapturePlan(candidate.recipe,
        tuple(replace(stream, required=True) for stream in candidate.streams), version=1,
        schema_version="unified-dataset-v1")
    original = legacy.to_dict()
    store = FrozenQualityStore(tmp_path, legacy, finish_topics=["/d405_1/color/camera_info"])
    runner = FakeQuality()
    result = evaluate(store, runner)
    call = runner.calls[0]
    assert set(call["expected_topics"]) == set(legacy.record_topics())
    assert call["optional_topics"] == []
    assert call["expected_image_rates_hz"] == {
        stream.topic: 30. for stream in legacy.streams if stream.type_name == "sensor_msgs/msg/Image"}
    assert [warning["code"] for warning in call["advisories"]] == ["recorded_allowlist_changed"]
    assert result["verdict"] == "warnings"
    assert store.document["metadata"]["start"]["capture_plan"] == original


def test_v2_empty_optional_sources_are_not_quality_warnings_in_real_child(tmp_path):
    candidate = resolve_capture_plan(Recipe("Available sources"))
    actual = "/d405_1/color/camera_info"
    plan = require_available_streams(candidate, [actual])
    store = FrozenQualityStore(tmp_path, plan, counts={actual: 1})
    result = evaluate(store, QualityRunner(timeout_s=10))
    full = json.loads((tmp_path / "quality.json").read_text())
    assert result["verdict"] == full["status"] == "ok"
    assert full["warnings"] == []
    assert full["runner_provenance"]["expected_topics"] == [actual]
    assert full["runner_provenance"]["optional_topics"] == sorted(plan.optional_topics())
    assert full["topic_coverage"]["absent_optional_topics"] == sorted(plan.optional_topics())
    assert len(full["runner_provenance"]["expected_image_rates_hz"]) == 6
    assert all(entry["rate_assessment"] == "insufficient_samples"
               for entry in full["topics"].values() if entry["type"] == "sensor_msgs/msg/Image")
    assert result["report"]["topic_coverage"] == full["topic_coverage"]


def test_v2_required_source_lost_after_admission_remains_missing_even_if_finish_drops_it(tmp_path):
    candidate = resolve_capture_plan(Recipe("Available sources"))
    actual, disappeared = "/d405_1/color/camera_info", "/vive/left_wrist/sample"
    plan = require_available_streams(candidate, [actual, disappeared])
    store = FrozenQualityStore(tmp_path, plan, counts={actual: 1},
                               finish_topics=[topic for topic in plan.record_topics() if topic != disappeared])
    result = evaluate(store, QualityRunner(timeout_s=10))
    full = json.loads((tmp_path / "quality.json").read_text())
    assert result["verdict"] == full["status"] == "warnings"
    assert {w.get("topic") for w in full["warnings"] if w["code"] == "expected_topic_missing"} == {disappeared}
    assert "recorded_allowlist_changed" in {w["code"] for w in full["warnings"]}
    assert disappeared in full["runner_provenance"]["expected_topics"]
    assert disappeared not in full["topic_coverage"]["absent_optional_topics"]
    assert set(full["topic_coverage"]["absent_optional_topics"]) == set(plan.optional_topics())


@pytest.mark.parametrize("recipe_version", [1, 2])
def test_image_rate_contract_survives_calibration_schema_upgrade(tmp_path, recipe_version):
    candidate = resolve_capture_plan(Recipe("Rates", version=recipe_version))
    available = [stream.topic for stream in candidate.streams if stream.device_id.startswith("camera:")]
    plan = require_available_streams(candidate, available)
    store = FrozenQualityStore(tmp_path, plan)
    runner = FakeQuality()
    evaluate(store, runner)
    assert len(runner.calls[0]["expected_image_rates_hz"]) == 6
    assert set(runner.calls[0]["expected_image_rates_hz"].values()) == {30.}
    assert len(runner.calls[0]["expected_topics"]) == (12 if recipe_version == 1 else 18)
    assert len(runner.calls[0]["optional_topics"]) == 3
