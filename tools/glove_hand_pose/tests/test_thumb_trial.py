"""Guided local recording preserves raw evidence and never claims accuracy."""
from copy import deepcopy
import hashlib
import json
import sys
from types import SimpleNamespace

import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import iter_csv
from vt_glove_pose import thumb_trial


def snapshot(t=100., count=1, *, valid=True):
    return dict(kind="thumb_sensor_orientation_preview", host_monotonic_s=t, pair_count=count,
                node_ids=[5, 6], paired_fresh=valid, reference_set=True, reference_ready=valid,
                reference_source="operator_display_reference", reference_host_s=99., valid=valid,
                reasons=[] if valid else ["stale_data"],
                nodes={str(node): dict(sensor_quat_wxyz=[1., 0., 0., 0.],
                                       relative_quat_wxyz=[1., 0., 0., 0.] if valid else None,
                                       gyro_dps=[.1, .2, .3], accel_g=[0., 0., 1.], fresh=valid,
                                       age_s=0. if valid else 1., rotation_magnitude_deg=0. if valid else None,
                                       gravity_direction_change_deg=0. if valid else None,
                                       accel_norm_g=1., gyro_dps_per_lsb=.0175)
                       for node in (5, 6)}, limitations=["ignored repeated prose"])


def samples(t=100.):
    return [Sample(node, (node, -node, 0, 11, 12, 8197), t, 1000+t, node) for node in range(1, 13)]


@pytest.fixture
def analysis(monkeypatch):
    calls = []
    def analyze_trial(rows, start_s, node_ids, completion_reason="completed"):
        calls.append(deepcopy(dict(rows=rows, start_s=start_s, node_ids=node_ids, reason=completion_reason)))
        return dict(kind="test_analysis", rows=len(rows), completion_reason=completion_reason,
                    production_acceptance="not_evaluated")
    monkeypatch.setitem(sys.modules, "vt_glove_pose.thumb_trial_analysis", SimpleNamespace(analyze_trial=analyze_trial))
    return calls


@pytest.fixture
def trial(tmp_path, analysis):
    result = thumb_trial.ThumbTrial(load_config(), tmp_path, 100., snapshot())
    yield result
    result.finish(131., reason="test_cleanup")


def test_unique_directory_and_frozen_configuration_reference(tmp_path, analysis):
    config, reference = load_config(), snapshot()
    original_config, original_reference = deepcopy(config), deepcopy(reference)
    first = thumb_trial.ThumbTrial(config, tmp_path, 100., reference)
    second = thumb_trial.ThumbTrial(config, tmp_path, 100., reference)
    try:
        config["sample_rate_hz"] = 30.
        reference["nodes"]["5"]["sensor_quat_wxyz"][0] = .1
        assert first.directory != second.directory
        assert first.directory.name.startswith("thumb_trial_")
        metadata = json.loads((first.directory/"metadata.json").read_text())
        assert metadata["config"] == original_config
        assert metadata["reference_snapshot"] == original_reference
        assert metadata["planned_duration_s"] == 31.
        assert not metadata["modifies_reference"] and not metadata["modifies_gyro_bias"]
        assert [(p["start_s"], p["end_s"]) for p in metadata["planned_phases"]] == [(0, 3), (3, 8), (8, 20), (20, 23), (23, 31)]
    finally:
        first.finish(100., "cancelled"); second.finish(100., "cancelled")


@pytest.mark.parametrize("elapsed,phase,remaining", [(0, "prepare", 3), (3, "initial_hold", 5),
                                                    (8, "motion", 12), (20, "return", 3),
                                                    (23, "final_hold", 8), (31, "finishing", 0)])
def test_guidance_exact_phase_boundaries(trial, elapsed, phase, remaining):
    status = trial.status(100.+elapsed)
    assert status["phase"] == phase
    assert status["remaining_s"] == remaining
    assert status["total_remaining_s"] == 31-elapsed
    assert status["active"] is (elapsed < 31)
    assert trial.due(100.+elapsed) is (elapsed >= 31)


def test_motion_guides_three_slow_cycles_without_changing_state(trial):
    assert "第 1 / 3" in trial.status(108.)["label"]
    assert "第 2 / 3" in trial.status(112.)["label"]
    assert "第 3 / 3" in trial.status(116.)["label"]
    assert "近节可以自然活动" in trial.status(109.)["instruction"]
    assert trial.pair_snapshot_count == 0


def test_raw_retains_exact_received_nodes_without_inventing_missing_frames(trial):
    received = samples()+[samples(100.01)[4], samples(100.01)[5]]
    assert trial.feed_raw(received)
    assert list(iter_csv(trial.directory/"raw.csv")) == received
    assert trial.raw_sample_count == 14
    assert trial._raw_by_node["5"] == 2 and trial._raw_by_node["12"] == 1


def test_same_timestamp_outputs_are_not_merged_and_invalid_reference_is_preserved(trial):
    first, second = snapshot(), snapshot(count=2, valid=False)
    original = deepcopy(second)
    assert trial.feed_snapshot(first)
    assert trial.feed_snapshot(second)
    second["nodes"]["5"]["gyro_dps"][0] = 999.
    rows = [json.loads(line) for line in (trial.directory/"online.jsonl").read_text().splitlines()]
    assert len(rows) == 2 and rows[0]["host_monotonic_s"] == rows[1]["host_monotonic_s"]
    assert rows[1]["pair_count"] == 2
    assert rows[1]["nodes"] == original["nodes"]
    assert rows[1]["valid"] is False and rows[1]["reference_ready"] is False
    assert rows[1]["reasons"] == ["stale_data"]
    assert "limitations" not in rows[1]
    assert trial.feed_snapshot(None) is False and trial.pair_snapshot_count == 2


def test_finish_hashes_files_calls_analysis_and_is_idempotent(trial, analysis):
    trial.feed_raw(samples())
    trial.feed_snapshot(snapshot())
    result = trial.finish(131.)
    assert result["status"] == "completed" and result["export_complete"]
    assert result["report"]["production_acceptance"] == "not_evaluated"
    assert analysis[-1]["reason"] == "completed"
    assert analysis[-1]["rows"][0]["nodes"]["5"]["accel_g"] == [0, 0, 1]
    manifest = json.loads((trial.directory/"manifest.json").read_text())
    assert manifest["raw_sample_count"] == 12 and manifest["pair_snapshot_count"] == 1
    assert manifest["status"] == "completed" and manifest["product_acceptance"] == "not_evaluated"
    for name, info in manifest["files"].items():
        content = (trial.directory/name).read_bytes()
        assert info == dict(bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
    assert trial.finish(132.) == result
    assert len(analysis) == 1
    assert not trial.feed_raw(samples(132.)) and not trial.feed_snapshot(snapshot(132.))
    assert trial.status(132.)["phase"] == "completed"


@pytest.mark.parametrize("reason", ["cancelled", "disconnected", "stale_data", "window_closed"])
def test_interruption_preserves_raw_online_and_report_without_completion_claim(trial, analysis, reason):
    trial.feed_raw(samples())
    trial.feed_snapshot(snapshot())
    result = trial.finish(108., reason=reason)
    assert result["status"] == "aborted" and result["reason"] == reason
    assert result["export_complete"]  # Complete export of an aborted capture.
    assert analysis[-1]["reason"] == reason
    assert trial.status(110.)["phase"] == "aborted"
    assert list(iter_csv(trial.directory/"raw.csv")) == samples()


@pytest.mark.parametrize("at,raw,pair,expected", [(105., True, True, "ended_before_plan"),
                                                (131., False, False, "no_raw_data"),
                                                (131., True, False, "no_pair_data")])
def test_early_or_empty_recording_cannot_claim_completed(trial, at, raw, pair, expected):
    if raw: trial.feed_raw(samples())
    if pair: trial.feed_snapshot(snapshot())
    result = trial.finish(at)
    assert result["status"] == "aborted" and result["reason"] == expected


def test_raw_capacity_stops_without_deleting_prefix(trial, monkeypatch):
    monkeypatch.setattr(thumb_trial, "MAX_RAW_SAMPLES", 4)
    assert not trial.feed_raw(samples())
    assert list(iter_csv(trial.directory/"raw.csv")) == samples()[:4]
    assert trial.status(101.)["phase"] == "aborted" and trial.due(101.)
    result = trial.finish(101.)
    assert result["reason"] == "raw_sample_limit" and result["raw_sample_count"] == 4


def test_snapshot_capacity_stops_without_losing_existing_rows(trial, monkeypatch):
    monkeypatch.setattr(thumb_trial, "MAX_PAIR_SNAPSHOTS", 2)
    trial.feed_raw(samples())
    assert trial.feed_snapshot(snapshot(count=1)) and trial.feed_snapshot(snapshot(count=2))
    assert not trial.feed_snapshot(snapshot(count=3))
    assert trial.finish(101.)["reason"] == "pair_snapshot_limit"
    assert len((trial.directory/"online.jsonl").read_text().splitlines()) == 2


def test_invalid_snapshot_keeps_previous_evidence_and_surfaces_error(trial):
    trial.feed_raw(samples())
    trial.feed_snapshot(snapshot())
    broken = snapshot(count=2)
    broken["nodes"]["5"]["gyro_dps"][0] = float("nan")
    assert not trial.feed_snapshot(broken)
    result = trial.finish(131.)
    assert result["status"] == "aborted" and not result["export_complete"]
    assert len((trial.directory/"online.jsonl").read_text().splitlines()) == 1
    assert result["errors"]


def test_analysis_failure_keeps_raw_and_writes_manifest_with_failure(trial, monkeypatch):
    trial.feed_raw(samples()); trial.feed_snapshot(snapshot())
    def fail(*args, **kwargs):
        raise RuntimeError("analysis test failure")
    monkeypatch.setitem(sys.modules, "vt_glove_pose.thumb_trial_analysis", SimpleNamespace(analyze_trial=fail))
    result = trial.finish(131.)
    assert result["status"] == "aborted" and not result["export_complete"]
    assert result["report"] is None
    manifest = json.loads((trial.directory/"manifest.json").read_text())
    assert manifest["capture_status"] == "completed" and manifest["status"] == "aborted"
    assert "analysis test failure" in manifest["errors"][0]
    assert len(list(iter_csv(trial.directory/"raw.csv"))) == 12


def test_report_write_failure_preserves_evidence_and_reports_save_error(trial, monkeypatch):
    trial.feed_raw(samples()); trial.feed_snapshot(snapshot())
    original = thumb_trial.write_json_exclusive
    def fail_report(path, value):
        if path.name == "report.json": raise OSError("disk simulation")
        return original(path, value)
    monkeypatch.setattr(thumb_trial, "write_json_exclusive", fail_report)
    result = trial.finish(131.)
    assert result["status"] == "aborted" and not result["export_complete"]
    assert result["report_path"] is None
    assert (trial.directory/"manifest.json").exists()
    assert len(list(iter_csv(trial.directory/"raw.csv"))) == 12


def test_real_analysis_roundtrip_for_complete_synthetic_trial(tmp_path):
    """Exercise the real analyzer, not hardware accuracy or operator motion."""
    config = load_config()
    config["sample_rate_hz"] = 30.
    trial = thumb_trial.ThumbTrial(config, tmp_path, 100., snapshot())
    received = [sample for index in range(930) for sample in samples(100.+index/30.)]
    assert trial.feed_raw(received)
    for index in range(930):
        assert trial.feed_snapshot(snapshot(100.+index/30., count=index+1))
    result = trial.finish(131.)
    assert result["status"] == "completed" and result["export_complete"]
    assert result["report"]["kind"] == "independent_sensor_orientation_trial_analysis"
    assert result["report"]["status"] == "available"
    assert result["report"]["acceptance_evaluated"] is False
    assert result["report"]["nodes"]["5"]["return_orientation_difference_deg"] == pytest.approx(0.)
    assert len(list(iter_csv(trial.directory/"raw.csv"))) == 11160
    assert len((trial.directory/"online.jsonl").read_text().splitlines()) == 930
    assert json.loads((trial.directory/"report.json").read_text()) == result["report"]
