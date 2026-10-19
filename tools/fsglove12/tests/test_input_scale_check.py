"""Synthetic physical tilt truth; no serial, VQF or human accuracy claims."""
import copy
import csv
import hashlib
import json
from threading import Event

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import digest, load_config, validate_config
from vt_fsglove12.input_scale_check import run_input_scale_check, scale_check_summary
from vt_fsglove12.live_stream import RawSample
import vt_fsglove12.input_scale_check as checker


def synthetic(*, gains=None, degrees=60., pure_yaw=(), corrupt=None, nodes=range(1, 13)):
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {}
    rate = float(config["sample_rate_hz"])
    holds = [dict(label=label, start_s=1000. + start, end_s=1002. + start, time_basis="absolute_monotonic")
             for label, start in (("neutral", 0.), ("fingers_down", 3.5), ("thumb_up", 7.))]
    times = np.arange(round(9.2 * rate) + 1) / rate
    angles = degrees * np.clip(times - 2.2, 0., 1.) - .75 * degrees * np.clip(times - 5.7, 0., 1.)
    speeds = np.r_[0., np.diff(angles)] * rate
    gravity = Rotation.from_rotvec(np.deg2rad(angles)[:, None] * [1., 0., 0.]).inv().apply([0., 0., 1.])
    samples = []
    for index, stamp in enumerate(times):
        for node in nodes:
            scale = config["gyro_dps_per_lsb"] * (gains or {}).get(node, 1.)
            velocity = np.array([0., 0., speeds[index]]) if node in pure_yaw else np.array([speeds[index], 0., 0.])
            gyro = np.rint(velocity / scale + [12, -8, 4]).astype(int)
            g = [0., 0., 1.] if node in pure_yaw else gravity[index]
            accel = np.rint(np.asarray(g) / config["accel_g_per_lsb"]).astype(int)
            values = [*gyro.tolist(), *accel.tolist()]
            if corrupt:
                result = corrupt(values, stamp, node, index)
                if result is None:
                    continue
                values = result
            samples.append(RawSample(node, values, 1000. + stamp, 2000. + stamp, index % 256))
    return tuple(samples), config, holds


def test_mixed_nominal_double_half_scales_and_hash_bound_new_evidence(tmp_path):
    samples, config, holds = synthetic(gains={5: 2., 6: .5})
    before_config, before_holds = copy.deepcopy(config), copy.deepcopy(holds)
    target = tmp_path / "new_check"
    report = run_input_scale_check(samples, config, holds, target)
    summary = scale_check_summary(report)
    assert report["status"] == summary["status"] == "completed"
    assert summary["recommended_nodes"] == ["5", "6"]
    assert len(summary["supported_nodes"]) == 10 and summary["inconclusive_nodes"] == []
    assert summary["nodes"]["5"]["recommended_gain"] == 2.
    assert summary["nodes"]["6"]["recommended_gain"] == .5
    assert "名义 dt" in summary["note"] and "未自动应用" in summary["nodes"]["5"]["reason"]
    assert summary["candidate_available"] and report["candidate_available"]
    saved = json.loads((target / "report.json").read_text())
    candidate = json.loads((target / "config.candidate.json").read_text())
    assert saved == report and report["candidate_config"] == candidate
    assert candidate["gyro_dps_per_lsb_by_node"] == {"5": .0175, "6": .004375}
    assert not candidate["units_verified"]
    validate_config(candidate)
    assert config == before_config and holds == before_holds
    binding = report["source_binding"]
    assert binding["source_config_sha256"] == digest(config)
    assert binding["holds_sha256"] == digest(holds)
    assert binding["written_samples"] == binding["supplied_samples"] == len(samples)
    assert candidate["input_unit_fit_evidence"]["automatic_selection"] == binding
    assert len(binding["algorithm_source_sha256"]) == len(binding["selector_source_sha256"]) == 64
    raw = (target / "raw.csv").read_bytes()
    assert hashlib.sha256(raw).hexdigest() == binding["raw_sha256"]
    assert hashlib.sha256(raw[:report["source_prefix_bytes"]]).hexdigest() == report["source_prefix_sha256"]
    with (target / "raw.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == len(samples)
    for row, sample in zip(rows, samples):
        assert float(row["host_parse_monotonic_s"]) == sample.host_monotonic_s
        assert float(row["host_parse_wall_s"]) == sample.host_wall_s
        assert int(row["node_id"]) == sample.node_id
        assert int(row["status"]) == sample.status
        assert tuple(int(row[name + "_lsb"]) for name in ("gx", "gy", "gz", "ax", "ay", "az")) == sample.values
    for hold in report["selection"]["holds"]:
        assert len(hold["candidates"]) == 31 and len(hold["qualified_nodes"]) == 12
        assert hold["hold"]["start_s"] <= hold["selected_window"]["start_s"]
        assert hold["selected_window"]["end_s"] <= hold["hold"]["end_s"]
        assert hold["selected_window"]["end_s"] - hold["selected_window"]["start_s"] == .5
    assert not report["selection"]["fit_residual_used"] and not report["selection"]["fused_orientation_used"]
    assert not report["physical_range_verified"] and not report["sample_rate_verified"]
    assert not report["pose_accuracy_verified"] and not report["hardware_accessed"]


def test_nominal_all_supported_does_not_write_candidate(tmp_path):
    samples, config, holds = synthetic()
    report = run_input_scale_check(samples, config, holds, tmp_path / "nominal")
    assert len(report["nominal_supported_nodes"]) == 12
    assert not report["recommended_nodes"] and not report["candidate_available"]
    assert report["candidate_path"] is None
    assert not (tmp_path / "nominal" / "config.candidate.json").exists()


def test_bad_channels_do_not_block_supported_or_recommended_channels(tmp_path):
    def corrupt(values, time, node, index):
        if node == 2:
            return None
        if node == 3 and 2.4 < time < 2.65:
            return None
        if node == 4 and 2.4 < time < 2.42:
            values[0] = 32764
        if node == 7 and any(start <= time <= start + 2 for start in (0., 3.5, 7.)):
            values[0] += 80 if index % 2 else -80
        if node == 8:
            values[3:] = [0, 0, 100]
        return values

    samples, config, holds = synthetic(gains={5: 2.}, pure_yaw=(1,), corrupt=corrupt)
    report = run_input_scale_check(samples, config, holds, tmp_path / "mixed")
    summary = scale_check_summary(report)
    assert summary["recommended_nodes"] == ["5"]
    assert summary["supported_nodes"] == ["6", "9", "10", "11", "12"]
    assert summary["inconclusive_nodes"] == ["1", "2", "3", "4", "7", "8"]
    assert "insufficient_tilt_excitation" in summary["nodes"]["1"]["reason_codes"]
    assert "纯航向" in summary["nodes"]["1"]["reason"]
    assert "missing_node_samples" in summary["nodes"]["2"]["reason_codes"]
    assert "transition_receive_gap" in summary["nodes"]["3"]["reason_codes"]
    assert "encoding_boundary_event" in summary["nodes"]["4"]["reason_codes"]
    assert "gyro_bias_window_variation_too_large" in summary["nodes"]["7"]["reason_codes"]
    assert "gravity_magnitude_inconsistent_with_config" in summary["nodes"]["8"]["reason_codes"]
    candidate = json.loads((tmp_path / "mixed" / "config.candidate.json").read_text())
    assert candidate["gyro_dps_per_lsb_by_node"] == {"5": .0175}


def test_small_tilt_is_inconclusive_not_nominal_success(tmp_path):
    samples, config, holds = synthetic(degrees=10., nodes=(1,))
    report = run_input_scale_check(samples, config, holds, tmp_path / "small")
    summary = scale_check_summary(report)
    assert summary["supported_nodes"] == summary["recommended_nodes"] == []
    assert "insufficient_tilt_excitation" in summary["nodes"]["1"]["reason_codes"]
    assert not summary["candidate_available"]


def test_startup_jitter_moves_window_later_without_using_fit_residual(tmp_path):
    def jitter(values, time, node, index):
        if any(start <= time < start + .85 for start in (0., 3.5, 7.)):
            values[0] += 100 if index % 2 else -100
        return values

    samples, config, holds = synthetic(corrupt=jitter, nodes=(1,))
    report = run_input_scale_check(samples, config, holds, tmp_path / "jitter")
    assert report["nominal_supported_nodes"] == ["1"]
    for hold in report["selection"]["holds"]:
        assert hold["selected_window"]["start_s"] >= hold["hold"]["start_s"] + .85
        assert hold["qualified_nodes"] == ["1"]
        assert hold["candidates"][0]["qualified_nodes"] == []


def test_selection_is_unchanged_when_only_transition_data_change(tmp_path):
    samples, config, holds = synthetic(nodes=(1,))
    selected = checker._select(samples, config, holds, None)
    changed = tuple(RawSample(sample.node_id, (300, 400, 500, 0, 0, 8197), sample.host_monotonic_s,
                              sample.host_wall_s, sample.status)
                    if 1002.2 < sample.host_monotonic_s < 1003.2 else sample for sample in samples)
    assert checker._select(changed, config, holds, None) == selected
    moved_holds = [{**hold, "start_s": hold["start_s"] + .5, "end_s": hold["end_s"] + .5} for hold in holds]
    other = checker._select(samples, config, moved_holds, None)
    assert other != selected
    for chosen, hold in zip(other["holds"], moved_holds):
        assert chosen["selected_window"]["start_s"] >= hold["start_s"]
        assert chosen["selected_window"]["end_s"] <= hold["end_s"]
    assert config == load_config() | {"gyro_dps_per_lsb_by_node": {}}
    assert holds[0]["start_s"] == 1000.


def test_best_window_maximizes_count_before_minimizing_noise():
    def corruption(values, time, node, index):
        local = next((time - start for start in (0., 3.5, 7.) if start <= time <= start + 2), None)
        if local is not None:
            if node == 1 and local < 1.:
                return None
            # A perfectly quiet but one-node early window must not win over
            # the slightly noisier later interval with both nodes qualified.
            if node == 2 and local >= 1.:
                values[0] += 2 if index % 2 else -2
        return values

    samples, config, holds = synthetic(corrupt=corruption, nodes=(1, 2))
    selected = checker._select(samples, config, holds, None)
    for hold in selected["holds"]:
        assert hold["qualified_nodes"] == ["1", "2"]
        assert hold["selected_window"]["start_s"] > hold["hold"]["start_s"] + .9


def test_sparse_hold_data_does_not_establish_full_window_coverage(tmp_path):
    def sparse(values, time, node, index):
        if any(start <= time <= start + 2 for start in (0., 3.5, 7.)):
            if not any(start + .9 <= time <= start + 1.15 for start in (0., 3.5, 7.)):
                return None
        return values

    samples, config, holds = synthetic(corrupt=sparse, nodes=(1,))
    report = run_input_scale_check(samples, config, holds, tmp_path / "sparse")
    assert not report["nominal_supported_nodes"] and not report["recommended_nodes"]
    assert "automatic_stationary_selection_rejected" in report["nodes"]["1"]["reasons"]
    assert "stationary_sample_count_inconsistent_with_nominal_dt" in scale_check_summary(report)["nodes"]["1"]["reason_codes"]


def test_empty_samples_create_evidence_not_guessed_candidate(tmp_path):
    _, config, holds = synthetic(nodes=())
    report = run_input_scale_check((), config, holds, tmp_path / "empty")
    assert len(report["rejected_nodes"]) == 12 and not report["candidate_available"]
    assert all(not hold["selection_accepted"] for hold in report["selection"]["holds"])
    assert report["source_binding"]["written_samples"] == 0


@pytest.mark.parametrize("change", ["too_long", "short", "relative", "overlap", "duplicate_label", "nan"])
def test_invalid_holds_are_not_guessed_or_written(tmp_path, change):
    samples, config, holds = synthetic(nodes=())
    if change == "too_long":
        holds[0]["end_s"] += 20
    elif change == "short":
        holds[0]["end_s"] -= 1.
    elif change == "relative":
        holds[0]["time_basis"] = "relative_to_recording_start"
    elif change == "overlap":
        holds[1] = {**holds[1], "start_s": holds[0]["start_s"], "end_s": holds[0]["end_s"]}
    elif change == "duplicate_label":
        holds[1]["label"] = holds[0]["label"]
    else:
        holds[0]["start_s"] = float("nan")
    with pytest.raises(ValueError):
        run_input_scale_check(samples, config, holds, tmp_path / "invalid")
    assert not (tmp_path / "invalid").exists()


def test_existing_output_is_never_replaced(tmp_path):
    _, config, holds = synthetic(nodes=())
    target = tmp_path / "existing"
    target.mkdir()
    (target / "raw.csv").write_text("original")
    with pytest.raises(FileExistsError):
        run_input_scale_check((), config, holds, target)
    assert (target / "raw.csv").read_text() == "original"
    assert sorted(path.name for path in target.iterdir()) == ["raw.csv"]


def test_cancel_before_start_creates_no_files(tmp_path):
    _, config, holds = synthetic(nodes=())
    cancelled = Event()
    cancelled.set()
    report = run_input_scale_check((), config, holds, tmp_path / "cancelled", cancelled=cancelled.is_set)
    assert report["status"] == "cancelled" and report["candidate_path"] is None
    assert not (tmp_path / "cancelled").exists()
    assert scale_check_summary(report)["inconclusive_nodes"] == list(map(str, range(1, 13)))


def test_cancel_during_raw_write_keeps_only_partial_evidence(tmp_path):
    samples, config, holds = synthetic(nodes=(1, 2))
    calls = 0

    def cancelled():
        nonlocal calls
        calls += 1
        return calls >= 3

    report = run_input_scale_check(samples, config, holds, tmp_path / "cancelled", cancelled=cancelled)
    assert report["status"] == "cancelled"
    assert 0 < report["source_binding"]["written_samples"] < len(samples)
    assert not report["candidate_available"]
    assert json.loads((tmp_path / "cancelled" / "report.json").read_text())["status"] == "cancelled"


def test_cancel_during_fit_does_not_publish_candidate(tmp_path, monkeypatch):
    samples, config, holds = synthetic(nodes=(1,))
    cancelled = Event()

    def fitting(*args, **kwargs):
        cancelled.set()
        return {"candidate_config": {"must_not_be_saved": True}, "recommended_nodes": ["1"]}

    monkeypatch.setattr(checker.input_unit_fit, "fit_input_units", fitting)
    report = run_input_scale_check(samples, config, holds, tmp_path / "cancelled", cancelled=cancelled.is_set)
    assert report["status"] == "cancelled" and report["recommended_nodes"] == []
    assert report["candidate_path"] is None and not report["candidate_available"]
    assert not list((tmp_path / "cancelled").glob("config.candidate*"))


def test_cancel_after_candidate_staging_removes_it(tmp_path, monkeypatch):
    samples, config, holds = synthetic(gains={1: 2.}, nodes=(1,))
    cancelled = Event()
    real_write = checker.write_json

    def write(path, payload):
        real_write(path, payload)
        if path.name == "config.candidate.pending.json":
            cancelled.set()

    monkeypatch.setattr(checker, "write_json", write)
    report = run_input_scale_check(samples, config, holds, tmp_path / "cancelled", cancelled=cancelled.is_set)
    assert report["status"] == "cancelled" and not report["candidate_available"]
    assert not list((tmp_path / "cancelled").glob("config.candidate*"))
    assert json.loads((tmp_path / "cancelled" / "report.json").read_text())["status"] == "cancelled"


def test_summary_is_detached_from_report(tmp_path):
    _, config, holds = synthetic(nodes=())
    report = run_input_scale_check((), config, holds, tmp_path / "empty")
    original = copy.deepcopy(report)
    summary = scale_check_summary(report)
    summary["nodes"]["1"]["reason_codes"].append("mutated")
    summary["inconclusive_nodes"].clear()
    assert report == original
