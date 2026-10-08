"""Recorded online state must never be replaced by a fresh offline observer."""
import copy
import json

import numpy as np
import pytest

from vt_glove_pose.hinge_online import online_hinge_report
from vt_glove_pose.hinge_validation import analyze_hinge_validation
from vt_glove_pose.pair_capture import load_pair_capture
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import RawRecorder


SETTINGS = dict(node_ids=[5, 6], sample_rate_hz=20., gyro_dps_per_lsb=.00875, accel_g_per_lsb=.000122)


def inputs(n=60):
    t = 100. + np.arange(n) / SETTINGS["sample_rate_hz"]
    gp = np.column_stack((np.arange(n), np.zeros((n, 2), int))).astype(np.int16)
    gd = gp + 10
    acc = np.tile([0, 0, 8197], (n, 1)).astype(np.int16)
    data = dict(prox_gyro_raw=gp, dist_gyro_raw=gd, prox_acc_raw=acc, dist_acc_raw=acc.copy(),
                host_monotonic_s=np.repeat(t[:, None], 2, axis=1),
                host_wall_s=np.repeat((t + 1000)[:, None], 2, axis=1),
                metadata=dict(source="test.csv", proximal=5, distal=6, sample_rate_hz=20.,
                              gyro_dps_per_lsb=.00875, dropped_pair_candidates=0, stop_reason="eof"))
    trace = dict(kind="vt_glove_pose.hinge_online_trace", schema_version=1,
                 settings=copy.deepcopy(SETTINGS), discarded_rows=0, series=[])
    for i in range(n):
        trace["series"].append(dict(pair_sequence=i + 901, connection_id=2,
            host_monotonic_s=float(t[i]), host_wall_s=float(t[i] + 1000),
            proximal_raw=np.r_[gp[i], acc[i]].tolist(), distal_raw=np.r_[gd[i], acc[i]].tolist(),
            angle_deg=40. + i / 10, valid=True, current_angle_observed=True,
            quality="tracking", history_loss=False, relative_bias_dps=2.4,
            angle_std_deg=.7, raw_near_limit=False, host_gap_before=False,
            reason_codes=[], observation_sources=["gravity"]))
    return trace, data


def test_actual_angles_bias_and_nonzero_origin_are_preserved_without_an_observer(monkeypatch):
    from vt_glove_pose.hinge_observer import ConstrainedHingeObserver
    from vt_glove_pose.thumb_flexion import ThumbFlexionEstimator
    def forbidden(*args, **kwargs):
        raise AssertionError("report must never initialize or update an observer")
    monkeypatch.setattr(ConstrainedHingeObserver, "__init__", forbidden)
    monkeypatch.setattr(ThumbFlexionEstimator, "__init__", forbidden)
    trace, data = inputs()
    report = online_hinge_report(trace, data, SETTINGS)
    assert report["replay_status"] == "completed"
    assert [r["angle_deg"] for r in report["series"]] == [r["angle_deg"] for r in trace["series"]]
    assert report["series"][0]["host_elapsed_s"] == 0
    assert report["series"][0]["pair_sequence"] == 901
    assert report["summary"]["final_relative_bias_dps"] == 2.4
    assert report["summary"]["final_model_angle_std_deg"] == .7
    report["series"][0]["reason_codes"].append("edited_copy")
    assert trace["series"][0]["reason_codes"] == []
    json.dumps(report, allow_nan=False)


def test_one_usb_timestamp_does_not_merge_distinct_pairs():
    trace, data = inputs(4)
    data["host_monotonic_s"][:] = 100
    data["host_wall_s"][:] = 1100
    for row in trace["series"]:
        row.update(host_monotonic_s=100., host_wall_s=1100.)
    report = online_hinge_report(trace, data, SETTINGS)
    assert report["summary"]["matched_online_rows"] == 4
    assert [r["angle_deg"] for r in report["series"]] == [40., 40.1, 40.2, 40.3]
    assert [r["host_elapsed_s"] for r in report["series"]] == [0.] * 4


def test_missing_output_keeps_raw_position_and_only_its_window_is_insufficient():
    trace, data = inputs()
    del trace["series"][25]
    report = online_hinge_report(trace, data, SETTINGS)
    assert len(report["series"]) == 60
    missing = report["series"][25]
    assert missing["angle_deg"] is None and not missing["valid"] and not missing["current_angle_observed"]
    assert missing["proximal_raw"][0] == 25
    assert report["series"][26]["angle_deg"] == 42.6
    windows = [dict(kind="common_rotation", pose_id="p", start_s=0., end_s=1.),
               dict(kind="common_rotation", pose_id="p", start_s=1., end_s=2.)]
    analysis = analyze_hinge_validation(report, windows)
    assert analysis["windows"][0]["data_sufficient"]
    assert not analysis["windows"][1]["data_sufficient"]
    assert "missing_or_nonfinite_angles" in analysis["windows"][1]["reasons"]


def test_uninitialized_output_is_real_row_not_a_zero():
    trace, data = inputs(2)
    trace["series"][0].update(angle_deg=None, valid=False, current_angle_observed=False,
                               quality="uninitialized", relative_bias_dps=None, angle_std_deg=None)
    result = online_hinge_report(trace, data, SETTINGS)
    assert result["summary"]["matched_online_rows"] == 2
    assert result["series"][0]["angle_deg"] is None
    assert result["summary"]["angle_range_deg"] == [40.1, 40.1]


@pytest.mark.parametrize("where", ["trace", "capture", "source_configuration"])
def test_settings_disagreement_is_rejected(where):
    trace, data = inputs(2)
    if where == "trace":
        trace["settings"]["sample_rate_hz"] = 30.
    elif where == "capture":
        data["metadata"]["sample_rate_hz"] = 30.
    else:
        data["metadata"]["source_configuration"] = dict(SETTINGS, accel_g_per_lsb=.002)
    with pytest.raises(ValueError, match="differ"):
        online_hinge_report(trace, data, SETTINGS)


def test_payload_mismatch_is_counted_and_never_silently_recomputed():
    trace, data = inputs(4)
    trace["series"][1]["distal_raw"][0] += 1
    result = online_hinge_report(trace, data, SETTINGS)
    assert result["summary"]["unmatched_online_rows"] == 1
    assert result["summary"]["missing_online_rows"] == 1
    assert result["series"][1]["angle_deg"] is None
    assert result["series"][2]["angle_deg"] == 40.2
    assert result["replay_status"] == "incomplete_online_trace"


def test_unknown_discarded_location_blocks_all_windows_without_hiding_angles():
    trace, data = inputs()
    trace["discarded_rows"] = 1
    result = online_hinge_report(trace, data, SETTINGS)
    assert result["replay_status"] == "incomplete_online_trace"
    analysis = analyze_hinge_validation(result, [dict(kind="common_rotation", pose_id="p", start_s=0., end_s=1.)])
    assert analysis["windows"][0]["angle"]["median_deg"] > 40
    assert "replay_not_completed" in analysis["windows"][0]["reasons"]


def test_max_pairs_prefix_ignores_only_tail_and_reports_truncation():
    trace, data = inputs(5)
    for name, value in list(data.items()):
        if isinstance(value, np.ndarray):
            data[name] = value[:3].copy()
    data["metadata"]["stop_reason"] = "max_pairs"
    result = online_hinge_report(trace, data, SETTINGS)
    assert result["replay_status"] == "completed"
    assert result["summary"]["capture_truncated"]
    assert result["summary"]["ignored_trace_rows_after_capture"] == 2
    assert result["summary"]["unmatched_online_rows"] == 0


@pytest.mark.parametrize("bad_sequence", [0, 900, True])
def test_bad_or_nonincreasing_pair_sequence_is_rejected(bad_sequence):
    trace, data = inputs(2)
    trace["series"][1]["pair_sequence"] = bad_sequence
    with pytest.raises(ValueError, match="pair_sequence"):
        online_hinge_report(trace, data, SETTINGS)


def test_identical_payload_and_timestamp_missing_row_has_unknown_gap_location():
    trace, data = inputs(3)
    for value in data.values():
        if isinstance(value, np.ndarray):
            value[:] = value[0]
    for row in trace["series"]:
        for name in ("host_monotonic_s", "host_wall_s", "proximal_raw", "distal_raw"):
            row[name] = copy.deepcopy(trace["series"][0][name])
    del trace["series"][1]
    result = online_hinge_report(trace, data, SETTINGS)
    assert result["replay_status"] == "incomplete_online_trace"
    assert "identical_payload_missing_output_position_ambiguous" in result["reason_codes"]
    assert result["summary"]["missing_online_rows"] == 1


def test_empty_capture_produces_json_safe_empty_report():
    trace, data = inputs(0)
    result = online_hinge_report(trace, data, SETTINGS)
    assert result["replay_status"] == "empty_capture"
    assert result["series"] == []
    assert result["summary"]["final_angle_deg"] is None
    json.dumps(result, allow_nan=False)


def test_real_csv_pair_reader_keeps_same_timestamp_wire_order(tmp_path):
    trace, _ = inputs(4)
    path = tmp_path / "paired.csv"
    samples = []
    for row in trace["series"]:
        row.update(host_monotonic_s=12.5, host_wall_s=1200.)
        samples.extend([Sample(5, tuple(row["proximal_raw"]), 12.5, 1200.),
                        Sample(6, tuple(row["distal_raw"]), 12.5, 1200.)])
    with RawRecorder(path) as recorder:
        recorder.write(samples)
    capture = load_pair_capture(path, proximal=5, distal=6, sample_rate_hz=20.,
                                gyro_dps_per_lsb=.00875)
    result = online_hinge_report(trace, capture, SETTINGS)
    assert result["replay_status"] == "completed"
    assert result["summary"]["matched_online_rows"] == 4
    assert [row["angle_deg"] for row in result["series"]] == [40., 40.1, 40.2, 40.3]
