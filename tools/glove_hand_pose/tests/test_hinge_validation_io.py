"""Exported raw CSV + frozen profile + declared windows, with no hardware."""
import json

import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.hinge_export import export_hinge_session
from vt_glove_pose.hinge_validation_io import validate_hinge_protocol
from vt_glove_pose.records import Sample


def simple_profile():
    # A fixed ideal coordinate fixture for I/O tests, not a solved real glove.
    identity = [[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]
    return dict(kind="vt_glove_pose.offline_hinge_profile", schema_version=1,
        offline_only=True, usable_as_live_calibration=False, profile_status="offline_candidate_ready",
        settings=dict(node_ids=[5,6], sample_rate_hz=30., gyro_dps_per_lsb=.00875, accel_g_per_lsb=.000122),
        axis_fit=dict(candidate_status="candidate_supported"),
        frames=dict(status="frames_ready", prox_sensor_to_functional=identity, dist_sensor_to_functional=identity),
        neutral=dict(prox_bias_dps=[0.,0.,0.], dist_bias_dps=[0.,0.,0.],
                     prox_acc_g=[0.,0.,1.], dist_acc_g=[0.,0.,1.]))


def make_protocol(tmp_path, *, delayed_pair=False, online=False, discarded_rows=0):
    samples = [Sample(1, (0,0,0,0,0,8197), 10., 1010.)] if delayed_pair else []
    rows = []
    for index in range(360):
        t = 10.+(.5 if delayed_pair else 0.)+index/30.
        samples.extend(Sample(node, (0,0,0,0,0,8197), t, 1000.+t) for node in range(1,7))
        rows.append(dict(pair_sequence=index+901, connection_id=2,
            host_monotonic_s=t, host_wall_s=1000.+t,
            proximal_raw=[0,0,0,0,0,8197], distal_raw=[0,0,0,0,0,8197],
            angle_deg=37., relative_bias_dps=1.23, angle_std_deg=.4, relative_bias_std_dps=.1,
            valid=True, current_angle_observed=True, history_loss=False,
            raw_near_limit=False, host_gap_before=False, quality="tracking"))
    markers = [dict(kind=kind, pose_id="reference_pose", status="completed", window_id=i+1,
                    start_host_monotonic_s=start, end_host_monotonic_s=end)
               for i,(kind,start,end) in enumerate((("reference",11.,13.),("return",16.,18.)))]
    folder = tmp_path/"experiment"
    export_hinge_session(folder, {}, config=load_config(), validation_attempts=[dict(
        kind="validation", started_s=9.9, ended_s=22.5, duration_s=13., status="complete",
        samples=samples, series=[], markers=markers, profile=simple_profile(),
        **(dict(online=rows, online_discarded_rows=discarded_rows) if online else {}))])
    return folder/"001_validation_protocol.json"


def test_exported_protocol_replays_frozen_profile_without_accuracy_claims(tmp_path):
    path = make_protocol(tmp_path)
    result = validate_hinge_protocol(path)
    assert result["provenance"]["replay_status"] == "completed"
    assert result["provenance"]["exported_hash_verified"] == dict(capture=True, profile=True)
    assert result["provenance"]["evaluation_source"] == "replay"
    assert len(result["windows"]) == 2
    assert result["criteria"]["return_repeatability"]["status"] == "insufficient_data"
    assert result["absolute_accuracy"]["status"] == "not_evaluated"
    assert result["end_to_end_latency"]["status"] == "not_evaluated"
    assert result["production_acceptance"] == "not_evaluated"
    json.dumps(result, allow_nan=False)


def test_csv_origin_and_first_matched_pair_are_not_conflated(tmp_path, monkeypatch):
    from vt_glove_pose import hinge_validation
    path = make_protocol(tmp_path, delayed_pair=True)
    seen = []
    monkeypatch.setattr(hinge_validation, "analyze_hinge_validation", lambda replay, windows: seen.extend(windows) or {})
    result = validate_hinge_protocol(path)
    assert result["provenance"]["csv_first_to_first_pair_s"] == .5
    assert seen[0]["start_s"] == .5
    assert seen[0]["end_s"] == 2.5


def test_changed_raw_or_profile_is_not_silently_used(tmp_path):
    path = make_protocol(tmp_path)
    profile = path.parent/"001_validation_profile.json"
    profile.write_text(profile.read_text()+"\n", encoding="utf-8")
    with pytest.raises(ValueError, match="SHA256"):
        validate_hinge_protocol(path)


def test_protocol_reference_paths_and_time_basis_are_explicit(tmp_path):
    path = make_protocol(tmp_path)
    protocol = json.loads(path.read_text())
    protocol["capture_file"] = "../outside.csv"
    path.write_text(json.dumps(protocol), encoding="utf-8")
    with pytest.raises(ValueError, match="filenames"):
        validate_hinge_protocol(path)
    protocol["time_basis"] = "device_time"
    path.write_text(json.dumps(protocol), encoding="utf-8")
    with pytest.raises(ValueError, match="time basis"):
        validate_hinge_protocol(path)


def test_sample_limit_does_not_turn_missing_windows_into_success(tmp_path):
    path = make_protocol(tmp_path)
    result = validate_hinge_protocol(path, max_pairs=30)
    assert result["criteria"]["return_repeatability"]["status"] == "insufficient_data"
    assert result["provenance"]["replay_summary"]["samples"] == 30


def test_cli_json_and_html_are_exclusive_and_no_device_is_opened(tmp_path, monkeypatch):
    from vt_glove_pose.cli import main
    from vt_glove_pose.stream import SerialSource
    def forbidden(*args, **kwargs):
        raise AssertionError("offline command must not open hardware")
    monkeypatch.setattr(SerialSource, "open", forbidden)
    path = make_protocol(tmp_path)
    output, html = tmp_path/"result.json", tmp_path/"result.html"
    args = ["validate-hinge", str(path), "--output", str(output), "--html", str(html)]
    assert main(args) == 0
    assert json.loads(output.read_text())["production_acceptance"] == "not_evaluated"
    assert "<!doctype html" in html.read_text().lower()
    before = output.read_bytes()
    assert main(args) == 2
    assert output.read_bytes() == before


def test_online_auto_keeps_recorded_nonzero_angle_and_bias_without_rerunning(tmp_path, monkeypatch):
    from vt_glove_pose import hinge_validation_io
    from vt_glove_pose.hinge_validation import render_validation_html
    path = make_protocol(tmp_path, online=True)
    def forbidden(*args, **kwargs):
        raise AssertionError("auto with online trace must not rerun the estimator")
    monkeypatch.setattr(hinge_validation_io, "replay_hinge_pair", forbidden)
    result = validate_hinge_protocol(path)
    provenance = result["provenance"]
    assert provenance["evaluation_source"] == "online"
    assert provenance["exported_hash_verified"] == dict(capture=True, profile=True, online=True)
    assert provenance["replay_summary"]["matched_online_rows"] == 360
    assert provenance["replay_summary"]["final_relative_bias_dps"] == 1.23
    assert result["windows"][0]["angle"]["median_deg"] == 37.
    assert "实际在线逐配对输出" in render_validation_html(result)
    assert result["production_acceptance"] == "not_evaluated"


def test_replay_is_explicit_for_an_alternative_observer_on_online_protocol(tmp_path):
    path = make_protocol(tmp_path, online=True)
    with pytest.raises(ValueError, match="--source replay"):
        validate_hinge_protocol(path, observer="baseline")
    result = validate_hinge_protocol(path, source="replay", observer="baseline")
    assert result["provenance"]["evaluation_source"] == "replay"
    assert result["windows"][0]["angle"]["median_deg"] == pytest.approx(0.)


def test_online_source_never_silently_falls_back_when_trace_is_missing(tmp_path):
    path = make_protocol(tmp_path)
    with pytest.raises(ValueError, match="no full-rate online output"):
        validate_hinge_protocol(path, source="online")
    protocol = json.loads(path.read_text())
    protocol["online_file"] = "missing_online.json"
    path.write_text(json.dumps(protocol), encoding="utf-8")
    with pytest.raises(FileNotFoundError):
        validate_hinge_protocol(path)


def test_online_trace_change_and_wrong_csv_binding_are_rejected(tmp_path):
    from vt_glove_pose.hinge_export import file_sha256
    path = make_protocol(tmp_path, online=True)
    protocol = json.loads(path.read_text())
    trace_path = path.parent/protocol["online_file"]
    trace = json.loads(trace_path.read_text())
    trace["capture_sha256"] = "wrong"
    trace_path.write_text(json.dumps(trace), encoding="utf-8")
    with pytest.raises(ValueError, match="online trace SHA256"):
        validate_hinge_protocol(path)
    protocol["online_sha256"] = file_sha256(trace_path)
    path.write_text(json.dumps(protocol), encoding="utf-8")
    with pytest.raises(ValueError, match="not bound"):
        validate_hinge_protocol(path)


def test_online_unknown_discarded_rows_cannot_be_called_stable(tmp_path):
    path = make_protocol(tmp_path, online=True, discarded_rows=2)
    result = validate_hinge_protocol(path)
    assert result["provenance"]["replay_status"] == "incomplete_online_trace"
    assert all(not window["data_sufficient"] for window in result["windows"])


def test_online_prefix_limit_is_explicit_and_missing_windows_do_not_pass(tmp_path):
    path = make_protocol(tmp_path, online=True)
    result = validate_hinge_protocol(path, max_pairs=30)
    summary = result["provenance"]["replay_summary"]
    assert summary["samples"] == 30
    assert summary["capture_truncated"]
    assert summary["ignored_trace_rows_after_capture"] == 330
    assert all(not window["data_sufficient"] for window in result["windows"])
