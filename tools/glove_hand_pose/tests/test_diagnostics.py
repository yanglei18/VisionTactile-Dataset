"""Offline reproduction and hostile/interrupted log handling; no hardware."""
from copy import deepcopy
from dataclasses import asdict
import json

import numpy as np
import pytest

from vt_glove_pose.cli import main
from vt_glove_pose.config import load_config
from vt_glove_pose.diagnostics import diagnose_session, iter_session
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.records import Sample


def write_events(path, events):
    with path.open("x", encoding="utf-8") as stream:
        for sequence, event in enumerate(events):
            stream.write(json.dumps(dict(event, seq=sequence), allow_nan=False) + "\n")


def session_events(*, hand="right", calibrated=False, algorithm_mode="thumb_paired_gyro_gravity_v1"):
    config = load_config()
    config["hand"] = hand
    engine = PoseEngine(config, algorithm_mode=algorithm_mode)
    for tick in range(20):
        for node in range(1, 13):
            engine.process(Sample(node, (0, 0, 0, 0, 0, 8197), 10 + tick / 120, 1000 + tick / 120))
    if calibrated:
        engine.neutral_ready = True
        engine.mount_confirmed = True
        engine.zero = {n: s["quat"].copy() for n, s in engine.states.items()}
        engine.mounts = {n: np.array([1., 0., 0., 0.]) for n in range(1, 13)}
        engine.axis_sign_versions = {n: 2 for n in range(1, 13)}
        engine.neutral_acc = {n: np.array([0., 0., 1.]) for n in range(1, 13)}
    events = [dict(type="header", kind="vt_glove_pose.diagnostic_session", schema_version=1,
                   time_basis="host_observed_not_sensor_latency", checkpoint=engine.export_diagnostic_checkpoint())]
    for tick in range(20, 25):
        # Deliberately repeat per-node parse times across two batches. Order,
        # not unique parse time, is the integration clock.
        t = 10 + (tick // 2 * 2) / 120
        samples = [Sample(n, (20 * n, 0, 0, 0, 0, 8197), t, 1000 + t) for n in range(1, 13)]
        for sample in samples:
            engine.process(sample)
        events.append(dict(type="batch", samples=[asdict(s) for s in samples],
                           performance={"serial_backlog_before_bytes": tick * 2, "fusion_elapsed_ms": 2.}))
        events.append(dict(type="snapshot", now_s=t, snapshot=engine.get_snapshot(now=t), performance={}))
    events += [dict(type="marker", label="returned"), dict(type="end", reason="manual", complete=True)]
    return events, engine


@pytest.mark.parametrize("hand", ["left", "right"])
@pytest.mark.parametrize("calibrated", [False, True])
def test_midstream_checkpoint_ordered_reproduction(tmp_path, hand, calibrated):
    events, _ = session_events(hand=hand, calibrated=calibrated)
    path = tmp_path / "session.jsonl"
    write_events(path, events)
    report = diagnose_session(path)
    assert report["session_complete"]
    assert report["reconstruction"]["reproducible"]
    assert report["reconstruction"]["max_numeric_absolute_error"] < 1e-10
    assert report["total_records"] == 60
    assert report["nodes"]["1"]["equal_host_timestamp_records"] == 2
    assert report["host_metric_peaks"]["serial_backlog_before_bytes"] == 48
    assert report["markers"] == [{"seq": 11, "label": "returned", "after_sample": 60}]
    assert report["recorded_pose_statistics"]["index"]["valid_snapshots"] == (5 if calibrated else 0)


def test_command_and_timeout_snapshot_are_replayed_in_order(tmp_path):
    events, engine = session_events()
    events = events[:-2]
    engine.begin_neutral(duration=5, confirm_mount=True)
    events.append(dict(type="command", name="neutral", args=dict(duration=5, confirm_mount=True)))
    # get_snapshot cancels calibration when a source pauses; preserve its time.
    now = engine.last_time + 2
    snapshot = engine.get_snapshot(now=now)
    assert snapshot["calibration"]["active"] is None
    events.append(dict(type="snapshot", now_s=now, snapshot=snapshot))
    engine.cancel_calibration()
    events.append(dict(type="command", name="cancel", args={}))
    events.append(dict(type="checkpoint", checkpoint=engine.export_diagnostic_checkpoint()))
    events.append(dict(type="snapshot", now_s=now, snapshot=engine.get_snapshot(now=now)))
    events.append(dict(type="end", complete=True, reason="manual"))
    path = tmp_path / "commands.jsonl"
    write_events(path, events)
    report = diagnose_session(path)
    assert report["reconstruction"]["status"] == "match"
    assert report["counts"]["commands"] == 2
    assert report["counts"]["checkpoints"] == 1


def test_altered_pose_is_not_passing_and_reports_clipping(tmp_path):
    events, _ = session_events(calibrated=True)
    finger = events[2]["snapshot"]["model"]["fingers"]["index"]
    finger["mcp_deg"] = 99.
    finger["raw_angles_deg"]["mcp_deg"] = 220.
    finger["clipped_angles"] = ["mcp_deg"]
    path = tmp_path / "altered.jsonl"
    write_events(path, events)
    report = diagnose_session(path)
    assert report["session_complete"]
    assert report["reconstruction"]["status"] == "mismatch"
    assert not report["reconstruction"]["reproducible"]
    assert report["recorded_pose_statistics"]["index"]["clipped_snapshots"] == 1
    assert report["recorded_pose_statistics"]["index"]["raw_angle_range_deg"]["mcp_deg"][1] == 220.
    assert main(["diagnose-session", str(path), "--output", str(tmp_path / "report.json")]) == 1


def test_save_message_is_not_a_pose_difference(tmp_path):
    events, _ = session_events()
    events[2]["snapshot"]["calibration"]["message"] = "Saved to a private local path"
    path = tmp_path / "message.jsonl"
    write_events(path, events)
    assert diagnose_session(path)["reconstruction"]["status"] == "match"


@pytest.mark.parametrize("change", ["no_footer", "false_footer", "after_footer", "unknown", "bad_sample", "bad_command"])
def test_incomplete_and_malformed_sessions_never_pass(tmp_path, change):
    events, _ = session_events()
    if change == "no_footer":
        events.pop()
    elif change == "false_footer":
        events[-1]["complete"] = False
    elif change == "after_footer":
        events.append(dict(type="marker", label="rest"))
    elif change == "unknown":
        events[-1] = dict(type="execute", path="/does/not/run")
    elif change == "bad_sample":
        events[3]["samples"][0]["node_id"] = 13
    elif change == "bad_command":
        events[-1] = dict(type="command", name="load", args={"path": "/does/not/read"})
    path = tmp_path / "broken.jsonl"
    write_events(path, events)
    report = diagnose_session(path)
    assert not report["session_complete"]
    assert not report["reconstruction"]["reproducible"]
    assert report["total_records"] >= 12


def test_version_mismatch_preserves_raw_statistics_and_raw_only_status(tmp_path):
    events, _ = session_events()
    events[0]["checkpoint"]["vqf_version"] = "not-installed"
    path = tmp_path / "version.jsonl"
    write_events(path, events)
    report = diagnose_session(path)
    assert report["session_complete"]
    assert report["total_records"] == 60
    assert report["reconstruction"]["status"] == "unverified"
    assert report["reconstruction"]["blocked_reason"]
    output = tmp_path / "raw.json"
    assert main(["diagnose-session", str(path), "--raw-only", "--output", str(output)]) == 0
    assert json.loads(output.read_text())["reconstruction"]["status"] == "unverified"
    original = output.read_bytes()
    assert main(["diagnose-session", str(path), "--raw-only", "--output", str(output)]) == 2
    assert output.read_bytes() == original


@pytest.mark.parametrize("line", [
    '{"seq":0,"seq":0,"type":"header"}\n',
    '{"seq":0,"type":"header","bad":NaN}\n',
    '{"seq":0,"type":"header","bad":1e999}\n',
    '{"seq":1,"type":"header"}\n',
    '{"seq":true,"type":"header"}\n',
    '{"seq":0,"type":"header"}',
    '{"seq":0,"type":"header","a":' + '[' * 50 + '0' + ']' * 50 + '}\n',
])
def test_reader_rejects_nonfinite_duplicate_keys_sequences_and_truncation(tmp_path, line):
    path = tmp_path / "bad.jsonl"
    path.write_text(line)
    with pytest.raises(ValueError):
        list(iter_session(path))
    assert main(["diagnose-session", str(path)]) == 2


def test_truncated_tail_and_oversized_line_return_prefix(tmp_path, monkeypatch):
    events, _ = session_events()
    path = tmp_path / "prefix.jsonl"
    write_events(path, events[:-1])
    with path.open("a") as stream:
        stream.write('{"seq":12,"type":"end"')
    report = diagnose_session(path)
    assert not report["session_complete"]
    assert report["total_records"] == 60
    assert "truncated" in report["format_error"]
    monkeypatch.setattr("vt_glove_pose.diagnostics.MAX_LINE_BYTES", 10)
    with pytest.raises(ValueError, match="exceeds"):
        list(iter_session(path))


def test_empty_data_is_not_reproduction_success(tmp_path):
    events, _ = session_events()
    path = tmp_path / "empty_data.jsonl"
    write_events(path, [events[0], events[-1]])
    report = diagnose_session(path)
    assert report["session_complete"]
    assert report["reconstruction"]["status"] == "unverified"
    assert not report["reconstruction"]["reproducible"]


@pytest.mark.parametrize("mode,schema", [("legacy_quaternion_v1", 1), ("thumb_paired_gyro_gravity_v1", 2)])
def test_report_explicitly_identifies_the_replayed_algorithm(tmp_path, mode, schema):
    events, _ = session_events(calibrated=True, algorithm_mode=mode)
    path = tmp_path / "versioned.jsonl"
    write_events(path, events)
    result = diagnose_session(path)["reconstruction"]
    assert result["status"] == "match"
    assert result["algorithm_mode"] == mode
    assert result["checkpoint_schema_version"] == schema


def test_session_cannot_silently_switch_pose_algorithm(tmp_path):
    events, engine = session_events(calibrated=True)
    events = events[:-2]
    switched = engine.export_diagnostic_checkpoint()
    switched["schema_version"] = 1
    switched.pop("pose_algorithm")
    for key in ("thumb_estimator", "thumb_pair_counts", "thumb_common_capture"):
        switched["engine"].pop(key)
    events += [dict(type="checkpoint", checkpoint=switched), dict(type="end", complete=True, reason="manual")]
    path = tmp_path / "switched.jsonl"
    write_events(path, events)
    report = diagnose_session(path)
    assert not report["reconstruction"]["reproducible"]
    assert "changed pose algorithm/schema" in report["format_error"]


def test_joint_a_command_and_cancellation_replay_without_hardware(tmp_path):
    events, engine = session_events(calibrated=True)
    events = events[:-2]
    engine.begin_axis("thumb_common", duration=8)
    events += [dict(type="command", name="axis", args=dict(target="thumb_common", duration=8)),
               dict(type="snapshot", now_s=engine.last_time, snapshot=engine.get_snapshot(now=engine.last_time))]
    engine.cancel_calibration()
    events += [dict(type="command", name="cancel", args={}),
               dict(type="snapshot", now_s=engine.last_time, snapshot=engine.get_snapshot(now=engine.last_time)),
               dict(type="end", reason="manual", complete=True)]
    path = tmp_path / "joint_command.jsonl"
    write_events(path, events)
    report = diagnose_session(path)
    assert report["reconstruction"]["status"] == "match"
    assert report["counts"]["commands"] == 2
