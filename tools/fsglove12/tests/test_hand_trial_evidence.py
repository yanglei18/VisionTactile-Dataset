"""Active hashes, producer identities and replay scope survive read-only capture."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from vt_fsglove12.config import digest
from vt_fsglove12.hand_trial_evidence import collect_hand_evidence


@pytest.fixture
def active(tmp_path):
    config, rest, profile = {"hand": "left"}, {"reference": [1, 2]}, {"profile_version": 3, "values": [0.1]}
    for name, document in (("config", config), ("rest", rest), ("profile.v003", profile)):
        (tmp_path / f"{name}.json").write_text(json.dumps(document))
    snapshot = dict(runtime=dict(connected=True, session_id="session", output_dir=str(tmp_path), ros_export=dict(sent=20)),
        calibration=dict(profile_version=3, config_sha256=digest(config), rest_sha256=digest(rest),
            profile_sha256=digest(profile), profile_artifact_path=str(tmp_path / "profile.v003.json")),
        pose=dict(profile_version=3), input_snapshot=dict(runtime=dict(connected=True, session_id="session",
            received_samples=30, published_monotonic_s=1., ros_raw_export=dict(sent_samples=30, dropped_samples=0)),
            orientation=dict(session_id="session", config_sha256=digest(config), input_kind="raw_serial",
                nodes={"9": dict(sample_index=10, epoch=0, host_monotonic_ns=123, host_realtime_ns=456)})))
    return snapshot


def collect(active, **options):
    return collect_hand_evidence(active, hand="left", service_instance_id="service", software={"source_sha256":"code"}, **options)


def test_originals_match_running_child_and_keep_asynchronous_replay_scope(active):
    evidence = collect(active)
    assert evidence["status"] == "verified_snapshot"
    assert evidence["source_mode"] == "live" and evidence["input_kind"] == "raw_serial"
    assert set(evidence["artifacts"]) == {"config", "rest", "profile"}
    assert evidence["producer_counters"]["received_samples"] == 30
    assert evidence["source_indices"]["9"]["filter_epoch"] == 0
    assert evidence["boundary_exact"] is False
    assert evidence["producer_observed_monotonic_ns"] == 1_000_000_000
    assert evidence["service_observed_monotonic_ns"] > 0
    assert evidence["replay_scope"]["exact_raw_replay_from_trial_start"] is False
    evidence["artifacts"]["profile"]["document"]["values"].append(100)
    assert collect(active)["artifacts"]["profile"]["document"]["values"] == [0.1]


@pytest.mark.parametrize("failure", ["artifact", "session", "config", "profile", "source", "sensor", "symlink"])
def test_missing_mismatched_or_foreign_evidence_is_never_certified(active, failure, tmp_path):
    if failure == "artifact":
        (tmp_path / "profile.v003.json").write_text('{"profile_version":3,"changed":true}')
    elif failure == "session":
        active["input_snapshot"]["runtime"]["session_id"] = "other"
    elif failure == "config":
        active["input_snapshot"]["orientation"]["config_sha256"] = "wrong"
    elif failure == "profile":
        active["pose"]["profile_version"] = 4
    elif failure == "source":
        active["input_snapshot"]["orientation"]["input_kind"] = "unknown"
    elif failure == "sensor":
        active["input_snapshot"]["orientation"]["nodes"]["9"].pop("sample_index")
    else:
        (tmp_path / "profile.v003.json").rename(tmp_path / "original.json")
        (tmp_path / "profile.v003.json").symlink_to(tmp_path / "original.json")
    evidence = collect(active)
    assert evidence["status"] == "unavailable" and evidence["problems"]


def test_light_observation_never_reads_files_and_replay_mode_is_canonical(active, monkeypatch):
    active["input_snapshot"]["orientation"]["input_kind"] = "csv_replay"
    import vt_fsglove12.hand_trial_evidence as module
    monkeypatch.setattr(module, "_artifact", lambda *a: pytest.fail("No files in a lightweight snapshot"))
    evidence = collect(active, include_artifacts=False)
    assert evidence["status"] == "verified_snapshot" and evidence["artifacts"] == {}
    assert evidence["source_mode"] == "replay"
