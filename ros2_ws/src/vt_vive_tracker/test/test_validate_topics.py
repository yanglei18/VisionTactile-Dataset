from __future__ import annotations

import json
from pathlib import Path
import stat
from types import SimpleNamespace

import pytest
import rclpy

from vt_vive_tracker.validate_topics import (
    TopicValidationSession,
    _ValidationNode,
    _validate_output_path,
    _write_private_report,
)


ROLES = ("left_wrist", "right_wrist", "torso")
IDS = {
    "left_wrist": "a" * 64,
    "right_wrist": "b" * 64,
    "torso": "c" * 64,
}


def header(realtime_ns):
    seconds, nanoseconds = divmod(realtime_ns, 1_000_000_000)
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds),
        frame_id="vive_map",
    )


def sample(
    role,
    index,
    *,
    tracker_id=None,
    monotonic_ns=None,
    realtime_ns=None,
    pose_valid=True,
):
    monotonic_ns = (
        index * 33_333_333 if monotonic_ns is None else monotonic_ns
    )
    realtime_ns = (
        1_700_000_000_000_000_000 + monotonic_ns
        if realtime_ns is None
        else realtime_ns
    )
    return SimpleNamespace(
        header=header(realtime_ns),
        role=role,
        tracker_id=IDS[role] if tracker_id is None else tracker_id,
        host_monotonic_ns=monotonic_ns,
        host_realtime_ns=realtime_ns,
        pose_valid=pose_valid,
    )


def status(role, *, tracker_id=None, dropped=0):
    return SimpleNamespace(
        role=role,
        tracker_id=IDS[role] if tracker_id is None else tracker_id,
        dropped_queue_count=dropped,
    )


def add_passing_role(session, role, count=900):
    for index in range(count):
        message = sample(role, index)
        session.observe_sample(role, message)
        if message.pose_valid:
            session.observe_pose(role, message.header)
    session.observe_status(role, status(role))


def passing_session():
    session = TopicValidationSession(duration_seconds=30.0)
    for role in ROLES:
        add_passing_role(session, role)
    return session


def test_validation_node_retains_all_nine_tracker_subscriptions():
    node = None
    rclpy.init()
    try:
        node = _ValidationNode(TopicValidationSession(duration_seconds=1.0))
        assert len(node._tracker_subscriptions) == 9
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_thirty_seconds_thirty_hz_small_gap_and_ninety_percent_valid_passes():
    session = TopicValidationSession(duration_seconds=30.0)
    for role in ROLES:
        for index in range(1000):
            message = sample(role, index, pose_valid=index % 10 != 0)
            session.observe_sample(role, message)
            if message.pose_valid:
                session.observe_pose(role, message.header)
        session.observe_status(role, status(role))

    report = session.report()

    assert report["verdict"] == "PASS"
    assert all(
        value["valid_rate_hz"] >= 30.0
        and value["valid_ratio"] >= 0.90
        and value["max_gap_ms"] <= 100.0
        for value in report["roles"].values()
    )


def test_missing_role_fails():
    session = TopicValidationSession(duration_seconds=30.0)
    add_passing_role(session, "left_wrist")
    add_passing_role(session, "right_wrist")

    assert session.report()["verdict"] == "FAIL"


def test_identity_change_on_one_role_fails():
    session = passing_session()
    session.observe_sample(
        "left_wrist",
        sample("left_wrist", 901, tracker_id="d" * 64),
    )

    report = session.report()
    assert report["verdict"] == "FAIL"
    assert report["identity_swap_count"] == 1


def test_duplicate_identity_across_roles_fails():
    session = TopicValidationSession(duration_seconds=30.0)
    for role in ROLES:
        for index in range(900):
            value = IDS["left_wrist"] if role == "right_wrist" else IDS[role]
            message = sample(role, index, tracker_id=value)
            session.observe_sample(role, message)
            session.observe_pose(role, message.header)
        session.observe_status(
            role,
            status(
                role,
                tracker_id=(
                    IDS["left_wrist"]
                    if role == "right_wrist"
                    else IDS[role]
                ),
            ),
        )

    report = session.report()
    assert report["verdict"] == "FAIL"
    assert report["identity_collision_count"] >= 1


def test_gap_over_one_hundred_ms_fails():
    session = passing_session()
    session.observe_sample(
        "torso",
        sample("torso", 901, monotonic_ns=40_000_000_000),
    )

    assert session.report()["verdict"] == "FAIL"
    assert session.report()["roles"]["torso"]["max_gap_ms"] > 100.0


def test_any_dropped_queue_count_fails():
    session = passing_session()
    session.observe_status("right_wrist", status("right_wrist", dropped=1))

    report = session.report()
    assert report["verdict"] == "FAIL"
    assert report["roles"]["right_wrist"]["dropped_queue_count"] == 1


def test_nonmonotonic_host_monotonic_time_fails():
    session = passing_session()
    session.observe_sample(
        "left_wrist",
        sample("left_wrist", 901, monotonic_ns=1),
    )

    assert session.report()["verdict"] == "FAIL"
    assert session.report()["roles"]["left_wrist"][
        "nonmonotonic_count"
    ] == 1


def test_header_and_realtime_mismatch_fails():
    session = passing_session()
    message = sample("torso", 901)
    message.header.stamp.nanosec += 1
    session.observe_sample("torso", message)

    assert session.report()["verdict"] == "FAIL"
    assert session.report()["roles"]["torso"][
        "timestamp_mismatch_count"
    ] == 1


def test_json_report_contains_only_redacted_metrics():
    report = passing_session().report()
    rendered = json.dumps(report, sort_keys=True)

    assert set(report) == {
        "verdict",
        "roles",
        "identity_swap_count",
        "identity_collision_count",
    }
    assert set(report["roles"]) == set(ROLES)
    assert all(
        set(value)
        == {
            "tracker_id",
            "sample_count",
            "valid_pose_count",
            "pose_message_count",
            "matched_pose_count",
            "status_count",
            "valid_rate_hz",
            "pose_rate_hz",
            "valid_ratio",
            "max_gap_ms",
            "dropped_queue_count",
            "nonmonotonic_count",
            "timestamp_mismatch_count",
        }
        for value in report["roles"].values()
    )
    assert all(IDS[role] in rendered for role in ROLES)
    assert "230142b782d3" not in rendered
    assert "/home/" not in rendered


def test_output_must_be_absolute_and_outside_git_worktree():
    with pytest.raises(ValueError, match="absolute"):
        _validate_output_path(Path("relative-report.json"))
    with pytest.raises(ValueError, match="outside"):
        _validate_output_path(
            Path(__file__).parents[1] / "report.json"
        )


def test_report_is_written_atomically_with_private_permissions(tmp_path):
    output = tmp_path / "report.json"
    report = passing_session().report()

    _write_private_report(output, report)

    assert json.loads(output.read_text(encoding="utf-8")) == report
    assert stat.S_IMODE(output.stat().st_mode) == 0o600
    assert not tuple(tmp_path.glob(".report.json.*"))
