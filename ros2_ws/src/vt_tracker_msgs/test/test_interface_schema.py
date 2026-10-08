from pathlib import Path


ROOT = Path(__file__).parents[1]


def fields(name: str) -> list[str]:
    return [
        line.split()[-1]
        for line in (ROOT / "msg" / name).read_text().splitlines()
        if line and not line.startswith("#") and "=" not in line
    ]


def test_tracker_sample_schema_is_stable():
    assert fields("TrackerSample.msg") == [
        "header",
        "role",
        "tracker_id",
        "host_monotonic_ns",
        "host_realtime_ns",
        "packet_index",
        "tracking_status",
        "raw_buttons",
        "pose_valid",
        "pose",
        "linear_acceleration",
        "angular_velocity",
    ]


def test_tracker_status_schema_is_stable():
    assert fields("TrackerStatus.msg") == [
        "header",
        "role",
        "tracker_id",
        "state",
        "tracking_status",
        "valid_sample_count",
        "invalid_report_count",
        "dropped_queue_count",
        "last_report_monotonic_ns",
        "last_valid_pose_monotonic_ns",
    ]
