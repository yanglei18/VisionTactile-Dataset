from pathlib import Path

ROOT = Path(__file__).parents[1]


def test_public_message_set_contains_only_implemented_capture_interfaces():
    assert {path.name for path in (ROOT / "msg").glob("*.msg")} == {
        "CameraDescriptor.msg",
        "CameraTrackerCalibration.msg",
        "CameraFrameTiming.msg",
        "CameraGroupStatus.msg",
        "CaptureCommand.msg",
        "CaptureContext.msg",
        "CaptureEvent.msg",
        "CaptureStatus.msg",
        "SessionInfo.msg",
        "StreamProfile.msg",
        "StreamStatus.msg",
    }


def fields(name: str) -> list[str]:
    lines = (ROOT / "msg" / name).read_text().splitlines()
    return [
        line.split()[-1]
        for line in lines
        if line and not line.startswith("#") and "=" not in line
    ]


def test_camera_frame_timing_has_one_group_and_two_raw_stream_audits():
    assert fields("CameraFrameTiming.msg") == [
        "header", "camera_name", "camera_model", "serial_number",
        "shared_ros_timestamp_ns",
        "color_frame_number", "depth_frame_number",
        "color_timestamp_domain", "depth_timestamp_domain",
        "color_device_timestamp_ns", "depth_device_timestamp_ns",
        "color_sensor_timestamp_ns", "depth_sensor_timestamp_ns",
        "color_backend_timestamp_ns", "depth_backend_timestamp_ns",
        "color_host_monotonic_raw_ns", "depth_host_monotonic_raw_ns",
        "color_host_realtime_ns", "depth_host_realtime_ns",
        "group_host_monotonic_raw_ns", "group_host_realtime_ns",
        "host_callback_spread_ns",
        "color_validity_flags", "depth_validity_flags",
        "group_validity_flags",
    ]


def test_capture_status_keeps_streams_and_adds_three_camera_groups():
    assert fields("CameraGroupStatus.msg") == [
        "camera_name", "rgb_frames", "depth_frames", "timing_frames",
        "complete_groups", "incomplete_groups", "duplicate_keys",
        "fps", "max_gap_ms", "complete_group_coverage", "alive",
    ]
    assert fields("CaptureStatus.msg")[-6:] == [
        "streams", "camera_groups", "recorder_alive", "disk_free_bytes",
        "observed_write_mb_s", "detail",
    ]


def test_managed_capture_interfaces_are_typed_and_bound_to_controller_session():
    assert fields("CaptureContext.msg") == [
        "controller_instance_id", "state_revision", "state", "session_id",
        "start_request_id", "session_label", "output_path", "recorder_alive",
        "elapsed_sec", "planned_duration_sec", "topics", "max_duration_sec", "detail",
    ]
    assert {p.name for p in (ROOT / "srv").glob("*.srv")} == {
        "GetCaptureSnapshot.srv", "ManagedCaptureCommand.srv",
    }
    request, response = (ROOT / "srv/ManagedCaptureCommand.srv").read_text().split("---")
    assert "uint8 START=1" in request and "uint8 STOP=2" in request
    assert [line.split()[-1] for line in request.splitlines() if line and "=" not in line] == [
        "request_id", "command", "expected_controller_instance_id", "expected_session_id",
        "expected_state_revision", "session_label", "planned_duration_sec",
    ]
    assert [line.split()[-1] for line in response.splitlines() if line] == [
        "accepted", "code", "detail", "context",
    ]


def test_camera_tracker_calibration_binds_transform_to_identity_and_revision():
    assert fields("CameraTrackerCalibration.msg") == [
        "header", "schema_version", "manager_instance_id", "camera_name", "camera_model", "camera_serial", "camera_frame_id",
        "tracker_role", "tracker_id", "world_frame_id", "tracker_from_camera", "revision",
        "sha256", "activated_ns", "valid", "valid_until_ns", "reason", "quality_grade",
    ]
