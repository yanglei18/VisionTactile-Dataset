from pathlib import Path


def test_solved_state_schema_has_input_time_and_no_raw_measurements():
    text = (Path(__file__).parents[1] / "msg" / "GloveState.msg").read_text()
    lines = [line for line in text.splitlines() if line and not line.startswith("#")]
    assert "std_msgs/Header header" in lines
    assert "uint64 host_realtime_ns" in lines
    assert "uint64[] source_sample_indices" in lines
    assert "float64[] mano_pose_rotvec" in lines
    assert "bool pose_valid" in lines and "bool time_valid" in lines
    assert "bool[] inferred_mask" in lines
    assert not any("gyro" in line or "accel" in line or "imu_raw" in line for line in lines)
