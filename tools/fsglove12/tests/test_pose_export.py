"""Results-only transport tests: no ROS, serial, calibration or model assets."""
from copy import deepcopy
import json
import socket

import pytest

from vt_fsglove12.config import load_config
from vt_fsglove12.pose_export import PoseDatagramExporter, build_state, validate_socket_path
from vt_fsglove12.sensor_quality import GyroNearRailMonitor


def inputs():
    frame = {"timestamp_s": 99., "nodes": {
        "12": dict(host_realtime_ns=1_800_000_000_000_000_123,
                   host_monotonic_ns=1_000_000_123, sample_index=44, epoch=2),
        "5": dict(host_realtime_ns=1_800_000_000_000_000_100,
                  host_monotonic_ns=1_000_000_100, sample_index=42, epoch=0),
        "11": dict(host_realtime_ns=1_800_000_000_000_000_101,
                   host_monotonic_ns=1_000_000_101, sample_index=44, epoch=0),
    }}
    pose = dict(pose_available=True, solver_success=True,
                mano_pose_rotvec=[[0., 0., .1]] * 16, root_quat_xyzw=[0., 0., 0., 1.],
                shape_params=[0.] * 10, profile_version=3, frame_index=27,
                session_id="test-session", input_kind="raw_serial", quality="not_accuracy_validated",
                algorithm="official_fsglove_sparse_adapter", reason="official_inference_complete",
                source_labels=["prior_not_observation"] * 16,
                segment_observed_mask=[True] * 16, joint_observed_mask=[False] * 16,
                inferred_mask=[True] * 16, sensor_quality={"gyro_near_rail": {"active_nodes": ["5"]}})
    geometry = dict(joints_m=[[0., .02, .03]] * 21, vertices_m=[[1., 2., 3.]] * 778)
    return frame, pose, geometry


def state(frame=None, pose=None, geometry=None):
    defaults = inputs()
    return build_state(frame if frame is not None else defaults[0],
                       pose if pose is not None else defaults[1],
                       geometry if geometry is not None else defaults[2],
                       load_config(), root_node="12", solve_completed_monotonic_ns=1_030_000_000)


@pytest.mark.parametrize("where", ["pose", "geometry", "provenance"])
def test_display_only_rotations_and_geometry_cannot_enter_measurement_export(where):
    frame, pose, geometry = inputs()
    if where == "pose":
        pose["display_only"] = True
    elif where == "geometry":
        geometry["display_only"] = True
    else:
        geometry["provenance"] = {"display_only": True}
    with pytest.raises(ValueError, match="display-only"):
        state(frame, pose, geometry)


def test_result_uses_exact_palm_input_time_and_same_geometry_not_tick_or_publish_time():
    frame, pose, geometry = inputs()
    before = deepcopy((frame, pose, geometry))
    result = state(frame, pose, geometry)
    assert (frame, pose, geometry) == before
    assert result["pose_valid"] and result["time_valid"]
    assert result["host_realtime_ns"] == 1_800_000_000_000_000_123
    assert result["host_monotonic_ns"] == 1_000_000_123
    assert result["source_node_ids"] == [5, 12]  # Auxiliary node 11 is not used by the model.
    assert result["source_sample_indices"] == [42, 44]
    assert result["source_filter_epochs"] == [0, 2]
    assert result["mano_pose_rotvec"] == [0., 0., .1] * 16
    assert result["joint_positions_m"] == [0., .02, .03] * 21
    assert result["calibration_version"] == 3
    assert not result["units_verified"] and not result["anatomical_accuracy_verified"]
    assert result["inferred_mask"] == [True] * 16
    assert "vertices_m" not in json.dumps(result)
    assert "gx_lsb" not in json.dumps(result) and "gyro_rad_s" not in json.dumps(result)


def test_unknown_or_legacy_source_time_is_not_invented():
    frame, pose, geometry = inputs()
    frame["nodes"]["12"]["host_realtime_ns"] = None
    result = state(frame, pose, geometry)
    assert not result["pose_valid"] and not result["time_valid"]
    assert result["reason"] == "source_time_unavailable"
    assert result["host_realtime_ns"] == result["host_monotonic_ns"] == 0
    assert result["mano_pose_rotvec"] == result["joint_positions_m"] == []


def test_result_metadata_distinguishes_recomputed_same_source_from_new_input():
    frame, pose, geometry = inputs()
    pose.update(source_fresh=False, adapter_execution={"official_anatomical_fk_calls": 1})
    result = state(frame, pose, geometry)
    metadata = json.loads(result["metadata_json"])
    assert metadata["source_fresh"] is False
    assert metadata["adapter_execution"] == {"official_anatomical_fk_calls": 1}
    assert result["host_monotonic_ns"] == frame["nodes"]["12"]["host_monotonic_ns"]
    assert result["source_sample_indices"] == [42, 44]


def test_per_node_accel_contract_metadata_does_not_publish_raw_or_filter_diagnostics():
    frame, pose, geometry = inputs()
    frame["nodes"]["12"].update(raw_values=[32764, 0, 0, 0, 0, 8197],
                                gyro_rad_s=[1., 2., 3.], fusion_state={"gyro_bias_rad_s": [0., 0., 0.]})
    frame["input_observation"] = {"nodes": {"12": {"samples": 100}}}
    config = load_config()
    config["accel_g_per_lsb_by_node"] = {"9": .000244}
    result = build_state(frame, pose, geometry, config, root_node="12", solve_completed_monotonic_ns=1_030_000_000)
    metadata = json.loads(result["metadata_json"])
    assert metadata["accel_g_per_lsb_by_node"] == {"9": .000244}
    assert metadata["input_contract"]["sensor_model"] == "LSM6DSRTR"
    encoded = json.dumps(result)
    assert "raw_values" not in encoded and "gyro_rad_s" not in encoded
    assert "fusion_state" not in encoded and "input_observation" not in encoded


def test_boundary_event_export_keeps_provenance_but_not_raw_gyro_or_source_mutations():
    frame, pose, geometry = inputs()
    monitor = GyroNearRailMonitor()
    monitor.observe(5, (-32768, 12345, 32761), 1.)
    monitor.observe(12, (23456, 32767, -32123), 1.)
    monitor.observe(11, (1, 2, 3), 1.)
    monitor.observe(5, (4, 5, 6), 1.1)
    frame["sensor_quality"] = pose["sensor_quality"] = monitor.snapshot(1.1)
    before = deepcopy((frame, pose, geometry))
    result = state(frame, pose, geometry)
    exported = json.loads(result["quality_json"])["sensor_quality"]
    expected = deepcopy(pose["sensor_quality"])
    for key in ("5", "12"):
        del expected["gyro_near_rail"]["nodes"][key]["last_boundary_event"]["gyro_lsb"]
    assert exported == expected
    assert exported["gyro_near_rail"]["nodes"]["11"]["last_boundary_event"] is None
    assert (frame, pose, geometry) == before
    assert pose["sensor_quality"]["gyro_near_rail"]["nodes"]["5"]["last_boundary_event"]["gyro_lsb"] == [-32768, 12345, 32761]
    encoded = json.dumps(result)
    assert "gyro_lsb" not in encoded and '"raw_values"' not in json.dumps(exported)
    for raw_integer in (-32768, 12345, 32761, 23456, 32767, -32123):
        assert str(raw_integer) not in encoded


def test_unavailable_results_still_report_actual_time_and_quality():
    frame, pose, geometry = inputs()
    pose.update(pose_available=False, reason="calibration_required", input_kind="csv_replay")
    result = state(frame, pose, geometry)
    assert result["time_valid"] and not result["pose_valid"]
    assert result["source_mode"] == "replay"
    assert result["reason"] == "calibration_required"
    assert result["mano_pose_rotvec"] == []
    assert json.loads(result["quality_json"])["sensor_quality"]["gyro_near_rail"]["active_nodes"] == ["5"]


def test_optional_heading_result_is_reported_without_claiming_absolute_heading():
    frame, pose, geometry = inputs()
    pose["relative_heading_correction"] = {"enabled": True, "applied_slots": [15],
                                           "absolute_heading_observed": False}
    config = load_config()
    config["relative_heading_correction"] = {"enabled": True}
    result = build_state(frame, pose, geometry, config, root_node="12", solve_completed_monotonic_ns=1_030_000_000)
    assert json.loads(result["quality_json"])["relative_heading_correction"] == pose["relative_heading_correction"]
    assert json.loads(result["metadata_json"])["relative_heading_correction"] == {"enabled": True}
    assert not result["yaw_drift_corrected"]
    assert "gx_lsb" not in json.dumps(result)


def test_little_heading_fix_reports_results_only_quality_and_processing_revision():
    frame, pose, geometry = inputs()
    pose["little_relative_heading"] = dict(applied=True, slot=8, delta_rad=.2,
                                          residual_deg=1., absolute_heading_observed=False)
    pose["pose_processing_revision"] = "little_relative_heading_and_hinge_continuity_v1"
    exported = state(frame, pose, geometry)
    quality = json.loads(exported["quality_json"])
    assert quality["little_relative_heading"] == pose["little_relative_heading"]
    assert quality["pose_processing_revision"] == pose["pose_processing_revision"]
    assert not exported["yaw_drift_corrected"] and not exported["anatomical_accuracy_verified"]
    assert "raw_values" not in json.dumps(exported)


@pytest.mark.parametrize("status", ["valid", "degraded", "unavailable"])
def test_joint_validity_exports_immediately_without_suppressing_other_joints_or_provenance(status):
    frame, pose, geometry = inputs()
    pose.update(joint_valid_mask=[True] * 16, joint_display_available_mask=[True] * 16,
                joint_quality=["valid"] * 16,
                joint_estimation={"8": {"quality": status, "reason": "test", "residual_deg": 15.1}})
    for slot in (8, 9):
        pose["joint_valid_mask"][slot] = status == "valid"
        pose["joint_display_available_mask"][slot] = status != "unavailable"
        pose["joint_quality"][slot] = status
    before = deepcopy(pose)
    result = state(frame, pose, geometry)
    assert result["pose_valid"] is (status == "valid")
    if status != "valid":
        assert result["reason"] == "joint_estimation_" + status
    assert result["mano_pose_rotvec"] == [0., 0., .1] * 16
    assert result["joint_positions_m"] == [0., .02, .03] * 21
    assert result["segment_observed_mask"] == pose["segment_observed_mask"]
    assert result["inferred_mask"] == pose["inferred_mask"]
    quality = json.loads(result["quality_json"])
    for key in ("joint_valid_mask", "joint_display_available_mask", "joint_quality", "joint_estimation"):
        assert quality[key] == pose[key]
    contract = json.loads(result["metadata_json"])["joint_validity_contract"]
    assert contract["display_hysteresis_exported"] is False
    assert "NOT valid measurement" in contract["partial_arrays"]
    assert pose == before


def test_partial_new_joint_contract_fails_closed_in_export_not_assumed_legacy_valid():
    frame, pose, geometry = inputs()
    pose["joint_valid_mask"] = [True] * 16
    result = state(frame, pose, geometry)
    assert result["pose_valid"] is False
    assert json.loads(result["quality_json"])["joint_quality"] == ["unavailable"] * 16


@pytest.mark.parametrize("key,value", [("mano_pose_rotvec", [[float("nan"), 0, 0]] * 16),
                                     ("shape_params", [0] * 9), ("root_quat_xyzw", [0, 0, 1])])
def test_incomplete_or_nonfinite_pose_is_not_published_as_valid(key, value):
    frame, pose, geometry = inputs()
    pose[key] = value
    result = state(frame, pose, geometry)
    assert not result["pose_valid"]
    json.dumps(result, allow_nan=False)


def test_datagram_bridge_can_appear_restart_and_fail_without_harming_sender(tmp_path):
    path = str(tmp_path / "s")
    sender = PoseDatagramExporter(path)
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    receiver.settimeout(.2)
    try:
        assert not sender.publish(state())
        assert sender.snapshot()["dropped"] == 1
        receiver.bind(path)
        assert sender.publish(state())
        payload = json.loads(receiver.recv(60_001))
        assert payload["producer_dropped_total"] == 1
        assert payload["kind"] == "glove_state"
        assert sender.snapshot()["last_error"] is None
        receiver.close()
        assert not sender.publish(state())
        assert not sender.publish({"invalid": float("nan")} )
        assert not sender.publish({"too_large": "x" * 60_001})
        assert sender.snapshot()["dropped"] == 4
    finally:
        receiver.close()
        sender.close()


@pytest.mark.parametrize("path", ["relative.sock", "/tmp/\0socket", "/tmp/" + "x" * 101])
def test_bad_socket_paths_are_rejected_without_connecting(path):
    with pytest.raises(ValueError):
        validate_socket_path(path)
