"""Guard-status consumers, using no device, ROS, upstream checkout, or MANO asset.

The tiny tensor/calibrator doubles exercise routing and masks only. They are
not substitutes for, or a numerical verification of, official MANO inference.
"""
from copy import deepcopy
from contextlib import nullcontext
import json
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.live_stream import OnlineFusion, RawSample
from vt_fsglove12.pose_export import build_state
from vt_fsglove12.upstream_backend import (
    OFFICIAL_STEPS, OfficialFSGloveBackend, _capture_node,
)


SESSION = "input-guard-consumer-test"


def captures(config, count=3):
    result = {}
    for pose_index, (step, angle) in enumerate(zip(OFFICIAL_STEPS, [0., 50., -50.])):
        quaternion = Rotation.from_euler("x", angle, degrees=True).as_quat().tolist()
        result[step["key"]] = [
            dict(timestamp_s=100. + pose_index + index * .01, session_id=SESSION,
                 nodes={node: dict(quat_xyzw=quaternion, age_s=.001, epoch=0,
                                   sample_index=pose_index * 10 + index + 1)
                        for node in config["node_to_slot"]})
            for index in range(count)]
    return result


class _ArrayTensor:
    """Only the CPU tensor readback interface used by the production wrapper."""
    def __init__(self, values):
        self.values = np.asarray(values)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values

    def float(self):
        return _ArrayTensor(self.values.astype(np.float32))

    def __getitem__(self, key):
        return _ArrayTensor(self.values[key])

    def __setitem__(self, key, value):
        self.values[key] = value.values


@pytest.fixture
def backend_data():
    calls = []

    def get_mano_param(rotations, **_):
        calls.append(rotations.as_quat())
        return _ArrayTensor(np.zeros((16, 3)))

    def create_calibrator(base_pose, **_):
        return SimpleNamespace(
            _base_quat=np.asarray(base_pose), _delta_rot_fa_rotvec=np.zeros((16, 3)),
            _global_installation_error=np.zeros((16, 3)),
            _local_installation_error=np.zeros((16, 3)),
            _joint_fa=np.array([0, *PARENTS[1:]]), _fk_seq=np.arange(1, 16),
            get_mano_param=get_mano_param,
            _axislayer=SimpleNamespace(compose=lambda tensor: tensor),
        )

    config = load_config()
    # These short fixtures exercise transient interception, not the separate
    # receive-health warmup/recovery contract covered by input_health tests.
    config["input_health"] = {"enabled": False}
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    core = SimpleNamespace(create_calibrator=create_calibrator,
                           torch=SimpleNamespace(from_numpy=_ArrayTensor, no_grad=nullcontext),
                           provenance={"upstream_commit": "consumer-routing-test-double"})
    backend = OfficialFSGloveBackend(config, rest, checkout="unused", mano_assets_root="unused", core=core)
    rows = captures(config)
    profile = backend.fit_calibration(rows, session_id=SESSION)
    return backend, profile, deepcopy(rows["base_pose"][-1]), calls


@pytest.mark.parametrize("node", ["3", "6"])
def test_invalid_finger_is_excluded_locally_and_recovers_without_profile_reset(backend_data, node):
    backend, profile, frame, calls = backend_data
    profile_before = deepcopy(profile)
    slot = backend.config["node_to_slot"][node]
    invalid_quat = Rotation.from_euler("xyz", [1., -.4, .3]).as_quat().tolist()
    frame["nodes"][node].update(orientation_valid=False, quat_xyzw=invalid_quat)
    original = deepcopy(frame)
    pose = backend.solve(frame, profile)
    assert pose["pose_available"] and pose["solver_success"]
    assert pose["calibration_rejections"] == {node: "input_filter_prolonged_invalid"}
    assert not pose["observed_mask"][slot] and not pose["joint_observed_mask"][slot]
    assert pose["inferred_mask"][slot] and slot in pose["virtual_input_slots"]
    for other, mapped in backend.config["node_to_slot"].items():
        assert pose["observed_mask"][mapped] == (other != node)
    assert len(calls) == 1
    for forwarded in calls:
        assert not np.allclose(forwarded[slot], invalid_quat)
    assert frame == original and profile == profile_before
    frame["nodes"][node]["orientation_valid"] = True
    recovered = backend.solve(frame, profile)
    assert recovered["pose_available"] and recovered["observed_mask"][slot]
    assert recovered["calibration_rejections"] == {}
    np.testing.assert_allclose(calls[-1][slot], invalid_quat, atol=1e-15)
    assert profile == profile_before


def test_invalid_palm_withholds_pose_before_inference_and_recovers(backend_data):
    backend, profile, frame, calls = backend_data
    frame["nodes"]["12"]["orientation_valid"] = False
    pose = backend.solve(frame, profile)
    assert not pose["pose_available"] and not pose["solver_success"]
    assert pose["mano_pose_rotvec"] is None and pose["root_quat_xyzw"] is None
    assert pose["reason"] == "missing_or_stale_root"
    assert pose["calibration_rejections"]["12"] == "input_filter_prolonged_invalid"
    assert not pose["observed_mask"][0] and calls == []
    frame["nodes"]["12"]["orientation_valid"] = True
    assert backend.solve(frame, profile)["pose_available"] and len(calls) == 1


def test_transient_protected_but_orientation_valid_finger_remains_usable(backend_data):
    backend, profile, frame, _ = backend_data
    frame["nodes"]["6"].update(orientation_valid=True,
                               input_filter=dict(gyro_rejected=True, accel_rejected=False))
    pose = backend.solve(frame, profile)
    assert pose["pose_available"] and pose["observed_mask"][15]
    assert pose["calibration_rejections"] == {}


@pytest.mark.parametrize("protection", [
    dict(orientation_valid=False, input_filter=dict(gyro_rejected=False, accel_rejected=False)),
    dict(orientation_valid=True, input_filter=dict(gyro_rejected=True, accel_rejected=False)),
    dict(orientation_valid=True, input_filter=dict(gyro_rejected=False, accel_rejected=True)),
])
def test_capture_skips_protected_samples_before_quaternion_epoch_and_dedup_checks(protection):
    config = load_config()
    rows = captures(config)
    expected_means, expected_epoch, expected_counts = _capture_node(rows, "6", .15, SESSION)
    for step_rows in rows.values():
        bad = deepcopy(step_rows[0])
        # Protected observations must not enter the quaternion mean, bind an
        # epoch, or consume a unique sample index from a subsequent good row.
        bad["nodes"]["6"].update(protection, quat_xyzw=[0., 0., 0., 0.])
        wrong_epoch = deepcopy(bad)
        wrong_epoch["nodes"]["6"].update(epoch=999, sample_index=999)
        step_rows[0:0] = [bad, wrong_epoch]
    original = deepcopy(rows)
    actual_means, actual_epoch, actual_counts = _capture_node(rows, "6", .15, SESSION)
    np.testing.assert_array_equal(actual_means, expected_means)
    assert actual_epoch == expected_epoch == 0
    assert actual_counts == expected_counts == [3, 3, 3]
    assert rows == original
    # A protected sample cannot satisfy the minimum three real observations.
    for step_rows in rows.values():
        del step_rows[-1]
    with pytest.raises(ValueError, match="requires at least 3 fresh unique samples"):
        _capture_node(rows, "6", .15, SESSION)


def test_capture_accepts_clean_status_identically_to_legacy_no_status():
    rows = captures(load_config(), count=4)
    expected = _capture_node(rows, "6", .15, SESSION)
    for step_rows in rows.values():
        for frame in step_rows:
            frame["nodes"]["6"].update(orientation_valid=True,
                input_filter=dict(enabled=True, gyro_rejected=False, accel_rejected=False))
    actual = _capture_node(rows, "6", .15, SESSION)
    np.testing.assert_array_equal(actual[0], expected[0])
    assert actual[1:] == expected[1:] == (0, [4, 4, 4])


@pytest.mark.parametrize("invalid_palm", [False, True])
def test_pose_export_keeps_real_interception_status_without_raw_or_fusion_si(backend_data, invalid_palm):
    backend, profile, _, _ = backend_data
    fusion = OnlineFusion(backend.config)
    protected = 12 if invalid_palm else 6
    count = 32 if invalid_palm else 1
    for index in range(count + 1):
        stamp_ns = 1_000_000_000 + index * 8_333_333
        realtime_ns = 1_800_000_000_000_000_000 + stamp_ns
        fusion.update([
            RawSample(node, (((-1) ** index * 32764 if node == protected and index else 0), 0, 0, 0, 0, 8197),
                      stamp_ns / 1_000_000_000, realtime_ns / 1_000_000_000, host_monotonic_ns=stamp_ns,
                      host_realtime_ns=realtime_ns)
            for node in (6, 12)])
    frame = fusion.snapshot(stamp_ns / 1_000_000_000)
    frame["session_id"] = SESSION
    pose = backend.solve(frame, profile)
    assert pose["pose_available"] == (not invalid_palm)
    pose.update(input_filter=deepcopy(frame["input_filter"]),
                sensor_quality=deepcopy(frame["sensor_quality"]),
                session_id=SESSION, input_kind="raw_serial")
    geometry = dict(joints_m=[[0., 0., 0.]] * 21, vertices_m=[[0., 0., 0.]] * 778)
    original = deepcopy((frame, pose, geometry))
    result = build_state(frame, pose, geometry, backend.config, root_node="12",
                         solve_completed_monotonic_ns=stamp_ns + 1_000_000)
    exported = json.loads(result["quality_json"])
    assert exported["input_filter"] == frame["input_filter"]
    status = exported["input_filter"]["nodes"][str(protected)]
    assert status["gyro_rejected"] and status["reasons"] == ["gyro_boundary"]
    assert status["gyro_rejected_samples"] == count
    assert status["orientation_valid"] == (not invalid_palm)
    assert exported["input_filter"]["active_nodes"] == [str(protected)]
    assert result["pose_valid"] == (not invalid_palm) and result["time_valid"]
    assert json.loads(result["metadata_json"])["input_filter"] == backend.config["input_filter"]
    assert (frame, pose, geometry) == original
    encoded = json.dumps({**result, "quality_json": exported,
                          "metadata_json": json.loads(result["metadata_json"])}, allow_nan=False)
    for private_field in ("raw_values", "gyro_lsb", "gx_lsb", "gyro_rad_s", "accel_m_s2",
                          "fusion_gyro_rad_s", "fusion_accel_m_s2", "fusion_state",
                          "input_observation", "vertices_m"):
        assert f'"{private_field}"' not in encoded
