import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from vt_fsglove12.calibration import apply_calibration, calibrate, check_profile_binding
from vt_fsglove12.config import PARENTS, load_config


def data():
    config = load_config()
    config["node_to_slot"] = {"12": 0, "5": 14}
    rest = {"parents": list(PARENTS), "rest_global_xyzw": R.from_euler("z", (np.arange(16) * .03)[:, None]).as_quat().tolist(),
            "provenance": {"kind": "synthetic_test_only"}}
    roots = R.from_euler("xyz", [[0, 0, 0], [60, 0, 0], [0, 70, 0]], degrees=True)
    mounts = {"12": R.from_euler("xyz", [12, -20, 30], degrees=True),
              "5": R.from_euler("xyz", [-25, 10, 45], degrees=True)}
    alignments = {"12": R.from_euler("z", 25, degrees=True), "5": R.from_euler("z", -70, degrees=True)}
    manifest = {"schema_version": 1, "time_origin": "first_frame", "poses": []}
    frames = []
    for p, root in enumerate(roots):
        manifest["poses"].append({"start_s": p, "end_s": p + .9, "root_quat_xyzw": root.as_quat().tolist()})
        for i in range(8):
            nodes = {}
            for node, slot in config["node_to_slot"].items():
                target = root * R.from_quat(rest["rest_global_xyzw"][slot])
                sensor = alignments[node].inv() * target * mounts[node].inv()
                q = sensor.as_quat() * (-1 if i % 2 else 1)
                nodes[node] = {"quat_xyzw": q.tolist(), "age_s": 0, "epoch": 0}
            frames.append({"timestamp_s": 100 + p + i * .1, "nodes": nodes})
    return config, rest, manifest, frames, roots


def test_recover_mount_and_initial_heading_from_labelled_poses():
    config, rest, manifest, frames, roots = data()
    profile = calibrate(frames, manifest, rest, config, stream_sha256="a" * 64)
    assert profile["nodes"]["5"]["label_fit_rms_deg"] < 1e-6
    assert profile["status"] == "provisional_not_accuracy_validated"
    assert "11" not in profile["nodes"]
    for p in range(3):
        observations, rejected = apply_calibration(frames[p * 8], profile)
        assert not rejected
        for node, slot in config["node_to_slot"].items():
            expected = roots[p] * R.from_quat(rest["rest_global_xyzw"][slot])
            assert (expected.inv() * R.from_quat(observations[node])).magnitude() < 1e-8


def test_degenerate_motion_has_actionable_error():
    config, rest, manifest, frames, _ = data()
    for p in manifest["poses"]:
        p["root_quat_xyzw"] = [0, 0, 0, 1]
    with pytest.raises(ValueError, match="two distinct axes"):
        calibrate(frames, manifest, rest, config, stream_sha256="a" * 64)


def test_local_missing_sensor_and_epoch_reset_not_reused():
    config, rest, manifest, frames, _ = data()
    profile = calibrate(frames, manifest, rest, config, stream_sha256="a" * 64)
    frame = copy.deepcopy(frames[0])
    frame["nodes"]["5"]["epoch"] = 1
    observation, rejected = apply_calibration(frame, profile)
    assert set(observation) == {"12"}
    assert "epoch" in rejected["5"]
    frame["nodes"].pop("12")
    assert not apply_calibration(frame, profile)[0]


def test_profile_bound_to_stream_model_and_configuration():
    config, rest, manifest, frames, _ = data()
    profile = calibrate(frames, manifest, rest, config, stream_sha256="a" * 64)
    check_profile_binding(profile, config, rest, "a" * 64)
    with pytest.raises(ValueError, match="stream_sha256"):
        check_profile_binding(profile, config, rest, "b" * 64)
    changed = copy.deepcopy(rest)
    changed["rest_global_xyzw"][1] = [0, 0, 0, 1]
    with pytest.raises(ValueError, match="rest_sha256"):
        check_profile_binding(profile, config, changed, "a" * 64)


def test_cross_reset_finger_is_reported_not_fabricated():
    config, rest, manifest, frames, _ = data()
    frames[-1]["nodes"]["5"]["epoch"] = 1
    profile = calibrate(frames, manifest, rest, config, stream_sha256="a" * 64)
    assert set(profile["nodes"]) == {"12"}
    assert "5" in profile["rejected_nodes"]
