"""Explicit neutral reference, without disguising inconsistent three-pose fits."""
from copy import deepcopy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.calibration import _fit, apply_calibration
from vt_fsglove12.guided_calibration import _collect_node
from test_guided_calibration import KEYS, assert_same_rotation, data, fit


def altered_finger(*, pitch_scale=1.6, roll_scale=1., node="5"):
    values = data()
    config, rest, _, captures, _, _, roots, make_frame = values
    bone_rest = Rotation.from_quat(rest["rest_global_xyzw"][config["node_to_slot"][node]])
    for key, root, scale in zip(KEYS, roots, (1., pitch_scale, roll_scale)):
        bone = Rotation.from_rotvec(root.as_rotvec() * scale) * bone_rest
        for frame in captures[key]:
            index = frame["nodes"][node]["sample_index"]
            frame["nodes"][node] = make_frame({node: bone}, frame["timestamp_s"], index)["nodes"][node]
    return values


@pytest.mark.parametrize("pitch_scale,roll_scale,node", [(1.6, 1., "5"), (1.6, 1.6, "5"), (1., 1.8, "3")])
def test_nonrigid_amplitude_fit_keeps_mount_and_anchors_neutral_without_hiding_error(pitch_scale, roll_scale, node):
    values = altered_finger(pitch_scale=pitch_scale, roll_scale=roll_scale, node=node)
    config, rest, _, captures = values[:4]
    before = deepcopy(captures)
    profile = fit(values)
    params = profile["nodes"][node]
    means, _, _, _, _ = _collect_node(captures, node, config["stale_after_s"])
    target = Rotation.from_quat([profile["pose_root_quat_xyzw"][key] for key in KEYS]) * Rotation.from_quat(
        rest["rest_global_xyzw"][config["node_to_slot"][node]])
    old_mount, old_alignment, old_errors = _fit(means, target)
    assert_same_rotation(params["sensor_from_bone_xyzw"], old_mount.as_quat())
    assert_same_rotation(params["pre_anchor_world_alignment_xyzw"], old_alignment.as_quat())
    assert_same_rotation(params["neutral_sensor_xyzw"], means[0].as_quat())
    assert_same_rotation(params["neutral_target_xyzw"], target[0].as_quat())
    np.testing.assert_allclose(params["pre_anchor_label_fit_per_pose_deg"], old_errors, atol=1e-9)
    assert params["pre_anchor_label_fit_per_pose_deg"][0] > 1.
    assert params["label_fit_per_pose_deg"][0] < 1e-9
    assert params["label_fit_rms_deg"] > 5
    assert any(f"节点 {node}" in warning for warning in profile["warnings"])
    assert any("幅度" in warning for warning in profile["warnings"])
    assert not profile["rejected_nodes"]  # No stricter recapture gate was introduced.
    # The exact neutral model is available to apply_calibration without changing
    # its schema or introducing a solver-side display offset.
    observed, rejected = apply_calibration(captures["neutral"][0], profile)
    assert not rejected
    for key, quaternion in observed.items():
        assert_same_rotation(quaternion, rest["rest_global_xyzw"][config["node_to_slot"][key]])
    assert captures == before
    json.dumps(profile, allow_nan=False)


def test_reported_post_anchor_errors_are_actual_mapping_errors_not_old_fit_metrics():
    values = altered_finger()
    config, rest, _, captures = values[:4]
    profile = fit(values)
    params = profile["nodes"]["5"]
    errors = []
    for key in KEYS:
        observed, _ = apply_calibration(captures[key][0], profile)
        target = Rotation.from_quat(profile["pose_root_quat_xyzw"][key]) * Rotation.from_quat(
            rest["rest_global_xyzw"][config["node_to_slot"]["5"]])
        errors.append(float(np.rad2deg((target.inv() * Rotation.from_quat(observed["5"])).magnitude())))
    np.testing.assert_allclose(params["label_fit_per_pose_deg"], errors, atol=1e-8)
    assert params["label_fit_rms_deg"] == pytest.approx(np.sqrt(np.mean(np.square(errors))))
    assert params["pre_anchor_label_fit_rms_deg"] == pytest.approx(
        np.sqrt(np.mean(np.square(params["pre_anchor_label_fit_per_pose_deg"]))))
    np.testing.assert_allclose(params["rigid_motion_magnitude_error_deg"], [19.8, 0.], atol=1e-8)
    assert params["label_fit_rms_deg"] > params["pre_anchor_label_fit_rms_deg"]


def test_neutral_mean_not_arbitrary_first_sample_is_saved_and_anchored():
    values = data()
    for index, frame in enumerate(values[3]["neutral"]):
        sample = frame["nodes"]["5"]
        sample["quat_xyzw"] = (Rotation.from_quat(sample["quat_xyzw"])
                               * Rotation.from_rotvec([.1 * (-1) ** index, 0., 0.])).as_quat().tolist()
    profile = fit(values)
    params = profile["nodes"]["5"]
    neutral_mean = Rotation.from_quat([frame["nodes"]["5"]["quat_xyzw"] for frame in values[3]["neutral"]]).mean()
    assert_same_rotation(params["neutral_sensor_xyzw"], neutral_mean.as_quat())
    first = Rotation.from_quat(values[3]["neutral"][0]["nodes"]["5"]["quat_xyzw"])
    assert (neutral_mean.inv() * first).magnitude() > .09
    mapped = Rotation.from_quat(params["world_alignment_xyzw"]) * neutral_mean * Rotation.from_quat(params["sensor_from_bone_xyzw"])
    assert_same_rotation(mapped.as_quat(), params["neutral_target_xyzw"])
    assert max(params["within_pose_p95_deg"]) > 5.
    assert any("节点 5" in warning for warning in profile["warnings"])


def test_ideal_data_is_unchanged_and_reference_is_neither_drift_correction_nor_rom():
    values = data()
    profile = fit(values)
    assert profile["method"] == "guided_rigid_pose_hand_eye_neutral_anchor_v2"
    assert profile["schema_version"] == 1
    assert not profile["warnings"]
    assert not profile["yaw_drift_corrected"]
    assert not profile["anatomical_accuracy_verified"]
    assert not profile["personal_rom_calibrated"]
    assert profile["neutral_reference"] == dict(kind="explicit_neutral_pose_mean", step="neutral",
                                               target="model_rest_global", mounting_fit_preserved=True,
                                               automatically_updated=False)
    for node, params in profile["nodes"].items():
        assert_same_rotation(params["world_alignment_xyzw"], params["pre_anchor_world_alignment_xyzw"])
        assert params["label_fit_rms_deg"] < 1e-8
        assert params["pre_anchor_label_fit_rms_deg"] < 1e-8
        np.testing.assert_allclose(params["rigid_motion_magnitude_error_deg"], [0., 0.], atol=1e-8)


def test_small_nonrigid_amplitude_mismatch_warns_even_if_rms_below_five_degrees():
    profile = fit(altered_finger(pitch_scale=39. / 33.))
    params = profile["nodes"]["5"]
    assert params["label_fit_rms_deg"] < 5
    assert params["rigid_motion_magnitude_error_deg"][0] == pytest.approx(6.)
    assert any("节点 5" in text and "幅度" in text for text in profile["warnings"])
    assert "5" not in profile["rejected_nodes"]
