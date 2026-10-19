"""Synthetic explicit-zero contracts; these do not validate human accuracy."""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.calibration import apply_calibration
from vt_fsglove12.config import PARENTS, digest, load_config
from vt_fsglove12.neutral_reference import reanchor_neutral_reference
from vt_fsglove12.solver import SparseHandSolver


def case():
    rng = np.random.default_rng(1001)
    config = load_config()
    rest_rotations = Rotation.random(16, random_state=rng)
    rest = dict(parents=list(PARENTS), rest_global_xyzw=rest_rotations.as_quat().tolist())
    root = Rotation.from_euler("XYZ", [.5, -.3, .7])
    nodes, sensor_nodes = {}, {}
    for node, slot in config["node_to_slot"].items():
        mount, alignment = Rotation.random(2, random_state=rng)
        sensor = alignment.inv() * root * rest_rotations[slot] * mount.inv()
        wrong_alignment = alignment if slot == 0 else Rotation.from_euler("XYZ", [.2, -.4, .3]) * alignment
        nodes[node] = dict(slot=slot, epoch=2, sensor_from_bone_xyzw=mount.as_quat().tolist(),
                           world_alignment_xyzw=wrong_alignment.as_quat().tolist(), label_fit_rms_deg=12.)
        sensor_nodes[node] = dict(quat_xyzw=sensor.as_quat().tolist(), age_s=.005, epoch=2)
    rows = [dict(timestamp_s=100. + .05 * i, nodes={node: dict(data, sample_index=i+1)
             for node, data in sensor_nodes.items()}) for i in range(20)]
    profile = dict(nodes=nodes, stale_after_s=.15, session_id="current", rest_sha256=digest(rest),
                   config_sha256=digest(config), profile_version=3,
                   status="provisional_not_accuracy_validated", warnings=["axis_fit_warning"])
    return config, rest, profile, rows, root


def test_explicit_reference_removes_offset_without_reorienting_palm_or_refitting_mounts():
    config, rest, profile, rows, root = case()
    before, input_rows = deepcopy(profile), deepcopy(rows)
    result = reanchor_neutral_reference(rows, profile, rest, session_id="current")
    assert profile == before and rows == input_rows
    assert result["nodes"]["12"] == profile["nodes"]["12"]
    assert result["warnings"] == profile["warnings"]
    assert result["manual_neutral_reference"]["previous_profile_version"] == 3
    for node in profile["nodes"]:
        assert result["nodes"][node]["sensor_from_bone_xyzw"] == profile["nodes"][node]["sensor_from_bone_xyzw"]
        assert result["nodes"][node]["label_fit_rms_deg"] == 12.
    observations, rejected = apply_calibration(rows[-1], result)
    assert not rejected
    pose = SparseHandSolver(config, rest["rest_global_xyzw"]).solve(observations, 101.)
    np.testing.assert_allclose(pose["anatomical_euler_rad"], 0., atol=1e-7)
    assert (root.inv() * Rotation.from_quat(pose["root_quat_xyzw"])).magnitude() < 1e-7
    assert not result["yaw_drift_corrected"]
    assert not result["manual_neutral_reference"]["mounting_axes_refitted"]


def test_missing_or_epoch_changed_finger_keeps_previous_reference_locally():
    _, rest, profile, rows, _ = case()
    for row in rows:
        del row["nodes"]["9"]
        row["nodes"]["10"]["epoch"] += 1
    result = reanchor_neutral_reference(rows, profile, rest, session_id="current")
    assert result["nodes"]["9"] == profile["nodes"]["9"]
    assert result["nodes"]["10"] == profile["nodes"]["10"]
    assert set(result["manual_neutral_reference"]["rejected_nodes"]) == {"9", "10"}
    assert "5" in result["manual_neutral_reference"]["updated_nodes"]


@pytest.mark.parametrize("kind", ["missing", "epoch", "frozen", "stale"])
def test_palm_failure_rejects_reference_atomically(kind):
    _, rest, profile, rows, _ = case()
    before = deepcopy(profile)
    for row in rows:
        if kind == "missing":
            del row["nodes"]["12"]
        elif kind == "epoch":
            row["nodes"]["12"]["epoch"] += 1
        elif kind == "frozen":
            row["nodes"]["12"]["sample_index"] = 1
        else:
            row["nodes"]["12"]["age_s"] = 1.
    with pytest.raises(ValueError):
        reanchor_neutral_reference(rows, profile, rest, session_id="current")
    assert before == profile


def test_moving_finger_is_not_silently_taken_as_straight():
    _, rest, profile, rows, _ = case()
    for i, row in enumerate(rows):
        row["nodes"]["9"]["quat_xyzw"] = Rotation.from_euler("z", i * .05).as_quat().tolist()
    result = reanchor_neutral_reference(rows, profile, rest, session_id="current")
    assert "运动较大" in result["manual_neutral_reference"]["rejected_nodes"]["9"]
    assert result["nodes"]["9"] == profile["nodes"]["9"]


def test_other_session_or_model_cannot_reuse_reference():
    _, rest, profile, rows, _ = case()
    with pytest.raises(ValueError, match="连续连接"):
        reanchor_neutral_reference(rows, profile, rest, session_id="other")
    with pytest.raises(ValueError, match="模型"):
        reanchor_neutral_reference(rows, profile, dict(rest, changed=True), session_id="current")


def test_new_neutral_invalidates_functional_axes_in_previous_coordinates():
    _, rest, profile, rows, _ = case()
    profile["functional_fingers"] = {"little": {"old": "reference"}}
    result = reanchor_neutral_reference(rows, profile, rest, session_id="current")
    assert "functional_fingers" not in result
    assert "functional_fingers" in profile
    assert result["manual_neutral_reference"]["functional_axes_cleared"]
    assert result["neutral_reference"]["target"] == "model_rest_relative_to_unchanged_palm"
    for node in result["manual_neutral_reference"]["updated_nodes"]:
        params = result["nodes"][node]
        from vt_fsglove12.calibration import quat
        target = quat(params["world_alignment_xyzw"]) * quat(params["neutral_sensor_xyzw"]) * quat(params["sensor_from_bone_xyzw"])
        assert (target.inv() * quat(params["neutral_target_xyzw"])).magnitude() < 1e-7
