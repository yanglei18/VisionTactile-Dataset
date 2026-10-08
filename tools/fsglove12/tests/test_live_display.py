from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.live_display import (JointQualityPresentation, demonstration_root, display_basis, downstream_display_mask,
                                     joint_angle_text, joint_quality_message, measured_pose_visible,
                                     neutral_geometry_slot_map, unavailable_pose_message)
from vt_fsglove12.joint_quality import joint_quality_state


AXES = {"roll_axis": [-1., 0., 0.], "pitch_axis": [0., 0., 1.], "palm_up_axis": [0., 1., 0.]}


def snapshot():
    return {"runtime": {"connected": True, "last_sample_age_s": .01, "published_monotonic_s": 10.},
            "pose": {"timestamp_s": 10., "quality": "experimental", "pose_available": True, "solver_success": True}}


def test_fresh_pose_only_visible_while_connected_and_successful():
    assert measured_pose_visible(snapshot(), 10.01)
    for key, value in (("connected", False), ("last_sample_age_s", None),
                       ("last_sample_age_s", .3), ("published_monotonic_s", 8.)):
        sample = snapshot()
        sample["runtime"][key] = value
        assert not measured_pose_visible(sample, 10.01)
    for key, value in (("pose_available", False), ("solver_success", False),
                       ("timestamp_s", 9.), ("quality", "unavailable")):
        sample = snapshot()
        sample["pose"][key] = value
        assert not measured_pose_visible(sample, 10.01)
    assert not measured_pose_visible(snapshot(), 11.)


@pytest.mark.parametrize("version", ["v1", "v2"])
def test_official_pose_requires_executed_core_for_either_adapter_version(version):
    sample = snapshot()
    sample["pose"].update(
        algorithm=f"official_fsglove_raw2mano_with_explicit_sparse_adapter_{version}",
        quality="not_accuracy_validated", upstream_fsglove_solver_executed=True)
    assert measured_pose_visible(sample, 10.01)
    sample["pose"]["upstream_fsglove_solver_executed"] = False
    assert not measured_pose_visible(sample, 10.01)


def test_functional_official_pose_remains_visible_but_requires_executed_official_core():
    sample = snapshot()
    sample["pose"].update(algorithm="official_fsglove_with_qmt_functional_adapter_v1",
                          quality="not_accuracy_validated", upstream_fsglove_solver_executed=True)
    assert measured_pose_visible(sample, 10.01)
    sample["pose"]["upstream_fsglove_solver_executed"] = False
    assert not measured_pose_visible(sample, 10.01)


def test_correct_chirality_and_fixed_root_rotation_not_cancelled():
    axes = deepcopy(AXES)
    basis = display_basis(axes)
    np.testing.assert_allclose(basis @ basis.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(basis) > .9999
    np.testing.assert_allclose(basis @ axes["roll_axis"], [0, 1, 0])
    np.testing.assert_allclose(basis @ axes["pitch_axis"], [-1, 0, 0])
    assert axes == AXES
    points = np.array([[.01, .02, .03], [-.1, 0, 0]])
    root = Rotation.from_euler("XYZ", [.3, -.5, .7])
    assert not np.allclose(root.apply(points) @ basis.T, points @ basis.T)


def test_demonstration_is_fixed_instruction_not_a_measured_input():
    np.testing.assert_allclose(demonstration_root("neutral", AXES), [0, 0, 0, 1])
    for step, axis in (("fingers_down", "pitch_axis"), ("thumb_up", "roll_axis")):
        vector = Rotation.from_quat(demonstration_root(step, AXES)).as_rotvec()
        np.testing.assert_allclose(vector, np.deg2rad(40) * np.array(AXES[axis]))


def test_unavailable_status_distinguishes_short_stall_and_invalid_calibration():
    state = snapshot()
    state["pose"].update(pose_available=False, solver_success=False, reason="missing_or_stale_root")
    original = deepcopy(state)
    assert "暂不重做标定" in unavailable_pose_message(state)
    assert state == original
    state["pose"].update(reason="calibration_epoch_changed", calibration_rejections={
        "12": "filter_epoch_changed_recalibration_required"})
    assert "标定失效" in unavailable_pose_message(state)
    assert "仅重设伸直参考不能恢复" in unavailable_pose_message(state)
    state["pose"].update(reason="calibration_required", calibration_rejections={})
    assert "本次连接尚未标定" in unavailable_pose_message(state)
    state["pose"]["reason"] = "process_unresponsive"
    assert "计算进程暂不可用" in unavailable_pose_message(state)
    state["pose"]["reason"] = "disconnected"
    state["runtime"]["connected"] = False
    assert "当前未连接" in unavailable_pose_message(state)


def quality_pose(state="valid", slot=8):
    pose = dict(anatomical_euler_rad=[[0., 0., .4] for _ in range(16)],
                joint_valid_mask=[True] * 16, joint_display_available_mask=[True] * 16,
                joint_quality=["valid"] * 16)
    pose["joint_valid_mask"][slot] = state == "valid"
    pose["joint_display_available_mask"][slot] = state != "unavailable"
    pose["joint_quality"][slot] = state
    return pose


def test_joint_validity_is_immediate_but_visual_recovery_requires_stable_interval():
    view = JointQualityPresentation(recovery_s=.2)
    bad, good = quality_pose("degraded"), quality_pose()
    assert not any(view.update(good, 10., session="one"))
    assert view.update(bad, 10.01, session="one")[8]
    assert joint_quality_state(bad)[0][8] is False
    assert view.update(good, 10.02, session="one")[8]
    assert joint_quality_state(good)[0][8] is True  # no delayed validity
    assert view.update(bad, 10.15, session="one")[8]
    assert view.update(good, 10.16, session="one")[8]
    assert view.update(good, 10.35, session="one")[8]
    assert not view.update(good, 10.37, session="one")[8]
    assert view.update(bad, 11., session="one")[8]
    assert not view.update(good, 11.01, session="new-session")[8]
    with pytest.raises(ValueError):
        JointQualityPresentation(recovery_s=10.)


def test_unavailable_angle_never_renders_placeholder_zero_or_last_angle():
    pose = quality_pose("unavailable")
    before = deepcopy(pose)
    assert joint_angle_text(pose, 8) == "— [不可用]"
    assert joint_angle_text(pose, 7) == "22.9°"
    assert "完整手形仅供显示" in joint_quality_message(pose, [True] * 16)
    assert pose == before
    pose = quality_pose("degraded")
    assert joint_angle_text(pose, 8) == "22.9° [暂估]"
    pose["anatomical_euler_rad"][8][2] = -.8
    assert joint_angle_text(pose, 8) == "-45.8° [暂估]"  # current, not last-good
    assert "不代表有效测量" in joint_quality_message(pose, [True] * 16)


def test_malformed_partial_joint_quality_fails_closed_without_mutation():
    pose = {"joint_valid_mask": [True] * 16}
    before = deepcopy(pose)
    assert joint_quality_state(pose) == ([False] * 16, [False] * 16, ["unavailable"] * 16)
    assert pose == before
    pose = quality_pose()
    pose["joint_valid_mask"][8] = False
    valid, available, quality = joint_quality_state(pose)
    assert not valid[8] and not available[8] and quality[8] == "unavailable"
    assert all(valid[slot] for slot in range(16) if slot != 8)


def test_neutral_segment_assignment_localizes_pip_dip_without_touching_mcp_or_palm():
    joints = np.zeros((21, 3))
    for finger, base in enumerate((1, 5, 9, 13, 17)):
        for offset in range(4):
            joints[base + offset] = [finger * .025 - .05, .04 + offset * .025, 0.]
    # Vertices sit within each little-finger segment, plus palm and index PIP.
    vertices = [(joints[a] + joints[b]) / 2 for a, b in ((0, 17), (17, 18), (18, 19), (19, 20), (6, 7))]
    vertex_slots, joint_slots = neutral_geometry_slot_map(dict(vertices_m=vertices, joints_m=joints))
    assert vertex_slots.tolist() == [0, 7, 8, 9, 2]
    assert joint_slots[[17, 18, 19, 20]].tolist() == [7, 8, 9, 9]
    assert joint_slots[[1, 2, 3, 4]].tolist() == [13, 14, 15, 15]


def test_partial_joint_failure_does_not_hide_remaining_current_hand():
    sample = snapshot()
    sample["pose"].update(quality_pose("unavailable"))
    assert measured_pose_visible(sample, 10.01)


def test_unavailable_segment_hides_immediately_and_cannot_flash_back_on_short_recovery():
    view = JointQualityPresentation(recovery_s=.2)
    bad, usable = quality_pose("unavailable"), quality_pose("degraded")
    view.update(bad, 10., session="one")
    assert view.hidden[8]
    view.update(usable, 10.01, session="one")
    assert view.hidden[8]
    view.update(bad, 10.15, session="one")
    view.update(usable, 10.16, session="one")
    assert view.hidden[8]
    provisional = view.update(usable, 10.37, session="one")
    assert not view.hidden[8] and provisional[8]  # current grey estimate resumes
    assert not any(view.hidden[slot] for slot in range(16) if slot != 8)


def test_downstream_visibility_does_not_let_valid_relative_child_float_after_invalid_mcp():
    mask = [False] * 16
    mask[7] = True
    expanded = downstream_display_mask(mask)
    assert np.flatnonzero(expanded).tolist() == [7, 8, 9]
    assert mask == [slot == 7 for slot in range(16)]
    mask = [slot == 8 for slot in range(16)]
    assert np.flatnonzero(downstream_display_mask(mask)).tolist() == [8, 9]
    mask = [slot == 14 for slot in range(16)]
    assert np.flatnonzero(downstream_display_mask(mask)).tolist() == [14, 15]
