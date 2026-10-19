from copy import deepcopy

import numpy as np
import pytest

from vt_fsglove12.display_pose import DisplayPoseCache


def pose(stamp=10.):
    return dict(timestamp_s=stamp, pose_available=True, solver_success=True,
                mano_pose_rotvec=np.full((16, 3), .2).tolist(),
                anatomical_euler_rad=np.full((16, 3), .1).tolist(),
                joint_valid_mask=[True] * 16, joint_display_available_mask=[True] * 16,
                joint_quality=["valid"] * 16, shape_params=[.3] * 10,
                session_id="one", frame_index=1, profile_version=4)


def quality(p, slot, state):
    p["joint_valid_mask"][slot] = state == "valid"
    p["joint_display_available_mask"][slot] = state != "unavailable"
    p["joint_quality"][slot] = state


def test_source_not_mutated_or_promoted_to_measurement():
    original = pose()
    expected = deepcopy(original)
    shown = DisplayPoseCache().update(original, context=("one", 4), now=10.2)
    assert original == expected
    assert shown["display_only"] and not shown["measurement_valid"]
    assert shown["source_timestamp_s"] == 10.
    assert shown["source_age_by_slot"]["8"] == pytest.approx(.2)
    assert shown["mano_pose_rotvec"] == original["mano_pose_rotvec"]
    shown["mano_pose_rotvec"][8][0] = 5.
    shown["shape_params"][0] = 9.
    assert original == expected


def test_missing_joint_holds_local_rotation_while_current_parent_and_root_move():
    cache = DisplayPoseCache()
    cache.update(pose(), context="same", now=10.)
    newer = pose(11.)
    newer["mano_pose_rotvec"][0] = [.5, .1, .8]
    newer["mano_pose_rotvec"][7] = [.7, .3, .2]
    quality(newer, 8, "unavailable")
    newer["mano_pose_rotvec"][8] = [0., 0., 0.]
    original = deepcopy(newer)
    shown = cache.update(newer, context="same", now=11.2)
    assert shown["mano_pose_rotvec"][8] == [.2] * 3
    for slot in (0, 7):
        assert shown["mano_pose_rotvec"][slot] == newer["mano_pose_rotvec"][slot]
    assert shown["held_slots"] == [8]
    assert shown["source_age_by_slot"]["8"] == pytest.approx(1.2)
    assert shown["source_joint_valid_mask"][8] is False
    assert newer == original


def test_degraded_to_unavailable_does_not_jump_back_to_older_valid_angle():
    cache = DisplayPoseCache()
    cache.update(pose(), context="same", now=10.)
    provisional = pose(11.)
    quality(provisional, 8, "degraded")
    provisional["mano_pose_rotvec"][8] = [.8, .1, .2]
    a = cache.update(provisional, context="same", now=11.)
    unavailable = pose(12.)
    quality(unavailable, 8, "unavailable")
    b = cache.update(unavailable, context="same", now=12.)
    assert b["mano_pose_rotvec"][8] == a["mano_pose_rotvec"][8]
    assert b["held_source_quality_by_slot"] == {"8": "degraded"}
    assert b["source_timestamp_by_slot"]["8"] == 11.


@pytest.mark.parametrize("changed", [("other_session", 4), ("one", 5)])
def test_context_change_cannot_reuse_old_local_rotations(changed):
    cache = DisplayPoseCache()
    cache.update(pose(), context=("one", 4), now=10.)
    missing = pose(11.)
    quality(missing, 8, "unavailable")
    shown = cache.update(missing, context=changed, now=11.)
    assert shown["held_slots"] == []
    assert shown["unknown_slots"] == [8]
    assert shown["source_timestamp_by_slot"]["8"] is None
    assert shown["mano_pose_rotvec"][8] == [0.] * 3


def test_explicit_clear_and_source_time_reversal_discard_history():
    for reset in (True, False):
        cache = DisplayPoseCache()
        cache.update(pose(10.), context="one", now=10.)
        if reset:
            cache.reset()
        missing = pose(9. if not reset else 11.)
        quality(missing, 8, "unavailable")
        shown = cache.update(missing, context="one", now=12.)
        assert shown["unknown_slots"] == [8]


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, None])
def test_invalid_time_does_not_create_display_pose(value):
    p = pose()
    p["timestamp_s"] = value
    assert DisplayPoseCache().update(p, context="one", now=12.) is None


def test_future_time_and_invalid_root_do_not_replace_old_display():
    cache = DisplayPoseCache()
    cache.update(pose(), context="one", now=10.)
    assert cache.update(pose(12.), context="one", now=11.) is None
    missing_root = pose(11.)
    quality(missing_root, 0, "unavailable")
    assert cache.update(missing_root, context="one", now=11.) is None
    missing_joint = pose(12.)
    quality(missing_joint, 8, "unavailable")
    assert cache.update(missing_joint, context="one", now=12.)["source_timestamp_by_slot"]["8"] == 10.


@pytest.mark.parametrize("field", ["mano_pose_rotvec", "anatomical_euler_rad"])
def test_nan_frame_is_rejected_atomically(field):
    cache = DisplayPoseCache()
    cache.update(pose(), context="one", now=10.)
    bad = pose(11.)
    bad["mano_pose_rotvec"][8] = [.9] * 3
    bad[field][15][1] = float("nan")
    assert cache.update(bad, context="one", now=11.) is None
    missing = pose(12.)
    quality(missing, 8, "unavailable")
    shown = cache.update(missing, context="one", now=12.)
    assert shown["mano_pose_rotvec"][8] == [.2] * 3
    assert shown["source_timestamp_by_slot"]["8"] == 10.


def test_entire_unavailable_pose_does_not_invent_root_or_time():
    cache = DisplayPoseCache()
    p = pose()
    p["pose_available"] = False
    assert cache.update(p, context="one", now=10.) is None
    assert cache.update(None, context="two", now=10.) is None


def test_duplicate_source_frames_cannot_refresh_held_age():
    cache = DisplayPoseCache()
    cache.update(pose(), context="one", now=10.)
    cache.update(pose(), context="one", now=11.)
    missing = pose(12.)
    quality(missing, 8, "unavailable")
    shown = cache.update(missing, context="one", now=12.)
    assert shown["source_age_by_slot"]["8"] == 2.
