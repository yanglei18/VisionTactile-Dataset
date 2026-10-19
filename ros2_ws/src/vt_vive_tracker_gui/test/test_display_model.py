import math
from dataclasses import FrozenInstanceError

import pytest

from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    PoseValue,
    RoleSnapshot,
    StatusValue,
    VisualHealth,
)
from vt_vive_tracker_gui.display_model import (
    OverallState,
    card_for_snapshot,
    overall_state,
    quaternion_to_rpy_degrees,
)


def roles(*health_values):
    return tuple(
        RoleSnapshot(role, health, None, (), 0.0, None)
        for role, health in zip(FIXED_ROLES, health_values)
    )


def snapshot_with_pose_and_status():
    return RoleSnapshot(
        role="left_wrist",
        health=VisualHealth.FRESH,
        pose=PoseValue(
            position=(1.0, 2.0, 3.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
        trail=(),
        receive_rate_hz=125.0,
        status=StatusValue(
            state=2,
            valid_sample_count=100,
            invalid_report_count=2,
            dropped_queue_count=0,
            tracker_id="a" * 64,
        ),
        pose_age_ns=12_500_000,
    )


def no_pose_snapshot():
    return RoleSnapshot(
        role="right_wrist",
        health=VisualHealth.OFFLINE,
        pose=None,
        trail=(),
        receive_rate_hz=0.0,
        status=None,
        pose_age_ns=None,
    )


def test_overall_state_requires_all_three_fresh_for_live():
    assert (
        overall_state(
            roles(
                VisualHealth.FRESH,
                VisualHealth.FRESH,
                VisualHealth.FRESH,
            )
        )
        is OverallState.LIVE
    )
    assert (
        overall_state(
            roles(
                VisualHealth.FRESH,
                VisualHealth.DELAYED,
                VisualHealth.FRESH,
            )
        )
        is OverallState.DEGRADED
    )
    assert (
        overall_state(
            roles(
                VisualHealth.OFFLINE,
                VisualHealth.OFFLINE,
                VisualHealth.OFFLINE,
            )
        )
        is OverallState.DISCONNECTED
    )


def test_overall_state_rejects_non_fixed_role_tuple():
    with pytest.raises(ValueError, match="fixed role order"):
        overall_state(roles(VisualHealth.FRESH, VisualHealth.FRESH))

    reversed_roles = tuple(
        reversed(
            roles(
                VisualHealth.FRESH,
                VisualHealth.FRESH,
                VisualHealth.FRESH,
            )
        )
    )
    with pytest.raises(ValueError, match="fixed role order"):
        overall_state(reversed_roles)


def test_identity_quaternion_formats_zero_rpy_and_status_metadata():
    card = card_for_snapshot(snapshot_with_pose_and_status())

    assert card.position == ("1.0000", "2.0000", "3.0000")
    assert card.quaternion == (
        "0.00000",
        "0.00000",
        "0.00000",
        "1.00000",
    )
    assert card.rpy_degrees == ("0.00°", "0.00°", "0.00°")
    assert card.tracker_id == "a" * 64
    assert card.rate == "125.0 Hz"
    assert card.age == "12.5 ms"
    assert card.counters == "valid 100 · invalid 2 · dropped 0"


def test_no_pose_uses_dashes_instead_of_zeroes():
    card = card_for_snapshot(no_pose_snapshot())

    assert card.position == ("—", "—", "—")
    assert card.quaternion == ("—", "—", "—", "—")
    assert card.rpy_degrees == ("—", "—", "—")
    assert card.age == "—"


def test_card_model_is_immutable():
    card = card_for_snapshot(snapshot_with_pose_and_status())

    with pytest.raises(FrozenInstanceError):
        card.rate = "0.0 Hz"


def test_non_unit_quaternion_is_normalized_before_rpy_conversion():
    scale = math.sqrt(2.0)

    assert quaternion_to_rpy_degrees(
        (0.0, 0.0, scale, scale)
    ) == pytest.approx(
        (0.0, 0.0, 90.0),
    )


def test_large_finite_quaternion_is_normalized_without_overflow():
    assert quaternion_to_rpy_degrees(
        (1e308, 0.0, 0.0, 1e308)
    ) == pytest.approx(
        (90.0, 0.0, 0.0),
    )


def test_tiny_finite_quaternion_is_normalized_without_underflow():
    assert quaternion_to_rpy_degrees(
        (1e-300, 0.0, 0.0, 1e-300)
    ) == pytest.approx(
        (90.0, 0.0, 0.0),
    )


@pytest.mark.parametrize(
    "quaternion",
    [
        (0.0, 0.0, 0.0, 0.0),
        (math.inf, 0.0, 0.0, 1.0),
        (0.0, math.nan, 0.0, 1.0),
    ],
)
def test_invalid_quaternion_is_rejected(quaternion):
    with pytest.raises(ValueError, match="quaternion"):
        quaternion_to_rpy_degrees(quaternion)
