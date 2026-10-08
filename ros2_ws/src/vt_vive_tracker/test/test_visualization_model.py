import math

import pytest

from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    PoseValue,
    TrackerVisualizationModel,
    VisualHealth,
)


def pose(x=1.0):
    return PoseValue(
        (x, 2.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def by_role(model, now_ns):
    return {item.role: item for item in model.snapshots(now_ns)}


def test_fixed_roles_start_offline_without_a_pose_or_trail():
    model = TrackerVisualizationModel()

    values = by_role(model, 0)

    assert tuple(values) == FIXED_ROLES
    assert all(
        value.health is VisualHealth.OFFLINE
        for value in values.values()
    )
    assert all(
        value.pose is None and value.trail == ()
        for value in values.values()
    )


def test_monotonic_freshness_boundaries_and_disconnected_precedence():
    model = TrackerVisualizationModel()
    model.observe_status("left_wrist", 2, 10, 1, 0)
    model.observe_pose(
        "left_wrist", pose(), arrival_ns=1_000_000_000
    )

    assert (
        by_role(model, 1_250_000_000)["left_wrist"].health
        is VisualHealth.FRESH
    )
    assert (
        by_role(model, 1_250_000_001)["left_wrist"].health
        is VisualHealth.DELAYED
    )
    assert (
        by_role(model, 2_000_000_000)["left_wrist"].health
        is VisualHealth.DELAYED
    )
    assert (
        by_role(model, 2_000_000_001)["left_wrist"].health
        is VisualHealth.OFFLINE
    )

    model.observe_status("left_wrist", 0, 10, 1, 0)

    assert (
        by_role(model, 1_100_000_000)["left_wrist"].health
        is VisualHealth.OFFLINE
    )


@pytest.mark.parametrize("state", (1, 3))
def test_nontracking_and_invalid_status_prevent_fresh_state(state):
    model = TrackerVisualizationModel()
    model.observe_status("torso", state, 7, 2, 1)
    model.observe_pose("torso", pose(), arrival_ns=1_000_000_000)

    assert (
        by_role(model, 1_100_000_000)["torso"].health
        is VisualHealth.DELAYED
    )


def test_trail_and_rate_histories_are_time_and_count_bounded():
    model = TrackerVisualizationModel(max_trail_points=3)
    model.observe_status("right_wrist", 2, 4, 0, 0)
    for index, now_ns in enumerate(
        (0, 1_000_000_000, 2_000_000_000, 3_000_000_000)
    ):
        model.observe_pose(
            "right_wrist", pose(float(index)), arrival_ns=now_ns
        )

    value = by_role(model, 3_000_000_000)["right_wrist"]

    assert tuple(item.position[0] for item in value.trail) == (
        1.0,
        2.0,
        3.0,
    )
    assert value.receive_rate_hz == 2.0


def test_unknown_roles_and_nonmonotonic_arrivals_are_rejected():
    model = TrackerVisualizationModel()

    assert model.observe_pose("client0", pose(), arrival_ns=1) is False
    assert model.observe_status("client0", 2, 0, 0, 0) is False
    assert model.observe_pose("torso", pose(), arrival_ns=2) is True
    assert model.observe_pose("torso", pose(), arrival_ns=1) is False


@pytest.mark.parametrize(
    "value",
    (
        PoseValue((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        PoseValue((0.0, 0.0, 0.0), (0.0, 0.0, math.inf, 1.0)),
    ),
)
def test_nonfinite_poses_are_rejected(value):
    model = TrackerVisualizationModel()

    assert model.observe_pose("left_wrist", value, arrival_ns=1) is False


@pytest.mark.parametrize(
    "arguments",
    (
        (4, 0, 0, 0),
        (2, -1, 0, 0),
        (2, 0, -1, 0),
        (2, 0, 0, -1),
        (True, 0, 0, 0),
    ),
)
def test_invalid_status_values_are_rejected(arguments):
    model = TrackerVisualizationModel()

    assert model.observe_status("left_wrist", *arguments) is False


@pytest.mark.parametrize(
    "metadata",
    (
        {"tracker_id": 64},
        {"tracking_status": True},
        {"tracking_status": -1},
        {"tracking_status": 256},
    ),
)
def test_invalid_status_metadata_is_rejected(metadata):
    model = TrackerVisualizationModel()

    assert (
        model.observe_status("left_wrist", 2, 0, 0, 0, **metadata)
        is False
    )


def test_snapshot_exposes_pose_age_and_full_status_metadata():
    model = TrackerVisualizationModel()
    assert model.observe_status(
        "left_wrist",
        2,
        12,
        3,
        1,
        tracker_id="a" * 64,
        tracking_status=4,
    )
    model.observe_pose("left_wrist", pose(), arrival_ns=1_000_000_000)

    value = by_role(model, 1_125_000_000)["left_wrist"]

    assert value.pose_age_ns == 125_000_000
    assert value.status.tracker_id == "a" * 64
    assert value.status.tracking_status == 4


def test_no_pose_has_no_age_and_legacy_status_call_still_works():
    model = TrackerVisualizationModel()
    assert model.observe_status("torso", 2, 1, 0, 0)

    value = by_role(model, 0)["torso"]

    assert value.pose_age_ns is None
    assert value.status.tracker_id == ""
    assert value.status.tracking_status == 0


def test_injected_monotonic_clock_is_used_when_snapshot_time_is_omitted():
    model = TrackerVisualizationModel(monotonic_ns=lambda: 1_100_000_000)
    model.observe_status("torso", 2, 1, 0, 0)
    model.observe_pose("torso", pose(), arrival_ns=1_000_000_000)

    assert model.snapshots()[2].health is VisualHealth.FRESH
