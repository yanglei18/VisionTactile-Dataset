"""Keep pure-core intent aligned with the Recorder's existing bag contract."""

from itertools import product

import pytest

from vt_capture_core.capture_plan import recipe_for_topics, resolve_capture_plan
from vt_capture_core.recipes import CAMERA_IDS, Recipe
from vt_realsense_capture.bag_contract import (
    SCHEMA_VERSION, expected_topic_types, expected_topics,
)


@pytest.mark.parametrize("enabled", list(product([False, True], repeat=4)))
def test_resolved_plan_matches_recorder_contract_for_every_hand_combination(enabled):
    pose = tuple(hand for hand, selected in zip(("left", "right"), enabled[:2]) if selected)
    tactile = tuple(hand for hand, selected in zip(("left", "right"), enabled[2:]) if selected)
    recipe = Recipe("Contract", pose_hands=pose, tactile_hands=tactile)
    additional_types = {
        **{f"/gloves/{hand}/state": "vt_glove_msgs/msg/GloveState" for hand in pose},
        **{f"/gloves/{hand}/tactile/samples": "vt_tactile_msgs/msg/TactileSampleArray" for hand in tactile},
    }
    recorder_topics = expected_topics(CAMERA_IDS, tuple(additional_types))
    plan = resolve_capture_plan(recipe)

    assert plan.schema_version == SCHEMA_VERSION
    assert plan.record_topics() == recorder_topics
    assert plan.topic_types == expected_topic_types(CAMERA_IDS, additional_types)
    assert recipe_for_topics(recorder_topics, name=recipe.name, duration_s=recipe.duration_s) == recipe


def test_default_recorder_and_plan_include_factory_calibration_per_camera():
    plan = resolve_capture_plan(Recipe("Factory calibration", trackers=()))
    assert plan.schema_version == SCHEMA_VERSION == "unified-dataset-v2"
    assert len(plan.streams) == 18
    for camera in CAMERA_IDS:
        assert plan.topic_types[f"/{camera}/depth/camera_info"] == "sensor_msgs/msg/CameraInfo"
        assert plan.topic_types[f"/{camera}/extrinsics/depth_to_color"] == "realsense2_camera_msgs/msg/Extrinsics"
    assert not plan.required_topics()
