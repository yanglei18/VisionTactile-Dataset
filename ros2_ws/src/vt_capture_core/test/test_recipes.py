from itertools import product

import pytest

from vt_capture_core.recipes import (
    CAMERA_IDS, TRACKER_IDS, Recipe, RecipeError, validate_recipe,
)


@pytest.mark.parametrize("left_pose,right_pose,left_tactile,right_tactile", list(product([False, True], repeat=4)))
def test_recording_all_hand_combinations(left_pose, right_pose, left_tactile, right_tactile):
    pose = tuple(hand for hand, enabled in (("left", left_pose), ("right", right_pose)) if enabled)
    tactile = tuple(hand for hand, enabled in (("left", left_tactile), ("right", right_tactile)) if enabled)
    recipe = Recipe("Trial", pose_hands=pose, tactile_hands=tactile)
    assert validate_recipe(recipe.to_dict()) == recipe
    topics = recipe.record_topics()
    assert len(topics) == 21 + len(pose) + len(tactile)
    assert len(topics) == len(set(topics))
    for hand in ("left", "right"):
        assert (f"/gloves/{hand}/state" in topics) == (hand in pose)
        assert (f"/gloves/{hand}/tactile/samples" in topics) == (hand in tactile)
    assert len(recipe.device_ids) == 6 + len(pose) + len(tactile)


def test_recording_contract_topics():
    assert Recipe("base").record_topics() == (
        "/d405_1/color/image_raw", "/d405_1/depth/image_rect_raw",
        "/d405_1/color/camera_info", "/d405_1/frame_timing",
        "/d405_1/depth/camera_info", "/d405_1/extrinsics/depth_to_color",
        "/d405_2/color/image_raw", "/d405_2/depth/image_rect_raw",
        "/d405_2/color/camera_info", "/d405_2/frame_timing",
        "/d405_2/depth/camera_info", "/d405_2/extrinsics/depth_to_color",
        "/d436/color/image_raw", "/d436/depth/image_rect_raw",
        "/d436/color/camera_info", "/d436/frame_timing",
        "/d436/depth/camera_info", "/d436/extrinsics/depth_to_color",
        "/vive/left_wrist/sample", "/vive/right_wrist/sample", "/vive/torso/sample",
    )


@pytest.mark.parametrize("cameras,trackers,pose,tactile", [
    ((), (), (), ()), (("d436",), (), (), ()),
    ((), ("torso",), (), ()), ((), (), ("left",), ()),
    ((), (), (), ("right",)), (("d405_2",), ("right_wrist",), ("right",), ("left",)),
])
def test_arbitrary_preview_subsets(cameras, trackers, pose, tactile):
    recipe = Recipe("Preview", mode="preview", duration_s=0,
                    cameras=cameras, trackers=trackers, pose_hands=pose, tactile_hands=tactile)
    assert validate_recipe(recipe.to_dict()) == recipe
    assert len(recipe.device_ids) == sum(map(len, (cameras, trackers, pose, tactile)))
    with pytest.raises(RecipeError, match="preview-only"):
        recipe.record_topics()


@pytest.mark.parametrize("field,value", [
    ("version", True), ("version", 3), ("name", ""), ("name", 9),
    ("mode", "unknown"), ("duration_s", 0), ("duration_s", 300.01),
    ("duration_s", True), ("duration_s", float("nan")),
    ("duration_s", float("inf")), ("duration_s", -1),
    ("cameras", ["d405_1", "d405_2", "unrecognized"]),
    ("pose_hands", ["left", "left"]), ("tactile_hands", ["both"]),
    ("pose_hands", "left"), ("tactile_hands", [1]),
])
def test_invalid_recipes(field, value):
    document = Recipe("Test").to_dict()
    document[field] = value
    with pytest.raises(RecipeError):
        validate_recipe(document)


def test_schema_explicit_unknown_and_missing_fields():
    document = Recipe("Test").to_dict()
    document["unknown"] = True
    with pytest.raises(RecipeError):
        validate_recipe(document)
    del document["unknown"]
    del document["tactile_hands"]
    with pytest.raises(RecipeError):
        validate_recipe(document)


def test_normalization_does_not_mutate_input():
    document = Recipe("Test").to_dict()
    document["cameras"].reverse()
    document["trackers"].reverse()
    document["pose_hands"] = ["right", "left"]
    recipe = validate_recipe(document)
    assert recipe.cameras == CAMERA_IDS
    assert recipe.trackers == TRACKER_IDS
    assert recipe.pose_hands == ("left", "right")
    assert document["pose_hands"] == ["right", "left"]


def test_preview_cannot_claim_recording_duration():
    with pytest.raises(RecipeError, match="preview-only duration"):
        Recipe("Preview", mode="preview").to_dict()


@pytest.mark.parametrize("duration", [1, 1.5, 299.9, 300])
def test_record_duration_boundaries(duration):
    assert Recipe("Trial", duration_s=duration).to_dict()["duration_s"] == duration


@pytest.mark.parametrize("cameras,trackers,pose,tactile", [
    (("d436",), (), (), ()), ((), ("torso",), (), ()),
    ((), (), ("left",), ()), ((), (), (), ("right",)),
    (("d405_1", "d405_2"), ("left_wrist",), ("right",), ("left",)),
])
def test_recording_accepts_any_nonempty_device_selection(cameras, trackers, pose, tactile):
    recipe = Recipe("Available devices", cameras=cameras, trackers=trackers,
                    pose_hands=pose, tactile_hands=tactile)
    assert validate_recipe(recipe.to_dict()) == recipe
    assert len(recipe.record_topics()) == 6 * len(cameras) + len(trackers) + len(pose) + len(tactile)


def test_recording_rejects_empty_selection():
    with pytest.raises(RecipeError, match="at least one"):
        Recipe("Empty", cameras=(), trackers=()).to_dict()


def test_recipe_version_controls_camera_calibration_bundle_without_rewriting_history():
    legacy = Recipe("Historical", version=1)
    current = Recipe("Current")
    assert current.version == 2
    assert validate_recipe(legacy.to_dict()) == legacy
    assert len(legacy.record_topics()) == 15
    assert len(current.record_topics()) == 21
    assert "/d436/depth/camera_info" not in legacy.record_topics()
    assert "/d436/extrinsics/depth_to_color" not in legacy.record_topics()
    assert "/d436/depth/camera_info" in current.record_topics()
    assert "/d436/extrinsics/depth_to_color" in current.record_topics()
