from concurrent.futures import ThreadPoolExecutor
import json
from math import sqrt
from threading import Barrier

import numpy as np
import pytest

from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_manager.optical.hand_fusion import read_mounts
from vt_capture_manager.optical.mount_calibration import fit_mount, save_mount


LAYOUT = "a" * 64
REST = "b" * 64
Q = sqrt(.5)
CUBE = RigidTransform((.09, -.045, .40), (Q, 0., 0., Q))
WRIST = RigidTransform((.09, .025, .42), (0., 0., Q, Q))


def landmarks(hand):
    thumb_side = -1 if hand == "right" else 1
    observed = np.array([WRIST.translation_m, [.09+.022*thumb_side, -.061, .419],
                         [.09-.033*thumb_side, -.048, .415]])
    # Arbitrary translated neutral wrist and different reference axes from both
    # camera and cube; simulated coordinates are in the same units as MANO.
    rest_origin = np.array([.11, -.023, -.037])
    joints = np.zeros((21, 3))
    for idx, camera_point in zip((0, 5, 17), observed):
        joints[idx] = rest_origin + WRIST.inverse().apply(camera_point)
    return joints, observed


@pytest.mark.parametrize("hand", ["left", "right"])
def test_recovers_pose_in_actual_neutral_reference_and_preserves_scale(hand):
    joints, observed = landmarks(hand)
    # A human hand can be larger than the registered neutral model.
    measured = observed[0] + 1.18 * (observed - observed[0])
    mounted = fit_mount(hand, joints, measured, CUBE)
    wrist = CUBE.compose(mounted)
    assert wrist.translation_m == pytest.approx(WRIST.translation_m)
    for point in [(0., 0., 1.), *joints[[5, 17]]]:
        assert wrist.apply(point) == pytest.approx(WRIST.apply(point))
    assert wrist.apply(joints[5] - joints[0]) == pytest.approx(observed[1])


def test_slot_order_and_facing_wrong_hand_are_rejected_at_setup_only():
    joints, observed = landmarks("right")
    for hand, selected in [("left", observed), ("right", observed[[0, 2, 1]])]:
        with pytest.raises(ValueError, match="手背朝向相机"):
            fit_mount(hand, joints, selected, CUBE)


@pytest.mark.parametrize("fault", ["collinear", "coincident", "background", "other_cube", "nan", "behind"])
def test_bad_landmarks_cannot_emit_a_mount(fault):
    joints, observed = landmarks("right")
    if fault == "collinear":
        observed[2] = observed[0]*.4 + observed[1]*.6
    elif fault == "coincident":
        observed[2] = observed[1]
    elif fault == "background":
        observed[2, 2] += .4
    elif fault == "other_cube":
        observed[:, 0] += .30
    elif fault == "nan":
        joints[17, 0] = np.nan
    else:
        observed[1, 2] = -.01
    with pytest.raises(ValueError):
        fit_mount("right", joints, observed, CUBE)


def test_concurrent_first_saves_and_model_hash_round_trip(tmp_path):
    dest = tmp_path / "mounts" / "default.json"
    start = Barrier(2)

    def update(hand):
        model, points = landmarks(hand)
        transform = fit_mount(hand, model, points, CUBE)
        start.wait(timeout=3)
        save_mount(dest, hand, LAYOUT, transform, model_rest_sha256=REST)

    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(update, ("left", "right")))
    assert set(read_mounts(dest)) == {"left", "right"}
    document = json.loads(dest.read_bytes())
    assert all(value["model_rest_sha256"] == REST for value in document["hands"].values())
    previous_right = document["hands"]["right"]
    save_mount(dest, "left", LAYOUT, RigidTransform.identity())
    retained = json.loads(dest.read_bytes())["hands"]
    assert retained["right"] == previous_right
    assert "model_rest_sha256" not in retained["left"]
    assert read_mounts(dest)["left"].cube_from_wrist == RigidTransform.identity()


def test_rejected_save_preserves_last_good_write_and_keeps_invalid_files_intact(tmp_path):
    dest = tmp_path / "mount.json"
    save_mount(dest, "left", LAYOUT, RigidTransform.identity())
    good = dest.read_bytes()
    with pytest.raises(ValueError, match="参数不完整"):
        save_mount(dest, "right", "invalid", RigidTransform.identity())
    assert dest.read_bytes() == good
    dest.write_bytes(b"broken private calibration")
    with pytest.raises(ValueError, match="已有安装参数文件无效"):
        save_mount(dest, "right", LAYOUT, RigidTransform.identity())
    assert dest.read_bytes() == b"broken private calibration"
