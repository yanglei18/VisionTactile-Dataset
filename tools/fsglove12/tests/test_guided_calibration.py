"""Guided calibration contracts using synthetic rotations, not glove accuracy."""
from copy import deepcopy
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.calibration import apply_calibration
from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.guided_calibration import (
    GuidedCalibrationError, STEPS, fit_guided_calibration, model_motion_axes,
)


KEYS = tuple(step["key"] for step in STEPS)


def sample_joints():
    joints = np.zeros((21, 3))
    joints[9] = [0., .1, 0.]
    joints[5] = [-.03, .085, 0.]
    joints[17] = [.035, .07, 0.]
    return joints


def data(*, rest=None, axes=None, pitch_deg=33., roll_deg=51.):
    config = load_config()
    rng = np.random.default_rng(3012)
    if rest is None:
        rest = {"parents": list(PARENTS),
                "rest_global_xyzw": Rotation.random(16, random_state=rng).as_quat().tolist(),
                "provenance": {"kind": "synthetic_test_only"}}
    axes = model_motion_axes(sample_joints()) if axes is None else axes
    rest_rotations = Rotation.from_quat(rest["rest_global_xyzw"])
    up = np.asarray(axes["palm_up_axis"])
    world_to_model, _ = Rotation.align_vectors([up], [[0., 0., 1.]])
    headings = {node: Rotation.from_rotvec(up * rng.uniform(-np.pi, np.pi)) * world_to_model
                for node in config["node_to_slot"]}
    mounts = {node: Rotation.random(random_state=rng) for node in config["node_to_slot"]}
    roots = Rotation.from_rotvec([[0., 0., 0.],
                                  *(np.asarray(axes[key]) * np.deg2rad(deg)
                                    for key, deg in (("pitch_axis", pitch_deg), ("roll_axis", roll_deg)))])

    def frame(bones, timestamp, sample_index):
        return {"timestamp_s": timestamp, "nodes": {
            node: {"quat_xyzw": (headings[node].inv() * bone * mounts[node].inv()).as_quat().tolist(),
                   "age_s": .01, "sample_index": sample_index, "epoch": int(node) % 2}
            for node, bone in bones.items()}}

    captures = {}
    for index, (key, root) in enumerate(zip(KEYS, roots)):
        bones = {node: root * rest_rotations[slot] for node, slot in config["node_to_slot"].items()}
        captures[key] = [frame(bones, 100. + 3 * index + .25 * i, 8 * index + i + 1) for i in range(8)]
    return config, rest, axes, captures, mounts, headings, roots, frame


def fit(values):
    config, rest, axes, captures = values[:4]
    return fit_guided_calibration(captures, rest, config, session_id="test-current-session", model_axes=axes)


def assert_same_rotation(first, second):
    assert (Rotation.from_quat(first).inv() * Rotation.from_quat(second)).magnitude() < 1e-7


def test_model_axes_and_guidance_have_correct_right_hand_direction():
    axes = model_motion_axes(sample_joints())
    forward, thumbward, up = (np.asarray(axes[key]) for key in ("roll_axis", "pitch_axis", "palm_up_axis"))
    assert np.dot(np.cross(thumbward, forward), up) < -.999
    assert np.dot(np.cross(forward, thumbward), up) > .999
    assert KEYS == ("neutral", "fingers_down", "thumb_up")
    assert all(step["duration_s"] == 2. and step["title"] and step["instruction"] for step in STEPS)


def test_actual_unequal_angles_recover_random_mounts_and_independent_headings():
    values = data()
    config, rest, _, captures, mounts, headings, roots, make_frame = values
    profile = fit(values)
    assert not profile["rejected_nodes"] and not profile["warnings"]
    assert profile["session_id"] == "test-current-session" and "stream_sha256" not in profile
    assert profile["status"] == "provisional_not_accuracy_validated"
    assert not profile["yaw_drift_corrected"] and not profile["anatomical_accuracy_verified"]
    np.testing.assert_allclose(profile["palm_motion"]["rotation_deg"], [33., 51.], atol=1e-10)
    for index, key in enumerate(KEYS):
        assert_same_rotation(profile["pose_root_quat_xyzw"][key], roots[index].as_quat())
    for node, params in profile["nodes"].items():
        assert_same_rotation(params["sensor_from_bone_xyzw"], mounts[node].as_quat())
        assert_same_rotation(params["world_alignment_xyzw"], headings[node].as_quat())
        assert params["frames_per_pose"] == [8, 8, 8]
    # New, not calibrated-on pose: independent model orientations must recover.
    common = Rotation.from_euler("XYZ", [.6, -.3, .7])
    bones = {node: common * Rotation.from_quat(rest["rest_global_xyzw"][slot])
             * Rotation.from_rotvec([0., .03 * slot, .02 * slot])
             for node, slot in config["node_to_slot"].items()}
    observations, rejected = apply_calibration(make_frame(bones, 110., 100), profile)
    assert not rejected
    for node in bones:
        assert_same_rotation(observations[node], bones[node].as_quat())


@pytest.mark.parametrize("pitch,roll,step", [(3., 40., "fingers_down"), (35., 0., "thumb_up")])
def test_insufficient_palm_motion_only_requests_affected_step(pitch, roll, step):
    with pytest.raises(GuidedCalibrationError, match="不足 10") as caught:
        fit(data(pitch_deg=pitch, roll_deg=roll))
    assert caught.value.redo_steps == (step,)


@pytest.mark.parametrize("pitch,roll", [(-35., 40.), (35., -40.)])
def test_one_reversed_motion_has_actionable_gravity_direction_error(pitch, roll):
    with pytest.raises(GuidedCalibrationError, match="方向.*不一致") as caught:
        fit(data(pitch_deg=pitch, roll_deg=roll))
    assert caught.value.redo_steps == ("fingers_down", "thumb_up")


def test_both_reversed_motions_ambiguity_is_not_misrepresented_as_detectable():
    profile = fit(data(pitch_deg=-35., roll_deg=-40.))
    assert any("Reversing both" in text for text in profile["limitations"])
    assert not profile["anatomical_accuracy_verified"]


def test_parallel_palm_motion_is_unobservable():
    values = data()
    captures = values[3]
    for index, frame in enumerate(captures["thumb_up"]):
        for node in frame["nodes"]:
            frame["nodes"][node]["quat_xyzw"] = captures["fingers_down"][index]["nodes"][node]["quat_xyzw"]
    with pytest.raises(GuidedCalibrationError, match="太相近"):
        fit(values)


@pytest.mark.parametrize("kind", ["missing", "stale", "invalid", "epoch", "duplicate"])
def test_bad_finger_is_local_and_never_fabricated(kind):
    values = data()
    for frame in values[3]["thumb_up"]:
        node = frame["nodes"]["3"]
        if kind == "missing":
            del frame["nodes"]["3"]
        elif kind == "stale":
            node["age_s"] = 1.
        elif kind == "invalid":
            node["quat_xyzw"] = [0., 0., 0., 0.]
        elif kind == "epoch":
            node["epoch"] += 1
        else:
            node["sample_index"] = 17
    profile = fit(values)
    assert "3" in profile["rejected_nodes"] and "3" not in profile["nodes"]
    assert "12" in profile["nodes"] and "1" in profile["nodes"] and "11" not in profile["nodes"]


@pytest.mark.parametrize("kind", ["missing", "stale", "duplicate", "no_index", "epoch"])
def test_bad_palm_stops_fit_with_targeted_recapture(kind):
    values = data()
    for frame in values[3]["fingers_down"]:
        node = frame["nodes"]["12"]
        if kind == "missing":
            del frame["nodes"]["12"]
        elif kind == "stale":
            node["age_s"] = 1.
        elif kind == "duplicate":
            node["sample_index"] = 9
        elif kind == "no_index":
            node.pop("sample_index")
        else:
            node["epoch"] += 1
    with pytest.raises(GuidedCalibrationError) as caught:
        fit(values)
    assert caught.value.redo_steps == (KEYS if kind == "epoch" else ("fingers_down",))


def test_duplicate_snapshots_are_counted_once_and_same_host_stamp_is_allowed():
    values = data()
    for key in KEYS:
        # Different sensor samples may share a USB/host timestamp.
        for frame in values[3][key]:
            frame["timestamp_s"] = 100.
        values[3][key] += deepcopy(values[3][key])
    profile = fit(values)
    for node in profile["nodes"]:
        assert profile["nodes"][node]["frames_per_pose"] == [8, 8, 8]
        assert profile["capture_diagnostics"][node]["neutral"]["duplicate"] == 8


def test_recapture_order_does_not_require_neutral_to_be_earliest():
    values = data()
    for frame in values[3]["neutral"]:
        frame["timestamp_s"] += 100.
        for node in frame["nodes"].values():
            node["sample_index"] += 100
    assert not fit(values)["rejected_nodes"]


def test_deviating_finger_motion_warns_instead_of_failing_entire_hand():
    values = data()
    frames = values[3]["thumb_up"]
    for index, frame in enumerate(frames):
        node = frame["nodes"]["5"]
        node["quat_xyzw"] = (Rotation.from_quat(node["quat_xyzw"])
                             * Rotation.from_rotvec([.3 * (-1) ** index, 0., 0.])).as_quat().tolist()
    profile = fit(values)
    assert "5" in profile["nodes"] and any("节点 5" in warning for warning in profile["warnings"])


def test_inputs_unchanged_and_invalid_geometry_is_clear():
    values = data()
    before = deepcopy(values[3])
    fit(values)
    assert values[3] == before
    with pytest.raises(ValueError, match="退化"):
        model_motion_axes(np.zeros((21, 3)))
    with pytest.raises(ValueError, match="21, 3"):
        model_motion_axes(np.zeros((16, 3)))


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_MANO_ASSETS"), reason="No explicit real MANO assets")
def test_real_mano_guided_mounts_recover_nonzero_hand_geometry():
    from vt_fsglove12.mano import ManoBackend
    from vt_fsglove12.solver import SparseHandSolver

    backend = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    rest = backend.export_rest()
    neutral = backend.render(np.zeros((16, 3)))
    axes = model_motion_axes(neutral["joints_m"])
    values = data(rest=rest, axes=axes, pitch_deg=37., roll_deg=46.)
    profile = fit(values)
    assert not profile["rejected_nodes"]
    config, _, _, _, _, _, _, make_frame = values
    angles = np.zeros((16, 3))
    for i, proximal in enumerate((1, 4, 7, 10)):
        angles[proximal, 1:] = [.1 * (-1) ** i, .3 + .1 * i]
        angles[proximal + 1, 2] = .55 + .08 * i
        angles[proximal + 2, 2] = config["dip_ratio"] * angles[proximal + 1, 2]
    angles[14, 1:] = [.15, .5]
    angles[15, 2] = .65
    root = Rotation.from_euler("XYZ", [.4, -.3, .6])
    target = backend.render(angles, root.as_quat())
    with backend._torch.no_grad():
        output = backend._mano(backend._tensor(np.asarray(target["mano_pose_rotvec"]).reshape(1, 48)),
                               backend._tensor(np.zeros((1, 10))))
        transforms = backend._axis(output.transforms_abs)[0].detach().cpu().numpy()
    rotations = Rotation.from_matrix(transforms[0, :, :3, :3])
    bones = {node: rotations[slot] for node, slot in config["node_to_slot"].items()}
    observations, rejected = apply_calibration(make_frame(bones, 110., 100), profile)
    assert not rejected
    result = SparseHandSolver(config, rest["rest_global_xyzw"]).solve(observations, 110.)
    np.testing.assert_allclose(result["anatomical_euler_rad"], angles, atol=5e-5)
    rendered = backend.render(result["anatomical_euler_rad"], result["root_quat_xyzw"])
    np.testing.assert_allclose(rendered["vertices_m"], target["vertices_m"], atol=5e-6)
    np.testing.assert_allclose(rendered["joints_m"], target["joints_m"], atol=5e-6)
    # Model flexion sign: positive middle MCP bending moves the fingertip down.
    flex = np.zeros((16, 3)); flex[4, 2] = .3
    bent = backend.render(flex)
    tip_delta = np.asarray(bent["joints_m"])[12] - np.asarray(neutral["joints_m"])[12]
    assert np.dot(tip_delta, axes["palm_up_axis"]) < 0
