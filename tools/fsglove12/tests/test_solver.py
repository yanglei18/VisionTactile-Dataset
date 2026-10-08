"""Synthetic model-contract checks, not evidence of real glove accuracy."""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.calibration import apply_calibration, calibrate
from vt_fsglove12.config import load_config
from vt_fsglove12.solver import PARENTS, SparseHandSolver


MAPPING = {"1": 4, "2": 5, "3": 1, "4": 2, "5": 14, "6": 15,
           "7": 10, "8": 11, "9": 7, "10": 8, "12": 0}


def make_solver(*, cmc=False):
    # Nonidentity root and heterogeneous rest axes detect incorrect world/local
    # conjugation that an all-identity template would hide.
    rest = Rotation.from_euler("XYZ", np.random.default_rng(12).uniform(-.25, .25, (16, 3))).as_quat()
    config = dict(node_to_slot=dict(MAPPING), dip_ratio=.65)
    if cmc:
        config["node_to_slot"]["11"] = 13
    return SparseHandSolver(config, rest), config, rest


def synthesize(config, rest, angles, common=None):
    rest_rotations = Rotation.from_quat(rest)
    root = rest_rotations[0] if common is None else common * rest_rotations[0]
    global_rotations = [root]
    for slot in range(1, 16):
        rest_local = rest_rotations[PARENTS[slot]].inv() * rest_rotations[slot]
        global_rotations.append(global_rotations[PARENTS[slot]] * rest_local * Rotation.from_euler("XYZ", angles[slot]))
    return {node: global_rotations[slot].as_quat() for node, slot in config["node_to_slot"].items()}


def assert_rotation_equal(a, b):
    assert (Rotation.from_quat(a).inv() * Rotation.from_quat(b)).magnitude() < 1e-7


def test_common_rotation_preserves_hand_and_root_is_mano_delta():
    solver, config, rest = make_solver()
    angles = np.zeros((16, 3)); angles[1, 1:] = [.15, .4]; angles[2, 2] = .7
    angles[3, 2] = .65 * .7; angles[14, 2] = .3; angles[15, 2] = .6
    common = Rotation.from_euler("XYZ", [.8, -.4, .5])
    a = solver.solve(synthesize(config, rest, angles), 1.)
    b = solver.solve(synthesize(config, rest, angles, common), 2.)
    np.testing.assert_allclose(a["anatomical_euler_rad"], b["anatomical_euler_rad"], atol=1e-6)
    np.testing.assert_allclose(b["anatomical_euler_rad"], angles, atol=1e-6)
    assert_rotation_equal(a["root_quat_xyzw"], [0, 0, 0, 1])
    assert_rotation_equal(b["root_quat_xyzw"], common.as_quat())


@pytest.mark.parametrize("proximal,middle,distal", [(1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12)])
def test_independent_finger_and_exact_inferred_dip(proximal, middle, distal):
    solver, config, rest = make_solver()
    angles = np.zeros((16, 3)); angles[proximal, 1:] = [-.1, .35]
    angles[middle, 2] = .8; angles[distal, 2] = .52
    result = solver.solve(synthesize(config, rest, angles), 1.)
    np.testing.assert_allclose(result["anatomical_euler_rad"], angles, atol=1e-6)
    assert result["inferred_mask"][distal]
    assert not result["segment_observed_mask"][distal]
    assert result["source_labels"][distal] == "inferred_dip_ratio"
    assert result["residual_deg_by_slot"][str(distal)] is None


def test_two_thumb_sensors_do_not_claim_measured_cmc_mcp_split():
    solver, config, rest = make_solver()
    angles = np.zeros((16, 3)); angles[14, 1:] = [.2, .45]; angles[15, 2] = .7
    result = solver.solve(synthesize(config, rest, angles), 1.)
    np.testing.assert_allclose(result["anatomical_euler_rad"], angles, atol=1e-6)
    assert result["segment_observed_mask"][14]
    assert not result["joint_observed_mask"][14] and result["inferred_mask"][14]
    assert result["joint_observed_mask"][15] and not result["inferred_mask"][15]
    assert result["source_labels"][13] == "inferred_thumb_cmc_prior"
    assert result["source_labels"][14] == "inferred_parent_decomposition"


def test_explicit_auxiliary_cmc_mapping_is_optional():
    solver, config, rest = make_solver(cmc=True)
    angles = np.zeros((16, 3)); angles[13] = [.15, -.2, .25]; angles[14, 2] = .4
    result = solver.solve(synthesize(config, rest, angles), 1.)
    np.testing.assert_allclose(result["anatomical_euler_rad"], angles, atol=1e-6)
    assert all(result["joint_observed_mask"][i] for i in (13, 14, 15))


def test_missing_segment_does_not_become_identity_or_invalidate_other_finger():
    solver, config, rest = make_solver()
    angles = np.zeros((16, 3)); angles[1, 2] = .3; angles[2, 2] = .5
    angles[4, 2] = .4; angles[5, 2] = .6
    observations = synthesize(config, rest, angles)
    full = solver.solve(observations, 1.)
    del observations["3"]
    result = solver.solve(observations, 2.)
    assert result["quality"] == "experimental_partial"
    assert not result["observed_mask"][1]
    assert not result["joint_observed_mask"][2]
    assert result["inferred_mask"][1] and result["inferred_mask"][2]
    assert result["residual_deg_by_slot"]["1"] is None
    np.testing.assert_allclose(np.asarray(full["anatomical_euler_rad"])[4:7],
                               np.asarray(result["anatomical_euler_rad"])[4:7], atol=1e-6)


@pytest.mark.parametrize("bad", [[0., 0., 0., 0.], [0., float("nan"), 0., 1.],
                                 [0., 0., 0., float("inf")], [0., 0., 1.],
                                 [0., 0., 0., 2.], [0., 0., 0., 1.1],
                                 [1e308, 0., 0., 1e308], None])
def test_invalid_quaternion_is_rejected_and_root_failure_is_nonfatal(bad):
    solver, config, rest = make_solver()
    observations = synthesize(config, rest, np.zeros((16, 3)))
    observations["3"] = bad
    result = solver.solve(observations, 1.)
    assert "3" in result["rejected_observations"] and not result["observed_mask"][1]
    observations["12"] = bad
    unavailable = solver.solve(observations, 2.)
    assert unavailable["quality"] == "unavailable"
    assert unavailable["root_quat_xyzw"] is None and unavailable["anatomical_euler_rad"] is None
    assert not unavailable["pose_available"]
    assert not any(unavailable["joint_observed_mask"])
    assert not any(unavailable["inferred_mask"])
    assert unavailable["segment_observed_mask"][4]  # Raw input presence is distinct from a valid hand pose.


def test_small_norm_roundoff_is_normalized_but_gross_rest_scaling_is_rejected():
    solver, config, rest = make_solver()
    angles = np.zeros((16, 3)); angles[1, 2] = .4; angles[2, 2] = .6
    observations = synthesize(config, rest, angles)
    result = solver.solve(observations, 1.)
    rounded = {node: q * (1.005 if int(node) % 2 else .995) for node, q in observations.items()}
    other = solver.solve(rounded, 2.)
    assert other["pose_available"] and not other["rejected_observations"]
    np.testing.assert_allclose(other["anatomical_euler_rad"], result["anatomical_euler_rad"], atol=1e-7)
    with pytest.raises(ValueError, match="norm"):
        SparseHandSolver(config, rest * 2.)


def test_missing_root_and_no_lag_after_bend_and_return():
    solver, config, rest = make_solver()
    neutral = np.zeros((16, 3)); bent = neutral.copy(); bent[1, 2] = .8; bent[2, 2] = 1.1
    solver.solve(synthesize(config, rest, bent), 1.)
    result = solver.solve(synthesize(config, rest, neutral), 2.)
    np.testing.assert_allclose(result["anatomical_euler_rad"], neutral, atol=1e-6)
    assert not result["anatomical_accuracy_verified"] and not result["yaw_drift_corrected"]
    observations = synthesize(config, rest, neutral); del observations["12"]
    assert solver.solve(observations, 3.)["quality"] == "unavailable"


def test_inputs_are_not_mutated_and_configuration_is_validated():
    solver, config, rest = make_solver()
    observations = synthesize(config, rest, np.zeros((16, 3)))
    original = deepcopy(observations)
    solver.solve(observations, 1.)
    for node in observations:
        np.testing.assert_array_equal(observations[node], original[node])
    with pytest.raises(ValueError):
        solver.solve(observations, float("nan"))
    with pytest.raises(ValueError):
        SparseHandSolver(dict(config, dip_ratio=float("nan")), rest)
    with pytest.raises(ValueError):
        SparseHandSolver(dict(config, node_to_slot=dict(MAPPING, **{"11": 6})), rest)
    with pytest.raises(ValueError):
        SparseHandSolver(config, np.zeros((16, 4)))


def test_labelled_calibration_to_full_hand_solver_preserves_frames_and_missing_masks():
    """Cross-module synthetic contract, not real sensor or MANO mesh accuracy."""
    config = load_config()
    assert config["node_to_slot"] == MAPPING  # Current palm + two sensors/finger layout.
    rng = np.random.default_rng(20260930)
    rest_quats = Rotation.from_euler("XYZ", rng.uniform(-.4, .4, (16, 3))).as_quat()
    rest = {"parents": [int(parent) for parent in PARENTS], "rest_global_xyzw": rest_quats.tolist(),
            "provenance": {"kind": "synthetic_test_only"}}
    mounts = {node: Rotation.random(random_state=rng) for node in MAPPING}
    headings = {node: Rotation.from_euler("z", rng.uniform(-np.pi, np.pi)) for node in MAPPING}

    def sensor_frame(bone_quats, timestamp):
        # Independent sensor frames: Y * R_sensor * X = R_world_bone.
        return {"timestamp_s": timestamp, "nodes": {
            node: {"quat_xyzw": (headings[node].inv() * Rotation.from_quat(q)
                                  * mounts[node].inv()).as_quat().tolist(),
                   "age_s": 0., "epoch": 7}
            for node, q in bone_quats.items()}}

    roots = Rotation.from_euler("XYZ", [[0, 0, 0], [60, 0, 0], [0, 70, 0]], degrees=True)
    manifest = {"schema_version": 1, "time_origin": "first_frame", "poses": []}
    frames = []
    for index, root in enumerate(roots):
        manifest["poses"].append({"start_s": index, "end_s": index + .9,
                                  "root_quat_xyzw": root.as_quat().tolist()})
        rigid_bones = {node: (root * Rotation.from_quat(rest_quats[slot])).as_quat()
                       for node, slot in MAPPING.items()}
        frames.extend(sensor_frame(rigid_bones, 100. + index + sample * .1) for sample in range(8))
    profile = calibrate(frames, manifest, rest, config, stream_sha256="a" * 64)
    assert profile["stream_first_timestamp_s"] == 100.
    assert profile["status"] == "provisional_not_accuracy_validated"
    assert not profile["rejected_nodes"] and "11" not in profile["nodes"]
    for node, fitted in profile["nodes"].items():
        assert_rotation_equal(fitted["sensor_from_bone_xyzw"], mounts[node].as_quat())
        assert_rotation_equal(fitted["world_alignment_xyzw"], headings[node].as_quat())

    angles = np.zeros((16, 3))
    for index, proximal in enumerate((1, 4, 7, 10)):
        angles[proximal, 1:] = [(-1) ** index * .1, .25 + .1 * index]
        angles[proximal + 1, 2] = .4 + .1 * index
        angles[proximal + 2, 2] = config["dip_ratio"] * angles[proximal + 1, 2]
    # The unmeasured thumb CMC remains the explicit neutral prior, not ground truth recovery.
    angles[14, 1:] = [.1, .4]
    angles[15, 2] = .6
    root_delta = Rotation.from_euler("XYZ", [.6, -.35, .8])
    expected_bones = synthesize(config, rest_quats, angles, root_delta)
    frame = sensor_frame(expected_bones, 104.)
    observations, rejected = apply_calibration(frame, profile)
    assert not rejected
    for node in MAPPING:
        assert_rotation_equal(observations[node], expected_bones[node])
    solver = SparseHandSolver(config, rest_quats)
    result = solver.solve(observations, frame["timestamp_s"])
    assert result["pose_available"] and result["solver_success"]
    np.testing.assert_allclose(result["anatomical_euler_rad"], angles, atol=1e-6)
    assert_rotation_equal(result["root_quat_xyzw"], root_delta.as_quat())
    assert not result["anatomical_accuracy_verified"] and not result["yaw_drift_corrected"]
    for slot in (3, 6, 9, 12, 13):
        assert not result["segment_observed_mask"][slot]
        assert not result["joint_observed_mask"][slot] and result["inferred_mask"][slot]
    assert result["segment_observed_mask"][14] and not result["joint_observed_mask"][14]
    assert result["inferred_mask"][14] and result["joint_observed_mask"][15]

    # Missing inputs stay missing across calibration, rather than becoming neutral quaternions.
    del frame["nodes"]["3"]
    observations, rejected = apply_calibration(frame, profile)
    assert rejected == {"3": "missing_or_stale"} and "3" not in observations
    partial = solver.solve(observations, 105.)
    assert partial["pose_available"] and not partial["segment_observed_mask"][1]
    assert partial["inferred_mask"][1] and partial["inferred_mask"][2]
    np.testing.assert_allclose(np.asarray(partial["anatomical_euler_rad"])[4:], angles[4:], atol=1e-6)
    del frame["nodes"]["12"]
    observations, rejected = apply_calibration(frame, profile)
    unavailable = solver.solve(observations, 106.)
    assert "12" in rejected and not unavailable["pose_available"]
    assert unavailable["root_quat_xyzw"] is None and not any(unavailable["joint_observed_mask"])
