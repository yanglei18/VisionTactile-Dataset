"""Synthetic motion-axis contracts, not anatomical or live glove validation."""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import PARENTS, digest, load_config
from vt_fsglove12.calibration import apply_calibration
from vt_fsglove12.functional_axes import apply_functional_axes, fit_functional_finger
from vt_fsglove12.solver import SparseHandSolver


def setup_case(finger="little", moving_slots=None):
    config = load_config()
    rest_rotation = Rotation.from_euler("XYZ", np.random.default_rng(912).uniform(-.3, .3, (16, 3)))
    rest = dict(parents=list(PARENTS), rest_global_xyzw=rest_rotation.as_quat().tolist())
    profile = dict(session_id="s1", rest_sha256=digest(rest), stale_after_s=.15, nodes={
        node: dict(slot=slot, epoch=2, sensor_from_bone_xyzw=[0., 0., 0., 1.], world_alignment_xyzw=[0., 0., 0., 1.])
        for node, slot in config["node_to_slot"].items()})
    slots = (7, 8) if finger == "little" else (14, 15)
    basis = {slot: Rotation.from_euler("XYZ", [.7, -.5, .2]) for slot in slots}
    neutral = {slot: Rotation.from_euler("XYZ", [.1, -.08, .05]) for slot in slots}

    def frame(index, angle, common=None):
        world = [rest_rotation[0] if common is None else common * rest_rotation[0]]
        for slot in range(1, 16):
            delta = Rotation.identity()
            if slot in slots:
                amount = angle if moving_slots is None or slot in moving_slots else 0.
                delta = neutral[slot] * basis[slot] * Rotation.from_rotvec([0., 0., amount]) * basis[slot].inv()
            local = rest_rotation[PARENTS[slot]].inv() * rest_rotation[slot]
            world.append(world[PARENTS[slot]] * local * delta)
        return dict(timestamp_s=index * .03, nodes={node: dict(quat_xyzw=world[slot].as_quat().tolist(),
                    epoch=2, sample_index=index, age_s=.01) for node, slot in config["node_to_slot"].items()})

    neutral_frames = [frame(i, 0.) for i in range(1, 9)]
    motion_frames = [frame(i, angle) for i, angle in enumerate(np.linspace(.2, 1., 8), 9)]
    return config, rest, profile, frame, neutral_frames, motion_frames


def fit(case, finger="little"):
    config, rest, profile, _, neutral, motion = case
    return fit_functional_finger(neutral, motion, profile, rest, config, finger=finger, session_id="s1")


def local_deltas(observations, profile, rest):
    rotations = Rotation.from_quat(rest["rest_global_xyzw"])
    world = {profile["nodes"][node]["slot"]: Rotation.from_quat(q) for node, q in observations.items()}
    if 13 not in world:
        world[13] = world[0] * rotations[0].inv() * rotations[13]
    return {slot: (rotations[PARENTS[slot]].inv() * rotations[slot]).inv() * world[PARENTS[slot]].inv() * world[slot]
            for slot in world if slot and PARENTS[slot] in world}


@pytest.mark.parametrize("finger,slots", [("little", (7, 8)), ("thumb", (14, 15))])
def test_retargets_axis_without_gain_preserves_root_and_does_not_create_cmc(finger, slots):
    case = setup_case(finger)
    config, rest, profile, frame, _, _ = case
    entry = fit(case, finger)
    profile["functional_fingers"] = {finger: entry}
    assert set(entry["slot_refs"]) == {str(slot) for slot in slots}
    assert not entry["observed_range_is_anatomical_limit"]
    if finger == "thumb":
        assert entry["slot_refs"]["14"]["source"] == "inferred_cmc_prior_parent"
        assert entry["slot_refs"]["15"]["source"] == "observed_parent_child_relative_motion"
    for angle in (0., .3, .8, 1.2, -.25):
        raw = {node: data["quat_xyzw"] for node, data in frame(30, angle, Rotation.from_euler("XYZ", [.5, .3, -.4]))["nodes"].items()}
        before = deepcopy(raw)
        result, diagnostic = apply_functional_axes(raw, profile, rest)
        assert raw == before and set(result) == set(raw) and "11" not in result
        assert result["12"] == raw["12"]
        assert diagnostic["applied_slots"] == list(slots)
        assert diagnostic["residual_domain"] == "functional_axis_retargeted_observations"
        relative = local_deltas(result, profile, rest)
        for slot in slots:
            np.testing.assert_allclose(relative[slot].as_rotvec(), [0., 0., angle], atol=1e-12)
        for node, slot in config["node_to_slot"].items():
            if slot not in slots:
                assert result[node] == raw[node]


def test_only_moving_joint_is_fitted_and_unmodified_child_relation_propagates():
    case = setup_case("little", moving_slots=(7,))
    config, rest, profile, frame, _, _ = case
    entry = fit(case)
    assert set(entry["slot_refs"]) == {"7"} and "8" in entry["rejected_slots"]
    profile["functional_fingers"] = {"little": entry}
    raw = {node: data["quat_xyzw"] for node, data in frame(30, .6)["nodes"].items()}
    result, diag = apply_functional_axes(raw, profile, rest)
    assert diag["applied_slots"] == [7] and diag["propagated_slots"] == [8]
    original, new = local_deltas(raw, profile, rest), local_deltas(result, profile, rest)
    assert (original[8].inv() * new[8]).magnitude() < 1e-12
    assert Rotation.from_quat(result["10"]).magnitude() != Rotation.from_quat(raw["10"]).magnitude()


@pytest.mark.parametrize("failure", ["frozen", "stale", "epoch", "small", "missing_parent"])
def test_bad_samples_cannot_manufacture_a_motion_axis(failure):
    case = setup_case()
    config, rest, profile, frame, neutral, motion = case
    if failure == "frozen":
        motion[:] = [deepcopy(motion[0])] * 8
    elif failure == "small":
        motion[:] = [frame(i, .05) for i in range(9, 17)]
    else:
        for item in motion:
            for node in ("12", "9", "10"):
                if failure == "stale":
                    item["nodes"][node]["age_s"] = .5
                elif failure == "epoch":
                    item["nodes"][node]["epoch"] = 3
                else:
                    item["nodes"].pop(node)
    with pytest.raises(ValueError, match="没有可更新"):
        fit(case)


def test_failed_local_recalibration_retains_previous_reference_only_in_same_binding():
    case = setup_case()
    profile = case[2]
    old = fit(case)
    profile["functional_fingers"] = {"little": old}
    partial = setup_case(moving_slots=(7,))
    new = fit_functional_finger(partial[4], partial[5], profile, case[1], case[0], finger="little", session_id="s1")
    assert new["slot_refs"]["8"] == old["slot_refs"]["8"]
    assert new["diagnostics"]["8"]["status"] == "retained_previous"
    old["session_id"] = "old"
    new = fit_functional_finger(partial[4], partial[5], profile, case[1], case[0], finger="little", session_id="s1")
    assert set(new["slot_refs"]) == {"7"}


def test_apply_missing_parent_skips_joint_and_binding_mismatch_is_noop():
    case = setup_case()
    _, rest, profile, frame, _, _ = case
    profile["functional_fingers"] = {"little": fit(case)}
    raw = {node: data["quat_xyzw"] for node, data in frame(30, .6)["nodes"].items()}
    del raw["9"]
    result, diagnostic = apply_functional_axes(raw, profile, rest)
    assert result == raw and not diagnostic["applied_slots"] and "8" in diagnostic["skipped_slots"]
    profile["functional_fingers"]["little"]["session_id"] = "old"
    result, diagnostic = apply_functional_axes(raw, profile, rest)
    assert result == raw and "little" in diagnostic["skipped_slots"]


def test_no_entries_and_missing_root_do_not_invent_observations():
    case = setup_case()
    _, rest, profile, frame, _, _ = case
    raw = {node: data["quat_xyzw"] for node, data in frame(30, .6)["nodes"].items()}
    assert apply_functional_axes(raw, profile, rest)[0] == raw
    profile["functional_fingers"] = {"little": fit(case)}
    del raw["12"]
    result, diagnostic = apply_functional_axes(raw, profile, rest)
    assert result == raw and not diagnostic["applied_slots"]
    assert diagnostic["skipped_slots"]["root"] == "missing_root"


def test_unsupported_finger_and_cross_session_are_rejected():
    config, rest, profile, _, neutral, motion = setup_case()
    for finger, session in (("index", "s1"), ("little", "old")):
        with pytest.raises(ValueError):
            fit_functional_finger(neutral, motion, profile, rest, config, finger=finger, session_id=session)


@pytest.mark.parametrize("vectors,accepted", [
    (np.tile(np.eye(3) * .7, (3, 1)), False),  # Three incompatible movement axes.
    (np.tile([[0., 0., .7], [0., 0., -.7]], (4, 1)), False),  # No instructed sign.
    (np.tile([[0., 0., -.7]], (8, 1)), True),  # Opposite observed axis maps to +Z.
])
def test_axis_consistency_and_sign_are_identifiable_or_rejected(vectors, accepted):
    config, rest, profile, _, _, _ = setup_case()
    template = Rotation.from_quat(rest["rest_global_xyzw"])

    def make_frame(index, vector):
        world = [template[0]]
        for slot in range(1, 16):
            delta = Rotation.from_rotvec(vector) if slot in (7, 8) else Rotation.identity()
            world.append(world[PARENTS[slot]] * template[PARENTS[slot]].inv() * template[slot] * delta)
        return dict(nodes={node: dict(quat_xyzw=world[slot].as_quat().tolist(), epoch=2, sample_index=index, age_s=.01)
                           for node, slot in config["node_to_slot"].items()})

    neutral = [make_frame(i, [0., 0., 0.]) for i in range(1, 5)]
    motion = [make_frame(i, vector) for i, vector in enumerate(vectors, 5)]
    if not accepted:
        with pytest.raises(ValueError, match="没有可更新"):
            fit_functional_finger(neutral, motion, profile, rest, config, finger="little", session_id="s1")
        return
    entry = fit_functional_finger(neutral, motion, profile, rest, config, finger="little", session_id="s1")
    profile["functional_fingers"] = {"little": entry}
    raw = {node: data["quat_xyzw"] for node, data in motion[0]["nodes"].items()}
    result, _ = apply_functional_axes(raw, profile, rest)
    for slot in (7, 8):
        np.testing.assert_allclose(local_deltas(result, profile, rest)[slot].as_rotvec(), [0., 0., .7], atol=1e-12)


def sensor_frame(config, rest, profile, angles, index, common=None):
    """Independent tree + inverse Y*sensor*X; inputs are synthetic, not logs."""
    template = Rotation.from_quat(rest["rest_global_xyzw"])
    world = [template[0] if common is None else common * template[0]]
    for slot in range(1, 16):
        local = template[PARENTS[slot]].inv() * template[slot]
        world.append(world[PARENTS[slot]] * local * Rotation.from_euler("XYZ", angles[slot]))
    nodes = {}
    for node, slot in config["node_to_slot"].items():
        params = profile["nodes"][node]
        sensor = Rotation.from_quat(params["world_alignment_xyzw"]).inv() * world[slot] * Rotation.from_quat(params["sensor_from_bone_xyzw"]).inv()
        nodes[node] = dict(quat_xyzw=sensor.as_quat().tolist(), epoch=params["epoch"], sample_index=index, age_s=.01)
    return dict(timestamp_s=index * .03, nodes=nodes), world


@pytest.mark.parametrize("finger,slots", [("little", (7, 8)), ("thumb", (14, 15))])
def test_negative_observed_axis_old_solver_clips_then_retarget_bends_and_returns_without_lag(finger, slots):
    config, rest, profile, _, _, _ = setup_case(finger)

    def frame(index, amount):
        angles = np.zeros((16, 3))
        angles[list(slots), 2] = -amount
        return sensor_frame(config, rest, profile, angles, index)[0]

    entry = fit_functional_finger([frame(i, 0.) for i in range(1, 5)],
        [frame(i, angle) for i, angle in enumerate(np.linspace(.2, 1., 8), 5)],
        profile, rest, config, finger=finger, session_id="s1")
    solver = SparseHandSolver(config, rest["rest_global_xyzw"])
    original, _ = apply_calibration(frame(20, .7), profile)
    old = solver.solve(original, 1.)
    np.testing.assert_allclose(np.asarray(old["anatomical_euler_rad"])[list(slots), 2], [-.25, -.15], atol=1e-6)
    profile["functional_fingers"] = {finger: entry}
    transformed, _ = apply_functional_axes(original, profile, rest)
    new = solver.solve(transformed, 2.)
    np.testing.assert_allclose(np.asarray(new["anatomical_euler_rad"])[list(slots), 2], [.7, .7], atol=1e-6)
    neutral, _ = apply_calibration(frame(21, 0.), profile)
    returned = solver.solve(apply_functional_axes(neutral, profile, rest)[0], 3.)
    np.testing.assert_allclose(returned["anatomical_euler_rad"], 0., atol=1e-6)
    assert not new["segment_observed_mask"][13]
    assert new["source_labels"][13] == "inferred_thumb_cmc_prior"
    print(f"SYNTHETIC {finger}: observed Z=-0.700000 rad; old fitted Z="
          f"{np.asarray(old['anatomical_euler_rad'])[list(slots), 2].tolist()}; "
          f"retarget fitted Z={np.asarray(new['anatomical_euler_rad'])[list(slots), 2].tolist()}; "
          f"next-neutral maxabs={np.abs(returned['anatomical_euler_rad']).max():.3e} rad")


@pytest.mark.parametrize("prior", [[0., 0., 0.], [.12, -.18, .22]])
def test_mount_heading_common_root_and_missing_cmc_prior_preserve_parent_child_contract(prior):
    config, rest, profile, _, _, _ = setup_case("thumb")
    config["thumb_cmc_prior_rad"] = prior
    rng = np.random.default_rng(11921)
    for params in profile["nodes"].values():
        params["sensor_from_bone_xyzw"] = Rotation.random(random_state=rng).as_quat().tolist()
        params["world_alignment_xyzw"] = Rotation.from_euler("z", rng.uniform(-np.pi, np.pi)).as_quat().tolist()
    slots = (7, 8, 14, 15)

    def frame(index, angle, common):
        angles = np.zeros((16, 3))
        angles[13] = prior
        angles[list(slots), 2] = -angle
        return sensor_frame(config, rest, profile, angles, index, common)

    neutral = [frame(i, 0., Rotation.random(random_state=rng))[0] for i in range(1, 5)]
    motion = [frame(i, angle, Rotation.random(random_state=rng))[0]
              for i, angle in enumerate(np.linspace(.2, 1., 8), 5)]
    profile["functional_fingers"] = {finger: fit_functional_finger(neutral, motion, profile, rest, config,
                                         finger=finger, session_id="s1") for finger in ("thumb", "little")}
    common = Rotation.from_euler("XYZ", [.9, -.4, 1.2])
    captured, world = frame(20, .65, common)
    calibrated, rejected = apply_calibration(captured, profile)
    assert not rejected
    for node, slot in config["node_to_slot"].items():
        assert (world[slot].inv() * Rotation.from_quat(calibrated[node])).magnitude() < 1e-12
    retargeted, diagnostic = apply_functional_axes(calibrated, profile, rest)
    solver = SparseHandSolver(config, rest["rest_global_xyzw"])
    result = solver.solve(retargeted, 1.)
    np.testing.assert_allclose(np.asarray(result["anatomical_euler_rad"])[list(slots), 2], .65, atol=1e-6)
    np.testing.assert_allclose(result["anatomical_euler_rad"][13], prior, atol=1e-12)
    assert (common.inv() * Rotation.from_quat(result["root_quat_xyzw"])).magnitude() < 1e-12
    assert set(diagnostic["applied_slots"]) == set(slots)
    assert not result["joint_observed_mask"][14] and not result["segment_observed_mask"][13]
    expected_angles = np.zeros((16, 3))
    expected_angles[13] = prior
    expected_angles[list(slots), 2] = .65
    _, expected_world = sensor_frame(config, rest, profile, expected_angles, 21, common)
    for node, slot in config["node_to_slot"].items():
        assert (expected_world[slot].inv() * Rotation.from_quat(retargeted[node])).magnitude() < 1e-12
