"""Configurable bounds/coupling diagnostics, not measured anatomical accuracy."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import FINGER_DIPS, load_config, validate_config
from vt_fsglove12 import solver as module
from vt_fsglove12.solver import PARENTS, SparseHandSolver


def make_solver(**overrides):
    config = load_config()
    config.update(overrides)
    rest = Rotation.from_euler("XYZ", np.random.default_rng(413).uniform(-.2, .2, (16, 3))).as_quat()
    return SparseHandSolver(config, rest), config, rest


def observations(config, rest, angles):
    rotations = Rotation.from_quat(rest)
    world = [rotations[0]]
    for slot in range(1, 16):
        local = rotations[PARENTS[slot]].inv() * rotations[slot]
        world.append(world[PARENTS[slot]] * local * Rotation.from_euler("XYZ", angles[slot]))
    return {node: world[slot].as_quat() for node, slot in config["node_to_slot"].items()}


@pytest.mark.parametrize("overrides", [
    {"joint_limits_rad": None}, {"joint_limits_rad": []},
    {"joint_limits_rad": {"0": {"z": [-1, 1]}}},
    {"joint_limits_rad": {"16": {"z": [-1, 1]}}},
    {"joint_limits_rad": {"07": {"z": [-1, 1]}}},
    {"joint_limits_rad": {7: {"z": [-1, 1]}}},
    {"joint_limits_rad": {"7": {}}}, {"joint_limits_rad": {"7": [-1, 1]}},
    {"joint_limits_rad": {"7": {"x": [-1, 1]}}},
    {"joint_limits_rad": {"8": {"y": [-1, 1]}}},
    {"joint_limits_rad": {"7": {"yz": [-1, 1]}}},
    {"joint_limits_rad": {"7": {1: [-1, 1]}}},
    {"joint_limits_rad": {"7": {"z": [1, 1]}}},
    {"joint_limits_rad": {"7": {"z": [2, 1]}}},
    {"joint_limits_rad": {"7": {"z": [False, 1]}}},
    {"joint_limits_rad": {"7": {"z": [0, "1"]}}},
    {"joint_limits_rad": {"7": {"z": [0]}}},
    {"joint_limits_rad": {"7": {"z": [0, 1, 2]}}},
    {"joint_limits_rad": {"7": {"z": [float("nan"), 1]}}},
    {"joint_limits_rad": {"7": {"z": [0, float("inf")]}}},
    {"joint_limits_rad": {"7": {"z": [0, 10 ** 400]}}},
    {"joint_limits_rad": {"13": {"z": [-1, 1]}}},  # Missing CMC: prior, not a variable.
    {"joint_limits_rad": {"9": {"z": [-1, 1]}}},  # Missing DIP: coupling, not a variable.
    {"dip_ratio_by_finger": []}, {"dip_ratio_by_finger": None},
    {"dip_ratio_by_finger": {"thumb": .5}}, {"dip_ratio_by_finger": {"pinky": .5}},
    {"dip_ratio_by_finger": {"little": -1}}, {"dip_ratio_by_finger": {"little": 1.1}},
    {"dip_ratio_by_finger": {"little": True}}, {"dip_ratio_by_finger": {"little": ".5"}},
    {"dip_ratio_by_finger": {"little": float("nan")}},
    {"dip_ratio_by_finger": {"little": 10 ** 400}},
])
def test_invalid_overrides_rejected_by_config_and_direct_solver(overrides):
    config = load_config()
    config.update(overrides)
    with pytest.raises(ValueError):
        validate_config(config)
    with pytest.raises(ValueError):
        SparseHandSolver(config, np.tile([0., 0., 0., 1.], (16, 1)))


def test_valid_overrides_are_independently_copied_by_config():
    config = load_config()
    config.update(joint_limits_rad={"7": {"y": [-.5, .5], "z": [-.2, 1.]}},
                  dip_ratio_by_finger={"little": .4})
    validated = validate_config(config)
    config["joint_limits_rad"]["7"]["z"][0] = -.9
    assert validated["joint_limits_rad"]["7"]["z"] == [-.2, 1.]


def test_empty_overrides_preserve_default_fit_exactly():
    default, config, rest = make_solver()
    explicit, _, _ = make_solver(joint_limits_rad={}, dip_ratio_by_finger={})
    angles = np.zeros((16, 3))
    angles[7, 1:] = [.1, .5]
    angles[8, 2] = .8
    angles[14, 1:] = [-.15, .3]
    angles[15, 2] = -.4  # Includes a bound-active default fit.
    frame = observations(config, rest, angles)
    a, b = default.solve(frame, 1.), explicit.solve(frame, 1.)
    assert a == b


def test_custom_thumb_limit_changes_only_requested_bound_and_is_used():
    default, config, rest = make_solver()
    custom, _, _ = make_solver(joint_limits_rad={"15": {"z": [-.6, 1.7]}})
    angles = np.zeros((16, 3))
    angles[15, 2] = -.4
    frame = observations(config, rest, angles)
    before, after = default.solve(frame, 1.), custom.solve(frame, 1.)
    assert before["anatomical_euler_rad"][15][2] == pytest.approx(-.15, abs=1e-6)
    np.testing.assert_allclose(after["anatomical_euler_rad"], angles, atol=1e-6)
    assert not after["active_joint_limits"]
    observed = np.ones(16, dtype=bool)
    changed = [(a, b) for a, b in zip(default._variables(observed), custom._configured_variables(observed)) if a != b]
    assert changed == [((15, 2, -.15, 1.7), (15, 2, -.6, 1.7))]
    assert custom._variables(observed) == default._variables(observed)  # Old static API remains default.


@pytest.mark.parametrize("value,bounds,side,limit", [(-.5, [-.25, 1.65], "lower", -.25),
                                                   (.8, [-.25, .3], "upper", .3)])
def test_local_preconstraint_angle_and_active_limit_are_distinct(value, bounds, side, limit):
    solver, config, rest = make_solver(joint_limits_rad={"7": {"z": bounds}})
    angles = np.zeros((16, 3))
    angles[7, 2] = value
    result = solver.solve(observations(config, rest, angles), 1.)
    initial = {item["slot"]: item for item in result["preconstraint_relative_angles"]}
    assert initial[7]["anatomical_euler_rad"][2] == pytest.approx(value, abs=1e-12)
    assert initial[7]["source"] == "observed_parent_child_relative_euler"
    assert initial[7]["parent_slot"] == 0
    assert result["preconstraint_angle_kind"] == "local_euler_initializer_not_unconstrained_optimum"
    active = next(item for item in result["active_joint_limits"] if item["slot"] == 7 and item["axis"] == "z")
    assert active["side"] == side and active["limit"] == limit
    assert active["value"] == pytest.approx(limit, abs=1e-6)
    assert active["source"] == "observed_relative_constrained_fit"
    assert not {3, 6, 9, 12, 13}.intersection(item["slot"] for item in result["active_joint_limits"])


def test_missing_parent_diagnostics_do_not_claim_a_measured_joint():
    solver, config, rest = make_solver()
    frame = observations(config, rest, np.zeros((16, 3)))
    del frame["9"]  # Little MCP is missing; PIP has only an inferred parent.
    result = solver.solve(frame, 1.)
    initial = {item["slot"]: item for item in result["preconstraint_relative_angles"]}
    assert initial[8]["source"] == initial[14]["source"] == "inferred_parent_initializer"
    assert initial[15]["source"] == "observed_parent_child_relative_euler"
    assert not {0, 7, 9, 13}.intersection(initial)
    assert result["source_labels"][13] == "inferred_thumb_cmc_prior"
    assert not result["joint_observed_mask"][14]
    del frame["12"]
    unavailable = solver.solve(frame, 2.)
    assert not unavailable["pose_available"]
    assert unavailable["preconstraint_relative_angles"] == unavailable["active_joint_limits"] == []


def test_per_finger_coupling_and_default_fallback_do_not_add_lag():
    solver, config, rest = make_solver(dip_ratio_by_finger={"index": 0., "middle": 1., "little": .4})
    angles = np.zeros((16, 3))
    for distal in FINGER_DIPS.values():
        angles[distal - 1, 2] = .8
    frame = observations(config, rest, angles)
    result = solver.solve(frame, 1.)
    for finger, distal in FINGER_DIPS.items():
        ratio = config["dip_ratio_by_finger"].get(finger, config["dip_ratio"])
        assert result["anatomical_euler_rad"][distal][2] == ratio * result["anatomical_euler_rad"][distal - 1][2]
        assert result["source_labels"][distal] == "inferred_dip_ratio"
    neutral = solver.solve(observations(config, rest, np.zeros((16, 3))), 2.)
    np.testing.assert_allclose(neutral["anatomical_euler_rad"], 0., atol=1e-6)


def test_measured_dip_uses_own_fit_not_coupling_and_dropout_restores_coupling():
    mapping = load_config()["node_to_slot"]
    mapping["8"] = 9  # Synthetic alternative layout: ring-middle input becomes little DIP.
    solver, config, rest = make_solver(node_to_slot=mapping, dip_ratio_by_finger={"little": .1},
                                       joint_limits_rad={"9": {"z": [-.2, 1.2]}})
    angles = np.zeros((16, 3))
    angles[8, 2], angles[9, 2] = .8, .4
    frame = observations(config, rest, angles)
    measured = solver.solve(frame, 1.)
    assert measured["anatomical_euler_rad"][9][2] == pytest.approx(.4, abs=1e-6)
    assert measured["source_labels"][9] == "observed_relative_constrained_fit"
    del frame["8"]
    missing = solver.solve(frame, 2.)
    assert missing["anatomical_euler_rad"][9][2] == .1 * missing["anatomical_euler_rad"][8][2]
    assert missing["source_labels"][9] == "inferred_dip_ratio"
    assert 9 not in [item["slot"] for item in missing["active_joint_limits"]]

    narrower = SparseHandSolver(dict(config, joint_limits_rad={"9": {"z": [-.2, .2]}}), rest)
    limited = narrower.solve(observations(config, rest, angles), 3.)
    assert limited["anatomical_euler_rad"][9][2] == pytest.approx(.2, abs=1e-6)
    assert any(item["slot"] == 9 and item["axis"] == "z" and item["side"] == "upper"
               for item in limited["active_joint_limits"])


def test_custom_cmc_bounds_require_mapping_but_never_create_a_missing_observation():
    mapping = load_config()["node_to_slot"]
    mapping["11"] = 13
    solver, config, rest = make_solver(node_to_slot=mapping, joint_limits_rad={"13": {"x": [-.1, .1]}})
    angles = np.zeros((16, 3))
    angles[13, 0] = .3
    frame = observations(config, rest, angles)
    result = solver.solve(frame, 1.)
    assert result["anatomical_euler_rad"][13][0] == pytest.approx(.1, abs=1e-6)
    assert result["joint_observed_mask"][13]
    del frame["11"]
    missing = solver.solve(frame, 2.)
    assert missing["anatomical_euler_rad"][13] == [0., 0., 0.]
    assert not missing["segment_observed_mask"][13]
    assert missing["source_labels"][13] == "inferred_thumb_cmc_prior"


def test_bounds_excluding_zero_work_when_that_segment_is_missing():
    solver, config, rest = make_solver(joint_limits_rad={"7": {"z": [.2, .4]}})
    frame = observations(config, rest, np.zeros((16, 3)))
    del frame["9"]
    del frame["10"]
    result = solver.solve(frame, 1.)
    assert result["pose_available"] and result["solver_success"]
    assert .2 <= result["anatomical_euler_rad"][7][2] <= .4
    active = next(item for item in result["active_joint_limits"] if item["slot"] == 7)
    assert active["source"] == "inferred_missing_segment_prior"


def test_limit_margin_is_only_a_diagnostic_and_keeps_strict_fit_options(monkeypatch):
    solver, config, rest = make_solver()
    frame = observations(config, rest, np.zeros((16, 3)))
    expected = {}

    def fake_fit(_residual, initial, **kwargs):
        assert kwargs["ftol"] == kwargs["xtol"] == kwargs["gtol"] == 1e-9
        assert kwargs["max_nfev"] == 100
        values = initial.copy()
        variables = solver._variables(np.array([slot in config["node_to_slot"].values() for slot in range(16)]))
        for index, (slot, axis, lo, _hi) in enumerate(variables):
            if (slot, axis) in ((7, 2), (8, 2)):
                values[index] = lo + (5e-7 if slot == 7 else 2e-6)
                expected[slot] = values[index]
        return SimpleNamespace(x=values, success=True, nfev=1)

    monkeypatch.setattr(module, "least_squares", fake_fit)
    result = solver.solve(frame, 1.)
    assert result["joint_limit_margin_rad"] == 1e-6
    for slot, value in expected.items():
        assert result["anatomical_euler_rad"][slot][2] == value
    assert [(item["slot"], item["axis"]) for item in result["active_joint_limits"]] == [(7, "z")]


def test_solver_copies_constraint_configuration():
    solver, config, rest = make_solver(joint_limits_rad={"15": {"z": [-.6, 1.7]}},
                                       dip_ratio_by_finger={"little": .4})
    saved = deepcopy(config)
    config["joint_limits_rad"]["15"]["z"][0] = -.1
    config["dip_ratio_by_finger"]["little"] = .9
    angles = np.zeros((16, 3))
    angles[15, 2], angles[8, 2] = -.4, .8
    result = solver.solve(observations(saved, rest, angles), 1.)
    assert result["anatomical_euler_rad"][15][2] == pytest.approx(-.4, abs=1e-6)
    assert result["anatomical_euler_rad"][9][2] == .4 * result["anatomical_euler_rad"][8][2]
