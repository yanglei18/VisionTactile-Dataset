"""Math-equivalence/operation-count regressions, not glove accuracy tests.

The legacy fit below deliberately retains the pre-optimization SciPy Rotation
residual and per-joint FK. Wall-clock speed is measured separately on recorded
frames; CI does not impose machine-dependent timing thresholds.
"""
from types import SimpleNamespace
import warnings

import numpy as np
import pytest
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import load_config
from vt_fsglove12 import solver as module
from vt_fsglove12.solver import PARENTS, SparseHandSolver


def legacy_forward(solver, angles, root):
    local = solver._rest_local_matrices @ Rotation.from_euler("XYZ", angles).as_matrix()
    matrices = np.empty((16, 3, 3))
    matrices[0] = root
    for slot in range(1, 16):
        matrices[slot] = matrices[PARENTS[slot]] @ local[slot]
    return matrices


def legacy_fit(solver, observations):
    """Old objective and initializer, independent of the optimized helpers."""
    rotations = {solver.node_to_slot[node]: Rotation.from_quat(q) for node, q in observations.items()}
    observed = np.array([slot in rotations for slot in range(16)])
    variables = solver._variables(observed)
    lower, upper = np.array([v[2] for v in variables]), np.array([v[3] for v in variables])

    def unpack(parameters):
        angles = np.zeros((16, 3))
        if not observed[13]:
            angles[13] = solver.thumb_prior
        for value, (slot, axis, _, _) in zip(parameters, variables):
            angles[slot, axis] = value
        for middle, distal in ((2, 3), (5, 6), (8, 9), (11, 12)):
            if not observed[distal]:
                angles[distal, 2] = solver.dip_ratio * angles[middle, 2]
        return angles

    initial = np.zeros(len(variables))
    for slot in range(1, 16):
        if slot not in rotations:
            continue
        parents = Rotation.from_matrix(legacy_forward(solver, unpack(initial), rotations[0].as_matrix()))
        parent = rotations.get(int(PARENTS[slot]), parents[PARENTS[slot]])
        local = solver.rest_local[slot].inv() * parent.inv() * rotations[slot]
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            estimate = local.as_euler("XYZ")
        for index, (variable_slot, axis, _, _) in enumerate(variables):
            if variable_slot == slot:
                initial[index] = np.clip(estimate[axis], lower[index], upper[index])
    measured = [slot for slot in rotations if slot != 0]
    matrices = np.asarray([rotations[slot].as_matrix() for slot in measured]).reshape(-1, 3, 3)

    def residual(parameters):
        predicted = legacy_forward(solver, unpack(parameters), rotations[0].as_matrix())
        errors = []
        if measured:
            difference = predicted[measured].transpose(0, 2, 1) @ matrices
            errors.append(Rotation.from_matrix(difference).as_rotvec().ravel())
        return np.concatenate([*errors, 1e-5 * parameters])

    fit = least_squares(residual, initial, bounds=(lower, upper), max_nfev=solver.max_nfev,
                        ftol=1e-9, xtol=1e-9, gtol=1e-9)
    return fit, unpack(fit.x), residual, variables


@pytest.mark.parametrize("count", [0, 1, 16, 1000])
def test_batched_euler_matches_scipy_intrinsic_xyz(count):
    angles = np.random.default_rng(709).uniform(-8., 8., (count, 3))
    actual = module._euler_xyz_matrices(angles)
    assert actual.shape == (count, 3, 3)
    if count:
        np.testing.assert_allclose(actual, Rotation.from_euler("XYZ", angles).as_matrix(), atol=2e-15)


def test_so3_log_matches_scipy_for_random_rotations_and_zero_pi_boundaries():
    rng = np.random.default_rng(710)
    axes = rng.normal(size=(2000, 3))
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    angles = np.concatenate((np.zeros(10), np.full(10, 1e-12), np.full(10, 1e-8),
                             np.full(10, np.pi), np.full(10, np.pi - 1e-10),
                             np.full(10, np.pi + 1e-10), np.full(10, np.pi - .02),
                             rng.uniform(0., 2. * np.pi, 1930)))
    matrices = Rotation.from_rotvec(axes * angles[:, None]).as_matrix()
    expected = Rotation.from_matrix(matrices).as_rotvec()
    actual = module._rotation_log(matrices)
    np.testing.assert_allclose(actual, expected, atol=5e-12, rtol=1e-12)
    np.testing.assert_allclose(Rotation.from_rotvec(actual).as_matrix(), matrices, atol=5e-12)


def test_common_log_and_euler_path_does_not_allocate_scipy_rotations(monkeypatch):
    angles = np.array([[.2, -.3, .4], [0., 0., 0.]])
    matrices = Rotation.from_euler("XYZ", angles).as_matrix()

    class ForbiddenRotation:
        @staticmethod
        def from_matrix(*_args, **_kwargs):
            pytest.fail("Ordinary internal rotations should not need projection")

    monkeypatch.setattr(module, "Rotation", ForbiddenRotation)
    assert np.isfinite(module._rotation_log(matrices)).all()
    np.testing.assert_allclose(module._euler_xyz_matrices(angles), matrices, atol=1e-15)


def test_batched_fk_matches_legacy_with_nonidentity_root_and_rest():
    rng = np.random.default_rng(711)
    config = load_config()
    rest = Rotation.random(16, random_state=rng).as_quat()
    solver = SparseHandSolver(config, rest)
    for _ in range(15):
        angles = rng.uniform(-np.pi, np.pi, (16, 3))
        root = Rotation.random(random_state=rng).as_matrix()
        np.testing.assert_allclose(solver._forward_matrices(angles, root),
                                   legacy_forward(solver, angles, root), atol=3e-15)


@pytest.mark.parametrize("missing,cmc,noise", [
    ((), False, 0.), ((), False, .02), ((), False, .3),
    ((), True, .1), (("3",), False, .03), (("4", "8"), False, .03),
    (("5",), False, .03), (("1", "2", "9", "10"), True, .1),
])
def test_optimized_solver_preserves_legacy_objective_angles_and_masks(missing, cmc, noise):
    rng = np.random.default_rng(712)
    config = load_config()
    if cmc:
        config["node_to_slot"]["11"] = 13
    rest = Rotation.from_euler("XYZ", rng.uniform(-.4, .4, (16, 3))).as_quat()
    solver = SparseHandSolver(config, rest)
    angles = np.zeros((16, 3))
    for proximal in (1, 4, 7, 10):
        angles[proximal, 1:] = rng.uniform(-.4, .4), rng.uniform(.2, 1.2)
        angles[proximal + 1, 2] = rng.uniform(.3, 1.8)
        angles[proximal + 2, 2] = solver.dip_ratio * angles[proximal + 1, 2]
    if cmc:
        angles[13] = [.15, -.2, .3]
    angles[14, 1:], angles[15, 2] = [.15, .5], .8
    root = Rotation.from_euler("XYZ", [.7, -.6, .9]).as_matrix()
    matrices = legacy_forward(solver, angles, root)
    observations = {
        node: (Rotation.from_rotvec(rng.normal(size=3) * noise) * Rotation.from_matrix(matrices[slot])).as_quat()
        for node, slot in config["node_to_slot"].items() if node not in missing}
    before, expected, legacy_residual, variables = legacy_fit(solver, observations)
    after = solver.solve(observations, 1.)
    actual = np.asarray(after["anatomical_euler_rad"])
    np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=1e-5)
    parameters = np.array([actual[slot, axis] for slot, axis, _, _ in variables])
    np.testing.assert_allclose(np.dot(legacy_residual(parameters), legacy_residual(parameters)),
                               2. * before.cost, atol=2e-10, rtol=2e-8)
    assert after["solver_success"] == before.success
    for node, slot in config["node_to_slot"].items():
        assert after["segment_observed_mask"][slot] == (node not in missing)
    assert not after["yaw_drift_corrected"] and not after["anatomical_accuracy_verified"]


@pytest.mark.parametrize("cmc,expected_forward_calls", [(False, 2), (True, 1)])
def test_initializer_only_runs_fk_when_parent_is_unmeasured(monkeypatch, cmc, expected_forward_calls):
    config = load_config()
    if cmc:
        config["node_to_slot"]["11"] = 13
    solver = SparseHandSolver(config, np.tile([0., 0., 0., 1.], (16, 1)))
    calls = []
    original_forward = solver._forward

    def counted(*args):
        calls.append(1)
        return original_forward(*args)

    def fake_fit(_residual, initial, **kwargs):
        assert kwargs["ftol"] == kwargs["xtol"] == kwargs["gtol"] == 1e-9
        assert kwargs["max_nfev"] == 100
        return SimpleNamespace(x=initial, success=True, nfev=1)

    monkeypatch.setattr(solver, "_forward", counted)
    monkeypatch.setattr(module, "least_squares", fake_fit)
    solver.solve({node: [0., 0., 0., 1.] for node in config["node_to_slot"]}, 1.)
    assert len(calls) == expected_forward_calls  # Includes one final output FK.
