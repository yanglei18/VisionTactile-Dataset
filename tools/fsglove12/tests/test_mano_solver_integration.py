"""Opt-in real MANO coordinate-contract tests, not glove accuracy validation.

These tests deserialize a trusted, user-supplied MANO model only when
FSGLOVE12_MANO_ASSETS is explicitly set. A configured but broken backend fails;
there is no mocked or synthetic-geometry fallback. The pose inputs are synthetic
model poses, not claimed sensor measurements or physical ground truth.
"""

import os
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import load_config
from vt_fsglove12.mano import ManoBackend
from vt_fsglove12.solver import PARENTS, SparseHandSolver


pytestmark = pytest.mark.skipif(
    not os.environ.get("FSGLOVE12_MANO_ASSETS"),
    reason="Real MANO integration requires explicitly supplied FSGLOVE12_MANO_ASSETS",
)


@pytest.fixture(scope="module")
def real_backend_and_rest():
    backend = ManoBackend(Path(os.environ["FSGLOVE12_MANO_ASSETS"]), hand="right")
    rest = backend.export_rest()
    assert rest["parents"] == PARENTS.tolist()
    assert rest["provenance"]["backend"] == "manotorch.ManoLayer+AxisLayerFK"
    return backend, rest


def anatomical_global_from_actual_mano(backend, rendered):
    """Read upstream anatomical transforms, independently of solver FK.

    The bridge returns its actual MANO pose. Passing that pose back through the
    loaded real ManoLayer/AxisLayerFK exposes all 16 global anatomical frames,
    including the nonidentity rest root, rather than deriving them from the
    solver's own forward-kinematics implementation.
    """
    pose = np.asarray(rendered["mano_pose_rotvec"], dtype=np.float32).reshape(1, 48)
    with backend._torch.no_grad():
        output = backend._mano(backend._tensor(pose), backend._tensor(np.zeros((1, 10))))
        transforms = backend._axis(output.transforms_abs)[0].detach().cpu().numpy()
    assert transforms.shape == (1, 16, 4, 4) and np.isfinite(transforms).all()
    matrices = transforms[0, :, :3, :3]
    np.testing.assert_allclose(matrices.transpose(0, 2, 1) @ matrices,
                               np.broadcast_to(np.eye(3), matrices.shape), atol=2e-5)
    np.testing.assert_allclose(np.linalg.det(matrices), 1., atol=2e-5)
    return Rotation.from_matrix(matrices)


def pose_angles(case, dip_ratio):
    angles = np.zeros((16, 3))
    if case == "curl":
        spreads, mcp, pip = [0., .03, -.05, -.02], [.35, .43, .51, .59], [.6, .68, .76, .84]
        thumb = [.12, .4, .6]
    elif case == "spread":
        spreads, mcp, pip = [-.25, -.05, .2, .3], [.1, .15, .05, .2], [.12, .18, .25, .2]
        thumb = [-.25, .15, .2]
    else:
        spreads, mcp, pip = [.15, -.1, .1, -.12], [.7, .3, .8, .5], [1., .55, 1.2, .85]
        thumb = [.3, .75, 1.]
    for index, proximal in enumerate((1, 4, 7, 10)):
        angles[proximal, 1:] = spreads[index], mcp[index]
        angles[proximal + 1, 2] = pip[index]
        angles[proximal + 2, 2] = dip_ratio * pip[index]
    # Current layout does not measure thumb CMC: keep the same explicit zero
    # prior used by the solver, rather than asserting its unobservable recovery.
    angles[14, 1:] = thumb[:2]
    angles[15, 2] = thumb[2]
    return angles


@pytest.mark.parametrize("case,root_xyz", [
    ("curl", [.4, -.25, .7]),
    ("spread", [-.7, .4, -1.2]),
    ("asymmetric", [.5, .6, 1.1]),
])
def test_real_mano_sparse_solver_global_rotation_contract(real_backend_and_rest, case, root_xyz):
    backend, rest = real_backend_and_rest
    config = load_config()
    solver = SparseHandSolver(config, rest["rest_global_xyzw"])
    expected_angles = pose_angles(case, config["dip_ratio"])
    root_delta = Rotation.from_euler("XYZ", root_xyz)

    target_mesh = backend.render(expected_angles, root_delta.as_quat())
    target_global = anatomical_global_from_actual_mano(backend, target_mesh)
    expected_root = root_delta * Rotation.from_quat(rest["rest_global_xyzw"][0])
    assert (expected_root.inv() * target_global[0]).magnitude() < 5e-5
    observations = {node: target_global[slot].as_quat()
                    for node, slot in config["node_to_slot"].items()}
    result = solver.solve(observations, timestamp_s=1.)

    assert result["pose_available"] and result["solver_success"]
    # Tolerances cover upstream float32 operations, not physical sensor error.
    np.testing.assert_allclose(result["anatomical_euler_rad"], expected_angles, atol=5e-5)
    solved_root = Rotation.from_quat(result["root_quat_xyzw"])
    assert (root_delta.inv() * solved_root).magnitude() < 5e-5
    predicted_global = Rotation.from_quat(result["predicted_global_xyzw"])
    target_error = (target_global.inv() * predicted_global).magnitude()
    assert float(target_error.max()) < 5e-5, dict(enumerate(target_error.tolist()))

    reconstructed_mesh = backend.render(result["anatomical_euler_rad"], result["root_quat_xyzw"])
    reconstructed_global = anatomical_global_from_actual_mano(backend, reconstructed_mesh)
    compose_error = (reconstructed_global.inv() * predicted_global).magnitude()
    assert float(compose_error.max()) < 5e-5, dict(enumerate(compose_error.tolist()))
    np.testing.assert_allclose(reconstructed_mesh["vertices_m"], target_mesh["vertices_m"], atol=5e-6)
    np.testing.assert_allclose(reconstructed_mesh["joints_m"], target_mesh["joints_m"], atol=5e-6)
    assert result["inferred_mask"][13] and not result["joint_observed_mask"][14]
    assert not result["anatomical_accuracy_verified"] and not result["yaw_drift_corrected"]
