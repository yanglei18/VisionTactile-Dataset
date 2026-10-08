"""Source-provenance tests plus explicit opt-in real official-core execution.

Set FSGLOVE12_UPSTREAM_CHECKOUT and FSGLOVE12_MANO_ASSETS for numerical tests.
Those tests execute trusted external code and deserialize the supplied model.
No test starts a GUI, serial stream, or network service.
"""

import ast
import builtins
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12 import upstream_core


def test_select_nodes_preserves_complete_official_syntax():
    source = "import forbidden_dependency\nX = 3\n@decorate\ndef fit(a=X):\n    return a + X\n"
    tree, hashes = upstream_core._select_nodes(source, "external.py", ("X", "fit"))
    original = ast.parse(source)
    assert [ast.dump(x) for x in tree.body] == [ast.dump(x) for x in original.body[1:]]
    assert all(not isinstance(node, (ast.Import, ast.ImportFrom)) for node in tree.body)
    assert hashes["fit"] == hashlib.sha256(ast.dump(original.body[-1], include_attributes=False).encode()).hexdigest()
    assert tree.body[-1].lineno == 4


def test_select_nodes_rejects_missing_or_duplicate_symbols():
    with pytest.raises(upstream_core.UpstreamSourceError):
        upstream_core._select_nodes("def fit(): pass\n", "missing.py", ("fit", "absent"))
    with pytest.raises(upstream_core.UpstreamSourceError):
        upstream_core._select_nodes("def fit(): pass\ndef fit(): pass\n", "duplicate.py", ("fit",))


def test_missing_checkout_fails_before_optional_import(tmp_path, monkeypatch):
    monkeypatch.setattr(upstream_core, "_dependency", lambda _: pytest.fail("must not import optional dependencies"))
    with pytest.raises(upstream_core.UpstreamSourceError, match="missing"):
        upstream_core.load_upstream_core(tmp_path, tmp_path)


def test_modified_checkout_rejected_before_optional_import(tmp_path, monkeypatch):
    path = tmp_path / upstream_core._PINNED_SOURCES["calibration"][0]
    path.parent.mkdir(parents=True)
    path.write_text("# not the pinned official implementation\n", encoding="utf-8")
    monkeypatch.setattr(upstream_core, "_dependency", lambda _: pytest.fail("must not import optional dependencies"))
    with pytest.raises(upstream_core.UpstreamSourceError, match="hash mismatch"):
        upstream_core.load_upstream_core(tmp_path, tmp_path)


def test_asset_factory_changes_only_explicit_model_root(tmp_path):
    calls = []

    def RealLayer(*args, **kwargs):
        calls.append((args, kwargs))
        return "actual_layer"

    factory = upstream_core._asset_factory(RealLayer, tmp_path)
    assert factory(5, side="right", mano_assets_root="assets/mano", flat_hand_mean=True) == "actual_layer"
    assert calls == [((5,), dict(side="right", mano_assets_root=str(tmp_path), flat_hand_mean=True))]
    assert factory.__wrapped__ is RealLayer


@pytest.fixture(scope="module")
def real_core():
    checkout = os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT")
    assets = os.environ.get("FSGLOVE12_MANO_ASSETS")
    if not checkout or not assets:
        pytest.skip("Explicit trusted FSGlove checkout and MANO assets required")
    return upstream_core.load_upstream_core(checkout, assets)


def calibration_poses():
    # One rigid hand rotation around a horizontal world axis, with per-sensor
    # installation orientations. This is an algebraic fixture, not real data.
    mount = Rotation.from_euler("XYZ", np.linspace(-.3, .4, 48).reshape(16, 3))
    return mount.as_quat(), (Rotation.from_euler("x", -60, degrees=True) * mount).as_quat(), (Rotation.from_euler("x", 60, degrees=True) * mount).as_quat()


def test_real_provenance_json_and_original_symbols(real_core):
    assert real_core.provenance["upstream_commit"] == upstream_core.UPSTREAM_COMMIT
    assert real_core.provenance["numerical_ast_modified"] is False
    json.dumps(real_core.provenance)
    assert set(real_core.provenance["sources"]["calibration"]["symbols_ast_sha256"]) == set(upstream_core.CALIBRATION_SYMBOLS)
    assert set(real_core.provenance["sources"]["utils"]["symbols_ast_sha256"]) == set(upstream_core.UTILS_SYMBOLS)
    assert real_core.calibration.Raw2MANOCalibrator.get_mano_param.__code__.co_filename.endswith("modules/hand/calibration.py")


def test_real_seed_scope_restores_rng_including_on_error(real_core):
    torch = real_core.torch
    state = torch.random.get_rng_state().clone()
    np_state = np.random.get_state()
    with pytest.raises(RuntimeError):
        with real_core.seeded(123):
            sample = torch.rand(6)
            np_sample = np.random.rand(6)
            raise RuntimeError("test exception")
    assert torch.equal(state, torch.random.get_rng_state())
    new_np_state = np.random.get_state()
    assert np_state[0] == new_np_state[0]
    assert np.array_equal(np_state[1], new_np_state[1])
    assert np_state[2:] == new_np_state[2:]
    with real_core.seeded(123):
        assert torch.equal(sample, torch.rand(6))
        assert np.array_equal(np_sample, np.random.rand(6))


def _whole_original_calibration(real_core):
    """Execute the entire official source text with local dependency wiring.

    This independent baseline does not select any calibration AST nodes. Only
    import resolution of its assets and three support modules is scoped here.
    """
    bindings = {
        "modules.datamodels": real_core.datamodels,
        "modules.enumerates": real_core.enumerates,
        "modules.utils": real_core.utils,
        "manotorch.manolayer": SimpleNamespace(ManoLayer=real_core.calibration.ManoLayer, MANOOutput=real_core.calibration.MANOOutput),
        "manotorch.axislayer": SimpleNamespace(AxisLayerFK=real_core.calibration.AxisLayerFK),
    }

    def scoped_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name in bindings:
            return bindings[name]
        return builtins.__import__(name, globals, locals, fromlist, level)

    filename = real_core.provenance["sources"]["calibration"]["path"]
    namespace = {"__name__": "_official_whole_source_test", "__builtins__": dict(vars(builtins), __import__=scoped_import)}
    exec(compile(Path(filename).read_text(), filename, "exec", dont_inherit=True), namespace)
    return SimpleNamespace(**namespace)


def test_real_16_slot_output_matches_whole_official_source(real_core):
    base, pose1, pose2 = calibration_poses()
    extracted = real_core.create_calibrator(base, pose1, pose2, seed=42)
    reference_module = _whole_original_calibration(real_core)
    with real_core.seeded(42):
        reference = reference_module.Raw2MANOCalibrator(reference_module.CalibrationQuatSet(
            base_pose=base, z_rot_pose_1=pose1, z_rot_pose_2=pose2), side="right")
    perturb = Rotation.from_euler("XYZ", np.linspace(-.15, .3, 48).reshape(16, 3))
    raw = Rotation.from_quat(base) * perturb
    for anatomical in (False, True):
        for world in (False, True):
            actual = extracted.get_mano_param(raw, ana_repr=anatomical, map_to_world=world)
            expected = reference.get_mano_param(raw, ana_repr=anatomical, map_to_world=world)
            assert actual.shape == (16, 3)
            assert real_core.torch.isfinite(actual).all()
            np.testing.assert_array_equal(actual.numpy(), expected.numpy())
    with pytest.raises(AssertionError):
        extracted.get_mano_param(raw[:12])
    # Both optional local-installation correction and the primary inference
    # remain the actual original methods, with the same random inputs.
    labels = np.zeros((2, 16, 3))
    raw_poses = np.stack((base, raw.as_quat()))
    with real_core.seeded(9):
        extracted.correct_local_inst_error(raw_poses, labels)
    with real_core.seeded(9):
        reference.correct_local_inst_error(raw_poses, labels)
    np.testing.assert_array_equal(extracted._local_installation_error, reference._local_installation_error)


def test_real_official_shape_optimizer_and_gradients(real_core):
    torch = real_core.torch
    joints = torch.zeros((4, 16, 3))
    categories = torch.tensor([2, 3, 4, 5])
    actual = real_core.optimize_shape(joints, categories, iter_num=3, seed=7)
    with real_core.seeded(7):
        expected = real_core.utils.get_optimized_shape_params(joints, categories, "right", iter_num=3)
    assert actual.shape == (10,)
    assert torch.isfinite(actual).all()
    assert torch.equal(actual, expected)
    with real_core.seeded(7):
        initial = torch.randn(10)
    assert not torch.equal(actual, initial), "Official Adam must actually update MANO beta"
    points = torch.randn(4, 21, 3, requires_grad=True)
    real_core.utils.get_loss(torch.nn.MSELoss(), points, categories).backward()
    assert points.grad is not None and torch.isfinite(points.grad).all()


def test_real_wrist_and_gpr_official_symbols_are_usable(real_core):
    points = np.array([[0., 0, 0], [.1, 0, 0], [0, .1, 0], [0, 0, .1], [.1, .1, 0]])
    rotation = Rotation.from_euler("XYZ", [.1, -.2, .3])
    translation = np.array([1., 2., 3.])
    wrist = real_core.calibration.WristPoseCalibrator(
        Rotation.identity(), np.zeros(3), points, rotation.apply(points) + translation)
    output_rotation, output_position = wrist.get_wrist_transformation_matrix(Rotation.identity(), np.zeros(3))
    np.testing.assert_allclose(output_rotation.as_matrix(), rotation.as_matrix(), atol=1e-12)
    np.testing.assert_allclose(output_position, translation, atol=1e-12)
    training = np.linspace(-.2, .2, 4 * 16 * 3).reshape(4, 16, 3)
    gpr = real_core.calibration.GPRInstallationCalibrator(training, training * .8)
    result = gpr.get_mano_param(real_core.torch.from_numpy(training[0]))
    assert result.shape == (16, 3)
    assert real_core.torch.isfinite(result).all()


def test_real_remaining_keypoint_and_rotation_utilities(real_core):
    torch = real_core.torch
    model = real_core.calibration.ManoLayer(rot_mode="axisang", use_pca=False,
                                           side="right", center_idx=None, flat_hand_mean=True)
    with torch.no_grad():
        output = model(torch.zeros(1, 48), torch.zeros(1, 10))
    ids = real_core.calibration.keypoints_verts_ids
    keypoints = output.verts[0, ids].clone()
    with real_core.seeded(3):
        first = real_core.calibration.get_optimized_shape_params(keypoints, "right", iter_num=3)
    with real_core.seeded(3):
        second = real_core.utils.get_optimized_shape_params_from_keypoints(keypoints, ids, "right", iter_num=3)
    assert torch.equal(first, second)
    assert torch.isfinite(first).all()
    matched = real_core.calibration.point_match(keypoints, keypoints + torch.tensor([1., 2., 3.]))
    torch.testing.assert_close(matched, keypoints + torch.tensor([1., 2., 3.]))
    params = real_core.calibration.InstallationParams()
    assert params.rot_bias.shape == (16, 3)
    assert params.rot_amp.shape == (16,)
    first_pose = Rotation.identity(16)
    second_pose = Rotation.from_rotvec(np.tile([.5, 0., 0.], (16, 1)))
    virtual = real_core.calibration.get_vr_rotation(first_pose, second_pose)
    assert virtual.as_matrix().shape == (16, 3, 3)
    np.testing.assert_allclose(np.linalg.det(virtual.as_matrix()), 1, atol=1e-12)
