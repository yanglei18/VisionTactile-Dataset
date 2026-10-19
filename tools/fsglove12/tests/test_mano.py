"""Bridge contract tests. Mock tests are NOT real MANO validation.

Set FSGLOVE12_MANO_ASSETS to explicitly opt into the final real-model test with
already-installed torch/manotorch and user-supplied, licensed model assets.
"""

from contextlib import nullcontext
import importlib.metadata
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12 import mano


class MockTensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value


def test_direct_mano_render_preserves_official_pose_and_shape_without_axis_compose(mocked_backend):
    backend, state = mocked_backend
    pose = np.linspace(-.1, .2, 48).reshape(16, 3)
    betas = np.linspace(-.2, .2, 10)
    result = backend.render_mano(pose, betas)
    assert not state.compose_inputs
    np.testing.assert_allclose(state.mano_calls[-1][0], pose, atol=1e-7)
    np.testing.assert_allclose(state.mano_calls[-1][1], betas[None], atol=1e-7)
    assert result["provenance"]["pose_input"] == "upstream_mano_axis_angle_direct"


@pytest.fixture
def mocked_backend(tmp_path, monkeypatch):
    """Small API stand-ins: no MANO model is deserialized by this fixture."""
    models = tmp_path / "models"
    models.mkdir()
    (models / "MANO_RIGHT.pkl").write_bytes(b"mock sentinel, not a model")
    state = SimpleNamespace(mano_calls=[], compose_inputs=[], mano_kwargs=None, axis_kwargs=None)
    template = np.zeros((778, 3), dtype=np.float64)
    template[:3] = [[0.01, 0.02, 0.03], [0.04, -0.02, 0.01], [-0.03, 0.01, 0.05]]
    state.template = template
    rest_angles = np.zeros((16, 3))
    rest_angles[:, 2] = np.linspace(0.15, 0.6, 16)
    state.rest_rotations = Rotation.from_euler("XYZ", rest_angles)

    class MockManoLayer:
        def __init__(self, **kwargs):
            state.mano_kwargs = kwargs
            self.th_faces = MockTensor(np.array([[0, 1, 2]], dtype=np.int64))

        def eval(self):
            return self

        def __call__(self, pose, betas):
            pose = pose.value.reshape(16, 3)
            state.mano_calls.append((pose.copy(), betas.value.copy()))
            root = Rotation.from_rotvec(pose[0])
            transforms = np.tile(np.eye(4), (1, 16, 1, 1))
            return SimpleNamespace(
                verts=MockTensor(root.apply(template)[None]),
                joints=MockTensor(root.apply(template[:21])[None]),
                transforms_abs=MockTensor(transforms),
            )

    class MockAxisLayerFK:
        transf_parent_mapping = [0, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14]

        def __init__(self, **kwargs):
            state.axis_kwargs = kwargs

        def eval(self):
            return self

        def __call__(self, transforms):
            result = transforms.value.copy()
            result[0, :, :3, :3] = state.rest_rotations.as_matrix()
            return MockTensor(result), None, None

        def compose(self, angles):
            state.compose_inputs.append(angles.value.copy())
            result = Rotation.from_euler("XYZ", angles.value.reshape(16, 3)).as_rotvec()
            result[0] = 0  # Mirror the root-dropping upstream API behavior.
            return MockTensor(result[None])

    fake_modules = {
        "torch": SimpleNamespace(
            as_tensor=lambda value, **kwargs: MockTensor(np.asarray(value, dtype=np.float32)),
            float32=np.float32, no_grad=nullcontext, __version__="mock-not-a-real-runtime",
        ),
        "manotorch": SimpleNamespace(__file__="/mock/manotorch/__init__.py", __version__="mock-version"),
        "manotorch.manolayer": SimpleNamespace(ManoLayer=MockManoLayer, __file__="/mock/manotorch/manolayer.py"),
        "manotorch.axislayer": SimpleNamespace(AxisLayerFK=MockAxisLayerFK, __file__="/mock/manotorch/axislayer.py"),
    }
    state.imports = []

    def fake_import(name):
        state.imports.append(name)
        return fake_modules[name]

    def missing_metadata(name):
        raise importlib.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(mano.importlib, "import_module", fake_import)
    monkeypatch.setattr(mano.importlib.metadata, "version", missing_metadata)
    state.modules = fake_modules
    return mano.ManoBackend(tmp_path), state


def test_constructor_is_lazy_and_model_paths_are_explicit(mocked_backend):
    backend, state = mocked_backend
    assert not state.imports
    assert backend.model_file.name == "MANO_RIGHT.pkl"


def test_missing_model_is_clear_before_import(tmp_path, monkeypatch):
    def forbidden_import(name):
        pytest.fail(f"Must validate assets before importing {name}")
    monkeypatch.setattr(mano.importlib, "import_module", forbidden_import)
    with pytest.raises(mano.DependencyError, match="models/MANO_RIGHT.pkl"):
        mano.ManoBackend(tmp_path)
    with pytest.raises(mano.DependencyError, match="models/MANO_LEFT.pkl"):
        mano.ManoBackend(tmp_path, hand="left")
    with pytest.raises(ValueError, match="hand"):
        mano.ManoBackend(tmp_path, hand="both")


def test_left_asset_selection_reaches_both_model_layers(mocked_backend):
    right, state = mocked_backend
    (right.assets_root / "models" / "MANO_LEFT.pkl").write_bytes(b"mock left model, never deserialized")
    left = mano.ManoBackend(right.assets_root, hand="left")
    result = left.export_rest()
    assert left.model_file.name == "MANO_LEFT.pkl"
    assert state.mano_kwargs["side"] == state.axis_kwargs["side"] == "left"
    assert result["provenance"]["hand"] == "left"
    assert right.hand == "right" and right._mano is None


def test_optional_dependency_failure_is_clear(mocked_backend, monkeypatch):
    backend, _ = mocked_backend
    def unavailable(name):
        raise ModuleNotFoundError("No module named 'torch'")
    monkeypatch.setattr(mano.importlib, "import_module", unavailable)
    with pytest.raises(mano.DependencyError, match="torch"):
        backend.export_rest()


def test_resource_load_failure_is_not_reported_as_missing_torch(mocked_backend):
    backend, state = mocked_backend
    def bad_model(**kwargs):
        raise ValueError("invalid pickle model structure")
    state.modules["manotorch.manolayer"].ManoLayer = bad_model
    with pytest.raises(mano.ModelResourceError, match="MANO_RIGHT.pkl.*invalid pickle"):
        backend.export_rest()
    assert backend._mano is None


def test_export_rest_uses_model_axes_and_zero_shape(mocked_backend):
    backend, state = mocked_backend
    result = backend.export_rest()
    np.testing.assert_allclose(
        Rotation.from_quat(result["rest_global_xyzw"]).as_matrix(), state.rest_rotations.as_matrix(), atol=1e-7
    )
    assert result["parents"] == [-1, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14]
    np.testing.assert_array_equal(state.mano_calls[0][0], np.zeros((16, 3)))
    np.testing.assert_array_equal(state.mano_calls[0][1], np.zeros((1, 10)))
    assert state.mano_kwargs["use_pca"] is False
    assert state.mano_kwargs["flat_hand_mean"] is True
    assert state.mano_kwargs["rot_mode"] == "axisang"
    assert state.mano_kwargs["center_idx"] == 0
    assert state.mano_kwargs["side"] == state.axis_kwargs["side"] == "right"
    provenance = result["provenance"]
    assert provenance["fsglove_reference_commit"] == mano.FSGLOVE_REFERENCE_COMMIT
    assert provenance["manotorch_version"] == "mock-version"
    assert provenance["manotorch_path"] == "/mock/manotorch/__init__.py"
    assert provenance["shape_calibrated"] is False
    json.dumps(result)


def test_render_explicit_nonidentity_root_is_not_lost_or_doubled(mocked_backend):
    backend, state = mocked_backend
    angles = np.zeros((16, 3))
    angles[0] = [0.8, -0.2, 0.3]  # Must be overridden by explicit quaternion.
    angles[2] = [0.1, -0.2, 0.5]
    original = angles.copy()
    root = Rotation.from_euler("XYZ", [0.3, 0.4, -0.7])
    result = backend.render(angles, root.as_quat() * 2)
    np.testing.assert_allclose(result["vertices_m"], root.apply(state.template), atol=1e-7)
    np.testing.assert_allclose(Rotation.from_rotvec(result["mano_pose_rotvec"][0]).as_matrix(), root.as_matrix(), atol=1e-7)
    np.testing.assert_array_equal(state.compose_inputs[0][0, 0], np.zeros(3))
    np.testing.assert_array_equal(angles, original)
    np.testing.assert_array_equal(state.mano_calls[-1][1], np.zeros((1, 10)))
    assert result["faces"] == [[0, 1, 2]]
    assert len(result["joints_m"]) == 21
    json.dumps(result)


def test_render_implicit_root_uses_xyz_euler(mocked_backend):
    backend, state = mocked_backend
    angles = np.zeros((16, 3))
    angles[0] = [-0.5, 0.3, 0.8]
    result = backend.render(angles)
    expected = Rotation.from_euler("XYZ", angles[0])
    np.testing.assert_allclose(result["vertices_m"], expected.apply(state.template), atol=1e-7)
    identity = backend.render(np.zeros((16, 3)))
    np.testing.assert_allclose(identity["vertices_m"], state.template)


@pytest.mark.parametrize("angles,quat", [
    (np.zeros((15, 3)), None),
    (np.full((16, 3), np.nan), None),
    (np.zeros((16, 3)), [0, 0, 0, 0]),
    (np.zeros((16, 3)), [0, 0, 1]),
    (np.zeros((16, 3)), [0, 0, np.inf, 1]),
])
def test_invalid_input_does_not_import_backend(mocked_backend, angles, quat):
    backend, state = mocked_backend
    with pytest.raises(ValueError):
        backend.render(angles, quat)
    assert not state.imports


def test_doctor_does_not_import_or_load_models(tmp_path, monkeypatch):
    monkeypatch.setattr(mano.importlib.util, "find_spec", lambda name: None)
    def forbidden_import(name):
        pytest.fail(f"doctor must not import {name}")
    monkeypatch.setattr(mano.importlib, "import_module", forbidden_import)
    result = mano.doctor(tmp_path)
    assert not result["ok"]
    assert "presence_only" in result["check"]
    assert len(result["issues"]) == 3
    assert str(tmp_path / "models" / "MANO_RIGHT.pkl") in result["issues"][-1]
    json.dumps(result)


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_MANO_ASSETS"), reason="No explicit licensed MANO assets supplied; mock tests are not integration validation")
def test_real_mano_root_rotation_and_rest_integration():
    """Opt-in: a configured but broken real backend fails, rather than skipping."""
    backend = mano.ManoBackend(Path(os.environ["FSGLOVE12_MANO_ASSETS"]))
    rest = backend.export_rest()
    assert np.asarray(rest["rest_global_xyzw"]).shape == (16, 4)
    angles = np.zeros((16, 3))
    angles[1] = [0.0, 0.1, 0.3]
    neutral_root = backend.render(angles)
    root = Rotation.from_euler("XYZ", [0.3, -0.4, 0.6])
    rotated = backend.render(angles, root.as_quat())
    np.testing.assert_allclose(rotated["vertices_m"], root.apply(neutral_root["vertices_m"]), atol=2e-6)
    np.testing.assert_allclose(rotated["joints_m"], root.apply(neutral_root["joints_m"]), atol=2e-6)
    np.testing.assert_allclose(np.asarray(rotated["joints_m"])[0], np.zeros(3), atol=1e-7)
    assert np.asarray(rotated["vertices_m"]).shape == (778, 3)
    assert rest["provenance"]["shape_calibrated"] is False
