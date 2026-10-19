"""Exact execution-reuse parity and calibration-cache invalidation checks."""
from copy import deepcopy
import warnings

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_upstream_backend import official_core, fit_data, synthetic_state
from test_upstream_priors import encode, inputs
from vt_fsglove12.upstream_backend import complete_missing_quaternions
from vt_fsglove12.upstream_inference import (
    CalibrationRotationCache, EXECUTION_REVISION, official_pose_outputs,
)
from vt_fsglove12.upstream_priors import complete_with_priors


def original_zero_completion(calibrator, raw, observed):
    """Pre-cache adapter equations, preserving their scalar evaluation order."""
    result = raw.copy()
    base = Rotation.from_quat(calibrator._base_quat)
    global_error = Rotation.from_rotvec(calibrator._global_installation_error)
    local_error = Rotation.from_rotvec(calibrator._local_installation_error)
    rest_delta = Rotation.from_rotvec(calibrator._delta_rot_fa_rotvec)
    fk = [None] * 16
    fk[0] = global_error[0] * base[0].inv() * Rotation.from_quat(result[0]) * local_error[0].inv()
    for slot in calibrator._fk_seq:
        current = fk[int(calibrator._joint_fa[slot])] * rest_delta[slot]
        if observed[slot]:
            fk[slot] = global_error[slot] * base[slot].inv() * Rotation.from_quat(result[slot]) * local_error[slot].inv()
        else:
            result[slot] = (base[slot] * global_error[slot].inv() * current * local_error[slot]).as_quat()
            fk[slot] = current
    return result


@pytest.mark.parametrize("missing", [[3, 6, 9, 12, 13], [1, 2, 3, 5, 9, 13, 14, 15], list(range(1, 16))])
def test_cached_zero_completion_is_bitwise_equal_to_original_scalar_formula(missing):
    calibrator = synthetic_state()
    raw = Rotation.random(16, random_state=np.random.default_rng(22)).as_quat()
    raw[[1, 5, 11]] *= -1
    observed = np.ones(16, dtype=bool)
    observed[missing] = False
    expected = original_zero_completion(calibrator, raw, observed)
    cache = CalibrationRotationCache()
    for _ in range(2):
        actual = complete_missing_quaternions(calibrator, raw, observed, rotation_cache=cache)
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(actual[observed], raw[observed])


@pytest.mark.parametrize("field", ["_base_quat", "_global_installation_error",
                                  "_local_installation_error", "_delta_rot_fa_rotvec"])
@pytest.mark.parametrize("replace", [False, True])
def test_cache_rebuilds_after_inplace_or_replacement_calibration_changes(field, replace):
    calibrator = synthetic_state()
    _, raw, observed = inputs(calibrator)
    cache = CalibrationRotationCache()
    first = cache.get(calibrator)
    assert cache.get(calibrator) is first
    previous = deepcopy(vars(calibrator))
    value = getattr(calibrator, field).copy()
    if field == "_base_quat":
        value = (Rotation.from_rotvec([.1, -.2, .3]) * Rotation.from_quat(value)).as_quat()
    else:
        value[1, 1] += .1
    if replace:
        setattr(calibrator, field, value)
    else:
        getattr(calibrator, field)[:] = value
    changed = cache.get(calibrator)
    assert changed is not first and cache.get(calibrator) is changed
    cached, detail = complete_with_priors(calibrator, raw, observed, {}, rotation_cache=cache)
    uncached, expected_detail = complete_with_priors(calibrator, raw, observed, {})
    np.testing.assert_array_equal(cached, uncached)
    assert detail == expected_detail
    np.testing.assert_array_equal(
        complete_missing_quaternions(calibrator, raw, observed, rotation_cache=cache),
        original_zero_completion(calibrator, raw, observed))
    # Restoring previous values is another content change, not a cache hit.
    for name, saved in previous.items():
        setattr(calibrator, name, saved)
    assert cache.get(calibrator) is not changed


def test_cache_is_backend_owned_and_rejects_other_calibrator_identity():
    first = synthetic_state()
    before = deepcopy(vars(first))
    cache = CalibrationRotationCache()
    cached = cache.get(first)
    other = deepcopy(first)
    assert cache.get(other) is not cached
    assert vars(first).keys() == before.keys()
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(first, name), value)
    cache.clear()
    assert cache._calibrator is None and cache._rotations is None
    assert cache._snapshots == ()


def test_full16_does_not_access_or_fill_rotation_cache():
    class NoCacheAccess:
        def get(self, _):
            pytest.fail("full16 must not access calibration rotations")
    raw = Rotation.random(16, random_state=np.random.default_rng(6)).as_quat()
    observed = np.ones(16, dtype=bool)
    cache = NoCacheAccess()
    np.testing.assert_array_equal(
        complete_missing_quaternions(object(), raw, observed, rotation_cache=cache), raw)
    result, _ = complete_with_priors(object(), raw, observed, {}, rotation_cache=cache)
    np.testing.assert_array_equal(result, raw)


@pytest.mark.parametrize("kind", ["random", "identity", "near_pi", "gimbal"])
def test_one_official_fk_matches_original_two_calls_bitwise(official_core, monkeypatch, kind):
    backend, profile, _ = fit_data(official_core)
    calibrator = backend._calibrator(profile)
    rng = np.random.default_rng(181)
    original = calibrator.get_mano_param
    calls = []

    def counted(rotations, ana_repr=False, map_to_world=False):
        calls.append((ana_repr, map_to_world))
        return original(rotations, ana_repr=ana_repr, map_to_world=map_to_world)

    monkeypatch.setattr(calibrator, "get_mano_param", counted)
    for sample in range(8):
        angles = rng.uniform(-1., 1., (16, 3))
        if kind == "identity":
            angles[:] = 0.
        elif kind == "near_pi":
            angles[:, 2] = (np.pi - 1e-8) * (-1 if sample % 2 else 1)
        elif kind == "gimbal":
            angles[:, 1] = np.pi / 2 + (sample - 4) * 1e-8
        raw = encode(calibrator, angles)
        raw[::2] *= -1
        rotations = Rotation.from_quat(raw)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            expected_mano = original(rotations).numpy()
            expected_anatomical = original(rotations, ana_repr=True).numpy()
            expected_euler = Rotation.from_rotvec(expected_anatomical).as_euler("XYZ")
        actual_mano, actual_anatomical, actual_euler = official_pose_outputs(
            calibrator, rotations, official_core.torch)
        np.testing.assert_array_equal(actual_mano, expected_mano)
        np.testing.assert_array_equal(actual_anatomical, expected_anatomical)
        np.testing.assert_array_equal(actual_euler, expected_euler)
        assert actual_mano.dtype == np.float32
        assert actual_anatomical.dtype == actual_euler.dtype == np.float64
    assert calls == [(True, False)] * 8


def test_backend_records_execution_reuse_and_clears_cache_on_reset(official_core, monkeypatch):
    backend, profile, frame = fit_data(official_core)
    calibrator = backend._calibrator(profile)
    original = calibrator.get_mano_param
    calls = []

    def counted(rotations, ana_repr=False, map_to_world=False):
        calls.append((ana_repr, map_to_world))
        return original(rotations, ana_repr=ana_repr, map_to_world=map_to_world)

    monkeypatch.setattr(calibrator, "get_mano_param", counted)
    pose = backend.solve(frame, profile)
    assert calls == [(True, False)]
    assert pose["adapter_execution"] == {
        "revision": EXECUTION_REVISION, "official_anatomical_fk_calls": 1,
        "original_axis_compose": True, "pinned_numerical_ast_modified": False,
        "sparse_rotation_cache": "calibrator_identity_and_state_content",
    }
    assert profile["upstream_provenance"] == official_core.provenance
    assert backend._rotation_cache._calibrator is calibrator
    backend.reset_calibration()
    assert backend._rotation_cache._calibrator is None
    assert backend._rotation_cache._rotations is None
    restored = backend.solve(frame, profile)
    assert backend._rotation_cache._calibrator is not calibrator
    np.testing.assert_array_equal(restored["mano_pose_rotvec"], pose["mano_pose_rotvec"])
