"""Frame-local missing-DIP priors, not glove-accuracy measurements."""
from copy import deepcopy
import os
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import FINGER_DIPS, PARENTS
from vt_fsglove12.upstream_priors import DEFAULT_DIP_MAX_RAD, PRIOR_POLICY, complete_with_priors


def state(seed=41):
    rng = np.random.default_rng(seed)
    return SimpleNamespace(
        _base_quat=Rotation.random(16, random_state=rng).as_quat(),
        _global_installation_error=Rotation.random(16, random_state=rng).as_rotvec(),
        _local_installation_error=Rotation.random(16, random_state=rng).as_rotvec(),
        _delta_rot_fa_rotvec=Rotation.random(16, random_state=rng).as_rotvec(),
        _joint_fa=np.array([0, *PARENTS[1:]]),
        _fk_seq=[1, 4, 7, 10, 13, 2, 5, 8, 11, 14, 3, 6, 9, 12, 15],
    )


def encode(calibrator, angles):
    """Test oracle: synthesize 16 inputs from specified anatomical local poses."""
    local = Rotation.from_euler("XYZ", angles)
    delta = Rotation.from_rotvec(calibrator._delta_rot_fa_rotvec)
    fk = [None] * 16
    fk[0] = local[0]
    for slot in calibrator._fk_seq:
        fk[slot] = fk[calibrator._joint_fa[slot]] * delta[slot] * local[slot]
    fk = Rotation.from_quat([item.as_quat() for item in fk])
    base = Rotation.from_quat(calibrator._base_quat)
    global_error = Rotation.from_rotvec(calibrator._global_installation_error)
    local_error = Rotation.from_rotvec(calibrator._local_installation_error)
    return (base * global_error.inv() * fk * local_error).as_quat()


def decode(calibrator, quaternions):
    """Independent matrix check of the official local-rotation equation."""
    base = Rotation.from_quat(calibrator._base_quat)
    global_error = Rotation.from_rotvec(calibrator._global_installation_error)
    local_error = Rotation.from_rotvec(calibrator._local_installation_error)
    fk = global_error * base.inv() * Rotation.from_quat(quaternions) * local_error.inv()
    delta = Rotation.from_rotvec(calibrator._delta_rot_fa_rotvec)
    local = [fk[0]] + [(fk[calibrator._joint_fa[slot]] * delta[slot]).inv() * fk[slot]
                       for slot in range(1, 16)]
    return Rotation.from_quat([item.as_quat() for item in local]).as_euler("XYZ")


def inputs(calibrator):
    angles = np.zeros((16, 3))
    angles[0] = [.4, -.1, .2]
    angles[[1, 4, 7, 10, 13]] = [.1, -.2, .25]
    angles[[2, 5, 8, 11], 2] = [1.1, .8, .6, .4]
    observed = np.ones(16, dtype=bool)
    observed[[3, 6, 9, 12, 13]] = False
    return angles, encode(calibrator, angles), observed


@pytest.mark.parametrize("seed", [4, 27, 111])
def test_random_matrices_recover_exact_coupled_targets(seed):
    calibrator = state(seed)
    angles, raw, observed = inputs(calibrator)
    completed, diagnostics = complete_with_priors(calibrator, raw, observed, {"dip_ratio": .65})
    np.testing.assert_array_equal(completed[observed], raw[observed])
    actual = decode(calibrator, completed)
    for finger, slot in FINGER_DIPS.items():
        np.testing.assert_allclose(actual[slot], [0., 0., .65 * angles[slot - 1, 2]], atol=2e-15)
        entry = diagnostics["dip_by_finger"][finger]
        assert entry["applied"] and entry["source"] == "inferred_not_measured"
        assert not entry["clip_applied"]
    np.testing.assert_allclose(actual[13], [0., 0., 0.], atol=2e-15)
    assert diagnostics["coupled_dip_slots"] == [3, 6, 9, 12]
    assert diagnostics["zero_prior_slots"] == [13]
    assert diagnostics["policy"] == PRIOR_POLICY and not diagnostics["inference_is_measurement"]


def test_full16_preserves_every_input_value_and_does_not_touch_calibrator():
    raw = Rotation.random(16, random_state=np.random.default_rng(54)).as_quat()
    raw[[3, 5, 9]] *= -1
    result, diagnostics = complete_with_priors(object(), raw, np.ones(16, dtype=bool), {})
    np.testing.assert_array_equal(result, raw)
    assert not diagnostics["virtual_input_slots"] and not diagnostics["coupled_dip_slots"]
    assert all(item["reason"] == "dip_observed_input_unchanged"
               for item in diagnostics["dip_by_finger"].values())


def test_observed_dip_is_not_coupled_even_when_other_dips_are_missing():
    calibrator = state()
    angles, _, observed = inputs(calibrator)
    observed[3] = True
    angles[3] = [.1, -.15, .9]
    raw = encode(calibrator, angles)
    result, diagnostics = complete_with_priors(calibrator, raw, observed, {})
    np.testing.assert_array_equal(result[3], raw[3])
    np.testing.assert_allclose(decode(calibrator, result)[3], angles[3], atol=2e-15)
    assert not diagnostics["dip_by_finger"]["index"]["applied"]


def test_per_finger_ratios_zero_ratio_extension_and_upper_clip_are_explicit():
    calibrator = state()
    angles, _, observed = inputs(calibrator)
    angles[[2, 5, 8, 11], 2] = [2.2, -.5, .9, 1.2]
    raw = encode(calibrator, angles)
    config = {"dip_ratio": .65, "dip_ratio_by_finger": {"index": 1., "little": 0., "ring": .25}}
    result, diagnostics = complete_with_priors(calibrator, raw, observed, config)
    actual = decode(calibrator, result)
    np.testing.assert_allclose(actual[[3, 6, 9, 12], 2], [DEFAULT_DIP_MAX_RAD, 0., 0., .3], atol=2e-15)
    assert diagnostics["dip_by_finger"]["index"]["clip_applied"]
    assert diagnostics["dip_by_finger"]["middle"]["clip_applied"]
    assert diagnostics["dip_by_finger"]["little"]["applied"]
    assert diagnostics["dip_by_finger"]["little"]["ratio"] == 0.
    result, diagnostics = complete_with_priors(calibrator, raw, observed, {**config, "upstream_dip_max_rad": 1.})
    assert abs(decode(calibrator, result)[3, 2] - 1.) < 2e-15
    assert diagnostics["dip_by_finger"]["index"]["limits_rad"] == [0., 1.]


@pytest.mark.parametrize("missing", [[1], [2], [1, 2]])
def test_dependency_loss_immediately_falls_back_without_previous_frame(missing):
    calibrator = state()
    _, raw, observed = inputs(calibrator)
    first, _ = complete_with_priors(calibrator, raw, observed, {})
    assert decode(calibrator, first)[3, 2] > .5
    lost = observed.copy()
    lost[missing] = False
    second, diagnostics = complete_with_priors(calibrator, raw, lost, {})
    np.testing.assert_allclose(decode(calibrator, second)[3], 0., atol=2e-15)
    entry = diagnostics["dip_by_finger"]["index"]
    assert entry["reason"] == "pip_parent_or_child_not_observed_neutral_fallback"
    assert entry["missing_dependency_slots"] == missing and not entry["applied"]
    assert diagnostics["depends_on_previous_frame"] is False
    again, _ = complete_with_priors(calibrator, raw, observed, {})
    np.testing.assert_array_equal(again, first)


@pytest.mark.parametrize("config", [
    {"dip_ratio": -1.}, {"dip_ratio": 1.01}, {"dip_ratio": True},
    {"dip_ratio_by_finger": {"thumb": .5}}, {"dip_ratio_by_finger": {"index": float("nan")}},
    {"dip_ratio_by_finger": []}, {"upstream_dip_max_rad": 0.},
    {"upstream_dip_max_rad": np.pi}, {"upstream_dip_max_rad": float("inf")},
    {"upstream_dip_max_rad": True},
])
def test_invalid_prior_parameters_are_rejected(config):
    calibrator = state()
    _, raw, observed = inputs(calibrator)
    with pytest.raises(ValueError):
        complete_with_priors(calibrator, raw, observed, config)


@pytest.mark.parametrize("kind", ["root", "mask", "quaternion", "nan"])
def test_invalid_input_is_not_promoted_to_observation(kind):
    calibrator = state()
    _, raw, observed = inputs(calibrator)
    if kind == "root":
        observed[0] = False
    elif kind == "mask":
        observed = observed.astype(int)
    elif kind == "quaternion":
        raw[1] = [0., 0., 0., 0.]
    else:
        raw[1, 0] = np.nan
    with pytest.raises(ValueError):
        complete_with_priors(calibrator, raw, observed, {})


def test_function_does_not_mutate_config_raw_or_calibrator():
    calibrator = state()
    _, raw, observed = inputs(calibrator)
    original, original_state, original_mask = raw.copy(), deepcopy(vars(calibrator)), observed.copy()
    config = {"dip_ratio": .6, "dip_ratio_by_finger": {"little": .8}}
    config_copy = deepcopy(config)
    complete_with_priors(calibrator, raw, observed, config)
    np.testing.assert_array_equal(raw, original)
    np.testing.assert_array_equal(observed, original_mask)
    for name in original_state:
        np.testing.assert_array_equal(getattr(calibrator, name), original_state[name])
    assert config == config_copy


@pytest.fixture(scope="module")
def real_calibrator():
    checkout, assets = os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT"), os.environ.get("FSGLOVE12_MANO_ASSETS")
    if not checkout or not assets:
        pytest.skip("explicit trusted official checkout and MANO resource opt-in required")
    from vt_fsglove12.upstream_core import load_upstream_core
    core = load_upstream_core(checkout, assets)
    core.torch.set_num_threads(1)
    base = Rotation.random(16, random_state=np.random.default_rng(7))
    return core.create_calibrator(base.as_quat(),
        (Rotation.from_rotvec([.6, 0., 0.]) * base).as_quat(),
        (Rotation.from_rotvec([-.6, 0., 0.]) * base).as_quat())


def test_real_official_inference_exactly_recovers_declared_coupled_prior(real_calibrator):
    angles, raw, observed = inputs(real_calibrator)
    result, diagnostics = complete_with_priors(real_calibrator, raw, observed, {})
    anatomy = real_calibrator.get_mano_param(Rotation.from_quat(result), ana_repr=True).numpy()
    actual = Rotation.from_rotvec(anatomy).as_euler("XYZ")
    for slot in FINGER_DIPS.values():
        np.testing.assert_allclose(actual[slot], [0., 0., .65 * angles[slot - 1, 2]], atol=2e-15)
    np.testing.assert_allclose(actual[13], 0., atol=2e-15)
    assert diagnostics["coupled_dip_slots"] == [3, 6, 9, 12]
    mano = real_calibrator.get_mano_param(Rotation.from_quat(result)).numpy()
    assert np.isfinite(mano).all() and mano.shape == (16, 3)


def test_real_full16_output_is_identical_to_unwrapped_official_inference(real_calibrator):
    _, raw, _ = inputs(real_calibrator)
    expected = real_calibrator.get_mano_param(Rotation.from_quat(raw)).numpy()
    completed, _ = complete_with_priors(real_calibrator, raw, np.ones(16, dtype=bool), {})
    np.testing.assert_array_equal(completed, raw)
    actual = real_calibrator.get_mano_param(Rotation.from_quat(completed)).numpy()
    np.testing.assert_array_equal(actual, expected)
