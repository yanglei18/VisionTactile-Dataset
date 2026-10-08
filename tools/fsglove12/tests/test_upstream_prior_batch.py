"""Batched sparse completion against the previous scalar adapter equations."""
from copy import deepcopy
import json
import os
from pathlib import Path
from types import SimpleNamespace
import warnings

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_upstream_backend import synthetic_state
from vt_fsglove12.config import FINGER_DIPS
from vt_fsglove12.upstream_inference import CalibrationRotationCache
from vt_fsglove12.upstream_priors import _parameters, complete_with_priors


def scalar_prior_reference(calibrator, raw, observed, config):
    """Frozen pre-batching loop; full16 only supplies unchanged diagnostic keys."""
    result = raw.copy()
    _, diagnostics = complete_with_priors(calibrator, raw, np.ones(16, dtype=bool), config)
    diagnostics["virtual_input_slots"] = np.flatnonzero(~observed).tolist()
    for finger, slot in FINGER_DIPS.items():
        if not observed[slot]:
            diagnostics["dip_by_finger"][finger].update(
                source="inferred_not_measured", reason="pending_missing_input")
    if observed.all():
        return result, diagnostics
    ratios, maximum = _parameters(config)
    base = Rotation.from_quat(calibrator._base_quat)
    global_error = Rotation.from_rotvec(calibrator._global_installation_error)
    local_error = Rotation.from_rotvec(calibrator._local_installation_error)
    rest_delta = Rotation.from_rotvec(calibrator._delta_rot_fa_rotvec)
    dip_fingers = {slot: finger for finger, slot in FINGER_DIPS.items()}
    fk, local = [None] * 16, [None] * 16
    fk[0] = global_error[0] * base[0].inv() * Rotation.from_quat(result[0]) * local_error[0].inv()
    local[0] = fk[0]
    for slot in calibrator._fk_seq:
        slot = int(slot)
        parent = int(calibrator._joint_fa[slot])
        current = fk[parent] * rest_delta[slot]
        if observed[slot]:
            fk[slot] = global_error[slot] * base[slot].inv() * Rotation.from_quat(result[slot]) * local_error[slot].inv()
            local[slot] = current.inv() * fk[slot]
            continue
        target = np.zeros(3)
        finger = dip_fingers.get(slot)
        if finger is not None:
            entry = diagnostics["dip_by_finger"][finger]
            missing = [item for item in [slot - 2, slot - 1] if not observed[item]]
            if missing:
                entry.update(reason="pip_parent_or_child_not_observed_neutral_fallback",
                             missing_dependency_slots=missing, target_flexion_rad=0.)
            else:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", UserWarning)
                    pip_z = float(local[parent].as_euler("XYZ")[2])
                unbounded = ratios[finger] * pip_z
                target[2] = min(maximum, ratios[finger] * max(0., pip_z))
                entry.update(applied=True, reason="current_observed_pip_coupling",
                             pip_flexion_rad=pip_z, unclipped_target_rad=unbounded,
                             target_flexion_rad=float(target[2]),
                             clip_applied=bool(unbounded < 0. or unbounded > maximum),
                             missing_dependency_slots=[])
                diagnostics["coupled_dip_slots"].append(slot)
        if not np.any(target):
            diagnostics["zero_prior_slots"].append(slot)
        diagnostics["target_anatomical_euler_rad_by_slot"][str(slot)] = target.tolist()
        local[slot] = Rotation.from_euler("XYZ", target)
        fk[slot] = current * local[slot]
        result[slot] = (base[slot] * global_error[slot].inv() * fk[slot] * local_error[slot]).as_quat()
    diagnostics["coupled_dip_slots"].sort()
    diagnostics["zero_prior_slots"].sort()
    return result, diagnostics


def assert_scalar_parity(calibrator, raw, observed, config, cache):
    expected, expected_diagnostics = scalar_prior_reference(calibrator, raw, observed, config)
    actual, diagnostics = complete_with_priors(
        calibrator, raw, observed, config, rotation_cache=cache)
    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(actual[observed], raw[observed])
    assert diagnostics == expected_diagnostics
    error = (Rotation.from_quat(expected).inv() * Rotation.from_quat(actual)).magnitude()
    assert float(np.max(error)) < 1e-15


@pytest.mark.parametrize("seed", [4, 31, 83, 105])
def test_batch_matches_scalar_for_random_masks_and_calibration_changes(seed):
    rng = np.random.default_rng(seed)
    calibrator, cache = synthetic_state(), CalibrationRotationCache()
    original_state = deepcopy(vars(calibrator))
    for index in range(64):
        raw = Rotation.random(16, random_state=rng).as_quat()
        raw[::2] *= -1
        observed = rng.random(16) > .4
        observed[0] = True
        if index % 16 == 0:
            observed[:] = False
            observed[0] = True
        elif index % 16 == 1:
            observed[:] = True
        elif index % 16 == 2:
            observed[:] = True
            observed[[3, 6, 9, 12, 13]] = False
        config = dict(dip_ratio=float(rng.random()),
                      dip_ratio_by_finger={"index": 0., "little": 1.},
                      upstream_dip_max_rad=float(rng.uniform(.01, 3.)))
        if index == 32:
            calibrator._local_installation_error[5, 1] += .2
        assert_scalar_parity(calibrator, raw, observed, config, cache)
    # Cache creation must not mutate the source installation state.
    original_state["_local_installation_error"][5, 1] += .2
    for key, value in original_state.items():
        np.testing.assert_array_equal(getattr(calibrator, key), value)


def test_full_opt_in_recording_batch_matches_scalar():
    directory = os.environ.get("FSGLOVE12_PRIOR_PARITY_RECORDING")
    if not directory:
        pytest.skip("set FSGLOVE12_PRIOR_PARITY_RECORDING for read-only recorded-frame parity")
    directory = Path(directory)
    profile = json.loads((directory / "profile.json").read_text())
    config = json.loads((directory / "config.json").read_text())
    calibrator = SimpleNamespace(**{key: np.asarray(value) for key, value in profile["upstream_state"].items()})
    cache, count = CalibrationRotationCache(), 0
    with (directory / "orientations.jsonl").open() as stream:
        for line in stream:
            frame = json.loads(line)
            observed = np.zeros(16, dtype=bool)
            raw = np.tile([0., 0., 0., 1.], (16, 1))
            for node, slot in config["node_to_slot"].items():
                data = frame["nodes"].get(node)
                binding = profile["nodes"].get(node)
                if (data is not None and binding is not None
                        and data.get("epoch") == binding["epoch"]
                        and data.get("orientation_valid") is not False
                        and 0. <= data.get("age_s", float("inf")) <= config["stale_after_s"]):
                    observed[slot] = True
                    raw[slot] = data["quat_xyzw"]
            if observed[0]:
                assert_scalar_parity(calibrator, raw, observed, profile["missing_slot_options"], cache)
                count += 1
    assert count > 0
