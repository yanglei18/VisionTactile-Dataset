"""Opt-in regression against actual pinned FSGlove and licensed MANO assets.

Set FSGLOVE12_UPSTREAM_CHECKOUT and FSGLOVE12_MANO_ASSETS. No device is opened.
The before comparison instantiates the untouched original LEFT class, not a
reimplementation of its squared-axis behavior.
"""
import os
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_installation_frame import moving_hand, poses
from test_upstream_backend import official_core
from vt_fsglove12.config import PARENTS
from vt_fsglove12.installation_frame import REVISION, left_calibrator_type
from vt_fsglove12.upstream_inference import official_pose_outputs


def construct(core, cls, inputs, hand):
    bundle = core.calibration.CalibrationQuatSet(
        base_pose=inputs[0], z_rot_pose_1=inputs[1], z_rot_pose_2=inputs[2])
    with core.seeded(0):
        return cls(bundle, side=hand)


def cumulative_rest(calibrator):
    delta = Rotation.from_rotvec(calibrator._delta_rot_fa_rotvec)
    rest = [Rotation.identity()]
    for slot, parent in enumerate(PARENTS[1:], 1):
        rest.append(rest[parent] * delta[slot])
    return Rotation.concatenate(rest)


@pytest.mark.parametrize("articulated", [False, True])
def test_actual_left_core_preserves_joints_when_whole_hand_direction_changes(official_core, articulated):
    core = official_core
    if not (Path(os.environ["FSGLOVE12_MANO_ASSETS"]) / "models/MANO_LEFT.pkl").is_file():
        pytest.skip("Licensed left-hand MANO model not supplied")
    axis = np.array([-.6, .8, 0.])
    mount, heading, inputs = poses(axis, np.linspace(-2.8, 2.9, 16))
    original_type = core.calibration.Raw2MANOCalibrator
    original = construct(core, original_type, inputs, "left")
    repaired = construct(core, left_calibrator_type(original_type), inputs, "left")
    assert repaired.installation_coordinate_policy == REVISION
    rest = cumulative_rest(original)
    np.testing.assert_array_equal(original._delta_rot_fa_rotvec, repaired._delta_rot_fa_rotvec)
    previous_children = None
    original_errors = []
    for root_rotvec in ([0., 0., 0.], [.2, -.4, .3], [1.4, -.8, 2.0], [-1., .7, -.8]):
        raw, _, expected = moving_hand(axis, mount, heading, Rotation.from_rotvec(root_rotvec),
                                       articulated, rest=rest)
        actual = repaired.get_mano_param(raw, ana_repr=True).numpy()
        before = original.get_mano_param(raw, ana_repr=True).numpy()
        assert np.max((expected.inv() * Rotation.from_rotvec(actual)).magnitude()) < 1e-10
        original_errors.append(np.max((expected.inv() * Rotation.from_rotvec(before)).magnitude()))
        if previous_children is not None:
            assert np.max((Rotation.from_rotvec(previous_children).inv()
                           * Rotation.from_rotvec(actual[1:])).magnitude()) < 1e-10
        previous_children = actual[1:].copy()
        # The adapter still calls the original FK/composer once with the same
        # scalar conversions and float32 MANO output as the actual class.
        mano, anatomy, _ = official_pose_outputs(repaired, raw, core.torch)
        np.testing.assert_array_equal(anatomy, actual)
        np.testing.assert_array_equal(mano, repaired.get_mano_param(raw).numpy())
    assert max(original_errors) > np.deg2rad(10.), "Fixture must expose the original LEFT defect"


def test_left_specialization_preserves_original_right_class_and_outputs(official_core):
    models = Path(os.environ["FSGLOVE12_MANO_ASSETS"]) / "models"
    if not all((models / f"MANO_{side}.pkl").is_file() for side in ("LEFT", "RIGHT")):
        pytest.skip("This parity test uses both licensed MANO models")
    core = official_core
    _, _, inputs = poses(np.array([-.6, .8, 0.]), np.linspace(-1.2, 1.5, 16))
    original_type = core.calibration.Raw2MANOCalibrator
    original_method = original_type._get_global_installation_error
    before = construct(core, original_type, inputs, "right")
    left_type = left_calibrator_type(original_type)
    construct(core, left_type, inputs, "left")
    after = construct(core, original_type, inputs, "right")
    assert original_type._get_global_installation_error is original_method
    assert left_type.get_mano_param is original_type.get_mano_param
    assert left_type._get_local_installation_error is original_type._get_local_installation_error
    for name in ("_global_installation_error", "_local_installation_error", "_base_quat", "_delta_rot_fa_rotvec"):
        np.testing.assert_array_equal(getattr(before, name), getattr(after, name))
    raw = Rotation.random(16, random_state=np.random.default_rng(883))
    np.testing.assert_array_equal(before.get_mano_param(raw).numpy(), after.get_mano_param(raw).numpy())
    with pytest.raises(ValueError, match="requires side='left'"):
        construct(core, left_type, inputs, "right")
