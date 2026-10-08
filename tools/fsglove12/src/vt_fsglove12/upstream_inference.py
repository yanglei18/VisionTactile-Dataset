"""Execution reuse around the unchanged, pinned official numerical methods.

No official source or AST is patched here. One official anatomical FK result
feeds the original AxisLayerFK composer, using the same Euler conversion,
float32 cast and root overwrite as Raw2MANOCalibrator.get_mano_param. Sparse
input completion can separately reuse immutable installation rotations.
"""
from dataclasses import dataclass
import warnings

import numpy as np
from scipy.spatial.transform import Rotation


EXECUTION_REVISION = "official_anatomical_fk_once_original_axis_compose_batched_sparse_v2"
_ROTATION_FIELDS = (
    "_base_quat", "_global_installation_error", "_local_installation_error",
    "_delta_rot_fa_rotvec",
)


def official_pose_outputs(calibrator, rotations, torch):
    """Return MANO pose, anatomical rotvec and Euler without a second FK pass.

    The caller's live path uses map_to_world=False, matching the root overwrite
    below. Float64 anatomy/Euler and float32 composed MANO match upstream; do
    not move the float32 conversion before SciPy's Euler conversion.
    """
    anatomical = calibrator.get_mano_param(rotations, ana_repr=True).detach().cpu().numpy()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        euler = Rotation.from_rotvec(anatomical).as_euler("XYZ")
    with torch.no_grad():
        mano = calibrator._axislayer.compose(torch.from_numpy(euler.reshape(1, 16, 3)).float())[0]
        mano[0] = torch.from_numpy(anatomical[0])
    return mano.detach().cpu().numpy(), anatomical, euler


@dataclass(frozen=True)
class _CalibrationRotations:
    base: tuple
    base_inv: tuple
    global_error: tuple
    global_error_inv: tuple
    local_error: tuple
    local_error_inv: tuple
    rest_delta: tuple
    base_inv_batch: Rotation
    global_error_batch: Rotation
    local_error_inv_batch: Rotation

    @classmethod
    def from_arrays(cls, arrays):
        base_batch = Rotation.from_quat(arrays[0])
        global_error_batch = Rotation.from_rotvec(arrays[1])
        local_error_batch = Rotation.from_rotvec(arrays[2])
        base = tuple(base_batch)
        global_error = tuple(global_error_batch)
        local_error = tuple(local_error_batch)
        rest_delta = tuple(Rotation.from_rotvec(arrays[3]))
        # Keep the original scalar inversions and multiplication order. In
        # particular, do not pre-multiply adjacent constant transforms.
        return cls(base, tuple(item.inv() for item in base),
                   global_error, tuple(item.inv() for item in global_error),
                   local_error, tuple(item.inv() for item in local_error), rest_delta,
                   base_batch.inv(), global_error_batch, local_error_batch.inv())


class CalibrationRotationCache:
    """Backend-owned cache, invalidated by identity AND calibration contents.

    Checking snapshots also detects in-place updates and the public official
    correct_local_inst_error method. Nothing is attached to or frozen on the
    upstream calibrator, and no caller-owned array is modified.
    """
    def __init__(self):
        self.clear()

    def clear(self):
        self._calibrator = None
        self._snapshots = ()
        self._rotations = None

    def get(self, calibrator):
        arrays = tuple(np.asarray(getattr(calibrator, name)) for name in _ROTATION_FIELDS)
        if (calibrator is self._calibrator
                and all(np.array_equal(value, saved) for value, saved in zip(arrays, self._snapshots))):
            return self._rotations
        snapshots = tuple(value.copy() for value in arrays)
        rotations = _CalibrationRotations.from_arrays(snapshots)
        self._calibrator, self._snapshots, self._rotations = calibrator, snapshots, rotations
        return rotations


def calibration_rotations(calibrator, cache=None):
    """Use an explicit cache, or remain a stateless completion helper."""
    if cache is not None:
        return cache.get(calibrator)
    return _CalibrationRotations.from_arrays(
        tuple(getattr(calibrator, name) for name in _ROTATION_FIELDS))
