"""Versioned LEFT installation-frame adapter for the pinned FSGlove core.

The established three-pose contract supplies base_pose, z_rot_pose_1 (whole
hand tilted up), then z_rot_pose_2 (whole hand tilted down). Their ordered
rotation defines a directed horizontal axis. World +Z is gravity-up in the
VQF input frame. We keep that direction; handedness comes from MANO_LEFT,
not a guessed sensor sign reversal or a reflection of raw measurements.

For independent sensor headings E_i about gravity, V_i must transform as
E_i V. Then G_i=V_i^-1 B_i aligns their references before relative joint
rotations are formed. Squaring axis components, as the pinned LEFT branch
does, violates this identity. An orthonormal directed frame satisfies it.
This repairs coordinate consistency; it does not observe absolute yaw or
compensate time-varying gyroscope bias.
"""
from functools import lru_cache

import numpy as np
from scipy.spatial.transform import Rotation


REVISION = "left_directed_horizontal_frame_v1"
_NUMERICAL_EPS = 1e-10


def _quaternions(value, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (16, 4) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain finite (16, 4) xyzw quaternions")
    if not np.allclose(np.linalg.norm(array, axis=1), 1., rtol=0., atol=1e-5):
        raise ValueError(f"{name} must contain unit quaternions")
    return Rotation.from_quat(array)


def directed_installation_error(base_pose, z_rot_pose_1, z_rot_pose_2):
    """Return 16 global installation rotvecs in a right-handed SO(3) frame.

    Only numerically undefined frames are rejected: no sweep, a half-turn
    whose axis sign is ambiguous, or a sweep parallel to gravity. The small
    tolerance is a numerical uniqueness condition, not an action threshold.
    No source array is modified and no fallback axis is invented.
    """
    base = _quaternions(base_pose, "base_pose")
    first = _quaternions(z_rot_pose_1, "z_rot_pose_1")
    second = _quaternions(z_rot_pose_2, "z_rot_pose_2")
    sweep = (second * first.inv()).as_rotvec()
    angle = np.linalg.norm(sweep, axis=1)
    ambiguous = (angle <= _NUMERICAL_EPS) | (np.pi - angle <= _NUMERICAL_EPS)
    if np.any(ambiguous):
        slots = np.flatnonzero(ambiguous).tolist()
        raise ValueError(f"Calibration sweep has no unique directed axis at slots {slots}")
    sweep /= angle[:, None]
    y_axis = np.tile([0., 0., 1.], (16, 1))
    z_axis = sweep - y_axis * np.sum(sweep * y_axis, axis=1, keepdims=True)
    horizontal_norm = np.linalg.norm(z_axis, axis=1)
    if np.any(horizontal_norm <= _NUMERICAL_EPS):
        slots = np.flatnonzero(horizontal_norm <= _NUMERICAL_EPS).tolist()
        raise ValueError(f"Calibration sweep has no horizontal direction at slots {slots}")
    z_axis /= horizontal_norm[:, None]
    x_axis = np.cross(y_axis, z_axis)
    virtual_frame = Rotation.from_matrix(np.stack([x_axis, y_axis, z_axis], axis=2))
    return (virtual_frame.inv() * base).as_rotvec()


@lru_cache(maxsize=None)
def left_calibrator_type(upstream_type):
    """Specialize only global installation; retain original constructor/FK.

    The original constructor calls the override before deriving its local
    installation state, so both states use the same frame. The upstream class
    and source remain unchanged. The caller selects the original class for
    RIGHT, preserving its existing numerical path and serialized provenance.
    """
    class LeftDirectedFrameCalibrator(upstream_type):
        installation_coordinate_policy = REVISION

        def _get_global_installation_error(self, calibration_quat):
            if getattr(self.side, "value", self.side) != "left":
                raise ValueError("Directed LEFT installation adapter requires side='left'")
            return directed_installation_error(
                calibration_quat.base_pose,
                calibration_quat.z_rot_pose_1,
                calibration_quat.z_rot_pose_2,
            )

    return LeftDirectedFrameCalibrator
