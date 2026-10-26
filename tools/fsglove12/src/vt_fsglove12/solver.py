"""Experimental sparse-IMU adapter, not the upstream FSGlove/DiffHCal solver.

Input rotations are calibrated anatomical segment-to-world rotations, xyzw.
For child i, G_i = G_parent * rest_local_i * EulerXYZ(e_i); XYZ is intrinsic,
with x=twist, y=abduction and z=flexion. The measured root is never optimized.
This frame-wise fit adds neither smoothing nor an automatic rest/heading reset.

This multiplication order matches manotorch AxisLayerFK.compose. Its official
scripts/simple_compose.py uses positive z (pi/2) for right-finger bending, and
AnatomyConstraintLossEE likewise places bending in component 2. This source
contract does not substitute for a real-model check with licensed MANO assets.
"""
from __future__ import annotations

import math
import warnings

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .config import FINGER_DIPS, validate_solver_constraints


PARENTS = np.array([-1, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14])
_FINGERS = ((1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12))
_DIPS = {distal: middle for _, middle, distal in _FINGERS}
_LEVELS = (np.array([1, 4, 7, 10, 13]), np.array([2, 5, 8, 11, 14]),
           np.array([3, 6, 9, 12, 15]))
_LIMIT_MARGIN_RAD = 1e-6  # Diagnostic only: never changes the fitted pose.


def _euler_xyz_matrices(angles):
    """Batched intrinsic XYZ matrices, without allocating Rotation objects."""
    sx, sy, sz = np.sin(angles).T
    cx, cy, cz = np.cos(angles).T
    result = np.empty((len(angles), 3, 3))
    result[:, 0, 0], result[:, 0, 1], result[:, 0, 2] = cy * cz, -cy * sz, sy
    result[:, 1, 0] = cx * sz + sx * sy * cz
    result[:, 1, 1] = cx * cz - sx * sy * sz
    result[:, 1, 2] = -sx * cy
    result[:, 2, 0] = sx * sz - cx * sy * cz
    result[:, 2, 1] = sx * cz + cx * sy * sz
    result[:, 2, 2] = cx * cy
    return result


def _rotation_log(matrices):
    """Principal SO(3) log for internally generated proper rotation matrices.

    Inputs are products of validated unit-quaternion rotations and exact Euler
    rotations, not arbitrary measured 3x3 arrays requiring projection. The
    skew/atan2 expression keeps the original angular (not chordal) residual.
    Near pi, where the skew axis becomes ill-conditioned, use SciPy's robust
    conversion to retain its branch/sign convention. Near zero the scale tends
    to one, avoiding both cancellation in acos and division by zero.
    """
    skew = .5 * np.stack((matrices[:, 2, 1] - matrices[:, 1, 2],
                          matrices[:, 0, 2] - matrices[:, 2, 0],
                          matrices[:, 1, 0] - matrices[:, 0, 1]), axis=1)
    sine = np.linalg.norm(skew, axis=1)
    cosine = np.clip(.5 * (np.trace(matrices, axis1=1, axis2=2) - 1.), -1., 1.)
    angle = np.arctan2(sine, cosine)
    scale = np.divide(angle, sine, out=np.ones_like(sine), where=sine > 1e-8)
    result = skew * scale[:, None]
    near_pi = cosine < -.9999
    if np.any(near_pi):
        result[near_pi] = Rotation.from_matrix(matrices[near_pi]).as_rotvec()
    return result


def _quaternion(value):
    """Allow only small numerical norm error, matching calibration.quat.

    A grossly scaled input is a malformed observation, not a rotation that
    should silently become an accepted measurement after normalization.
    """
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must have four finite xyzw components")
    with np.errstate(over="ignore", invalid="ignore"):
        norm = np.linalg.norm(q)
    if not math.isfinite(float(norm)) or norm < 1e-12:
        raise ValueError("quaternion must have nonzero finite norm")
    if abs(float(norm) - 1.) > .01:
        raise ValueError("quaternion norm must be within 0.01 of one")
    return q / norm


class SparseHandSolver:
    """Fit limited hand DOFs to a sparse set of independently measured segments.

    ``observed_mask`` is an alias for ``segment_observed_mask``, NOT a claim
    that a joint angle was measured. ``joint_observed_mask[i]`` additionally
    requires its parent's observation and an available root. ``inferred_mask``
    describes available inferred joints, not unavailable output placeholders.
    Missing thumb CMC is fixed to an explicit prior: its decomposition from
    thumb MCP is unobservable, even if both measured thumb segments are present.
    """

    def __init__(self, config, rest_global_xyzw):
        mapping = config.get("node_to_slot")
        if not isinstance(mapping, dict) or not mapping:
            raise ValueError("node_to_slot must be a nonempty mapping")
        if any(not isinstance(n, str) or not n.isdigit() or str(int(n)) != n
               or not 1 <= int(n) <= 12 or type(s) is not int or not 0 <= s < 16
               for n, s in mapping.items()):
            raise ValueError("node_to_slot requires node strings 1..12 and slots 0..15")
        if len(set(mapping.values())) != len(mapping) or 0 not in mapping.values():
            raise ValueError("node slots must be unique and include a root mapping")
        self.node_to_slot = dict(mapping)
        ratio = config.get("dip_ratio", .65)
        if isinstance(ratio, bool) or not isinstance(ratio, (int, float)) or not math.isfinite(ratio) or not 0 <= ratio <= 1:
            raise ValueError("dip_ratio must be finite and between zero and one")
        self.dip_ratio = float(ratio)
        validate_solver_constraints(config)
        self._dip_ratios = {slot: float(config.get("dip_ratio_by_finger", {}).get(finger, ratio))
                            for finger, slot in FINGER_DIPS.items()}
        self._joint_limits = {(int(slot), "xyz".index(axis)): tuple(float(v) for v in bounds)
                              for slot, axes in config.get("joint_limits_rad", {}).items()
                              for axis, bounds in axes.items()}
        rest = np.asarray(rest_global_xyzw, dtype=float)
        if rest.shape != (16, 4):
            raise ValueError("rest_global_xyzw must have shape (16,4)")
        self.rest_global = [Rotation.from_quat(_quaternion(q)) for q in rest]
        self.rest_local = [self.rest_global[0]] + [
            self.rest_global[PARENTS[i]].inv() * self.rest_global[i] for i in range(1, 16)
        ]
        self._rest_local_matrices = np.stack([r.as_matrix() for r in self.rest_local])
        self.thumb_prior = np.asarray(config.get("thumb_cmc_prior_rad", [0., 0., 0.]), dtype=float)
        if self.thumb_prior.shape != (3,) or not np.all(np.isfinite(self.thumb_prior)):
            raise ValueError("thumb_cmc_prior_rad must have three finite angles")
        self.max_nfev = config.get("max_nfev", 100)
        if type(self.max_nfev) is not int or not 1 <= self.max_nfev <= 1000:
            raise ValueError("max_nfev must be an integer from 1 to 1000")

    def _forward(self, angles, root):
        return Rotation.from_matrix(self._forward_matrices(angles, root.as_matrix()))

    def _forward_matrices(self, angles, root_matrix):
        # The five fingers are independent chains of equal depth. Batch each
        # level rather than issuing 15 tiny Python-to-NumPy matrix operations.
        local = self._rest_local_matrices @ _euler_xyz_matrices(angles)
        rotations = np.empty((16, 3, 3))
        rotations[0] = root_matrix
        first, second, third = _LEVELS
        rotations[first] = root_matrix @ local[first]
        rotations[second] = rotations[first] @ local[second]
        rotations[third] = rotations[second] @ local[third]
        return rotations

    @staticmethod
    def _variables(observed):
        variables = []
        for proximal, middle, distal in _FINGERS:
            variables.extend(((proximal, 1, -.65, .65), (proximal, 2, -.25, 1.65),
                              (middle, 2, -.15, 2.2)))
            if observed[distal]:
                variables.append((distal, 2, -.15, 1.65))
        if observed[13]:
            variables.extend(((13, 0, -.7, .7), (13, 1, -1.1, 1.1), (13, 2, -1.2, 1.6)))
        variables.extend(((14, 1, -.65, .65), (14, 2, -.25, 1.65), (15, 2, -.15, 1.7)))
        return variables

    def _configured_variables(self, observed):
        variables = self._variables(observed)
        if not self._joint_limits:
            return variables
        return [(slot, axis, *self._joint_limits.get((slot, axis), (lower, upper)))
                for slot, axis, lower, upper in variables]

    def solve(self, observations, timestamp_s):
        if isinstance(timestamp_s, bool) or not isinstance(timestamp_s, (int, float)) or not math.isfinite(timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if not isinstance(observations, dict):
            raise ValueError("observations must be a node-to-quaternion mapping")
        rotations, rejected = {}, {}
        for node, slot in self.node_to_slot.items():
            if node not in observations:
                continue
            try:
                rotations[slot] = Rotation.from_quat(_quaternion(observations[node]))
            except (ValueError, TypeError, OverflowError) as exc:
                rejected[node] = str(exc)
        observed = np.array([i in rotations for i in range(16)])
        joint_observed = np.array([observed[i] and (i == 0 or observed[PARENTS[i]]) for i in range(16)])
        result = dict(timestamp_s=float(timestamp_s), observed_mask=observed.tolist(),
                      segment_observed_mask=observed.tolist(), joint_observed_mask=joint_observed.tolist(),
                      inferred_mask=(~joint_observed).tolist(), rejected_observations=rejected,
                      anatomical_accuracy_verified=False, yaw_drift_corrected=False,
                      algorithm="experimental_sparse_constrained_hand_v1",
                      preconstraint_relative_angles=[],
                      preconstraint_angle_kind="local_euler_initializer_not_unconstrained_optimum",
                      active_joint_limits=[], joint_limit_margin_rad=_LIMIT_MARGIN_RAD,
                      residual_deg_by_slot={str(i): None for i in range(16)})
        if not observed[0]:
            result.update(quality="unavailable", reason="missing_or_invalid_root",
                          anatomical_euler_rad=None, root_quat_xyzw=None, predicted_global_xyzw=None,
                          source_labels=["unavailable_missing_root"] * 16,
                          joint_observed_mask=[False] * 16, inferred_mask=[False] * 16,
                          pose_available=False,
                          solver_success=False)
            return result

        variables = self._configured_variables(observed)
        lower, upper = np.array([v[2] for v in variables]), np.array([v[3] for v in variables])

        def unpack(parameters):
            angles = np.zeros((16, 3))
            if not observed[13]:
                angles[13] = self.thumb_prior
            for value, (slot, axis, _, _) in zip(parameters, variables):
                angles[slot, axis] = value
            for distal, middle in _DIPS.items():
                if not observed[distal]:
                    angles[distal, 2] = self._dip_ratios[distal] * angles[middle, 2]
            return angles

        # A deterministic, current-frame initializer avoids both lag and an
        # optimizer having to discover a simple measured parent/child relation.
        initial = np.clip(np.zeros(len(variables)), lower, upper)
        for slot in range(1, 16):
            if slot not in rotations:
                continue
            parent = rotations.get(int(PARENTS[slot]))
            if parent is None:
                parent = self._forward(unpack(initial), rotations[0])[PARENTS[slot]]
            local = self.rest_local[slot].inv() * parent.inv() * rotations[slot]
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)  # Euler gimbal warning; fit uses rotations.
                estimate = local.as_euler("XYZ")
            # This is the current-frame local Euler decomposition before its
            # coordinates are clipped for initialization, NOT a second,
            # unconstrained least-squares fit. A predicted parent already
            # depends on the prior and preceding bounded initial values.
            result["preconstraint_relative_angles"].append({
                "slot": slot, "parent_slot": int(PARENTS[slot]),
                "anatomical_euler_rad": estimate.tolist(),
                "source": ("observed_parent_child_relative_euler" if joint_observed[slot]
                           else "inferred_parent_initializer"),
            })
            for index, (variable_slot, axis, _, _) in enumerate(variables):
                if variable_slot == slot:
                    initial[index] = np.clip(estimate[axis], lower[index], upper[index])

        measured_slots = [slot for slot in rotations if slot != 0]
        measured_matrices = np.array([rotations[slot].as_matrix() for slot in measured_slots]).reshape(-1, 3, 3)
        root_matrix = rotations[0].as_matrix()

        def residual(parameters):
            predicted = self._forward_matrices(unpack(parameters), root_matrix)
            if measured_slots:
                difference = predicted[measured_slots].transpose(0, 2, 1) @ measured_matrices
                errors = [_rotation_log(difference).ravel()]
            else:
                errors = []
            # Tiny, explicit minimum-norm prior chooses a deterministic answer
            # for unobserved DOFs, rather than inventing missing measurements.
            errors.append(1e-5 * parameters)
            return np.concatenate(errors)

        fit = least_squares(residual, initial, bounds=(lower, upper), max_nfev=self.max_nfev,
                            ftol=1e-9, xtol=1e-9, gtol=1e-9)
        angles = unpack(fit.x)
        predicted = self._forward(angles, rotations[0])
        sources = []
        for slot in range(16):
            if slot == 0:
                source = "measured_root"
            elif slot in _DIPS and not observed[slot]:
                source = "inferred_dip_ratio"
            elif slot == 13 and not observed[slot]:
                source = "inferred_thumb_cmc_prior"
            elif joint_observed[slot]:
                source = "observed_relative_constrained_fit"
            elif observed[slot]:
                source = "inferred_parent_decomposition"
            else:
                source = "inferred_missing_segment_prior"
            sources.append(source)
        for value, (slot, axis, low, high) in zip(fit.x, variables):
            # For unusually narrow custom intervals, use the nearer endpoint
            # rather than labelling both bounds active from the margin alone.
            side, limit = ("lower", low) if value - low <= high - value else ("upper", high)
            if abs(value - limit) <= _LIMIT_MARGIN_RAD:
                result["active_joint_limits"].append({
                    "slot": slot, "axis": "xyz"[axis], "side": side,
                    "value": float(value), "limit": float(limit), "source": sources[slot],
                })
        for slot in rotations:
            result["residual_deg_by_slot"][str(slot)] = float(np.degrees((predicted[slot].inv() * rotations[slot]).magnitude()))
        quality = "experimental" if all(observed[s] for s in self.node_to_slot.values()) else "experimental_partial"
        if not fit.success:
            quality = "experimental_not_converged"
        result.update(quality=quality, pose_available=True, anatomical_euler_rad=angles.tolist(),
                      root_quat_xyzw=(rotations[0] * self.rest_global[0].inv()).as_quat().tolist(),
                      predicted_global_xyzw=[r.as_quat().tolist() for r in predicted],
                      source_labels=sources, solver_success=bool(fit.success), solver_nfev=int(fit.nfev),
                      limitations=["Experimental adapter, not upstream FSGlove/DiffHCal.",
                                   "No yaw-drift correction or absolute position estimate.",
                                   "A missing thumb CMC uses a prior; CMC/MCP decomposition is not measured.",
                                   "Missing parent/child observations do not measure a relative joint."])
        return result
