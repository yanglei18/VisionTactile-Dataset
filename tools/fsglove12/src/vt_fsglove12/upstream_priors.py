"""Explicit missing-IMU priors around the unchanged official FSGlove core.

Only missing input slots are synthesized. A missing four-finger DIP follows
the current, measured PIP flexion when BOTH PIP segments are observed; all
other missing slots retain the neutral local prior. This is a configurable
empirical coupling, not an anatomical measurement or an upstream algorithm.
No sample history, filtering, automatic reference reset, or mesh edit occurs.
"""
from __future__ import annotations

import math
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import quat
from .config import FINGER_DIPS
from .upstream_inference import calibration_rotations


DEFAULT_DIP_MAX_RAD = 1.65
PRIOR_POLICY = "current_observed_pip_coupled_dip_with_neutral_fallback_v1"


def _number(value, name, *, lower=0., upper=1., strict=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not (lower < value < upper if strict else lower <= value <= upper)):
        relation = "(" if strict else "["
        closing = ")" if strict else "]"
        raise ValueError(f"{name} must be finite in {relation}{lower}, {upper}{closing}")
    return float(value)


def _parameters(config):
    if not isinstance(config, dict):
        raise ValueError("prior config must be an object")
    ratio = _number(config.get("dip_ratio", .65), "dip_ratio")
    overrides = config.get("dip_ratio_by_finger", {})
    if not isinstance(overrides, dict) or set(overrides) - set(FINGER_DIPS):
        raise ValueError("dip_ratio_by_finger requires only index/middle/little/ring keys")
    ratios = {finger: _number(overrides.get(finger, ratio), f"dip_ratio_by_finger.{finger}")
              for finger in FINGER_DIPS}
    maximum = _number(config.get("upstream_dip_max_rad", DEFAULT_DIP_MAX_RAD),
                      "upstream_dip_max_rad", lower=0., upper=math.pi, strict=True)
    return ratios, maximum


def complete_with_priors(calibrator, raw_quaternions, observed_mask, config, *, rotation_cache=None):
    """Return ``(completed_xyzw, diagnostics)`` for one current input frame.

    Upstream uses A_i = C_i^-1 G_i (B_i^-1 Q_i) L_i^-1, where
    C_i = FK_parent D_i. For a missing slot, choosing
    Q_i = B_i G_i^-1 C_i A_prior L_i gives exactly the declared local
    anatomical prior when the unmodified official inference is subsequently
    called. Measured Q_i values are never replaced, including measured DIPs.

    Missing DIP targets are XYZ Euler [0, 0, clip(ratio * max(0, PIP_z))].
    Both the proximal and middle phalanx must currently be observed. Masking a
    stale, invalid, or wrong-epoch observation is the caller's responsibility.
    No past observation is retained when these prerequisites disappear.
    """
    result = np.asarray(raw_quaternions, dtype=float)
    if result.shape != (16, 4) or not np.isfinite(result).all():
        raise ValueError("raw_quaternions must be a finite (16, 4) array")
    result = result.copy()
    observed = np.asarray(observed_mask)
    if observed.shape != (16,) or observed.dtype != np.bool_:
        raise ValueError("observed_mask must contain 16 booleans")
    if not observed[0]:
        raise ValueError("a current measured root is required")
    observed_slots = np.flatnonzero(observed)
    for slot in observed_slots:
        quat(result[slot], f"observed slot {slot}")
    ratios, maximum = _parameters(config)
    diagnostics = {
        "policy": PRIOR_POLICY,
        "inference_is_measurement": False,
        "depends_on_previous_frame": False,
        "virtual_input_slots": np.flatnonzero(~observed).tolist(),
        "coupled_dip_slots": [], "zero_prior_slots": [],
        "target_anatomical_euler_rad_by_slot": {},
        "dip_by_finger": {},
        "dip_max_rad": maximum,
    }
    for finger, slot in FINGER_DIPS.items():
        diagnostics["dip_by_finger"][finger] = {
            "dip_slot": slot, "pip_slot": slot - 1, "proximal_slot": slot - 2,
            "ratio": ratios[finger], "limits_rad": [0., maximum],
            "applied": False, "clip_applied": False,
            "pip_flexion_rad": None, "unclipped_target_rad": None,
            "target_flexion_rad": None,
            "missing_dependency_slots": [],
            "source": "observed_input_unchanged" if observed[slot] else "inferred_not_measured",
            "reason": "dip_observed_input_unchanged" if observed[slot] else "pending_missing_input",
        }
    # No official member access or rotation re-encoding on the full16 path.
    if observed.all():
        return result, diagnostics

    transforms = calibration_rotations(calibrator, rotation_cache)
    base, local_error = transforms.base, transforms.local_error
    rest_delta = transforms.rest_delta
    dip_fingers = {slot: finger for finger, slot in FINGER_DIPS.items()}
    # Measured segment globals are independent of their parents. SciPy applies
    # the same scalar quaternion products in a batch; retain the original
    # left-to-right multiplication order and never re-encode measured inputs.
    measured_fk = (transforms.global_error_batch[observed_slots]
                   * transforms.base_inv_batch[observed_slots]
                   * Rotation.from_quat(result[observed_slots])
                   * transforms.local_error_inv_batch[observed_slots])
    fk = [None] * 16
    for slot, rotation in zip(observed_slots, measured_fk):
        fk[slot] = rotation
    for slot in calibrator._fk_seq:
        slot = int(slot)
        if observed[slot]:
            continue
        parent = int(calibrator._joint_fa[slot])
        current = fk[parent] * rest_delta[slot]

        target = np.zeros(3)
        finger = dip_fingers.get(slot)
        if finger is not None:
            entry = diagnostics["dip_by_finger"][finger]
            dependencies = [slot - 2, slot - 1]
            missing = [item for item in dependencies if not observed[item]]
            if missing:
                entry.update(reason="pip_parent_or_child_not_observed_neutral_fallback",
                             missing_dependency_slots=missing, target_flexion_rad=0.)
            else:
                # Only this observed PIP local rotation contributes to a
                # missing-DIP target. Other measured locals were unused.
                pip_current = fk[int(calibrator._joint_fa[parent])] * rest_delta[parent]
                pip_local = pip_current.inv() * fk[parent]
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", UserWarning)
                    pip_z = float(pip_local.as_euler("XYZ")[2])
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
        fk[slot] = current * Rotation.from_euler("XYZ", target)
        result[slot] = (base[slot] * transforms.global_error_inv[slot] * fk[slot] * local_error[slot]).as_quat()

    diagnostics["coupled_dip_slots"].sort()
    diagnostics["zero_prior_slots"].sort()
    return result, diagnostics
