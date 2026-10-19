"""Experimental offline gyro-only hinge-axis identification (NumPy only).

The physical constraint is ``|omega_p x j_p| == |omega_d x j_d|``.
Both links may move; no fixed-proximal or preceding rigid-common-motion capture
is required. The equations are described, for example, in Olsson et al.,
"Robust Plug-and-Play Joint Axis Estimation Using Inertial Sensors", Sensors
2020, 20(12), 3534, https://doi.org/10.3390/s20123534 . This is an independent
implementation of the gyro constraint, NOT a port of the authors' software or
their combined gyro/accelerometer method.

IMPORTANT: paired gyro inputs must already have verified units, timing, range
and bias handling. This routine cannot verify those conditions. A small residual
does not establish anatomical validity, physical angle accuracy or sensor health.
The two output axes are *unoriented lines*, each with an independent +/- sign
ambiguity. They must not be installed as live calibration or used directly for
signed joint integration. No neutral, angle, heading or common mounting rotation
is estimated. All thresholds below are experimental configuration, not product
acceptance criteria.
"""

from __future__ import annotations

import math
from numbers import Integral, Real

import numpy as np


def _scalar(value, name, low, high):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
    return float(value)


def _integer(value, name, low, high):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in [{low}, {high}]")
    return int(value)


def _gyros(value, name):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf":
            raise ValueError("real numeric data required")
        data = np.array(raw, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain real numeric (N, 3) vectors") from exc
    if data.ndim != 2 or data.shape[1] != 3 or not 4 <= len(data) <= 30000:
        raise ValueError(f"{name} must have shape (N, 3), 4 <= N <= 30000")
    if not np.all(np.isfinite(data)) or np.any(np.abs(data) > 1e6):
        raise ValueError(f"{name} must be finite with absolute components <= 1e6 dps")
    return data


def _basis(axis):
    """Orthonormal tangent basis: rotation-free 2D coordinates on S^2."""
    pole = np.eye(3)[np.argmin(np.abs(axis))]
    first = np.cross(axis, pole)
    first /= np.linalg.norm(first)
    return np.column_stack((first, np.cross(axis, first)))


def _residual_jacobian(gp, gd, jp, jd):
    bp, bd = _basis(jp), _basis(jd)
    # Smooth only the norm cusp at exactly parallel omega and j. This epsilon
    # is numerical, not a noise threshold or a motion-dependent selection.
    cp = np.sqrt(np.sum(np.cross(gp, jp) ** 2, axis=1) + 1e-16)
    cd = np.sqrt(np.sum(np.cross(gd, jd) ** 2, axis=1) + 1e-16)
    p_dot, d_dot = gp @ jp, gd @ jd
    jac = np.column_stack((-(p_dot / cp)[:, None] * (gp @ bp),
                          (d_dot / cd)[:, None] * (gd @ bd)))
    return cp - cd, jac, bp, bd


def _huber(residual, delta):
    a = np.abs(residual)
    return float(np.mean(np.where(a <= delta, .5 * a * a, delta * (a - .5 * delta))))


def _canonical(axis):
    axis = axis / np.linalg.norm(axis)
    return -axis if axis[np.argmax(np.abs(axis))] < 0 else axis


def _optimize(gp, gd, jp, jd, delta, iterations):
    """Damped robust Gauss-Newton with retraction on S^2 x S^2."""
    damping, converged = 1e-3, False
    for iteration in range(iterations):
        residual, jac, bp, bd = _residual_jacobian(gp, gd, jp, jd)
        objective = _huber(residual, delta)
        weights = np.minimum(1., delta / np.maximum(np.abs(residual), 1e-15))
        hessian = jac.T @ (weights[:, None] * jac) / len(gp)
        gradient = jac.T @ (weights * residual) / len(gp)
        if np.linalg.norm(gradient) < 1e-8:
            converged = True
            break
        diagonal = np.maximum(np.diag(hessian), 1e-6)
        accepted = False
        for _ in range(12):
            step = np.linalg.solve(hessian + damping * np.diag(diagonal), -gradient)
            step *= min(1., .5 / max(np.linalg.norm(step[:2]), np.linalg.norm(step[2:]), 1e-15))
            pp, dd = jp + bp @ step[:2], jd + bd @ step[2:]
            pp, dd = pp / np.linalg.norm(pp), dd / np.linalg.norm(dd)
            new_residual, _, _, _ = _residual_jacobian(gp, gd, pp, dd)
            new_objective = _huber(new_residual, delta)
            if new_objective < objective:
                jp, jd = pp, dd
                accepted = True
                damping = max(1e-10, damping * .3)
                converged = bool(np.linalg.norm(step) < 1e-7 or objective - new_objective < 1e-12)
                break
            damping *= 10.
        if converged or not accepted:
            break
    residual, _, _, _ = _residual_jacobian(gp, gd, jp, jd)
    return _canonical(jp), _canonical(jd), _huber(residual, delta), iteration + 1, converged


def _metrics(residual, limit, delta):
    absolute = np.abs(residual)
    inliers = absolute <= limit
    return {
        "sample_count": int(len(residual)),
        "rms_dps": float(np.sqrt(np.mean(residual ** 2))),
        "median_abs_dps": float(np.median(absolute)),
        "p95_abs_dps": float(np.percentile(absolute, 95)),
        "max_abs_dps": float(np.max(absolute)),
        "inlier_count": int(inliers.sum()),
        "inlier_fraction": float(np.mean(inliers)),
        "inlier_rms_dps": float(np.sqrt(np.mean(residual[inliers] ** 2))) if inliers.any() else None,
        "huber_objective_dps_squared": _huber(residual, delta),
    }


def _rigid_diagnostic(gp, gd, noise, minimum_fraction):
    """A training-only competing common-rigid model, not a calibration stage."""
    weights = np.ones(len(gp))
    for _ in range(6):
        u, _, vt = np.linalg.svd(gp.T @ (weights[:, None] * gd))
        rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
        errors = np.linalg.norm(gp @ rotation - gd, axis=1)
        weights = np.minimum(1., 3. * noise / np.maximum(errors, 1e-15))
    inliers = errors <= 6. * noise
    inlier_rms = float(np.sqrt(np.mean(errors[inliers] ** 2))) if inliers.any() else None
    return {
        "fit_subset": "train_only",
        "rms_dps": float(np.sqrt(np.mean(errors ** 2))),
        "inlier_fraction": float(np.mean(inliers)),
        "inlier_rms_dps": inlier_rms,
        "compatible_with_common_rigid_motion": bool(
            np.mean(inliers) >= minimum_fraction and inlier_rms is not None and inlier_rms <= 3. * noise),
    }


def fit_hinge_axes(
    prox_gyro_dps, dist_gyro_dps, *, sample_rate_hz,
    noise_dps=.5, residual_limit_dps=None, holdout_block_s=.5,
    min_samples_per_split=60, min_inlier_fraction=.9,
    min_tangent_sensitivity_dps=2., max_condition_number=40.,
    n_starts=16, max_iterations=100, seed=0,
):
    """Return an offline *candidate*, never a live or anatomical calibration.

    Input arrays are paired bias-handled real (N, 3) gyro values in deg/s.
    Equal lengths and finite bounded values are mandatory. Alternate fixed
    blocks (even=train, odd=holdout) are allocated *before* any filtering.
    ``sample_rate_hz`` assigns blocks only; no sample timing is estimated.
    Adjacent blocks are not claimed to be statistically independent sessions.

    PCA initializers, robust optimization, start selection and identifiability
    use training rows only. Holdout rows evaluate the single selected solution;
    they never change it or trigger a refit. Low-residual common-rigid motion
    cannot identify a hinge axis and is rejected. The 4D tangent Jacobian is
    evaluated on training inliers, with singular values divided by sqrt(N).
    It describes local sensitivity, not global uniqueness or angle accuracy.

    Status is ``candidate_supported`` (experimental consistency checks only),
    ``insufficient_excitation`` or ``inconsistent``. Format/parameter errors
    raise ValueError. The result contains no NumPy values or NaN/Infinity and
    can be serialized with json.dumps(..., allow_nan=False). Inputs are never
    mutated. No hardware, files, previous calibration or global RNG is used.
    """
    gp, gd = _gyros(prox_gyro_dps, "prox_gyro_dps"), _gyros(dist_gyro_dps, "dist_gyro_dps")
    if len(gp) != len(gd):
        raise ValueError("prox_gyro_dps and dist_gyro_dps must have equal lengths")
    rate = _scalar(sample_rate_hz, "sample_rate_hz", 1., 10000.)
    noise = _scalar(noise_dps, "noise_dps", 1e-4, 1000.)
    limit = 4. * noise if residual_limit_dps is None else _scalar(residual_limit_dps, "residual_limit_dps", 1e-4, 1e5)
    block_s = _scalar(holdout_block_s, "holdout_block_s", .01, 60.)
    minimum = _integer(min_samples_per_split, "min_samples_per_split", 20, 10000)
    fraction = _scalar(min_inlier_fraction, "min_inlier_fraction", .5, 1.)
    sensitivity = _scalar(min_tangent_sensitivity_dps, "min_tangent_sensitivity_dps", 1e-6, 1e5)
    condition_limit = _scalar(max_condition_number, "max_condition_number", 1., 1e6)
    starts = _integer(n_starts, "n_starts", 4, 64)
    iterations = _integer(max_iterations, "max_iterations", 5, 500)
    seed = _integer(seed, "seed", 0, 2**32 - 1)
    block_samples = max(1, int(round(rate * block_s)))
    train = (np.arange(len(gp)) // block_samples) % 2 == 0
    config = dict(noise_dps=noise, residual_limit_dps=limit, holdout_block_s=block_s,
                  min_samples_per_split=minimum, min_inlier_fraction=fraction,
                  min_tangent_sensitivity_dps=sensitivity, max_condition_number=condition_limit,
                  n_starts=starts, max_iterations=iterations, seed=seed)
    result = {
        "schema_version": 1,
        "method": "gyro_norm_hinge_axes_experimental_v1",
        "candidate_status": "insufficient_excitation",
        "reason_codes": [],
        "axis_prox": None, "axis_dist": None,
        "axis_sign_convention": "independent_axis_line_signs_unresolved",
        "usable_as_live_calibration": False,
        "anatomical_accuracy_verified": False,
        "hardware_timing_verified": False,
        "zero_pose_estimated": False,
        "configuration_scope": "experimental_not_product_acceptance",
        "configuration": config,
        "split": {"method": "fixed_alternating_blocks_even_train", "block_samples": block_samples,
                  "total_samples": int(len(gp)), "train_samples": int(train.sum()),
                  "holdout_samples": int((~train).sum()), "nominal_sample_rate_hz": rate},
        "train": None, "holdout": None, "identifiability": None,
        "common_rigid_diagnostic": None, "optimization": None,
        "limitations": ["No independent angle ground truth is evaluated.",
                        "Local rank does not prove global axis uniqueness.",
                        "Independent +/- axis signs and joint zero remain unresolved.",
                        "Timing, bias, scale, saturation and hinge anatomy are not verified here."],
    }
    if train.sum() < minimum or (~train).sum() < minimum:
        result["reason_codes"] = ["too_few_samples_in_fixed_split"]
        return result
    tp, td = gp[train], gd[train]
    motion_rms = np.sqrt(np.mean(np.sum(tp * tp + td * td, axis=1)) / 2.)
    if motion_rms <= max(3. * noise, 1.):
        result["reason_codes"] = ["training_motion_below_noise"]
        return result
    delta = 1.5 * math.sqrt(2.) * noise
    rigid = _rigid_diagnostic(tp, td, noise, fraction)
    result["common_rigid_diagnostic"] = rigid
    # Fixed local RNG; neither held-out measurements nor global RNG affect it.
    rng = np.random.default_rng(seed)
    _, _, vp = np.linalg.svd(tp, full_matrices=False)
    _, _, vd = np.linalg.svd(td, full_matrices=False)
    initial = [(vp[i], vd[j]) for i in range(3) for j in range(3)]
    # Guarantee random starts even when the caller requests fewer than nine.
    initial = initial[:min(9, starts // 2)]
    while len(initial) < starts:
        p, d = rng.normal(size=(2, 3))
        initial.append((p / np.linalg.norm(p), d / np.linalg.norm(d)))
    fits = [_optimize(tp, td, p.copy(), d.copy(), delta, iterations) for p, d in initial]
    selected = min(range(starts), key=lambda index: fits[index][2])
    jp, jd, objective, count, converged = fits[selected]
    residual, jac, _, _ = _residual_jacobian(tp, td, jp, jd)
    heldout, _, _, _ = _residual_jacobian(gp[~train], gd[~train], jp, jd)
    result["axis_prox"], result["axis_dist"] = jp.tolist(), jd.tolist()
    result["train"] = _metrics(residual, limit, delta)
    result["holdout"] = _metrics(heldout, limit, delta)
    result["optimization"] = {
        "selection_subset": "train_only", "selected_start": int(selected),
        "selected_train_objective_dps_squared": float(objective),
        "iterations": int(count), "converged": bool(converged),
        "start_train_objectives_dps_squared": [float(fit[2]) for fit in fits],
        "holdout_used_for_fitting_or_selection": False,
    }
    inliers = np.abs(residual) <= limit
    singular = np.linalg.svd(jac[inliers], compute_uv=False) / math.sqrt(max(1, int(inliers.sum())))
    singular = np.pad(singular, (0, 4 - len(singular)))
    rank_threshold = max(1e-8, singular[0] * 1e-6)
    rank = int(np.sum(singular > rank_threshold))
    condition = float(singular[0] / singular[-1]) if singular[-1] > rank_threshold else None
    result["identifiability"] = {
        "subset": "training_residual_inliers_only", "tangent_dimensions": 4,
        "inlier_samples": int(inliers.sum()), "numerical_rank": rank,
        "singular_values_rms_dps_per_radian": singular.tolist(),
        "condition_number": condition, "minimum_sensitivity_dps_per_radian": float(singular[-1]),
        "rank_threshold_dps_per_radian": float(rank_threshold),
    }
    deficient = []
    if rigid["compatible_with_common_rigid_motion"]:
        deficient.append("training_compatible_with_common_rigid_motion")
    if rank < 4:
        deficient.append("training_tangent_jacobian_rank_deficient")
    if singular[-1] < sensitivity:
        deficient.append("training_tangent_sensitivity_too_small")
    if condition is None or condition > condition_limit:
        deficient.append("training_tangent_jacobian_ill_conditioned")
    if int(inliers.sum()) < minimum:
        deficient.append("too_few_training_residual_inliers")
    # A quiet holdout gives a vacuous zero residual and is not validation.
    # This decision is deliberately after fitting: it cannot change the axes.
    heldout_motion = np.sqrt(np.sum(gp[~train] ** 2 + gd[~train] ** 2, axis=1) / 2.)
    heldout_active = int(np.sum(heldout_motion > max(3. * noise, 1.)))
    result["holdout"]["active_sample_count"] = heldout_active
    if heldout_active < minimum:
        deficient.append("too_few_active_holdout_samples")
    inconsistent = [f"{subset}_constraint_residual_inconsistent"
                    for subset in ("train", "holdout")
                    if result[subset]["inlier_fraction"] < fraction]
    if deficient:
        result["reason_codes"] = deficient + inconsistent
    elif inconsistent:
        result["candidate_status"] = "inconsistent"
        result["reason_codes"] = inconsistent
    elif not converged:
        result["candidate_status"] = "inconsistent"
        result["reason_codes"] = ["optimizer_not_converged"]
    else:
        result["candidate_status"] = "candidate_supported"
        result["reason_codes"] = ["experimental_consistency_checks_only"]
    return result
