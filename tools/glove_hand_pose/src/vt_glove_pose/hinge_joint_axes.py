"""Offline, experimental joint gyro/acceleration hinge-axis candidate.

Independent NumPy implementation of the two kinematic constraints described
by Olsson et al., Sensors 2020, 20(12), 3534, doi:10.3390/s20123534:
``|omega_p x j_p| - |omega_d x j_d| = 0`` and
``acc_p . j_p - acc_d . j_d = 0``. The latter is approximate unless the
sensor-offset rotational accelerations are negligible. Norm/energy filters
cannot prove that assumption. This is NOT a port of qmt or a reproduction of
the paper's sample-selection, uncertainty or global-optimality guarantees.
For comparison, qmt documents its combined estimator at
https://qmt.readthedocs.io/en/latest/python_functions.html#qmt.jointAxisEstHingeOlsson .

Only the caller supplies units, bias correction and synchronization. No scale
is guessed, no hardware/files are accessed and no live calibration is changed.
The default gyro acceptance bound is unchanged from the gyro-only candidate.
An accepted local fit does not verify a physical hinge or an anatomical angle.
"""

from __future__ import annotations

import math

import numpy as np

from .hinge_axes import (_basis, _gyros, _huber, _integer, _metrics,
                         _residual_jacobian, _rigid_diagnostic, _scalar)


def _acceleration(value, name, count):
    values = _gyros(value, name)
    if len(values) != count or np.any(np.abs(values) > 100.):
        raise ValueError(f"{name} must match gyro length and have components <= 100 g")
    return values


def _eligible_acceleration(gp, gd, ap, ad, maximum_rate):
    pn, dn = np.linalg.norm(ap, axis=1), np.linalg.norm(ad, axis=1)
    # Pointwise, axis-independent, fixed before optimization; no temporal
    # windows may cross the train/holdout boundary.
    return ((pn >= .85) & (pn <= 1.15) & (dn >= .85) & (dn <= 1.15)
            & (np.abs(pn-dn) <= .10)
            & (np.maximum(np.linalg.norm(gp, axis=1), np.linalg.norm(gd, axis=1)) <= maximum_rate))


def _joint_residual_jacobian(gp, gd, ap, ad, jp, jd, mask, gyro_sigma, accel_sigma):
    gyro, jac, bp, bd = _residual_jacobian(gp, gd, jp, jd)
    accel = ap[mask] @ jp - ad[mask] @ jd
    ajac = np.column_stack((ap[mask] @ bp, -(ad[mask] @ bd)))
    return (np.r_[gyro / gyro_sigma, accel / accel_sigma],
            np.vstack((jac / gyro_sigma, ajac / accel_sigma)), bp, bd)


def _optimize(gp, gd, ap, ad, mask, jp, jd, gyro_sigma, accel_sigma, iterations):
    damping, converged = 1e-3, False
    for iteration in range(iterations):
        residual, jac, bp, bd = _joint_residual_jacobian(
            gp, gd, ap, ad, jp, jd, mask, gyro_sigma, accel_sigma)
        objective = _huber(residual, 1.5)
        weights = np.minimum(1., 1.5 / np.maximum(np.abs(residual), 1e-15))
        hessian = jac.T @ (weights[:, None] * jac) / len(residual)
        gradient = jac.T @ (weights * residual) / len(residual)
        if np.linalg.norm(gradient) < 1e-8:
            converged = True
            break
        # At machine-precision minima the objective may no longer decrease.
        # Check the undamped local step, never an artificially tiny LM step.
        local_step = np.linalg.lstsq(hessian, -gradient, rcond=1e-10)[0]
        if np.linalg.norm(local_step) < 1e-9:
            converged = True
            break
        diagonal = np.maximum(np.diag(hessian), 1e-6)
        accepted = False
        for _ in range(12):
            step = np.linalg.solve(hessian + damping * np.diag(diagonal), -gradient)
            step *= min(1., .5 / max(np.linalg.norm(step[:2]), np.linalg.norm(step[2:]), 1e-15))
            pp, dd = jp + bp @ step[:2], jd + bd @ step[2:]
            pp, dd = pp / np.linalg.norm(pp), dd / np.linalg.norm(dd)
            next_residual = _joint_residual_jacobian(
                gp, gd, ap, ad, pp, dd, mask, gyro_sigma, accel_sigma)[0]
            next_objective = _huber(next_residual, 1.5)
            if next_objective < objective:
                jp, jd, accepted = pp, dd, True
                damping = max(1e-10, damping * .3)
                converged = bool(np.linalg.norm(step) < 1e-7 or objective-next_objective < 1e-12)
                break
            damping *= 10.
        if converged or not accepted:
            break
    # Only simultaneous sign reversal preserves the acceleration constraint.
    if jp[np.argmax(np.abs(jp))] < 0:
        jp, jd = -jp, -jd
    residual = _joint_residual_jacobian(gp, gd, ap, ad, jp, jd, mask, gyro_sigma, accel_sigma)[0]
    return jp, jd, _huber(residual, 1.5), iteration+1, converged


def _accel_metrics(ap, ad, jp, jd, mask, limit):
    residual = ap @ jp - ad @ jd
    selected = residual[mask]
    return dict(total_samples=len(residual), eligible_samples=int(mask.sum()),
                eligible_fraction=float(mask.mean()),
                all_samples_rms_g=float(np.sqrt(np.mean(residual**2))),
                eligible_rms_g=float(np.sqrt(np.mean(selected**2))) if len(selected) else None,
                eligible_p95_abs_g=float(np.percentile(np.abs(selected), 95)) if len(selected) else None,
                eligible_inlier_fraction=float(np.mean(np.abs(selected) <= limit)) if len(selected) else None)


def evaluate_joint_hinge_axes(axis_prox, axis_dist, prox_gyro_dps, dist_gyro_dps,
                             prox_acc_g, dist_acc_g, *, residual_limit_dps=2.,
                             accel_residual_limit_g=.08, accel_max_rate_dps=60., noise_dps=.5):
    """Evaluate fixed, signed axes without refitting (e.g. a separate session).

    Returned metrics do not constitute an acceptance decision. They include
    all gyro rows and report acceleration selection coverage explicitly.
    """
    gp, gd = _gyros(prox_gyro_dps, "prox_gyro_dps"), _gyros(dist_gyro_dps, "dist_gyro_dps")
    if len(gp) != len(gd):
        raise ValueError("gyro arrays must have equal lengths")
    ap = _acceleration(prox_acc_g, "prox_acc_g", len(gp))
    ad = _acceleration(dist_acc_g, "dist_acc_g", len(gp))
    axes = []
    for value in (axis_prox, axis_dist):
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or raw.shape != (3,):
            raise ValueError("axes must be real finite unit vectors")
        axis = raw.astype(float)
        if not np.all(np.isfinite(axis)) or not math.isclose(np.linalg.norm(axis), 1., abs_tol=1e-7):
            raise ValueError("axes must be real finite unit vectors")
        axes.append(axis)
    limit = _scalar(residual_limit_dps, "residual_limit_dps", 1e-4, 1e5)
    alim = _scalar(accel_residual_limit_g, "accel_residual_limit_g", 1e-5, .12)
    rate = _scalar(accel_max_rate_dps, "accel_max_rate_dps", 1., 1000.)
    noise = _scalar(noise_dps, "noise_dps", 1e-4, 1000.)
    jp, jd = axes
    gyro = _residual_jacobian(gp, gd, jp, jd)[0]
    mask = _eligible_acceleration(gp, gd, ap, ad, rate)
    return dict(gyro=_metrics(gyro, limit, 1.5*math.sqrt(2)*noise),
                acceleration=_accel_metrics(ap, ad, jp, jd, mask, alim),
                fit_performed=False)


def fit_joint_hinge_axes(
    prox_gyro_dps, dist_gyro_dps, prox_acc_g, dist_acc_g, *, sample_rate_hz,
    noise_dps=.5, accel_noise_g=.015, residual_limit_dps=2.,
    accel_residual_limit_g=.08, accel_max_rate_dps=60.,
    holdout_block_s=.5, min_samples_per_split=60,
    min_inlier_fraction=.9, min_accel_eligible_fraction=.2,
    min_tangent_sensitivity=2., max_condition_number=40.,
    n_starts=16, max_iterations=100, seed=0,
):
    """Fit on fixed even blocks, then evaluate untouched odd blocks once.

    Gyro inputs are paired and bias-corrected deg/s, acceleration is in g.
    No neutral samples are inferred or used here. Initializers, robust fitting,
    start selection and Jacobian use training rows only. Parameters must be
    chosen before looking at held-out outcomes, not tuned to make a run pass.
    Acceleration selection is pointwise, axis-independent and reports coverage.
    The whitened joint Jacobian describes local sensitivity only.

    ``candidate_supported`` means experimental constraint consistency, never
    live or anatomical calibration. Shared sign is canonical, relative sign
    comes from the acceleration constraint; anatomical flexion remains unknown.
    """
    gp, gd = _gyros(prox_gyro_dps, "prox_gyro_dps"), _gyros(dist_gyro_dps, "dist_gyro_dps")
    if len(gp) != len(gd):
        raise ValueError("gyro arrays must have equal lengths")
    ap = _acceleration(prox_acc_g, "prox_acc_g", len(gp))
    ad = _acceleration(dist_acc_g, "dist_acc_g", len(gp))
    rate = _scalar(sample_rate_hz, "sample_rate_hz", 1., 10000.)
    noise = _scalar(noise_dps, "noise_dps", 1e-4, 1000.)
    anoise = _scalar(accel_noise_g, "accel_noise_g", 1e-5, 1.)
    limit = _scalar(residual_limit_dps, "residual_limit_dps", 1e-4, 1e5)
    alim = _scalar(accel_residual_limit_g, "accel_residual_limit_g", 1e-5, .12)
    maximum_rate = _scalar(accel_max_rate_dps, "accel_max_rate_dps", 1., 1000.)
    block_s = _scalar(holdout_block_s, "holdout_block_s", .01, 60.)
    minimum = _integer(min_samples_per_split, "min_samples_per_split", 20, 10000)
    fraction = _scalar(min_inlier_fraction, "min_inlier_fraction", .5, 1.)
    coverage = _scalar(min_accel_eligible_fraction, "min_accel_eligible_fraction", .1, 1.)
    sensitivity = _scalar(min_tangent_sensitivity, "min_tangent_sensitivity", 1e-6, 1e5)
    condition_limit = _scalar(max_condition_number, "max_condition_number", 1., 1e6)
    starts = _integer(n_starts, "n_starts", 4, 64)
    iterations = _integer(max_iterations, "max_iterations", 5, 500)
    seed = _integer(seed, "seed", 0, 2**32-1)
    block_samples = max(1, int(round(rate*block_s)))
    train = (np.arange(len(gp)) // block_samples) % 2 == 0
    config = dict(noise_dps=noise, accel_noise_g=anoise, residual_limit_dps=limit,
                  accel_residual_limit_g=alim, accel_max_rate_dps=maximum_rate,
                  holdout_block_s=block_s, min_samples_per_split=minimum,
                  min_inlier_fraction=fraction, min_accel_eligible_fraction=coverage,
                  min_tangent_sensitivity=sensitivity, max_condition_number=condition_limit,
                  n_starts=starts, max_iterations=iterations, seed=seed)
    result = dict(schema_version=1, method="joint_gyro_accel_hinge_axes_experimental_v1",
                  candidate_status="insufficient_excitation", reason_codes=[],
                  axis_prox=None, axis_dist=None, axis_sign_convention="shared_sign_canonical_relative_sign_candidate_only",
                  relative_axis_sign_verified=False,
                  offline_only=True, usable_as_live_calibration=False,
                  hinge_model_supported_by_heldout_constraints=False,
                  anatomical_accuracy_verified=False, hardware_timing_verified=False,
                  scale_verified_by_this_function=False,
                  configuration_scope="experimental_not_product_acceptance", configuration=config,
                  split=dict(method="fixed_alternating_blocks_even_train", block_samples=block_samples,
                             total_samples=len(gp), train_samples=int(train.sum()),
                             holdout_samples=int((~train).sum()), nominal_sample_rate_hz=rate),
                  train=None, holdout=None, optimization=None, identifiability=None,
                  common_rigid_diagnostic=None,
                  limitations=["No independent physical-angle ground truth.",
                               "Local rank is not a global uniqueness guarantee.",
                               "Acceleration norm/energy filters cannot exclude all dynamic acceleration.",
                               "Firmware scale, synchronization, mounting and 1-DoF anatomy remain unverified."])
    if min(train.sum(), (~train).sum()) < minimum:
        result["reason_codes"] = ["too_few_samples_in_fixed_split"]
        return result
    tp, td, ta, tb = (v[train] for v in (gp, gd, ap, ad))
    if np.sqrt(np.mean(np.sum(tp*tp+td*td, axis=1))/2) <= max(3*noise, 1.):
        result["reason_codes"] = ["training_motion_below_noise"]
        return result
    mask = _eligible_acceleration(tp, td, ta, tb, maximum_rate)
    if mask.sum() < minimum or mask.mean() < coverage:
        result["reason_codes"] = ["too_few_eligible_training_acceleration_samples"]
        return result
    gs, ass = math.sqrt(2)*noise, math.sqrt(2)*anoise
    rng = np.random.default_rng(seed)
    # The linear acceleration constraint gives a useful initial direction,
    # but normalization may change its residual; it is not itself a solution.
    _, _, va = np.linalg.svd(np.column_stack((ta[mask], -tb[mask])), full_matrices=False)
    initial = []
    for v in va[-3:]:
        if min(np.linalg.norm(v[:3]), np.linalg.norm(v[3:])) > 1e-8:
            initial.append((v[:3]/np.linalg.norm(v[:3]), v[3:]/np.linalg.norm(v[3:])))
    while len(initial) < starts:
        p, d = rng.normal(size=(2, 3))
        initial.append((p/np.linalg.norm(p), d/np.linalg.norm(d)))
    fits = [_optimize(tp, td, ta, tb, mask, p, d, gs, ass, iterations) for p, d in initial[:starts]]
    selected = min(range(starts), key=lambda i: fits[i][2])
    jp, jd, objective, count, converged = fits[selected]
    result.update(axis_prox=jp.tolist(), axis_dist=jd.tolist())
    for name, subset in (("train", train), ("holdout", ~train)):
        result[name] = evaluate_joint_hinge_axes(jp, jd, gp[subset], gd[subset], ap[subset], ad[subset],
                                               residual_limit_dps=limit, accel_residual_limit_g=alim,
                                               accel_max_rate_dps=maximum_rate, noise_dps=noise)
    result["optimization"] = dict(selection_subset="train_only", selected_start=int(selected),
                                  selected_train_objective=float(objective), iterations=int(count),
                                  converged=bool(converged),
                                  start_train_objectives=[float(f[2]) for f in fits],
                                  holdout_used_for_fitting_or_selection=False)
    residual, jac, _, _ = _joint_residual_jacobian(tp, td, ta, tb, jp, jd, mask, gs, ass)
    bounds = np.r_[np.full(len(tp), limit/gs), np.full(mask.sum(), alim/ass)]
    inliers = np.abs(residual) <= bounds
    singular = np.linalg.svd(jac[inliers], compute_uv=False) / math.sqrt(max(1, int(inliers.sum())))
    singular = np.pad(singular, (0, 4-len(singular)))
    rank_threshold = max(1e-8, singular[0]*1e-6)
    rank = int(np.sum(singular > rank_threshold))
    condition = float(singular[0]/singular[-1]) if singular[-1] > rank_threshold else None
    result["identifiability"] = dict(subset="training_constraint_inliers_only", numerical_rank=rank,
                                    tangent_dimensions=4, inlier_residual_count=int(inliers.sum()),
                                    singular_values_rms_whitened_per_radian=singular.tolist(),
                                    condition_number=condition,
                                    minimum_sensitivity_whitened_per_radian=float(singular[-1]))
    rigid = _rigid_diagnostic(tp, td, noise, fraction)
    result["common_rigid_diagnostic"] = rigid
    deficient, inconsistent = [], []
    if rigid["compatible_with_common_rigid_motion"]:
        deficient.append("training_compatible_with_common_rigid_motion")
    if rank < 4 or singular[-1] < sensitivity or condition is None or condition > condition_limit:
        deficient.append("training_joint_tangent_jacobian_insufficient")
    if not converged:
        deficient.append("training_optimizer_not_converged")
    heldout_active = np.sqrt(np.sum(gp[~train]**2+gd[~train]**2, axis=1)/2) > max(3*noise, 1.)
    result["holdout"]["active_sample_count"] = int(heldout_active.sum())
    if heldout_active.sum() < minimum:
        deficient.append("too_few_active_holdout_samples")
    for subset in ("train", "holdout"):
        metrics = result[subset]
        if metrics["gyro"]["inlier_fraction"] < fraction:
            inconsistent.append(subset+"_gyro_constraint_residual_inconsistent")
        accel = metrics["acceleration"]
        if accel["eligible_samples"] < minimum or accel["eligible_fraction"] < coverage:
            deficient.append(subset+"_insufficient_eligible_acceleration")
        elif accel["eligible_inlier_fraction"] < fraction:
            inconsistent.append(subset+"_acceleration_constraint_residual_inconsistent")
    result["reason_codes"] = deficient+inconsistent
    if deficient:
        result["candidate_status"] = "insufficient_excitation"
    elif inconsistent:
        result["candidate_status"] = "inconsistent"
    else:
        result["candidate_status"] = "candidate_supported"
        result["hinge_model_supported_by_heldout_constraints"] = True
    return result
