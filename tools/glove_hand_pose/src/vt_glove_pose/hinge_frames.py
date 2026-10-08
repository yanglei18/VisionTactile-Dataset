"""Offline functional frames from unoriented hinge axes and explicit neutral.

An independent construction from vector projections; no device or live state is
accessed. A physical hinge has equal gravity components along its consistently
oriented axis in both sensor frames. This allows relative axis-sign resolution
only when the supplied acceleration observations contain a usable component
along that axis. Global sign remains a mathematical convention, not anatomical
flexion direction.

Acceleration magnitude near 1 g does NOT prove that a vector is gravity. The
caller must establish synchronized gravity-dominated paired calibration data,
a genuine static explicit neutral capture, sensor-to-segment rigidity, and the
applicability of the fitted hinge axes. The checks here can reject some failures
but cannot certify those premises or physical angle accuracy. In particular no
neutral pose is selected automatically, and no world heading is estimated.
"""

from __future__ import annotations

import math
from numbers import Integral, Real

import numpy as np


def _vectors(value, name, *, single=False):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "ifu":
            raise ValueError("real numeric data required")
        out = np.array(raw, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} requires finite numeric vectors") from exc
    shape_ok = out.shape == (3,) if single else out.ndim == 2 and out.shape[1] == 3 and 1 <= len(out) <= 30000
    if not shape_ok or not np.all(np.isfinite(out)) or np.any(np.abs(out) > 1e6):
        shape = "(3,)" if single else "(N,3), 1<=N<=30000"
        raise ValueError(f"{name} requires finite {shape} with components <=1e6")
    return out


def _number(value, name, low, high):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
    return float(value)


def _axis(value, name):
    axis = _vectors(value, name, single=True)
    scale = float(np.max(np.abs(axis)))
    if scale == 0.:
        raise ValueError(f"{name} must be a nonzero axis")
    axis /= scale
    axis /= np.linalg.norm(axis)
    # Unoriented input lines: canonicalize first so input +/- has no effect.
    return -axis if axis[np.argmax(np.abs(axis))] < 0. else axis


def _rms(values):
    return float(np.sqrt(np.mean(values ** 2)))


def _make_frame(axis, neutral_unit):
    x = -axis
    z = neutral_unit - np.dot(neutral_unit, x) * x
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    # Recompute z to remove roundoff while preserving x cross y = z.
    z = np.cross(x, y)
    return np.stack((x, y, z))


def _block_sign_evidence(p, d, indices, informative, *, block_samples,
                         min_samples, min_blocks, margin, mismatch):
    """Resolve relative sign without diluting useful evidence with zero rows.

    Blocks are sequence-index groups, NOT statistically independent samples or
    physical timestamps. The amplitude mask is invariant to either axis sign.
    Every sufficiently populated block must be internally consistent; a strong
    contrary block is not outvoted by many compatible blocks.
    """
    block_ids = indices // block_samples
    blocks = []
    for block_id in np.unique(block_ids[informative]):
        keep = informative & (block_ids == block_id)
        count = int(keep.sum())
        if count < min_samples:
            continue
        pp, dd = p[keep], d[keep]
        product = float(np.median(pp * dd))
        sign = 1 if product > 0. else -1 if product < 0. else 0
        same, opposite = _rms(pp - dd), _rms(pp + dd)
        residual = pp - sign * dd if sign else np.maximum(np.abs(pp-dd), np.abs(pp+dd))
        p95 = float(np.percentile(np.abs(residual), 95))
        chosen, other = (same, opposite) if sign == 1 else (opposite, same)
        strength = other - chosen if sign else 0.
        blocks.append(dict(block_id=int(block_id), informative_samples=count,
                           median_component_product=product, sign=sign,
                           same_sign_rms=same, opposite_sign_rms=opposite,
                           directional_rms_margin=strength, selected_residual_p95=p95,
                           supported=bool(sign and strength >= margin and p95 <= mismatch)))
    supported = [b for b in blocks if b["supported"]]
    signs = {b["sign"] for b in supported}
    evidence = dict(block_samples=block_samples, min_blocks=min_blocks,
                    min_informative_samples_per_block=min_samples,
                    populated_blocks=len(blocks), supported_blocks=len(supported),
                    positive_blocks=sum(b["sign"] == 1 for b in supported),
                    negative_blocks=sum(b["sign"] == -1 for b in supported),
                    sequence_block_independence_verified=False, blocks=blocks)
    if len(signs) > 1:
        return None, "inconsistent", "conflicting_informative_axis_sign_blocks", evidence
    if any(b["selected_residual_p95"] > mismatch for b in blocks):
        return None, "inconsistent", "informative_axis_sign_block_inconsistent", evidence
    if len(supported) < min_blocks:
        return None, "sign_unobservable", "insufficient_supported_axis_sign_blocks", evidence
    return signs.pop(), None, None, evidence


def build_hinge_frames(
    axis_prox, axis_dist, prox_acc_g, dist_acc_g, *,
    neutral_prox_acc_g, neutral_dist_acc_g,
    min_samples=30, min_parallel_rms=.10, max_axis_mismatch=.08,
    min_sign_margin=.10, min_neutral_projection=.40,
    min_usable_fraction=.80, sign_strategy="whole_capture",
    sign_block_samples=60, min_sign_blocks=3, sample_indices=None,
):
    """Return a JSON-safe candidate sensor->functional frame pair for replay.

    Acceleration arrays are paired (N,3), in g, from the axis-calibration
    recording. Neutral vectors are separate required (3,) keyword arguments;
    they must describe a caller-verified simultaneous static reference, not
    necessarily an anatomically straight joint. Input line signs and lengths
    are ignored; the proximal line is canonicalized by its largest component.

    The distal line's sign is selected by comparing same/opposite projected
    gravity components. Lack of a component along the axis leaves the sign
    unresolved; the function will not guess from neutral or an axis index.
    Frames have rows x=-j, z=normalized neutral projection perpendicular to j,
    y=z cross x. Multiply column sensor vectors with the returned matrix; for
    row batches use ``sensor_vectors @ frame.T``.

    For ThumbFlexionEstimator, pg.x-dg.x is the angular rate about the selected
    positive hinge axis j, with distal motion relative to proximal. Anatomical
    flexion/extension direction, full mounting geometry and static validity are
    NOT inferred. ``frames_ready`` means these conditional consistency checks
    passed, not permission to install this result in live calibration.

    The default scalar bounds are experimental configuration, not product
    criteria. Norms in [0.85,1.15] g and pair norm difference <=0.10 g are only
    preliminary filters; they do not detect all linear acceleration. At least
    min_samples and min_usable_fraction must survive those filters. No state,
    input arrays, stored calibration or hardware are mutated.

    ``sign_strategy='informative_blocks'`` is an opt-in alternative for
    captures mixing useful along-axis gravity with near-perpendicular poses.
    It retains the same amplitude, residual and separation bounds, but does
    not dilute sign evidence with uninformative rows. Use original increasing
    ``sample_indices`` after selecting recording intervals to keep block
    boundaries across gaps. Without these indices, array row order is used.
    Blocks check consistency in sequence order, not independent physical time.
    Neither strategy certifies the fitted axes; that is a separate decision.
    """
    jp, jd = _axis(axis_prox, "axis_prox"), _axis(axis_dist, "axis_dist")
    ap = _vectors(prox_acc_g, "prox_acc_g")
    ad = _vectors(dist_acc_g, "dist_acc_g")
    if len(ap) != len(ad):
        raise ValueError("prox_acc_g and dist_acc_g must have equal lengths")
    np_ = _vectors(neutral_prox_acc_g, "neutral_prox_acc_g", single=True)
    nd = _vectors(neutral_dist_acc_g, "neutral_dist_acc_g", single=True)
    if isinstance(min_samples, (bool, np.bool_)) or not isinstance(min_samples, Integral) or not 3 <= min_samples <= 10000:
        raise ValueError("min_samples must be an integer in [3, 10000]")
    min_samples = int(min_samples)
    parallel = _number(min_parallel_rms, "min_parallel_rms", 1e-4, 1.)
    mismatch = _number(max_axis_mismatch, "max_axis_mismatch", 1e-5, .12)
    margin = _number(min_sign_margin, "min_sign_margin", 1e-4, 2.)
    projection = _number(min_neutral_projection, "min_neutral_projection", .3, .99)
    usable_fraction = _number(min_usable_fraction, "min_usable_fraction", .5, 1.)
    if not isinstance(sign_strategy, str) or sign_strategy not in ("whole_capture", "informative_blocks"):
        raise ValueError("sign_strategy must be whole_capture or informative_blocks")
    for name, value, low in (("sign_block_samples", sign_block_samples, 3),
                              ("min_sign_blocks", min_sign_blocks, 2)):
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or not low <= value <= 10000:
            raise ValueError(f"{name} must be an integer in [{low}, 10000]")
    sign_block_samples, min_sign_blocks = int(sign_block_samples), int(min_sign_blocks)
    if sign_strategy == "informative_blocks" and sign_block_samples < min_samples:
        raise ValueError("sign_block_samples must be >= min_samples")
    if sample_indices is None:
        indices = np.arange(len(ap), dtype=np.int64)
    else:
        raw = np.asarray(sample_indices)
        has_bool = any(isinstance(x, (bool, np.bool_)) for x in np.asarray(sample_indices, dtype=object).flat)
        if (has_bool or raw.dtype.kind not in "iu" or raw.shape != (len(ap),)
                or np.any(raw < 0) or np.any(raw > 10**12)
                or np.any(raw[1:] <= raw[:-1])):
            raise ValueError("sample_indices must be increasing nonnegative integer row indices <=1e12")
        indices = raw.astype(np.int64)
    result = {
        "schema_version": 1,
        "method": "hinge_gravity_functional_frames_experimental_v1",
        "status": "insufficient_data", "reason_codes": [],
        "prox_sensor_to_functional": None, "dist_sensor_to_functional": None,
        "signed_axis_prox": None, "signed_axis_dist": None,
        "functional_neutral_prox_acc_g": None, "functional_neutral_dist_acc_g": None,
        "sign_metrics": None, "neutral_metrics": None,
        "usable_as_live_calibration": False,
        "anatomical_flexion_sign_verified": False,
        "relative_axis_sign_resolved_conditionally": False,
        "static_neutral_verified_by_this_function": False,
        "linear_acceleration_excluded_by_this_function": False,
        "neutral_source": "separate_explicit_caller_supplied_reference",
        "positive_angle_convention": "distal_relative_to_proximal_about_canonical_prox_axis",
        "configuration_scope": "experimental_not_product_acceptance",
        "configuration": dict(min_samples=min_samples, min_parallel_rms=parallel,
                              max_axis_mismatch=mismatch, min_sign_margin=margin,
                              min_neutral_projection=projection, min_usable_fraction=usable_fraction,
                              sign_strategy=sign_strategy, sign_block_samples=sign_block_samples,
                              min_sign_blocks=min_sign_blocks),
        "limitations": [
            "Acceleration norms near 1 g do not prove gravity-only motion or static neutral.",
            "Caller must verify timing, rigid mounts, hinge applicability and accepted axis fitting.",
            "Only relative axis sign is resolved; anatomical flexion sign remains unknown.",
            "Joint zero is the separate explicit reference, not an inferred straight hand.",
            "Frames do not establish global heading or independently verified angle accuracy.",
            "Sign evidence conditional on fitted axes cannot exclude shared bias or axis-estimation error.",
        ],
    }
    if sign_strategy == "informative_blocks":
        result["method"] = "hinge_gravity_functional_frames_informative_blocks_experimental_v2"
        result["sign_block_index_source"] = "caller_original_sample_indices" if sample_indices is not None else "array_row_order"
    if len(ap) < min_samples:
        result["reason_codes"] = ["too_few_paired_gravity_observations"]
        return result
    norms_p, norms_d = np.linalg.norm(ap, axis=1), np.linalg.norm(ad, axis=1)
    usable = ((norms_p >= .85) & (norms_p <= 1.15) & (norms_d >= .85) & (norms_d <= 1.15)
              & (np.abs(norms_p - norms_d) <= .10))
    result["sign_metrics"] = {"total_samples": int(len(ap)), "usable_samples": int(usable.sum()),
                              "usable_fraction": float(np.mean(usable)),
                              "selected_distal_sign_from_canonical_line": None}
    if usable.sum() < min_samples or np.mean(usable) < usable_fraction:
        result["status"] = "inconsistent"
        result["reason_codes"] = ["insufficient_norm_consistent_acceleration"]
        return result
    component_p = (ap[usable] / norms_p[usable, None]) @ jp
    component_d = (ad[usable] / norms_d[usable, None]) @ jd
    same, opposite = component_p - component_d, component_p + component_d
    same_rms, opposite_rms = _rms(same), _rms(opposite)
    sign = 1 if same_rms < opposite_rms else -1
    best_residual = same if sign == 1 else opposite
    p95 = float(np.percentile(np.abs(best_residual), 95))
    informative_count = int(np.sum((np.abs(component_p) >= parallel) & (np.abs(component_d) >= parallel)))
    result["sign_metrics"].update(
        proximal_parallel_rms=_rms(component_p), distal_parallel_rms=_rms(component_d),
        same_sign_rms=same_rms, opposite_sign_rms=opposite_rms,
        rms_margin=abs(same_rms - opposite_rms), selected_residual_p95=p95,
        informative_sample_count=informative_count,
    )
    if sign_strategy == "informative_blocks":
        full_p, full_d = np.zeros(len(ap)), np.zeros(len(ad))
        full_p[usable], full_d[usable] = component_p, component_d
        informative = usable & (np.abs(full_p) >= parallel) & (np.abs(full_d) >= parallel)
        sign, status, reason, evidence = _block_sign_evidence(
            full_p, full_d, indices, informative, block_samples=sign_block_samples,
            min_samples=min_samples, min_blocks=min_sign_blocks, margin=margin, mismatch=mismatch)
        result["sign_metrics"]["block_evidence"] = evidence
        if status is not None:
            result["status"], result["reason_codes"] = status, [reason]
            return result
        # A block unable to vote can still contradict the selected direction.
        # Check all norm-usable blocks against ONE final sign, including weak
        # projections and blocks whose RMS vote was suppressed by an outlier.
        consistency_blocks = []
        block_ids = indices // sign_block_samples
        for block_id in np.unique(block_ids[usable]):
            keep = usable & (block_ids == block_id)
            if keep.sum() < min_samples:
                continue
            residual_p95 = float(np.percentile(np.abs(full_p[keep]-sign*full_d[keep]), 95))
            consistency_blocks.append(dict(block_id=int(block_id), usable_samples=int(keep.sum()),
                                            final_sign_residual_p95=residual_p95))
        evidence["final_sign_consistency_blocks"] = consistency_blocks
        if any(b["final_sign_residual_p95"] > mismatch for b in consistency_blocks):
            result["status"] = "inconsistent"
            result["reason_codes"] = ["sequence_block_axis_gravity_components_inconsistent"]
            return result
        best_residual = component_p - sign * component_d
        p95 = float(np.percentile(np.abs(best_residual), 95))
        info_residual = full_p[informative] - sign * full_d[informative]
        info_p95 = float(np.percentile(np.abs(info_residual), 95))
        result["sign_metrics"].update(selected_residual_p95=p95,
                                       informative_selected_residual_p95=info_p95)
        if info_p95 > mismatch:
            result["status"] = "inconsistent"
            result["reason_codes"] = ["informative_axis_gravity_components_inconsistent"]
            return result
    if p95 > mismatch:
        result["status"] = "inconsistent"
        result["reason_codes"] = ["paired_axis_gravity_components_inconsistent"]
        return result
    if sign_strategy == "whole_capture" and (informative_count < min_samples
            or min(_rms(component_p), _rms(component_d)) < parallel
            or abs(same_rms - opposite_rms) < margin):
        result["status"] = "sign_unobservable"
        result["reason_codes"] = ["insufficient_parallel_gravity_component_to_resolve_axis_sign"]
        return result
    jd = sign * jd
    result["relative_axis_sign_resolved_conditionally"] = True
    result["sign_metrics"]["selected_distal_sign_from_canonical_line"] = int(sign)
    result["signed_axis_prox"], result["signed_axis_dist"] = jp.tolist(), jd.tolist()
    neutral_norm_p, neutral_norm_d = float(np.linalg.norm(np_)), float(np.linalg.norm(nd))
    result["neutral_metrics"] = {"prox_norm_g": neutral_norm_p, "dist_norm_g": neutral_norm_d}
    if not (.85 <= neutral_norm_p <= 1.15 and .85 <= neutral_norm_d <= 1.15
            and abs(neutral_norm_p - neutral_norm_d) <= .10):
        result["status"] = "inconsistent"
        result["reason_codes"] = ["explicit_neutral_acceleration_norm_inconsistent"]
        return result
    up, ud = np_ / neutral_norm_p, nd / neutral_norm_d
    parallel_p, parallel_d = float(up @ jp), float(ud @ jd)
    proj_p = float(np.linalg.norm(up - parallel_p * jp))
    proj_d = float(np.linalg.norm(ud - parallel_d * jd))
    result["neutral_metrics"].update(
        proximal_axis_component=parallel_p, distal_axis_component=parallel_d,
        axis_component_difference=abs(parallel_p - parallel_d),
        proximal_perpendicular_projection=proj_p, distal_perpendicular_projection=proj_d,
    )
    if abs(parallel_p - parallel_d) > mismatch:
        result["status"] = "inconsistent"
        result["reason_codes"] = ["explicit_neutral_axis_gravity_mismatch"]
        return result
    if min(proj_p, proj_d) < projection:
        result["status"] = "neutral_unobservable"
        result["reason_codes"] = ["explicit_neutral_gravity_nearly_parallel_to_hinge"]
        return result
    fp, fd = _make_frame(jp, up), _make_frame(jd, ud)
    result.update(status="frames_ready", reason_codes=["conditional_offline_frames_only"],
                  prox_sensor_to_functional=fp.tolist(), dist_sensor_to_functional=fd.tolist(),
                  functional_neutral_prox_acc_g=(fp @ np_).tolist(),
                  functional_neutral_dist_acc_g=(fd @ nd).tolist())
    return result
