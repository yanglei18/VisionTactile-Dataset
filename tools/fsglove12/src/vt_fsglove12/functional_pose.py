"""One owner for functional joint estimation, quality and downstream priors.

FSGlove supplies the installation transform and MANO composer. MCPs retain
their two degrees of freedom; every calibrated PIP/IP uses the SAME stateful
QMT hinge estimator. There is no uncorrected-hinge fallback after rejection.
Finite placeholders are only for composing provisional geometry, never valid
joint measurements. The per-joint masks are part of the result contract.
"""
from copy import deepcopy

import numpy as np

from .config import FINGER_DIPS
from .heading_correction import HINGES
from .hinge_estimator import FunctionalHingeEstimator
from .joint_quality import joint_quality_state
from .upstream_functional import (FINGER_SLOTS, SUPPRESSED_ROTATION_WARNING_DEG,
                                  apply_functional_fingers, FunctionalDependencyError)


REVISION = "unified_functional_calibration_window_v3"
_FUNCTIONAL_SLOTS = tuple(sorted(slot for slots in FINGER_SLOTS.values() for slot in slots))


def _base_output_provenance(result):
    """Capture the official output before adaptation changes its source masks.

    Base validity describes a usable numerical result, NOT an independently
    observed joint or physical accuracy. Explicit sparse priors can be usable
    representations while ``base_joint_observed`` remains false.
    """
    valid, _, _ = joint_quality_state(result)
    available = bool(result.get("pose_available") and result.get("solver_success"))
    return [dict(base_output_valid=bool(available and valid[slot]),
                 base_joint_observed=bool(result["joint_observed_mask"][slot]),
                 base_source_label=result["source_labels"][slot])
            for slot in range(16)]


def _attach_calibration_coverage(result, profile, base):
    """Add results-only provenance, independently of numerical joint validity.

    ``reference_present`` is saved coverage, not a binding/accuracy assertion.
    ``corrected`` means the current functional projection passes its existing
    estimator conditions; it does not mean anatomically validated, nor require
    a new heading observation on every frame. Priors never gain direct coverage.
    Missing references leave the valid base result and display untouched.
    """
    entries = profile.get("functional_fingers", {})
    enabled = profile.get("functional_enabled", True) is not False
    references = [False] * 16
    for finger, slots in FINGER_SLOTS.items():
        entry = entries.get(finger, {})
        for slot in slots:
            references[slot] = str(slot) in entry.get("slot_refs", {})
    joint_valid, joint_display, _ = joint_quality_state(result)
    adapter = result.get("functional_adapter", {})
    applied_slots = set(adapter.get("applied_slots", []))
    applied = [bool(enabled and references[slot] and slot in applied_slots and joint_display[slot])
               for slot in range(16)]
    valid = [bool(applied[slot] and joint_valid[slot] and base[slot]["base_output_valid"])
             for slot in range(16)]
    # The MCP adapter preserves its numerical output even when its existing
    # >10-degree suppressed-rotation warning says correction is not verified.
    # Reflect that distinction in coverage, without changing legacy validity,
    # angles, display, or the adapter's own threshold.
    for slot in (1, 4, 7, 10):
        residual = adapter.get("residual_deg_by_slot", {}).get(str(slot))
        valid[slot] = bool(valid[slot] and isinstance(residual, (int, float))
                           and np.isfinite(residual) and residual <= SUPPRESSED_ROTATION_WARNING_DEG)
    statuses, provenance = [], []
    for slot in range(16):
        status = ("not_applicable" if slot not in _FUNCTIONAL_SLOTS else
                  "reference_missing" if not references[slot] else
                  "disabled" if not enabled else
                  "corrected" if valid[slot] else
                  "degraded" if applied[slot] else "unavailable")
        statuses.append(status)
        source = ("unavailable_geometry" if not joint_display[slot] else
                  "functional" if applied[slot] else
                  "base" if base[slot]["base_joint_observed"] else "prior")
        provenance.append({**base[slot], "source_kind": source,
            "source_label": result["source_labels"][slot],
            "anatomical_accuracy_verified": False})
    reference_count = sum(references)
    reference_coverage = ("complete" if reference_count == len(_FUNCTIONAL_SLOTS) else
                          "partial" if reference_count else "none")
    status = ("disabled" if not enabled else "no_references" if not reference_count else
              "complete" if all(valid[slot] for slot in _FUNCTIONAL_SLOTS) else
              "partial" if any(applied) else "unavailable")
    result.update(joint_calibration_status=statuses,
        functional_calibration_coverage=dict(version=1, order="MANO16",
            expected_slots=list(_FUNCTIONAL_SLOTS), enabled=enabled, status=status,
            reference_coverage=reference_coverage, reference_present_mask=references,
            correction_applied_mask=applied, correction_valid_mask=valid,
            anatomical_accuracy_verified=False),
        joint_output_provenance=provenance)
    return result


class FunctionalPosePipeline:
    def __init__(self, config=None):
        # The functional layer cannot rehabilitate a source rejected as stale
        # by its owner. Retain the stricter of the two freshness policies.
        self.hinges = FunctionalHingeEstimator({
            "max_age_s": min(.15, (config or {}).get("stale_after_s", .15))})

    def reset(self):
        self.hinges.reset()

    def apply(self, result, profile, calibrator, frame, binding, core, *, coupled, algorithm):
        base = _base_output_provenance(result)
        entries = profile.get("functional_fingers", {})
        if not entries or profile.get("functional_enabled", True) is False:
            self.reset()
            if entries:
                result["functional_adapter"] = dict(enabled=False, applied_slots=[],
                    skipped_slots={}, residual_deg_by_slot={}, warnings=[])
            return _attach_calibration_coverage(result, profile, base)
        original = np.asarray(result["anatomical_euler_rad"], dtype=float)
        # Only MCPs use the stateless two-DOF adaptation. In particular this
        # function must NOT produce a second, competing hinge angle.
        try:
            adjusted, diagnostic = apply_functional_fingers(original, entries,
                result["joint_observed_mask"], binding=binding, slots={1, 4, 7, 10})
        except FunctionalDependencyError as exc:
            adjusted = original.copy()
            diagnostic = dict(applied_slots=[], skipped_slots={"dependency": str(exc)},
                warnings=[str(exc)], residual_deg_by_slot={}, projection_deg_by_slot={})
        adjusted = np.asarray(adjusted, dtype=float).copy()
        managed = set()
        joint_valid, joint_display, joint_quality = joint_quality_state(result)
        estimates = {}
        continuity = {}
        failed_mcps = set()
        for finger, slot in (("index", 1), ("middle", 4), ("little", 7), ("ring", 10)):
            if (str(slot) in entries.get(finger, {}).get("slot_refs", {})
                    and slot not in diagnostic["applied_slots"]):
                failed_mcps.add(slot)
                adjusted[slot] = 0.
                joint_valid[slot] = joint_display[slot] = False
                joint_quality[slot] = "unavailable"
                estimates[str(slot)] = dict(valid=False, display_available=False,
                    quality="unavailable", status="functional_mcp_unavailable",
                    reason=diagnostic["skipped_slots"].get(str(slot), "functional_dependency_unavailable"))
        for finger, slot in HINGES.items():
            entry = entries.get(finger, {})
            if str(slot) not in entry.get("slot_refs", {}):
                self.hinges.reset(slot)
                continue
            managed.add(slot)
            angle, detail = self.hinges.apply(slot, frame, profile, calibrator, binding)
            estimates[str(slot)] = detail
            valid = bool(joint_valid[slot] and angle is not None and detail["valid"])
            visible = bool(joint_display[slot] and angle is not None and detail["display_available"])
            joint_valid[slot], joint_display[slot] = valid, visible
            joint_quality[slot] = "valid" if valid else "degraded" if visible else "unavailable"
            # No estimate -> numerical rest placeholder for the compositor.
            # It is masked and labelled, not a reported straight finger.
            adjusted[slot] = [0., 0., float(angle) if visible else 0.]
            if visible:
                diagnostic["applied_slots"].append(slot)
                diagnostic["projection_deg_by_slot"][str(slot)] = float(np.rad2deg(angle))
                if detail.get("residual_deg") is not None:
                    diagnostic["residual_deg_by_slot"][str(slot)] = detail["residual_deg"]
            else:
                diagnostic["skipped_slots"][str(slot)] = detail.get("reason", detail["status"])
            continuity[str(slot)] = detail.get("continuity", {})

        if not managed and not failed_mcps and not diagnostic["applied_slots"]:
            result["functional_adapter"] = {**diagnostic, "enabled": True}
            return _attach_calibration_coverage(result, profile, base)
        result["upstream_raw_anatomical_euler_rad"] = result["anatomical_euler_rad"]
        result["upstream_raw_mano_pose_rotvec"] = result["mano_pose_rotvec"]
        result["upstream_raw_source_labels"] = list(result["source_labels"])
        result["unconstrained_joint_observed_mask"] = list(result["joint_observed_mask"])
        result["upstream_raw_sparse_prior"] = deepcopy(result["sparse_prior"])
        diagnostic["applied_slots"] = sorted(set(diagnostic["applied_slots"]))
        diagnostic.update(enabled=True, angle_continuity_by_slot=continuity,
            depends_on_previous_frame=bool(managed), unified_hinge_slots=sorted(managed),
            no_uncorrected_hinge_fallback=True)

        if coupled:
            options = profile["missing_slot_options"]
            for finger, dip in FINGER_DIPS.items():
                pip = dip - 1
                if pip not in managed or result["observed_mask"][dip]:
                    continue
                parent = estimates[str(pip)]
                joint_valid[dip], joint_display[dip] = joint_valid[pip], joint_display[pip]
                joint_quality[dip] = joint_quality[pip]
                estimates[str(dip)] = dict(status="coupled_from_parent", parent_slot=pip,
                    valid=joint_valid[dip], display_available=joint_display[dip],
                    quality=joint_quality[dip], measured=False)
                ratio = options.get("dip_ratio_by_finger", {}).get(finger, options["dip_ratio"])
                value = float(adjusted[pip, 2])
                target = min(options["upstream_dip_max_rad"], ratio * max(0., value))
                adjusted[dip] = [0., 0., target]
                item = result["sparse_prior"]["dip_by_finger"][finger]
                item.update(applied=joint_display[dip], valid=joint_valid[dip],
                    quality=joint_quality[dip], pip_flexion_rad=value,
                    unclipped_target_rad=ratio * value, target_flexion_rad=target,
                    clip_applied=bool(ratio * value < 0 or ratio * value > options["upstream_dip_max_rad"]),
                    source="unified_hinge_coupling_not_measurement")
                result["sparse_prior"]["target_anatomical_euler_rad_by_slot"][str(dip)] = adjusted[dip].tolist()
                zeros = set(result["sparse_prior"]["zero_prior_slots"])
                zeros.add(dip) if target == 0. else zeros.discard(dip)
                result["sparse_prior"]["zero_prior_slots"] = sorted(zeros)
                result["source_labels"][dip] = ("pip_coupled_dip_from_continuous_hinge_not_observation"
                    if joint_display[dip] else "unavailable_geometry_placeholder_not_measurement")
                result["joint_observed_mask"][dip] = False
                result["inferred_mask"][dip] = True

        with core.torch.no_grad():
            tensor = calibrator._axislayer.compose(
                core.torch.from_numpy(adjusted.reshape(1, 16, 3)).float())[0]
            mano = tensor.detach().cpu().numpy().copy()
        mano[0] = np.asarray(result["upstream_raw_mano_pose_rotvec"])[0]
        if mano.shape != (16, 3) or not np.isfinite(mano).all():
            raise ValueError("functional MANO composition must be finite (16, 3)")
        for slot in set(diagnostic["applied_slots"]) | managed | failed_mcps:
            result["joint_observed_mask"][slot] = False
            result["inferred_mask"][slot] = True
            result["source_labels"][slot] = (
                "continuous_qmt_hinge_not_direct_joint_measurement" if slot in managed and joint_display[slot]
                else "unavailable_geometry_placeholder_not_measurement" if slot in managed or slot in failed_mcps
                else "qmt_functional_mcp_not_direct_joint_measurement")
        result.update(algorithm=algorithm, anatomical_euler_rad=adjusted.tolist(),
            mano_pose_rotvec=mano.tolist(), functional_adapter=diagnostic,
            joint_valid_mask=joint_valid, joint_display_available_mask=joint_display,
            joint_quality=joint_quality, joint_estimation=estimates,
            pose_postprocessing="qmt_functional_axis_and_joint_constraint",
            pose_processing_revision=REVISION, physical_joint_axes_verified=False)
        # Read-only compatibility diagnostic for existing log readers. This is
        # the SAME estimator state, not a little-specific algorithm or fallback.
        if "8" in estimates:
            result["little_relative_heading"] = deepcopy(estimates["8"])
        return _attach_calibration_coverage(result, profile, base)
