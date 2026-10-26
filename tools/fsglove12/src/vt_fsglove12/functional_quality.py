"""Shared functional-capture and hinge-model acceptance criteria.

These are checks of the captured rotation model, not anatomical accuracy.
In particular, a two-degree-of-freedom MCP must not be rejected for failing
the single-axis residual criterion used by PIP/IP hinge projection.
"""
from __future__ import annotations

import math


FUNCTIONAL_REFERENCE_P95_MAX_DEG = 5.
HINGE_RESIDUAL_MAX_DEG = 15.
FIT_ACCEPTANCE_POLICY = "functional_capture_model_residual_v1"


def functional_fit_acceptance(mode, *, holdout_p95_deg, moving_p95_deg):
    """Assess a fitted candidate without editing references or measurements.

    The held-out frames take no part in axis fitting. Both their residual
    and the residual across the moving capture must fit the online hinge
    model's existing tolerance. MCP candidates retain their two-dimensional
    motion and therefore receive no single-axis residual gate here.
    """
    if mode not in {"hinge_projection", "mcp_two_dof"}:
        raise ValueError("unsupported functional joint model")
    values = (holdout_p95_deg, moving_p95_deg)
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("functional residuals must be finite nonnegative degrees")
    hinge = mode == "hinge_projection"
    accepted = not hinge or max(values) <= HINGE_RESIDUAL_MAX_DEG + 1e-9
    return dict(policy=FIT_ACCEPTANCE_POLICY, status="accepted" if accepted else "rejected",
                mode=mode, criterion="single_axis_residual" if hinge else "axis_direction_only",
                residual_limit_deg=HINGE_RESIDUAL_MAX_DEG if hinge else None,
                holdout_p95_residual_deg=float(holdout_p95_deg),
                moving_p95_residual_deg=float(moving_p95_deg),
                reason_codes=[] if accepted else ["hinge_model_residual_too_large"],
                anatomical_accuracy_verified=False,
                validation_scope="interleaved_samples_same_capture_not_independent_session")
