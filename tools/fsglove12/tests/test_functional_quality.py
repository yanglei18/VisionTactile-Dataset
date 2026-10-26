"""Recorded model-fit failures must not become accepted hinge references."""
import pytest

from vt_fsglove12.functional_quality import functional_fit_acceptance


@pytest.mark.parametrize("holdout,moving,accepted", [
    # Retained profile diagnostics from the 2026-10-26 physical experiment.
    # These are projection residuals, not independently measured angle errors.
    (27.098334978534307, 27.128649768553245, False),
    (27.21056918943867, 28.268557908957206, False),
    (15.437242887527121, 16.198496019112913, False),
    (13.49227276969756, 12.93947382713082, True),
    (6.066824049798626, 6.1371818666932, True),
    (4., 20., False),  # A good holdout cannot hide a bad moving-capture fit.
    (20., 4., False),  # Training support cannot overrule held-out failure.
])
def test_recorded_and_independent_residual_checks(holdout, moving, accepted):
    result = functional_fit_acceptance("hinge_projection",
        holdout_p95_deg=holdout, moving_p95_deg=moving)
    assert result["status"] == ("accepted" if accepted else "rejected")
    assert not result["anatomical_accuracy_verified"]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1., True, None])
def test_invalid_residual_evidence_cannot_pass_acceptance(bad):
    with pytest.raises(ValueError, match="residuals"):
        functional_fit_acceptance("hinge_projection", holdout_p95_deg=bad, moving_p95_deg=0.)
