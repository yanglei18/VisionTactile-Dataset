"""Actual sample windows, no timers/GUI/hardware or synthetic zeroing."""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.functional_capture import FunctionalReferenceCapture


def sample(t, i, *, angles=(0., 0.), parent=None, epoch=0):
    return ({"timestamp_s": t, "nodes": {"source": {"sample_index": i}}},
            {str(slot): dict(quat_xyzw=Rotation.from_euler("z", angle, degrees=True).as_quat().tolist(),
                            identity=((epoch, i if parent is None else parent), (epoch, i)))
             for slot, angle in zip((1, 2), angles)})


def run(capture, start, end, angle=lambda t: (0., 0.), **kwargs):
    status = None
    for i, t in enumerate(np.arange(start, end + .001, .02), int(start * 50) + 1):
        frame, local = sample(float(t), i, angles=angle(t), **kwargs)
        status = capture.observe(frame, local, float(t))
        if status in {"accepted", "partial", "failed"}:
            break
    return status


def test_stable_reference_requires_real_two_second_window_and_preserves_inputs():
    capture = FunctionalReferenceCapture((1, 2), 3.)
    assert run(capture, 0., 2.9) == "pending"
    assert run(capture, 3., 4.98) == "waiting_for_stability"
    frame, local = sample(5., 1000)
    original = deepcopy((frame, local))
    assert capture.observe(frame, local, 5.) == "accepted"
    assert (frame, local) == original
    assert capture.summary(5.)["window_end_s"] - capture.summary(5.)["window_start_s"] >= 2.
    capture.selected[0]["nodes"].clear()
    assert frame == original[0]


def test_moving_reference_is_replaced_by_a_later_real_stable_window():
    capture = FunctionalReferenceCapture((1, 2), 0.)
    assert run(capture, 0., 2.98, lambda t: (30 * np.sin(t * 5), 0.)) == "waiting_for_stability"
    assert run(capture, 3., 5.2) == "accepted"
    assert capture.summary(5.2)["window_start_s"] >= 2.9
    # The returned raw records are the selected window, not copies retimed
    # to appear fresh. Neutral is an operator-declared posture, not inferred.
    stamps = [row["timestamp_s"] for row in capture.selected]
    assert max(np.diff(stamps)) <= .021


def test_deadline_can_accept_only_stable_joints_for_explicit_partial_fit():
    capture = FunctionalReferenceCapture((1, 2), 0., wait_s=3.)
    assert run(capture, 0., 3., lambda t: (0., 40 * np.sin(t * 8))) == "partial"
    summary = capture.summary(3.)
    assert summary["stable_slots"] == ["1"]
    assert summary["unstable_slots"] == ["2"]
    assert summary["diagnostics"]["2"]["p95_deg"] > 5.


def test_all_moving_expires_without_selecting_bad_reference():
    capture = FunctionalReferenceCapture((1, 2), 0., wait_s=3.)
    assert run(capture, 0., 3., lambda t: (30 * np.sin(t * 8), 40 * np.cos(t * 8))) == "failed"
    assert capture.selected is None


def test_repeated_parent_does_not_fake_independent_pair_samples():
    capture = FunctionalReferenceCapture((1, 2), 0., wait_s=3.)
    assert run(capture, 0., 3., parent=1) == "failed"
    assert capture.selected is None


def test_repeated_frame_does_not_advance_reference_window():
    capture = FunctionalReferenceCapture((1, 2), 0., wait_s=3.)
    frame, local = sample(0., 1)
    for t in np.arange(0., 3.01, .02):
        capture.observe(frame, local, float(t))
    assert capture.status == "failed"


def test_long_gap_cannot_count_as_continuous_stability():
    capture = FunctionalReferenceCapture((1, 2), 0., wait_s=3.)
    run(capture, 0., .98)
    assert run(capture, 2., 3.) == "failed"


@pytest.mark.parametrize("change", ["epoch", "index"])
def test_changed_source_identity_clears_prior_window(change):
    capture = FunctionalReferenceCapture((1, 2), 0., wait_s=3.)
    run(capture, 0., 1.98)
    for i, t in enumerate(np.arange(2., 3.01, .02), 1):
        frame, local = sample(float(t), i if change == "index" else i + 100,
                              epoch=1 if change == "epoch" else 0)
        capture.observe(frame, local, float(t))
    assert capture.status == "failed"


def test_window_memory_and_assessment_rate_are_bounded(monkeypatch):
    capture = FunctionalReferenceCapture((1, 2), 0.)
    checks=[]
    original=capture._assess
    def assess(now):
        checks.append(now)
        original(now)
    monkeypatch.setattr(capture, "_assess", assess)
    for i in range(11000):
        now=i/1000.
        frame,local=sample(now,i+1,angles=(30*np.sin(now*8),0.))
        capture.observe(frame,local,now)
    assert len(capture._rows) <= 123
    assert len(checks) <= 111
