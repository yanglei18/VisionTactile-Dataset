"""Known causal trajectories test evidence, cadence, and real QMT integration."""
import importlib

import numpy as np
import pytest

pytest.importorskip("qmt")

from test_hinge_estimator import frame_for_bend, fixture_model, SLOTS
from vt_fsglove12.hinge_estimator import FunctionalHingeEstimator


def root_trajectory(t):
    return (45*np.sin(1.1*t), 35*np.cos(.9*t), 25*np.sin(.5*t))


def trajectory(rate, *, slot=8, end=16., offaxis=0., drift_rate=.2, estimator=None):
    model = fixture_model()
    cal, binding, profile, *_ = model
    estimator = estimator or FunctionalHingeEstimator()
    rows = []
    for i in range(int(end * rate) + 1):
        t = i / rate
        bend = .65 + .45*np.sin(2*np.pi*.8*t)
        frame = frame_for_bend(model, 1.+t, bend, drift=drift_rate*t,
                               root=root_trajectory(t), offaxis=offaxis)
        angle, d = estimator.apply(slot, frame, profile, cal, binding)
        rows.append((t, bend, angle, d))
    return rows, estimator, model


@pytest.mark.parametrize("slot", SLOTS)
def test_excited_backward_window_corrects_real_slow_yaw_drift(slot):
    rows, _, _ = trajectory(30, slot=slot)
    assert all(row[2] is not None for row in rows)
    assert any(row[3]["heading_window"].get("accepted") for row in rows)
    assert any(row[3]["valid"] for row in rows)
    assert rows[0][3]["delta_rad"] == 0.
    # A causal finite history has bounded lag; it is not zero-delay yaw truth.
    assert abs(np.rad2deg(rows[-1][3]["delta_rad"]) + 3.2) < 1.2
    assert abs(np.rad2deg(rows[-1][2] - rows[-1][1])) < 1.5
    for first, second in zip(rows, rows[1:]):
        change = (second[3]["delta_rad"] - first[3]["delta_rad"] + np.pi) % (2*np.pi) - np.pi
        assert abs(change) <= np.deg2rad(5/30) + 1e-9


def test_30_and_120_hz_same_source_trajectory_have_equivalent_heading_and_angle():
    low, _, _ = trajectory(30)
    high, _, _ = trajectory(120)
    assert len(high[::4]) == len(low)
    # Real time, not number of renderer/solver calls, controls all adaptation.
    np.testing.assert_allclose([r[2] for r in low], [r[2] for r in high[::4]], atol=np.deg2rad(.15), rtol=0)
    np.testing.assert_allclose([r[3]["delta_rad"] for r in low],
                               [r[3]["delta_rad"] for r in high[::4]], atol=np.deg2rad(.15), rtol=0)
    assert max(r[3]["heading_window"].get("window_samples", 0) for r in high) <= 41


def test_offaxis_during_flexion_and_palm_motion_cannot_be_trusted_as_large_yaw():
    rows, _, _ = trajectory(30, offaxis=3., drift_rate=0.)
    assert all(row[2] is not None for row in rows)
    rejected = [r for r in rows if r[3]["heading_window"].get("status") in
                ("heading_model_inconsistent", "heading_subwindows_disagree",
                 "heading_window_does_not_improve_both_halves", "heading_increases_model_variation")]
    assert rejected, "model error must be tested, not hidden behind an ideal hinge fixture"
    assert max(abs(np.rad2deg(r[3]["delta_rad"])) for r in rows) < 2.


def test_static_parent_and_pure_yaw_rotation_do_not_prove_model_consistency():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    cal, binding, profile, *_ = model
    for i in range(181):
        t = i/30
        frame = frame_for_bend(model, 1+t, .6+.45*np.sin(5*t), drift=15,
                               root=(0., 0., 25*t))
        angle, d = estimator.apply(8, frame, profile, cal, binding)
        assert angle is not None and d["delta_rad"] == 0.
    assert d["heading_window"]["status"] == "insufficient_parent_tilt_diversity"


def test_heading_evidence_expiry_does_not_delete_fresh_pose_or_reset_delta():
    rows, estimator, model = trajectory(30, drift_rate=0., end=8.)
    assert any(row[3]["valid"] for row in rows)
    cal, binding, profile, *_ = model
    delta = rows[-1][3]["delta_rad"]
    for i in range(1, 301):
        stamp = 9.+i/10
        frame = frame_for_bend(model, stamp, .7, vertical=True)
        angle, d = estimator.apply(8, frame, profile, cal, binding)
        assert angle == pytest.approx(.7, abs=1e-7)
        assert d["delta_rad"] == pytest.approx(delta, abs=1e-10)
        assert d["display_available"]
    assert not d["valid"] and d["heading_quality"] == "held_unobservable"


def test_failed_qmt_window_is_soft_rejection_and_current_angle_still_follows(monkeypatch):
    module = importlib.import_module("qmt.functions.heading_correction")
    monkeypatch.setattr(module, "estimateDelta1d", lambda *args: (float("nan"), .9, 0.))
    rows, _, _ = trajectory(30, end=4., drift_rate=0.)
    assert any(r[3]["status"] == "heading_window_fit_rejected" for r in rows)
    for _, expected, actual, d in rows:
        assert actual == pytest.approx(expected, abs=1e-7)
        assert not d["valid"] and d["delta_rad"] == 0.


def test_duplicate_render_calls_do_not_change_parameter_pacing_or_window(monkeypatch):
    model = fixture_model()
    cal, binding, profile, *_ = model
    a, b = FunctionalHingeEstimator(), FunctionalHingeEstimator()
    for i in range(241):
        t = i/30
        frame = frame_for_bend(model, 1+t, .65+.45*np.sin(5*t),
                               drift=.2*t, root=root_trajectory(t))
        angle_a, da = a.apply(8, frame, profile, cal, binding)
        angle_b, db = b.apply(8, frame, profile, cal, binding)
        assert angle_a == pytest.approx(angle_b, abs=1e-12)
        assert da["delta_rad"] == db["delta_rad"]
        for _ in range(3):
            _, duplicate = b.apply(8, frame, profile, cal, binding)
            assert not duplicate["fresh_pair"] and not duplicate["heading_updated"]
    assert len(a._states[8].observer.history) == len(b._states[8].observer.history)
