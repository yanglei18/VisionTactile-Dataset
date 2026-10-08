"""Independent regressions for observer continuity and evidence isolation.

These exercise known synthetic source motion, not hardware/anatomical accuracy.
In particular, a changing correction parameter must not manufacture the motion
evidence used to keep changing that same parameter.
"""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("qmt")

from test_heading_window import root_trajectory, trajectory
from test_hinge_estimator import fixture_model, frame_for_bend
from vt_fsglove12.heading_window import FunctionalHeadingWindow
from vt_fsglove12.hinge_estimator import DEFAULTS, FunctionalHingeEstimator


def test_correction_parameter_cannot_supply_articulation_after_source_motion_stops():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    cal, binding, profile, *_ = model
    stop = 2.6
    after_stop = []
    accepted = False
    for i in range(151):
        t = i / 30
        physical_time = min(t, stop)
        frame = frame_for_bend(model, 1 + t,
            .65 + .45 * np.sin(5 * physical_time), drift=30.,
            root=root_trajectory(physical_time))
        angle, detail = estimator.apply(8, frame, profile, cal, binding)
        assert angle is not None
        accepted |= detail["heading_window"].get("accepted", False)
        if t > stop + .4:
            after_stop.append((angle, detail["delta_rad"]))
            assert not detail["heading_window"].get("recent_articulation", False), (
                "identical physical q1/q2 cannot become articulation because "
                "their projection used a changing yaw parameter")
            assert not detail["heading_updated"]
    assert accepted, "the fixture must first create a real QMT target"
    np.testing.assert_allclose(after_stop, [after_stop[0]] * len(after_stop), atol=1e-10)


@pytest.mark.parametrize("fault", ["missing", "stale", "skew"])
def test_soft_pair_fault_invalidates_window_without_erasing_applied_heading(fault):
    rows, estimator, model = trajectory(30, end=4., drift_rate=1.)
    cal, binding, profile, *_ = model
    state = estimator._states[8]
    assert len(state.observer.history) >= 20
    assert any(row[3]["heading_window"].get("accepted") for row in rows)
    old_delta = state.delta
    bad = frame_for_bend(model, 5 + 1 / 30, .7, root=root_trajectory(4.))
    if fault == "missing":
        del bad["nodes"]["10"]
    else:
        bad["nodes"]["10"]["age_s"] = .2 if fault == "stale" else .04
    angle, detail = estimator.apply(8, bad, profile, cal, binding)
    assert angle is None and not detail["hard_reset"]
    assert state.delta == old_delta
    assert not state.observer.history, "a known bad interval must interrupt fit evidence"
    assert state.observer.target is None
    recovered = frame_for_bend(model, 5 + 2 / 30, 1.1, root=root_trajectory(4.))
    angle, detail = estimator.apply(8, recovered, profile, cal, binding)
    assert angle is not None and detail["display_available"]
    assert detail["delta_rad"] == old_delta
    assert detail["heading_window"]["window_samples"] == 1
    assert not detail["heading_updated"]


def test_protection_discards_the_old_fit_target_and_requires_a_new_clean_window():
    rows, estimator, model = trajectory(30, end=4., drift_rate=1.)
    cal, binding, profile, *_ = model
    assert any(row[3]["heading_window"].get("accepted") for row in rows)
    state = estimator._states[8]
    old_delta = state.delta
    protected = frame_for_bend(model, 5 + 1 / 30, .7, root=root_trajectory(4.))
    protected["nodes"]["10"]["input_filter"] = {"gyro_rejected": True}
    angle, detail = estimator.apply(8, protected, profile, cal, binding)
    assert angle is not None and detail["quality"] == "degraded"
    assert not state.observer.history and state.observer.target is None
    for i in range(2, 60):
        t = 4 + i / 30
        frame = frame_for_bend(model, 1 + t, .65 + .45 * np.sin(5*t),
                               drift=t, root=root_trajectory(t))
        angle, detail = estimator.apply(8, frame, profile, cal, binding)
        assert angle is not None
        assert detail["delta_rad"] == old_delta
        assert not detail["heading_updated"] and not detail["valid"]


def test_one_new_node_is_not_a_fresh_pair_or_new_heading_evidence():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    cal, binding, profile, *_ = model
    frame = frame_for_bend(model, 1., .4)
    estimator.apply(8, frame, profile, cal, binding)
    index = frame["nodes"]["9"]["sample_index"]
    for i in range(1, 7):
        partial = frame_for_bend(model, 1 + i / 60, .5 + i / 20)
        partial["nodes"]["9"]["sample_index"] = index
        partial["nodes"]["9"]["age_s"] = i / 600
        angle, detail = estimator.apply(8, partial, profile, cal, binding)
        assert angle is not None and not detail["fresh_pair"]
        assert not detail["heading_updated"] and not detail["valid"]
        assert len(estimator._states[8].observer.history) == 1


def test_increasing_host_time_does_not_hide_reversed_source_time():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    cal, binding, profile, *_ = model
    estimator.apply(8, frame_for_bend(model, 1., .4), profile, cal, binding)
    frame = frame_for_bend(model, 1.02, .5)
    for key in ("9", "10"):
        frame["nodes"][key]["age_s"] = .08
    angle, detail = estimator.apply(8, frame, profile, cal, binding)
    assert angle is None and detail["hard_reset"]
    assert detail["reason"] == "source_time_reversed"
    assert 8 not in estimator._states


@pytest.mark.parametrize("sample_rate,window_s", [(10., 4.), (30., 10.)])
def test_history_stays_bounded_at_supported_window_and_observer_rates(sample_rate, window_s):
    settings = {**DEFAULTS, "heading_sample_rate_hz": sample_rate,
                "heading_window_s": window_s}
    observer = FunctionalHeadingWindow(settings)
    q1 = Rotation.identity()
    axis = np.array([1., 0., 0.])
    q2 = Rotation.from_rotvec(axis * .6)
    maximum = math.floor(sample_rate * window_s) + 1
    for i in range(120 * 30 + 1):
        target, detail = observer.observe(1 + i / 120, q1, q2, axis, 0.)
        assert target is None
        assert len(observer.history) <= maximum
        assert detail["window_span_s"] <= window_s + 1e-8
    assert len(observer.history) >= maximum - 1


def test_fit_evidence_does_not_mark_unconverged_correction_valid():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    cal, binding, profile, *_ = model
    saw_accepted_unconverged = False
    for i in range(151):
        t = i / 30
        frame = frame_for_bend(model, 1 + t, .65 + .45 * np.sin(5*t),
                               drift=25., root=root_trajectory(t))
        angle, detail = estimator.apply(8, frame, profile, cal, binding)
        state = estimator._states[8]
        if state.observer.target is not None:
            error = abs((state.observer.target - state.delta + np.pi) % (2*np.pi) - np.pi)
            if error > np.deg2rad(.5):
                saw_accepted_unconverged = True
                assert not detail["valid"] and detail["quality"] == "degraded"
                assert angle is not None and detail["display_available"]
    assert saw_accepted_unconverged
