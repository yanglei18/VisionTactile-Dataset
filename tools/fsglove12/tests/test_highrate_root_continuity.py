"""Missing a root snapshot is not a new relative-heading calibration."""
from copy import deepcopy

import numpy as np
import pytest

from test_upstream_backend import official_core
from test_upstream_functional_integration import trial


@pytest.mark.parametrize('first_bend,next_bend', [(.6, .6), (np.deg2rad(179.9), np.deg2rad(180.1))])
def test_brief_missing_root_keeps_hinge_reference_without_publishing_a_pose(
        official_core, first_bend, next_bend):
    backend, profile, frame, _, _ = trial(official_core, "little")
    initial = backend.solve(frame(first_bend, sequence=400), profile)
    # A bound calibration already defines the current projection, but a
    # single source pair cannot supply independent heading-window evidence.
    assert initial["joint_display_available_mask"][8]
    assert not initial["joint_valid_mask"][8] and initial['joint_quality'][8] == 'degraded'
    assert initial['anatomical_euler_rad'][8][2] == pytest.approx(first_bend, abs=1e-12)
    state = backend._functional_pipeline.hinges._states[8]
    delta, source_time = state.delta, state.source_time
    history = deepcopy(list(state.observer.history))
    assert delta == 0. and state.observer.evidence_time is None
    missing = frame(first_bend, sequence=401)
    missing["nodes"].pop(backend._root_node)
    for _ in range(8):
        result = backend.solve(missing, profile)
        assert not result["pose_available"] and result["reason"] == "missing_or_stale_root"
        assert backend._functional_pipeline.hinges._states[8] is state
        assert state.delta == delta and state.source_time == source_time
        assert len(state.observer.history) == len(history)
        for old, current in zip(history, state.observer.history):
            np.testing.assert_equal(old, current)
    recovered = backend.solve(frame(next_bend, sequence=402), profile)
    assert recovered['pose_available'] and recovered["joint_display_available_mask"][8]
    assert not recovered["joint_valid_mask"][8] and recovered['joint_quality'][8] == 'degraded'
    assert recovered['anatomical_euler_rad'][8][2] == pytest.approx(next_bend, abs=1e-12)
    assert recovered['joint_estimation']['8']['continuity']['reset_reason'] is None
    assert recovered['joint_estimation']['8']['continuity']['branch_corrected'] == (next_bend > np.pi)
    assert recovered['joint_estimation']['8']['delta_rad'] == delta
    assert backend._functional_pipeline.hinges._states[8] is state


def test_real_input_health_fault_still_invalidates_root_reference(official_core):
    backend, profile, frame, _, _ = trial(official_core, "little")
    backend.solve(frame(.6, sequence=400), profile)
    failed = frame(.6, sequence=401)
    failed["nodes"][backend._root_node].update(
        orientation_valid=False, input_health=dict(healthy=False, state="faulted"))
    before = deepcopy(failed)
    result = backend.solve(failed, profile)
    assert not result["pose_available"] and result["reason"] == "input_health_invalid"
    assert backend._functional_pipeline.hinges._states == {}
    assert result["calibration_rejections"][backend._root_node] == "input_health_invalid"
    assert failed == before


@pytest.mark.parametrize("finger,slot", [("index", 2), ("middle", 5), ("ring", 11), ("little", 8), ("thumb", 15)])
def test_repeated_flexion_and_return_have_same_stability_at_30_and_120_solves(official_core, finger, slot):
    # Both schedules observe one deterministic 120 Hz orientation stream.
    # This isolates solve-cadence effects; no replay timestamps are invented
    # for measured hardware and no empirical accuracy claim is made.
    outputs, qualities = {}, {}
    for stride in (1, 4):
        backend, profile, frame, _, _ = trial(official_core, finger)
        sampled, sampled_quality = {}, {}
        for index in range(0, 361, stride):
            t = index / 120.
            bend = .45 * (1. - np.cos(2. * np.pi * t))
            sample = frame(bend, root=(1.1 + .08 * np.sin(t), -.8, .3), sequence=1000 + index)
            sample["timestamp_s"] = 300. + t
            result = backend.solve(sample, profile)
            # A first frame is not heading evidence. Four fingers later have
            # excited parent segments because this fixture flexes their MCPs
            # too; the thumb fixture moves only its IP, so its parent remains
            # insufficiently excited. Both pending and qualified states must
            # expose the same actual zero-yaw calibrated projection.
            assert result["joint_display_available_mask"][slot]
            detail = result['joint_estimation'][str(slot)]
            qualified = index >= 300 and slot != 15
            assert result['joint_valid_mask'][slot] == qualified
            assert result['joint_quality'][slot] == ('valid' if qualified else 'degraded')
            assert result['joint_calibration_status'][slot] == ('corrected' if qualified else 'degraded')
            if qualified:
                assert detail['heading_quality'] == 'observed_window'
                assert 0. <= detail['heading_reference_age_s'] <= .5 + 1e-8
            else:
                assert detail['heading_reference_age_s'] is None
            assert detail['delta_rad'] == pytest.approx(0., abs=1e-10)
            assert not detail['heading_updated']
            assert result['anatomical_euler_rad'][slot][2] == pytest.approx(bend, abs=1e-6)
            if index % 4 == 0:
                sampled[index] = np.asarray(result["anatomical_euler_rad"]).copy()
                sampled_quality[index] = (result['joint_valid_mask'], result['joint_quality'],
                                          result['joint_calibration_status'])
        outputs[stride] = sampled
        qualities[stride] = sampled_quality
        # Three flexion cycles must not walk away from the calibrated return.
        for index in (120, 240, 360):
            assert abs(sampled[index][slot, 2]) < 1e-6
    for index in outputs[4]:
        np.testing.assert_allclose(outputs[1][index], outputs[4][index], atol=1e-6, rtol=0.)
        assert qualities[1][index] == qualities[4][index]
