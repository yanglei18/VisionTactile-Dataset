"""Causal pre-VQF input rejection without hardware or raw-record mutation."""
from copy import deepcopy
import json
import math

import numpy as np
import pytest

from vt_fsglove12.input_guard import DEFAULTS, ImuInputGuard, guard_settings


G = 9.80665
RAW = (0, 0, 0, 0, 0, 8197)


def guard(**options):
    return ImuInputGuard(dict(sample_rate_hz=100.0, input_filter=options))


def feed(instance, *, node=1, raw=RAW, gyro=(0., 0., 0.), accel=(0., 0., G), bias=None):
    return instance.process(node, raw, gyro, accel, gyro_bias_rad_s=bias)


def test_effective_settings_are_detached_and_missing_section_is_legacy_disabled():
    assert guard_settings({}) == {**DEFAULTS, "enabled": False}
    config = dict(input_filter={})
    actual = guard_settings(config)
    assert actual == DEFAULTS and actual["enabled"]
    actual["enabled"] = False
    assert config == dict(input_filter={}) and DEFAULTS["enabled"]


@pytest.mark.parametrize("axis", range(3))
@pytest.mark.parametrize("value", [32764, -32764, -32768])
def test_gyro_boundary_each_axis_and_sign_is_rejected_without_touching_raw(axis, value):
    instance = guard()
    baseline = np.array([.1, .2, .3])
    feed(instance, gyro=baseline)
    raw = np.array(RAW, dtype=np.int16)
    raw[axis] = value
    original = raw.copy()
    incoming = baseline + (.2, -.1, .4)
    incoming[axis] = 5.
    used, accel, status = feed(instance, raw=raw, gyro=incoming)
    np.testing.assert_array_equal(raw, original)
    expected = incoming.copy()
    expected[axis] = baseline[axis]
    np.testing.assert_array_equal(used, expected)
    # The boundary axis alone is held; legitimate simultaneous movement on
    # both other axes must not be replaced by their previous measurements.
    np.testing.assert_array_equal(used[np.arange(3) != axis], incoming[np.arange(3) != axis])
    np.testing.assert_array_equal(accel, (0., 0., G))
    assert status["gyro_action"] == "held"
    assert status["gyro_rejected"] and not status["accel_rejected"]
    assert status["reasons"] == ["gyro_boundary"]
    assert status["raw_samples"] == 2 and status["rejected_samples"] == 1
    assert status["orientation_valid"]
    assert status["gyro_rejected_axes"] == ["xyz"[axis]]
    assert status["gyro_action_by_axis"] == {name: "held" if i == axis else "pass" for i, name in enumerate("xyz")}


def test_normal_input_and_exact_jump_threshold_pass_without_smoothing_or_delay():
    instance = guard()
    for gyro, accel in [((0., 0., 0.), (0., 0., G)),
                        ((math.radians(120.), -.5, .1), (1.5 * G, 0., G)),
                        ((2., -.4, .2), (G, .1, G))]:
        used_gyro, used_accel, status = feed(instance, gyro=gyro, accel=accel)
        np.testing.assert_array_equal(used_gyro, gyro)
        np.testing.assert_array_equal(used_accel, accel)
        assert status["gyro_action"] == status["accel_action"] == "pass"
        assert status["reasons"] == [] and status["rejected_samples"] == 0


def test_isolated_subboundary_spike_holds_then_baseline_returns_immediately():
    instance = guard()
    feed(instance, gyro=(.1, .2, .3))
    used, _, status = feed(instance, raw=(12000, 0, 0, 0, 0, 8197), gyro=(5., .25, .4))
    np.testing.assert_array_equal(used, (.1, .25, .4))
    assert status["reasons"] == ["gyro_jump"] and status["gyro_run"] == 1
    used, _, status = feed(instance, gyro=(.2, .3, .4))
    np.testing.assert_array_equal(used, (.2, .3, .4))
    assert status["gyro_action"] == "pass" and status["gyro_run"] == 0
    assert status["gyro_rejected_samples"] == 1


def test_consistent_second_candidate_confirms_sustained_valid_step_exactly():
    instance = guard()
    feed(instance)
    _, _, first = feed(instance, gyro=(5., 0., 0.), accel=(20., 0., G))
    used_gyro, used_accel, second = feed(instance, gyro=(5.1, .1, 0.), accel=(21., 1., G))
    assert first["gyro_rejected"] and first["accel_rejected"]
    np.testing.assert_array_equal(used_gyro, (5.1, .1, 0.))
    np.testing.assert_array_equal(used_accel, (21., 1., G))
    assert second["gyro_action"] == second["accel_action"] == "pass"
    assert second["gyro_run"] == second["accel_run"] == 0
    assert second["orientation_valid"]


def test_inconsistent_candidates_do_not_confirm_and_nonfinite_clears_candidate():
    instance = guard()
    feed(instance)
    for value in (5., -5., 5.):
        assert feed(instance, gyro=(value, 0., 0.))[2]["gyro_rejected"]
    feed(instance, gyro=(math.nan, 0., 0.))
    assert feed(instance, gyro=(5., 0., 0.))[2]["gyro_rejected"]
    assert not feed(instance, gyro=(5., 0., 0.))[2]["gyro_rejected"]


def test_nonfinite_burst_gyro_hold_is_bounded_then_bad_axis_bias_only():
    instance = guard(max_invalid_s=.02)
    feed(instance, gyro=(.7, .2, -.3))
    bias = np.array([.03, -.04, .01])
    for index in range(1, 20):
        incoming = (math.nan, .02 * index, -.03 * index)
        used, _, status = feed(instance, gyro=incoming, bias=bias)
        expected = (.7 if index <= 2 else bias[0], incoming[1], incoming[2])
        np.testing.assert_array_equal(used, expected)
        np.testing.assert_array_equal(used[1:], incoming[1:])
        if index > 2:
            assert used[0] - bias[0] == 0.
            assert not np.array_equal(used[1:], bias[1:])
        assert status["gyro_action"] == ("held" if index <= 2 else "zero_motion")
        assert status["gyro_run"] == status["gyro_rejected_samples"] == index
        assert status["orientation_valid"] == (index <= 2)
    used, _, status = feed(instance, gyro=(.8, .2, -.3))
    np.testing.assert_array_equal(used, (.8, .2, -.3))
    assert status["orientation_valid"] and status["gyro_run"] == 0


def test_zero_motion_without_bias_and_zero_hold_option():
    instance = guard(max_hold_samples=0)
    feed(instance, gyro=(.5, 0., 0.))
    used, _, status = feed(instance, raw=(32764, 0, 0, 0, 0, 8197), gyro=(5., 0., 0.))
    np.testing.assert_array_equal(used, np.zeros(3))
    assert status["gyro_action"] == "zero_motion"


@pytest.mark.parametrize("sign", [-1, 1])
def test_smooth_approach_to_boundary_is_not_replaced_with_zero_motion(sign):
    instance = guard()
    for counts in (0, 10000, 20000, 30000, 32764, 32764, 32764, 30000, 20000, 10000, 0):
        value = sign * math.radians(counts * .00875)
        used, _, status = feed(instance, raw=(sign * counts, 0, 0, 0, 0, 8197), gyro=(value, .1, -.2))
        np.testing.assert_array_equal(used, (value, .1, -.2))
        assert status["gyro_action"] == "pass"
        assert status["gyro_boundary_integrated_axes"] == (["x"] if counts == 32764 else [])
    assert status["gyro_boundary_samples_by_axis"] == {"x": 3, "y": 0, "z": 0}
    assert status["gyro_rejected_samples"] == 0


def test_initial_boundary_requires_confirmation_and_reversals_are_still_rejected():
    instance = guard()
    for sign in (1, -1, 1, -1):
        used, _, status = feed(instance, raw=(sign * 32764, 0, 0, 0, 0, 8197), gyro=(sign * 5., 0., 0.))
        assert not status["orientation_valid"] and status["gyro_rejected"]
        assert used[0] == 0.
    used, _, status = feed(instance, raw=(-32764, 0, 0, 0, 0, 8197), gyro=(-5., 0., 0.))
    assert status["orientation_valid"] and not status["gyro_rejected"]
    assert used[0] == -5.
    assert status["gyro_boundary_integrated_axes"] == ["x"]


@pytest.mark.parametrize("raw,accel,reason", [
    ((0, 0, 0, 32764, 0, 8197), (30., 0., G), "accel_boundary"),
    ((0, 0, 0, 0, -32764, 8197), (0., -30., G), "accel_boundary"),
    ((0, 0, 0, 0, 0, 0), (0., 0., 0.), "accel_zero_norm"),
    ((0, 0, 0, 0, 0, 0), (0., 0., 1e-8), "accel_zero_norm"),
    (RAW, (20., 0., G), "accel_jump"),
])
def test_rejected_accel_always_skips_never_holds_previous_gravity(raw, accel, reason):
    instance = guard()
    feed(instance)
    used_gyro, used_accel, status = feed(instance, raw=raw, gyro=(.1, .2, .3), accel=accel)
    np.testing.assert_array_equal(used_gyro, (.1, .2, .3))
    np.testing.assert_array_equal(used_accel, np.zeros(3))
    assert status["accel_action"] == "skip_accel" and status["reasons"] == [reason]
    assert status["accel_rejected_samples"] == status["accel_run"] == 1


def test_accel_boundary_never_confirms_and_invalid_orientation_recovers():
    instance = guard(max_invalid_s=.02)
    feed(instance)
    for index in range(1, 8):
        _, used, status = feed(instance, raw=(0, 0, 0, 0, 0, 32764), accel=(0., 0., 30.))
        np.testing.assert_array_equal(used, np.zeros(3))
        assert status["accel_run"] == index and status["gyro_run"] == 0
        assert status["orientation_valid"] == (index <= 2)
    assert feed(instance)[2]["orientation_valid"]


def test_initial_invalid_input_cannot_report_valid_orientation():
    instance = guard()
    _, _, status = feed(instance, raw=(32764, 0, 0, 0, 0, 8197), gyro=(5., 0., 0.))
    assert status["gyro_action"] == "zero_motion" and not status["orientation_valid"]
    assert feed(instance)[2]["orientation_valid"]
    instance.reset()
    assert not feed(instance, accel=(0., 0., 0.))[2]["orientation_valid"]


def test_nodes_vectors_and_reset_are_independent():
    instance = guard()
    feed(instance, node=1)
    feed(instance, node=2, gyro=(5., 0., 0.))
    assert feed(instance, node=1, gyro=(5., 0., 0.))[2]["gyro_rejected"]
    assert not feed(instance, node=2, gyro=(5., 0., 0.))[2]["gyro_rejected"]
    instance.reset("1")
    status1 = feed(instance, node=1, gyro=(5., 0., 0.))[2]
    status2 = feed(instance, node=2, gyro=(5., 0., 0.))[2]
    assert status1["raw_samples"] == 1 and not status1["gyro_rejected"]
    assert status2["raw_samples"] == 3
    instance.reset()
    assert feed(instance, node=2)[2]["raw_samples"] == 1


@pytest.mark.parametrize("config", [{}, {"input_filter": {"enabled": False}}])
def test_disabled_is_exact_passthrough_even_for_extremes_zero_and_nonfinite(config):
    instance = ImuInputGuard(config)
    for gyro, accel in [((1e10, -1e10, .5), (1e10, 0., 0.)),
                        ((-1e10, 1e10, -.5), (0., 0., 0.)),
                        ((math.nan, math.inf, -math.inf), (0., 0., 0.))]:
        used_gyro, used_accel, status = feed(instance, raw=(32764,) * 6, gyro=gyro, accel=accel)
        np.testing.assert_array_equal(used_gyro, gyro)
        np.testing.assert_array_equal(used_accel, accel)
        assert not status["enabled"] and status["orientation_valid"]
        assert status["gyro_action"] == status["accel_action"] == "pass"
        assert status["rejected_samples"] == 0


def test_returned_arrays_and_status_do_not_alias_guard_state_or_caller():
    instance = guard()
    gyro, accel = np.array([.1, .2, .3]), np.array([0., 0., G])
    used_gyro, used_accel, status = feed(instance, gyro=gyro, accel=accel)
    snapshot = deepcopy(status)
    used_gyro[:] = used_accel[:] = gyro[:] = accel[:] = 99.
    held, _, following = feed(instance, gyro=(5., 0., 0.))
    np.testing.assert_array_equal(held, (.1, 0., 0.))  # Only X is held; Y/Z retain their new input.
    assert status == snapshot
    assert json.loads(json.dumps(following, allow_nan=False)) == following
    assert not any(isinstance(value, np.ndarray) for value in following.values())


def test_continuous_boundary_preserves_other_axes_and_their_integrated_angle():
    instance = guard()
    feed(instance)
    inputs, outputs = [], []
    for index in range(12):  # A 120 ms boundary burst at this test's 100 Hz.
        incoming = np.array([5., .3 + .01 * index, -.4 - .02 * index])
        used, _, status = feed(instance, raw=(32764, 0, 0, 0, 0, 8197), gyro=incoming)
        inputs.append(incoming)
        outputs.append(used)
        assert status["gyro_rejected_axes"] == (["x"] if index == 0 else [])
        assert status["gyro_action_by_axis"]["y"] == status["gyro_action_by_axis"]["z"] == "pass"
    input_angle = np.sum(inputs, axis=0) * instance.sample_period
    output_angle = np.sum(outputs, axis=0) * instance.sample_period
    np.testing.assert_array_equal(output_angle[1:], input_angle[1:])
    assert np.all(np.abs(output_angle[1:]) > 0.)
    assert output_angle[0] == pytest.approx(11 * 5. * instance.sample_period)
    assert status["gyro_boundary_integrated_axes"] == ["x"]
    assert status["gyro_boundary_samples_by_axis"]["x"] == 12
    assert status["gyro_zero_motion_samples_by_axis"]["x"] == 0
    assert status["boundary_is_physical_saturation_proof"] is False


def test_axis_candidate_confirmation_is_independent_of_another_axis_nonfinite_value():
    instance = guard()
    feed(instance)
    used, _, first = feed(instance, gyro=(5., 5., .2))
    np.testing.assert_array_equal(used, (0., 0., .2))
    assert first["gyro_rejected_axes"] == ["x", "y"]
    used, _, second = feed(instance, gyro=(math.nan, 5.1, .3))
    np.testing.assert_array_equal(used, (0., 5.1, .3))
    assert second["gyro_action_by_axis"] == {"x": "held", "y": "pass", "z": "pass"}
    # X's hard-invalid sample cleared its candidate, but not Y's independent
    # confirmation. X must supply two new consistent finite samples.
    used, _, third = feed(instance, gyro=(5.2, 5.2, .4))
    np.testing.assert_array_equal(used, (0., 5.2, .4))
    assert third["gyro_action_by_axis"] == {"x": "zero_motion", "y": "pass", "z": "pass"}
    used, _, fourth = feed(instance, gyro=(5.3, 5.3, .5))
    np.testing.assert_array_equal(used, (5.3, 5.3, .5))
    assert fourth["gyro_action"] == "pass" and fourth["gyro_run"] == 0


def test_mixed_axis_fallbacks_and_reasons_keep_aggregate_frame_counters():
    instance = guard()
    feed(instance)
    for _ in range(2):
        feed(instance, gyro=(math.nan, .1, .2))
    used, _, status = feed(instance, gyro=(math.nan, math.nan, 5.), bias=(.03, -.04, .01))
    np.testing.assert_array_equal(used, (.03, .1, .2))
    assert status["gyro_action"] == "zero_motion"
    assert status["gyro_action_by_axis"] == {"x": "zero_motion", "y": "held", "z": "held"}
    assert status["gyro_reasons_by_axis"] == {"x": "gyro_nonfinite", "y": "gyro_nonfinite", "z": "gyro_jump"}
    assert status["reasons"] == ["gyro_nonfinite"]
    assert status["gyro_run"] == status["gyro_rejected_samples"] == status["rejected_samples"] == 3
    assert status["gyro_run_by_axis"] == {"x": 3, "y": 1, "z": 1}
    assert status["gyro_rejected_samples_by_axis"] == {"x": 3, "y": 1, "z": 1}


def test_alternating_rejected_axes_keep_legacy_continuous_invalid_timeout():
    instance = guard(max_invalid_s=.02)
    feed(instance)
    for count, axis in enumerate((0, 1, 2, 0), 1):
        raw = list(RAW)
        raw[axis] = 32764
        incoming = [.1 * count, -.1 * count, .05 * count]
        incoming[axis] = 5.
        used, _, status = feed(instance, raw=raw, gyro=incoming)
        assert status["gyro_action"] == "held"  # Each axis is in its own first rejected sample.
        assert status["gyro_run"] == status["gyro_rejected_samples"] == count
        assert status["gyro_run_by_axis"] == {name: int(i == axis) for i, name in enumerate("xyz")}
        assert status["orientation_valid"] == (count <= 2)
        for i in range(3):
            if i != axis:
                assert used[i] == incoming[i]
    assert feed(instance)[2]["orientation_valid"]


def test_initial_missing_gyro_axis_remains_invalid_while_other_axes_pass():
    instance = guard()
    for count in range(3):
        incoming = (math.nan, .1 + count * .1, -.1 - count * .1)
        used, _, status = feed(instance, gyro=incoming)
        np.testing.assert_array_equal(used, (0., incoming[1], incoming[2]))
        assert not status["orientation_valid"]
        assert status["gyro_action_by_axis"] == {"x": "zero_motion", "y": "pass", "z": "pass"}
    assert feed(instance)[2]["orientation_valid"]


def test_axis_replacement_counters_survive_good_input_are_detached_and_reset_per_node():
    instance = guard()
    for node in (1, 2):
        feed(instance, node=node)
        for _ in range(4):
            feed(instance, node=node, gyro=(math.nan, .2, .3))
    _, _, status = feed(instance)
    assert status["gyro_action"] == "pass" and status["gyro_rejected_axes"] == []
    assert status["gyro_run_by_axis"] == {"x": 0, "y": 0, "z": 0}
    assert status["gyro_rejected_samples_by_axis"] == {"x": 4, "y": 0, "z": 0}
    assert status["gyro_held_samples_by_axis"] == {"x": 2, "y": 0, "z": 0}
    assert status["gyro_zero_motion_samples_by_axis"] == {"x": 2, "y": 0, "z": 0}
    for name in ("gyro_held_samples_by_axis", "gyro_zero_motion_samples_by_axis", "gyro_rejected_samples_by_axis"):
        status[name]["x"] = 99
    following = feed(instance)[2]
    assert following["gyro_held_samples_by_axis"]["x"] == following["gyro_zero_motion_samples_by_axis"]["x"] == 2
    assert following["gyro_rejected_samples_by_axis"]["x"] == 4
    instance.reset(1)
    reset, unchanged = feed(instance)[2], feed(instance, node=2)[2]
    for name in ("gyro_held_samples_by_axis", "gyro_zero_motion_samples_by_axis", "gyro_rejected_samples_by_axis"):
        assert reset[name] == {"x": 0, "y": 0, "z": 0}
        assert unchanged[name]["x"] > 0
    instance.reset()
    assert feed(instance, node=2)[2]["gyro_zero_motion_samples_by_axis"] == {"x": 0, "y": 0, "z": 0}


def test_nonfinite_axis_and_nonfinite_bias_do_not_replace_finite_other_axes():
    instance = guard(max_hold_samples=0)
    feed(instance)
    used, _, status = feed(instance, gyro=(math.nan, .25, -.4), bias=(.03, math.nan, math.inf))
    np.testing.assert_array_equal(used, (.03, .25, -.4))
    assert status["gyro_rejected_axes"] == ["x"]
    used, _, _ = feed(instance, gyro=(math.inf, .3, -.5), bias=(math.nan, 0., 0.))
    np.testing.assert_array_equal(used, (0., .3, -.5))


def test_nonfinite_vectors_cannot_poison_accepted_baseline():
    instance = guard()
    feed(instance)
    used_gyro, used_accel, status = feed(instance, gyro=(math.nan, 0., 0.), accel=(0., math.inf, G))
    np.testing.assert_array_equal(used_gyro, np.zeros(3))
    np.testing.assert_array_equal(used_accel, np.zeros(3))
    assert status["reasons"] == ["gyro_nonfinite", "accel_nonfinite"]
    assert feed(instance)[2]["reasons"] == []


@pytest.mark.parametrize("options", [
    None, [], {"typo": True}, {"enabled": 1},
    {"boundary_lsb": True}, {"boundary_lsb": 32760.0}, {"boundary_lsb": 0},
    {"boundary_lsb": 32769}, {"gyro_jump_dps": 0}, {"gyro_jump_dps": math.nan},
    {"gyro_jump_dps": True}, {"accel_jump_g": -1}, {"accel_jump_g": math.inf},
    {"confirmation_samples": 1}, {"confirmation_samples": 3}, {"confirmation_samples": 2.0},
    {"max_hold_samples": -1}, {"max_hold_samples": 2.0}, {"max_hold_samples": True},
    {"max_invalid_s": 0}, {"max_invalid_s": math.inf}, {"max_invalid_s": "0.25"},
])
def test_bad_configuration_is_rejected(options):
    with pytest.raises(ValueError, match="input_filter"):
        guard_settings({"input_filter": options})


@pytest.mark.parametrize("rate", [0., -1., math.inf, math.nan, True, "120"])
def test_invalid_sample_rate_is_rejected(rate):
    with pytest.raises(ValueError, match="sample_rate_hz"):
        ImuInputGuard(dict(sample_rate_hz=rate, input_filter={}))
