"""Pure branch-continuity tests; no devices, fitting, smoothing or anatomy."""
from dataclasses import FrozenInstanceError
import math

import pytest

from vt_fsglove12.hinge_continuity import HingeAngleContinuity


def update_degrees(tracker, degrees, timestamp, *, slot=8, token="session/profile/epoch"):
    return tracker.update(slot, math.radians(degrees), timestamp, token)


def test_positive_branch_crossing_does_not_zero_coupled_dip():
    tracker = HingeAngleContinuity()
    first = update_degrees(tracker, 179.99, 1.)
    second = update_degrees(tracker, -179.99, 1.05)
    assert first.angle_rad == math.radians(179.99)
    assert math.degrees(second.angle_rad) == pytest.approx(180.01)
    assert second.branch_corrected and second.branch_turns == 1
    assert second.reset_reason is None
    assert min(1.65, .65 * max(0., second.angle_rad)) == 1.65


def test_reverse_crossing_preserves_negative_angle_without_autozero():
    tracker = HingeAngleContinuity()
    first = update_degrees(tracker, -179.99, 1.)
    second = update_degrees(tracker, 179.99, 1.05)
    assert first.angle_rad == math.radians(-179.99)
    assert not first.branch_corrected
    assert math.degrees(second.angle_rad) == pytest.approx(-180.01)
    assert second.branch_turns == -1


def test_multiple_turns_and_branch_diagnostics_persist_without_smoothing():
    tracker = HingeAngleContinuity()
    raw = [0., 90., 179., -179., -90., 0., 90., 179., -179.]
    expected = [0., 90., 179., 181., 270., 360., 450., 539., 541.]
    results = [update_degrees(tracker, value, i * .05) for i, value in enumerate(raw)]
    assert [math.degrees(result.angle_rad) for result in results] == pytest.approx(expected)
    assert results[-1].branch_turns == 2
    assert results[-2].branch_corrected


def test_no_branch_means_no_gain_smoothing_limits_or_negative_angle_change():
    tracker = HingeAngleContinuity()
    for i, angle in enumerate([-.4, -.39, -.6, 0., .8, .01]):
        result = tracker.update(8, angle, i * .05, "binding")
        assert result.angle_rad == angle
        assert result.branch_turns == 0
        assert not result.branch_corrected


def test_duplicate_binding_timestamp_and_raw_angle_returns_exact_cached_result():
    tracker = HingeAngleContinuity()
    update_degrees(tracker, 179., 1.)
    result = update_degrees(tracker, -179., 1.1)
    assert update_degrees(tracker, -179., 1.1) is result
    assert math.degrees(update_degrees(tracker, -178., 1.2).angle_rad) == pytest.approx(182.)
    with pytest.raises(FrozenInstanceError):
        result.angle_rad = 0.


def test_changed_raw_at_same_timestamp_updates_without_reset_or_freezing():
    tracker = HingeAngleContinuity()
    update_degrees(tracker, 179., 1.)
    previous = update_degrees(tracker, -179., 1.1)
    revised = update_degrees(tracker, -177., 1.1)
    assert revised is not previous
    assert math.degrees(revised.angle_rad) == pytest.approx(183.)
    assert revised.branch_turns == 1
    assert revised.reset_reason is None
    assert update_degrees(tracker, -177., 1.1) is revised
    # Even a materially different observation is not held or smoothed.
    changed = update_degrees(tracker, 25., 1.1)
    assert changed.angle_rad == math.radians(25.)
    assert changed.branch_turns == 0
    assert changed.reset_reason is None
    assert math.degrees(update_degrees(tracker, -178., 1.2).angle_rad) == pytest.approx(182.)


@pytest.mark.parametrize("timestamp,token,reason", [
    (1.1, "different", "binding_changed"),
    (.9, "session/profile/epoch", "time_reversed"),
    (1.351, "session/profile/epoch", "gap_exceeded"),
])
def test_discontinuity_starts_from_raw_without_carrying_branch(timestamp, token, reason):
    tracker = HingeAngleContinuity(max_gap_s=.25)
    update_degrees(tracker, 179., 1.)
    update_degrees(tracker, -179., 1.1)
    result = update_degrees(tracker, -178., timestamp, token=token)
    assert result.angle_rad == math.radians(-178.)
    assert result.reset_reason == reason
    assert not result.branch_corrected


def test_gap_at_threshold_is_continuous():
    tracker = HingeAngleContinuity(max_gap_s=.25)
    update_degrees(tracker, 179., 1.)
    result = update_degrees(tracker, -179., 1.25)
    assert math.degrees(result.angle_rad) == pytest.approx(181.)
    assert result.reset_reason is None


def test_slots_are_independent_and_reset_can_target_one_or_all():
    tracker = HingeAngleContinuity()
    for slot in (2, 8):
        update_degrees(tracker, 179., 1., slot=slot)
        update_degrees(tracker, -179., 1.1, slot=slot)
    tracker.reset(8)
    assert update_degrees(tracker, -178., 1.2, slot=8).reset_reason == "initialized"
    assert math.degrees(update_degrees(tracker, -178., 1.2, slot=2).angle_rad) == pytest.approx(182.)
    tracker.reset()
    for slot in (2, 8):
        result = update_degrees(tracker, -177., 1.3, slot=slot)
        assert result.reset_reason == "initialized"
        assert not result.branch_corrected


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, True, "0", None, 10 ** 1000])
@pytest.mark.parametrize("field", ["raw_angle_rad", "timestamp_s"])
def test_invalid_numeric_inputs_do_not_change_existing_state(bad, field):
    tracker = HingeAngleContinuity()
    update_degrees(tracker, 179., 1.)
    args = dict(slot=8, raw_angle_rad=0., timestamp_s=1.1, binding_token="session/profile/epoch")
    args[field] = bad
    with pytest.raises(ValueError, match=field):
        tracker.update(**args)
    assert math.degrees(update_degrees(tracker, -179., 1.2).angle_rad) == pytest.approx(181.)


@pytest.mark.parametrize("bad", [0., -.1, math.nan, math.inf, True, "0.25"])
def test_gap_must_be_finite_positive_number(bad):
    with pytest.raises(ValueError, match="max_gap_s"):
        HingeAngleContinuity(max_gap_s=bad)


@pytest.mark.parametrize("slot", [-1, 16, True, 8., "8"])
def test_slot_bound_keeps_state_storage_finite(slot):
    tracker = HingeAngleContinuity()
    with pytest.raises(ValueError, match="slot"):
        tracker.update(slot, 0., 1., "binding")
    with pytest.raises(ValueError, match="slot"):
        tracker.reset(slot)


def test_binding_is_hashable_and_tuple_token_is_supported():
    tracker = HingeAngleContinuity()
    with pytest.raises(ValueError, match="binding_token"):
        tracker.update(8, 0., 1., {"session": "a"})
    assert tracker.update(8, -.2, 1., ("session", 4, 0)).angle_rad == -.2


def test_exact_half_turn_does_not_claim_a_branch_change():
    tracker = HingeAngleContinuity()
    tracker.update(8, 0., 1., "binding")
    assert not tracker.update(8, math.pi, 1.1, "binding").branch_corrected
