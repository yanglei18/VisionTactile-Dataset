"""Trial-long bounded retention, without hardware or model resources."""
import pytest

from vt_fsglove12.live_stream import RawSample
from vt_fsglove12.scale_capture_buffer import (
    DEFAULT_MAX_SAMPLES, ScaleCaptureBuffer, ScaleHistoryEvictedError,
)


def sample(stamp, node=12):
    return RawSample(node, (32767, 0, 0, 0, 0, 8192), float(stamp), float(stamp + 100))


def test_slow_interactive_holds_and_all_transitions_remain_without_time_pruning():
    first_hold = [sample(3), sample(4), sample(5)]
    rotations = [sample(35), sample(95), sample(145)]
    second_hold = [sample(153), sample(154), sample(155)]
    last_rotation = [sample(195)]
    third_hold = [sample(253), sample(254), sample(255)]
    data = first_hold + rotations + second_hold + last_rotation + third_hold
    buffer = ScaleCaptureBuffer()
    for group in (first_hold, rotations, second_hold, last_rotation, third_hold):
        buffer.extend(group)
    assert buffer.freeze() == tuple(data)
    assert all(actual is expected for actual, expected in zip(buffer, data))
    assert buffer[0] is first_hold[0] and buffer[-1] is third_hold[-1]
    assert buffer.maxlen == DEFAULT_MAX_SAMPLES
    assert not buffer.history_evicted
    assert buffer.snapshot()["status"] == "retained"


def test_budget_remains_bounded_and_loss_is_explicit_and_sticky():
    buffer = ScaleCaptureBuffer(maxlen=3)
    buffer.extend(sample(stamp) for stamp in range(10))
    assert len(buffer) == 3
    assert [row.host_monotonic_s for row in buffer] == [7., 8., 9.]
    assert buffer.snapshot() == dict(status="history_evicted", sample_count=3,
        max_samples=3, history_evicted=True, evicted_samples=7, evicted_through_s=6.)
    with pytest.raises(ScaleHistoryEvictedError, match="history_evicted"):
        buffer.freeze()
    buffer.append(sample(1000))
    with pytest.raises(ScaleHistoryEvictedError, match="new three-hold trial"):
        buffer.freeze()


def test_legacy_pruning_is_detected_and_clear_starts_a_new_trial():
    data = [sample(1), sample(2)]
    buffer = ScaleCaptureBuffer(data, maxlen=3)
    assert buffer.popleft() is data[0]
    assert buffer.history_evicted
    with pytest.raises(ScaleHistoryEvictedError):
        buffer.freeze()
    buffer.clear()
    assert not buffer and buffer.evicted_samples == 0
    assert buffer.evicted_through_s is None
    buffer.append(data[1])
    assert buffer.freeze() == (data[1],)


def test_frozen_prefix_is_independent_of_later_reader_appends_and_clear():
    first = sample(1)
    buffer = ScaleCaptureBuffer([first], maxlen=2)
    prefix = buffer.freeze()
    buffer.extend([sample(2), sample(3)])
    assert buffer.history_evicted
    buffer.clear()
    assert prefix == (first,) and prefix[0] is first


@pytest.mark.parametrize("budget", [0, -1, True, 3., "3", None, DEFAULT_MAX_SAMPLES + 1])
def test_invalid_or_unbounded_budgets_are_rejected(budget):
    with pytest.raises(ValueError, match="maxlen"):
        ScaleCaptureBuffer(maxlen=budget)
