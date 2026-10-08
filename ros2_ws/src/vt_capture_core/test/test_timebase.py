import pytest

from vt_capture_core.timebase import (
    MONOTONIC, MONOTONIC_RAW, MonotonicClock, MonotonicStamp, age_seconds, elapsed_seconds,
)


def test_injected_receipt_clock_and_unknown_age():
    ticks = iter([1_000_000_000, 1_025_000_000])
    clock = MonotonicClock(origin_id="observer", clock_ns=lambda: next(ticks))
    start, end = clock.now(), clock.now()
    assert start.domain == MONOTONIC
    assert elapsed_seconds(start, end) == 0.025
    assert age_seconds(start, end) == 0.025
    assert age_seconds(None, end) is None


def test_default_clock_has_unique_lifetime():
    first, second = MonotonicClock(), MonotonicClock()
    assert first.origin_id != second.origin_id
    assert elapsed_seconds(first.now(), first.now()) >= 0


@pytest.mark.parametrize("start,end", [
    (MonotonicStamp(0, MONOTONIC, "one"), MonotonicStamp(1, MONOTONIC_RAW, "one")),
    (MonotonicStamp(0, MONOTONIC, "one"), MonotonicStamp(1, MONOTONIC, "two")),
    (MonotonicStamp(2, MONOTONIC, "one"), MonotonicStamp(1, MONOTONIC, "one")),
    (1.0, 2.0),
])
def test_cannot_mix_domains_lifetimes_or_go_backwards(start, end):
    with pytest.raises(ValueError):
        elapsed_seconds(start, end)


@pytest.mark.parametrize("ns,domain,origin", [
    (-1, MONOTONIC, "x"), (True, MONOTONIC, "x"),
    (10 ** 400, MONOTONIC, "x"),
    (1.5, MONOTONIC, "x"), (1, "ROS_TIME", "x"),
    (1, "WALL_TIME", "x"), (1, MONOTONIC, ""), (1, MONOTONIC, None),
])
def test_invalid_stamps(ns, domain, origin):
    with pytest.raises(ValueError):
        MonotonicStamp(ns, domain, origin)


def test_injected_raw_clock_retains_raw_identity():
    clock = MonotonicClock(domain=MONOTONIC_RAW, clock_ns=lambda: 100)
    assert clock.now().domain == MONOTONIC_RAW


def test_bad_injected_clock_is_rejected():
    with pytest.raises(ValueError):
        MonotonicClock(clock_ns=123)
    with pytest.raises(ValueError):
        MonotonicClock(clock_ns=lambda: -1).now()
