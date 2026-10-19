"""Explicit local monotonic time for receipt freshness and interval measurement.

Never subtract ROS header stamps, device clocks or wall time from these stamps.
CLOCK_MONOTONIC and CLOCK_MONOTONIC_RAW are distinct domains. An origin_id
identifies one observer lifetime; comparing across restarts/processes requires an
explicitly shared origin and identical clock domain, not a guessed conversion.
Neither domain includes suspended time on Linux. Freshness is receipt freshness,
not sensor acquisition latency. Wire age_s values are already computed durations.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable
from uuid import uuid4


MONOTONIC = "CLOCK_MONOTONIC"
MONOTONIC_RAW = "CLOCK_MONOTONIC_RAW"
MAX_MONOTONIC_NS = (1 << 64) - 1


@dataclass(frozen=True)
class MonotonicStamp:
    ns: int
    domain: str
    origin_id: str

    def __post_init__(self):
        if type(self.ns) is not int or not 0 <= self.ns <= MAX_MONOTONIC_NS:
            raise ValueError("monotonic ns must be an unsigned 64-bit integer")
        if self.domain not in (MONOTONIC, MONOTONIC_RAW):
            raise ValueError("unsupported monotonic clock domain")
        if type(self.origin_id) is not str or not self.origin_id.strip():
            raise ValueError("monotonic origin_id must be a nonempty string")


class MonotonicClock:
    def __init__(self, *, domain: str = MONOTONIC, origin_id: str | None = None,
                 clock_ns: Callable[[], int] | None = None):
        self.domain = domain
        self.origin_id = str(uuid4()) if origin_id is None else origin_id
        MonotonicStamp(0, domain, self.origin_id)
        if clock_ns is not None:
            if not callable(clock_ns):
                raise ValueError("clock_ns must be callable")
            self._clock_ns = clock_ns
        elif domain == MONOTONIC:
            self._clock_ns = time.monotonic_ns
        else:
            if not hasattr(time, "CLOCK_MONOTONIC_RAW"):
                raise ValueError("CLOCK_MONOTONIC_RAW is unavailable on this platform")
            self._clock_ns = lambda: time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)

    def now(self) -> MonotonicStamp:
        return MonotonicStamp(self._clock_ns(), self.domain, self.origin_id)


def elapsed_seconds(start: MonotonicStamp, end: MonotonicStamp) -> float:
    """Compute an interval only for matching clock domains and observer origins."""
    if not isinstance(start, MonotonicStamp) or not isinstance(end, MonotonicStamp):
        raise ValueError("both timestamps must be MonotonicStamp instances")
    if (start.domain, start.origin_id) != (end.domain, end.origin_id):
        raise ValueError("cannot mix clock domains or observer origins")
    if end.ns < start.ns:
        raise ValueError("monotonic interval cannot run backwards")
    return (end.ns - start.ns) / 1_000_000_000


def age_seconds(received: MonotonicStamp | None, now: MonotonicStamp) -> float | None:
    """An unobserved device has unknown age (None), not an invented zero."""
    return None if received is None else elapsed_seconds(received, now)
