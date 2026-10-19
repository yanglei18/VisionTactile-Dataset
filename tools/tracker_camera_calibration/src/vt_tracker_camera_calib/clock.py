"""Clock continuity checks, performed before any realtime-based sorting.

Monotonic domains are stream-local: Tracker CLOCK_MONOTONIC is never compared
directly with camera CLOCK_MONOTONIC_RAW. This gate detects steps and resets;
it does not estimate exposure timing or certify cross-host synchronization.
"""
from __future__ import annotations

from collections.abc import Iterable


MAX_CLOCK_STEP_NS = 5_000_000


class HostClockContinuity:
    """Require increasing host clocks and at most 5 ms of step disagreement."""

    def __init__(self, stream_name: str) -> None:
        self.stream_name = stream_name
        self._previous: tuple[int, int] | None = None
        self._count = 0

    def observe(self, realtime_ns: int, monotonic_ns: int) -> None:
        if any(
            type(value) is not int or value <= 0
            for value in (realtime_ns, monotonic_ns)
        ):
            raise ValueError(f"{self.stream_name}: host clocks must be positive integers")
        if self._previous is not None:
            realtime_delta = realtime_ns - self._previous[0]
            monotonic_delta = monotonic_ns - self._previous[1]
            if realtime_delta <= 0 or monotonic_delta <= 0:
                raise ValueError(
                    f"{self.stream_name}: non-increasing host clock at sample {self._count}; "
                    "clock reset or reordered records cannot be calibrated"
                )
            if abs(realtime_delta - monotonic_delta) > MAX_CLOCK_STEP_NS:
                raise ValueError(
                    f"{self.stream_name}: host clock step exceeds 5 ms at sample {self._count}"
                )
        self._previous = realtime_ns, monotonic_ns
        self._count += 1


def validate_optional_clock_sequence(
    stream_name: str, values: Iterable[tuple[int, int | None]]
) -> None:
    """Validate clock metadata when supplied by callers outside the bag reader.

    Legacy and synthetic inputs may omit it for the entire stream. A partially
    populated stream cannot provide a continuity guarantee and is rejected.
    """
    clock = HostClockContinuity(stream_name)
    has_metadata: bool | None = None
    for realtime_ns, monotonic_ns in values:
        present = monotonic_ns is not None
        if has_metadata is not None and present != has_metadata:
            raise ValueError(f"{stream_name}: incomplete monotonic clock metadata")
        has_metadata = present
        if monotonic_ns is not None:
            clock.observe(realtime_ns, monotonic_ns)
