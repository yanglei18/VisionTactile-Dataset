from __future__ import annotations

from dataclasses import dataclass

from .model import ClockObservation


@dataclass(frozen=True)
class ClockViolation:
    index: int
    reason: str
    realtime_delta_ns: int
    monotonic_delta_ns: int
    step_error_ns: int


@dataclass(frozen=True)
class ClockAudit:
    stream_name: str
    sample_count: int
    maximum_step_error_ns: int
    violations: tuple[ClockViolation, ...]

    @property
    def valid(self) -> bool:
        return not self.violations

    def as_document(self) -> dict[str, object]:
        return {
            "stream": self.stream_name,
            "sample_count": self.sample_count,
            "maximum_step_error_ns": self.maximum_step_error_ns,
            "valid": self.valid,
            "violations": [
                {
                    "index": value.index,
                    "reason": value.reason,
                    "realtime_delta_ns": value.realtime_delta_ns,
                    "monotonic_delta_ns": value.monotonic_delta_ns,
                    "step_error_ns": value.step_error_ns,
                }
                for value in self.violations
            ],
        }


def audit_clock_stream(
    stream_name: str,
    observations: tuple[ClockObservation, ...],
    *,
    max_step_ns: int,
) -> ClockAudit:
    if not stream_name:
        raise ValueError("stream_name must not be empty")
    if type(max_step_ns) is not int or max_step_ns < 0:
        raise ValueError("max_step_ns must be a non-negative integer")
    if any(not isinstance(value, ClockObservation) for value in observations):
        raise TypeError("observations must contain ClockObservation values")
    violations: list[ClockViolation] = []
    maximum = 0
    for index, (previous, current) in enumerate(
        zip(observations, observations[1:]), start=1
    ):
        realtime_delta = current.realtime_ns - previous.realtime_ns
        monotonic_delta = current.monotonic_ns - previous.monotonic_ns
        step_error = abs(realtime_delta - monotonic_delta)
        maximum = max(maximum, step_error)
        reason = ""
        if realtime_delta <= 0 or monotonic_delta <= 0:
            reason = "non_increasing"
        elif step_error > max_step_ns:
            reason = "clock_step"
        if reason:
            violations.append(
                ClockViolation(
                    index=index,
                    reason=reason,
                    realtime_delta_ns=realtime_delta,
                    monotonic_delta_ns=monotonic_delta,
                    step_error_ns=step_error,
                )
            )
    return ClockAudit(
        stream_name=stream_name,
        sample_count=len(observations),
        maximum_step_error_ns=maximum,
        violations=tuple(violations),
    )
