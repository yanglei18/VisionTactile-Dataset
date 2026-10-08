from __future__ import annotations

from dataclasses import dataclass
import math

from .model import PoseSample, ValidationThresholds


@dataclass(frozen=True)
class TrackerReport:
    tracker_id: str
    samples: int
    invalid_tracking_samples: int
    duration_s: float
    effective_hz: float
    max_gap_ms: float
    passed: bool


@dataclass(frozen=True)
class SessionReport:
    expected_trackers: int
    observed_trackers: int
    trackers: tuple[TrackerReport, ...]
    passed: bool


class ValidationSession:
    def __init__(self, thresholds: ValidationThresholds) -> None:
        self.thresholds = thresholds
        self._samples: dict[str, list[PoseSample]] = {}

    def add(self, sample: PoseSample) -> None:
        values = self._samples.setdefault(sample.tracker_id, [])
        if (
            values
            and sample.host_monotonic_ns < values[-1].host_monotonic_ns
        ):
            raise ValueError("host monotonic timestamps moved backwards")
        values.append(sample)

    def _report(
        self,
        tracker_id: str,
        values: list[PoseSample],
    ) -> TrackerReport:
        duration_s = (
            values[-1].host_monotonic_ns
            - values[0].host_monotonic_ns
        ) / 1e9
        effective_hz = (
            (len(values) - 1) / duration_s
            if duration_s > 0
            else 0.0
        )
        gaps_ms = [
            (right.host_monotonic_ns - left.host_monotonic_ns) / 1e6
            for left, right in zip(values, values[1:])
        ]
        max_gap_ms = max(gaps_ms, default=0.0)
        invalid_tracking_samples = sum(
            value.tracking_status
            != self.thresholds.full_tracking_status
            for value in values
        )
        norms = [
            math.sqrt(
                sum(part * part for part in value.quaternion_wxyz)
            )
            for value in values
        ]
        passed = (
            duration_s >= self.thresholds.duration_s
            and effective_hz >= self.thresholds.min_hz
            and max_gap_ms <= self.thresholds.max_gap_ms
            and invalid_tracking_samples == 0
            and min(norms) >= self.thresholds.quaternion_norm_min
            and max(norms) <= self.thresholds.quaternion_norm_max
        )
        return TrackerReport(
            tracker_id=tracker_id,
            samples=len(values),
            invalid_tracking_samples=invalid_tracking_samples,
            duration_s=duration_s,
            effective_hz=effective_hz,
            max_gap_ms=max_gap_ms,
            passed=passed,
        )

    def finish(self, expected_trackers: int) -> SessionReport:
        reports = tuple(
            self._report(tracker_id, values)
            for tracker_id, values in sorted(self._samples.items())
            if values
        )
        return SessionReport(
            expected_trackers=expected_trackers,
            observed_trackers=len(reports),
            trackers=reports,
            passed=(
                len(reports) == expected_trackers
                and all(report.passed for report in reports)
            ),
        )
