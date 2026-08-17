from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass

from .model import StampedTrackerSample
from .roles import RoleMap


DISCONNECTED = 0
CONNECTED_NO_TRACKING = 1
TRACKING = 2
INVALID_DATA = 3


@dataclass(frozen=True)
class TrackerHealthSnapshot:
    role: str
    tracker_id: str
    state: int
    tracking_status: int
    valid_sample_count: int
    invalid_report_count: int
    dropped_queue_count: int
    last_report_monotonic_ns: int
    last_valid_pose_monotonic_ns: int


@dataclass
class _MutableHealth:
    role: str
    tracker_id: str
    state: int = DISCONNECTED
    tracking_status: int = 0
    valid_sample_count: int = 0
    invalid_report_count: int = 0
    dropped_queue_count: int = 0
    last_report_monotonic_ns: int = 0
    last_valid_pose_monotonic_ns: int = 0
    has_report: bool = False
    has_valid_pose: bool = False

    def snapshot(self) -> TrackerHealthSnapshot:
        return TrackerHealthSnapshot(
            role=self.role,
            tracker_id=self.tracker_id,
            state=self.state,
            tracking_status=self.tracking_status,
            valid_sample_count=self.valid_sample_count,
            invalid_report_count=self.invalid_report_count,
            dropped_queue_count=self.dropped_queue_count,
            last_report_monotonic_ns=self.last_report_monotonic_ns,
            last_valid_pose_monotonic_ns=self.last_valid_pose_monotonic_ns,
        )


class TrackerHealthBook:
    def __init__(
        self,
        role_map: RoleMap,
        *,
        disconnect_timeout_ns: int = 1_000_000_000,
    ) -> None:
        if type(role_map) is not RoleMap:
            raise TypeError("role_map must be RoleMap")
        if (
            type(disconnect_timeout_ns) is not int
            or disconnect_timeout_ns <= 0
        ):
            raise ValueError("disconnect_timeout_ns must be positive")
        self._disconnect_timeout_ns = disconnect_timeout_ns
        self._values = {
            role: _MutableHealth(
                role=role,
                tracker_id=role_map.tracker_id_for_role(role),
            )
            for role in sorted(role_map.by_role)
        }
        self._lock = threading.Lock()
        self._status_publish_requested = False

    @staticmethod
    def _require_timestamp(value: object) -> None:
        if type(value) is not int:
            raise TypeError("monotonic timestamp must be int")
        if value < 0:
            raise ValueError("monotonic timestamp must not be negative")

    @staticmethod
    def _set_state(value: _MutableHealth, state: int) -> bool:
        changed = value.state != state
        value.state = state
        return changed

    def _value_for(self, role: str, tracker_id: str) -> _MutableHealth:
        try:
            value = self._values[role]
        except KeyError:
            raise ValueError("unknown tracker role") from None
        if value.tracker_id != tracker_id:
            raise ValueError("tracker identity does not match role")
        return value

    @staticmethod
    def _require_ordered_timestamp(
        value: _MutableHealth, monotonic_ns: int
    ) -> None:
        if value.has_report and monotonic_ns < value.last_report_monotonic_ns:
            raise ValueError("report monotonic timestamp moved backwards")

    def observe_sample(self, sample: StampedTrackerSample) -> bool:
        if type(sample) is not StampedTrackerSample:
            raise TypeError("sample must be StampedTrackerSample")
        self._require_timestamp(sample.host_monotonic_ns)
        with self._lock:
            value = self._value_for(sample.role, sample.tracker_id)
            self._require_ordered_timestamp(
                value, sample.host_monotonic_ns
            )
            state = TRACKING if sample.pose_valid else CONNECTED_NO_TRACKING
            changed = self._set_state(value, state)
            value.tracking_status = sample.tracking_status
            value.valid_sample_count += 1
            value.last_report_monotonic_ns = sample.host_monotonic_ns
            value.has_report = True
            if sample.pose_valid:
                value.last_valid_pose_monotonic_ns = (
                    sample.host_monotonic_ns
                )
                value.has_valid_pose = True
            self._status_publish_requested |= changed
            return changed

    def observe_invalid(
        self,
        role: str,
        tracker_id: str,
        monotonic_ns: int,
    ) -> bool:
        self._require_timestamp(monotonic_ns)
        with self._lock:
            value = self._value_for(role, tracker_id)
            self._require_ordered_timestamp(value, monotonic_ns)
            has_recent_valid_pose = (
                value.has_valid_pose
                and monotonic_ns - value.last_valid_pose_monotonic_ns
                < self._disconnect_timeout_ns
            )
            changed = (
                False
                if has_recent_valid_pose
                else self._set_state(value, INVALID_DATA)
            )
            value.invalid_report_count += 1
            value.last_report_monotonic_ns = monotonic_ns
            value.has_report = True
            self._status_publish_requested |= changed
            return changed

    def record_drop(self, role: str) -> None:
        with self._lock:
            try:
                value = self._values[role]
            except KeyError:
                raise ValueError("unknown tracker role") from None
            value.dropped_queue_count += 1

    def snapshot(self, now_ns: int) -> tuple[TrackerHealthSnapshot, ...]:
        self._require_timestamp(now_ns)
        with self._lock:
            for value in self._values.values():
                if (
                    value.has_report
                    and now_ns - value.last_report_monotonic_ns
                    >= self._disconnect_timeout_ns
                ):
                    changed = self._set_state(value, DISCONNECTED)
                    self._status_publish_requested |= changed
            return tuple(
                self._values[role].snapshot()
                for role in sorted(self._values)
            )

    def consume_status_publish_request(self) -> bool:
        with self._lock:
            requested = self._status_publish_requested
            self._status_publish_requested = False
            return requested


class BoundedSampleQueue:
    def __init__(
        self,
        health_book: TrackerHealthBook,
        *,
        capacity: int = 4096,
    ) -> None:
        if type(health_book) is not TrackerHealthBook:
            raise TypeError("health_book must be TrackerHealthBook")
        if type(capacity) is not int or capacity <= 0:
            raise ValueError("capacity must be positive")
        self._health_book = health_book
        self._capacity = capacity
        self._values: deque[StampedTrackerSample] = deque(maxlen=capacity)
        self._lock = threading.Lock()

    def put(self, sample: StampedTrackerSample) -> None:
        if type(sample) is not StampedTrackerSample:
            raise TypeError("sample must be StampedTrackerSample")
        with self._lock:
            if len(self._values) == self._capacity:
                dropped = self._values[0]
                self._health_book.record_drop(dropped.role)
            self._values.append(sample)

    def drain(self) -> tuple[StampedTrackerSample, ...]:
        with self._lock:
            values = tuple(self._values)
            self._values.clear()
            return values

    def __len__(self) -> int:
        with self._lock:
            return len(self._values)
