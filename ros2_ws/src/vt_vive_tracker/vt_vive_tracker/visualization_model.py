from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable


FIXED_ROLES = ("left_wrist", "right_wrist", "torso")
FRESH_NS = 250_000_000
OFFLINE_NS = 1_000_000_000
TRAIL_NS = 3_000_000_000
RATE_NS = 1_000_000_000


class VisualHealth(Enum):
    FRESH = "FRESH"
    DELAYED = "DELAYED"
    OFFLINE = "OFFLINE"


@dataclass(frozen=True)
class PoseValue:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class StatusValue:
    state: int
    valid_sample_count: int
    invalid_report_count: int
    dropped_queue_count: int
    tracker_id: str = ""
    tracking_status: int = 0


@dataclass(frozen=True)
class TimedPose:
    arrival_ns: int
    pose: PoseValue


@dataclass(frozen=True)
class RoleSnapshot:
    role: str
    health: VisualHealth
    pose: PoseValue | None
    trail: tuple[PoseValue, ...]
    receive_rate_hz: float
    status: StatusValue | None
    pose_age_ns: int | None = None


@dataclass
class _RoleState:
    trail: deque[TimedPose]
    rate_times: deque[int]
    last_pose: TimedPose | None = None
    status: StatusValue | None = None


def _is_exact_nonnegative_int(value: object) -> bool:
    return type(value) is int and value >= 0


def _valid_pose(value: object) -> bool:
    if type(value) is not PoseValue:
        return False
    if (
        type(value.position) is not tuple
        or len(value.position) != 3
        or type(value.orientation_xyzw) is not tuple
        or len(value.orientation_xyzw) != 4
    ):
        return False
    components = (*value.position, *value.orientation_xyzw)
    return all(
        type(component) is float and math.isfinite(component)
        for component in components
    )


class TrackerVisualizationModel:
    def __init__(
        self,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        *,
        max_trail_points: int = 1024,
    ) -> None:
        if not callable(monotonic_ns):
            raise TypeError("monotonic_ns must be callable")
        if (
            type(max_trail_points) is not int
            or max_trail_points <= 0
        ):
            raise ValueError("max_trail_points must be a positive int")
        self._monotonic_ns = monotonic_ns
        self._states = {
            role: _RoleState(
                trail=deque(maxlen=max_trail_points),
                rate_times=deque(maxlen=max_trail_points),
            )
            for role in FIXED_ROLES
        }

    def observe_pose(
        self,
        role: str,
        pose: PoseValue,
        *,
        arrival_ns: int,
    ) -> bool:
        state = self._states.get(role)
        if (
            state is None
            or not _valid_pose(pose)
            or not _is_exact_nonnegative_int(arrival_ns)
        ):
            return False
        if (
            state.last_pose is not None
            and arrival_ns < state.last_pose.arrival_ns
        ):
            return False
        timed_pose = TimedPose(arrival_ns, pose)
        state.last_pose = timed_pose
        state.trail.append(timed_pose)
        state.rate_times.append(arrival_ns)
        return True

    def observe_status(
        self,
        role: str,
        state: int,
        valid_sample_count: int,
        invalid_report_count: int,
        dropped_queue_count: int,
        *,
        tracker_id: str = "",
        tracking_status: int = 0,
    ) -> bool:
        role_state = self._states.get(role)
        values = (
            valid_sample_count,
            invalid_report_count,
            dropped_queue_count,
        )
        if (
            role_state is None
            or type(state) is not int
            or not 0 <= state <= 3
            or any(not _is_exact_nonnegative_int(value) for value in values)
            or type(tracker_id) is not str
            or type(tracking_status) is not int
            or not 0 <= tracking_status <= 255
        ):
            return False
        role_state.status = StatusValue(
            state,
            *values,
            tracker_id,
            tracking_status,
        )
        return True

    @staticmethod
    def _health(
        state: _RoleState, now_ns: int
    ) -> VisualHealth:
        if state.last_pose is None:
            return VisualHealth.OFFLINE
        age_ns = now_ns - state.last_pose.arrival_ns
        status_state = (
            None if state.status is None else state.status.state
        )
        if age_ns > OFFLINE_NS or status_state == 0:
            return VisualHealth.OFFLINE
        if age_ns > FRESH_NS or status_state in (None, 1, 3):
            return VisualHealth.DELAYED
        return VisualHealth.FRESH

    @staticmethod
    def _prune(state: _RoleState, now_ns: int) -> None:
        trail_cutoff = now_ns - TRAIL_NS
        while state.trail and state.trail[0].arrival_ns < trail_cutoff:
            state.trail.popleft()
        rate_cutoff = now_ns - RATE_NS
        while state.rate_times and state.rate_times[0] < rate_cutoff:
            state.rate_times.popleft()

    def snapshots(
        self, now_ns: int | None = None
    ) -> tuple[RoleSnapshot, ...]:
        if now_ns is None:
            now_ns = self._monotonic_ns()
        if not _is_exact_nonnegative_int(now_ns):
            raise ValueError("now_ns must be a non-negative int")
        values = []
        for role in FIXED_ROLES:
            state = self._states[role]
            self._prune(state, now_ns)
            values.append(
                RoleSnapshot(
                    role=role,
                    health=self._health(state, now_ns),
                    pose=(
                        None
                        if state.last_pose is None
                        else state.last_pose.pose
                    ),
                    trail=tuple(item.pose for item in state.trail),
                    receive_rate_hz=(
                        len(state.rate_times) * 1_000_000_000 / RATE_NS
                    ),
                    status=state.status,
                    pose_age_ns=(
                        None
                        if state.last_pose is None
                        else max(0, now_ns - state.last_pose.arrival_ns)
                    ),
                )
            )
        return tuple(values)
