"""Bounded, passive diagnostics for decoded gyro integers near int16 bounds.

An encoding-boundary event is not a verified physical saturation measurement.
The legacy ``gyro_near_rail`` field and warning IDs remain readable, but their
meaning is only a numeric observation, independent of configured sensitivity.
No values, units, filter parameters or sensor commands are modified here.
"""
from collections import deque
import math
from numbers import Integral, Real


GYRO_NEAR_RAIL_THRESHOLD_LSB = 32760
GYRO_NEAR_RAIL_WINDOW_S = 2.0
MAX_RECENT_GROUPS_PER_NODE = 4096
_AXES = ("x", "y", "z")


def _time(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value < 0:
        raise ValueError("quality timestamp must be finite and nonnegative")
    return float(value)


class GyroNearRailMonitor:
    """Lifetime counters and an exact recent window under normal input rates.

    Equal host timestamps are grouped without deduplicating any samples. At
    most 4096 near-rail timestamp groups per node are retained; if an unusual
    burst exceeds this cap, recent counts explicitly become lower bounds until
    the lost groups expire. Lifetime counters remain exact. Snapshots are pure
    observations, so rendering twice cannot consume a warning or an event.
    The last boundary event retains its exact decoded integers and node-local,
    one-based input sample index even after the recent warning expires. Input
    clock rollback clears this evidence along with the recent time domain.
    """
    def __init__(self):
        self._nodes = {}
        self._last_input = None
        self._clock_resets = 0

    def observe(self, node_id, gyro_lsb, host_monotonic_s):
        if isinstance(node_id, str):
            if node_id not in {str(n) for n in range(1, 13)}:
                raise ValueError("quality node ID must be in 1..12")
            key = node_id
        elif not isinstance(node_id, bool) and isinstance(node_id, Integral) and 1 <= node_id <= 12:
            key = str(int(node_id))
        else:
            raise ValueError("quality node ID must be in 1..12")
        values = tuple(gyro_lsb)
        if len(values) != 3 or any(isinstance(v, bool) or not isinstance(v, Integral)
                                   or not -32768 <= v <= 32767 for v in values):
            raise ValueError("quality gyro values must contain three int16 integers")
        stamp = _time(host_monotonic_s)
        if self._last_input is not None and stamp < self._last_input:
            self._clock_resets += 1
            for state in self._nodes.values():
                state["recent"].clear()
                state["last_dropped_stamp"] = None
                state["last_boundary_event"] = None
        self._last_input = stamp
        state = self._nodes.setdefault(key, dict(total_samples=0, near_rail_samples=0,
            total_by_axis=[0, 0, 0], recent=deque(), last_host_monotonic_s=None,
            last_dropped_stamp=None, recent_overflow_samples_total=0,
            last_boundary_event=None))
        state["total_samples"] += 1
        recent = state["recent"]
        while recent and recent[0][0] < stamp - GYRO_NEAR_RAIL_WINDOW_S:
            recent.popleft()
        # Convert before abs: abs(np.int16(-32768)) would overflow.
        axes = [int(abs(int(value)) >= GYRO_NEAR_RAIL_THRESHOLD_LSB) for value in values]
        if not any(axes):
            return
        state["near_rail_samples"] += 1
        state["last_host_monotonic_s"] = stamp
        state["last_boundary_event"] = dict(sample_index=state["total_samples"],
            host_monotonic_s=stamp, gyro_lsb=[int(value) for value in values],
            axes=[axis for axis, hit in zip(_AXES, axes) if hit])
        for axis in range(3):
            state["total_by_axis"][axis] += axes[axis]
        if recent and recent[-1][0] == stamp:
            recent[-1][1] += 1
            for axis in range(3):
                recent[-1][axis + 2] += axes[axis]
        else:
            if len(recent) >= MAX_RECENT_GROUPS_PER_NODE:
                dropped = recent.popleft()
                state["last_dropped_stamp"] = dropped[0]
                state["recent_overflow_samples_total"] += dropped[1]
            recent.append([stamp, 1, *axes])

    def snapshot(self, now):
        stamp = _time(now)
        nodes, active = {}, []
        for key in sorted(self._nodes, key=int):
            state = self._nodes[key]
            count, axes = 0, [0, 0, 0]
            for time, samples, x, y, z in state["recent"]:
                if 0 <= stamp - time <= GYRO_NEAR_RAIL_WINDOW_S:
                    count += samples
                    for axis, value in enumerate((x, y, z)):
                        axes[axis] += value
            dropped = state["last_dropped_stamp"]
            complete = dropped is None or not 0 <= stamp - dropped <= GYRO_NEAR_RAIL_WINDOW_S
            if count or not complete:
                active.append(key)
            event = state["last_boundary_event"]
            last_boundary_event = None
            if event is not None:
                elapsed = stamp - event["host_monotonic_s"]
                last_boundary_event = dict(sample_index=event["sample_index"],
                    host_monotonic_s=event["host_monotonic_s"],
                    gyro_lsb=list(event["gyro_lsb"]), axes=list(event["axes"]),
                    age_s=elapsed if elapsed >= 0 else None,
                    in_recent_window=0 <= elapsed <= GYRO_NEAR_RAIL_WINDOW_S)
            nodes[key] = dict(total_samples=state["total_samples"],
                near_rail_samples=state["near_rail_samples"], recent_near_rail_samples=count,
                axes=[axis for axis, value in zip(_AXES, axes) if value],
                total_by_axis=dict(zip(_AXES, state["total_by_axis"])),
                recent_by_axis=dict(zip(_AXES, axes)),
                last_host_monotonic_s=state["last_host_monotonic_s"],
                last_boundary_event=last_boundary_event,
                recent_counts_complete=complete,
                recent_overflow_samples_total=state["recent_overflow_samples_total"])
        return dict(gyro_near_rail=dict(threshold_lsb=GYRO_NEAR_RAIL_THRESHOLD_LSB,
            recent_window_s=GYRO_NEAR_RAIL_WINDOW_S, recent_groups_limit_per_node=MAX_RECENT_GROUPS_PER_NODE,
            nodes=nodes, active_nodes=active, clock_resets=self._clock_resets,
            interpretation="encoding_boundary_event_not_physical_saturation",
            observation_domain="decoded_signed_int16_gyro_fields",
            physical_saturation="unknown",
            detection_rule="any_gyro_axis_absolute_integer_gte_threshold",
            sample_count_unit="node_samples_with_at_least_one_boundary_axis",
            event_alone_proves_continuous_plateau=False,
            physical_range_verified=False,
            raw_values_modified=False, filter_state_modified=False))


def quality_warnings(quality):
    """Legacy event IDs, not saturation verdicts; counters never latch alerts."""
    return [f"node_{node}_gyro_near_rail" for node in quality["gyro_near_rail"]["active_nodes"]]
