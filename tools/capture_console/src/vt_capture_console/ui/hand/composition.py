"""Independent live display of solved geometry and tactile responses.

Both sources retain their own identity and source clocks. A latest GUI poll is
not a new measurement, and the palm's receive time is only a display reference
for an asynchronous hand: this is not hardware synchronization or a solver.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, replace
from numbers import Integral

import numpy as np

from vt_capture_console.backend.tactile_source import TactileDisplayState, TactileTiming
from vt_fsglove12.health_contract import blocked_input_nodes
from vt_fsglove12.joint_quality import joint_quality_state
from vt_fsglove12.live_display import measured_pose_visible
from vt_tactile_capture import host_clock
from vt_tactile_capture.grid import CELL_ROW_INDICES
from vt_tactile_capture.hand_layout import load_layout


MAX_AGE_NS = 250_000_000
# Absolute display limit from the last measured palm sample, including its live time.
MAX_SHAPE_AGE_NS = 500_000_000
MAX_HISTORY = 1  # Only the latest tactile timing is retained to reject clock regressions.
_ROW_INDEX = np.asarray(CELL_ROW_INDICES, dtype=np.intp)
_POSE_CONTEXT_FIELDS = ("session_id", "profile_version", "config_sha256", "model_rest_sha256")


@dataclass(frozen=True)
class CombinedFrame:
    """Display state at one GUI tick; skew is the absolute difference of the two receive times."""

    geometry: dict | None
    tactile_frame: dict | None
    pose_current: bool
    mode: str
    status: str
    pose_age_ms: float | None = None
    tactile_age_ms: float | None = None
    skew_ms: float | None = None
    geometry_key: tuple | None = None

    @property
    def uses_tracked_geometry(self):
        return self.geometry is not None and (self.pose_current or self.mode == "held")


@dataclass(frozen=True)
class _MeasuredShape:
    geometry: dict
    key: tuple
    source_ns: int
    epochs: dict


@dataclass(frozen=True)
class _TactileSample:
    timing: TactileTiming
    frame: dict


def _clock(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError("A nonnegative monotonic timestamp is required")
    return int(value)


def _positive_ns(value):
    return not isinstance(value, bool) and isinstance(value, Integral) and value > 0


class DisplayComposer:
    """Compose two independently timed display layers without changing source data.

    ``compose`` receives the exact public results from the two controllers.
    It never reads an assembler or model's private cache. Hand geometry remains
    owned by the input snapshot; tactile outputs are private copies.
    """

    def __init__(self, neutral_geometry, *, hand="right", layout_path=None):
        self.hand = hand
        self.neutral_geometry = deepcopy(neutral_geometry)
        self._vertices = len(neutral_geometry["vertices_m"])
        self._faces = np.asarray(neutral_geometry["faces"], dtype=np.int64)
        if not self._geometry_valid(neutral_geometry):
            raise ValueError("The reference hand geometry is invalid")
        load_layout(layout_path, expected_hand=hand)
        self._tactile_timing = self._pose_context = None
        self._last_shape = None
        self._last_now = None

    @property
    def history_size(self):
        return int(self._tactile_timing is not None)

    def reset(self):
        self._tactile_timing = self._pose_context = self._last_now = None
        self._last_shape = None

    def _geometry_valid(self, geometry):
        if not isinstance(geometry, dict):
            return False
        provenance = geometry.get("provenance", {})
        if not isinstance(provenance, dict) or provenance.get("hand", "right") != self.hand:
            return False
        try:
            vertices = np.asarray(geometry.get("vertices_m"), dtype=float)
            joints = np.asarray(geometry.get("joints_m"), dtype=float)
            faces = np.asarray(geometry.get("faces"), dtype=np.int64)
            return (vertices.shape == (self._vertices, 3) and joints.shape == (21, 3)
                    and np.isfinite(vertices).all() and np.isfinite(joints).all()
                    and np.array_equal(faces, self._faces))
        except (TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _identity(timing):
        return (timing.session_id, timing.stream_epoch, timing.transport_reset_generation,
                timing.transport_health_generation, timing.display_generation)

    @staticmethod
    def _fresh(timing, now):
        return (0 <= now - timing.source_monotonic_ns <= MAX_AGE_NS
                and 0 <= now - timing.snapshot_monotonic_ns <= MAX_AGE_NS)

    def _current_touch(self, state, now):
        if (not isinstance(state, TactileDisplayState) or not isinstance(state.frame, dict)
                or not isinstance(state.timing, TactileTiming)
                or state.frame.get("hand", "right") != self.hand
                or state.frame.get("reference_ready") is not True
                or state.response is None or state.response.phase != "ready"):
            self._tactile_timing = None
            return None
        timing = state.timing
        previous = self._tactile_timing
        if previous is not None and self._identity(previous) == self._identity(timing):
            if (timing.source_sequence < previous.source_sequence
                    or timing.source_monotonic_ns < previous.source_monotonic_ns
                    or timing.snapshot_monotonic_ns < previous.snapshot_monotonic_ns
                    or (timing.source_sequence == previous.source_sequence and timing != previous)):
                return None
        if not self._fresh(timing, now):
            return None
        frame = state.frame
        try:
            values = np.asarray(frame["values"], dtype=float)
            valid = np.asarray(frame["valid_mask"])
            levels = np.asarray(frame["levels"], dtype=float)
            if (frame.get("value_kind") != "tactile_response" or values.shape != (400,)
                    or not np.isfinite(values).all() or np.any(values < 0)
                    or valid.shape != (400,) or valid.dtype.kind not in "biu"
                    or np.any((valid != 0) & (valid != 1)) or levels.shape != (2,)
                    or not np.isfinite(levels).all() or levels[0] != 0 or levels[1] <= 0):
                return None
            stamps = np.asarray(timing.row_monotonic_ns, dtype=np.uint64)[_ROW_INDEX]
            age_limit = min(MAX_AGE_NS, timing.max_age_ns)
            mask = (valid.astype(bool) & (stamps > 0) & (stamps <= timing.source_monotonic_ns)
                    & (stamps >= max(0, now - age_limit)))
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        self._tactile_timing = timing
        if not mask.any():
            return None
        result = deepcopy(frame)
        result["valid_mask"] = mask.tolist()
        return _TactileSample(timing, result)

    def _fixed_hand(self, reason=None, *, pose_age_ms=None):
        self._last_shape = None
        if reason is not None:
            return CombinedFrame(None, None, False, "waiting", reason, pose_age_ms)
        return CombinedFrame(self.neutral_geometry, None, False, "reference", "固定参考手形 · 等待连接")

    @staticmethod
    def _input_allows_hold(frame, mapped, epochs):
        health = (frame or {}).get("input_health") or {}
        interlocked = set(health.get("faulted_nodes", ())) | set(health.get("recovering_nodes", ()))
        for node in blocked_input_nodes(frame, mapped):
            status = health.get("nodes", {}).get(node, {})
            # A stale node cannot supply a measurement, but age alone does
            # not invalidate the last displayed geometry or its filter epoch.
            if (node in interlocked or status.get("state") != "stale" or status.get("faulted") is not False
                    or status.get("integration_allowed") is not True or status.get("reasons") != []
                    or status.get("age_s", 0.) < 0.):
                return False
        # Aging may strip a solver rejection. The independent input monitor
        # still exposes a changed filter epoch or an explicitly bad orientation.
        for node, data in (frame or {}).get("nodes", {}).items():
            if node in epochs and (data.get("epoch", epochs[node]) != epochs[node]
                                  or (data.get("orientation_valid") is False
                                      and data.get("continuity") != "awaiting_backlog")):
                return False
        return True

    def _timing_gap(self, snapshot, now):
        """Bridge only an explicit missing/stale palm and only in the same calibration."""
        shape = self._last_shape
        pose = snapshot["pose"]
        if (shape is None or now - shape.source_ns > MAX_SHAPE_AGE_NS
                or pose.get("reason") != "missing_or_stale_root" or pose.get("pose_available") is not False
                or pose.get("hand", self.hand) != self.hand or snapshot["runtime"].get("error")
                or snapshot.get("geometry_error") or (snapshot.get("calibration") or {}).get("active")):
            return self._fixed_hand("手势数据已过期")
        # A missing palm has no current source time. If provided, it must agree
        # with the source clock. Neither it nor a new solver tick extends the hold.
        source = pose.get("source_monotonic_ns")
        if source is not None and (not _positive_ns(source) or not shape.source_ns <= source <= now):
            return self._fixed_hand("手势数据已过期")
        orientation = snapshot.get("orientation")
        latest_input = (snapshot.get("input_snapshot") or {}).get("orientation")
        mapped = (snapshot.get("calibration") or {}).get("calibrated_nodes") or None
        rejections = pose.get("calibration_rejections") or {}
        if (rejections.get(pose.get("source_reference_node")) == "node_not_calibrated"
                or any(reason not in ("missing_or_stale_node", "waiting_for_backlog", "node_not_calibrated")
                       for reason in rejections.values())
                or not all(self._input_allows_hold(frame, mapped, shape.epochs)
                           for frame in (pose, orientation, latest_input))):
            return self._fixed_hand("手势输入暂不可用")
        return CombinedFrame(shape.geometry, None, False, "held", "保留上一帧手形 · 等待姿态更新",
                             (now - shape.source_ns) / 1e6, geometry_key=shape.key)

    def _remember_shape(self, snapshot, geometry, key, source):
        pose = snapshot["pose"]
        calibration = snapshot.get("calibration") or {}
        if (pose.get("hardware_accessed") is not True or calibration.get("active")
                or any(pose.get(field) is None for field in _POSE_CONTEXT_FIELDS)):
            self._last_shape = None
            return
        nodes = (snapshot.get("orientation") or {}).get("nodes", {})
        mapped = calibration.get("calibrated_nodes") or nodes
        epochs = {node: nodes[node]["epoch"] for node in mapped if node in nodes and "epoch" in nodes[node]}
        self._last_shape = _MeasuredShape(geometry, key, source, epochs)

    def _pose_geometry(self, snapshot, pose):
        displayed = snapshot.get("display_pose")
        geometry = snapshot.get("display_geometry")
        if (isinstance(displayed, dict)
                and displayed.get("hand", self.hand) == self.hand
                and all(displayed.get(key) == pose.get(key) for key in (*_POSE_CONTEXT_FIELDS, "frame_index"))
                and displayed.get("timestamp_s") == pose.get("timestamp_s")
                and displayed.get("shape_preview_mode", "current") == "current"
                and self._geometry_valid(geometry)):
            return geometry
        geometry = snapshot.get("geometry")
        if (all(joint_quality_state(pose)[1]) and self._geometry_valid(geometry)
                and not geometry.get("display_only")):
            return geometry
        return None

    def _display_snapshot(self, snapshot, now):
        """Select one complete rendered frame without changing measurements.

        Surface work may finish between numeric frames. Its own pose/source
        time drives display age and tactile skew; the latest pose still gates
        session health, calibration and sensor continuity.
        """
        preview = snapshot.get('surface_preview')
        latest = snapshot.get('pose') or {}
        if (not isinstance(preview, dict) or not measured_pose_visible(snapshot, now / 1e9)
                or not self._geometry_valid(preview.get('geometry'))):
            return snapshot
        pose, display = preview.get('measurement_pose') or {}, preview.get('pose') or {}
        calibration = snapshot.get('calibration') or {}
        binding = preview.get('binding') or ()
        if (not isinstance(pose, dict) or not isinstance(display, dict) or len(binding) != 11
                or any(pose.get(key) != latest.get(key) for key in _POSE_CONTEXT_FIELDS)
                or pose.get('hand', self.hand) != self.hand
                or pose.get('shape_params') != latest.get('shape_params')
                or binding[1] != calibration.get('clear_generation', binding[1])
                or binding[5] != calibration.get('functional_enabled', binding[5])
                or any(display.get(key) != pose.get(key)
                       for key in (*_POSE_CONTEXT_FIELDS, 'frame_index', 'timestamp_s', 'source_monotonic_ns'))):
            return snapshot
        old_nodes = (preview.get('orientation') or {}).get('nodes', {})
        nodes = (snapshot.get('orientation') or {}).get('nodes', {})
        if any(node in nodes and item.get('epoch') != nodes[node].get('epoch')
               for node, item in old_nodes.items()):
            return snapshot
        return {**snapshot, 'pose': pose, 'orientation': preview.get('orientation'),
                'geometry': None, 'display_pose': display, 'display_geometry': preview['geometry']}

    def compose(self, snapshot, tactile_state, *, now_ns=None):
        now = host_clock.monotonic_ns() if now_ns is None else _clock(now_ns)
        if self._last_now is not None and now < self._last_now:
            self.reset()
            self._last_now = now
            return self._fixed_hand("时钟异常 · 等待新数据")
        self._last_now = now
        snapshot = snapshot if isinstance(snapshot, dict) else {}
        snapshot = self._display_snapshot(snapshot, now)
        pose_frame = self._compose_pose(snapshot, now)
        touch = self._current_touch(tactile_state, now)
        if touch is None:
            return pose_frame
        # A tracked overlay requires a dated pose from the hardware stream.
        if pose_frame.pose_current and (pose_frame.pose_age_ms is None
                                        or snapshot["pose"].get("hardware_accessed") is False):
            return pose_frame
        age_ms = (now - touch.timing.source_monotonic_ns) / 1e6
        status = pose_frame.status
        skew_ms = None
        if pose_frame.pose_current:
            status = "实时手势与触觉" if all(touch.frame["valid_mask"]) else "实时手势 · 部分触觉有效"
            skew_ms = abs(snapshot["pose"]["source_monotonic_ns"] - touch.timing.source_monotonic_ns) / 1e6
        elif pose_frame.mode == "reference":
            status = "固定参考手形 · 仅触觉"
        return replace(pose_frame, tactile_frame=touch.frame, status=status, tactile_age_ms=age_ms, skew_ms=skew_ms)

    def _compose_pose(self, snapshot, now):
        runtime = snapshot.get("runtime") or {}
        pose = snapshot.get("pose") or {}
        calibration = snapshot.get("calibration") or {}
        if not all(isinstance(value, dict) for value in (runtime, pose, calibration)):
            self._pose_context = None
            return self._fixed_hand("等待当前会话的手势")
        context = (runtime.get("session_id"), *(pose.get(key) for key in _POSE_CONTEXT_FIELDS),
                   calibration.get("clear_generation"), calibration.get("functional_enabled"),
                   pose.get("source_reference_node"), pose.get("input_kind"), pose.get("hardware_accessed"),
                   runtime.get("connected"))
        if context != self._pose_context:
            self._last_shape = None
            self._pose_context = context

        if runtime.get("connected") is False and not runtime.get("error"):
            return self._fixed_hand()
        session = runtime.get("session_id")
        if runtime.get("connected") and runtime.get("received_samples") == 0 and not runtime.get("error"):
            return self._fixed_hand("等待 IMU 数据")
        status = str(calibration.get("profile_status", ""))
        if (not isinstance(session, str) or not session or pose.get("session_id") != session
                or runtime.get("clear_pending") or status.startswith("fitting") or status == "clearing"
                or ("profile_version" in calibration and calibration["profile_version"] != pose.get("profile_version"))):
            return self._fixed_hand("等待当前会话的手势")
        active = calibration.get("active") or {}
        # A functional capture observes a hand already solved with the current
        # mounting profile. Its old profile stays active until fitting starts.
        functional_motion = (active.get("kind") == "functional_hand" and status == "collecting_functional_hand"
                             and bool(calibration.get("calibrated_nodes")))
        if (pose.get("reason") == "calibration_required" or (active and not functional_motion)
                or status == "not_calibrated"):
            return self._fixed_hand("等待手势标定")
        if not measured_pose_visible(snapshot, now / 1e9):
            return self._timing_gap(snapshot, now)
        if pose.get("hand", "right") != self.hand:
            return self._fixed_hand("等待当前会话的手势")
        geometry = self._pose_geometry(snapshot, pose)
        if geometry is None:
            return self._fixed_hand("手势模型暂不可用")
        source = pose.get("source_monotonic_ns")
        display_revision = ((snapshot.get("display_pose") or {}).get("shape_preview_revision")
                            if geometry is snapshot.get("display_geometry") else None)
        key = (context, pose.get("frame_index"), pose.get("timestamp_s"), source,
               geometry.get("display_only", False), display_revision)
        if not _positive_ns(source):
            self._last_shape = None
            return CombinedFrame(geometry, None, True, "live", "手势源时间不可用", geometry_key=key)
        source = int(source)
        age = (now - source) / 1e6
        if (not 0 <= now - source <= MAX_AGE_NS
                or self._last_shape is not None and source < self._last_shape.source_ns):
            return self._fixed_hand("手势数据已过期", pose_age_ms=age)
        self._remember_shape(snapshot, geometry, key, source)
        return CombinedFrame(geometry, None, True, "live", "实时手势 · 等待触觉", pose_age_ms=age,
                             geometry_key=key)
