"""Bounded display association of solved geometry and tactile responses.

Both sources retain their own identity and source clocks. A latest GUI poll is
not a new measurement, and the palm's receive time is only a display reference
for an asynchronous hand: this is not hardware synchronization or a solver.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from vt_capture_console.backend.tactile_source import TactileDisplayState, TactileTiming
from vt_fsglove12.joint_quality import joint_quality_state
from vt_fsglove12.live_display import measured_pose_visible
from vt_tactile_capture import host_clock
from vt_tactile_capture.hand_layout import load_layout


MAX_AGE_NS = 250_000_000
MAX_SKEW_NS = 150_000_000
MAX_HISTORY = 60
_BLOCK_INDEX = (np.arange(20)[:, None] * 8 + np.arange(20)[None, :] % 8).reshape(-1)
_FINGER_SLOTS = {"index": range(1, 4), "middle": range(4, 7), "little": range(7, 10),
                 "ring": range(10, 13), "thumb": range(13, 16)}
_POSE_CONTEXT_FIELDS = ("session_id", "profile_version", "config_sha256", "model_rest_sha256")


@dataclass(frozen=True)
class CombinedFrame:
    geometry: dict | None
    tactile_frame: dict | None
    pose_current: bool
    mode: str
    status: str
    pose_age_ms: float | None = None
    tactile_age_ms: float | None = None
    skew_ms: float | None = None


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
    """One display's independent history; no hardware, Qt, I/O or pose changes.

    ``compose`` receives the exact public results from the two controllers.
    It never reads an assembler or model's private cache. Hand geometry remains
    owned by the input snapshot; tactile history and output masks are copied.
    """

    def __init__(self, neutral_geometry):
        self.neutral_geometry = deepcopy(neutral_geometry)
        self._vertices = len(neutral_geometry["vertices_m"])
        self._faces = np.asarray(neutral_geometry["faces"], dtype=np.int64)
        if not self._geometry_valid(neutral_geometry):
            raise ValueError("The reference hand geometry is invalid")
        layout = load_layout(expected_hand="right")
        self._regions = {region: np.zeros(400, dtype=bool) for region in _FINGER_SLOTS}
        for point in layout.points:
            if point.region in self._regions:
                self._regions[point.region][point.row * 20 + point.col] = True
        self._history = deque(maxlen=MAX_HISTORY)
        self._tactile_identity = self._pose_context = None
        self._last_now = None

    @property
    def history_size(self):
        return len(self._history)

    def reset(self):
        self._history.clear()
        self._tactile_identity = self._pose_context = self._last_now = None

    def _geometry_valid(self, geometry):
        if not isinstance(geometry, dict):
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

    @staticmethod
    def _mask(sample, now, pose_ns=None):
        timing = sample.timing
        blocks = np.asarray(timing.block_monotonic_ns, dtype=np.uint64)[_BLOCK_INDEX]
        age = min(MAX_AGE_NS, timing.max_age_ns)
        mask = np.asarray(sample.frame["valid_mask"], dtype=bool).copy()
        mask &= ((blocks > 0) & (blocks <= timing.source_monotonic_ns)
                 & (blocks <= timing.snapshot_monotonic_ns) & (blocks <= now)
                 & (blocks >= max(0, now - age)))
        if pose_ns is not None:
            mask &= (blocks <= pose_ns) & (blocks >= max(0, pose_ns - age))
        return mask

    def _observe_tactile(self, state, now):
        # Any observable health/reference boundary invalidates prior hot spots.
        if (not isinstance(state, TactileDisplayState) or not isinstance(state.frame, dict)
                or not isinstance(state.timing, TactileTiming)
                or state.frame.get("reference_ready") is not True
                or state.response is None or state.response.phase != "ready"):
            self._history.clear()
            self._tactile_identity = None
            return
        timing = state.timing
        if not self._fresh(timing, now):
            self._history.clear()
            self._tactile_identity = None
            return
        try:
            frame = state.frame
            values = np.asarray(frame["values"], dtype=float)
            valid = np.asarray(frame["valid_mask"])
            levels = np.asarray(frame["levels"], dtype=float)
            if (frame.get("value_kind") != "tactile_response" or values.shape != (400,)
                    or not np.isfinite(values).all() or np.any(values < 0)
                    or valid.shape != (400,) or valid.dtype.kind not in "biu"
                    or np.any((valid != 0) & (valid != 1)) or levels.shape != (2,)
                    or not np.isfinite(levels).all() or levels[0] != 0 or levels[1] <= 0):
                raise ValueError("Invalid tactile response")
            sample = _TactileSample(timing, deepcopy(frame))
            if not np.any(self._mask(sample, now)):
                raise ValueError("No current tactile points")
        except (KeyError, TypeError, ValueError, OverflowError):
            self._history.clear()
            self._tactile_identity = None
            return
        identity = self._identity(timing)
        if identity != self._tactile_identity:
            self._history.clear()
            self._tactile_identity = identity
        if self._history:
            previous = self._history[-1].timing
            if (timing.source_sequence < previous.source_sequence
                    or timing.source_monotonic_ns < previous.source_monotonic_ns):
                self._history.clear()
                return
            if timing.source_sequence == previous.source_sequence:
                # A repeated poll may remove expired cells, but cannot retime
                # the sample or alter the rolling snapshot's provenance.
                if timing != previous:
                    self._history.clear()
                    return
                self._history[-1] = sample
                return
        self._history.append(sample)

    @staticmethod
    def _output(sample, mask):
        if not np.any(mask):
            return None
        result = deepcopy(sample.frame)
        result["valid_mask"] = mask.tolist()
        return result

    def _pose_geometry(self, snapshot, pose):
        displayed = snapshot.get("display_pose")
        geometry = snapshot.get("display_geometry")
        if (isinstance(displayed, dict)
                and all(displayed.get(key) == pose.get(key) for key in (*_POSE_CONTEXT_FIELDS, "frame_index"))
                and displayed.get("timestamp_s") == pose.get("timestamp_s")
                and displayed.get("shape_preview_mode", "current") == "current"
                and self._geometry_valid(geometry)):
            return geometry, displayed
        geometry = snapshot.get("geometry")
        if self._geometry_valid(geometry) and not geometry.get("display_only"):
            return geometry, pose
        return None, pose

    def compose(self, snapshot, tactile_state, *, now_ns=None):
        now = host_clock.monotonic_ns() if now_ns is None else _clock(now_ns)
        if self._last_now is not None and now < self._last_now:
            self.reset()
            self._last_now = now
            return CombinedFrame(None, None, False, "waiting", "时钟异常 · 等待新数据")
        self._last_now = now
        snapshot = snapshot if isinstance(snapshot, dict) else {}
        runtime = snapshot.get("runtime") or {}
        pose = snapshot.get("pose") or {}
        calibration = snapshot.get("calibration") or {}
        if not all(isinstance(value, dict) for value in (runtime, pose, calibration)):
            self._history.clear()
            return CombinedFrame(None, None, False, "waiting", "等待当前会话的手势")
        context = (runtime.get("session_id"), *(pose.get(key) for key in _POSE_CONTEXT_FIELDS),
                   calibration.get("clear_generation"), runtime.get("connected"))
        if context != self._pose_context:
            self._history.clear()
            self._pose_context = context
        self._observe_tactile(tactile_state, now)

        if runtime.get("connected") is False and not runtime.get("error"):
            sample = self._history[-1] if self._history else None
            touch = None if sample is None else self._output(sample, self._mask(sample, now))
            return CombinedFrame(self.neutral_geometry, touch, False, "reference",
                "固定参考手形 · 仅触觉" if touch is not None else "固定参考手形 · 等待连接",
                tactile_age_ms=None if sample is None else (now - sample.timing.source_monotonic_ns) / 1e6)
        session = runtime.get("session_id")
        if (not isinstance(session, str) or not session or pose.get("session_id") != session
                or runtime.get("clear_pending") or calibration.get("profile_status") in {"fitting", "clearing"}
                or ("profile_version" in calibration and calibration["profile_version"] != pose.get("profile_version"))):
            return CombinedFrame(None, None, False, "waiting", "等待当前会话的手势")
        if (pose.get("reason") == "calibration_required" or calibration.get("active")
                or calibration.get("profile_status") == "not_calibrated"):
            return CombinedFrame(None, None, False, "waiting", "等待手势标定")
        if not measured_pose_visible(snapshot, now / 1e9):
            return CombinedFrame(None, None, False, "waiting", "手势数据已过期")
        if pose.get("hand", "right") != "right":
            return CombinedFrame(None, None, False, "waiting", "等待当前会话的手势")
        geometry, displayed = self._pose_geometry(snapshot, pose)
        if geometry is None:
            return CombinedFrame(None, None, False, "waiting", "手势模型暂不可用")
        source = pose.get("source_monotonic_ns")
        if not _positive_ns(source):
            return CombinedFrame(geometry, None, True, "live", "手势源时间不可用")
        source = int(source)
        age = (now - source) / 1e6
        if not 0 <= now - source <= MAX_AGE_NS:
            return CombinedFrame(None, None, False, "waiting", "手势数据已过期", pose_age_ms=age)
        if not self._history or pose.get("hardware_accessed") is False:
            return CombinedFrame(geometry, None, True, "live", "实时手势 · 等待触觉", pose_age_ms=age)
        causal = [sample for sample in self._history if self._fresh(sample.timing, now)
                  and sample.timing.source_monotonic_ns <= source
                  and sample.timing.snapshot_monotonic_ns <= source]
        if not causal:
            return CombinedFrame(geometry, None, True, "live", "实时手势 · 等待时间对应的触觉", pose_age_ms=age)
        sample = max(causal, key=lambda item: (item.timing.source_monotonic_ns, item.timing.source_sequence))
        skew = source - sample.timing.source_monotonic_ns
        touch_age = (now - sample.timing.source_monotonic_ns) / 1e6
        if skew > MAX_SKEW_NS:
            return CombinedFrame(geometry, None, True, "live", "实时手势 · 等待时间对应的触觉",
                                 age, touch_age, skew / 1e6)
        mask = self._mask(sample, now, source)
        try:
            hidden = set(displayed.get("held_slots", ())) | set(displayed.get("unknown_slots", ()))
            if any(type(slot) is not int or not 0 <= slot < 16 for slot in hidden):
                raise ValueError("Invalid unavailable slots")
            _, available, _ = joint_quality_state(pose)
            hidden.update(slot for slot, usable in enumerate(available) if not usable)
        except (TypeError, ValueError):
            hidden = {0}
        if 0 in hidden:
            mask[:] = False
        else:
            for region, slots in _FINGER_SLOTS.items():
                if hidden.intersection(slots):
                    mask[self._regions[region]] = False
        touch = self._output(sample, mask)
        status = ("实时手势 · 等待触觉" if touch is None else
                  "实时手势与触觉" if mask.all() else "实时手势 · 部分触觉有效")
        return CombinedFrame(geometry, touch, True, "live", status, age, touch_age, skew / 1e6)
