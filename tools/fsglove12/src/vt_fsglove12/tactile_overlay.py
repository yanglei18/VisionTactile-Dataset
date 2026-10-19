"""Display-only tactile/pose joining, with no ROS, Qt, or solver dependency.

Each input is a rolling ADC state, not a simultaneous skin scan. Source and
per-block CLOCK_MONOTONIC times remain authoritative after IPC delivery. This
module is imported only when the optional tactile display is enabled.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from numbers import Integral, Real
import time
from types import SimpleNamespace

import numpy as np

from vt_tactile_capture.display_baseline import DisplayBaseline
from vt_tactile_capture.display_transport import LocalTactileReceiver
from vt_tactile_capture.grid import BLOCK_COUNT, CELL_COUNT, SCHEMA_VERSION, SHAPE, normalize_state
from vt_tactile_capture.hand_layout import load_layout


MAX_AGE_NS = 250_000_000
MAX_SKEW_NS = 150_000_000
MAX_HISTORY = 60
_BLOCK_INDEX = np.arange(SHAPE[0])[:, None] * 8 + np.arange(SHAPE[1])[None, :] % 8
# Physical addresses are the original upper-left 20x20 window, not a rescale.
_FINGER_SLOTS = ((range(1, 4), "index"), (range(4, 7), "middle"), (range(7, 10), "little"),
                 (range(10, 13), "ring"), (range(13, 16), "thumb"))
_UNITS = "ADC 参考差值，非压力"


@dataclass(frozen=True)
class TactileOverlayFrame:
    delta: np.ndarray | None
    valid_mask: np.ndarray | None
    status: str
    age_ms: float | None = None
    skew_ms: float | None = None
    sequence: int | None = None


def _readonly(value, dtype=None):
    result = np.array(value, dtype=dtype, copy=True, order="C")
    result.setflags(write=False)
    return result


def _uint(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or value < 0:
        raise ValueError(f"{name} 必须是非负整数")
    return int(value)


def _now(value):
    return time.monotonic_ns() if value is None else _uint(value, "now_ns")


def _copy_state(message, expected_hand="right"):
    """Keep a bounded independent copy, including when tests inject a receiver."""
    message = normalize_state(message)
    integers = {key: _uint(getattr(message, key), key) for key in (
        "schema_version", "rows", "cols", "stream_epoch", "source_sequence",
        "host_monotonic_ns", "snapshot_monotonic_ns", "max_age_ns")}
    if integers["schema_version"] != SCHEMA_VERSION or (integers["rows"], integers["cols"]) != SHAPE:
        raise ValueError("不支持的触觉数据格式")
    if not isinstance(message.session_id, str) or not message.session_id:
        raise ValueError("缺少采集会话")
    if getattr(message, "hand", "") != expected_hand:
        raise ValueError(f"触觉手别不匹配：需要 {expected_hand}")
    if getattr(message, "unit", "adc_counts") != "adc_counts":
        raise ValueError("触觉数据单位不是 ADC")
    health = {}
    for name in ("connected", "time_valid"):
        value = getattr(message, name)
        if not isinstance(value, (bool, np.bool_)):
            raise ValueError("无效的连接／时间状态")
        health[name] = bool(value)
    adc, mask, blocks = (np.asarray(getattr(message, key)) for key in (
        "adc", "valid_mask", "block_monotonic_ns"))
    if (adc.shape not in ((CELL_COUNT,), SHAPE) or adc.dtype.kind not in "iu"
            or np.any(adc < 0) or np.any(adc > 65535)):
        raise ValueError("无效的 ADC 数组")
    if (mask.shape not in ((CELL_COUNT,), SHAPE) or mask.dtype.kind not in "biu"
            or np.any((mask != 0) & (mask != 1))):
        raise ValueError("无效的触觉有效性掩码")
    if blocks.shape != (BLOCK_COUNT,) or blocks.dtype.kind not in "iu" or np.any(blocks < 0):
        raise ValueError("无效的逐块时间数组")
    return SimpleNamespace(**integers, **health, hand=message.hand, session_id=message.session_id,
        clock_domain=message.clock_domain, timing_source=message.timing_source,
        adc=_readonly(adc.reshape(-1), np.uint16),
        valid_mask=_readonly(mask.reshape(-1), bool),
        block_monotonic_ns=_readonly(blocks, np.uint64))


def _state_error(state, now_ns):
    if not state.connected:
        return "触觉已断开；请恢复连接并重新采集 ADC 参考"
    if not state.time_valid:
        return "触觉源时间无效；请重新采集 ADC 参考"
    if state.clock_domain != "CLOCK_MONOTONIC" or state.timing_source != "host_receive":
        return "触觉时钟不匹配（需要本机 CLOCK_MONOTONIC / host_receive）"
    if state.snapshot_monotonic_ns < state.host_monotonic_ns:
        return "触觉快照时间早于源时间"
    for label, stamp in (("源", state.host_monotonic_ns), ("快照", state.snapshot_monotonic_ns)):
        if stamp > now_ns:
            return f"触觉{label}时间在未来，叠加已清除"
        if now_ns - stamp > MAX_AGE_NS:
            return f"触觉{label}已过期（{(now_ns - stamp) / 1e6:.0f} ms），叠加已清除"
    return ""


def _cell_mask(state, now_ns, pose_ns=None):
    age_limit = min(MAX_AGE_NS, state.max_age_ns)
    stamps = state.block_monotonic_ns[_BLOCK_INDEX]
    # Unsigned source clocks are compared, never subtracted (no wraparound).
    valid = (state.valid_mask.reshape(SHAPE) & (stamps > 0)
             & (stamps <= state.host_monotonic_ns)
             & (stamps <= state.snapshot_monotonic_ns)
             & (stamps <= now_ns) & (stamps >= max(0, now_ns - age_limit)))
    if pose_ns is not None:
        valid &= (stamps <= pose_ns) & (stamps >= max(0, pose_ns - age_limit))
    return valid


class TactileOverlayController:
    """UI-thread controller; manual tactile reference is independent of IMUs.

    ``poll`` drains one bounded receiver batch and retains at most 60 states.
    ``frame`` never advances a hand, calibrates a solver, or extrapolates data.
    A dropped connection, clock fault, or session boundary invalidates the
    reference even when recovery happens within one IPC drain.
    """

    def __init__(self, path=None, *, receiver=None, expected_hand="right", layout_path=None):
        self.layout = load_layout(layout_path, expected_hand=expected_hand)
        self.expected_hand = expected_hand
        self._finger_masks = {}
        for slots, region in _FINGER_SLOTS:
            mask = np.zeros(SHAPE, dtype=bool)
            for point in self.layout.points:
                if point.region == region:
                    mask[point.row, point.col] = True
            self._finger_masks[region] = mask
        self.receiver = receiver if receiver is not None else LocalTactileReceiver(path, hand=expected_hand)
        self.baseline = DisplayBaseline(expected_hand=expected_hand)
        self.status = "等待触觉数据；尚无 ADC 参考，非压力标定"
        self._history = deque(maxlen=MAX_HISTORY)
        self._latest = None
        self._identity = None
        self._last_sequence = None
        self._last_now_ns = None
        self._generations = self._receiver_generations()
        self._closed = False

    def _receiver_generations(self):
        return (getattr(self.receiver, "reset_generation", 0),
                getattr(self.receiver, "health_generation", 0))

    def _invalidate(self, status):
        self._history.clear()
        self._latest = None
        self._last_sequence = None
        self.baseline.invalidate()
        self.status = status

    def poll(self, now_ns=None):
        now_ns = _now(now_ns)
        if self._closed:
            self.baseline.poll(now_ns)
            return
        try:
            if self._last_now_ns is not None and now_ns < self._last_now_ns:
                self._invalidate("本机单调时钟回退；请重新采集 ADC 参考")
            self._last_now_ns = now_ns
            message = self.receiver.receive_latest()
            generations = self._receiver_generations()
            if generations != self._generations:
                self._invalidate("触觉采集会话／连接状态已变化；请重新采集 ADC 参考")
                self._generations = generations
            if message is None:
                return
            state = _copy_state(message, self.expected_hand)
            identity = (state.session_id, state.stream_epoch)
            if self._identity is not None and identity != self._identity:
                self._invalidate("触觉采集会话／批次已变化；请重新采集 ADC 参考")
            self._identity = identity
            # Reject out-of-order packets, including fake/in-memory receivers.
            if self._last_sequence is not None and state.source_sequence <= self._last_sequence:
                return
            self._last_sequence = state.source_sequence
            error = _state_error(state, now_ns)
            if error:
                self._invalidate(error)
                self._latest = state
                self._last_sequence = state.source_sequence
                return
            self._latest = state
            self._history.append(state)
            mask = _cell_mask(state, now_ns)
            if np.any(mask):
                # Use a new validity array: source state and history stay intact.
                sample = SimpleNamespace(**vars(state))
                sample.valid_mask = mask.reshape(-1)
                self.baseline.observe(sample, now_ns)
            self.status = self._reference_status()
        except (AttributeError, OSError, TypeError, ValueError, OverflowError) as exc:
            self._invalidate(f"触觉数据不可用：{exc}；请重新采集 ADC 参考")
        finally:
            # This deadline must advance even if the stream stops publishing.
            self.baseline.poll(now_ns)

    def _reference_status(self):
        if self.baseline.collecting:
            return f"正在采集 3 秒空载 ADC 参考（{self.baseline.samples_count} 个不同源状态）；所有触觉区域请勿按压"
        if self.baseline.values is None:
            if "cancelled" in self.baseline.status:
                return "ADC 参考采集已取消：需要至少 10 个不同的新鲜源状态，并持续空载采集 3 秒；请重试"
            return "尚无 ADC 参考；请空载采集 3 秒参考，非压力标定"
        return "ADC 参考就绪；仅显示有符号 ADC 差值，非压力"

    def start_reference(self, now_ns=None) -> bool:
        now_ns = _now(now_ns)
        self.poll(now_ns)
        state = self._latest
        if self._closed or state is None:
            return False
        error = _state_error(state, now_ns)
        if error or not np.any(_cell_mask(state, now_ns)):
            self.status = error or "没有新鲜有效的触觉单元；无法开始 ADC 参考"
            return False
        self.baseline.start((state.session_id, state.stream_epoch), now_ns)
        # The already present state is one sample, never one sample per UI tick.
        sample = SimpleNamespace(**vars(state))
        sample.valid_mask = _cell_mask(state, now_ns).reshape(-1)
        self.baseline.observe(sample, now_ns)
        self.status = self._reference_status()
        return True

    def frame(self, pose_timestamp_s, *, pose_current=True, now_ns=None, metadata=None):
        now_ns = _now(now_ns)
        self.baseline.poll(now_ns)

        def empty(reason, state=None, skew=None):
            age = None if state is None else (now_ns - state.host_monotonic_ns) / 1e6
            self.status = reason if reason.endswith(_UNITS) else reason + " · " + _UNITS
            return TactileOverlayFrame(None, None, self.status, age, skew,
                                       None if state is None else state.source_sequence)

        if self._closed:
            return empty("触觉显示已关闭")
        metadata = metadata if isinstance(metadata, dict) else {}
        if metadata.get("hand", self.expected_hand) != self.expected_hand:
            return empty("手姿态与触觉手别不匹配；叠加已清除")
        if (not pose_current or metadata.get("demo") or metadata.get("is_demo")
                or metadata.get("calibrated") is False or metadata.get("realtime") is False):
            return empty(self._reference_status()
                         + " · 手姿态非当前已标定实时数据；仅清除触觉叠加，保留手形")
        if (isinstance(pose_timestamp_s, (bool, np.bool_)) or not isinstance(pose_timestamp_s, Real)
                or not np.isfinite(pose_timestamp_s) or pose_timestamp_s < 0):
            return empty("手姿态源时间不可用；保留手形")
        pose_ns = int(round(float(pose_timestamp_s) * 1e9))
        if pose_ns > now_ns or now_ns - pose_ns > MAX_AGE_NS:
            return empty("手姿态源时间在未来或已过期；仅清除触觉叠加，保留手形")
        if self._latest is None:
            return empty(self.status)
        error = _state_error(self._latest, now_ns)
        if error:
            return empty(error, self._latest)
        # Never show history from before a detected health/identity boundary.
        causal = [state for state in self._history
                  if state.host_monotonic_ns <= pose_ns and state.snapshot_monotonic_ns <= pose_ns]
        if not causal:
            return empty("没有不晚于手姿态的触觉状态；不使用未来数据")
        state = max(causal, key=lambda item: (item.host_monotonic_ns, item.source_sequence))
        skew = (pose_ns - state.host_monotonic_ns) / 1e6
        error = _state_error(state, now_ns)
        if error:
            return empty(error, state, skew)
        if pose_ns - state.host_monotonic_ns > MAX_SKEW_NS:
            return empty(f"触觉与手姿态时差过大（{skew:.0f} ms > 150 ms）", state, skew)
        if (self.baseline.values is None
                or self.baseline.identity != (state.session_id, state.stream_epoch)):
            return empty(self._reference_status(), state, skew)
        mask = _cell_mask(state, now_ns, pose_ns)
        try:
            held = set(metadata.get("held_slots", [])) | set(metadata.get("unknown_slots", []))
        except TypeError:
            return empty("局部手姿态状态不可用；保留手形", state, skew)
        if 0 in held:
            return empty("手根姿态保持／未知；触觉叠加已清除，保留手形", state, skew)
        for slots, region in _FINGER_SLOTS:
            if held.intersection(slots):
                mask[self._finger_masks[region]] = False
        delta = self.baseline.subtract(state.adc.reshape(SHAPE))
        mask &= np.isfinite(delta)
        if not np.any(mask):
            return empty("没有新鲜且与当前手姿态对应的有效触觉单元", state, skew)
        delta[~mask] = np.nan
        age = (now_ns - state.host_monotonic_ns) / 1e6
        self.status = (f"滚动触觉状态（非同时扫描）· 源年龄 {age:.0f} ms · 手姿态时差 {skew:.0f} ms"
                       + (" · 已屏蔽保持／未知手指" if held else "") + " · " + _UNITS)
        return TactileOverlayFrame(_readonly(delta), _readonly(mask, bool), self.status,
                                   age, skew, state.source_sequence)

    def close(self):
        if not self._closed:
            self._closed = True
            self._invalidate("触觉显示已关闭")
            self.receiver.close()
