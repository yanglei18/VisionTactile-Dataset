"""Qt/ROS-free validation and presentation of right-hand raw tactile snapshots."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from vt_tactile_capture.grid import BLOCK_COUNT, CELL_COUNT, SHAPE, normalize_state
from vt_tactile_capture import host_clock


MAX_AGE_NS = 250_000_000
_RATE_WINDOW_NS = 2_000_000_000
_BLOCK_INDEX = np.arange(20)[:, None] * 8 + np.arange(20)[None, :] % 8


@dataclass(frozen=True)
class TactileDisplayState:
    frame: dict | None
    status: str
    detail: str
    age_ms: float | None = None
    receive_hz: float = 0.0
    valid_cells: int = 0


def _clock(value):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or value < 0:
        raise ValueError("时间及序号必须为非负整数")
    return int(value)


class TactileDisplayModel:
    """Turn decoded snapshots into raw-ADC frames, reevaluating age on every poll.

    The receive rate counts distinct source snapshots delivered to this model,
    using local arrival time. It is not a device scan frequency.
    """

    def __init__(self):
        self._sample = None
        self._identity = None
        self._sequence = None
        self._generations = None
        self._arrivals = deque(maxlen=120)
        self._last_poll_ns = None
        self._status = "等待数据"
        self._detail = "等待右手触觉数据。"

    def invalidate(self, status, detail):
        self._sample = None
        self._arrivals.clear()
        self._status, self._detail = status, detail

    def observe(self, message, *, now_ns=None):
        now = host_clock.monotonic_ns() if now_ns is None else _clock(now_ns)
        try:
            message = normalize_state(message)
            if message.hand != "right":
                raise ValueError("触觉手别不匹配，需要右手数据。")
            if message.clock_domain != "CLOCK_MONOTONIC" or message.timing_source != "host_receive":
                raise ValueError("触觉时钟不匹配，需要本机单调时钟及主机接收时间。")
            if message.unit != "adc_counts":
                raise ValueError("触觉数据单位不是原始 ADC。")
            if not isinstance(message.session_id, str) or not message.session_id:
                raise ValueError("触觉数据缺少采集会话。")
            identity = (message.session_id, _clock(message.stream_epoch))
            sequence = _clock(message.source_sequence)
            generations = (getattr(message, "transport_reset_generation", 0),
                           getattr(message, "transport_health_generation", 0))
            if identity != self._identity or generations != self._generations:
                self.invalidate("等待数据", "触觉数据流已切换，等待当前会话数据。")
                self._sequence = None
            self._identity, self._generations = identity, generations
            if not isinstance(message.connected, (bool, np.bool_)) or not isinstance(message.time_valid, (bool, np.bool_)):
                raise ValueError("触觉连接或时间状态无效。")
            if not message.connected:
                self.invalidate("连接断开", "右手触觉源已断开，显示已清除。")
                return
            if not message.time_valid:
                raise ValueError("触觉源时间无效，显示已清除。")
            source = _clock(message.host_monotonic_ns)
            snapshot = _clock(message.snapshot_monotonic_ns)
            max_age = _clock(message.max_age_ns)
            if not source or snapshot < source or source > now or snapshot > now:
                raise ValueError("触觉源或快照时间无效、位于未来或顺序错误。")
            if max(now - source, now - snapshot) > MAX_AGE_NS:
                self.invalidate("数据过期", "触觉源或快照超过 250 毫秒未更新，显示已清除。")
                return
            if getattr(message, "transport_health_only", False):
                self._sequence = sequence
                self.invalidate("等待数据", "触觉连接状态已变化，等待新的源数据。")
                return
            adc = np.asarray(message.adc)
            mask = np.asarray(message.valid_mask)
            blocks = np.asarray(message.block_monotonic_ns)
            if (adc.shape != (CELL_COUNT,) or adc.dtype.kind not in "iu"
                    or np.any(adc < 0) or np.any(adc > 65535)):
                raise ValueError("触觉数据需要 400 个有效的无符号 ADC 值。")
            if (mask.shape != (CELL_COUNT,) or mask.dtype.kind not in "biu"
                    or np.any((mask != 0) & (mask != 1))):
                raise ValueError("触觉有效掩码格式错误。")
            if blocks.shape != (BLOCK_COUNT,) or blocks.dtype.kind not in "iu" or np.any(blocks < 0):
                raise ValueError("触觉逐块时间格式错误。")
            if self._sequence is not None and sequence < self._sequence:
                raise ValueError("触觉源序号倒退，显示已清除。")
            if sequence == self._sequence:
                return  # A repeated source cannot renew freshness or receive rate.
            self._sequence = sequence
            self._sample = (adc.copy(), mask.astype(bool, copy=True), blocks.copy(), source, snapshot, max_age)
            self._arrivals.append(now)
            self._status, self._detail = "实时", "原始 ADC 滚动快照；固定参考手形。"
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self.invalidate("数据无效", str(exc))

    def poll(self, *, now_ns=None):
        now = host_clock.monotonic_ns() if now_ns is None else _clock(now_ns)
        if self._last_poll_ns is not None and now < self._last_poll_ns:
            self.invalidate("数据无效", "本机单调时钟倒退，显示已清除。")
        self._last_poll_ns = now
        if self._sample is None:
            return TactileDisplayState(None, self._status, self._detail)
        adc, mask, blocks, source, snapshot, max_age = self._sample
        age_ms = (now - source) / 1_000_000
        if now < source or now < snapshot or now - source > MAX_AGE_NS or now - snapshot > MAX_AGE_NS:
            self.invalidate("数据过期", "触觉源或快照超过 250 毫秒未更新，显示已清除。")
            return TactileDisplayState(None, self._status, self._detail, max(0., age_ms))
        stamps = blocks[_BLOCK_INDEX]
        valid = (mask.reshape(SHAPE) & (stamps > 0) & (stamps <= source)
                 & (stamps <= snapshot) & (stamps <= now)
                 & (stamps >= max(0, now - min(max_age, MAX_AGE_NS))))
        count = int(valid.sum())
        while self._arrivals and self._arrivals[0] < now - _RATE_WINDOW_NS:
            self._arrivals.popleft()
        span = self._arrivals[-1] - self._arrivals[0] if len(self._arrivals) > 1 else 0
        rate = (len(self._arrivals) - 1) * 1e9 / span if span > 0 else 0.
        if not count:
            return TactileDisplayState(None, "数据过期", "没有仍在有效时限内的触点，显示已清除。", age_ms, rate)
        frame = {"values": adc.tolist(), "valid_mask": valid.reshape(-1).tolist()}
        return TactileDisplayState(frame, "实时" if count == CELL_COUNT else "部分有效",
                                   f"有效触点 {count}/400；原始 ADC 滚动快照。", age_ms, rate, count)
