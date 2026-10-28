"""Qt/ROS-free validation and presentation of right-hand raw tactile snapshots."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from vt_tactile_capture.grid import BLOCK_COUNT, CELL_COUNT, SHAPE, normalize_state
from vt_tactile_capture import host_clock
from vt_tactile_capture.display_response import ReferenceProfile, ResponseStatus, TactileResponseProcessor


MAX_AGE_NS = 250_000_000
_RATE_WINDOW_NS = 2_000_000_000
_BLOCK_INDEX = np.arange(20)[:, None] * 8 + np.arange(20)[None, :] % 8


@dataclass(frozen=True)
class TactileTiming:
    """Immutable provenance for the exact rolling sample used by a display.

    ``display_generation`` changes at local health, reference or display-setting
    boundaries. Consumers can discard old colors without inspecting private
    model state or treating a GUI repaint as another sensor sample.
    """

    session_id: str
    stream_epoch: int
    source_sequence: int
    source_monotonic_ns: int
    snapshot_monotonic_ns: int
    block_monotonic_ns: tuple[int, ...]
    max_age_ns: int
    transport_reset_generation: int = 0
    transport_health_generation: int = 0
    display_generation: int = 0

    def __post_init__(self):
        if not isinstance(self.session_id, str) or not self.session_id:
            raise ValueError("A tactile timing sample requires a session ID")
        for name in ("stream_epoch", "source_sequence", "source_monotonic_ns", "snapshot_monotonic_ns",
                     "max_age_ns", "transport_reset_generation", "transport_health_generation", "display_generation"):
            object.__setattr__(self, name, _clock(getattr(self, name)))
        blocks = tuple(_clock(value) for value in self.block_monotonic_ns)
        if len(blocks) != BLOCK_COUNT:
            raise ValueError("Tactile timing requires 160 block timestamps")
        if not self.source_monotonic_ns or self.snapshot_monotonic_ns < self.source_monotonic_ns:
            raise ValueError("Tactile source/snapshot timestamps are out of order")
        object.__setattr__(self, "block_monotonic_ns", blocks)


@dataclass(frozen=True)
class TactileDisplayState:
    frame: dict | None
    status: str
    detail: str
    age_ms: float | None = None
    receive_hz: float = 0.0
    valid_cells: int = 0
    response: ResponseStatus | None = None
    timing: TactileTiming | None = None


def _clock(value):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or value < 0:
        raise ValueError("时间及序号必须为非负整数")
    return int(value)


class TactileDisplayModel:
    """Turn decoded snapshots into raw-ADC frames, reevaluating age on every poll.

    The receive rate counts distinct source snapshots delivered to this model,
    using local arrival time. It is not a device scan frequency.
    """

    def __init__(self, *, response_enabled=False, response_settings=None,
                 response_reference=None, reference_source="saved"):
        if response_reference is not None and (not response_enabled or not isinstance(response_reference, ReferenceProfile)):
            raise ValueError("A stored reference requires response display and a validated profile")
        if reference_source not in ("saved", "preset", "default"):
            raise ValueError("Stored reference source must be saved, preset, or default")
        self._response = TactileResponseProcessor(response_settings) if response_enabled else None
        self._stored_reference = response_reference
        self._stored_reference_source = reference_source
        self._sample = None
        self._identity = None
        self._sequence = None
        self._generations = None
        self._display_generation = 0
        self._arrivals = deque(maxlen=120)
        self._last_poll_ns = None
        self._status = "等待数据"
        self._detail = "等待右手触觉数据。"

    @property
    def response_status(self):
        return self._response.status if self._response is not None else None

    @property
    def response_settings(self):
        return self._response.settings if self._response is not None else None

    def export_reference(self):
        if self._response is None:
            raise ValueError("当前数据源未启用触觉响应显示。")
        return self._response.export_reference()

    def retain_reference(self, profile, *, source):
        """Retain a reference whose device/layout scope was checked by its owner."""
        if self._response is None or not isinstance(profile, ReferenceProfile):
            raise ValueError("A validated reference requires response display")
        if source not in ("measured", "saved", "preset", "default"):
            raise ValueError("Invalid retained reference source")
        self._stored_reference, self._stored_reference_source = profile, source

    def configure_response(self, *, display_range_adc=None, direction=None):
        if self._response is None:
            raise ValueError("当前数据源未启用触觉响应显示。")
        previous = self._response.settings
        self._response.configure(display_range_adc=display_range_adc, direction=direction)
        if previous != self._response.settings:
            self._display_generation += 1

    def start_reference(self, *, now_ns=None):
        now = host_clock.monotonic_ns() if now_ns is None else _clock(now_ns)
        state = self.poll(now_ns=now)
        if self._response is None or state.frame is None or self._identity is None:
            raise ValueError("需要新鲜触觉数据才能采集无载参考。")
        self._response.begin_reference(self._response_identity(), now)
        self._display_generation += 1

    def _response_identity(self):
        return (*self._identity, *self._generations)

    def _state(self, frame, status, detail, age_ms=None, receive_hz=0., valid_cells=0):
        timing = None
        if frame is not None and self._sample is not None:
            _, _, blocks, source, snapshot, max_age = self._sample
            timing = TactileTiming(*self._identity, self._sequence, source, snapshot,
                                   tuple(map(int, blocks)), max_age,
                                   *self._generations, self._display_generation)
        return TactileDisplayState(frame, status, detail, age_ms, receive_hz,
                                   valid_cells, self.response_status, timing)

    @staticmethod
    def _valid_cells(mask, blocks, source, snapshot, max_age, now):
        stamps = blocks[_BLOCK_INDEX].reshape(-1)
        return (mask & (stamps > 0) & (stamps <= source) & (stamps <= snapshot)
                & (stamps <= now) & (stamps >= max(0, now - min(max_age, MAX_AGE_NS))))

    def invalidate(self, status, detail, *, reset_reference=False):
        self._display_generation += 1
        self._sample = None
        self._arrivals.clear()
        self._status, self._detail = status, detail
        if self._response is not None:
            if reset_reference:
                self._response.reset(reason="stream_changed")
            else:
                self._response.cancel_reference("stale_reference" if status == "数据过期" else "invalid_sample")
                self._response.clear_activity()

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
            generations = (_clock(getattr(message, "transport_reset_generation", 0)),
                           _clock(getattr(message, "transport_health_generation", 0)))
            if identity != self._identity or generations != self._generations:
                self.invalidate("等待数据", "触觉数据流已切换，等待当前会话数据。", reset_reference=True)
                self._sequence = None
            self._identity, self._generations = identity, generations
            if not isinstance(message.connected, (bool, np.bool_)) or not isinstance(message.time_valid, (bool, np.bool_)):
                raise ValueError("触觉连接或时间状态无效。")
            if not message.connected:
                self.invalidate("连接断开", "右手触觉源已断开，显示已清除。", reset_reference=True)
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
            if self._response is not None:
                valid = self._valid_cells(mask.astype(bool), blocks, source, snapshot, max_age, now)
                if (self._stored_reference is not None and self._response.status.phase == "unreferenced"
                        and np.any(valid)):
                    self._response.load_reference(self._stored_reference, self._response_identity(), now,
                                                  source=self._stored_reference_source)
                self._response.observe(adc, valid, blocks[_BLOCK_INDEX].reshape(-1),
                                       self._response_identity(), now)
            self._arrivals.append(now)
            self._status, self._detail = "实时", "原始 ADC 滚动快照；固定参考手形。"
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self.invalidate("数据无效", str(exc))

    def poll(self, *, now_ns=None):
        now = host_clock.monotonic_ns() if now_ns is None else _clock(now_ns)
        if self._last_poll_ns is not None and now < self._last_poll_ns:
            self.invalidate("数据无效", "本机单调时钟倒退，显示已清除。", reset_reference=True)
        if self._response is not None:
            self._response.poll(now)
        self._last_poll_ns = now
        if self._sample is None:
            return self._state(None, self._status, self._detail)
        adc, mask, blocks, source, snapshot, max_age = self._sample
        age_ms = (now - source) / 1_000_000
        if now < source or now < snapshot or now - source > MAX_AGE_NS or now - snapshot > MAX_AGE_NS:
            self.invalidate("数据过期", "触觉源或快照超过 250 毫秒未更新，显示已清除。")
            return self._state(None, self._status, self._detail, max(0., age_ms))
        valid = self._valid_cells(mask, blocks, source, snapshot, max_age, now)
        count = int(valid.sum())
        while self._arrivals and self._arrivals[0] < now - _RATE_WINDOW_NS:
            self._arrivals.popleft()
        span = self._arrivals[-1] - self._arrivals[0] if len(self._arrivals) > 1 else 0
        rate = (len(self._arrivals) - 1) * 1e9 / span if span > 0 else 0.
        if not count:
            if self._response is not None:
                self._response.cancel_reference("stale_reference")
                self._response.clear_activity()
            return self._state(None, "数据过期", "没有仍在有效时限内的触点，显示已清除。", age_ms, rate)
        frame = {"values": adc.tolist(), "valid_mask": valid.reshape(-1).tolist()}
        detail = f"有效触点 {count}/400；原始 ADC 滚动快照。"
        if self._response is not None:
            frame = self._response.transform(adc, valid, self._response_identity(), now)
            count = sum(frame["valid_mask"])
            detail = "无载参考差值，不代表压力。"
        return self._state(frame, "实时" if count == CELL_COUNT else "部分有效",
                           detail, age_ms, rate, count)
