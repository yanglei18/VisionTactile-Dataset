"""Explicit serial ownership for the independent tactile viewer; no ROS or Qt.

Only connect() starts a worker. The shared parser/assembler retain protocol and
loss boundaries; the viewer never sends firmware commands or reconnects itself.
"""
from __future__ import annotations

from collections import deque
from numbers import Integral
from pathlib import Path
import sys

from vt_tactile_capture import host_clock
from vt_tactile_capture.assembler import Assembler
from vt_tactile_capture.serial_source import SerialWorker

from .tactile_source import TactileDisplayModel, TactileDisplayState, _clock


DRAIN_LIMIT = 128
MAX_CELL_AGE_NS = 100_000_000
DISCONNECT_TIMEOUT_S = 0.1
CLOSE_TIMEOUT_S = 1.0
_WAITING_REASONS = {
    "connected_waiting_data": "串口已连接，等待第一组触觉数据。",
    "no_data_timeout": "串口已连接，暂未收到触觉数据。",
    "no_valid_packets": "未收到完整触觉数据包，请检查设备和波特率。",
}
_CLOCK_REASONS = {
    "clock_backwards": "检测到主机时间倒退，当前触觉数据不可用。",
    "clock_step": "检测到主机时钟跳变，当前触觉数据不可用。",
    "nonincreasing_realtime": "主机接收时间未递增，当前触觉数据不可用。",
    "snapshot_clock_backwards": "显示时间早于触觉源时间，当前数据不可用。",
    "invalid_clock_domain": "触觉时钟域不匹配，当前数据不可用。",
    "invalid_timing_source": "触觉时间来源无效，当前数据不可用。",
    "invalid_time": "主机接收时间无效，当前触觉数据不可用。",
}


def list_serial_ports(dev_root=None):
    """List ports without opening them; an explicit root isolates Linux discovery.

    Linux prefers stable device symlinks. Windows uses pyserial's native port
    enumeration, which includes COM numbers and USB device descriptions.
    """
    if dev_root is None and not sys.platform.startswith("linux"):
        return _native_serial_ports()
    root = Path("/dev") if dev_root is None else Path(dev_root)
    choices, targets = [], set()
    candidates = []
    for category in ("by-id", "by-path"):
        try:
            candidates.extend(sorted((root / "serial" / category).iterdir()))
        except OSError:
            pass
    for pattern in ("ttyACM*", "ttyUSB*"):
        try:
            candidates.extend(sorted(root.glob(pattern)))
        except OSError:
            pass
    for path in candidates:
        try:
            target = path.resolve(strict=True)
            if target in targets:
                continue
            targets.add(target)
            label = path.name if path == target else f"{path.name} → {target.name}"
            choices.append({"path": str(path), "label": label})
        except (OSError, RuntimeError):
            continue
    return choices


def _native_serial_ports():
    try:
        from serial.tools import list_ports
        ports = sorted(list_ports.comports())
    except (ImportError, OSError):
        # Manual entry remains available if discovery or its dependency fails.
        return []
    choices, seen = [], set()
    for port in ports:
        path = str(port.device or "").strip()
        key = path.casefold() if sys.platform == "win32" else path
        if not path or key in seen:
            continue
        seen.add(key)
        description = str(port.description or "").strip()
        label = f"{path} · {description}" if description and description != "n/a" else path
        choices.append({"path": path, "label": label})
    return choices


def _serial_error(reason):
    text = reason.lower()
    if any(word in text for word in ("portinuseerror", "already in use", "exclusively lock", "resource temporarily unavailable", "device or resource busy")):
        return "串口已被其他程序占用，请先在该程序中断开连接。"
    if any(word in text for word in ("clearcommerror failed", "getoverlappedresult failed", "winerror 1167")):
        return "串口连接已中断，请检查硬件连接后手动重连。"
    if (any(word in text for word in ("winerror 5", "access is denied", "access denied"))
            or (sys.platform == "win32" and "permissionerror" in text)):
        return "串口被占用或访问被拒绝，请关闭其他使用该串口的程序并检查设备权限。"
    if "permission" in text or "errno 13" in text:
        return "没有串口访问权限，请检查当前用户的设备权限。"
    if "filenotfound" in text or "no such file" in text or "errno 2]" in text:
        return "串口不存在或设备已拔出，请刷新列表并重新选择。"
    if any(word in text for word in ("errno 5", "input/output", "returned no data", "disconnected", "unplug")):
        return "串口连接已中断，请检查硬件连接后手动重连。"
    if "modulenotfounderror" in text and "serial" in text:
        return "缺少串口依赖，请安装 pyserial 后重试。"
    return "串口连接失败，请检查端口、波特率和设备连接；详情可查看诊断。"


class TactileSerialSource:
    """UI-thread facade around one serial worker with bounded shutdown.

    A worker_factory is a test seam. Production uses SerialWorker's exclusive
    ownership checks. A failed connection requires another explicit connect().
    """

    def __init__(self, *, worker_factory=None):
        self._worker_factory = worker_factory or SerialWorker
        self._worker = None
        self._assembler = Assembler(hand="right")
        self._model = TactileDisplayModel()
        self._connection_state = "disconnected"
        self._error = ""
        self._last_observed_source = None
        self._stream_issue = None
        self._diagnostics = deque(maxlen=32)
        self._disconnect_requested = self._close_requested = self._closed = False
        self.port = ""

    @property
    def connection_state(self):
        return self._connection_state

    @property
    def is_connected(self):
        return self._connection_state == "connected"

    @property
    def can_connect(self):
        return not self._close_requested and self._worker is None

    @property
    def can_disconnect(self):
        return self._worker is not None and not self._disconnect_requested

    @property
    def diagnostic_log(self):
        return "\n".join(self._diagnostics)[-8192:]

    def start(self):
        """Prepare the view only. Enumeration/initialization never opens a port."""

    def connect(self, port, baudrate=115200):
        if not self.can_connect:
            raise RuntimeError("请等待当前串口断开后再连接。")
        if not isinstance(port, str) or not port.strip() or "\x00" in port:
            raise ValueError("请选择一个有效串口。")
        if isinstance(baudrate, bool) or not isinstance(baudrate, Integral) or not 1 <= baudrate <= 4_000_000:
            raise ValueError("波特率必须为 1 至 4000000 的整数。")
        self.port = port
        self._assembler = Assembler(hand="right")
        self._model = TactileDisplayModel()
        self._error = ""
        self._last_observed_source = None
        self._stream_issue = None
        self._disconnect_requested = False
        self._connection_state = "connecting"
        try:
            self._worker = self._worker_factory(
                port=port, baudrate=int(baudrate), hand="right", queue_capacity=DRAIN_LIMIT,
                read_batch_bytes=1024, read_timeout=0.002, no_data_timeout=2.0,
                auto_reconnect=False)
            self._worker.start()
        except (OSError, ValueError, RuntimeError, ImportError) as exc:
            self._diagnostics.append(f"{type(exc).__name__}: {exc}"[:1024])
            self._error = _serial_error(self._diagnostics[-1])
            self._connection_state = "error"
            if self._worker is not None and not self._worker.running:
                self._worker = None

    def _release_finished_worker(self):
        if self._worker is None or self._worker.running:
            return
        self._worker.stop(timeout=0)
        self._worker = None
        if self._disconnect_requested:
            self._connection_state = "disconnected"
            if self._close_requested:
                self._closed = True
        else:
            self._connection_state = "error"
            if not self._error:
                self._error = "串口读取已停止，请检查设备后手动连接。"

    def poll(self, now_ns=None):
        # Capture the live clock after draining: a new batch may arrive during
        # the drain. Explicit clocks retain deterministic caller/test semantics.
        now = None if now_ns is None else _clock(now_ns)
        finished_before_drain = self._worker is not None and not self._worker.running
        batches = self._worker.drain(limit=DRAIN_LIMIT) if self._worker is not None else []
        if now is None:
            now = host_clock.monotonic_ns()
        if not self._disconnect_requested:
            for batch in batches:
                self._assembler.apply(batch)
                # Inspect each applied boundary, even if a healthy batch later
                # in this drain supersedes it. Only the final snapshot renders.
                checkpoint = self._assembler.snapshot(now, MAX_CELL_AGE_NS)
                if (not checkpoint.connected or not checkpoint.time_valid
                        or checkpoint.reason not in ("ok", "stale")):
                    self._model.invalidate("等待数据", "触觉数据流已中断，等待新的有效数据。")
                issue_reason = (batch.reason if not batch.time_valid and batch.reason in _CLOCK_REASONS
                                else checkpoint.reason)
                if issue_reason in _WAITING_REASONS:
                    self._stream_issue = ("等待数据", _WAITING_REASONS[issue_reason])
                elif issue_reason in _CLOCK_REASONS:
                    self._stream_issue = ("数据无效", _CLOCK_REASONS[issue_reason])
                    self._diagnostics.append(issue_reason)
                else:
                    self._stream_issue = None
                if batch.reason.startswith("serial_error:"):
                    self._diagnostics.append(batch.reason[:1024])
                    self._error = _serial_error(batch.reason)
                if batch.connected and not self._error:
                    self._connection_state = "connected"
                if batch.reason in ("no_data_timeout", "no_valid_packets"):
                    self._diagnostics.append(batch.reason)
            if batches:
                snapshot = self._assembler.snapshot(now, MAX_CELL_AGE_NS)
                source_key = (snapshot.session_id, snapshot.stream_epoch, snapshot.host_monotonic_ns)
                if (source_key != self._last_observed_source or not snapshot.connected
                        or not snapshot.time_valid):
                    self._model.observe(snapshot, now_ns=now)
                    self._last_observed_source = source_key
        # If the worker ended during this drain, its final error/stopped events
        # may still be queued. Keep it until the next bounded drain sees them.
        if finished_before_drain:
            self._release_finished_worker()
        if self._disconnect_requested:
            if self._worker is not None:
                self._worker.stop(timeout=0)
                self._release_finished_worker()
            if self._worker is not None:
                return TactileDisplayState(None, "正在断开", "正在等待串口读取线程退出。")
            return TactileDisplayState(None, "已关闭" if self._closed else "未连接", "请选择串口并连接。")
        if self._error:
            self._connection_state = "error"
            return TactileDisplayState(None, "连接失败", self._error)
        if self._connection_state == "disconnected":
            return TactileDisplayState(None, "未连接", "请选择右手触觉串口并连接。")
        if self._connection_state == "connecting":
            return TactileDisplayState(None, "正在连接", "正在打开所选串口。")
        if self._stream_issue is not None:
            return TactileDisplayState(None, *self._stream_issue)
        return self._model.poll(now_ns=now)

    def _disconnect(self, timeout):
        self._disconnect_requested = True
        self._model.invalidate("未连接", "串口显示已清除。")
        if self._worker is None:
            self._connection_state = "disconnected"
            return True
        self._connection_state = "disconnecting"
        self._worker.stop(timeout=timeout)
        self._release_finished_worker()
        return self._worker is None

    def disconnect(self):
        """Request stop, waiting at most 0.1 s; poll finishes deferred cleanup."""
        return self._disconnect(DISCONNECT_TIMEOUT_S)

    def close(self):
        """Wait at most one second; False keeps the window able to finish polling."""
        self._close_requested = True
        self._closed = self._disconnect(CLOSE_TIMEOUT_S)
        return self._closed
