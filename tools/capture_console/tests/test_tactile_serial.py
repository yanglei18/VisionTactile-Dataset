"""Serial-viewer lifecycle and full shared parser path with fake ports only."""
from collections import deque
from dataclasses import replace
import struct
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from vt_capture_console.backend import tactile_serial as module
from vt_tactile_capture.protocol import Block
from vt_tactile_capture.serial_source import Batch, SerialWorker


NOW = 10_000_000_000


def batch(sequence=0, stamp=NOW, *, row=0, group=0, **changes):
    value = Batch(hand="right", session_id="test", stream_epoch=1, sequence=sequence,
                  first_packet_index=sequence, host_monotonic_ns=stamp,
                  host_realtime_ns=stamp + 100_000_000_000,
                  connected=True, time_valid=True, reason="ok",
                  blocks=(Block(row, group, 0, tuple(range(10))),))
    return replace(value, **changes)


class Worker:
    def __init__(self, **kwargs):
        self.options = kwargs
        self.queue = deque()
        self.running = False
        self.stop_calls = []
        self.limits = []
        self.stuck = False

    def start(self):
        self.running = True

    def drain(self, limit):
        self.limits.append(limit)
        return [self.queue.popleft() for _ in range(min(limit, len(self.queue)))]

    def stop(self, timeout):
        self.stop_calls.append(timeout)
        if not self.stuck:
            self.running = False
        return not self.running


def source_with_worker():
    source = module.TactileSerialSource(worker_factory=Worker)
    source.connect("/dev/test-only")
    return source, source._worker


def test_initialization_and_start_do_not_construct_worker_or_connect():
    calls = []
    source = module.TactileSerialSource(worker_factory=lambda **kwargs: calls.append(kwargs))
    source.start()
    assert calls == [] and source.can_connect and not source.can_disconnect
    assert source.connection_state == "disconnected"
    assert source.poll(NOW).status == "未连接"
    assert source.close() and source.close()
    assert not source.can_connect


def test_explicit_connect_options_and_no_second_port_until_disconnected():
    source, worker = source_with_worker()
    assert worker.options["port"] == "/dev/test-only"
    assert worker.options["auto_reconnect"] is False
    assert worker.options["read_timeout"] == .002
    assert source.connection_state == "connecting"
    with pytest.raises(RuntimeError):
        source.connect("/dev/must-not-open")
    worker.queue.append(batch())
    assert source.poll(NOW).valid_cells == 3
    assert source.is_connected
    assert source.disconnect()
    assert source.connection_state == "disconnected" and source.can_connect
    source.connect("/dev/second-test")
    assert source._worker is not worker
    assert source.poll(NOW).frame is None
    source.close()


def test_ordered_drain_preserves_hidden_health_boundary_and_does_not_refresh_old_data():
    source, worker = source_with_worker()
    worker.queue.append(batch())
    assert source.poll(NOW).valid_cells == 3
    worker.queue.extend([
        batch(1, NOW + 10_000_000, connected=False, time_valid=False, blocks=(), reason="no_data_timeout"),
        batch(2, NOW + 20_000_000, first_packet_index=1, row=1),
    ])
    view = source.poll(NOW + 20_000_000)
    assert view.valid_cells == 3 and view.receive_hz == 0
    assert not any(view.frame["valid_mask"][:20])
    assert view.frame["valid_mask"][20]
    snapshot_time = source._model._sample[4]
    assert source.poll(NOW + 30_000_000).receive_hz == 0
    assert source._model._sample[4] == snapshot_time
    assert source.poll(NOW + 120_000_001).frame is None
    assert all(limit == module.DRAIN_LIMIT for limit in worker.limits)
    source.close()


def test_nonphysical_packet_does_not_refresh_physical_source_or_hz():
    source, worker = source_with_worker()
    worker.queue.append(batch())
    source.poll(NOW)
    worker.queue.append(batch(1, NOW + 20_000_000, row=21))
    current = source.poll(NOW + 20_000_000)
    assert current.age_ms == 20 and current.receive_hz == 0
    assert source._model._sample[4] == NOW
    source.close()


@pytest.mark.parametrize("reason,detail", [
    ("connected_waiting_data", "第一组"),
    ("no_data_timeout", "暂未收到"),
    ("no_valid_packets", "波特率"),
])
def test_waiting_health_status_persists_across_empty_polls_and_recovers(reason, detail):
    source, worker = source_with_worker()
    worker.queue.append(batch(0, blocks=(), time_valid=False, reason=reason))
    for offset in (0, 1_000_000, 300_000_000):
        view = source.poll(NOW + offset)
        assert view.status == "等待数据" and detail in view.detail and view.frame is None
        assert source.is_connected
    worker.queue.append(batch(1, NOW + 310_000_000, first_packet_index=0))
    view = source.poll(NOW + 310_000_000)
    assert view.valid_cells == 3 and view.status == "部分有效"
    source.close()


def test_clock_fault_keeps_distinct_explanation_until_valid_input():
    source, worker = source_with_worker()
    worker.queue.append(batch(0, time_valid=False, reason="clock_step"))
    assert "时钟跳变" in source.poll(NOW).detail
    assert "时钟跳变" in source.poll(NOW + 1_000_000).detail
    worker.queue.append(batch(1, NOW + 20_000_000))
    assert source.poll(NOW + 20_000_000).valid_cells == 3
    source.close()


@pytest.mark.parametrize("reason,text", [
    ("serial_error: PortInUseError: busy", "占用"),
    ("serial_error: PermissionError: [Errno 13]", "权限"),
    ("serial_error: FileNotFoundError: No such file", "不存在"),
    ("serial_error: OSError: [Errno 5] Input/output error", "中断"),
])
def test_terminal_error_survives_following_stopped_event(reason, text):
    source, worker = source_with_worker()
    worker.queue.extend([batch(0, blocks=(), connected=False, time_valid=False, reason=reason),
                         batch(1, NOW + 1, first_packet_index=0, blocks=(), connected=False,
                               time_valid=False, reason="stopped")])
    worker.running = False
    result = source.poll(NOW + 1)
    assert text in result.detail and result.frame is None
    assert source.connection_state == "error" and source.can_connect
    assert reason in source.diagnostic_log
    assert text in source.poll(NOW + 100).detail
    source.close()


def test_worker_finishing_during_drain_is_kept_until_terminal_events_are_read():
    source, worker = source_with_worker()
    original = worker.drain

    def drain(limit):
        result = original(limit)
        worker.running = False
        worker.queue.append(batch(0, blocks=(), connected=False, time_valid=False,
                                  reason="serial_error: PermissionError: denied"))
        worker.drain = original
        return result

    worker.drain = drain
    source.poll(NOW)
    assert source._worker is worker
    assert "权限" in source.poll(NOW).detail
    assert source._worker is None
    source.close()


@pytest.mark.parametrize("injected_now", [None, NOW])
def test_poll_uses_live_clock_after_drain(injected_now, monkeypatch):
    source, worker = source_with_worker()
    clock = [NOW]
    monkeypatch.setattr(module.host_clock, "monotonic_ns", lambda: clock[0])

    def drain(limit):
        clock[0] += 1_000_000
        return [batch(stamp=clock[0])]

    worker.drain = drain
    result = source.poll(injected_now)
    assert (result.frame is not None) == (injected_now is None)
    source.close()


def test_windows_120hz_samples_and_display_freshness_share_precise_clock_domain(monkeypatch):
    from vt_capture_console.backend.tactile_source import TactileDisplayModel

    clock = module.host_clock
    now = [NOW]
    monkeypatch.setattr(clock, "sys", SimpleNamespace(platform="win32", version_info=(3, 12)))
    monkeypatch.setattr(clock, "_windows_utc_reader", lambda: lambda: 1_700_000_000_000_000_000 + now[0])
    # Deliberately incompatible, stagnant coarse times must not enter either
    # the producer timestamps or presentation/future/staleness comparisons.
    monkeypatch.setattr(clock, "time", SimpleNamespace(
        time_ns=lambda: 1, monotonic_ns=lambda: 2, perf_counter_ns=lambda: now[0]))
    source = module.TactileSerialSource(worker_factory=Worker)
    source.connect("COM12")
    producer = SerialWorker("COM12")
    blocks = tuple(Block(row, group, 0, tuple(row * 100 + group + 8 * k for k in range(10)))
                   for row in range(20) for group in range(8))
    try:
        for index in range(10):
            producer._emit(connected=True, reason="ok", blocks=blocks)
            produced = producer.get_nowait()
            assert produced.time_valid and produced.host_monotonic_ns == now[0]
            source._worker.queue.append(produced)
            view = source.poll()
            assert view.valid_cells == 400 and view.age_ms == 0
            assert view.status == "实时"
            if index:
                assert view.receive_hz == pytest.approx(120, abs=.01)
            if index < 9:
                now[0] += 8_333_400

        standalone_model = TactileDisplayModel()
        standalone_model.observe(source._assembler.snapshot(now[0], module.MAX_CELL_AGE_NS))
        assert standalone_model.poll().valid_cells == 400
        now[0] += 20_000_000
        assert source.poll().age_ms == standalone_model.poll().age_ms == 20
        now[0] += 80_000_001
        assert source.poll().frame is None
        assert standalone_model.poll().frame is None
        now[0] += 150_000_000
        assert source.poll().status == standalone_model.poll().status == "数据过期"
    finally:
        assert source.close()


def test_bounded_disconnect_defers_reaping_and_blocks_reconnect():
    source, worker = source_with_worker()
    worker.stuck = True
    assert source.disconnect() is False
    assert worker.stop_calls == [module.DISCONNECT_TIMEOUT_S]
    assert source.connection_state == "disconnecting" and not source.can_connect
    assert source.poll(NOW).status == "正在断开"
    assert source.close() is False
    assert max(worker.stop_calls) <= 1
    worker.running = False
    source.poll(NOW)
    assert source.close() is True
    assert not source.can_connect and source.connection_state == "disconnected"


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux device symlink discovery")
def test_enumeration_prefers_stable_names_without_constructing_serial(tmp_path):
    root = tmp_path / "dev"
    (root / "serial/by-id").mkdir(parents=True)
    (root / "serial/by-path").mkdir(parents=True)
    (root / "ttyACM0").touch()
    (root / "ttyUSB1").touch()
    (root / "serial/by-id/stable").symlink_to(root / "ttyACM0")
    (root / "serial/by-path/same").symlink_to(root / "ttyACM0")
    (root / "serial/by-id/missing").symlink_to(root / "absent")
    choices = module.list_serial_ports(root)
    assert [item["path"] for item in choices] == [str(root / "serial/by-id/stable"), str(root / "ttyUSB1")]


def test_windows_enumeration_uses_com_numbers_descriptions_and_deduplicates(monkeypatch):
    import serial
    from serial.tools import list_ports
    from serial.tools.list_ports_common import ListPortInfo

    def port(device, description):
        result = ListPortInfo(device)
        result.description = description
        return result

    def no_open(**kwargs):
        raise AssertionError("enumeration must not open a serial port")

    monkeypatch.setattr(module, "sys", SimpleNamespace(platform="win32"))
    monkeypatch.setattr(serial, "Serial", no_open)
    monkeypatch.setattr(list_ports, "comports", lambda: [
        port("COM12", "USB Serial Device"), port("COM3", "触觉设备"),
        port("com3", "duplicate"), port("COM2", "n/a"),
    ])
    assert module.list_serial_ports() == [
        {"path": "COM2", "label": "COM2"},
        {"path": "COM3", "label": "COM3 · 触觉设备"},
        {"path": "COM12", "label": "COM12 · USB Serial Device"},
    ]


def test_windows_discovery_failure_leaves_manual_entry_available(monkeypatch):
    from serial.tools import list_ports

    def fail():
        raise OSError("device enumeration unavailable")

    monkeypatch.setattr(module, "sys", SimpleNamespace(platform="win32"))
    monkeypatch.setattr(list_ports, "comports", fail)
    assert module.list_serial_ports() == []
    source = module.TactileSerialSource(worker_factory=Worker)
    source.connect("COM7")
    assert source._worker.options["port"] == "COM7"
    assert source.close()


@pytest.mark.parametrize("reason,text", [
    ("serial_error: SerialException: could not open port 'COM3': PermissionError(13, '拒绝访问。', None, 5)", "占用或访问被拒绝"),
    ("serial_error: SerialException: could not open port 'COM3': FileNotFoundError(2, '系统找不到指定的文件。', None, 2)", "不存在"),
    ("serial_error: SerialException: ClearCommError failed (PermissionError(13, '设备不识别此命令。', None, 22))", "中断"),
    ("serial_error: SerialException: GetOverlappedResult failed (OSError(22, '设备未连接。', None, 1167))", "中断"),
])
def test_windows_serial_failures_offer_actionable_localized_guidance(monkeypatch, reason, text):
    monkeypatch.setattr(module, "sys", SimpleNamespace(platform="win32"))
    source, worker = source_with_worker()
    worker.queue.extend([
        batch(0, blocks=(), connected=False, time_valid=False, reason=reason),
        batch(1, NOW + 1, first_packet_index=0, blocks=(), connected=False,
              time_valid=False, reason="stopped"),
    ])
    worker.running = False
    view = source.poll(NOW + 1)
    assert text in view.detail and view.frame is None
    assert source.can_connect and source.connection_state == "error"
    assert reason in source.diagnostic_log
    assert source.close()


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = deque(chunks)
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, count):
        if self.chunks:
            data = self.chunks.popleft()
            if len(data) > count:
                self.chunks.appendleft(data[count:])
            return data[:count]
        threading.Event().wait(.002)
        return b""

    def close(self):
        self.closed = True

    def write(self, *_):
        raise AssertionError("viewer must not send firmware commands")


def test_shared_serial_parser_assembler_path_exact_400_values_and_release():
    payload = bytearray()
    for row in range(20):
        for group in range(8):
            values = [row * 100 + group + 8 * k for k in range(10)]
            payload.extend(b"\xa5\x5a" + struct.pack(">HBB10H", row, 0, group, *values))
    serial = FakeSerial([bytes(payload[:7]), bytes(payload[7:])])
    opened = []

    def worker_factory(**kwargs):
        def factory(**options):
            opened.append(options)
            return serial
        return SerialWorker(**kwargs, serial_factory=factory)

    source = module.TactileSerialSource(worker_factory=worker_factory)
    source.start()
    assert not opened
    source.connect("fake:only")
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            view = source.poll()
            if view.valid_cells == 400:
                break
            threading.Event().wait(.001)
        assert view.valid_cells == 400
        assert view.frame["values"] == [row * 100 + col for row in range(20) for col in range(20)]
        assert len(opened) == 1 and opened[0]["exclusive"] is True
    finally:
        assert source.close()
    assert serial.closed
