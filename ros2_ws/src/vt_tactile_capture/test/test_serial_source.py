"""Fake serial and private PTYs only: these tests never open physical devices."""

from collections import deque
import builtins
import os
import queue
import struct
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from vt_tactile_capture.protocol import Block
from vt_tactile_capture import serial_source as module
from vt_tactile_capture import host_clock
from vt_tactile_capture.serial_source import MAX_READ_BYTES, SerialWorker


def packet(row=2, group=3):
    return b"\xa5\x5a" + struct.pack(">HBB10H", row, 0xAC, group, *range(10))


class FakeSerial:
    def __init__(self, chunks=()):
        self._chunks = deque(chunks)
        self._lock = threading.Lock()
        self.read_sizes = []
        self.threads = []
        self.completion_clocks = []
        self.closed = False

    def push(self, chunk):
        with self._lock:
            self._chunks.append(chunk)

    @property
    def in_waiting(self):
        self.threads.append(threading.current_thread().name)
        with self._lock:
            if not self._chunks or isinstance(self._chunks[0], Exception):
                return 0
            return len(self._chunks[0])

    def read(self, count):
        self.threads.append(threading.current_thread().name)
        self.read_sizes.append(count)
        with self._lock:
            item = self._chunks.popleft() if self._chunks else b""
            if isinstance(item, Exception):
                raise item
            result, remaining = item[:count], item[count:]
            if remaining:
                self._chunks.appendleft(remaining)
        if not result:
            threading.Event().wait(0.002)
        self.completion_clocks.append((host_clock.realtime_ns(), host_clock.monotonic_ns()))
        return result

    def close(self):
        self.threads.append(threading.current_thread().name)
        self.closed = True

    def write(self, _data):
        raise AssertionError("acquisition must never send firmware commands")


class BlockingFakeSerial(FakeSerial):
    """Model pyserial read(size): accumulate until size bytes or timeout."""

    def __init__(self, timeout, initial=b""):
        super().__init__()
        self.timeout = timeout
        self.buffer = bytearray(initial)
        self.condition = threading.Condition()
        self.read_started = threading.Event()
        self.returned_sizes = []

    @property
    def in_waiting(self):
        with self.condition:
            return len(self.buffer)

    def push(self, chunk):
        with self.condition:
            self.buffer.extend(chunk)
            self.condition.notify_all()

    def read(self, count):
        self.read_sizes.append(count)
        deadline = time.monotonic() + self.timeout
        result = bytearray()
        with self.condition:
            self.read_started.set()
            while len(result) < count:
                take = min(count - len(result), len(self.buffer))
                result.extend(self.buffer[:take])
                del self.buffer[:take]
                remaining = deadline - time.monotonic()
                if len(result) == count or remaining <= 0:
                    break
                self.condition.wait(remaining)
        self.returned_sizes.append(len(result))
        self.completion_clocks.append((host_clock.realtime_ns(), host_clock.monotonic_ns()))
        return bytes(result)


def collect_until(worker, predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    batches = []
    while time.monotonic() < deadline:
        batches.extend(worker.drain())
        if predicate(batches):
            return batches
        threading.Event().wait(0.001)
    raise AssertionError(f"timed out; batches={batches!r}")


def test_one_thread_read_cap_read_completion_timestamps_and_no_writes():
    serial = FakeSerial([packet() * 400])
    opened = []

    def factory(**kwargs):
        opened.append((threading.current_thread().name, kwargs))
        return serial

    worker = SerialWorker("fake:test", serial_factory=factory, no_data_timeout=5)
    worker.start()
    try:
        batches = collect_until(worker, lambda items: sum(len(item.blocks) for item in items) == 400)
        samples = [item for item in batches if item.blocks]
        assert len(samples) == 3
        assert all(0 < count <= MAX_READ_BYTES for count in serial.read_sizes)
        assert opened[0][0] == "tactile-serial"
        assert opened[0][1]["exclusive"] is True
        assert opened[0][1]["port"] == "fake:test"
        assert opened[0][1]["baudrate"] == 115200
        assert samples[0].first_packet_index == 0
        assert samples[1].first_packet_index == len(samples[0].blocks)
        for sample, clocks in zip(samples, serial.completion_clocks):
            assert sample.host_realtime_ns >= clocks[0]
            assert sample.host_monotonic_ns >= clocks[1]
            assert sample.time_valid and sample.connected
            assert sample.clock_domain == "CLOCK_MONOTONIC"
            assert sample.timing_source == "host_receive"
            assert sample.blocks[0].status_raw == 0xAC
    finally:
        assert worker.stop(timeout=1)
    assert serial.closed
    assert set(serial.threads) == {"tactile-serial"}


def test_split_packet_uses_clock_of_completing_read():
    wire = packet()
    serial = FakeSerial([wire[:8], wire[8:]])
    worker = SerialWorker("fake:test", serial_factory=lambda **_: serial)
    worker.start()
    try:
        batches = collect_until(worker, lambda items: any(item.blocks for item in items))
        sample = next(item for item in batches if item.blocks)
        assert sample.host_realtime_ns >= serial.completion_clocks[1][0]
        assert sample.host_monotonic_ns >= serial.completion_clocks[1][1]
        assert len(sample.blocks) == 1
    finally:
        assert worker.stop()


def test_read_target_accumulates_arrivals_in_one_actual_read():
    wire = packet()
    serial = BlockingFakeSerial(timeout=0.1, initial=wire)
    options = []

    def factory(**kwargs):
        options.append(kwargs)
        return serial

    worker = SerialWorker("fake:test", serial_factory=factory,
                          read_batch_bytes=2 * len(wire), read_timeout=serial.timeout)
    worker.start()
    try:
        assert serial.read_started.wait(1)
        serial.push(wire)
        batches = collect_until(worker, lambda items: any(item.blocks for item in items))
        samples = [item for item in batches if item.blocks]
        assert serial.read_sizes[0] == serial.returned_sizes[0] == 2 * len(wire)
        assert len(samples) == 1 and len(samples[0].blocks) == 2
        assert samples[0].host_realtime_ns >= serial.completion_clocks[0][0]
        assert samples[0].host_monotonic_ns >= serial.completion_clocks[0][1]
        assert options[0]["timeout"] == serial.timeout
    finally:
        assert worker.stop(timeout=0.5)


def test_batched_timeout_short_reads_preserve_partial_packet_and_bounded_stop():
    wire = packet()
    serial = BlockingFakeSerial(timeout=0.002, initial=wire + wire[:8])
    worker = SerialWorker("fake:test", serial_factory=lambda **_: serial,
                          read_batch_bytes=1024, read_timeout=serial.timeout)
    worker.start()
    try:
        first = collect_until(worker, lambda items: any(item.blocks for item in items))
        first_sample = next(item for item in first if item.blocks)
        assert serial.read_sizes[0] == 1024
        assert serial.returned_sizes[0] == len(wire) + 8
        assert len(first_sample.blocks) == 1
        serial.push(wire[8:])
        second = collect_until(worker, lambda items: any(item.blocks for item in items))
        second_sample = next(item for item in second if item.blocks)
        assert len(second_sample.blocks) == 1 and second_sample.first_packet_index == 1
        assert second_sample.host_monotonic_ns > first_sample.host_monotonic_ns
        assert second_sample.producer_dropped_total == second_sample.discarded_bytes_total == 0
        assert all(size == 1024 for size in serial.read_sizes)
    finally:
        started = time.monotonic()
        assert worker.stop(timeout=0.5)
        assert time.monotonic() - started < 0.5
    assert serial.closed


@pytest.mark.parametrize("target", [1, 1024, MAX_READ_BYTES])
def test_batched_reads_drain_backlog_without_exceeding_cap(target):
    serial = FakeSerial([packet() * 400])
    worker = SerialWorker("fake:test", serial_factory=lambda **_: serial, read_batch_bytes=target)
    worker.start()
    try:
        batches = collect_until(worker, lambda items: sum(len(item.blocks) for item in items) == 400)
        assert serial.read_sizes[0] == MAX_READ_BYTES
        assert all(target <= size <= MAX_READ_BYTES for size in serial.read_sizes)
        assert sum(len(item.blocks) for item in batches) == 400
    finally:
        assert worker.stop()


def test_queue_is_bounded_and_preserves_latest_fault_and_loss_counters():
    worker = SerialWorker("fake:test", queue_capacity=2, serial_factory=lambda **_: None)
    block = Block(0, 0, 0, (0,) * 10)
    for index in range(4):
        worker._emit(connected=True, reason="ok", blocks=(block,), clocks=(100 + index, 10 + index, True))
    batches = worker.drain()
    assert [batch.sequence for batch in batches] == [2, 3]
    assert [batch.first_packet_index for batch in batches] == [2, 3]
    assert [batch.producer_dropped_total for batch in batches] == [1, 2]
    assert worker.producer_dropped_total == 2
    worker._emit(connected=False, reason="serial_error", clocks=(105, 15, True))
    event = worker.get_nowait()
    assert event.sequence == 4 and event.first_packet_index == 4
    assert not event.blocks and not event.time_valid and not event.connected
    with pytest.raises(queue.Empty):
        worker.get_nowait()


def test_reconnect_increments_epoch_and_discards_partial_old_packet():
    first = FakeSerial([packet(), packet()[:12], OSError("test unplug")])
    second = FakeSerial([packet(5, 7)])
    devices = iter([first, second])
    worker = SerialWorker("fake:test", serial_factory=lambda **_: next(devices), reconnect_interval_sec=0.005)
    worker.start()
    try:
        batches = collect_until(worker, lambda items: any(item.blocks and item.stream_epoch == 2 for item in items))
        samples = [item for item in batches if item.blocks]
        assert [item.stream_epoch for item in samples] == [1, 2]
        assert [item.first_packet_index for item in samples] == [0, 1]
        assert len({item.session_id for item in batches}) == 1
        assert any(not item.connected and "test unplug" in item.reason for item in batches)
        assert samples[-1].discarded_bytes_total == 12
        assert samples[-1].blocks[0].row == 5
    finally:
        assert worker.stop()
    assert first.closed and second.closed


@pytest.mark.parametrize("failure", [PermissionError("permission denied"), OSError("test unplug")])
def test_explicit_connection_mode_never_retries_failed_open(failure):
    calls = []

    def factory(**kwargs):
        calls.append(kwargs)
        raise failure

    worker = SerialWorker("fake:test", serial_factory=factory, auto_reconnect=False,
                          reconnect_interval_sec=.001)
    worker.start()
    events = collect_until(worker, lambda items: any(item.reason == "stopped" for item in items))
    assert worker.stop(timeout=.2)
    assert len(calls) == 1
    assert any(item.reason.startswith("serial_error:") for item in events)
    assert not worker.connected


def test_explicit_connection_mode_never_reopens_after_read_failure():
    serial = FakeSerial([packet(), OSError("test unplug")])
    calls = []

    def factory(**kwargs):
        calls.append(kwargs)
        return serial

    worker = SerialWorker("fake:test", serial_factory=factory, auto_reconnect=False,
                          reconnect_interval_sec=.001)
    worker.start()
    events = collect_until(worker, lambda items: any(item.reason == "stopped" for item in items))
    assert worker.stop(timeout=.2)
    assert len(calls) == 1 and serial.closed
    assert any(item.blocks for item in events)
    assert any("test unplug" in item.reason for item in events)
    assert events[-1].reason == "stopped"


@pytest.mark.parametrize("value", [0, 1, "false", None])
def test_auto_reconnect_requires_boolean(value):
    with pytest.raises(ValueError, match="auto_reconnect"):
        SerialWorker("fake:test", auto_reconnect=value)


def test_no_data_fault_is_empty_and_recovers_on_real_packet():
    serial = FakeSerial()
    worker = SerialWorker("fake:test", serial_factory=lambda **_: serial,
                          read_timeout=0.005, no_data_timeout=0.02)
    worker.start()
    try:
        batches = collect_until(worker, lambda items: any(item.reason == "no_data_timeout" for item in items))
        fault = next(item for item in batches if item.reason == "no_data_timeout")
        assert fault.connected and not fault.time_valid and fault.blocks == ()
        serial.push(packet())
        recovered = collect_until(worker, lambda items: any(item.blocks for item in items))
        sample = next(item for item in recovered if item.blocks)
        assert sample.reason == "ok" and sample.time_valid
    finally:
        assert worker.stop()
    stopped = worker.drain()[-1]
    assert stopped.reason == "stopped" and not stopped.blocks and not stopped.connected


def test_no_data_fault_discards_partial_packet_before_late_tail():
    wire = packet()
    serial = FakeSerial([wire[:8]])
    worker = SerialWorker("fake:test", serial_factory=lambda **_: serial,
                          read_timeout=0.005, no_data_timeout=0.02)
    worker.start()
    try:
        batches = collect_until(worker, lambda items: any(item.reason == "no_data_timeout" for item in items))
        fault = next(item for item in batches if item.reason == "no_data_timeout")
        assert fault.discarded_bytes_total == 8
        serial.push(wire[8:] + packet(5, 7))
        batches = collect_until(worker, lambda items: any(item.blocks for item in items))
        sample = next(item for item in batches if item.blocks)
        assert len(sample.blocks) == 1 and sample.blocks[0].row == 5
        assert sample.discarded_bytes_total == 26
    finally:
        assert worker.stop()


def test_clock_regression_is_flagged(monkeypatch):
    realtimes = iter([100, 99, 101])
    monotimes = iter([10, 11, 12])
    monkeypatch.setattr(host_clock, "realtime_ns", lambda: next(realtimes))
    monkeypatch.setattr(host_clock, "monotonic_ns", lambda: next(monotimes))
    worker = SerialWorker("fake:test")
    assert worker._clock() == (100, 10, True)
    assert worker._clock() == (99, 11, False)
    assert worker._clock() == (101, 12, True)


def test_silent_read_wallclock_step_invalidates_next_completed_batch(monkeypatch):
    realtimes = iter([1000, 6_001_010, 6_001_020])
    monotimes = iter([100, 110, 120])
    monkeypatch.setattr(host_clock, "realtime_ns", lambda: next(realtimes))
    monkeypatch.setattr(host_clock, "monotonic_ns", lambda: next(monotimes))
    worker = SerialWorker("fake:test")
    assert worker._clock()[2]
    assert not worker._clock()[2]
    good_clocks = worker._clock()
    assert good_clocks[2]
    worker._emit(connected=True, reason="ok", blocks=(Block(0, 0, 0, (0,) * 10),), clocks=good_clocks)
    sample = worker.get_nowait()
    assert not sample.time_valid and sample.reason == "clock_step"


def test_stop_interrupts_reconnect_wait_without_unbounded_join():
    def unavailable(**_kwargs):
        raise OSError("fake port unavailable")

    worker = SerialWorker("fake:test", serial_factory=unavailable, reconnect_interval_sec=10)
    worker.start()
    collect_until(worker, lambda items: any("unavailable" in item.reason for item in items))
    started = time.monotonic()
    assert worker.stop(timeout=0.5)
    assert time.monotonic() - started < 0.5


def test_windows_native_open_close_never_imports_posix_or_inspects_proc(monkeypatch):
    serial_module = pytest.importorskip("serial")
    device = FakeSerial()
    options = []

    def open_device(**kwargs):
        options.append(kwargs)
        return device

    original_import = builtins.__import__

    def import_without_posix(name, *args, **kwargs):
        if name in ("fcntl", "termios"):
            raise AssertionError("Windows must not import POSIX serial primitives")
        return original_import(name, *args, **kwargs)

    def no_proc(*args, **kwargs):
        raise AssertionError("Windows must not inspect Linux device paths or /proc")

    monkeypatch.setattr(module, "sys", SimpleNamespace(platform="win32"))
    monkeypatch.setattr(serial_module, "Serial", open_device)
    monkeypatch.setattr(module, "_assert_port_free", no_proc)
    monkeypatch.setattr(builtins, "__import__", import_without_posix)
    worker = SerialWorker("COM12", baudrate=230400, read_timeout=.002)
    assert worker._open_serial() is device
    assert options == [{"port": "COM12", "baudrate": 230400,
                        "timeout": .002, "write_timeout": .002}]
    worker._close_serial(device)
    assert device.closed


def test_windows_native_access_denied_is_terminal_without_implicit_reconnect(monkeypatch):
    serial_module = pytest.importorskip("serial")
    attempts = []

    def denied(**kwargs):
        attempts.append(kwargs)
        raise PermissionError(13, "Access is denied")

    monkeypatch.setattr(module, "sys", SimpleNamespace(platform="win32"))
    monkeypatch.setattr(serial_module, "Serial", denied)
    worker = SerialWorker("COM3", auto_reconnect=False)
    worker.start()
    events = collect_until(worker, lambda items: any(item.reason == "stopped" for item in items))
    assert worker.stop(timeout=.2)
    assert len(attempts) == 1
    assert not any(item.connected for item in events)
    assert any("PermissionError" in item.reason for item in events)


def test_linux_post_open_owner_race_releases_exclusive_mode_and_closes(monkeypatch):
    serial_module = pytest.importorskip("serial")
    fcntl = pytest.importorskip("fcntl")
    termios = pytest.importorskip("termios")
    device = FakeSerial()
    device.fileno = lambda: 42
    checked, locked, ioctls = [], [], []

    def ownership(port, exclude_fd=None):
        checked.append((port, exclude_fd))
        if exclude_fd is not None:
            raise module.PortInUseError("owner appeared during open")

    monkeypatch.setattr(module, "sys", SimpleNamespace(platform="linux"))
    monkeypatch.setattr(serial_module, "Serial", lambda **kwargs: device)
    monkeypatch.setattr(module, "_assert_port_free", ownership)
    monkeypatch.setattr(fcntl, "flock", lambda fd, operation: locked.append((fd, operation)))
    monkeypatch.setattr(fcntl, "ioctl", lambda fd, operation: ioctls.append((fd, operation)))
    worker = SerialWorker("/dev/fake-only")
    with pytest.raises(module.PortInUseError, match="owner appeared"):
        worker._open_serial()
    assert checked == [("/dev/fake-only", None), ("/dev/fake-only", 42)]
    assert locked == [(42, fcntl.LOCK_EX | fcntl.LOCK_NB)]
    assert ioctls == [(42, termios.TIOCEXCL), (42, termios.TIOCNXCL)]
    assert device.closed


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux ownership check")
def test_legacy_nonexclusive_owner_is_rejected_before_opening():
    import pty
    import termios

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    original_attributes = termios.tcgetattr(slave)
    worker = SerialWorker(port, reconnect_interval_sec=0.1)
    worker.start()
    try:
        batches = collect_until(worker, lambda items: any("already in use" in item.reason for item in items))
        assert not any(item.connected for item in batches)
        assert termios.tcgetattr(slave) == original_attributes
    finally:
        assert worker.stop()
        os.close(slave)
        os.close(master)


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="Linux PTY acquisition")
def test_real_pyserial_private_pty_and_second_reader_rejection():
    pytest.importorskip("serial")
    import pty

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    worker = SerialWorker(port, no_data_timeout=1)
    worker.start()
    second = None
    try:
        collect_until(worker, lambda items: any(item.connected for item in items))
        os.write(master, packet(79, 7))
        batches = collect_until(worker, lambda items: any(item.blocks for item in items))
        sample = next(item for item in batches if item.blocks)
        assert sample.blocks[0].row == 79 and sample.blocks[0].column_group == 7
        second = SerialWorker(port, reconnect_interval_sec=0.1)
        second.start()
        rejected = collect_until(second, lambda items: any("already in use" in item.reason for item in items))
        assert not any(item.connected for item in rejected)
    finally:
        if second is not None:
            assert second.stop()
        assert worker.stop()
        os.close(master)


@pytest.mark.parametrize("kwargs", [
    {"port": ""}, {"port": "fake", "baudrate": 0},
    {"port": "fake", "queue_capacity": 0}, {"port": "fake", "read_timeout": 0},
    {"port": "fake", "no_data_timeout": float("nan")},
    {"port": "fake", "reconnect_interval_sec": float("inf")},
    {"port": "fake", "hand": "middle"}, {"port": "fake", "max_clock_step_ns": -1},
    {"port": "fake", "read_batch_bytes": 0}, {"port": "fake", "read_batch_bytes": -1},
    {"port": "fake", "read_batch_bytes": 4097}, {"port": "fake", "read_batch_bytes": True},
    {"port": "fake", "read_batch_bytes": 1024.0}, {"port": "fake", "read_batch_bytes": "1024"},
])
def test_invalid_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        SerialWorker(**kwargs)
