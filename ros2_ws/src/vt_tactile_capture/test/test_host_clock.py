"""Precise Windows receive clocks and unchanged POSIX clock behavior."""
import ctypes
import sys
import time
from types import SimpleNamespace

import pytest

from vt_tactile_capture import host_clock
from vt_tactile_capture.protocol import Block
from vt_tactile_capture.serial_source import SerialWorker


@pytest.fixture(autouse=True)
def isolated_windows_api_cache():
    host_clock._windows_utc_reader.cache_clear()
    yield
    host_clock._windows_utc_reader.cache_clear()


@pytest.mark.parametrize("platform,version,expected_mono", [
    ("linux", (3, 12), 202), ("linux", (3, 13), 202),
    ("win32", (3, 13), 303), ("win32", (3, 14), 303),
])
def test_native_utc_and_platform_monotonic_selection(monkeypatch, platform, version, expected_mono):
    monkeypatch.setattr(host_clock, "sys", SimpleNamespace(platform=platform, version_info=version))
    monkeypatch.setattr(host_clock, "time", SimpleNamespace(
        time_ns=lambda: 101, monotonic_ns=lambda: 202, perf_counter_ns=lambda: 303))
    assert host_clock.realtime_ns() == 101
    assert host_clock.monotonic_ns() == expected_mono


def test_windows_filetime_binding_integer_conversion_and_no_shared_buffer(monkeypatch):
    expected = [-100, 0, 100, 1_700_000_000_123_456_700, 1_700_000_000_123_456_800]
    ticks = iter(host_clock._FILETIME_UNIX_EPOCH_TICKS + value // 100 for value in expected)
    buffers, libraries = [], []

    class PreciseApi:
        def __call__(self, pointer):
            value = next(ticks)
            stamp = pointer._obj
            stamp.dwLowDateTime = value & 0xFFFF_FFFF
            stamp.dwHighDateTime = value >> 32
            buffers.append(stamp)

    api = PreciseApi()

    def library(name, **kwargs):
        libraries.append((name, kwargs))
        return SimpleNamespace(GetSystemTimePreciseAsFileTime=api)

    def no_coarse_clock():
        pytest.fail("Windows 3.12 must not use its coarse standard clocks")

    monkeypatch.setattr(ctypes, "WinDLL", library, raising=False)
    monkeypatch.setattr(host_clock, "sys", SimpleNamespace(platform="win32", version_info=(3, 12)))
    monkeypatch.setattr(host_clock, "time", SimpleNamespace(
        time_ns=no_coarse_clock, monotonic_ns=no_coarse_clock, perf_counter_ns=lambda: 123))
    assert [host_clock.realtime_ns() for _ in expected] == expected
    assert host_clock.monotonic_ns() == 123
    assert libraries == [("kernel32", {"use_last_error": True})]
    assert ctypes.sizeof(api.argtypes[0]._type_) == 8
    assert api.restype is None
    assert len({id(buffer) for buffer in buffers}) == len(expected)


def test_missing_precise_api_never_silently_falls_back_to_coarse_clock(monkeypatch):
    def missing(*args, **kwargs):
        raise OSError("precise clock unavailable")

    monkeypatch.setattr(ctypes, "WinDLL", missing, raising=False)
    monkeypatch.setattr(host_clock, "sys", SimpleNamespace(platform="win32", version_info=(3, 12)))
    with pytest.raises(OSError, match="precise clock unavailable"):
        host_clock.realtime_ns()


@pytest.mark.parametrize("utc_delta,mono_delta,reason", [
    (2_000_000, 2_000_000, ""),
    (8_000_001, 2_000_000, "clock_step"),
    (-2_000_000, 2_000_000, "clock_backwards"),
    (2_000_000, 0, "clock_backwards"),
])
def test_millisecond_reads_use_precise_clocks_and_still_detect_real_faults(
        monkeypatch, utc_delta, mono_delta, reason):
    clocks = [1_700_000_000_000_000_000, 10_000_000_000]
    monkeypatch.setattr(host_clock, "sys", SimpleNamespace(platform="win32", version_info=(3, 12)))
    monkeypatch.setattr(host_clock, "_windows_utc_reader", lambda: lambda: clocks[0])
    # These coarse APIs intentionally never advance, reproducing Windows 3.12
    # within a timer tick. The worker must sample precise UTC/QPC instead.
    monkeypatch.setattr(host_clock, "time", SimpleNamespace(
        time_ns=lambda: 7, monotonic_ns=lambda: 8, perf_counter_ns=lambda: clocks[1]))
    worker = SerialWorker("COM12")
    for _ in range(20):
        assert worker._clock() == (*clocks, True)
        clocks[0] += 2_000_000
        clocks[1] += 2_000_000
    clocks[0] += utc_delta - 2_000_000
    clocks[1] += mono_delta - 2_000_000
    final = worker._clock()
    assert final == (*clocks, not reason)
    assert worker._pending_clock_fault == reason
    worker._emit(connected=True, reason="ok", blocks=(Block(0, 0, 0, (0,) * 10),), clocks=final)
    sample = worker.get_nowait()
    assert sample.time_valid == (not reason)
    assert sample.reason == (reason or "ok")
    assert sample.host_realtime_ns == clocks[0]
    assert sample.host_monotonic_ns == clocks[1]


@pytest.mark.skipif(sys.platform != "win32", reason="Native Windows clock ABI smoke")
def test_native_windows_clock_readings_have_submillisecond_resolution():
    # No sleeps/timer-resolution requests: coarse Windows clocks would return
    # repeated values here, whereas precise UTC and QPC distinguish reads.
    samples = [(host_clock.realtime_ns(), host_clock.monotonic_ns()) for _ in range(256)]
    assert abs(samples[-1][0] - time.time_ns()) < 1_000_000_000
    for column in (0, 1):
        values = [sample[column] for sample in samples]
        deltas = [later - earlier for earlier, later in zip(values, values[1:])]
        assert sum(0 < delta < 1_000_000 for delta in deltas) > 128
