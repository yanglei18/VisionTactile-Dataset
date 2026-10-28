"""One clock domain for pose/tactile pairing, without a serial device."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_fsglove12 import host_clock


@pytest.mark.parametrize('platform', ['linux', 'win32'])
def test_float_and_integer_clocks_select_the_same_unmodified_domain(monkeypatch, platform):
    readings = SimpleNamespace(monotonic=lambda: 12.125, monotonic_ns=lambda: 12_125_000_000,
                              perf_counter=lambda: 500.5, perf_counter_ns=lambda: 500_500_000_000)
    monkeypatch.setattr(host_clock, 'sys', SimpleNamespace(platform=platform))
    monkeypatch.setattr(host_clock, 'time', readings)
    expected = 500.5 if platform == 'win32' else 12.125
    assert host_clock.monotonic() == expected
    assert host_clock.monotonic_ns() == expected * 1e9


def test_windows_pose_and_tactile_use_qpc_without_offsets_or_timestamp_clamps(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3]
                                  / "ros2_ws/src/vt_tactile_capture"))
    from vt_tactile_capture import host_clock as tactile_clock

    qpc = [90_000_000_001]
    readings = SimpleNamespace(perf_counter=lambda: qpc[0] / 1e9,
                              perf_counter_ns=lambda: qpc[0],
                              monotonic=lambda: 2., monotonic_ns=lambda: 2_000_000_000)
    for clock in (host_clock, tactile_clock):
        monkeypatch.setattr(clock, 'sys', SimpleNamespace(platform='win32'))
        monkeypatch.setattr(clock, 'time', readings)
    for stamp in (90_000_000_001, 90_008_333_334, 89_000_000_000):
        qpc[0] = stamp
        assert host_clock.monotonic_ns() == tactile_clock.monotonic_ns() == stamp
        assert host_clock.monotonic() == stamp / 1e9
