from types import SimpleNamespace as N
from dataclasses import replace

import pytest

from pathlib import Path
from test_gui_live import timing, tracker
from vt_tracker_camera_calib.config import load_config

@pytest.fixture
def config():
    return load_config(Path(__file__).resolve().parents[1] / 'config/calibration.example.yaml')

def timing_message(config):
    value = timing(config)
    value.group_host_monotonic_raw_ns = 100
    return value

def tracker_message(config):
    value = tracker(config)
    value.host_monotonic_ns = 100
    return value
from vt_tracker_camera_calib.guided_live import GuidedMonitor


def test_input_identity_change_is_terminal(config):
    monitor = GuidedMonitor(config)
    first = tracker_message(config)
    monitor.on_tracker(first)
    second = tracker_message(config)
    second.tracker_id = 'b' * 64
    second.host_realtime_ns += 10_000_000
    second.host_monotonic_ns += 10_000_000
    monitor.on_tracker(second)
    assert 'identity' in monitor.error.lower()


def test_timing_conflict_and_clock_step_are_terminal(config):
    monitor = GuidedMonitor(config)
    first = timing_message(config)
    monitor.on_timing(first)
    second = timing_message(config)
    second.group_host_realtime_ns += 1
    monitor.on_timing(second)
    assert 'conflicting' in monitor.error


def test_missing_monotonic_clock_cannot_be_accepted(config):
    monitor = GuidedMonitor(config)
    message = tracker_message(config)
    message.host_monotonic_ns = 0
    monitor.on_tracker(message)
    assert 'positive' in monitor.error


def test_no_fresh_input_yields_no_preview(config):
    monitor = GuidedMonitor(config)
    snapshot = monitor.snapshot()
    assert snapshot['preview'] is None
    assert snapshot['ready'] is False
    assert snapshot['accepted'] == 0


def test_blocked_native_detector_does_not_block_cancel(config, monkeypatch):
    import threading
    import time
    import vt_tracker_camera_calib.guided_live as live
    from test_gui_live import fake_ros, image, info
    entered, release = threading.Event(), threading.Event()
    def blocked(*args, **kwargs):
        entered.set()
        release.wait(5)
        return None
    ros, _ = fake_ros()
    monkeypatch.setattr(live, 'load_ros', lambda: ros)
    monkeypatch.setattr(live, 'detect_board_pose', blocked)
    monitor = GuidedMonitor(config)
    monitor.start()
    try:
        monitor.on_info(info(config))
        monitor.on_timing(timing_message(config))
        monitor.on_image(image(config))
        started = time.monotonic()
        monitor.request_detection()
        assert time.monotonic() - started < .1
        assert entered.wait(1)
        with pytest.raises(RuntimeError, match='vision'):
            monitor.stop(timeout_s=.05)
        assert time.monotonic() - started < 1
    finally:
        release.set()
        monitor.stop()
