"""Bounded receive-health evidence and conservative frozen-bank detection."""
from copy import deepcopy

import pytest

from vt_fsglove12.config import load_config, validate_config
from vt_fsglove12.input_health import InputHealthMonitor, health_settings


def feed(monitor, index, stamp, *, frozen=False, quiet=False):
    events = []
    for node in range(1, 13):
        change = 0 if frozen and node >= 7 or quiet else index % 100
        values = (0 if quiet else 1000, change, -1000 if not quiet else 0,
                  100 + change, 20, 8197)
        events.extend(monitor.observe(node, values, 0, stamp,
                                     (0., 0., 0.) if quiet else (8.75, 0., -8.75)))
    return events


def test_explicit_configuration_and_absent_legacy_policy():
    assert health_settings({})["enabled"] is False
    config = load_config()
    assert config["input_health"]["enabled"] is True
    assert validate_config(config)["input_health"] == config["input_health"]
    legacy = deepcopy(config)
    del legacy["input_health"]
    assert "input_health" not in validate_config(legacy)


@pytest.mark.parametrize("options", [None, {"unknown": 1}, {"enabled": 1},
    {"rate_window_s": 0}, {"rate_window_s": float("inf")},
    {"rate_min_ratio": 1.1}, {"rate_max_ratio": .9}, {"frozen_min_samples": 1},
    {"recovery_min_changes": 2.5}, {"peer_min_changes": True}])
def test_rejects_invalid_policy(options):
    with pytest.raises(ValueError, match="input_health"):
        health_settings({"input_health": options})


@pytest.mark.parametrize("batch", [1, 6, 12, 24])
def test_120hz_batched_arrivals_are_counted_without_per_gap_rate_guess(batch):
    monitor = InputHealthMonitor({"input_health": {"enabled": True}})
    events = []
    for index in range(720):
        stamp = 10. + (index // batch) * batch / 120.
        events.extend(feed(monitor, index, stamp))
    assert events == []
    status = monitor.snapshot(stamp)
    assert not status["faulted_nodes"]
    for node in status["nodes"].values():
        assert node["state"] == "healthy"
        assert node["observed_receive_rate_hz"] == pytest.approx(120.)
        assert node["samples"] == 720
    assert all(len(node.groups) <= 8192 for node in monitor.nodes.values())


def test_six_hz_fault_is_sustained_latched_and_snapshot_does_not_advance():
    monitor = InputHealthMonitor({"input_health": {"enabled": True}})
    events = []
    for index in range(37):
        stamp = 10. + index / 6.
        events.extend(feed(monitor, index, stamp))
        if stamp < 13.:
            assert not monitor.snapshot(stamp)["faulted_nodes"]
    assert sorted(events) == [("fault", str(node)) for node in sorted(range(1, 13), key=str)]
    status = monitor.snapshot(stamp)
    assert len(status["faulted_nodes"]) == 12
    assert all(node["reasons"] == ["receive_rate_mismatch"] for node in status["nodes"].values())
    assert all(node["observed_receive_rate_hz"] == pytest.approx(6.) for node in status["nodes"].values())
    original = monitor.snapshot(stamp)
    status["nodes"]["1"]["reasons"].clear()
    monitor.snapshot(stamp + 100.)
    assert monitor.snapshot(stamp) == original


def test_stationary_equal_bank_payloads_are_not_frozen_evidence():
    monitor = InputHealthMonitor({"input_health": {"enabled": True}})
    for index in range(600):
        feed(monitor, index, 10. + index / 120., quiet=True)
    assert not monitor.snapshot(10. + 599 / 120.)["faulted_nodes"]
    # Even while the other bank advances, zero-rate repeated resting payloads
    # are insufficient evidence of a frozen acquisition bank.
    for index in range(600, 1200):
        stamp = 10. + index / 120.
        for node in range(1, 13):
            change = index % 100 if node <= 6 else 0
            monitor.observe(node, (0, 0, 0, change, 0, 8197), 0, stamp, (0., 0., 0.))
    assert not monitor.snapshot(stamp)["faulted_nodes"]


def test_nonzero_repeated_banks_without_advancing_peer_are_not_called_frozen():
    monitor = InputHealthMonitor({"input_health": {"enabled": True}})
    for index in range(480):
        feed(monitor, 0, 10. + index / 120.)
    assert not monitor.snapshot(10. + 479 / 120.)["faulted_nodes"]


def test_frozen_fault_recovers_only_after_sustained_changes_and_cadence():
    monitor = InputHealthMonitor({"input_health": {"enabled": True}})
    events = []
    for index in range(480):
        events.extend(feed(monitor, index, 10. + index / 120., frozen=True))
    assert set(events) == {("fault", str(node)) for node in range(7, 13)}
    for index in range(480, 540):
        events = feed(monitor, index, 10. + index / 120.)
        assert not any(event == "recovered" for event, _ in events)
    for index in range(540, 720):
        feed(monitor, index, 10. + index / 120.)
    assert not monitor.snapshot(10. + 719 / 120.)["faulted_nodes"]
    assert all(node.faults == (1 if int(key) >= 7 else 0) for key, node in monitor.nodes.items())
