"""Background discovery with metadata-only providers; no device access or Qt."""
from dataclasses import FrozenInstanceError
import threading
import time

import pytest

from vt_hand_tactile_viewer.serial_discovery import SerialPortDiscovery
from vt_hand_tactile_viewer.port_probe import ProbeResult


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    pytest.fail("Discovery did not reach the expected state")


def test_periodic_discovery_observes_insertion_and_removal_without_manual_requests():
    present = []
    discovery = SerialPortDiscovery(lambda: list(present), interval_s=.02)
    discovery.start()
    try:
        present.append(dict(path="COM4", label="Glove"))
        wait_for(lambda: discovery.latest().choices == (("COM4", "Glove"),))
        present.clear()
        wait_for(lambda: discovery.latest().choices == ())
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_snapshot_owns_immutable_sorted_choices_and_deduplicates_paths():
    entries = [dict(path="COM7", label="Touch"), dict(path="COM2", label="Glove"),
               dict(path="COM7", label="duplicate"), dict(path="", label="missing")]
    discovery = SerialPortDiscovery(lambda: entries, interval_s=10)
    discovery.start()
    try:
        wait_for(lambda: len(discovery.latest().choices) == 2)
        snapshot = discovery.latest()
        assert snapshot.choices == (("COM2", "Glove"), ("COM7", "Touch"))
        entries[0]["label"] = "changed outside the worker"
        assert snapshot.choices[1] == ("COM7", "Touch")
        with pytest.raises(FrozenInstanceError):
            snapshot.error = "cannot mutate"
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_blocked_scan_does_not_block_api_or_spawn_parallel_or_queued_scans():
    entered, release, second = threading.Event(), threading.Event(), threading.Event()
    calls = []

    def provider():
        calls.append(threading.get_ident())
        if len(calls) == 1:
            entered.set()
            assert release.wait(2)
        else:
            second.set()
        return [dict(path="COM4", label="Glove")]

    discovery = SerialPortDiscovery(provider, interval_s=10)
    discovery.start()
    discovery.start()
    try:
        assert entered.wait(1)
        began = time.monotonic()
        for _ in range(100):
            discovery.refresh()
            assert discovery.latest().choices == ()
        assert time.monotonic() - began < .5
        assert len(calls) == 1
        release.set()
        assert second.wait(1)
        wait_for(lambda: discovery.latest().choices == (("COM4", "Glove"),))
        time.sleep(.05)
        assert len(calls) == 2 and len(set(calls)) == 1
    finally:
        release.set()
        discovery.stop()
        assert discovery.join(1)


@pytest.mark.parametrize("late_error", [False, True])
def test_stop_returns_immediately_and_discards_late_success_or_failure(late_error):
    entered, release = threading.Event(), threading.Event()
    calls = []

    def provider():
        calls.append(True)
        if len(calls) == 1:
            return [dict(path="COM3", label="Original")]
        entered.set()
        assert release.wait(2)
        if late_error:
            raise OSError("late error")
        return [dict(path="COM9", label="Late result")]

    discovery = SerialPortDiscovery(provider, interval_s=10)
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        before = discovery.latest()
        discovery.refresh()
        assert entered.wait(1)
        began = time.monotonic()
        discovery.stop()
        assert time.monotonic() - began < .5
        assert not discovery.join()
        release.set()
        assert discovery.join(1)
        assert discovery.latest() is before
        discovery.start()
        discovery.refresh()
        assert discovery.latest() is before and len(calls) == 2
    finally:
        release.set()
        discovery.stop()
        assert discovery.join(1)


def test_failed_scan_preserves_choices_and_next_success_clears_error():
    outcome = [[dict(path="COM3", label="Glove")]]

    def provider():
        if isinstance(outcome[0], Exception):
            raise outcome[0]
        return outcome[0]

    discovery = SerialPortDiscovery(provider, interval_s=10)
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        outcome[0] = OSError("enumeration unavailable")
        discovery.refresh()
        wait_for(lambda: discovery.latest().error == "enumeration unavailable")
        assert discovery.latest().choices == (("COM3", "Glove"),)
        outcome[0] = [dict(path="COM9", label="Reconnected")]
        discovery.refresh()
        wait_for(lambda: discovery.latest().choices == (("COM9", "Reconnected"),))
        assert discovery.latest().error is None
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_refresh_before_start_does_not_start_thread_or_call_provider():
    calls = []
    discovery = SerialPortDiscovery(lambda: calls.append(True) or [])
    discovery.refresh()
    assert discovery.latest().choices == () and calls == []
    discovery.stop()
    discovery.start()
    assert discovery.join() and calls == []


@pytest.mark.parametrize("interval", [0, -1, float("nan"), float("inf"), True])
def test_invalid_intervals_fail_before_start(interval):
    with pytest.raises(ValueError):
        SerialPortDiscovery(lambda: [], interval_s=interval)


def device(path="COM4", identity="usb-a"):
    return dict(path=path, label=path + " · STM32", identity=identity)


def roles(discovery):
    return {item.path: item.role for item in discovery.latest().classifications}


def test_refresh_does_not_probe_and_explicit_identification_runs_off_caller_thread():
    calls = []

    def probe(path, **kwargs):
        calls.append((path, threading.get_ident(), kwargs))
        return ProbeResult("imu")

    discovery = SerialPortDiscovery(lambda: [device()], probe=probe, interval_s=.02)
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        discovery.refresh()
        time.sleep(.04)
        assert not calls
        assert discovery.identify(baudrates=(115200, 230400))
        wait_for(lambda: roles(discovery).get("COM4") == "imu" and not discovery.latest().identifying)
        assert len(calls) == 1 and calls[0][1] != threading.get_ident()
        assert calls[0][2]["baudrates"] == (115200, 230400)
        discovery.refresh()
        time.sleep(.04)
        assert roles(discovery) == {"COM4": "imu"} and len(calls) == 1
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_excluded_active_port_keeps_earlier_probe_type_without_reopening():
    calls = []
    discovery = SerialPortDiscovery(lambda: [device("COM4", "imu-usb"), device("COM5", "touch-usb")],
                                    probe=lambda path, **_: calls.append(path) or ProbeResult("imu" if path == "COM4" else "tactile"))
    discovery.start()
    try:
        wait_for(lambda: len(discovery.latest().choices) == 2)
        assert discovery.identify()
        wait_for(lambda: not discovery.latest().identifying)
        assert calls == ["COM4", "COM5"]
        assert roles(discovery) == {"COM4": "imu", "COM5": "tactile"}
        calls.clear()
        assert discovery.identify(excluded=(r"\\.\COM04",))
        wait_for(lambda: not discovery.latest().identifying)
        assert calls == ["COM5"] and roles(discovery) == {"COM4": "imu", "COM5": "tactile"}
    finally:
        discovery.stop()
        assert discovery.join(1)


@pytest.mark.parametrize("change", ["remove", "replace_identity", "missing_identity"])
def test_unplug_or_same_com_device_replacement_discards_old_type(change):
    entries = [device()]
    discovery = SerialPortDiscovery(lambda: list(entries), interval_s=10,
                                    probe=lambda *_args, **_kw: ProbeResult("imu"))
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        discovery.identify()
        wait_for(lambda: roles(discovery).get("COM4") == "imu" and not discovery.latest().identifying)
        entries[:] = [] if change == "remove" else [device(identity="usb-b" if change == "replace_identity" else None)]
        discovery.refresh()
        wait_for(lambda: not roles(discovery))
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_probe_stop_cancels_and_waits_for_handle_release_without_late_results():
    entered, released = threading.Event(), threading.Event()

    def probe(_port, *, cancel, **_kwargs):
        entered.set()
        assert cancel.wait(2)
        assert released.wait(2)
        return ProbeResult("imu")

    discovery = SerialPortDiscovery(lambda: [device()], probe=probe)
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        discovery.identify()
        assert entered.wait(1)
        for _ in range(20):
            assert not discovery.identify()
        discovery.stop()
        before = discovery.latest()
        assert not discovery.join(0)
        released.set()
        assert discovery.join(1)
        assert discovery.latest() is before and roles(discovery) == {"COM4": "identifying"}
    finally:
        released.set()
        discovery.stop()
        assert discovery.join(1)


def test_device_replaced_during_probe_does_not_receive_predecessor_type():
    entries = [device()]

    def probe(*_args, **_kwargs):
        entries[:] = [device(identity="usb-b")]
        return ProbeResult("imu")

    discovery = SerialPortDiscovery(lambda: list(entries), probe=probe)
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        discovery.identify()
        wait_for(lambda: not discovery.latest().identifying)
        assert not roles(discovery)
        assert discovery.latest().identities == (("COM4", "usb-b"),)
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_probe_failure_is_local_and_does_not_prevent_other_ports_or_retry():
    calls = []

    def probe(path, **_kwargs):
        calls.append(path)
        if path == "COM4" and calls.count(path) == 1:
            raise OSError("permission denied")
        return ProbeResult("tactile")

    discovery = SerialPortDiscovery(lambda: [device(), device("COM5", "usb-b")], probe=probe)
    discovery.start()
    try:
        wait_for(lambda: len(discovery.latest().choices) == 2)
        discovery.identify()
        wait_for(lambda: not discovery.latest().identifying)
        assert roles(discovery) == {"COM4": "error", "COM5": "tactile"}
        discovery.identify()
        wait_for(lambda: not discovery.latest().identifying)
        assert roles(discovery) == {"COM4": "tactile", "COM5": "tactile"}
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_failed_enumeration_does_not_probe_stale_list():
    result, calls = [[device()]], []

    def provider():
        if isinstance(result[0], Exception):
            raise result[0]
        return result[0]

    discovery = SerialPortDiscovery(provider, probe=lambda *a, **kw: calls.append(a), interval_s=10)
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        result[0] = OSError("unavailable")
        discovery.identify()
        wait_for(lambda: not discovery.latest().identifying)
        assert not calls and discovery.latest().error == "unavailable"
    finally:
        discovery.stop()
        assert discovery.join(1)


def test_missing_usb_identity_keeps_temporary_probe_evidence_until_observed_unplug():
    entries = [device(identity=None)]
    discovery = SerialPortDiscovery(lambda: list(entries), probe=lambda *a, **kw: ProbeResult("imu"), interval_s=.02)
    discovery.start()
    try:
        wait_for(lambda: bool(discovery.latest().choices))
        discovery.identify()
        wait_for(lambda: roles(discovery).get("COM4") == "imu" and not discovery.latest().identifying)
        discovery.refresh()
        time.sleep(.05)
        assert roles(discovery) == {"COM4": "imu"}
        assert discovery.latest().identities == (("COM4", None),)
        entries.clear()
        wait_for(lambda: not roles(discovery))
    finally:
        discovery.stop()
        assert discovery.join(1)
