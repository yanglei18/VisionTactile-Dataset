"""Synthetic sysfs/journal evidence only; never opens or resets camera hardware."""
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from vt_capture_manager.camera_access import CameraStartGuard, failed_controllers


BOOT = "a" * 32
ADDRESS = "0000:80:14.0"


def event(stamp, failed=True, *, address=ADDRESS, boot=BOOT):
    action = "HC died; cleaning up" if failed else "new USB bus registered, assigned bus number 4"
    return dict(MESSAGE=f"xhci_hcd {address}: {action}", _BOOT_ID=boot,
                __MONOTONIC_TIMESTAMP=str(stamp))


def test_only_unrecovered_failures_in_current_boot_count_independent_of_output_order():
    assert failed_controllers([event(10), event(1, False)], BOOT) == (ADDRESS,)
    assert failed_controllers([event(10), event(20, False), event(1, False)], BOOT) == ()
    assert failed_controllers([event(20, False), event(10)], BOOT) == ()
    assert failed_controllers([event(10, boot="b" * 32)], BOOT) == ()


def test_recovery_only_clears_its_own_controller():
    events = [event(10), event(20, False, address="0000:00:0d.0")]
    assert failed_controllers(events, BOOT) == (ADDRESS,)


@pytest.mark.parametrize("record", [None, {}, {"MESSAGE": []},
    dict(event(10), MESSAGE="camera unavailable: HC died"),
    dict(event(10), MESSAGE="xhci_hcd 0000:80:14.0: WARN short transfer"),
    dict(event(10), __MONOTONIC_TIMESTAMP="-1"),
    dict(event(10), __MONOTONIC_TIMESTAMP=10)])
def test_malformed_or_unrelated_entries_never_prove_a_controller_failure(record):
    assert failed_controllers([record], BOOT) == ()


@pytest.fixture
def evidence(tmp_path):
    usb = tmp_path / "usb"
    usb.mkdir()
    boot = tmp_path / "boot_id"
    boot.write_text(BOOT)
    config = tmp_path / "cameras.yaml"
    config.write_text('cameras:\n  - serial: "123"\n    asic_serial: "456"\n')
    calls = []
    records = [event(1, False), event(10)]
    clock = [0.]

    def run(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(returncode=0, stdout="\n".join(map(json.dumps, records)).encode())

    guard = CameraStartGuard(usb_root=usb, boot_id_path=boot, run=run, clock=lambda: clock[0])
    return SimpleNamespace(guard=guard, usb=usb, boot=boot, config=config, records=records,
                           calls=calls, clock=clock,
                           resources=("realsense-cameras", "camera-config:" + str(config)))


def enumerate_camera(evidence, serial, vendor="8086"):
    device = evidence.usb / "4-6"
    device.mkdir()
    (device / "idVendor").write_text(vendor)
    (device / "serial").write_text(serial)


def test_confirmed_failure_and_missing_cameras_refuse_start_without_device_access(evidence):
    code, detail = evidence.guard(evidence.resources)
    assert code == "usb_controller_failed"
    assert ADDRESS in detail and "HC died" in detail and "未枚举" in detail
    argv, options = evidence.calls[0]
    assert "--dmesg" in argv and "--boot=" + BOOT in argv
    assert "--lines=128" in argv and any(arg.startswith("--grep=^xhci_hcd") for arg in argv)
    assert options["timeout"] <= .5 and options["stderr"] is subprocess.DEVNULL


def test_ordinary_missing_camera_does_not_mean_controller_failure(evidence):
    evidence.records[:] = [event(1, False)]
    code, detail = evidence.guard(evidence.resources)
    assert code == "camera_unavailable" and "HC died" not in detail


@pytest.mark.parametrize("serial", ["123", "456"])
def test_enumerated_configured_camera_clears_guard_even_with_cached_failure(evidence, serial):
    assert evidence.guard(evidence.resources)[0] == "usb_controller_failed"
    enumerate_camera(evidence, serial)
    assert evidence.guard(evidence.resources, refresh=True) is None
    assert len(evidence.calls) == 1


def test_unrelated_device_does_not_hide_missing_configured_cameras(evidence):
    enumerate_camera(evidence, "999")
    assert evidence.guard(evidence.resources)[0] == "usb_controller_failed"


def test_journal_reads_are_cached_and_controller_recovery_expires_failure(evidence):
    for _ in range(10):
        assert evidence.guard(evidence.resources)[0] == "usb_controller_failed"
    assert len(evidence.calls) == 1
    evidence.records.append(event(20, False))
    evidence.clock[0] += 2.1
    assert evidence.guard(evidence.resources)[0] == "camera_unavailable"
    assert len(evidence.calls) == 2


@pytest.mark.parametrize("failure", [PermissionError(), FileNotFoundError(),
                                     subprocess.TimeoutExpired("journalctl", .35)])
def test_unavailable_journal_never_fabricates_failure(evidence, failure):
    def run(*args, **kwargs):
        raise failure
    evidence.guard._run = run
    code, detail = evidence.guard(evidence.resources)
    assert code == "camera_diagnostics_unavailable" and "HC died" not in detail


@pytest.mark.parametrize("config", ["not: a-camera-config", "cameras: [broken]", "x" * 65537,
                                    'cameras: [{serial: "unsafe/not-a-serial"}]', "{invalid"])
def test_unusable_configuration_does_not_become_a_usb_diagnosis(evidence, config):
    evidence.config.write_text(config)
    assert evidence.guard(evidence.resources) is None
    assert evidence.calls == []


def test_unreadable_sysfs_is_not_absence_evidence(evidence):
    evidence.usb.rmdir()
    assert evidence.guard(evidence.resources)[0] == "camera_diagnostics_unavailable"
    assert evidence.calls == []


def test_other_resources_never_read_usb_or_journal(evidence):
    assert evidence.guard(("camera",)) is None
    assert evidence.guard(("camera-config:" + str(evidence.config),)) is None
    assert evidence.calls == []


def test_snapshot_cache_skips_repeated_sysfs_reads_but_execution_refreshes(evidence, monkeypatch):
    enumerate_camera(evidence, "123")
    assert evidence.guard(evidence.resources) is None
    (evidence.usb / "4-6" / "serial").unlink()
    assert evidence.guard(evidence.resources) is None
    assert evidence.guard(evidence.resources, refresh=True)[0] == "usb_controller_failed"
    assert len(evidence.calls) == 1


@pytest.mark.parametrize("result", [SimpleNamespace(returncode=1, stdout=b""),
                                   SimpleNamespace(returncode=0, stdout=b"not json"),
                                   SimpleNamespace(returncode=0, stdout=b"x" * 262145)])
def test_failed_or_unbounded_journal_read_stays_unknown(evidence, result):
    evidence.guard._run = lambda *args, **kwargs: result
    assert evidence.guard(evidence.resources)[0] == "camera_diagnostics_unavailable"
