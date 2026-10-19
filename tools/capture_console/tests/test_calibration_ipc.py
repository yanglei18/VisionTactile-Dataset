"""Calibration commands cannot downgrade or use a stale manager connection."""
import time

import pytest

from vt_capture_console.backend import ipc_client
from vt_capture_core import PlanControlSnapshot
from vt_capture_core.calibration_protocol import empty_calibration


def params():
    return dict(expected_manager_instance_id="manager", session_id="session")


def connected_backend(tmp_path, monkeypatch, version):
    monkeypatch.setattr(ipc_client.IpcBackend, "_run", lambda self: None)
    backend = ipc_client.IpcBackend(tmp_path / "not-used.sock")
    value = PlanControlSnapshot("live", "manager").to_dict()
    if version == 6:
        value["capabilities"]["calibration"] = True
        value["calibration"] = empty_calibration() | dict(enabled=True, busy=True)
    backend._accept_snapshot(value, version)
    return backend


def test_v6_calibration_submits_without_record_capability(tmp_path, monkeypatch):
    backend = connected_backend(tmp_path, monkeypatch, 6)
    try:
        identifier = backend.submit("calibration.cancel", params())
        request = backend._commands.get_nowait()
        assert request["version"] == 6 and request["request_id"] == identifier
        assert backend.supports_calibration
        assert backend.supports_capture_plan is False
    finally:
        backend.close()


def test_v5_rejects_calibration_without_sending(tmp_path, monkeypatch):
    backend = connected_backend(tmp_path, monkeypatch, 5)
    try:
        identifier = backend.submit("calibration.cancel", params())
        assert backend.operation_result(identifier)["state"] == "rejected"
        assert backend._commands.empty()
    finally:
        backend.close()


def test_stale_snapshot_invalidates_calibration_display_and_capability(tmp_path, monkeypatch):
    backend = connected_backend(tmp_path, monkeypatch, 6)
    try:
        backend._received = time.monotonic() - 3
        snapshot = backend.snapshot()
        assert not snapshot["capabilities"]["calibration"]
        assert not snapshot["calibration"]["enabled"]
        assert backend._snapshot["calibration"]["enabled"]
    finally:
        backend.close()


def test_v6_preserves_capture_plan_capability(tmp_path, monkeypatch):
    backend = connected_backend(tmp_path, monkeypatch, 6)
    try:
        backend._snapshot["capabilities"]["capture_plan"] = True
        assert backend.supports_capture_plan
    finally:
        backend.close()


def test_configuration_reload_rejects_active_calibration():
    from vt_capture_console.backend.local_runtime import _snapshot_problem
    snapshot = PlanControlSnapshot("live", "manager").to_dict()
    snapshot["calibration"] = empty_calibration() | dict(enabled=True, busy=True)
    assert _snapshot_problem(snapshot)
