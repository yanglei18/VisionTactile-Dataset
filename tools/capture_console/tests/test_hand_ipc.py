"""v7 hand controls preserve older capabilities and never fall back on writes."""
import time

import pytest

from vt_capture_console.backend import ipc_client
from vt_capture_core import PlanControlSnapshot
from vt_capture_core.calibration_protocol import empty_calibration


def params():
    return dict(hand="left", name="capture", args={"step": "neutral"},
                expected_manager_instance_id="manager", expected_service_instance_id="service",
                expected_session_id=None, expected_profile_version=None)


@pytest.fixture
def backend(tmp_path, monkeypatch):
    monkeypatch.setattr(ipc_client.IpcBackend, "_run", lambda self: None)
    value = ipc_client.IpcBackend(tmp_path / "not-connected.sock")
    snapshot = PlanControlSnapshot("live", "manager").to_dict()
    snapshot["capabilities"].update(hand_control=True, calibration=True, capture_plan=True)
    snapshot["calibration"] = empty_calibration() | dict(enabled=True)
    snapshot["hand_sessions"] = {"left": dict(service_state="ready", connected=True,
                    raw_fresh=True, calibration_ready=True, pose_valid=True, detail="current")}
    value._accept_snapshot(snapshot, 7)
    yield value
    value.close()


def test_v7_preserves_camera_calibration_and_plan_controls(backend):
    assert backend.supports_hand_control
    assert backend.supports_calibration and backend.supports_capture_plan
    assert backend.snapshot()["calibration"]["enabled"]
    identifier = backend.submit("hand.command", params())
    request = backend._commands.get_nowait()
    assert request["version"] == 7 and request["request_id"] == identifier
    assert request["params"] == params()


def test_stale_manager_invalidates_hand_controls_without_mutating_cache(backend):
    backend._received = time.monotonic() - 3
    assert not backend.supports_hand_control
    snapshot = backend.snapshot()
    assert not snapshot["capabilities"]["hand_control"]
    assert snapshot["hand_sessions"]["left"]["service_state"] == "unavailable"
    assert not snapshot["hand_sessions"]["left"]["pose_valid"]
    assert backend._snapshot["hand_sessions"]["left"]["pose_valid"]
    identifier = backend.submit("hand.command", params())
    assert backend.operation_result(identifier)["state"] == "rejected"
    assert backend._commands.empty()


def test_downgrade_never_resubmits_hand_command_under_an_older_version(backend, monkeypatch):
    identifier = backend.submit("hand.command", params())
    request = backend._commands.get_nowait()
    backend._version = 6
    monkeypatch.setattr(ipc_client, "_exchange", lambda *args, **kwargs: pytest.fail("unexpected command IO"))
    backend._send_command(request)
    assert backend.operation_result(identifier)["state"] == "rejected"
    assert not backend.supports_hand_control
    assert backend.supports_calibration
    assert not backend.snapshot()["capabilities"]["hand_control"]
