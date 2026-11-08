"""Local control receipts cannot duplicate calibration or cross a hand/session."""
from copy import deepcopy
import socket
import struct
import threading

import pytest

from vt_fsglove12.hand_service import HandService, HandServiceServer, _claim_socket
from vt_fsglove12.hand_service_protocol import MAX_REQUEST_BYTES, request, receive


class Controller:
    def __init__(self):
        self.calls = []
        self.alive = True
        self.snapshot = dict(runtime=dict(ipc_sequence=1, connected=False, session_id=None,
            command_completed_generation=0, command_rejected_count=0,
            calibration_activity=dict(busy=False, observed_monotonic_s=1., command_completed_generation=0)),
            calibration=dict(profile_version=None), pose=dict(pose_available=False),
            input_snapshot=dict(runtime={}, orientation=None))

    def latest_snapshot(self):
        return deepcopy(self.snapshot)

    def is_alive(self):
        return self.alive

    def submit_bound(self, name, args, **binding):
        self.calls.append((name, args, binding))
        return True

    def complete(self, *, error=None, input_only=False):
        runtime = self.snapshot["input_snapshot"]["runtime"] if input_only else self.snapshot["runtime"]
        runtime.update(command_completed_generation=runtime.get("command_completed_generation", 0) + 1,
                       last_command_name=self.calls[-1][0], last_command_error=error)

    def stop(self):
        self.alive = False


@pytest.fixture
def service():
    return HandService(Controller(), hand="right", instance_id="service-a")


def command(service, request_id="one", **overrides):
    return dict(schema_version=1, op="command", hand="right", expected_service_instance_id=service.instance_id,
                request_id=request_id, command="capture", args={"step": "neutral"},
                expected_session_id=None, expected_profile_version=None, **overrides)


def status(service):
    return service.handle(dict(schema_version=1, op="snapshot", hand="right"))


def test_ready_service_does_not_claim_connected_or_valid_pose(service):
    response = status(service)
    assert response["ok"] and response["service_state"] == "ready"
    assert not response["snapshot"]["runtime"]["connected"]
    assert response["snapshot"]["pose"]["pose_available"] is False
    assert response["pose_valid"] is False
    assert response["snapshot"]["input_snapshot"] == service.controller.snapshot["input_snapshot"]
    response["snapshot"]["runtime"]["connected"] = True
    assert not status(service)["snapshot"]["runtime"]["connected"]


def test_activity_and_queued_receipt_close_input_acknowledgement_window(service):
    assert status(service)["calibration_busy"] is False
    assert service.handle(command(service))["calibration_busy"] is True
    service.controller.complete(input_only=True)
    service.controller.snapshot["input_snapshot"]["runtime"]["calibration_activity"] = dict(
        busy=True, observed_monotonic_s=2., command_completed_generation=1)
    current = status(service)
    assert current["receipts"][0]["state"] == "executed"
    assert current["calibration_busy"] is True
    service.controller.snapshot["runtime"]["calibration_activity"] = dict(
        busy=False, observed_monotonic_s=3., command_completed_generation=1)
    assert status(service)["calibration_busy"] is False


def test_missing_activity_fails_closed_for_recording(service):
    del service.controller.snapshot["runtime"]["calibration_activity"]
    assert status(service)["calibration_busy"] is True


def test_service_pose_valid_uses_measurement_contract_not_display_availability(service):
    from vt_fsglove12.config import load_config
    service.controller.config = load_config()
    current = service.controller.snapshot
    current["runtime"]["connected"] = True
    current["orientation"] = dict(nodes={"12": dict(host_realtime_ns=1, host_monotonic_ns=1,
                                                   sample_index=1, epoch=0)})
    current["pose"] = dict(pose_available=True, solver_success=True,
        mano_pose_rotvec=[[0., 0., 0.]] * 16, root_quat_xyzw=[0., 0., 0., 1.], shape_params=[0.] * 10)
    assert status(service)["pose_valid"] is True
    current["display_pose"] = deepcopy(current["pose"])
    del current["orientation"]["nodes"]["12"]["host_realtime_ns"]
    assert status(service)["pose_valid"] is False
    current["orientation"]["nodes"]["12"]["host_realtime_ns"] = 1
    current["pose"]["display_only"] = True
    assert status(service)["pose_valid"] is False


def test_shape_executor_error_is_not_a_successful_receipt(service):
    message = {**command(service), "command": "apply_shape_candidate", "args": {}}
    service.handle(message)
    service.controller.complete(input_only=True)
    service.controller.snapshot["input_snapshot"]["runtime"]["shape_command_error"] = "No candidate"
    current = status(service)
    assert current["receipts"][0]["state"] == "rejected"
    assert current["receipts"][0]["error"] == "No candidate"


def test_duplicate_request_returns_receipt_without_resubmission(service):
    message = command(service)
    assert service.handle(message)["receipt"]["state"] == "queued"
    assert service.handle(message)["receipt"]["state"] == "queued"
    assert len(service.controller.calls) == 1
    service.controller.complete()
    assert service.handle(message)["receipt"]["state"] == "executed"
    assert len(service.controller.calls) == 1
    changed = {**message, "args": {"step": "thumb_up"}}
    assert not service.handle(changed)["ok"]
    assert len(service.controller.calls) == 1


@pytest.mark.parametrize("field,value", [("hand", "left"), ("expected_service_instance_id", "old"),
    ("expected_session_id", "old-session"), ("expected_profile_version", 3), ("expected_profile_version", True)])
def test_cross_context_command_is_rejected_before_submit(service, field, value):
    response = service.handle({**command(service), field: value})
    assert not response["ok"] or response["receipt"]["state"] == "rejected"
    assert not service.controller.calls


def test_rejected_id_cannot_become_valid_after_context_changes(service):
    message = {**command(service), "expected_session_id": "later"}
    assert service.handle(message)["receipt"]["state"] == "rejected"
    service.controller.snapshot["runtime"]["session_id"] = "later"
    assert service.handle(message)["receipt"]["state"] == "rejected"
    assert not service.controller.calls


def test_pending_serializes_commands_and_input_receipt_cannot_ack_the_next_command(service):
    assert service.handle(command(service))["receipt"]["state"] == "queued"
    assert service.handle(command(service, "two"))["receipt"]["state"] == "rejected"
    service.controller.complete(input_only=True)
    assert status(service)["receipts"][0]["state"] == "executed"
    assert service.handle(command(service, "three"))["receipt"]["state"] == "queued"
    assert status(service)["receipts"][-1]["state"] == "queued"
    service.controller.complete(input_only=True)
    assert status(service)["receipts"][-1]["state"] == "executed"
    assert len(service.controller.calls) == 2


def test_lost_worker_is_unknown_and_never_retried(service):
    service.handle(command(service))
    service.controller.alive = False
    response = status(service)
    assert response["service_state"] == "failed"
    assert response["receipts"][0]["state"] == "unknown"
    assert service.handle(command(service))["receipt"]["state"] == "unknown"
    assert len(service.controller.calls) == 1


def test_execution_error_is_distinct_from_queue_acceptance(service):
    service.handle(command(service))
    service.controller.complete(error="profile changed")
    receipt = service.handle(dict(schema_version=1, op="receipt", hand="right",
        expected_service_instance_id=service.instance_id, request_id="one"))["receipt"]
    assert receipt["state"] == "rejected" and receipt["error"] == "profile changed"


def test_receipt_capacity_does_not_evict_duplicate_protection(service, monkeypatch):
    monkeypatch.setattr("vt_fsglove12.hand_service.MAX_RECEIPTS", 1)
    service.handle(command(service))
    service.controller.complete()
    assert not service.handle(command(service, "two"))["ok"]
    assert service.handle(command(service))["receipt"]["state"] == "executed"
    assert len(service.controller.calls) == 1


def test_socket_protocol_roundtrip_and_duplicate_request(service, tmp_path):
    path = tmp_path / "hand.sock"
    server = HandServiceServer(path, service)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01})
    thread.start()
    try:
        result = request(path, dict(schema_version=1, op="snapshot", hand="right"))
        assert result["service_instance_id"] == service.instance_id
        assert request(path, command(service))["receipt"]["state"] == "queued"
        assert request(path, command(service))["receipt"]["state"] == "queued"
        assert len(service.controller.calls) == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_oversized_length_rejected_before_reading_payload():
    left, right = socket.socketpair()
    try:
        left.sendall(struct.pack("!I", MAX_REQUEST_BYTES + 1))
        right.settimeout(.1)
        with pytest.raises(ValueError, match="size limit"):
            receive(right, MAX_REQUEST_BYTES)
    finally:
        left.close()
        right.close()


def test_socket_claim_never_replaces_regular_file(tmp_path):
    path = tmp_path / "control.sock"
    path.write_text("retain")
    with pytest.raises(ValueError, match="non-socket"):
        _claim_socket(path)
    assert path.read_text() == "retain"


def test_socket_claim_excludes_another_service(tmp_path):
    import os
    path = tmp_path / "control.sock"
    lock = _claim_socket(path)
    try:
        with pytest.raises(BlockingIOError):
            _claim_socket(path)
    finally:
        os.close(lock)
