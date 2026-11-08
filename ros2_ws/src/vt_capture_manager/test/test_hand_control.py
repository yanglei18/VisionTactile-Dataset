"""Ownership, receipt recovery and calibration admission without hardware."""
from copy import deepcopy
from threading import RLock
from types import SimpleNamespace as NS

import pytest

from vt_capture_manager.hand_control import HandControl


class Harness:
    def __init__(self):
        self.now = 10.
        self.allowed = (True, "")
        self.groups = [dict(id="right_glove", state="RUNNING", owned=True, pid=123)]
        self.calls = []
        self.response = dict(hand="right", service_instance_id="service-a", service_state="ready",
            pose_valid=False, calibration_busy=False, receipts=[], snapshot=dict(
                runtime=dict(connected=True, last_sample_age_s=.01, session_id="session-a"),
                calibration=dict(profile_version=None), pose=dict(pose_available=True)))
        self.control = HandControl([dict(id="right_glove", preset="glove", parameters=dict(
            hand="right", resources="/assets", glove_python="/env/python", port="/serial"))],
            manager_instance_id="manager-a", allowed=lambda: self.allowed, admission_lock=RLock(),
            process_snapshot=lambda: dict(groups=deepcopy(self.groups)),
            exchange=self.exchange, monotonic=lambda: self.now)
        self.control._poll("right")

    def exchange(self, path, request, **kwargs):
        self.calls.append(deepcopy(request))
        if isinstance(self.response, Exception):
            raise self.response
        return deepcopy(self.response)

    def params(self, **changes):
        return dict(hand="right", expected_manager_instance_id="manager-a",
            expected_service_instance_id="service-a", expected_session_id="session-a",
            expected_profile_version=None, name="capture", args={"step": "neutral"}) | changes


@pytest.fixture
def h():
    return Harness()


def test_service_without_profile_is_ready_but_pose_is_not_calibrated(h):
    row = h.control.snapshot()["right"]
    assert row["service_state"] == "ready" and row["raw_fresh"]
    assert not row["calibration_ready"] and not row["pose_valid"]
    assert row["profile_version"] == 0 and row["glove_python"] == "/env/python"
    assert h.control.ready(NS(kind="glove", id="right_glove")) is True
    assert h.control.ready(NS(kind="tactile", id="right_tactile")) is None
    assert h.control.command(h.params(), "neutral")["state"] == "accepted"
    _, params = h.control._queue.get_nowait()
    assert params["expected_profile_version"] is None


def test_pose_valid_and_busy_use_authoritative_envelope(h):
    h.response["snapshot"]["pose"]["pose_available"] = False
    h.response.update(pose_valid=True, calibration_busy=True)
    h.control._poll("right")
    assert h.control.snapshot()["right"]["pose_valid"]
    assert h.control.busy
    del h.response["calibration_busy"]
    h.control._poll("right")
    assert h.control.busy  # Unknown service activity cannot permit recording.


def test_idempotent_request_queues_once_and_conflicting_id_is_rejected(h):
    result = h.control.command(h.params(), "one")
    assert h.control.command(h.params(), "one") == result
    assert h.control._queue.qsize() == 1
    other = h.control.command(h.params(args={"step": "thumb_up"}), "one")
    assert other["state"] == "rejected" and other["code"] == "request_id_conflict"
    assert h.control._queue.qsize() == 1


@pytest.mark.parametrize("changes", [dict(expected_manager_instance_id="old"),
    dict(expected_service_instance_id="old"), dict(hand="left")])
def test_stale_or_wrong_identity_never_queues(h, changes):
    assert h.control.command(h.params(**changes), "one")["state"] == "rejected"
    assert h.control._queue.empty()


@pytest.mark.parametrize("group", [dict(state="STOPPED", owned=False, pid=None),
    dict(state="EXTERNAL", owned=False, pid=33), dict(state="UNKNOWN", owned=True, pid=33)])
def test_only_owned_live_process_can_receive_commands(h, group):
    h.groups = [dict(id="right_glove", **group)]
    assert h.control.command(h.params(), "one")["state"] == "rejected"
    assert h.control._queue.empty()


def test_recording_admission_rejection_never_dispatches(h):
    h.allowed = (False, "Recording is active")
    receipt = h.control.command(h.params(), "one")
    assert receipt["state"] == "rejected" and "Recording" in receipt["detail"]
    assert h.control._queue.empty() and not h.control.busy


def test_input_ack_does_not_release_busy_while_service_calibrates(h):
    h.control.command(h.params(), "one")
    h.response.update(calibration_busy=True, receipts=[dict(request_id="one", state="executed")])
    h.control._poll("right")
    assert h.control.operation("one")["code"] == "hand_executed"
    assert not h.control._pending and h.control.busy
    h.response["calibration_busy"] = False
    h.control._poll("right")
    assert not h.control.busy


def test_socket_timeout_and_stale_observation_do_not_release_uncertain_command(h):
    h.control.command(h.params(), "one")
    h.control._update_receipt("one", dict(state="unknown"))
    h.response = OSError("timeout")
    h.control._poll("right")
    h.now += 5.
    h.control._retire_stopped()
    assert h.control.busy and h.control.operation("one")["state"] == "unknown"
    row = h.control.snapshot()["right"]
    assert not row["pose_valid"] and not row["raw_fresh"]


@pytest.mark.parametrize("state", ["STOPPED", "FAILED"])
def test_verified_process_exit_releases_block_without_retrying_unknown_action(h, state):
    params = h.params()
    h.control.command(params, "one")
    h.control._update_receipt("one", dict(state="unknown"))
    h.groups = [dict(id="right_glove", state=state, owned=False, pid=None)]
    h.control._retire_stopped()
    receipt = h.control.operation("one")
    assert receipt["state"] == "unknown" and receipt["code"] == "hand_stopped"
    assert not h.control.busy
    assert h.control.command(params, "one") == receipt
    h.control._update_receipt("one", dict(state="executed"))
    assert h.control.operation("one") == receipt
    assert h.control._queue.qsize() == 1  # Retired queue entry cannot be resubmitted.


@pytest.mark.parametrize("group", [dict(state="FAILED", owned=True, pid=3),
    dict(state="STOPPED", owned=False, pid=3), dict(state="UNKNOWN", owned=False, pid=None),
    dict(state="STOPPED", owned=False)])
def test_incomplete_or_live_ownership_evidence_cannot_clear_pending(h, group):
    h.control.command(h.params(), "one")
    h.groups = [dict(id="right_glove", **group)]
    h.control._retire_stopped()
    assert h.control.busy and "one" in h.control._pending


def test_service_restart_retires_old_request_and_preserves_no_resend(h):
    params = h.params()
    h.control.command(params, "one")
    h.response["service_instance_id"] = "service-b"
    h.control._poll("right")
    receipt = h.control.operation("one")
    assert receipt["code"] == "hand_restarted" and receipt["state"] == "unknown"
    assert not h.control.busy
    assert h.control.command(params, "one") == receipt
    h.control._update_receipt("one", dict(state="executed"))
    assert h.control.operation("one") == receipt


def test_process_observation_is_never_called_under_hand_lock(h):
    def process_snapshot():
        assert not h.control._lock._is_owned()
        return dict(groups=h.groups)
    h.control._process_snapshot = process_snapshot
    h.control._retire_stopped()
    assert h.control.command(h.params(), "one")["state"] == "accepted"


def test_pending_hand_capture_blocks_recording_but_allows_explicit_device_stop(tmp_path):
    from test_management import FakeAdapter, model, create_batch, start
    from vt_capture_manager.management import ManagementService
    adapter = FakeAdapter()
    service = ManagementService(model(), adapter, enabled=True, state_dir=tmp_path/"state", timeout_s=.01)
    adapter.store = service.store
    h = Harness()
    h.control._admission = service._mutex
    h.control._allowed = service.hand_calibration_allowed
    service.hands = h.control
    try:
        assert service.poll_once()
        batch = create_batch(service)
        assert h.control.command(h.params(), "hand-one")["state"] == "accepted"
        result = start(service, adapter, batch_id=batch)
        assert result["operation"]["code"] == "hand_calibration_busy"
        assert not adapter.commands and service._commands.empty()
        assert not service.lifecycle_allowed("device.start")[0]
        assert service.lifecycle_allowed("device.stop")[0]
        assert not service.calibration_allowed()[0]
        h.groups = [dict(id="right_glove", state="STOPPED", owned=False, pid=None)]
        h.control._retire_stopped()
        assert service.lifecycle_allowed("device.start")[0]
        assert service.calibration_allowed()[0]
        assert start(service, adapter, identifier="start-after-stop", batch_id=batch)["operation"]["state"] == "accepted"
    finally:
        service.close()


def test_recording_or_device_transition_blocks_hand_mutation(tmp_path):
    from test_management import FakeAdapter, model, start, execute
    from vt_capture_manager.management import ManagementService
    adapter = FakeAdapter()
    service = ManagementService(model(), adapter, enabled=True, state_dir=tmp_path/"state", timeout_s=.01)
    adapter.store = service.store
    h = Harness()
    h.control._admission = service._mutex
    h.control._allowed = service.hand_calibration_allowed
    service.hands = h.control
    try:
        assert service.poll_once()
        service.processes = NS(busy=True)
        assert not service.hand_calibration_allowed()[0]
        service.processes = None
        assert start(service, adapter)["operation"]["state"] == "accepted"
        execute(service)
        assert adapter.context["recorder_alive"]
        result = h.control.command(h.params(), "during-recording")
        assert result["state"] == "rejected" and h.control._queue.empty()
    finally:
        service.processes = None
        service.close()


@pytest.mark.parametrize("hand", ["left", "right"])
@pytest.mark.parametrize("bundle", ["", "/verified/display-bundle"])
def test_tactile_only_metadata_never_contacts_or_claims_a_motion_service(hand, bundle):
    from vt_capture_core.hand_protocol import validate_hand_sessions
    calls = []
    control = HandControl([dict(id=f"{hand}_tactile", preset="tactile", enabled=True,
        parameters=dict(hand=hand, resources=bundle, glove_python="/unused/python",
                        port="/touch-serial", display_socket="/touch.sock"))],
        manager_instance_id="manager-a", allowed=lambda: (True, ""), admission_lock=RLock(),
        process_snapshot=lambda: dict(groups=[]), exchange=lambda *a, **kw: calls.append(a))
    control._poll(hand)
    row = control.snapshot()[hand]
    validate_hand_sessions(control.snapshot())
    assert row["resources"] == bundle and row["tactile_port"] == "/touch-serial"
    assert row["tactile_socket"] == "/touch.sock"
    assert row["group_id"] == row["control_socket"] == row["port"] == ""
    assert row["service_instance_id"] == row["session_id"] == ""
    assert not any(row[key] for key in ("connected", "raw_fresh", "calibration_ready", "pose_valid", "busy"))
    assert row["service_state"] == "unavailable" and row["age_s"] is None
    assert control.ready(NS(kind="tactile", id=f"{hand}_tactile")) is None
    assert not control.busy and calls == []
    result = control.command(dict(hand=hand, expected_manager_instance_id="manager-a",
        expected_service_instance_id="no-service", expected_session_id="no-session",
        expected_profile_version=None, name="capture", args={"step": "neutral"}), "forbidden-motion")
    assert result["state"] == "rejected" and control._queue.empty() and not control.busy


def test_mixed_hand_keeps_motion_resources_and_independent_tactile_endpoint():
    control = HandControl([
        dict(id="motion", preset="glove", parameters=dict(hand="left", resources="/motion",
             port="/imu", control_socket="/motion.sock", glove_python="/motion-python")),
        dict(id="touch", preset="tactile", parameters=dict(hand="left", resources="/touch-display",
             port="/touch", display_socket="/touch.sock"))],
        manager_instance_id="manager-a", allowed=lambda: (True, ""), admission_lock=RLock(),
        process_snapshot=lambda: dict(groups=[]), exchange=lambda *a, **kw: {})
    row = control.snapshot()["left"]
    assert row["resources"] == "/motion" and row["glove_python"] == "/motion-python"
    assert row["group_id"] == "motion" and row["control_socket"] == "/motion.sock"
    assert row["port"] == "/imu" and row["tactile_port"] == "/touch"
    assert row["tactile_socket"] == "/touch.sock"


@pytest.mark.parametrize("hand", ["left", "right"])
@pytest.mark.parametrize("mode", ["disabled", "tactile", "mixed"])
def test_six_profile_configuration_selects_mode_without_borrowing_disabled_motion_resources(hand, mode):
    profiles = [dict(id=preset, preset=preset, enabled=False, parameters={})
                for preset in ("cameras", "trackers")]
    profiles += [dict(id=f"{side}_{preset}", preset=preset, enabled=False, parameters=dict(hand=side))
                 for side in ("left", "right") for preset in ("glove", "tactile")]
    assert len(profiles) == 6
    for profile in profiles:
        if profile["preset"] in {"glove", "tactile"}:
            params = profile["parameters"]
            params["resources"] = f"/{params['hand']}/{profile['preset']}"
            profile["enabled"] = (params["hand"] == hand and
                                  (mode == "mixed" or mode == "tactile" and profile["preset"] == "tactile"))
    # Missing tactile bundle remains explicit; never borrow a disabled motion bundle.
    next(p for p in profiles if p["id"] == f"{hand}_tactile")["parameters"]["resources"] = ""
    control = HandControl(profiles, manager_instance_id="manager-a", allowed=lambda: (True, ""),
        admission_lock=RLock(), process_snapshot=lambda: dict(groups=[]), exchange=lambda *a, **kw: {})
    row = control.snapshot()[hand]
    if mode == "tactile":
        assert row["group_id"] == row["control_socket"] == row["resources"] == ""
    else:
        assert row["group_id"] == f"{hand}_glove" and row["control_socket"]
        assert row["resources"] == f"/{hand}/glove"
