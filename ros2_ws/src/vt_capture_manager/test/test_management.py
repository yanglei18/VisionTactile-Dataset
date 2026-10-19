"""Durable control tests use fake adapters only; never start rosbag or hardware."""
from concurrent.futures import Future, ThreadPoolExecutor
from copy import deepcopy
import json
import socket
import sqlite3
import stat
import uuid
from types import SimpleNamespace as NS

import pytest

from vt_capture_core import decode_message, encode_message
from vt_capture_manager.config import ManagerConfig
from vt_capture_manager.management import ManagementService
from vt_capture_manager.model import SnapshotModel
from vt_capture_manager.server import SnapshotServer
from vt_capture_manager.storage import utc_now


def finished(value):
    result = Future()
    result.set_result(value)
    return result


def context(**changes):
    value = dict(controller_instance_id="controller-one", state_revision=1, state=0, session_id="",
                 start_request_id="", session_label="", output_path="", recorder_alive=False,
                 elapsed_sec=0., planned_duration_sec=0, topics=["/synthetic/only"], max_duration_sec=300, detail="Idle")
    value.update(changes)
    return value


class FakeAdapter:
    def __init__(self):
        self.context = context()
        self.commands = []
        self.behavior = "accept"
        self.store = None
        self.available = True

    def get_snapshot(self):
        if not self.available:
            result = Future()
            result.set_exception(ConnectionError("test unavailable"))
            return result
        return finished(deepcopy(self.context))

    def command(self, wire):
        # Assert durable intent precedes every external side effect.
        assert self.store.operation(wire["request_id"])["phase"] == "DISPATCHED"
        self.commands.append(deepcopy(wire))
        if self.behavior == "timeout":
            return Future()
        if self.behavior == "reject":
            return finished(dict(accepted=False, code="state_revision_mismatch", detail="rejected test command", context=deepcopy(self.context)))
        self.context["state_revision"] += 1
        if wire["command"] == 1:
            self.context.update(state=3, session_id="session-"+wire["request_id"], start_request_id=wire["request_id"],
                                session_label=wire["session_label"], output_path="/external/bags/session-test",
                                planned_duration_sec=wire["planned_duration_sec"], recorder_alive=True, detail="Recording")
        else:
            self.context.update(state=5, recorder_alive=False, elapsed_sec=3.5, detail="Complete; quality not assessed")
        if self.behavior == "timeout_after_effect":
            return Future()
        if self.behavior == "reject_after_effect":
            return finished(dict(accepted=False, code="session_journal_failed", detail="Recording exists despite journal failure", context=deepcopy(self.context)))
        return finished(dict(accepted=True, code="accepted", detail="Controller accepted; see context", context=deepcopy(self.context)))


def request(identifier, command, **params):
    return dict(version=2, request_id=identifier, command=command, params=params)


def model():
    # v0.2 compatibility scenarios deliberately disable the new independent
    # post-record quality stage; v0.3 queue behavior has its own tests.
    return SnapshotModel(ManagerConfig((), quality_enabled=False), disk_usage=lambda _: NS(free=123))


@pytest.fixture
def managed(tmp_path):
    adapter = FakeAdapter()
    service = ManagementService(model(), adapter, enabled=True, state_dir=tmp_path/"state", timeout_s=.01)
    adapter.store = service.store
    assert service.poll_once()
    yield service, adapter
    service.close()


def create_batch(service, identifier="batch-one"):
    result = service.dispatch(request(identifier, "batch.create", label="实验批次"))
    assert result["ok"]
    return result["snapshot"]["management"]["batches"][0]["id"]


def start(service, adapter, identifier="start-one", batch_id=None):
    batch_id = batch_id or create_batch(service)
    return service.dispatch(request(identifier, "capture.start", batch_id=batch_id, label="试次", duration_s=30,
                                    expected_controller_instance_id=adapter.context["controller_instance_id"],
                                    expected_state_revision=adapter.context["state_revision"]))


def execute(service):
    item = service._commands.get_nowait()
    service._execute(*item)
    service._commands.task_done()


def stop(service, adapter, trial_id, identifier="stop-one", **changes):
    params = dict(trial_id=trial_id, expected_controller_instance_id=adapter.context["controller_instance_id"],
                  expected_session_id=adapter.context["session_id"])
    params.update(changes)
    return service.dispatch(request(identifier, "capture.stop", **params))


def test_default_is_read_only_and_does_not_create_state_directory(tmp_path):
    directory = tmp_path/"must-not-exist"
    service = ManagementService(model(), state_dir=directory)
    result = service.dispatch(request("create", "batch.create", label="test"))
    assert result["operation"]["code"] == "read_only"
    assert not result["snapshot"]["capabilities"]["record"]
    assert not directory.exists()
    assert "management" not in service.snapshot(1)
    service.close()


def test_intent_committed_before_external_start_and_manifest_tracks_actual_bag(managed):
    service, adapter = managed
    result = start(service, adapter)
    operation = result["operation"]
    assert operation["state"] == "accepted" and operation["code"] == "queued"
    trial_id = operation["trial_id"]
    assert trial_id and not adapter.commands
    assert service.store.trial(trial_id)["state"] == "STARTING"
    assert not result["snapshot"]["capabilities"]["record"]
    execute(service)
    trial = service.store.trial(trial_id)
    assert trial["state"] == "RECORDING"
    assert trial["session_id"] == "session-start-one"
    assert trial["output_path"] == "/external/bags/session-test"
    manifest = json.loads((service.store.manifests/(trial_id+".json")).read_text())
    assert manifest["output_path"] == trial["output_path"]
    assert manifest["start_request_id"] == "start-one"
    assert manifest["quality"] == "NOT_ASSESSED"
    assert stat.S_IMODE((service.store.directory/"capture.sqlite3").stat().st_mode) == 0o600
    assert stat.S_IMODE((service.store.manifests/(trial_id+".json")).stat().st_mode) == 0o600
    assert service.snapshot(2)["capabilities"]["record"]
    assert not service.snapshot(1)["capabilities"]["record"]


def test_owned_stop_finalizes_without_fake_quality(managed):
    service, adapter = managed
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    result = stop(service, adapter, trial_id)
    assert result["ok"] and service.store.trial(trial_id)["state"] == "STOPPING"
    assert not result["snapshot"]["capabilities"]["record"]
    execute(service)
    trial = service.store.trial(trial_id)
    assert trial["state"] == "COMPLETE" and trial["elapsed_s"] == 3.5
    assert trial["quality"] == "NOT_ASSESSED"
    assert adapter.commands[-1]["expected_session_id"] == "session-start-one"
    assert adapter.commands[-1]["planned_duration_sec"] == 0


def test_same_id_is_deduplicated_and_changed_payload_rejected(managed):
    service, adapter = managed
    batch_id = create_batch(service)
    first = start(service, adapter, batch_id=batch_id)
    duplicate = start(service, adapter, batch_id=batch_id)
    assert first["operation"] == duplicate["operation"]
    assert service._commands.qsize() == 1
    conflict = service.dispatch(request("start-one", "batch.create", label="different"))
    assert not conflict["ok"] and conflict["operation"]["code"] == "request_id_conflict"
    execute(service)
    assert len(adapter.commands) == 1


def test_concurrent_duplicate_requests_create_one_intent(managed):
    service, adapter = managed
    batch_id = create_batch(service)
    with ThreadPoolExecutor(max_workers=8) as pool:
        values = list(pool.map(lambda _: start(service, adapter, batch_id=batch_id), range(16)))
    assert len({value["operation"]["trial_id"] for value in values}) == 1
    assert len(service.store.active_trials()) == 1
    assert service._commands.qsize() == 1


@pytest.mark.parametrize("behavior", ["timeout", "timeout_after_effect"])
def test_start_timeout_is_unknown_without_retry_and_snapshot_can_reconcile(managed, behavior):
    service, adapter = managed
    adapter.behavior = behavior
    result = start(service, adapter)
    trial_id = result["operation"]["trial_id"]
    execute(service)
    assert service.store.operation("start-one")["state"] == "unknown"
    assert service.store.trial(trial_id)["state"] == "UNKNOWN"
    service.poll_once()
    assert len(adapter.commands) == 1
    expected = "RECORDING" if behavior == "timeout_after_effect" else "UNKNOWN"
    assert service.store.trial(trial_id)["state"] == expected
    result = start(service, adapter, identifier="not-an-auto-retry", batch_id=service.store.trial(trial_id)["batch_id"])
    assert not result["ok"]
    assert len(adapter.commands) == 1


def test_stop_timeout_stays_unknown_until_effect_observed(managed):
    service, adapter = managed
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    adapter.behavior = "timeout"
    stop(service, adapter, trial_id)
    execute(service)
    service.poll_once()
    assert service.store.trial(trial_id)["state"] == "UNKNOWN"
    assert service.store.operation("stop-one")["state"] == "unknown"
    assert len(adapter.commands) == 2
    adapter.context.update(state=5, state_revision=adapter.context["state_revision"]+1, recorder_alive=False)
    service.poll_once()
    assert service.store.trial(trial_id)["state"] == "COMPLETE"
    assert service.store.operation("stop-one")["state"] == "accepted"
    assert len(adapter.commands) == 2


@pytest.mark.parametrize("field,value,code", [("expected_controller_instance_id", "old-instance", "controller_instance_mismatch"),
    ("expected_session_id", "old-session", "session_not_owned"), ("trial_id", "not-owned", "session_not_owned")])
def test_stale_stop_cannot_stop_another_session(managed, field, value, code):
    service, adapter = managed
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    params = dict(trial_id=trial_id, expected_controller_instance_id=adapter.context["controller_instance_id"],
                  expected_session_id=adapter.context["session_id"])
    params[field] = value
    result = service.dispatch(request("stale-stop", "capture.stop", **params))
    assert result["operation"]["code"] == code
    assert len(adapter.commands) == 1


def test_external_session_is_displayed_but_never_adopted(managed):
    service, adapter = managed
    adapter.context.update(state=3, state_revision=2, session_id="external", start_request_id="somebody-else", recorder_alive=True)
    service.poll_once()
    snapshot = service.snapshot(2)
    assert snapshot["management"]["controller"]["session_id"] == "external"
    assert snapshot["management"]["trials"] == []
    assert not snapshot["capabilities"]["record"]
    result = stop(service, adapter, "not-local")
    assert result["operation"]["code"] == "session_not_owned"
    assert not adapter.commands


def test_rejected_start_is_failed_and_preserves_receipt(managed):
    service, adapter = managed
    adapter.behavior = "reject"
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    assert service.store.trial(trial_id)["state"] == "FAILED"
    assert service.store.trial(trial_id)["detail"] == "rejected test command"
    assert service.store.operation("start-one")["state"] == "rejected"


def test_rejected_receipt_with_confirmed_recording_is_owned_not_restarted(managed):
    service, adapter = managed
    adapter.behavior = "reject_after_effect"
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    assert service.store.operation("start-one")["state"] == "rejected"
    assert service.store.trial(trial_id)["state"] == "RECORDING"
    assert service.store.trial(trial_id)["session_id"] == "session-start-one"
    assert len(adapter.commands) == 1
    assert service.snapshot(2)["capabilities"]["record"]


def test_invalid_controller_with_live_recorder_is_not_terminal(managed):
    service, adapter = managed
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    adapter.context.update(state=6, state_revision=3, recorder_alive=True, detail="writer failed")
    service.poll_once()
    trial = service.store.trial(trial_id)
    assert trial["state"] == "UNKNOWN"
    assert "still alive" in trial["detail"]
    assert trial["quality"] == "NOT_ASSESSED"
    assert not service.snapshot(2)["capabilities"]["record"]
    adapter.context.update(state_revision=4, recorder_alive=False)
    service.poll_once()
    assert service.store.trial(trial_id)["state"] == "FAILED"


def test_same_start_request_cannot_replace_confirmed_session_ownership(managed):
    service, adapter = managed
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    original = service.store.trial(trial_id)
    adapter.context.update(state_revision=3, session_id="unexpected-other-session", output_path="/other/bag")
    service.poll_once()
    trial = service.store.trial(trial_id)
    assert trial["state"] == "UNKNOWN"
    assert trial["session_id"] == original["session_id"]
    assert trial["output_path"] == original["output_path"]
    assert not service.snapshot(2)["capabilities"]["record"]
    result = stop(service, adapter, trial_id)
    assert result["operation"]["code"] == "session_not_owned"
    assert len(adapter.commands) == 1


def test_restart_recovers_owned_recording_without_sending_commands(tmp_path):
    adapter = FakeAdapter()
    directory = tmp_path/"state"
    first = ManagementService(model(), adapter, enabled=True, state_dir=directory, timeout_s=.01)
    adapter.store = first.store
    first.poll_once()
    trial_id = start(first, adapter)["operation"]["trial_id"]
    execute(first)
    first.close()
    assert len(adapter.commands) == 1  # close did not send STOP.
    second = ManagementService(model(), adapter, enabled=True, state_dir=directory, timeout_s=.01)
    adapter.store = second.store
    try:
        assert second.store.trial(trial_id)["state"] == "UNKNOWN"
        assert second.poll_once()
        assert second.store.trial(trial_id)["state"] == "RECORDING"
        assert second._commands.empty() and len(adapter.commands) == 1
        batch_id = second.store.trial(trial_id)["batch_id"]
        # Exact original request payload remains idempotent across restart.
        result = second.dispatch(request("start-one", "capture.start", batch_id=batch_id, label="试次", duration_s=30,
            expected_controller_instance_id="controller-one", expected_state_revision=1))
        assert result["operation"]["trial_id"] == trial_id
        assert second._commands.empty()
    finally:
        second.close()


def test_restart_after_intent_before_send_does_not_replay(tmp_path):
    adapter = FakeAdapter()
    directory = tmp_path/"state"
    first = ManagementService(model(), adapter, enabled=True, state_dir=directory, timeout_s=.01)
    adapter.store = first.store
    first.poll_once()
    trial_id = start(first, adapter)["operation"]["trial_id"]
    first.close()
    second = ManagementService(model(), adapter, enabled=True, state_dir=directory, timeout_s=.01)
    adapter.store = second.store
    try:
        second.poll_once()
        assert second.store.operation("start-one")["state"] == "unknown"
        assert second.store.trial(trial_id)["state"] == "UNKNOWN"
        assert second._commands.empty() and not adapter.commands
    finally:
        second.close()


def test_restart_does_not_bind_a_historical_session_to_an_older_unresolved_intent(managed):
    service, adapter = managed
    adapter.context.update(state=5, state_revision=7, session_id="external-old-session",
                           start_request_id="historical-start", output_path="/external/old-bag")
    assert service.poll_once()
    batch_id = create_batch(service)
    trial_id = uuid.uuid4().hex
    params = dict(batch_id=batch_id, label="new trial", duration_s=30,
                  expected_controller_instance_id="controller-one", expected_state_revision=7)
    # Represent an unresolved intent written by a manager predating the new
    # admission guard. Its durable revision must still prevent historical adoption.
    with service.store.transaction() as db:
        db.execute("INSERT INTO trials(id,batch_id,label,state,duration_s,created_at,controller_instance_id,start_request_id) "
                   "VALUES(?,?,?,'UNKNOWN',?,?,?,?)",
                   (trial_id, batch_id, "new trial", 30, utc_now(), "controller-one", "historical-start"))
        service.store.insert_operation(db, "historical-start", "capture.start", params,
            state="unknown", code="outcome_unknown", detail="Receipt was lost.", trial_id=trial_id, phase="UNKNOWN")
        service.store.insert_start_metadata(db, trial_id, dict(controller_before_start=adapter.context))
    directory = service.store.directory
    service.close()
    restarted = ManagementService(model(), adapter, enabled=True, state_dir=directory, timeout_s=.01)
    adapter.store = restarted.store
    try:
        assert restarted.poll_once()
        trial = restarted.store.trial(trial_id)
        assert trial["state"] == "UNKNOWN"
        assert not trial["session_id"] and not trial["output_path"]
        assert restarted.store.details(trial_id)["metadata"]["finish"] is None
        assert restarted.store.quality_job(trial_id) is None
        assert restarted._commands.empty() and not adapter.commands
    finally:
        restarted.close()


def test_controller_restart_or_disconnect_removes_control_capability(managed):
    service, adapter = managed
    assert service.snapshot(2)["capabilities"]["record"]
    adapter.available = False
    assert not service.poll_once()
    assert service.snapshot(2)["management"]["controller"] is None
    assert not service.snapshot(2)["capabilities"]["record"]
    adapter.available = True
    service.poll_once()
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    adapter.context = context(controller_instance_id="controller-restarted")
    service.poll_once()
    assert service.store.trial(trial_id)["state"] == "UNKNOWN"
    assert not service.snapshot(2)["capabilities"]["record"]


def test_private_state_exclusive_owner(managed):
    service, _ = managed
    with pytest.raises((BlockingIOError, OSError)):
        ManagementService(model(), enabled=True, state_dir=service.store.directory)


def test_future_database_schema_is_rejected_without_downgrade(tmp_path):
    directory = tmp_path/"future"
    directory.mkdir(mode=0o700)
    database = directory/"capture.sqlite3"
    with sqlite3.connect(database) as connection:
        connection.execute("PRAGMA user_version=99")
        connection.execute("CREATE TABLE preserve_me(value TEXT)")
        connection.execute("INSERT INTO preserve_me VALUES('future data')")
    with pytest.raises(ValueError, match="refusing to downgrade"):
        ManagementService(model(), enabled=True, state_dir=directory)
    with sqlite3.connect(database) as connection:
        assert connection.execute("PRAGMA user_version").fetchone()[0] == 99
        assert connection.execute("SELECT value FROM preserve_me").fetchone()[0] == "future data"
        assert connection.execute("SELECT name FROM sqlite_master WHERE name='trials'").fetchone() is None


def socket_call(path, value):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(2)
        client.connect(str(path))
        client.sendall(encode_message(value))
        data = bytearray()
        while not data.endswith(b"\n"):
            block = client.recv(65536)
            assert block
            data.extend(block)
        return decode_message(bytes(data))


def test_v1_v2_real_socket_share_backend_without_v1_control(managed, tmp_path):
    service, adapter = managed
    path = tmp_path/"socket"/"manager.sock"
    with SnapshotServer(service.model, path, management=service):
        old = socket_call(path, dict(version=1, request_id="old", command="snapshot"))
        assert "management" not in old["snapshot"] and not old["snapshot"]["capabilities"]["record"]
        new = socket_call(path, request("new", "snapshot"))
        assert new["snapshot"]["management"]["enabled"]
        assert new["snapshot"]["capabilities"]["record"]
        created = socket_call(path, request("batch", "batch.create", label="socket batch"))
        repeated = socket_call(path, request("batch", "batch.create", label="socket batch"))
        assert created["ok"] and repeated["operation"] == created["operation"]
        assert len(repeated["snapshot"]["management"]["batches"]) == 1
    assert not adapter.commands
