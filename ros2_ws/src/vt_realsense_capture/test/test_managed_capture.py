"""Guarded control tests: temporary files and fake Recorder, never rosbag/hardware."""

from dataclasses import asdict, replace
import json
from pathlib import Path

import pytest

from vt_realsense_capture.config import load_config
from vt_realsense_capture.controller import (
    CaptureController, CaptureCommandFact, CommandKind, RecorderHealth, RecorderStopFact,
)
from vt_realsense_capture.managed import ManagedCommandFact
from vt_realsense_capture.session import CaptureState, InvalidTransitionError
from vt_realsense_capture.storage import FileStorage


CONFIG = load_config(Path(__file__).parents[1] / "config/cameras.yaml")


@pytest.fixture(autouse=True)
def ample_disk_space(monkeypatch):
    # Lifecycle tests must not depend on the host's temporary-directory capacity.
    monkeypatch.setattr(FileStorage, "disk_free_bytes", lambda self, path: 1024**4)


class Clock:
    now = 10_000_000_000

    def monotonic_ns(self):
        return self.now


class Recorder:
    def __init__(self):
        self.starts = []
        self.stops = 0
        self.alive = True
        self.confirmed = True

    def start(self, spec):
        # This assertion proves the persistent intent precedes process creation.
        intent = json.loads((spec.session_dir / "controller_session.json").read_text())
        assert intent["lifecycle"] == "start_intent"
        assert intent["session_id"] == spec.session_id
        assert intent["output_path"] == str(spec.output_path)
        self.starts.append(spec)

    def health(self):
        return RecorderHealth.ALIVE if self.alive else RecorderHealth.EXITED

    def stop(self):
        self.stops += 1
        self.alive = not self.confirmed
        return RecorderStopFact(0 if self.confirmed else None, "fake stop", self.confirmed, not self.confirmed)


def make(tmp_path, *, storage=None, recorder=None):
    recorder = recorder or Recorder()
    clock = Clock()
    sequence = iter(range(100))
    controller = CaptureController(config=CONFIG, output_root=tmp_path, clock=clock,
        storage=storage or FileStorage(), recorder_factory=lambda: recorder,
        event_sink=lambda _: None, status_sink=lambda _: None, session_sink=lambda _: None,
        session_id_factory=lambda: f"session-{next(sequence)}")
    return controller, recorder, clock


def request(controller, request_id="start", command=1, **changes):
    current = controller.capture_context()
    result = ManagedCommandFact(request_id, command, current.controller_instance_id,
                               current.session_id, current.state_revision,
                               "trial" if command == 1 else "", 30 if command == 1 else 0)
    return replace(result, **changes)


def journal(tmp_path, session="session-0"):
    return json.loads((tmp_path / session / "controller_session.json").read_text())


def test_snapshot_revision_elapsed_and_original_start_mapping(tmp_path):
    controller, recorder, clock = make(tmp_path)
    before = controller.capture_context()
    assert before.state_revision == 0 and before.max_duration_sec == 300
    assert before.topics and before.elapsed_sec == 0
    command = request(controller)
    receipt = controller.accept_managed(command)
    assert receipt.accepted and receipt.code == "recorder_started"
    assert len(recorder.starts) == 1
    assert journal(tmp_path)["start_request_id"] == command.request_id
    assert journal(tmp_path)["start_parameters"]["session_label"] == "trial"
    active = controller.capture_context()
    assert active.start_request_id == "start" and active.session_label == "trial"
    assert active.output_path == str(tmp_path / "session-0/bag")
    assert active.planned_duration_sec == 30 and active.recorder_alive
    clock.now += 2_500_000_000
    controller.publish_periodic_status()
    later = controller.capture_context()
    assert later.elapsed_sec == 2.5 and later.state_revision == active.state_revision
    stopped = controller.accept_managed(request(controller, "stop", 2))
    assert stopped.accepted and stopped.context.state == CaptureState.COMPLETE
    assert stopped.context.start_request_id == "start"
    assert stopped.context.state_revision > later.state_revision
    clock.now += 100_000_000_000
    assert controller.capture_context().elapsed_sec == 2.5
    assert journal(tmp_path)["lifecycle"] == "complete"
    assert journal(tmp_path)["managed_requests"][-1]["request"]["request_id"] == "stop"


def test_exact_retries_keep_same_receipt_and_changed_payload_conflicts(tmp_path):
    controller, recorder, _ = make(tmp_path)
    command = request(controller)
    receipt = controller.accept_managed(command)
    assert controller.accept_managed(command) is receipt
    stop = request(controller, "stop", 2)
    stopped = controller.accept_managed(stop)
    assert controller.accept_managed(stop) is stopped
    assert controller.accept_managed(command) is receipt  # Original receipt, not a new start.
    for change in (dict(session_label="other"), dict(planned_duration_sec=10),
                   dict(expected_state_revision=999), dict(command=2)):
        assert controller.accept_managed(replace(command, **change)).code == "request_id_conflict"
    assert len(recorder.starts) == 1 and recorder.stops == 1


@pytest.mark.parametrize("changes,code", [
    ({"expected_controller_instance_id": "old-instance"}, "controller_instance_mismatch"),
    ({"expected_state_revision": 99}, "state_revision_mismatch"),
    ({"expected_session_id": "other"}, "session_mismatch"),
    ({"planned_duration_sec": 0}, "invalid_duration"),
    ({"planned_duration_sec": 301}, "invalid_duration"),
    ({"planned_duration_sec": True}, "invalid_request"),
    ({"command": 9}, "invalid_request"),
])
def test_start_guards_have_no_filesystem_or_recorder_effect(tmp_path, changes, code):
    controller, recorder, _ = make(tmp_path)
    result = controller.accept_managed(request(controller, **changes))
    assert not result.accepted and result.code == code
    assert not recorder.starts and not list(tmp_path.iterdir())


def test_active_start_and_wrong_session_stop_cannot_affect_current_recorder(tmp_path):
    controller, recorder, _ = make(tmp_path)
    controller.accept_managed(request(controller))
    busy = controller.accept_managed(request(controller, "second-start"))
    assert busy.code == "active_session" and not busy.accepted
    wrong = controller.accept_managed(request(controller, "wrong-stop", 2, expected_session_id="another"))
    assert wrong.code == "session_mismatch" and not wrong.accepted
    assert recorder.stops == 0 and len(recorder.starts) == 1


def test_old_stop_cannot_stop_next_session(tmp_path):
    controller, recorder, _ = make(tmp_path)
    controller.accept_managed(request(controller))
    stop = request(controller, "stop", 2)
    receipt = controller.accept_managed(stop)
    controller.accept_managed(request(controller, "next-start"))
    assert controller.accept_managed(stop) is receipt
    stale = replace(stop, request_id="delayed-stop", expected_state_revision=controller.capture_context().state_revision)
    assert controller.accept_managed(stale).code == "session_mismatch"
    assert recorder.stops == 1


class FailingStorage(FileStorage):
    def __init__(self, fail_from):
        self.calls = 0
        self.fail_from = fail_from

    def write_controller_session(self, directory, document):
        self.calls += 1
        if self.calls >= self.fail_from:
            raise OSError("injected journal disk failure")
        super().write_controller_session(directory, document)


@pytest.mark.parametrize("fail_from,started", [(1, False), (2, True)])
def test_sidecar_failure_is_not_success_and_never_automatically_stops(tmp_path, fail_from, started):
    controller, recorder, _ = make(tmp_path, storage=FailingStorage(fail_from))
    command = request(controller)
    result = controller.accept_managed(command)
    assert not result.accepted and result.code == "session_journal_failed"
    assert bool(recorder.starts) == started
    assert result.context.recorder_alive == started
    assert recorder.stops == 0
    assert controller.accept_managed(command) is result
    assert len(recorder.starts) == int(started)


def test_stop_still_terminates_when_sidecar_becomes_unwritable(tmp_path):
    storage = FailingStorage(100)
    controller, recorder, _ = make(tmp_path, storage=storage)
    assert controller.accept_managed(request(controller)).accepted
    storage.fail_from = storage.calls + 1
    command = request(controller, "stop", 2)
    result = controller.accept_managed(command)
    assert not result.accepted and result.code == "session_journal_failed"
    assert result.context.state == CaptureState.COMPLETE and not result.context.recorder_alive
    assert controller.accept_managed(command) is result and recorder.stops == 1


def test_restart_exposes_unresolved_original_mapping_without_process_adoption(tmp_path):
    first, recorder, _ = make(tmp_path)
    command = request(first)
    first.accept_managed(command)
    second, untouched, _ = make(tmp_path)
    context = second.capture_context()
    assert context.controller_instance_id != command.expected_controller_instance_id
    assert context.session_id == "session-0" and context.start_request_id == "start"
    assert context.state == CaptureState.INVALID and context.recorder_alive
    assert "recovery_required" in context.detail
    assert second.accept_managed(command).code == "controller_instance_mismatch"
    assert second.accept_managed(request(second, "new-start")).code == "recovery_required"
    assert second.accept_managed(request(second, "new-stop", 2)).code == "recovery_required"
    with pytest.raises(InvalidTransitionError, match="recovery_required"):
        second.accept(CaptureCommandFact("legacy", CommandKind.START))
    assert second.shutdown() is False
    status = second.publish_periodic_status()
    assert status.state == CaptureState.INVALID and status.recorder_alive
    assert status.session_id == "session-0" and "recovery_required" in status.detail
    assert second.capture_context().state_revision == context.state_revision
    assert not untouched.starts and untouched.stops == recorder.stops == 0


def test_completed_journal_does_not_claim_old_session_active_after_restart(tmp_path):
    first, _, _ = make(tmp_path)
    first.accept_managed(request(first))
    first.accept_managed(request(first, "stop", 2))
    second, _, _ = make(tmp_path)
    assert second.capture_context().state == CaptureState.IDLE
    assert not second.capture_context().recorder_alive
    assert second.capture_context().controller_instance_id != first.capture_context().controller_instance_id


def test_legacy_topic_start_is_projected_and_zero_duration_still_means_max(tmp_path):
    controller, recorder, clock = make(tmp_path)
    controller.accept(CaptureCommandFact("legacy-start", CommandKind.START, "external trial", 0))
    context = controller.capture_context()
    assert context.start_request_id == "legacy-start" and context.session_label == "external trial"
    assert context.planned_duration_sec == 300
    clock.now += 300_000_000_000
    controller.tick()
    assert controller.capture_context().state == CaptureState.COMPLETE
    assert recorder.stops == 1


def test_unconfirmed_stop_stays_finalizing_and_receipt_retry_does_not_repeat_stop(tmp_path):
    controller, recorder, _ = make(tmp_path)
    controller.accept_managed(request(controller))
    recorder.confirmed = False
    command = request(controller, "stop", 2)
    result = controller.accept_managed(command)
    assert result.accepted and result.context.state == CaptureState.FINALIZING
    assert result.context.recorder_alive and journal(tmp_path)["lifecycle"] == "finalizing"
    assert controller.accept_managed(command) is result and recorder.stops == 1
    recorder.confirmed = True
    controller.tick()
    assert controller.capture_context().state == CaptureState.COMPLETE and recorder.stops == 2


def test_atomic_journal_replacement_failure_preserves_previous_data(tmp_path, monkeypatch):
    storage = FileStorage()
    storage.write_controller_session(tmp_path, {"version": 1, "lifecycle": "start_intent"})
    def fail(*_args):
        raise OSError("replace failed")
    monkeypatch.setattr("vt_realsense_capture.storage.os.replace", fail)
    with pytest.raises(OSError):
        storage.write_controller_session(tmp_path, {"version": 1, "lifecycle": "recording"})
    assert json.loads((tmp_path / "controller_session.json").read_text())["lifecycle"] == "start_intent"
    assert not list(tmp_path.glob(".controller_session-*"))


def test_corrupt_journal_requires_review_instead_of_starting(tmp_path):
    directory = tmp_path / "previous"
    directory.mkdir()
    (directory / "controller_session.json").write_text("not-json")
    controller, recorder, _ = make(tmp_path)
    assert controller.accept_managed(request(controller)).code == "recovery_required"
    assert not recorder.starts


def test_legacy_same_id_changed_parameters_conflict_without_second_spawn(tmp_path):
    from vt_realsense_capture.session import RequestConflictError
    controller, recorder, _ = make(tmp_path)
    command = CaptureCommandFact("legacy", CommandKind.START, "trial", 30)
    controller.accept(command)
    with pytest.raises(RequestConflictError):
        controller.accept(replace(command, session_label="another"))
    assert len(recorder.starts) == 1


def test_rejected_managed_id_cannot_be_reinterpreted_through_legacy_topic(tmp_path):
    from vt_realsense_capture.session import RequestConflictError
    controller, recorder, _ = make(tmp_path)
    invalid = request(controller, command=9)
    first = controller.accept_managed(invalid)
    assert controller.accept_managed(invalid) is first
    assert controller.accept_managed(replace(invalid, command=1)).code == "request_id_conflict"
    with pytest.raises(RequestConflictError):
        controller.accept(CaptureCommandFact(invalid.request_id, CommandKind.START, "trial", 30))
    assert not recorder.starts


def test_disk_budget_rejection_has_command_idempotency_without_session_creation(tmp_path, monkeypatch):
    monkeypatch.setattr(FileStorage, "disk_free_bytes", lambda self, path: 1)
    controller, recorder, _ = make(tmp_path)
    command = request(controller)
    receipt = controller.accept_managed(command)
    assert not receipt.accepted and receipt.code == "insufficient_disk_space"
    assert "磁盘空间不足" in receipt.detail
    assert not list(tmp_path.iterdir()) and not recorder.starts
    monkeypatch.setattr(FileStorage, "disk_free_bytes", lambda self, path: 1024**4)
    assert controller.accept_managed(command) is receipt
    assert controller.accept_managed(request(controller, "new-request")).accepted
    assert len(recorder.starts) == 1


def test_next_start_disk_rejection_does_not_rewrite_completed_session_journal(tmp_path, monkeypatch):
    controller, recorder, _ = make(tmp_path)
    assert controller.accept_managed(request(controller)).accepted
    assert controller.accept_managed(request(controller, "stop", 2)).accepted
    path = tmp_path / "session-0/controller_session.json"
    completed_bytes = path.read_bytes()
    monkeypatch.setattr(FileStorage, "disk_free_bytes", lambda self, path: 1)
    result = controller.accept_managed(request(controller, "next-start"))
    assert result.code == "insufficient_disk_space" and not result.accepted
    assert path.read_bytes() == completed_bytes
    assert not (tmp_path / "session-1").exists()
    assert len(recorder.starts) == 1 and recorder.stops == 1


@pytest.mark.parametrize("query_failure", [False, True])
def test_disk_guard_termination_reason_is_persisted_in_completed_journal(tmp_path, monkeypatch, query_failure):
    controller, recorder, clock = make(tmp_path)
    assert controller.accept_managed(request(controller)).accepted
    if query_failure:
        def fail(self, path):
            raise OSError("query failed")
        monkeypatch.setattr(FileStorage, "disk_free_bytes", fail)
        reason = "磁盘空间查询失败"
    else:
        monkeypatch.setattr(FileStorage, "disk_free_bytes", lambda self, path: 0)
        reason = "磁盘空间达到停止阈值"
    clock.now += 500_000_000
    controller.tick()
    assert recorder.stops == 1
    assert controller.state is CaptureState.COMPLETE
    record = journal(tmp_path)
    assert record["lifecycle"] == "complete"
    assert not record["recorder_alive"]
    assert reason in record["detail"]
    assert reason in controller.capture_context().detail


def test_disk_guard_stop_survives_journal_write_failure(tmp_path, monkeypatch):
    storage = FailingStorage(100)
    controller, recorder, clock = make(tmp_path, storage=storage)
    assert controller.accept_managed(request(controller)).accepted
    storage.fail_from = storage.calls + 1
    monkeypatch.setattr(FileStorage, "disk_free_bytes", lambda self, path: 0)
    clock.now += 500_000_000
    controller.tick()
    assert recorder.stops == 1
    assert controller.state is CaptureState.COMPLETE
    assert "session_journal_failed" in controller.capture_context().detail
    assert "磁盘空间达到停止阈值" in controller.capture_context().detail


@pytest.mark.parametrize("returncode,detail", [(0, "clean_exit"), (17, "nonzero_exit"), (-9, "forced_kill")])
def test_finalization_persists_process_facts_without_a_quality_verdict(tmp_path, returncode, detail):
    controller, recorder, _ = make(tmp_path)
    assert controller.accept_managed(request(controller)).accepted
    recorder.stop = lambda: RecorderStopFact(returncode, detail, True, False)
    assert controller.accept_managed(request(controller, "stop", 2)).accepted
    record = journal(tmp_path)
    assert record["lifecycle"] == "complete"
    assert record["state"] == CaptureState.COMPLETE
    assert record["finalization"] == dict(reason="STOP requested", returncode=returncode,
        stop_detail=detail, termination_confirmed=True, possibly_alive=False)
    assert "quality not evaluated" in record["detail"]


def test_finalization_retry_keeps_original_stop_reason_and_records_uncertainty(tmp_path):
    controller, recorder, _ = make(tmp_path)
    controller.accept_managed(request(controller))
    recorder.confirmed = False
    controller.accept_managed(request(controller, "stop", 2))
    pending = journal(tmp_path)["finalization"]
    assert pending["reason"] == "STOP requested" and not pending["termination_confirmed"]
    recorder.confirmed = True
    controller.tick()
    final = journal(tmp_path)["finalization"]
    assert final["reason"] == "STOP requested" and final["termination_confirmed"]


def test_stop_publication_failure_cannot_prevent_termination_or_erase_its_reason(tmp_path):
    controller, recorder, _ = make(tmp_path)
    controller.accept_managed(request(controller))
    recorder.confirmed = False

    def fail_finalizing(event):
        if event.code == "finalizing":
            assert journal(tmp_path)["finalization"]["reason"] == "STOP requested"
            raise OSError("injected event publisher failure")

    controller._event_sink = fail_finalizing
    receipt = controller.accept_managed(request(controller, "stop", 2))
    assert not receipt.accepted and receipt.context.state == CaptureState.FINALIZING
    assert recorder.stops == 1
    assert journal(tmp_path)["finalization"]["reason"] == "STOP requested"
    recorder.confirmed = True
    controller.tick()
    assert recorder.stops == 2 and controller.state == CaptureState.COMPLETE
    assert journal(tmp_path)["finalization"]["reason"] == "STOP requested"
