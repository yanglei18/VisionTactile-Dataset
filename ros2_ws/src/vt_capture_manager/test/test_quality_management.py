"""Metadata and durable quality scheduling without hardware or actual recorders."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sqlite3
import threading
import time

import pytest

from vt_capture_core import encode_message, decode_message
from vt_capture_core.validation import validate_trial_details
from vt_capture_manager.config import ManagerConfig, load_config
from vt_capture_manager.management import ManagementService
from vt_capture_manager.metadata import MetadataCollector, observe_reference
from vt_capture_manager.model import SnapshotModel
from vt_capture_manager.quality_runner import QualityCancelled
from vt_capture_manager.server import SnapshotServer
from vt_capture_manager.storage import IntentStore
from test_management import FakeAdapter, create_batch, execute, start, stop, request, socket_call


class FakeQuality:
    def __init__(self, *, error=False, blocking=False):
        self.calls = []
        self.error, self.blocking = error, blocking
        self.entered, self.cancelled = threading.Event(), threading.Event()

    def run(self, bag, report_path, *, expected_topics, alignment_config=None, advisories=(),
            expected_image_rates_hz=None, optional_topics=(), cancel=lambda: False):
        self.calls.append(dict(bag=bag, report_path=report_path, expected_topics=expected_topics,
                               alignment_content=Path(alignment_config).read_bytes() if alignment_config else None,
                               advisories=list(advisories), expected_image_rates_hz=expected_image_rates_hz,
                               optional_topics=list(optional_topics)))
        self.entered.set()
        if self.blocking:
            while not cancel():
                time.sleep(.005)
            self.cancelled.set()
            raise QualityCancelled("test preemption")
        if self.error:
            raise RuntimeError("test runner failure")
        warnings = list(advisories)
        report = dict(schema_version=1, report_kind="post_record_capture_quality", status="warnings" if warnings else "ok",
                      summary=dict(warning_count=len(warnings)), warnings=warnings, topics={})
        Path(report_path).write_text(json.dumps(report))
        return report


def make(tmp_path, *, runner=None, config=None):
    adapter = FakeAdapter()
    service = ManagementService(SnapshotModel(config or ManagerConfig(())), adapter, enabled=True,
                                state_dir=tmp_path/"state", timeout_s=.02, quality_runner=runner or FakeQuality())
    adapter.store = service.store
    service.poll_once()
    return service, adapter


@pytest.fixture
def quality_managed(tmp_path):
    service, adapter = make(tmp_path)
    yield service, adapter
    service.close()


def complete(service, adapter, identifier="start-one", batch_id=None):
    result = start(service, adapter, identifier=identifier, batch_id=batch_id)
    trial_id = result["operation"]["trial_id"]
    execute(service)
    stop(service, adapter, trial_id, identifier="stop-"+identifier)
    execute(service)
    return trial_id


def test_default_quality_is_enabled_but_read_only_creates_nothing(tmp_path):
    assert load_config().quality_enabled
    service = ManagementService(SnapshotModel(ManagerConfig(())), state_dir=tmp_path/"absent")
    service.start()
    assert service._quality is None and not (tmp_path/"absent").exists()
    service.close()


def test_start_and_finish_frozen_once_and_separate_from_quality(quality_managed):
    service, adapter = quality_managed
    trial_id = complete(service, adapter)
    details = service.store.details(trial_id)
    assert details["quality"]["state"] == "QUEUED"
    assert details["metadata"]["finish"]["controller"]["recorder_alive"] is False
    assert details["metadata"]["start"]["controller_before_start"]["state"] == 0
    assert "not this trial" in details["metadata"]["start"]["controller_context_semantics"]
    assert "uncommitted" in details["metadata"]["start"]["git_commit_scope"]
    start_bytes = Path(details["artifacts"]["start_metadata_path"]).read_bytes()
    finish_bytes = Path(details["artifacts"]["finish_metadata_path"]).read_bytes()
    for _ in range(5):
        service.poll_once()
    assert service.store.quality_job(trial_id)["attempt"] == 0
    assert Path(details["artifacts"]["start_metadata_path"]).read_bytes() == start_bytes
    assert Path(details["artifacts"]["finish_metadata_path"]).read_bytes() == finish_bytes
    with pytest.raises(sqlite3.IntegrityError):
        with service.store.transaction() as db:
            db.execute("UPDATE trial_metadata SET start_json='{}' WHERE trial_id=?", (trial_id,))
    assert service._create_quality_scheduler().run_once()
    assert service._quality_runner.calls[0]["expected_image_rates_hz"] == {}
    details = service.store.details(trial_id)
    validate_trial_details(details)
    assert details["quality"]["state"] == "COMPLETED" and details["quality"]["verdict"] == "ok"
    assert service.store.trial(trial_id)["state"] == "COMPLETE"
    assert service.store.trial(trial_id)["quality"] == "OK"
    assert service._create_quality_scheduler().run_once() is False
    manifest = json.loads((service.store.manifests/(trial_id+".json")).read_text())
    assert manifest["quality"] == "OK" and manifest["quality_details"]["state"] == "COMPLETED"


def test_invalid_live_recorder_never_freezes_or_queues(quality_managed):
    service, adapter = quality_managed
    trial_id = start(service, adapter)["operation"]["trial_id"]
    execute(service)
    adapter.context.update(state=6, state_revision=3, recorder_alive=True)
    service.poll_once()
    assert service.store.details(trial_id)["metadata"]["finish"] is None
    assert service.store.quality_job(trial_id) is None
    adapter.context.update(state_revision=4, recorder_alive=False)
    service.poll_once()
    assert service.store.trial(trial_id)["state"] == "FAILED"
    assert service.store.quality_job(trial_id)["state"] == "QUEUED"


def test_each_start_refreshes_refs_and_alignment_uses_frozen_copy(tmp_path):
    reference = tmp_path/"calibration.json"
    reference.write_text('{"version":1}')
    alignment = tmp_path/"alignment.yaml"
    alignment.write_text("version: first\n")
    config = ManagerConfig((), metadata_refs=({"name": "mounting", "kind": "calibration", "path": str(reference)},),
                           quality_alignment_config=str(alignment))
    service, adapter = make(tmp_path, config=config)
    try:
        trial_id = complete(service, adapter)
        old = service.store.details(trial_id)["metadata"]["start"]
        reference.write_text('{"version":2}')
        alignment.write_text("version: second\n")
        service._create_quality_scheduler().run_once()
        assert service._quality_runner.calls[0]["alignment_content"] == b"version: first\n"
        batch_id = service.store.trial(trial_id)["batch_id"]
        next_trial = start(service, adapter, identifier="start-two", batch_id=batch_id)["operation"]["trial_id"]
        new = service.store.details(next_trial)["metadata"]["start"]
        assert old["references"][0]["sha256"] != new["references"][0]["sha256"]
        assert new["references"][0]["sha256"] == hashlib.sha256(reference.read_bytes()).hexdigest()
        assert old["references"][0]["detail"].startswith("File content observation")
    finally:
        service.close()


def test_missing_refs_do_not_block_start_and_alignment_warning_is_consistent(tmp_path):
    config = ManagerConfig((), quality_alignment_config=str(tmp_path/"missing.yaml"),
                           metadata_refs=({"name": "missing", "kind": "config", "path": str(tmp_path/"missing.json")},))
    service, adapter = make(tmp_path, config=config)
    try:
        trial_id = complete(service, adapter)
        assert service.store.details(trial_id)["metadata"]["start"]["references"][0]["status"] == "unknown"
        service._create_quality_scheduler().run_once()
        quality = service.store.details(trial_id)["quality"]
        disk = json.loads(Path(quality["report_path"]).read_text())
        assert quality["verdict"] == disk["status"] == "warnings"
        assert quality["report"]["warnings"] == disk["warnings"]
        assert disk["summary"]["warning_count"] == 1
        assert service.store.trial(trial_id)["quality"] == "WARNINGS"
    finally:
        service.close()


@pytest.mark.parametrize("name", ["MANO_LEFT.pkl", "credentials.json", "secret.yaml", "token.json"])
def test_sensitive_or_model_references_are_not_read(tmp_path, monkeypatch, name):
    path = tmp_path/name
    path.write_bytes(b"do not inspect")
    def forbidden(*args, **kwargs):
        raise AssertionError("forbidden reference was opened")
    monkeypatch.setattr("vt_capture_manager.metadata.os.open", forbidden)
    observation, content = observe_reference(dict(name="test", kind="config", path=str(path)))
    assert observation["status"] == "unknown" and content is None


def test_hash_preparation_runs_outside_control_lock(quality_managed):
    service, adapter = quality_managed
    original = service._metadata.prepare
    def prepare():
        assert not service._mutex._is_owned()
        return original()
    service._metadata.prepare = prepare
    assert start(service, adapter)["ok"]


def test_metadata_failure_is_explicit_unknown_not_a_recording_gate(quality_managed):
    service, adapter = quality_managed
    def fail(*args):
        raise OSError("metadata unavailable")
    service._metadata.start_document = fail
    result = start(service, adapter)
    assert result["ok"]
    details = service.store.details(result["operation"]["trial_id"])
    assert details["metadata"]["start"]["metadata_status"] == "unknown"
    assert service._commands.qsize() == 1


def test_v1_database_migration_preserves_history_without_backfilling_quality(tmp_path):
    service, adapter = make(tmp_path, config=ManagerConfig((), quality_enabled=False))
    trial_id = complete(service, adapter)
    before_trial = service.store.trial(trial_id)
    before_operation = service.store.operation("start-one")
    service.close()
    database = tmp_path/"state/capture.sqlite3"
    # Recreate the v0.2 schema exactly by removing only v0.3 additive tables.
    # Leftover manifests deliberately exist: migration must not scan/adopt them.
    with sqlite3.connect(database) as db:
        for table in ("trial_metadata", "alignment_snapshots", "quality_jobs", "quality_attempts"):
            db.execute("DROP TABLE "+table)
        db.execute("PRAGMA user_version=1")
    second, _ = make(tmp_path)
    try:
        assert second.store.trial(trial_id) == before_trial
        assert second.store.operation("start-one") == before_operation
        assert second.store.details(trial_id)["metadata"] == dict(start=None, finish=None)
        assert second.store.quality_job(trial_id) is None
        result = second.dispatch(dict(version=3, request_id="legacy-retry", command="quality.retry", params={"trial_id": trial_id}))
        assert result["operation"]["code"] == "quality_not_eligible"
        with sqlite3.connect(database) as db:
            assert db.execute("PRAGMA user_version").fetchone()[0] == 2
    finally:
        second.close()


def test_large_report_keeps_full_file_and_explicitly_bounds_details(quality_managed):
    service, adapter = quality_managed
    trial_id = complete(service, adapter)
    original = service._quality_runner.run
    def large(*args, **kwargs):
        report = original(*args, **kwargs)
        report["warnings"] = [dict(code="test", detail="observation"*1000) for _ in range(500)]
        report["summary"]["warning_count"] = 500
        report["status"] = "warnings"
        Path(args[1]).write_text(json.dumps(report))
        return report
    service._quality_runner.run = large
    service._create_quality_scheduler().run_once()
    details = service.store.details(trial_id)
    validate_trial_details(details)
    assert details["quality"]["report"]["preview_truncated"]
    full = json.loads(Path(details["quality"]["report_path"]).read_text())
    assert len(full["warnings"]) == 500
    assert len(json.dumps(details).encode()) <= 512*1024


def test_quality_error_and_explicit_retry_are_durable_and_do_not_change_record_state(quality_managed):
    service, adapter = quality_managed
    service._quality_runner.error = True
    trial_id = complete(service, adapter)
    service._create_quality_scheduler().run_once()
    assert service.store.trial(trial_id)["state"] == "COMPLETE"
    assert service.store.trial(trial_id)["quality"] == "ERROR"
    command = dict(version=3, request_id="retry", command="quality.retry", params={"trial_id": trial_id})
    first = service.dispatch(command)
    repeated = service.dispatch(command)
    assert first["operation"] == repeated["operation"]
    assert service.store.quality_job(trial_id)["attempt"] == 1
    service._quality_runner.error = False
    service._create_quality_scheduler().run_once()
    assert service.store.quality_job(trial_id)["attempt"] == 2
    assert service.store.trial(trial_id)["quality"] == "OK"
    assert len(adapter.commands) == 2  # No extra recorder START or STOP.


def test_disabled_quality_rejects_retry(tmp_path):
    service, adapter = make(tmp_path, config=ManagerConfig((), quality_enabled=False))
    try:
        trial_id = complete(service, adapter)
        result = service.dispatch(dict(version=3, request_id="retry", command="quality.retry", params={"trial_id": trial_id}))
        assert result["operation"]["code"] == "quality_disabled"
        assert service.store.quality_job(trial_id)["state"] == "NOT_ASSESSED"
    finally:
        service.close()


def test_running_quality_recovery_uses_new_attempt_and_preserves_existing_report(tmp_path):
    service, adapter = make(tmp_path)
    trial_id = complete(service, adapter)
    claimed = service.store.claim_quality()
    Path(claimed["report_path"]).write_text("previous completed report before DB commit")
    service.close()
    second, second_adapter = make(tmp_path)
    second_adapter.context = deepcopy(adapter.context)
    second.poll_once()
    try:
        assert second.store.quality_job(trial_id)["state"] == "QUEUED"
        second._create_quality_scheduler().run_once()
        assert second.store.quality_job(trial_id)["attempt"] == 2
        assert Path(claimed["report_path"]).read_text() == "previous completed report before DB commit"
        assert not second_adapter.commands
    finally:
        second.close()


def test_new_start_preempts_read_only_quality_and_requeues_without_blocking(tmp_path):
    runner = FakeQuality(blocking=True)
    service, adapter = make(tmp_path, runner=runner)
    try:
        old_trial = complete(service, adapter)
        scheduler = service._create_quality_scheduler().start()
        assert runner.entered.wait(1)
        before = time.monotonic()
        result = start(service, adapter, identifier="new-start", batch_id=service.store.trial(old_trial)["batch_id"])
        assert result["ok"] and time.monotonic()-before < .5
        assert runner.cancelled.wait(1)
        deadline = time.monotonic()+1
        while service.store.quality_job(old_trial)["state"] != "INTERRUPTED" and time.monotonic() < deadline:
            time.sleep(.005)
        assert service.store.quality_job(old_trial)["state"] == "INTERRUPTED"
        assert service.store.trial(old_trial)["state"] == "COMPLETE"
        assert len(adapter.commands) == 2  # New START is only queued, never a QC side effect.
    finally:
        service.close()


def test_v3_details_socket_and_v1_v2_strict_projection(quality_managed, tmp_path):
    service, adapter = quality_managed
    batch = create_batch(service)
    value = service.dispatch(dict(version=3, request_id="named-start", command="capture.start", params=dict(
        batch_id=batch, label="trial", duration_s=15, expected_controller_instance_id="controller-one", expected_state_revision=1,
        task_name="assembly", operator_name="operator A")))
    trial_id = value["operation"]["trial_id"]
    path = tmp_path/"socket"/"manager.sock"
    with SnapshotServer(service.model, path, management=service):
        details = socket_call(path, dict(version=3, request_id="details", command="trial.details", params={"trial_id": trial_id}))
        assert details["trial_details"]["metadata"]["start"]["task_name"] == "assembly"
        assert details["trial_details"]["metadata"]["start"]["operator_name"] == "operator A"
        for version in (1, 2, 3):
            reply = socket_call(path, dict(version=version, request_id="snapshot", command="snapshot"))
            assert "trial_details" not in reply
            assert ("management" in reply["snapshot"]) == (version != 1)
            if version != 1:
                assert set(reply["snapshot"]["management"]["trials"][0]) == {
                    "id", "batch_id", "label", "state", "session_id", "output_path", "duration_s", "elapsed_s", "quality", "detail", "created_at"}
