"""Quality queue storage faults use scratch SQLite and a synthetic runner only."""
import json
import sqlite3
import threading
import time

import pytest

from test_management import start
from test_quality_management import complete, make


def wait_for(predicate, timeout=2.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    assert predicate()


def observe_storage_errors(service):
    observed = threading.Event()
    original = service.model.event
    def event(level, detail, **kwargs):
        original(level, detail, **kwargs)
        if kwargs.get("key") == "quality-storage":
            observed.set()
    service.model.event = event
    return observed


def test_sqlite_write_lock_reports_failure_then_resumes_same_queued_job(tmp_path):
    service, adapter = make(tmp_path)
    service.context_ttl_s = 10.
    blocker = None
    try:
        trial_id = complete(service, adapter)
        errors = observe_storage_errors(service)
        service.store._db.execute("PRAGMA busy_timeout=20")
        blocker = sqlite3.connect(service.store.directory / "capture.sqlite3", isolation_level=None)
        blocker.execute("BEGIN IMMEDIATE")
        scheduler = service._create_quality_scheduler().start()
        assert errors.wait(1.), "storage failure must be visible without killing the worker"
        assert scheduler._thread.is_alive()
        assert not service._quality_runner.calls
        blocker.execute("ROLLBACK")
        blocker.close()
        blocker = None
        scheduler.wake()
        wait_for(lambda: service.store.quality_job(trial_id)["state"] == "COMPLETED")
        assert len(service._quality_runner.calls) == 1
        assert service.store.quality_job(trial_id)["attempt"] == 1
        assert len(adapter.commands) == 2
    finally:
        if blocker is not None:
            blocker.close()
        service.close()


@pytest.mark.parametrize("new_recording", [False, True])
def test_failed_result_commit_is_retried_without_rerunning_quality(tmp_path, monkeypatch, new_recording):
    service, adapter = make(tmp_path)
    service.context_ttl_s = 10.
    repaired = threading.Event()
    try:
        trial_id = complete(service, adapter)
        errors = observe_storage_errors(service)
        original = service.store.finish_quality
        def finish(*args, **kwargs):
            if not repaired.is_set():
                raise sqlite3.OperationalError("injected temporary result commit failure")
            return original(*args, **kwargs)
        monkeypatch.setattr(service.store, "finish_quality", finish)
        scheduler = service._create_quality_scheduler().start()
        assert errors.wait(1.)
        assert scheduler._thread.is_alive()
        assert len(service._quality_runner.calls) == 1
        assert service.store.quality_job(trial_id)["state"] == "RUNNING"
        if new_recording:
            response = start(service, adapter, identifier="next-start", batch_id=service.store.trial(trial_id)["batch_id"])
            assert response["operation"]["code"] == "queued"
        repaired.set()
        scheduler.wake()
        wait_for(lambda: service.store.quality_job(trial_id)["state"] == "COMPLETED")
        assert service.store.quality_job(trial_id)["attempt"] == 1
        assert len(service._quality_runner.calls) == 1
        assert len(adapter.commands) == 2
    finally:
        repaired.set()
        service.close()


def test_storage_failure_after_claim_releases_attempt_before_retry(tmp_path, monkeypatch):
    service, adapter = make(tmp_path)
    service.context_ttl_s = 10.
    try:
        trial_id = complete(service, adapter)
        errors = observe_storage_errors(service)
        original = service.store.alignment_snapshot
        failed = False
        def alignment(identifier):
            nonlocal failed
            if not failed:
                failed = True
                raise sqlite3.OperationalError("injected temporary metadata read failure")
            return original(identifier)
        monkeypatch.setattr(service.store, "alignment_snapshot", alignment)
        scheduler = service._create_quality_scheduler().start()
        assert errors.wait(1.)
        wait_for(lambda: service.store.quality_job(trial_id)["state"] == "COMPLETED")
        assert scheduler._thread.is_alive()
        assert service.store.quality_job(trial_id)["attempt"] == 2
        assert len(service._quality_runner.calls) == 1
        assert len(adapter.commands) == 2
        attempts = service.store._db.execute("SELECT state FROM quality_attempts WHERE trial_id=? ORDER BY attempt", (trial_id,)).fetchall()
        assert [row["state"] for row in attempts] == ["INTERRUPTED", "COMPLETED"]
    finally:
        service.close()


def test_storage_retry_backoff_is_bounded_and_shutdown_interruptible(tmp_path, monkeypatch):
    service, adapter = make(tmp_path)
    service.context_ttl_s = 10.
    try:
        trial_id = complete(service, adapter)
        errors = observe_storage_errors(service)
        claims = []
        def unavailable():
            claims.append(time.monotonic())
            raise sqlite3.OperationalError("injected persistent queue failure")
        monkeypatch.setattr(service.store, "claim_quality", unavailable)
        scheduler = service._create_quality_scheduler().start()
        assert errors.wait(1.)
        for _ in range(20):
            scheduler.wake()
            time.sleep(.005)
        before_close = time.monotonic()
        scheduler.close()
        assert time.monotonic() - before_close < .5
        assert not scheduler._thread.is_alive()
        assert claims
        assert all(later - earlier >= .08 for earlier, later in zip(claims, claims[1:]))
        assert not service._quality_runner.calls
        assert service.store.quality_job(trial_id)["state"] == "QUEUED"
        assert len(adapter.commands) == 2
    finally:
        service.close()


def test_database_failure_during_final_manifest_retries_without_losing_result(tmp_path, monkeypatch):
    service, adapter = make(tmp_path)
    service.context_ttl_s = 10.
    try:
        trial_id = complete(service, adapter)
        errors = observe_storage_errors(service)
        original = service.store.write_manifest
        failed = False
        def manifest(identifier):
            nonlocal failed
            if not failed and service.store.quality_job(identifier)["state"] == "COMPLETED":
                failed = True
                raise sqlite3.OperationalError("injected final manifest metadata read failure")
            return original(identifier)
        monkeypatch.setattr(service.store, "write_manifest", manifest)
        scheduler = service._create_quality_scheduler().start()
        assert errors.wait(1.)
        path = service.store.manifests / (trial_id + ".json")
        wait_for(lambda: json.loads(path.read_text())["quality"] == "OK")
        assert scheduler._thread.is_alive()
        assert service.store.quality_job(trial_id)["attempt"] == 1
        assert len(service._quality_runner.calls) == 1
        assert len(adapter.commands) == 2
    finally:
        service.close()
