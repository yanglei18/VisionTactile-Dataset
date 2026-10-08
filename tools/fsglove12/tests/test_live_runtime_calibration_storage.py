"""Calibration artifacts survive the bulk-log latch; no device or ROS process."""
from copy import deepcopy
import json

import pytest

from vt_fsglove12 import calibration_artifacts, live_runtime
from vt_fsglove12.live_runtime import STEP_KEYS
from vt_fsglove12.recording_policy import SessionStorageBudget
from test_live_runtime_high_rate import setup_direct, feed
from test_live_runtime_recording_isolation import Exporter


def fitted_profile(controller, session):
    return dict(status="fitted", warnings=[], rejected_nodes={}, nodes={"12": {"epoch": 0}},
                session_id=session.session_id, config_sha256=controller._config_sha,
                rest_sha256=controller._rest_sha)


def latch_budget(controller, session):
    (session.directory / "raw.csv").write_text("historical bulk records")
    session.storage_budget = SessionStorageBudget(session.directory,
        {"max_session_bytes": 1, "min_free_bytes": 0})
    assert not controller._check_recording(session)
    assert session.recording_disabled.is_set()
    assert "max_session_bytes" in session.recording_errors[0]


def test_latched_bulk_quota_still_saves_basic_and_functional_profiles(tmp_path):
    controller, session, backend, clock = setup_direct(tmp_path)
    exporter = controller._pose_exporter = Exporter()
    latch_budget(controller, session)
    log_error = deepcopy(session.recording_errors)
    profile = fitted_profile(controller, session)
    backend.fit_calibration = lambda *args, **kwargs: deepcopy(profile)
    backend.fit_functional_finger = lambda *args, **kwargs: dict(slot_refs={"1": {"axis": [1, 0, 0]}})
    controller._captures = {step: [{"test": step}] for step in STEP_KEYS}

    controller._fit_profile(session)
    basic = json.loads((tmp_path / "profile.v001.json").read_text())
    assert basic["profile_version"] == 1
    assert json.loads((tmp_path / "profile.json").read_text()) == basic
    controller._fit_functional_profile(session, dict(rows=[], motion_rows=[], finger="index"))
    functional = json.loads((tmp_path / "profile.v002.json").read_text())
    assert functional["functional_fingers"]["index"]["slot_refs"]["1"]["axis"] == [1, 0, 0]
    assert session.profile_saved and session.profiles == ["profile.v001.json", "profile.v002.json"]
    assert controller._profile["profile_version"] == 2

    for _ in range(3):
        feed(session, clock, 12)
        assert controller._tick(session)
        clock.advance(1 / 120)
    snapshot = controller.latest_snapshot()
    assert snapshot["runtime"]["connected"] and snapshot["runtime"]["error"] is None
    assert len(exporter.documents) == len(backend.calls) == 3
    assert not exporter.errors and not session.stop.is_set()
    assert session.recording_disabled.is_set() and session.recording_errors == log_error
    assert all(not stream.getvalue() for stream in session.streams.values())
    assert snapshot["runtime"]["calibration_storage"]["status"] == "saved"
    assert snapshot["runtime"]["calibration_storage"]["pending_count"] == 0


def test_completed_mounting_captures_save_unique_bounded_evidence_after_quota(tmp_path):
    controller, session, _backend, clock = setup_direct(tmp_path)
    latch_budget(controller, session)
    for index in range(2):
        rows = [dict(timestamp_s=clock(), nodes={"12": dict(sample_index=index + 1, epoch=0)})]
        controller._active = dict(step="neutral", collect_at=clock() - 2, end_at=clock(), rows=rows)
        controller._capture(session, dict(nodes={}), clock())
    files = list((tmp_path / "calibration").glob("capture.neutral.*.json"))
    assert len(files) == 2
    captures = [json.loads(path.read_text()) for path in files]
    assert len({row["capture_id"] for row in captures}) == 2
    assert {row["frames"][0]["nodes"]["12"]["sample_index"] for row in captures} == {1, 2}
    assert all(row["session_id"] == session.session_id and row["step"] == "neutral" for row in captures)
    assert controller._captures["neutral"] == rows
    assert session.recording_disabled.is_set() and not session.stop.is_set()


@pytest.mark.parametrize("latched", [False, True])
def test_failed_profile_save_retries_original_documents_without_changing_live_or_log_state(
        tmp_path, monkeypatch, latched):
    controller, session, backend, clock = setup_direct(tmp_path)
    exporter = controller._pose_exporter = Exporter()
    if latched:
        latch_budget(controller, session)
    original_writer = live_runtime.write_artifact
    attempts = []

    def fail(path, document):
        attempts.append(path)
        raise OSError("disk full")

    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    profile = fitted_profile(controller, session)
    controller._save_profile(session, profile, event="calibration_fitted")
    controller._save_profile(session, {**profile, "functional_enabled": True}, event="functional_updated")
    assert not session.profile_saved
    assert len(attempts) == 3
    controller._profile["warnings"].append("later mutable warning")
    for _ in range(2):
        feed(session, clock, 12)
        controller._tick(session)
        clock.advance(1 / 120)
    state = controller.latest_snapshot()
    assert len(attempts) == 3  # No repeated per-tick disk retry.
    assert state["runtime"]["connected"] and state["runtime"]["error"] is None
    assert state["runtime"]["calibration_storage"]["pending_count"] == 3
    assert state["runtime"]["calibration_storage"]["retry_available"]
    assert len(exporter.documents) == len(backend.calls) == 2
    assert not session.stop.is_set() and session.recording_disabled.is_set() is latched

    monkeypatch.setattr(live_runtime, "write_artifact", original_writer)
    alternate = tmp_path / "alternate"
    assert controller._retry_calibration_saves(session, alternate)
    assert session.profile_saved and session.profile_version == 2
    assert json.loads((alternate / "profile.v001.json").read_text())["warnings"] == []
    assert json.loads((alternate / "profile.v002.json").read_text())["functional_enabled"]
    assert session.recording_disabled.is_set() is latched
    status = controller.latest_snapshot()["runtime"]["calibration_storage"]
    assert status["status"] == "saved" and status["pending_count"] == 0
    assert status["warning"] == "" and not status["retry_available"]


def test_existing_profile_is_never_overwritten_and_alias_does_not_claim_version_saved(tmp_path):
    controller, session, _backend, _clock = setup_direct(tmp_path)
    path = tmp_path / "profile.v001.json"
    path.write_text('{"old":true}')
    controller._save_profile(session, fitted_profile(controller, session), event="calibration_fitted")
    assert not session.profile_saved and session.profiles == []
    assert (tmp_path / "profile.json").exists()
    assert not controller._retry_calibration_saves(session)
    assert path.read_text() == '{"old":true}'
    assert not session.profile_saved and not session.recording_disabled.is_set()


def test_pending_calibration_documents_have_count_and_byte_limits(tmp_path, monkeypatch):
    controller, session, _backend, _clock = setup_direct(tmp_path)
    monkeypatch.setattr(live_runtime, "CALIBRATION_PENDING_LIMIT", 2)
    monkeypatch.setattr(live_runtime, "CALIBRATION_PENDING_BYTES", 128)

    def fail(*args):
        raise OSError("disk unavailable")

    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    for index in range(8):
        controller._persist_calibration_artifact(session, tmp_path / f"capture.{index}.json",
                                                 {"index": index}, kind="mounting_capture")
    status = controller._calibration_storage_status(session)
    assert status["pending_count"] == 2 and status["pending_bytes"] <= 128
    assert status["evicted_count"] == 6 and "缓存已达到上限" in status["warning"]
    assert [entry["document"]["index"] for entry in session.calibration_pending] == [6, 7]
    assert not session.recording_disabled.is_set()


def test_pending_document_bytes_match_indented_utf8_file_including_newline(tmp_path, monkeypatch):
    controller, session, _backend, _clock = setup_direct(tmp_path)
    document = {"组": [{"rows": [1, 2, 3]}]}
    size = len((json.dumps(document, ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode("utf-8"))

    def fail(*args):
        raise OSError("disk unavailable")

    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    assert not controller._persist_calibration_artifact(session, tmp_path / "capture.json",
                                                        document, kind="mounting_capture")
    assert controller._calibration_storage_status(session)["pending_bytes"] == size
    assert session.calibration_pending[0]["bytes"] == size


def test_pretty_print_size_limit_does_not_queue_permanently_unwritable_documents(tmp_path, monkeypatch):
    controller, session, _backend, _clock = setup_direct(tmp_path)
    document = {"rows": [{"nested": [1, 2, 3]}] * 3}
    compact_size = len(json.dumps(document, ensure_ascii=False).encode("utf-8"))
    monkeypatch.setattr(calibration_artifacts, "MAX_ARTIFACT_BYTES", compact_size + 1)
    assert not controller._persist_calibration_artifact(session, tmp_path / "capture.json",
                                                        document, kind="mounting_capture")
    status = controller._calibration_storage_status(session)
    assert status["status"] == "save_failed" and "size limit" in status["error"]
    assert status["pending_count"] == status["pending_bytes"] == 0
    assert not status["retry_available"] and not (tmp_path / "capture.json").exists()
    assert not session.recording_disabled.is_set()


def test_explicit_alternate_directory_applies_to_future_profiles_and_nested_captures(tmp_path, monkeypatch):
    controller, session, _backend, _clock = setup_direct(tmp_path)
    profile = fitted_profile(controller, session)
    alternate = tmp_path / "alternate"
    assert controller._retry_calibration_saves(session, alternate)
    assert controller._calibration_storage_status(session)["destination_dir"] == str(alternate)
    controller._save_profile(session, profile, event="calibration_fitted")
    assert (alternate / "profile.v001.json").exists() and (alternate / "profile.json").exists()
    assert not (tmp_path / "profile.v001.json").exists()
    document = {"capture": 1}
    logical_path = tmp_path / "calibration" / "capture.neutral.test.json"

    def fail(*args):
        raise OSError("disk temporarily unavailable")

    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    assert not controller._persist_calibration_artifact(session, logical_path,
                                                        document, kind="mounting_capture")
    assert controller._retry_calibration_saves(session)
    saved_path = alternate / "calibration" / logical_path.name
    assert json.loads(saved_path.read_text()) == document
    assert not logical_path.exists()
    assert session.calibration_directory == alternate
    assert not session.recording_disabled.is_set()


def test_failed_directory_fsync_retry_resumes_same_document_without_overwriting(tmp_path, monkeypatch):
    import stat

    controller, session, _backend, _clock = setup_direct(tmp_path)
    original_fsync = calibration_artifacts.os.fsync
    directory_sync_failed = False

    def fail_directory_once(descriptor):
        nonlocal directory_sync_failed
        if stat.S_ISDIR(calibration_artifacts.os.fstat(descriptor).st_mode) and not directory_sync_failed:
            directory_sync_failed = True
            raise OSError("directory fsync interrupted")
        original_fsync(descriptor)

    monkeypatch.setattr(calibration_artifacts.os, "fsync", fail_directory_once)
    controller._save_profile(session, fitted_profile(controller, session), event="calibration_fitted")
    path = tmp_path / "profile.v001.json"
    assert path.exists() and directory_sync_failed
    before = path.read_bytes()
    assert not session.profile_saved and session.profiles == []
    assert controller._calibration_storage_status(session)["pending_count"] == 1
    assert controller._retry_calibration_saves(session)
    assert path.read_bytes() == before
    assert session.profile_saved and session.profiles == [path.name]
    assert controller._calibration_storage_status(session)["status"] == "saved"
    assert not session.recording_disabled.is_set()


@pytest.mark.parametrize("alternate", [False, True])
def test_disconnect_retains_bounded_artifact_owner_and_retries_real_files(tmp_path, monkeypatch, alternate):
    controller, session, _backend, _clock = setup_direct(tmp_path)
    original_resume = live_runtime.resume_artifact

    def fail(*args):
        raise OSError("injected unavailable storage")

    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    controller._save_profile(session, fitted_profile(controller, session), event="calibration_fitted")
    controller._disconnect()
    owner = controller._calibration_retry_session()
    assert controller._session is None and owner is not session
    assert owner.session_id == session.session_id
    assert not hasattr(owner, "fusion") and not hasattr(owner, "reader") and not hasattr(owner, "source")
    assert not session.calibration_pending  # Evidence transferred, not duplicated.
    status = controller.latest_snapshot()["runtime"]["calibration_storage"]
    assert status["detached"] and status["pending_count"] == 2 and status["retry_available"]
    target = tmp_path / "recovered" if alternate else None
    monkeypatch.setattr(live_runtime, "resume_artifact", fail)
    assert not controller._retry_calibration_saves(owner, target)
    assert controller.latest_snapshot()["runtime"]["calibration_storage"]["pending_count"] == 2
    monkeypatch.setattr(live_runtime, "resume_artifact", original_resume)
    assert controller._retry_calibration_saves(owner)
    destination = target or tmp_path
    assert json.loads((destination / "profile.v001.json").read_text())["session_id"] == session.session_id
    assert (destination / "profile.json").exists()
    state = controller.latest_snapshot()
    assert not state["runtime"]["connected"]
    assert state["runtime"]["calibration_storage"]["status"] == "saved"
    assert state["runtime"]["calibration_storage"]["pending_count"] == 0
    assert not state["runtime"]["calibration_storage"]["retry_available"]
    assert state["runtime"]["local_recording"]["profile_saved"]


def test_new_session_keeps_detached_pending_owner_and_prioritizes_current_failures(tmp_path, monkeypatch):
    controller, first, _backend, _clock = setup_direct(tmp_path)

    def fail(*args):
        raise OSError("injected unavailable storage")

    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    controller._save_profile(first, fitted_profile(controller, first), event="calibration_fitted")
    controller._disconnect()
    detached = controller._calibration_retry_session()
    current_directory = tmp_path / "second_session"
    current_directory.mkdir()
    _unused, current, _backend, _clock = setup_direct(current_directory)
    current.session_id = "SECOND"
    controller._session = current
    controller._publish_calibration_storage(current)
    assert controller._calibration_retry_session() is detached
    assert controller.latest_snapshot()["runtime"]["calibration_storage"]["source_session_id"] == first.session_id
    controller._save_profile(current, fitted_profile(controller, current), event="calibration_fitted")
    assert controller._calibration_retry_session() is current
    assert not controller._retry_calibration_saves(current)  # Previous session still needs its own retry.
    assert (current_directory / "profile.v001.json").exists()
    assert controller._calibration_retry_session() is detached
    target = tmp_path / "recovery"
    assert controller._retry_calibration_saves(detached, target)
    assert (target / "profile.v001.json").exists()
    assert controller._calibration_retry_session() is current
    assert current.calibration_directory == target
    assert controller.latest_snapshot()["runtime"]["calibration_storage"]["pending_count"] == 0


def test_replacing_detached_owner_reports_evicted_pending_artifacts(tmp_path, monkeypatch):
    controller, first, _backend, _clock = setup_direct(tmp_path)

    def fail(*args):
        raise OSError("injected unavailable storage")

    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    controller._save_profile(first, fitted_profile(controller, first), event="calibration_fitted")
    controller._disconnect()
    old_owner = controller._calibration_retry_session()
    second_directory = tmp_path / "second_session"
    second_directory.mkdir()
    _unused, second, _backend, _clock = setup_direct(second_directory)
    second.session_id = "SECOND"
    controller._session = second
    controller._save_profile(second, fitted_profile(controller, second), event="calibration_fitted")
    controller._disconnect()
    owner = controller._calibration_retry_session()
    assert owner is not old_owner and owner.session_id == "SECOND"
    status = controller.latest_snapshot()["runtime"]["calibration_storage"]
    assert status["pending_count"] == 2 and status["evicted_count"] == 2
    assert "2 份旧文件" in status["warning"]
