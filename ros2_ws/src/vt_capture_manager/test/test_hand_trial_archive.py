"""Full hand originals survive preview bounds, transactions and interrupted I/O."""
from contextlib import closing
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3

import pytest

from vt_capture_manager.metadata import bounded_document
from vt_capture_manager.storage import IntentStore


TRIAL = "a" * 32


def canonical(document):
    return json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def artifact(document):
    return dict(document=document, sha256=hashlib.sha256(canonical(document)).hexdigest(),
                hash_semantics="canonical_json_sha256")


def evidence(phase="before_start"):
    hands = {}
    for hand in ("left", "right"):
        session = hand + "-observed-session"
        profile = dict(schema_version=3, hand=hand, session_id=session, profile_version=7,
            nodes={str(node): dict(slot=node, epoch=2,
                world_alignment_xyzw=[0., 0., 0., 1.], sensor_from_bone_xyzw=[0., 0., 0., 1.],
                capture_observations=[dict(sample_index=index + 1, epoch=2,
                    quat_xyzw=[.001 * (index % 10), .002 * node, 0., .999],
                    host_monotonic_ns=10_000_000_123 + index * 10_000_000,
                    host_realtime_ns=1_800_000_000_000_000_123 + index * 10_000_000)
                    for index in range(256)]) for node in range(1, 13)},
            diagnostics=dict(note="合成标定原件；无需真实硬件或模型文件"))
        documents = dict(config=dict(hand=hand, sample_rate_hz=104.,
                node_to_slot={str(node): node for node in range(1, 13)},
                input_filter=dict(enabled=True)),
            rest=dict(parents=[-1, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14],
                      rest_global_xyzw=[[0., 0., 0., 1.]] * 16, provenance=dict(hand=hand)),
            profile=profile)
        artifacts = {name: artifact(document) for name, document in documents.items()}
        hands[hand] = dict(status="verified_snapshot", hand=hand, session_id=session,
            service_instance_id=hand + "-service", source_mode="live", profile_version=7,
            config_sha256=artifacts["config"]["sha256"], rest_sha256=artifacts["rest"]["sha256"],
            profile_sha256=artifacts["profile"]["sha256"], artifacts=artifacts,
            source_indices={str(node): dict(sample_index=256 if phase == "before_start" else 512,
                filter_epoch=2, host_monotonic_ns=12_550_000_123,
                host_realtime_ns=1_800_000_002_550_000_123) for node in range(1, 13)},
            replay_scope=dict(vqf_state_captured=False, exact_raw_replay_from_trial_start=False))
    return dict(schema_version=1, observation_phase=phase, boundary_exact=False, hands=hands)


def insert_trial(db):
    db.execute("INSERT INTO batches VALUES(?,?,?)", ("batch", "双手合成验证", "2026-11-23"))
    db.execute("INSERT INTO trials(id,batch_id,label,state,duration_s,created_at,controller_instance_id,start_request_id) "
               "VALUES(?,?,?,?,?,?,?,?)", (TRIAL, "batch", "test", "RECORDING", 60.,
                                           "2026-11-23", "controller", "start"))


def freeze_start(store, document):
    with store.transaction() as db:
        insert_trial(db)
        store.insert_start_metadata(db, TRIAL, {"request_id": "start"}, hand_evidence=document)


def verify_references(root, summary, original):
    archive = summary["archive"]
    path = root / archive["relative_path"]
    assert not Path(archive["relative_path"]).is_absolute()
    content = path.read_bytes()
    assert hashlib.sha256(content).hexdigest() == archive["sha256"]
    assert json.loads(content) == original
    assert archive["hash_semantics"] == "file_bytes_sha256"
    assert archive["complete_originals_retained"] is True
    for hand, detail in summary["hands"].items():
        for name, reference in detail["artifacts"].items():
            assert "document" not in reference
            relative = Path(reference["relative_path"])
            assert not relative.is_absolute() and ".." not in relative.parts
            stored = json.loads((root / relative).read_bytes())
            assert stored == original["hands"][hand]["artifacts"][name]["document"]
            assert reference["hash_semantics"] == "canonical_json_sha256"
            assert hashlib.sha256(canonical(stored)).hexdigest() == reference["sha256"]


def test_full_two_hand_originals_survive_preview_bounds_restart_and_relocation(tmp_path):
    before, after = evidence(), evidence("after_stop")
    assert len(canonical(before)) > 128 * 1024
    assert all(len(canonical(row["artifacts"]["profile"]["document"])) < 1024 * 1024
               for row in before["hands"].values())
    assert bounded_document(before)["preview_truncated"] is True
    original = deepcopy(before)
    directory = tmp_path / "state"
    with closing(IntentStore(directory)) as store:
        freeze_start(store, before)
        assert before == original  # Removing preview documents must not mutate the observed evidence.
        with store.transaction() as db:
            assert store.freeze_finish(db, TRIAL, {"request_id": "stop"}, quality_enabled=True,
                                       hand_evidence=after)
        assert not tuple(store.manifests.iterdir())  # Simulate exit after DB commit, before artifact I/O.
    with closing(IntentStore(directory)) as store:
        store.write_manifest(TRIAL)
        metadata = store.details(TRIAL)["metadata"]
        for key, source in (("start", before), ("finish", after)):
            summary = metadata[key]["hand_runtime_evidence"]
            assert "preview_truncated" not in summary
            assert len(canonical(summary)) < 96 * 1024
            verify_references(store.manifests, summary, source)
            assert json.loads((store.manifests / (TRIAL + f".{key}.json")).read_bytes()) == metadata[key]
        assert store.quality_job(TRIAL)["state"] == "QUEUED"
        with store.transaction() as db:
            assert not store.freeze_finish(db, TRIAL, {"request_id": "another-stop"}, quality_enabled=True,
                                           hand_evidence=after)
        assert store.details(TRIAL)["metadata"] == metadata
        # Relative references remain usable when the complete trial archive is moved.
        relocated = tmp_path / "exported-trials"
        shutil.copytree(store.manifests, relocated)
        for key, source in (("start", before), ("finish", after)):
            summary = json.loads((relocated / (TRIAL + f".{key}.json")).read_bytes())["hand_runtime_evidence"]
            verify_references(relocated, summary, source)


@pytest.mark.parametrize("phase", ["before_start", "after_stop"])
def test_checksum_mismatch_rolls_back_metadata_evidence_and_trial_side_effects(tmp_path, phase):
    with closing(IntentStore(tmp_path / "state")) as store:
        if phase == "after_stop":
            freeze_start(store, evidence())
        document = evidence(phase)
        document["hands"]["right"]["artifacts"]["profile"]["document"]["profile_version"] = 999
        with pytest.raises(ValueError, match="canonical hash"):
            with store.transaction() as db:
                if phase == "before_start":
                    insert_trial(db)
                    store.insert_start_metadata(db, TRIAL, {}, hand_evidence=document)
                else:
                    db.execute("UPDATE trials SET state='STOPPED' WHERE id=?", (TRIAL,))
                    store.freeze_finish(db, TRIAL, {}, quality_enabled=True, hand_evidence=document)
        assert not tuple(store.manifests.iterdir())
        assert store._db.execute("SELECT COUNT(*) FROM hand_trial_evidence WHERE phase=?", (phase,)).fetchone()[0] == 0
        assert store.quality_job(TRIAL) is None
        if phase == "before_start":
            assert store.trial(TRIAL) is None
            assert store._db.execute("SELECT COUNT(*) FROM trial_metadata").fetchone()[0] == 0
        else:
            assert store.trial(TRIAL)["state"] == "RECORDING"
            assert store.details(TRIAL)["metadata"]["finish"] is None
            assert store.details(TRIAL)["metadata"]["start"]["hand_runtime_evidence"]["hands"]["right"]["profile_version"] == 7


@pytest.mark.parametrize("phase", ["before_start", "after_stop"])
def test_later_transaction_failure_cannot_leave_orphan_hand_evidence(tmp_path, phase):
    with closing(IntentStore(tmp_path / "state")) as store:
        if phase == "after_stop":
            freeze_start(store, evidence())
        with pytest.raises(RuntimeError, match="later transaction failure"):
            with store.transaction() as db:
                if phase == "before_start":
                    insert_trial(db)
                    store.insert_start_metadata(db, TRIAL, {}, hand_evidence=evidence(phase))
                else:
                    store.freeze_finish(db, TRIAL, {}, quality_enabled=True, hand_evidence=evidence(phase))
                raise RuntimeError("later transaction failure")
        assert store._db.execute("SELECT COUNT(*) FROM hand_trial_evidence WHERE phase=?", (phase,)).fetchone()[0] == 0
        assert store.quality_job(TRIAL) is None
        if phase == "before_start":
            assert store.details(TRIAL) is None
        else:
            assert store.details(TRIAL)["metadata"]["finish"] is None


def test_interrupted_archive_write_recovers_from_complete_database_without_publishing_half_manifest(tmp_path, monkeypatch):
    document = evidence()
    directory = tmp_path / "state"
    with closing(IntentStore(directory)) as store:
        freeze_start(store, document)
        write = store._write_immutable

        def interrupted(path, content):
            if path.name == "profile.json":
                raise OSError("simulated disk failure")
            write(path, content)

        monkeypatch.setattr(store, "_write_immutable", interrupted)
        with pytest.raises(OSError, match="simulated disk failure"):
            store.write_manifest(TRIAL)
        assert not (store.manifests / (TRIAL + ".json")).exists()
        assert not (store.manifests / (TRIAL + ".start.json")).exists()
        assert not list(store.manifests.rglob("*.tmp"))
        assert store.details(TRIAL)["metadata"]["start"] is not None
    with closing(IntentStore(directory)) as store:
        store.write_manifest(TRIAL)
        summary = store.details(TRIAL)["metadata"]["start"]["hand_runtime_evidence"]
        verify_references(store.manifests, summary, document)
        assert (store.manifests / (TRIAL + ".json")).is_file()
        original_bytes = {path.relative_to(store.manifests): path.read_bytes()
                          for path in store.manifests.rglob("*.json")}
        store.write_manifest(TRIAL)
        assert original_bytes == {path.relative_to(store.manifests): path.read_bytes()
                                  for path in store.manifests.rglob("*.json")}


def test_frozen_database_and_existing_artifact_cannot_be_silently_rewritten(tmp_path):
    with closing(IntentStore(tmp_path / "state")) as store:
        freeze_start(store, evidence())
        store.write_manifest(TRIAL)
        with pytest.raises(sqlite3.IntegrityError, match="immutable"):
            with store.transaction() as db:
                db.execute("UPDATE hand_trial_evidence SET document_json='{}' WHERE trial_id=?", (TRIAL,))
        reference = store.details(TRIAL)["metadata"]["start"]["hand_runtime_evidence"]["hands"]["left"]["artifacts"]["config"]
        path = store.manifests / reference["relative_path"]
        path.write_bytes(b"tampered local artifact\n")
        with pytest.raises(ValueError, match="differs from the database"):
            store.write_manifest(TRIAL)
        assert path.read_bytes() == b"tampered local artifact\n"
