"""Bounded calibration storage and TASK round trips, without hardware/models."""
from copy import deepcopy
import errno
import json
from pathlib import Path
import stat
from types import SimpleNamespace
from uuid import uuid4

import pytest

from vt_fsglove12 import calibration_artifacts as artifacts


def task(categories=(2,)):
    binding = ["software-test-session", 0, "a" * 64, "b" * 64, "c" * 64, "d" * 64, True]
    groups = {str(category): dict(binding=list(binding), rows=[dict(
        category=category, pose=[[0., 0., 0.] for _ in range(16)],
        timestamp_s=float(index), input_identity=[["12", 0, index + 1]],
        contact_confirmed=True) for index in range(2)]) for category in categories}
    return dict(schema_version=1, kind=artifacts.TASK_KIND, task_id=str(uuid4()),
        binding=binding, source_session_id=binding[0], groups=groups,
        upstream=dict(upstream_project="software-test-only", upstream_commit="test-commit",
                      numerical_ast_modified=False, sources={"utils": {"sha256": "e" * 64}}),
        mano_model=dict(hand="right", sha256="f" * 64, path="/test-only/model.pkl"),
        before_shape_params=[0.] * 10, contact_labels_confirmed=True,
        fixture_note="SYNTHETIC SOFTWARE TEST; NOT PHYSICAL CONTACT EVIDENCE")


def test_left_task_keeps_model_side_and_rejects_right_even_with_same_digest():
    document = task()
    document["mano_model"]["hand"] = "left"
    expected = {**document["upstream"], "mano_model": deepcopy(document["mano_model"])}
    assert artifacts.validate_task(document, expected)["mano_model"]["hand"] == "left"
    expected["mano_model"]["hand"] = "right"
    with pytest.raises(ValueError, match="different MANO model"):
        artifacts.validate_task(document, expected)


def test_write_is_exclusive_atomic_and_independent_of_recording_threshold(tmp_path, monkeypatch):
    monkeypatch.setattr(artifacts.shutil, "disk_usage", lambda path: SimpleNamespace(free=32 * 1024 * 1024))
    destination = tmp_path / "new" / "profile.json"
    link = artifacts.os.link
    publications = []
    def observe(source, target):
        assert not target.exists()
        publications.append(json.loads(source.read_text()))
        link(source, target)
    monkeypatch.setattr(artifacts.os, "link", observe)
    assert artifacts.write_artifact(destination, {"value": 1}) == str(destination.resolve())
    assert publications == [{"value": 1}]
    monkeypatch.setattr(artifacts.os, "link", link)
    with pytest.raises(FileExistsError, match="Calibration artifact"):
        artifacts.write_artifact(destination, {"value": 2})
    assert json.loads(destination.read_text()) == {"value": 1}
    assert list(destination.parent.iterdir()) == [destination]


def test_preflight_checks_actual_space_and_actual_disk_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(artifacts.shutil, "disk_usage", lambda path: SimpleNamespace(free=16 * 1024 * 1024))
    with pytest.raises(OSError, match="Insufficient free space") as caught:
        artifacts.preflight(tmp_path, estimated_bytes=1)
    assert caught.value.errno == errno.ENOSPC
    obstacle = tmp_path / "not-a-directory"
    obstacle.write_bytes(b"existing data")
    with pytest.raises(OSError, match="Calibration artifact storage unavailable"):
        artifacts.write_artifact(obstacle / "artifact.json", {"value": 1})
    assert obstacle.read_bytes() == b"existing data"


def test_artifact_freezes_document_before_disk_preflight(tmp_path, monkeypatch):
    document = {"value": 1}
    def mutate_while_checking(*args, **kwargs):
        document["value"] = float("nan")
        return str(tmp_path)
    monkeypatch.setattr(artifacts, "preflight", mutate_while_checking)
    path = artifacts.write_artifact(tmp_path / "artifact.json", document)
    assert json.loads(Path(path).read_text()) == {"value": 1}


@pytest.mark.parametrize("number", [float("nan"), float("inf"), object()])
def test_invalid_artifact_never_creates_destination(tmp_path, number):
    destination = tmp_path / "never-created" / "artifact.json"
    with pytest.raises(ValueError, match="finite JSON"):
        artifacts.write_artifact(destination, {"number": number})
    assert not destination.parent.exists()


def test_artifact_limit_and_failed_write_leave_no_loadable_partial(tmp_path, monkeypatch):
    monkeypatch.setattr(artifacts, "MAX_ARTIFACT_BYTES", 32)
    destination = tmp_path / "artifact.json"
    with pytest.raises(ValueError, match="size limit"):
        artifacts.write_artifact(destination, {"payload": "a" * 32})
    monkeypatch.setattr(artifacts, "MAX_ARTIFACT_BYTES", 16 * 1024 * 1024)
    monkeypatch.setattr(artifacts, "preflight", lambda *args, **kwargs: str(tmp_path))
    def full_disk(descriptor):
        raise OSError(errno.ENOSPC, "disk filled during write")
    monkeypatch.setattr(artifacts.os, "fsync", full_disk)
    with pytest.raises(OSError, match="disk filled during write"):
        artifacts.write_artifact(destination, {"value": 1})
    assert list(tmp_path.iterdir()) == []


def test_artifact_size_matches_written_unicode_indented_bytes(tmp_path, monkeypatch):
    document = {"nested": [[0, 1], [2, 3]], "label": "触指"}
    path = Path(artifacts.write_artifact(tmp_path / "artifact.json", document))
    assert artifacts.artifact_size(document) == len(path.read_bytes())
    assert artifacts.artifact_size(document) > len(json.dumps(document, ensure_ascii=False).encode("utf-8"))
    monkeypatch.setattr(artifacts, "MAX_ARTIFACT_BYTES", artifacts.artifact_size(document) - 1)
    with pytest.raises(ValueError, match="size limit"):
        artifacts.artifact_size(document)
    with pytest.raises(ValueError, match="size limit"):
        artifacts.write_artifact(tmp_path / "oversized.json", document)


def test_resume_recovers_published_file_after_directory_fsync_failure(tmp_path, monkeypatch):
    path, document = tmp_path / "artifact.json", {"value": 1}
    original_fsync = artifacts.os.fsync
    def fail_directory(descriptor):
        if stat.S_ISDIR(artifacts.os.fstat(descriptor).st_mode):
            raise OSError(errno.EIO, "injected directory fsync failure")
        original_fsync(descriptor)
    monkeypatch.setattr(artifacts.os, "fsync", fail_directory)
    with pytest.raises(OSError, match="directory fsync failure"):
        artifacts.write_artifact(path, document)
    assert json.loads(path.read_text()) == document
    original_bytes, original_stat = path.read_bytes(), path.stat()
    synced = []
    def observe_fsync(descriptor):
        synced.append("directory" if stat.S_ISDIR(artifacts.os.fstat(descriptor).st_mode) else "file")
        original_fsync(descriptor)
    monkeypatch.setattr(artifacts.os, "fsync", observe_fsync)
    assert artifacts.resume_artifact(path, document) == str(path)
    assert synced == ["file", "directory"]
    assert path.read_bytes() == original_bytes
    assert path.stat().st_ino == original_stat.st_ino
    assert path.stat().st_mtime_ns == original_stat.st_mtime_ns
    with pytest.raises(FileExistsError):
        artifacts.write_artifact(path, document)


def test_resume_creates_missing_artifact_and_accepts_matching_json_value(tmp_path):
    path = tmp_path / "new" / "artifact.json"
    document = {"b": 2, "a": {"x": True}}
    assert artifacts.resume_artifact(path, document) == str(path)
    equivalent = tmp_path / "equivalent.json"
    equivalent.write_text('{"a":{"x":true},"b":2}')
    before = equivalent.read_bytes()
    assert artifacts.resume_artifact(equivalent, document) == str(equivalent)
    assert equivalent.read_bytes() == before


@pytest.mark.parametrize("existing", ['{"value":2}', '{"value":true}', '{"value":NaN}', '{broken'])
def test_resume_rejects_different_or_invalid_existing_content(tmp_path, existing):
    path = tmp_path / "artifact.json"
    path.write_text(existing)
    with pytest.raises((OSError, ValueError)):
        artifacts.resume_artifact(path, {"value": 1})
    assert path.read_text() == existing


@pytest.mark.parametrize("kind", ["symlink", "dangling_symlink", "directory", "fifo"])
def test_resume_rejects_symlink_or_nonregular_destination_without_writes(tmp_path, kind):
    path = tmp_path / "artifact.json"
    target = tmp_path / "target.json"
    if kind == "symlink":
        artifacts.write_artifact(target, {"value": 1})
        path.symlink_to(target)
    elif kind == "dangling_symlink":
        path.symlink_to(target)
    elif kind == "directory":
        path.mkdir()
    else:
        artifacts.os.mkfifo(path)
    with pytest.raises(OSError):
        artifacts.resume_artifact(path, {"value": 1})
    if kind == "symlink":
        assert path.is_symlink() and json.loads(target.read_text()) == {"value": 1}
    elif kind == "dangling_symlink":
        assert path.is_symlink() and not target.exists()


def test_partial_task_roundtrip_preserves_all_evidence_without_rebinding(tmp_path):
    document = task((2, 4))
    destination = tmp_path / "touch-task.json"
    assert artifacts.save_task(destination, document) == str(destination)
    reloaded = artifacts.load_task(destination)
    assert reloaded == document
    assert set(reloaded["groups"]) == {"2", "4"}
    assert reloaded["source_session_id"] == "software-test-session"
    with pytest.raises(ValueError, match="all four"):
        artifacts.task_to_contacts(reloaded)
    reloaded["groups"]["2"]["rows"][0]["pose"][0][0] = 10.
    assert document["groups"]["2"]["rows"][0]["pose"][0][0] == 0.


def test_complete_reloaded_task_is_optimizer_compatible(tmp_path):
    document = task((2, 3, 4, 5))
    destination = Path(artifacts.save_task(tmp_path / "task.json", document))
    expected = {**document["upstream"], "mano_model": {**document["mano_model"], "path": "/moved/model.pkl"}}
    contacts = artifacts.task_to_contacts(artifacts.load_task(destination, expected), expected)
    assert contacts["finger_category"] == [2, 2, 3, 3, 4, 4, 5, 5]
    assert contacts["task_id"] == document["task_id"]
    assert contacts["binding"] == document["binding"]
    assert contacts["capture_evidence"][0] == {key: value for key, value in document["groups"]["2"]["rows"][0].items() if key != "pose"}
    assert contacts["contact_labels_confirmed"] is True


@pytest.mark.parametrize("mutation", [
    lambda t: t.update(schema_version=True),
    lambda t: t.update(kind="shape_profile"),
    lambda t: t.pop("task_id"),
    lambda t: t.update(task_id="invented"),
    lambda t: t.update(groups={}),
    lambda t: t["groups"].update({"6": deepcopy(t["groups"]["2"])}),
    lambda t: t.update(contact_labels_confirmed=False),
    lambda t: t.pop("contact_labels_confirmed"),
    lambda t: t.update(contact_labels_source="inferred_from_imu"),
    lambda t: t.update(before_shape_params=[0.] * 9),
    lambda t: t.update(before_shape_params={"unexpected": "object"}),
    lambda t: t.update(source_session_id="new-session"),
    lambda t: t["binding"].__setitem__(2, "not-a-digest"),
    lambda t: t["groups"]["2"]["binding"].__setitem__(1, 1),
    lambda t: t["groups"]["2"].update(rows=[]),
    lambda t: t["groups"]["2"].update(rows=t["groups"]["2"]["rows"] * 61),
    lambda t: t["groups"]["2"]["rows"][0].update(category=3),
    lambda t: t["groups"]["2"]["rows"][0].pop("contact_confirmed"),
    lambda t: t["groups"]["2"]["rows"][0].update(contact_confirmed=1),
    lambda t: t["groups"]["2"]["rows"][0].update(pose=[[0.] * 3] * 15),
    lambda t: t["groups"]["2"]["rows"][0].update(pose={"unexpected": "object"}),
    lambda t: t["groups"]["2"]["rows"][0].update(timestamp_s=float("nan")),
    lambda t: t["groups"]["2"]["rows"][1].update(timestamp_s=0.),
    lambda t: t["groups"]["2"]["rows"][0].update(input_identity=[]),
    lambda t: t["groups"]["2"]["rows"][0].update(input_identity=[["12", 0, 0]]),
    lambda t: t["groups"]["2"]["rows"][1].update(input_identity=[["12", 0, 1]]),
    lambda t: t["mano_model"].update(hand="both"),
    lambda t: t["upstream"].update(numerical_ast_modified=True),
    lambda t: t["upstream"]["sources"].update(utils=None),
])
def test_malformed_partial_tasks_rejected_without_fabricated_evidence(mutation):
    document = task()
    mutation(document)
    before = deepcopy(document)
    with pytest.raises(ValueError):
        artifacts.validate_task(document)
    assert document.keys() == before.keys()


@pytest.mark.parametrize("mismatch", ["model", "upstream"])
def test_task_provenance_mismatch_rejected(mismatch):
    document = task()
    expected = {**deepcopy(document["upstream"]), "mano_model": deepcopy(document["mano_model"])}
    if mismatch == "model":
        expected["mano_model"]["sha256"] = "0" * 64
    else:
        expected["sources"]["utils"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="different"):
        artifacts.validate_task(document, expected)


def test_task_loader_rejects_oversized_and_broken_json(tmp_path, monkeypatch):
    path = tmp_path / "task.json"
    path.write_bytes(b"{broken")
    with pytest.raises(ValueError, match="valid JSON"):
        artifacts.load_task(path)
    monkeypatch.setattr(artifacts, "MAX_ARTIFACT_BYTES", 3)
    with pytest.raises(ValueError, match="size limit"):
        artifacts.load_task(path)
