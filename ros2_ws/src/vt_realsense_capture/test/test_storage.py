from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from vt_realsense_capture.recorder import required_topics
from vt_realsense_capture.storage import FileStorage


CAMERA_NAMES = ("d405_1", "d405_2", "d436")


def test_file_storage_creates_session_and_writes_qos(tmp_path: Path) -> None:
    storage = FileStorage()

    session = storage.create_session(tmp_path, "session-1")
    qos_path = session / "qos_overrides.yaml"
    topics = required_topics(CAMERA_NAMES)
    storage.write_qos(qos_path, topics)

    assert session == tmp_path / "session-1"
    assert session.is_dir()
    assert set(yaml.safe_load(qos_path.read_text())) == set(topics)


@pytest.mark.parametrize(
    "session_id",
    [
        pytest.param("", id="empty"),
        pytest.param("../escape", id="parent-traversal"),
        pytest.param("nested/session", id="separator"),
        pytest.param(".hidden", id="leading-period"),
        pytest.param(123, id="non-string"),
    ],
)
def test_file_storage_rejects_unsafe_or_invalid_session_id(
    tmp_path: Path, session_id: object
) -> None:
    with pytest.raises(ValueError, match="session id"):
        FileStorage().create_session(tmp_path, session_id)  # type: ignore[arg-type]


def test_file_storage_rejects_nonabsolute_root(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="absolute existing directory"):
        FileStorage().create_session(Path("relative-root"), "session-1")


def test_file_storage_rejects_symlink_root(tmp_path: Path) -> None:
    linked_root = tmp_path.parent / f"{tmp_path.name}-link"
    linked_root.symlink_to(tmp_path, target_is_directory=True)

    with pytest.raises(ValueError, match="absolute existing directory"):
        FileStorage().create_session(linked_root, "session-1")


def test_file_storage_refuses_existing_session_directory(tmp_path: Path) -> None:
    existing = tmp_path / "session-1"
    existing.mkdir()

    with pytest.raises(FileExistsError):
        FileStorage().create_session(tmp_path, "session-1")

    assert existing.is_dir()


def test_file_storage_propagates_session_directory_write_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def denied_mkdir(_path: object, _mode: int) -> None:
        raise PermissionError("write denied")

    monkeypatch.setattr(os, "mkdir", denied_mkdir)

    with pytest.raises(PermissionError, match="write denied"):
        FileStorage().create_session(tmp_path, "session-1")


def test_file_storage_propagates_directory_fsync_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def failed_fsync(_descriptor: int) -> None:
        raise OSError("directory fsync failed")

    monkeypatch.setattr(os, "fsync", failed_fsync)

    with pytest.raises(OSError, match="directory fsync failed"):
        FileStorage().create_session(tmp_path, "session-1")

    assert (tmp_path / "session-1").is_dir()
