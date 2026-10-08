"""Minimal filesystem boundary for capture sessions."""

from __future__ import annotations

import os
import json
import re
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path

from .recorder import write_qos_overrides
from .recording_qos import RecorderQosOverride


class FileStorage:
    def write_controller_session(self, session_dir: Path, document: dict) -> None:
        """Fsync a same-directory temporary file, atomically replace, fsync parent.

        This is lifecycle bookkeeping, not a promise that a bag survives power loss.
        Existing Recorder/QoS storage behavior is intentionally unchanged.
        """
        directory = Path(session_dir)
        if not directory.is_dir() or directory.is_symlink():
            raise ValueError("session journal directory must be a real directory")
        payload = json.dumps(document, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2) + "\n"
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory,
                                             prefix=".controller_session-", delete=False) as handle:
                temporary = Path(handle.name)
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, directory / "controller_session.json")
            self._fsync_directory(directory)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def unresolved_controller_sessions(self, output_root: Path) -> list[dict]:
        """Find unfinished prior intents without taking ownership of their processes."""
        root = Path(output_root)
        if not root.is_absolute() or not root.is_dir() or root.is_symlink():
            return []  # Existing create_session validation still rejects invalid roots.
        unresolved = []
        for path in sorted(root.glob("*/controller_session.json")):
            if path.is_symlink() or path.parent.is_symlink() or path.stat().st_size > 2 * 1024 * 1024:
                raise ValueError(f"unsafe controller journal: {path}")
            document = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(document, dict) or document.get("version") != 1:
                raise ValueError(f"invalid controller journal: {path}")
            if document.get("lifecycle") not in {"complete", "start_failed"}:
                for name in ("session_id", "start_request_id", "session_label", "output_path"):
                    if type(document.get(name)) is not str:
                        raise ValueError(f"invalid controller journal {name}: {path}")
                duration = document.get("planned_duration_sec")
                if type(duration) is not int or not 1 <= duration <= 300:
                    raise ValueError(f"invalid controller journal duration: {path}")
                unresolved.append(document)
        return unresolved

    def create_session(self, output_root: Path, session_id: str) -> Path:
        if (
            type(session_id) is not str
            or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", session_id) is None
        ):
            raise ValueError("session id is not a safe directory name")
        root = Path(output_root)
        if not root.is_absolute() or not root.is_dir() or root.is_symlink():
            raise ValueError("output root must be an absolute existing directory")
        session = root / session_id
        session.mkdir(mode=0o750, parents=False, exist_ok=False)
        self._fsync_directory(root)
        return session

    def write_qos(
        self,
        path: Path,
        topics: tuple[str, ...],
        *,
        topic_types: Mapping[str, str] | None = None,
        overrides: Sequence[RecorderQosOverride] = (),
    ) -> None:
        write_qos_overrides(
            path, topics, topic_types=topic_types, overrides=overrides
        )

    @staticmethod
    def _fsync_directory(directory: Path) -> None:
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        descriptor = os.open(directory, flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
