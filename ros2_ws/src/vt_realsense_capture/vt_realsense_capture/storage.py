"""Minimal filesystem boundary for capture sessions."""

from __future__ import annotations

import os
import re
from pathlib import Path

from .recorder import write_qos_overrides


class FileStorage:
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

    def write_qos(self, path: Path, topics: tuple[str, ...]) -> None:
        write_qos_overrides(path, topics)

    @staticmethod
    def _fsync_directory(directory: Path) -> None:
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        descriptor = os.open(directory, flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
