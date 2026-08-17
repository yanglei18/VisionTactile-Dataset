from dataclasses import dataclass
from threading import Lock

from vt_vive_tracker.visualization_model import FIXED_ROLES, RoleSnapshot


@dataclass(frozen=True)
class StoredSnapshot:
    version: int
    roles: tuple[RoleSnapshot, ...]


@dataclass(frozen=True)
class DiagnosticEvent:
    version: int
    text: str
    monotonic_ns: int


class LatestSnapshotStore:
    def __init__(self) -> None:
        self._lock = Lock()
        self._version = 0
        self._latest: StoredSnapshot | None = None
        self._diagnostic_version = 0
        self._latest_diagnostic: DiagnosticEvent | None = None

    def publish(
        self, roles: tuple[RoleSnapshot, ...]
    ) -> StoredSnapshot:
        if type(roles) is not tuple or tuple(item.role for item in roles) != FIXED_ROLES:
            raise ValueError("snapshots must use fixed role order")
        with self._lock:
            self._version += 1
            self._latest = StoredSnapshot(self._version, roles)
            return self._latest

    def latest(self) -> StoredSnapshot | None:
        with self._lock:
            return self._latest

    def publish_diagnostic(
        self, text: str, monotonic_ns: int
    ) -> DiagnosticEvent:
        if type(text) is not str or not text:
            raise ValueError("diagnostic text must not be empty")
        if type(monotonic_ns) is not int or monotonic_ns < 0:
            raise ValueError("diagnostic time must be non-negative")
        with self._lock:
            self._diagnostic_version += 1
            self._latest_diagnostic = DiagnosticEvent(
                self._diagnostic_version, text, monotonic_ns
            )
            return self._latest_diagnostic

    def latest_diagnostic(self) -> DiagnosticEvent | None:
        with self._lock:
            return self._latest_diagnostic
