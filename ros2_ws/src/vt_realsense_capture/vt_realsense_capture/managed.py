"""ROS-independent typed contracts for guarded Recorder commands."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CaptureContextFact:
    controller_instance_id: str
    state_revision: int
    state: int
    session_id: str
    start_request_id: str
    session_label: str
    output_path: str
    recorder_alive: bool
    elapsed_sec: float
    planned_duration_sec: int
    topics: tuple[str, ...]
    max_duration_sec: int
    detail: str


@dataclass(frozen=True)
class ManagedCommandFact:
    request_id: str
    command: int
    expected_controller_instance_id: str
    expected_session_id: str
    expected_state_revision: int
    session_label: str = ""
    planned_duration_sec: int = 0

    def validate(self) -> None:
        for name in ("request_id", "expected_controller_instance_id", "expected_session_id", "session_label"):
            value = getattr(self, name)
            if type(value) is not str or len(value) > 4096:
                raise ValueError(f"{name} must be a bounded string")
        if not self.request_id.strip() or not self.expected_controller_instance_id.strip():
            raise ValueError("request_id and expected_controller_instance_id must not be empty")
        if type(self.command) is not int or self.command not in (1, 2):
            raise ValueError("command must be START=1 or STOP=2")
        if type(self.expected_state_revision) is not int or not 0 <= self.expected_state_revision < 2**64:
            raise ValueError("expected_state_revision must be uint64")
        if type(self.planned_duration_sec) is not int or not 0 <= self.planned_duration_sec < 2**32:
            raise ValueError("planned_duration_sec must be uint32")


@dataclass(frozen=True)
class ManagedCommandReceipt:
    accepted: bool
    code: str
    detail: str
    context: CaptureContextFact
