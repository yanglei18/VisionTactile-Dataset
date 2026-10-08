"""Shared DTOs. Older schemas stay unchanged; v4 adds device process control."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

from .validation import (
    CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION, PROTOCOL_VERSION, validate_capabilities, validate_device, validate_event,
    validate_controller, validate_management, validate_message, validate_operation,
    validate_process_control, validate_process_group, validate_recording, validate_resources, validate_snapshot,
)


@dataclass(frozen=True)
class RecordingSnapshot:
    state: str = "UNAVAILABLE"
    session_id: str = ""
    elapsed_s: float = 0.0
    duration_s: float = 0.0
    detail: str = ""

    def to_dict(self) -> dict:
        value = asdict(self)
        validate_recording(value)
        return value


@dataclass(frozen=True)
class DeviceSnapshot:
    id: str
    label: str
    kind: str
    side: str = ""
    status: str = "unknown"
    topic: str = ""
    receive_hz: float = 0.0
    valid_hz: float | None = None
    age_s: float | None = None
    detail: str = ""
    preview: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        value = asdict(self)
        validate_device(value)
        return value


@dataclass(frozen=True)
class EventSnapshot:
    id: str
    level: str
    message: str

    def to_dict(self) -> dict:
        value = asdict(self)
        validate_event(value)
        return value


@dataclass(frozen=True)
class ResourceSnapshot:
    disk_free_bytes: int | None = None

    def to_dict(self) -> dict:
        value = asdict(self)
        validate_resources(value)
        return value


@dataclass(frozen=True)
class Capabilities:
    record: bool = False
    device_control: bool = False

    def to_dict(self) -> dict:
        value = asdict(self)
        validate_capabilities(value)
        return value


@dataclass(frozen=True)
class CaptureSnapshot:
    mode: str
    instance_id: str
    sequence: int = 0
    recording: RecordingSnapshot = field(default_factory=RecordingSnapshot)
    devices: list[DeviceSnapshot] = field(default_factory=list)
    events: list[EventSnapshot] = field(default_factory=list)
    resources: ResourceSnapshot = field(default_factory=ResourceSnapshot)
    capabilities: Capabilities = field(default_factory=Capabilities)

    def to_dict(self) -> dict:
        value = asdict(self)
        validate_snapshot(value)
        return value


@dataclass(frozen=True)
class Request:
    request_id: str
    command: str = "snapshot"
    version: int = PROTOCOL_VERSION

    def to_dict(self) -> dict:
        value = asdict(self)
        validate_message(value)
        return value


@dataclass(frozen=True)
class Response:
    request_id: str
    snapshot: CaptureSnapshot
    ok: bool = True
    error: str | None = None
    version: int = PROTOCOL_VERSION

    def to_dict(self) -> dict:
        value = asdict(self)
        if value["error"] is None:
            del value["error"]
        validate_message(value)
        return value


@dataclass(frozen=True)
class ControllerSnapshot:
    controller_instance_id: str
    state_revision: int = 0
    state: int = 0
    session_id: str = ""
    start_request_id: str = ""
    session_label: str = ""
    output_path: str = ""
    recorder_alive: bool = False
    elapsed_sec: float = 0.0
    planned_duration_sec: int = 0
    topics: list[str] = field(default_factory=list)
    max_duration_sec: int = 300
    detail: str = ""

    def to_dict(self):
        value = asdict(self)
        validate_controller(value)
        return value


@dataclass(frozen=True)
class OperationSnapshot:
    request_id: str
    state: str
    code: str
    detail: str = ""
    trial_id: str = ""

    def to_dict(self):
        value = asdict(self)
        validate_operation(value)
        return value


@dataclass(frozen=True)
class ManagementSnapshot:
    enabled: bool = False
    controller: ControllerSnapshot | None = None
    batches: list[dict] = field(default_factory=list)
    trials: list[dict] = field(default_factory=list)
    operation: OperationSnapshot | None = None

    def to_dict(self):
        value = asdict(self)
        validate_management(value)
        return value


@dataclass(frozen=True)
class ControlSnapshot(CaptureSnapshot):
    management: ManagementSnapshot = field(default_factory=ManagementSnapshot)

    def to_dict(self):
        value = asdict(self)
        validate_snapshot(value, version=CONTROL_PROTOCOL_VERSION)
        return value


@dataclass(frozen=True)
class ControlRequest:
    request_id: str
    command: str = "snapshot"
    params: dict = field(default_factory=dict)
    version: int = CONTROL_PROTOCOL_VERSION

    def to_dict(self):
        value = asdict(self)
        validate_message(value)
        return value


@dataclass(frozen=True)
class ControlResponse:
    request_id: str
    snapshot: ControlSnapshot
    ok: bool = True
    operation: OperationSnapshot | None = None
    error: str | None = None
    version: int = CONTROL_PROTOCOL_VERSION

    def to_dict(self):
        value = asdict(self)
        if value["error"] is None:
            del value["error"]
        validate_message(value)
        return value


@dataclass(frozen=True)
class DetailRequest(ControlRequest):
    version: int = DETAIL_PROTOCOL_VERSION


@dataclass(frozen=True)
class DetailResponse(ControlResponse):
    version: int = DETAIL_PROTOCOL_VERSION
    trial_details: dict | None = None

    def to_dict(self):
        value = super().to_dict()
        if value["trial_details"] is None:
            del value["trial_details"]
        return value


@dataclass(frozen=True)
class ProcessGroupSnapshot:
    id: str
    label: str
    kind: str
    state: str = "DISABLED"
    owned: bool = False
    pid: int | None = None
    detail: str = ""
    log_path: str = ""
    can_start: bool = False
    can_stop: bool = False
    can_restart: bool = False
    topics: list[str] = field(default_factory=list)

    def to_dict(self):
        value = asdict(self)
        validate_process_group(value)
        return value


@dataclass(frozen=True)
class ProcessControlSnapshot:
    enabled: bool = False
    groups: list[ProcessGroupSnapshot] = field(default_factory=list)

    def to_dict(self):
        value = asdict(self)
        validate_process_control(value)
        return value


@dataclass(frozen=True)
class DeviceControlSnapshot(ControlSnapshot):
    process_control: ProcessControlSnapshot = field(default_factory=ProcessControlSnapshot)

    def to_dict(self):
        value = asdict(self)
        validate_snapshot(value, version=DEVICE_PROTOCOL_VERSION)
        return value


@dataclass(frozen=True)
class DeviceControlRequest(DetailRequest):
    version: int = DEVICE_PROTOCOL_VERSION


@dataclass(frozen=True)
class DeviceControlResponse(DetailResponse):
    snapshot: DeviceControlSnapshot
    version: int = DEVICE_PROTOCOL_VERSION
