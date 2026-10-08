"""Pure Recorder-only capture lifecycle orchestration."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from enum import Enum, IntEnum
from pathlib import Path
from typing import Callable, Protocol
from uuid import uuid4

from .config import CaptureConfig
from .managed import CaptureContextFact, ManagedCommandFact, ManagedCommandReceipt
from .recorder import required_topics
from .recording_qos import RecorderQosOverride
from .session import (
    CaptureState,
    CaptureStateMachine,
    InvalidTransitionError,
    RequestConflictError,
    RequestValidationError,
)


NANOSECONDS_PER_SECOND = 1_000_000_000


class CommandKind(IntEnum):
    START = 1
    STOP = 2


class EventSeverity(IntEnum):
    INFO = 0
    WARNING = 1
    FATAL = 2


class RecorderHealth(Enum):
    ALIVE = "alive"
    EXITED = "exited"
    UNCERTAIN = "uncertain"


@dataclass(frozen=True)
class RecorderStopFact:
    returncode: int | None
    detail: str
    termination_confirmed: bool
    possibly_alive: bool


@dataclass(frozen=True)
class RecorderStartSpec:
    session_id: str
    session_dir: Path
    output_path: Path
    qos_overrides_path: Path
    topics: tuple[str, ...]


class Recorder(Protocol):
    def start(self, spec: RecorderStartSpec) -> None: ...

    def health(self) -> RecorderHealth: ...

    def stop(self) -> RecorderStopFact: ...


class Clock(Protocol):
    def monotonic_ns(self) -> int: ...


class Storage(Protocol):
    def create_session(self, output_root: Path, session_id: str) -> Path: ...

    def write_qos(
        self,
        path: Path,
        topics: tuple[str, ...],
        *,
        topic_types: Mapping[str, str] | None = None,
        overrides: Sequence[RecorderQosOverride] = (),
    ) -> None: ...


@dataclass(frozen=True)
class CaptureCommandFact:
    request_id: str
    command: CommandKind
    session_label: str = ""
    planned_duration_sec: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.request_id, str) or not self.request_id.strip():
            raise RequestValidationError("request_id must be a nonempty string")
        if not isinstance(self.command, CommandKind):
            raise RequestValidationError("command must be START or STOP")
        if not isinstance(self.session_label, str):
            raise RequestValidationError("session_label must be a string")
        if (
            type(self.planned_duration_sec) is not int
            or self.planned_duration_sec < 0
            or self.planned_duration_sec > (1 << 32) - 1
        ):
            raise RequestValidationError(
                "planned_duration_sec must be a uint32 integer"
            )


@dataclass(frozen=True)
class RawCaptureCommandFact:
    request_id: str
    command: object
    session_label: object
    planned_duration_sec: object


def submit_raw_capture_command(
    message: object,
    submitter: Callable[[RawCaptureCommandFact], object],
) -> object:
    """Validate the routable request ID before submitting untouched fields."""

    request_id = getattr(message, "request_id", None)
    if not isinstance(request_id, str) or not request_id.strip():
        raise RequestValidationError("request_id must be a nonempty string")
    raw = RawCaptureCommandFact(
        request_id=request_id,
        command=getattr(message, "command"),
        session_label=getattr(message, "session_label"),
        planned_duration_sec=getattr(message, "planned_duration_sec"),
    )
    return submitter(raw)


@dataclass(frozen=True)
class CommandReceipt:
    request_id: str
    command: CommandKind
    code: str
    state: CaptureState


@dataclass(frozen=True)
class CaptureStatusFact:
    request_id: str
    session_id: str
    state: CaptureState
    streams: tuple[object, ...]
    camera_groups: tuple[object, ...]
    recorder_alive: bool
    disk_free_bytes: int
    observed_write_mb_s: float
    detail: str


@dataclass(frozen=True)
class CaptureEventFact:
    request_id: str
    session_id: str
    severity: EventSeverity
    code: str
    camera_name: str = ""
    stream_name: str = ""
    detail: str = ""


@dataclass(frozen=True)
class SessionInfoFact:
    session_id: str
    request_id: str
    session_label: str


class CaptureController:
    """Coordinate one capture using only a Recorder process lifecycle."""

    def __init__(
        self,
        *,
        config: CaptureConfig,
        output_root: Path | str,
        clock: Clock,
        storage: Storage,
        recorder_factory: Callable[[], Recorder],
        event_sink: Callable[[CaptureEventFact], None],
        status_sink: Callable[[CaptureStatusFact], None],
        session_sink: Callable[[SessionInfoFact], None],
        session_id_factory: Callable[[], str],
        request_cache_size: int = 256,
    ) -> None:
        if type(request_cache_size) is not int or request_cache_size <= 0:
            raise ValueError("request_cache_size must be a positive integer")
        self._config = config
        self._output_root = Path(output_root)
        self._clock = clock
        self._storage = storage
        self._recorder_factory = recorder_factory
        self._event_sink = event_sink
        self._status_sink = status_sink
        self._session_sink = session_sink
        self._session_id_factory = session_id_factory
        self._request_cache_size = request_cache_size
        self._machine = CaptureStateMachine(request_cache_size=request_cache_size)
        self._topics = required_topics(
            tuple(camera.name for camera in config.cameras),
            tuple(stream.topic for stream in config.additional_streams),
        )
        self._commands: OrderedDict[
            str, tuple[CommandKind, CommandReceipt]
        ] = OrderedDict()
        self._command_parameters: dict[str, CaptureCommandFact] = {}
        self._recorder: Recorder | None = None
        self._recording_started_ns: int | None = None
        self._planned_deadline_ns: int | None = None
        self._recorder_termination_confirmed = False
        self._instance_id = uuid4().hex
        self._state_revision = 0
        self._context_signature = (CaptureState.IDLE, "")
        self._context_session_id = ""
        self._start_request_id = ""
        self._session_label = ""
        self._session_dir: Path | None = None
        self._output_path = ""
        self._planned_duration_sec = 0
        self._elapsed_final_sec: float | None = None
        self._context_detail = "Recorder idle"
        self._journal_error = ""
        self._lifecycle = "idle"
        self._start_parameters: dict = {}
        self._managed_requests: dict[str, tuple[ManagedCommandFact, ManagedCommandReceipt]] = {}
        self._session_managed_ids: list[str] = []
        self._recovery: dict | None = None
        recovery_reader = getattr(self._storage, "unresolved_controller_sessions", None)
        if recovery_reader is not None:
            try:
                unresolved = recovery_reader(self._output_root)
                if unresolved:
                    self._recovery = unresolved[-1]
                    self._context_detail = (
                        f"recovery_required: {len(unresolved)} unfinished controller session(s); "
                        "Recorder ownership/termination is unknown. No process was adopted or stopped."
                    )
            except Exception as exc:
                self._recovery = {}
                self._context_detail = f"recovery_required: cannot inspect controller journals: {exc}"

    @property
    def state(self) -> CaptureState:
        return CaptureState.INVALID if self._recovery is not None else self._machine.state

    @property
    def session_id(self) -> str:
        return str(self._recovery.get("session_id", "")) if self._recovery is not None else self._machine.session_id

    def capture_context(self) -> CaptureContextFact:
        """Read on the same workflow worker as commands; polls do not revise state."""
        self._sync_revision()
        if self._recovery is not None:
            old = self._recovery
            return CaptureContextFact(
                self._instance_id, self._state_revision, int(CaptureState.INVALID),
                str(old.get("session_id", "")), str(old.get("start_request_id", "")),
                str(old.get("session_label", "")), str(old.get("output_path", "")),
                True, 0.0, int(old.get("planned_duration_sec", 0)), self._topics,
                self._config.max_bag_duration_seconds, self._context_detail,
            )
        elapsed = self._elapsed_seconds()
        detail = self._context_detail
        if self._journal_error:
            detail += f"; session_journal_failed: {self._journal_error}"
        return CaptureContextFact(
            self._instance_id, self._state_revision, int(self.state),
            self._context_session_id, self._start_request_id, self._session_label,
            self._output_path, self._recorder_alive(), elapsed,
            self._planned_duration_sec, self._topics,
            self._config.max_bag_duration_seconds, detail,
        )

    def accept_managed(self, command: ManagedCommandFact) -> ManagedCommandReceipt:
        """Guarded START/STOP with immutable receipts and full-payload idempotency."""
        if not isinstance(command, ManagedCommandFact):
            raise RequestValidationError("command must be ManagedCommandFact")
        routable_id = type(command.request_id) is str and bool(command.request_id.strip()) and len(command.request_id) <= 4096
        cached = self._managed_requests.get(command.request_id) if routable_id else None
        if cached is not None:
            if cached[0] != command:
                return ManagedCommandReceipt(False, "request_id_conflict",
                    "request_id was already used with different parameters", self.capture_context())
            return cached[1]
        try:
            command.validate()
        except ValueError as exc:
            receipt = ManagedCommandReceipt(False, "invalid_request", str(exc), self.capture_context())
            if routable_id:
                self._managed_requests[command.request_id] = (command, receipt)
            return receipt

        def reject(code, detail):
            receipt = ManagedCommandReceipt(False, code, detail, self.capture_context())
            self._managed_requests[command.request_id] = (command, receipt)
            return receipt

        context = self.capture_context()
        if command.expected_controller_instance_id != context.controller_instance_id:
            return reject("controller_instance_mismatch", "Controller restarted or the command targets a different instance")
        if self._recovery is not None:
            return reject("recovery_required", self._context_detail)
        if command.request_id in self._commands:
            return reject("request_id_conflict", "request_id was already used through the legacy command interface")
        if command.expected_state_revision != context.state_revision:
            return reject("state_revision_mismatch", "Capture context changed; obtain a fresh snapshot and a new request ID")
        if command.command == int(CommandKind.START):
            if self.state not in {CaptureState.IDLE, CaptureState.COMPLETE, CaptureState.INVALID} or context.recorder_alive:
                return reject("active_session", "An active or uncertain Recorder session already exists")
            if command.expected_session_id not in {"", context.session_id}:
                return reject("session_mismatch", "START context refers to another session")
            if not 1 <= command.planned_duration_sec <= self._config.max_bag_duration_seconds:
                return reject("invalid_duration", "Managed START duration must be between 1 and the configured maximum (at most 300 seconds)")
            if not callable(getattr(self._storage, "write_controller_session", None)):
                return reject("session_journal_unavailable", "Managed START requires durable session bookkeeping")
        else:
            if not command.expected_session_id or command.expected_session_id != context.session_id:
                return reject("session_mismatch", "STOP must target the current nonempty session ID")
            if command.session_label or command.planned_duration_sec:
                return reject("invalid_request", "STOP must use an empty label and duration 0")
            if self.state not in {CaptureState.RECORDING, CaptureState.FINALIZING}:
                return reject("no_active_session", "The targeted session is not active")
        try:
            if command.command == int(CommandKind.STOP) and self.state is CaptureState.FINALIZING:
                # A second explicit stop never targets a different Recorder. The
                # existing lifecycle retry remains responsible for termination.
                code = "stop_accepted"
            else:
                legacy = self.accept(CaptureCommandFact(command.request_id, CommandKind(command.command),
                                                       command.session_label, command.planned_duration_sec))
                code = legacy.code
            self._session_managed_ids.append(command.request_id)
            accepted = code in {"recorder_started", "stop_accepted"} and not self._journal_error
            if self._journal_error:
                code = "session_journal_failed"
            receipt = ManagedCommandReceipt(accepted, code, self.capture_context().detail, self.capture_context())
        except Exception as exc:
            # Never stop a Recorder merely because publication/bookkeeping failed.
            receipt = ManagedCommandReceipt(False, "command_failed", str(exc), self.capture_context())
        self._managed_requests[command.request_id] = (command, receipt)
        if self._session_dir is not None and not self._persist_lifecycle(self._lifecycle):
            receipt = ManagedCommandReceipt(False, "session_journal_failed", self.capture_context().detail,
                                            self.capture_context())
            self._managed_requests[command.request_id] = (command, receipt)
        return receipt

    def _sync_revision(self) -> None:
        signature = (self.state, self._context_session_id)
        if signature != self._context_signature:
            self._state_revision += 1
            self._context_signature = signature

    def _elapsed_seconds(self) -> float:
        if self._elapsed_final_sec is not None:
            return self._elapsed_final_sec
        if self._recording_started_ns is None:
            return 0.0
        return max(0.0, (self._clock.monotonic_ns() - self._recording_started_ns) / NANOSECONDS_PER_SECOND)

    def _persist_lifecycle(self, lifecycle: str) -> bool:
        self._lifecycle = lifecycle
        self._sync_revision()
        writer = getattr(self._storage, "write_controller_session", None)
        if self._session_dir is None or writer is None:
            return True  # Compatibility for legacy pure-controller storage fakes.
        document = asdict(self.capture_context())
        document.update(version=1, lifecycle=lifecycle, start_parameters=self._start_parameters,
                        managed_requests=[{"request": asdict(self._managed_requests[key][0]),
                                           "receipt": asdict(self._managed_requests[key][1])}
                                          for key in self._session_managed_ids if key in self._managed_requests])
        try:
            writer(self._session_dir, document)
        except Exception as exc:
            self._journal_error = str(exc)
            return False
        self._journal_error = ""
        return True

    def accept(self, command: CaptureCommandFact) -> CommandReceipt:
        if not isinstance(command, CaptureCommandFact):
            raise RequestValidationError("command must be CaptureCommandFact")
        if self._recovery is not None:
            raise InvalidTransitionError(self._context_detail)
        if command.request_id in self._managed_requests:
            raise RequestConflictError(command.request_id, "managed command", "legacy command")

        cached = self._commands.get(command.request_id)
        if cached is not None:
            original_kind, receipt = cached
            if self._command_parameters[command.request_id] != command:
                self._publish_event(
                    "request_conflict",
                    EventSeverity.WARNING,
                    (
                        f"request_id {command.request_id!r} was used for "
                        f"{original_kind.name}, not {command.command.name}"
                    ),
                    request_id=command.request_id,
                )
                raise RequestConflictError(
                    command.request_id,
                    original_kind.name,
                    command.command.name,
                )
            return receipt

        if command.command is CommandKind.START:
            return self._accept_start(command)
        return self._accept_stop(command)

    def tick(self) -> None:
        if self._machine.state is CaptureState.RECORDING:
            recorder = self._require_recorder()
            health = recorder.health()
            if health is RecorderHealth.EXITED:
                self._machine.recorder_exited()
                self._persist_lifecycle("finalizing")
                self._drive_finalization("Recorder exited before STOP")
            elif (
                self._planned_deadline_ns is not None
                and self._clock.monotonic_ns() >= self._planned_deadline_ns
            ):
                self._machine.planned_duration_reached()
                self._persist_lifecycle("finalizing")
                self._drive_finalization("planned duration reached")
        elif self._machine.state is CaptureState.FINALIZING:
            self._drive_finalization("termination retry")

    def publish_periodic_status(self) -> CaptureStatusFact:
        return self._publish_status("periodic status")

    def shutdown(self) -> bool:
        """Stop an active Recorder and report whether exit was confirmed."""

        if self._recovery is not None:
            return False  # No ownership: never signal a recovered/unknown process.
        if self._machine.state in {
            CaptureState.IDLE,
            CaptureState.COMPLETE,
            CaptureState.INVALID,
        }:
            return True
        if self._machine.state is CaptureState.RECORDING:
            self._machine.shutdown_requested()
            self._persist_lifecycle("finalizing")
        return self._drive_finalization(
            "controller shutdown", best_effort_publish=True
        )

    def _accept_start(self, command: CaptureCommandFact) -> CommandReceipt:
        if command.planned_duration_sec > self._config.max_bag_duration_seconds:
            raise RequestValidationError(
                "planned_duration_sec exceeds configured maximum"
            )
        if self._machine.state not in {
            CaptureState.IDLE,
            CaptureState.COMPLETE,
            CaptureState.INVALID,
        }:
            raise InvalidTransitionError(
                f"START is invalid from {self._machine.state.name}"
            )

        if self._machine.state is not CaptureState.IDLE:
            self._machine = CaptureStateMachine(
                request_cache_size=self._request_cache_size
            )
        self._recorder = None
        self._recorder_termination_confirmed = False
        self._context_session_id = ""
        self._start_request_id = command.request_id
        self._session_label = command.session_label
        self._session_dir = None
        self._output_path = ""
        self._recording_started_ns = None
        self._elapsed_final_sec = None
        self._planned_duration_sec = command.planned_duration_sec or self._config.max_bag_duration_seconds
        self._journal_error = ""
        self._start_parameters = asdict(command)
        self._session_managed_ids = []
        try:
            session_id = self._session_id_factory()
            session_dir = self._storage.create_session(
                self._output_root, session_id
            )
        except Exception as exc:
            return self._startup_failed(
                command, "session_create_failed", str(exc)
            )

        self._context_session_id = session_id
        self._session_dir = session_dir
        self._output_path = str(session_dir / "bag")
        self._context_detail = "START intent persisted before Recorder spawn"
        if not self._persist_lifecycle("start_intent"):
            return self._startup_failed(command, "session_journal_failed", self._journal_error)

        qos_path = session_dir / "qos_overrides.yaml"
        try:
            self._storage.write_qos(
                qos_path,
                self._topics,
                topic_types={
                    stream.topic: stream.type_name
                    for stream in self._config.additional_streams
                },
                overrides=self._config.qos_overrides,
            )
        except Exception as exc:
            return self._startup_failed(
                command, "recorder_parameters_failed", str(exc)
            )

        spec = RecorderStartSpec(
            session_id=session_id,
            session_dir=session_dir,
            output_path=session_dir / "bag",
            qos_overrides_path=qos_path,
            topics=self._topics,
        )
        try:
            recorder = self._recorder_factory()
            recorder.start(spec)
        except Exception as exc:
            return self._startup_failed(
                command, "recorder_spawn_failed", str(exc)
            )

        self._recorder = recorder
        transition = self._machine.start(command.request_id, session_id)
        self._recording_started_ns = self._clock.monotonic_ns()
        effective_duration_sec = (
            command.planned_duration_sec
            or self._config.max_bag_duration_seconds
        )
        self._planned_deadline_ns = (
            self._recording_started_ns
            + effective_duration_sec * NANOSECONDS_PER_SECOND
        )
        self._context_detail = "Recorder process started"
        journal_ok = self._persist_lifecycle("recording")
        receipt = CommandReceipt(
            command.request_id,
            command.command,
            transition.code if journal_ok else "session_journal_failed",
            transition.state,
        )
        self._remember(command, receipt)
        self._session_sink(
            SessionInfoFact(session_id, command.request_id, command.session_label)
        )
        self._publish_event(
            "recorder_started",
            EventSeverity.INFO,
            "Recorder process started",
        )
        self._publish_status("Recorder process started")
        return receipt

    def _startup_failed(
        self,
        command: CaptureCommandFact,
        code: str,
        detail: str,
    ) -> CommandReceipt:
        self._context_detail = detail
        self._persist_lifecycle("start_failed")
        receipt = CommandReceipt(
            command.request_id,
            command.command,
            code,
            self._machine.state,
        )
        self._remember(command, receipt)
        self._publish_event(
            code,
            EventSeverity.FATAL,
            detail,
            request_id=command.request_id,
        )
        self._publish_status(detail, request_id=command.request_id)
        return receipt

    def _accept_stop(self, command: CaptureCommandFact) -> CommandReceipt:
        if self._machine.state is CaptureState.IDLE:
            transition = self._machine.stop_while_idle(command.request_id)
            receipt = CommandReceipt(
                command.request_id,
                command.command,
                transition.code,
                transition.state,
            )
            self._remember(command, receipt)
            self._publish_event(
                "stop_while_idle",
                EventSeverity.WARNING,
                "STOP received while idle",
                request_id=command.request_id,
            )
            self._publish_status(
                "STOP received while idle", request_id=command.request_id
            )
            return receipt

        transition = self._machine.stop(command.request_id)
        self._context_detail = "STOP accepted; finalizing"
        self._persist_lifecycle("finalizing")
        receipt = CommandReceipt(
            command.request_id,
            command.command,
            transition.code,
            transition.state,
        )
        self._remember(command, receipt)
        self._publish_event(
            "finalizing",
            EventSeverity.INFO,
            "STOP accepted",
        )
        self._publish_status("STOP accepted; finalizing")
        self._drive_finalization("STOP requested")
        return receipt

    def _drive_finalization(
        self, reason: str, *, best_effort_publish: bool = False
    ) -> bool:
        recorder = self._require_recorder()
        result = recorder.stop()
        if not result.termination_confirmed:
            self._publish_finalization_result(
                code="recorder_termination_unconfirmed",
                severity=EventSeverity.FATAL,
                event_detail=result.detail,
                status_detail="Recorder termination is unconfirmed",
                best_effort=best_effort_publish,
            )
            return False
        self._recorder_termination_confirmed = True
        self._machine.finalize_passed()
        self._elapsed_final_sec = self._elapsed_seconds()
        self._context_detail = "Recorder process lifecycle complete; quality not evaluated"
        self._persist_lifecycle("complete")
        self._publish_finalization_result(
            code="complete",
            severity=EventSeverity.INFO,
            event_detail=f"{reason}; returncode={result.returncode}",
            status_detail="Recorder process lifecycle complete",
            best_effort=best_effort_publish,
        )
        return True

    def _publish_finalization_result(
        self,
        *,
        code: str,
        severity: EventSeverity,
        event_detail: str,
        status_detail: str,
        best_effort: bool,
    ) -> None:
        publishers = (
            lambda: self._publish_event(code, severity, event_detail),
            lambda: self._publish_status(status_detail),
        )
        for publish in publishers:
            if not best_effort:
                publish()
                continue
            try:
                publish()
            except Exception:
                pass

    def _require_recorder(self) -> Recorder:
        if self._recorder is None:
            raise RuntimeError("active session has no Recorder")
        return self._recorder

    def _recorder_alive(self) -> bool:
        if self._recorder is None or self._recorder_termination_confirmed:
            return False
        health = self._recorder.health()
        return health in {RecorderHealth.ALIVE, RecorderHealth.UNCERTAIN}

    def _publish_event(
        self,
        code: str,
        severity: EventSeverity,
        detail: str,
        *,
        request_id: str | None = None,
    ) -> CaptureEventFact:
        event = CaptureEventFact(
            request_id=(
                self._machine.current_request_id
                if request_id is None
                else request_id
            ),
            session_id=self._machine.session_id,
            severity=severity,
            code=code,
            detail=detail,
        )
        self._event_sink(event)
        return event

    def _publish_status(
        self, detail: str, *, request_id: str | None = None
    ) -> CaptureStatusFact:
        if detail != "periodic status":
            self._context_detail = detail
        self._sync_revision()
        effective_request_id = (
            self._machine.current_request_id
            if request_id is None
            else request_id
        )
        recovered = self._recovery is not None
        status = CaptureStatusFact(
            request_id=effective_request_id,
            session_id=self.session_id,
            state=self.state,
            streams=(),
            camera_groups=(),
            recorder_alive=True if recovered else self._recorder_alive(),
            disk_free_bytes=0,
            observed_write_mb_s=0.0,
            detail=self._context_detail if recovered else detail,
        )
        self._status_sink(status)
        return status

    def _remember(
        self, command: CaptureCommandFact, receipt: CommandReceipt
    ) -> None:
        self._commands[receipt.request_id] = (command.command, receipt)
        self._command_parameters[receipt.request_id] = command
        while len(self._commands) > self._request_cache_size:
            key, _ = self._commands.popitem(last=False)
            self._command_parameters.pop(key, None)
