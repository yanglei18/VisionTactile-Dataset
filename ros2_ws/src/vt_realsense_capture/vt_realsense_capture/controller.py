"""Pure Recorder-only capture lifecycle orchestration."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from enum import Enum, IntEnum
from pathlib import Path
from typing import Callable, Protocol

from .config import CaptureConfig
from .recorder import required_topics
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

    def write_qos(self, path: Path, topics: tuple[str, ...]) -> None: ...


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
            tuple(camera.name for camera in config.cameras)
        )
        self._commands: OrderedDict[
            str, tuple[CommandKind, CommandReceipt]
        ] = OrderedDict()
        self._recorder: Recorder | None = None
        self._recording_started_ns: int | None = None
        self._planned_deadline_ns: int | None = None
        self._recorder_termination_confirmed = False

    @property
    def state(self) -> CaptureState:
        return self._machine.state

    @property
    def session_id(self) -> str:
        return self._machine.session_id

    def accept(self, command: CaptureCommandFact) -> CommandReceipt:
        if not isinstance(command, CaptureCommandFact):
            raise RequestValidationError("command must be CaptureCommandFact")

        cached = self._commands.get(command.request_id)
        if cached is not None:
            original_kind, receipt = cached
            if original_kind is not command.command:
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
                self._drive_finalization("Recorder exited before STOP")
            elif (
                self._planned_deadline_ns is not None
                and self._clock.monotonic_ns() >= self._planned_deadline_ns
            ):
                self._machine.planned_duration_reached()
                self._drive_finalization("planned duration reached")
        elif self._machine.state is CaptureState.FINALIZING:
            self._drive_finalization("termination retry")

    def publish_periodic_status(self) -> CaptureStatusFact:
        return self._publish_status("periodic status")

    def shutdown(self) -> bool:
        """Stop an active Recorder and report whether exit was confirmed."""

        if self._machine.state in {
            CaptureState.IDLE,
            CaptureState.COMPLETE,
            CaptureState.INVALID,
        }:
            return True
        if self._machine.state is CaptureState.RECORDING:
            self._machine.shutdown_requested()
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
        try:
            session_id = self._session_id_factory()
            session_dir = self._storage.create_session(
                self._output_root, session_id
            )
        except Exception as exc:
            return self._startup_failed(
                command, "session_create_failed", str(exc)
            )

        qos_path = session_dir / "qos_overrides.yaml"
        try:
            self._storage.write_qos(qos_path, self._topics)
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
        receipt = CommandReceipt(
            command.request_id,
            command.command,
            transition.code,
            transition.state,
        )
        self._remember(command.command, receipt)
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
        receipt = CommandReceipt(
            command.request_id,
            command.command,
            code,
            self._machine.state,
        )
        self._remember(command.command, receipt)
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
            self._remember(command.command, receipt)
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
        receipt = CommandReceipt(
            command.request_id,
            command.command,
            transition.code,
            transition.state,
        )
        self._remember(command.command, receipt)
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
        effective_request_id = (
            self._machine.current_request_id
            if request_id is None
            else request_id
        )
        status = CaptureStatusFact(
            request_id=effective_request_id,
            session_id=self._machine.session_id,
            state=self._machine.state,
            streams=(),
            camera_groups=(),
            recorder_alive=self._recorder_alive(),
            disk_free_bytes=0,
            observed_write_mb_s=0.0,
            detail=detail,
        )
        self._status_sink(status)
        return status

    def _remember(
        self, kind: CommandKind, receipt: CommandReceipt
    ) -> None:
        self._commands[receipt.request_id] = (kind, receipt)
        while len(self._commands) > self._request_cache_size:
            self._commands.popitem(last=False)
