"""Capture lifecycle state and command identity rules."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from enum import Enum, IntEnum


DEFAULT_REQUEST_CACHE_SIZE = 256


class CaptureState(IntEnum):
    """Values intentionally match ``vt_camera_msgs/CaptureStatus``."""

    IDLE = 0
    PREFLIGHT = 1
    WARMING_UP = 2
    RECORDING = 3
    FINALIZING = 4
    COMPLETE = 5
    INVALID = 6


class InvalidTransitionError(RuntimeError):
    """Raised when an event is not legal in the current capture state."""


class RequestValidationError(ValueError):
    """Raised for malformed request or session identifiers."""


class RequestConflictError(RequestValidationError):
    """Raised when one request ID is reused for a different command kind."""

    def __init__(
        self, request_id: str, original_command: str, attempted_command: str
    ) -> None:
        super().__init__(
            f"request_id {request_id!r} was used for {original_command}, "
            f"not {attempted_command}"
        )
        self.code = "request_id_conflict"
        self.request_id = request_id
        self.original_command = original_command
        self.attempted_command = attempted_command


@dataclass(frozen=True)
class Transition:
    """Immutable result of one accepted state-machine event."""

    state: CaptureState
    previous_state: CaptureState
    code: str
    detail: str = ""
    request_id: str = ""
    session_id: str = ""
    stop_recorder: bool = False
    sequence: int = 0


class _CommandKind(Enum):
    START = "START"
    STOP = "STOP"


@dataclass(frozen=True)
class _CachedRequest:
    command: _CommandKind
    transition: Transition


def _nonempty_identifier(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RequestValidationError(f"{name} must be a nonempty string")
    return value


class CaptureStateMachine:
    """Enforce capture lifecycle transitions and command idempotency."""

    def __init__(
        self, *, request_cache_size: int = DEFAULT_REQUEST_CACHE_SIZE
    ) -> None:
        if type(request_cache_size) is not int or request_cache_size <= 0:
            raise ValueError("request_cache_size must be a positive integer")
        self._state = CaptureState.IDLE
        self._request_cache_size = request_cache_size
        self._requests: OrderedDict[str, _CachedRequest] = OrderedDict()
        self._current_request_id = ""
        self._session_id = ""
        self._sequence = 0

    @property
    def state(self) -> CaptureState:
        return self._state

    @property
    def current_request_id(self) -> str:
        return self._current_request_id

    @property
    def session_id(self) -> str:
        return self._session_id

    @property
    def cached_request_ids(self) -> tuple[str, ...]:
        return tuple(self._requests)

    def _require_state(
        self, event: str, allowed_states: set[CaptureState]
    ) -> None:
        if self._state not in allowed_states:
            allowed = ", ".join(state.name for state in sorted(allowed_states))
            raise InvalidTransitionError(
                f"{event} is invalid from {self._state.name}; expected {allowed}"
            )

    def _result(
        self,
        state: CaptureState,
        code: str,
        *,
        detail: str = "",
        stop_recorder: bool = False,
        request_id: str | None = None,
    ) -> Transition:
        previous_state = self._state
        self._state = state
        self._sequence += 1
        return Transition(
            state=state,
            previous_state=previous_state,
            code=code,
            detail=detail,
            request_id=(
                self._current_request_id if request_id is None else request_id
            ),
            session_id=self._session_id,
            stop_recorder=stop_recorder,
            sequence=self._sequence,
        )

    def _cached_transition(
        self, request_id: str, command: _CommandKind
    ) -> Transition | None:
        cached = self._requests.get(request_id)
        if cached is None:
            return None
        if cached.command is not command:
            raise RequestConflictError(
                request_id,
                cached.command.value,
                command.value,
            )
        return cached.transition

    def _remember(
        self,
        request_id: str,
        command: _CommandKind,
        transition: Transition,
    ) -> Transition:
        self._requests[request_id] = _CachedRequest(command, transition)
        while len(self._requests) > self._request_cache_size:
            self._requests.popitem(last=False)
        return transition

    def start(self, request_id: str, session_id: str) -> Transition:
        """Enter RECORDING directly from IDLE or a terminal state."""

        request_id = _nonempty_identifier("request_id", request_id)
        session_id = _nonempty_identifier("session_id", session_id)
        cached = self._cached_transition(request_id, _CommandKind.START)
        if cached is not None:
            return cached
        self._require_state(
            "START", {CaptureState.IDLE, CaptureState.COMPLETE, CaptureState.INVALID}
        )
        self._current_request_id = request_id
        self._session_id = session_id
        return self._remember(
            request_id,
            _CommandKind.START,
            self._result(CaptureState.RECORDING, "recorder_started"),
        )

    def stop(self, request_id: str) -> Transition:
        """Accept a STOP once, returning the cached result for duplicates."""

        request_id = _nonempty_identifier("request_id", request_id)
        cached = self._cached_transition(request_id, _CommandKind.STOP)
        if cached is not None:
            return cached
        self._require_state("STOP", {CaptureState.RECORDING})
        self._current_request_id = request_id
        return self._remember(
            request_id,
            _CommandKind.STOP,
            self._result(
                CaptureState.FINALIZING,
                "stop_accepted",
                stop_recorder=True,
                request_id=request_id,
            ),
        )

    def planned_duration_reached(self) -> Transition:
        """Enter FINALIZING without inventing a wire-command request ID."""

        self._require_state("planned_duration_reached", {CaptureState.RECORDING})
        return self._result(
            CaptureState.FINALIZING,
            "planned_duration_complete",
            stop_recorder=True,
        )

    def recorder_exited(self) -> Transition:
        """Enter FINALIZING after the Recorder exits before a stop request."""

        self._require_state("recorder_exited", {CaptureState.RECORDING})
        return self._result(
            CaptureState.FINALIZING,
            "recorder_exited",
            stop_recorder=True,
        )

    def shutdown_requested(self) -> Transition:
        """Enter FINALIZING so node shutdown owns Recorder termination."""

        self._require_state("shutdown_requested", {CaptureState.RECORDING})
        return self._result(
            CaptureState.FINALIZING,
            "shutdown_requested",
            stop_recorder=True,
        )

    def stop_while_idle(self, request_id: str) -> Transition:
        """Acknowledge and cache an idempotent STOP without changing IDLE."""

        request_id = _nonempty_identifier("request_id", request_id)
        cached = self._cached_transition(request_id, _CommandKind.STOP)
        if cached is not None:
            return cached
        self._require_state("stop_while_idle", {CaptureState.IDLE})
        return self._remember(
            request_id,
            _CommandKind.STOP,
            self._result(
                CaptureState.IDLE,
                "stop_while_idle",
                request_id=request_id,
            ),
        )

    def finalize_passed(self) -> Transition:
        self._require_state("finalize_passed", {CaptureState.FINALIZING})
        return self._result(CaptureState.COMPLETE, "finalize_passed")

    def finalize_failed(self, code: str, detail: str = "") -> Transition:
        """Retain INVALID as a terminal wire-compatibility result."""

        self._require_state("finalize_failed", {CaptureState.FINALIZING})
        code = _nonempty_identifier("code", code)
        if not isinstance(detail, str):
            raise RequestValidationError("detail must be a string")
        return self._result(CaptureState.INVALID, code, detail=detail)
