from __future__ import annotations

import inspect
from dataclasses import dataclass
from pathlib import Path

import pytest

from vt_realsense_capture.config import load_config
from vt_realsense_capture.controller import (
    CaptureCommandFact,
    CaptureController,
    CaptureEventFact,
    CaptureStatusFact,
    Clock,
    CommandKind,
    EventSeverity,
    RecorderHealth,
    RecorderStartSpec,
    RecorderStopFact,
    SessionInfoFact,
)
from vt_realsense_capture.recorder import required_topics
from vt_realsense_capture.session import (
    CaptureState,
    RequestConflictError,
    RequestValidationError,
)


PACKAGE_ROOT = Path(__file__).parents[1]
CONFIG = load_config(PACKAGE_ROOT / "config" / "cameras.yaml")
EXPECTED_TOPICS = required_topics(tuple(camera.name for camera in CONFIG.cameras))
NANOSECONDS_PER_SECOND = 1_000_000_000


class FakeClock:
    def __init__(self) -> None:
        self.now_ns = 10 * NANOSECONDS_PER_SECOND

    def monotonic_ns(self) -> int:
        return self.now_ns


class FakeStorage:
    def __init__(self, output_root: Path) -> None:
        self.output_root = output_root
        self.created_with: list[tuple[Path, str]] = []
        self.qos_writes: list[tuple[Path, tuple[str, ...]]] = []
        self.create_error: BaseException | None = None
        self.qos_error: BaseException | None = None

    def create_session(self, output_root: Path, session_id: str) -> Path:
        self.created_with.append((output_root, session_id))
        if self.create_error is not None:
            raise self.create_error
        return self.output_root / session_id

    def write_qos(self, path: Path, topics: tuple[str, ...]) -> None:
        self.qos_writes.append((path, topics))
        if self.qos_error is not None:
            raise self.qos_error


class FakeRecorder:
    def __init__(self) -> None:
        self.started_with: RecorderStartSpec | None = None
        self.start_error: BaseException | None = None
        self.health_value = RecorderHealth.ALIVE
        self.stop_fact = RecorderStopFact(
            returncode=0,
            detail="stopped",
            termination_confirmed=True,
            possibly_alive=False,
        )
        self.stop_calls = 0

    def start(self, spec: RecorderStartSpec) -> None:
        if self.start_error is not None:
            raise self.start_error
        self.started_with = spec

    def health(self) -> RecorderHealth:
        return self.health_value

    def stop(self) -> RecorderStopFact:
        self.stop_calls += 1
        return self.stop_fact


def _must_never_run(*_args: object, **_kwargs: object) -> object:
    raise AssertionError("legacy quality provider must never run")


@dataclass
class ControllerFixture:
    controller: CaptureController
    recorder: FakeRecorder
    clock: FakeClock
    storage: FakeStorage
    events: list[CaptureEventFact]
    statuses: list[CaptureStatusFact]
    sessions: list[SessionInfoFact]
    device_provider: object = _must_never_run
    graph: object = _must_never_run
    environment_provider: object = _must_never_run
    bag_validator: object = _must_never_run
    timing_flush: object = _must_never_run
    delivery_wait: object = _must_never_run
    image_subscription_sink: object = _must_never_run


@pytest.fixture
def fixture(tmp_path: Path) -> ControllerFixture:
    clock = FakeClock()
    storage = FakeStorage(tmp_path)
    recorder = FakeRecorder()
    events: list[CaptureEventFact] = []
    statuses: list[CaptureStatusFact] = []
    sessions: list[SessionInfoFact] = []
    controller = CaptureController(
        config=CONFIG,
        output_root=tmp_path,
        clock=clock,
        storage=storage,
        recorder_factory=lambda: recorder,
        event_sink=events.append,
        status_sink=statuses.append,
        session_sink=sessions.append,
        session_id_factory=lambda: "session-1",
    )
    return ControllerFixture(
        controller,
        recorder,
        clock,
        storage,
        events,
        statuses,
        sessions,
    )


def start_command(
    request_id: str = "start-1", *, duration: int = 30
) -> CaptureCommandFact:
    return CaptureCommandFact(
        request_id,
        CommandKind.START,
        "trial",
        duration,
    )


def stop_command(request_id: str = "stop-1") -> CaptureCommandFact:
    return CaptureCommandFact(request_id, CommandKind.STOP)


def test_constructor_contains_no_legacy_quality_or_artifact_providers() -> None:
    parameters = tuple(inspect.signature(CaptureController).parameters)

    assert parameters == (
        "config",
        "output_root",
        "clock",
        "storage",
        "recorder_factory",
        "event_sink",
        "status_sink",
        "session_sink",
        "session_id_factory",
        "request_cache_size",
    )
    assert "graph" not in parameters
    assert "bag_path_inspector" not in parameters


def test_start_immediately_spawns_recorder_without_quality_providers(
    fixture: ControllerFixture,
) -> None:
    receipt = fixture.controller.accept(
        CaptureCommandFact("start-1", CommandKind.START, "trial", 30)
    )

    assert receipt.state is CaptureState.RECORDING
    assert fixture.recorder.started_with is not None
    assert fixture.recorder.started_with.topics == EXPECTED_TOPICS
    assert fixture.recorder.started_with.output_path == Path(
        fixture.storage.output_root / "session-1" / "bag"
    )
    assert fixture.statuses[-1].streams == ()
    assert fixture.statuses[-1].camera_groups == ()
    assert fixture.statuses[-1].recorder_alive is True
    assert fixture.statuses[-1].disk_free_bytes == 0
    assert fixture.statuses[-1].observed_write_mb_s == 0.0
    assert fixture.sessions == [
        SessionInfoFact("session-1", "start-1", "trial")
    ]
    assert fixture.events[-1].code == "recorder_started"


@pytest.mark.parametrize("returncode", [0, 1, -2, 137])
def test_confirmed_stop_completes_regardless_of_returncode(
    fixture: ControllerFixture, returncode: int
) -> None:
    fixture.controller.accept(start_command(duration=30))
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=returncode,
        detail=f"exit {returncode}",
        termination_confirmed=True,
        possibly_alive=False,
    )

    fixture.controller.accept(stop_command())

    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.events[-1].code == "complete"
    assert fixture.events[-1].detail.endswith(f"returncode={returncode}")
    assert fixture.statuses[-1].recorder_alive is False


@pytest.mark.parametrize(
    ("boundary", "error_code"),
    [
        pytest.param("create", "session_create_failed", id="directory"),
        pytest.param("qos", "recorder_parameters_failed", id="qos"),
        pytest.param("spawn", "recorder_spawn_failed", id="spawn"),
    ],
)
def test_startup_failure_stays_idle_and_publishes_fatal_status(
    fixture: ControllerFixture,
    boundary: str,
    error_code: str,
) -> None:
    if boundary == "create":
        fixture.storage.create_error = OSError("mkdir failed")
    elif boundary == "qos":
        fixture.storage.qos_error = OSError("qos failed")
    else:
        fixture.recorder.start_error = OSError("spawn failed")

    receipt = fixture.controller.accept(start_command())

    assert receipt.state is CaptureState.IDLE
    assert receipt.code == error_code
    assert fixture.controller.state is CaptureState.IDLE
    assert fixture.events[-1].severity is EventSeverity.FATAL
    assert fixture.events[-1].code == error_code
    assert fixture.statuses[-1].state is CaptureState.IDLE
    assert fixture.statuses[-1].recorder_alive is False
    assert fixture.recorder.stop_calls == 0
    assert fixture.controller.accept(start_command()) is receipt


def test_qos_failure_does_not_construct_or_start_recorder(tmp_path: Path) -> None:
    clock = FakeClock()
    storage = FakeStorage(tmp_path)
    storage.qos_error = OSError("qos failed")
    recorder_factory_calls = 0

    def recorder_factory() -> FakeRecorder:
        nonlocal recorder_factory_calls
        recorder_factory_calls += 1
        return FakeRecorder()

    controller = CaptureController(
        config=CONFIG,
        output_root=tmp_path,
        clock=clock,
        storage=storage,
        recorder_factory=recorder_factory,
        event_sink=lambda _event: None,
        status_sink=lambda _status: None,
        session_sink=lambda _session: None,
        session_id_factory=lambda: "session-1",
    )

    controller.accept(start_command())

    assert recorder_factory_calls == 0


def test_failed_start_after_complete_returns_to_idle(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())
    fixture.controller.accept(stop_command())
    assert fixture.controller.state is CaptureState.COMPLETE
    fixture.storage.create_error = OSError("mkdir failed")

    receipt = fixture.controller.accept(start_command("start-2"))

    assert receipt.state is CaptureState.IDLE
    assert fixture.controller.state is CaptureState.IDLE
    assert fixture.statuses[-1].state is CaptureState.IDLE


def test_early_process_exit_uses_the_same_process_only_completion(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())
    fixture.recorder.health_value = RecorderHealth.EXITED
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=23,
        detail="unexpected exit",
        termination_confirmed=True,
        possibly_alive=False,
    )

    fixture.controller.tick()

    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.recorder.stop_calls == 1
    assert fixture.events[-1].detail == (
        "Recorder exited before STOP; returncode=23"
    )


def test_unconfirmed_termination_stays_finalizing_and_retries_on_tick(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())
    fixture.recorder.health_value = RecorderHealth.UNCERTAIN
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=None,
        detail="process may still be alive",
        termination_confirmed=False,
        possibly_alive=True,
    )

    fixture.controller.accept(stop_command())

    assert fixture.controller.state is CaptureState.FINALIZING
    assert fixture.events[-1].code == "recorder_termination_unconfirmed"
    assert fixture.statuses[-1].recorder_alive is True
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=-9,
        detail="killed",
        termination_confirmed=True,
        possibly_alive=False,
    )
    fixture.controller.tick()
    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.recorder.stop_calls == 2


def test_shutdown_from_recording_uses_bounded_recorder_stop(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())

    termination_confirmed = fixture.controller.shutdown()

    assert termination_confirmed is True
    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.recorder.stop_calls == 1
    assert fixture.events[-1].code == "complete"


def test_shutdown_from_finalizing_retries_recorder_stop(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=None,
        detail="process may still be alive",
        termination_confirmed=False,
        possibly_alive=True,
    )
    fixture.controller.accept(stop_command())
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=0,
        detail="reaped on shutdown retry",
        termination_confirmed=True,
        possibly_alive=False,
    )

    termination_confirmed = fixture.controller.shutdown()

    assert termination_confirmed is True
    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.recorder.stop_calls == 2


def test_shutdown_completes_after_confirmed_nonzero_exit(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=143,
        detail="terminated",
        termination_confirmed=True,
        possibly_alive=False,
    )

    termination_confirmed = fixture.controller.shutdown()

    assert termination_confirmed is True
    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.events[-1].detail.endswith("returncode=143")


def test_shutdown_keeps_unconfirmed_termination_finalizing(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())
    fixture.recorder.health_value = RecorderHealth.UNCERTAIN
    fixture.recorder.stop_fact = RecorderStopFact(
        returncode=None,
        detail="process may still be alive",
        termination_confirmed=False,
        possibly_alive=True,
    )

    termination_confirmed = fixture.controller.shutdown()

    assert termination_confirmed is False
    assert fixture.controller.state is CaptureState.FINALIZING
    assert fixture.events[-1].severity is EventSeverity.FATAL
    assert fixture.events[-1].code == "recorder_termination_unconfirmed"
    assert fixture.statuses[-1].state is CaptureState.FINALIZING
    assert fixture.statuses[-1].recorder_alive is True
    assert all(event.code != "complete" for event in fixture.events)


def test_shutdown_stops_recorder_when_ros_fact_sinks_are_unavailable(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())

    def unavailable_sink(_fact: object) -> None:
        raise RuntimeError("ROS context is invalid")

    fixture.controller._event_sink = unavailable_sink
    fixture.controller._status_sink = unavailable_sink

    assert fixture.controller.shutdown() is True
    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.recorder.stop_calls == 1


def test_shutdown_is_idempotent_from_idle_and_complete(
    fixture: ControllerFixture,
) -> None:
    assert fixture.controller.shutdown() is True
    assert fixture.controller.shutdown() is True
    assert fixture.controller.state is CaptureState.IDLE
    assert fixture.recorder.stop_calls == 0
    assert fixture.events == []
    assert fixture.statuses == []

    fixture.controller.accept(start_command())
    fixture.controller.accept(stop_command())
    stop_calls = fixture.recorder.stop_calls
    event_count = len(fixture.events)
    status_count = len(fixture.statuses)

    assert fixture.controller.shutdown() is True
    assert fixture.controller.shutdown() is True
    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.recorder.stop_calls == stop_calls
    assert len(fixture.events) == event_count
    assert len(fixture.statuses) == status_count


def test_requested_duration_above_configured_limit_is_rejected_without_io(
    fixture: ControllerFixture,
) -> None:
    with pytest.raises(RequestValidationError, match="planned_duration_sec"):
        fixture.controller.accept(start_command(duration=301))

    assert fixture.controller.state is CaptureState.IDLE
    assert fixture.storage.created_with == []
    assert fixture.recorder.started_with is None


@pytest.mark.parametrize("duration", [0, 300])
def test_zero_or_max_duration_uses_the_300_second_deadline(
    fixture: ControllerFixture, duration: int
) -> None:
    fixture.controller.accept(start_command(duration=duration))

    fixture.clock.now_ns += 299 * NANOSECONDS_PER_SECOND
    fixture.controller.tick()
    assert fixture.controller.state is CaptureState.RECORDING

    fixture.clock.now_ns += NANOSECONDS_PER_SECOND
    fixture.controller.tick()
    assert fixture.controller.state is CaptureState.COMPLETE
    assert fixture.events[-1].detail.startswith("planned duration reached")


def test_duplicate_request_returns_identical_receipt_without_side_effects(
    fixture: ControllerFixture,
) -> None:
    first = fixture.controller.accept(start_command())
    event_count = len(fixture.events)
    status_count = len(fixture.statuses)

    duplicate = fixture.controller.accept(start_command())

    assert duplicate is first
    assert len(fixture.events) == event_count
    assert len(fixture.statuses) == status_count
    assert len(fixture.storage.created_with) == 1


def test_opposite_command_with_same_request_id_reports_conflict_only(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command("same-id"))
    stop_calls = fixture.recorder.stop_calls

    with pytest.raises(RequestConflictError):
        fixture.controller.accept(stop_command("same-id"))

    assert fixture.events[-1].code == "request_conflict"
    assert fixture.controller.state is CaptureState.RECORDING
    assert fixture.recorder.stop_calls == stop_calls


def test_periodic_status_has_only_truthful_minimal_metrics(
    fixture: ControllerFixture,
) -> None:
    fixture.controller.accept(start_command())

    status = fixture.controller.publish_periodic_status()

    assert fixture.statuses[-1] is status
    assert status.streams == ()
    assert status.camera_groups == ()
    assert status.disk_free_bytes == 0
    assert status.observed_write_mb_s == 0.0
    assert status.recorder_alive is True


def test_controller_accepts_the_declared_clock_protocol(
    fixture: ControllerFixture,
) -> None:
    clock: Clock = fixture.clock

    assert clock.monotonic_ns() == fixture.clock.now_ns
