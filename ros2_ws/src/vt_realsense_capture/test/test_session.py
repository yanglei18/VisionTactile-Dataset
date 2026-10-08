import pytest

from vt_realsense_capture.session import (
    CaptureState,
    CaptureStateMachine,
    InvalidTransitionError,
    RequestValidationError,
)


def test_runtime_path_skips_preflight_and_warmup() -> None:
    machine = CaptureStateMachine()
    started = machine.start("start-1", "session-1")
    assert started.previous_state is CaptureState.IDLE
    assert started.state is CaptureState.RECORDING
    stopped = machine.stop("stop-1")
    assert stopped.state is CaptureState.FINALIZING
    assert stopped.stop_recorder is True
    assert machine.finalize_passed().state is CaptureState.COMPLETE


def test_planned_duration_uses_the_same_finalization_path() -> None:
    machine = CaptureStateMachine()
    machine.start("start-1", "session-1")
    transition = machine.planned_duration_reached()
    assert transition.state is CaptureState.FINALIZING
    assert transition.stop_recorder is True


def test_early_recorder_exit_has_its_own_informational_transition() -> None:
    machine = CaptureStateMachine()
    machine.start("start-1", "session-1")
    transition = machine.recorder_exited()
    assert transition.state is CaptureState.FINALIZING
    assert transition.code == "recorder_exited"
    assert transition.stop_recorder is True


@pytest.mark.parametrize("state", [CaptureState.PREFLIGHT, CaptureState.WARMING_UP])
def test_legacy_wire_states_are_never_runtime_targets(state: CaptureState) -> None:
    machine = CaptureStateMachine()
    assert machine.state is CaptureState.IDLE
    assert machine.start("start-1", "session-1").state is not state


def test_capture_state_wire_values_are_preserved() -> None:
    assert {state.name: state.value for state in CaptureState} == {
        "IDLE": 0,
        "PREFLIGHT": 1,
        "WARMING_UP": 2,
        "RECORDING": 3,
        "FINALIZING": 4,
        "COMPLETE": 5,
        "INVALID": 6,
    }


def test_duplicate_stop_returns_the_cached_transition() -> None:
    machine = CaptureStateMachine()
    machine.start("start-1", "session-1")

    first = machine.stop("stop-1")
    duplicate = machine.stop("stop-1")

    assert duplicate is first
    assert machine.state is CaptureState.FINALIZING


def test_start_after_complete_begins_a_new_recording_session() -> None:
    machine = CaptureStateMachine()
    machine.start("start-1", "session-1")
    machine.stop("stop-1")
    machine.finalize_passed()

    transition = machine.start("start-2", "session-2")

    assert transition.previous_state is CaptureState.COMPLETE
    assert transition.state is CaptureState.RECORDING
    assert transition.request_id == "start-2"
    assert transition.session_id == "session-2"


@pytest.mark.parametrize("request_id", [None, "", "   "])
def test_start_rejects_empty_request_ids(request_id: object) -> None:
    machine = CaptureStateMachine()

    with pytest.raises(RequestValidationError, match="request_id"):
        machine.start(request_id, "session-1")  # type: ignore[arg-type]

    assert machine.state is CaptureState.IDLE


@pytest.mark.parametrize("session_id", [None, "", "   "])
def test_start_rejects_empty_session_ids(session_id: object) -> None:
    machine = CaptureStateMachine()

    with pytest.raises(RequestValidationError, match="session_id"):
        machine.start("start-1", session_id)  # type: ignore[arg-type]

    assert machine.state is CaptureState.IDLE


def test_start_is_rejected_while_recording() -> None:
    machine = CaptureStateMachine()
    machine.start("start-1", "session-1")

    with pytest.raises(InvalidTransitionError, match="START.*RECORDING"):
        machine.start("start-2", "session-2")


def test_start_is_rejected_while_finalizing() -> None:
    machine = CaptureStateMachine()
    machine.start("start-1", "session-1")
    machine.stop("stop-1")

    with pytest.raises(InvalidTransitionError, match="START.*FINALIZING"):
        machine.start("start-2", "session-2")
