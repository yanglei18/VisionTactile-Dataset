"""Ownership policy matrix, independent of storage, threads and ROS."""
import pytest

from vt_capture_manager.reconciliation import observe_trial


def trial(**changes):
    value = dict(controller_instance_id="one", start_request_id="start", session_id="", state="STARTING")
    value.update(changes)
    return value


def context(**changes):
    value = dict(controller_instance_id="one", start_request_id="start", session_id="allocated",
                 state=0, recorder_alive=False, detail="failed before Recorder spawn")
    value.update(changes)
    return value


@pytest.mark.parametrize("state", [0, 5, 6])
def test_idle_or_terminal_label_never_overrides_a_possibly_live_recorder(state):
    result = observe_trial(trial(), context(state=state, recorder_alive=True), start_rejection="rejected")
    assert result.state == "UNKNOWN"
    assert not result.start_failed and not result.freeze_finish


@pytest.mark.parametrize("changes", [
    {"controller_instance_id": "restarted"},
    {"start_request_id": "external", "state": 3, "recorder_alive": True},
    {"session_id": "", "state": 3, "recorder_alive": True},
])
def test_negative_receipt_cannot_resolve_uncertain_identity(changes):
    result = observe_trial(trial(), context(**changes), start_rejection="rejected")
    assert result.state == "UNKNOWN"
    assert not result.bind_context and not result.freeze_finish


@pytest.mark.parametrize("changes", [{"session_id": "other"}, {"start_request_id": "external"}, {"state": 0}])
@pytest.mark.parametrize("rejection", [None, "journal failed after Recorder started"])
def test_existing_ownership_is_not_erased_by_incompatible_observations(changes, rejection):
    result = observe_trial(trial(session_id="allocated", state="UNKNOWN"), context(**changes),
                           start_rejection=rejection)
    assert result.state == "UNKNOWN"
    assert not result.bind_context and not result.freeze_finish


def test_snapshot_of_another_request_does_not_resolve_a_lost_response():
    assert observe_trial(trial(), context(start_request_id="previous")) is None


@pytest.mark.parametrize("session_id", ["", "allocated"])
def test_matching_idle_without_recorder_confirms_failure_without_quality(session_id):
    result = observe_trial(trial(), context(session_id=session_id))
    assert result.state == "FAILED" and result.start_failed and result.bind_context
    assert not result.start_effect and not result.stop_effect and not result.freeze_finish


@pytest.mark.parametrize("stop,expected", [(None, "RECORDING"), ("accepted", "STOPPING"), ("unknown", "UNKNOWN")])
def test_pending_stop_remains_unresolved_until_effect_observed(stop, expected):
    result = observe_trial(trial(), context(state=3, recorder_alive=True), pending_stop=stop)
    assert result.state == expected and result.bind_context and result.start_effect
    assert not result.stop_effect and not result.freeze_finish


@pytest.mark.parametrize("rejection,expected", [(None, "UNKNOWN"), ("request ID conflict", "FAILED")])
def test_initial_session_binding_requires_evidence_after_start_intent(rejection, expected):
    result = observe_trial(trial(), context(state=5, state_revision=7),
                           start_rejection=rejection, expected_start_revision=7)
    assert result.state == expected
    assert not result.bind_context and not result.start_effect and not result.freeze_finish


def test_new_recording_remains_owned_when_bookkeeping_receipt_is_negative():
    result = observe_trial(trial(), context(state=3, state_revision=8, recorder_alive=True),
                           start_rejection="session journal failed", expected_start_revision=7)
    assert result.state == "RECORDING" and result.bind_context and result.start_effect


def test_request_conflict_cannot_bind_an_external_session_that_advanced_revision():
    result = observe_trial(trial(), context(state=3, state_revision=8, recorder_alive=True),
                           start_rejection="request ID conflict", start_rejection_code="request_id_conflict",
                           expected_start_revision=7)
    assert result.state == "FAILED" and result.start_failed
    assert not result.bind_context and not result.freeze_finish
