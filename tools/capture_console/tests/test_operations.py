"""Command coordination is independent of Qt and never retries an operation."""
from copy import deepcopy
import subprocess
import sys

import pytest

from vt_capture_console import i18n
from vt_capture_console.viewmodels.operations import OperationCoordinator


@pytest.fixture(autouse=True)
def restore_language():
    previous = i18n.language()
    i18n.set_language("zh_CN", persist=False)
    yield
    i18n.set_language(previous, persist=False)


def _state():
    return {"instance_id": "manager-a"}, {
        "enabled": True, "controller": {"controller_instance_id": "controller-a"},
        "batches": [{"id": "b1", "label": "Existing"}], "trials": [], "operation": None,
    }


def _receipt(state="unknown", code="QUEUED", trial_id="", request_id="r1"):
    return dict(request_id=request_id, state=state, code=code, detail="diagnostic", trial_id=trial_id)


def _trial(state="RECORDING"):
    return {"id": "t1", "state": state, "label": "用户名称", "detail": "trial detail"}


def _submit(coordinator, command="capture.start", *, snapshot=None, management=None):
    original_snapshot, original_management = _state()
    snapshot = original_snapshot if snapshot is None else snapshot
    management = original_management if management is None else management
    calls = []

    def sender(command, params):
        calls.append((command, deepcopy(params)))
        return "r1"

    update = coordinator.submit(command, {"label": "用户名称"}, sender=sender,
                                snapshot=snapshot, management=management)
    return snapshot, management, calls, update


def test_submit_tracks_one_request_and_does_not_dispatch_twice():
    coordinator = OperationCoordinator()
    snapshot, management, calls, update = _submit(coordinator)
    assert coordinator.pending.request_id == "r1"
    assert coordinator.pending.command == "capture.start"
    assert coordinator.pending.manager_id == "manager-a"
    assert coordinator.pending.controller_id == "controller-a"
    assert update.notice_page == "recording"
    assert len(calls) == 1
    second = coordinator.submit("capture.stop", {}, sender=lambda *_: pytest.fail("duplicate dispatch"),
                                snapshot=snapshot, management=management)
    assert second is None
    assert coordinator.pending.request_id == "r1"


def test_queued_and_saved_intent_wait_for_a_confirmed_trial():
    coordinator = OperationCoordinator()
    snapshot, management, calls, _ = _submit(coordinator)
    for receipt in (_receipt(), _receipt("accepted", "intent_saved", "t1")):
        assert coordinator.observe(snapshot, management, receipt) is None
        assert coordinator.pending.request_id == "r1"
    management["trials"] = [_trial("STARTING")]
    assert coordinator.observe(snapshot, management, _receipt("accepted", "intent_saved", "t1")) is None
    management["trials"][0]["state"] = "RECORDING"
    update = coordinator.observe(snapshot, management, _receipt("accepted", "started", "t1"))
    assert coordinator.pending is None
    assert "用户名称" in update.notice
    assert update.notice_page == "recording"
    assert coordinator.observe(snapshot, management, _receipt("accepted", "started", "t1")) is None
    assert len(calls) == 1


@pytest.mark.parametrize("command,waiting,finished", [
    ("capture.start", "STARTING", "COMPLETE"),
    ("capture.start", "STARTING", "FAILED"),
    ("capture.start", "STARTING", "UNKNOWN"),
    ("capture.stop", "RECORDING", "COMPLETE"),
    ("capture.stop", "STOPPING", "FAILED"),
    ("capture.stop", "STOPPING", "UNKNOWN"),
])
def test_terminal_trial_states_finish_only_the_matching_request(command, waiting, finished):
    coordinator = OperationCoordinator()
    snapshot, management, _, _ = _submit(coordinator, command)
    receipt = _receipt("accepted", "intent_saved", "t1")
    management["trials"] = [_trial(waiting)]
    assert coordinator.observe(snapshot, management, receipt) is None
    management["trials"][0]["state"] = finished
    assert coordinator.observe(snapshot, management, {**receipt, "request_id": "other"}) is None
    assert coordinator.observe(snapshot, management, receipt) is not None
    assert coordinator.pending is None


@pytest.mark.parametrize("state,code", [("rejected", "DENIED"), ("unknown", "OUTCOME_UNKNOWN")])
def test_rejected_or_uncertain_outcomes_clear_waiting_without_retry(state, code):
    coordinator = OperationCoordinator()
    snapshot, management, calls, _ = _submit(coordinator)
    update = coordinator.observe(snapshot, management, _receipt(state, code))
    assert code in update.notice and "diagnostic" in update.notice
    assert update.notice_page is None
    assert coordinator.pending is None
    for _ in range(3):
        assert coordinator.observe(snapshot, management, None) is None
    assert len(calls) == 1


@pytest.mark.parametrize("changed", ["manager", "controller"])
def test_replaced_backend_without_matching_trial_ends_waiting(changed):
    coordinator = OperationCoordinator()
    snapshot, management, calls, _ = _submit(coordinator)
    if changed == "manager":
        snapshot["instance_id"] = "manager-b"
    else:
        management["controller"]["controller_instance_id"] = "controller-b"
    update = coordinator.observe(snapshot, management, _receipt("accepted", "saved", "t1"))
    assert "待核实" in update.notice
    assert coordinator.pending is None
    assert len(calls) == 1


def test_replaced_backend_with_matching_trial_keeps_confirmation_path():
    coordinator = OperationCoordinator()
    snapshot, management, _, _ = _submit(coordinator)
    snapshot["instance_id"] = "manager-b"
    management["trials"] = [_trial("STARTING")]
    receipt = _receipt("accepted", "saved", "t1")
    assert coordinator.observe(snapshot, management, receipt) is None
    assert coordinator.pending is not None
    management["trials"][0]["state"] = "RECORDING"
    assert "用户名称" in coordinator.observe(snapshot, management, receipt).notice
    assert coordinator.pending is None


def test_temporary_missing_context_does_not_claim_backend_replacement():
    coordinator = OperationCoordinator()
    _, management, _, _ = _submit(coordinator)
    management["controller"] = None
    assert coordinator.observe({}, management, None) is None
    assert coordinator.pending is not None


def test_live_batch_confirmation_does_not_select_another_clients_batch():
    coordinator = OperationCoordinator()
    snapshot, management, _, _ = _submit(coordinator, "batch.create")
    management["batches"].append({"id": "other", "label": "Another client"})
    assert coordinator.pending.batch_ids == frozenset({"b1"})
    assert coordinator.observe(snapshot, management, _receipt()) is None
    update = coordinator.observe(snapshot, management, _receipt("accepted", "batch_created"))
    assert update.clear_batch_name and update.update_batch_selection
    assert update.batch_id is None
    assert coordinator.pending is None


@pytest.mark.parametrize("added,selected", [(1, "new-0"), (2, None)])
def test_demo_selects_only_one_new_memory_batch(added, selected):
    coordinator = OperationCoordinator(demo=True)
    snapshot, management, _, _ = _submit(coordinator, "batch.create")
    management["batches"].extend({"id": f"new-{index}"} for index in range(added))
    update = coordinator.observe(snapshot, management, _receipt("accepted", "DEMO_APPLIED"))
    assert update.update_batch_selection and update.clear_batch_name
    assert update.batch_id == selected


def test_quality_acceptance_refreshes_details_and_preserves_trial_state():
    coordinator = OperationCoordinator()
    snapshot, management, _, submitted = _submit(coordinator, "quality.retry")
    management["trials"] = [_trial("COMPLETE")]
    before = deepcopy(management)
    assert submitted.notice_page == "archive"
    update = coordinator.observe(snapshot, management, _receipt("accepted", "quality_queued", "t1"))
    assert update.refresh_details
    assert update.notice_page == "archive"
    assert coordinator.pending is None
    assert management == before


@pytest.mark.parametrize("error", [ValueError("invalid"), RuntimeError("disconnected"), OSError("socket")])
def test_submission_error_leaves_no_pending_operation(error):
    coordinator = OperationCoordinator()
    snapshot, management = _state()

    def fail(*_):
        raise error

    update = coordinator.submit("capture.start", {}, sender=fail, snapshot=snapshot, management=management)
    assert coordinator.pending is None
    assert str(error) in update.notice
    assert update.notice_page is None


def test_snapshot_receipt_is_used_when_latest_receipt_belongs_to_other_request():
    coordinator = OperationCoordinator()
    snapshot, management, _, _ = _submit(coordinator)
    management["operation"] = _receipt("rejected", "FROM_SNAPSHOT")
    update = coordinator.observe(snapshot, management, _receipt(request_id="other"))
    assert "FROM_SNAPSHOT" in update.notice
    assert coordinator.pending is None


def test_language_switch_changes_only_presentation_and_preserves_user_text():
    previous = i18n.language()
    try:
        i18n.set_language("zh_CN", persist=False)
        coordinator = OperationCoordinator()
        snapshot, management, calls, submitted = _submit(coordinator)
        pending = coordinator.pending
        i18n.set_language("en", persist=False)
        assert "backend confirmation" in str(submitted.notice).lower()
        assert coordinator.pending is pending
        management["trials"] = [_trial()]
        update = coordinator.observe(snapshot, management, _receipt("accepted", "started", "t1"))
        assert "用户名称" in str(update.notice)
        assert calls[0][1]["label"] == "用户名称"
    finally:
        i18n.set_language(previous, persist=False)


def test_coordinator_import_does_not_load_qt_or_ui():
    result = subprocess.run([sys.executable, "-c", (
        "import sys; import vt_capture_console.viewmodels.operations; "
        "assert not any(name.startswith(('PySide6', 'vt_capture_console.ui')) for name in sys.modules)"
    )], capture_output=True, text=True, timeout=15, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
