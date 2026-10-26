"""Cross-page calibration facts and save actions, without devices or GL paint."""
from copy import deepcopy

import pytest

from test_live_ui_workspace import workspace, receive_required_nodes, assert_global


BASE_ERROR = "profile.v007.json: directory is not writable"
TOUCH_ERROR = "touch-task.json: disk is full"


def ready_saved(window, controller, now):
    receive_required_nodes(window, controller, now)
    controller.state["pose"]["profile_version"] = 7
    controller.state["calibration"].update(clear_generation=0,
        shape_calibration={"status": "waiting", "busy": False, "contacts": {},
                           "preview_mode": "current"})
    controller.state["runtime"].update(
        local_recording={"enabled": True, "profile_saved": True, "warning": ""},
        calibration_storage={"error": None, "retry_available": False},
        shape_command_generation=0)


def fail_saves(controller, *, base=False, touch=False):
    controller.state["runtime"]["calibration_storage"].update(
        error=BASE_ERROR if base else None, retry_available=base)
    controller.state["calibration"]["shape_calibration"].update(
        storage_error=TOUCH_ERROR if touch else None, can_retry_save=touch,
        storage_message="触指数据保留在内存，可重试保存。" if touch else "")
    if touch:
        controller.state["calibration"]["shape_calibration"]["contacts"] = {
            "2": {"frames": 12, "confirmed": True, "saved": False},
        }


def retry_buttons(window):
    return (window.retry_recording_save, window.choose_recording_directory,
            window.shape_panel.retry_save_button, window.shape_panel.retry_save_to_button)


def test_epoch_invalidated_profile_is_never_reported_as_effective(workspace):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    window.refresh()
    assert "已生效" in window.result_status.text()
    controller.state["pose"].update(pose_available=False, solver_success=False,
        reason="calibration_epoch_changed",
        calibration_rejections={"11": "filter_epoch_changed_recalibration_required"})
    before = deepcopy(controller.state)
    window.refresh()
    assert "失效" in window.result_status.text()
    assert "已生效" not in window.result_status.text()
    assert "已保存" in window.result_status.text()
    assert "失效" in window.workflow_status.text()
    assert "已生效" not in window.workflow_status.text()
    assert "失效" in window.visual_status.text()
    assert controller.state == before and controller.commands == []


@pytest.mark.parametrize("resolved", ["base", "touch"])
def test_both_save_failures_keep_the_other_error_after_partial_recovery(workspace, resolved):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, base=True, touch=True)
    window.refresh()
    for label in (window.storage_error_status, window.shape_panel.status):
        assert BASE_ERROR in label.text() and TOUCH_ERROR in label.text()
    assert all(button.isEnabled() for button in retry_buttons(window))
    fail_saves(controller, base=resolved != "base", touch=resolved != "touch")
    controller.state["runtime"]["shape_command_generation"] += 1
    before = deepcopy(controller.state)
    window.refresh()
    remaining, removed = ((TOUCH_ERROR, BASE_ERROR) if resolved == "base"
                          else (BASE_ERROR, TOUCH_ERROR))
    for label in (window.storage_error_status, window.shape_panel.status):
        assert remaining in label.text() and removed not in label.text()
    assert_global(window.storage_error_status, window)
    assert all(button.isEnabled() for button in retry_buttons(window))
    assert controller.state == before and controller.commands == []


@pytest.mark.parametrize("busy", ["base_capture", "base_fitting", "shape_capture", "shape_optimizing", "clearing"])
def test_every_save_retry_entry_uses_the_same_busy_guard(workspace, busy):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, base=True, touch=True)
    window.refresh()
    assert all(button.isEnabled() for button in retry_buttons(window))
    calibration = controller.state["calibration"]
    if busy == "base_capture":
        calibration["active"] = dict(kind="pose", step="neutral", phase="preparing", remaining_s=2.)
    elif busy == "base_fitting":
        calibration["profile_status"] = "fitting"
    elif busy in {"shape_capture", "shape_optimizing"}:
        calibration["shape_calibration"].update(
            busy=True, status="capturing" if busy == "shape_capture" else "optimizing")
    else:
        calibration["profile_status"] = "clearing"
    before = deepcopy(controller.state)
    window.refresh()
    assert all(not button.isEnabled() for button in retry_buttons(window))
    for button in retry_buttons(window):
        button.click()
    assert controller.state == before and controller.commands == []


@pytest.mark.parametrize("origin", ["footer", "shape"])
def test_save_retry_is_shared_and_waits_for_backend_acknowledgement(workspace, origin):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, base=True, touch=True)
    window.refresh()
    first, second = ((window.retry_recording_save, window.shape_panel.retry_save_button)
                     if origin == "footer" else
                     (window.shape_panel.retry_save_button, window.retry_recording_save))
    before = deepcopy(controller.state)
    first.click()
    second.click()
    first.click()
    assert controller.commands == [("retry_calibration_save", {})]
    assert controller.state == before
    assert all(not button.isEnabled() for button in retry_buttons(window))
    # Unrelated input activity is not an acknowledgement of a disk operation.
    controller.state["runtime"]["received_samples"] = 1234
    window.refresh()
    assert all(not button.isEnabled() for button in retry_buttons(window))
    assert BASE_ERROR in window.storage_error_status.text()
    assert TOUCH_ERROR in window.storage_error_status.text()
    # One source recovers; the unresolved source remains retryable everywhere.
    controller.state["runtime"]["shape_command_generation"] += 1
    fail_saves(controller, touch=True)
    window.refresh()
    assert BASE_ERROR not in window.storage_error_status.text()
    assert TOUCH_ERROR in window.storage_error_status.text()
    assert all(button.isEnabled() for button in retry_buttons(window))
    second.click()
    first.click()
    assert controller.commands == [("retry_calibration_save", {})] * 2
    controller.state["runtime"]["shape_command_generation"] += 1
    fail_saves(controller)
    window.refresh()
    assert window.storage_error_status.isHidden()
    assert all(button.isHidden() for button in retry_buttons(window))
    assert controller.commands == [("retry_calibration_save", {})] * 2


def test_touch_task_save_failure_and_retry_are_visible_on_every_page(workspace):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, touch=True)
    before = deepcopy(controller.state)
    for index in range(window.workspace_tabs.count()):
        window.workspace_tabs.setCurrentIndex(index)
        window.refresh()
        assert TOUCH_ERROR in window.storage_error_status.text()
        assert_global(window.storage_error_status, window)
        assert_global(window.retry_recording_save, window)
        assert window.retry_recording_save.isEnabled()
        assert window.choose_recording_directory.isEnabled()
        assert window.shape_panel.retry_save_button.isEnabled()
    assert controller.state == before and controller.commands == []


def test_local_epoch_failure_is_distinguished_from_complete_reference_loss(workspace):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    window.refresh()
    controller.state["pose"].update(reason="official_inference_complete",
        calibration_rejections={"3": "filter_epoch_changed_recalibration_required"})
    before = deepcopy(controller.state)
    window.refresh()
    for text in (window.result_status.text(), window.workflow_status.text()):
        assert "部分" in text and "失效" in text
        assert "已生效" not in text
    assert "已保存" in window.result_status.text()
    assert window.view.mesh.visible()
    assert controller.state == before and controller.commands == []


@pytest.mark.parametrize("input_gap", ["missing_root", "stale_publication"])
def test_temporary_input_gap_does_not_claim_effective_or_permanently_invalid(workspace, input_gap):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    window.refresh()
    if input_gap == "missing_root":
        controller.state["pose"].update(pose_available=False, solver_success=False,
            reason="missing_or_stale_root", calibration_rejections={"11": "missing_or_stale_node"})
    else:
        clock[0] += 1.
    before = deepcopy(controller.state)
    window.refresh()
    for text in (window.result_status.text(), window.workflow_status.text()):
        assert "已生效" not in text and "失效" not in text
    assert "已保存" in window.result_status.text()
    assert controller.state == before and controller.commands == []


def test_rejected_retry_submission_does_not_lock_available_save_actions(workspace):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, base=True, touch=True)
    window.refresh()
    attempts = []

    def reject(name, **args):
        attempts.append((name, args))
        return False

    controller.submit = reject
    before = deepcopy(controller.state)
    window.retry_recording_save.click()
    window.refresh()
    assert attempts == [("retry_calibration_save", {})]
    assert all(button.isEnabled() for button in retry_buttons(window))
    assert BASE_ERROR in window.storage_error_status.text()
    assert TOUCH_ERROR in window.storage_error_status.text()
    assert controller.state == before


def test_sparse_save_failures_stay_visible_and_new_session_does_not_inherit_them(workspace):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, base=True, touch=True)
    window.refresh()
    controller.state["runtime"].pop("calibration_storage")
    shape = controller.state["calibration"]["shape_calibration"]
    for key in ("storage_error", "storage_message", "can_retry_save"):
        shape.pop(key)
    before = deepcopy(controller.state)
    for index in range(window.workspace_tabs.count()):
        window.workspace_tabs.setCurrentIndex(index)
        window.refresh()
        assert_global(window.storage_error_status, window)
        for label in (window.storage_error_status, window.shape_panel.status):
            assert BASE_ERROR in label.text() and TOUCH_ERROR in label.text()
        assert all(button.isEnabled() for button in retry_buttons(window))
    assert controller.state == before and controller.commands == []
    # A new backend session explicitly starts with no storage failure. The old
    # session's cached failures cannot replace this newer acknowledgement.
    ready_saved(window, controller, clock[0])
    controller.state["runtime"]["session_id"] = "new-session-with-explicit-storage-state"
    controller.state["pose"]["profile_version"] = 1
    fail_saves(controller)
    before = deepcopy(controller.state)
    for index in range(window.workspace_tabs.count()):
        window.workspace_tabs.setCurrentIndex(index)
        window.refresh()
        assert window.storage_error_status.isHidden()
        assert all(button.isHidden() for button in retry_buttons(window))
        assert BASE_ERROR not in window.shape_panel.status.text()
        assert TOUCH_ERROR not in window.shape_panel.status.text()
    assert controller.state == before and controller.commands == []


def test_backend_rejected_save_ack_unlocks_shared_actions_without_erasing_errors(workspace):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, base=True, touch=True)
    controller.state["runtime"]["shape_rejected_count"] = 0
    window.refresh()
    window.retry_recording_save.click()
    assert all(not button.isEnabled() for button in retry_buttons(window))
    controller.state["runtime"].update(shape_rejected_count=1,
        shape_command_error="保存命令队列繁忙，未执行。")
    before = deepcopy(controller.state)
    window.refresh()
    assert all(button.isEnabled() for button in retry_buttons(window))
    assert BASE_ERROR in window.storage_error_status.text()
    assert TOUCH_ERROR in window.storage_error_status.text()
    assert controller.state == before
    assert controller.commands == [("retry_calibration_save", {})]
    window.shape_panel.retry_save_button.click()
    assert controller.commands == [("retry_calibration_save", {})] * 2
    assert all(not button.isEnabled() for button in retry_buttons(window))


def test_epoch_failure_blocks_derived_calibration_but_allows_basic_recalibration(workspace):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    window.refresh()
    assert window.functional_hand_capture.isEnabled()
    assert window.shape_panel.confirm.isEnabled()
    window.shape_panel.confirm.setChecked(True)
    assert window.shape_panel.capture_button.isEnabled()
    controller.state["pose"].update(pose_available=False, solver_success=False,
        reason="calibration_epoch_changed",
        calibration_rejections={"11": "filter_epoch_changed_recalibration_required"})
    before = deepcopy(controller.state)
    window.refresh()
    assert all(button.isEnabled() for button in window.step_buttons.values())
    assert not window.functional_hand_capture.isEnabled()
    assert not window.functional_capture.isEnabled()
    assert not window.shape_panel.confirm.isEnabled()
    assert not window.shape_panel.capture_button.isEnabled()
    assert "失效" in window.instructions.text()
    assert "已完成" not in window.instructions.text()
    window.functional_hand_capture.click()
    window.shape_panel.capture_button.click()
    assert controller.state == before and controller.commands == []


@pytest.mark.parametrize("origin", ["footer", "shape"])
def test_directory_selection_rechecks_busy_state_before_sending_save(workspace, monkeypatch, tmp_path, origin):
    window, controller, clock = workspace
    ready_saved(window, controller, clock[0])
    fail_saves(controller, base=True, touch=True)
    window.refresh()
    dialogs = []

    def choose_then_capture(*args, **kwargs):
        dialogs.append(True)
        controller.state["calibration"]["active"] = dict(
            kind="pose", step="neutral", phase="preparing", remaining_s=2.)
        return str(tmp_path)

    monkeypatch.setattr("vt_fsglove12.live_ui.QtWidgets.QFileDialog.getExistingDirectory", choose_then_capture)
    button = (window.choose_recording_directory if origin == "footer"
              else window.shape_panel.retry_save_to_button)
    button.click()
    assert dialogs == [True]
    assert controller.commands == []
    assert all(not button.isEnabled() for button in retry_buttons(window))
    assert BASE_ERROR in window.storage_error_status.text()
    assert TOUCH_ERROR in window.storage_error_status.text()
