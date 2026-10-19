"""Shape wizard interactions, offline Qt only: no serial, optimizer, or GL loop."""
from copy import deepcopy

import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("pyqtgraph")
pytest.importorskip("OpenGL")

from vt_fsglove12 import live_ui
from test_live_ui import official_window, window, make_measured


def ready(controller, **shape):
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["calibration"].update(profile_status="fitted", clear_generation=0,
        captured_steps=["neutral", "fingers_down", "thumb_up"],
        shape_calibration=dict(status="waiting", contacts={}, busy=False, preview_mode="current", **shape))


def all_contacts():
    return {str(category): {"frames": 20 + category, "confirmed": True} for category in range(2, 6)}


def acknowledge(controller, **shape):
    runtime = controller.state["runtime"]
    runtime["shape_command_generation"] = runtime.get("shape_command_generation", 0) + 1
    controller.state["calibration"]["shape_calibration"].update(shape)


def test_shape_panel_official_only_and_startup_is_read_only(window, official_window):
    legacy, legacy_controller, _ = window
    w, controller = official_window
    assert legacy.shape_panel.isHidden() and legacy.workflow_panel.isHidden()
    assert not w.shape_panel.isHidden() and not w.workflow_panel.isHidden()
    assert not w.shape_panel.confirm.isChecked()
    assert not w.shape_panel.capture_button.isEnabled()
    assert w.shape_panel.load_button.isEnabled()
    assert controller.commands == legacy_controller.commands == []
    for widget in (w.shape_panel.capture_button, w.shape_panel.apply_button, w.shape_panel.revert_button):
        assert w.sidebar_scroll.isAncestorOf(widget)


def test_workflow_navigation_scrolls_to_shape_panel_without_commands(official_window, monkeypatch):
    w, controller = official_window
    w.ensurePolished()
    w.resize(1920, 1000)
    w.layout().activate()
    w.centralWidget().layout().activate()
    w.sidebar_scroll.widget().layout().activate()
    assert w.minimumSizeHint().height() <= 1000
    assert w.workflow_panel.isAncestorOf(w.shape_navigation_button)
    assert w.shape_navigation_button.text() == "前往个性化手形 / 触指标定"
    targets = []
    ensure_visible = w.sidebar_scroll.ensureWidgetVisible

    def scroll_to(widget):
        targets.append(widget)
        ensure_visible(widget)

    monkeypatch.setattr(w.sidebar_scroll, "ensureWidgetVisible", scroll_to)
    w.sidebar_scroll.verticalScrollBar().setValue(0)
    w.shape_navigation_button.click()
    assert targets == [w.shape_panel]
    assert w.sidebar_scroll.verticalScrollBar().value() > 0
    assert controller.commands == []


@pytest.mark.parametrize("category", [2, 3, 4, 5])
def test_each_pair_requires_explicit_fresh_confirmation(official_window, category):
    w, controller = official_window
    ready(controller)
    w.refresh()
    panel = w.shape_panel
    original = deepcopy(controller.state)
    panel.pair_buttons[category].click()
    panel.capture_button.click()
    panel.capture()
    assert controller.commands == [] and not panel.confirm.isChecked()
    panel.confirm.click()
    assert panel.capture_button.isEnabled()
    panel.capture_button.click()
    assert controller.commands == [("capture_shape_contact", dict(category=category, contact_confirmed=True))]
    assert not panel.confirm.isChecked() and not panel.capture_button.isEnabled()
    assert controller.state == original
    panel.capture()
    assert len(controller.commands) == 1


def test_pair_selection_resets_confirmation(official_window):
    w, controller = official_window
    ready(controller)
    w.refresh()
    panel = w.shape_panel
    panel.confirm.click()
    panel.pair_buttons[5].click()
    assert not panel.confirm.isChecked() and "拇指—小指" in panel.confirm.text()
    assert not panel.capture_button.isEnabled() and controller.commands == []


@pytest.mark.parametrize("phase,remaining,words", [("preparing", 2.5, "准备，暂未采集"),
                                                   ("collecting", 1.2, "保持真实轻触")])
def test_countdown_cancel_and_live_geometry_are_separate_from_demo(official_window, phase, remaining, words):
    w, controller = official_window
    ready(controller)
    w.refresh()
    panel = w.shape_panel
    controller.state["calibration"]["shape_calibration"].update(status="capturing", phase=phase,
        remaining_s=remaining, active_category=3, busy=True)
    w.refresh()
    assert words in panel.progress.format() and f"{remaining:.1f}" in panel.progress.format()
    assert "拇指—中指" in panel.progress.format()
    assert panel.cancel_button.isEnabled()
    assert w.view.mesh.visible() and w._demo_key is None
    assert all(not button.isEnabled() for button in w.step_buttons.values())
    assert not w.functional_hand_capture.isEnabled() and not w.functional_capture.isEnabled()
    assert w.clear.isEnabled()
    panel.cancel_button.click()
    assert controller.commands == [("cancel_shape", {})]
    assert not panel.confirm.isChecked()


@pytest.mark.parametrize("kind", ["active", "fitting", "disconnected", "can_capture"])
def test_capture_rechecks_latest_state_before_command(official_window, kind):
    w, controller = official_window
    ready(controller)
    w.refresh()
    panel = w.shape_panel
    panel.confirm.click()
    if kind == "active":
        controller.state["calibration"]["active"] = {"kind": "functional", "finger": "index"}
    elif kind == "fitting":
        controller.state["calibration"]["profile_status"] = "fitting"
    elif kind == "disconnected":
        controller.state["runtime"]["connected"] = False
    else:
        controller.state["calibration"]["shape_calibration"]["can_capture"] = False
    panel.capture_button.click()
    assert controller.commands == []


def test_optimization_requires_all_confirmed_pairs_and_explicit_apply(official_window):
    w, controller = official_window
    ready(controller)
    shape = controller.state["calibration"]["shape_calibration"]
    shape.update(contacts=all_contacts())
    shape["contacts"]["5"]["confirmed"] = False
    w.refresh()
    panel = w.shape_panel
    assert not panel.optimize_button.isEnabled()
    shape["contacts"]["5"]["confirmed"] = True
    w.refresh()
    panel.optimize_button.click()
    panel.optimize_button.click()
    assert controller.commands == [("optimize_shape", {})]
    acknowledge(controller, status="candidate", candidate_available=True, candidate_path="/tmp/candidate.json",
        metrics={"per_pair": {"index": {"before": {"mean_gap_mm": 12.5}, "after": {"mean_gap_mm": 3.25}}}})
    w.refresh()
    assert "候选就绪，尚未采用" in panel.status.text()
    assert "12.50 → 3.25 mm" in panel.metrics.text()
    assert "用于拟合的同批接触，不是独立精度验收" in panel.metrics.text()
    assert panel.apply_button.isEnabled() and controller.commands == [("optimize_shape", {})]
    panel.preview.setCurrentIndex(1)  # Programmatic refresh never submits.
    assert controller.commands == [("optimize_shape", {})]
    panel.preview.activated.emit(1)
    assert controller.commands[-1] == ("preview_shape", {"mode": "candidate"})
    acknowledge(controller, preview_mode="candidate", preview_revision=1)
    controller.state["pose"].update(shape_preview_mode="candidate", shape_preview_revision=1)
    w.refresh()
    assert "仅显示对比" in panel.status.text() and "候选手形预览" in w.shape_view_status.text()
    panel.apply_button.click()
    panel.apply_button.click()
    assert controller.commands[-1] == ("apply_shape_candidate", {})
    assert len(controller.commands) == 3
    acknowledge(controller, status="applied", shape_calibrated=True, applied_path="/tmp/applied.json", can_revert=True)
    w.refresh()
    assert panel.revert_button.isEnabled() and "/tmp/applied.json" in panel.paths.text()
    panel.revert_button.click()
    assert controller.commands[-1] == ("revert_shape", {})


def test_file_dialog_load_disconnected_and_cancel_are_explicit(official_window, monkeypatch, tmp_path):
    w, controller = official_window
    panel = w.shape_panel
    path = str(tmp_path / "chosen.json")
    choices = iter([("", ""), (path, "")])
    monkeypatch.setattr(live_ui.QtWidgets.QFileDialog, "getOpenFileName", lambda *a, **k: next(choices))
    panel.load_button.click()
    assert controller.commands == []
    panel.load_button.click()
    assert controller.commands == [("load_shape_profile", {"path": path})]
    assert not controller.state["runtime"]["connected"]
    # Empty pose calibration must retain the worker's newly loaded preference.
    controller.state["calibration"].update(profile_status="not_calibrated",
        shape_calibration={"status": "applied", "shape_calibrated": True, "applied_path": path})
    w.refresh()
    assert path in panel.paths.text() and "已采用" in panel.status.text()
    assert controller.commands == [("load_shape_profile", {"path": path})]


def test_export_refuses_existing_file_and_sends_selected_new_path(official_window, monkeypatch, tmp_path):
    w, controller = official_window
    ready(controller, candidate_available=True, candidate_path="/tmp/candidate.json")
    w.refresh()
    existing = tmp_path / "existing.json"
    existing.touch()
    target = tmp_path / "new-result.json"
    choices = iter([(str(existing), ""), (str(target), "")])
    monkeypatch.setattr(live_ui.QtWidgets.QFileDialog, "getSaveFileName", lambda *a, **k: next(choices))
    w.shape_panel.save_button.click()
    assert controller.commands == [] and "不覆盖历史结果" in w.shape_panel.status.text()
    w.shape_panel.save_button.click()
    assert controller.commands == [("save_shape_profile", {"path": str(target)})]
    assert not target.exists()  # UI delegates all persistence to the worker.


def test_clear_and_new_session_reset_confirmation_and_candidates(official_window):
    w, controller = official_window
    ready(controller, candidate_available=True, candidate_path="/tmp/old.json")
    w.refresh()
    panel = w.shape_panel
    panel.confirm.click()
    w.clear.click()
    assert not panel.confirm.isChecked() and not panel.apply_button.isEnabled()
    assert "/tmp/old.json" not in panel.paths.text()
    assert controller.commands == [("clear_calibration", {})]
    controller.state["runtime"].update(connected=False, session_id="next-session")
    controller.state["calibration"].update(active=None, captured_steps=[], calibrated_nodes=[],
        clear_generation=1, profile_status="not_calibrated", shape_calibration={"status": "waiting"})
    w.refresh()
    assert not panel.confirm.isChecked() and panel.pairs.checkedId() == 2
    assert not panel.apply_button.isEnabled() and panel.load_button.isEnabled()


def test_scale_inconclusive_never_claims_workflow_complete(official_window):
    w, controller = official_window
    ready(controller, shape_calibrated=True)
    controller.state["calibration"].update(functional_fingers=["index", "middle", "ring", "little", "thumb"],
        input_scale_check={"status": "completed", "inconclusive_nodes": ["6"]})
    before = deepcopy(controller.state)
    w.refresh()
    assert "证据不足（6号）；仍未解决" in w.workflow_status.text()
    assert "不是全手标定全部完成" in w.workflow_status.text()
    assert controller.commands == [] and controller.state == before


def test_shape_command_rejection_unlocks_without_automatic_retry(official_window):
    w, controller = official_window
    ready(controller, candidate_available=True)
    w.refresh()
    w.shape_panel.apply_button.click()
    assert not w.shape_panel.apply_button.isEnabled()
    controller.state["runtime"].update(shape_rejected_count=1, shape_command_error="队列拒绝；未自动重试")
    w.refresh()
    assert w.shape_panel.apply_button.isEnabled()
    assert "队列拒绝" in w.shape_panel.status.text()
    assert controller.commands == [("apply_shape_candidate", {})]


def test_failed_or_delayed_preview_does_not_relabel_retained_mesh(official_window):
    w, controller = official_window
    ready(controller, candidate_available=True)
    w.refresh()
    shape = controller.state["calibration"]["shape_calibration"]
    shape.update(preview_mode="candidate", preview_revision=1)
    # Worker still returns the existing current image: a request is not proof
    # that the candidate was rendered, even at the same source timestamp.
    w.refresh()
    assert "当前已采用手形" in w.shape_view_status.text()
    assert "新预览等待有效网格" in w.shape_view_status.text()
    assert "候选手形预览" not in w.shape_view_status.text()
    # Actual candidate metadata and geometry arrive later, same pose timestamp.
    controller.state["pose"].update(shape_preview_mode="candidate", shape_preview_revision=1)
    w.refresh()
    assert "候选手形预览" in w.shape_view_status.text()
    assert "新预览等待" not in w.shape_view_status.text()
    shape.update(preview_mode="baseline", preview_revision=2)
    controller.state["pose"].update(shape_preview_mode="baseline", shape_preview_revision=2)
    w.backend.fail = True
    w.refresh()
    assert w.view.mesh.visible()
    assert "候选手形预览" in w.shape_view_status.text()
    assert "新预览等待有效网格" in w.shape_view_status.text()
    assert "基准手形对比" not in w.shape_view_status.text()


def test_saved_badges_and_storage_retry_preserve_confirmation_data(official_window):
    w, controller = official_window
    ready(controller, can_save_task=True, can_retry_save=True,
        storage_error="磁盘不可写", storage_message="内存保留，可重试或保存到新位置")
    shape = controller.state["calibration"]["shape_calibration"]
    shape["contacts"] = all_contacts()
    shape["contacts"]["2"]["saved"] = True
    w.refresh()
    panel = w.shape_panel
    assert "已保存" in panel.contact_status[2].text()
    assert "待保存" in panel.contact_status[3].text()
    assert not panel.retry_save_button.isHidden() and panel.retry_save_button.isEnabled()
    assert not panel.retry_save_to_button.isHidden() and panel.retry_save_to_button.isEnabled()
    assert panel.save_task_button.isEnabled()
    assert "磁盘不可写" in panel.status.text() and "内存保留" in panel.status.text()
    before = deepcopy(controller.state)
    panel.retry_save_button.click()
    panel.retry_save_button.click()
    assert controller.commands == [("retry_calibration_save", {})]
    assert controller.state == before
    # Only executed/rejected evidence releases the click latch; no automatic retry.
    w.refresh()
    assert not panel.retry_save_button.isEnabled()
    acknowledge(controller, storage_error=None, storage_message="已保存", can_retry_save=False)
    w.refresh()
    assert panel.retry_save_button.isHidden()
    assert panel.retry_save_to_button.isHidden()
    assert controller.commands == [("retry_calibration_save", {})]


def test_task_export_new_path_does_not_write_or_recapture_in_ui(official_window, monkeypatch, tmp_path):
    w, controller = official_window
    ready(controller, can_save_task=True)
    controller.state["calibration"]["shape_calibration"]["contacts"] = {"2": all_contacts()["2"]}
    w.refresh()
    existing = tmp_path / "existing-task.json"
    existing.touch()
    target = tmp_path / "new-task.json"
    choices = iter([(str(existing), ""), (str(target), "")])
    monkeypatch.setattr(live_ui.QtWidgets.QFileDialog, "getSaveFileName", lambda *a, **k: next(choices))
    panel = w.shape_panel
    panel.save_task_button.click()
    assert controller.commands == [] and "不覆盖历史任务" in panel.status.text()
    panel.save_task_button.click()
    assert controller.commands == [("save_shape_task", {"path": str(target)})]
    assert not target.exists()


def test_base_calibration_storage_failure_can_retry_without_touch_groups(official_window):
    w, controller = official_window
    ready(controller)
    controller.state["runtime"]["calibration_storage"] = dict(
        status="save_failed", error="profile.v001.json: permission denied",
        pending_count=1, retry_available=True)
    w.refresh()
    panel = w.shape_panel
    assert not panel.retry_save_button.isHidden() and panel.retry_save_button.isEnabled()
    assert not panel.optimize_button.isEnabled()
    assert "profile.v001.json: permission denied" in panel.status.text()
    panel.retry_save_button.click()
    assert controller.commands == [("retry_calibration_save", {})]


def test_base_calibration_can_choose_new_save_directory_without_recapture(official_window, monkeypatch, tmp_path):
    w, controller = official_window
    ready(controller)
    controller.state["runtime"]["calibration_storage"] = dict(
        status="save_failed", error="No space left on device", pending_count=1, retry_available=True)
    w.refresh()
    before = deepcopy(controller.state)
    choices = iter(["", str(tmp_path)])
    monkeypatch.setattr(live_ui.QtWidgets.QFileDialog, "getExistingDirectory", lambda *a, **k: next(choices))
    panel = w.shape_panel
    panel.retry_save_to_button.click()
    assert controller.commands == []
    panel.retry_save_to_button.click()
    panel.retry_save_to_button.click()
    assert controller.commands == [("retry_calibration_save_to", {"directory": str(tmp_path)})]
    assert controller.state == before
    assert not panel.retry_save_button.isEnabled() and not panel.retry_save_to_button.isEnabled()
    assert list(tmp_path.iterdir()) == []


def test_new_save_directory_rechecks_pending_state_after_dialog(official_window, monkeypatch):
    w, controller = official_window
    ready(controller)
    controller.state["runtime"]["calibration_storage"] = dict(error="unwritable", retry_available=True)
    w.refresh()
    def choose_after_storage_recovers(*args, **kwargs):
        controller.state["runtime"]["calibration_storage"].update(error=None, retry_available=False)
        return "new-directory"
    monkeypatch.setattr(live_ui.QtWidgets.QFileDialog, "getExistingDirectory", choose_after_storage_recovers)
    w.shape_panel.retry_save_to_button.click()
    assert controller.commands == []
    assert w.shape_panel.retry_save_to_button.isHidden()


def test_task_import_is_disconnected_only_and_never_connects(official_window, monkeypatch, tmp_path):
    w, controller = official_window
    panel = w.shape_panel
    path = str(tmp_path / "frozen-task.json")
    choices = iter([("", ""), (path, "")])
    monkeypatch.setattr(live_ui.QtWidgets.QFileDialog, "getOpenFileName", lambda *a, **k: next(choices))
    assert panel.load_task_button.isEnabled()
    panel.load_task_button.click()
    assert controller.commands == []
    panel.load_task_button.click()
    assert controller.commands == [("load_shape_task", {"path": path})]
    assert not controller.state["runtime"]["connected"]
    ready(controller, can_load_task=True)
    w.refresh()
    assert not panel.load_task_button.isEnabled()
    panel.load_task()
    assert controller.commands == [("load_shape_task", {"path": path})]


@pytest.mark.parametrize("complete,allowed", [(True, True), (False, True), (True, False)])
def test_frozen_task_can_optimize_offline_but_never_preview_apply_or_append(official_window, complete, allowed):
    w, controller = official_window
    contacts = all_contacts()
    if not complete:
        contacts.pop("5")
    controller.state["calibration"].update(active=None, captured_steps=[], calibrated_nodes=[],
        profile_status="not_calibrated", shape_calibration=dict(status="ready", task_loaded=True,
            task_path="frozen-task.json", contacts=contacts, can_optimize=allowed,
            can_save_task=True, can_capture=True, can_apply=True, candidate_available=True))
    w.refresh()
    panel = w.shape_panel
    assert panel.optimize_button.isEnabled() is (complete and allowed)
    assert not panel.capture_button.isEnabled() and not panel.confirm.isEnabled()
    assert not panel.preview.isEnabled() and not panel.apply_button.isEnabled()
    assert panel.save_button.isEnabled() and panel.save_task_button.isEnabled()
    assert "frozen-task.json" in panel.paths.text() and "不续采" in panel.status.text()
    panel.optimize_button.click()
    assert controller.commands == ([("optimize_shape", {})] if complete and allowed else [])
    # Even stale flags during a connection transition cannot append fresh captures.
    ready(controller, task_loaded=True, can_capture=True)
    w.refresh()
    assert not panel.confirm.isEnabled()
    assert all(not button.isEnabled() for button in panel.pair_buttons.values())


def test_task_dialog_rechecks_connection_before_loading(official_window, monkeypatch):
    w, controller = official_window
    def choose_while_connection_changes(*args, **kwargs):
        controller.state["runtime"]["connected"] = True
        return "frozen-task.json", ""
    monkeypatch.setattr(live_ui.QtWidgets.QFileDialog, "getOpenFileName", choose_while_connection_changes)
    w.shape_panel.load_task_button.click()
    assert controller.commands == []
