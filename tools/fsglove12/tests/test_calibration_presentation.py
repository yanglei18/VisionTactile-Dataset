"""Calibration evidence and save failures agree without Qt, hardware or a clock."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

from vt_fsglove12.calibration_presentation import (
    calibration_status, save_actions_allowed, shape_busy, storage_status, workflow_status_text,
)


def snapshot():
    return {
        "runtime": {"connected": True, "session_id": "current", "published_monotonic_s": 100.,
                    "last_sample_age_s": .005, "local_recording": {"profile_saved": True}},
        "calibration": {"calibrated_nodes": ["1", "11"], "profile_status": "fitted",
                        "captured_steps": ["neutral", "fingers_down", "thumb_up"],
                        "active": None, "rejected_nodes": {}, "functional_fingers": ["index"],
                        "shape_calibration": {}},
        "pose": {"session_id": "current", "timestamp_s": 100., "profile_version": 2,
                 "pose_available": True, "solver_success": True, "calibration_rejections": {},
                 "reason": "official_inference_complete"},
    }


@pytest.mark.parametrize("saved,expected", [(True, "已保存"), (False, "未保存"),
                                            (None, "保存状态待确认"), (1, "保存状态待确认")])
def test_reference_validity_and_saved_file_are_independent(saved, expected):
    source = snapshot()
    source["runtime"]["local_recording"]["profile_saved"] = saved
    state = calibration_status(source, 100.)
    assert state.state == "valid" and state.has_profile and state.current_pose
    assert "已生效" in state.summary and expected in state.summary
    assert state.saved is (saved if type(saved) is bool else None)


@pytest.mark.parametrize("reason", ["calibration_epoch_changed", "calibration_session_mismatch"])
def test_invalid_reference_never_inherits_effective_label_from_existing_profile(reason):
    source = snapshot()
    source["pose"].update(pose_available=False, solver_success=False, reason=reason)
    state = calibration_status(source, 100.)
    assert state.state == "invalid" and state.has_profile and not state.current_pose
    assert "已失效" in state.summary and "已保存" in state.summary and "已生效" not in state.summary
    assert "已失效" in workflow_status_text(source, state)
    assert "已生效" not in workflow_status_text(source, state)


def test_partial_epoch_failure_is_not_reported_as_whole_hand_valid_or_permanent_disconnect():
    source = snapshot()
    source["pose"]["calibration_rejections"] = {
        "10": "filter_epoch_changed_recalibration_required",
        "2": "filter_epoch_changed_recalibration_required",
        "5": "missing_or_stale_node",
    }
    state = calibration_status(source, 100.)
    assert state.state == "partial" and state.invalid_nodes == ("2", "10")
    assert state.current_pose and "部分骨段标定失效" in state.summary
    assert "基础标定已生效" not in state.summary


@pytest.mark.parametrize("reason", ["missing_or_stale_root", "input_health_invalid", "process_unresponsive"])
def test_temporary_input_loss_preserves_parameters_without_calling_them_current(reason):
    source = snapshot()
    source["pose"].update(reason=reason, pose_available=False, solver_success=False)
    source["pose"]["calibration_rejections"] = {"11": "missing_or_stale_node"}
    state = calibration_status(source, 100.)
    assert state.state == "waiting" and state.has_profile and state.invalid_nodes == ()
    assert "等待有效输入" in state.summary and "已保存" in state.summary
    assert "已失效" not in state.summary and "已生效" not in state.summary


@pytest.mark.parametrize("field,value", [("timestamp_s", 99.), ("timestamp_s", 101.),
                                        ("timestamp_s", float("nan")), ("timestamp_s", True)])
def test_old_future_or_invalid_pose_timestamp_never_establishes_current_validity(field, value):
    source = snapshot()
    source["pose"][field] = value
    assert calibration_status(source, 100.).state == "waiting"


@pytest.mark.parametrize("field,value", [("published_monotonic_s", 99.),
                                        ("last_sample_age_s", 1.), ("last_sample_age_s", None)])
def test_acquisition_and_publication_freshness_are_required_as_well_as_pose_time(field, value):
    source = snapshot()
    source["runtime"][field] = value
    assert calibration_status(source, 100.).state == "waiting"


def test_pose_from_another_session_cannot_validate_or_invalidate_new_session_profile():
    source = snapshot()
    source["pose"].update(session_id="previous", reason="calibration_epoch_changed")
    source["pose"]["calibration_rejections"] = {"11": "filter_epoch_changed_recalibration_required"}
    state = calibration_status(source, 100.)
    assert state.state == "waiting" and not state.current_pose and not state.invalid_nodes


def test_disconnect_retains_file_fact_without_claiming_live_output():
    source = snapshot()
    source["runtime"]["connected"] = False
    state = calibration_status(source, 100.)
    assert state.state == "retained" and "已保留" in state.summary and "已保存" in state.summary
    assert not state.current_pose and "已生效" not in state.summary


def test_uncalibrated_nodes_are_distinguished_from_epoch_invalid_nodes():
    source = snapshot()
    source["pose"]["calibration_rejections"] = {"2": "node_not_calibrated"}
    state = calibration_status(source, 100.)
    assert state.state == "partial" and not state.invalid_nodes
    assert "部分骨段未完成标定" in state.summary


@pytest.mark.parametrize("profile_status,active,expected", [
    ("clearing", None, "clearing"),
    ("fitting", None, "fitting"),
    ("fitting_functional_hand", {"kind": "functional_hand"}, "fitting"),
    ("collecting", {"step": "neutral"}, "collecting"),
])
def test_operation_in_progress_has_priority_over_old_result(profile_status, active, expected):
    source = snapshot()
    source["calibration"].update(profile_status=profile_status, active=active)
    state = calibration_status(source, 100.)
    assert state.state == expected
    assert "已生效" not in state.summary and "已保存" not in state.summary


def test_absent_profile_and_fit_failure_do_not_reuse_old_pose_or_saved_flag():
    source = snapshot()
    source["calibration"].update(calibrated_nodes=[], captured_steps=[], profile_status="not_calibrated")
    state = calibration_status(source, 100.)
    assert state.state == "absent" and not state.has_profile and not state.current_pose
    assert "已保存" not in state.summary
    assert "已采集 0/3 步" in workflow_status_text(source, state)
    source["calibration"].update(profile_status="fit_failed", captured_steps=["neutral", "fingers_down", "thumb_up"])
    state = calibration_status(source, 100.)
    assert state.state == "failed" and "拟合未通过" in workflow_status_text(source, state)
    assert not state.current_pose


def test_explicit_saved_override_supports_context_owned_cache_without_mutating_snapshot():
    source = snapshot()
    before = deepcopy(source)
    assert calibration_status(source, 100., profile_saved=False).saved is False
    assert calibration_status(source, 100., profile_saved=None).saved is None
    assert calibration_status(source, 100.).saved is True
    assert source == before
    with pytest.raises(FrozenInstanceError):
        calibration_status(source, 100.).state = "invalid"


def test_workflow_keeps_input_scale_and_functional_progress_separate_from_mounting():
    source = snapshot()
    source["calibration"].update(functional_enabled=False,
        input_scale_check={"status": "completed", "inconclusive_nodes": ["2", "3"]})
    text = workflow_status_text(source, now=100.)
    assert "基础标定：已生效" in text
    assert "已有 1/5 指参数 · 当前关闭" in text
    assert "2、3号待核对" in text


def failing_storage():
    source = snapshot()
    source["runtime"]["calibration_storage"] = {
        "error": "profile.json: permission denied", "retry_available": True,
        "source_session_id": "current", "warning": "标定参数仍保留在内存。",
    }
    source["calibration"]["shape_calibration"] = {
        "storage_error": "touch.json: no space", "can_retry_save": True,
        "storage_message": "触指数据已保留在内存。",
    }
    return source


def test_storage_aggregates_both_failure_sources_with_no_priority_loss():
    source = failing_storage()
    before = deepcopy(source)
    status = storage_status(source)
    assert [item.source for item in status.issues] == ["calibration_files", "touch_task"]
    assert "标定文件保存失败：profile.json" in status.error_text
    assert "触指任务保存失败：touch.json" in status.error_text
    assert status.has_errors and status.has_notices and status.can_retry
    assert len(status.messages) == 2 and "触指数据" in status.message
    assert source == before
    with pytest.raises(FrozenInstanceError):
        status.can_retry = False


@pytest.mark.parametrize("first", ["files", "touch"])
def test_partial_save_recovery_keeps_other_failure_and_retry(first):
    source = failing_storage()
    if first == "files":
        source["runtime"]["calibration_storage"].update(error=None, retry_available=False)
        remaining = "touch_task"
    else:
        source["calibration"]["shape_calibration"].update(storage_error=None, can_retry_save=False)
        remaining = "calibration_files"
    status = storage_status(source)
    assert [item.source for item in status.issues] == [remaining]
    assert save_actions_allowed(source, status)
    source["runtime"]["calibration_storage"].update(error=None, retry_available=False)
    source["calibration"]["shape_calibration"].update(storage_error=None, can_retry_save=False)
    recovered = storage_status(source)
    assert not recovered.has_errors and not save_actions_allowed(source, recovered)


def test_touch_only_save_failure_is_global_and_does_not_claim_profile_was_unsaved():
    source = failing_storage()
    source["runtime"].pop("calibration_storage")
    storage = storage_status(source)
    assert storage.has_errors and storage.can_retry
    assert [issue.source for issue in storage.issues] == ["touch_task"]
    assert calibration_status(source, 100.).saved is True


def test_evicted_history_notice_survives_current_error_recovery_without_inventing_retry():
    source = snapshot()
    source["runtime"]["calibration_storage"] = {"error": None, "retry_available": False, "evicted_count": 2}
    storage = storage_status(source)
    assert not storage.has_errors and storage.has_notices
    assert "2 份" in storage.notice_text and not save_actions_allowed(source, storage)


@pytest.mark.parametrize("change", ["capture", "fitting", "fitting_hand", "clearing", "shape_capture", "shape_optimize", "shape_busy"])
def test_all_save_entry_points_share_busy_gates(change):
    source = failing_storage()
    calibration = source["calibration"]
    if change == "capture":
        calibration["active"] = {"step": "neutral"}
    elif change in {"fitting", "fitting_hand", "clearing"}:
        calibration["profile_status"] = "fitting_functional_hand" if change == "fitting_hand" else change
    elif change in {"shape_capture", "shape_optimize"}:
        calibration["shape_calibration"]["status"] = "capturing" if change == "shape_capture" else "optimizing"
    else:
        calibration["shape_calibration"]["busy"] = True
    assert not save_actions_allowed(source, storage_status(source))


@pytest.mark.parametrize("kwargs", [{"pending": True}, {"clearing": True}, {"official": False}])
def test_pending_or_local_clear_blocks_retry_even_before_backend_acknowledgement(kwargs):
    source = failing_storage()
    assert not save_actions_allowed(source, storage_status(source), **kwargs)


def test_disconnected_retained_save_owner_can_retry_without_a_new_connection():
    source = failing_storage()
    source["runtime"]["connected"] = False
    source["runtime"]["calibration_storage"].update(detached=True, source_session_id="previous")
    assert save_actions_allowed(source, storage_status(source))


def test_shape_busy_and_empty_snapshot_are_pure_and_conservative():
    assert shape_busy({"shape_calibration": {"status": "optimizing"}})
    assert not shape_busy({})
    assert calibration_status({}).state == "absent"
    assert not storage_status({}).has_errors
    assert not save_actions_allowed({}, storage_status({}))


def test_presentation_import_does_not_load_gui_solver_or_hardware():
    module = Path(importlib.util.find_spec("vt_fsglove12.calibration_presentation").origin)
    script = f"""
import importlib.abc
import importlib.util
import sys
class BlockRuntime(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {{'PyQt5', 'PySide6', 'pyqtgraph', 'serial', 'numpy', 'scipy', 'vqf'}}:
            raise AssertionError(fullname)
sys.meta_path.insert(0, BlockRuntime())
spec = importlib.util.spec_from_file_location('presentation_only', {str(module)!r})
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
assert module.calibration_status({{}}).state == 'absent'
"""
    result = subprocess.run([sys.executable, "-I", "-c", script], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
