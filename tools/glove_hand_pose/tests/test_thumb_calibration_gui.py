"""Joint-thumb controls tested without sources, filters, files, or hardware."""

from copy import deepcopy
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5")
from PyQt5 import QtCore, QtWidgets

from vt_glove_pose import gui


@pytest.fixture(scope="module")
def app():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["thumb-calibration-tests"])
    application.setQuitOnLastWindowClosed(False)
    return application


class SnapshotWorker:
    def __init__(self, clock):
        self.snapshot = {
            "runtime": {"connected": True, "running": True, "error": "", "message": "采集中",
                        "performance": {"snapshot_published_monotonic_s": clock[0]}},
            "calibration": {"active": None, "neutral_ready": True, "message": ""},
            "model": {"fingers": {}},
            "nodes": {str(node): {"fresh": True, "age_s": 0.0} for node in range(1, 13)},
        }
        self.commands = []
        self.accept = True
        self.started = self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def wait(self, timeout):
        return True

    def latest_snapshot(self):
        return deepcopy(self.snapshot)

    def submit(self, command, **arguments):
        if not self.accept:
            return False
        self.commands.append((command, arguments))
        return True


@pytest.fixture
def setup(app, monkeypatch):
    clock = [1000.0]
    monkeypatch.setattr(gui, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    monkeypatch.setattr(gui, "list_serial_ports", lambda: [])
    worker = SnapshotWorker(clock)
    window = gui.HandPoseWindow({"stale_after_s": .5}, worker=worker)
    # Tests advance the same timeout handler explicitly; no wall-clock sleeps.
    window.timer.stop()
    yield window, worker, clock
    window.close()
    app.processEvents()


def tick(window, worker, clock, seconds, *, update_snapshot=True):
    clock[0] += seconds
    if update_snapshot:
        worker.snapshot["runtime"]["performance"]["snapshot_published_monotonic_s"] = clock[0]
    window._advance_thumb_preparation()


def ready_for_b(worker):
    worker.snapshot["calibration"]["quality_feedback"] = dict(
        stage="A", status="ready_for_b", message="A 成功，请手动开始 B。", paired_samples=810)


def test_clear_new_entries_old_thumb_label_and_manual_initial_state(setup):
    window, worker, _ = setup
    assert window.axis_combo.itemData(0) == "thumb_common"
    assert window.axis_combo.itemData(1) == "thumb_hinge"
    assert "旧版" in window.axis_combo.itemText(window.axis_combo.findData("thumb"))
    assert window.thumb_common_button.isEnabled()
    assert not window.thumb_hinge_button.isEnabled()
    assert window._thumb_preparation is None
    assert not window._thumb_prepare_timer.isActive()
    assert worker.commands == []
    assert window.styleSheet() == ""


@pytest.mark.parametrize("target", ["thumb_common", "thumb_hinge"])
@pytest.mark.parametrize("entry", ["dedicated", "old_bridge"])
def test_joint_actions_prepare_3_2_1_then_submit_one_robust_capture(setup, target, entry):
    window, worker, clock = setup
    if target == "thumb_hinge":
        ready_for_b(worker)
    window.axis_combo.setCurrentIndex(window.axis_combo.findData(target))
    window._refresh_snapshot()
    button = (window.axis_button if entry == "old_bridge" else
              window.thumb_common_button if target == "thumb_common" else window.thumb_hinge_button)
    button.click()
    assert worker.commands == []
    assert "准备 3 秒" in window.thumb_capture_label.text()
    assert "尚未采集标定动作" in window.thumb_capture_label.text()
    assert window._thumb_prepare_timer.isActive()
    if target == "thumb_hinge":
        assert "扶稳近节" in window.thumb_capture_label.text()
        assert "先弯再伸" in window.thumb_capture_label.text()
    for other in (window.axis_button, window.thumb_common_button, window.thumb_hinge_button,
                  window.neutral_button, window.load_button, window.save_button, window.axis_combo):
        assert not other.isEnabled()
    assert window.cancel_button.isEnabled() and window.disconnect_button.isEnabled()
    window._request_axis(target)  # Direct callback, bypassing Qt disabled state.
    tick(window, worker, clock, 1)
    assert "准备 2 秒" in window.thumb_capture_label.text()
    tick(window, worker, clock, 1)
    assert "准备 1 秒" in window.thumb_capture_label.text()
    assert worker.commands == []
    tick(window, worker, clock, 1)
    expected = [("axis", dict(target=target, calibration_method="robust_v2", _gui_request_id=1))]
    assert worker.commands == expected
    assert window._thumb_preparation is None
    assert not window._thumb_prepare_timer.isActive()
    assert "等待后台确认" in window.thumb_capture_label.text()
    window._request_axis(target)
    window._advance_thumb_preparation()
    assert worker.commands == expected


@pytest.mark.parametrize("target", ["palm", "thumb", "index", "middle", "ring", "little"])
def test_other_axes_preserve_immediate_historical_command_shape(setup, target):
    window, worker, _ = setup
    window.axis_combo.setCurrentIndex(window.axis_combo.findData(target))
    window.axis_button.click()
    assert worker.commands == [("axis", {"target": target})]
    assert window._thumb_preparation is None
    assert not window._thumb_prepare_timer.isActive()


@pytest.mark.parametrize("command", ["cancel", "disconnect", "neutral", "load", "set_hand"])
def test_state_changing_commands_cancel_pending_timer(setup, command):
    window, worker, clock = setup
    window.thumb_common_button.click()
    window._submit(command)
    tick(window, worker, clock, 4)
    assert worker.commands == [(command, {})]
    assert window._thumb_preparation is None
    assert not window._thumb_prepare_timer.isActive()


def test_close_cancels_preparation_without_issuing_axis(setup):
    window, worker, clock = setup
    window.thumb_common_button.click()
    window.close()
    tick(window, worker, clock, 4)
    window._request_axis("thumb_common")
    assert worker.stopped
    assert worker.commands == []
    assert window._thumb_preparation is None
    assert not window._thumb_prepare_timer.isActive()


@pytest.mark.parametrize("failure", ["disconnect", "stopped", "startup", "eof", "neutral", "active", "missing", "stale"])
@pytest.mark.parametrize("at_start", [False, True])
def test_fresh_state_is_required_at_start_and_rechecked_before_capture(setup, failure, at_start):
    window, worker, clock = setup
    if not at_start:
        window.thumb_common_button.click()
    if failure == "disconnect":
        worker.snapshot["runtime"]["connected"] = False
    elif failure == "stopped":
        worker.snapshot["runtime"]["running"] = False
    elif failure == "startup":
        worker.snapshot["runtime"].pop("running")
    elif failure == "eof":
        worker.snapshot["runtime"]["eof"] = True
    elif failure == "neutral":
        worker.snapshot["calibration"]["neutral_ready"] = False
    elif failure == "active":
        worker.snapshot["calibration"]["active"] = {"kind": "neutral"}
    elif failure == "missing":
        worker.snapshot["nodes"].pop("11")
    else:
        worker.snapshot["nodes"]["6"]["fresh"] = False
    if at_start:
        window._request_axis("thumb_common")
    else:
        tick(window, worker, clock, 3)
    assert worker.commands == []
    assert window._thumb_preparation is None
    assert not window._thumb_prepare_timer.isActive()
    assert window._ui_notice


def test_stalled_mailbox_cannot_claim_nodes_still_fresh(setup):
    window, worker, clock = setup
    window.thumb_common_button.click()
    tick(window, worker, clock, 3, update_snapshot=False)
    assert worker.commands == []
    assert "快照已过期" in window.status_label.text()
    assert window._thumb_preparation is None


def test_snapshot_refresh_cancels_immediately_on_disconnect(setup):
    window, worker, _ = setup
    window.thumb_common_button.click()
    worker.snapshot["runtime"]["connected"] = False
    window._refresh_snapshot()
    assert window._thumb_preparation is None
    assert "准备已取消" in window.status_label.text()
    assert worker.commands == []


def test_b_requires_a_and_success_never_automatically_begins_b(setup):
    window, worker, clock = setup
    window._request_axis("thumb_hinge")
    assert "先完成本版拇指 A" in window.status_label.text()
    assert worker.commands == []
    window.thumb_common_button.click()
    tick(window, worker, clock, 3)
    ready_for_b(worker)
    worker.snapshot["runtime"]["last_axis_request_id"] = 1
    clock[0] += .1
    worker.snapshot["runtime"]["performance"]["snapshot_published_monotonic_s"] = clock[0]
    window._refresh_snapshot()
    assert window.thumb_hinge_button.isEnabled()
    assert "A 成功" in window.thumb_capture_label.text()
    assert window._thumb_preparation is None
    tick(window, worker, clock, 20)
    assert worker.commands == [("axis", {"target": "thumb_common", "calibration_method": "robust_v2", "_gui_request_id": 1})]


def test_queue_full_does_not_leave_countdown_or_claim_capture_started(setup):
    window, worker, clock = setup
    window.thumb_common_button.click()
    worker.accept = False
    tick(window, worker, clock, 3)
    assert worker.commands == []
    assert window._thumb_preparation is None and window._thumb_queued_at is None
    assert "操作队列已满" in window.status_label.text()
    assert "采集中" not in window.thumb_capture_label.text()


def test_live_feedback_is_displayed_verbatim_with_numbers_not_rejudged(setup):
    window, worker, _ = setup
    worker.snapshot["calibration"].update(
        active={"kind": "axis", "target": "thumb_common", "remaining_s": 6.4},
        quality_feedback={"stage": "A", "status": "needs_attention", "message": "慢动作质量提示；并非最终判定。",
                          "paired_samples": 123, "proximal_speed_p95_dps": 12.3,
                          "distal_speed_p95_dps": 10.1, "amplitude_ratio": .8211})
    window._refresh_snapshot()
    text = window.thumb_capture_label.text()
    assert "A 采集中" in text and "6.4 / 8 秒" in text
    assert "慢动作质量提示；并非最终判定。" in text
    assert "123" in text and "12.3 / 10.1" in text
    assert "0.8211" in window.thumb_capture_label.toolTip()
    assert not window.thumb_common_button.isEnabled()
    assert not window.thumb_hinge_button.isEnabled()
    assert worker.commands == []


def test_long_numeric_failure_is_visible_selectable_and_not_height_truncated(setup):
    window, worker, _ = setup
    reason = "A 失败：有效比例 0.612，旋转残差 4.32 °/s；" * 12
    worker.snapshot["calibration"]["message"] = reason
    window._refresh_snapshot()
    assert reason in window.calibration_label.text()
    assert reason in window.calibration_label.toolTip()
    assert "数值原因见下方" in window.thumb_capture_label.text()
    assert window.calibration_result_scroll.maximumHeight() <= 64
    assert window.calibration_label.textInteractionFlags() & QtCore.Qt.TextSelectableByMouse


def test_detailed_help_is_collapsed_by_default_and_only_changes_layout(setup):
    window, worker, _ = setup
    assert window.calibration_help.isHidden()
    assert not window.calibration_help_button.isChecked()
    assert window.calibration_help_button.arrowType() == QtCore.Qt.RightArrow
    window.calibration_help_button.click()
    assert not window.calibration_help.isHidden()
    assert window.calibration_help_button.arrowType() == QtCore.Qt.DownArrow
    text = " ".join(label.text() for label in window.calibration_help.findChildren(QtWidgets.QLabel))
    assert "四指 / 旧版独立轴" in text and "看到“采集中”后" in text
    window.calibration_help_button.click()
    assert window.calibration_help.isHidden()
    window.thumb_common_button.click()
    assert "准备 3 秒" in window.thumb_capture_label.text()
    assert not window.thumb_capture_label.isHidden()
    assert worker.commands == []


def test_long_result_scrolls_without_expanding_default_window(setup, app):
    window, worker, _ = setup
    reason = "A 失败：有效配对 432/810，幅度比 1.627；完整数值原因可滚动及复制。" * 40
    worker.snapshot["calibration"]["message"] = reason
    window._refresh_snapshot()
    window.show()
    window.resize(1440, 1000)
    app.processEvents()
    assert window.height() <= 1000
    assert window.canvas.height() >= 380
    assert window.calibration_result_scroll.height() <= 64
    bar = window.calibration_result_scroll.verticalScrollBar()
    assert bar.maximum() > 0
    bar.setValue(bar.maximum())
    assert bar.value() == bar.maximum()
    assert reason in window.calibration_label.text()
    assert window.calibration_result_scroll.widget() is window.calibration_label


@pytest.mark.parametrize("method", [None, "strict_v1", "robust_v2"])
def test_worker_forwards_and_records_exact_requested_axis_method(method):
    calls, events = [], []
    worker = gui.PoseWorker({})
    worker._source = object()  # Presence sentinel, never opened/read.
    worker._engine = SimpleNamespace(begin_axis=lambda **args: calls.append(args), message="A 采集中")
    worker._emit_diagnostic = lambda event, **data: events.append((event, data))
    arguments = {"target": "thumb_common"}
    if method is not None:
        arguments["calibration_method"] = method
    worker._command("axis", dict(arguments, _gui_request_id=12))
    expected = dict(arguments, duration=8)
    if method == "strict_v1":
        expected.pop("calibration_method")
    assert calls == [expected]
    assert events == [("command", dict(name="axis", args=expected))]
    assert worker._last_axis_request_id == 12


def test_worker_does_not_record_rejected_axis_as_successful_command():
    def fail(**args):
        raise ValueError("A/B 方法不一致")

    events = []
    worker = gui.PoseWorker({})
    worker._source = object()
    worker._engine = SimpleNamespace(begin_axis=fail)
    worker._emit_diagnostic = lambda *args, **kwargs: events.append((args, kwargs))
    with pytest.raises(ValueError, match="方法不一致"):
        worker._command("axis", {"target": "thumb_hinge", "calibration_method": "robust_v2", "_gui_request_id": 7})
    assert events == []
    assert worker._last_axis_request_id == 7


def test_late_publication_with_old_error_does_not_acknowledge_a_new_request(setup):
    window, worker, clock = setup
    worker.snapshot["runtime"]["error"] = "上一操作的旧错误"
    window.thumb_common_button.click()
    tick(window, worker, clock, 3)
    clock[0] += .1
    worker.snapshot["runtime"]["performance"]["snapshot_published_monotonic_s"] = clock[0]
    window._refresh_snapshot()
    window._request_axis("thumb_common")
    assert len(worker.commands) == 1
    assert window._thumb_queued_at is not None
    assert not window.thumb_common_button.isEnabled()
    worker.snapshot["runtime"]["last_axis_request_id"] = 1
    window._refresh_snapshot()
    assert window._thumb_queued_at is None
    assert window.thumb_common_button.isEnabled()
