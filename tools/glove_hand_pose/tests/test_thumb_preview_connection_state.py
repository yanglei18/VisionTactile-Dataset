"""A lost display reference is not a disconnected or stale serial stream."""
from copy import deepcopy
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5 import QtWidgets

from vt_glove_pose.config import load_config
from vt_glove_pose.thumb_preview_gui import OrientationPairWidget
from vt_glove_pose.thumb_preview_live import LiveOrientationWindow, connection_status


def state(*, connected=False, fresh=False, reference=False, reasons=None):
    return dict(connected=connected, snapshot=dict(node_ids=[5, 6], paired_fresh=fresh,
        reference_ready=reference, valid=connected and fresh and reference, reasons=reasons or []),
        current=None, history=[], notice="旧提示：显示参考已设置。", error="")


@pytest.mark.parametrize("source,mode,text", [
    (state(), "disconnected", "串口未连接"),
    (state(connected=True), "waiting_for_data", "正在等待"),
    (state(connected=True, fresh=True, reasons=["sample_gap"]), "reference_required", "数据正常"),
    (state(connected=True, fresh=True, reference=True), "streaming", "正在显示两路独立朝向")])
def test_connection_data_reference_are_four_separate_states(source, mode, text):
    before = deepcopy(source)
    output = connection_status(source)
    assert output["mode"] == mode and text in output["status"]
    assert source == before


def test_fresh_data_with_lost_reference_never_claims_disconnected_or_stale():
    output = connection_status(state(connected=True, fresh=True, reasons=["sample_gap"]))
    assert "显示参考已失效" in output["status"]
    assert "无需重新连接" in output["status"]
    assert "显示参考已设置" not in output["status"]
    assert "未连接" not in output["empty_message"]
    assert "过期" not in output["empty_message"]


def test_initial_reference_and_invalidated_reference_have_distinct_explanations():
    initial = connection_status(state(connected=True, fresh=True, reasons=["display_reference_not_set"]))
    assert "尚未设置显示参考" in initial["status"]
    assert "已失效" not in initial["status"]


@pytest.mark.parametrize("source", [state(), state(connected=True),
    state(connected=True, fresh=True), state(connected=True, fresh=True, reference=True)])
def test_error_remains_visible_without_overwriting_current_connection_facts(source):
    source["error"] = "测试保存失败：磁盘不可写"
    result = connection_status(source)
    assert source["error"] in result["status"]
    assert "串口" in result["status"]


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakeWorker:
    def __init__(self, payload):
        self.payload = payload
        self.commands = []

    def start(self):
        pass

    def stop(self):
        pass

    def wait(self, timeout):
        return True

    def latest_snapshot(self):
        return deepcopy(self.payload)

    def submit(self, name, **kwargs):
        self.commands.append((name, kwargs))


def test_window_preserves_completed_report_but_explains_missing_reference(app, tmp_path):
    payload = state(connected=True, fresh=True, reasons=["sample_gap"])
    payload["last_trial"] = dict(status="completed", export_complete=True,
                                 summary="已完成测试的测量结果", directory=str(tmp_path))
    worker = FakeWorker(payload)
    window = LiveOrientationWindow(load_config(), worker=worker, output_root=tmp_path)
    try:
        window.refresh()
        assert window.connect_button.text() == "断开"
        assert window.reference_button.isEnabled()
        assert not window.start_trial_button.isEnabled()
        assert "数据正常" in window.status.text()
        assert "显示参考已失效" in window.status.text()
        assert "已完成" in window.guide.text()
        assert "已完成测试的测量结果" in window.trial_result.text()
        assert window.orientation.quaternions == [None, None]
        assert "实时数据正常" in window.orientation.empty_message
        assert "未连接" not in window.orientation.empty_message
        assert worker.commands == []  # Refresh never reconnects or resets reference.
        window.reference_button.click()
        assert worker.commands == [("reference", {})]
    finally:
        window.close()


def test_canvas_empty_reason_does_not_create_or_reset_a_pose(app):
    canvas = OrientationPairWidget()
    assert "未连接或数据已过期" in canvas.empty_message  # Legacy replay default retained.
    canvas.set_empty_message("数据正常，请设置参考")
    assert canvas.quaternions == [None, None]
    canvas.set_orientations([1., 0., 0., 0.], [1., 0., 0., 0.])
    canvas.set_empty_message("另一条说明")
    assert all(q[0] == 1. for q in canvas.quaternions)
    canvas.set_sample(None)
    assert canvas.quaternions == [None, None]
    canvas.resize(800, 350)
    assert not canvas.grab().isNull()
    canvas.close()
