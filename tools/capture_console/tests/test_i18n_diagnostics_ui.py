"""Diagnostic rendering changes no payload, user label, log, or device state."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from PySide6.QtCore import QObject, Signal

from vt_capture_console import i18n


@pytest.fixture(autouse=True)
def english_display():
    previous = i18n.language()
    i18n.set_language("en", persist=False)
    yield
    i18n.set_language(previous, persist=False)


def test_device_details_receipts_and_builtin_labels_preserve_source(qapp):
    from vt_capture_console.ui.widgets.device_controls import DeviceControls
    group = dict(id="cameras", kind="camera", label="三相机与录制控制器",
                 state="FAILED", detail="后台响应超时", owned=False, pid=None,
                 can_start=False, can_stop=False, can_restart=False,
                 log_path="/用户/后台响应超时.log", topics=["/用户/相机/样本"])
    operation = dict(request_id="请求-1", state="rejected", code="TEST_CODE",
                     detail="后台响应超时")
    backend = SimpleNamespace(mode="live", supports_device_control=True,
                              latest_operation=operation)
    snapshot = dict(instance_id="manager-1", capabilities={"device_control": True},
                    process_control={"enabled": True, "groups": [group]})
    original = deepcopy(snapshot)
    controls = DeviceControls(backend)
    controls.pending_request, controls.pending_instance_id = "请求-1", "manager-1"
    controls.refresh(snapshot)
    assert "Backend response timeout" in controls.detail.text()
    assert "Backend response timeout" in controls.operation_notice.text()
    assert "TEST_CODE" in controls.operation_notice.text()
    assert controls.table.item(0, 0).text() == str(i18n.tr(group["label"]))
    assert controls.log_path.text() == group["log_path"]
    assert group["topics"][0] in controls.detail.toolTip()
    assert snapshot == original
    i18n.set_language("zh_CN", persist=False)
    assert "后台响应超时" in controls.detail.text()
    assert "后台响应超时" in controls.operation_notice.text()
    assert controls.table.item(0, 0).text() == group["label"]
    controls.close()


@pytest.mark.parametrize("group", [
    dict(id="custom-group", kind="camera", label="三相机与录制控制器"),
    dict(id="cameras", kind="camera", label="用户定制相机组"),
    dict(id="cameras", kind="tracker", label="三相机与录制控制器"),
])
def test_custom_group_names_are_never_translated(group):
    from vt_capture_console.ui.widgets.device_controls import _group_label
    assert _group_label(group) == group["label"]


def test_tracker_read_receipt_is_translated_without_changing_operation():
    from vt_capture_console.ui.desktop_tools import DesktopTools
    receipt = dict(request_id="request-1", state="rejected", code="TEST_CODE",
                   detail="后台响应超时")
    before = receipt.copy()
    feedback = []
    tools = SimpleNamespace(
        _tracker_read_request="request-1", _tracker_read_instance="manager-1",
        tracker_dialog=object(), _tracker_feedback=feedback.append,
        backend=SimpleNamespace(
            supports_device_control=True, latest_operation=receipt,
            snapshot=lambda: dict(instance_id="manager-1", mode="live")))
    DesktopTools._poll_tracker_reading(tools)
    assert "Backend response timeout" in str(feedback[0])
    assert "TEST_CODE" in str(feedback[0])
    assert receipt == before
    i18n.set_language("zh_CN", persist=False)
    assert "后台响应超时" in str(feedback[0])


def test_tracker_wizard_translates_instruction_but_keeps_logs(qapp):
    from vt_capture_console.ui.widgets.tracker_setup import TrackerSetupDialog
    dialog = TrackerSetupDialog(lambda: {"mode": "demo"})
    view = dict(state="FAILED", prompt="", detail="初始化未完成。",
                logs=["后台响应超时", "/用户/录制日志.txt"])
    before = deepcopy(view)
    dialog.workflow = SimpleNamespace(snapshot=lambda: view)
    dialog.refresh()
    assert dialog.instruction.text() == "Initialization incomplete."
    assert "A fresh snapshot from a real backend" in dialog.guard_label.text()
    assert dialog.log.toPlainText() == "\n".join(view["logs"])
    i18n.set_language("zh_CN", persist=False)
    assert dialog.instruction.text() == view["detail"]
    assert dialog.log.toPlainText() == "\n".join(view["logs"])
    assert view == before
    dialog.close()


def test_offline_task_translates_status_but_preserves_paths_and_logs(qapp, tmp_path):
    from vt_capture_console.ui.widgets.offline_alignment import OfflineAlignmentDialog
    dialog = OfflineAlignmentDialog(tmp_path)
    state = dict(state="FAILED", detail="工具执行失败（退出码 2）；请查看日志。",
                 output_path="", log="后台响应超时\n原始输出：用户文本")
    before = state.copy()
    dialog.task = SimpleNamespace(status=state, running=False)
    dialog.fields["bag"].setText("/用户/中文试次")
    dialog.refresh()
    assert dialog.status.text() == "Tool failed (exit code 2). Check the logs."
    assert dialog.log.toPlainText() == state["log"]
    assert dialog.fields["bag"].text() == "/用户/中文试次"
    i18n.set_language("zh_CN", persist=False)
    assert dialog.status.text() == state["detail"]
    assert state == before
    dialog.close()


def test_spatial_backend_status_and_reason_are_display_only(qapp):
    from vt_capture_console.backend.scene_demo import DemoSceneBackend
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    status = dict(state="error", message="场景后台响应超时")
    frame = deepcopy(backend.latest())
    frame["trackers"][0]["label"] = "用户实体名称"
    frame["trackers"][0]["reason"] = "后台响应超时"
    original = deepcopy(frame)
    page.apply_frame(frame, status)
    assert "Scene backend response timeout" in page.status.text()
    assert page.status.toolTip() == "Scene backend response timeout"
    page.select(frame["trackers"][0]["id"])
    assert "Backend response timeout" in page.detail.text()
    assert "用户实体名称" in page.detail.text()
    i18n.set_language("zh_CN", persist=False)
    assert "后台响应超时" in page.detail.text()
    assert "用户实体名称" in page.detail.text()
    assert frame == original
    assert status == dict(state="error", message="场景后台响应超时")
    page.close()
    backend.close()


def test_glove_diagnostic_keeps_device_hint_and_retranslates(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import glove_3d
    class Worker(QObject):
        initialized = Signal(object)
        ready = Signal(object)
        failed = Signal(str)
    worker = Worker()
    monkeypatch.setattr(glove_3d, "pose_preview", lambda hand: worker)
    widget = glove_3d.Glove3DWidget("right")
    device = {"glove_hint": "此手姿态模块未启用；请先在配置设备中启用。"}
    widget.set_sample(None, held=True, mode="live", device=device)
    assert widget.message.text() == str(i18n.tr(device["glove_hint"]))
    widget._failed("MANO 显示进程已退出；请重新加载模型")
    assert "MANO display process exited; reload the model" in widget.message.text()
    i18n.set_language("zh_CN", persist=False)
    assert "MANO 显示进程已退出；请重新加载模型" in widget.message.text()
    assert device["glove_hint"] == "此手姿态模块未启用；请先在配置设备中启用。"
    widget.dispose()


def test_tactile_diagnostic_tooltip_keeps_opaque_error_tail(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    class Loader(QObject):
        ready = Signal(str, object)
        failed = Signal(str, str)
        def request(self, hand):
            pass
        def get(self, hand):
            return None
        def error(self, hand):
            return ""
    loader = Loader()
    monkeypatch.setattr(tactile_3d, "model_loader", lambda: loader)
    widget = tactile_3d.Tactile3DWidget("right")
    raw = "MANO 显示进程无法运行：/用户/模型/诊断 {raw}"
    widget._model_failed("right", raw)
    assert widget.message.toolTip() == "MANO display process cannot run: /用户/模型/诊断 {raw}"
    assert "/用户/模型/诊断 {raw}" in widget.message.text()
    i18n.set_language("zh_CN", persist=False)
    assert widget.message.toolTip() == raw
    widget.dispose()
