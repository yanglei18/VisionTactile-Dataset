"""Calibration workspace regressions using snapshots, without devices or GL paint."""
from copy import deepcopy

import numpy as np
import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("pyqtgraph")
pytest.importorskip("OpenGL")
from PyQt5 import QtWidgets

from test_live_ui import Backend, Controller, make_measured
from test_live_ui_local_recording import WARNING, paused
from vt_fsglove12 import live_ui
from vt_fsglove12.config import load_config
from vt_fsglove12.ui_sections import CollapsibleSection


@pytest.fixture
def workspace(monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(live_ui, "list_serial_ports", lambda: [])
    clock = [100.]
    monkeypatch.setattr(live_ui.time, "monotonic", lambda: clock[0])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config = load_config()
    config["node_to_slot"] = {
        "1": 15, "2": 14, "3": 2, "4": 1, "5": 5, "6": 4,
        "7": 11, "8": 10, "9": 8, "10": 7, "11": 0,
    }
    config.setdefault("input_health", {})["enabled"] = True
    controller, backend = Controller(), Backend()
    backend.render_mano = lambda *_: backend.render(None)
    window = live_ui.LiveWindow(config, backend, output_root=tmp_path,
        controller=controller, upstream_checkout="explicit_mock_checkout")
    window.timer.stop()
    try:
        yield window, controller, clock
    finally:
        window.close()
        app.processEvents()


def receive_required_nodes(window, controller, now):
    """Eleven measured bones are required; node 12 is an optional auxiliary."""
    make_measured(controller)
    controller.state["calibration"].update(
        calibrated_nodes=list(window.config["node_to_slot"]),
        captured_steps=["neutral", "fingers_down", "thumb_up"],
        profile_status="fitted")
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["orientation"] = {
        "timestamp_s": now,
        "nodes": {node: {"age_s": .001, "sample_index": 5, "epoch": 0,
                         "orientation_valid": True}
                  for node in window.config["node_to_slot"]},
        "input_health": {
            "enabled": True,
            "nodes": {node: {"state": "healthy", "healthy": True}
                      for node in window.config["node_to_slot"]},
            "faulted_nodes": [], "recovering_nodes": [],
        },
    }


def assert_global(widget, window):
    assert not window.workspace_tabs.isAncestorOf(widget)
    assert widget.isVisibleTo(window.centralWidget())


def test_workspace_keeps_action_guidance_and_hand_outside_scrolling_pages(workspace):
    window, controller, _clock = workspace
    tabs = window.workspace_tabs
    assert tabs.objectName() == "calibrationTabs"
    assert [tabs.tabText(i) for i in range(tabs.count())] == ["姿态标定", "个性化手形", "诊断"]
    assert tabs.currentWidget() is window.sidebar_scroll
    assert window.shape_scroll.isAncestorOf(window.shape_panel)
    for widget in (window.functional_enabled, window.details, window.stats, window.joint_status):
        assert window.diagnostics_scroll.isAncestorOf(widget)
        assert not window.sidebar_scroll.isAncestorOf(widget)
    for widget in (window.view, window.instructions, window.progress, window.cancel_capture):
        assert_global(widget, window)
    assert controller.commands == []


def test_page_navigation_and_disclosures_do_not_change_commands_session_or_hand(workspace):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    window.refresh()
    state = deepcopy(controller.state)
    mesh = window.view.mesh
    vertices = window.view.mesh_data.vertexes().copy()
    camera = deepcopy(window.view.opts)
    sections = window.findChildren(CollapsibleSection)
    assert window.single_finger_section in sections
    assert not window.single_finger_section.isExpanded()
    assert window.single_finger_section.content.isAncestorOf(window.functional_selector)
    assert window.single_finger_section.content.isAncestorOf(window.functional_capture)
    for index in range(window.workspace_tabs.count()):
        window.workspace_tabs.setCurrentIndex(index)
        for section in sections:
            initial = section.isExpanded()
            section.button.click()
            assert section.isExpanded() is not initial
            section.button.click()
            assert section.isExpanded() is initial
        window.refresh()
        assert_global(window.view, window)
        assert window.view.mesh is mesh and mesh.visible()
        np.testing.assert_array_equal(window.view.mesh_data.vertexes(), vertices)
        assert window.view.opts == camera
    assert controller.commands == []
    assert controller.state == state


def test_page_change_refreshes_shape_guidance_without_starting_capture(workspace):
    window, controller, clock = workspace
    window.refresh()
    window.workspace_tabs.setCurrentWidget(window.shape_scroll)
    assert "请先" in window.instructions.text() and "基础标定" in window.instructions.text()
    assert window.capture_controls.isHidden()
    assert not window.shape_view_status.isHidden()
    window.workspace_tabs.setCurrentWidget(window.sidebar_scroll)
    receive_required_nodes(window, controller, clock[0])
    window.refresh()
    base_guidance = window.instructions.text()
    before = deepcopy(controller.state)
    assert not window.capture_controls.isHidden()
    assert window.shape_view_status.isHidden()
    # No explicit refresh: changing the tab must update its guidance immediately.
    window.workspace_tabs.setCurrentWidget(window.shape_scroll)
    assert "采集触指姿态" in window.instructions.text()
    assert "优化、预览并采用" in window.instructions.text()
    assert window.capture_controls.isHidden()
    assert not window.progress.isVisibleTo(window.centralWidget())
    assert not window.cancel_capture.isVisibleTo(window.centralWidget())
    assert not window.shape_view_status.isHidden()
    window.workspace_tabs.setCurrentWidget(window.sidebar_scroll)
    assert window.instructions.text() == base_guidance
    assert not window.capture_controls.isHidden()
    assert window.shape_view_status.isHidden()
    assert controller.commands == [] and controller.state == before


def test_switching_to_shape_during_base_capture_keeps_guidance_and_cancel(workspace):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    controller.state["calibration"]["active"] = {
        "kind": "pose", "step": "neutral", "phase": "preparing", "remaining_s": 2.,
    }
    window.refresh()
    guidance = window.capture_guidance.text()
    progress = window.progress.format()
    before = deepcopy(controller.state)
    window.workspace_tabs.setCurrentWidget(window.shape_scroll)
    assert window.capture_guidance.text() == guidance
    assert window.progress.format() == progress
    assert_global(window.capture_guidance, window)
    assert_global(window.capture_controls, window)
    assert_global(window.cancel_capture, window)
    assert window.cancel_capture.isEnabled()
    assert controller.commands == [] and controller.state == before


def test_no_tactile_source_stays_hidden_while_pose_controls_remain_available(workspace):
    window, controller, clock = workspace
    assert window._tactile_socket is None
    for connected in (False, True):
        if connected:
            receive_required_nodes(window, controller, clock[0])
        window.refresh()
        assert window.tactile_section.isHidden()
        assert not window.tactile_enabled.isVisibleTo(window.centralWidget())
        assert not window.tactile_reference.isVisibleTo(window.centralWidget())
    assert window.step_buttons["neutral"].isEnabled()
    assert controller.commands == []


def test_auxiliary_node_missing_does_not_block_required_channels(workspace):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    window.refresh()
    assert "12" not in controller.state["orientation"]["nodes"]
    assert "11/11" in window.device_summary.text()
    assert "可采集" in window.device_summary.text()
    assert window.device_summary.property("statusState") == "success"
    assert all(button.isEnabled() for button in window.step_buttons.values())
    assert controller.commands == []


@pytest.mark.parametrize("evidence", ["unhealthy", "missing"])
def test_received_required_channels_need_health_evidence_before_claiming_ready(workspace, evidence):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    frame = controller.state["orientation"]
    frame["nodes"]["12"] = {"age_s": .001, "sample_index": 5, "epoch": 0}
    if evidence == "unhealthy":
        frame["input_health"]["nodes"]["3"] = {"state": "recovering", "healthy": False}
        frame["input_health"]["recovering_nodes"] = ["3"]
    else:
        frame["input_health"]["nodes"].pop("3")
    before = deepcopy(controller.state)
    window.refresh()
    assert len(frame["nodes"]) == 12
    assert "11/11" in window.device_summary.text()
    assert "可采集" not in window.device_summary.text()
    assert window.device_summary.property("statusState") == "warning"
    assert not any(button.isEnabled() for button in window.step_buttons.values())
    assert controller.commands == [] and controller.state == before


def test_stale_snapshot_does_not_claim_required_channels_ready(workspace):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    window.refresh()
    assert "可采集" in window.device_summary.text()
    clock[0] += 1.
    window.refresh()
    assert "可采集" not in window.device_summary.text()
    assert window.device_summary.property("statusState") == "warning"
    assert controller.commands == []


@pytest.mark.parametrize(("saved", "expected"), [(False, "未保存"), (True, "已保存"), (None, "保存状态待确认")])
def test_applied_calibration_and_saved_file_are_reported_separately(workspace, saved, expected):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    controller.state["runtime"]["local_recording"] = {
        "enabled": True, "warning": "", "errors": [], "profile_saved": saved,
    }
    window.refresh()
    assert "已生效" in window.result_status.text()
    assert expected in window.result_status.text()
    assert window.view.mesh.visible()
    assert controller.commands == []


def test_unsaved_result_and_recording_failure_remain_visible_on_every_page(workspace):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    recording = paused()
    recording["profile_saved"] = False
    controller.state["runtime"]["local_recording"] = recording
    controller.state["runtime"]["calibration_storage"] = {
        "error": "mock directory unavailable", "retry_available": True,
    }
    window.refresh()
    mesh = window.view.mesh
    vertices = window.view.mesh_data.vertexes().copy()
    for index in range(window.workspace_tabs.count()):
        window.workspace_tabs.setCurrentIndex(index)
        window.refresh()
        assert WARNING in window.local_recording_status.text()
        assert "当前标定未保存到本地文件" in window.local_recording_status.text()
        assert "已生效" in window.result_status.text() and "未保存" in window.result_status.text()
        assert "mock directory unavailable" in window.storage_error_status.text()
        for widget in (window.local_recording_status, window.storage_error_status, window.result_status):
            assert_global(widget, window)
        assert window.view.mesh is mesh and mesh.visible()
        np.testing.assert_array_equal(window.view.mesh_data.vertexes(), vertices)
    # Sparse IPC updates must not make a known save failure appear resolved.
    controller.state["runtime"].pop("local_recording")
    window.refresh()
    assert "未保存" in window.result_status.text()
    assert WARNING in window.local_recording_status.text()
    assert controller.commands == []


@pytest.mark.parametrize("new_context", ["session", "profile"])
def test_confirmed_save_is_not_inherited_by_new_session_or_profile(workspace, new_context):
    window, controller, clock = workspace
    receive_required_nodes(window, controller, clock[0])
    controller.state["pose"]["profile_version"] = 7
    controller.state["runtime"]["local_recording"] = {
        "enabled": True, "warning": "", "errors": [], "profile_saved": True,
    }
    window.refresh()
    assert "已生效" in window.result_status.text() and "已保存" in window.result_status.text()
    controller.state["runtime"].pop("local_recording")
    window.refresh()
    assert "已保存" in window.result_status.text()  # Sparse update of the same result.
    if new_context == "session":
        controller.state["runtime"]["session_id"] = "new-explicit-connection"
    else:
        controller.state["pose"]["profile_version"] = 8
    before = deepcopy(controller.state)
    window.refresh()
    assert "已生效" in window.result_status.text()
    assert "保存状态待确认" in window.result_status.text()
    assert "已保存" not in window.result_status.text()
    assert controller.commands == [] and controller.state == before
