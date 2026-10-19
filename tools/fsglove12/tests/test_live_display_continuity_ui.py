"""Complete display continuity and independent input views, real Qt/no device."""
from copy import deepcopy

import numpy as np
import pytest

from test_live_ui import make_measured, quality_frame, window
from vt_fsglove12 import live_ui
from vt_fsglove12.live_display import StableDisplayWarning, input_display_snapshot


def publish_input(controller, stamp, *, connected=True):
    frame = quality_frame(stamp, active=False)
    frame["nodes"].update({str(node): dict(age_s=.005, sample_index=100,
        raw_values=[node, 2, 3, 0, 0, 8197]) for node in range(1, 13) if node != 5})
    controller.state["input_snapshot"] = dict(runtime=dict(connected=connected,
        session_id=controller.state["runtime"].get("session_id"),
        last_sample_age_s=.005, published_monotonic_s=stamp,
        received_samples=1200, input_diagnostics=dict(source_kind="raw_serial",
            bytes_received=24000, frames_decoded=1200, bytes_discarded=0)), orientation=frame)


def publish_display(controller, backend, stamp, *, index=1, offset=0., **metadata):
    pose = dict(display_only=True, session_id=controller.state["runtime"].get("session_id"),
        timestamp_s=stamp, source_timestamp_s=stamp, frame_index=index,
        profile_version=controller.state["pose"].get("profile_version"),
        held_slots=[], unknown_slots=[], provisional_slots=[], source_age_by_slot={})
    pose.update(metadata)
    geometry = backend.render()
    geometry["vertices_m"] = (np.asarray(geometry["vertices_m"]) + offset).tolist()
    controller.state.update(display_pose=pose, display_geometry=geometry)
    return geometry


def setup_display(window, monkeypatch):
    w, controller, backend = window
    clock = [10.]
    monkeypatch.setattr(live_ui.time, "monotonic", lambda: clock[0])
    w.timer.stop()
    make_measured(controller)
    controller.renders_geometry = True
    publish_input(controller, clock[0])
    publish_display(controller, backend, clock[0])
    w.refresh()
    return w, controller, backend, clock


def test_input_view_uses_acquisition_not_stale_solver_snapshot(window, monkeypatch):
    w, controller, backend, clock = setup_display(window, monkeypatch)
    controller.state["runtime"].update(published_monotonic_s=1., last_sample_age_s=9.)
    controller.state["orientation"] = dict(timestamp_s=1., nodes={})
    controller.state["pose"].update(timestamp_s=1., pose_available=False, solver_success=False)
    publish_display(controller, backend, 9., index=2)
    before = deepcopy(controller.state)
    w.input_contract_button.click()
    w.refresh()
    assert "新鲜通道 12/12" in w.stats.text()
    assert "累计样本 1,200" in w.stats.text()
    assert "最新数据年龄 5 ms" in w.stats.text()
    assert "24,000 字节" in w.stats.text()
    assert all("#d6f3e7" in badge.styleSheet() for badge in w.badges.values())
    assert w.input_contract_table.item(4, 6).text() == "20 / 2 / -10"
    assert w.view.mesh.visible() and "解算延迟" in w.view_state.text()
    assert "输入中断" not in w.view_state.text() and "1000 ms" in w.view_state.text()
    assert "不显示历史角度" in w.joint_status.text()
    assert controller.state == before and controller.commands == []


def test_expired_pose_keeps_complete_last_display_and_source_age_keeps_increasing(window, monkeypatch):
    w, controller, _, clock = setup_display(window, monkeypatch)
    vertices, joints = w.view.mesh_data.vertexes().copy(), w.view.joints.pos.copy()
    for stamp in (10.3, 11., 15.):
        clock[0] = stamp
        publish_input(controller, stamp)
        controller.state.update(display_pose=None, display_geometry=None)
        controller.state["pose"].update(pose_available=False, solver_success=False)
        w.refresh()
        assert w.view.mesh.visible() and w.view.joints.visible()
        np.testing.assert_array_equal(w.view.mesh_data.vertexes(), vertices)
        np.testing.assert_array_equal(w.view.joints.pos, joints)
        assert "解算延迟" in w.view_state.text() and "非新测量" in w.view_state.text()
        assert "#fbe1df" not in w.view_state.styleSheet()
        assert "新鲜通道 12/12" in w.stats.text()
    assert "5000 ms" in w.view_state.text()


def test_new_display_geometry_is_accepted_even_when_measurement_has_expired(window, monkeypatch):
    w, controller, backend, clock = setup_display(window, monkeypatch)
    clock[0] = 10.5
    publish_input(controller, clock[0])
    controller.state["pose"].update(pose_available=False, solver_success=False)
    geometry = publish_display(controller, backend, 10.1, index=2, offset=.01,
        held_slots=[8, 9], source_timestamp_by_slot={"8": 9., "9": 9.},
        held_source_quality_by_slot={"8": "degraded", "9": "degraded"})
    w.refresh()
    np.testing.assert_allclose(w.view.mesh_data.vertexes(), np.asarray(geometry["vertices_m"]) @ w.view.basis.T)
    assert w.view.mesh.visible() and w.view.joints.pos.shape == (21, 3)
    w.view.mesh.parseMeshData()
    assert w.view.mesh.faces is w.view.mesh_data.faces()
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), w.view.colors)
    assert "局部保持" in w.view_state.text() and "非新测量" in w.view_state.text()
    assert "最旧局部参考 1500 ms" in w.view_state.text()
    assert "400 ms" in w.view_state.text()


def test_stable_recovery_waits_two_hundred_ms_but_current_mesh_updates_immediately(window, monkeypatch):
    w, controller, backend, clock = setup_display(window, monkeypatch)
    clock[0] = 10.4
    publish_input(controller, clock[0])
    controller.state.update(display_pose=None, display_geometry=None)
    w.refresh()
    assert "#fff3d9" in w.view_state.styleSheet()
    for index, stamp in enumerate((10.41, 10.51, 10.62), 2):
        clock[0] = stamp
        controller.state["runtime"].update(published_monotonic_s=stamp)
        controller.state["pose"].update(timestamp_s=stamp)
        publish_input(controller, stamp)
        geometry = publish_display(controller, backend, stamp, index=index, offset=index * .001)
        w.refresh()
        np.testing.assert_allclose(w.view.mesh_data.vertexes(), np.asarray(geometry["vertices_m"]) @ w.view.basis.T)
        assert w.view.mesh.visible()
        assert ("#d6f3e7" in w.view_state.styleSheet()) is (stamp == 10.62)
        assert ("实时重建" in w.view_state.text()) is (stamp == 10.62)


def test_healthy_input_relabels_automatic_pose_stale_but_preserves_io_errors_and_calibration_feedback(window, monkeypatch):
    w, controller, _, clock = setup_display(window, monkeypatch)
    controller.state["runtime"]["message"] = "手掌数据缺失或已过期；隐藏旧姿态，等待新数据。"
    controller.state["pose"].update(pose_available=False, reason="missing_or_stale_root")
    w.refresh()
    assert "输入正常" in w.status.text() and "等待更新" in w.status.text()
    assert "缺失" not in w.status.text() and "隐藏" not in w.status.text()
    controller.state["runtime"]["error"] = "串口读取失败：test"
    w.refresh()
    assert w.status.text() == "串口读取失败：test"
    controller.state["runtime"].update(error=None, message="全手功能标定已保存；非实测精度")
    w.refresh()
    assert "标定已保存" in w.status.text()


def test_nan_geometry_and_render_failure_do_not_corrupt_previous_complete_model(window, monkeypatch):
    w, controller, backend, clock = setup_display(window, monkeypatch)
    vertices, joints = w.view.mesh_data.vertexes().copy(), w.view.joints.pos.copy()
    clock[0] = 10.01
    geometry = publish_display(controller, backend, clock[0], index=2, offset=.02)
    geometry["joints_m"][8][0] = float("nan")
    w.refresh()
    assert w.view.mesh.visible() and "更新失败" in w.view_state.text()
    np.testing.assert_array_equal(w.view.mesh_data.vertexes(), vertices)
    np.testing.assert_array_equal(w.view.joints.pos, joints)
    assert "已冻结" in w.view_state.text() and "#fbe1df" not in w.view_state.styleSheet()
    controller.state.update(display_geometry=None, display_geometry_error="test worker failure")
    w.refresh()
    assert w.view.mesh.visible() and w._render_error == "test worker failure"
    np.testing.assert_array_equal(w.view.mesh_data.vertexes(), vertices)


def test_disconnection_is_explicit_freeze_but_new_session_never_reuses_previous_hand(window, monkeypatch):
    w, controller, _, clock = setup_display(window, monkeypatch)
    clock[0] = 10.3
    publish_input(controller, clock[0], connected=False)
    controller.state["runtime"]["connected"] = False
    controller.state.update(display_pose=None, display_geometry=None)
    w.refresh()
    assert w.view.mesh.visible() and "已断开" in w.view_state.text() and "已冻结" in w.view_state.text()
    controller.state["runtime"].update(connected=True, session_id="new-session")
    publish_input(controller, clock[0])
    w.refresh()
    assert not w.view.mesh.visible() and not w._has_display_cache


def test_new_reference_and_clear_drop_old_display_even_if_worker_still_sends_it(window, monkeypatch):
    w, controller, backend, clock = setup_display(window, monkeypatch)
    controller.state["pose"]["profile_version"] = 1
    publish_display(controller, backend, clock[0], index=2)
    w.refresh()
    assert w._has_display_cache
    controller.state["pose"]["profile_version"] = 2
    w.refresh()
    assert not w._has_display_cache and not w.view.mesh.visible()
    publish_display(controller, backend, clock[0], index=3)
    w.refresh()
    assert w._has_display_cache
    controller.state["calibration"].update(profile_status="clearing", clear_generation=1)
    w.refresh()
    assert not w._has_display_cache and not w.view.mesh.visible()
    assert w._last_pose is None and "正在清除" in w.view_state.text()


def test_unknown_reference_outline_is_explicitly_not_measurement(window, monkeypatch):
    w, controller, backend, clock = setup_display(window, monkeypatch)
    publish_display(controller, backend, clock[0], index=2, unknown_slots=[8, 9])
    w.refresh()
    assert w.view.mesh.visible() and "无历史参考" in w.view_state.text()
    assert "中立轮廓不是测量值" in w.view_state.text()


def test_child_table_throttles_and_unchanged_tooltips_do_not_rebuild(window, monkeypatch):
    w, controller, _, clock = setup_display(window, monkeypatch)
    w.input_contract_button.click()
    item = w.input_contract_table.item(4, 10)
    calls = []
    monkeypatch.setattr(item, "setToolTip", lambda text: calls.append(text))
    before = w._input_contract_refresh_at
    clock[0] += .05
    publish_input(controller, clock[0])
    w.refresh()
    assert w._input_contract_refresh_at == before
    clock[0] += .06
    publish_input(controller, clock[0])
    w.refresh()
    assert w._input_contract_refresh_at == clock[0] and calls == []


def test_input_snapshot_contract_and_display_warning_are_read_only():
    legacy = {"runtime": {"connected": True}}
    assert input_display_snapshot(legacy) is legacy
    assert input_display_snapshot({**legacy, "input_snapshot": None})["orientation"] is None
    state = StableDisplayWarning()
    assert not state.update(False, 1.)
    assert state.update(True, 1.1)
    assert state.update(False, 1.2)
    assert state.update(False, 1.39)
    assert not state.update(False, 1.41)
