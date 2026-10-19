"""Optional Qt interaction tests; no serial, real model, or GL paint loop."""
from copy import deepcopy
import time

import numpy as np
import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("pyqtgraph")
pytest.importorskip("OpenGL")
from PyQt5 import QtWidgets

from vt_fsglove12.config import PARENTS, load_config as _load_config
from vt_fsglove12 import live_ui


def load_config():
    # These injected-controller tests predate receive-health evidence. Exercise
    # their original UI contracts explicitly; dedicated health tests enable it.
    config = _load_config()
    config["input_health"] = {"enabled": False}
    return config


class Controller:
    def __init__(self):
        self.commands, self.started, self.stopped = [], False, False
        self.state = {"runtime": {"connected": False, "received_samples": 0, "last_sample_age_s": None,
                                  "published_monotonic_s": time.monotonic(), "message": "idle", "error": None},
                      "calibration": {"active": None, "captured_steps": [], "calibrated_nodes": [],
                                      "rejected_nodes": {}, "warnings": []},
                      "orientation": None, "pose": None}

    def start(self): self.started = True
    def stop(self): self.stopped = True
    def join(self, timeout): return True
    def latest_snapshot(self): return deepcopy(self.state)
    def submit(self, name, **args):
        self.commands.append((name, args))
        return True


class Backend:
    fail = False
    def __init__(self):
        self.rendered_angles = []

    def export_rest(self):
        return {"parents": list(PARENTS), "rest_global_xyzw": [[0, 0, 0, 1]] * 16}

    def render(self, *_):
        if self.fail:
            raise RuntimeError("deliberate mock render failure")
        if _ and _[0] is not None:
            self.rendered_angles.append(np.asarray(_[0]).copy())
        joints = np.zeros((21, 3))
        joints[5], joints[9], joints[17] = [-.08, 0, .03], [-.09, 0, 0], [-.07, 0, -.04]
        return {"vertices_m": [[0, 0, 0], [-.1, 0, 0], [0, 0, .03]], "faces": [[0, 1, 2]],
                "joints_m": joints.tolist()}


@pytest.fixture
def window(monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(live_ui, "list_serial_ports", lambda: [])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    controller, backend = Controller(), Backend()
    w = live_ui.LiveWindow(load_config(), backend, output_root=tmp_path,
                           port="/mock/imu", controller=controller)
    try:
        yield w, controller, backend
    finally:
        w.close()
        app.processEvents()


def make_measured(controller):
    now = time.monotonic()
    controller.state["runtime"].update(connected=True, published_monotonic_s=now,
                                      last_sample_age_s=.005, session_id="mock")
    controller.state["calibration"]["calibrated_nodes"] = ["12"]
    controller.state["pose"] = dict(timestamp_s=now, pose_available=True, solver_success=True,
                                     quality="experimental_partial", anatomical_euler_rad=[[0, 0, 0]] * 16,
                                     root_quat_xyzw=[0, 0, 0, 1])


def test_independent_left_and_right_windows_keep_hand_and_controls_separate(monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(live_ui, "list_serial_ports", lambda: [])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    windows, controllers = [], []
    try:
        for hand in ("right", "left"):
            config, backend, controller = _load_config(hand=hand), Backend(), Controller()
            config["input_health"] = {"enabled": False}
            backend.hand = hand
            window = live_ui.LiveWindow(config, backend, output_root=tmp_path / hand,
                port=f"/mock/{hand}", controller=controller, upstream_checkout="NO_MODEL_OR_HARDWARE")
            windows.append(window)
            controllers.append(controller)
            assert window.hand == window.axes["hand"] == hand
            assert ("右手" if hand == "right" else "左手") in window.windowTitle()
            assert window.ports.currentText() == f"/mock/{hand}"
            assert controller.started and not controller.commands  # Never auto-connect.
        assert windows[0].view is not windows[1].view
        assert "左手候选映射未实测" in windows[1].mode.text()
        assert "显式左手映射" in windows[1].tactile_mapping_note.text()
        assert "9 锚点已核对" not in windows[1].tactile_mapping_note.text()
        windows[0].send("clear_calibration")
        assert controllers[0].commands and not controllers[1].commands
    finally:
        for window in windows:
            window.close()
        app.processEvents()


def test_tactile_colors_never_replace_pose_mesh_and_disable_restores_base(window):
    w, controller, _backend = window
    colors = w.view.colors.copy()
    mesh, topology = w.view.mesh, w.view.mesh_data.faces()
    class Binding:
        def vertex_colors(self, delta, valid, base, *, limit):
            assert limit == 3000.
            result = base.copy()
            result[0] = [1., .2, .1, 1.]
            return result
    w.view.tactile_binding = Binding()
    w.view.set_tactile(np.ones((20, 20)), np.ones((20, 20), bool))
    assert not np.array_equal(w.view.mesh_data.vertexColors(), colors)
    np.testing.assert_array_equal(w.view.colors, colors)
    w.view.set_geometry(w.neutral)
    assert w.view.mesh is mesh and w.view.mesh_data.faces() is topology
    assert not np.array_equal(w.view.mesh_data.vertexColors(), colors)
    w.view.set_tactile()
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), colors, atol=1e-7)
    assert w.view.mesh.visible()
    assert controller.commands == []


def test_tactile_reference_is_not_imu_calibration_and_stale_overlay_keeps_hand(window):
    from types import SimpleNamespace
    w, controller, _backend = window
    class Overlay:
        def __init__(self):
            self.baseline = SimpleNamespace(collecting=False)
            self.started = 0
        def poll(self): pass
        def start_reference(self):
            self.started += 1
            self.baseline.collecting = True
        def frame(self, *args, **kwargs):
            return SimpleNamespace(delta=None, valid_mask=None, status="触觉断流；手势继续",
                                   age_ms=None, skew_ms=None)
        def close(self): pass
    overlay = w.tactile_controller = Overlay()
    w.tactile_enabled.setEnabled(True)
    w.tactile_enabled.setChecked(True)
    w.start_tactile_reference()
    assert overlay.started == 1 and not w.tactile_reference.isEnabled()
    assert controller.commands == []
    assert w.view.mesh.visible()
    assert "触觉断流" in w.tactile_status.text()
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), w.view.colors, atol=1e-7)


def test_per_joint_quality_is_text_only_and_unavailable_legacy_geometry_does_not_replace_display(window, monkeypatch):
    w, controller, backend = window
    clock = [10.]
    monkeypatch.setattr(live_ui.time, "monotonic", lambda: clock[0])
    make_measured(controller)
    # The pure neutral-segment mapping has its own test. This three-vertex
    # backend lets us inspect the real Qt/GLMeshData per-slot color path.
    w.view.vertex_slots = np.asarray([7, 8, 9])
    pose = controller.state["pose"]
    pose.update(joint_valid_mask=[True] * 16, joint_display_available_mask=[True] * 16,
                joint_quality=["valid"] * 16)
    w.refresh()
    base_colors = w.view.colors.copy()
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), base_colors)
    topology = w.view.mesh_data.faces()
    mesh = w.view.mesh

    def change(state, angle, advance=.01):
        clock[0] += advance
        controller.state["runtime"]["published_monotonic_s"] = clock[0]
        pose["timestamp_s"] = clock[0]
        pose["anatomical_euler_rad"] = [[0., 0., 0.] for _ in range(16)]
        pose["anatomical_euler_rad"][8][2] = angle
        for slot in (8, 9):
            pose["joint_valid_mask"][slot] = state == "valid"
            pose["joint_display_available_mask"][slot] = state != "unavailable"
            pose["joint_quality"][slot] = state
        w.refresh()

    change("degraded", .4)
    assert w.view.mesh.visible() and w.view.joints.visible()
    assert "22.9° [暂估]" in w.joint_status.text()
    assert "不代表有效测量" in w.view_state.text()
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), base_colors)
    assert w.view.mesh is mesh and w.view.mesh_data.faces() is topology

    change("degraded", -.5)
    assert "-28.6° [暂估]" in w.joint_status.text()
    assert backend.rendered_angles[-1][8, 2] == -.5  # no last-good hold
    previous = w.view.mesh_data.vertexes().copy()
    change("unavailable", 0.)
    assert "小指中节 — [不可用]" in w.joint_status.text()
    assert "不代表有效测量" in w.view_state.text()
    assert w.view.mesh.visible() and w.view.joints.visible()
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), base_colors)
    np.testing.assert_array_equal(w.view.mesh_data.vertexes(), previous)
    assert backend.rendered_angles[-1][8, 2] == -.5  # zero placeholder was not rendered
    w.view.mesh.parseMeshData()
    assert w.view.mesh.faces is topology  # no surface removal / chopped fingers
    assert w.view._joint_visible.all()
    assert w.view.mesh_data.faces() is topology

    change("valid", .3)
    assert pose["joint_valid_mask"][8] is True
    assert "恢复" in w.view_state.text()
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), base_colors)
    w.view.mesh.parseMeshData()
    assert w.view.mesh.faces is topology
    change("valid", .4, advance=.21)
    np.testing.assert_allclose(w.view.mesh_data.vertexColors(), base_colors)
    w.view.mesh.parseMeshData()
    assert w.view.mesh.faces is topology and w.view._joint_visible.all()
    assert controller.commands == [] and controller.state["runtime"]["connected"]

    # A failed parent still does not chop the cached complete model.
    pose["joint_valid_mask"][7] = pose["joint_display_available_mask"][7] = False
    pose["joint_quality"][7] = "unavailable"
    w.refresh()
    assert w.view._joint_visible.all()
    assert pose["joint_valid_mask"][8] and pose["joint_valid_mask"][9]


def quality_frame(stamp, *, active=True):
    return dict(timestamp_s=stamp, frame_index=99, nodes={"5": {"age_s": .001, "sample_index": 1000,
                "raw_values": ([32760, 2, -32764, 0, 0, 8197] if active else [20, 2, -10, 0, 0, 8197])}}, sensor_quality={
        "gyro_near_rail": dict(threshold_lsb=32760, recent_window_s=2.,
            active_nodes=["5"] if active else [], nodes={"5": dict(
                total_samples=1000, near_rail_samples=8,
                recent_near_rail_samples=8 if active else 0,
                axes=["x", "z"] if active else [], recent_counts_complete=True)})})


def with_input_protection(frame, *, gyro_action="held", accel_action="skip_accel", valid=True):
    status = dict(gyro_action=gyro_action, accel_action=accel_action,
                  gyro_rejected=gyro_action != "pass", accel_rejected=accel_action != "pass",
                  raw_samples=1000, rejected_samples=9, gyro_rejected_samples=8,
                  accel_rejected_samples=4, gyro_run=1, accel_run=1,
                  orientation_valid=valid, reasons=["test_anomaly"])
    frame["input_filter"] = dict(enabled=True, rejected_samples=9, active_nodes=["5"], nodes={"5": status})
    frame["nodes"]["5"]["input_filter"] = deepcopy(status)
    frame["nodes"]["5"]["orientation_valid"] = valid
    return frame


def with_axis_input_protection(frame, *, actions=None):
    actions = actions or {"x": "pass", "y": "pass", "z": "pass"}
    aggregate = "zero_motion" if "zero_motion" in actions.values() else "held" if "held" in actions.values() else "pass"
    with_input_protection(frame, gyro_action=aggregate, accel_action="pass")
    status = frame["nodes"]["5"]["input_filter"]
    raw = frame["nodes"]["5"]["raw_values"]
    observed = [axis for axis, value in zip("xyz", raw[:3]) if abs(value) >= 32760]
    status.update(revision="per_axis_gyro_continuity_v2", gyro_action_by_axis=dict(actions),
                  gyro_boundary_axes=observed,
                  gyro_boundary_integrated_axes=[axis for axis in observed if actions[axis] == "pass"],
                  gyro_boundary_samples_by_axis={"x": 8, "y": 0, "z": 2},
                  gyro_held_samples_by_axis={"x": 2, "y": 0, "z": 0},
                  gyro_zero_motion_samples_by_axis={"x": 1, "y": 0, "z": 0},
                  boundary_is_physical_saturation_proof=False)
    frame["input_filter"]["nodes"]["5"] = deepcopy(status)
    return frame


def test_input_protection_keeps_disabled_quality_text_exactly_unchanged():
    frame = quality_frame(100.)
    before = live_ui.input_quality_text(frame, 100., connected=True)
    frame["input_filter"] = dict(enabled=False, rejected_samples=99, active_nodes=["5"])
    assert live_ui.input_quality_text(frame, 100., connected=True) == before
    assert "不丢弃或截断数据" in before[0]


def test_input_protection_exposes_actions_without_changing_raw_boundary_comparison():
    frame = with_input_protection(quality_frame(100., active=False))
    before = deepcopy(frame)
    text, current = live_ui.input_quality_text(frame, 100., connected=True)
    assert current == [] and "当前原始值未命中编码边界" in text
    assert "解算输入保护：已启用" in text and "累计拦截 9 条" in text
    assert "当前保护节点：5号(陀螺/加速度)" in text
    assert "原始记录不丢弃或截断；解算输入按保护策略处理" in text
    assert "不能恢复被拦截期间的真实运动" in text
    rows = live_ui.input_contract_rows(live_ui.describe_input_contract(load_config()), frame, 100., connected=True)
    assert rows[4][6:8] == ["20 / 2 / -10", "0 / 0 / 8197"]
    assert rows[4][11] == "陀螺：保持上次有效输入\n加速度：跳过更新"
    assert frame == before
    frame["nodes"]["5"]["raw_values"][0] = 32764
    text, current = live_ui.input_quality_text(frame, 100., connected=True)
    assert current == ["5"] and "当前帧编码边界提示：5号(X)" in text
    assert "硬件饱和未确认" in text


def test_input_protection_degraded_and_recovered_actions_do_not_hide_history():
    frame = with_input_protection(quality_frame(100.), gyro_action="zero_motion", accel_action="pass", valid=False)
    text, _ = live_ui.input_quality_text(frame, 100., connected=True)
    assert "当前保护节点：5号(陀螺)" in text and "姿态降级/不可用：5号" in text
    assert "历史事件" in text and "5号(X/Z) 8条" in text
    assert live_ui.input_filter_node_text(frame["nodes"]["5"]["input_filter"]) == (
        "陀螺：零运动（偏置回退）\n加速度：透传\n姿态降级：不可用")
    frame = with_input_protection(quality_frame(100., active=False), gyro_action="pass", accel_action="pass")
    text, _ = live_ui.input_quality_text(frame, 100., connected=True)
    assert "当前无拦截动作（不代表姿态准确）" in text and "累计拦截 9 条" in text
    assert "当前保护节点：5号" not in text


@pytest.mark.parametrize("now,connected,expected", [(100.3, True, "数据已过期"),
                                                   (99., True, "数据已过期"),
                                                   (100., False, "未连接")])
def test_input_protection_stale_status_never_claims_current_protected_nodes(now, connected, expected):
    frame = with_input_protection(quality_frame(100.))
    text, current = live_ui.input_quality_text(frame, now, connected=connected)
    assert expected in text and not current
    assert "累计拦截 9 条" in text and "当前保护节点：5号" not in text
    rows = live_ui.input_contract_rows(live_ui.describe_input_contract(load_config()), frame, now, connected=connected)
    assert rows[4][11] == "—"


def test_input_protection_main_and_read_only_detail_show_same_actions(window):
    w, controller, _ = window
    make_measured(controller)
    controller.state["orientation"] = with_input_protection(quality_frame(time.monotonic(), active=False))
    w.input_contract_button.click()
    w.refresh()
    assert "异常输入已拦截：5号" in w.input_status.text()
    assert "当前保护节点：5号(陀螺/加速度)" in w.input_contract_status.text()
    assert "快照 #99" in w.input_status.text() and "快照 #99" in w.input_contract_status.text()
    assert w.input_contract_table.item(4, 11).text() == "陀螺：保持上次有效输入\n加速度：跳过更新"
    assert w.input_contract_table.item(4, 6).text() == "20 / 2 / -10"
    assert "#ffe0a3" not in w.badges[5].styleSheet()
    assert "#dceefe" in w.badges[5].styleSheet()
    assert w.step_buttons["neutral"].isEnabled() and w._last_pose is not None
    assert controller.commands == []


def test_main_handled_boundary_is_information_not_an_unhandled_alarm(window):
    w, controller, _ = window
    make_measured(controller)
    frame = with_input_protection(quality_frame(time.monotonic()))
    controller.state["orientation"] = frame
    before = deepcopy(frame)
    w.input_contract_button.click()
    w.refresh()
    assert "异常输入已拦截：5号" in w.input_status.text()
    assert "累计已拦截 9 条" in w.input_status.text()
    assert "编码边界提示" not in w.input_status.text()
    assert "32760" not in w.input_status.text()
    assert "#fff3d9" not in w.input_status.styleSheet()
    assert "#dceefe" in w.badges[5].styleSheet()
    assert "#ffe0a3" not in w.badges[5].styleSheet()
    assert "当前帧编码边界提示" in w.input_contract_status.text()
    assert w.input_contract_table.item(4, 6).text() == "32760 / 2 / -32764"
    assert w.step_buttons["neutral"].isEnabled()
    assert controller.commands == [] and controller.state["orientation"] == before


def test_main_keeps_real_unhandled_boundary_warning_even_if_filter_enabled():
    frame = with_input_protection(quality_frame(100.), gyro_action="pass", accel_action="pass")
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert warnings == ["5"]
    assert "尚未确认" in text
    assert "异常输入已拦截" not in text
    assert "当前输入已通过" not in text


def test_new_boundary_integration_is_explicit_information_not_an_unhandled_alarm():
    frame = with_axis_input_protection(quality_frame(100.))
    before = deepcopy(frame)
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert not warnings
    assert "边界报告值已积分：5号(X/Z)；是否饱和未确认" in text
    assert "编码边界累计（按轴）：5号(X 8 / Z 2)" in text
    assert "尚未确认这些通道的过滤处理结果" not in text
    assert "当前输入已通过过滤检查" not in text
    assert "相关姿态暂不可用" not in text
    assert frame == before


@pytest.mark.parametrize("patch", [
    {"revision": "old_rule"}, {"gyro_boundary_axes": ["x"]},
    {"gyro_boundary_integrated_axes": []}, {"boundary_is_physical_saturation_proof": True},
    {"gyro_action_by_axis": {"x": "pass"}},
    {"gyro_action_by_axis": {"x": "held", "y": "pass", "z": "pass"}},
])
def test_boundary_integration_claim_requires_complete_matching_new_contract(patch):
    frame = with_axis_input_protection(quality_frame(100.))
    frame["nodes"]["5"]["input_filter"].update(patch)
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert warnings == ["5"] and "尚未确认" in text
    assert "边界报告值已积分" not in text


def test_mixed_gyro_axis_actions_do_not_describe_good_axes_as_zero_motion():
    frame = with_axis_input_protection(quality_frame(100.),
                                      actions={"x": "zero_motion", "y": "pass", "z": "pass"})
    status = frame["nodes"]["5"]["input_filter"]
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert not warnings
    assert "异常输入已拦截：5号" in text
    assert "边界报告值已积分：5号(Z)" in text
    cell = live_ui.input_filter_node_text(status)
    assert "陀螺：X 零运动（偏置回退）；Y/Z 透传" in cell
    assert "边界报告值已积分：Z；是否饱和未确认" in cell
    assert "累计保持：X 2" in cell and "累计偏置回退：X 1" in cell
    assert "5号(陀螺 X)" in live_ui.input_filter_summary(frame, 100., connected=True)


def test_new_boundary_history_remains_visible_after_current_values_return_to_normal():
    frame = with_axis_input_protection(quality_frame(100., active=False))
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert not warnings and "当前输入已通过过滤检查" in text
    assert "边界报告值已积分：" not in text
    assert "编码边界累计（按轴）：5号(X 8 / Z 2)" in text
    assert "历史不代表当前故障，是否饱和未确认" in text
    cell = live_ui.input_filter_node_text(frame["nodes"]["5"]["input_filter"])
    assert "X/Y/Z 透传" in cell
    assert "编码边界累计：X 8 / Z 2" in cell
    assert "累计保持：X 2" in cell and "累计偏置回退：X 1" in cell


def test_new_boundary_status_keeps_badges_stable_and_does_not_hide_pose_or_disable_controls(window):
    w, controller, _ = window
    make_measured(controller)
    w.input_contract_button.click()
    styles = []
    for at_boundary in (True, False, True, False):
        frame = with_axis_input_protection(quality_frame(time.monotonic(), active=at_boundary))
        controller.state["orientation"] = frame
        before = deepcopy(frame)
        w.refresh()
        styles.append(w.badges[5].styleSheet())
        assert "#fff3d9" not in w.input_status.styleSheet()
        assert "编码边界累计（按轴）：5号(X 8 / Z 2)" in w.input_status.text()
        assert w.step_buttons["neutral"].isEnabled() and w._last_pose is not None
        assert controller.state["orientation"] == before and controller.commands == []
    assert len(set(styles)) == 1 and "#d6f3e7" in styles[0]


def test_main_keeps_degraded_pose_warning_after_prolonged_rejection():
    frame = with_input_protection(quality_frame(100.), gyro_action="zero_motion", valid=False)
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert warnings == ["5"]
    assert "相关姿态暂不可用：5号" in text
    assert "异常输入已拦截：5号" in text


def test_main_recovered_input_does_not_alarm_on_remembered_history():
    frame = with_input_protection(quality_frame(100., active=False), gyro_action="pass", accel_action="pass")
    rail = frame["sensor_quality"]["gyro_near_rail"]
    rail["active_nodes"] = ["5"]
    rail["nodes"]["5"].update(recent_near_rail_samples=8, axes=["x"])
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert not warnings and "当前输入已通过过滤检查" in text
    assert "累计已拦截 9 条" in text and "累计数不是当前故障" in text
    assert "8条" not in text
    assert "历史事件" in live_ui.input_quality_text(frame, 100., connected=True)[0]


@pytest.mark.parametrize("now,connected", [(100.3, True), (99., True), (100., False)])
def test_main_never_claims_handled_or_healthy_for_expired_input(now, connected):
    frame = with_input_protection(quality_frame(100.))
    text, warnings = live_ui.solver_input_status(frame, now, connected=connected)
    assert not warnings
    assert "异常输入已拦截：" not in text and "当前输入已通过" not in text


def test_main_absent_node_protection_is_unconfirmed_not_assumed_filtered():
    frame = with_input_protection(quality_frame(100.))
    frame["nodes"]["5"].pop("input_filter")
    frame["input_filter"]["nodes"] = {}
    text, warnings = live_ui.solver_input_status(frame, 100., connected=True)
    assert warnings == ["5"] and "尚未确认" in text


def test_input_quality_warning_does_not_disable_calibration_or_hide_fresh_pose(window):
    w, controller, backend = window
    make_measured(controller)
    controller.state["orientation"] = quality_frame(time.monotonic())
    w.refresh()
    assert "编码边界提示" in w.input_status.text()
    assert "硬件饱和未确认" in w.input_status.text()
    assert "不丢弃或截断数据" in w.input_status.text()
    assert "放慢动作" not in w.input_status.text()
    assert "5号(X/Z)" in w.input_status.text()
    assert "#ffe0a3" in w.badges[5].styleSheet()
    assert w.step_buttons["neutral"].isEnabled()
    assert w._last_pose is not None
    assert controller.commands == []
    assert "5号=0.00875" in w.details.toPlainText()


def test_passive_serial_counters_are_visible_without_device_commands(window):
    w, controller, _ = window
    controller.state["runtime"]["input_diagnostics"] = dict(
        source_kind="raw_serial", bytes_received=1234, frames_decoded=10, bytes_discarded=80)
    w.refresh()
    assert "串口接收 1,234 字节" in w.stats.text()
    assert "解码 IMU 帧 10" in w.stats.text()
    assert "解析丢弃 80 字节" in w.stats.text()
    assert "不代表 CRC 校验通过" in w.stats.text()


def test_ros_results_status_does_not_issue_device_commands(window):
    w, controller, _ = window
    controller.state["runtime"]["ros_export"] = dict(enabled=True, sent=17, dropped=2,
                                                     last_error="missing bridge")
    w.refresh()
    assert "ROS 结果出口：已发送 17 帧" in w.stats.text()
    assert "非 bag 接收确认" in w.stats.text()
    assert "手套采集与标定继续" in w.stats.text()
    assert controller.commands == []
    assert controller.commands == []
    controller.state["runtime"]["input_diagnostics"] = None
    w.refresh()
    assert "串口接收" not in w.stats.text()


def test_input_quality_recent_clear_keeps_history_without_green_accuracy_claim(window):
    w, controller, _ = window
    make_measured(controller)
    controller.state["orientation"] = quality_frame(time.monotonic(), active=False)
    w.refresh()
    assert "累计 8 条" in w.input_status.text()
    assert "不代表无漂移" in w.input_status.text()
    assert "疑似截顶" not in w.input_status.text()
    assert "无编码边界事件" in w.input_status.text()
    assert "硬件饱和未确认" in w.input_status.text()
    assert "#ffe0a3" not in w.badges[5].styleSheet()
    assert controller.commands == []


def test_exact_same_frame_low_current_value_and_historical_peak_are_not_one_alarm():
    from vt_fsglove12.live_stream import OnlineFusion, RawSample
    config = load_config()
    fusion = OnlineFusion(config)
    fusion.update([RawSample(5, (32764, 0, 0, 0, 0, 8197), 100., 200.),
                   RawSample(5, (8000, 0, 0, 0, 0, 8197), 100.01, 200.01)])
    frame = fusion.snapshot(100.02)
    original = deepcopy(frame)
    text, current = live_ui.input_quality_text(frame, 100.02, connected=True)
    rows = live_ui.input_contract_rows(live_ui.describe_input_contract(config), frame, 100.02, connected=True)
    assert current == []
    assert "当前原始值未命中编码边界" in text
    assert "历史事件" in text and "5号(X) 1条" in text
    assert rows[4][6] == "8000 / 0 / 0" and rows[4][9] == "2"
    assert "#1 @ 100.000000s" in rows[4][10] and "32764 / 0 / 0" in rows[4][10]
    assert "20 ms 前" in rows[4][10] and "不是当前值" in rows[4][10]
    assert frame == original


def test_historical_count_without_saved_raw_never_fabricates_current_or_trigger_value():
    frame = quality_frame(100.)
    frame["nodes"]["5"].pop("raw_values")
    text, current = live_ui.input_quality_text(frame, 100., connected=True)
    assert current == [] and "当前原始值不可用" in text
    rows = live_ui.input_contract_rows(live_ui.describe_input_contract(load_config()), frame, 100., connected=True)
    assert rows[4][6] == "—" and rows[4][10] == "旧记录未保存触发原值"


def test_main_and_child_share_input_snapshot_but_child_table_updates_at_most_ten_hz(window, monkeypatch):
    from vt_fsglove12.live_stream import OnlineFusion, RawSample
    w, controller, _ = window
    w.timer.stop()
    make_measured(controller)
    now = time.monotonic()
    monkeypatch.setattr(live_ui.time, "monotonic", lambda: now)
    fusion = OnlineFusion(load_config())
    fusion.update([RawSample(5, (32764, 0, 0, 0, 0, 8197), now, 200.)])
    controller.state["orientation"] = fusion.snapshot(now)
    w.input_contract_button.click()
    w.refresh()
    assert w.input_contract_table.item(4, 6).text() == "32764 / 0 / 0"
    assert "#ffe0a3" in w.badges[5].styleSheet()
    now += .02
    fusion.update([RawSample(5, (8000, 0, 0, 0, 0, 8197), now, 200.02)])
    controller.state["orientation"] = fusion.snapshot(now)
    calls = []
    def latest():
        calls.append(1)
        return deepcopy(controller.state)
    monkeypatch.setattr(controller, "latest_snapshot", latest)
    w.refresh()
    assert len(calls) == 1
    assert w.input_contract_table.item(4, 6).text() == "32764 / 0 / 0"
    assert "当前输入已通过过滤检查" in w.input_status.text()
    assert "#ffe0a3" not in w.badges[5].styleSheet()
    now += .1
    w.refresh()
    assert w.input_status.text().splitlines()[0] == w.input_contract_status.text().splitlines()[0]
    assert w.input_contract_table.item(4, 6).text() == "8000 / 0 / 0"
    assert w.input_contract_table.item(4, 9).text() == "2"
    assert "32764 / 0 / 0" in w.input_contract_table.item(4, 10).text()
    assert controller.commands == []


def test_input_quality_missing_disconnected_and_stale_are_not_current_alerts():
    assert "等待统计" in live_ui.input_quality_text({}, 100., connected=True)[0]
    frame = quality_frame(100.)
    text, nodes = live_ui.input_quality_text(frame, 100.3, connected=True)
    assert "数据已过期" in text and not nodes
    text, nodes = live_ui.input_quality_text(frame, 100., connected=False)
    assert "未连接" in text and not nodes
    text, nodes = live_ui.input_quality_text(frame, 99., connected=True)
    assert "数据已过期" in text and not nodes
    frame["nodes"] = {}
    text, nodes = live_ui.input_quality_text(frame, 100., connected=True)
    assert "等待新鲜原始数据" in text and "累计 8 条" in text and not nodes
    assert "未检测到" not in text


def test_empty_or_silent_real_fusion_snapshot_never_claims_clean_input():
    from vt_fsglove12.live_stream import OnlineFusion, RawSample
    fusion = OnlineFusion(load_config())
    empty = fusion.snapshot(100.)
    text, nodes = live_ui.input_quality_text(empty, 100., connected=True)
    assert "等待新鲜原始数据" in text and not nodes
    fusion.update([RawSample(5, (32764, 0, 0, 0, 0, 8197), 100., 1000.)])
    silent = fusion.snapshot(103.)
    assert silent["timestamp_s"] == 103. and silent["nodes"] == {}
    text, nodes = live_ui.input_quality_text(silent, 103., connected=True)
    assert "等待新鲜原始数据" in text and "累计 1 条" in text and not nodes


def test_legacy_monitor_interpretation_does_not_claim_physical_clipping():
    frame = quality_frame(100.)
    frame["sensor_quality"]["gyro_near_rail"]["interpretation"] = "suspected_raw_clipping_not_verified_sensor_range"
    text, nodes = live_ui.input_quality_text(frame, 100., connected=True)
    assert nodes == ["5"] and "硬件饱和未确认" in text
    assert "疑似截顶" not in text and "原始陀螺疑似" not in text
    frame["sensor_quality"]["gyro_near_rail"]["nodes"]["5"]["recent_counts_complete"] = False
    assert "至少 8条" in live_ui.input_quality_text(frame, 100., connected=True)[0]


def test_input_contract_summary_and_twelve_rows_use_real_mapping_and_overrides():
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175}
    config["node_to_slot"]["5"], config["node_to_slot"]["3"] = 1, 14
    contract = live_ui.describe_input_contract(config)
    summary = live_ui.input_contract_summary(contract)
    assert "LSM6DSRTR" in summary and "用户提供" in summary
    assert "未核实" in summary and "120 Hz" in summary
    assert "不是芯片 ODR" in summary and "未读回" in summary
    assert "没有绝对航向观测" in summary and "不能靠三姿势标定持续消除" in summary
    assert "算法估计，不是硬件真值" in summary
    rows = live_ui.input_contract_rows(contract, {}, 100., connected=False)
    assert len(rows) == 12
    assert rows[4][1:4] == ["食指·近", "0.0175", "0.000122"]
    assert rows[2][1] == "拇指·近"
    assert rows[10][1] == "辅助·未映射" and rows[11][1] == "掌参考"
    assert all(row[4:] == ["未读回 / 未确认", "—", "—", "—", "—", "—", "—", "—"] for row in rows)


def test_legacy_input_config_is_not_relabelled_as_verified_lsm_device():
    config = load_config()
    config.pop("input_contract", None)
    contract = live_ui.describe_input_contract(config)
    summary = live_ui.input_contract_summary(contract)
    assert "型号：unknown" in summary and "配置未声明" in summary
    assert "用户提供" not in summary
    assert contract["nodes"]["5"]["nominal_gyro_full_scale_dps"] is None


def test_input_contract_live_rows_hide_stale_data_and_do_not_mutate_snapshot():
    contract = live_ui.describe_input_contract(load_config())
    frame = quality_frame(100.)
    frame["nodes"]["5"]["raw_values"] = [32764, -7, 0, 1, 2, 8197]
    frame["input_observation"] = {"nodes": {"5": {"observed_receive_rate_hz": 119.875}}}
    before = deepcopy(frame)
    rows = live_ui.input_contract_rows(contract, frame, 100., connected=True)
    assert rows[4][5:] == ["119.88", "32764 / -7 / 0", "1 / 2 / 8197", "—", "1000", "旧记录未保存触发原值", "未启用"]
    for now, connected in ((100.3, True), (99., True), (100., False)):
        stale = live_ui.input_contract_rows(contract, frame, now, connected=connected)
        assert stale[4][5:] == ["—", "—", "—", "—", "—", "—", "—"]
    assert frame == before


def test_vqf_display_converts_bias_units_and_never_infers_absolute_heading():
    state = dict(rest_detected=True, gyro_bias_rad_s=np.deg2rad([.1, -.2, .3]).tolist(),
                 bias_sigma_rad_s=float(np.deg2rad(.01)), absolute_heading_observed=False)
    before = deepcopy(state)
    assert live_ui.input_fusion_text(state) == "判为静止\n偏置 +0.100 / -0.200 / +0.300 °/s"
    state["rest_detected"] = False
    assert live_ui.input_fusion_text(state).startswith("未判静止")
    state["gyro_bias_rad_s"] = [float("nan"), 0, 0]
    assert live_ui.input_fusion_text(state) == "未判静止\n偏置未知"
    assert live_ui.input_fusion_text(None) == "—"
    frame = quality_frame(100.)
    frame["nodes"]["5"]["fusion_state"] = before
    contract = live_ui.describe_input_contract(load_config())
    assert live_ui.input_contract_rows(contract, frame, 100., connected=True)[4][8].startswith("判为静止")
    assert live_ui.input_contract_rows(contract, frame, 100.3, connected=True)[4][8] == "—"
    assert live_ui.input_contract_rows(contract, frame, 100., connected=False)[4][8] == "—"
    assert frame["nodes"]["5"]["fusion_state"] == before


def test_input_contract_dialog_is_read_only_and_does_not_dispatch_hardware_commands(window):
    w, controller, _ = window
    make_measured(controller)
    now = time.monotonic()
    controller.state["orientation"] = quality_frame(now)
    controller.state["orientation"]["nodes"]["5"]["raw_values"] = [1, 2, 3, 4, 5, 6]
    controller.state["orientation"]["nodes"]["5"]["fusion_state"] = dict(
        rest_detected=False, gyro_bias_rad_s=[0., 0., 0.], bias_sigma_rad_s=float(np.deg2rad(.01)),
        absolute_heading_observed=False)
    w.input_contract_button.click()
    table = w.input_contract_table
    assert table.rowCount() == 12 and table.columnCount() == 12
    assert table.editTriggers() == QtWidgets.QAbstractItemView.NoEditTriggers
    assert table.item(4, 1).text() == "拇指·近"
    assert table.item(4, 6).text() == "1 / 2 / 3"
    assert table.item(4, 8).text() == "未判静止\n偏置 +0.000 / +0.000 / +0.000 °/s"
    assert "不是硬件偏置真值" in table.item(4, 8).toolTip()
    assert "0.0100 °/s" in table.item(4, 8).toolTip()
    assert "硬件饱和未确认" in w.input_contract_status.text()
    w.input_contract_dialog.hide()
    assert controller.commands == [] and not controller.stopped


def test_closing_main_window_hides_input_details_window(window):
    w, controller, _ = window
    w.input_contract_button.click()
    assert w.input_contract_dialog.isVisible()
    w.close()
    assert not w.input_contract_dialog.isVisible() and controller.stopped


@pytest.fixture
def official_window(monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(live_ui, "list_serial_ports", lambda: [])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    controller, backend = Controller(), Backend()
    backend.render_mano = lambda *_: backend.render(None)
    w = live_ui.LiveWindow(load_config(), backend, output_root=tmp_path,
                           controller=controller, upstream_checkout="explicit_mock_checkout")
    try:
        yield w, controller
    finally:
        w.close()
        app.processEvents()


def test_official_five_finger_controls_and_checkbox_refresh_is_read_only(official_window):
    w, controller = official_window
    assert not w.functional_selector.isHidden() and not w.functional_capture.isHidden()
    assert w.functional_selector.count() == 5
    assert not w.functional_capture.isEnabled()
    assert controller.commands == []
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    w.refresh()
    assert w.functional_capture.isEnabled() and w.functional_enabled.isChecked()
    for index, finger in enumerate(("index", "middle", "ring", "little", "thumb")):
        w.functional_selector.setCurrentIndex(index)
        w.functional_capture.click()
        assert controller.commands[-1] == ("capture_functional", {"finger": finger})
        assert "functional_" + finger in w._demonstrations
    assert "IP" in w.functional_selector.currentText() and "MCP/CMC" in w.functional_selector.currentText()
    count = len(controller.commands)
    controller.state["calibration"]["functional_enabled"] = False
    for _ in range(3):
        w.refresh()
    assert not w.functional_enabled.isChecked() and len(controller.commands) == count
    w.functional_enabled.click()
    assert controller.commands[-1] == ("set_functional_enabled", {"enabled": True})


def test_official_whole_hand_dispatch_requires_completed_three_pose_fit(official_window):
    w, controller = official_window
    assert not w.functional_hand_capture.isHidden()
    assert not w.functional_hand_capture.isEnabled()
    assert "24 秒" in w.functional_hand_capture.toolTip()
    w.functional_hand_capture.click()
    assert controller.commands == []
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["calibration"]["captured_steps"] = ["neutral", "fingers_down"]
    w.refresh()
    assert not w.functional_hand_capture.isEnabled()
    controller.state["calibration"]["captured_steps"].append("thumb_up")
    controller.state["calibration"]["calibrated_nodes"] = []
    w.refresh()
    assert not w.functional_hand_capture.isEnabled()  # Three captures without a fit do not suffice.
    controller.state["calibration"]["calibrated_nodes"] = ["12"]
    w.refresh()
    assert w.functional_hand_capture.isEnabled() and w.functional_capture.isEnabled()
    w.functional_hand_capture.click()
    assert controller.commands[-1] == ("capture_functional_hand", {})
    controller.state["runtime"]["connected"] = False
    w.refresh()
    assert not w.functional_hand_capture.isEnabled()


@pytest.mark.parametrize("phase,duration,demo,words", [
    ("preparing", 3., "neutral", ("准备 3 秒", "五指自然伸直")),
    ("collecting_neutral", 2., "neutral", ("静止 2 秒", "先不要弯曲")),
    ("moving_fingers", 8., "functional_hand_fingers", ("四指弯伸", "拇指放松", "2–3 次")),
    ("preparing_thumb", 3., "neutral", ("全手重新伸直", "准备 3 秒", "不采集运动")),
    ("moving_thumb", 8., "functional_hand_thumb", ("拇指末节 IP", "不校正 MCP / CMC", "2–3 次")),
])
def test_official_whole_hand_stages_are_explicit_cached_demos_and_cancellable(
        official_window, phase, duration, demo, words):
    w, controller = official_window
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["calibration"].update(captured_steps=["neutral", "fingers_down", "thumb_up"],
        active=dict(kind="functional_hand", phase=phase, remaining_s=duration / 2.))
    w.backend.fail = True  # All demonstration geometry must already be cached.
    w.refresh()
    assert all(word in w.instructions.text() for word in words)
    assert w.progress.value() == 50
    assert f"{duration / 2.:.1f} 秒" in w.progress.format()
    assert w._demo_key == demo and w.view.mesh.visible() and not w._render_error
    assert "动作示意" in w.view_state.text() and "不是你的实时手势" in w.view_state.text()
    assert not w.functional_hand_capture.isEnabled()
    assert not w.functional_capture.isEnabled() and not w.functional_selector.isEnabled()
    assert not w.functional_enabled.isEnabled() and w.clear.isEnabled()
    assert all(not button.isEnabled() for button in w.step_buttons.values())
    assert all(not button.isEnabled() for button in w.functional_buttons.values())
    assert w.cancel_capture.isEnabled()
    before = len(controller.commands)
    for button in [w.functional_hand_capture, w.functional_capture, w.functional_enabled,
                   *w.step_buttons.values()]:
        button.click()
    assert len(controller.commands) == before
    w.cancel_capture.click()
    assert controller.commands[-1] == ("cancel_capture", {})


def test_official_whole_hand_demo_bends_four_fingers_together_and_only_thumb_ip(official_window):
    w, _ = official_window
    four_fingers = np.zeros((16, 3))
    for mcp, pip in ((1, 2), (4, 5), (7, 8), (10, 11)):
        four_fingers[mcp, 2], four_fingers[pip, 2], four_fingers[pip + 1, 2] = .6, .8, .52
    thumb_ip = np.zeros((16, 3))
    thumb_ip[15, 2] = .8
    assert any(np.array_equal(angles, four_fingers) for angles in w.backend.rendered_angles)
    assert any(np.array_equal(angles, thumb_ip) for angles in w.backend.rendered_angles)
    assert w._demonstrations["functional_hand_thumb"] is w._demonstrations["functional_thumb"]
    assert sum(item[0] for item in live_ui._FUNCTIONAL_HAND_PHASES.values()) == 24.


@pytest.mark.parametrize("active", [
    dict(step="neutral", phase="preparing", remaining_s=2.),
    dict(step="neutral", kind="functional", finger="index", phase="moving", remaining_s=7.),
    dict(step="neutral", kind="functional_hand", phase="moving_thumb", remaining_s=0.),
])
def test_official_all_capture_types_disable_whole_hand_and_keep_cancel(official_window, active):
    w, controller = official_window
    make_measured(controller)
    controller.state["calibration"].update(captured_steps=["neutral", "fingers_down", "thumb_up"], active=active)
    w.refresh()
    assert not w.functional_hand_capture.isEnabled()
    assert not w.functional_capture.isEnabled() and not w.functional_enabled.isEnabled()
    assert w.clear.isEnabled() and w.cancel_capture.isEnabled()
    assert all(not button.isEnabled() for button in w.step_buttons.values())


def test_official_whole_hand_fitting_stays_busy_without_requesting_more_motion(official_window):
    w, controller = official_window
    make_measured(controller)
    controller.state["calibration"].update(
        captured_steps=["neutral", "fingers_down", "thumb_up"], profile_status="fitting_functional_hand",
        active=dict(kind="functional_hand", phase="moving_thumb", remaining_s=0.))
    w.refresh()
    assert "正在逐指计算" in w.instructions.text() and "无需继续弯伸" in w.instructions.text()
    assert "现在只弯伸" not in w.instructions.text()
    assert "正在计算" in w.progress.format() and "0.0 秒" not in w.progress.format()
    assert "正在计算" in w.functional_hand_feedback.text()
    assert w._demo_key == "neutral" and "动作示意" in w.view_state.text()
    assert not w.functional_hand_capture.isEnabled() and not w.functional_capture.isEnabled()
    assert not w.functional_enabled.isEnabled() and w.clear.isEnabled()
    assert w.cancel_capture.isEnabled()
    w.cancel_capture.click()
    assert controller.commands[-1] == ("cancel_capture", {})


@pytest.mark.parametrize("status,expected", [
    ("complete", "采集结束"), ("partial", "仅部分更新"),
    ("failed", "未更新，原有参数保留"), ("cancelled", "已取消，未更新参数"),
])
def test_whole_hand_terminal_refresh_clears_busy_instruction_on_repeated_runs(official_window, status, expected):
    w, controller = official_window
    make_measured(controller)
    calibration = controller.state["calibration"]
    calibration["captured_steps"] = ["neutral", "fingers_down", "thumb_up"]
    # Identical consecutive results exercise the Qt text cache, not just the
    # first run. Refresh must not send commands or alter source snapshots.
    for _ in range(2):
        calibration.update(profile_status="fitting_functional_hand",
            active=dict(kind="functional_hand", phase="moving_thumb", remaining_s=0.))
        w.refresh()
        assert "正在逐指计算" in w.instructions.text()
        assert "font-weight: bold" in w.instructions.styleSheet()
        calibration.update(active=None, profile_status="fitted",
                           functional_hand_result={"status": status, "fingers": {}})
        before = deepcopy(controller.state)
        w.refresh()
        assert controller.state == before and controller.commands == []
        for text in (w.instructions.text(), w.progress.format(), w.functional_hand_feedback.text()):
            assert "正在计算" not in text and "正在逐指计算" not in text
            assert "现在只弯伸" not in text
        assert expected in w.instructions.text()
        assert "只补采未更新的手指" in w.instructions.text()
        assert "font-weight: bold" not in w.instructions.styleSheet()
        assert w.functional_capture.isEnabled() and w.functional_hand_capture.isEnabled()
        assert not w.cancel_capture.isEnabled()
        terminal = w.instructions.text()
        controller.state["runtime"]["message"] = "ordinary later status update"
        w.refresh()
        assert w.instructions.text() == terminal


def test_single_retry_separates_current_partial_result_from_old_whole_hand_report(official_window):
    w, controller = official_window
    make_measured(controller)
    calibration = controller.state["calibration"]
    calibration.update(captured_steps=["neutral", "fingers_down", "thumb_up"],
        functional_fingers=["index", "middle"], functional_hand_result={"status": "partial", "fingers": {
            "index": {"status": "failed", "error": "旧一轮食指失败"},
            "middle": {"status": "updated", "updated_slots": [4, 5]}}},
        active=dict(kind="functional", finger="index", phase="moving", remaining_s=1.))
    w.refresh()
    assert "本次食指采集进行中" in w.functional_hand_feedback.text()
    assert "旧一轮" not in w.functional_hand_feedback.text()
    calibration.update(active=None, profile_status="fitted", functional_diagnostics={"index": {
        "1": {"status": "retained_previous", "reason": "MCP 转轴混合"},
        "2": {"status": "fitted"}}})
    controller.state["runtime"]["message"] = "食指校准已保存；非实测精度。"
    before = deepcopy(controller.state)
    w.refresh()
    assert controller.state == before and controller.commands == []
    assert "食指本次采集已结束" in w.instructions.text()
    assert "当前参数：关节 1 保留上次" in w.instructions.text()
    assert "当前参数：关节 2 已拟合" in w.instructions.text()
    assert "MCP 转轴混合" in w.instructions.text()
    assert "可单独补采该指，不必重做全手" in w.instructions.text()
    assert "已存参数不代表本次全部更新" in w.instructions.text()
    feedback = w.functional_hand_feedback.text()
    assert feedback.index("食指校准已保存") < feedback.index("以下为此前全手结果，不包含后续单指补采")
    assert "旧一轮食指失败" in feedback
    assert "现在弯伸" not in w.instructions.text()
    assert not w.cancel_capture.isEnabled() and w.functional_capture.isEnabled()
    w.functional_selector.setCurrentIndex(0)
    w.functional_capture.click()
    assert controller.commands == [("capture_functional", {"finger": "index"})]


@pytest.mark.parametrize("error,message", [
    ("中立参考不稳定", "弯曲方向未更新；原标定与其他手指保留，只需重采这根手指。"),
    (None, "已取消当前采集；已完成步骤保留。"),
])
def test_single_failed_or_cancelled_retry_does_not_claim_old_parameters_were_updated(official_window, error, message):
    w, controller = official_window
    make_measured(controller)
    calibration = controller.state["calibration"]
    calibration.update(functional_fingers=["thumb"],
        functional_diagnostics={"thumb": {"15": {"status": "fitted"}}},
        active=dict(kind="functional", finger="thumb", phase="moving", remaining_s=1.))
    w.refresh()
    calibration.update(active=None, profile_status="fitted")
    controller.state["runtime"].update(message=message, error=error)
    w.refresh()
    text = w.instructions.text()
    assert message in text and "本次采集已结束" in text
    assert "现在弯伸" not in text and "校准已保存" not in text
    assert "已存参数不代表本次全部更新" in text
    if error:
        assert error in text and "已拟合" not in text
    assert "已存参数" in w.functional_selector.itemText(4)
    assert w.functional_capture.isEnabled() and controller.commands == []


def test_base_fit_without_active_capture_stays_busy_then_returns_to_terminal_guidance(official_window):
    w, controller = official_window
    make_measured(controller)
    calibration = controller.state["calibration"]
    calibration.update(captured_steps=["neutral", "fingers_down", "thumb_up"],
        active=dict(step="thumb_up", phase="collecting", remaining_s=.1))
    w.refresh()
    calibration.update(active=None, profile_status="fitting")
    w.refresh()
    assert "正在计算基础标定" in w.instructions.text()
    assert "无需继续保持姿势" in w.instructions.text()
    assert "正在计算基础标定" in w.progress.format()
    assert all(not button.isEnabled() for button in w.step_buttons.values())
    assert not w.functional_capture.isEnabled() and not w.cancel_capture.isEnabled()
    calibration["profile_status"] = "fitted"
    controller.state["runtime"]["message"] = "基础标定拟合完成；非实物精度验收。"
    w.refresh()
    assert "基础标定拟合完成" in w.instructions.text()
    assert "正在计算" not in w.instructions.text()
    assert all(button.isEnabled() for button in w.step_buttons.values())
    assert controller.commands == []


def test_disconnect_and_new_session_drop_old_capture_instruction(official_window):
    w, controller = official_window
    make_measured(controller)
    controller.state["calibration"]["active"] = dict(kind="functional", finger="index", phase="moving", remaining_s=2.)
    w.refresh()
    assert "现在弯伸食指" in w.instructions.text()
    controller.state["runtime"]["connected"] = False
    controller.state["calibration"]["active"] = None
    w.refresh()
    assert "连接已结束，本次采集已停止" in w.instructions.text()
    controller.state["runtime"].update(connected=True, session_id="new-session")
    w.refresh()
    assert "连接已结束" not in w.instructions.text()
    assert "现在弯伸" not in w.instructions.text()
    assert "食指本次" not in w.functional_hand_feedback.text()
    assert controller.commands == []


def test_official_whole_hand_partial_result_does_not_promote_preexisting_entries(official_window):
    w, controller = official_window
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["calibration"].update(
        captured_steps=["neutral", "fingers_down", "thumb_up"],
        functional_fingers=["index", "middle", "ring", "little", "thumb"],
        functional_hand_result=dict(status="partial", fingers={
            "index": {"status": "updated", "updated_slots": [1, 2]},
            "middle": {"status": "partial", "updated_slots": [4], "rejected_slots": {"5": "运动不足"}},
            "ring": {"status": "failed", "error": "缺少中节观测"},
            "little": {"status": "updated", "updated_slots": [7, 8]},
            "thumb": {"status": "failed", "error": "拇指 IP 没有有效弯伸"},
        }))
    w.refresh()
    feedback = w.functional_hand_feedback.text()
    assert "仅部分更新" in feedback
    assert "食指：已更新" in feedback and "中指：部分更新" in feedback
    assert "无名指：未更新" in feedback and "拇指 IP：未更新" in feedback
    assert "关节 5 未更新：运动不足" in feedback and "缺少中节观测" in feedback
    assert "全部成功" not in feedback and "✓" not in feedback
    assert "✓" not in w.functional_hand_capture.text()
    assert all("已存参数" in w.functional_selector.itemText(i) for i in range(5))
    assert all("✓" not in w.functional_selector.itemText(i) for i in range(5))
    assert w.functional_hand_capture.isEnabled() and not w.cancel_capture.isEnabled()
    w.functional_selector.setCurrentIndex(4)
    w.functional_capture.click()
    assert controller.commands[-1] == ("capture_functional", {"finger": "thumb"})
    controller.state["calibration"].update(
        active=dict(kind="functional_hand", phase="preparing", remaining_s=3.))
    w.refresh()
    assert "进行中" in w.functional_hand_feedback.text()
    assert "食指：已更新" not in w.functional_hand_feedback.text()


@pytest.mark.parametrize("status,expected", [
    ("complete", "采集结束"), ("failed", "未更新，原有参数保留"),
    ("cancelled", "已取消，未更新参数"),
])
def test_official_whole_hand_terminal_result_is_separate_from_saved_parameters(official_window, status, expected):
    w, controller = official_window
    controller.state["calibration"].update(functional_fingers=["index"],
        functional_hand_result={"status": status, "fingers": {}})
    before = len(controller.commands)
    w.refresh()
    text = w.functional_hand_feedback.text()
    assert expected in text and "食指：已更新" not in text
    if status == "cancelled":
        assert "食指" not in text
    assert len(controller.commands) == before


@pytest.mark.parametrize("finger,name", [("index", "食指"), ("middle", "中指"), ("ring", "无名指"),
                                        ("little", "小指"), ("thumb", "拇指 IP")])
def test_official_functional_guidance_is_staged_demo_not_measured(official_window, finger, name):
    w, controller = official_window
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    for phase in ("preparing", "collecting_neutral", "moving"):
        controller.state["calibration"]["active"] = dict(
            step="neutral", kind="functional", finger=finger, phase=phase, remaining_s=1.)
        w.refresh()
        assert name in w.instructions.text()
        assert "动作示意" in w.view_state.text() and "不是你的实时手势" in w.view_state.text()
        assert not w.functional_capture.isEnabled() and not w.functional_enabled.isEnabled()
        if phase == "moving":
            assert "现在弯伸" in w.instructions.text() and "正在采集运动" in w.progress.format()
            assert w._demo_key == "functional_" + finger
        elif phase == "preparing":
            assert "暂未采集" in w.progress.format()
        else:
            assert "保持不动" in w.progress.format()
    if finger == "thumb":
        assert "不校正 MCP / CMC" in w.instructions.text()


def test_official_functional_residual_warning_does_not_hide_fresh_geometry(official_window):
    w, controller = official_window
    make_measured(controller)
    controller.state["pose"].update(mano_pose_rotvec=[[0., 0., 0.]] * 16,
        functional_adapter={"enabled": True, "applied_slots": [1, 2], "skipped_slots": {},
                            "residual_deg_by_slot": {"1": 24.}, "warnings": ["示例偏轴残差提示"]})
    controller.state["calibration"].update(functional_fingers=["index"], functional_enabled=True,
        functional_diagnostics={"index": {"1": {"status": "fitted", "holdout_p95_residual_deg": 23., "warnings": []}}})
    w.refresh()
    assert w.view.mesh.visible() and "偏轴残差提示" in w.view_state.text()
    assert "23.0°（非真值误差）" in w.details.toPlainText()
    assert "单指轴校正 + 关节约束，非实测精度" in w.details.toPlainText()
    controller.state["pose"]["functional_adapter"]["warnings"] = []
    w.refresh()
    assert "单指轴校正 + 关节约束" in w.view_state.text()


@pytest.mark.parametrize('saved_slots,trusted_slots,enabled', [
    ([1, 2, 4, 5, 7, 8, 10, 11, 15], [1, 2, 4, 5, 7, 8, 10, 11, 15], True),
    ([5, 7, 10, 15], [7, 10], True),
    ([1, 2, 4, 5, 7, 8, 10, 11, 15], [], False),
    ([], [], True),
])
def test_functional_coverage_summary_is_read_only_and_does_not_hide_base_pose(
        official_window, saved_slots, trusted_slots, enabled):
    w, controller = official_window
    make_measured(controller)
    pose = controller.state['pose']
    pose.update(mano_pose_rotvec=[[0., 0., 0.]] * 16,
        functional_calibration_coverage=dict(expected_slots=[1, 2, 4, 5, 7, 8, 10, 11, 15],
            enabled=enabled,
            reference_present_mask=[slot in saved_slots for slot in range(16)],
            correction_valid_mask=[slot in trusted_slots for slot in range(16)]))
    original = deepcopy(controller.state)
    w.refresh()
    detail = w.details.toPlainText()
    assert f'已存 {len(saved_slots)}/9；当前可信校正 {len(trusted_slots)}/9' in detail
    assert '功能校正与覆盖标记不等于独立人体精度验收' in detail
    assert '航向等待激励窗口不代表输入断流' in detail
    assert ('校正已关闭' in detail) == (not enabled)
    assert w.view.mesh.visible() and w.view.joints.visible()
    assert controller.commands == [] and controller.state == original


def test_official_soft_warnings_coexist_instead_of_overwriting(official_window):
    w, controller = official_window
    make_measured(controller)
    controller.state["pose"].update(mano_pose_rotvec=[[0., 0., 0.]] * 16,
        functional_adapter={"enabled": True, "applied_slots": [1], "warnings": ["偏轴提示"]},
        active_joint_limits=[dict(slot=1, axis="z", side="lower", value=-.25, limit=-.25)])
    controller.state["calibration"]["motion_diagnostics"] = {
        "warnings": ["转轴需核对"], "root": {"up_to_down_deg": 49., "opposite_axis_error_deg": 18.}}
    w.refresh()
    text = w.view_state.text()
    assert "单指轴校正存在偏轴残差提示" in text
    assert "标定方向需核对" in text and "49.0°" in text
    assert "姿态触及软件限位" in text
    assert w.view.mesh.visible()  # All three warnings remain non-gating.


def test_official_sidebar_fits_1080p_and_keeps_controls_scroll_accessible(official_window):
    w, _ = official_window
    w.ensurePolished()
    w.resize(1920, 1000)  # Reserve desktop/window decorations on a 1080p display.
    w.layout().activate()
    w.centralWidget().layout().activate()
    assert w.minimumSizeHint().height() <= 1000
    assert w.height() <= 1000
    scroll = w.sidebar_scroll
    assert scroll.widgetResizable()
    assert scroll.isAncestorOf(w.functional_selector)
    assert scroll.isAncestorOf(w.functional_hand_capture)
    assert scroll.isAncestorOf(w.functional_hand_feedback)
    assert scroll.isAncestorOf(w.cancel_capture)
    assert scroll.isAncestorOf(w.functional_capture)
    assert scroll.isAncestorOf(w.functional_enabled)
    assert scroll.isAncestorOf(w.details)


def test_official_ui_has_distinct_calibration_and_direct_mesh_path(monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(live_ui, "list_serial_ports", lambda: [])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    controller, backend, direct_calls = Controller(), Backend(), []
    def render_mano(pose, shape):
        direct_calls.append((pose, shape))
        return backend.render(None)
    backend.render_mano = render_mano
    w = live_ui.LiveWindow(load_config(), backend, output_root=tmp_path,
                           controller=controller, upstream_checkout="explicit_mock_checkout")
    try:
        assert "官方核心" in w.windowTitle()
        assert w.reanchor.isHidden() and all(b.isHidden() for b in w.functional_buttons.values())
        w.capture("fingers_down")
        assert "向上" in w.instructions.text()
        w.capture("thumb_up")
        assert "向下" in w.instructions.text() and "不是拇指侧上抬" in w.instructions.text()
        make_measured(controller)
        controller.state["pose"].update(
            algorithm="official_fsglove_raw2mano_with_explicit_sparse_adapter_v1",
            quality="not_accuracy_validated", upstream_fsglove_solver_executed=True,
            mano_pose_rotvec=[[0., 0., 0.]] * 16, shape_params=[.1] * 10)
        w.refresh()
        assert direct_calls and w.view.mesh.visible() and "官方 FSGlove" in w.view_state.text()
        controller.state["pose"].update(
            algorithm="official_fsglove_raw2mano_with_explicit_sparse_adapter_v2",
            missing_slot_policy="pip_coupled_dip_with_neutral_other_missing")
        controller.state["pose"]["anatomical_euler_rad"][3] = [0., 0., np.deg2rad(26.)]
        controller.state["calibration"]["motion_diagnostics"] = {
            "blocking": False, "warnings": ["转轴偏离同轴，建议重采。"],
            "root": {"up_to_down_deg": 48.5, "opposite_axis_error_deg": 48.2}}
        w.refresh()
        assert w.view.mesh.visible()  # Guidance must not reject otherwise fresh data.
        assert "标定方向需核对" in w.view_state.text()
        assert "48.5°" in w.view_state.text() and "48.2°" in w.view_state.text()
        assert "#fff3d9" in w.view_state.styleSheet()
        assert "末节推算：食指 26.0°" in w.joint_status.text()
        assert "非实测" in w.joint_status.text()
        controller.state["calibration"]["motion_diagnostics"]["warnings"] = []
        w.refresh()
        assert "官方 FSGlove" in w.view_state.text() and "标定方向需核对" not in w.view_state.text()
        controller.state["runtime"]["last_sample_age_s"] = 1.
        w.refresh()
        assert w.view.mesh.visible() and "输入中断" in w.view_state.text()
    finally:
        w.close()
        app.processEvents()


def test_start_idle_then_buttons_send_exact_runtime_contract(window):
    w, controller, _ = window
    assert controller.started and controller.commands == []
    assert "不是你的实时手势" in w.view_state.text()
    assert w.functional_hand_capture.isHidden() and w.functional_hand_feedback.isHidden()
    assert not w.functional_hand_capture.isEnabled()
    w.toggle_connection()
    assert controller.commands[-1] == ("connect", {"port": "/mock/imu"})
    controller.state["runtime"]["connected"] = True
    w.toggle_connection()
    assert controller.commands[-1] == ("disconnect", {})
    w.capture("thumb_up")
    assert controller.commands[-1] == ("capture", {"step": "thumb_up"})
    assert "不要只动拇指" in w.instructions.text()


def test_measured_stale_and_demo_states_cannot_be_confused(window):
    w, controller, _ = window
    make_measured(controller)
    w.refresh()
    assert "实时重建" in w.view_state.text() and w.view.mesh.visible()
    controller.state["runtime"]["last_sample_age_s"] = 1.
    w.refresh()
    assert "已冻结" in w.view_state.text() and w.view.mesh.visible()
    assert "非当前测量" in w.view_state.text()
    controller.state["calibration"]["active"] = dict(step="fingers_down", phase="preparing", remaining_s=2.)
    w.refresh()
    assert "动作示意" in w.view_state.text() and w.view.mesh.visible()
    assert "暂未采集" in w.progress.format()


def test_model_error_hides_previous_mesh_and_shows_failure(window):
    w, controller, backend = window
    backend.fail = True
    make_measured(controller)
    w.refresh()
    assert not w.view.mesh.visible()
    assert "渲染失败" in w.view_state.text()


def test_optional_joint_annotation_and_close_are_explicit(window):
    w, controller, _ = window
    w.toggle_joints(False)
    assert not w.view.joints.visible()
    w.toggle_joints(True)
    assert w.view.joints.visible()
    w.close()
    assert controller.stopped


def test_process_geometry_never_calls_mano_in_gui(window):
    w, controller, backend = window
    make_measured(controller)
    controller.renders_geometry = True
    controller.state["geometry"] = backend.render()
    backend.fail = True  # GUI rendering would raise; a worker mesh must not.
    w.refresh()
    assert w.view.mesh.visible()
    assert not w._render_error
    assert "提交网格时姿态年龄" in w.stats.text()


def test_missing_worker_mesh_hides_model_without_gui_fallback(window):
    w, controller, backend = window
    make_measured(controller)
    controller.renders_geometry = True
    controller.state.update(geometry=None, geometry_error="worker mesh failed")
    w.refresh()
    assert not w.view.mesh.visible()
    assert w._render_error == "worker mesh failed"
    controller.state["geometry"] = backend.render()
    controller.state["geometry_error"] = ""
    w.refresh()
    assert w.view.mesh.visible() and not w._render_error


def test_demonstrations_are_cached_before_runtime_starts(window):
    w, controller, backend = window
    backend.fail = True
    controller.state["calibration"]["active"] = dict(
        step="thumb_up", phase="preparing", remaining_s=2.)
    w.refresh()
    assert w._demo_key == "thumb_up"
    assert w.view.mesh.visible() and not w._render_error


def test_pose_expiring_during_injected_render_is_labelled_frozen_not_live(window, monkeypatch):
    w, controller, backend = window
    make_measured(controller)
    stamp = controller.state["pose"]["timestamp_s"]
    original = backend.render

    def expired_render(*args):
        geometry = original(*args)
        monkeypatch.setattr(live_ui.time, "monotonic", lambda: stamp + .5)
        return geometry

    backend.render = expired_render
    w.refresh()
    assert w.view.mesh.visible()
    assert w._last_pose is not None
    assert "解算延迟" in w.view_state.text() and "500 ms" in w.view_state.text()
    assert "不显示历史角度" in w.joint_status.text()


def test_unchanged_badge_style_and_details_do_not_rebuild(window, monkeypatch):
    w, _, _ = window
    calls = []
    monkeypatch.setattr(w.badges[1], "setStyleSheet", lambda value: calls.append(value))
    monkeypatch.setattr(w.details, "setPlainText", lambda value: calls.append(value))
    w.refresh()
    w.refresh()
    assert calls == []


def test_reference_button_only_available_after_calibration_and_has_clear_instruction(window):
    w, controller, _ = window
    assert not w.reanchor.isEnabled()
    make_measured(controller)
    w.refresh()
    assert w.reanchor.isEnabled()
    w.reanchor.click()
    assert controller.commands[-1] == ("reanchor_neutral", {})
    controller.state["calibration"]["active"] = dict(
        step="neutral", kind="neutral_reference", phase="preparing", remaining_s=3.)
    w.refresh()
    assert not w.reanchor.isEnabled()
    assert "保留手掌朝向和安装轴" in w.instructions.text()
    assert "动作示意" in w.view_state.text()


def test_limit_banner_is_warning_not_accuracy_green_and_stale_angles_are_hidden(window):
    w, controller, _ = window
    make_measured(controller)
    controller.state["pose"]["active_joint_limits"] = [
        dict(slot=7, axis="z", side="lower", value=-.25, limit=-.25, source="test")]
    w.refresh()
    assert "动作死区" in w.view_state.text()
    assert "软件下限" in w.joint_status.text()
    assert "#fff3d9" in w.view_state.styleSheet()
    controller.state["runtime"]["last_sample_age_s"] = 1.
    w.refresh()
    assert "软件下限" not in w.joint_status.text()


def test_functional_button_and_motion_phase_use_distinct_instructions(window):
    w, controller, _ = window
    assert not w.functional_buttons["little"].isEnabled()
    make_measured(controller)
    w.refresh()
    w.functional_buttons["little"].click()
    assert controller.commands[-1] == ("capture_functional", {"finger": "little"})
    controller.state["calibration"]["active"] = dict(
        step="neutral", kind="functional", finger="little", phase="collecting_neutral", remaining_s=2.)
    w.refresh()
    assert "先不要弯小指" in w.instructions.text()
    controller.state["calibration"]["active"].update(phase="moving", remaining_s=8.)
    w.refresh()
    assert "现在弯伸小指" in w.instructions.text()
    assert "现在弯伸" in w.progress.format()
    assert w._demo_key == "functional_little"
    assert "动作示意" in w.view_state.text()


def test_gap_recovery_feedback_does_not_present_stale_mesh_or_request_recalibration(window):
    w, controller, _ = window
    make_measured(controller)
    controller.state["pose"].update(pose_available=False, solver_success=False, reason="missing_or_stale_root")
    controller.state["orientation"] = {"nodes": {}, "gap_diagnostics": {"12": {
        "short_gaps": 1, "filter_resets": 0, "last_gap_s": .164, "last_action": "filter_preserved"}}}
    w.refresh()
    assert not w.view.mesh.visible()
    assert "暂不重做标定" in w.view_state.text()
    assert "短间隔保留滤波 1 次" in w.details.toPlainText()
    make_measured(controller)
    w.refresh()
    assert w.view.mesh.visible() and "稳定观察" in w.view_state.text()
    assert "实时重建" not in w.view_state.text()
    controller.state["pose"].update(pose_available=False, solver_success=False,
                                     reason="calibration_epoch_changed",
                                     calibration_rejections={"12": "filter_epoch_changed_recalibration_required"})
    w.refresh()
    assert w.view.mesh.visible() and "非当前测量" in w.view_state.text()
    assert "标定失效" in w.view_state.text()
