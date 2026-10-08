"""Compact visualization status, offline Qt only: no device or GL paint loop."""
from copy import deepcopy

import numpy as np
import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("pyqtgraph")
pytest.importorskip("OpenGL")
from PyQt5 import QtCore, QtWidgets

from test_live_ui import make_measured, official_window, quality_frame, window
from vt_fsglove12 import live_ui


@pytest.fixture(params=["window", "official_window"])
def compact_window(request, monkeypatch):
    injected = request.getfixturevalue(request.param)
    w, controller = injected[:2]
    w.timer.stop()
    clock = [100.]
    monkeypatch.setattr(live_ui.time, "monotonic", lambda: clock[0])
    make_measured(controller)
    if w.official:
        controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["orientation"] = quality_frame(clock[0])
    w.refresh()
    return w, controller, clock


def activate_layout(w):
    # Do not show the main window: geometry checks need no OpenGL context.
    w.ensurePolished()
    w.resize(1220, 900)
    w.layout().activate()
    w.centralWidget().layout().activate()


@pytest.mark.parametrize(("text", "expected"), [
    ("正在清除本次标定 · 已隐藏模型；完成后请从第1步开始", "清除"),
    ("动作示意 · 人工指定模型姿态，不是你的实时手势", "非实时手势"),
    ("官方 FSGlove 核心 · 12 路缺测适配 · 非完整 16 路观测", "含推算关节"),
    ("模型渲染更新失败 · 已冻结最近展示手形，非当前测量", "非当前测量"),
    ("已断开 · 已冻结最近展示手形，非当前测量", "已断开"),
    ("输入异常保护 · 已冻结最近展示手形，非当前测量", "输入保护"),
    ("输入中断 · 已冻结最近展示手形，非当前测量", "输入中断"),
    ("解算延迟 / 显示等待更新 · 显示冻结，非新测量", "非实时"),
    ("解算延迟 / 显示等待更新 · 结果已恢复，稳定观察中", "非实时"),
    ("连续显示参考 · 部分关节暂估 / 保持，非完整有效测量", "非完整有效测量"),
    ("等待首个可展示完整手形 · 暂无有效姿态", "等待"),
    ("实时重建 · 标定失效，需查看节点反馈", "失效"),
])
def test_compact_summary_preserves_measurement_category(text, expected):
    summary = live_ui.compact_view_status(text)
    assert expected in summary and "\n" not in summary
    assert len(summary) <= 40


def test_compact_summary_omits_fast_changing_ages_and_marks_replay():
    heading = "官方 FSGlove 核心 · 12 路缺测适配 · 非完整 16 路观测"
    summaries = [live_ui.compact_view_status(heading + f" · 源年龄 {age} ms")
                 for age in (5, 12, 30, 200)]
    assert len(set(summaries)) == 1
    assert "实时" in summaries[0]
    replay = live_ui.compact_view_status(heading, replay=True)
    assert "回放" in replay and "实时" not in replay


@pytest.mark.parametrize("warning", [
    "单指轴校正存在偏轴残差提示 · 仍继续显示", "标定方向需核对 · 上下转角 49.0°",
    "姿态触及软件限位 · 可能出现动作死区",
])
def test_compact_summary_marks_soft_warnings_beyond_first_line(warning):
    heading = "官方 FSGlove 核心 · 12 路缺测适配 · 非完整 16 路观测"
    assert "有提示" not in live_ui.compact_view_status(heading)
    assert "有提示" in live_ui.compact_view_status(heading + "\n" + warning)


def test_verbose_diagnostics_never_grow_status_bar_or_shrink_mesh(compact_window):
    w, controller, _clock = compact_window
    activate_layout(w)
    assert w.visual_status_bar.minimumHeight() == w.visual_status_bar.maximumHeight() == 40
    assert w.visual_status_bar.height() == 40
    assert not w.visual_status.wordWrap()
    assert "\n" not in w.visual_status.text()
    assert w.view.mesh.visible()
    dimensions = (w.view.width(), w.view.height(), w.minimumSizeHint().height())
    mesh = w.view.mesh
    vertices = w.view.mesh_data.vertexes().copy()
    state = deepcopy(controller.state)
    original_view, original_input = w.view_state.text(), w.input_status.text()
    for line_count in (1, 12, 80):
        w.view_state.setText(original_view + "\n" + "局部保持：完整诊断仍可查看。\n" * line_count)
        w.input_status.setText(original_input + "\n" + "输入详细诊断与历史计数。\n" * line_count)
        activate_layout(w)
        assert w.visual_status_bar.height() == 40
        assert (w.view.width(), w.view.height(), w.minimumSizeHint().height()) == dimensions
        assert w.view.mesh is mesh and mesh.visible()
        np.testing.assert_array_equal(w.view.mesh_data.vertexes(), vertices)
    assert controller.commands == [] and controller.state == state


def test_independent_input_warning_remains_visible_but_history_is_not_an_alarm(compact_window, monkeypatch):
    w, controller, _clock = compact_window
    monkeypatch.setattr(live_ui, "solver_input_status", lambda *args, **kwargs: ("当前输入需核对", ["5"]))
    w.refresh()
    assert "输入有提示" in w.visual_status.text()
    assert "#fff3d9" in w.visual_status.styleSheet()
    monkeypatch.setattr(live_ui, "solver_input_status", lambda *args, **kwargs: ("历史已过滤 999 条；当前正常", []))
    w.refresh()
    assert "输入有提示" not in w.visual_status.text()
    assert "999" in w.input_status.text() and "999" not in w.visual_status.text()
    assert controller.commands == []


def test_details_are_nonmodal_read_only_scrollable_and_reusable(compact_window):
    w, controller, _clock = compact_window
    assert w.visual_details_dialog is None
    expected_text = (w.view_state.text(), w.input_status.text())
    expected_style = (w.view_state.styleSheet(), w.input_status.styleSheet())
    state = deepcopy(controller.state)
    mesh = w.view.mesh
    vertices = w.view.mesh_data.vertexes().copy()
    camera = deepcopy(w.view.opts)
    w.visual_details_button.click()
    dialog = w.visual_details_dialog
    assert dialog.isVisible() and not dialog.isModal()
    assert dialog.windowModality() == QtCore.Qt.NonModal
    assert dialog.isAncestorOf(w.view_state) and dialog.isAncestorOf(w.input_status)
    scrolls = dialog.findChildren(QtWidgets.QScrollArea)
    assert any(scroll.widgetResizable() and scroll.isAncestorOf(w.view_state)
               and scroll.isAncestorOf(w.input_status) for scroll in scrolls)
    assert (w.view_state.text(), w.input_status.text()) == expected_text
    assert (w.view_state.styleSheet(), w.input_status.styleSheet()) == expected_style
    assert w.view.mesh is mesh and mesh.visible()
    np.testing.assert_array_equal(w.view.mesh_data.vertexes(), vertices)
    assert w.view.opts == camera
    dialog.close()
    assert not dialog.isVisible() and not controller.stopped
    w.visual_details_button.click()
    assert w.visual_details_dialog is dialog and dialog.isVisible()
    assert controller.commands == [] and controller.state == state


def test_open_details_track_live_input_and_frozen_measurement_and_close_with_main(compact_window):
    w, controller, clock = compact_window
    w.visual_details_button.click()
    dialog = w.visual_details_dialog
    original_view = w.view_state.text()
    original_input = w.input_status.text()
    clock[0] += .5
    controller.state["runtime"].update(connected=False, published_monotonic_s=clock[0],
                                       last_sample_age_s=.5)
    w.refresh()
    assert dialog.isVisible()
    assert w.view_state.text() != original_view and "已断开" in w.view_state.text()
    assert "非当前测量" in w.view_state.text()
    assert w.input_status.text() != original_input
    assert "未连接" in w.input_status.text()
    assert "已断开" in w.visual_status.text() or "冻结" in w.visual_status.text()
    assert "\n" not in w.visual_status.text() and w.visual_status_bar.height() == 40
    assert w.view.mesh.visible() and controller.commands == []
    w.close()
    assert controller.stopped and not dialog.isVisible()
