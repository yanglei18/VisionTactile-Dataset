"""Persistent local-storage status must not masquerade as IMU/pose failure."""
from copy import deepcopy
import time

import numpy as np
import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("pyqtgraph")
pytest.importorskip("OpenGL")

from test_live_ui import make_measured, official_window


WARNING = ("本地记录已暂停；实时采集、解算和 ROS 发布继续，"
           "暂停后的数据未写入本地文件。原因：当前会话达到容量限额。")


def live_state(controller):
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["orientation"] = dict(timestamp_s=time.monotonic(),
        nodes={str(node): dict(age_s=.001, sample_index=5, epoch=0) for node in range(1, 13)})


def paused(**extra):
    return dict(enabled=False, warning=WARNING, errors=["session limit"], policy={}, storage={},
                recorded_samples=50, received_samples=100, profile_saved=None, **extra)


def test_local_warning_is_independent_of_runtime_error_input_color_and_hand(official_window):
    window, controller = official_window
    live_state(controller)
    window.refresh()
    badges = {key: badge.styleSheet() for key, badge in window.badges.items()}
    input_style = window.input_status.styleSheet()
    colors = window.view.mesh_data.vertexColors().copy()
    controller.state["runtime"]["local_recording"] = paused()
    original = deepcopy(controller.state)
    window.refresh()
    assert WARNING == window.local_recording_status.text()
    assert not window.local_recording_status.isHidden()
    assert controller.state["runtime"]["error"] is None
    assert {key: badge.styleSheet() for key, badge in window.badges.items()} == badges
    assert window.input_status.styleSheet() == input_style
    np.testing.assert_array_equal(window.view.mesh_data.vertexColors(), colors)
    assert window.view.mesh.visible() and controller.commands == []
    assert controller.state == original


def test_warning_survives_messages_missing_ipc_fields_and_counter_updates_without_repainting(
        official_window, monkeypatch):
    window, controller = official_window
    live_state(controller)
    text_calls, visibility_calls = [], []
    original_text, original_visibility = (window.local_recording_status.setText,
                                          window.local_recording_status.setVisible)

    def set_text(value):
        text_calls.append(value)
        original_text(value)

    def set_visible(value):
        visibility_calls.append(value)
        original_visibility(value)

    monkeypatch.setattr(window.local_recording_status, "setText", set_text)
    monkeypatch.setattr(window.local_recording_status, "setVisible", set_visible)
    for index in range(12):
        controller.state["runtime"].update(message=f"capture update {index}", error=None)
        if index % 2:
            controller.state["runtime"].pop("local_recording", None)
        else:
            data = paused()
            data["received_samples"] += index * 100
            controller.state["runtime"]["local_recording"] = data
        window.refresh()
        assert window.local_recording_status.text() == WARNING
        assert not window.local_recording_status.isHidden()
    assert text_calls == [WARNING]
    assert visibility_calls == [True]
    assert controller.commands == []


def test_disconnect_keeps_loss_notice_and_new_enabled_recording_clears_it(official_window):
    window, controller = official_window
    live_state(controller)
    controller.state["runtime"]["local_recording"] = paused()
    window.refresh()
    controller.state["runtime"].update(connected=False, message="disconnected")
    controller.state["runtime"].pop("local_recording")
    window.refresh()
    assert window.local_recording_status.text() == WARNING
    live_state(controller)
    controller.state["runtime"].update(session_id="new-explicit-connection",
        local_recording=dict(enabled=True, warning="", errors=[], profile_saved=None))
    window.refresh()
    assert window.local_recording_status.text() == ""
    assert window.local_recording_status.isHidden()
    assert controller.commands == []


def test_unsaved_profile_is_explicit_without_clearing_in_memory_hand(official_window):
    window, controller = official_window
    live_state(controller)
    local = paused()
    local["profile_saved"] = False
    controller.state["runtime"]["local_recording"] = local
    window.refresh()
    text = window.local_recording_status.text()
    assert WARNING in text and "当前标定未保存到本地文件" in text
    assert window.view.mesh.visible()
    for _ in range(3):
        window.refresh()
        assert window.local_recording_status.text() == text


def test_disabled_without_warning_has_honest_fallback_and_does_not_use_runtime_error(official_window):
    window, controller = official_window
    live_state(controller)
    controller.state["runtime"].update(local_recording={"enabled": False, "warning": ""},
                                        error="unrelated calibration feedback")
    window.refresh()
    assert "暂停后的数据未写入本地文件" in window.local_recording_status.text()
    assert "unrelated calibration" not in window.local_recording_status.text()
    assert "unrelated calibration" in window.status.text()
