"""Clear-calibration UI regressions with injected controllers, no hardware."""
from copy import deepcopy
from pathlib import Path
import runpy

import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("pyqtgraph")
pytest.importorskip("OpenGL")
from PyQt5 import QtWidgets

from vt_fsglove12 import live_ui
from test_live_ui import load_config


_HELPERS = runpy.run_path(str(Path(__file__).with_name("test_live_ui.py")))
Controller, Backend = _HELPERS["Controller"], _HELPERS["Backend"]
make_measured = _HELPERS["make_measured"]


@pytest.fixture(params=[False, True], ids=["legacy", "official"])
def clear_window(request, monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(live_ui, "list_serial_ports", lambda: [])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    controller, backend = Controller(), Backend()
    controller.latest_snapshot = lambda: controller.state
    backend.render_mano = lambda angles, *args: backend.render(angles)
    window = live_ui.LiveWindow(load_config(), backend, output_root=tmp_path,
        port="/mock/imu", controller=controller,
        upstream_checkout="explicit_mock_checkout" if request.param else None)
    window.timer.stop()
    try:
        yield window, controller, backend
    finally:
        window.close()
        app.processEvents()


def populated(controller):
    make_measured(controller)
    controller.state["pose"].update(profile_version=7, mano_pose_rotvec=[[0., 0., 0.]] * 16)
    controller.state["calibration"].update(
        captured_steps=["neutral", "fingers_down", "thumb_up"], profile_status="fitted",
        clear_generation=3, functional_fingers=["index", "middle", "ring", "little", "thumb"],
        functional_enabled=False,
        functional_diagnostics={"little": {"8": {"status": "fitted", "warnings": ["old functional warning"]}}},
        functional_hand_result={"status": "complete", "fingers": {"little": {"status": "updated"}}},
        motion_diagnostics={"warnings": ["old motion warning"]})


def acknowledge(controller, generation=4):
    # Intentionally retain an old successful pose/geometry and old functional
    # feedback: an explicitly empty calibration must never render that state.
    controller.state["calibration"].update(
        active=None, captured_steps=[], calibrated_nodes=[],
        profile_status="not_calibrated", clear_generation=generation)


def refresh_read_only(window, controller):
    before = deepcopy(controller.state)
    window.refresh()
    assert controller.state == before


def assert_empty_ui(window, *, pending):
    assert window._step == "neutral"
    assert window._last_pose is None
    assert not window._render_error and window._render_times == []
    assert window._pose_age_ms is None
    assert "第1步" in window.instructions.text()
    assert window._steps[0]["instruction"] in window.instructions.text()
    assert window.progress.value() == 0
    assert all("✓" not in button.text() for button in window.step_buttons.values())
    assert all("✓" not in button.text() for button in window.functional_buttons.values())
    assert all("已存参数" not in window.functional_selector.itemText(i)
               for i in range(window.functional_selector.count()))
    assert "已更新" not in window.functional_hand_feedback.text()
    assert "已拟合" not in window.details.toPlainText()
    assert "old functional warning" not in window.details.toPlainText()
    assert "old motion warning" not in window.details.toPlainText()
    assert not window.functional_capture.isEnabled()
    assert not window.functional_hand_capture.isEnabled()
    assert not window.cancel_capture.isEnabled()
    if pending:
        assert window._clear_pending
        assert window._demo_key is None and not window.view.mesh.visible()
        assert "正在清除" in window.view_state.text()
        assert not window.clear.isEnabled()
        assert all(not button.isEnabled() for button in window.step_buttons.values())
    else:
        assert not window._clear_pending
        assert window._demo_key == "neutral" and window.view.mesh.visible()
        assert "动作示意" in window.view_state.text() and "不是你的实时手势" in window.view_state.text()
        assert window.clear.isEnabled()
        assert all(button.isEnabled() for button in window.step_buttons.values())


@pytest.mark.parametrize("active", [
    None,
    {"step": "thumb_up", "phase": "preparing", "remaining_s": 2.},
    {"step": "fingers_down", "phase": "collecting", "remaining_s": 1.},
    {"kind": "neutral_reference", "step": "neutral", "phase": "preparing", "remaining_s": 2.},
    {"kind": "functional", "finger": "little", "phase": "moving", "remaining_s": 6.},
    {"kind": "functional_hand", "phase": "moving_fingers", "remaining_s": 6.},
    {"kind": "functional_hand", "phase": "moving_thumb", "remaining_s": 0.},
])
def test_clear_interrupts_connected_capture_and_drops_stale_ui_until_generation_ack(clear_window, active):
    window, controller, backend = clear_window
    if not window.official and active and active.get("kind") == "functional_hand":
        pytest.skip("whole-hand capture exists only in the official UI")
    populated(controller)
    controller.state["calibration"]["active"] = deepcopy(active)
    if active and active.get("kind") == "functional_hand" and active["remaining_s"] == 0.:
        controller.state["calibration"]["profile_status"] = "fitting_functional_hand"
    refresh_read_only(window, controller)
    assert window.clear.isEnabled()
    style, layout_count = window.styleSheet(), window.centralWidget().layout().count()
    window._render_error = "old render failure"
    window._render_times = [1., 2.]
    before = deepcopy(controller.state)
    window.clear.click()
    assert controller.commands == [("clear_calibration", {})]
    assert controller.state == before
    assert_empty_ui(window, pending=True)
    for _ in range(3):
        refresh_read_only(window, controller)
        window.clear.click()
        window.step_buttons["neutral"].click()
        window.functional_hand_capture.click()
        assert_empty_ui(window, pending=True)
        assert controller.commands == [("clear_calibration", {})]
    # Empty fields alone are not acknowledgment of this requested clear.
    acknowledge(controller, generation=3)
    refresh_read_only(window, controller)
    assert_empty_ui(window, pending=True)
    acknowledge(controller, generation=4)
    backend.fail = True  # Only the already cached neutral demo may be shown.
    refresh_read_only(window, controller)
    assert_empty_ui(window, pending=False)
    assert window.styleSheet() == style
    assert window.centralWidget().layout().count() == layout_count
    assert controller.commands == [("clear_calibration", {})]
    window.step_buttons["neutral"].click()
    assert controller.commands[-1] == ("capture", {"step": "neutral"})


def test_external_empty_calibration_snapshot_resets_last_step_and_feedback(clear_window):
    window, controller, backend = clear_window
    populated(controller)
    controller.state["calibration"]["active"] = {
        "step": "thumb_up", "phase": "preparing", "remaining_s": 2.}
    refresh_read_only(window, controller)
    assert window._step == "thumb_up"
    assert window._demo_key == "thumb_up"
    acknowledge(controller)
    backend.fail = True
    refresh_read_only(window, controller)
    assert_empty_ui(window, pending=False)
    assert controller.commands == []
    # The unchanged empty snapshot does not reset/re-upload the demo each tick.
    geometry_calls = []
    original = window.view.set_geometry
    window.view.set_geometry = lambda geometry: (geometry_calls.append(geometry), original(geometry))
    refresh_read_only(window, controller)
    assert not geometry_calls


def test_queue_rejection_does_not_pretend_calibration_was_cleared(clear_window):
    window, controller, _ = clear_window
    populated(controller)
    refresh_read_only(window, controller)
    token, demo = window._last_pose, window._demo_key
    before = deepcopy(controller.state)
    controller.submit = lambda *args, **kwargs: False
    window.clear.click()
    assert not window._clear_pending
    assert window._last_pose == token and window._demo_key == demo
    assert controller.state == before
    assert "操作队列正忙" in window.status.text()
    assert all("✓" in button.text() for button in window.step_buttons.values())


def test_synchronous_clear_ack_is_compared_with_pre_submit_generation(clear_window):
    window, controller, _ = clear_window
    populated(controller)
    refresh_read_only(window, controller)

    def clear_immediately(name, **kwargs):
        controller.commands.append((name, kwargs))
        acknowledge(controller)
        return True

    controller.submit = clear_immediately
    window.clear.click()
    assert controller.commands == [("clear_calibration", {})]
    assert_empty_ui(window, pending=False)


def test_clear_is_unavailable_while_disconnected(clear_window):
    window, controller, _ = clear_window
    assert not window.clear.isEnabled()
    window.clear.click()
    window.clear_calibration()
    assert controller.commands == [] and not window._clear_pending
