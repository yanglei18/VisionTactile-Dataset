"""Stable-reference guidance stays consistent through motion and retries."""
from copy import deepcopy

import pytest

from test_live_ui import official_window, make_measured


@pytest.mark.parametrize("kind,finger", [("functional_hand", None), ("functional", "index"),
                                         ("functional", "thumb")])
def test_reference_wait_uses_elapsed_budget_without_flashing_or_starting_motion(
        official_window, kind, finger):
    window, controller = official_window
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    calibration = controller.state["calibration"]
    calibration["captured_steps"] = ["neutral", "fingers_down", "thumb_up"]
    values, instructions = [], []
    for elapsed, status, remaining in ((1., "collecting", 1.), (3., "unstable", 2.),
                                       (5., "collecting", .2), (9., "unstable", 2.)):
        calibration["active"] = dict(kind=kind, finger=finger, step="neutral",
            phase="collecting_neutral", remaining_s=remaining, reference_waiting=True,
            reference_wait_elapsed_s=elapsed, reference_wait_limit_s=12.,
            reference_status=status, unstable_slots=[2, 5])
        before = deepcopy(controller.state)
        window.refresh()
        values.append(window.progress.value())
        instructions.append(window.instructions.text())
        assert "等待稳定参考" in window.progress.format()
        assert "连续 2 秒" in window.capture_guidance.text()
        assert "先不要弯曲" in window.capture_guidance.text()
        assert window._demo_key == "neutral"
        assert window.cancel_capture.isEnabled()
        assert not window.functional_capture.isEnabled()
        assert controller.state == before and controller.commands == []
    assert values == sorted(values) and len(set(values)) == len(values)
    assert len(set(instructions)) == 1
    calibration["active"].update(reference_waiting=False,
        phase="moving_fingers" if kind == "functional_hand" else "moving", remaining_s=8.)
    window.refresh()
    assert "现在" in window.capture_guidance.text()
    assert "正在采集运动" in window.progress.format()
    assert "等待稳定参考" not in window.progress.format()
    assert controller.commands == []


def test_reference_timeout_explains_failure_without_reporting_old_axes_as_success(official_window):
    window, controller = official_window
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    reason = "未取得连续稳定参考；本次未更新，可单独补采。"
    controller.state["calibration"].update(functional_fingers=["index", "middle"],
        functional_hand_result=dict(status="failed", reason=reason, fingers={}))
    window.refresh()
    assert reason in window.functional_hand_feedback.text()
    assert "原有参数保留" in window.functional_hand_feedback.text()
    assert "食指：已更新" not in window.functional_hand_feedback.text()


def test_fitted_axes_are_not_presented_as_completed_physical_validation(official_window):
    window, controller = official_window
    make_measured(controller)
    controller.state["pose"]["mano_pose_rotvec"] = [[0., 0., 0.]] * 16
    controller.state["calibration"]["functional_hand_result"] = dict(status="complete",
        fingers={name: dict(status="updated", updated_slots=["2"], rejected_slots={})
                 for name in ("index", "middle", "ring", "little", "thumb")})
    window.refresh()
    text = window.functional_hand_feedback.text()
    assert "请验证弯伸与回位" in text
    assert text.count("请验证弯伸与回位") == 1
    assert text.count("：已更新") == 5


def test_structured_fit_failure_is_shown_once_per_joint(official_window):
    window, _ = official_window
    reason = "关节 15 弯曲轴未通过模型验收；本次不替换该关节参数。"
    text = window._functional_hand_result_text(dict(status="failed", fingers={
        "thumb": dict(status="failed", error="本次未更新。" + reason,
                      rejected_slots={"15": reason})}))
    assert text.count(reason) == 1
    assert "拇指 IP：未更新" in text
