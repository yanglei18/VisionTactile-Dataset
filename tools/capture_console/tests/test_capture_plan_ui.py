"""The displayed Recorder allowlist and submitted intent share core rules."""
from copy import deepcopy

import pytest

from vt_capture_core.capture_plan import topics_fingerprint
from vt_capture_core.recipes import Recipe
from vt_capture_console import i18n
from vt_capture_console.ui.main_window import MainWindow
from vt_capture_console.viewmodels.capture_plan import capture_plan_view

from test_managed_workflow import ManagedBackend
from test_visual_layout import assert_inside, rect_in, settle, themed_window


@pytest.fixture(autouse=True)
def restore_language():
    previous = i18n.language()
    i18n.set_language("zh_CN", persist=False)
    yield
    i18n.set_language(previous, persist=False)


def test_plan_view_uses_configured_optional_streams_and_keeps_machine_values():
    recipe = Recipe("controller-config", duration_s=42, pose_hands=("left",), tactile_hands=("right",))
    topics = list(reversed(recipe.record_topics()))
    before = deepcopy(topics)
    view = capture_plan_view({"topics": topics}, duration_s=42, supported=True, required=True)
    assert not view.blocks_start
    assert view.start_parameters() == {
        "recipe": recipe.to_dict(), "expected_topics_fingerprint": topics_fingerprint(topics),
    }
    assert "左手" in view.summary and "右手" in view.summary
    assert "按可用数据录制" in view.status and "未连接设备不阻止开始" in view.status
    assert set(view.topics_text.splitlines()) == set(topics)
    i18n.set_language("en", persist=False)
    assert "left hand" in str(view.summary).lower() and "right hand" in str(view.summary).lower()
    assert "available data" in str(view.status)
    assert "Disconnected devices do not prevent" in str(view.status)
    assert view.start_parameters()["recipe"] == recipe.to_dict()
    assert topics == before


def test_camera_only_plan_does_not_require_tracker_topics():
    recipe = Recipe("controller-config", duration_s=30, cameras=("d405_1",), trackers=())
    topics = list(recipe.record_topics())
    view = capture_plan_view({"topics": topics}, duration_s=30, supported=True, required=True)
    assert not view.blocks_start
    assert view.start_parameters()["recipe"] == recipe.to_dict()
    assert "相机 1 台" in view.summary and "追踪器 0 个" in view.summary
    assert view.topics_text.splitlines() == topics


def test_unsupported_plan_shows_incomplete_camera_and_unknown_topic_without_partial_intent():
    topics = list(Recipe("base").record_topics())
    missing = topics.pop(0)
    topics.append("/custom/stream")
    view = capture_plan_view({"topics": topics}, duration_s=30, supported=True, required=True)
    assert view.blocks_start
    assert "设备数据主题配置不完整" in view.status and missing in view.status
    assert "/custom/stream" in view.status
    assert view.start_parameters() == {}


def test_legacy_backend_is_explicit_and_keeps_legacy_allowlist_semantics():
    view = capture_plan_view({"topics": ["/custom/stream"]}, duration_s=30,
                             supported=False, required=False)
    assert not view.blocks_start
    assert "未启用计划校验" in view.status
    assert view.topics_text == "/custom/stream"
    assert view.start_parameters() == {}


def test_v5_without_capture_plan_capability_cannot_fall_back():
    view = capture_plan_view({"topics": list(Recipe("base").record_topics())},
                             duration_s=30, supported=False, required=True)
    assert view.blocks_start
    assert "不可用" in view.status
    assert view.start_parameters() == {}


def _planned_backend():
    backend = ManagedBackend()
    backend.supports_capture_plan = True
    backend.supports_capture_policy = True
    backend.supports_details = True
    backend.value["capabilities"]["capture_plan"] = True
    backend.value["management"]["controller"]["topics"] = list(
        Recipe("controller-config", pose_hands=("right",), version=3).record_topics())
    return backend


def test_start_submits_current_plan_and_configuration_fingerprint(qapp):
    backend = _planned_backend()
    window = MainWindow(backend)
    try:
        window.duration.setValue(42)
        window._start_recording()
        params = backend.commands[0][2]
        assert params["recipe"] == Recipe("controller-config", duration_s=42, pose_hands=("right",), version=3).to_dict()
        assert params["expected_topics_fingerprint"] == topics_fingerprint(
            backend.value["management"]["controller"]["topics"])
        assert "右手" in window.capture_plan_summary.text()
        assert "正式采集" in window.capture_plan_status.text()
        assert params["admission_policy"] == "formal"
        assert window.capture_plan_summary.accessibleName() == "配置的数据类别"
    finally:
        window.close()


def test_invalid_or_unavailable_v5_plan_blocks_direct_start(qapp):
    backend = _planned_backend()
    backend.value["management"]["controller"]["topics"].pop(0)
    window = MainWindow(backend)
    try:
        assert not window.start_button.isEnabled()
        window._start_recording()
        assert backend.commands == []
        assert "/d405_1/color/image_raw" in window.operation_notice.toolTip()
        backend.supports_capture_plan = False
        backend.value["capabilities"]["capture_plan"] = False
        window.refresh()
        window._start_recording()
        assert backend.commands == []
        assert "不可用" in window.capture_plan_status.text()
    finally:
        window.close()


def test_legacy_start_is_unmodified_and_marked_unvalidated(qapp):
    backend = ManagedBackend()
    window = MainWindow(backend)
    try:
        assert "未启用计划校验" in window.capture_plan_status.text()
        window._start_recording()
        assert "recipe" not in backend.commands[0][2]
        assert "expected_topics_fingerprint" not in backend.commands[0][2]
    finally:
        window.close()


def test_demo_does_not_claim_real_plan_validation(window):
    assert "演示" in window.capture_plan_status.text()
    window._start_recording()
    assert window.backend.state == "RECORDING"


def test_explicit_both_hand_target_does_not_shrink_to_camera_only_controller(qapp):
    backend = _planned_backend()
    backend.value["management"]["controller"]["topics"] = list(Recipe("controller-config").record_topics())
    window = MainWindow(backend)
    try:
        for selector in window.capture_hand_modes.values():
            selector.setCurrentIndex(selector.findData("both"))
        assert not window.start_button.isEnabled()
        assert all(selector.isEnabled() for selector in window.capture_hand_modes.values())
        window._start_recording()
        assert backend.commands == []
        details = window.capture_plan_status.toolTip()
        for hand in ("left", "right"):
            for suffix in ("state", "imu/raw", "tactile/samples"):
                assert f"/gloves/{hand}/{suffix}" in details
        assert "cameras.formal_both_gloves_tactile.yaml" in details
        # Repair remains possible in the same view after a rejected selection.
        for selector in window.capture_hand_modes.values():
            selector.setCurrentIndex(selector.findData("configured"))
        assert window.start_button.isEnabled()
    finally:
        window.close()


def test_old_backend_requires_explicit_debug_choice_without_policy_downgrade(qapp):
    backend = _planned_backend()
    backend.supports_capture_policy = False
    backend.value["management"]["controller"]["topics"] = list(Recipe("controller-config").record_topics())
    window = MainWindow(backend)
    try:
        assert not window.start_button.isEnabled()
        window._start_recording()
        assert backend.commands == []
        assert "不支持正式采集" in window.capture_plan_status.text()
        window.capture_policy.setCurrentIndex(window.capture_policy.findData("debug"))
        window._start_recording()
        assert len(backend.commands) == 1
        assert "admission_policy" not in backend.commands[0][2]
        assert backend.commands[0][2]["recipe"]["version"] == 2
    finally:
        window.close()


def test_formal_pure_tactile_selection_has_no_implicit_pose_or_imu():
    recipe = Recipe("controller-config", duration_s=30, cameras=(), trackers=(), tactile_hands=("left",))
    view = capture_plan_view({"topics": list(recipe.record_topics())}, duration_s=30,
                             supported=True, required=True, admission_policy="formal",
                             hand_selection={"left": "tactile", "right": "none"})
    assert not view.blocks_start
    assert view.start_parameters()["recipe"] == recipe.to_dict()
    assert view.plan.required_topics() == ("/gloves/left/tactile/samples",)


@pytest.mark.parametrize("language", ["zh_CN", "en"])
def test_live_capture_plan_and_actions_fit_minimum_window(themed_window, qapp, language):
    window = themed_window(1080, 720, mode="live")
    window.backend.close()
    window.backend = _planned_backend()
    window.language_buttons[language].click()
    window.navigation.button(4).click()
    settle(window, qapp)
    assert (window.width(), window.height()) == (1080, 720)
    for widget in (window.capture_plan_summary, window.capture_plan_status,
                   window.capture_plan_topics, window.start_button, window.stop_button):
        assert widget.isVisible()
        assert_inside(widget, window.recording_console)
        assert_inside(widget, window)
    assert rect_in(window.capture_plan_status, window).bottom() < rect_in(window.elapsed, window).top()
    assert window.capture_plan_status.heightForWidth(window.capture_plan_status.width()) <= window.capture_plan_status.height()
    assert window.start_button.isEnabled()
