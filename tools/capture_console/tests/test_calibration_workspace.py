"""Guided calibration remains a thin, embedded client of the manager."""
from copy import deepcopy

from vt_capture_console.backend.calibration_client import CalibrationClient
from vt_capture_console.viewmodels.calibration import calibration_view
from vt_capture_console.ui.widgets.calibration_workspace import CalibrationWorkspace


def snapshot(state=None, **updates):
    value = dict(mode="live", instance_id="manager-a", capabilities={"calibration": True},
                 calibration=dict(enabled=True, busy=bool(state), cameras=[{"name": "d405_1"}],
                                  trackers=[{"role": "torso"}], session=None, active=[]))
    if state:
        value["calibration"]["session"] = dict(session_id="session-a", state=state, round=1,
                                              rounds=3, accepted=12, target=50,
                                              hint_code="HOLD_STILL", detail="", metrics={},
                                              preview=None, result=None)
    value["calibration"].update(updates)
    return value


class Backend:
    def __init__(self):
        self.value = snapshot()
        self.calls = []
        self.receipts = {}

    def submit(self, command, params):
        identifier = f"request-{len(self.calls)}"
        self.calls.append((command, deepcopy(params)))
        return identifier

    def operation_result(self, identifier):
        return self.receipts.get(identifier)


def test_new_workspace_uses_verified_board_defaults_through_language_switch_and_start(qapp):
    from vt_capture_console.i18n import set_language

    expected = dict(squares_x=12, squares_y=9, square_length_mm=15.,
                    marker_length_mm=11.25, dictionary="DICT_5X5_1000")
    backend = Backend()
    previous = CalibrationWorkspace(Backend())
    previous.squares_x.setValue(8)
    previous.square_length.setValue(35.)
    widget = CalibrationWorkspace(backend)
    try:
        for code in ("zh_CN", "en", "zh_CN"):
            set_language(code, persist=False)
            widget.refresh(backend.value)
            displayed = dict(squares_x=widget.squares_x.value(), squares_y=widget.squares_y.value(),
                             square_length_mm=widget.square_length.value(),
                             marker_length_mm=widget.marker_length.value(),
                             dictionary=widget.dictionary.currentData())
            assert displayed == expected
        widget.assembly_confirmed.setChecked(True)
        widget.primary_button.click()
        command, params = backend.calls[0]
        assert command == "calibration.start"
        assert params["board"] == expected
        assert params["config_path"] == ""
        assert previous.squares_x.value() == 8
        assert previous.square_length.value() == 35.
    finally:
        set_language("zh_CN", persist=False)
        widget.close()
        previous.close()
        widget.deleteLater()
        previous.deleteLater()


def test_shared_board_defaults_select_dictionary_by_identity(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import calibration_workspace as module

    defaults = dict(squares_x=10, squares_y=8, square_length_mm=25.,
                    marker_length_mm=18.75, dictionary="DICT_4X4_100")
    monkeypatch.setattr(module, "board_geometry_defaults", lambda: dict(defaults))
    backend = Backend()
    widget = CalibrationWorkspace(backend)
    try:
        widget.refresh(backend.value)
        assert widget.dictionary.currentData() == defaults["dictionary"]
        widget.assembly_confirmed.setChecked(True)
        widget.primary_button.click()
        assert backend.calls[0][1]["board"] == defaults
    finally:
        widget.close()
        widget.deleteLater()


def test_client_correlates_one_command_and_never_automatically_retries():
    backend = Backend()
    client = CalibrationClient(backend)
    client.observe(backend.value)
    assert client.start(dict(mode="guided", camera_name="d405_1", tracker_role="torso"))
    assert not client.start(dict(mode="guided"))
    assert backend.calls[0][1]["expected_manager_instance_id"] == "manager-a"
    backend.receipts["request-0"] = dict(request_id="request-0", state="accepted", code="OK")
    client.observe(backend.value)
    assert client.pending  # accepted receipt is not yet an observed session
    client.observe(snapshot("COLLECTING"))
    assert not client.pending
    assert len(backend.calls) == 1
    assert client.cancel()
    assert backend.calls[-1] == ("calibration.cancel", dict(
        expected_manager_instance_id="manager-a", session_id="session-a"))


def test_manager_restart_clears_pending_without_retry():
    backend = Backend()
    client = CalibrationClient(backend)
    client.observe(backend.value)
    client.start(dict(mode="guided"))
    changed = snapshot()
    changed["instance_id"] = "manager-b"
    client.observe(changed)
    assert not client.pending
    assert client.notice
    assert len(backend.calls) == 1


def test_another_clients_session_cannot_hide_our_rejected_receipt():
    backend = Backend()
    client = CalibrationClient(backend)
    client.observe(snapshot())
    client.start(dict(mode="guided"))
    client.observe(snapshot("COLLECTING"))
    assert client.pending
    backend.receipts["request-0"] = dict(request_id="request-0", state="rejected",
                                         code="calibration_busy", detail="Another client started a session.")
    client.observe(snapshot("COLLECTING"))
    assert not client.pending
    assert "Another client" in client.notice


def test_view_does_not_treat_candidate_or_waiting_publication_as_complete():
    candidate = calibration_view(snapshot("CANDIDATE", busy=False))
    waiting = calibration_view(snapshot("APPLIED_PENDING_PUBLICATION", busy=False))
    assert candidate.stage == waiting.stage == "result"
    assert not candidate.published and not waiting.published
    assert waiting.can_retry_publish
    assert not calibration_view(snapshot("COMPLETE", busy=False)).published


def test_completed_result_uses_matching_active_publication_and_identity():
    value = snapshot("COMPLETE", busy=False)
    value["calibration"]["session"]["camera_name"] = "d405_1"
    active = dict(source_session_id="session-a", camera_name="d405_1", quality_grade="TARGET",
                  revision="revision-a", topic="/calibration/d405_1/tracker_from_camera",
                  published=True, identity_valid=True)
    value["calibration"]["active"] = [active]
    view = calibration_view(value)
    assert view.published
    assert view.quality_summary == "质量达到目标要求"
    assert view.active_revision == "revision-a"
    active["identity_valid"] = False
    assert calibration_view(value).instruction == "标定已保存，当前设备身份尚未核验"
    active["published"] = False
    assert not calibration_view(value).published
    active["source_session_id"] = "another-session"
    view = calibration_view(value)
    assert not view.published
    assert not view.active_revision
    assert view.instruction == "标定已完成，当前发布状态待核实"


def test_restore_uses_current_and_previous_revision_with_manager_guard():
    backend = Backend()
    value = snapshot("COMPLETE", busy=False)
    value["calibration"]["session"]["camera_name"] = "d405_1"
    value["calibration"]["active"] = [dict(camera_name="d405_1", revision="revision-current",
                                              previous_revision="revision-old")]
    client = CalibrationClient(backend)
    client.observe(value)
    assert client.restore_previous()
    assert backend.calls == [("calibration.restore", dict(expected_manager_instance_id="manager-a",
        camera_name="d405_1", revision="revision-old", expected_active_revision="revision-current"))]
    assert not client.restore_previous()
    value["calibration"]["busy"] = True
    another = CalibrationClient(backend)
    another.observe(value)
    assert not another.restore_previous()


def test_restore_button_is_a_collapsed_secondary_result_action(qapp):
    backend = Backend()
    widget = CalibrationWorkspace(backend)
    value = snapshot("COMPLETE", busy=False)
    value["calibration"]["session"]["camera_name"] = "d405_1"
    value["calibration"]["active"] = [dict(camera_name="d405_1", revision="current", previous_revision="old")]
    widget.refresh(value)
    assert widget.restore_button.isEnabled()
    assert not widget.restore_button.isVisibleTo(widget)
    widget.details_toggle.setChecked(True)
    assert widget.restore_button.isVisibleTo(widget)
    widget.restore_button.click()
    assert backend.calls[0][0] == "calibration.restore"
    widget.close()


def test_help_describes_embedded_camera_calibration(qapp):
    from PySide6.QtWidgets import QLabel
    from vt_capture_console.backend.demo_backend import DemoBackend
    from vt_capture_console.ui.widgets.help_dialog import HelpDialog
    help_dialog = HelpDialog(DemoBackend())
    text = "\n".join(label.text() for label in help_dialog.findChildren(QLabel))
    assert "相机–追踪器标定嵌入主程序" in text
    assert "标定是独立窗口" not in text
    help_dialog.close()


def test_cancelling_keeps_progress_and_does_not_offer_another_cancel():
    view = calibration_view(snapshot("CANCELLING"))
    assert view.stage == "collect"
    assert not view.can_cancel
    assert not view.can_start


def test_marker_length_error_explains_disabled_action(qapp):
    widget = CalibrationWorkspace(Backend())
    widget.refresh(snapshot())
    widget.assembly_confirmed.setChecked(True)
    widget.marker_length.setValue(50.)
    assert not widget.primary_button.isEnabled()
    assert widget.notice.text() == "标记边长必须小于方格边长"
    widget.close()


def test_missing_capability_and_offline_backend_disable_commands():
    backend = Backend()
    client = CalibrationClient(backend)
    value = snapshot()
    value["capabilities"] = {}
    client.observe(value)
    assert not client.start(dict(mode="guided"))
    value["capabilities"]["calibration"] = True
    value["mode"] = "demo"
    client.observe(value)
    assert not client.start(dict(mode="guided"))
    assert backend.calls == []


def test_embedded_form_requires_confirmation_and_preserves_drafts(qapp):
    from PySide6.QtWidgets import QWidget
    backend = Backend()
    parent = QWidget()
    widget = CalibrationWorkspace(backend, parent)
    widget.refresh(backend.value)
    assert not widget.isWindow()
    assert not widget.primary_button.isEnabled()
    widget.assembly_confirmed.setChecked(True)
    assert widget.primary_button.isEnabled()
    widget.square_length.setValue(42.5)
    widget.dictionary.setCurrentIndex(widget.dictionary.findData("DICT_4X4_100"))
    widget.refresh(backend.value)
    assert widget.square_length.value() == 42.5
    widget.primary_button.click()
    assert backend.calls[0][0] == "calibration.start"
    params = backend.calls[0][1]
    assert params["board"]["square_length_mm"] == 42.5
    assert params["board"]["dictionary"] == "DICT_4X4_100"
    assert params["camera_name"] == "d405_1" and params["tracker_role"] == "torso"
    assert not widget.primary_button.isEnabled()
    widget.close()


def test_ui_refresh_uses_one_hint_and_collapsed_details(qapp):
    backend = Backend()
    widget = CalibrationWorkspace(backend)
    value = snapshot("COLLECTING")
    value["calibration"]["session"]["detail"] = "private technical diagnostics"
    before = deepcopy(value)
    widget.refresh(value)
    assert widget.instruction.text() == "保持静止，等待自动采样"
    assert "private" not in widget.instruction.text()
    assert not widget.details_toggle.isChecked()
    assert widget.progress.value() == 12 and widget.progress.maximum() == 50
    assert value == before
    assert widget.cancel_button.isEnabled()
    widget.close()


def test_language_switch_preserves_identity_paths_and_form_values(qapp):
    from vt_capture_console.i18n import set_language
    backend = Backend()
    widget = CalibrationWorkspace(backend)
    widget.refresh(backend.value)
    widget.config_path.setText("/用户/配置.yaml")
    try:
        set_language("en", persist=False)
        widget.refresh(backend.value)
        assert widget.primary_button.text() == "Start calibration"
        assert widget.camera.currentData() == "d405_1"
        assert widget.config_path.text() == "/用户/配置.yaml"
    finally:
        set_language("zh_CN", persist=False)
        widget.close()


def test_guided_start_does_not_submit_saved_offline_bag_selection(qapp):
    from vt_capture_core.protocol import CALIBRATION_PROTOCOL_VERSION, encode_message
    backend = Backend()
    widget = CalibrationWorkspace(backend)
    widget.refresh(snapshot())
    widget.bags.addItem("/用户/历史数据/bag")
    widget.assembly_confirmed.setChecked(True)
    widget.primary_button.click()
    command, params = backend.calls[0]
    assert params["bag_paths"] == []
    encode_message(dict(version=CALIBRATION_PROTOCOL_VERSION, request_id="request-0", command=command, params=params))
    widget.close()


def test_imported_config_does_not_submit_invalid_disabled_board_draft(qapp):
    from vt_capture_core.protocol import CALIBRATION_PROTOCOL_VERSION, encode_message
    backend = Backend()
    widget = CalibrationWorkspace(backend)
    widget.refresh(snapshot())
    widget.marker_length.setValue(90.)
    widget.config_path.setText("/用户/有效配置.yaml")
    widget.assembly_confirmed.setChecked(True)
    widget.primary_button.click()
    command, params = backend.calls[0]
    encode_message(dict(version=CALIBRATION_PROTOCOL_VERSION, request_id="request-0", command=command, params=params))
    assert params["config_path"] == "/用户/有效配置.yaml"
    assert params["board"] == dict(squares_x=12, squares_y=9, square_length_mm=15.,
                                  marker_length_mm=11.25, dictionary="DICT_5X5_1000")
    assert widget.marker_length.value() == 90.
    widget.close()


def test_rendered_calibration_chinese_labels_and_roles_do_not_leak_english(qapp):
    import re
    from PySide6.QtWidgets import QLabel, QAbstractButton
    from vt_capture_console.i18n import set_language

    set_language("zh_CN", persist=False)
    backend = Backend()
    value = snapshot(trackers=[{"role": "torso"}, {"role": "left_wrist"}, {"role": "right_wrist"}])
    before = deepcopy(value)
    widget = CalibrationWorkspace(backend)
    try:
        widget.refresh(value)
        widget.board_toggle.setChecked(True)
        widget.resize(1080, 720)
        widget.show()
        qapp.processEvents()
        rendered = [item.text() for kind in (QLabel, QAbstractButton)
                    for item in widget.findChildren(kind) if item.isVisibleTo(widget)]
        assert "相机–追踪器标定" in rendered
        assert not [text for text in rendered if re.search(r"[A-Za-z]", text)]
        assert [widget.tracker.itemText(i) for i in range(3)] == ["躯干", "左腕", "右腕"]
        assert widget.dictionary.currentText() == "5 × 5 · 1000 个标记"
        assert widget.dictionary.currentData() == "DICT_5X5_1000"
        assert widget.square_length.suffix() == " 毫米"
        assert widget.camera.currentData() == "d405_1"
        assert value == before
    finally:
        widget.close()
        widget.deleteLater()


def test_calibration_language_switch_localizes_roles_without_changing_commands_or_data(qapp):
    from vt_capture_console.i18n import set_language

    backend = Backend()
    value = snapshot(trackers=[{"role": "torso"}, {"role": "left_wrist"}, {"role": "right_wrist"}])
    widget = CalibrationWorkspace(backend)
    try:
        set_language("zh_CN", persist=False)
        widget.refresh(value)
        widget.tracker.setCurrentIndex(1)
        widget.square_length.setValue(42.5)
        widget.config_path.setText("/用户/Board 配置.yaml")
        set_language("en", persist=False)
        widget.refresh(value)
        assert widget.tracker.currentText() == "Left wrist"
        assert widget.tracker.currentData() == "left_wrist"
        assert widget.dictionary.currentText() == "5 × 5 · 1000 markers"
        assert widget.square_length.suffix() == " mm"
        set_language("zh_CN", persist=False)
        widget.refresh(value)
        assert widget.tracker.currentText() == "左腕"
        assert widget.square_length.value() == 42.5
        assert widget.config_path.text() == "/用户/Board 配置.yaml"
        widget.assembly_confirmed.setChecked(True)
        widget.primary_button.click()
        assert backend.calls[0][1]["tracker_role"] == "left_wrist"
        assert backend.calls[0][1]["board"]["dictionary"] == "DICT_5X5_1000"
    finally:
        set_language("zh_CN", persist=False)
        widget.close()
        widget.deleteLater()


def test_calibration_result_labels_translate_but_topic_and_report_paths_remain_opaque(qapp):
    from vt_capture_console.i18n import set_language

    backend = Backend()
    value = snapshot("COMPLETE", busy=False)
    session = value["calibration"]["session"]
    session.update(camera_name="d405_1", tracker_role="right_wrist", result=dict(
        topic="/calibration/d405_1/tracker_from_camera", report_path="/记录/Board/report.json"))
    before = deepcopy(value)
    widget = CalibrationWorkspace(backend)
    try:
        set_language("zh_CN", persist=False)
        widget.refresh(value)
        assert widget.device_summary.text() == "d405_1 · 右腕追踪器"
        assert "发布主题: /calibration/d405_1/tracker_from_camera" in widget.details.toPlainText()
        set_language("en", persist=False)
        widget.refresh(value)
        assert widget.device_summary.text() == "d405_1 · Right Wrist Tracker"
        assert "Published topic: /calibration/d405_1/tracker_from_camera" in widget.details.toPlainText()
        assert "/记录/Board/report.json" in widget.details.toPlainText()
        assert value == before
        assert backend.calls == []
    finally:
        set_language("zh_CN", persist=False)
        widget.close()
        widget.deleteLater()
