"""Graphical configuration with fake asynchronous runtime; no hardware access."""
from copy import deepcopy
import builtins
import os
from types import SimpleNamespace

import pytest
from PySide6.QtCore import QEvent, QPoint, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QFileDialog

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.main_window import MainWindow
from vt_capture_console.ui.widgets.workstation_settings import (
    BackendLogDialog, FIELDS, GROUPS, WorkstationSettingsDialog,
)


def configuration():
    result = {"disk_path": "/data", "quality_enabled": False, "quality_timeout_s": 321.,
              "quality_alignment_config": "/local/alignment.json", "stale_after_s": 2., "offline_after_s": 6.,
              "metadata_refs": [{"name": "right_calibration", "kind": "calibration", "path": "/local/calibration.json"}],
              "devices": [{"id": "right", "label": "right", "kind": "glove", "side": "right", "topic": "/gloves/right/state"}],
              "process_groups": []}
    for identifier, title, preset, hand in GROUPS:
        parameters = {"hand": hand} if hand else {}
        if preset == "glove":
            parameters.update(launch_gui="true", socket_path=f"/run/test/{hand}.sock", output_root=f"/data/{hand}")
        if preset == "trackers":
            parameters["frame_id"] = "custom_tracker_frame"
        result["process_groups"].append(dict(id=identifier, label=title, preset=preset, enabled=False,
                                             startup_timeout_s=90., parameters=parameters))
    return result


class Runtime:
    def __init__(self):
        self.configuration = configuration()
        self.status = {"state": "ready", "detail": "后台已连接", "last_apply": {"state": "idle", "detail": ""}}
        self.log_path = None
        self.ports = [{"path": "/dev/serial/by-id/test-imu", "label": "IMU 候选名称"}]
        self.enumerations = 0
        self.applications = []
        self.starts = 0
        self.accept = True
        self.detections = []
        self.discovery = None

    def serial_ports(self):
        self.enumerations += 1
        return deepcopy(self.ports)

    def apply(self, value, snapshot):
        if not self.accept:
            self.status.update(state="error", detail="外部后台不能由当前窗口重启")
            return False
        self.applications.append((deepcopy(value), deepcopy(snapshot)))
        self.status.update(state="applying", detail="正在异步保存", last_apply={"state": "pending", "detail": ""})
        return True

    def start(self):
        self.starts += 1
        self.status.update(state="starting", detail="后台正在启动")

    def detect_environment(self, value):
        self.detections.append(deepcopy(value))
        return self.discovery or SimpleNamespace(configuration=deepcopy(value),
                                                 findings=(), updated_fields=())


def dispose(widget, qapp):
    widget.close()
    widget.deleteLater()
    qapp.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


@pytest.fixture
def settings(qapp):
    runtime = Runtime()
    snapshot = DemoBackend().snapshot()
    dialog = WorkstationSettingsDialog(runtime, lambda: snapshot)
    dialog.show()
    qapp.processEvents()
    yield dialog, runtime, snapshot
    dispose(dialog, qapp)


def fill(dialog, identifier, key, value):
    editor = dialog.fields[(identifier, key)]
    if hasattr(editor, "setEditText"):
        editor.setEditText(value)
    elif hasattr(editor, "setValue"):
        editor.setValue(float(value))
    else:
        editor.setText(value)


def enable_complete(dialog, identifier):
    preset = next(g[2] for g in GROUPS if g[0] == identifier)
    dialog.enabled_checks[identifier].setChecked(True)
    for key, _, kind in FIELDS[preset]:
        fill(dialog, identifier, key, "120" if kind == "rate" else f"/local/{identifier}/{key}")


def test_open_and_cancel_only_enumerates_names_never_writes(qapp, monkeypatch):
    runtime = Runtime()
    original = deepcopy(runtime.configuration)

    def forbidden(*args, **kwargs):
        raise AssertionError("dialog must not open a serial device or settings file")

    monkeypatch.setattr(builtins, "open", forbidden)
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    enable_complete(dialog, "right_glove")
    dialog.cancel_button.click()
    assert runtime.configuration == original
    assert runtime.applications == []
    assert runtime.enumerations == 1
    assert not dialog.timer.isActive()
    dispose(dialog, qapp)


def test_save_all_six_profiles_preserves_non_form_configuration(settings):
    dialog, runtime, snapshot = settings
    for identifier, *_ in GROUPS:
        enable_complete(dialog, identifier)
    original = deepcopy(runtime.configuration)
    dialog.apply_button.click()
    assert len(runtime.applications) == 1
    value, sent_snapshot = runtime.applications[0]
    assert sent_snapshot == snapshot
    assert {k: v for k, v in value.items() if k != "process_groups"} == {
        k: v for k, v in original.items() if k != "process_groups"}
    groups = {g["id"]: g for g in value["process_groups"]}
    for identifier, *_ in GROUPS:
        assert groups[identifier]["enabled"]
        assert groups[identifier]["startup_timeout_s"] == 90.
    assert groups["trackers"]["parameters"]["frame_id"] == "custom_tracker_frame"
    assert groups["left_glove"]["parameters"]["socket_path"] == "/run/test/left.sock"
    assert groups["right_glove"]["parameters"]["output_root"] == "/data/right"
    assert groups["right_glove"]["parameters"]["output_rate_hz"] == "120"
    assert runtime.configuration == original  # Runtime, not this dialog, owns persistence.


def test_disabling_left_or_all_groups_keeps_parameters_and_stream_configuration(settings):
    dialog, runtime, _ = settings
    for identifier in ("left_glove", "left_tactile"):
        enable_complete(dialog, identifier)
        dialog.enabled_checks[identifier].setChecked(False)
    dialog.apply()
    result = runtime.applications[0][0]
    assert not any(group["enabled"] for group in result["process_groups"])
    left = next(group for group in result["process_groups"] if group["id"] == "left_glove")
    assert left["parameters"]["port"] == "/local/left_glove/port"
    assert result["devices"] == runtime.configuration["devices"]


def test_missing_required_fields_reports_actionable_error_without_submission(settings):
    dialog, runtime, _ = settings
    enable_complete(dialog, "right_glove")
    fill(dialog, "right_glove", "port", "")
    dialog.apply()
    assert runtime.applications == []
    assert "IMU 串口" in dialog.status_label.text()
    assert "取消启用" in dialog.status_label.text()
    assert dialog.apply_button.isEnabled()


def test_disabled_empty_groups_can_be_saved(settings):
    dialog, runtime, _ = settings
    dialog.apply()
    assert len(runtime.applications) == 1
    assert not any(g["enabled"] for g in runtime.applications[0][0]["process_groups"])


def finding(group_id, key, status, value="", candidates=()):
    return SimpleNamespace(group_id=group_id, key=key, status=status,
                           value=value, candidates=candidates)


def test_environment_detection_updates_draft_by_actual_group_id_and_cancel_does_not_save(qapp):
    runtime = Runtime()
    runtime.configuration["process_groups"][0]["id"] = "custom_cameras"
    original = deepcopy(runtime.configuration)
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    try:
        fill(dialog, "cameras", "config_path", "/old/computer/cameras.yaml")
        fill(dialog, "right_glove", "port", "/dev/manual-draft")
        dialog.enabled_checks["cameras"].setChecked(True)
        result = dialog.configuration()
        result["process_groups"][0]["parameters"].update(
            config_path="/new/computer/cameras.yaml", output_root="/new/data")
        result["scene_config_path"] = "/new/scene.yaml"
        result["disk_path"] = "/new/data"
        runtime.discovery = SimpleNamespace(
            configuration=result,
            updated_fields=(("custom_cameras", "config_path"),
                            ("custom_cameras", "output_root"),
                            ("", "scene_config_path"), ("", "disk_path")),
            findings=(finding("custom_cameras", "config_path", "ready", "/new/computer/cameras.yaml"),
                      finding("custom_cameras", "output_root", "default", "/new/data")),
        )
        enumerations = runtime.enumerations
        dialog.detect_environment_button.click()
        draft = dialog.configuration()
        assert dialog.fields[("cameras", "config_path")].text() == "/new/computer/cameras.yaml"
        assert draft["scene_config_path"] == "/new/scene.yaml"
        assert draft["disk_path"] == "/new/data"
        assert draft["quality_alignment_config"] == original["quality_alignment_config"]
        assert draft["metadata_refs"] == original["metadata_refs"]
        assert draft["devices"] == original["devices"]
        assert draft["process_groups"][1]["parameters"]["frame_id"] == "custom_tracker_frame"
        assert draft["process_groups"][0]["enabled"]
        assert not draft["process_groups"][1]["enabled"]
        assert dialog._value(dialog.fields[("right_glove", "port")]) == "/dev/manual-draft"
        assert runtime.detections[0]["process_groups"][0]["parameters"]["config_path"] == "/old/computer/cameras.yaml"
        assert runtime.enumerations == enumerations
        assert runtime.applications == [] and runtime.starts == 0
        assert runtime.configuration == original
        assert "已补全 4 项" in dialog.status_label.text()
        assert "保存并应用" in dialog.fields[("cameras", "output_root")].toolTip()
        dialog.cancel_button.click()
        assert runtime.configuration == original and runtime.applications == []
    finally:
        dispose(dialog, qapp)


def test_environment_summary_counts_only_enabled_required_fields_and_exposes_candidates(settings):
    dialog, runtime, _ = settings
    dialog.enabled_checks["cameras"].setChecked(True)
    runtime.discovery = SimpleNamespace(
        configuration=dialog.configuration(), updated_fields=(),
        findings=(finding("cameras", "config_path", "ambiguous", candidates=("/one.yaml", "/two.yaml")),
                  finding("cameras", "output_root", "default", "/new/data"),
                  finding("right_glove", "mano_assets", "missing"),
                  finding("right_glove", "port", "manual")),
    )
    dialog.detect_environment()
    assert "1 项必需配置" in dialog.status_label.text()
    tooltip = dialog.fields[("cameras", "config_path")].toolTip()
    assert "/one.yaml" in tooltip and "/two.yaml" in tooltip and "多个候选" in tooltip
    assert "未找到" in dialog.fields[("right_glove", "mano_assets")].toolTip()
    assert "手动选择" in dialog.fields[("right_glove", "port")].toolTip()
    assert "MANO" not in dialog.status_label.text()


@pytest.mark.parametrize("state", ["starting", "applying", "saving", "restarting", "stopping"])
def test_environment_detection_is_disabled_while_runtime_busy(settings, state):
    dialog, runtime, _ = settings
    runtime.status["state"] = state
    dialog.refresh_status()
    assert not dialog.detect_environment_button.isEnabled()
    dialog.detect_environment()
    assert runtime.detections == []


def test_environment_detection_without_runtime_capability_is_unavailable(qapp):
    runtime = Runtime()
    runtime.detect_environment = None
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    try:
        assert not dialog.detect_environment_button.isEnabled()
        dialog.detect_environment()
        assert runtime.applications == []
    finally:
        dispose(dialog, qapp)


def test_environment_detection_error_keeps_user_draft_and_configuration(settings):
    dialog, runtime, _ = settings
    fill(dialog, "cameras", "config_path", "/keep/my-choice.yaml")
    before = dialog.configuration()

    def fail(_value):
        raise OSError("test discovery failure")

    runtime.detect_environment = fail
    dialog.detect_environment()
    assert dialog.configuration() == before
    assert "检测未完成" in dialog.status_label.text()
    assert dialog.detect_environment_button.isEnabled()
    assert runtime.applications == [] and runtime.starts == 0


def test_camera_configuration_always_prompts_identity_review_and_browse_clears_old_finding(settings, monkeypatch):
    dialog, runtime, _ = settings
    editor = dialog.fields[("cameras", "config_path")]
    assert "相机序列号" in editor.toolTip()
    runtime.discovery = SimpleNamespace(
        configuration=dialog.configuration(), updated_fields=(),
        findings=(finding("cameras", "config_path", "ready", "/old/cameras.yaml"),),
    )
    dialog.detect_environment()
    assert "相机序列号" in editor.toolTip() and "/old/cameras.yaml" in editor.toolTip()
    monkeypatch.setattr(QFileDialog, "getOpenFileName", lambda *a, **k: ("/new/cameras.yaml", ""))
    dialog._browse(editor, "file", "配置")
    assert editor.text() == "/new/cameras.yaml"
    assert "相机序列号" in editor.toolTip() and "/old/cameras.yaml" not in editor.toolTip()


def test_environment_suggestions_are_submitted_only_on_save_and_detection_is_locked(settings):
    dialog, runtime, _ = settings
    dialog.enabled_checks["cameras"].setChecked(True)
    result = dialog.configuration()
    result["process_groups"][0]["parameters"].update(config_path="/local/cameras.yaml", output_root="/local/data")
    runtime.discovery = SimpleNamespace(
        configuration=result, findings=(),
        updated_fields=(("cameras", "config_path"), ("cameras", "output_root")),
    )
    dialog.detect_environment()
    assert not runtime.applications
    dialog.apply_button.click()
    assert len(runtime.applications) == 1
    camera = runtime.applications[0][0]["process_groups"][0]
    assert camera["parameters"]["config_path"] == "/local/cameras.yaml"
    assert camera["parameters"]["output_root"] == "/local/data"
    assert not dialog.detect_environment_button.isEnabled()
    dialog.detect_environment()
    assert len(runtime.detections) == 1


def test_environment_feedback_retranslates_without_redetection_or_losing_draft(settings, qapp):
    from vt_capture_console import i18n
    dialog, runtime, _ = settings
    runtime.discovery = SimpleNamespace(
        configuration=dialog.configuration(), updated_fields=(),
        findings=(finding("cameras", "config_path", "ready", "/local/相机.yaml"),),
    )
    dialog.detect_environment()
    fill(dialog, "left_glove", "port", "/dev/用户草稿")
    before = dialog.configuration()
    previous = i18n.language()
    try:
        i18n.set_language("en", persist=False)
        qapp.processEvents()
        assert dialog.detect_environment_button.text() == "Detect Local Environment"
        assert "Updated 0" in dialog.status_label.text()
        assert "Found locally" in dialog.fields[("cameras", "config_path")].toolTip()
        assert "camera serial numbers" in dialog.fields[("cameras", "config_path")].toolTip()
        assert "/local/相机.yaml" in dialog.fields[("cameras", "config_path")].toolTip()
        assert dialog.configuration() == before and len(runtime.detections) == 1
        assert runtime.applications == [] and runtime.starts == 0
    finally:
        i18n.set_language(previous, persist=False)


def test_relative_paths_are_rejected_without_backend_call(settings):
    dialog, runtime, _ = settings
    enable_complete(dialog, "cameras")
    fill(dialog, "cameras", "output_root", "relative/output")
    dialog.apply()
    assert runtime.applications == []
    assert "绝对路径" in dialog.status_label.text()


def test_serial_choices_send_path_not_display_label_and_preserve_manual_port(settings):
    dialog, runtime, _ = settings
    right = dialog.fields[("right_tactile", "port")]
    right.setCurrentIndex(1)
    left = dialog.fields[("left_tactile", "port")]
    left.setEditText("/dev/custom-unplugged")
    dialog.refresh_ports()
    assert dialog._value(right) == "/dev/serial/by-id/test-imu"
    assert dialog._value(left) == "/dev/custom-unplugged"
    assert runtime.enumerations == 2
    dialog.enabled_checks["right_tactile"].setChecked(True)
    dialog.apply()
    result = next(g for g in runtime.applications[0][0]["process_groups"] if g["id"] == "right_tactile")
    assert result["parameters"]["port"] == "/dev/serial/by-id/test-imu"


def test_file_and_directory_browse_cancel_do_not_clear_fields(settings, monkeypatch):
    dialog, _, _ = settings
    editor = dialog.fields[("cameras", "config_path")]
    editor.setText("/local/current.yaml")
    monkeypatch.setattr(QFileDialog, "getOpenFileName", lambda *a, **k: ("", ""))
    dialog._browse(editor, "file", "配置")
    assert editor.text() == "/local/current.yaml"
    monkeypatch.setattr(QFileDialog, "getOpenFileName", lambda *a, **k: ("/local/new.yaml", ""))
    dialog._browse(editor, "file", "配置")
    assert editor.text() == "/local/new.yaml"
    monkeypatch.setattr(QFileDialog, "getExistingDirectory", lambda *a, **k: "/local/output")
    dialog._browse(editor, "directory", "输出")
    assert editor.text() == "/local/output"


def test_apply_is_async_and_polls_background_success_without_resubmitting(settings, qapp):
    dialog, runtime, _ = settings
    dialog.apply()
    assert dialog._applying
    assert not dialog.apply_button.isEnabled()
    alive = []
    QTimer.singleShot(0, lambda: alive.append(True))
    qapp.processEvents()
    assert alive == [True]
    dialog.apply()
    assert len(runtime.applications) == 1
    runtime.status.update(state="ready", detail="配置已应用；尚未启动设备", last_apply={"state": "applied", "detail": "已保存"})
    dialog.refresh_status()
    assert not dialog._applying
    assert dialog.apply_button.isEnabled()
    assert "配置已应用" in dialog.status_label.text()


def test_background_error_keeps_editor_usable_and_does_not_retry(settings):
    dialog, runtime, _ = settings
    dialog.apply()
    runtime.status.update(state="error", detail="后台启动失败，请查看日志", last_apply={"state": "error", "detail": "failure"})
    for _ in range(3):
        dialog.refresh_status()
    assert "启动失败" in dialog.status_label.text()
    assert dialog.tabs.isEnabled()
    assert dialog.apply_button.isEnabled()
    assert len(runtime.applications) == 1


def test_synchronous_rejection_does_not_lock_dialog(settings):
    dialog, runtime, _ = settings
    runtime.accept = False
    dialog.apply()
    assert not dialog._applying
    assert dialog.apply_button.isEnabled()
    assert "外部后台" in dialog.status_label.text()
    assert runtime.applications == []


def test_close_pending_does_not_cancel_background_apply(settings):
    dialog, runtime, _ = settings
    dialog.apply()
    dialog.cancel_button.click()
    assert len(runtime.applications) == 1
    assert runtime.status["state"] == "applying"
    assert not dialog.timer.isActive()


def test_device_page_configuration_and_runtime_retry_are_explicit(qapp):
    backend = DemoBackend()
    backend.runtime = Runtime()
    window = MainWindow(backend)
    try:
        window.show()
        window.navigation.button(2).click()
        qapp.processEvents()
        assert window.configure_devices_button.isVisible()
        assert "配置设备" in window.setup_callout.text()
        assert "启用本次需要的模块" in window.setup_callout.text()
        assert backend.runtime.starts == 0
        window.configure_devices_button.click()
        assert window.settings_dialog.isVisible()
        assert backend.runtime.applications == []
        window.retry_backend_button.click()
        assert backend.runtime.starts == 1
        assert not window.retry_backend_button.isEnabled()
    finally:
        dispose(window, qapp)
    assert backend.runtime.applications == []


def test_non_desktop_and_demo_have_setup_callout_not_terminal_instructions(window):
    window.navigation.button(2).click()
    window.configure_devices_button.click()
    assert "桌面工作台入口" in window.setup_callout.text()
    assert "桌面工作台入口" in window.operation_notice.text()
    assert window.settings_dialog is None
    assert not window.retry_backend_button.isVisible()
    assert not window.backend_log_button.isVisible()


def test_readonly_log_is_bounded_and_does_not_follow_links(qapp, tmp_path):
    runtime = Runtime()
    path = tmp_path / "backend.log"
    path.write_text("x" * 70000 + "日志末尾", encoding="utf-8")
    runtime.log_path = path
    dialog = BackendLogDialog(runtime)
    try:
        assert dialog.text.isReadOnly()
        assert "64 KiB" in dialog.text.toPlainText()
        assert dialog.text.toPlainText().endswith("日志末尾")
        assert len(dialog.text.toPlainText().encode()) < 66000
        link = tmp_path / "link.log"
        link.symlink_to(path)
        runtime.log_path = link
        dialog.refresh()
        assert "不可读取" in dialog.text.toPlainText()
        fifo = tmp_path / "not-a-log"
        os.mkfifo(fifo)
        runtime.log_path = fifo
        dialog.refresh()
        assert "不是普通文件" in dialog.text.toPlainText()
    finally:
        dispose(dialog, qapp)


@pytest.mark.parametrize("size", [(1080, 720), (1366, 768), (1440, 900)])
def test_settings_dialog_is_scrollable_and_buttons_fit_real_theme(settings, qapp, size):
    from vt_capture_console.theme import stylesheet
    dialog, _, _ = settings
    font, style = QFont(qapp.font()), qapp.styleSheet()
    active = QFont()
    active.setFamilies(["Inter", "Noto Sans", "Noto Sans CJK SC"])
    active.setPointSize(10)
    qapp.setFont(active)
    qapp.setStyleSheet(stylesheet())
    try:
        dialog.resize(min(900, size[0] - 80), min(650, size[1] - 80))
        for index in range(4):
            dialog.tabs.setCurrentIndex(index)
            qapp.processEvents()
            for widget in (dialog.apply_button, dialog.cancel_button, dialog.refresh_ports_button,
                           dialog.detect_environment_button):
                assert widget.isVisible()
                assert dialog.rect().contains(widget.mapTo(dialog, QPoint(widget.width() - 1, widget.height() - 1)))
            assert dialog.tabs.currentWidget().widgetResizable()
            assert dialog.width() <= size[0] and dialog.height() <= size[1]
    finally:
        qapp.setFont(font)
        qapp.setStyleSheet(style)
