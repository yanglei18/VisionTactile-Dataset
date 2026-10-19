"""Real Qt language switching with memory-only backends and isolated settings."""

from copy import deepcopy
import re

import pytest
from PySide6.QtCore import QEvent, QSettings
from PySide6.QtTest import QSignalSpy
from PySide6.QtWidgets import QLabel

from test_device_controls import DeviceBackend
from test_managed_workflow import ManagedBackend
from test_workstation_settings_ui import Runtime
from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.backend.scene_demo import DemoSceneBackend
from vt_capture_console.i18n import initialize, language, set_language
from vt_capture_console.ui.main_window import MainWindow


_CHINESE = re.compile(r"[\u3400-\u9fff]")
_PAGE_IDS = (2, 0, 3, 4, 1)


@pytest.fixture(autouse=True)
def restore_language(qapp, isolated_presentation_settings):
    set_language("zh_CN", persist=False)
    yield
    set_language("zh_CN", persist=False)


@pytest.fixture
def make_window(qapp):
    windows = []

    def create(backend=None):
        window = MainWindow(
            backend or DemoBackend(), scene_backend=DemoSceneBackend()
        )
        windows.append(window)
        window.motion_toggle.setChecked(False)
        window.timer.stop()
        window.show()
        qapp.processEvents()
        return window

    yield create
    for window in reversed(windows):
        window.close()
        window.deleteLater()
    qapp.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    qapp.processEvents()


def _switch(window, code, qapp):
    window.language_buttons[code].click()
    qapp.processEvents()
    assert language() == code
    assert window.language_buttons[code].isChecked()


def _headers(table):
    return [table.horizontalHeaderItem(i).text() for i in range(table.columnCount())]


def _display_surface(window):
    help_dialog = window.help_dialog
    return {
        "tabs": [window.navigation.button(i).text() for i in _PAGE_IDS],
        "tab_accessibility": [window.navigation.button(i).accessibleName() for i in _PAGE_IDS],
        "help": window.help_button.text(),
        "help_accessibility": window.help_button.accessibleName(),
        "help_tooltip": window.help_button.toolTip(),
        "motion_accessibility": window.motion_toggle.accessibleName(),
        "motion_description": window.motion_toggle.accessibleDescription(),
        "language_accessibility": window.language_switch.accessibleName(),
        "configure": window.configure_devices_button.text(),
        "process_headers": _headers(window.device_controls.table),
        "stream_headers": _headers(window.device_table),
        "trial_headers": _headers(window.trial_table),
        "batch_placeholder": window.batch_name.placeholderText(),
        "task_placeholder": window.task_name.placeholderText(),
        "operator_accessibility": window.operator_name.accessibleName(),
        "help_title": help_dialog.windowTitle(),
        "help_close_accessibility": help_dialog.close_button.accessibleName(),
        "help_scroll_accessibility": help_dialog.scroll.accessibleName(),
        "help_content": [label.text() for label in help_dialog.findChildren(QLabel)],
    }


def _all_strings(value):
    if isinstance(value, dict):
        for child in value.values():
            yield from _all_strings(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            yield from _all_strings(child)
    else:
        yield value


def test_five_pages_help_headers_and_accessibility_switch_in_place(make_window, qapp):
    window = make_window()
    window.navigation.button(1).click()
    window.help_button.click()
    qapp.processEvents()
    help_dialog = window.help_dialog
    before = _display_surface(window)
    assert before["tabs"] == ["设备", "预览", "空间", "录制", "档案"]
    original_pages = [window.pages.widget(i) for i in range(window.pages.count())]

    _switch(window, "en", qapp)
    english = _display_surface(window)
    assert english["tabs"] == ["Devices", "Preview", "Spatial", "Recording", "Archive"]
    assert english["tab_accessibility"] == english["tabs"]
    assert english["configure"] == "Configure Devices"
    assert english["help_title"] == "Help / About"
    assert english["process_headers"] == ["Process Group", "Type", "Status", "Ownership", "PID"]
    assert english["trial_headers"][0] == "Trial"
    assert english["help_tooltip"] != before["help_tooltip"]
    assert english["motion_description"] != before["motion_description"]
    assert not [text for text in _all_strings(english) if _CHINESE.search(text)]
    assert window.help_dialog is help_dialog and help_dialog.isVisible()
    assert window.pages.currentIndex() == 1
    assert [window.pages.widget(i) for i in range(window.pages.count())] == original_pages

    _switch(window, "zh_CN", qapp)
    assert _display_surface(window) == before
    assert window.help_dialog is help_dialog and help_dialog.isVisible()
    assert window.pages.currentIndex() == 1


def test_open_configuration_retranslates_without_applying_or_losing_edits(make_window, qapp):
    backend = DemoBackend()
    backend.runtime = Runtime()
    window = make_window(backend)
    window.navigation.button(2).click()
    window.configure_devices_button.click()
    qapp.processEvents()
    dialog = window.settings_dialog
    dialog.timer.stop()
    dialog.tabs.setCurrentIndex(3)
    camera_path = dialog.fields[("cameras", "config_path")]
    camera_path.setText("/local/相机配置.json")
    serial = dialog.fields[("left_glove", "port")]
    serial.setEditText("/dev/serial/by-id/用户设备")
    before_tabs = [dialog.tabs.tabText(i) for i in range(dialog.tabs.count())]
    before_placeholder = serial.lineEdit().placeholderText()
    before_configuration = deepcopy(backend.runtime.configuration)
    enumerations = backend.runtime.enumerations

    _switch(window, "en", qapp)
    assert window.settings_dialog is dialog and dialog.isVisible()
    assert dialog.windowTitle() == "Configure Devices · Local Workstation"
    assert dialog.apply_button.text() == "Save and Apply"
    assert dialog.cancel_button.text() == "Cancel"
    assert dialog.tabs.tabText(0) == "Cameras"
    assert dialog.tabs.tabText(3) == "Left hand"
    assert serial.lineEdit().placeholderText() != before_placeholder
    assert not _CHINESE.search(serial.lineEdit().placeholderText())
    assert not _CHINESE.search(serial.lineEdit().accessibleName())
    assert dialog.tabs.currentIndex() == 3
    assert camera_path.text() == "/local/相机配置.json"
    assert serial.currentText() == "/dev/serial/by-id/用户设备"

    _switch(window, "zh_CN", qapp)
    assert [dialog.tabs.tabText(i) for i in range(dialog.tabs.count())] == before_tabs
    assert serial.lineEdit().placeholderText() == before_placeholder
    assert dialog.tabs.currentIndex() == 3
    assert camera_path.text() == "/local/相机配置.json"
    assert serial.currentText() == "/dev/serial/by-id/用户设备"
    assert backend.runtime.configuration == before_configuration
    assert backend.runtime.enumerations == enumerations
    assert backend.runtime.applications == [] and backend.runtime.starts == 0


def test_language_switch_preserves_drafts_stream_and_batch_selection(make_window, qapp):
    backend = ManagedBackend()
    backend.supports_details = True
    window = make_window(backend)
    window.filter.setCurrentIndex(window.filter.findData("tracker"))
    window.refresh()
    tracker = [device["id"] for device in window.devices.values() if device["kind"] == "tracker"][-1]
    window._select(tracker)
    window.navigation.button(4).click()
    editors = {
        "batch_name": "保留批次 batch-β",
        "trial_name": "用户试次 trial-02",
        "task_name": "抓取 cup",
        "operator_name": "实验员 OP-7",
    }
    for name, value in editors.items():
        getattr(window, name).setText(value)
    window.duration.setValue(73)
    batch_id = window.batch_selector.currentData()
    batch_label = window.batch_selector.currentText()
    filter_changed = QSignalSpy(window.filter.currentIndexChanged)
    batch_changed = QSignalSpy(window.batch_selector.currentIndexChanged)
    before_backend = deepcopy(backend.value)

    for code in ("en", "zh_CN"):
        _switch(window, code, qapp)
        assert window.pages.currentIndex() == 4
        assert window.filter.currentData() == "tracker"
        assert window.selected_id == tracker
        assert window.batch_selector.currentData() == batch_id
        assert window.batch_selector.currentText() == batch_label
        assert {name: getattr(window, name).text() for name in editors} == editors
        assert window.duration.value() == 73
        assert backend.value == before_backend
        assert backend.commands == []
    assert filter_changed.count() == 0
    assert batch_changed.count() == 0


def test_queued_recording_request_survives_language_switch_without_resubmission(make_window, qapp):
    backend = ManagedBackend()
    window = make_window(backend)
    window.navigation.button(4).click()
    window.start_button.click()
    assert window.pending_request
    request = window.pending_request
    pending = (
        window.pending_action, window.pending_instance_id, window.pending_controller_id
    )
    commands = deepcopy(backend.commands)
    state = deepcopy(backend.value["recording"])
    operation = deepcopy(backend.latest_operation)
    window.navigation.button(0).click()

    for code in ("en", "zh_CN"):
        _switch(window, code, qapp)
        assert window.pages.currentIndex() == 0
        assert window.pending_request == request
        assert (window.pending_action, window.pending_instance_id, window.pending_controller_id) == pending
        assert not window.start_button.isEnabled()
        assert backend.commands == commands
        assert backend.value["recording"] == state
        assert backend.latest_operation == operation
    assert len(commands) == 1 and commands[0][1] == "capture.start"


def test_queued_device_request_survives_language_switch_without_resubmission(make_window, qapp):
    backend = DeviceBackend()
    window = make_window(backend)
    window.navigation.button(2).click()
    controls = window.device_controls
    controls.stop_button.click()
    assert controls.pending_request
    request = controls.pending_request
    group_id = controls.selected_id
    manager_id = controls.pending_instance_id
    commands = deepcopy(backend.commands)
    process_control = deepcopy(backend.value["process_control"])

    for code in ("en", "zh_CN"):
        _switch(window, code, qapp)
        assert window.pages.currentIndex() == 2
        assert controls.pending_request == request
        assert controls.pending_instance_id == manager_id
        assert controls.selected_id == group_id
        assert not controls.stop_button.isEnabled()
        assert backend.commands == commands
        assert backend.value["process_control"] == process_control
    assert len(commands) == 1 and commands[0][0] == "device.stop"


def test_active_recording_and_user_metadata_survive_language_switch(make_window, qapp):
    backend = DemoBackend()
    window = make_window(backend)
    window.navigation.button(4).click()
    window.trial_name.setText("用户试次 untouched")
    window.task_name.setText("用户任务 untouched")
    window.operator_name.setText("操作员 OP-8")
    window.start_button.click()
    assert backend.state == "RECORDING"
    backend.clock = lambda: backend.started + 8.0
    window.refresh()
    trials = deepcopy(backend.trials)
    window.navigation.button(1).click()

    for code in ("en", "zh_CN"):
        _switch(window, code, qapp)
        assert window.pages.currentIndex() == 1
        assert backend.state == "RECORDING"
        assert backend.trials == trials
        assert window.stop_button.isEnabled()
        assert "00:08" in window.recording_nav_status.text()
        assert window.trial_table.item(0, 0).text() == "用户试次 untouched"
        assert window.task_name.text() == "用户任务 untouched"
        assert window.operator_name.text() == "操作员 OP-8"


def test_language_choice_persists_across_close_and_reopen(make_window, qapp):
    first = make_window()
    _switch(first, "en", qapp)
    settings = QSettings(first._ui_settings.fileName(), QSettings.Format.IniFormat)
    assert settings.value("ui/language") == "en"
    first.close()

    second = make_window()
    assert language() == "en"
    assert second.navigation.button(2).text() == "Devices"
    assert second.language_buttons["en"].isChecked()
    _switch(second, "zh_CN", qapp)
    second.close()

    third = make_window()
    assert language() == "zh_CN"
    assert third.navigation.button(2).text() == "设备"
    assert third.language_buttons["zh_CN"].isChecked()
    settings.sync()
    assert settings.value("ui/language") == "zh_CN"


def test_unknown_saved_locale_falls_back_to_chinese(tmp_path):
    settings = QSettings(str(tmp_path / "unknown-locale.ini"), QSettings.Format.IniFormat)
    settings.setValue("ui/language", "unsupported-locale")
    set_language("en", persist=False)
    initialize(settings)
    assert language() == "zh_CN"
    # Reading an unsupported preference is not an implicit settings rewrite.
    assert settings.value("ui/language") == "unsupported-locale"
