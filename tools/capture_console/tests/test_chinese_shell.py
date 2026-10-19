"""Authored shell labels follow the selected language without altering data."""
from copy import deepcopy

import pytest
from PySide6.QtCore import QEvent
from PySide6.QtWidgets import QLabel

from test_device_controls import DeviceBackend
from vt_capture_console.backend.scene_demo import DemoSceneBackend
from vt_capture_console.i18n import set_language
from vt_capture_console.ui.main_window import MainWindow
from vt_capture_console.ui.widgets.capture_stage import CaptureStage
from vt_capture_console.ui.widgets.preview import PreviewWidget


@pytest.fixture(autouse=True)
def chinese_language():
    set_language("zh_CN", persist=False)
    yield
    set_language("zh_CN", persist=False)


def _operation_headings(window):
    return {item.text() for item in window.findChildren(QLabel)
            if item.objectName() == "operationEyebrow"}


def test_shell_and_recording_headings_retranslate_without_changing_page_ids(window, qapp):
    window.timer.stop()
    pages = [window.pages.widget(index) for index in range(window.pages.count())]
    headings = {0: "02 / 实时预览", 1: "05 / 数据档案", 2: "01 / 设备设置",
                3: "03 / 空间视图", 4: "04 / 录制"}
    for index, heading in headings.items():
        window._navigate(index)
        assert window.page_eyebrow.text() == heading
        assert not any(char.isascii() and char.isalpha() for char in window.workspace_caption.text())
    assert window.overview.text() == "数据流列表"
    assert window.mode_badge.text() == "●  演示"
    assert window.windowTitle() == "VisionTactile · 采集工作台 [演示]"
    assert window.segments.button(2).text().split()[0] == "追踪器"
    assert window.filter.itemData(2) == "tracker"
    assert window.device_table.horizontalHeaderItem(5).text() == "数据主题"
    assert _operation_headings(window) == {
        "01 / 任务设置", "02 / 录制控制", "已用时长 / 计划时长", "所选试次", "设备 / 采集前检查"}

    window._set_language("en")
    assert window.page_eyebrow.text() == "04 / RECORDING"
    assert window.overview.text() == "DATA STREAMS"
    assert window.mode_badge.text() == "●  DEMO"
    assert window.windowTitle() == "VisionTactile · Immersive Studio [DEMO]"
    assert window.segments.button(2).text().split()[0] == "Tracker"
    assert window.device_table.horizontalHeaderItem(5).text() == "Topic"
    assert _operation_headings(window) == {
        "01 / SESSION SETUP", "02 / CAPTURE CONTROL", "ELAPSED / DURATION",
        "SELECTED SESSION", "HARDWARE / PREFLIGHT"}
    window._set_language("zh_CN")
    assert window.page_eyebrow.text() == "04 / 录制"
    assert [window.pages.widget(index) for index in range(window.pages.count())] == pages
    assert window.filter.itemData(2) == "tracker"


@pytest.mark.parametrize("managed, control, chinese, english", [
    (True, True, "采集", "Capture"),
    (False, True, "设备", "Devices"),
    (False, False, "只读", "Read-only"),
])
def test_live_mode_badges_retranslate_without_submitting_commands(qapp, managed, control, chinese, english):
    backend = DeviceBackend()
    backend.value["management"]["enabled"] = managed
    backend.value["process_control"]["enabled"] = control
    window = MainWindow(backend, scene_backend=DemoSceneBackend())
    window.timer.stop()
    try:
        assert window.mode_badge.text() == f"●  实时 / {chinese}"
        assert window.windowTitle() == "VisionTactile · 采集工作台 [实时]"
        assert "LIVE" not in window.banner.text()
        assert "LIVE" not in window.footer.text()
        window._set_language("en")
        assert window.mode_badge.text() == f"●  LIVE / {english}"
        assert window.windowTitle() == "VisionTactile · Immersive Studio [LIVE]"
        window._set_language("zh_CN")
        assert window.mode_badge.text() == f"●  实时 / {chinese}"
        assert backend.commands == []
    finally:
        window.close()
        window.deleteLater()
        qapp.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        qapp.processEvents()


@pytest.mark.parametrize("kind, chinese, english", [
    ("camera", "相机", "Cameras"), ("tracker", "追踪器", "Tracker"),
    ("glove", "手势", "Hand Pose"), ("tactile", "触觉", "Tactile"),
])
def test_preview_chrome_switches_language_and_preserves_external_labels(qapp, kind, chinese, english):
    stage, source = CaptureStage(), PreviewWidget(kind)
    device = {"kind": kind, "id": "custom_stream_01", "label": "LIVE custom Camera A",
              "side": "right", "topic": "/custom/Topic/Tracker", "status": "offline", "preview": {}}
    before = deepcopy(device)
    try:
        assert stage.eyebrow.text() == "数据流监看"
        assert stage.hero_kicker.text() == "多模态观测"
        source.set_device(device)
        stage.set_source(source, device)
        assert stage.eyebrow.text() == chinese
        assert stage.hero_kicker.text() == "等待信号"
        assert stage.title.text() == device["label"]
        assert device["topic"] in stage.stream_info.text()
        assert stage.status.text() == "● 实时 / 离线"
        set_language("en", persist=False)
        assert stage.eyebrow.text() == english
        assert stage.hero_kicker.text() == "AWAITING SIGNAL / Awaiting Signal"
        assert stage.title.text() == device["label"]
        assert device["topic"] in stage.stream_info.text()
        stage.clear_source()
        assert stage.eyebrow.text() == "STREAM MONITOR"
        assert stage.hero_kicker.text() == "MULTIMODAL OBSERVATORY"
        set_language("zh_CN", persist=False)
        assert stage.eyebrow.text() == "数据流监看"
        assert stage.hero_kicker.text() == "多模态观测"
        assert device == before
    finally:
        stage.close()
        source.close()
        stage.deleteLater()
        source.deleteLater()
        qapp.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        qapp.processEvents()
