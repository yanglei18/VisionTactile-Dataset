"""Standalone language changes preserve serial and renderer ownership."""
from pathlib import Path

import pytest
from PySide6 import QtCore, QtWidgets

from vt_capture_console import i18n, tactile_app
from vt_capture_console.backend.tactile_settings import TactileSettings
from vt_capture_console.ui.tactile_window import TactileWindow
from test_tactile_window import FakeRenderer, FakeSource, frame, sample


@pytest.fixture(autouse=True)
def viewer_language(qapp, tmp_path):
    store = QtCore.QSettings(str(tmp_path / "viewer.ini"), QtCore.QSettings.Format.IniFormat)
    i18n.initialize(store, override="zh_CN")
    yield store
    i18n.set_language("zh_CN", persist=False)


@pytest.fixture
def viewer(viewer_language, monkeypatch, tmp_path):
    from vt_capture_console.ui.widgets import tactile_3d
    monkeypatch.setattr(tactile_3d, "Tactile3DWidget", FakeRenderer)
    source = FakeSource()
    settings = TactileSettings(viewer_language, model_bundle=tmp_path / "right_hand.json")
    window = TactileWindow(source, model_settings=settings, port_provider=lambda: [
        {"path": "COM12", "label": "COM12 · 用户设备 {unchanged}"}])
    yield window, source
    window.shutdown()
    window.close()
    window.deleteLater()


def select_language(window, code):
    index = window.language_combo.findData(code)
    window.language_combo.setCurrentIndex(index)
    window.language_combo.activated.emit(index)


def test_live_switch_preserves_connection_frames_range_and_renderer(viewer, viewer_language):
    window, source = viewer
    window.port_combo.setCurrentIndex(1)
    window.toggle_connection()
    source.connection_state = "connected"
    source.state = sample(frame(22000), status="实时", detail="", age_ms=12.2, receive_hz=119.8)
    window.start()
    view, data, levels = window.preview._tactile_3d, window.preview._frame, window.preview._levels
    counters = (source.starts, source.polls, source.closes, source.disconnects, len(source.connect_calls))
    for code, title, button in (("en", "Tactile 3D Viewer · Right Hand", "Disconnect"),
                                ("zh_CN", "触觉三维预览 · 右手", "断开"),
                                ("en", "Tactile 3D Viewer · Right Hand", "Disconnect")):
        select_language(window, code)
        assert window.windowTitle() == title
        assert window.connection_button.text() == button
        assert window.preview._tactile_3d is view
        assert window.preview._frame is data and window.preview._levels is levels
        assert window.timer.isActive() and not window.port_combo.isEnabled()
        assert window.selected_port() == "COM12"
        assert window.port_combo.currentText() == "COM12 · 用户设备 {unchanged}"
        assert (source.starts, source.polls, source.closes, source.disconnects, len(source.connect_calls)) == counters
        assert source.state.status == "实时"
        assert viewer_language.value("ui/language") == code
    assert window.status.text() == str(i18n.tr("实时"))
    assert "119.8 Hz" in window.metrics.text() and "Data age 12 ms" in window.metrics.text()
    assert "原始" not in " ".join(label.text() for label in window.findChildren(QtWidgets.QLabel))


def test_switch_preserves_typed_port_selection_and_stays_disconnected(viewer):
    window, source = viewer
    window.port_combo.setEditText("COM12 / 用户自定义 {port}")
    editor = window.port_combo.lineEdit()
    editor.setSelection(2, 6)
    draft = editor.text(), editor.cursorPosition(), editor.selectionStart(), editor.selectionLength()
    select_language(window, "en")
    assert (editor.text(), editor.cursorPosition(), editor.selectionStart(), editor.selectionLength()) == draft
    assert not source.connect_calls and source.polls == 0
    assert window.port_combo.itemText(0) == "Select a tactile serial port"
    assert window.connection_button.text() == "Connect"


def test_held_frame_status_and_error_retranslate_without_new_poll(viewer):
    window, source = viewer
    source.state = sample(frame(22000), status="实时", detail="")
    window.refresh()
    source.state = sample(status="未连接", detail="请选择右手触觉串口并连接。")
    window.refresh()
    data, polls = window.preview._frame, source.polls
    select_language(window, "en")
    assert "历史" not in window.notice.text() and "history" in window.notice.text()
    assert not any("\u4e00" <= c <= "\u9fff" for c in window.notice.text())
    assert window.preview._frame is data and window.preview.held
    assert source.polls == polls
    window.port_combo.setEditText("COM12")
    source.connect_error = OSError("串口被占用或访问被拒绝，请关闭其他使用该串口的程序并检查设备权限。")
    window.toggle_connection()
    assert "Serial port operation failed:" in window.notice.text()
    assert "access was denied" in window.notice.text()
    select_language(window, "zh_CN")
    assert "串口操作失败" in window.notice.text() and "访问被拒绝" in window.notice.text()


def test_saved_language_is_independent_and_cli_override_does_not_write(viewer_language, tmp_path):
    workstation = tmp_path / "capture-console.ini"
    workstation.write_text("[ui]\nlanguage=zh_CN\n", encoding="utf-8")
    viewer_language.setValue("ui/language", "en")
    viewer_language.setValue("model/python", "C:/用户/python.exe")
    viewer_language.sync()
    before = Path(viewer_language.fileName()).read_bytes()
    tactile_app.initialize_language(viewer_language)
    assert i18n.language() == "en"
    tactile_app.initialize_language(viewer_language, "zh_CN")
    assert i18n.language() == "zh_CN"
    assert Path(viewer_language.fileName()).read_bytes() == before
    tactile_app.initialize_language(QtCore.QSettings(viewer_language.fileName(), QtCore.QSettings.Format.IniFormat))
    assert i18n.language() == "en"
    assert workstation.read_text() == "[ui]\nlanguage=zh_CN\n"


@pytest.mark.parametrize("locale,expected", [("zh_CN", "zh_CN"), ("zh_TW", "zh_CN"), ("en_US", "en"), ("de_DE", "en")])
def test_first_launch_uses_system_language_without_saving_a_choice(viewer_language, monkeypatch, locale, expected):
    monkeypatch.setattr(QtCore.QLocale, "system", lambda: QtCore.QLocale(locale))
    tactile_app.initialize_language(viewer_language)
    assert i18n.language() == expected
    assert viewer_language.allKeys() == []


def test_language_cli_accepts_both_supported_languages():
    assert tactile_app.parser().parse_args([]).language is None
    for code in ("zh_CN", "en"):
        assert tactile_app.parser().parse_args(["--language", code]).language == code
    with pytest.raises(SystemExit):
        tactile_app.parser().parse_args(["--language", "unsupported"])


def test_native_renderer_text_binding_retranslates_without_replacing_widget(viewer_language):
    from vt_capture_console.i18n.widgets import bind_text
    view = QtWidgets.QWidget()
    try:
        description = i18n.tr("静态参考手，非实时手势。{v0}/400 个地址具有候选表面位置；{v1} 个地址未映射；并非逐点皮肤标定。", v0=400, v1=0)
        bind_text(view, "setAccessibleDescription", description)
        bind_text(view, "setToolTip", description)
        i18n.set_language("en", persist=False)
        assert "400/400" in view.toolTip()
        assert not any("\u4e00" <= c <= "\u9fff" for c in view.toolTip())
        assert view.toolTip() == view.accessibleDescription()
        i18n.set_language("zh_CN", persist=False)
        assert "静态参考手" in view.toolTip()
    finally:
        view.deleteLater()


def test_cached_model_failure_retranslates_inner_error_and_existing_toolbar(viewer_language, tmp_path, monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    from vt_capture_console.ui.widgets.tactile_model_loader import TactileModelLoader
    model = tmp_path / "right_hand.json"
    model.write_text('{"schema_version":1,"schema_version":1}', encoding="utf-8")
    settings = TactileSettings(viewer_language, model_bundle=model)
    loader = TactileModelLoader(lambda: settings.configuration)
    monkeypatch.setattr(tactile_3d, "model_loader", lambda: loader)
    widget = tactile_3d.Tactile3DWidget("right")
    try:
        assert "重复字段" in widget.message.text()
        for code in ("en", "zh_CN", "en"):
            i18n.set_language(code, persist=False)
            if code == "en":
                assert "duplicate fields" in widget.message.text()
                assert not any("\u4e00" <= c <= "\u9fff" for c in widget.message.text())
                assert not any("\u4e00" <= c <= "\u9fff" for c in str(loader.error("right")))
                assert widget.buttons[0].text() == str(i18n.tr("掌面"))
            else:
                assert "重复字段" in widget.message.text()
            widget._model_failed("right", loader.error("right"))
    finally:
        widget.dispose()
        loader.shutdown()


def test_model_error_signal_keeps_hand_label_and_opengl_error_translatable(viewer_language, monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    from vt_capture_console.ui.widgets.tactile_model_loader import TactileModelLoader
    loader = TactileModelLoader(lambda: {"process_groups": []})
    monkeypatch.setattr(tactile_3d, "model_loader", lambda: loader)
    widget = tactile_3d.Tactile3DWidget("right")
    try:
        # Re-request after widget construction to exercise the Qt error signal.
        loader.request("right")
        assert "右手" in widget.message.text()
        i18n.set_language("en", persist=False)
        assert not any("\u4e00" <= c <= "\u9fff" for c in widget.message.text())
        i18n.set_language("zh_CN", persist=False)
        assert "右手" in widget.message.text()
        # Offscreen fails before geometry use, exercising the renderer's own
        # authored exception path rather than a backend diagnostic.
        widget._model_ready("right", {})
        assert "不支持 OpenGL" in widget.message.text()
        i18n.set_language("en", persist=False)
        assert "OpenGL" in widget.message.text()
        assert not any("\u4e00" <= c <= "\u9fff" for c in widget.message.text())
    finally:
        widget.dispose()
        loader.shutdown()
