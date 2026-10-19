"""Standalone serial controls using the actual workbench preview, without GL."""
from pathlib import Path
from types import SimpleNamespace

import pytest
from PySide6 import QtCore, QtWidgets

from vt_capture_console import tactile_app
from vt_capture_console.ui.tactile_window import TactileWindow
from vt_capture_console.ui.widgets.preview import PreviewWidget


def sample(frame=None, **changes):
    return SimpleNamespace(**{
        "frame": frame, "status": "等待数据", "detail": "等待触觉数据。",
        "age_ms": None, "receive_hz": 0., "valid_cells": 0, **changes,
    })


def frame(value):
    return {"values": [value] * 400, "valid_mask": [True] * 400,
            "rows": 20, "cols": 20}


class FakeSource:
    def __init__(self):
        self.state = sample()
        self.starts = self.polls = self.closes = 0
        self.connect_calls = []
        self.disconnects = 0
        self.connection_state = "disconnected"
        self.close_result = True
        self.connect_error = None

    @property
    def can_connect(self):
        return self.connection_state in ("disconnected", "error")

    @property
    def can_disconnect(self):
        return self.connection_state in ("connecting", "connected")

    def connect(self, port, baudrate):
        self.connect_calls.append((port, baudrate))
        if self.connect_error:
            self.connection_state = "error"
            raise self.connect_error
        self.connection_state = "connecting"
        self.state = sample(status="正在连接")

    def disconnect(self):
        self.disconnects += 1
        self.connection_state = "disconnected"
        self.state = sample(status="未连接")
        return True

    def start(self):
        self.starts += 1

    def poll(self):
        self.polls += 1
        return self.state

    def close(self):
        self.closes += 1
        self.connection_state = "disconnected" if self.close_result else "disconnecting"
        return self.close_result


class FakeRenderer(QtWidgets.QWidget):
    def __init__(self, hand, parent):
        super().__init__(parent)
        self.hand = hand
        self.frames = []

    def set_frame(self, value, levels):
        self.frames.append((value, levels))

    def dispose(self):
        self.hide()
        self.deleteLater()


@pytest.fixture(autouse=True)
def fake_3d(monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    monkeypatch.setattr(tactile_3d, "Tactile3DWidget", FakeRenderer)


@pytest.fixture
def viewer(qapp):
    source = FakeSource()
    window = TactileWindow(source, port_provider=lambda: [])
    yield window, source
    window.shutdown()
    window.close()
    window.deleteLater()


def test_raw_values_display_immediately_with_the_workbench_range_and_history(viewer):
    window, source = viewer
    assert type(window.preview) is PreviewWidget
    assert window.preview.tactile_view == "3d"
    assert source.starts == source.polls == 0
    window.start()
    assert source.starts == source.polls == 1 and window.timer.isActive()
    assert not window.preview.has_frame
    source.state = sample(frame(22000), status="实时", detail="", age_ms=12.2,
                          receive_hz=29.7, valid_cells=400)
    window.refresh()
    assert window.preview._levels == (21950., 22050.)
    assert window.preview._frame["values"] == (22000.,) * 400
    assert window.preview._tactile_3d.frames[-1] == (window.preview._frame, (21950., 22050.))
    assert window.status.text() == "实时"
    assert "29.7 Hz" in window.metrics.text() and "12 ms" in window.metrics.text()
    assert window.notice.isHidden()
    assert not hasattr(window, "reference_button") and not hasattr(window, "high")
    source.state = sample(frame(23500), status="实时", valid_cells=400)
    window.refresh()
    assert window.preview._levels == (21950., 22050.)
    previous = window.preview._frame
    source.state = sample(status="数据过期", detail="源已过期，显示已清除。", age_ms=600.)
    window.refresh()
    assert window.preview.held and window.preview._frame is previous
    assert "历史" in window.preview._footer_text()
    assert "历史保持" in window.notice.text() and "已清除" not in window.notice.text()
    source.state = sample(frame(22500), status="实时", detail="", valid_cells=400)
    window.refresh()
    assert not window.preview.held and window.notice.isHidden()


def test_same_samples_match_main_preview_colors_masks_and_held_state(viewer):
    window, source = viewer
    main = PreviewWidget("tactile")
    try:
        initial = frame(22000)
        initial["values"][0] = 21000
        initial["values"][1] = 23000
        initial["values"][2] = 65535
        initial["valid_mask"][2] = False
        partial = frame(24000)
        partial["valid_mask"][3] = False
        missing = frame(0)
        missing["valid_mask"] = [False] * 400
        for value in (initial, partial, missing, None, frame(21500)):
            source.state = sample(value, status="实时" if value else "数据过期")
            window.refresh()
            main.set_device(window.preview.device_snapshot)
            main.set_tactile_view("3d")
            assert window.preview._frame == main._frame
            assert window.preview._levels == main._levels == (21000., 23000.)
            assert window.preview._cell_colors == main._cell_colors
            assert window.preview.held == main.held
            assert window.preview._tactile_3d.frames[-1] == main._tactile_3d.frames[-1]
    finally:
        main.close()
        main.deleteLater()


def test_disconnect_holds_history_but_new_connection_resets_color_range(viewer):
    window, source = viewer
    source.connection_state = "connected"
    source.state = sample(frame(22000), status="实时", valid_cells=400)
    window.refresh()
    window.toggle_connection()
    assert source.disconnects == 1 and window.preview.held
    window.port_combo.setEditText("/dev/new-port")
    window.toggle_connection()
    assert source.connect_calls == [("/dev/new-port", 115200)]
    assert not window.preview.has_frame and window.preview._levels is None
    source.state = sample(frame(18000), status="实时", valid_cells=400)
    window.refresh()
    assert window.preview._levels == (17950., 18050.)


def test_close_is_idempotent_stops_updates_and_releases_only_owned_preview(viewer, qapp):
    window, source = viewer
    other_source = FakeSource()
    other = TactileWindow(other_source, port_provider=lambda: [])
    destroyed = []
    window.preview.destroyed.connect(lambda: destroyed.append(True))
    try:
        window.start()
        other.start()
        polls = source.polls
        window.close()
        window.shutdown()
        window.refresh()
        assert not window.timer.isActive() and source.closes == 1
        assert source.polls == polls and other_source.closes == 0
        assert other.timer.isActive()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        assert destroyed == [True]
    finally:
        other.shutdown()
        other.close()
        other.deleteLater()


def test_enumeration_is_passive_and_connect_uses_selected_path_not_label(qapp):
    source = FakeSource()
    entries = [{"path": "/dev/fake-a", "label": "设备甲 → fake-a"},
               {"path": "/dev/fake-b", "label": "设备乙 → fake-b"}]
    window = TactileWindow(source, port_provider=lambda: entries)
    try:
        window.start()
        assert not source.connect_calls
        assert window.selected_port() == "" and not window.connection_button.isEnabled()
        window.port_combo.setCurrentIndex(2)
        window.baudrate.setValue(230400)
        window.refresh_ports_button.click()
        assert window.selected_port() == "/dev/fake-b" and not source.connect_calls
        window.connection_button.click()
        assert source.connect_calls == [("/dev/fake-b", 230400)]
        assert window.connection_button.text() == "断开"
        assert not window.port_combo.isEnabled()
        assert not window.refresh_ports_button.isEnabled() and not window.baudrate.isEnabled()
        window.connection_button.click()
        assert source.disconnects == 1 and window.connection_button.text() == "连接"
        assert window.port_combo.isEnabled() and window.baudrate.isEnabled()
    finally:
        window.shutdown()
        window.close()
        window.deleteLater()


def test_prefilled_or_typed_port_requires_click_and_connection_error_can_retry(qapp):
    source = FakeSource()
    window = TactileWindow(source, port_provider=lambda: [], port="/dev/MUST_NOT_OPEN")
    try:
        window.start()
        assert window.selected_port() == "/dev/MUST_NOT_OPEN" and not source.connect_calls
        window.port_combo.setEditText("  /dev/typed-port  ")
        assert window.selected_port() == "/dev/typed-port"
        source.connect_error = OSError("合成占用错误")
        window.connection_button.click()
        assert source.connect_calls == [("/dev/typed-port", 115200)]
        assert window.connection_button.isEnabled() and window.port_combo.isEnabled()
        assert "合成占用错误" in window.notice.text()
        source.connect_error = None
        window.connection_button.click()
        assert len(source.connect_calls) == 2 and source.connection_state == "connecting"
        assert "合成占用错误" not in window.notice.text()
    finally:
        window.shutdown()
        window.close()
        window.deleteLater()


def test_close_pending_keeps_window_and_timer_until_port_is_released(viewer):
    window, source = viewer
    window.show()
    window.start()
    source.close_result = False
    assert window.close() is False
    assert window.isVisible() and window.timer.isActive()
    assert "正在释放串口" in window.notice.text()
    source.close_result = True
    assert window.close() is True
    assert not window.timer.isActive() and source.closes == 2


def test_cli_defaults_and_explicit_arguments_are_typed_without_launching():
    defaults = tactile_app.parser().parse_args([])
    assert defaults.port == "" and defaults.baudrate == 115200
    assert not hasattr(defaults, "change_max") and not hasattr(defaults, "adc_max")
    args = tactile_app.parser().parse_args([
        "--port", "/dev/MUST_NOT_OPEN", "--baudrate", "230400",
        "--glove-python", "/venv/bin/python", "--mano-assets", "/trusted assets",
        "--smoke-seconds", "0.5", "--screenshot", "/tmp/tactile.png"])
    assert args.port == "/dev/MUST_NOT_OPEN" and args.baudrate == 230400
    assert args.screenshot == Path("/tmp/tactile.png") and args.smoke_seconds == .5


@pytest.mark.parametrize("arguments", [
    ["--change-max", "3000"], ["--adc-min", "0"], ["--adc-max", "24000"],
    ["--smoke-seconds", "-1"], ["--smoke-seconds", "nan"],
    ["--smoke-seconds", "inf"], ["--unknown-option"],
    ["--baudrate", "0"], ["--baudrate", "4000001"], ["--baudrate", "fast"],
    ["--screenshot", "/tmp/missing-deadline.png"],
    ["--topic", "/must-not-enable-ros"], ["--workspace", "/must-not-enable-ros"],
    ["--model-bundle", "/model.json", "--glove-python", "/python"],
    ["--self-check", "/report.json", "--smoke-seconds", "1"],
    ["--self-check", "/report.json", "--mano-assets", "/models"],
])
def test_cli_rejects_invalid_arguments_before_gui_or_error_dialog(arguments, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail("Invalid CLI arguments must not start Qt or display a modal dialog")

    monkeypatch.setattr(QtWidgets, "QApplication", forbidden)
    monkeypatch.setattr(QtWidgets.QMessageBox, "critical", forbidden)
    with pytest.raises(SystemExit) as error:
        tactile_app.main(arguments)
    assert error.value.code == 2 and "error:" in capsys.readouterr().err


def test_installed_model_hides_development_environment_fields_and_never_connects(qapp, tmp_path):
    from vt_capture_console.backend.tactile_settings import TactileSettings

    store = QtCore.QSettings(str(tmp_path / "settings.ini"), QtCore.QSettings.Format.IniFormat)
    settings = TactileSettings(store, model_bundle=tmp_path / "right_hand.json")
    source = FakeSource()
    window = TactileWindow(source, port_provider=lambda: [], model_settings=settings)
    try:
        assert not window.settings_toggle.isChecked()
        assert not hasattr(window, "model_python") and not hasattr(window, "model_assets")
        assert any("无需安装" in label.text() for label in window.settings.findChildren(QtWidgets.QLabel))
        window.start()
        assert not source.connect_calls and not store.allKeys()
    finally:
        window.shutdown()
        window.close()
        window.deleteLater()
