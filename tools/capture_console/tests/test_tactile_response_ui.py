"""Explicit zeroing and response settings do not interrupt source ownership."""
from PySide6 import QtCore, QtWidgets
import pytest

from vt_capture_console import i18n
from vt_capture_console.ui.tactile_window import TactileWindow
from test_tactile_window import FakeRenderer, FakeSource, frame, response_status, sample


def response_frame(value=0., maximum=1000.):
    return {**frame(value), "levels": [0., maximum], "value_kind": "tactile_response", "reference_ready": False}


@pytest.fixture
def response_viewer(qapp, tmp_path, monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    monkeypatch.setattr(tactile_3d, "Tactile3DWidget", FakeRenderer)
    settings = QtCore.QSettings(str(tmp_path / "language.ini"), QtCore.QSettings.Format.IniFormat)
    i18n.initialize(settings, override="zh_CN")
    source = FakeSource()
    window = TactileWindow(source, port_provider=lambda: [])
    yield window, source
    window.shutdown()
    window.close()
    window.deleteLater()
    i18n.set_language("zh_CN", persist=False)


def connect_with_frame(window, source, value=0.):
    source.connection_state = "connected"
    source.state = sample(response_frame(value), status="实时", detail="", valid_cells=400)
    window.refresh()


def test_connect_requires_explicit_zeroing_and_uses_fixed_response_levels(response_viewer):
    window, source = response_viewer
    assert not window.reference_button.isEnabled()
    assert source.reference_calls == 0 and source.response_calls == []
    connect_with_frame(window, source)
    assert window.reference_button.isEnabled()
    assert "尚未归零" in window.reference_status.text()
    assert "无载归零" in window.notice.text() and not window.notice.isHidden()
    assert window.preview._levels == (0., 1000.)
    assert source.reference_calls == 0
    assert window.response_range.value() == 1000
    assert window.response_direction.currentData() == "both"


def test_manual_zeroing_has_nonblocking_progress_and_readiness(response_viewer):
    window, source = response_viewer
    connect_with_frame(window, source, 500)
    previous = window.preview._frame
    window.reference_button.click()
    assert source.reference_calls == 1 and window.preview._frame is not previous
    assert window.preview._frame["values"] == (0.,) * 400
    assert not window.reference_button.isEnabled()
    assert not window.reference_progress.isHidden()
    assert "3 秒" in window.notice.text() and "不受压" in window.notice.text()
    source.response_status = response_status("collecting", progress=.57)
    window.refresh()
    assert window.reference_progress.value() == 57
    assert source.reference_calls == 1
    source.response_status = response_status("ready", progress=1., reference_cells=398)
    source.state.frame = response_frame(30.)
    source.state.frame["valid_mask"][-2:] = [False, False]
    window.refresh()
    assert window.reference_progress.isHidden() and window.reference_button.isEnabled()
    assert "参考就绪 398/400" in window.reference_status.text()
    assert window.notice.isHidden()
    window.reference_button.click()
    assert source.reference_calls == 2


@pytest.mark.parametrize("origin,zh,en", [
    ("default", "通用参考 400/400", "General reference 400/400"),
    ("preset", "默认参考 400/400", "Default reference 400/400"),
    ("saved", "已保存参考 400/400", "Saved reference 400/400"),
    ("measured", "参考就绪 400/400", "Reference ready 400/400"),
])
def test_ready_reference_source_is_visible_without_requiring_zeroing(response_viewer, origin, zh, en):
    window, source = response_viewer
    source.response_status = response_status("ready", reference_cells=400, reference_source=origin)
    connect_with_frame(window, source, 25.)
    assert window.reference_status.text() == zh
    if origin == "default":
        assert "通用参考" in window.reference_status.toolTip()
        assert "可选" in window.reference_status.toolTip()
    assert window.notice.isHidden()
    assert window.reference_button.isEnabled()
    assert "可选" in window.reference_button.toolTip()
    assert source.reference_calls == 0
    window._set_language("en")
    assert window.reference_status.text() == en
    if origin == "default":
        assert "general reference" in window.reference_status.toolTip().lower()
    assert "Optional" in window.reference_button.toolTip()
    assert source.reference_calls == 0
    window.reference_button.click()
    assert source.reference_calls == 1
    assert window.reference_status.toolTip() == ""


def test_reference_failure_and_data_loss_are_visible_and_retryable(response_viewer):
    window, source = response_viewer
    connect_with_frame(window, source)
    source.reference_error = ValueError("需要新鲜触觉数据才能采集无载参考。")
    old_frame = window.preview._frame
    window.reference_button.click()
    assert source.reference_calls == 0 and window.preview._frame is not None
    assert window.preview._frame["values"] == old_frame["values"]
    assert "无法开始归零" in window.notice.text()
    source.reference_error = None
    window.reference_button.click()
    source.response_status = response_status("failed", reason="insufficient_samples")
    window.refresh()
    assert "归零未完成" in window.reference_status.text()
    assert "有效采样不足" in window.notice.text()
    assert window.reference_button.isEnabled()
    source.state = sample(status="数据过期", detail="")
    window.refresh()
    assert not window.reference_button.isEnabled()
    assert "历史保持" in window.notice.text()


def test_reference_save_failure_is_visible_without_discarding_live_data(response_viewer):
    window, source = response_viewer
    source.response_status = response_status("ready", reference_cells=400, reference_source="measured")
    source.reference_persistence_error = "当前参考和显示设置未能保存；下次启动仍使用原配置。"
    connect_with_frame(window, source, 25.)
    assert not window.notice.isHidden()
    assert "未能保存" in window.notice.text()
    assert window.preview._frame["values"] == (25.,) * 400
    assert window.reference_status.text() == "参考就绪 400/400"
    window._set_language("en")
    assert "could not be saved" in window.notice.text()


def test_profile_settings_update_controls_without_resubmitting(response_viewer):
    window, source = response_viewer
    source.configure_response(display_range_adc=2400., direction="decrease")
    calls = list(source.response_calls)
    connect_with_frame(window, source)
    assert window.response_range.value() == 2400
    assert window.response_direction.currentData() == "decrease"
    assert source.response_calls == calls
    window.response_range.setValue(700)
    window.refresh()
    assert window.response_range.value() == 700  # unchanged source must not erase an edit


def test_profile_setting_sync_defers_while_range_is_being_edited(response_viewer, qapp):
    window, source = response_viewer
    window.show()
    window.settings_toggle.setChecked(True)
    window.response_range.setFocus()
    qapp.processEvents()
    assert window.response_range.hasFocus()
    window.response_range.setValue(700)
    source.configure_response(display_range_adc=2400., direction="decrease")
    window.refresh()
    assert window.response_range.value() == 700
    assert len(source.response_calls) == 1
    window.response_range.editingFinished.emit()
    assert source.response_settings.display_range_adc == 700


def test_fixed_display_range_and_polarity_changes_preserve_reference(response_viewer):
    window, source = response_viewer
    connect_with_frame(window, source, 30.)
    source.response_status = response_status("ready", progress=1., reference_cells=400)
    window.refresh()
    previous = window.preview._frame
    polls = source.polls
    window.response_range.editingFinished.emit()
    assert source.response_calls == [] and source.polls == polls
    assert window.preview._frame is previous
    window.response_range.setValue(2000)
    assert source.response_calls == []
    window.response_range.editingFinished.emit()
    assert source.response_calls == [(2000., "both")]
    assert window.preview._levels == (0., 2000.)
    index = window.response_direction.findData("decrease")
    window.response_direction.setCurrentIndex(index)
    window.response_direction.activated.emit(index)
    assert source.response_calls[-1] == (2000., "decrease")
    assert source.reference_calls == 0
    assert source.response_status.phase == "ready"


def test_language_switch_during_reference_is_presentation_only(response_viewer):
    window, source = response_viewer
    connect_with_frame(window, source)
    window.reference_button.click()
    source.response_status = response_status("collecting", progress=.4)
    window.refresh()
    view, old_frame = window.preview._tactile_3d, window.preview._frame
    counters = (source.polls, source.reference_calls, tuple(source.response_calls), tuple(source.connect_calls))
    index = window.language_combo.findData("en")
    window.language_combo.setCurrentIndex(index)
    window.language_combo.activated.emit(index)
    assert (source.polls, source.reference_calls, tuple(source.response_calls), tuple(source.connect_calls)) == counters
    assert window.preview._tactile_3d is view and window.preview._frame is old_frame
    assert window.reference_progress.value() == 40
    assert window.reference_button.text() == "Zeroing…"
    assert "unloaded" in window.notice.text().lower()
    assert window.response_direction.itemText(0) == "Both directions"
    for label in window.findChildren(QtWidgets.QLabel):
        assert not any("\u4e00" <= char <= "\u9fff" for char in label.text()), label.text()


def test_real_source_zero_hold_release_and_language_with_fake_serial(qapp, monkeypatch, tmp_path):
    """Exercise source/assembler/reference/UI together, opening no hardware."""
    from vt_capture_console.backend import tactile_serial
    from vt_capture_console.ui.widgets import tactile_3d
    from vt_tactile_capture.protocol import Block
    from vt_tactile_capture.serial_source import SerialWorker
    from test_tactile_serial import NOW, Worker

    monkeypatch.setattr(tactile_3d, "Tactile3DWidget", FakeRenderer)
    clock = [NOW]
    monkeypatch.setattr(tactile_serial.host_clock, "monotonic_ns", lambda: clock[0])
    monkeypatch.setattr(tactile_serial.host_clock, "realtime_ns", lambda: 1_700_000_000_000_000_000 + clock[0])
    settings = QtCore.QSettings(str(tmp_path / "language.ini"), QtCore.QSettings.Format.IniFormat)
    i18n.initialize(settings, override="zh_CN")
    source = tactile_serial.TactileSerialSource(worker_factory=Worker)
    window = TactileWindow(source, port="COM_SYNTHETIC_ONLY", port_provider=lambda: [])
    producer = SerialWorker("COM_SYNTHETIC_ONLY")

    def deliver(*, response=0, advance_ns=50_000_000):
        clock[0] += advance_ns
        blocks = tuple(Block(row, group, 0, tuple(
            1000 + response if row == group == index == 0 else 1000
            for index in range(10))) for row in range(20) for group in range(8))
        producer._emit(connected=True, reason="ok", blocks=blocks)
        source._worker.queue.append(producer.get_nowait())
        window.refresh()

    try:
        window.connection_button.click()
        deliver(advance_ns=0)
        assert source.response_status.phase == "unreferenced"
        assert window.preview._frame["values"] == (0.,) * 400
        window.reference_button.click()
        for _ in range(30):
            deliver()
        assert source.response_status.phase == "collecting"
        assert window.reference_progress.value() == 50
        phase_before = source.response_status
        window._set_language("en")
        assert source.response_status == phase_before
        assert "unloaded" in window.notice.text().lower()
        for _ in range(30):
            deliver()
        assert source.response_status.phase == "ready"
        assert source.response_status.reference_cells == 400
        assert window.reference_status.text() == "Reference ready 400/400"
        assert window.preview._levels == (0., 1000.)
        assert window.preview._frame["values"] == (0.,) * 400
        deliver(response=100)
        held_value = window.preview._frame["values"][0]
        assert held_value > 0
        for _ in range(20):
            deliver(response=100)
        assert window.preview._frame["values"][0] == held_value
        assert all(value == 0 for value in window.preview._frame["values"][1:])
        deliver()
        assert window.preview._frame["values"] == (0.,) * 400
        assert source.response_status.phase == "ready"
        assert window.notice.isHidden()
    finally:
        window.shutdown()
        window.close()
        window.deleteLater()
        i18n.set_language("zh_CN", persist=False)
