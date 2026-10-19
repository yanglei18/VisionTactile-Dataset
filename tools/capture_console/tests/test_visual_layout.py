"""Real-theme geometry regressions, independent of exact copy or paint colors.

These are offscreen widget tests, not screenshot comparisons. They deliberately
load the application's font stack and resolved stylesheet: default-Qt size
hints missed CJK/title clipping in the first dashboard implementation.
"""
from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

import pytest
from PySide6.QtCore import QPoint, QRect
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QFrame, QScrollArea


SIZES = ((1366, 768), (1440, 900), (1080, 720))


@pytest.fixture
def themed_window(qapp):
    from vt_capture_console.backend.demo_backend import DemoBackend
    from vt_capture_console.theme import stylesheet
    from vt_capture_console.ui.main_window import MainWindow

    previous_font = QFont(qapp.font())
    previous_stylesheet = qapp.styleSheet()
    previous_style_name = qapp.style().objectName()
    qapp.setStyle("Fusion")
    font = QFont()
    font.setFamilies(["Inter", "Noto Sans", "Noto Sans CJK SC"])
    font.setPointSize(10)
    qapp.setFont(font)
    qapp.setStyleSheet(stylesheet())
    windows = []

    class OfflineLiveBackend(DemoBackend):
        """Exercise real-mode chrome without opening sockets or devices."""
        mode = "live"
        path = "/nonexistent/test-only/manager-control.sock"

        def snapshot(self):
            value = super().snapshot()
            value["mode"] = "live"
            value["recording"].update(state="UNAVAILABLE", duration_s=0.,
                                       detail="后台暂不可达，保留只读界面；尚未获得正式录制状态。")
            for device in value["devices"]:
                device.update(status="unknown", age_s=None, receive_hz=0., valid_hz=None, preview={})
            return value

    def create(width, height, mode="demo"):
        backend = DemoBackend(clock=lambda: 12.) if mode == "demo" else OfflineLiveBackend(clock=lambda: 12.)
        window = MainWindow(backend)
        window.timer.stop()  # Geometry tests do not need time-dependent repaint.
        window.motion_toggle.setChecked(False)
        window.resize(width, height)
        window.show()
        windows.append(window)
        settle(window, qapp)
        return window

    yield create

    for window in windows:
        window.close()
        window.deleteLater()
    qapp.processEvents()
    qapp.setStyle(previous_style_name)
    qapp.setFont(previous_font)
    qapp.setStyleSheet(previous_stylesheet)
    qapp.processEvents()


def settle(window, qapp):
    # Stage, filmstrip, and inspector geometry settle across layout passes.
    # No sleeps or hardware events are needed.
    for _ in range(4):
        qapp.processEvents()
        window.refresh()
    qapp.processEvents()


def rect_in(widget, ancestor):
    return QRect(widget.mapTo(ancestor, QPoint(0, 0)), widget.size())


def assert_inside(widget, ancestor):
    rect = rect_in(widget, ancestor)
    assert rect.width() > 0 and rect.height() > 0
    assert ancestor.rect().contains(rect), (
        f"{widget.objectName() or type(widget).__name__}: {rect} extends outside {ancestor.rect()}"
    )


def reveal_recording_field(window, widget, qapp):
    # Setup remains scrollable on short displays, while Stop never requires a scroll.
    if window.recording_setup.isAncestorOf(widget):
        scroll = window.findChild(QScrollArea, "recordingSetupScroll")
        scroll.ensureWidgetVisible(widget, 4, 4)
        qapp.processEvents()
        assert_inside(widget, scroll.viewport())


@pytest.mark.parametrize("size", SIZES)
@pytest.mark.parametrize("mode", ["demo", "live"])
def test_real_theme_shell_regions_and_record_controls_fit(themed_window, qapp, size, mode):
    window = themed_window(*size, mode=mode)
    assert (window.width(), window.height()) == size
    assert qapp.styleSheet() and "@THEME_DIR@" not in qapp.styleSheet()
    assert window.centralWidget().minimumSizeHint().width() <= size[0]
    assert window.centralWidget().minimumSizeHint().height() <= size[1]

    header = window.findChild(QFrame, "workspaceHeader")
    assert header is not None and header.isVisible()
    assert window.findChild(QFrame, "recordingBar") is None
    assert not window.recording_console.isVisible()
    assert not window.page_heading.isVisible()
    assert not window.scroll.isVisible()
    assert not window.streams_toggle.isChecked()
    assert not window.details_panel.isVisible()
    assert window.inspector_toggle.isVisible()
    for widget in (header, window.footer, window.capture_stage,
                   window.inspector_toggle, window.streams_toggle, window.motion_toggle,
                   window.help_button):
        assert_inside(widget, window)
    assert_inside(window.motion_toggle, header)
    assert_inside(window.help_button, header)
    assert rect_in(header, window).bottom() < rect_in(window.pages, window).top()
    buttons = sorted(window.navigation.buttons(), key=lambda button: rect_in(button, header).x())
    assert len(buttons) == window.pages.count() == 5
    assert len({(button.width(), button.height()) for button in buttons}) == 1
    centers = [rect_in(button, header).center().x() for button in buttons]
    assert len({right - left for left, right in zip(centers, centers[1:])}) == 1
    for index, button in enumerate(buttons):
        assert button.isVisible()
        assert_inside(button, header)
        for other in buttons[index + 1:]:
            assert not rect_in(button, header).intersects(rect_in(other, header))
    assert window.capture_stage.width() >= 500
    assert window.capture_stage.height() >= 300

    # The first view gives the stage the full canvas. Browsing streams is an
    # explicit action and restores the horizontally scrollable filmstrip.
    full_stage_height = window.capture_stage.height()
    window.streams_toggle.click()
    settle(window, qapp)
    assert window.scroll.isVisible()
    assert window.capture_stage.height() < full_stage_height
    assert window.scroll.viewport().width() >= 360
    assert window.scroll.viewport().height() >= 120
    assert window.scroll.verticalScrollBar().maximum() == 0
    assert window.scroll.horizontalScrollBar().maximum() > 0
    assert not rect_in(window.capture_stage, window).intersects(
        rect_in(window.scroll.viewport(), window))

    assert not window.start_button.isVisible()
    window.navigation.button(4).click()
    settle(window, qapp)
    for control in (window.duration, window.start_button, window.stop_button):
        assert control.isVisible()
        reveal_recording_field(window, control, qapp)
        assert_inside(control, window.recording_page)
        if control is not window.duration:
            assert_inside(control, window.recording_console)
        assert_inside(control, window)


@pytest.mark.parametrize("size", SIZES)
@pytest.mark.parametrize("scenario", ["normal", "stale", "offline"])
def test_card_titles_keep_visible_width_with_real_cjk_font(themed_window, qapp, size, scenario):
    window = themed_window(*size)
    window.backend.scenario = scenario
    window.streams_toggle.click()
    settle(window, qapp)
    viewport = window.scroll.viewport()
    visible_titles = 0
    for index in range(window.grid.count()):
        card = window.grid.itemAt(index).widget()
        assert card is not None
        assert card.title.width() > 40
        assert card.title.height() >= card.title.fontMetrics().height()
        assert_inside(card.title, card)
        assert_inside(card.state, card)
        assert not rect_in(card.title, card).intersects(rect_in(card.state, card))
        shown = rect_in(card.title, viewport).intersected(viewport.rect())
        # A horizontal filmstrip may clip a title at its right edge. Count only
        # titles with a usable visible area; offscreen titles share its height.
        if shown.height() > 0 and shown.width() > 40:
            visible_titles += 1
    assert visible_titles >= 2


@pytest.mark.parametrize("size", SIZES)
def test_filters_and_work_pages_keep_controls_with_their_theme(themed_window, qapp, size):
    window = themed_window(*size)
    window.streams_toggle.click()
    for index in range(window.filter.count()):
        window.segments.button(index).click()
        assert window.filter.currentIndex() == index
        settle(window, qapp)
        selected_kind = window.filter.currentData()
        expected = sum(selected_kind == "all" or device["kind"] == selected_kind
                       for device in window.devices.values())
        assert window.grid.count() == expected
        assert window.scroll.viewport().height() >= 120
        assert window.scroll.verticalScrollBar().maximum() == 0
        assert all(window.grid.getItemPosition(item)[0] == 0
                   for item in range(window.grid.count()))
        assert not window.start_button.isVisible()
    window.filter.setCurrentIndex(0)  # Preserve the existing programmatic API.
    assert window.segments.button(0).isChecked()
    for button in window.navigation.buttons():
        button.click()
        settle(window, qapp)
        assert window.pages.currentIndex() == window.navigation.id(button)
        assert window.page_heading.isVisible() == (window.pages.currentIndex() not in (0, 3))
        assert button.isChecked() and button.isVisible()
        for control in (window.start_button, window.stop_button, window.duration,
                        window.batch_selector, window.trial_name):
            assert control.isVisible() == (window.pages.currentIndex() == 4)
            if control.isVisible():
                reveal_recording_field(window, control, qapp)
                assert_inside(control, window.recording_page)
                assert_inside(control, window)
        assert_inside(window.footer, window)
        assert window.centralWidget().minimumSizeHint().height() <= size[1]


@pytest.mark.parametrize("size", SIZES)
def test_inspector_toggle_preserves_selected_stream_and_active_recording(themed_window, qapp, size):
    window = themed_window(*size)
    window._select("d405_2")
    window.navigation.button(4).click()
    window.start_button.click()
    window.navigation.button(0).click()
    settle(window, qapp)
    assert window.backend.state == "RECORDING"
    assert not window.details_panel.isVisible()
    selected_id = window.selected_id
    selected_topic = window.detail_topic.text()
    stage_width = window.capture_stage.width()

    window.inspector_toggle.click()
    settle(window, qapp)
    assert window.details_panel.isVisible()
    assert_inside(window.details_panel, window)
    assert_inside(window.capture_stage, window)
    assert not rect_in(window.details_panel, window).intersects(
        rect_in(window.capture_stage, window))
    assert not window.recording_console.isVisible()
    assert window.selected_id == selected_id
    assert window.detail_topic.text() == selected_topic
    assert window.backend.state == "RECORDING"
    assert window.stop_button.isEnabled() and not window.start_button.isEnabled()

    window.inspector_toggle.click()
    settle(window, qapp)
    assert not window.details_panel.isVisible()
    assert window.capture_stage.width() == stage_width
    assert window.selected_id == selected_id
    assert window.detail_topic.text() == selected_topic
    assert window.backend.state == "RECORDING"
    assert window.stop_button.isEnabled() and not window.start_button.isEnabled()
    window.navigation.button(4).click()
    window.stop_button.click()
    assert window.backend.state == "COMPLETE"


@pytest.mark.parametrize("size", SIZES)
def test_maximum_duration_text_fits_actual_spinbox_editor(themed_window, qapp, size):
    window = themed_window(*size)
    window.navigation.button(4).click()
    duration = window.duration
    duration.setValue(duration.maximum())
    settle(window, qapp)
    reveal_recording_field(window, duration, qapp)
    editor = duration.lineEdit()
    expected = duration.prefix() + str(duration.maximum()) + duration.suffix()
    assert duration.maximum() == 300
    assert editor.text() == duration.text() == expected
    margins = editor.textMargins()
    available = editor.contentsRect().width() - margins.left() - margins.right()
    required = editor.fontMetrics().horizontalAdvance(expected) + 4
    assert available >= required, (
        f"Maximum duration {expected!r} needs {required}px at the actual theme font, "
        f"but the spinbox editor has only {available}px; text would be clipped."
    )
    assert_inside(editor, duration)


@pytest.mark.parametrize("size", SIZES)
def test_recording_form_and_archive_fit_their_own_pages_after_commands(themed_window, qapp, size):
    window = themed_window(*size)
    window.navigation.button(4).click()
    window.batch_name.setText("采集批次 " + "较长名称" * 8)
    window.create_batch_button.click()
    window.trial_name.setText("全手抓握试次 " + "详细描述" * 8)
    window.start_button.click()
    window.stop_button.click()
    settle(window, qapp)
    for widget in (window.batch_selector, window.batch_name, window.create_batch_button,
                   window.trial_name, window.operation_notice,
                   window.start_button, window.stop_button):
        assert widget.isVisible()
        reveal_recording_field(window, widget, qapp)
        assert_inside(widget, window)
    assert not window.trial_table.isVisible()
    assert not rect_in(window.batch_selector, window).intersects(rect_in(window.batch_name, window))

    window.navigation.button(1).click()
    window.trial_table.selectRow(0)
    settle(window, qapp)
    for widget in (window.trial_table, window.trial_details, window.inspect_trial_button):
        assert widget.isVisible()
        assert_inside(widget, window)
    for widget in (window.batch_selector, window.trial_name, window.start_button, window.stop_button):
        assert not widget.isVisible()
    assert window.trial_table.viewport().height() >= 100
    assert window.centralWidget().minimumSizeHint().height() <= size[1]


def test_styled_frontend_still_has_no_ros_legacy_qt_or_hardware_imports():
    code = """
import sys
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication
from vt_capture_console.theme import stylesheet
from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.main_window import MainWindow
app = QApplication([])
app.setStyle('Fusion')
font = QFont()
font.setFamilies(['Inter', 'Noto Sans', 'Noto Sans CJK SC'])
font.setPointSize(10)
app.setFont(font)
app.setStyleSheet(stylesheet())
window = MainWindow(DemoBackend(clock=lambda: 12.))
window.resize(1080, 720)
window.show()
app.processEvents()
blocked = ('PyQt5', 'PyQt6', 'rclpy', 'torch', 'pyrealsense2', 'serial')
assert not any(name == prefix or name.startswith(prefix + '.')
               for name in sys.modules for prefix in blocked)
window.close()
"""
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    source = str(Path(__file__).resolve().parents[1] / "src")
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, (source, environment.get("PYTHONPATH"))))
    result = subprocess.run([sys.executable, "-c", code], env=environment, capture_output=True,
                            text=True, timeout=15)
    assert result.returncode == 0, result.stderr
