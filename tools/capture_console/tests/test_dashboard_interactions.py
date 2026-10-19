import sys

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest


def test_navigation_exposes_only_each_pages_work_controls(window):
    for index in range(window.pages.count()):
        QTest.mouseClick(window.navigation.button(index), Qt.MouseButton.LeftButton)
        assert window.pages.currentIndex() == index
        assert window.start_button.isVisible() == (index == 4)
        assert window.stop_button.isVisible() == (index == 4)
        assert window.batch_selector.isVisible() == (index == 4)
        assert window.page_heading.isVisible() == (index not in (0, 3))


def test_demo_recording_controls(window):
    window.navigation.button(4).click()
    window.duration.setValue(12)
    QTest.mouseClick(window.start_button, Qt.MouseButton.LeftButton)
    assert window.backend.state == "RECORDING"
    assert not window.start_button.isEnabled()
    assert window.stop_button.isEnabled()
    QTest.mouseClick(window.stop_button, Qt.MouseButton.LeftButton)
    assert window.backend.state == "COMPLETE"
    assert window.trial_table.rowCount() == 1


def test_filter_and_details(window):
    window.streams_toggle.click()
    window.filter.setCurrentIndex(4)
    window.refresh()
    assert window.grid.count() == 2
    window._select("right_tactile")
    assert window.detail_topic.text() == "/gloves/right/tactile/state"


def test_segmented_filter_uses_same_model_and_preserves_selection(window):
    window.streams_toggle.click()
    QTest.mouseClick(window.segments.button(3), Qt.MouseButton.LeftButton)
    window.refresh()
    assert window.filter.currentData() == "glove"
    assert window.grid.count() == 2
    assert window.segments.button(3).isChecked()
    window.filter.setCurrentIndex(4)
    window.refresh()
    assert window.segments.button(4).isChecked()
    assert window.grid.count() == 2


def test_cards_retained_not_rebuilt_every_refresh(window):
    old_cards = dict(window.cards)
    for _ in range(4):
        window.refresh()
    assert window.cards == old_cards
    window.backend.scenario = "stale"
    window.refresh()
    assert "陈旧" in window.cards["right_glove"].state.text()


def test_card_titles_have_space_and_preview_selects_without_blocking_drag(window, qapp):
    assert not window.scroll.isVisible()
    window.streams_toggle.click()
    qapp.processEvents()
    card = window.cards["d405_2"]
    window.scroll.ensureWidgetVisible(card)
    qapp.processEvents()
    assert card.title.width() > 50
    QTest.mouseClick(card.preview, Qt.MouseButton.LeftButton)
    assert window.selected_id == "d405_2"


def test_resize_and_hand_modes(window, qapp):
    window.motion_toggle.setChecked(False)
    for width, height in ((1366, 768), (1440, 900), (1080, 720)):
        window.resize(width, height)
        qapp.processEvents()
        window.refresh()
        assert window.width() == width
        assert window.height() == height
        assert window.centralWidget().minimumSizeHint().width() <= width
    window.backend.hands = "none"
    window.refresh()
    assert len(window.cards) == 6


def test_stream_browser_can_collapse_without_changing_selected_data(window, qapp):
    assert not window.streams_toggle.isChecked()
    assert not window.scroll.isVisible()
    initial_cards = dict(window.cards)
    window.streams_toggle.click()
    qapp.processEvents()
    assert window.scroll.isVisible()
    window._select("d405_2")
    source = window.capture_stage.source_preview
    window.streams_toggle.click()
    window.refresh()
    assert not window.scroll.isVisible()
    assert window.selected_id == "d405_2"
    assert window.capture_stage.source_preview is source
    assert window.cards == initial_cards


def test_reduced_motion_keeps_recording_and_incoming_preview_updates(window):
    window.motion_toggle.setChecked(True)
    window.navigation.button(4).click()
    window.start_button.click()
    assert window.backend.state == "RECORDING"
    assert window.timer.isActive() and window.motion_toggle.isChecked()
    window.navigation.button(0).click()
    previous_frame = window.capture_stage.preview._frame
    source = window.capture_stage.source_preview
    window.motion_toggle.click()
    assert not window.motion_toggle.isChecked()
    assert not window.ambient._timer.isActive()
    assert window.backend.state == "RECORDING" and window.timer.isActive()

    window.backend.clock = lambda: window.backend.origin + 2.
    window.refresh()
    assert window.capture_stage.source_preview is source
    assert window.capture_stage.preview._frame == source._frame
    assert window.capture_stage.preview._frame != previous_frame
    assert window.backend.state == "RECORDING"
    window.navigation.button(1).click()
    assert window.pages.currentIndex() == 1 and window.pages._overlay is None
    assert window.stop_button.isEnabled()
    window.motion_toggle.click()
    assert window.motion_toggle.isChecked() and window.ambient._timer.isActive()
    window.navigation.button(4).click()
    window.stop_button.click()
    assert window.backend.state == "COMPLETE"


def test_no_ros_or_legacy_qt_import(window):
    assert not any(name == "rclpy" or name.startswith(("PyQt5", "PyQt6", "torch")) for name in sys.modules)
