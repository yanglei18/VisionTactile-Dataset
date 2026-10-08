import sys

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest


def test_navigation_and_recording_stays_visible(window):
    for index in range(4):
        QTest.mouseClick(window.navigation.button(index), Qt.MouseButton.LeftButton)
        assert window.pages.currentIndex() == index
        assert window.start_button.isVisible()


def test_demo_recording_controls(window):
    window.duration.setValue(12)
    QTest.mouseClick(window.start_button, Qt.MouseButton.LeftButton)
    assert window.backend.state == "RECORDING"
    assert not window.start_button.isEnabled()
    assert window.stop_button.isEnabled()
    QTest.mouseClick(window.stop_button, Qt.MouseButton.LeftButton)
    assert window.backend.state == "COMPLETE"
    assert window.trial_table.rowCount() == 1


def test_filter_and_details(window):
    window.filter.setCurrentIndex(4)
    window.refresh()
    assert window.grid.count() == 2
    window._select("right_tactile")
    assert window.detail_topic.text() == "/gloves/right/tactile/state"


def test_segmented_filter_uses_same_model_and_preserves_selection(window):
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


def test_card_titles_have_space_and_preview_selects_without_blocking_drag(window):
    card = window.cards["d405_2"]
    assert card.title.width() > 50
    QTest.mouseClick(card.preview, Qt.MouseButton.LeftButton)
    assert window.selected_id == "d405_2"


def test_resize_and_hand_modes(window, qapp):
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


def test_no_ros_or_legacy_qt_import(window):
    assert not any(name == "rclpy" or name.startswith(("PyQt5", "PyQt6", "torch")) for name in sys.modules)
