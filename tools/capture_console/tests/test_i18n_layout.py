"""Bilingual geometry with the production stylesheet and memory-only devices."""

import pytest
from PySide6.QtWidgets import QFormLayout, QFrame, QLabel, QStyle, QStyleOptionButton

from test_visual_layout import assert_inside, rect_in, settle, themed_window
from test_workstation_settings_ui import Runtime
from vt_capture_console.i18n import set_language


_SIZES = ((1080, 720), (1366, 768), (1440, 900))


@pytest.fixture(autouse=True)
def restore_language(qapp, isolated_presentation_settings):
    set_language("zh_CN", persist=False)
    yield
    set_language("zh_CN", persist=False)


def _english_window(themed_window, qapp, size):
    window = themed_window(*size)
    window.language_buttons["en"].click()
    settle(window, qapp)
    return window


def _assert_button_text_fits(button):
    option = QStyleOptionButton()
    option.initFrom(button)
    option.text = button.text()
    contents = button.style().subElementRect(
        QStyle.SubElement.SE_PushButtonContents, option, button
    )
    required = button.fontMetrics().horizontalAdvance(button.text())
    if not button.icon().isNull():
        required += button.iconSize().width() + 4
    assert required <= contents.width(), (
        f"{button.objectName() or type(button).__name__} {button.text()!r}: "
        f"text and icon need {required}px; styled contents provide {contents.width()}px "
        f"inside a {button.width()}px button"
    )


@pytest.mark.parametrize("size", _SIZES)
@pytest.mark.parametrize("code", ("zh_CN", "en"))
def test_bilingual_header_preserves_requested_size_and_even_navigation(themed_window, qapp, size, code):
    window = themed_window(*size)
    window.language_buttons[code].click()
    settle(window, qapp)
    assert (window.width(), window.height()) == size, (
        f"{code} shell expanded beyond requested {size}: "
        f"{window.width()} × {window.height()}"
    )
    header = window.findChild(QFrame, "workspaceHeader")
    buttons = sorted(window.navigation.buttons(), key=lambda b: rect_in(b, header).left())
    header_controls = buttons + [
        window.motion_toggle, window.help_button, window.language_switch, window.mode_badge
    ]
    for index, button in enumerate(header_controls):
        assert button.isVisible(), f"Hidden header control: {button.objectName()}"
        assert_inside(button, header)
        for other in header_controls[index + 1:]:
            assert not rect_in(button, header).intersects(rect_in(other, header)), (
                f"Overlapping {code} header controls: {button.objectName()!r} / {other.objectName()!r}"
            )
    gaps = [rect_in(b, header).left() - rect_in(a, header).right() - 1
            for a, b in zip(buttons, buttons[1:])]
    assert len(set(gaps)) == 1 and gaps[0] > 0, f"Uneven navigation spacing: {gaps}"
    for value, button in window.language_buttons.items():
        assert button.isChecked() == (value == code)
        assert_inside(button, window.language_switch)
        _assert_button_text_fits(button)


@pytest.mark.parametrize("size", _SIZES)
def test_each_english_navigation_title_fits_selected_and_unselected(themed_window, qapp, size):
    window = _english_window(themed_window, qapp, size)
    for index in (2, 0, 3, 4, 1):
        window.navigation.button(index).click()
        settle(window, qapp)
        for button in window.navigation.buttons():
            _assert_button_text_fits(button)


@pytest.mark.parametrize("size", _SIZES)
def test_english_recording_start_and_stop_stay_visible_and_operable(themed_window, qapp, size):
    window = _english_window(themed_window, qapp, size)
    window.navigation.button(4).click()
    settle(window, qapp)
    for control in (window.start_button, window.stop_button):
        assert control.isVisible()
        assert_inside(control, window.recording_console)
        assert_inside(control, window)
        _assert_button_text_fits(control)
    assert not rect_in(window.start_button, window.recording_console).intersects(
        rect_in(window.stop_button, window.recording_console)
    )
    assert window.start_button.isEnabled()
    window.start_button.click()
    settle(window, qapp)
    assert window.backend.state == "RECORDING"
    assert window.stop_button.isEnabled() and window.stop_button.isVisible()
    window.stop_button.click()
    settle(window, qapp)
    assert window.backend.state == "COMPLETE"


@pytest.mark.parametrize("size", _SIZES)
def test_english_settings_tabs_and_form_labels_are_readable(themed_window, qapp, size):
    window = _english_window(themed_window, qapp, size)
    window.backend.runtime = Runtime()
    window.navigation.button(2).click()
    window.configure_devices_button.click()
    dialog = window.settings_dialog
    dialog.timer.stop()
    dialog.resize(min(900, size[0] - 80), min(650, size[1] - 70))
    qapp.processEvents()
    tabs = dialog.tabs
    bar = tabs.tabBar()
    failures = []
    for index in range(tabs.count()):
        tabs.setCurrentIndex(index)
        qapp.processEvents()
        title = tabs.tabText(index)
        required = bar.fontMetrics().horizontalAdvance(title) + 24
        if bar.tabRect(index).width() < required:
            failures.append(
                f"Tab {title!r} needs {required}px including padding; "
                f"has {bar.tabRect(index).width()}px"
            )
        for form in tabs.widget(index).findChildren(QFormLayout):
            for row in range(form.rowCount()):
                item = form.itemAt(row, QFormLayout.ItemRole.LabelRole)
                label = item.widget() if item else None
                if not isinstance(label, QLabel) or label.wordWrap():
                    continue
                required = label.fontMetrics().horizontalAdvance(label.text())
                if required > label.contentsRect().width():
                    failures.append(
                        f"Tab {title!r}, label {label.text()!r} needs {required}px; "
                        f"has {label.contentsRect().width()}px"
                    )
                if label.height() < label.fontMetrics().height():
                    failures.append(f"Tab {title!r}, label {label.text()!r} is clipped vertically")
    for control in (dialog.cancel_button, dialog.apply_button):
        assert control.isVisible()
        assert_inside(control, dialog)
        _assert_button_text_fits(control)
    assert not failures, "\n".join(failures)
    assert window.backend.runtime.applications == []
    assert window.backend.runtime.starts == 0
