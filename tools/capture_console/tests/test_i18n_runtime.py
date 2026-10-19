"""Translation resources have bounded lifetime and retain user-owned values."""
from copy import copy, deepcopy

from PySide6.QtCore import QEvent, QTranslator
from PySide6.QtTest import QSignalSpy

from vt_capture_console.i18n import initialize, set_language, tr
from vt_capture_console.i18n.widgets import QComboBox, QLineEdit, QLabel


def test_repeated_switch_does_not_accumulate_native_translators(qapp, tmp_path):
    from PySide6.QtCore import QSettings
    initialize(QSettings(str(tmp_path / 'ui.ini'), QSettings.Format.IniFormat))
    for _ in range(12):
        set_language('en', persist=False)
        set_language('zh_CN', persist=False)
        qapp.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        assert len(qapp.findChildren(QTranslator)) <= 1


def test_file_dialog_translates_builtin_labels_on_existing_dialog(qapp, tmp_path):
    from PySide6.QtCore import QCoreApplication
    from PySide6.QtWidgets import QFileDialog, QLabel as NativeLabel

    set_language('zh_CN', persist=False)
    dialog = QFileDialog()
    dialog.setOption(QFileDialog.Option.DontUseNativeDialog)
    dialog.setDirectory(str(tmp_path))
    filename = 'Board 配置.yaml'
    name_filter = 'User data (*.yaml *.json)'
    dialog.setNameFilter(name_filter)
    dialog.selectFile(filename)
    dialog.show()
    try:
        for code, look_in, file_type, file_name in (
                ('zh_CN', '查找位置(&I)：', '文件类型(&T)：', '文件名称(&N)：'),
                ('en', '&Look in:', 'Files of &type:', 'File &name:'),
                ('zh_CN', '查找位置(&I)：', '文件类型(&T)：', '文件名称(&N)：')):
            set_language(code, persist=False)
            qapp.processEvents()
            assert dialog.findChild(NativeLabel, 'lookInLabel').text() == look_in
            assert dialog.findChild(NativeLabel, 'fileTypeLabel').text() == file_type
            assert dialog.findChild(NativeLabel, 'fileNameLabel').text() == file_name
            assert dialog.selectedFiles() == [str(tmp_path / filename)]
            assert dialog.selectedNameFilter() == name_filter
            # Qt-owned labels are translated by their context, never by a
            # global string replacement that could rename user-owned text.
            assert QCoreApplication.translate('UserData', '&Look in:') == '&Look in:'
    finally:
        dialog.close()
        dialog.deleteLater()
        set_language('zh_CN', persist=False)


def test_copy_retains_translation_recipe_without_modifying_values(qapp):
    value = tr('有效 {v0:.1f} Hz', v0=30.25)
    duplicate = deepcopy({'value': value})['value']
    assert copy(value) is value and duplicate is value
    try:
        set_language('en', persist=False)
        assert str(duplicate) == 'Valid 30.2 Hz'
        set_language('zh_CN', persist=False)
        assert str(duplicate) == '有效 30.2 Hz'
    finally:
        set_language('zh_CN', persist=False)


def test_user_draft_cursor_and_selection_survive_combo_retranslation(qapp):
    combo = QComboBox()
    combo.setEditable(True)
    combo.addItem(tr('未选择'), '')
    combo.setEditText('/dev/serial/by-id/用户接口')
    editor = combo.lineEdit()
    editor.setSelection(4, 6)
    editor.setModified(True)
    original = (editor.text(), editor.cursorPosition(), editor.selectionStart(), editor.selectionLength())
    changed, edited = QSignalSpy(combo.currentTextChanged), QSignalSpy(editor.textChanged)
    try:
        set_language('en', persist=False)
        assert (editor.text(), editor.cursorPosition(), editor.selectionStart(), editor.selectionLength()) == original
        assert editor.isModified()
        assert changed.count() == edited.count() == 0
        assert combo.currentData() == ''
    finally:
        set_language('zh_CN', persist=False)
        combo.close()
        combo.deleteLater()


def test_raw_user_text_matching_a_catalog_entry_is_never_retranslated(qapp):
    label, editor = QLabel('设备'), QLineEdit('设备')
    try:
        set_language('en', persist=False)
        assert label.text() == editor.text() == '设备'
    finally:
        set_language('zh_CN', persist=False)
        label.deleteLater()
        editor.deleteLater()


def test_builtin_demo_batch_display_does_not_rename_user_batches(window, qapp):
    backend = window.backend
    backend.batches.append({'id': 'user-batch', 'label': '演示批次', 'created_at': ''})
    original = deepcopy(backend.batches)
    try:
        window._set_language('en')
        assert window.batch_selector.itemText(window.batch_selector.findData('demo-batch')) == 'Demo batch'
        assert window.batch_selector.itemText(window.batch_selector.findData('user-batch')) == '演示批次'
        assert backend.batches == original
        titles = [card.title.text() for card in window.cards.values()]
        assert len(titles) == len(backend.snapshot()['devices'])
        assert all(text and not any('\u4e00' <= char <= '\u9fff' for char in text) for text in titles)

    finally:
        window._set_language('zh_CN')
