"""Qt widgets that retain explicitly marked presentation text on language switch.

Only LocalizedText values are bound. User-entered text, paths, identifiers,
combo item data, raw device logs and backend state are never translated.
There is no polling timer or global Qt monkey-patch.
"""
from __future__ import annotations

import weakref
from PySide6 import QtCore, QtGui, QtWidgets
from shiboken6 import isValid

from . import LocalizedText, language, render

_bound = weakref.WeakValueDictionary()
_native_translator = None
_native_language = None


class _NativeChineseTranslator(QtCore.QTranslator):
    """Complete Qt's file-dialog labels while retaining its native catalog."""

    _labels = {
        ('QFileDialog', '&Look in:'): '查找位置(&I)：',
        ('QFileDialog', 'Files of &type:'): '文件类型(&T)：',
    }

    def translate(self, context, source_text, disambiguation=None, n=-1):
        if not disambiguation and n == -1:
            text = self._labels.get((context, source_text))
            if text is not None:
                return text
        # PySide converts Qt's null QString to ""; return None for a miss so
        # QCoreApplication can fall back to the original source text.
        return super().translate(context, source_text, disambiguation, n) or None


def _resolve(value):
    if isinstance(value, (tuple, list)):
        return type(value)(_resolve(v) for v in value)
    return render(value)


def _marked(value):
    if isinstance(value, LocalizedText):
        return True
    return isinstance(value, (tuple, list)) and any(_marked(v) for v in value)


def _remember(obj, key, setter, args):
    bindings = getattr(obj, '_translated_properties', None)
    if bindings is None:
        bindings = obj._translated_properties = {}
    if _marked(args):
        bindings[key] = (setter, args)
        _bound[id(obj)] = obj
    else:
        bindings.pop(key, None)
    return setter(obj, *(_resolve(v) for v in args))


def bind_text(obj, method, text):
    """Mark a text property on a native child control created internally by Qt."""
    if method not in {'setText', 'setPlaceholderText', 'setToolTip', 'setAccessibleName', 'setAccessibleDescription'}:
        raise ValueError('Not a supported display-text property')
    return _remember(obj, method, getattr(type(obj), method), (text,))


def retranslate_all(*, changed=True):
    global _native_translator, _native_language
    app = QtWidgets.QApplication.instance()
    if app is None:
        return
    if _native_language != language():
        if _native_translator is not None:
            app.removeTranslator(_native_translator)
            _native_translator.deleteLater()
        _native_translator = None
        if language() == 'zh_CN':
            translator = _NativeChineseTranslator(app)
            if translator.load('qtbase_zh_CN', QtCore.QLibraryInfo.path(QtCore.QLibraryInfo.LibraryPath.TranslationsPath)):
                app.installTranslator(translator)
                _native_translator = translator
            else:
                translator.deleteLater()
        _native_language = language()
    if not changed:
        return
    for obj in tuple(_bound.values()):
        if not isValid(obj):
            continue
        # Combo/tab label changes must not trigger selection or device actions.
        # Preserve a custom port/path being typed into an editable combo even
        # when Qt still associates its editor with the old selected item.
        blocker = QtCore.QSignalBlocker(obj) if isinstance(obj, (QtWidgets.QComboBox, QtWidgets.QTabWidget)) else None
        editor = obj.lineEdit() if isinstance(obj, QtWidgets.QComboBox) and obj.isEditable() else None
        draft = None
        editor_blocker = None
        if editor is not None and obj.currentText() != obj.itemText(obj.currentIndex()):
            draft = (editor.text(), editor.cursorPosition(), editor.selectionStart(), editor.selectionLength(), editor.isModified())
            editor_blocker = QtCore.QSignalBlocker(editor)
        try:
            for setter, args in tuple(getattr(obj, '_translated_properties', {}).values()):
                setter(obj, *(_resolve(v) for v in args))
        finally:
            if draft is not None:
                text, cursor, start, length, modified = draft
                editor.setText(text)
                editor.setCursorPosition(cursor)
                if start >= 0:
                    editor.setSelection(start, length)
                editor.setModified(modified)
            del editor_blocker
            del blocker
    for window in app.topLevelWidgets():
        window.update()
        # Also invalidate child paint canvases whose labels are drawn directly.
        for child in window.findChildren(QtWidgets.QWidget):
            child.update()


def _widget_type(name, base, constructor_property=None, *, editable=False):
    attributes = {'__module__': __name__}
    setters = ('setWindowTitle', 'setToolTip', 'setStatusTip', 'setWhatsThis',
               'setAccessibleName', 'setAccessibleDescription', 'setPlaceholderText',
               'setTitle', 'setPrefix', 'setSuffix', 'setFormat', 'setInformativeText',
               'setDetailedText')
    # Explicitly marked generated read-only text can retranslate; raw metadata
    # and logs are plain strings and are deliberately left unbound.
    if name in {'QTextEdit', 'QPlainTextEdit'}:
        setters += ('setPlainText',)
    if not editable:
        setters += ('setText',)
    for method in setters:
        if not hasattr(base, method):
            continue
        setter = getattr(base, method)
        def translated(self, *args, _setter=setter, _method=method):
            return _remember(self, _method, _setter, args)
        attributes[method] = translated
    def init(self, *args, **kwargs):
        base.__init__(self, *(_resolve(v) for v in args), **{k: _resolve(v) for k, v in kwargs.items()})
        if constructor_property and not editable:
            text = next((v for v in args if isinstance(v, str)), kwargs.get(constructor_property))
            if isinstance(text, LocalizedText):
                getattr(self, 'set' + constructor_property[0].upper() + constructor_property[1:])(text)
    attributes['__init__'] = init
    return type(name, (base,), attributes)


for _name in ('QWidget', 'QMainWindow', 'QDialog', 'QFrame', 'QScrollArea', 'QLabel',
              'QPushButton', 'QToolButton', 'QCheckBox', 'QRadioButton', 'QGroupBox',
              'QLineEdit', 'QTextEdit', 'QPlainTextEdit', 'QSpinBox', 'QDoubleSpinBox',
              'QProgressBar', 'QSlider', 'QMenu', 'QMenuBar', 'QDialogButtonBox',
              'QMessageBox', 'QFileDialog', 'QTableWidgetItem', 'QListWidgetItem'):
    _property = ('text' if _name in {'QLabel', 'QPushButton', 'QToolButton', 'QCheckBox',
                                    'QRadioButton', 'QTableWidgetItem', 'QListWidgetItem'}
                 else 'title' if _name in {'QGroupBox', 'QMenu'} else None)
    globals()[_name] = _widget_type(_name, getattr(QtWidgets, _name), _property,
                                  editable=_name in {'QLineEdit', 'QTextEdit', 'QPlainTextEdit'})

QAction = _widget_type('QAction', QtGui.QAction, 'text')


class QFileDialog(QFileDialog):
    @staticmethod
    def getOpenFileName(*args, **kwargs):
        return QtWidgets.QFileDialog.getOpenFileName(*map(_resolve, args), **{k: _resolve(v) for k, v in kwargs.items()})

    @staticmethod
    def getOpenFileNames(*args, **kwargs):
        return QtWidgets.QFileDialog.getOpenFileNames(*map(_resolve, args), **{k: _resolve(v) for k, v in kwargs.items()})

    @staticmethod
    def getSaveFileName(*args, **kwargs):
        return QtWidgets.QFileDialog.getSaveFileName(*map(_resolve, args), **{k: _resolve(v) for k, v in kwargs.items()})

    @staticmethod
    def getExistingDirectory(*args, **kwargs):
        return QtWidgets.QFileDialog.getExistingDirectory(*map(_resolve, args), **{k: _resolve(v) for k, v in kwargs.items()})


class QMenu(QMenu):
    def addAction(self, *args, **kwargs):
        action = super().addAction(*(_resolve(v) for v in args), **kwargs)
        text = next((v for v in args if isinstance(v, LocalizedText)), None)
        if text is not None:
            _remember(action, 'setText', QtGui.QAction.setText, (text,))
        return action

    def addMenu(self, *args):
        menu = super().addMenu(*(_resolve(v) for v in args))
        text = next((v for v in args if isinstance(v, LocalizedText)), None)
        if text is not None and isinstance(menu, QtWidgets.QMenu):
            _remember(menu, 'setTitle', QtWidgets.QMenu.setTitle, (text,))
        return menu


class QComboBox(_widget_type('QComboBoxBase', QtWidgets.QComboBox)):
    def addItem(self, *args, **kwargs):
        index = self.count()
        super().addItem(*(_resolve(v) for v in args), **kwargs)
        value = next((v for v in args[:2] if isinstance(v, str)), None)
        if value is not None:
            self.setItemText(index, value)

    def addItems(self, texts):
        for text in texts:
            self.addItem(text)

    def setItemText(self, index, text):
        return _remember(self, ('item', index), QtWidgets.QComboBox.setItemText, (index, text))

    def clear(self):
        bindings = getattr(self, '_translated_properties', {})
        for key in list(bindings):
            if isinstance(key, tuple) and key[0] == 'item':
                del bindings[key]
        super().clear()


class QTabWidget(_widget_type('QTabWidgetBase', QtWidgets.QTabWidget)):
    def addTab(self, *args):
        index = super().addTab(*(_resolve(v) for v in args))
        self.setTabText(index, args[-1])
        return index

    def setTabText(self, index, text):
        return _remember(self, ('tab', index), QtWidgets.QTabWidget.setTabText, (index, text))


class QTableWidget(_widget_type('QTableWidgetBase', QtWidgets.QTableWidget)):
    def setHorizontalHeaderLabels(self, texts):
        texts = list(texts)
        for index, text in enumerate(texts):
            self.setHorizontalHeaderItem(index, QTableWidgetItem(text))

    def setVerticalHeaderLabels(self, texts):
        for index, text in enumerate(texts):
            self.setVerticalHeaderItem(index, QTableWidgetItem(text))


class QFormLayout(QtWidgets.QFormLayout):
    def addRow(self, *args):
        if len(args) == 2 and isinstance(args[0], LocalizedText):
            label = QLabel(args[0])
            if isinstance(args[1], QtWidgets.QWidget):
                label.setBuddy(args[1])
            return super().addRow(label, args[1])
        return super().addRow(*args)


class QPainter(QtGui.QPainter):
    def drawText(self, *args):
        return super().drawText(*(_resolve(v) for v in args))


def __getattr__(name):
    return getattr(QtWidgets, name)
