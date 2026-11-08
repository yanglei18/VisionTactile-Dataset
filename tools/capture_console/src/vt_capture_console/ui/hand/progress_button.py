"""A themed button whose background displays backend-supplied progress."""
from __future__ import annotations

from PySide6 import QtCore, QtGui, QtWidgets

from vt_fsglove12.ui_theme import PALETTE


class _ButtonProgressBar(QtWidgets.QProgressBar):
    """Use Qt's progress animation, then draw the button label above the fill."""

    def paintEvent(self, event):
        super().paintEvent(event)
        button = self.parentWidget()
        option = QtWidgets.QStyleOptionButton()
        button.initStyleOption(option)
        option.rect.translate(-self.pos())
        # Busy buttons remain non-interactive, but their instructions must stay
        # legible against the fill. This changes painting only, not enabled state.
        if self.maximum() == 0 or self.value() > 0:
            option.state |= QtWidgets.QStyle.StateFlag.State_Enabled
        painter = QtGui.QPainter(self)
        button.style().drawControl(QtWidgets.QStyle.ControlElement.CE_PushButtonLabel,
                                   option, painter, button)


class ProgressButton(QtWidgets.QPushButton):
    """The owner supplies progress; this widget never advances measurements."""

    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
        self.setAutoDefault(False)
        self.progress_bar = _ButtonProgressBar(self)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.progress_bar.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.progress_bar.setStyleSheet(
            "QProgressBar { border: none; border-radius: 1px; min-height: 0px; "
            f"max-height: 16777215px; background: {PALETTE['raised']}; }}"
            f"QProgressBar::chunk {{ background: {PALETTE['border']}; }}")
        self.set_progress()

    def setText(self, text):
        super().setText(text)
        if hasattr(self, "progress_bar"):
            self.progress_bar.update()

    def set_progress(self, value=None, *, busy=False):
        maximum = 0 if busy else 1000
        if self.progress_bar.maximum() != maximum:
            self.progress_bar.setRange(0, maximum)
        if not busy:
            target = 0 if value is None else round(max(0., min(1., value)) * 1000)
            if self.progress_bar.value() != target:
                self.progress_bar.setValue(target)
        # An empty track is still visible before capture and after cancellation.
        # Do not hide/show it on phase changes or restart Qt's busy animation.
        if self.progress_bar.isHidden():
            self.progress_bar.show()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.progress_bar.setGeometry(2, 2, max(0, self.width() - 4), max(0, self.height() - 4))
