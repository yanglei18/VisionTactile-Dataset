"""Small presentation-only containers shared by the calibration pages."""
from PyQt5 import QtCore, QtWidgets


class CollapsibleSection(QtWidgets.QWidget):
    """Keep optional controls available without occupying the normal workflow."""

    def __init__(self, title, parent=None, *, expanded=False):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.button = QtWidgets.QToolButton(objectName="disclosure")
        self.button.setText(title)
        self.button.setCheckable(True)
        self.button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.content = QtWidgets.QWidget()
        self.content_layout = QtWidgets.QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(0, 4, 0, 4)
        self.content_layout.setSpacing(8)
        layout.addWidget(self.button)
        layout.addWidget(self.content)
        self.button.toggled.connect(self._set_content_visible)
        self.setExpanded(expanded)

    def _set_content_visible(self, expanded):
        self.content.setVisible(expanded)
        self.button.setArrowType(QtCore.Qt.DownArrow if expanded else QtCore.Qt.RightArrow)

    def setExpanded(self, expanded):
        self.button.setChecked(bool(expanded))
        self._set_content_visible(bool(expanded))

    def isExpanded(self):
        return self.button.isChecked()
