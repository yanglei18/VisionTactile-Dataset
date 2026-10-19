"""Native non-modal spatial window; data is supplied by the owning page."""

from ...i18n import tr
from PySide6.QtCore import Qt
from ...i18n.widgets import QDialog, QVBoxLayout

from .spatial_page import SpatialPage


class SpatialDetailDialog(QDialog):
    def __init__(self, backend, parent=None):
        super().__init__(parent)
        self.setObjectName("spatialDetailWindow")
        self.setWindowTitle(tr('联合空间 · 独立视口'))
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.setWindowFlag(Qt.WindowType.WindowMaximizeButtonHint, True)
        self.setWindowFlag(Qt.WindowType.WindowMinimizeButtonHint, True)
        self.setModal(False)
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.resize(1360, 860)
        self.setMinimumSize(800, 580)
        self.page = SpatialPage(backend, self, allow_detail=False)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        layout.addWidget(self.page)

    def closeEvent(self, event):
        self.page.close()
        super().closeEvent(event)
