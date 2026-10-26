"""Gesture controls and enlarged preview, sharing the main snapshot/renderer."""
from __future__ import annotations

from ...i18n import tr
from ...i18n.messages import message as display_message

from PySide6.QtCore import Qt, Signal
from ...i18n.widgets import QButtonGroup, QDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .preview import PreviewWidget


class GloveViewControls(QWidget):
    view_requested = Signal(str)
    calibration_requested = Signal()
    expand_requested = Signal()

    def __init__(self, *, expandable=False, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        self.setStyleSheet("""
            QPushButton {font-size: 11px; padding: 3px 5px; border-radius: 2px;}
            QPushButton:checked {color: #ffffff; background: #242424; border-color: #999999;}
        """)
        self.group = QButtonGroup(self)
        self.three_d_button = QPushButton(tr('3D 手形'))
        self.skeleton_button = QPushButton(tr('骨架'))
        for i, button in enumerate((self.three_d_button, self.skeleton_button)):
            button.setCheckable(True)
            button.setAutoDefault(False)
            self.group.addButton(button, i)
            row.addWidget(button)
        self.group.idClicked.connect(lambda i: self.view_requested.emit(("3d", "skeleton")[i]))
        self.three_d_button.setToolTip(tr('从收到的 MANO 解算参数重建网格，不重复进行 IMU 解算'))
        self.skeleton_button.setToolTip(tr('查看数据主题中原有的 21 个关节点；不依赖许可模型'))
        row.addStretch()
        self.calibration_button = QPushButton(tr('标定'))
        self.calibration_button.setAutoDefault(False)
        self.calibration_button.setToolTip(tr('打开本手的原标定工具；已运行时切换窗口，不另建采集进程'))
        self.calibration_button.clicked.connect(self.calibration_requested)
        row.addWidget(self.calibration_button)
        self.expand_button = None
        if expandable:
            self.expand_button = QPushButton(tr('放大'))
            self.expand_button.setAutoDefault(False)
            self.expand_button.clicked.connect(self.expand_requested)
            row.addWidget(self.expand_button)
        for button in self.findChildren(QPushButton):
            button.setFixedHeight(28)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.three_d_button.setAccessibleName(tr('三维手势模型'))
        self.skeleton_button.setAccessibleName(tr('手势关节点骨架'))
        self.calibration_button.setAccessibleName(tr('打开本手的姿态标定工具'))

    def sync(self, preview):
        self.three_d_button.setChecked(preview.glove_view == "3d")
        self.skeleton_button.setChecked(preview.glove_view == "skeleton")


class GloveDetailDialog(QDialog):
    view_requested = Signal(str)
    calibration_requested = Signal()

    def __init__(self, source, device, parent=None):
        super().__init__(parent)
        self.identity = (device.get("id"), device.get("side"))
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.setWindowFlag(Qt.WindowType.WindowMaximizeButtonHint, True)
        self.setWindowTitle(device["label"] + tr(' · 手势放大查看'))
        self.resize(760, 740)
        self.setMinimumSize(600, 600)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        header = QHBoxLayout()
        self.title = QLabel(device["label"])
        self.title.setObjectName("sectionTitle")
        header.addWidget(self.title, 1)
        self.controls = GloveViewControls()
        self.controls.view_requested.connect(self._request_view)
        self.controls.calibration_requested.connect(self.calibration_requested)
        header.addWidget(self.controls)
        layout.addLayout(header)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        self.status.setObjectName("muted")
        self._status_text = ""
        layout.addWidget(self.status)
        self.preview = PreviewWidget("glove")
        self.preview.glove_view_changed.connect(lambda *_: self.controls.sync(self.preview))
        self.preview.glove_display_changed.connect(self._refresh_status)
        layout.addWidget(self.preview, 1)
        note = QLabel(tr('来自手势数据主题的解算结果 · 腕部相对坐标，非世界定位。\n拖动旋转、滚轮缩放；标定仍在原 FSGlove 窗口中完成。'))
        note.setWordWrap(True)
        note.setObjectName("muted")
        layout.addWidget(note)
        footer = QHBoxLayout()
        footer.addStretch()
        button = QPushButton(tr('关闭'))
        button.clicked.connect(self.close)
        footer.addWidget(button)
        layout.addLayout(footer)
        self.update_preview(source, device)
        self.set_glove_view(source.glove_view)

    def _request_view(self, mode):
        self.set_glove_view(mode)
        self.view_requested.emit(mode)

    def set_glove_view(self, mode):
        self.preview.set_glove_view(mode)
        self.controls.sync(self.preview)

    def update_preview(self, source, device):
        if self.identity != (device.get("id"), device.get("side")):
            raise ValueError(tr('手势放大窗口不能更换设备或手别'))
        self._status_text = tr('{v0} · 接收 {v1:.1f} Hz · 有效 {v2:.1f} Hz\n', v0=device.get('topic', ''), v1=device.get('receive_hz', 0), v2=device.get('valid_hz') or 0) + display_message(device.get("glove_hint", ""))
        self.preview.copy_glove_state_from(source)
        self.controls.sync(self.preview)
        self.controls.calibration_button.setEnabled(device.get("glove_action_enabled", True))

    def _refresh_status(self):
        status = self._status_text
        if self.preview.glove_source_summary:
            status += "\n" + self.preview.glove_source_summary
        self.status.setText(status)
