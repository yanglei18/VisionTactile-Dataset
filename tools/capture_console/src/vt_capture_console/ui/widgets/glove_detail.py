"""Gesture controls and enlarged preview, sharing the main snapshot/renderer."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QButtonGroup, QDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

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
            QPushButton {font-size: 11px; padding: 3px 5px; border-radius: 6px;}
            QPushButton:checked {color: #0877db; background: #edf5ff; border-color: #b7d6ff;}
        """)
        self.group = QButtonGroup(self)
        self.three_d_button = QPushButton("3D 手形")
        self.skeleton_button = QPushButton("骨架")
        for i, button in enumerate((self.three_d_button, self.skeleton_button)):
            button.setCheckable(True)
            button.setAutoDefault(False)
            self.group.addButton(button, i)
            row.addWidget(button)
        self.group.idClicked.connect(lambda i: self.view_requested.emit(("3d", "skeleton")[i]))
        self.three_d_button.setToolTip("从收到的 MANO 解算参数重建网格，不重复进行 IMU 解算")
        self.skeleton_button.setToolTip("查看 Topic 中原有的 21 个关节点；不依赖许可模型")
        row.addStretch()
        self.calibration_button = QPushButton("标定")
        self.calibration_button.setAutoDefault(False)
        self.calibration_button.setToolTip("打开本手的原标定工具；已运行时切换窗口，不另建采集进程")
        self.calibration_button.clicked.connect(self.calibration_requested)
        row.addWidget(self.calibration_button)
        self.expand_button = None
        if expandable:
            self.expand_button = QPushButton("放大")
            self.expand_button.setAutoDefault(False)
            self.expand_button.clicked.connect(self.expand_requested)
            row.addWidget(self.expand_button)
        for button in self.findChildren(QPushButton):
            button.setFixedHeight(28)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.three_d_button.setAccessibleName("三维手势模型")
        self.skeleton_button.setAccessibleName("手势关节点骨架")
        self.calibration_button.setAccessibleName("打开本手的姿态标定工具")

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
        self.setWindowTitle(device["label"] + " · 手势放大查看")
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
        layout.addWidget(self.status)
        self.preview = PreviewWidget("glove")
        self.preview.glove_view_changed.connect(lambda *_: self.controls.sync(self.preview))
        layout.addWidget(self.preview, 1)
        note = QLabel("来自手势 Topic 的解算结果 · 腕部相对坐标，非世界定位。\n拖动旋转、滚轮缩放；标定仍在原 FSGlove 窗口中完成。")
        note.setWordWrap(True)
        note.setObjectName("muted")
        layout.addWidget(note)
        footer = QHBoxLayout()
        footer.addStretch()
        button = QPushButton("关闭")
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
            raise ValueError("手势放大窗口不能更换设备或手别")
        self.preview.copy_glove_state_from(source)
        self.status.setText(f"{device.get('topic', '')} · 接收 {device.get('receive_hz', 0):.1f} Hz · "
                            f"有效 {device.get('valid_hz') or 0:.1f} Hz\n" + device.get("glove_hint", ""))
        self.controls.sync(self.preview)
        self.controls.calibration_button.setEnabled(device.get("glove_action_enabled", True))
