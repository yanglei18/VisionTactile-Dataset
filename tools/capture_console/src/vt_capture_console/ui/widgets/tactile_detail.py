"""Tactile-only display controls; reuse the console's validated snapshot cache.

No timers, device connections, subscriptions or calibration changes live here.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup, QDialog, QFrame, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QVBoxLayout, QWidget,
)

from .preview import PreviewWidget


def _label(text, name="muted", wrap=False):
    widget = QLabel(text)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setObjectName(name)
    widget.setWordWrap(wrap)
    return widget


class TactileViewControls(QWidget):
    """Small, keyboard-accessible selector shared by cards and detail windows."""

    view_requested = Signal(str)
    expand_requested = Signal()

    def __init__(self, *, expandable=False, parent=None):
        super().__init__(parent)
        self.setObjectName("tactileViewControls")
        self.setCursor(Qt.CursorShape.ArrowCursor)
        self.setStyleSheet("""
            QFrame#tactileSegments {
                background: #edf1f6; border: 1px solid #e0e6ee; border-radius: 8px;
            }
            QPushButton#tactileView, QPushButton#tactileExpand {
                color: #647184; background: transparent; border: 1px solid transparent;
                border-radius: 6px; padding: 3px 5px; font-size: 11px; font-weight: 500;
            }
            QPushButton#tactileView:checked {
                color: #0877db; background: white; border-color: #d9e2ee; font-weight: 600;
            }
            QPushButton#tactileView:hover, QPushButton#tactileExpand:hover {
                color: #0877db; background: #e4effc;
            }
            QPushButton#tactileView:disabled { color: #a3acb8; background: transparent; }
            QPushButton#tactileView:focus, QPushButton#tactileExpand:focus { border-color: #88bbff; }
            QPushButton#tactileExpand { color: #0877db; background: #edf5ff; }
        """)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        segments = QFrame()
        segments.setObjectName("tactileSegments")
        row = QHBoxLayout(segments)
        row.setContentsMargins(2, 2, 2, 2)
        row.setSpacing(2)
        self.group = QButtonGroup(self)
        self.hand_button = QPushButton("手形")
        self.matrix_button = QPushButton("矩阵")
        self.three_d_button = QPushButton("3D 手形")
        for index, button in enumerate((self.hand_button, self.matrix_button, self.three_d_button)):
            button.setObjectName("tactileView")
            button.setCheckable(True)
            button.setAutoDefault(False)
            button.setMinimumWidth(35)
            button.setFixedHeight(24)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            self.group.addButton(button, index)
            row.addWidget(button)
        self.hand_button.setAccessibleName("触觉手形视图")
        self.matrix_button.setAccessibleName("触觉矩阵视图")
        self.three_d_button.setAccessibleName("触觉三维 MANO 手形视图")
        self.matrix_button.setToolTip("原始 20 × 20 矩阵；保留行优先采样顺序")
        self.group.idClicked.connect(self._request_view)
        layout.addWidget(segments)
        layout.addStretch(1)
        self.expand_button = None
        if expandable:
            self.expand_button = QPushButton("放大")
            self.expand_button.setObjectName("tactileExpand")
            self.expand_button.setAutoDefault(False)
            self.expand_button.setFixedHeight(28)
            self.expand_button.setCursor(Qt.CursorShape.PointingHandCursor)
            self.expand_button.setAccessibleName("放大触觉预览")
            self.expand_button.setToolTip("放大查看此触觉数据流；不建立新的设备连接")
            self.expand_button.clicked.connect(self.expand_requested)
            layout.addWidget(self.expand_button)

    def _request_view(self, index):
        self.view_requested.emit(("hand", "matrix", "3d")[index])

    def sync(self, preview):
        self.hand_button.setEnabled(preview.tactile_hand_available)
        self.hand_button.setToolTip(preview.tactile_layout_note)
        self.hand_button.setChecked(preview.tactile_view == "hand")
        self.matrix_button.setChecked(preview.tactile_view == "matrix")
        self.three_d_button.setChecked(preview.tactile_view == "3d")
        self.three_d_button.setEnabled(preview.tactile_3d_available)
        self.three_d_button.setToolTip(
            "与手势标定工具相同的 MANO 模型 · 静态参考手，不代表实时手势；实时 ADC 触觉候选映射。"
            if preview.tactile_3d_available else preview.tactile_layout_note)


class TactileDetailDialog(QDialog):
    """A larger view of exactly one card, refreshed only by its existing owner."""

    view_requested = Signal(str)

    def __init__(self, source, device, parent=None):
        super().__init__(parent)
        self.identity = (device.get("id"), device.get("side"))
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.setWindowFlag(Qt.WindowType.WindowMaximizeButtonHint, True)
        self.setWindowTitle(f"{device['label']} · 触觉放大查看")
        self.resize(760, 740)
        self.setMinimumSize(600, 600)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        layout.setSpacing(10)
        header = QHBoxLayout()
        self.title = _label(device["label"], "sectionTitle")
        self.title.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        header.addWidget(self.title, 1)
        self.controls = TactileViewControls()
        self.controls.view_requested.connect(self._request_view)
        header.addWidget(self.controls)
        layout.addLayout(header)
        self.stream = _label("", wrap=True)
        layout.addWidget(self.stream)
        self.preview = PreviewWidget("tactile")
        self.preview.tactile_view_changed.connect(self._sync_controls)
        layout.addWidget(self.preview, 1)
        self.layout_note = _label("", wrap=True)
        layout.addWidget(self.layout_note)
        layout.addWidget(_label("颜色表示原始 ADC，不是压力；灰色为无效采样，3D 未映射部位保留模型底色。", wrap=True))
        layout.addWidget(_label("只读预览 · 共用工作台快照；不启动设备、不修改标定。", wrap=True))
        footer = QHBoxLayout()
        footer.addStretch()
        self.close_button = QPushButton("关闭")
        self.close_button.setAutoDefault(False)
        self.close_button.clicked.connect(self.close)
        footer.addWidget(self.close_button)
        layout.addLayout(footer)
        self.update_preview(source, device)
        self.set_tactile_view(source.tactile_view)

    def _request_view(self, mode):
        self.set_tactile_view(mode)
        self.view_requested.emit(mode)

    def set_tactile_view(self, mode):
        self.preview.set_tactile_view(mode)
        self._sync_controls()

    def _sync_controls(self, *_args):
        self.controls.sync(self.preview)
        full_note = self.preview.tactile_layout_note
        if self.preview.tactile_view == "3d":
            summary = "静态 MANO 参考手 · 非实时手势；拖动旋转、滚轮缩放。触觉表面为候选映射。"
        elif self.preview.tactile_hand_available:
            summary = "右手掌面 · 400 点候选位置；描边锚点已观测，尚非逐点标定。"
        elif self.identity[1] == "left":
            summary = "左手未映射 · 仅原始 20 × 20 矩阵，不镜像或推断手形。"
        elif self.identity[1] == "right":
            summary = "右手布局不可用 · 仅原始 20 × 20 矩阵。"
        else:
            summary = "未指定左右手 · 仅原始 20 × 20 矩阵。"
        self.layout_note.setText(summary)
        self.layout_note.setToolTip(full_note)
        self.layout_note.setAccessibleDescription(full_note)

    def update_preview(self, source, device):
        if self.identity != (device.get("id"), device.get("side")):
            raise ValueError("触觉放大窗口不能更换设备或左右手")
        # Copy already validated state, including history/DEMO/replay markers.
        # In particular, opening a dialog while offline must not invent an
        # online sample just to seed its last-good-frame cache.
        self.preview.copy_display_state_from(source)
        self.title.setText(device["label"])
        self.setWindowTitle(f"{device['label']} · 触觉放大查看")
        self.stream.setText(f"{device.get('topic', '')}  ·  接收 {device.get('receive_hz', 0.):.1f} Hz")
        self._sync_controls()
