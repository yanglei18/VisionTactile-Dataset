"""Enlarged camera/Tracker views of an existing workbench snapshot cache.

No timers, subscriptions, decoding, process control, or hardware access. The
card's owner supplies updates; closing this window only closes this preview.
"""
from __future__ import annotations

from ...i18n import tr

from collections.abc import Mapping
import math

from PySide6.QtCore import QEvent, Qt
from ...i18n.widgets import QDialog, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout

from .preview import PreviewWidget


def _label(text="", *, name="muted", wrap=False):
    label = QLabel(text)
    label.setTextFormat(Qt.TextFormat.PlainText)
    label.setObjectName(name)
    label.setWordWrap(wrap)
    return label


def _hz(value):
    return tr('{v0:.1f}', v0=value) if type(value) in (int, float) and math.isfinite(value) and value >= 0 else "—"


class PreviewDetailDialog(QDialog):
    """A non-modal native window for exactly one camera or Tracker card."""

    def __init__(self, source_preview, device, parent=None):
        if not isinstance(source_preview, PreviewWidget) or source_preview.kind not in ("camera", "tracker"):
            raise ValueError(tr('放大预览仅支持相机和追踪器；手势/触觉使用各自的放大窗口'))
        if not isinstance(device, Mapping):
            raise ValueError(tr('放大预览需要对应设备信息'))
        super().__init__(parent)
        self.kind = source_preview.kind
        self.identity = (device.get("id"), device.get("side"))
        self._kind_label = tr('相机') if self.kind == "camera" else tr('追踪器')
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.setWindowFlag(Qt.WindowType.WindowMaximizeButtonHint, True)
        self.setWindowFlag(Qt.WindowType.WindowMinimizeButtonHint, True)
        self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False)
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.setModal(False)
        self.resize(1000, 720)
        self.setMinimumSize(640, 440)
        self.setSizeGripEnabled(True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        layout.setSpacing(10)
        header = QHBoxLayout()
        self.title = _label(name="sectionTitle")
        self.title.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        header.addWidget(self.title, 1)
        self.maximize_button = QPushButton(tr('最大化'))
        self.maximize_button.setAutoDefault(False)
        self.maximize_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.maximize_button.clicked.connect(self._toggle_maximized)
        header.addWidget(self.maximize_button)
        layout.addLayout(header)
        self.status = _label(wrap=True)
        layout.addWidget(self.status)
        self.preview = PreviewWidget(self.kind)
        # Match the card's initial inspection view once. Subsequent updates
        # preserve independent rotation and zoom in this enlarged window.
        self.preview.copy_view_state_from(source_preview)
        layout.addWidget(self.preview, 1)
        self.source_note = _label(
            tr('使用工作台已有的降采样 JPEG 预览；并非相机原始分辨率，放大不会增加图像细节。')
            if self.kind == "camera" else
            tr('左键拖动旋转 · 滚轮缩放 · 双击复位；与小图共用已收到的位姿和有限长度轨迹。'),
            wrap=True)
        layout.addWidget(self.source_note)
        self.readonly_note = _label(tr('只读放大预览 · 不建立新订阅或设备连接；关闭窗口不停止采集或录制。'), wrap=True)
        layout.addWidget(self.readonly_note)
        footer = QHBoxLayout()
        footer.addStretch()
        self.close_button = QPushButton(tr('关闭预览'))
        self.close_button.setAutoDefault(False)
        self.close_button.clicked.connect(self.close)
        footer.addWidget(self.close_button)
        layout.addLayout(footer)
        self._sync_maximize_button()
        self.update_preview(source_preview, device)

    def update_preview(self, source_preview, device):
        if (not isinstance(device, Mapping) or device.get("kind", self.kind) != self.kind
                or self.identity != (device.get("id"), device.get("side"))):
            raise ValueError(tr('放大窗口不能更换设备、设备类型或手别'))
        if (not isinstance(source_preview, PreviewWidget) or source_preview.kind != self.kind
                or source_preview.identity != self.identity):
            raise ValueError(tr('放大窗口必须使用同一设备的已有预览缓存'))
        self.preview.copy_camera_tracker_state_from(source_preview)
        title = str(device.get("label") or self._kind_label)
        self.title.setText(title)
        self.title.setToolTip(title)
        self.setWindowTitle(tr('{v0} · {v1}放大预览', v0=title, v1=self._kind_label))
        detail = tr('{v0} · 接收 {v1} Hz', v0=device.get('topic', ''), v1=_hz(device.get('receive_hz')))
        if self.kind == "tracker":
            detail += tr(' · 有效 {v0} Hz', v0=_hz(device.get('valid_hz')))
        self.status.setText(detail)
        self.status.setToolTip(source_preview.preview_error)

    def _toggle_maximized(self):
        self.showNormal() if self.isMaximized() else self.showMaximized()
        self._sync_maximize_button()

    def _sync_maximize_button(self):
        title = tr('恢复窗口') if self.isMaximized() else tr('最大化')
        self.maximize_button.setText(title)
        self.maximize_button.setAccessibleName(tr('{v0}{v1}预览', v0=title, v1=self._kind_label))
        self.maximize_button.setToolTip(tr('恢复普通窗口大小') if self.isMaximized() else tr('最大化当前预览窗口'))

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QEvent.Type.WindowStateChange and hasattr(self, "maximize_button"):
            self._sync_maximize_button()
