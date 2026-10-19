"""Reusable device presentation for live capture and archive playback.

The card emits selection/calibration requests; it never imports the application
window or owns sensor processes. MainWindow re-exports DeviceCard temporarily
for existing integrations; new consumers should import this module directly.
"""
from __future__ import annotations

from copy import deepcopy

from PySide6.QtCore import QEvent, QSize, Qt, Signal

from ...i18n import tr
from ...i18n.messages import message as diagnostic
from ...i18n.widgets import QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout
from ..icons import icon
from .glove_detail import GloveDetailDialog, GloveViewControls
from .preview import PreviewWidget
from .preview_detail import PreviewDetailDialog
from .tactile_detail import TactileDetailDialog, TactileViewControls


STATUS = {"online": (tr('在线'), "#f5f5f5"), "stale": (tr('数据陈旧'), "#bdbdbd"),
          "offline": (tr('离线'), "#999999"), "unknown": (tr('等待数据'), "#808080")}


def _label(text, name="", wrap=False):
    result = QLabel(text)
    result.setTextFormat(Qt.TextFormat.PlainText)
    result.setObjectName(name)
    result.setWordWrap(wrap)
    return result


class DeviceCard(QFrame):
    selected = Signal(str)
    calibration_requested = Signal(str)

    def __init__(self, device, parent=None, *, compact=False, embedded_replay=False):
        super().__init__(parent)
        self.embedded_replay = embedded_replay
        self.identifier = device["id"]
        self._device = deepcopy(device)
        self.tactile_dialog = None
        self.tactile_controls = None
        self.glove_dialog = None
        self.glove_controls = None
        self.preview_dialog = None
        self.setObjectName("card")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumWidth(190)
        self.setProperty("compact", compact)
        if compact:
            self.setFixedWidth(224)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8) if compact else layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(4 if compact else 8)
        header = QHBoxLayout()
        header.setSpacing(8)
        kind_icon = _label("")
        kind_icon.setPixmap(icon(device["kind"], "#b5b5b5", 18).pixmap(18, 18))
        kind_icon.setFixedSize(20, 20)
        self.title = _label(device["label"], "cardTitle")
        self.title.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.title.setToolTip(device["label"])
        self.state = _label("")
        header.addWidget(kind_icon)
        header.addWidget(self.title, 1)
        layout.addLayout(header)
        metadata = QHBoxLayout()
        metadata.setSpacing(6)
        metadata.addWidget(_label({"camera": tr('相机'), "tracker": tr('追踪器'), "glove": tr('手部姿态'), "tactile": tr('触觉')}[device["kind"]], "cardKind"))
        metadata.addStretch()
        metadata.addWidget(self.state)
        layout.addLayout(metadata)
        self.preview = PreviewWidget(device["kind"])
        self.preview.setFixedHeight(100 if compact else 256 if device["kind"] in ("tactile", "glove") else 166)
        if compact:
            self.preview.set_presentation("thumbnail")
        self.preview.installEventFilter(self)
        if device["kind"] == "tactile":
            self.tactile_controls = TactileViewControls(expandable=True)
            self.tactile_controls.view_requested.connect(self.preview.set_tactile_view)
            self.tactile_controls.expand_requested.connect(self.open_detail)
            self.preview.tactile_view_changed.connect(self._sync_tactile_controls)
            layout.addWidget(self.tactile_controls)
        if device["kind"] == "glove":
            self.glove_controls = GloveViewControls(expandable=True)
            self.glove_controls.view_requested.connect(self.preview.set_glove_view)
            self.glove_controls.expand_requested.connect(self.open_detail)
            self.glove_controls.calibration_requested.connect(self._request_calibration)
            self.preview.glove_view_changed.connect(self._sync_glove_controls)
            layout.addWidget(self.glove_controls)
        # Keep a single, consistently placed expansion control on every card.
        # Reuse the existing hand controls' button/API, moving it to the header.
        controls = self.tactile_controls or self.glove_controls
        if controls is not None:
            self.expand_button = controls.expand_button
            controls.layout().removeWidget(self.expand_button)
        else:
            self.expand_button = QPushButton(tr('放大'))
            self.expand_button.clicked.connect(self.open_detail)
        self.expand_button.setObjectName("previewExpand")
        self.expand_button.setAutoDefault(False)
        self.expand_button.setFixedHeight(28)
        self.expand_button.setIcon(icon("expand", "#eeeeee", 14))
        self.expand_button.setIconSize(QSize(14, 14))
        self.expand_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.expand_button.setAccessibleName(tr('放大 ') + device["label"])
        self.expand_button.setToolTip(tr('在档案内聚焦此数据流') if embedded_replay else
                                     tr('在独立窗口放大查看；不启停设备、不影响录制'))
        header.addWidget(self.expand_button)
        if device["kind"] == "camera" and not embedded_replay:
            self.preview.setToolTip(tr('双击画面或点击右上角“放大”，打开实时大图'))
        layout.addWidget(self.preview)
        if self.tactile_controls is not None:
            self.tactile_note = _label("", "muted")
            self.tactile_note.setStyleSheet("color: #8a8a8a; font-size: 10px;")
            self.tactile_note.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            layout.addWidget(self.tactile_note)
        if self.glove_controls is not None:
            self.glove_note = _label("", "muted", wrap=True)
            self.glove_note.setMaximumHeight(36)
            layout.addWidget(self.glove_note)
        self.metrics = _label("", "cardMetrics")
        self.metrics.setToolTip(tr('接收 Hz 为后台最近一秒收到的消息数；不表示相机输出帧率或数据包写入帧率。录制完整性请查看档案中的质量报告。'))
        layout.addWidget(self.metrics)
        if compact:
            self.metrics.hide()
            for widget in (self.glove_controls, self.tactile_controls, getattr(self, "glove_note", None), getattr(self, "tactile_note", None)):
                if widget is not None:
                    widget.hide()
        self.update_device(device)
        if embedded_replay and self.glove_controls is not None:
            self.glove_controls.calibration_button.hide()
        if self.glove_controls is not None and not device.get("preview", {}).get("demo"):
            self.preview.set_glove_view("3d")

    @property
    def device_snapshot(self) -> dict:
        """Return isolated presentation data, including nested preview metadata."""
        return deepcopy(self._device)

    def clear_frame(self) -> None:
        """Discard the displayed sample and its history before a seek or reset."""
        self.preview.clear_frame()
        self._refresh_presentation()

    def update_metadata(self, device: dict) -> None:
        """Refresh an unchanged sample's metadata without decoding it again.

        The preview validates sample identity and content before either widget
        accepts the update. New samples must use update_device().
        """
        device = self._presentation_device(device)
        self.preview.refresh_metadata(device)
        self._device = device
        self._refresh_presentation()

    def update_device(self, device: dict) -> None:
        """Display a sample and synchronize any owned detail views."""
        device = self._presentation_device(device)
        if ((self._device.get("id"), self._device.get("side"))
                != (device.get("id"), device.get("side"))):
            self.close_details()
        self._device = device
        self.preview.set_device(device)
        self._refresh_presentation()

    def _presentation_device(self, device: dict) -> dict:
        """Keep archive presentation historical, including empty samples."""
        snapshot = deepcopy(device)
        if self.embedded_replay:
            snapshot["mode"] = "replay"
            preview = snapshot.get("preview")
            if isinstance(preview, dict) and preview:
                preview.pop("demo", None)
                preview.update(mode="replay", replay=True)
        return snapshot

    def _refresh_presentation(self):
        device = self._device
        state, color = STATUS.get(device["status"], STATUS["unknown"])
        self.state.setText("● " + state)
        status_style = tr('color: {v0}; font-size: 11px; background: transparent;', v0=color)
        if self.state.styleSheet() != status_style:
            self.state.setStyleSheet(status_style)
        glove_label = {"STARTABLE": tr('未启动'), "STOPPED": tr('未启动'), "DISABLED": tr('未启用'),
                       "RUNNING_NO_DATA": tr('已启动 · 待结果'), "INVALID_DATA": tr('姿态待就绪'),
                       "STALE_DATA": tr('姿态断流')}.get(device.get("glove_state"))
        if glove_label:
            self.state.setText("● " + glove_label)
        rate = device["receive_hz"]
        valid = device.get("valid_hz")
        age = device.get("age_s")
        tail = tr('有效 {v0:.1f} Hz', v0=valid) if valid is not None else (tr('接收龄期 {v0:.0f} ms', v0=age * 1000) if age is not None else tr('尚无样本'))
        self.metrics.setText(tr('接收 {v0:.1f} Hz    ·    {v1}', v0=rate, v1=tail))
        if self.embedded_replay:
            self.state.setText(tr('历史样本') if device.get("replay_available") else
                               tr('未录制') if device.get("recorded_count") == 0 else tr('无有效样本'))
            self.state.setToolTip(device.get("detail", ""))
        if self.preview_dialog is not None:
            self.preview_dialog.update_preview(self.preview, device)
        if self.tactile_controls is not None:
            self._sync_tactile_controls()
            if self.tactile_dialog is not None:
                self.tactile_dialog.update_preview(self.preview, device)
        if self.glove_controls is not None:
            self.glove_note.setText(diagnostic(device.get("glove_hint", tr('MANO 解算结果 · 腕部相对坐标'))))
            self.glove_note.setToolTip(diagnostic(device.get("glove_hint", "")) + "\n" + diagnostic(device.get("detail", "")))
            self.glove_controls.calibration_button.setEnabled(device.get("glove_action_enabled", True))
            self._sync_glove_controls()
            if self.glove_dialog is not None:
                self.glove_dialog.update_preview(self.preview, device)

    def open_detail(self):
        if self.embedded_replay:
            self.selected.emit(self.identifier)
            return
        if self._device["kind"] == "tactile":
            self.open_tactile_detail()
        elif self._device["kind"] == "glove":
            self.open_glove_detail()
        else:
            if self.preview_dialog is None:
                self.preview_dialog = PreviewDetailDialog(self.preview, self._device, self.window())
                self.preview_dialog.finished.connect(self._preview_detail_closed)
            if self.preview_dialog.isMinimized():
                self.preview_dialog.showNormal()
            else:
                self.preview_dialog.show()
            self.preview_dialog.raise_()
            self.preview_dialog.activateWindow()

    def _preview_detail_closed(self, *_args):
        self.preview_dialog = None

    def close_details(self):
        if self.preview_dialog is not None:
            self.preview_dialog.close()
        self.close_tactile_detail()
        self.close_glove_detail()

    def _request_calibration(self):
        if self.embedded_replay:
            return
        self.calibration_requested.emit(self._device.get("side", ""))

    def _sync_glove_controls(self, *_args):
        self.glove_controls.sync(self.preview)
        if self.glove_dialog is not None:
            self.glove_dialog.set_glove_view(self.preview.glove_view)

    def open_glove_detail(self):
        if self.embedded_replay:
            self.selected.emit(self.identifier)
            return
        if self.glove_controls is None:
            return
        if self.glove_dialog is None:
            dialog = GloveDetailDialog(self.preview, self._device, self.window())
            dialog.view_requested.connect(self.preview.set_glove_view)
            dialog.calibration_requested.connect(self._request_calibration)
            dialog.finished.connect(self._glove_detail_closed)
            self.glove_dialog = dialog
        if self.glove_dialog.isMinimized():
            self.glove_dialog.showNormal()
        else:
            self.glove_dialog.show()
        self.glove_dialog.raise_()
        self.glove_dialog.activateWindow()

    def _glove_detail_closed(self, *_args):
        self.glove_dialog = None

    def close_glove_detail(self):
        if self.glove_dialog is not None:
            self.glove_dialog.close()

    def _sync_tactile_controls(self, *_args):
        self.tactile_controls.sync(self.preview)
        note = tr('候选映射') if self.preview.tactile_hand_available else tr('映射未确认')
        if self.preview.tactile_view == "3d":
            note = tr('静态参考手 · 候选映射')
        self.tactile_note.setText(tr('{v0} · ADC（非压力）', v0=note))
        self.tactile_note.setToolTip(self.preview.tactile_layout_note + tr('\n颜色表示原始 ADC 读数，不是标定压力。'))
        if self.tactile_dialog is not None:
            self.tactile_dialog.set_tactile_view(self.preview.tactile_view)

    def open_tactile_detail(self):
        if self.embedded_replay:
            self.selected.emit(self.identifier)
            return
        if self.tactile_controls is None:
            return
        if self.tactile_dialog is None:
            dialog = TactileDetailDialog(self.preview, self._device, self.window())
            dialog.view_requested.connect(self.preview.set_tactile_view)
            dialog.finished.connect(self._tactile_detail_closed)
            self.tactile_dialog = dialog
        if self.tactile_dialog.isMinimized():
            self.tactile_dialog.showNormal()
        else:
            self.tactile_dialog.show()
        self.tactile_dialog.raise_()
        self.tactile_dialog.activateWindow()

    def _tactile_detail_closed(self, *_args):
        self.tactile_dialog = None

    def close_tactile_detail(self):
        if self.tactile_dialog is not None:
            self.tactile_dialog.close()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.selected.emit(self.identifier)
        super().mousePressEvent(event)

    def eventFilter(self, watched, event):
        if (watched is self.preview and self._device["kind"] == "camera"
                and event.type() == QEvent.Type.MouseButtonDblClick
                and event.button() == Qt.MouseButton.LeftButton):
            self.open_detail()
            return True
        if watched is self.preview and event.type() == QEvent.Type.MouseButtonPress:
            if event.button() == Qt.MouseButton.LeftButton:
                self.selected.emit(self.identifier)
        return super().eventFilter(watched, event)
