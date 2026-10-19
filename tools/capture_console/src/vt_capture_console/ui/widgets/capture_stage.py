"""Focused workbench preview fed by the existing device-card display cache.

The stage never receives sensor data itself. Its owner refreshes it after the
selected card; decoded images and shared MANO services remain those of the
workbench. Switching streams only changes this presentation surface.
"""
from __future__ import annotations

from ...i18n import tr

from collections.abc import Mapping
import math

from PySide6.QtCore import QEasingCurve, QEvent, QPropertyAnimation, Qt, Signal
from ...i18n.widgets import (
    QFrame, QGraphicsOpacityEffect, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QStackedWidget,
    QVBoxLayout, QWidget,
)
from shiboken6 import isValid

from .glove_detail import GloveViewControls
from .preview import PreviewWidget
from .tactile_detail import TactileViewControls


_KINDS = {"camera": tr('相机'), "tracker": tr('追踪器'),
          "glove": tr('手势'), "tactile": tr('触觉')}
_STATES = {"online": tr('在线'), "stale": tr('数据延迟'), "offline": tr('离线'), "unknown": tr('等待数据')}


def _label(text="", name="stageHint"):
    widget = QLabel(text)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setObjectName(name)
    widget.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    return widget


def _rate(value):
    return tr('{v0:.1f}', v0=value) if type(value) in (float, int) and math.isfinite(value) and value >= 0 else "—"


class CaptureStage(QFrame):
    """One large, selectable stream view without a second acquisition path."""

    expand_requested = Signal()
    calibration_requested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("captureStage")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumHeight(220)
        self._source = None
        self._device = {}
        self._identity = None
        self._overlay_signature = None
        self._empty_ready = False
        self._connections = []
        self.preview = None

        self._motion_enabled = True
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.header = QFrame()
        self.header.setObjectName("stageHeader")
        row = QHBoxLayout(self.header)
        row.setContentsMargins(22, 8, 22, 8)
        row.setSpacing(16)
        self.eyebrow = _label(tr('数据流监看'), "stageEyebrow")
        self.eyebrow.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        self.title = _label(tr('选择一个数据流'), "stageTitle")
        row.addWidget(self.eyebrow)
        row.addWidget(self.title, 1)
        row.addStretch(1)
        self.status = _label("", "stageStatus")
        self.status.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        row.addWidget(self.status)
        self.expand_button = QPushButton(tr('展开画面 ↗'))
        self.expand_button.setObjectName("stageExpand")
        self.expand_button.setAutoDefault(False)
        self.expand_button.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.expand_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.expand_button.setAccessibleName(tr('在独立窗口查看所选数据流'))
        self.expand_button.clicked.connect(self.expand_requested)
        row.addWidget(self.expand_button)
        layout.addWidget(self.header)

        self.glove_controls = GloveViewControls()
        self.tactile_controls = TactileViewControls()
        for controls in (self.glove_controls, self.tactile_controls):
            controls.setStyleSheet("")
            controls.setObjectName("stageControls")
            controls.view_requested.connect(self._request_view)
            for button in controls.findChildren(QPushButton):
                button.setObjectName("stageViewSelector")
            layout.addWidget(controls)
        self.glove_controls.calibration_button.setObjectName("stageAction")
        self.glove_controls.calibration_requested.connect(self._request_calibration)

        self.canvas = QStackedWidget()
        self.canvas.setObjectName("stageCanvas")
        self.canvas.setMinimumHeight(120)
        self.canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.canvas.installEventFilter(self)
        self.empty = QWidget()
        self.empty.setObjectName("stageEmpty")
        self.canvas.addWidget(self.empty)
        layout.addWidget(self.canvas, 1)

        # Intro is presentation chrome, never drawn over live/replayed pixels.
        # A matching reserved area in synthetic/empty previews keeps the
        # illustration separate. The original source cache is never changed.
        self.hero = QWidget(self.canvas)
        self.hero.setObjectName("stageHero")
        self.hero.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        hero_layout = QVBoxLayout(self.hero)
        hero_layout.setContentsMargins(0, 0, 0, 0)
        hero_layout.setSpacing(13)
        hero_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.hero_kicker = _label(tr('多模态观测'), "stageHeroKicker")
        self.hero_title = _label(tr('感知，\n在此汇聚。'), "stageHeroTitle")
        self.hero_lead = _label(tr('展开下方数据流，进入你的观测现场。'), "stageHeroLead")
        self.hero_lead.setWordWrap(True)
        hero_layout.addWidget(self.hero_kicker)
        hero_layout.addWidget(self.hero_title)
        hero_layout.addWidget(self.hero_lead)
        self._hero_opacity = QGraphicsOpacityEffect(self.hero)
        self._hero_opacity.setOpacity(1.)
        self.hero.setGraphicsEffect(self._hero_opacity)
        self._hero_animation = QPropertyAnimation(self._hero_opacity, b"opacity", self)
        self._hero_animation.setDuration(380)
        self._hero_animation.setEasingCurve(QEasingCurve.Type.OutCubic)
        # Keep these aliases for callers that inspect the empty presentation.
        self.empty_title, self.empty_hint = self.hero_title, self.hero_lead

        self.metric_panel = QWidget(self.canvas)
        self.metric_panel.setObjectName("stageMetric")
        self.metric_panel.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        metric_layout = QVBoxLayout(self.metric_panel)
        metric_layout.setContentsMargins(0, 0, 0, 0)
        metric_layout.setSpacing(0)
        self.metric_value = _label("—", "stageMetricValue")
        self.metric_caption = _label(tr('接收 / Hz'), "stageMetricCaption")
        for item in (self.metric_value, self.metric_caption):
            item.setAlignment(Qt.AlignmentFlag.AlignRight)
            metric_layout.addWidget(item)

        self.footer = QFrame()
        self.footer.setObjectName("stageFooter")
        footer = QHBoxLayout(self.footer)
        footer.setContentsMargins(22, 5, 22, 9)
        footer.setSpacing(12)
        self.stream_info = _label("", "stageStream")
        self.hint = _label()
        self.hint.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        footer.addWidget(self.stream_info, 3)
        footer.addWidget(self.hint, 2)
        layout.addWidget(self.footer)
        self.clear_source()

    @property
    def source_preview(self):
        return self._source

    def refresh_layout(self):
        """Reflow presentation chrome after language, font or style changes.

        The selected source and its decoded frame stay unchanged.
        """
        self._position_overlays(force=True)

    def set_source(self, source_preview, device):
        """Select a card's already validated cache, keeping its data identity."""
        self._validate_source(source_preview, device)
        identity = (source_preview.kind, device.get("id"), device.get("side"))
        if source_preview is self._source and identity == self._identity:
            self.refresh_source(device)
            return
        self._disconnect_source()
        self._empty_ready = False
        if identity != self._identity or self.preview is None:
            self._dispose_preview()
            self.preview = PreviewWidget(source_preview.kind)
            self.preview.set_presentation("stage")
            self.preview.set_motion_enabled(self._motion_enabled)
            self.preview.setMinimumHeight(120)
            self.preview.copy_view_state_from(source_preview)
            self.canvas.addWidget(self.preview)
        self._source = source_preview
        self._identity = identity
        self._connect(source_preview.destroyed, self._source_destroyed)
        if source_preview.kind == "glove":
            self._connect(source_preview.glove_view_changed, self._source_view_changed)
        elif source_preview.kind == "tactile":
            self._connect(source_preview.tactile_view_changed, self._source_view_changed)
        self.canvas.setCurrentWidget(self.preview)
        self.glove_controls.setVisible(source_preview.kind == "glove")
        self.tactile_controls.setVisible(source_preview.kind == "tactile")
        self.expand_button.setEnabled(True)
        self.refresh_source(device)
        self._position_overlays(force=True)
        self._reveal_hero()

    @staticmethod
    def _validate_source(source, device):
        if not isinstance(source, PreviewWidget) or not isValid(source) or not isinstance(device, Mapping):
            raise ValueError(tr('聚焦预览需要已有的数据流画面和设备信息'))
        if (device.get("kind") != source.kind or source.identity != (device.get("id"), device.get("side"))):
            raise ValueError(tr('聚焦预览必须使用同一设备类型、标识和手别的缓存'))

    def refresh_source(self, device=None):
        """Refresh once after the owner updates the source card; no own timer."""
        source = self._source
        if source is None or not isValid(source):
            self.clear_source()
            return
        device = source.device_snapshot if device is None else device
        self._validate_source(source, device)
        identity = (source.kind, device.get("id"), device.get("side"))
        if identity != self._identity:
            self.set_source(source, device)
            return
        self._device = dict(device)
        if source.kind in ("camera", "tracker"):
            self.preview.copy_camera_tracker_state_from(source)
        elif source.kind == "glove":
            self.preview.copy_glove_state_from(source)
            if self.preview.glove_view != source.glove_view:
                self.preview.set_glove_view(source.glove_view)
            self.glove_controls.sync(source)
            self.glove_controls.calibration_button.setEnabled(
                not self._is_replay() and device.get("glove_action_enabled", True))
        else:
            self.preview.copy_display_state_from(source)
            if self.preview.requested_tactile_view != source.requested_tactile_view:
                self.preview.set_tactile_view(source.requested_tactile_view)
            self.tactile_controls.sync(source)
        self._refresh_labels()

    def _refresh_labels(self):
        source, device = self._source, self._device
        if self._is_replay():
            self._refresh_replay_labels()
            return
        self.canvas.setCurrentWidget(self.preview)
        mode = {"demo": tr('演示'), "replay": tr('回放')}.get(source.mode, tr('实时'))
        self.eyebrow.setText(_KINDS[source.kind])
        self.title.setText(str(device.get("label") or device.get("id") or tr('数据流')))
        self.title.setToolTip(self.title.text())
        introductory = source.mode == "demo" or not source.has_frame
        self.title.setVisible(not introductory)
        self.hero.setVisible(introductory)
        self.metric_panel.setVisible(introductory and source.has_frame)
        self.preview.set_stage_intro(introductory)
        demo = source.mode == "demo"
        self.hero_kicker.setText(tr('合成演示') if demo else tr('等待信号'))
        self.hero_title.setText({"camera": tr('让感知\n触手可及。'), "tracker": tr('每一寸移动，\n尽在掌握。'),
                                "glove": tr('读懂\n每一个手势。'), "tactile": tr('看见触碰\n的细微之处。')}.get(source.kind, tr('感知，在此汇聚。'))
                               if demo else {"camera": tr('等待相机'), "tracker": tr('等待位姿。'),
                                             "glove": tr('等待手势。'), "tactile": tr('等待触觉。')}[source.kind])
        self.hero_lead.setText((self.title.text() + tr(' · 合成示意，非真实采集画面。')) if demo
                               else self.title.text() + tr(' · 数据接入后自动显示。'))
        self.metric_value.setText(_rate(device.get("receive_hz")))
        self.metric_caption.setText(tr('示意接收 / Hz') if demo else tr('接收 / Hz'))
        self._position_overlays()
        state = str(device.get("status", "unknown"))
        status = _STATES.get(state, tr('等待数据'))
        if source.held:
            status += tr(' · 保持上一画面')
        self.status.setText(tr('● {v0} / {v1}', v0=mode, v1=status))
        if self.status.property("state") != state:
            self.status.setProperty("state", state)
            self.status.style().unpolish(self.status)
            self.status.style().polish(self.status)
        self.status.setToolTip(source.preview_error)
        detail = tr('{v0}    ·    接收 {v1} Hz', v0=device.get('topic') or device.get('id') or tr('数据流'), v1=_rate(device.get('receive_hz')))
        if source.kind in ("tracker", "glove"):
            detail += tr('    ·    有效 {v0} Hz', v0=_rate(device.get('valid_hz')))
        self.stream_info.setText(detail)
        self.stream_info.setToolTip(detail)
        if source.kind == "camera":
            hint = tr('合成传感器示意 · 非真实影像') if source.mode == "demo" else tr('等比显示相机预览')
        elif source.kind == "tracker":
            hint = tr('拖动旋转 · 滚轮缩放 · 双击复位')
        elif source.kind == "glove":
            hint = tr('腕部相对坐标 · 拖动旋转 · 滚轮缩放')
        else:
            hint = (tr('静态参考手 · ') if source.tactile_view == "3d" else "") + (
                tr('候选手形映射 · 原始 ADC') if source.tactile_hand_available else tr('未映射手别 · 原始 ADC 矩阵'))
        self.hint.setText(hint)
        self.hint.setToolTip(source.tactile_layout_note if source.kind == "tactile" else hint)

    def _is_replay(self):
        preview = self._device.get("preview")
        return (self._device.get("mode") == "replay"
                or isinstance(preview, Mapping) and (preview.get("mode") == "replay" or preview.get("replay") is True)
                or self._source is not None and self._source.mode == "replay")

    def _refresh_replay_labels(self):
        """Describe stored samples without borrowing live connection semantics."""
        source, device = self._source, self._device
        available = source.has_frame and device.get("replay_available", True)
        self.eyebrow.setText(_KINDS[source.kind])
        self.title.setText(str(device.get("label") or device.get("id") or tr('数据流')))
        self.title.setToolTip(self.title.text())
        self.title.show()
        self.canvas.setCurrentWidget(self.preview if available else self.empty)
        self.hero.setVisible(not available)
        self.hero_kicker.setText(tr('历史回放'))
        self.hero_title.setText(tr('该时刻无样本'))
        self.hero_lead.setText(str(device.get("detail") or source.preview_error
                                   or tr('此话题在所选回放时刻没有可显示的样本。')))
        self.metric_panel.hide()
        self.metric_value.clear()
        self.metric_caption.clear()
        self.preview.set_stage_intro(False)
        self._position_overlays()
        self.status.setText(tr('● {v0} / {v1}', v0=tr('回放'),
                               v1=tr('历史样本') if available else tr('该时刻无样本')))
        state = "ready" if available else "unknown"
        if self.status.property("state") != state:
            self.status.setProperty("state", state)
            self.status.style().unpolish(self.status)
            self.status.style().polish(self.status)
        self.status.setToolTip(str(device.get("detail") or source.preview_error))
        detail = str(device.get("topic") or device.get("id") or tr('数据流'))
        age = device.get("sample_age_s")
        if available and type(age) in (int, float) and math.isfinite(age) and age >= 0:
            detail += tr('    ·    距播放位置 {v0:.3f} 秒', v0=age)
        self.stream_info.setText(detail)
        timestamps = [detail]
        for key, label in (("bag_timestamp_ns", tr('录制时间戳')),
                           ("source_realtime_ns", tr('源时间戳'))):
            if type(device.get(key)) is int:
                timestamps.append(f"{label}: {device[key]} ns")
        self.stream_info.setToolTip("\n".join(timestamps))
        hints = {"camera": tr('等比显示相机预览'),
                 "tracker": tr('拖动旋转 · 滚轮缩放 · 双击复位'),
                 "glove": tr('腕部相对坐标 · 拖动旋转 · 滚轮缩放'),
                 "tactile": tr('历史触觉样本 · 原始 ADC')}
        self.hint.setText(hints[source.kind] if available else tr('查看缺失提示或选择其他话题'))
        self.hint.setToolTip(self.hint.text())

    def _request_view(self, view):
        if self._source is None or not isValid(self._source):
            return
        if self._source.kind == "glove":
            self._source.set_glove_view(view)
        elif self._source.kind == "tactile":
            self._source.set_tactile_view(view)
        self.refresh_source()

    def _request_calibration(self):
        if (self._source is not None and self._device.get("kind") == "glove"
                and not self._is_replay() and self._device.get("glove_action_enabled", True)):
            self.calibration_requested.emit(self._device.get("side", ""))

    def _source_view_changed(self, *_args):
        self.refresh_source()

    def _connect(self, signal, callback):
        signal.connect(callback)
        self._connections.append((signal, callback))

    def _disconnect_source(self):
        for signal, callback in self._connections:
            try:
                signal.disconnect(callback)
            except (RuntimeError, TypeError):
                pass
        self._connections.clear()

    def _source_destroyed(self, *_args):
        self._connections.clear()
        if isValid(self):
            self.clear_source()

    def _dispose_preview(self):
        if self.preview is None:
            return
        for name in ("_glove_3d", "_tactile_3d"):
            renderer = getattr(self.preview, name)
            if renderer is not None:
                renderer.dispose()
                setattr(self.preview, name, None)
        self.canvas.removeWidget(self.preview)
        self.preview.hide()
        self.preview.deleteLater()
        self.preview = None

    def clear_source(self):
        """Release this view's connections while leaving the source running."""
        if self._empty_ready and self._source is None and self.preview is None:
            return
        # QGraphicsOpacityEffect defaults to 0.7. Empty/live-disconnected
        # views must be fully legible, including when a source vanishes during
        # its entry animation. Repeated empty refreshes never restart motion.
        self._hero_animation.stop()
        self._hero_opacity.setOpacity(1.)
        self._disconnect_source()
        self._source = None
        self._identity = None
        self._device = {}
        self._dispose_preview()
        self.canvas.setCurrentWidget(self.empty)
        self.eyebrow.setText(tr('数据流监看'))
        self.title.setText(tr('选择一个数据流'))
        self.title.setToolTip("")
        self.title.hide()
        self.hero_kicker.setText(tr('多模态观测'))
        self.hero_title.setText(tr('感知，\n在此汇聚。'))
        self.hero_lead.setText(tr('展开下方数据流，进入你的观测现场。'))
        self.hero.show()
        self.metric_panel.hide()
        self._position_overlays(force=True)
        self.status.clear()
        self.status.setToolTip("")
        self.stream_info.setText(tr('等待选择数据流'))
        self.hint.setText(tr('点击下方缩略图，在这里聚焦查看'))
        self.expand_button.setEnabled(False)
        self.glove_controls.hide()
        self.tactile_controls.hide()
        self._empty_ready = True

    def set_motion_enabled(self, enabled):
        """Reduce decorative motion only; incoming frames keep refreshing."""
        self._motion_enabled = bool(enabled)
        if not self._motion_enabled:
            self._hero_animation.stop()
            self._hero_opacity.setOpacity(1.)
        if self.preview is not None:
            self.preview.set_motion_enabled(enabled)

    def _reveal_hero(self):
        self._hero_animation.stop()
        self._hero_opacity.setOpacity(1.)
        if self._motion_enabled and self.hero.isVisible():
            self._hero_animation.setStartValue(.15)
            self._hero_animation.setEndValue(1.)
            self._hero_animation.start()

    def _position_overlays(self, *, force=False):
        if not hasattr(self, "hero"):
            return
        width, height = self.canvas.width(), self.canvas.height()
        signature = (width, height, *(item.text() for item in (self.hero_kicker, self.hero_title, self.hero_lead)),
                     *(item.font().key() for item in (self.hero_kicker, self.hero_title, self.hero_lead)))
        if not force and signature == self._overlay_signature:
            return
        self._overlay_signature = signature
        hero_width = max(145, int(width * .4) - 24)
        kicker_height = max(18, self.hero_kicker.fontMetrics().height())
        title_height = self.hero_title.fontMetrics().height() * (self.hero_title.text().count("\n") + 1) + 8
        lead_height = min(90, max(38, self.hero_lead.heightForWidth(hero_width)))
        self.hero_kicker.setFixedHeight(kicker_height)
        self.hero_title.setFixedHeight(title_height)
        self.hero_lead.setFixedHeight(lead_height)
        hero_height = min(height - 20, kicker_height + title_height + lead_height + 26)
        self.hero.setGeometry(28, max(10, (height - hero_height) // 2 - 16), hero_width, max(1, hero_height))
        self.metric_panel.setGeometry(max(0, width - 200), 14, 172, 85)
        self.hero.raise_()
        self.metric_panel.raise_()

    def eventFilter(self, watched, event):
        if watched is self.canvas and event.type() == QEvent.Type.Resize:
            self._position_overlays()
        return super().eventFilter(watched, event)

    def closeEvent(self, event):
        self._hero_animation.stop()
        self.clear_source()
        super().closeEvent(event)
