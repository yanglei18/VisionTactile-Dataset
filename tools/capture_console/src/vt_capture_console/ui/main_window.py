"""Capture presentation and explicit IPC actions; no device or Recorder ownership."""

from __future__ import annotations

import time

from PySide6.QtCore import QEvent, QSize, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QButtonGroup, QComboBox, QFrame, QGridLayout,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QProgressBar, QPushButton,
    QScrollArea, QSizePolicy, QSpinBox, QStackedWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from .widgets import PreviewWidget
from .widgets.trial_details import TrialDetailsDialog, QUALITY_LABELS
from .widgets.device_controls import DeviceControls
from .widgets.tactile_detail import TactileDetailDialog, TactileViewControls
from .widgets.glove_detail import GloveDetailDialog, GloveViewControls
from .widgets.preview_detail import PreviewDetailDialog
from ..backend.glove_calibration import inspect_status as glove_status, focus_owned_calibration
from .widgets.workstation_settings import BackendLogDialog, WorkstationSettingsDialog, runtime_busy
from .icons import icon
from ..viewmodels.recording import TRIAL_LABELS, owned_active_trial, recording_access


STATUS = {"online": ("在线", "#248563"), "stale": ("数据陈旧", "#9a6b18"),
          "offline": ("离线", "#9b5b63"), "unknown": ("等待数据", "#717780")}
KIND = {"camera": "相机", "tracker": "Tracker", "glove": "手势", "tactile": "触觉"}
RECORDING = {"UNAVAILABLE": "状态未知", "IDLE": "待命", "RECORDING": "录制中", "COMPLETE": "已结束"}


def label(text, name="", wrap=False):
    result = QLabel(text)
    result.setTextFormat(Qt.TextFormat.PlainText)
    result.setObjectName(name)
    result.setWordWrap(wrap)
    return result


def table(headers):
    result = QTableWidget(0, len(headers))
    result.setHorizontalHeaderLabels(headers)
    result.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    result.verticalHeader().hide()
    result.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
    result.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
    result.setAlternatingRowColors(True)
    result.setShowGrid(False)
    return result


def set_rows(widget, rows):
    widget.setRowCount(len(rows))
    for row_index, row in enumerate(rows):
        for column, value in enumerate(row):
            old = widget.item(row_index, column)
            text = str(value)
            if old is None:
                widget.setItem(row_index, column, QTableWidgetItem(text))
            elif old.text() != text:
                old.setText(text)


class DeviceCard(QFrame):
    selected = Signal(str)
    calibration_requested = Signal(str)

    def __init__(self, device, parent=None):
        super().__init__(parent)
        self.identifier = device["id"]
        self._device = dict(device)
        self.tactile_dialog = None
        self.tactile_controls = None
        self.glove_dialog = None
        self.glove_controls = None
        self.preview_dialog = None
        self.setObjectName("card")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumWidth(190)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(10)
        header = QHBoxLayout()
        header.setSpacing(8)
        kind_icon = label("")
        kind_icon.setPixmap(icon(device["kind"], "#68717c", 18).pixmap(18, 18))
        kind_icon.setFixedSize(20, 20)
        self.title = label(device["label"], "cardTitle")
        self.title.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.title.setToolTip(device["label"])
        self.state = label("")
        header.addWidget(kind_icon)
        header.addWidget(self.title, 1)
        header.addWidget(self.state)
        layout.addLayout(header)
        self.preview = PreviewWidget(device["kind"])
        self.preview.setFixedHeight(256 if device["kind"] in ("tactile", "glove") else 166)
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
            self.expand_button = QPushButton("放大")
            self.expand_button.clicked.connect(self.open_detail)
        self.expand_button.setObjectName("previewExpand")
        self.expand_button.setAutoDefault(False)
        self.expand_button.setFixedHeight(28)
        self.expand_button.setIcon(icon("expand", "#0877db", 14))
        self.expand_button.setIconSize(QSize(14, 14))
        self.expand_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.expand_button.setAccessibleName("放大 " + device["label"])
        self.expand_button.setToolTip("在独立窗口放大查看；不启停设备、不影响录制")
        self.expand_button.setStyleSheet("""
            QPushButton {color: #0877db; background: #edf5ff; border: 1px solid transparent;
                         border-radius: 7px; padding: 3px 6px; font-size: 11px;}
            QPushButton:hover {background: #e0edff; border-color: #b7d6ff;}
            QPushButton:focus {border-color: #88bbff;}
        """)
        header.addWidget(self.expand_button)
        if device["kind"] == "camera":
            self.preview.setToolTip("双击画面或点击右上角“放大”，打开实时大图")
        layout.addWidget(self.preview)
        if self.tactile_controls is not None:
            self.tactile_note = label("", "muted")
            self.tactile_note.setStyleSheet("color: #7d8796; font-size: 10px;")
            self.tactile_note.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            layout.addWidget(self.tactile_note)
        if self.glove_controls is not None:
            self.glove_note = label("", "muted", wrap=True)
            self.glove_note.setMaximumHeight(36)
            layout.addWidget(self.glove_note)
        self.metrics = label("", "muted")
        layout.addWidget(self.metrics)
        self.update_device(device)
        if self.glove_controls is not None and not device.get("preview", {}).get("demo"):
            self.preview.set_glove_view("3d")

    def update_device(self, device):
        if ((self._device.get("id"), self._device.get("side"))
                != (device.get("id"), device.get("side"))):
            self.close_details()
        self._device = dict(device)
        state, color = STATUS.get(device["status"], STATUS["unknown"])
        self.state.setText("● " + state)
        self.state.setStyleSheet(f"color: {color}; font-size: 11px; background: transparent;")
        glove_label = {"STARTABLE": "未启动", "STOPPED": "未启动", "DISABLED": "未启用",
                       "RUNNING_NO_DATA": "已启动 · 待结果", "INVALID_DATA": "姿态待就绪",
                       "STALE_DATA": "姿态断流"}.get(device.get("glove_state"))
        if glove_label:
            self.state.setText("● " + glove_label)
        rate = device["receive_hz"]
        valid = device.get("valid_hz")
        age = device.get("age_s")
        tail = f"有效 {valid:.1f} Hz" if valid is not None else (f"接收龄期 {age * 1000:.0f} ms" if age is not None else "尚无样本")
        self.metrics.setText(f"接收 {rate:.1f} Hz    ·    {tail}")
        self.preview.set_device(device)
        if self.preview_dialog is not None:
            self.preview_dialog.update_preview(self.preview, device)
        if self.tactile_controls is not None:
            self._sync_tactile_controls()
            if self.tactile_dialog is not None:
                self.tactile_dialog.update_preview(self.preview, device)
        if self.glove_controls is not None:
            self.glove_note.setText(device.get("glove_hint", "MANO 解算结果 · 腕部相对坐标"))
            self.glove_note.setToolTip(device.get("glove_hint", "") + "\n" + device.get("detail", ""))
            self.glove_controls.calibration_button.setEnabled(device.get("glove_action_enabled", True))
            self._sync_glove_controls()
            if self.glove_dialog is not None:
                self.glove_dialog.update_preview(self.preview, device)

    def open_detail(self):
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
        self.calibration_requested.emit(self._device.get("side", ""))

    def _sync_glove_controls(self, *_args):
        self.glove_controls.sync(self.preview)
        if self.glove_dialog is not None:
            self.glove_dialog.set_glove_view(self.preview.glove_view)

    def open_glove_detail(self):
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
        note = "候选映射" if self.preview.tactile_hand_available else "映射未确认"
        if self.preview.tactile_view == "3d":
            note = "静态参考手 · 候选映射"
        self.tactile_note.setText(f"{note} · ADC（非压力）")
        self.tactile_note.setToolTip(self.preview.tactile_layout_note + "\n颜色表示原始 ADC 读数，不是标定压力。")
        if self.tactile_dialog is not None:
            self.tactile_dialog.set_tactile_view(self.preview.tactile_view)

    def open_tactile_detail(self):
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


class MainWindow(QMainWindow):
    def __init__(self, backend, refresh_ms=100):
        super().__init__()
        self.backend = backend
        self.demo = backend.mode == "demo"
        self.devices = {}
        self.cards = {}
        self.selected_id = None
        self.grid_signature = None
        self.last_events = None
        self.last_snapshot = None
        self.management = {}
        self.pending_request = None
        self.pending_action = None
        self.pending_instance_id = None
        self.pending_controller_id = None
        self.pending_batch_ids = set()
        self._batches_signature = None
        self._trial_rows = []
        self.details_dialog = None
        self.settings_dialog = None
        self.backend_log_dialog = None
        self._details_polled_at = 0.0
        self.setWindowTitle("Vision Tactile · 集成采集工作台" + (" [DEMO]" if self.demo else " [LIVE]"))
        self.setWindowIcon(icon("orbit", "#007aff", 32))
        self.resize(1440, 900)
        self.setMinimumSize(1080, 720)
        shell = QWidget()
        shell.setObjectName("shell")
        self.setCentralWidget(shell)
        root = QHBoxLayout(shell)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._sidebar())
        body = QWidget()
        content = QVBoxLayout(body)
        content.setContentsMargins(24, 22, 24, 12)
        content.setSpacing(14)
        title_row = QHBoxLayout()
        headings = QVBoxLayout()
        headings.setSpacing(3)
        headings.addWidget(label("WORKSPACE  /  多模态采集", "eyebrow"))
        self.page_title = label("采集工作台", "title")
        headings.addWidget(self.page_title)
        title_row.addLayout(headings)
        title_row.addStretch()
        self.mode_badge = label("◦  DEMO  ·  演示空间" if self.demo else "◦  LIVE  ·  只读空间", "topBadge")
        self.mode_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mode_badge.setFixedHeight(32)
        title_row.addWidget(self.mode_badge)
        content.addLayout(title_row)
        banner = "演示模式：画面与频率均为合成数据，模拟录制不会生成 bag。" if self.demo else "只读模式：预览现有数据流，不启动设备、不修改标定、不发送录制命令。"
        self.banner = label(banner, "demoBanner" if self.demo else "liveBanner", True)
        content.addWidget(self.banner)
        content.addWidget(self._recording_bar())
        self.operation_notice = label("", "muted", True)
        self.operation_notice.setMaximumHeight(42)
        self.operation_notice.hide()
        content.addWidget(self.operation_notice)
        self.pages = QStackedWidget()
        self.pages.addWidget(self._dashboard())
        self.pages.addWidget(self._trials_page())
        self.pages.addWidget(self._devices_page())
        self.pages.addWidget(self._maintenance_page())
        content.addWidget(self.pages, 1)
        self.footer = label("", "footer")
        content.addWidget(self.footer)
        root.addWidget(body, 1)
        self.timer = QTimer(self)
        self.timer.setInterval(refresh_ms)
        self.timer.timeout.connect(self.refresh)
        self.refresh()
        self.timer.start()

    def _sidebar(self):
        frame = QFrame()
        frame.setObjectName("sidebar")
        frame.setFixedWidth(176)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(14, 24, 14, 20)
        layout.setSpacing(6)
        mark = label("", "sidebarMark")
        mark.setPixmap(icon("orbit", "#007aff", 38).pixmap(38, 38))
        mark.setFixedSize(48, 48)
        mark.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(mark)
        layout.addSpacing(6)
        layout.addWidget(label("VisionTactile", "brand"))
        layout.addWidget(label("Capture Studio", "muted"))
        layout.addSpacing(26)
        layout.addWidget(label("工作空间", "navCaption"))
        self.navigation = QButtonGroup(self)
        for index, title in enumerate(("采集工作台", "任务与试次", "设备与标定", "系统维护")):
            button = QPushButton(title)
            button.setObjectName("nav")
            button.setCheckable(True)
            button.setChecked(index == 0)
            button.setIcon(icon(("dashboard", "trials", "devices", "settings")[index], "#007aff" if index == 0 else "#6b7079"))
            button.setIconSize(QSize(18, 18))
            self.navigation.addButton(button, index)
            layout.addWidget(button)
        self.navigation.idClicked.connect(self._navigate)
        layout.addStretch()
        line = QFrame()
        line.setObjectName("divider")
        line.setFixedHeight(1)
        layout.addWidget(line)
        layout.addSpacing(10)
        layout.addWidget(label("本地工作站", "navCaption"))
        layout.addWidget(label("v0.5  ·  采集工作台", "muted"))
        return frame

    def _recording_bar(self):
        frame = QFrame()
        frame.setObjectName("recordingBar")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(12)
        record_icon = label("", "recordIcon")
        record_icon.setPixmap(icon("orbit", "#007aff", 24).pixmap(24, 24))
        record_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        record_icon.setFixedSize(44, 44)
        layout.addWidget(record_icon)
        information = QVBoxLayout()
        information.setSpacing(3)
        self.record_state = label("待命", "sectionTitle")
        self.record_detail = label("", "muted", True)
        information.addWidget(self.record_state)
        information.addWidget(self.record_detail)
        layout.addLayout(information, 1)
        timing = QVBoxLayout()
        self.elapsed = label("00:00 / 00:30", "recordTime")
        self.progress = QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setRange(0, 1000)
        timing.addWidget(self.elapsed)
        timing.addWidget(self.progress)
        layout.addLayout(timing)
        layout.addSpacing(4)
        self.duration = QSpinBox()
        self.duration.setRange(1, 300)
        self.duration.setValue(30)
        self.duration.setSuffix(" 秒")
        self.duration.setFixedWidth(82)
        self.duration.setAccessibleName("录制时长")
        self.duration.setEnabled(self.demo)
        layout.addWidget(self.duration)
        self.start_button = QPushButton("模拟开始" if self.demo else "开始录制")
        self.start_button.setObjectName("primary")
        self.start_button.setIcon(icon("play", "#ffffff", 15))
        self.start_button.setIconSize(QSize(15, 15))
        self.start_button.setEnabled(self.demo)
        self.start_button.clicked.connect(self._start_recording)
        self.stop_button = QPushButton("模拟停止" if self.demo else "停止")
        self.stop_button.setIcon(icon("stop", "#737984", 13))
        self.stop_button.setIconSize(QSize(13, 13))
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self._stop_recording)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        return frame

    def _dashboard(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(18)
        middle = QVBoxLayout()
        tools = QHBoxLayout()
        self.overview = label("实时预览", "sectionTitle")
        tools.addWidget(self.overview)
        tools.addStretch()
        self.stream_count = label("", "muted")
        tools.addWidget(self.stream_count)
        middle.addLayout(tools)
        # The combo remains the single filter model; native segmented buttons
        # are its visible presentation and use the same selection path.
        self.filter = QComboBox(page)
        self.filter.hide()
        self.segments = QButtonGroup(self)
        segment_bar = QFrame()
        segment_bar.setObjectName("segmentBar")
        segment_layout = QHBoxLayout(segment_bar)
        segment_layout.setContentsMargins(4, 4, 4, 4)
        segment_layout.setSpacing(3)
        self.segment_titles = ("全部", "相机", "Tracker", "手势", "触觉")
        for index, (title, value) in enumerate(zip(self.segment_titles, ("all", "camera", "tracker", "glove", "tactile"))):
            self.filter.addItem(title, value)
            button = QPushButton(title)
            button.setObjectName("segment")
            button.setCheckable(True)
            button.setChecked(index == 0)
            button.setAccessibleName("筛选" + title)
            self.segments.addButton(button, index)
            segment_layout.addWidget(button, 1)
        self.segments.idClicked.connect(self.filter.setCurrentIndex)
        self.filter.currentIndexChanged.connect(self._invalidate_grid)
        self.filter.currentIndexChanged.connect(lambda index: self.segments.button(index).setChecked(True))
        middle.addWidget(segment_bar)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.grid_host = QWidget()
        self.grid_host.setObjectName("previewGrid")
        self.grid = QGridLayout(self.grid_host)
        self.grid.setContentsMargins(0, 0, 8, 0)
        self.grid.setSpacing(14)
        self.grid.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.scroll.setWidget(self.grid_host)
        middle.addWidget(self.scroll, 1)
        layout.addLayout(middle, 1)
        details = QFrame()
        details.setObjectName("details")
        details.setFixedWidth(232)
        panel = QVBoxLayout(details)
        panel.setContentsMargins(18, 20, 18, 18)
        panel.setSpacing(12)
        panel.addWidget(label("检查器", "inspectorHeading"))
        panel.addWidget(label("所选数据流", "muted"))
        self.detail_name = label("选择一张设备卡片", "sectionTitle", True)
        self.detail_topic = label("", "topicBox", True)
        self.detail_topic.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.detail_state = label("", wrap=True)
        self.detail_text = label("", "muted", True)
        for item in (self.detail_name, self.detail_state, self.detail_topic, self.detail_text):
            panel.addWidget(item)
        line = QFrame()
        line.setObjectName("divider")
        line.setFixedHeight(1)
        panel.addWidget(line)
        panel.addSpacing(4)
        panel.addWidget(label("最近活动", "inspectorHeading"))
        self.events_label = label("", "muted", True)
        panel.addWidget(self.events_label)
        panel.addStretch()
        panel.addWidget(label("频率为 Topic 接收速率；\n预览刷新率不代表采集帧率。", "muted", True))
        layout.addWidget(details)
        return page

    def _trials_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(label("演示批次与试次仅存于内存，关闭后清空。" if self.demo else "显示本后台最近的批次与试次；录制结束不代表质量合格。未自动导入外部历史 bag。", "muted", True))
        batch_row = QHBoxLayout()
        batch_row.addWidget(label("采集批次"))
        self.batch_selector = QComboBox()
        self.batch_selector.setMinimumWidth(160)
        self.batch_selector.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.batch_selector.setAccessibleName("采集批次")
        self.batch_selector.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.batch_selector.setMinimumContentsLength(8)
        batch_row.addWidget(self.batch_selector, 1)
        self.batch_name = QLineEdit()
        self.batch_name.setPlaceholderText("新批次名称")
        self.batch_name.setMaxLength(120)
        self.batch_name.setAccessibleName("新批次名称")
        batch_row.addWidget(self.batch_name, 1)
        self.create_batch_button = QPushButton("创建批次")
        self.create_batch_button.clicked.connect(self._create_batch)
        batch_row.addWidget(self.create_batch_button)
        layout.addLayout(batch_row)
        name_row = QHBoxLayout()
        name_row.addWidget(label("试次名称"))
        self.trial_name = QLineEdit("采集试次")
        self.trial_name.setMaxLength(120)
        self.trial_name.setAccessibleName("试次名称")
        name_row.addWidget(self.trial_name, 1)
        name_row.addWidget(label("使用顶部开始 / 停止按钮", "muted"))
        layout.addLayout(name_row)
        metadata_row = QHBoxLayout()
        metadata_row.addWidget(label("任务名称"))
        self.task_name = QLineEdit()
        self.task_name.setPlaceholderText("可选，如抓取水杯")
        self.task_name.setMaxLength(120)
        self.task_name.setAccessibleName("任务名称")
        metadata_row.addWidget(self.task_name, 1)
        metadata_row.addWidget(label("操作员"))
        self.operator_name = QLineEdit()
        self.operator_name.setPlaceholderText("可选，建议使用工号或代号")
        self.operator_name.setMaxLength(120)
        self.operator_name.setAccessibleName("操作员")
        metadata_row.addWidget(self.operator_name, 1)
        layout.addLayout(metadata_row)
        self.trial_table = table(["试次", "批次", "录制状态", "时长", "质量检查", "bag 路径"])
        self.trial_table.itemSelectionChanged.connect(self._show_trial_details)
        self.trial_table.cellDoubleClicked.connect(lambda *_: self._open_trial_inspector())
        layout.addWidget(self.trial_table)
        self.trial_details = label("选择试次可查看会话 ID、实际路径及最后状态。", "muted", True)
        self.trial_details.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.trial_details.setMaximumHeight(100)
        detail_row = QHBoxLayout()
        detail_row.addWidget(self.trial_details, 1)
        self.inspect_trial_button = QPushButton("试次详情 / 质检")
        self.inspect_trial_button.clicked.connect(self._open_trial_inspector)
        self.inspect_trial_button.setEnabled(False)
        detail_row.addWidget(self.inspect_trial_button)
        layout.addLayout(detail_row)
        return page

    def _devices_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self.settings_actions = QHBoxLayout()
        self.configure_devices_button = QPushButton("配置设备")
        self.configure_devices_button.setObjectName("primary")
        self.configure_devices_button.clicked.connect(self._open_workstation_settings)
        self.settings_actions.addWidget(self.configure_devices_button)
        self.retry_backend_button = QPushButton("重试连接")
        self.retry_backend_button.clicked.connect(self._retry_desktop_backend)
        self.settings_actions.addWidget(self.retry_backend_button)
        self.backend_log_button = QPushButton("后台日志")
        self.backend_log_button.clicked.connect(self._open_backend_log)
        self.settings_actions.addWidget(self.backend_log_button)
        self.settings_actions.addStretch()
        layout.addLayout(self.settings_actions)
        self.setup_callout = label("", "muted", True)
        self.setup_callout.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.setup_callout.setMaximumHeight(34)
        layout.addWidget(self.setup_callout)
        self.device_controls = DeviceControls(self.backend)
        layout.addWidget(self.device_controls, 3)
        layout.addWidget(label("数据流 · 保留所有订阅流，与进程状态独立", "muted"))
        self.device_table = table(["设备", "类型", "状态", "接收 Hz", "有效 Hz", "Topic"])
        # With the real theme, the table header consumes ~49 px. Retain a
        # scrollable data row area even on the 1080 x 720 workstation layout.
        self.device_table.setMinimumHeight(100)
        layout.addWidget(self.device_table, 2)
        return page

    def _open_workstation_settings(self):
        runtime = getattr(self.backend, "runtime", None)
        if runtime is None:
            self._notice("演示模式只模拟流程，不保存设备配置；请从桌面工作台入口打开配置。" if self.demo else
                         "请从桌面工作台入口打开配置；此连接不拥有本地后台，不能修改其设置。")
            return
        if self.settings_dialog is not None and self.settings_dialog.isVisible():
            self.settings_dialog.raise_()
            self.settings_dialog.activateWindow()
            return
        if self.settings_dialog is not None:
            self.settings_dialog.deleteLater()
        try:
            self.settings_dialog = WorkstationSettingsDialog(runtime, lambda: self.last_snapshot, self)
            self.settings_dialog.show()
        except (ValueError, RuntimeError, OSError) as exc:
            self._notice(f"配置暂不可打开：{exc}")

    def _retry_desktop_backend(self):
        runtime = getattr(self.backend, "runtime", None)
        if runtime is None or runtime_busy(runtime):
            return
        try:
            runtime.start()
        except (ValueError, RuntimeError, OSError) as exc:
            self._notice(f"后台暂不可启动：{exc}")
        self._refresh_desktop_runtime()

    def _open_backend_log(self):
        runtime = getattr(self.backend, "runtime", None)
        if runtime is None:
            return
        if self.backend_log_dialog is not None:
            self.backend_log_dialog.close()
            self.backend_log_dialog.deleteLater()
        self.backend_log_dialog = BackendLogDialog(runtime, self)
        self.backend_log_dialog.show()

    def _refresh_desktop_runtime(self):
        runtime = getattr(self.backend, "runtime", None)
        self.retry_backend_button.setVisible(runtime is not None)
        self.backend_log_button.setVisible(runtime is not None)
        if runtime is None:
            text = ("演示设备控制仅在内存中运行，不保存设置。真实配置请从桌面工作台入口打开。" if self.demo else
                    "从桌面工作台入口打开配置。当前连接仅展示已有后台；进程启停与数据有效性分开判断。")
        else:
            status = runtime.status or {}
            text = status.get("detail") or "先配置设备，再按需启动进程组；保存配置不会自动连接设备。"
            if status.get("state") == "ready" and not any(
                    group.get("enabled") for group in runtime.configuration.get("process_groups", ())):
                text = "后台已就绪。先点击“配置设备”启用本次需要的模块，再选择模块启动。"
            self.retry_backend_button.setEnabled(not runtime_busy(runtime))
        self.setup_callout.setText(text)
        self.setup_callout.setToolTip(text + "\n标定仍独立；关闭旧手套标定窗口可能停止其解算。")

    def _maintenance_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(label("工作站运行模式", "sectionTitle"))
        layout.addWidget(label("独立 Qt 界面通过本地协议连接后台。关闭本工作台不会终止已有 ROS 节点。", "muted", True))
        if self.demo:
            layout.addSpacing(12)
            row = QHBoxLayout()
            row.addWidget(label("演示场景"))
            self.scenario = QComboBox()
            for title, value in (("全部正常", "normal"), ("手部数据陈旧", "stale"), ("头部相机离线", "offline")):
                self.scenario.addItem(title, value)
            self.scenario.currentIndexChanged.connect(lambda: setattr(self.backend, "scenario", self.scenario.currentData()))
            row.addWidget(self.scenario)
            row.addWidget(label("手部模块"))
            self.hands = QComboBox()
            for title, value in (("双手", "both"), ("仅右手", "right"), ("仅左手", "left"), ("不展示手部", "none")):
                self.hands.addItem(title, value)
            self.hands.currentIndexChanged.connect(lambda: setattr(self.backend, "hands", self.hands.currentData()))
            row.addWidget(self.hands)
            row.addStretch()
            layout.addLayout(row)
        else:
            layout.addWidget(label(f"后台地址：{self.backend.path}", "muted", True))
        layout.addSpacing(20)
        layout.addWidget(label("当前交付边界", "sectionTitle"))
        layout.addWidget(label("已实现：集成预览、录制管理、试次元信息、录后质检与按组设备启停。\n桌面入口支持图形化配置、手动 Tracker 初始化、独立标定与离线工具；打开窗口不会自动启动硬件或录制。\n仅管理本后台启动的采集进程，不接管外部进程。关闭工作台不会停止传感器或 Recorder；活动初始化／离线任务先取消后退出。\n旧手套标定窗口尚未与解算进程拆分，关闭它仍可能停止手势采集。系统依赖安装、硬件操作及真机验收仍需完成。", "muted", True))
        layout.addStretch()
        return page

    def _navigate(self, index):
        self.pages.setCurrentIndex(index)
        self.page_title.setText(("采集工作台", "任务与试次", "设备与标定", "系统维护")[index])
        for item, name in enumerate(("dashboard", "trials", "devices", "settings")):
            self.navigation.button(item).setIcon(icon(name, "#007aff" if item == index else "#6b7079"))

    def _invalidate_grid(self):
        self.grid_signature = None

    def show_tracker_preview(self, message=""):
        """Navigate to existing Tracker subscriptions without starting devices."""
        self.navigation.button(0).setChecked(True)
        self._navigate(0)
        self.filter.setCurrentIndex(self.filter.findData("tracker"))
        self.refresh()
        if message:
            self.statusBar().showMessage(message, 15000)

    def _select(self, identifier):
        self.selected_id = identifier
        self._update_details()
        for key, card in self.cards.items():
            card.setProperty("selected", key == identifier)
            card.style().unpolish(card)
            card.style().polish(card)

    def _update_details(self):
        device = self.devices.get(self.selected_id)
        if not device:
            self.detail_name.setText("暂无设备数据")
            self.detail_state.setText("等待后台或 Topic 发布")
            self.detail_topic.clear()
            self.detail_text.clear()
            return
        self.detail_name.setText(device["label"])
        self.detail_topic.setText(device["topic"])
        age = device.get("age_s")
        freshness = "尚未收到样本" if age is None else f"接收龄期 {age * 1000:.0f} ms"
        self.detail_state.setText(STATUS.get(device["status"], STATUS["unknown"])[0] + "\n" + freshness)
        self.detail_text.setText(device.get("detail", ""))

    def _notice(self, message):
        self.operation_notice.setText(message[:180])
        self.operation_notice.setToolTip(message)
        self.operation_notice.show()

    def _submit(self, command, params):
        if self.pending_request:
            return
        try:
            self.pending_batch_ids = {b["id"] for b in self.management.get("batches", [])}
            self.pending_instance_id = (self.last_snapshot or {}).get("instance_id")
            self.pending_controller_id = (self.management.get("controller") or {}).get("controller_instance_id")
            self.pending_request = self.backend.submit(command, params)
            self.pending_action = command
            self._notice("命令正在处理，请等待后台确认；不会自动重复提交。")
        except (ValueError, RuntimeError, OSError) as exc:
            self._notice(f"未提交：{exc}")
        self.refresh()

    def _create_batch(self):
        if not (self.demo or self.management.get("enabled")):
            self._notice("后台未启用管理模式，不能创建正式批次。")
            return
        title = self.batch_name.text().strip()
        if not title:
            self._notice("请填写新批次名称。")
            self.batch_name.setFocus()
            return
        self._submit("batch.create", {"label": title})

    def _start_recording(self):
        if self.pending_request:
            return
        if not self.demo and not recording_access(self.last_snapshot or {}, self.management)[0]:
            self._notice("当前不能开始：请检查后台管理模式、控制器连接和未决试次。")
            return
        batch_id = self.batch_selector.currentData()
        title = self.trial_name.text().strip()
        if not batch_id or not title:
            self.navigation.button(1).click()
            self._notice("请先创建或选择采集批次，并填写试次名称。")
            return
        context = self.management.get("controller") or {}
        params = {"batch_id": batch_id, "label": title, "duration_s": self.duration.value(),
                  "expected_controller_instance_id": context.get("controller_instance_id", "demo"),
                  "expected_state_revision": context.get("state_revision", 0)}
        if getattr(self.backend, "supports_details", False):
            params.update(task_name=self.task_name.text().strip(), operator_name=self.operator_name.text().strip())
        elif self.task_name.text().strip() or self.operator_name.text().strip():
            self._notice("后台当前不支持元信息；请恢复 v3 连接，或清空任务 / 操作员字段后使用旧版录制。")
            return
        self._submit("capture.start", params)

    def _stop_recording(self):
        if self.pending_request:
            return
        if self.demo:
            trial = next((t for t in self.management.get("trials", []) if t["state"] == "RECORDING"), None)
        else:
            if not recording_access(self.last_snapshot or {}, self.management)[1]:
                self._notice("当前没有可由本工作台停止的已确认录制。")
                return
            trial = owned_active_trial(self.management)
        if not trial:
            return
        context = self.management.get("controller") or {}
        self._submit("capture.stop", {"trial_id": trial["id"],
                     "expected_controller_instance_id": context.get("controller_instance_id", "demo"),
                     "expected_session_id": trial["session_id"]})

    def _show_trial_details(self):
        row = self.trial_table.currentRow()
        self.inspect_trial_button.setEnabled(0 <= row < len(self._trial_rows))
        if 0 <= row < len(self._trial_rows):
            trial = self._trial_rows[row]
            self.trial_table.setProperty("selectedTrialId", trial["id"])
            text = f"试次 {trial['id']}  ·  会话 {trial.get('session_id') or '尚未确认'}\n"
            text += f"bag：{trial.get('output_path') or '暂无确认路径'}\n{trial.get('detail', '')}"
            self.trial_details.setText(text)
            self.trial_details.setToolTip(text)

    def _open_trial_inspector(self):
        row = self.trial_table.currentRow()
        if not 0 <= row < len(self._trial_rows):
            return
        if self.details_dialog is None:
            self.details_dialog = TrialDetailsDialog(self)
            self.details_dialog.refresh_requested.connect(self._request_trial_details)
            self.details_dialog.retry_requested.connect(self._retry_quality)
        self.details_dialog.bind(self._trial_rows[row])
        self.details_dialog.show()
        self.details_dialog.raise_()
        self._request_trial_details(self.details_dialog.trial_id)
        self._refresh_trial_inspector()

    def _request_trial_details(self, trial_id):
        self._details_polled_at = time.monotonic()
        if not trial_id or not getattr(self.backend, "supports_details", False):
            return
        try:
            self.backend.request_details(trial_id)
        except (ValueError, RuntimeError, OSError) as exc:
            if self.details_dialog:
                self.details_dialog.connection.setText(f"详情暂不可用：{exc}")

    def _retry_quality(self, trial_id):
        if self.pending_request or not getattr(self.backend, "supports_details", False):
            return
        details = getattr(self.backend, "latest_details", None)
        status = getattr(self.backend, "details_status", {})
        if (details and details.get("trial_id") == trial_id and status.get("state") == "ready"
                and details["quality"]["state"] in {"ERROR", "INTERRUPTED"}):
            self._submit("quality.retry", {"trial_id": trial_id})

    def _refresh_trial_inspector(self):
        dialog = self.details_dialog
        if dialog is None or not dialog.isVisible():
            return
        trial = next((t for t in self._trial_rows if t["id"] == dialog.trial_id), None)
        if trial is None:
            dialog.connection.setText("当前后台列表中没有此试次，保留上次详情；请重新选择。")
            dialog.retry_button.setEnabled(False)
            return
        status = getattr(self.backend, "details_status", {"state": "unsupported"})
        supported = bool(getattr(self.backend, "supports_details", False))
        # One selected trial, on demand. No full report in the 10 Hz snapshots.
        if supported and status.get("state") != "loading" and time.monotonic() - self._details_polled_at >= 2.0:
            self._request_trial_details(dialog.trial_id)
            status = getattr(self.backend, "details_status", status)
        dialog.display(trial, getattr(self.backend, "latest_details", None), status,
                       supported=supported, pending=bool(self.pending_request), demo=self.demo)

    def _refresh_management(self, snapshot):
        self.management = snapshot.get("management") or (getattr(self.backend, "management", {}) if self.demo else {})
        batches = self.management.get("batches", [])
        signature = tuple((batch["id"], batch["label"]) for batch in batches)
        if signature != self._batches_signature:
            selected = self.batch_selector.currentData()
            self.batch_selector.blockSignals(True)
            self.batch_selector.clear()
            if not batches:
                self.batch_selector.addItem("尚无批次", None)
            for batch in batches:
                self.batch_selector.addItem(batch["label"], batch["id"])
            index = self.batch_selector.findData(selected)
            if index >= 0:
                self.batch_selector.setCurrentIndex(index)
            self.batch_selector.blockSignals(False)
            self._batches_signature = signature
        operations = [getattr(self.backend, "latest_operation", None), self.management.get("operation")]
        operation = next((o for o in operations if o and o.get("request_id") == self.pending_request), None)
        context = self.management.get("controller") or {}
        if self.pending_request and not self.demo and self.management.get("enabled"):
            # A different backend/state directory may not contain the old
            # intent. End local waiting, but never turn uncertainty into an
            # automatic resend or claim that the previous capture stopped.
            changed_manager = (self.pending_instance_id and snapshot.get("instance_id")
                               and self.pending_instance_id != snapshot["instance_id"])
            changed_controller = (self.pending_controller_id and context.get("controller_instance_id")
                                  and self.pending_controller_id != context["controller_instance_id"])
            matching_trial = operation and any(t["id"] == operation.get("trial_id")
                                                for t in self.management.get("trials", []))
            if (changed_manager or changed_controller) and not matching_trial:
                self._notice("后台或控制器已更换；上次操作结果待核实，未自动重试。请确认原试次状态。")
                self.pending_request = self.pending_action = None
        if self.pending_request and operation:
            trial = next((t for t in self.management.get("trials", []) if t["id"] == operation.get("trial_id")), None)
            finished = operation.get("state") in {"rejected", "unknown"} and operation.get("code") != "QUEUED"
            new_batches = [b for b in batches if b["id"] not in self.pending_batch_ids]
            if self.pending_action == "batch.create" and operation.get("state") == "accepted":
                # v2 receipts intentionally do not carry a batch ID. Other
                # clients may create batches concurrently: only the in-memory
                # demo can safely infer ownership from a newly appearing row.
                self.batch_selector.setCurrentIndex(
                    self.batch_selector.findData(new_batches[0]["id"]) if self.demo and len(new_batches) == 1 else -1)
                self.batch_name.clear()
                finished = True
            if self.pending_action == "capture.start" and trial and trial["state"] in {"RECORDING", "COMPLETE", "FAILED", "UNKNOWN"}:
                finished = True
            if self.pending_action == "capture.stop" and trial and trial["state"] in {"COMPLETE", "FAILED", "UNKNOWN"}:
                finished = True
            if self.pending_action == "quality.retry" and operation.get("state") == "accepted":
                finished = True
                self._details_polled_at = 0.0
            if finished:
                if operation.get("state") in {"rejected", "unknown"}:
                    self._notice(f"{operation.get('code', '')}：{operation.get('detail', '')}")
                elif trial:
                    self._notice(f"{trial.get('label', trial['id'])}：{TRIAL_LABELS.get(trial['state'], trial['state'])}。{trial.get('detail', '')}")
                else:
                    self._notice("批次已创建。请选择目标批次，填写试次名称后开始录制。")
                self.pending_request = self.pending_action = None
        enabled = bool(self.demo or self.management.get("enabled"))
        self.create_batch_button.setEnabled(enabled and not self.pending_request)
        self.batch_name.setEnabled(enabled and not self.pending_request)
        self.batch_selector.setEnabled(enabled and not self.pending_request)
        self.trial_name.setEnabled(enabled and not self.pending_request)
        supports_metadata = enabled and bool(getattr(self.backend, "supports_details", False)) and not self.pending_request
        for editor in (self.task_name, self.operator_name):
            # After a v3 -> v2 downgrade, retain the operator's input, but let
            # them explicitly clear it to use the legacy recording workflow.
            editor.setEnabled(supports_metadata or (enabled and not self.pending_request and bool(editor.text())))
            editor.setToolTip("录制开始时冻结；可留空" if supports_metadata else "旧后台不支持保存此字段；可清空已有内容后使用旧版录制，不会静默丢弃元信息")
        batch_labels = {b["id"]: b["label"] for b in batches}
        self._trial_rows = self.management.get("trials", [])
        rows = [(t.get("label", t["id"]), batch_labels.get(t["batch_id"], t["batch_id"]),
                 TRIAL_LABELS.get(t["state"], t["state"]), f"{t['elapsed_s']:.1f} / {t['duration_s']:.0f} 秒",
                 QUALITY_LABELS.get(t.get("quality"), t.get("quality", "未执行")),
                 t.get("output_path") or ("仅演示，无 bag" if self.demo else "等待确认")) for t in self._trial_rows]
        selected_id = self.trial_table.property("selectedTrialId")
        self.trial_table.blockSignals(True)
        set_rows(self.trial_table, rows)
        if selected_id:
            index = next((i for i, t in enumerate(self._trial_rows) if t["id"] == selected_id), -1)
            if index >= 0:
                self.trial_table.selectRow(index)
            else:
                self.trial_table.clearSelection()
                self.trial_table.setCurrentCell(-1, -1)
        self.trial_table.blockSignals(False)
        self._show_trial_details()
        self._refresh_trial_inspector()

    def open_glove_calibration(self, hand):
        """Reuse the managed glove group, never launch a competing serial owner."""
        if hand not in ("right", "left") or self.demo:
            self.statusBar().showMessage("演示模式不启动真实标定工具。", 8000)
            return
        snapshot = self.backend.snapshot()
        status = glove_status(snapshot, hand)
        if status["can_focus"]:
            _, message = focus_owned_calibration(status["group"], hand)
            self.statusBar().showMessage(message, 10000)
            return
        if not status["can_start"]:
            self.statusBar().showMessage(status["message"], 10000)
            return
        runtime = getattr(self.backend, "runtime", None)
        if runtime is None:
            self.statusBar().showMessage("请从桌面工作台打开标定；只读连接不会自行启动手套窗口。", 10000)
            return
        group = status["group"]
        profile = next((g for g in runtime.configuration.get("process_groups", ()) if g["id"] == group["id"]), None)
        parameters = profile.get("parameters", {}) if profile else {}
        if parameters.get("hand") != hand or str(parameters.get("launch_gui", "true")).lower() != "true":
            self.statusBar().showMessage("请在配置设备中确认本手的 hand 与 launch_gui=true；不会另起串口程序。", 10000)
            return
        self.device_controls.refresh(snapshot)
        self.device_controls.selected_id = group["id"]
        pending_before = self.device_controls.pending_request
        self.device_controls.submit("start")
        accepted = self.device_controls.pending_request and self.device_controls.pending_request != pending_before
        message = ("已请求打开本手原标定窗口。请在窗口中连接 IMU 并标定；不会自动标定。" if accepted
                   else self.device_controls.operation_notice.text() or "当前无法提交启动请求，请核对进程状态。")
        self.statusBar().showMessage(message, 10000)

    def refresh(self):
        snapshot = self.backend.snapshot()
        self.last_snapshot = snapshot
        self._refresh_management(snapshot)
        self.device_controls.refresh(snapshot)
        self._refresh_desktop_runtime()
        self.devices = {device["id"]: device for device in snapshot["devices"]}
        for identifier, device in tuple(self.devices.items()):
            if device["kind"] == "glove" and device.get("side") in ("right", "left"):
                status = glove_status(snapshot, device["side"])
                self.devices[identifier] = {**device, "glove_state": status["state"],
                    "glove_hint": "演示手势 · 不连接硬件" if self.demo else status["message"],
                    "glove_action_enabled": bool(not self.demo and (status["can_start"] or status["can_focus"]))}
        online = sum(device["status"] == "online" for device in self.devices.values())
        self.stream_count.setText(f"{online} / {len(self.devices)} 路在线")
        for index, title in enumerate(self.segment_titles):
            kind = self.filter.itemData(index)
            count = len(self.devices) if kind == "all" else sum(d["kind"] == kind for d in self.devices.values())
            self.segments.button(index).setText(f"{title}  {count}")
        selected_kind = self.filter.currentData()
        shown = [d for d in self.devices.values() if selected_kind == "all" or d["kind"] == selected_kind]
        columns = 3 if self.scroll.viewport().width() >= 690 else 2
        signature = (tuple(d["id"] for d in shown), columns)
        for identifier in set(self.cards) - set(self.devices):
            card = self.cards.pop(identifier)
            card.close_details()
            card.deleteLater()
        if signature != self.grid_signature:
            for index in reversed(range(self.grid.count())):
                item = self.grid.takeAt(index)
                if item.widget():
                    item.widget().hide()
            for index, device in enumerate(shown):
                if device["id"] not in self.cards:
                    card = DeviceCard(device)
                    card.selected.connect(self._select)
                    card.calibration_requested.connect(self.open_glove_calibration)
                    self.cards[device["id"]] = card
                card = self.cards[device["id"]]
                self.grid.addWidget(card, index // columns, index % columns)
                card.show()
            self.grid_signature = signature
        for device in shown:
            self.cards[device["id"]].update_device(device)
        shown_ids = {device["id"] for device in shown}
        for identifier, card in self.cards.items():
            if identifier not in shown_ids and (card.tactile_dialog is not None or card.glove_dialog is not None
                                               or card.preview_dialog is not None):
                # A filter may hide the card, but its already-open detail view
                # keeps following this same snapshot. No extra poll or timer.
                card.update_device(self.devices[identifier])
        if self.selected_id not in self.devices and self.devices:
            self._select(next(iter(self.devices)))
        self._update_details()
        recording = snapshot["recording"]
        state = recording["state"]
        prefix = "模拟 · " if self.demo else "录制 · "
        self.record_state.setText(prefix + RECORDING.get(state, state))
        self.record_detail.setText(recording.get("detail", ""))
        if (not self.demo and state == "UNAVAILABLE" and self.management.get("enabled")
                and not self.management.get("controller") and not any(
                    trial.get("state") in {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"}
                    for trial in self.management.get("trials", ()))):
            self.record_state.setText("录制 · 等待控制器")
            self.record_detail.setText("尚无新鲜录制状态；请在“设备与标定”配置并启动相机与录制控制器组。")
        duration = recording["duration_s"] if not self.demo else (recording["duration_s"] or self.duration.value())
        elapsed = recording["elapsed_s"]
        if duration > 0:
            self.elapsed.setText(f"{int(elapsed) // 60:02d}:{int(elapsed) % 60:02d} / {int(duration) // 60:02d}:{int(duration) % 60:02d}")
            self.progress.setValue(int(1000 * min(1.0, elapsed / duration)))
        else:
            self.elapsed.setText("时长未知")
            self.progress.setValue(0)
        can_start, can_stop = recording_access(snapshot, self.management, bool(self.pending_request))
        if self.demo:
            can_start = state != "RECORDING" and not self.pending_request
            can_stop = state == "RECORDING" and not self.pending_request
        self.start_button.setEnabled(can_start)
        self.stop_button.setEnabled(can_stop)
        self.duration.setEnabled(can_start)
        self.start_button.setToolTip("创建批次后开始录制；只记录控制器已配置的 Topics" if can_start else "需要启用管理后台、连接控制器且无未决录制；不会自动重试")
        if not self.demo:
            managed = self.management.get("enabled", False)
            device_control = self.device_controls._available
            self.mode_badge.setText("◦  LIVE  ·  采集管理" if managed else
                                    "◦  LIVE  ·  设备控制" if device_control else "◦  LIVE  ·  只读空间")
            if device_control:
                self.banner.setText("设备控制已启用：在设备页显式启停本后台进程；不接管外部进程，不修改标定。")
            else:
                self.banner.setText("管理模式：可手动控制现有 Recorder；设备控制需后台单独启用。" if managed else "只读模式：预览现有数据流；管理功能需后台显式启用。")
        if snapshot["events"] != self.last_events:
            self.events_label.setText("\n\n".join(event["message"] for event in snapshot["events"][:5]) or "暂无事件")
            self.last_events = snapshot["events"]
        set_rows(self.device_table, [(d["label"], KIND.get(d["kind"], d["kind"]), STATUS.get(d["status"], STATUS["unknown"])[0],
                                      f"{d['receive_hz']:.1f}", "—" if d["valid_hz"] is None else f"{d['valid_hz']:.1f}", d["topic"])
                                     for d in self.devices.values()])
        free = snapshot["resources"].get("disk_free_bytes")
        disk = f"剩余空间 {free / 1024**3:.1f} GiB" if free is not None else "磁盘空间 未检测"
        mode_text = "DEMO · 合成数据" if self.demo else ("LIVE · 采集管理" if self.management.get("enabled") else
                    "LIVE · 设备控制" if self.device_controls._available else "LIVE · 只读")
        self.footer.setText(f"{mode_text}    |    {disk}    |    状态刷新上限 {1000 // self.timer.interval()} Hz    |    不等同于采集频率")

    def closeEvent(self, event):
        self.timer.stop()
        for card in self.cards.values():
            card.close_details()
        if self.details_dialog is not None:
            self.details_dialog.close()
        if self.settings_dialog is not None:
            self.settings_dialog.close()
        if self.backend_log_dialog is not None:
            self.backend_log_dialog.close()
        self.backend.close()
        event.accept()
