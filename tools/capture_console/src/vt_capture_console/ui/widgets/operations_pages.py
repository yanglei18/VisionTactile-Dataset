"""Task-focused page composition; commands remain owned by MainWindow."""
from __future__ import annotations

from ...i18n import tr

from PySide6.QtCore import QSize, Qt
from PySide6.QtWidgets import QSplitter
from ...i18n.widgets import (
    QAbstractItemView, QComboBox, QFrame, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QProgressBar, QPushButton, QScrollArea, QSizePolicy, QSpinBox,
    QTableWidget, QTabWidget, QVBoxLayout, QWidget,
)

from ..icons import icon
from .device_controls import DeviceControls


def _label(text, name="operationHint", *, wrap=False):
    widget = QLabel(text)
    widget.setObjectName(name)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setWordWrap(wrap)
    if wrap:
        widget.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    return widget


def _panel(eyebrow, title):
    panel = QFrame()
    panel.setObjectName("operationPanel")
    layout = QVBoxLayout(panel)
    layout.setContentsMargins(20, 18, 20, 18)
    layout.setSpacing(10)
    layout.addWidget(_label(eyebrow, "operationEyebrow"))
    layout.addWidget(_label(title, "operationTitle"))
    return panel, layout


def _field(layout, title, widget):
    field = QVBoxLayout()
    field.setContentsMargins(0, 0, 0, 0)
    field.setSpacing(4)
    field.addWidget(_label(title, "operationStep"))
    field.addWidget(widget)
    layout.addLayout(field)


def _table(headers):
    widget = QTableWidget(0, len(headers))
    widget.setHorizontalHeaderLabels(headers)
    widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    widget.verticalHeader().hide()
    widget.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
    widget.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
    widget.setAlternatingRowColors(True)
    widget.setShowGrid(False)
    return widget


def _line_edit(accessible_name, *, text="", placeholder=""):
    widget = QLineEdit(text)
    widget.setAccessibleName(accessible_name)
    widget.setMaxLength(120)
    widget.setPlaceholderText(placeholder)
    return widget


def build_recording_page(window):
    """Keep task parameters and recording actions in one dedicated workspace."""
    page = QWidget()
    page.setObjectName("recordingPage")
    layout = QHBoxLayout(page)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(16)

    setup, setup_layout = _panel(tr('01 / 任务设置'), tr('定义采集任务'))
    setup.setObjectName("recordingSetup")
    window.recording_setup = setup
    setup_layout.setContentsMargins(18, 16, 18, 16)
    setup_layout.setSpacing(8)
    window.batch_selector = QComboBox()
    window.batch_selector.setMinimumWidth(160)
    window.batch_selector.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    window.batch_selector.setAccessibleName(tr('采集批次'))
    window.batch_selector.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
    window.batch_selector.setMinimumContentsLength(8)
    _field(setup_layout, tr('采集批次'), window.batch_selector)
    window.batch_name = _line_edit(tr('新批次名称'), placeholder=tr('新批次名称'))
    _field(setup_layout, tr('新建批次'), window.batch_name)
    window.create_batch_button = QPushButton(tr('创建批次'))
    window.create_batch_button.clicked.connect(window._create_batch)
    setup_layout.addWidget(window.create_batch_button)
    window.trial_name = _line_edit(tr('试次名称'), text=tr('采集试次'))
    _field(setup_layout, tr('试次名称'), window.trial_name)
    window.task_name = _line_edit(tr('任务名称'), placeholder=tr('可选，如抓取水杯'))
    _field(setup_layout, tr('任务名称'), window.task_name)
    window.operator_name = _line_edit(tr('操作员'), placeholder=tr('可选，工号或代号'))
    _field(setup_layout, tr('操作员'), window.operator_name)
    window.duration = QSpinBox()
    window.duration.setRange(1, 300)
    window.duration.setValue(30)
    window.duration.setSuffix(tr(' 秒'))
    window.duration.setAccessibleName(tr('录制时长'))
    window.duration.setEnabled(window.demo)
    _field(setup_layout, tr('录制时长'), window.duration)
    setup_layout.addStretch()
    setup_layout.addWidget(_label(
        tr('演示录制仅存于内存，不生成数据包。') if window.demo else
        tr('录制使用控制器已配置的数据主题；请先在设备页完成准备。'),
        wrap=True))
    setup_scroll = QScrollArea()
    setup_scroll.setObjectName("recordingSetupScroll")
    setup_scroll.setWidgetResizable(True)
    setup_scroll.setFrameShape(QFrame.Shape.NoFrame)
    setup_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    setup_scroll.setFixedWidth(304)
    setup_scroll.setWidget(setup)
    layout.addWidget(setup_scroll)

    console = QFrame()
    console.setObjectName("recordingConsole")
    console.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
    window.recording_console = console
    console_layout = QVBoxLayout(console)
    console_layout.setContentsMargins(28, 24, 28, 24)
    console_layout.setSpacing(16)
    heading = QHBoxLayout()
    heading.addWidget(_label(tr('02 / 录制控制'), "operationEyebrow"), 1)
    window.recording_archive_button = QPushButton(tr('查看档案 →'))
    window.recording_archive_button.setAccessibleName(tr('查看录制档案'))
    window.recording_archive_button.clicked.connect(lambda: window._navigate(1))
    heading.addWidget(window.recording_archive_button)
    console_layout.addLayout(heading)

    window.record_state = _label(tr('模拟 · 待命') if window.demo else tr('录制 · 待命'), "recordingStatus")
    window.record_state.setAccessibleName(tr('录制状态'))
    console_layout.addWidget(window.record_state)
    window.record_detail = _label("", "recordDetail", wrap=True)
    window.record_detail.setAccessibleName(tr('录制状态说明'))
    window.record_detail.setMaximumHeight(64)
    console_layout.addWidget(window.record_detail)
    plan = QFrame()
    plan.setObjectName("capturePlan")
    plan_layout = QVBoxLayout(plan)
    plan_layout.setContentsMargins(0, 0, 0, 0)
    plan_layout.setSpacing(4)
    plan_heading = QHBoxLayout()
    plan_heading.addWidget(_label(tr('配置范围'), "operationStep"), 1)
    window.capture_plan_topics = _label(tr('数据主题清单 · 悬停查看'), "operationHint")
    window.capture_plan_topics.setAccessibleName(tr('控制器录制数据主题清单'))
    plan_heading.addWidget(window.capture_plan_topics)
    plan_layout.addLayout(plan_heading)
    window.capture_plan_summary = _label("", "recordingContext", wrap=True)
    window.capture_plan_summary.setAccessibleName(tr('配置的数据类别'))
    plan_layout.addWidget(window.capture_plan_summary)
    window.capture_plan_status = _label("", wrap=True)
    window.capture_plan_status.setAccessibleName(tr('采集计划校验状态'))
    window.capture_plan_status.setMaximumHeight(56)
    plan_layout.addWidget(window.capture_plan_status)
    console_layout.addWidget(plan)
    console_layout.addStretch(2)

    timing = QVBoxLayout()
    timing.setSpacing(12)
    timing.addWidget(_label(tr('已用时长 / 计划时长'), "operationEyebrow"))
    window.elapsed = _label("00:00 / 00:30", "recordingClock")
    window.elapsed.setAccessibleName(tr('已录制时长与计划时长'))
    window.elapsed.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    timing.addWidget(window.elapsed)
    window.progress = QProgressBar()
    window.progress.setObjectName("recordingProgress")
    window.progress.setAccessibleName(tr('录制进度'))
    window.progress.setTextVisible(False)
    window.progress.setRange(0, 1000)
    window.progress.setValue(0)
    timing.addWidget(window.progress)
    window.recording_context = _label(
        tr('选择采集批次并填写试次名称'), "recordingContext", wrap=True)
    window.recording_context.setAccessibleName(tr('当前录制任务'))
    window.recording_context.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    window.recording_context.setMaximumHeight(72)
    timing.addWidget(window.recording_context)
    console_layout.addLayout(timing)
    console_layout.addStretch(3)

    actions = QWidget()
    actions.setObjectName("recordingActions")
    actions_layout = QHBoxLayout(actions)
    actions_layout.setContentsMargins(0, 0, 0, 0)
    actions_layout.setSpacing(12)
    window.start_button = QPushButton(tr('模拟开始') if window.demo else tr('开始录制'))
    window.start_button.setObjectName("primary")
    window.start_button.setIcon(icon("play", "#080808", 15))
    window.start_button.setIconSize(QSize(15, 15))
    window.start_button.setMinimumHeight(48)
    window.start_button.setEnabled(window.demo)
    window.start_button.clicked.connect(window._start_recording)
    window.stop_button = QPushButton(tr('模拟停止') if window.demo else tr('停止录制'))
    window.stop_button.setIcon(icon("stop", "#bcbcbc", 13))
    window.stop_button.setIconSize(QSize(13, 13))
    window.stop_button.setMinimumHeight(48)
    window.stop_button.setEnabled(False)
    window.stop_button.clicked.connect(window._stop_recording)
    actions_layout.addWidget(window.start_button, 1)
    actions_layout.addWidget(window.stop_button, 1)
    console_layout.addWidget(actions)
    console_layout.addWidget(_label(
        tr('切换页面后，录制标签持续显示状态。录制结束后请到档案查看质量报告。'),
        wrap=True))
    layout.addWidget(console, 1)
    return page


def build_archive_page(window):
    """Browse completed and active trial records without duplicating controls."""
    page = QWidget()
    page.setObjectName("archivePage")
    layout = QVBoxLayout(page)
    layout.setContentsMargins(0, 0, 0, 0)

    history = QFrame()
    history.setObjectName("archivePanel")
    window.archive_history = history
    history_layout = QVBoxLayout(history)
    history_layout.setContentsMargins(16, 12, 16, 12)
    history_layout.setSpacing(8)
    toolbar = QHBoxLayout()
    toolbar.addWidget(_label(tr('试次记录'), "operationTitle"), 1)
    window.archive_open_bag_button = QPushButton(tr('打开本机录制包'))
    window.archive_open_bag_button.clicked.connect(window._choose_archive_bag)
    toolbar.addWidget(window.archive_open_bag_button)
    window.archive_record_button = QPushButton(tr('前往录制 →'))
    window.archive_record_button.clicked.connect(lambda: window._navigate(4))
    toolbar.addWidget(window.archive_record_button)
    history_layout.addLayout(toolbar)
    history_layout.addWidget(_label(
        tr('演示批次与试次仅存于内存，关闭后清空。') if window.demo else
        tr('显示本后台最近的批次与试次；录制结束不代表质量合格。未自动导入外部历史数据包。'),
        wrap=True))
    window.trial_table = _table([tr('试次'), tr('批次'), tr('录制状态'), tr('时长'), tr('质量检查'), tr('数据包路径')])
    window.trial_table.setMinimumHeight(86)
    window.trial_table.itemSelectionChanged.connect(window._show_trial_details)
    window.trial_table.cellDoubleClicked.connect(lambda *_: window._open_trial_inspector())
    history_layout.addWidget(window.trial_table, 1)
    history_layout.addWidget(_label(tr('所选试次'), "operationEyebrow"))
    window.trial_details = _label(tr('选择试次可查看会话编号、实际路径及最后状态。'), wrap=True)
    window.trial_details.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    window.trial_details.setMaximumHeight(100)
    detail_row = QHBoxLayout()
    detail_row.setSpacing(16)
    detail_row.addWidget(window.trial_details, 1)
    window.inspect_trial_button = QPushButton(tr('试次详情 / 质检'))
    window.inspect_trial_button.clicked.connect(window._open_trial_inspector)
    window.inspect_trial_button.setEnabled(False)
    detail_row.addWidget(window.inspect_trial_button)
    window.replay_trial_button = QPushButton(tr('回放所选试次'))
    window.replay_trial_button.clicked.connect(window._open_trial_replay)
    window.replay_trial_button.setEnabled(False)
    detail_row.addWidget(window.replay_trial_button)
    history_layout.addLayout(detail_row)
    from .topic_replay import TopicReplayPanel
    window.archive_replay = TopicReplayPanel(path_blocker=window._archive_path_active)
    window.archive_replay.browse_requested.connect(window._browse_archive)
    window.archive_replay.hide()
    splitter = QSplitter(Qt.Orientation.Vertical)
    splitter.setChildrenCollapsible(False)
    splitter.addWidget(history)
    splitter.addWidget(window.archive_replay)
    splitter.setStretchFactor(0, 1)
    splitter.setStretchFactor(1, 2)
    splitter.setSizes([250, 450])
    layout.addWidget(splitter, 1)
    return page


def build_devices_page(window):
    page = QWidget()
    layout = QVBoxLayout(page)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(16)

    setup = QFrame()
    setup.setObjectName("operationPanel")
    setup_layout = QVBoxLayout(setup)
    setup_layout.setContentsMargins(18, 14, 18, 14)
    setup_layout.setSpacing(10)
    heading = QHBoxLayout()
    heading.setSpacing(30)
    titles = QVBoxLayout()
    titles.setSpacing(5)
    titles.addWidget(_label(tr('设备 / 采集前检查'), "operationEyebrow"))
    titles.addWidget(_label(tr('设备准备'), "operationTitle"))
    heading.addLayout(titles)
    window.settings_actions = QHBoxLayout()
    window.settings_actions.setSpacing(10)
    window.configure_devices_button = QPushButton(tr('配置设备'))
    window.configure_devices_button.setObjectName("primary")
    window.configure_devices_button.clicked.connect(window._open_workstation_settings)
    window.settings_actions.addWidget(window.configure_devices_button)
    window.retry_backend_button = QPushButton(tr('重试连接'))
    window.retry_backend_button.clicked.connect(window._retry_desktop_backend)
    window.settings_actions.addWidget(window.retry_backend_button)
    window.backend_log_button = QPushButton(tr('后台日志'))
    window.backend_log_button.clicked.connect(window._open_backend_log)
    window.settings_actions.addWidget(window.backend_log_button)
    # DesktopTools inserts its calibration menu immediately before this stretch.
    window.settings_actions.addStretch()
    heading.addLayout(window.settings_actions, 1)
    setup_layout.addLayout(heading)
    window.setup_callout = _label("", wrap=True)
    window.setup_callout.setMaximumHeight(40)
    setup_layout.addWidget(window.setup_callout)
    layout.addWidget(setup)

    window.device_tabs = QTabWidget()
    window.device_tabs.setObjectName("operationTabs")
    window.device_tabs.setDocumentMode(True)
    controls_page = QWidget()
    controls_layout = QVBoxLayout(controls_page)
    controls_layout.setContentsMargins(16, 14, 16, 14)
    window.device_controls = DeviceControls(window.backend)
    controls_layout.addWidget(window.device_controls, 1)
    window.device_tabs.addTab(controls_page, tr('01 / 进程控制'))
    streams_page = QWidget()
    streams_layout = QVBoxLayout(streams_page)
    streams_layout.setContentsMargins(16, 14, 16, 14)
    streams_layout.setSpacing(12)
    streams_layout.addWidget(_label(tr('数据流 · 保留所有订阅流，与进程状态独立'), wrap=True))
    window.device_table = _table([tr('设备'), tr('类型'), tr('状态'), tr('接收 Hz'), tr('有效 Hz'), tr('数据主题')])
    window.device_table.setMinimumHeight(100)
    streams_layout.addWidget(window.device_table, 1)
    window.device_tabs.addTab(streams_page, tr('02 / 订阅数据流'))
    layout.addWidget(window.device_tabs, 1)
    return page
