"""Capture presentation and explicit IPC actions; no device or Recorder ownership."""

from __future__ import annotations

from ..i18n import LocalizedText, initialize, language, set_language, tr
from ..i18n.messages import message as diagnostic
from ..i18n.labels import device_label

import time

from PySide6.QtCore import QEvent, QSettings, Qt, QTimer
from PySide6.QtGui import QActionGroup
from PySide6.QtWidgets import QApplication, QWidget as QtWidget
from ..i18n.widgets import (
    QAbstractItemView, QButtonGroup, QComboBox, QFrame, QGridLayout,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QMenu, QPushButton,
    QScrollArea, QSizePolicy, QTableWidget,
    QTableWidgetItem, QToolButton, QVBoxLayout, QWidget,
)

from .widgets.trial_details import TrialDetailsDialog, QUALITY_LABELS
from .widgets.device_card import DeviceCard, STATUS
from ..backend.glove_calibration import inspect_status as glove_status, focus_owned_calibration
from ..backend.operation_receipts import operation_for
from .widgets.workstation_settings import BackendLogDialog, WorkstationSettingsDialog, runtime_busy
from .icons import icon
from ..viewmodels.recording import TRIAL_LABELS, owned_active_trial, recording_access
from ..viewmodels.capture_plan import capture_plan_view
from ..viewmodels.operations import OperationCoordinator, OperationUpdate


KIND = {"camera": tr('相机'), "tracker": tr('追踪器'), "glove": tr('手势'), "tactile": tr('触觉')}
RECORDING = {"UNAVAILABLE": tr('状态未知'), "IDLE": tr('待命'), "RECORDING": tr('录制中'), "COMPLETE": tr('已结束')}


def _presentation_settings():
    # Explicit names also work when launched without the normal app entry point.
    return QSettings(QSettings.Format.IniFormat, QSettings.Scope.UserScope,
                     "visiontactile", "capture-console")


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
            text = value if isinstance(value, LocalizedText) else str(value)
            if old is None:
                widget.setItem(row_index, column, QTableWidgetItem(text))
            elif old.text() != text:
                old.setText(text)


class MainWindow(QMainWindow):
    def __init__(self, backend, refresh_ms=100, scene_backend=None, *, language_override=None):
        super().__init__()
        self._ui_settings = _presentation_settings()
        initialize(self._ui_settings, override=language_override)
        self._motion_enabled = self._ui_settings.value("motion/enabled", False, type=bool)
        self._presentation_active = False
        self._applied_motion = None
        self._closing = False
        self._refresh_ms = max(1, int(refresh_ms))
        self.backend = backend
        self.demo = backend.mode == "demo"
        if scene_backend is None:
            from ..backend.scene_demo import DemoSceneBackend
            from ..backend.scene_backend import SceneBackend
            scene_backend = DemoSceneBackend() if self.demo else SceneBackend(control_path=getattr(backend, "path", None))
        self.scene_backend = scene_backend
        self.spatial_settings_dialog = None
        self.devices = {}
        self.cards = {}
        self.selected_id = None
        self.grid_signature = None
        self.last_events = None
        self.last_snapshot = None
        self.management = {}
        self.operations = OperationCoordinator(demo=self.demo)
        self._batches_signature = None
        self._trial_rows = []
        self.details_dialog = None
        self.settings_dialog = None
        self._deployment_close_pending = False
        self.backend_log_dialog = None
        self.help_dialog = None
        self.demo_menu = None
        self.scenario_actions = {}
        self.hands_actions = {}
        self._details_polled_at = 0.0
        self.setWindowTitle(tr('VisionTactile · 采集工作台 [演示]') if self.demo
                            else tr('VisionTactile · 采集工作台 [实时]'))
        self.setWindowIcon(icon("orbit", "#ffffff", 32))
        self.resize(1440, 900)
        self.setMinimumSize(1080, 720)
        from .widgets.motion import AmbientBackdrop, TransitionStack
        shell = AmbientBackdrop()
        self.ambient = shell
        shell.setObjectName("shell")
        self.setCentralWidget(shell)
        root = QVBoxLayout(shell)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._workspace_header())
        body = QWidget()
        body.setObjectName("workspaceBody")
        content = QVBoxLayout(body)
        content.setContentsMargins(28, 4, 28, 8)
        content.setSpacing(8)
        self.page_heading = QWidget()
        self.page_heading.setObjectName("workspaceHeading")
        title_row = QHBoxLayout(self.page_heading)
        title_row.setContentsMargins(0, 0, 0, 0)
        title_row.setSpacing(16)
        self.page_eyebrow = label(tr('02 / 实时预览'), "eyebrow")
        self.page_title = label(tr('预览'), "title")
        title_row.addWidget(self.page_title)
        title_row.addWidget(self.page_eyebrow)
        title_row.addStretch()
        self.workspace_caption = label(tr('多模态采集'), "workspaceCaption")
        title_row.addWidget(self.workspace_caption)
        content.addWidget(self.page_heading)
        self.page_heading.hide()
        self._notice_page = None
        self.operation_notice = label("", "operationNotice", True)
        self.operation_notice.setMaximumHeight(38)
        self.operation_notice.hide()
        content.addWidget(self.operation_notice)
        self.pages = TransitionStack()
        self.pages.setObjectName("workspacePages")
        from .widgets.operations_pages import build_archive_page, build_recording_page
        self.preview_page = self._dashboard()
        self.archive_page = build_archive_page(self)
        self.device_page = self._devices_page()
        self.pages.addWidget(self.preview_page)
        self.pages.addWidget(self.archive_page)
        self.pages.addWidget(self.device_page)
        from .widgets.spatial_page import SpatialPage
        self.spatial_page = SpatialPage(self.scene_backend)
        self.spatial_page.configure_requested.connect(self._open_spatial_settings)
        self.spatial_page.archive_replay_requested.connect(self._choose_archive_bag)
        self.pages.addWidget(self.spatial_page)
        self.recording_page = build_recording_page(self)
        self.pages.addWidget(self.recording_page)
        content.addWidget(self.pages, 1)
        self.footer = label("", "footer")
        content.addWidget(self.footer)
        root.addWidget(body, 1)
        self._apply_motion()
        self.timer = QTimer(self)
        self.timer.setInterval(max(500, self._refresh_ms))
        self.timer.timeout.connect(self.refresh)
        self.refresh()
        self.timer.start()
        # A detail window can remain active while the main window is already
        # inactive. Observe application window events so that switching between
        # detail windows, other applications and minimized windows also updates
        # the shared presentation cadence. Coalesce activation event bursts.
        self._presentation_sync_timer = QTimer(self)
        self._presentation_sync_timer.setSingleShot(True)
        self._presentation_sync_timer.timeout.connect(self._sync_presentation)
        self._presentation_app = QApplication.instance()
        self._presentation_app.installEventFilter(self)

    def _workspace_header(self):
        frame = QFrame()
        frame.setObjectName("workspaceHeader")
        frame.setFixedHeight(68)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(28, 0, 28, 0)
        layout.setSpacing(12)
        brand = QVBoxLayout()
        brand.setSpacing(1)
        brand.addWidget(label("VISIONTACTILE", "brand"))
        self.banner = label(tr('演示 · 合成数据 / 不生成录包') if self.demo else tr('实时 · 正在连接工作站'), "brandCaption")
        self.banner.setFixedWidth(208)
        self.banner.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        brand.addWidget(self.banner)
        layout.addLayout(brand)
        layout.addStretch(1)
        self.navigation = QButtonGroup(self)
        navigation_bar = QWidget()
        navigation_bar.setObjectName("workspaceNavigation")
        navigation_layout = QHBoxLayout(navigation_bar)
        navigation_layout.setContentsMargins(0, 0, 0, 0)
        navigation_layout.setSpacing(8)
        # Device setup and Tracker tools retain their existing page IDs.
        for index, title in ((2, tr('设备')), (0, tr('预览')), (3, tr('空间')), (4, tr('录制')), (1, tr('档案'))):
            button = QPushButton(title)
            button.setObjectName("nav")
            button.setCheckable(True)
            button.setChecked(index == 0)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            button.setAccessibleName(title)
            button.setFixedSize(80, 64)
            if index == 4:
                # Status occupies a second line without moving any tab title.
                self.recording_nav_status = label("", "navStatus")
                self.recording_nav_status.setParent(button)
                self.recording_nav_status.setGeometry(0, 42, 80, 18)
                self.recording_nav_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
                self.recording_nav_status.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            self.navigation.addButton(button, index)
            navigation_layout.addWidget(button)
        layout.addWidget(navigation_bar)
        self.navigation.idClicked.connect(self._navigate)
        layout.addStretch(1)
        self.motion_toggle = QPushButton(tr('动效 开') if self._motion_enabled else tr('动效 关'))
        self.motion_toggle.setObjectName("motionToggle")
        self.motion_toggle.setCheckable(True)
        self.motion_toggle.setChecked(self._motion_enabled)
        self.motion_toggle.setAccessibleName(tr('界面动态效果'))
        self.motion_toggle.setToolTip(tr('记住界面动效偏好；后台自动暂停动效，不停止数据接收或录制'))
        self.motion_toggle.toggled.connect(self._set_motion_enabled)
        layout.addWidget(self.motion_toggle)
        self.help_button = QPushButton(tr('帮助'))
        self.help_button.setObjectName("utilityButton")
        self.help_button.setAccessibleName(tr('帮助与关于'))
        self.help_button.setToolTip(tr('查看使用说明、运行模式与软件信息'))
        self.help_button.clicked.connect(self._open_help)
        layout.addWidget(self.help_button)
        self.language_switch = QWidget()
        self.language_switch.setObjectName("languageSwitch")
        self.language_switch.setAccessibleName(tr('界面语言'))
        self.language_switch.setToolTip(tr('切换中英文界面；不影响采集和录制'))
        language_layout = QHBoxLayout(self.language_switch)
        language_layout.setContentsMargins(0, 0, 0, 0)
        language_layout.setSpacing(0)
        language_group = QButtonGroup(self.language_switch)
        language_group.setExclusive(True)
        self.language_buttons = {}
        for code, title, name in (("zh_CN", "中", "中文"), ("en", "EN", "English")):
            if self.language_buttons:
                divider = label("/", "languageDivider")
                divider.setAlignment(Qt.AlignmentFlag.AlignCenter)
                divider.setFixedWidth(10)
                language_layout.addWidget(divider)
            button = QPushButton(title)
            button.setObjectName("languageChoice")
            button.setAccessibleName(name)
            button.setToolTip(name)
            button.setCheckable(True)
            button.setChecked(language() == code)
            button.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            button.setFixedSize(30 if code == "zh_CN" else 34, 30)
            language_group.addButton(button)
            button.clicked.connect(lambda _checked, code=code: self._set_language(code))
            language_layout.addWidget(button)
            self.language_buttons[code] = button
        self.language_switch.setFixedSize(74, 30)
        layout.addWidget(self.language_switch)
        if self.demo:
            self.mode_badge = QToolButton()
            self.mode_badge.setObjectName("demoMenuButton")
            self.mode_badge.setText(tr('●  演示'))
            self.mode_badge.setAccessibleName(tr('演示设置'))
            self.mode_badge.setToolTip(tr('切换合成演示场景和手部模块'))
            self.mode_badge.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
            self.mode_badge.setMenu(self._build_demo_menu())
        else:
            self.mode_badge = label(tr('●  实时'), "topBadge")
            self.mode_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mode_badge.setFixedHeight(30)
        layout.addWidget(self.mode_badge)
        return frame

    def _set_language(self, code):
        if language() == code:
            set_language(code)  # An explicit choice also saves a CLI override.
            return
        # Translate existing widgets in place: drafts, selection, dialogs and
        # recorder/backend ownership all retain their original identities.
        set_language(code)
        for value, button in self.language_buttons.items():
            button.setChecked(value == language())
        self.last_events = None
        self._batches_signature = None
        self.refresh()
        self.capture_stage.refresh_layout()

    def _build_demo_menu(self):
        menu = QMenu(self)
        menu.setObjectName("demoMenu")
        self.demo_menu = menu
        for option, title, entries, actions in (
            ("scenario", tr('演示场景'), ((tr('全部正常'), "normal"), (tr('手部数据陈旧'), "stale"),
                                   (tr('头部相机离线'), "offline")), self.scenario_actions),
            ("hands", tr('手部模块'), ((tr('双手'), "both"), (tr('仅右手'), "right"),
                                (tr('仅左手'), "left"), (tr('不展示手部'), "none")), self.hands_actions),
        ):
            if menu.actions():
                menu.addSeparator()
            # Fusion renders titled separators as lines; keep group labels visible.
            heading = menu.addAction(title)
            heading.setEnabled(False)
            group = QActionGroup(menu)
            group.setExclusive(True)
            for title, value in entries:
                action = menu.addAction(title)
                action.setData(value)
                action.setCheckable(True)
                group.addAction(action)
                action.triggered.connect(
                    lambda _checked, option=option, value=value: self._set_demo_option(option, value))
                actions[value] = action
        menu.aboutToShow.connect(self._sync_demo_menu)
        self._sync_demo_menu()
        return menu

    def _sync_demo_menu(self):
        for option, actions in (("scenario", self.scenario_actions), ("hands", self.hands_actions)):
            current = getattr(self.backend, option, None)
            for value, action in actions.items():
                action.setChecked(value == current)

    def _set_demo_option(self, option, value):
        actions = {"scenario": self.scenario_actions, "hands": self.hands_actions}
        if not self.demo or value not in actions.get(option, {}):
            return
        setattr(self.backend, option, value)
        self._sync_demo_menu()
        self.refresh()

    def _open_help(self):
        if self.help_dialog is None:
            from .widgets.help_dialog import HelpDialog
            self.help_dialog = HelpDialog(self.backend, self)
        if self.help_dialog.isMinimized():
            self.help_dialog.showNormal()
        else:
            self.help_dialog.show()
        self.help_dialog.raise_()
        self.help_dialog.activateWindow()

    def _dashboard(self):
        from .widgets.capture_stage import CaptureStage
        page = QWidget()
        page.setObjectName("capturePage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        stage_row = QHBoxLayout()
        stage_row.setSpacing(12)
        self.capture_stage = CaptureStage()
        self.capture_stage.expand_requested.connect(self._expand_selected)
        self.capture_stage.calibration_requested.connect(self.open_glove_calibration)
        stage_row.addWidget(self.capture_stage, 1)
        self.details_panel = QFrame()
        self.details_panel.setObjectName("details")
        self.details_panel.setFixedWidth(262)
        panel = QVBoxLayout(self.details_panel)
        panel.setContentsMargins(18, 16, 18, 16)
        panel.setSpacing(10)
        panel.addWidget(label(tr('数据流详情'), "inspectorHeading"))
        self.detail_name = label(tr('选择数据流'), "sectionTitle", True)
        self.detail_topic = label("", "topicBox", True)
        self.detail_topic.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.detail_state = label("", wrap=True)
        self.detail_text = label("", "muted", True)
        for item in (self.detail_name, self.detail_state, self.detail_topic, self.detail_text):
            panel.addWidget(item)
        panel.addWidget(label(tr('最近活动'), "inspectorHeading"))
        self.events_label = label("", "muted", True)
        self.events_label.setMaximumHeight(110)
        panel.addWidget(self.events_label)
        panel.addStretch()
        stage_row.addWidget(self.details_panel)
        self.details_panel.hide()
        layout.addLayout(stage_row, 1)
        tools = QHBoxLayout()
        tools.setSpacing(14)
        self.overview = label(tr('数据流列表'), "streamHeading")
        tools.addWidget(self.overview)
        self.filter = QComboBox(page)
        self.filter.hide()
        self.segments = QButtonGroup(self)
        self.segment_titles = (tr('全部'), tr('相机'), tr('追踪器'), tr('手势'), tr('触觉'))
        for index, (title, value) in enumerate(zip(self.segment_titles, ("all", "camera", "tracker", "glove", "tactile"))):
            self.filter.addItem(title, value)
            button = QPushButton(title)
            button.setObjectName("segment")
            button.setCheckable(True)
            button.setChecked(index == 0)
            button.setAccessibleName(tr('筛选') + title)
            self.segments.addButton(button, index)
            tools.addWidget(button)
        self.segments.idClicked.connect(self.filter.setCurrentIndex)
        self.filter.currentIndexChanged.connect(self._invalidate_grid)
        self.filter.currentIndexChanged.connect(lambda index: self.segments.button(index).setChecked(True))
        tools.addStretch()
        self.stream_count = label("", "streamCount")
        tools.addWidget(self.stream_count)
        self.streams_toggle = QPushButton(tr('浏览数据流'))
        self.streams_toggle.setObjectName("streamsToggle")
        self.streams_toggle.setCheckable(True)
        self.streams_toggle.setAccessibleName(tr('展开或收起数据流缩略图'))
        tools.addWidget(self.streams_toggle)
        self.inspector_toggle = QPushButton(tr('数据详情'))
        self.inspector_toggle.setObjectName("inspectorToggle")
        self.inspector_toggle.setCheckable(True)
        self.inspector_toggle.toggled.connect(self.details_panel.setVisible)
        tools.addWidget(self.inspector_toggle)
        layout.addLayout(tools)
        self.scroll = QScrollArea()
        self.scroll.setObjectName("streamFilmstrip")
        self.scroll.setWidgetResizable(True)
        self.scroll.setFixedHeight(148)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.grid_host = QWidget()
        self.grid_host.setObjectName("previewGrid")
        self.grid = QGridLayout(self.grid_host)
        self.grid.setContentsMargins(0, 0, 0, 8)
        self.grid.setSpacing(10)
        self.grid.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.scroll.setWidget(self.grid_host)
        layout.addWidget(self.scroll)
        self.scroll.hide()
        self.streams_toggle.toggled.connect(self.scroll.setVisible)
        self.segments.idClicked.connect(lambda _index: self.streams_toggle.setChecked(True))
        self.filter.currentIndexChanged.connect(lambda _index: self.streams_toggle.setChecked(True))
        return page

    def _expand_selected(self):
        card = self.cards.get(self.selected_id)
        if card is not None:
            card.open_detail()

    def _devices_page(self):
        from PySide6.QtWidgets import QStackedWidget
        from .widgets.operations_pages import build_devices_page
        self.device_workspace = QStackedWidget()
        self.device_workspace.addWidget(build_devices_page(self))
        self.calibration_workspace = None
        return self.device_workspace

    def open_camera_calibration(self):
        """Reuse the device page and its existing main navigation index."""
        if self.calibration_workspace is None:
            from .widgets.calibration_workspace import CalibrationWorkspace
            self.calibration_workspace = CalibrationWorkspace(self.backend, self.device_workspace)
            self.calibration_workspace.back_requested.connect(lambda: self.device_workspace.setCurrentIndex(0))
            self.device_workspace.addWidget(self.calibration_workspace)
        self.calibration_workspace.refresh(self.last_snapshot or {})
        self.device_workspace.setCurrentWidget(self.calibration_workspace)
        self._navigate(2)

    def _open_workstation_settings(self):
        runtime = getattr(self.backend, "runtime", None)
        if runtime is None:
            self._notice(tr('演示模式只模拟流程，不保存设备配置；请从桌面工作台入口打开配置。') if self.demo else
                         tr('请从桌面工作台入口打开配置；此连接不拥有本地后台，不能修改其设置。'))
            return
        if runtime_busy(runtime):
            self._notice(tr('请等待后台准备完成，再打开设备配置。'))
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
            self._notice(tr('配置暂不可打开：{v0}', v0=exc))

    def _retry_desktop_backend(self):
        runtime = getattr(self.backend, "runtime", None)
        if runtime is None or runtime_busy(runtime):
            return
        try:
            runtime.start()
        except (ValueError, RuntimeError, OSError) as exc:
            self._notice(tr('后台暂不可启动：{v0}', v0=exc))
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
        self.configure_devices_button.setEnabled(runtime is None or not runtime_busy(runtime))
        if runtime is None:
            text = (tr('演示设备控制仅在内存中运行，不保存设置。真实配置请从桌面工作台入口打开。') if self.demo else
                    tr('从桌面工作台入口打开配置。当前连接仅展示已有后台；进程启停与数据有效性分开判断。'))
        else:
            status = runtime.status or {}
            text = diagnostic(status.get("detail", "")) or tr('新后台启动前会自动准备本地路径；需要时可调整配置，再按需启动设备。')
            if status.get("state") == "ready" and not any(
                    group.get("enabled") for group in runtime.configuration.get("process_groups", ())):
                text += " " + tr('在“配置设备”启用本次需要的模块，再选择模块启动。')
            self.retry_backend_button.setEnabled(not runtime_busy(runtime))
        self.setup_callout.setText(text)
        self.setup_callout.setToolTip(text + tr('\n相机标定在本窗口内进行；关闭旧手套标定窗口可能停止其解算。'))

    def _navigate(self, index):
        archive_playing = index == 1 and hasattr(self, "archive_replay") and not self.archive_replay.isHidden()
        self.page_heading.setVisible(index not in (0, 3) and not archive_playing)
        self.pages.setCurrentIndex(index)
        self.operation_notice.setVisible(bool(self.operation_notice.text()) and self._notice_page in (None, index))
        self.navigation.button(index).setChecked(True)
        self.ambient.set_mode(("capture", "sessions", "devices", "spatial", "recording")[index])
        self.page_eyebrow.setText((tr('02 / 实时预览'), tr('05 / 数据档案'), tr('01 / 设备设置'),
                                  tr('03 / 空间视图'), tr('04 / 录制'))[index])
        self.page_title.setText((tr('预览'), tr('档案'), tr('设备'), tr('空间'), tr('录制'))[index])
        self.workspace_caption.setText((tr('实时数据'), tr('试次与质检'), tr('设备与标定'), tr('统一空间'), tr('批次与试次'))[index])

    def _set_motion_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled != self._motion_enabled:
            self._motion_enabled = enabled
            self._ui_settings.setValue("motion/enabled", enabled)
            self._ui_settings.sync()
        self._apply_motion()

    def _apply_motion(self):
        enabled = self._motion_enabled and self._presentation_active and not self._closing
        if enabled != self._applied_motion:
            self._applied_motion = enabled
            self.ambient.set_motion_enabled(enabled)
            self.pages.set_motion_enabled(enabled)
            self.capture_stage.set_motion_enabled(enabled)
            self.spatial_page.set_motion_enabled(enabled)
        self.motion_toggle.setText(tr('动效 开') if self._motion_enabled else tr('动效 关'))
        self.motion_toggle.setAccessibleDescription(
            tr('动态效果已开启') if enabled else
            tr('动态效果在后台暂停；返回前台后恢复') if self._motion_enabled else
            tr('已减少动态效果；实时数据继续更新'))

    def _sync_presentation(self):
        if not hasattr(self, "timer") or self._closing:
            return
        app = QApplication.instance()
        active = app.activeWindow()
        foreground = (app.applicationState() == Qt.ApplicationState.ApplicationActive
                      and active is not None and active.isVisible() and not active.isMinimized())
        # Decorative motion belongs to the main window; live data polling also
        # serves independently focused camera, Tracker, hand and spatial views.
        self._presentation_active = foreground and active is self
        self._apply_motion()
        # Only presentation polling slows down. The IPC receivers and manager's
        # frame counters keep running, including while recording in background.
        data_active = self._presentation_active or (foreground and self._is_data_window(active))
        interval = self._refresh_ms if data_active else max(500, self._refresh_ms)
        if self.timer.interval() != interval:
            self.timer.setInterval(interval)

    def _is_data_window(self, window):
        if window is None:
            return False
        if window is self.spatial_page.detail_window:
            return True
        return any(window is dialog for card in self.cards.values()
                   for dialog in (card.preview_dialog, card.glove_dialog, card.tactile_dialog))

    def eventFilter(self, watched, event):
        if (not self._closing and event.type() in (
                QEvent.Type.WindowActivate, QEvent.Type.WindowDeactivate,
                QEvent.Type.WindowStateChange, QEvent.Type.Show, QEvent.Type.Hide,
                QEvent.Type.ApplicationStateChange)
                and (watched is self._presentation_app
                     or isinstance(watched, QtWidget) and watched.isWindow())):
            if not self._presentation_sync_timer.isActive():
                self._presentation_sync_timer.start(0)
        return super().eventFilter(watched, event)

    def showEvent(self, event):
        super().showEvent(event)
        self._sync_presentation()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._sync_presentation()

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() in (QEvent.Type.ActivationChange, QEvent.Type.WindowStateChange):
            self._sync_presentation()

    def _open_spatial_settings(self):
        from .widgets.spatial_settings import SpatialSettingsDialog
        if self.spatial_settings_dialog is None:
            self.spatial_settings_dialog = SpatialSettingsDialog(
                getattr(self.backend, "runtime", None), lambda: self.last_snapshot, self)
            self.spatial_settings_dialog.finished.connect(self._spatial_settings_closed)
        self.spatial_settings_dialog.show()
        self.spatial_settings_dialog.raise_()

    def _spatial_settings_closed(self, *_args):
        self.spatial_settings_dialog = None

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
        card = self.cards.get(identifier)
        if card is not None:
            self.capture_stage.set_source(card.preview, card.device_snapshot)
            self.scroll.ensureWidgetVisible(card, 12, 0)
        for key, card in self.cards.items():
            card.setProperty("selected", key == identifier)
            card.style().unpolish(card)
            card.style().polish(card)

    def _update_details(self):
        device = self.devices.get(self.selected_id)
        if not device:
            self.detail_name.setText(tr('暂无设备数据'))
            self.detail_state.setText(tr('等待后台或数据主题发布'))
            self.detail_topic.clear()
            self.detail_text.clear()
            return
        self.detail_name.setText(device["label"])
        self.detail_topic.setText(device["topic"])
        age = device.get("age_s")
        freshness = tr('尚未收到样本') if age is None else tr('接收龄期 {v0:.0f} ms', v0=age * 1000)
        self.detail_state.setText(STATUS.get(device["status"], STATUS["unknown"])[0] + "\n" + freshness)
        self.detail_text.setText(diagnostic(device.get("detail", "")))

    def _notice(self, message, *, page=None):
        self._notice_page = page
        self.operation_notice.setText(diagnostic(message)[:180])
        self.operation_notice.setToolTip(diagnostic(message))
        self.operation_notice.setVisible(page is None or self.pages.currentIndex() == page)

    @property
    def pending_request(self):
        pending = self.operations.pending
        return pending.request_id if pending else None

    @property
    def pending_action(self):
        pending = self.operations.pending
        return pending.command if pending else None

    @property
    def pending_instance_id(self):
        pending = self.operations.pending
        return pending.manager_id if pending else None

    @property
    def pending_controller_id(self):
        pending = self.operations.pending
        return pending.controller_id if pending else None

    @property
    def pending_batch_ids(self):
        pending = self.operations.pending
        return pending.batch_ids if pending else frozenset()

    def _apply_operation_update(self, update: OperationUpdate | None):
        if update is None:
            return
        if update.update_batch_selection:
            self.batch_selector.setCurrentIndex(
                self.batch_selector.findData(update.batch_id) if update.batch_id is not None else -1)
        if update.clear_batch_name:
            self.batch_name.clear()
        if update.refresh_details:
            self._details_polled_at = 0.0
        self._notice(update.notice, page={"archive": 1, "recording": 4}.get(update.notice_page))

    def _submit(self, command, params):
        update = self.operations.submit(command, params, sender=self.backend.submit,
                                        snapshot=self.last_snapshot or {}, management=self.management)
        if update is None:
            return
        self._apply_operation_update(update)
        self.refresh()

    def _create_batch(self):
        if not (self.demo or self.management.get("enabled")):
            self._notice(tr('后台未启用管理模式，不能创建正式批次。'))
            return
        title = self.batch_name.text().strip()
        if not title:
            self._notice(tr('请填写新批次名称。'), page=4)
            self.batch_name.setFocus()
            return
        self._submit("batch.create", {"label": title})

    def _start_recording(self):
        if self.pending_request:
            return
        if not self.demo and not recording_access(self.last_snapshot or {}, self.management)[0]:
            self._notice(tr('当前不能开始：请检查后台管理模式、控制器连接和未决试次。'))
            return
        batch_id = self.batch_selector.currentData()
        title = self.trial_name.text().strip()
        if not batch_id or not title:
            self.navigation.button(4).click()
            self._notice(tr('请先创建或选择采集批次，并填写试次名称。'), page=4)
            return
        context = self.management.get("controller") or {}
        params = {"batch_id": batch_id, "label": title, "duration_s": self.duration.value(),
                  "expected_controller_instance_id": context.get("controller_instance_id", "demo"),
                  "expected_state_revision": context.get("state_revision", 0)}
        plan_view = self._capture_plan_view()
        if plan_view.blocks_start:
            self._notice(plan_view.status, page=4)
            return
        params.update(plan_view.start_parameters())
        if getattr(self.backend, "supports_details", False):
            params.update(task_name=self.task_name.text().strip(), operator_name=self.operator_name.text().strip())
        elif self.task_name.text().strip() or self.operator_name.text().strip():
            self._notice(tr('后台当前不支持元信息；请恢复 v3 连接，或清空任务 / 操作员字段后使用旧版录制。'))
            return
        self._submit("capture.start", params)

    def _capture_plan_view(self):
        capabilities = (self.last_snapshot or {}).get("capabilities", {})
        supported = bool(getattr(self.backend, "supports_capture_plan", False))
        return capture_plan_view(
            self.management.get("controller"), duration_s=self.duration.value(),
            supported=supported, required=supported or "capture_plan" in capabilities,
            demo=self.demo,
        )

    def _refresh_capture_plan(self):
        view = self._capture_plan_view()
        self.capture_plan_summary.setText(view.summary)
        self.capture_plan_status.setText(view.status)
        self.capture_plan_status.setToolTip(view.status)
        self.capture_plan_topics.setVisible(bool(view.topics_text))
        self.capture_plan_topics.setToolTip(view.topics_text)
        self.capture_plan_topics.setAccessibleDescription(view.topics_text)
        return view

    def _stop_recording(self):
        if self.pending_request:
            return
        if self.demo:
            trial = next((t for t in self.management.get("trials", []) if t["state"] == "RECORDING"), None)
        else:
            if not recording_access(self.last_snapshot or {}, self.management)[1]:
                self._notice(tr('当前没有可由本工作台停止的已确认录制。'))
                return
            trial = owned_active_trial(self.management)
        if not trial:
            return
        context = self.management.get("controller") or {}
        self._submit("capture.stop", {"trial_id": trial["id"],
                     "expected_controller_instance_id": context.get("controller_instance_id", "demo"),
                     "expected_session_id": trial["session_id"]})

    def _show_trial_details(self):
        row = self.trial_table.currentRow() if self.trial_table.selectionModel().hasSelection() else -1
        self.inspect_trial_button.setEnabled(0 <= row < len(self._trial_rows))
        from .widgets.topic_replay import trial_replay_allowed
        trial = self._trial_rows[row] if 0 <= row < len(self._trial_rows) else None
        self.replay_trial_button.setEnabled(trial_replay_allowed(trial, demo=self.demo))
        identity = trial.get("id") if trial else None
        if identity != getattr(self, "_archive_selected_id", None):
            self._archive_selected_id = identity
            self.archive_replay.clear()
            if trial is None:
                self.trial_details.setText(tr('选择试次可查看会话编号、实际路径及最后状态。'))
        self.trial_table.setProperty("selectedTrialId", identity)
        if 0 <= row < len(self._trial_rows):
            trial = self._trial_rows[row]
            text = tr('试次 {v0}  ·  会话 {v1}\n', v0=trial['id'], v1=trial.get('session_id') or tr('尚未确认'))
            text += tr('数据包：{v0}\n{v1}', v0=trial.get('output_path') or tr('暂无确认路径'), v1=diagnostic(trial.get('detail', '')))
            self.trial_details.setText(text)
            self.trial_details.setToolTip(text)

    def _archive_path_active(self, path):
        from pathlib import Path
        def candidates(value):
            resolved = Path(value).expanduser().resolve()
            result = {resolved, resolved / "bag"}
            if resolved.name == "bag":
                result.add(resolved.parent)
            return result
        try:
            selected = candidates(path)
            return any(t.get("output_path") and t.get("state") not in {"COMPLETE", "FAILED"}
                       and selected & candidates(t["output_path"]) for t in self._trial_rows)
        except (OSError, ValueError, RuntimeError):
            return True

    def _open_trial_replay(self, trial_id=None):
        from .widgets.topic_replay import trial_replay_allowed
        row = self.trial_table.currentRow()
        trial = next((t for t in self._trial_rows if t["id"] == trial_id), None) if isinstance(trial_id, str) else (
            self._trial_rows[row] if 0 <= row < len(self._trial_rows) else None)
        self._navigate(1)
        if not trial_replay_allowed(trial, demo=self.demo):
            self.archive_replay.clear(tr('此试次尚未结束或没有实际录制包。'))
            return
        if isinstance(trial_id, str):
            self.trial_table.selectRow(self._trial_rows.index(trial))
        if self.details_dialog is not None:
            self.details_dialog.close()
        self.archive_history.hide()
        self.archive_replay.show()
        self.page_heading.hide()
        self.archive_replay.load(trial["output_path"], label=trial.get("label", trial["id"]))

    def _browse_archive(self):
        self.archive_replay.hide()
        self.archive_history.show()
        self.page_heading.show()

    def _choose_archive_bag(self):
        from PySide6.QtWidgets import QFileDialog
        self._navigate(1)
        path = QFileDialog.getExistingDirectory(self, tr('打开已结束的录制包或会话目录'))
        if path:
            self.trial_table.clearSelection()
            self.trial_table.setCurrentCell(-1, -1)
            self.trial_table.setProperty("selectedTrialId", None)
            self._show_trial_details()
            self.trial_details.setText(path)
            self.archive_history.hide()
            self.archive_replay.show()
            self.page_heading.hide()
            self.archive_replay.load(path)

    def _open_trial_inspector(self):
        row = self.trial_table.currentRow()
        if not 0 <= row < len(self._trial_rows):
            return
        if self.details_dialog is None:
            self.details_dialog = TrialDetailsDialog(self)
            self.details_dialog.refresh_requested.connect(self._request_trial_details)
            self.details_dialog.retry_requested.connect(self._retry_quality)
            self.details_dialog.replay_requested.connect(self._open_trial_replay)
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
                self.details_dialog.connection.setText(tr('详情暂不可用：{v0}', v0=exc))

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
            dialog.connection.setText(tr('当前后台列表中没有此试次，保留上次详情；请重新选择。'))
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
                self.batch_selector.addItem(tr('尚无批次'), None)
            for batch in batches:
                self.batch_selector.addItem(self._batch_label(batch), batch["id"])
            index = self.batch_selector.findData(selected)
            if index >= 0:
                self.batch_selector.setCurrentIndex(index)
            self.batch_selector.blockSignals(False)
            self._batches_signature = signature
        self._apply_operation_update(self.operations.observe(
            snapshot, self.management, operation_for(self.backend, self.pending_request, self.management)))
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
            editor.setToolTip(tr('录制开始时冻结；可留空') if supports_metadata else tr('旧后台不支持保存此字段；可清空已有内容后使用旧版录制，不会静默丢弃元信息'))
        batch_labels = {b["id"]: self._batch_label(b) for b in batches}
        self._trial_rows = self.management.get("trials", [])
        rows = [(t.get("label", t["id"]), batch_labels.get(t["batch_id"], t["batch_id"]),
                 TRIAL_LABELS.get(t["state"], t["state"]), tr('{v0:.1f} / {v1:.0f} 秒', v0=t['elapsed_s'], v1=t['duration_s']),
                 QUALITY_LABELS.get(t.get("quality"), t.get("quality", tr('未执行'))),
                 t.get("output_path") or (tr('仅演示，无数据包') if self.demo else tr('等待确认'))) for t in self._trial_rows]
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

    def _batch_label(self, batch):
        if self.demo and batch.get("id") == "demo-batch" and batch.get("label") == "演示批次":
            return tr('演示批次')
        return batch.get("label", batch["id"])

    def open_glove_calibration(self, hand):
        """Reuse the managed glove group, never launch a competing serial owner."""
        if hand not in ("right", "left") or self.demo:
            self.statusBar().showMessage(tr('演示模式不启动真实标定工具。'), 8000)
            return
        snapshot = self.backend.snapshot()
        status = glove_status(snapshot, hand)
        if status["can_focus"]:
            _, message = focus_owned_calibration(status["group"], hand)
            self.statusBar().showMessage(str(diagnostic(message)), 10000)
            return
        if not status["can_start"]:
            self.statusBar().showMessage(str(diagnostic(status["message"])), 10000)
            return
        runtime = getattr(self.backend, "runtime", None)
        if runtime is None:
            self.statusBar().showMessage(tr('请从桌面工作台打开标定；只读连接不会自行启动手套窗口。'), 10000)
            return
        group = status["group"]
        profile = next((g for g in runtime.configuration.get("process_groups", ()) if g["id"] == group["id"]), None)
        parameters = profile.get("parameters", {}) if profile else {}
        if parameters.get("hand") != hand or str(parameters.get("launch_gui", "true")).lower() != "true":
            self.statusBar().showMessage(tr('请在配置设备中确认手别，并启用标定窗口；不会另起串口程序。'), 10000)
            return
        self.device_controls.refresh(snapshot)
        self.device_controls.selected_id = group["id"]
        pending_before = self.device_controls.pending_request
        self.device_controls.submit("start")
        accepted = self.device_controls.pending_request and self.device_controls.pending_request != pending_before
        message = (tr('已请求打开本手原标定窗口。请在窗口中连接 IMU 并标定；不会自动标定。') if accepted
                   else self.device_controls.operation_notice.text() or tr('当前无法提交启动请求，请核对进程状态。'))
        self.statusBar().showMessage(str(diagnostic(message)), 10000)

    def refresh(self):
        snapshot = self.backend.snapshot()
        self.last_snapshot = snapshot
        self.scene_backend.set_manager_instance(snapshot.get("instance_id"))
        self._refresh_management(snapshot)
        self.device_controls.refresh(snapshot)
        self._refresh_desktop_runtime()
        if self.calibration_workspace is not None:
            self.calibration_workspace.refresh(snapshot)
        self.devices = {device["id"]: {**device, "label": device_label(device, demo=self.demo)}
                        for device in snapshot["devices"]}
        for identifier, device in tuple(self.devices.items()):
            if device["kind"] == "glove" and device.get("side") in ("right", "left"):
                status = glove_status(snapshot, device["side"])
                self.devices[identifier] = {**device, "glove_state": status["state"],
                    "glove_hint": tr('演示手势 · 不连接硬件') if self.demo else status["message"],
                    "glove_action_enabled": bool(not self.demo and (status["can_start"] or status["can_focus"]))}
        online = sum(device["status"] == "online" for device in self.devices.values())
        self.stream_count.setText(tr('{v0} / {v1} 路在线', v0=online, v1=len(self.devices)))
        for index, title in enumerate(self.segment_titles):
            kind = self.filter.itemData(index)
            count = len(self.devices) if kind == "all" else sum(d["kind"] == kind for d in self.devices.values())
            self.segments.button(index).setText(tr('{v0}  {v1}', v0=title, v1=count))
        selected_kind = self.filter.currentData()
        shown = [d for d in self.devices.values() if selected_kind == "all" or d["kind"] == selected_kind]
        signature = tuple(d["id"] for d in shown)
        for identifier in set(self.cards) - set(self.devices):
            if identifier == self.selected_id:
                self.capture_stage.clear_source()
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
                    card = DeviceCard(device, compact=True)
                    card.selected.connect(self._select)
                    card.calibration_requested.connect(self.open_glove_calibration)
                    self.cards[device["id"]] = card
                card = self.cards[device["id"]]
                self.grid.addWidget(card, 0, index)
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
        if shown and self.selected_id not in shown_ids:
            self._select(shown[0]["id"])
        elif not shown:
            self.selected_id = None
            self.capture_stage.clear_source()
        if self.selected_id in self.cards and self.selected_id in self.devices:
            selected = self.cards[self.selected_id]
            self.capture_stage.set_source(selected.preview, selected.device_snapshot)
        self._update_details()
        recording = snapshot["recording"]
        state = recording["state"]
        prefix = tr('模拟 · ') if self.demo else tr('录制 · ')
        self.record_state.setText(prefix + RECORDING.get(state, state))
        self.record_detail.setText(diagnostic(recording.get("detail", "")))
        if (not self.demo and state == "UNAVAILABLE" and self.management.get("enabled")
                and not self.management.get("controller") and not any(
                    trial.get("state") in {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"}
                    for trial in self.management.get("trials", ()))):
            self.record_state.setText(tr('录制 · 等待控制器'))
            self.record_detail.setText(tr('尚无新鲜录制状态；请在“设备”配置并启动相机与录制控制器组。'))
        can_start, can_stop = recording_access(snapshot, self.management, bool(self.pending_request))
        if self.demo:
            can_start = state != "RECORDING" and not self.pending_request
            can_stop = state == "RECORDING" and not self.pending_request
        plan_view = self._refresh_capture_plan()
        can_start = can_start and not plan_view.blocks_start
        duration = self.duration.value() if state == "IDLE" and can_start else recording["duration_s"]
        elapsed = 0. if state == "IDLE" and can_start else recording["elapsed_s"]
        if duration > 0 and state != "UNAVAILABLE":
            self.elapsed.setText(tr('{v0:02d}:{v1:02d} / {v2:02d}:{v3:02d}', v0=int(elapsed) // 60, v1=int(elapsed) % 60, v2=int(duration) // 60, v3=int(duration) % 60))
            self.progress.setValue(int(1000 * min(1.0, elapsed / duration)))
        else:
            self.elapsed.setText(tr('时长未知'))
            self.progress.setValue(0)
        self.start_button.setEnabled(can_start)
        self.stop_button.setEnabled(can_stop)
        self.duration.setEnabled(can_start)
        self._refresh_recording_context(recording)
        self.start_button.setToolTip(plan_view.status if plan_view.blocks_start else
                                    tr('创建批次后开始录制；只记录控制器已配置的数据主题') if can_start else
                                    tr('需要启用管理后台、连接控制器且无未决录制；不会自动重试'))
        if not self.demo:
            managed = self.management.get("enabled", False)
            device_control = self.device_controls.control_available
            self.mode_badge.setText(tr('●  实时 / 采集') if managed else
                                    tr('●  实时 / 设备') if device_control else tr('●  实时 / 只读'))
            if device_control:
                self.banner.setText(tr('实时 · 设备控制已启用 / 手动启停'))
                self.banner.setToolTip(tr('在设备页显式启停本后台进程；不接管外部进程，不修改标定。'))
            else:
                self.banner.setText(tr('实时 · 采集管理 / 手动控制录制') if managed else tr('实时 · 只读模式 / 预览已有数据流'))
                self.banner.setToolTip(tr('设备控制需后台单独启用。') if managed else tr('管理功能需后台显式启用。'))
        if snapshot["events"] != self.last_events:
            self.events_label.setText(tr("\n\n").join(diagnostic(event["message"]) for event in snapshot["events"][:5]) or tr('暂无事件'))
            self.last_events = snapshot["events"]
        set_rows(self.device_table, [(d["label"], KIND.get(d["kind"], d["kind"]), STATUS.get(d["status"], STATUS["unknown"])[0],
                                      tr('{v0:.1f}', v0=d['receive_hz']), "—" if d["valid_hz"] is None else tr('{v0:.1f}', v0=d['valid_hz']), d["topic"])
                                     for d in self.devices.values()])
        free = snapshot["resources"].get("disk_free_bytes")
        disk = tr('剩余空间 {v0:.1f} GiB', v0=free / 1024**3) if free is not None else tr('磁盘空间 未检测')
        mode_text = tr('演示 · 合成数据') if self.demo else (tr('实时 · 采集管理') if self.management.get("enabled") else
                    tr('实时 · 设备控制') if self.device_controls.control_available else tr('实时 · 只读'))
        self.footer.setText(tr('{v0}    |    {v1}    |    状态刷新上限 {v2} Hz    |    不等同于采集频率', v0=mode_text, v1=disk, v2=1000 // self.timer.interval()))
        if self.demo:
            self.scene_backend.scenario = "offline" if getattr(self.backend, "scenario", "") == "offline" else "moving"
        self.spatial_page.refresh()
        self.spatial_page.set_preview_sources({key: card.preview for key, card in self.cards.items()
                                              if card.device_snapshot.get("kind") == "camera"})

    def _refresh_recording_context(self, recording):
        """Expose the same confirmed state in the recording page and its tab."""
        state = recording["state"]
        trials = self.management.get("trials", ())
        session_id = recording.get("session_id")
        current = next((trial for trial in trials if session_id and trial.get("session_id") == session_id), None)
        unresolved = [trial for trial in trials if trial.get("state") in {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"}]
        trial_state = (current or {}).get("state")
        phase = None
        if state == "UNAVAILABLE":
            phase = tr('待核实') if unresolved or self.pending_action in {"capture.start", "capture.stop"} else None
        elif trial_state == "UNKNOWN" or (not current and any(t.get("state") == "UNKNOWN" for t in unresolved)):
            phase = tr('待核实')
        elif self.pending_action == "capture.stop" or trial_state == "STOPPING":
            phase = tr('结束中')
        elif self.pending_action == "capture.start" or trial_state == "STARTING":
            phase = tr('开始中')
        elif state == "RECORDING":
            phase = tr('录制中')
        elif trial_state == "FAILED" or state == "FAILED":
            phase = tr('失败')

        button = self.navigation.button(4)
        if phase == tr('录制中'):
            elapsed = max(0, int(recording["elapsed_s"]))
            clock = tr('{v0:02d}:{v1:02d}', v0=elapsed // 60, v1=elapsed % 60)
            self.recording_nav_status.setText(clock)
        else:
            self.recording_nav_status.setText(phase or "")
        if phase:
            self.record_state.setText((tr('模拟 · ') if self.demo else tr('录制 · ')) + phase)
        self.record_detail.setToolTip(self.record_detail.text())
        button.setToolTip(self.record_state.text() + "\n" + self.record_detail.text() + tr('\n点击进入录制页'))
        button.setAccessibleDescription(button.toolTip())

        if current and state != "IDLE":
            batch = next((self._batch_label(b) for b in self.management.get("batches", ())
                          if b["id"] == current.get("batch_id")), current.get("batch_id") or tr('未确认批次'))
            text = tr('试次：{v0}\n批次：{v1}', v0=current.get('label', current['id']), v1=batch)
        elif state == "UNAVAILABLE" and unresolved:
            text = tr('存在状态未确认的试次，可到档案查看已记录的信息。')
        else:
            batch = self.batch_selector.currentText() if self.batch_selector.currentData() else tr('尚未选择批次')
            text = tr('本次设置：{v0}\n批次：{v1}', v0=self.trial_name.text().strip() or tr('尚未填写试次名称'), v1=batch)
        self.recording_context.setText(text)
        self.recording_context.setToolTip(text)

    def _finish_deployment_close(self, *_):
        self._deployment_close_pending = False
        if self.settings_dialog is not None:
            self.settings_dialog.finished.disconnect(self._finish_deployment_close)
        QTimer.singleShot(0, self.close)

    def closeEvent(self, event):
        if self.settings_dialog is not None and self.settings_dialog.deployment_running:
            if not self._deployment_close_pending:
                self._deployment_close_pending = True
                self.settings_dialog.finished.connect(self._finish_deployment_close)
            self.settings_dialog.close()
            event.ignore()
            return
        self._closing = True
        self._presentation_app.removeEventFilter(self)
        self._presentation_sync_timer.stop()
        self._apply_motion()
        self.timer.stop()
        self.archive_replay.close()
        self.capture_stage.clear_source()
        if self.help_dialog is not None:
            self.help_dialog.close()
        self.spatial_page.close()
        if self.spatial_settings_dialog is not None:
            self.spatial_settings_dialog.close()
        self.scene_backend.close()
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
