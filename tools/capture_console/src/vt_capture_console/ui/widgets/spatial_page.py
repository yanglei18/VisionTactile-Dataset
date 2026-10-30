"""Joint spatial page. All views consume one application-owned backend."""
from __future__ import annotations

from ...i18n import tr
from ...i18n.labels import scene_entity_label, tracker_role
from ...i18n.demo import demo_scene_frame
from ...i18n.messages import message as display_message

import datetime
import os
import uuid

from PySide6.QtCore import QEvent, QSize, Qt, Signal
from ...i18n.widgets import (QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFrame, QHBoxLayout,
                              QHeaderView, QLabel, QListWidget, QListWidgetItem, QMenu, QPushButton, QScrollArea, QSlider, QTabWidget,
                              QSizePolicy, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from ...viewmodels.spatial import SpatialViewModel, STATE_LABELS
from .preview import PreviewWidget
from .spatial_scene import SpatialScene
from ...backend.cloud_presentation import CloudPresentation
from ...backend.cloud_display import WristCloudDisplay
from .. import presentation_settings


def plain_label(text="", parent=None, *, name=""):
    widget = QLabel(text, parent)
    widget.setObjectName(name)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setWordWrap(True)
    return widget


def section_title(text):
    return plain_label(text, name="spatialSectionTitle")


def divider():
    line = QFrame()
    line.setObjectName("divider")
    line.setFixedHeight(1)
    return line


class SpatialPage(QWidget):
    configure_requested = Signal()
    archive_replay_requested = Signal()

    def __init__(self, backend, parent=None, *, allow_detail=True, archive_embedded=False):
        super().__init__(parent)
        self._archive_embedded = bool(archive_embedded)
        allow_detail = allow_detail and not self._archive_embedded
        self.backend = backend
        self._live_backend = backend
        self._replay_backend = None
        self.model = SpatialViewModel()
        self.viewer_id = "spatial-"+uuid.uuid4().hex
        self.detail_window = None
        self._list_signature = None
        self._preview_sources = {}
        self._preview_widgets = {}
        self._last_status = {}
        self._clouds = {}
        self._wrist_cloud_display = WristCloudDisplay()
        self._cloud_presentation = CloudPresentation()
        self._presentation_clouds = None
        self._allow_detail = allow_detail
        self._motion_enabled = True
        self._observed_window = None
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)
        toolbar_panel = QFrame()
        toolbar_panel.setObjectName("spatialToolbar")
        toolbar = QHBoxLayout(toolbar_panel)
        toolbar.setContentsMargins(0, 0, 0, 0)
        toolbar.setSpacing(8)
        self.preset = QComboBox()
        self.preset.setAccessibleName(tr('联合空间显示内容'))
        for label, key in ((tr('全部实体'), "combined"), (tr('追踪器'), "tracker"), (tr('相机'), "camera")):
            self.preset.addItem(label, key)
        self.preset.currentIndexChanged.connect(self._preset_changed)
        toolbar.addWidget(self.preset)
        self.status = plain_label(tr('等待联合空间数据'))
        self.status.setObjectName("spatialStatus")
        self.status.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(self.status, 1)
        self.replay_open_button = QPushButton(tr('回放'))
        self.replay_open_button.setObjectName("spatialAction")
        menu = QMenu(self.replay_open_button)
        menu.addAction(tr('在档案中兼容回放'), self.archive_replay_requested.emit)
        menu.addAction(tr('打开试次 / 对齐结果目录'), self._choose_replay_directory)
        menu.addAction(tr('打开场景快照 / 对齐帧文件'), self._choose_replay_file)
        self.replay_open_button.setMenu(menu)
        self.replay_open_button.setVisible(allow_detail)
        toolbar.addWidget(self.replay_open_button)
        self.live_button = QPushButton(tr('返回实时'))
        self.live_button.setObjectName("spatialAction")
        self.live_button.clicked.connect(self.return_live)
        self.live_button.hide()
        toolbar.addWidget(self.live_button)
        self.configure_button = QPushButton(tr('空间配置'))
        self.configure_button.setObjectName("spatialAction")
        self.configure_button.clicked.connect(self.configure_requested)
        self.configure_button.setEnabled(not self._archive_embedded)
        self.configure_button.setVisible(not self._archive_embedded)
        toolbar.addWidget(self.configure_button)
        self.expand_button = QPushButton(tr('放大'))
        self.expand_button.setObjectName("spatialAction")
        self.expand_button.clicked.connect(self.open_detail)
        self.expand_button.setVisible(allow_detail)
        toolbar.addWidget(self.expand_button)
        self.camera_toggle = QPushButton(tr('相机预览'))
        self.camera_toggle.setObjectName("spatialPreviewToggle")
        self.camera_toggle.setCheckable(True)
        self.camera_toggle.setToolTip(tr('展开独立实时相机预览；图像未与三维场景按帧配对'))
        self.camera_toggle.toggled.connect(self._update_camera_strip_visibility)
        self.camera_toggle.setVisible(not self._archive_embedded)
        toolbar.addWidget(self.camera_toggle)
        self.inspector_toggle = QPushButton(tr('场景面板'))
        self.inspector_toggle.setObjectName("spatialPanelToggle")
        self.inspector_toggle.setCheckable(True)
        self.inspector_toggle.setChecked(False)
        self.inspector_toggle.toggled.connect(self._toggle_inspector)
        toolbar.addWidget(self.inspector_toggle)
        root.addWidget(toolbar_panel)
        # Keep provenance separate from changing stream/cloud status and hidden
        # entity panels. It must remain visible even when geometry is unavailable.
        self.preview_notice = plain_label(name="spatialPreviewNotice")
        self.preview_notice.setAccessibleName(tr('空间预览说明'))
        self.preview_notice.setStyleSheet(
            "QLabel { color: #713f12; background: #fff3cd; border: 1px solid #d9a441; "
            "border-radius: 5px; padding: 8px 12px; }")
        self.preview_notice.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.preview_notice.hide()
        root.addWidget(self.preview_notice)

        workspace = QHBoxLayout()
        workspace.setContentsMargins(0, 0, 0, 0)
        workspace.setSpacing(12)
        scene_column = QWidget()
        scene_layout = QVBoxLayout(scene_column)
        scene_layout.setContentsMargins(0, 0, 0, 0)
        scene_layout.setSpacing(8)
        self.scene = SpatialScene()
        self.scene.selected.connect(self._inspect_entity)
        scene_layout.addWidget(self.scene, 1)

        view_tools = QFrame()
        view_tools.setObjectName("spatialViewTools")
        tools = QHBoxLayout(view_tools)
        tools.setContentsMargins(12, 8, 12, 8)
        tools.setSpacing(8)
        fit_button = QPushButton(tr('自动取景'))
        fit_button.setObjectName("spatialAction")
        fit_button.clicked.connect(self.scene.fit)
        tools.addWidget(fit_button)
        self.view_preset = QComboBox()
        self.view_preset.setAccessibleName(tr('三维视角'))
        for title, view in ((tr('自由视角'), "reset"), (tr('俯视'), "top"), (tr('前视'), "front"), (tr('侧视'), "side")):
            self.view_preset.addItem(title, view)
        self.view_preset.activated.connect(lambda: self.scene.standard_view(self.view_preset.currentData()))
        tools.addWidget(self.view_preset)
        self.follow = QCheckBox(tr('跟随所选'))
        tools.addWidget(self.follow)
        tools.addStretch()
        self.cruise_button = QPushButton(tr('巡航视角'))
        self.cruise_button.setObjectName("spatialCruise")
        self.cruise_button.setCheckable(True)
        self.cruise_button.setToolTip(tr('缓慢环绕场景；拖拽或缩放立即停止。仅改变观察视角。'))
        self.cruise_button.toggled.connect(self.scene.set_cruising)
        self.scene.cruise_changed.connect(self.cruise_button.setChecked)
        self.cruise_button.setChecked(getattr(backend, "mode", "live") == "demo")
        tools.addWidget(self.cruise_button)
        self.freeze_button = QPushButton(tr('冻结画面'))
        self.freeze_button.setObjectName("spatialFreeze")
        self.freeze_button.setCheckable(True)
        self.freeze_button.toggled.connect(self._freeze)
        self.freeze_button.setVisible(not self._archive_embedded)
        tools.addWidget(self.freeze_button)
        scene_layout.addWidget(view_tools)

        self.camera_strip = QWidget()
        self.camera_layout = QHBoxLayout(self.camera_strip)
        self.camera_layout.setContentsMargins(0, 0, 0, 0)
        self.camera_layout.setSpacing(8)
        self.camera_strip.hide()
        scene_layout.addWidget(self.camera_strip)
        workspace.addWidget(scene_column, 1)

        self.inspector = QFrame()
        self.inspector.setObjectName("spatialInspector")
        self.inspector.setFixedWidth(286)
        inspector_layout = QVBoxLayout(self.inspector)
        inspector_layout.setContentsMargins(14, 16, 14, 12)
        inspector_layout.setSpacing(14)
        heading = QHBoxLayout()
        heading.addWidget(section_title("SCENE CONTROL"), 1)
        dismiss = QPushButton(tr('收起'))
        dismiss.setObjectName("spatialAction")
        dismiss.clicked.connect(lambda: self.inspector_toggle.setChecked(False))
        heading.addWidget(dismiss)
        inspector_layout.addLayout(heading)
        self.inspector_tabs = QTabWidget()
        self.inspector_tabs.setObjectName("spatialInspectorTabs")
        self.inspector_tabs.setDocumentMode(True)
        self.inspector_tabs.setUsesScrollButtons(False)
        inspector_layout.addWidget(self.inspector_tabs, 1)

        def inspector_tab(title):
            page = QWidget()
            layout = QVBoxLayout(page)
            layout.setContentsMargins(0, 14, 0, 0)
            layout.setSpacing(12)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.Shape.NoFrame)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            scroll.setWidget(page)
            self.inspector_tabs.addTab(scroll, title)
            return layout

        entities_layout = inspector_tab(tr('实体'))
        entities_layout.addWidget(plain_label(tr('选择实体查看状态，勾选控制显示。'), name="spatialSectionCaption"))
        self.devices = QListWidget()
        self.devices.setObjectName("spatialDevices")
        self.devices.setMinimumHeight(136)
        self.devices.itemClicked.connect(lambda item: self.select(item.data(Qt.ItemDataRole.UserRole)))
        self.devices.itemChanged.connect(self._visibility_changed)
        entities_layout.addWidget(self.devices)
        entity_detail_button = QPushButton(tr('查看所选实体的数据 →'))
        entity_detail_button.setObjectName("spatialAction")
        entity_detail_button.clicked.connect(lambda: self.inspector_tabs.setCurrentIndex(3))
        entities_layout.addWidget(entity_detail_button)
        entities_layout.addStretch()
        entities_layout.addWidget(plain_label(tr('所有实体共用一个世界坐标系。\n实体外形为示意；视锥使用标定几何。\n参考网格不代表真实地面。'), name="spatialSectionCaption"))

        layer_layout = inspector_tab(tr('图层'))
        layer_layout.addWidget(section_title(tr('可见图层')))
        layer_layout.addWidget(plain_label(tr('仅影响当前视图，不改变采集数据。'), name="spatialSectionCaption"))
        self.layer_checks = {}
        for title, key in ((tr('追踪器刚体'), "tracker"), (tr('相机刚体'), "camera"), (tr('坐标轴'), "axes"),
                           (tr('参考网格'), "grid"), (tr('运动轨迹'), "trail"), (tr('相机视锥'), "frustum")):
            check = QCheckBox(title)
            check.setChecked(True)
            check.toggled.connect(lambda checked, layer=key: self._layer_changed(layer, checked))
            self.layer_checks[key] = check
            layer_layout.addWidget(check)
        layer_layout.addStretch()

        cloud_layout = inspector_tab(tr('点云'))
        cloud_layout.addWidget(section_title(tr('深度点云')))
        cloud_layout.addWidget(plain_label(tr('使用与图像时刻匹配的有效位姿，显示相机深度数据。'), name="spatialSectionCaption"))
        self.cloud_check = QCheckBox(tr('显示点云'))
        self.cloud_check.setToolTip(tr('需要完整深度几何及与图像时刻匹配的有效位姿'))
        if self._archive_embedded:
            self.cloud_check.setEnabled(False)
            self.cloud_check.setToolTip(tr('兼容空间回放仅显示原始追踪器位姿；尚未重建相机位姿、视锥和点云。'))
        self.cloud_check.toggled.connect(lambda checked: self._layer_changed("cloud", checked))
        cloud_layout.addWidget(self.cloud_check)
        self.cloud_color = QComboBox()
        self.cloud_color.addItem(tr('点云：来源分色'), "source")
        self.cloud_color.addItem(tr('点云：真实 RGB'), "rgb")
        self.cloud_color.currentIndexChanged.connect(self._update_clouds)
        cloud_layout.addWidget(self.cloud_color)
        self.cloud_size = QDoubleSpinBox()
        self.cloud_size.setRange(1., 8.)
        self.cloud_size.setValue(2.)
        self.cloud_size.setPrefix(tr('点大小 '))
        self.cloud_size.setSuffix(" px")
        self.cloud_size.setDecimals(0)
        self.cloud_size.valueChanged.connect(self._update_clouds)
        cloud_layout.addWidget(self.cloud_size)
        self.cloud_count = QComboBox()
        for count in (5000, 15000, 50000):
            self.cloud_count.addItem(tr('每路 ≤ {v0:,} 点', v0=count), count)
        from vt_capture_core.spatial.cloud_protocol import MAX_POINTS
        self.cloud_count.addItem(tr('全量点（显示端不抽样）'), MAX_POINTS)
        self.cloud_count.setCurrentIndex(3)
        self.cloud_count.currentIndexChanged.connect(self._update_clouds)
        cloud_layout.addWidget(self.cloud_count)
        self.cloud_depth = QDoubleSpinBox()
        self.cloud_depth.setRange(0., 100.)
        self.cloud_depth.setDecimals(2)
        self.cloud_depth.setSingleStep(.1)
        self.cloud_depth.setSpecialValueText(tr('通用上限：不额外裁剪'))
        self.cloud_depth.setPrefix(tr('通用上限 Z ≤ '))
        self.cloud_depth.setSuffix(" m")
        self.cloud_depth.setToolTip(tr('仅限制显示的彩色光学坐标 Z，不改变传感器测距范围或采集配置'))
        self.cloud_depth.valueChanged.connect(self._update_clouds)
        cloud_layout.addWidget(self.cloud_depth)
        self._display_settings = presentation_settings.presentation_settings()
        self.wrist_depth_check = QCheckBox(tr('腕部点云：仅显示 7–50 cm'))
        self.wrist_depth_check.setToolTip(tr('按左右腕相机自身的深度方向裁剪预览；头部与原始采集数据保持不变。'))
        self.wrist_depth_check.setChecked(self._display_settings.value('spatial/wrist_depth_7_50_cm', False, type=bool))
        self.wrist_depth_check.toggled.connect(self._wrist_depth_changed)
        cloud_layout.addWidget(self.wrist_depth_check)
        self.cloud_limits = plain_label(tr('后台深度范围：等待点云参数'), name='spatialCloudLimits')
        cloud_layout.addWidget(self.cloud_limits)
        cloud_layout.addStretch()

        detail_layout = inspector_tab(tr('数据'))
        detail_layout.addWidget(section_title(tr('所选实体')))
        self.detail = plain_label(tr('选择实体查看坐标、姿态与数据状态'))
        self.detail.setObjectName("spatialDetail")
        self.detail.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.detail.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        detail_layout.addWidget(self.detail)
        detail_layout.addWidget(divider())
        detail_layout.addWidget(section_title(tr('位置总览 / m')))
        self.positions = QTableWidget(0, 4)
        self.positions.setObjectName("spatialPositions")
        self.positions.setHorizontalHeaderLabels([tr('实体'), "X", "Y", "Z"])
        self.positions.horizontalHeader().setMinimumSectionSize(32)
        self.positions.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.positions.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.positions.verticalHeader().hide()
        self.positions.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.positions.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.positions.setShowGrid(False)
        self.positions.setAlternatingRowColors(True)
        self.positions.verticalHeader().setDefaultSectionSize(30)
        self.positions.setFixedHeight(220)
        self.positions.cellClicked.connect(self._table_selected)
        detail_layout.addWidget(self.positions)
        detail_layout.addStretch()
        workspace.addWidget(self.inspector)
        self.inspector.hide()
        root.addLayout(workspace, 1)

        self.replay_bar = QFrame()
        self.replay_bar.setObjectName("spatialReplayBar")
        replay_tools = QHBoxLayout(self.replay_bar)
        replay_tools.setContentsMargins(10, 8, 10, 8)
        replay_tools.setSpacing(6)
        self.play_button = QPushButton(tr('播放'))
        self.play_button.setObjectName("primary")
        self.play_button.clicked.connect(self._toggle_playback)
        replay_tools.addWidget(self.play_button)
        for title, delta in ((tr('上一帧'), -1), (tr('下一帧'), 1)):
            button = QPushButton(title)
            button.setObjectName("spatialAction")
            button.clicked.connect(lambda _=False, step=delta: self.backend.step(step))
            replay_tools.addWidget(button)
        self.timeline = QSlider(Qt.Orientation.Horizontal)
        self.timeline.sliderReleased.connect(lambda: self.backend.seek(self.timeline.value()))
        replay_tools.addWidget(self.timeline, 1)
        self.frame_label = plain_label("0 / 0")
        replay_tools.addWidget(self.frame_label)
        self.play_rate = QComboBox()
        for rate in (.25, .5, 1., 2., 4.):
            self.play_rate.addItem(tr('{v0:g}×', v0=rate), rate)
        self.play_rate.setCurrentIndex(2)
        self.play_rate.currentIndexChanged.connect(self._rate_changed)
        replay_tools.addWidget(self.play_rate)
        self.replay_bar.hide()
        root.addWidget(self.replay_bar)
        self.hint = plain_label(tr('单指按下拖动旋转 · Shift＋单指滑动平移 · 双指捏合/上下滑动缩放 · 点击选择 · 实体外形为示意'))
        self.hint.setToolTip(tr('光标放在三维区域，按住 Shift 并在触控板上单指滑动即可平移，无需按下触控板。松开 Shift 后，单指按下拖动可旋转。'))
        self.hint.setObjectName("spatialHint")
        self.hint.setVisible(not self._archive_embedded)
        root.addWidget(self.hint)

    def refresh(self, frame=None, status=None):
        # Only the owning page fetches. A detail window receives this same result.
        frame = self.backend.latest() if frame is None else frame
        status = self.backend.status() if status is None else status
        self.apply_frame(frame, status)
        if self.detail_window is not None:
            self.detail_window.page.apply_frame(frame, status)

    def apply_frame(self, frame, status):
        self._last_status = status
        cloud_backend = getattr(self.backend, 'cloud_backend', None)
        if not self.model.frozen:
            enabled = (status.get('state') == 'ready' and 'cloud' in self.model.layers
                       and frame is not None and frame.get('mode') == 'live'
                       and cloud_backend is not None)
            matches = cloud_backend.latest(frame) if enabled else {}
            frame, matches = self._cloud_presentation.select(frame, matches, enabled=enabled)
            self._presentation_clouds = matches if enabled else None
        self.model.update(demo_scene_frame(frame))
        self.model.source_available = status.get("state") == "ready"
        self._render()
        self._sync_interest()

    def _render(self):
        frame = self.model.frame
        notice = frame.get("preview_notice", "") if frame else ""
        notice = notice if isinstance(notice, str) else ""
        # Deployment-level presentation override, including an empty string to
        # hide an informational banner from an already-running older Manager.
        # Keep archived provenance, source frames and live diagnostics intact.
        if frame and frame.get("mode") == "live":
            notice = os.environ.get("VT_LIVE_PREVIEW_NOTICE", notice)
        self.preview_notice.setText(tr('预览提示：{notice}', notice=notice) if notice.strip() else "")
        self.preview_notice.setVisible(bool(notice.strip()))
        mode = frame.get("mode") if frame else getattr(self.backend, "mode", "live")
        prefix = {"demo": tr('合成数据'), "replay": tr('历史帧')}.get(mode, tr('实时监看'))
        if self.model.frozen:
            stamp = frame.get("generated_realtime_ns", 0) if frame else 0
            clock = datetime.datetime.fromtimestamp(stamp/1e9).strftime("%H:%M:%S") if stamp else "—"
            self.status.setText(prefix+tr(' · 已冻结 ')+clock)
        elif self._archive_embedded:
            summary = display_message(self._last_status.get("message", tr('等待数据')))
            if frame and frame.get("world_frame") not in (None, "unknown"):
                summary += " · " + str(frame["world_frame"])
            self.status.setText(summary)
        elif self._last_status.get("state") != "ready":
            self.status.setText(display_message(self._last_status.get("message", tr('等待数据'))))
        else:
            self.status.setText(prefix+" · "+str(frame.get("world_frame", "vive_map") if frame else tr('等待数据')))
            if frame:
                timing_label=(tr(' · 逐路独立匹配（非同时采样）') if frame.get('time_mode')=='independent' else
                              tr(' · 各实体独立时刻') if frame.get('time_mode')=='monitor' else tr(' · 按图像时刻匹配'))
                self.status.setText(self.status.text()+timing_label)
                if frame.get("map_identity") is None:
                    self.status.setText(self.status.text()+tr(' · 地图身份未核验'))
        self.status.setToolTip(display_message(self._last_status.get("message", "")))
        if self._archive_embedded and frame:
            details = [self._last_status.get("message", ""), *frame.get("diagnostics", ())]
            explanation = tr("\n").join(display_message(str(item)) for item in dict.fromkeys(details) if item)
            explanation += "\n" + tr('档案时间轴 / 各话题历史采样')
            if frame.get("map_identity") is None:
                explanation += "\n" + tr('地图身份未核验')
            self.status.setToolTip(explanation)
        self.scene.set_model(self.model)
        entities = self.model.all_entities()
        signature = tuple((e.get("id"), e.get("label"), e.get("state")) for e in entities)
        if signature != self._list_signature:
            self.devices.blockSignals(True)
            self.devices.clear()
            for entity in entities:
                item = QListWidgetItem(entity.get("label", entity["id"])+"\n"+STATE_LABELS.get(entity.get("state"), str(entity.get("state", ""))))
                item.setData(Qt.ItemDataRole.UserRole, entity["id"])
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(Qt.CheckState.Unchecked if entity["id"] in self.model.hidden_entities else Qt.CheckState.Checked)
                self.devices.addItem(item)
            self.devices.blockSignals(False)
            # Each entity keeps a complete label and state; the tab scrolls
            # on short windows without constraining the canvas height.
            self.devices.setFixedHeight(min(340, max(136,
                sum(self.devices.sizeHintForRow(i) for i in range(self.devices.count())) + 4)))
            self._list_signature = signature
        self.positions.setRowCount(len(entities))
        for index, entity in enumerate(entities):
            position = entity.get("position_m")
            short_label = scene_entity_label(entity, short=True)
            values = [short_label] + ([tr('{v0:.2f}', v0=v) for v in position] if position else ["—"]*3)
            for column, value in enumerate(values):
                old = self.positions.item(index, column)
                if old is None:
                    self.positions.setItem(index, column, QTableWidgetItem(value))
                elif old.text() != value:
                    old.setText(value)
        self._render_detail()
        self._update_clouds()
        self._update_camera_strip_visibility()
        if not self._archive_embedded:
            if mode == "replay":
                self.hint.setText(display_message(self._last_status.get("message", tr('回放加载中'))))
            elif frame and not frame.get("cameras"):
                self.hint.setText(tr('追踪器公共坐标 · 参考网格非真实地面。相机需在空间配置中设置时间对齐、外参和设备身份来源。'))
            else:
                self.hint.setText(tr('单指按下拖动旋转 · Shift＋单指滑动平移 · 双指捏合/上下滑动缩放 · 点击选择 · 实体外形为示意 · 参考网格非真实地面'))
        self._refresh_timeline()
        if self.follow.isChecked() and self.model.selected is not None:
            from ...viewmodels.spatial import display_point, drawable
            if drawable(self.model.selected):
                target = display_point(frame, self.model.selected["position_m"])
                viewport = self.scene.viewport
                if self.scene.render_mode == "projection":
                    viewport.target = target
                    viewport.update()
                else:
                    from PySide6.QtGui import QVector3D
                    viewport.opts["center"] = QVector3D(*target)
                    viewport.update()

    def _render_detail(self):
        entity = self.model.selected
        if entity is None:
            self.detail.setText(tr('选择实体查看坐标、姿态与数据状态\n\n三个追踪器保留公共原点。参考网格不代表真实地面。'))
            return
        position = entity.get("position_m")
        pos = ", ".join(tr('{v0:.4f}', v0=v) for v in position) if position else tr('不可用')
        q = entity.get("quaternion_xyzw")
        rotation = ", ".join(tr('{v0:.4f}', v0=v) for v in q) if q else tr('不可用')
        age = entity.get("age_s")
        age_text = tr('{v0:.0f} ms', v0=age*1000) if isinstance(age, (float, int)) else tr('未知')
        camera_detail = ""
        if "role" not in entity:
            registration = entity.get("registration_state", "missing")
            camera_detail = (tr('\n配准：{v0}\n绑定追踪器：{v1}', v0=STATE_LABELS.get(registration, display_message(registration)), v1=tracker_role(entity.get('tracker_role')) or tr('未提供')))
        reason = entity.get("reason") or ""
        reason = STATE_LABELS.get(reason, display_message(reason))
        self.detail.setText(tr('{v0}\n{v1}\n\n位置 XYZ / m\n{v2}\n\n姿态 xyzw\n{v3}\n\n数据年龄：{v4}\n坐标系：{v5}\n身份：{v6}{v7}\n{v8}', v0=entity.get('label', entity['id']), v1=STATE_LABELS.get(entity.get('state'), entity.get('state', '')), v2=pos, v3=rotation, v4=age_text, v5=entity.get('frame_id', '—'), v6=entity.get('tracker_id', entity.get('serial', '—')), v7=camera_detail, v8=reason))

    def _toggle_inspector(self, visible):
        self.inspector.setVisible(visible)

    def _inspect_entity(self, identifier):
        self.select(identifier)
        self.inspector_toggle.setChecked(True)
        self.inspector_tabs.setCurrentIndex(3)

    def _update_camera_strip_visibility(self, *_args):
        if not hasattr(self, "camera_strip"):
            return
        frame = self.model.frame
        available = bool(frame and frame.get("cameras")) and self.model.preset != "tracker" and self.backend.mode != "replay"
        self.camera_toggle.setEnabled(available)
        self.camera_strip.setVisible(available and self.camera_toggle.isChecked() and self.height() >= 480)

    def select(self, identifier):
        self.model.select(identifier)
        for index in range(self.devices.count()):
            item = self.devices.item(index)
            if item.data(Qt.ItemDataRole.UserRole) == identifier:
                self.devices.setCurrentItem(item)
                break
        self.scene.set_model(self.model)
        self._render_detail()

    def _table_selected(self, row, _column):
        entities = self.model.all_entities()
        if row < len(entities):
            self.select(entities[row]["id"])

    def _preset_changed(self):
        self.model.set_preset(self.preset.currentData())
        self._render()
        self._sync_interest()

    def _visibility_changed(self, item):
        identifier = item.data(Qt.ItemDataRole.UserRole)
        if item.checkState() == Qt.CheckState.Unchecked:
            self.model.hidden_entities.add(identifier)
        else:
            self.model.hidden_entities.discard(identifier)
        self.scene.set_model(self.model)

    def _layer_changed(self, layer, checked):
        self.model.layers.add(layer) if checked else self.model.layers.discard(layer)
        self.scene.set_model(self.model)
        self._sync_interest()
        self._update_clouds()

    def _wrist_depth_changed(self, enabled):
        self._display_settings.setValue('spatial/wrist_depth_7_50_cm', enabled)
        self._display_settings.sync()
        self._update_clouds()
        if self.detail_window is not None:
            self.detail_window.page.wrist_depth_check.setChecked(enabled)

    def _update_clouds(self, *_args):
        if not hasattr(self, "scene"):
            return
        backend = getattr(self.backend, "cloud_backend", None)
        if not self.model.frozen:
            self._clouds = ((self._presentation_clouds if self._presentation_clouds is not None
                             else backend.latest(self.model.frame)) if backend is not None
                            and "cloud" in self.model.layers and self.model.source_available else {})
        visible = {key: value for key, value in self._clouds.items() if key not in self.model.hidden_entities}
        if "cloud" not in self.model.layers:
            visible = {}
        visible = self._wrist_cloud_display.select(visible, self.model.frame.get('cameras', ()) if self.model.frame else (),
                                                   self.wrist_depth_check.isChecked())
        self.scene.set_clouds(visible, self.cloud_color.currentData(), self.cloud_size.value(),
                              self.cloud_count.currentData(), self.cloud_depth.value())
        limits = []
        if self.wrist_depth_check.isChecked():
            limits.append(tr('腕部预览范围：7–50 cm；头部保持原范围。'))
        for name, value in sorted(self._clouds.items()):
            source=value['header']['source_realtime_ns']
            now=self.model.frame.get('generated_realtime_ns',source)
            camera=next((c for c in self.model.frame.get('cameras',()) if c['id']==name),{})
            age=max(max(0,now-source)/1e9,camera.get('age_s') or 0.)
            limits.append(tr('{name}：源帧年龄 {age_ms} ms',name=name,age_ms=round(age*1000)))
            sampling = value['header'].get('sampling')
            origin = value['header'].get('origin')
            if origin and origin.get('source') == 'realsense':
                limits.append(f"{name}：RealSense 驱动点云（XYZ＋RGB）；后台保留驱动范围；接收 {value['header']['point_count']:,} 有效点 / 驱动 {origin['input_points']:,} 点记录；无效 {origin['invalid_points']:,}；{origin['topic']}")
            elif sampling:
                density = '全像素扫描' if sampling['stride'] == 1 else f"每 {sampling['stride']}×{sampling['stride']} 像素取一点"
                limits.append(f"{name}：后台深度 {sampling['min_depth_m']:g}–{sampling['max_depth_m']:g} m；{density}；接收 {value['header']['point_count']:,} 点")
            else:
                limits.append(f"{name}：后台未提供范围/抽样参数；接收 {value['header']['point_count']:,} 点")
            cropped = visible.get(name)
            if cropped is not None and cropped.get('display_range_m'):
                limits.append(tr('{name}：7–50 cm 内保留 {count:,} 点（显示端裁剪）', name=name, count=len(cropped['positions'])))
        self.cloud_limits.setText('\n'.join(limits) if limits else tr('后台深度范围：等待点云参数'))
        self.cloud_limits.setToolTip(tr('RealSense 来源直接使用驱动 XYZ/RGB，后台只去除无效点并转换坐标；不套用 RGB-D 重建的距离过滤。显示端若主动设置距离上限，仍以彩色光学 Z 裁剪。RGB-D 来源的后台范围以原始深度光学 Z 计。'))
        self.status.setText(self.status.text().split(tr(' · 点云'))[0].split(tr(' · 未配置相机'))[0])
        if "cloud" in self.model.layers:
            count = len(self.model.frame.get("cameras", ())) if self.model.frame else 0
            note = (tr(' · 点云 {v0}/{v1} 路已匹配', v0=len(self._clouds), v1=count) if self._clouds else
                    tr(' · 未配置相机') if count == 0 else tr(' · 点云未同步或几何未就绪'))
            self.status.setText(self.status.text().split(tr(' · 点云'))[0]+note)
            independent=self.model.frame and self.model.frame.get('time_mode')=='independent'
            if independent and len(self._clouds)>1:
                stamps=[c['header']['source_realtime_ns'] for c in self._clouds.values()]
                self.status.setText(self.status.text()+tr(' · 路间时间差 {skew_ms} ms',skew_ms=round((max(stamps)-min(stamps))/1e6)))
            if self._cloud_presentation.held and not self.model.frozen:
                self.status.setText(self.status.text()+(tr(' · 部分通道暂留上一有效帧') if independent else tr(' · 暂留上一匹配组（等待新帧）')))
            if self._cloud_presentation.stale_cameras and not self.model.frozen:
                self.status.setText(self.status.text()+tr(' · 保留画面非实时（最多3秒）'))
            diagnostics = self.model.frame.get("diagnostics", ()) if self.model.frame else ()
            explanation = (display_message(backend.status()) if backend else tr('当前数据源未提供点云'))+"\n"+tr("\n").join(display_message(str(item))[:300] for item in diagnostics[:8])
            self.cloud_check.setToolTip(explanation)
            self.status.setToolTip(explanation)

    def _freeze(self, frozen):
        self.model.set_frozen(frozen)
        self.freeze_button.setText(tr('恢复实时') if frozen else tr('冻结画面'))
        self._render()
        self._sync_interest()

    def set_motion_enabled(self, enabled):
        """Presentation preference shared by the workbench and detached scene."""
        self._motion_enabled = bool(enabled)
        self.scene.set_motion_enabled(enabled)
        if self.detail_window is not None:
            self.detail_window.page.set_motion_enabled(enabled)

    def _sync_interest(self):
        # Each view owns only its demand, never the shared backend. A frozen
        # view retains its immutable frame but does not require new cloud work.
        consuming = self.isVisible() and not self.window().isMinimized() and not self.model.frozen
        self.backend.set_interest(self.viewer_id, frozenset(self.model.layers) if consuming else frozenset())

    def set_preview_sources(self, sources):
        self._preview_sources = sources
        if not self.model.frozen and self.backend.mode != "replay":
            self._update_previews(sources)
        if self.detail_window is not None:
            self.detail_window.page.set_preview_sources(sources)

    def _update_previews(self, sources):
        for identifier, source in sources.items():
            if identifier not in self._preview_widgets:
                tile = QFrame()
                tile.setObjectName("spatialPanel")
                layout = QVBoxLayout(tile)
                layout.setContentsMargins(10, 8, 10, 10)
                layout.setSpacing(6)
                title = QPushButton(identifier+tr(' · 独立实时预览'))
                title.setObjectName("spatialCameraTitle")
                title.clicked.connect(lambda _=False, key=identifier: self.select(key))
                preview = PreviewWidget("camera")
                preview.setFixedHeight(110)
                preview.setProperty("spatial_camera_id", identifier)
                preview.installEventFilter(self)
                layout.addWidget(title)
                layout.addWidget(preview)
                self.camera_layout.addWidget(tile, 1)
                self._preview_widgets[identifier] = (preview, title)
            preview, title = self._preview_widgets[identifier]
            preview.copy_camera_tracker_state_from(source)
            age = source.device_snapshot.get("age_s")
            title.setText(identifier+tr(' · {v0:.0f} ms', v0=age*1000) if isinstance(age, (float, int)) else identifier+tr(' · 等待数据'))
            title.setToolTip(tr('独立实时预览；未与三维场景按帧配对'))

    def eventFilter(self, watched, event):
        if watched is self._observed_window and event.type() == QEvent.Type.WindowStateChange:
            self._sync_interest()
        if event.type() == QEvent.Type.MouseButtonRelease and watched.property("spatial_camera_id"):
            self.select(watched.property("spatial_camera_id"))
        return super().eventFilter(watched, event)

    def open_detail(self):
        if not self._allow_detail:
            return
        if self.detail_window is None:
            from .spatial_detail import SpatialDetailDialog
            self.detail_window = SpatialDetailDialog(self.backend, self.window())
            self.detail_window.finished.connect(self._detail_closed)
            self.detail_window.page.configure_requested.connect(self.configure_requested)
            self.detail_window.page._live_backend = self._live_backend
            self.detail_window.page.set_motion_enabled(self._motion_enabled)
            self.detail_window.page.apply_frame(self.model.frame, self._last_status)
            self.detail_window.page.set_preview_sources(self._preview_sources)
        self.detail_window.show()
        self.detail_window.raise_()
        self.detail_window.activateWindow()

    def _choose_replay_file(self):
        path, _ = QFileDialog.getOpenFileName(self, tr('打开离线空间回放'), "", tr('场景与对齐文件 (*.json *.jsonl)'))
        if path:
            self.load_replay(path)

    def _choose_replay_directory(self):
        path = QFileDialog.getExistingDirectory(self, tr('打开试次或离线对齐结果目录'))
        if path:
            self.load_replay(path)

    def load_replay(self, path):
        if self._archive_embedded:
            return
        from ...backend.spatial_replay import SpatialReplayBackend
        if self._replay_backend is None:
            self._replay_backend = SpatialReplayBackend()
        self._replay_backend.load(path)
        self._activate_backend(self._replay_backend)

    def _activate_backend(self, backend):
        self.backend.set_interest(self.viewer_id, frozenset())
        self.backend = backend
        self._cloud_presentation.clear()
        self._presentation_clouds = None
        self.freeze_button.setChecked(False)
        self.model.clear_frame()
        self._clouds = {}
        self._wrist_cloud_display.clear()
        self._list_signature = None
        self.scene.reset_view_fit()
        self.cruise_button.setChecked(getattr(backend, "mode", "live") == "demo")
        self.scene.clear_clouds()
        self.replay_bar.setVisible(backend.mode == "replay" and not self._archive_embedded)
        self.live_button.setVisible(self._allow_detail and backend.mode == "replay")
        self.configure_button.setEnabled(backend.mode != "replay" and not self._archive_embedded)
        # A saved scene never displays unrelated current camera images.
        for widget, _title in self._preview_widgets.values():
            widget.clear_frame()
        if self.detail_window is not None:
            self.detail_window.page._activate_backend(backend)
        self.refresh()

    def return_live(self):
        if self._archive_embedded:
            return
        if self._replay_backend is not None:
            self._replay_backend.pause()
        self._activate_backend(self._live_backend)

    def _toggle_playback(self):
        if self.backend.mode == "replay":
            self.backend.pause() if self.backend.timeline()["playing"] else self.backend.play()

    def _rate_changed(self):
        if self.backend.mode == "replay":
            self.backend.set_rate(self.play_rate.currentData())

    def _refresh_timeline(self):
        if self._archive_embedded:
            self.replay_bar.hide()
            return
        if self.backend.mode != "replay":
            return
        state = self.backend.timeline()
        self.replay_bar.show()
        self.timeline.setRange(0, max(0, state["frame_count"]-1))
        if not self.timeline.isSliderDown():
            self.timeline.setValue(state["index"])
        self.frame_label.setText(tr('{v0} / {v1}', v0=state['index']+1 if state['frame_count'] else 0, v1=state['frame_count']))
        self.play_button.setText(tr('暂停') if state["playing"] else tr('播放'))

    def _detail_closed(self, *_args):
        self.detail_window = None

    def showEvent(self, event):
        super().showEvent(event)
        window = self.window()
        if window is not self._observed_window:
            if self._observed_window is not None:
                self._observed_window.removeEventFilter(self)
            self._observed_window = window
            window.installEventFilter(self)
        self._sync_interest()

    def minimumSizeHint(self):
        # Stacked pages must not increase the workbench's established minimum.
        return QSize(530, 300)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_camera_strip_visibility()

    def hideEvent(self, event):
        self.backend.set_interest(self.viewer_id, frozenset())
        super().hideEvent(event)

    def closeEvent(self, event):
        self._wrist_cloud_display.clear()
        self.backend.set_interest(self.viewer_id, frozenset())
        self._clouds = {}
        self._cloud_presentation.clear()
        self._presentation_clouds = None
        self.scene.clear_clouds()
        if self.detail_window is not None:
            self.detail_window.close()
        if self._replay_backend is not None:
            self._replay_backend.close()
        super().closeEvent(event)
