"""Embedded calibration presentation; all acquisition and solving belongs to manager."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from vt_capture_core.calibration_policy import board_geometry_defaults

from ...backend.calibration_client import CalibrationClient
from ...i18n import tr
from ...i18n.messages import message as diagnostic
from ...i18n.widgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
                            QFrame, QHBoxLayout, QLabel, QLineEdit, QListWidget, QProgressBar,
                            QPushButton, QScrollArea, QSizePolicy, QSpinBox, QTextEdit,
                            QToolButton, QVBoxLayout, QWidget)
from ...viewmodels.calibration import calibration_view, preview_device, restorable_revision, tracker_role_label, can_resume
from .preview import PreviewWidget


def _label(text="", name="operationHint"):
    label = QLabel(text)
    label.setObjectName(name)
    label.setWordWrap(True)
    label.setTextFormat(Qt.TextFormat.PlainText)
    label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    return label


def _toggle(text, content):
    button = QToolButton()
    button.setText(text)
    button.setCheckable(True)
    button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
    button.setArrowType(Qt.ArrowType.RightArrow)
    content.hide()
    def changed(checked):
        content.setVisible(checked)
        button.setArrowType(Qt.ArrowType.DownArrow if checked else Qt.ArrowType.RightArrow)
    button.toggled.connect(changed)
    return button


class CalibrationWorkspace(QFrame):
    """Refresh from the main window's snapshots without an extra polling timer."""

    back_requested = Signal()

    def __init__(self, backend, parent=None):
        super().__init__(parent)
        self.setObjectName("operationPanel")
        self.client = CalibrationClient(backend)
        self._snapshot = {}
        self._new_session = False
        self._choices = None
        self._resume_state = None
        self._retry_choices = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(12)
        heading = QHBoxLayout()
        heading.addWidget(_label(tr('相机–追踪器标定'), "operationTitle"), 1)
        self.back_button = QPushButton(tr('返回设备'))
        self.back_button.clicked.connect(self.back_requested)
        heading.addWidget(self.back_button)
        layout.addLayout(heading)
        self.stage_label = _label(tr('准备 → 引导采集 → 结果'), "operationStep")
        layout.addWidget(self.stage_label)

        body = QHBoxLayout()
        body.setSpacing(20)
        self.preview = PreviewWidget("camera", self)
        self.preview.set_presentation("stage")
        body.addWidget(self.preview, 3)
        self.sidebar = QWidget()
        side = QVBoxLayout(self.sidebar)
        side.setContentsMargins(0, 0, 0, 0)
        side.setSpacing(12)
        self.setup = QWidget()
        form = QVBoxLayout(self.setup)
        form.setContentsMargins(0, 0, 0, 0)
        form.setSpacing(10)
        self.mode = QComboBox()
        self.mode.addItem(tr('引导标定'), "guided")
        self.mode.addItem(tr('离线标定'), "offline")
        self.mode.setAccessibleName(tr('标定模式'))
        form.addWidget(self.mode)
        devices = QFormLayout()
        self.camera, self.tracker = QComboBox(), QComboBox()
        self.camera.setAccessibleName(tr('相机'))
        self.tracker.setAccessibleName(tr('追踪器角色'))
        devices.addRow(tr('相机'), self.camera)
        devices.addRow(tr('追踪器角色'), self.tracker)
        form.addLayout(devices)
        self.board_summary = _label()
        form.addWidget(self.board_summary)
        self.board_panel = QWidget()
        board_form = QFormLayout(self.board_panel)
        board_form.setContentsMargins(0, 0, 0, 0)
        board_defaults = board_geometry_defaults()
        self.squares_x, self.squares_y = QSpinBox(), QSpinBox()
        for field, default in ((self.squares_x, board_defaults["squares_x"]),
                               (self.squares_y, board_defaults["squares_y"])):
            field.setRange(3, 30)
            field.setValue(default)
        self.square_length, self.marker_length = QDoubleSpinBox(), QDoubleSpinBox()
        for field, default in ((self.square_length, board_defaults["square_length_mm"]),
                               (self.marker_length, board_defaults["marker_length_mm"])):
            field.setRange(.1, 1000.)
            field.setDecimals(2)
            field.setSuffix(tr(' 毫米'))
            field.setValue(default)
        self.dictionary = QComboBox()
        for size, count in ((5, 1000), (4, 50), (4, 100), (4, 1000),
                            (5, 50), (5, 100), (6, 250), (6, 1000)):
            self.dictionary.addItem(tr('{v0} × {v0} · {v1} 个标记', v0=size, v1=count),
                                    f"DICT_{size}X{size}_{count}")
        self.dictionary.setCurrentIndex(self.dictionary.findData(board_defaults["dictionary"]))
        for title, field in ((tr('横向格数'), self.squares_x), (tr('纵向格数'), self.squares_y),
                             (tr('实测方格边长'), self.square_length), (tr('实测标记边长'), self.marker_length),
                             (tr('标记字典'), self.dictionary)):
            field.setAccessibleName(title)
            board_form.addRow(title, field)
        self.config_path = QLineEdit()
        self.config_path.setReadOnly(True)
        self.config_path.setPlaceholderText(tr('可选：导入已有标定配置'))
        self.config_path.setAccessibleName(tr('标定配置文件'))
        config_row = QHBoxLayout()
        config_row.addWidget(self.config_path, 1)
        choose_config = QPushButton(tr('选择'))
        choose_config.clicked.connect(self._choose_config)
        config_row.addWidget(choose_config)
        clear_config = QPushButton(tr('清除'))
        clear_config.clicked.connect(self.config_path.clear)
        config_row.addWidget(clear_config)
        board_form.addRow(config_row)
        self.board_toggle = _toggle(tr('标定板与配置'), self.board_panel)
        form.addWidget(self.board_toggle)
        form.addWidget(self.board_panel)
        self.motion_description = _label(tr('固定标定板，移动相机与追踪器整体。'))
        form.addWidget(self.motion_description)
        self.assembly_confirmed = QCheckBox(tr('装配与实测尺寸已确认'))
        self.assembly_confirmed.setToolTip(tr('已固定标定板、刚性连接相机与追踪器，并核对实测板尺寸'))
        self.assembly_confirmed.setAccessibleName(tr('确认标定装配与尺寸'))
        form.addWidget(self.assembly_confirmed)
        self.bag_panel = QWidget()
        bag_layout = QVBoxLayout(self.bag_panel)
        bag_layout.setContentsMargins(0, 0, 0, 0)
        bag_layout.addWidget(_label(tr('选择 1 个包检查候选；3 个独立录制包通过验证后自动应用。')))
        self.bags = QListWidget()
        self.bags.setMaximumHeight(120)
        self.bags.setAccessibleName(tr('离线标定数据包'))
        bag_layout.addWidget(self.bags)
        bag_actions = QHBoxLayout()
        self.add_bag = QPushButton(tr('添加数据包'))
        self.add_bag.clicked.connect(self._choose_bag)
        bag_actions.addWidget(self.add_bag)
        remove_bag = QPushButton(tr('移除所选'))
        remove_bag.clicked.connect(self._remove_bag)
        bag_actions.addWidget(remove_bag)
        bag_layout.addLayout(bag_actions)
        form.addWidget(self.bag_panel)
        side.addWidget(self.setup)
        self.instruction = _label("", "operationTitle")
        self.instruction.setAccessibleName(tr('当前标定提示'))
        side.addWidget(self.instruction)
        self.device_summary = _label()
        side.addWidget(self.device_summary)
        self.quality_summary = _label()
        side.addWidget(self.quality_summary)
        self.saved_rounds = _label()
        side.addWidget(self.saved_rounds)
        self.progress_label = _label()
        side.addWidget(self.progress_label)
        self.progress = QProgressBar()
        self.progress.setObjectName("recordingProgress")
        self.progress.setAccessibleName(tr('标定有效姿态进度'))
        self.progress.setTextVisible(False)
        side.addWidget(self.progress)
        self.notice = _label()
        side.addWidget(self.notice)
        self.details = QTextEdit()
        self.details.setReadOnly(True)
        self.details.setMaximumHeight(180)
        self.details.setAccessibleName(tr('标定详细信息'))
        self.details_panel = QWidget()
        details_layout = QVBoxLayout(self.details_panel)
        details_layout.setContentsMargins(0, 0, 0, 0)
        details_layout.addWidget(self.details)
        self.restore_button = QPushButton(tr('恢复上一版本'))
        self.restore_button.clicked.connect(self._restore)
        details_layout.addWidget(self.restore_button)
        self.details_toggle = _toggle(tr('查看详细信息'), self.details_panel)
        side.addWidget(self.details_toggle)
        side.addWidget(self.details_panel)
        side.addStretch()
        self.resume_confirmed = QCheckBox(tr('相机与追踪器的固定关系、标定板尺寸未改变'))
        self.resume_confirmed.toggled.connect(self._render)
        side.addWidget(self.resume_confirmed)
        self.retry_round = QComboBox()
        self.retry_round.setAccessibleName(tr('选择重拍轮次'))
        self.retry_round.currentIndexChanged.connect(self._render)
        side.addWidget(self.retry_round)
        self.resume_button = QPushButton(tr('保留已完成轮次，继续标定'))
        self.resume_button.clicked.connect(lambda: self.client.resume(
            assembly_unchanged=self.resume_confirmed.isChecked(),
            retry_rounds=[self.retry_round.currentData()] if self.retry_round.currentData() else None))
        side.addWidget(self.resume_button)
        actions = QHBoxLayout()
        self.primary_button = QPushButton(tr('开始标定'))
        self.primary_button.setObjectName("primary")
        self.primary_button.setMinimumHeight(42)
        self.primary_button.clicked.connect(self._primary)
        actions.addWidget(self.primary_button, 1)
        self.cancel_button = QPushButton(tr('取消'))
        self.cancel_button.clicked.connect(self._cancel)
        actions.addWidget(self.cancel_button)
        side.addLayout(actions)
        scroll = QScrollArea()
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(340)
        scroll.setWidget(self.sidebar)
        body.addWidget(scroll, 2)
        layout.addLayout(body, 1)
        for field in (self.squares_x, self.squares_y, self.square_length, self.marker_length):
            field.valueChanged.connect(self._render)
        self.mode.currentIndexChanged.connect(self._render)
        self.config_path.textChanged.connect(self._render)
        self.camera.currentIndexChanged.connect(self._render)
        self.tracker.currentIndexChanged.connect(self._render)
        self.assembly_confirmed.toggled.connect(self._render)
        self._render()

    def _choose_config(self):
        path, _ = QFileDialog.getOpenFileName(self, tr('选择标定配置'), "", tr('标定配置 (*.yaml *.yml)'))
        if path:
            self.config_path.setText(path)

    def _choose_bag(self):
        path = QFileDialog.getExistingDirectory(self, tr('选择数据包目录'))
        existing = [self.bags.item(i).text() for i in range(self.bags.count())]
        if path and path not in existing and len(existing) < 3:
            self.bags.addItem(path)
        self._render()

    def _remove_bag(self):
        self.bags.takeItem(self.bags.currentRow())
        self._render()

    def _primary(self):
        view = calibration_view(self._snapshot)
        if view.can_retry_publish:
            self.client.retry_publish()
        elif view.stage == "result" and not self._new_session:
            self._new_session = True
            self.assembly_confirmed.setChecked(False)
        else:
            # Imported geometry is read by the backend; its inactive UI draft
            # must not prevent submission of a valid imported configuration.
            params = dict(mode=self.mode.currentData(), camera_name=self.camera.currentData(),
                          tracker_role=self.tracker.currentData(), config_path=self.config_path.text(),
                          board=(board_geometry_defaults() if self.config_path.text() else
                                 dict(squares_x=self.squares_x.value(), squares_y=self.squares_y.value(),
                                      square_length_mm=self.square_length.value(),
                                      marker_length_mm=self.marker_length.value(),
                                      dictionary=self.dictionary.currentData())),
                          bag_paths=([self.bags.item(i).text() for i in range(self.bags.count())]
                                     if self.mode.currentData() == "offline" else []))
            self.client.start(params)
        self._render()

    def _cancel(self):
        self.client.cancel()
        self._render()

    def _restore(self):
        self.client.restore_previous()
        self._render()

    def refresh(self, snapshot):
        self._snapshot = snapshot
        self.client.observe(snapshot)
        service = snapshot.get("calibration") or {}
        cameras = tuple((entry["name"], entry.get("model", "")) for entry in service.get("cameras", ()))
        trackers = tuple(entry["role"] for entry in service.get("trackers", ()))
        choices = cameras, trackers
        if choices != self._choices:
            self._choices = choices
            for combo, values in ((self.camera, [(name, name) for name, _ in cameras]),
                                  (self.tracker, [(tracker_role_label(role), role) for role in trackers])):
                selected = combo.currentData()
                combo.blockSignals(True)
                combo.clear()
                for label, value in values:
                    combo.addItem(label, value)
                index = combo.findData(selected)
                if index >= 0:
                    combo.setCurrentIndex(index)
                combo.blockSignals(False)
        if service.get("busy"):
            self._new_session = False
        self.preview.set_device(preview_device(service.get("session") or {}, connected=calibration_view(snapshot).available))
        self._render()

    def _render(self, *_):
        view = calibration_view(self._snapshot)
        session = (self._snapshot.get("calibration") or {}).get("session") or {}
        resume_state = (session.get('session_id'), session.get('state'))
        if resume_state != self._resume_state:
            self._resume_state = resume_state
            self.resume_confirmed.setChecked(False)
        preparing = view.stage == "prepare" or self._new_session
        guided = self.mode.currentData() == "guided"
        self.setup.setVisible(preparing)
        self.setup.setEnabled(view.available and not self.client.pending)
        self.bag_panel.setVisible(not guided)
        self.motion_description.setVisible(guided)
        self.assembly_confirmed.setVisible(guided)
        self.preview.setVisible(guided and (preparing or view.stage == "collect"))
        self.instruction.setText(tr('选择设备并确认标定板参数') if preparing and view.available else view.instruction)
        self.device_summary.setText(tr('{v0} · {v1}追踪器', v0=session.get("camera_name", ""),
                                       v1=tracker_role_label(session.get("tracker_role", ""))))
        self.device_summary.setVisible(not preparing and bool(session.get("camera_name")))
        self.quality_summary.setText(view.quality_summary)
        self.quality_summary.setVisible(not preparing and bool(view.quality_summary))
        self.progress.setRange(0, view.target)
        self.progress.setValue(min(view.accepted, view.target))
        self.progress.setVisible(view.stage == "collect" and not preparing)
        self.progress_label.setVisible(view.stage == "collect" and not preparing)
        self.progress_label.setText(tr('第 {v0}/{v1} 轮 · 有效姿态 {v2}/{v3}', v0=view.round, v1=view.rounds,
                                       v2=view.accepted, v3=view.target))
        self.board_summary.setText(tr('标定板 {v0} × {v1} · 方格 {v2} 毫米 · 标记 {v3} 毫米',
                                      v0=self.squares_x.value(), v1=self.squares_y.value(),
                                      v2=self.square_length.value(), v3=self.marker_length.value())
                                   if not self.config_path.text() else tr('使用导入配置中的标定板参数'))
        valid_board = bool(self.config_path.text()) or self.marker_length.value() < self.square_length.value()
        for field in (self.squares_x, self.squares_y, self.square_length, self.marker_length, self.dictionary):
            field.setEnabled(not self.config_path.text())
        valid_input = bool(self.camera.currentData() and self.tracker.currentData() and valid_board
                           and (self.assembly_confirmed.isChecked() if guided else self.bags.count() in {1, 3}))
        primary_enabled = view.can_retry_publish or (view.can_start and (valid_input if preparing else True))
        self.primary_button.setEnabled(primary_enabled and not self.client.pending)
        self.primary_button.setText(tr('重试发布') if view.can_retry_publish else
                                    tr('开始标定') if preparing else tr('新建标定') if view.stage == "result" else
                                    tr('正在标定'))
        self.cancel_button.setEnabled(view.can_cancel and not self.client.pending)
        self.cancel_button.setVisible(view.busy)
        self.add_bag.setEnabled(self.bags.count() < 3)
        notice = self.client.notice or (tr('标记边长必须小于方格边长') if preparing and not valid_board else "")
        self.notice.setText(notice)
        self.notice.setVisible(bool(notice))
        metrics = session.get("metrics") or {}
        completed = metrics.get('completed_rounds', [])
        self.saved_rounds.setText(tr('已保存并通过单轮验证：第 {v0} 轮；原始数据和结果始终保留。',
                                    v0=', '.join(map(str, completed))) if completed else '')
        self.saved_rounds.setVisible(bool(completed) and not preparing)
        resume = can_resume(self._snapshot) and not preparing
        choices = (session.get('session_id'), tuple(sorted(completed)))
        if choices != self._retry_choices:
            previous = self.retry_round.currentData() if self._retry_choices and self._retry_choices[0] == choices[0] else 0
            self._retry_choices = choices
            self.retry_round.blockSignals(True)
            self.retry_round.clear()
            self.retry_round.addItem(tr('不重拍：继续未完成轮次或重新比较'), 0)
            for number in sorted(completed):
                self.retry_round.addItem(tr('仅重拍第 {v0} 轮，保留其余轮次', v0=number), number)
            index = self.retry_round.findData(previous)
            self.retry_round.setCurrentIndex(max(0, index))
            self.retry_round.blockSignals(False)
        selected_round = self.retry_round.currentData()
        self.retry_round.setVisible(resume and bool(completed))
        self.retry_round.setEnabled(not self.client.pending)
        self.resume_button.setText(
            tr('仅重拍第 {v0} 轮，保留其余轮次', v0=selected_round) if selected_round else
            tr('重新比较三轮结果（不采集）') if len(completed) == 3 else
            tr('保留已完成轮次，继续标定'))
        self.resume_confirmed.setVisible(resume)
        self.resume_button.setVisible(resume)
        self.resume_button.setEnabled(resume and self.resume_confirmed.isChecked() and not self.client.pending)
        details = []
        if session.get("detail"):
            details.append(str(diagnostic(session["detail"])))
        if metrics:
            details.append(str(tr('求解姿态：{v0} · 独立验证姿态：{v1}',
                                  v0=metrics.get("training_count", 0), v1=metrics.get("validation_count", 0))))
        result = {**(session.get("result") or {})}
        if view.active_revision:
            result.update(active_revision=view.active_revision, topic=view.topic)
        for name, title in (("active_revision", tr('活动版本')), ("topic", tr('发布主题')),
                            ("extrinsics_path", tr('外参结果')), ("report_path", tr('验证报告')),
                            ("repeatability_path", tr('重复性报告')), ("manifest_path", tr('会话记录'))):
            if result.get(name):
                details.append(str(title) + ": " + str(result[name]))
        restore = restorable_revision(self._snapshot)
        if restore:
            details.append(str(tr('当前版本：{v0}\n恢复版本：{v1}', v0=restore[2], v1=restore[1])))
        text = "\n\n".join(details)
        if self.details.toPlainText() != text:
            self.details.setPlainText(text)
        self.restore_button.setVisible(restore is not None)
        self.restore_button.setEnabled(restore is not None and not self.client.pending)
        has_details = bool(details) or restore is not None
        self.details_toggle.setVisible(has_details and not preparing)
        self.details.setVisible(bool(details))
        self.details_panel.setVisible(has_details and not preparing and self.details_toggle.isChecked())
