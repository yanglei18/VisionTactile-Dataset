"""Independent serial input embedding the workbench's tactile 3D preview."""
from __future__ import annotations

from PySide6 import QtCore

from ..i18n import LANGUAGES, language, set_language, tr
from ..i18n import widgets as QtWidgets
from ..i18n.messages import message

from .widgets.preview import PreviewWidget
from .widgets.tactile_model_loader import model_loader


class TactileWindow(QtWidgets.QMainWindow):
    """Explicitly connect one tactile serial source; never start a system backend."""

    def __init__(self, source, *, port="", baudrate=115200, port_provider=None,
                 model_settings=None, preview_factory=PreviewWidget):
        super().__init__()
        self.source = source
        if port_provider is None:
            from ..backend.tactile_serial import list_serial_ports
            port_provider = list_serial_ports
        self.port_provider = port_provider
        self.model_settings = model_settings
        self._action_error = ""
        self._latest_frame_available = False
        self._closed = False
        self.setWindowTitle(tr("触觉三维预览 · 右手"))
        self.resize(760, 800)
        self.setMinimumSize(520, 580)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(22, 18, 22, 16)
        layout.setSpacing(12)
        heading = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(tr("右手触觉"))
        title.setObjectName("title")
        heading.addWidget(title)
        heading.addStretch()
        self.language_combo = QtWidgets.QComboBox()
        self.language_combo.setAccessibleName(tr("界面语言"))
        self.language_combo.setToolTip(tr("切换界面语言，不中断串口连接。"))
        for code, name in LANGUAGES:
            self.language_combo.addItem(name, code)
        self.language_combo.setCurrentIndex(self.language_combo.findData(language()))
        self.language_combo.activated.connect(
            lambda index: self._set_language(self.language_combo.itemData(index)))
        heading.addWidget(self.language_combo)
        self.status = QtWidgets.QLabel(tr("等待数据"))
        self.status.setObjectName("topBadge")
        self.status.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        heading.addWidget(self.status)
        layout.addLayout(heading)
        caption = QtWidgets.QLabel(tr("固定手形 · 实时触觉"))
        caption.setObjectName("muted")
        layout.addWidget(caption)
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
        self.port_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.port_combo.setMinimumContentsLength(14)
        self.port_combo.setAccessibleName(tr("触觉串口"))
        self.port_combo.setToolTip(tr("选择触觉板串口，也可直接输入串口名称；点击连接后才会打开。"))
        self.refresh_ports_button = QtWidgets.QPushButton(tr("刷新"))
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        self.baudrate = QtWidgets.QSpinBox()
        self.baudrate.setRange(1, 4_000_000)
        self.baudrate.setValue(baudrate)
        self.baudrate.setAccessibleName(tr("波特率"))
        self.baudrate.setFixedWidth(110)
        self.connection_button = QtWidgets.QPushButton(tr("连接"))
        self.connection_button.setObjectName("primary")
        self.connection_button.clicked.connect(self.toggle_connection)
        ports = QtWidgets.QHBoxLayout()
        ports.addWidget(QtWidgets.QLabel(tr("串口")))
        ports.addWidget(self.port_combo, 1)
        ports.addWidget(self.refresh_ports_button)
        ports.addWidget(QtWidgets.QLabel(tr("波特率")))
        ports.addWidget(self.baudrate)
        ports.addWidget(self.connection_button)
        layout.addLayout(ports)
        self.refresh_ports()
        if port:
            self.port_combo.setEditText(port)
        self.port_combo.currentTextChanged.connect(self._sync_connection_controls)
        self.preview = preview_factory("tactile", central)
        self.preview.set_device(self._device_snapshot())
        self.preview.set_tactile_view("3d")
        layout.addWidget(self.preview, 1)
        self.metrics = QtWidgets.QLabel(tr("接收 — Hz · 数据年龄 —"))
        self.metrics.setObjectName("muted")
        self.metrics.setToolTip(tr("接收频率指本窗口收到的不同源触觉快照，不是设备扫描频率。"))
        layout.addWidget(self.metrics)
        reference_row = QtWidgets.QHBoxLayout()
        self.reference_button = QtWidgets.QPushButton(tr("无载归零"))
        self.reference_button.setToolTip(tr("保持触觉区域不受压，采集 3 秒无载参考。"))
        self.reference_button.clicked.connect(self.start_reference)
        reference_row.addWidget(self.reference_button)
        self.reference_status = QtWidgets.QLabel(tr("尚未归零"))
        self.reference_status.setObjectName("muted")
        self.reference_status.setWordWrap(True)
        reference_row.addWidget(self.reference_status, 1)
        self.reference_progress = QtWidgets.QProgressBar()
        self.reference_progress.setRange(0, 100)
        self.reference_progress.setTextVisible(False)
        self.reference_progress.setFixedWidth(100)
        self.reference_progress.setFixedHeight(8)
        self.reference_progress.setAccessibleName(tr("无载参考采集进度"))
        reference_row.addWidget(self.reference_progress)
        layout.addLayout(reference_row)
        settings_row = QtWidgets.QHBoxLayout()
        self.settings_toggle = QtWidgets.QPushButton(tr("配置设备"))
        self.settings_toggle.setCheckable(True)
        self.settings_toggle.setObjectName("inspectorToggle")
        settings_row.addWidget(self.settings_toggle)
        settings_row.addStretch()
        hint = QtWidgets.QLabel(tr("拖动旋转 · 滚轮缩放"))
        hint.setObjectName("muted")
        settings_row.addWidget(hint)
        layout.addLayout(settings_row)
        self.settings = self._make_settings()
        self.settings.hide()
        self.settings_toggle.toggled.connect(self.settings.setVisible)
        layout.addWidget(self.settings)
        self.notice = QtWidgets.QLabel(tr("选择触觉串口，然后点击连接。"))
        self.notice.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.notice.setObjectName("operationNotice")
        self.notice.setWordWrap(True)
        layout.addWidget(self.notice)
        note = QtWidgets.QLabel(tr("颜色表示相对触觉响应，不是压力；灰色表示无效或缺少参考。"))
        note.setObjectName("muted")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.refresh)
        self._sync_connection_controls()
        self._sync_reference_controls()
        if (model_settings is not None and not model_settings.uses_static_model
                and not (model_settings.glove_python and model_settings.mano_assets)):
            self.settings_toggle.setChecked(True)

    def _set_language(self, code):
        # Translate the existing controls in place. Do not poll the source,
        # recreate the renderer, reset the color range, or reopen the port.
        set_language(code)
        with QtCore.QSignalBlocker(self.language_combo):
            self.language_combo.setCurrentIndex(self.language_combo.findData(language()))
        self.preview.setAccessibleName(tr("右手触觉"))

    @staticmethod
    def _device_snapshot(state=None):
        """Adapt the serial reading to the existing workbench preview contract."""
        frame = state.frame if state is not None else None
        age = state.age_ms if state is not None else None
        return {"id": "right_tactile", "kind": "tactile", "side": "right",
                "label": "右手触觉", "mode": "live",
                "status": "online" if frame is not None else "offline",
                "age_s": age / 1000 if age is not None else None,
                "receive_hz": state.receive_hz if state is not None else 0.,
                "preview": frame if frame is not None else {}}

    def selected_port(self):
        index = self.port_combo.currentIndex()
        if index >= 0 and self.port_combo.currentText() == self.port_combo.itemText(index):
            return self.port_combo.itemData(index) or ""
        return self.port_combo.currentText().strip()

    def refresh_ports(self):
        selected = self.selected_port()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItem(tr("请选择触觉串口"), "")
        try:
            for entry in self.port_provider():
                self.port_combo.addItem(entry["label"], entry["path"])
        except OSError as exc:
            self._action_error = tr("无法枚举串口：{error}", error=message(str(exc)))
        if selected:
            index = self.port_combo.findData(selected)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
            else:
                self.port_combo.setEditText(selected)
        self.port_combo.blockSignals(False)
        self._sync_connection_controls()

    def _sync_connection_controls(self, *_args):
        connecting = self.source.connection_state in ("connecting", "connected")
        stopping = self.source.connection_state == "disconnecting"
        self.connection_button.setText(tr("正在断开…" if stopping else "断开" if connecting else "连接"))
        self.connection_button.setEnabled(self.source.can_disconnect if connecting else
                                          self.source.can_connect and bool(self.selected_port()))
        for control in (self.port_combo, self.refresh_ports_button, self.baudrate):
            control.setEnabled(self.source.can_connect)

    def toggle_connection(self):
        self._action_error = ""
        try:
            if self.source.can_disconnect:
                self.source.disconnect()
            elif self.source.can_connect:
                self.preview.clear_frame()
                self.source.connect(self.selected_port(), self.baudrate.value())
        except (OSError, RuntimeError, ValueError) as exc:
            self._action_error = tr("串口操作失败：{error}", error=message(str(exc)))
        self.refresh()

    def _make_settings(self):
        panel = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(panel)
        grid.setContentsMargins(0, 2, 0, 2)
        explanation = QtWidgets.QLabel(tr("按固定范围显示相对参考的响应；范围越小，同一变化的颜色越明显。"))
        explanation.setObjectName("muted")
        explanation.setWordWrap(True)
        grid.addWidget(explanation, 0, 0, 1, 5)
        response_settings = self.source.response_settings
        self._source_response_settings = (response_settings.display_range_adc, response_settings.direction)
        self.response_range = QtWidgets.QDoubleSpinBox()
        self.response_range.setRange(1., 65535.)
        self.response_range.setDecimals(0)
        self.response_range.setSingleStep(100.)
        self.response_range.setValue(response_settings.display_range_adc)
        self.response_range.setAccessibleName(tr("显示范围（ΔADC）"))
        self.response_range.setToolTip(tr("色阶从 0 到此数值固定显示，不自动放大无载噪声。"))
        self.response_range.editingFinished.connect(self.configure_response)
        grid.addWidget(QtWidgets.QLabel(tr("显示范围（ΔADC）")), 1, 0)
        grid.addWidget(self.response_range, 1, 1, 1, 4)
        self.response_direction = QtWidgets.QComboBox()
        self.response_direction.setAccessibleName(tr("响应方向"))
        for text, direction in ((tr("双向变化"), "both"), (tr("ADC 增大"), "increase"),
                                (tr("ADC 减小"), "decrease")):
            self.response_direction.addItem(text, direction)
        self.response_direction.setCurrentIndex(self.response_direction.findData(response_settings.direction))
        self.response_direction.setToolTip(tr("双向变化显示相对参考的变化幅度；已知硬件响应方向时可选择增大或减小。"))
        self.response_direction.activated.connect(self.configure_response)
        grid.addWidget(QtWidgets.QLabel(tr("响应方向")), 2, 0)
        grid.addWidget(self.response_direction, 2, 1, 1, 4)
        if self.model_settings is not None and self.model_settings.uses_static_model:
            label = QtWidgets.QLabel(tr("已配置固定手模型，无需安装或配置模型运行环境。"))
            label.setObjectName("muted")
            label.setWordWrap(True)
            grid.addWidget(label, 3, 0, 1, 5)
        elif self.model_settings is not None:
            grid.addWidget(QtWidgets.QLabel(tr("右手姿态 · 模型资源")), 3, 0, 1, 5)
            self.model_python = QtWidgets.QLineEdit(self.model_settings.glove_python)
            self.model_assets = QtWidgets.QLineEdit(self.model_settings.mano_assets)
            for row, label, field, directory in (
                (4, tr("手套 Python 环境"), self.model_python, False), (5, tr("MANO 资源目录"), self.model_assets, True)):
                field.setAccessibleName(label)
                grid.addWidget(QtWidgets.QLabel(label), row, 0)
                grid.addWidget(field, row, 1, 1, 3)
                choose = QtWidgets.QPushButton(tr("选择…"))
                choose.clicked.connect(lambda _checked=False, f=field, d=directory: self._choose_model(f, d))
                grid.addWidget(choose, row, 4)
            self.model_status = QtWidgets.QLabel(tr("参数含义与主程序一致，设置仅保存到本工具。"))
            self.model_status.setObjectName("muted")
            self.model_status.setWordWrap(True)
            grid.addWidget(self.model_status, 6, 0, 1, 4)
            save = QtWidgets.QPushButton(tr("保存并加载"))
            save.clicked.connect(self.save_model)
            grid.addWidget(save, 6, 4)
        return panel

    def start_reference(self):
        self._action_error = ""
        try:
            self.source.start_reference()
        except (RuntimeError, ValueError) as exc:
            self._action_error = tr("无法开始归零：{error}", error=message(str(exc)))
        else:
            self.preview.clear_frame()
        self.refresh()

    def configure_response(self, *_args):
        current = self.source.response_settings
        if (self.response_range.value() == current.display_range_adc
                and self.response_direction.currentData() == current.direction):
            return
        self._action_error = ""
        try:
            self.source.configure_response(display_range_adc=self.response_range.value(),
                                           direction=self.response_direction.currentData())
        except (RuntimeError, ValueError) as exc:
            self._action_error = tr("显示设置未应用：{error}", error=message(str(exc)))
            settings = self.source.response_settings
            with QtCore.QSignalBlocker(self.response_range), QtCore.QSignalBlocker(self.response_direction):
                self.response_range.setValue(settings.display_range_adc)
                self.response_direction.setCurrentIndex(self.response_direction.findData(settings.direction))
        else:
            # An old frame cannot be relabelled with a newly chosen scale.
            self.preview.clear_frame()
        self.refresh()

    def _sync_reference_controls(self):
        reference = self.source.response_status
        collecting = reference.phase == "collecting"
        self.reference_button.setText(tr("归零中…" if collecting else "无载归零"))
        self.reference_button.setToolTip(
            tr("可选：保持触觉区域不受压，采集 3 秒新参考。") if reference.phase == "ready"
            else tr("保持触觉区域不受压，采集 3 秒无载参考。"))
        self.reference_button.setEnabled(self.source.connection_state == "connected"
                                         and self._latest_frame_available and not collecting)
        self.reference_progress.setVisible(collecting)
        self.reference_progress.setValue(round(reference.progress * 100))
        self.response_range.setEnabled(not collecting)
        self.response_direction.setEnabled(not collecting)
        self.reference_status.setToolTip("")
        if collecting:
            self.reference_status.setText(tr("保持无载 · {percent}%", percent=round(reference.progress * 100)))
        elif reference.phase == "ready":
            if reference.reference_source == "default":
                self.reference_status.setText(tr("通用参考 {count}/400", count=reference.reference_cells))
                self.reference_status.setToolTip(tr("已载入通用参考，可直接查看触觉响应；如有无载偏差，可选无载归零。"))
            elif reference.reference_source == "preset":
                self.reference_status.setText(tr("默认参考 {count}/400", count=reference.reference_cells))
                self.reference_status.setToolTip(tr("已载入匹配设备的默认参考，可直接查看触觉响应。"))
            elif reference.reference_source == "saved":
                self.reference_status.setText(tr("已保存参考 {count}/400", count=reference.reference_cells))
                self.reference_status.setToolTip(tr("已恢复此设备的已保存参考，可直接查看触觉响应。"))
            else:
                self.reference_status.setText(tr("参考就绪 {count}/400", count=reference.reference_cells))
        elif reference.phase == "failed":
            self.reference_status.setText(tr("归零未完成"))
        else:
            self.reference_status.setText(tr("尚未归零"))

    def _sync_response_settings(self):
        settings = self.source.response_settings
        current = (settings.display_range_adc, settings.direction)
        if current == self._source_response_settings:
            return
        # A device profile may change the source settings on connection. Do not
        # replace an in-progress range edit before editingFinished submits it.
        if self.response_range.hasFocus() and self.response_range.value() != current[0]:
            return
        with QtCore.QSignalBlocker(self.response_range), QtCore.QSignalBlocker(self.response_direction):
            self.response_range.setValue(current[0])
            self.response_direction.setCurrentIndex(self.response_direction.findData(current[1]))
        self._source_response_settings = current

    def _reference_notice(self):
        reference = self.source.response_status
        if reference.phase == "collecting":
            return tr("保持触觉区域不受压，采集 3 秒无载参考。")
        if reference.phase == "unreferenced":
            return tr("请保持触觉区域不受压，然后点击“无载归零”。")
        if reference.phase == "failed":
            messages = {
                "insufficient_samples": tr("有效采样不足，请保持无载后重试。"),
                "stale_reference": tr("采集中断，请恢复数据后重新归零。"),
                "stream_changed": tr("数据流已切换，请重新归零。"),
                "clock_backwards": tr("数据时间异常，请重新归零。"),
                "invalid_sample": tr("采样无效，请重新归零。"),
            }
            return messages.get(reference.reason, tr("参考无效，请保持无载后重新归零。"))
        return ""

    def _choose_model(self, field, directory):
        if directory:
            path = QtWidgets.QFileDialog.getExistingDirectory(self, tr("选择可信 MANO 资源目录"), field.text())
        else:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(self, tr("选择手套 Python 环境"), field.text())
        if path:
            field.setText(path)

    def save_model(self):
        try:
            self.model_settings.save_model(self.model_python.text(), self.model_assets.text())
            self.model_status.setText(tr("模型设置已保存，正在加载。"))
            model_loader().request("right")
        except (OSError, ValueError) as exc:
            self.model_status.setText(message(str(exc)))

    def start(self):
        self.source.start()
        self.timer.start()
        self.refresh()

    def refresh(self):
        if self._closed:
            return
        state = self.source.poll()
        self._latest_frame_available = state.frame is not None
        self._sync_connection_controls()
        self._sync_response_settings()
        self._sync_reference_controls()
        self.preview.set_device(self._device_snapshot(state))
        self.preview.setAccessibleName(tr("右手触觉"))
        self.status.setText(tr(state.status))
        age = "—" if state.age_ms is None else f"{max(0, state.age_ms):.0f} ms"
        self.metrics.setText(tr("接收 {rate:.1f} Hz    ·    数据年龄 {age}", rate=state.receive_hz, age=age))
        detail = message(state.detail)
        if self.preview.held:
            detail = (tr("{detail} 历史画面已保持。", detail=detail) if state.status in
                      ("连接失败", "未连接", "正在连接", "正在断开") else
                      tr("当前数据不可用，保留上一有效画面（历史保持）。"))
        elif state.frame is not None:
            detail = self._reference_notice() or detail
        persistence_error = message(self.source.reference_persistence_error)
        self.notice.setText(self._action_error or persistence_error or detail)
        self.notice.setToolTip(getattr(self.source, "diagnostic_log", ""))
        self.notice.setVisible(bool(self._action_error or persistence_error) or state.frame is None or self.preview.held
                               or bool(self._reference_notice()))

    def closeEvent(self, event):
        if self.shutdown():
            super().closeEvent(event)
        else:
            event.ignore()

    def shutdown(self):
        if self._closed:
            return True
        if self.source.close() is False:
            self._action_error = tr("正在释放串口，请稍候再关闭窗口。")
            self.refresh()
            return False
        self._closed = True
        self.timer.stop()
        self.preview.close()
        self.preview.deleteLater()
        return True
