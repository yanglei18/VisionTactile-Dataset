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
        note = QtWidgets.QLabel(tr("颜色表示原始 ADC，不是压力；灰色为无效采样，3D 未映射部位保留模型底色。"))
        note.setObjectName("muted")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.refresh)
        self._sync_connection_controls()
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
        explanation = QtWidgets.QLabel(tr("与主采集预览相同：首次有效数据确定色标范围，此后保持固定。"))
        explanation.setObjectName("muted")
        explanation.setWordWrap(True)
        grid.addWidget(explanation, 0, 0, 1, 5)
        if self.model_settings is not None and self.model_settings.uses_static_model:
            label = QtWidgets.QLabel(tr("已配置固定手模型，无需安装或配置模型运行环境。"))
            label.setObjectName("muted")
            label.setWordWrap(True)
            grid.addWidget(label, 1, 0, 1, 5)
        elif self.model_settings is not None:
            grid.addWidget(QtWidgets.QLabel(tr("右手姿态 · 模型资源")), 1, 0, 1, 5)
            self.model_python = QtWidgets.QLineEdit(self.model_settings.glove_python)
            self.model_assets = QtWidgets.QLineEdit(self.model_settings.mano_assets)
            for row, label, field, directory in (
                (2, tr("手套 Python 环境"), self.model_python, False), (3, tr("MANO 资源目录"), self.model_assets, True)):
                field.setAccessibleName(label)
                grid.addWidget(QtWidgets.QLabel(label), row, 0)
                grid.addWidget(field, row, 1, 1, 3)
                choose = QtWidgets.QPushButton(tr("选择…"))
                choose.clicked.connect(lambda _checked=False, f=field, d=directory: self._choose_model(f, d))
                grid.addWidget(choose, row, 4)
            self.model_status = QtWidgets.QLabel(tr("参数含义与主程序一致，设置仅保存到本工具。"))
            self.model_status.setObjectName("muted")
            self.model_status.setWordWrap(True)
            grid.addWidget(self.model_status, 4, 0, 1, 4)
            save = QtWidgets.QPushButton(tr("保存并加载"))
            save.clicked.connect(self.save_model)
            grid.addWidget(save, 4, 4)
        return panel

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
        self._sync_connection_controls()
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
        self.notice.setText(self._action_error or detail)
        self.notice.setToolTip(getattr(self.source, "diagnostic_log", ""))
        self.notice.setVisible(bool(self._action_error) or state.frame is None or self.preview.held)

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
