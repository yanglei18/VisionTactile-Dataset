"""Graphical launch-profile editing, with all persistence owned by the runtime.

The dialog only enumerates serial names. It never opens a serial device, starts
a sensor, edits YAML, or imports ROS. Cancel discards the copied form state.
"""
from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path
import stat

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit, QPushButton,
    QScrollArea, QSizePolicy, QTabWidget, QVBoxLayout, QWidget,
)

from ...backend.workstation_settings import validate_configuration


GROUPS = (
    ("cameras", "相机与录制控制器", "cameras", ""),
    ("trackers", "Tracker", "trackers", ""),
    ("right_glove", "右手姿态 · IMU", "glove", "right"),
    ("right_tactile", "右手触觉", "tactile", "right"),
    ("left_glove", "左手姿态 · IMU", "glove", "left"),
    ("left_tactile", "左手触觉", "tactile", "left"),
)
FIELDS = {
    "cameras": (("config_path", "相机配置", "file"), ("output_root", "数据输出目录", "directory")),
    "trackers": (("bundle_path", "Tracker 启动 bundle", "file"), ("role_map_path", "角色映射", "file"),
                 ("pyvut_source", "扩展 PyVUT 源码目录", "directory"),
                 ("pyvut_site_packages", "PyVUT 依赖 site-packages", "directory")),
    "glove": (("glove_python", "手套 Python 环境", "file"), ("fsglove_source", "官方 FSGlove 源码", "directory"),
              ("mano_assets", "MANO 资源目录", "directory"), ("port", "IMU 串口", "serial"),
              ("config", "手套配置（可选）", "file"), ("output_rate_hz", "姿态输出上限 Hz", "rate")),
    "tactile": (("port", "触觉串口", "serial"), ("config", "触觉配置（可选）", "file")),
}
BUSY_STATES = {"starting", "applying", "saving", "restarting", "stopping"}


def _label(text="", wrap=True):
    widget = QLabel(text)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setWordWrap(wrap)
    widget.setObjectName("muted")
    return widget


def runtime_busy(runtime):
    return str((runtime.status or {}).get("state", "")).lower() in BUSY_STATES


class WorkstationSettingsDialog(QDialog):
    """Edit six known profiles while preserving every unrelated config field."""

    def __init__(self, runtime, snapshot_provider, parent=None):
        super().__init__(parent)
        self.runtime = runtime
        self.snapshot_provider = snapshot_provider
        self.original = deepcopy(runtime.configuration)
        self.fields = {}
        self.enabled_checks = {}
        self._indices = {}
        self._applying = False
        self.setWindowTitle("配置设备 · 本地工作站")
        self.setModal(False)
        self.resize(900, 650)
        self.setMinimumSize(660, 470)
        self._ensure_profiles()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        layout.setSpacing(10)
        layout.addWidget(_label("只启用本次需要的模块。保存将交给后台异步应用；设备仍需在设备页显式启动，标定保持独立。"))
        layout.addWidget(_label("串口列表仅枚举名称，不会连接。左右手 IMU 与触觉必须选择各自端口；停用模块不会清空其配置。"))
        self.tabs = QTabWidget()
        for title, identifiers in (("相机", ("cameras",)), ("Tracker", ("trackers",)),
                                   ("右手", ("right_glove", "right_tactile")),
                                   ("左手", ("left_glove", "left_tactile"))):
            page = QWidget()
            body = QVBoxLayout(page)
            body.setContentsMargins(12, 12, 12, 12)
            for identifier in identifiers:
                body.addWidget(self._group(identifier))
            body.addStretch()
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QScrollArea.Shape.NoFrame)
            scroll.setWidget(page)
            self.tabs.addTab(scroll, title)
        layout.addWidget(self.tabs, 1)
        self.status_label = _label()
        self.status_label.setMaximumHeight(64)
        self.status_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.status_label)
        actions = QHBoxLayout()
        self.refresh_ports_button = QPushButton("刷新串口列表")
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        actions.addWidget(self.refresh_ports_button)
        actions.addStretch()
        self.cancel_button = QPushButton("取消")
        self.cancel_button.clicked.connect(self.reject)
        actions.addWidget(self.cancel_button)
        self.apply_button = QPushButton("保存并应用")
        self.apply_button.setObjectName("primary")
        self.apply_button.clicked.connect(self.apply)
        actions.addWidget(self.apply_button)
        layout.addLayout(actions)
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start()
        self.refresh_ports()
        self.refresh_status()

    def _ensure_profiles(self):
        profiles = self.original.setdefault("process_groups", [])
        for identifier, title, preset, hand in GROUPS:
            index = next((i for i, group in enumerate(profiles)
                          if group.get("preset") == preset and
                          (not hand or group.get("parameters", {}).get("hand") == hand)), None)
            if index is None:
                parameters = {"hand": hand} if hand else {}
                if preset == "glove":
                    parameters.update(launch_gui="true")
                profiles.append(dict(id=identifier, label=title, preset=preset, enabled=False,
                                     parameters=parameters))
                index = len(profiles) - 1
            self._indices[identifier] = index

    def _group(self, identifier):
        group = self.original["process_groups"][self._indices[identifier]]
        title = next(item[1] for item in GROUPS if item[0] == identifier)
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        enabled = QCheckBox("启用此模块")
        enabled.setChecked(group.get("enabled", False))
        self.enabled_checks[identifier] = enabled
        layout.addWidget(enabled)
        form_widget = QWidget()
        form = QFormLayout(form_widget)
        form.setContentsMargins(0, 4, 0, 4)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        for key, title, kind in FIELDS[group["preset"]]:
            value = group.get("parameters", {}).get(key, "")
            if kind == "rate":
                editor = QDoubleSpinBox()
                editor.setRange(1, 1000)
                editor.setDecimals(1)
                editor.setValue(float(value) if value else 120.0)
                editor.setSuffix(" Hz")
                editor.setToolTip("输出频率上限，不代表实测有效频率。")
                form.addRow(title, editor)
            elif kind == "serial":
                editor = QComboBox()
                editor.setEditable(True)
                editor.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
                editor.lineEdit().setPlaceholderText("选择或填写绝对路径，不会连接")
                editor.setEditText(value)
                form.addRow(title, editor)
            else:
                editor = QLineEdit(value)
                editor.setPlaceholderText("选择本机路径" if "可选" not in title else "留空使用现有默认配置")
                editor.setMinimumWidth(0)
                row = QHBoxLayout()
                row.addWidget(editor, 1)
                browse = QPushButton("选择…")
                browse.clicked.connect(lambda checked=False, editor=editor, kind=kind, title=title:
                                       self._browse(editor, kind, title))
                row.addWidget(browse)
                form.addRow(title, row)
            editor.setAccessibleName(f"{box.title()}：{title}")
            self.fields[(identifier, key)] = editor
        enabled.toggled.connect(form_widget.setEnabled)
        form_widget.setEnabled(enabled.isChecked())
        layout.addWidget(form_widget)
        return box

    @staticmethod
    def _browse(editor, kind, title):
        if kind == "directory":
            value = QFileDialog.getExistingDirectory(editor, title, editor.text())
        else:
            value, _ = QFileDialog.getOpenFileName(editor, title, editor.text(), "所有文件 (*)")
        if value:
            editor.setText(value)

    @staticmethod
    def _value(editor):
        if isinstance(editor, QDoubleSpinBox):
            return f"{editor.value():g}"
        if isinstance(editor, QComboBox):
            if editor.currentIndex() >= 0 and editor.currentText() == editor.itemText(editor.currentIndex()):
                return editor.currentData() or ""
            return editor.currentText().strip()
        return editor.text().strip()

    def refresh_ports(self):
        try:
            ports = self.runtime.serial_ports()
        except (OSError, RuntimeError, ValueError) as exc:
            self._set_status(f"串口名称暂不可枚举，可手动填写路径：{exc}")
            return
        for editor in self.fields.values():
            if not isinstance(editor, QComboBox):
                continue
            value = self._value(editor)
            editor.blockSignals(True)
            editor.clear()
            editor.addItem("未选择", "")
            for port in ports:
                path = port.get("path", "")
                if path:
                    title = port.get("label", "")
                    editor.addItem(path + (f" · {title}" if title else ""), path)
            index = editor.findData(value)
            if index >= 0:
                editor.setCurrentIndex(index)
            else:
                editor.setEditText(value)
            editor.blockSignals(False)

    def configuration(self):
        result = deepcopy(self.original)
        for identifier, title, preset, hand in GROUPS:
            group = result["process_groups"][self._indices[identifier]]
            group["enabled"] = self.enabled_checks[identifier].isChecked()
            parameters = group.setdefault("parameters", {})
            for key, _, _ in FIELDS[preset]:
                parameters[key] = self._value(self.fields[(identifier, key)])
            if hand:
                parameters["hand"] = hand
        return result

    def _validate_required(self, configuration):
        for identifier, title, preset, _ in GROUPS:
            group = configuration["process_groups"][self._indices[identifier]]
            if not group["enabled"]:
                continue
            required = {"cameras": {"config_path", "output_root"},
                        "trackers": {"bundle_path", "role_map_path"},
                        "glove": {"glove_python", "fsglove_source", "mano_assets", "port"},
                        "tactile": {"port"}}[preset]
            if preset == "glove" and group["parameters"].get("launch_gui") == "false":
                required = set()  # Existing bridge-only profiles retain their meaning.
            for key, field_title, kind in FIELDS[preset]:
                value = group["parameters"].get(key, "")
                if key in required and not value:
                    self.fields[(identifier, key)].setFocus()
                    raise ValueError(f"{title}：请填写{field_title}，或取消启用该模块。")
                if value and kind in {"file", "directory", "serial"} and not Path(value).expanduser().is_absolute():
                    raise ValueError(f"{title} · {field_title}：请选择本机绝对路径。")

    def _set_status(self, text):
        self.status_label.setText(text[:400])
        self.status_label.setToolTip(text)

    def apply(self):
        if self._applying or runtime_busy(self.runtime):
            return
        try:
            configuration = self.configuration()
            self._validate_required(configuration)
            configuration = validate_configuration(configuration)
            # The runtime validates and persists on its worker, including
            # whether this process owns an idle backend. No wait()/join here.
            accepted = self.runtime.apply(configuration, deepcopy(self.snapshot_provider() or {}))
            if accepted is False:
                self._set_status("未应用：" + str((self.runtime.status or {}).get("detail", "后台暂不允许修改配置。")))
                return
        except (ValueError, RuntimeError, OSError) as exc:
            self._set_status(f"未应用：{exc}")
            return
        self._applying = True
        self._set_status("正在后台保存并应用。不会自动启动设备；关闭此窗口不撤销已提交操作。")
        self.tabs.setEnabled(False)
        self.apply_button.setEnabled(False)
        self.refresh_ports_button.setEnabled(False)
        self.cancel_button.setText("关闭")

    def refresh_status(self):
        state = dict(self.runtime.status or {})
        busy = runtime_busy(self.runtime)
        if self._applying:
            detail = state.get("detail", "")
            if detail:
                self._set_status(detail)
            if not busy:
                self._applying = False
                self.tabs.setEnabled(True)
                self.refresh_ports_button.setEnabled(True)
        elif busy:
            self._set_status(state.get("detail") or "后台正在处理，请稍候。")
        self.apply_button.setEnabled(not busy and not self._applying)

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)

    def done(self, result):
        self.timer.stop()
        super().done(result)


class BackendLogDialog(QDialog):
    """Explicit, bounded read of the desktop runtime's fixed local log path."""

    def __init__(self, runtime, parent=None):
        super().__init__(parent)
        self.runtime = runtime
        self.setWindowTitle("后台日志 · 只读")
        self.resize(820, 540)
        layout = QVBoxLayout(self)
        self.path_label = _label(str(runtime.log_path or "尚无后台日志"))
        layout.addWidget(self.path_label)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        layout.addWidget(self.text, 1)
        actions = QHBoxLayout()
        refresh = QPushButton("刷新日志")
        refresh.clicked.connect(self.refresh)
        actions.addWidget(refresh)
        actions.addStretch()
        close = QPushButton("关闭")
        close.clicked.connect(self.close)
        actions.addWidget(close)
        layout.addLayout(actions)
        self.refresh()

    def refresh(self):
        path = self.runtime.log_path
        if not path:
            self.text.setPlainText("尚无后台日志。请先在桌面工作台配置或启动后台。")
            return
        fd = None
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode):
                raise ValueError("日志路径不是普通文件")
            limit = 64 * 1024
            os.lseek(fd, max(0, info.st_size - limit), os.SEEK_SET)
            data = os.read(fd, limit).decode("utf-8", errors="replace")
            self.text.setPlainText(("（仅显示日志末尾 64 KiB）\n" if info.st_size > limit else "") + data)
        except (ValueError, OSError) as exc:
            self.text.setPlainText(f"日志暂不可读取：{exc}")
        finally:
            if fd is not None:
                os.close(fd)
