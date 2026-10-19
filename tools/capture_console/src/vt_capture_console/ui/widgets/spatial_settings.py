"""Explicit scene configuration application through the desktop runtime."""

from ...i18n import tr
from copy import deepcopy
from pathlib import Path

from PySide6.QtCore import Qt
from ...i18n.widgets import QDialog, QFileDialog, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout


class SpatialSettingsDialog(QDialog):
    def __init__(self, runtime, snapshot_provider, parent=None):
        super().__init__(parent)
        self.runtime, self.snapshot_provider = runtime, snapshot_provider
        self.setWindowTitle(tr('联合空间配置'))
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.resize(660, 240)
        layout = QVBoxLayout(self)
        note = QLabel(tr('选择本机场景配置文件，包含追踪器坐标、相机绑定与外参路径。\n保存通过工作站配置流程应用；活动采集期间保持现有配置。'))
        note.setWordWrap(True)
        layout.addWidget(note)
        row = QHBoxLayout()
        self.path = QLineEdit(runtime.configuration.get("scene_config_path", "") if runtime else "")
        self.path.setPlaceholderText(tr('本机 scene.yaml 的绝对路径'))
        row.addWidget(self.path, 1)
        browse = QPushButton(tr('选择文件'))
        browse.clicked.connect(self._browse)
        row.addWidget(browse)
        layout.addLayout(row)
        self.message = QLabel("" if runtime else tr('当前连接使用外部管理后台，请在该后台配置 scene_config_path 后启动。'))
        self.message.setWordWrap(True)
        self.message.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.message)
        buttons = QHBoxLayout()
        buttons.addStretch()
        self.save = QPushButton(tr('保存并应用'))
        self.save.setEnabled(runtime is not None)
        self.save.clicked.connect(self._apply)
        buttons.addWidget(self.save)
        close = QPushButton(tr('关闭'))
        close.clicked.connect(self.close)
        buttons.addWidget(close)
        layout.addLayout(buttons)

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(self, tr('选择场景配置'), self.path.text(), "YAML / JSON (*.yaml *.yml *.json)")
        if path:
            self.path.setText(path)

    def _apply(self):
        value = self.path.text().strip()
        path = Path(value).expanduser()
        if not value or not path.is_absolute() or not path.is_file():
            self.message.setText(tr('请选择存在的本机场景配置文件，路径须为绝对路径。'))
            return
        configuration = deepcopy(self.runtime.configuration)
        configuration["scene_config_path"] = str(path)
        accepted = self.runtime.apply(configuration, deepcopy(self.snapshot_provider() or {}))
        self.message.setText(tr('已提交配置应用，请等待管理后台状态刷新。') if accepted else tr('配置未应用；请查看工作站状态并确认当前无活动采集。'))
