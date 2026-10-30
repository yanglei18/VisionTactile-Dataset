"""Desktop-only Hub connection preference; saving never restarts the manager."""
from __future__ import annotations

from PySide6.QtCore import QTimer, QUrl
from PySide6.QtGui import QDesktopServices
from ...i18n import tr
from ...i18n.widgets import (QCheckBox, QDialog, QFileDialog, QHBoxLayout,
                            QLabel, QLineEdit, QPushButton, QVBoxLayout)


class ViveHubConnectionDialog(QDialog):
    def __init__(self, startup, parent=None):
        super().__init__(parent)
        self.startup = startup
        self.setWindowTitle(tr('VIVE Hub 自动连接'))
        self.resize(660, 300)
        layout = QVBoxLayout(self)
        self.enabled = QCheckBox(tr('打开工作台时，自动启动 VIVE Hub 并接入 Tracker 位姿'))
        layout.addWidget(self.enabled)
        layout.addWidget(QLabel(tr('VIVE Hub 安装目录')))
        row = QHBoxLayout()
        self.directory = QLineEdit()
        row.addWidget(self.directory, 1)
        browse = QPushButton(tr('选择目录'))
        browse.clicked.connect(self._browse)
        row.addWidget(browse)
        layout.addLayout(row)
        help_text = QLabel(tr('已有连接会直接复用。实体 Tracker 仍需开机；首次使用或更换场地时，在 Hub 中完成配对和建图。关闭工作台窗口会保留连接。'))
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.feedback = QLabel()
        self.feedback.setWordWrap(True)
        layout.addWidget(self.feedback)
        row = QHBoxLayout()
        self.save_button = QPushButton(tr('保存设置'))
        self.save_button.clicked.connect(self._save)
        row.addWidget(self.save_button)
        self.connect_button = QPushButton(tr('立即连接'))
        self.connect_button.clicked.connect(self._connect)
        row.addWidget(self.connect_button)
        log = QPushButton(tr('打开启动日志'))
        log.clicked.connect(lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(str(startup.log_path))))
        row.addWidget(log)
        row.addStretch()
        close = QPushButton(tr('关闭'))
        close.clicked.connect(self.reject)
        row.addWidget(close)
        layout.addLayout(row)
        try:
            settings = startup.settings
            self.enabled.setChecked(settings['enabled'])
            self.directory.setText(settings['install_dir'])
        except (OSError, ValueError) as exc:
            self.feedback.setText(str(exc))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        self.finished.connect(self.timer.stop)
        self.refresh()

    def _browse(self):
        directory = QFileDialog.getExistingDirectory(self, tr('选择 VIVE Hub 安装目录'), self.directory.text())
        if directory:
            self.directory.setText(directory)

    def _save(self):
        try:
            self.startup.save(dict(enabled=self.enabled.isChecked(), install_dir=self.directory.text().strip()))
        except (OSError, ValueError) as exc:
            self.feedback.setText(str(exc))
            return False
        self.feedback.setText(tr('设置已保存，下次打开工作台时生效；也可点击“立即连接”。'))
        return True

    def _connect(self):
        if self._save():
            self.startup.start()
            self.refresh()

    def refresh(self):
        status = self.startup.status
        self.status_label.setText(status['detail'])
        self.connect_button.setEnabled(status['state'] != 'connecting')

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)
