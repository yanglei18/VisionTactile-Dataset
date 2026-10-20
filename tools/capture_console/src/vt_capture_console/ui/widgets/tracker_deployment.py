"""Import a private Tracker deployment package without starting hardware."""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal

from ...backend.tracker_deployment import TrackerDeploymentTask
from ...i18n import tr
from ...i18n.widgets import (
    QDialog, QFileDialog, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout,
)


class TrackerDeploymentDialog(QDialog):
    installed = Signal(object)
    closed = Signal()

    def __init__(self, parent=None, *, task=None, blocker=lambda: ""):
        super().__init__(parent)
        self.task = task or TrackerDeploymentTask()
        self._blocker = blocker
        self._closing = False
        self._delivered = False
        self._closed = False
        self._started_package = ""
        self.setWindowTitle(tr('导入 Tracker 部署包'))
        self.setWindowModality(Qt.WindowModality.WindowModal)
        self.resize(620, 230)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        note = QLabel(tr('选择交付的本机部署包，后台完成校验和安装；不会启动追踪器。'))
        note.setWordWrap(True)
        note.setObjectName("muted")
        layout.addWidget(note)
        row = QHBoxLayout()
        self.package = QLineEdit()
        self.package.setReadOnly(True)
        self.package.setPlaceholderText(tr('选择 .vttracker 部署包'))
        self.package.setAccessibleName(tr('Tracker 部署包文件'))
        row.addWidget(self.package, 1)
        self.browse_button = QPushButton(tr('选择…'))
        self.browse_button.clicked.connect(self._browse)
        row.addWidget(self.browse_button)
        layout.addLayout(row)
        self.status_label = QLabel(tr('请选择部署包。'))
        self.status_label.setTextFormat(Qt.TextFormat.PlainText)
        self.status_label.setWordWrap(True)
        self.status_label.setObjectName("muted")
        layout.addWidget(self.status_label)
        actions = QHBoxLayout()
        actions.addStretch()
        self.close_button = QPushButton(tr('关闭'))
        self.close_button.clicked.connect(self.close)
        actions.addWidget(self.close_button)
        self.install_button = QPushButton(tr('安装部署包'))
        self.install_button.setObjectName("primary")
        self.install_button.clicked.connect(self.start)
        actions.addWidget(self.install_button)
        layout.addLayout(actions)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self.refresh()

    @property
    def running(self):
        return self.task.running

    def _browse(self):
        path, _ = QFileDialog.getOpenFileName(
            self, tr('选择 Tracker 部署包'), self.package.text(),
            tr('Tracker 部署包 (*.vttracker)'))
        if path:
            self.package.setText(path)
            self.refresh()

    def start(self):
        if self.running or not self.package.text():
            return
        problem = self._blocker()
        if problem:
            self.status_label.setText(problem)
            return
        self._delivered = False
        self._started_package = self.package.text()
        self.task.start(self.package.text())
        self.refresh()

    def refresh(self):
        view = self.task.status
        active = view["state"] in {"RUNNING", "CANCELLING"}
        if not active and self._started_package and self.package.text() != self._started_package:
            # The previous result belongs to the package that was installed,
            # not a newly selected file awaiting an explicit Install action.
            view = dict(view, state="IDLE")
        problem = self._blocker() if not active else ""
        self.browse_button.setEnabled(not active)
        self.install_button.setEnabled(not active and bool(self.package.text()) and not problem)
        self.install_button.setToolTip(problem)
        self.close_button.setText(tr('取消安装') if active else tr('关闭'))
        if view["state"] == "CANCELLING":
            text = tr('正在取消安装，完成清理后关闭。')
        elif view["state"] == "RUNNING":
            text = {
                "verify": tr('正在校验部署包…'),
                "environment": tr('正在安装独立 Python 环境…'),
                "validate": tr('正在校验设备配置…'),
                "activate": tr('正在完成本机安装…'),
            }.get(view["stage"], tr('正在安装部署包…'))
        elif view["state"] == "COMPLETE":
            text = tr('安装完成。关闭后核对设备配置，再保存并应用。')
            if not self._delivered and not self._closing:
                self._delivered = True
                self.installed.emit(view["installation"])
        elif view["state"] == "CANCELLED":
            text = tr('安装已取消，工作台配置未修改。')
        elif view["state"] == "FAILED":
            text = {
                "dependency": tr('部署组件不可用，请重新运行工作台安装器。'),
                "package": tr('部署包校验失败，请核对交付版本与文件完整性。'),
                "filesystem": tr('无法安装部署包，请检查本机目录权限和可用空间。'),
                "unsupported_platform": tr('此部署包要求 Ubuntu 24.04、Linux x86_64 和 Python 3.12。'),
                "unsupported_python": tr('需要系统 Python 3.12，请核对 Ubuntu 24.04 部署环境。'),
                "python_unavailable": tr('系统 Python 不可用，请检查 /usr/bin/python3。'),
                "unsafe_directory": tr('安装目录或权限不符合要求，请由部署人员检查本机部署目录。'),
                "process_timeout": tr('安装步骤超时；请检查系统环境后重新安装。'),
                "runtime_dependency": tr('运行依赖不可用；请检查 libhidapi-hidraw0、libhidapi-libusb0 及部署包依赖。'),
                "runtime_validation": tr('设备配置校验失败；请核对启动包、角色表与批准抓包属于同一套设备。'),
                "existing_installation_changed": tr('已有安装版本被修改；请由部署人员检查后重新导入。'),
                "incomplete_installation": tr('已有安装版本不完整；请由部署人员检查后重新导入。'),
                "venv_failed": tr('无法创建 Python 环境；请安装 python3-venv 并检查可用空间。'),
                "dependencies_failed": tr('离线依赖安装失败；请核对部署包中的依赖与 Python 3.12 环境。'),
            }.get(view["error"], tr('安装未完成，工作台配置未修改。请联系部署人员检查交付包。'))
        else:
            text = problem or tr('请选择部署包。')
        self.status_label.setText(text)
        if self._closing and not active:
            self.close()

    def reject(self):
        self.close()

    def closeEvent(self, event):
        if self.running:
            self._closing = True
            self.task.cancel()
            self.status_label.setText(tr('正在取消安装，完成清理后关闭。'))
            event.ignore()
            return
        self.timer.stop()
        event.accept()
        if not self._closed:
            self._closed = True
            self.closed.emit()
