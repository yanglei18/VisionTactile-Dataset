"""Non-blocking offline inspection/alignment/validation with a fixed launcher."""
from __future__ import annotations

from ...i18n import tr
from ...i18n.messages import message as display_message

from pathlib import Path

from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtGui import QDesktopServices
from ...i18n.widgets import (
    QComboBox, QDialog, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
    QLineEdit, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget,
)

from ...backend.alignment_task import AlignmentTask, recording_blocker


def _label(text=""):
    widget = QLabel(text)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setWordWrap(True)
    widget.setObjectName("muted")
    return widget


class OfflineAlignmentDialog(QDialog):
    def __init__(self, repo_root, parent=None, *, snapshot_provider=None):
        super().__init__(parent)
        self.repo_root = Path(repo_root)
        self.snapshot_provider = snapshot_provider
        self.task = AlignmentTask(repo_root, snapshot_provider=snapshot_provider)
        self._closing = False
        self._last_log = None
        self.setWindowTitle(tr('离线数据工具 · 检查 / 对齐 / 验证'))
        self.setModal(False)
        self.resize(880, 650)
        self.setMinimumSize(660, 480)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        layout.setSpacing(10)
        layout.addWidget(_label(tr('复用现有离线算法，不修改源数据包。录制期间不能开始离线任务；对齐只写入新的独立目录。')))
        self.form_widget = QWidget()
        form = QFormLayout(self.form_widget)
        form.setContentsMargins(0, 0, 0, 0)
        self.action = QComboBox()
        for title, value in ((tr('检查数据包（只读）'), "inspect"), (tr('对齐并导出新结果'), "align"), (tr('验证已有结果（只读）'), "validate")):
            self.action.addItem(title, value)
        self.action.currentIndexChanged.connect(self._update_fields)
        form.addRow(tr('操作'), self.action)
        self.fields = {}
        self.browse_buttons = {}
        for key, title in (("bag", tr('已结束的数据包目录')), ("config", tr('对齐配置文件')),
                           ("extrinsics", tr('外参目录')), ("output", tr('结果目录'))):
            field = QLineEdit()
            field.setAccessibleName(title)
            field.setPlaceholderText(tr('选择本机路径'))
            button = QPushButton(tr('选择…'))
            button.clicked.connect(lambda checked=False, key=key: self._browse(key))
            row = QHBoxLayout()
            row.addWidget(field, 1)
            row.addWidget(button)
            form.addRow(title, row)
            self.fields[key] = field
            self.browse_buttons[key] = button
        layout.addWidget(self.form_widget)
        self.status = _label(tr('请选择已结束的数据包和对应配置。'))
        self.status.setMaximumHeight(58)
        layout.addWidget(self.status)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText(tr('工具的标准输出和错误会显示在这里；日志最多保留最近 128 KiB。'))
        layout.addWidget(self.log, 1)
        actions = QHBoxLayout()
        self.start_button = QPushButton(tr('开始'))
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start)
        self.cancel_button = QPushButton(tr('取消本次任务'))
        self.cancel_button.clicked.connect(self.cancel)
        self.open_button = QPushButton(tr('打开结果目录'))
        self.open_button.clicked.connect(self.open_result)
        actions.addWidget(self.start_button)
        actions.addWidget(self.cancel_button)
        actions.addWidget(self.open_button)
        actions.addStretch()
        self.close_button = QPushButton(tr('关闭'))
        self.close_button.clicked.connect(self.close)
        actions.addWidget(self.close_button)
        layout.addLayout(actions)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self._update_fields()
        self.refresh()

    @property
    def running(self):
        return self.task.running

    def _blocker(self):
        if self.snapshot_provider is None:
            return ""
        try:
            return recording_blocker(self.snapshot_provider())
        except (RuntimeError, OSError, ValueError):
            return tr('采集后台状态暂不可读取，请先确认录制已经结束。')

    def _update_fields(self):
        action = self.action.currentData()
        for key, field in self.fields.items():
            enabled = ((key in {"bag", "config"} and action in {"inspect", "align"})
                       or (key == "extrinsics" and action == "align")
                       or (key == "output" and action in {"align", "validate"}))
            field.setEnabled(enabled)
            self.browse_buttons[key].setEnabled(enabled)
        self.fields["output"].setPlaceholderText(tr('选择新的目录名称，不覆盖已有结果') if action == "align" else tr('选择已有对齐结果目录'))

    def _browse(self, key):
        field = self.fields[key]
        if key == "config":
            value, _ = QFileDialog.getOpenFileName(self, tr('选择对齐配置'), field.text(), tr('配置文件 (*.yaml *.yml);;所有文件 (*)'))
        elif key == "output" and self.action.currentData() == "align":
            value, _ = QFileDialog.getSaveFileName(self, tr('指定新结果目录名称（不创建文件）'), field.text(), tr('目录名称 (*)'))
        else:
            value = QFileDialog.getExistingDirectory(self, tr('选择目录'), field.text())
        if value:
            field.setText(value)

    def start(self):
        if self.running:
            return
        problem = self._blocker()
        if problem:
            self.status.setText(display_message(problem))
            return
        try:
            self.task.start(self.action.currentData(), **{key: field.text().strip() for key, field in self.fields.items()})
        except (ValueError, RuntimeError, OSError) as exc:
            self.status.setText(tr('未开始：{v0}', v0=display_message(str(exc))))
            return
        self.refresh()

    def cancel(self):
        self.task.cancel()
        self.refresh()

    def refresh(self):
        state = self.task.status
        running = state["state"] in {"RUNNING", "CANCELLING"}
        problem = self._blocker() if not running else ""
        self.form_widget.setEnabled(not running)
        self.start_button.setEnabled(not running and not problem)
        self.start_button.setToolTip(display_message(problem) if problem else tr('只运行离线工具，不启动采集设备'))
        self.cancel_button.setEnabled(state["state"] == "RUNNING")
        self.open_button.setEnabled(not running and bool(state["output_path"]) and Path(state["output_path"]).is_dir())
        if running or state["state"] != "IDLE" or problem:
            self.status.setText(display_message(problem or state["detail"]))
        if state["log"] != self._last_log:
            self.log.setPlainText(state["log"])
            self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())
            self._last_log = state["log"]
        if self._closing and not running:
            self.close()

    def open_result(self):
        path = self.task.status["output_path"]
        if not self.running and path and Path(path).is_dir():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(path).resolve())))

    def showEvent(self, event):
        self._closing = False
        self.timer.start()
        self.refresh()
        super().showEvent(event)

    def reject(self):
        self.close()  # Escape follows the same cancel-and-wait path.

    def closeEvent(self, event):
        if self.running:
            self._closing = True
            self.task.cancel()
            self.status.setText(tr('正在取消本窗口的任务；子进程退出后关闭。'))
            event.ignore()
            return
        self.timer.stop()
        self._closing = False
        event.accept()
