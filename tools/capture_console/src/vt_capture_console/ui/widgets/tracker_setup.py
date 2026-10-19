"""Explicit Tracker bootstrap wizard, separate from manager configuration."""
from __future__ import annotations

from ...i18n import tr
from ...i18n.messages import message as display_message

import queue
import threading

from PySide6.QtCore import Qt, QTimer, Signal
from ...i18n.widgets import (
    QCheckBox, QDialog, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
    QLineEdit, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget,
)

from ...backend.tracker_workflow import (
    BootstrapConfig, PROMPTS, TrackerWorkflow, capture_sha256,
    detect_defaults, initialization_blocker,
)


def _label(text, name="muted"):
    result = QLabel(text)
    result.setObjectName(name)
    result.setTextFormat(Qt.TextFormat.PlainText)
    result.setWordWrap(True)
    return result


class TrackerSetupDialog(QDialog):
    """No process or device is opened by constructing/showing this dialog."""
    reading_requested = Signal()

    def __init__(self, snapshot_provider, parent=None, *, admission_path=None, fresh_snapshot_provider=None,
                 settings=None):
        super().__init__(parent)
        self.setWindowTitle(tr('追踪器 · 手动初始化'))
        self.setMinimumSize(650, 620)
        self.resize(760, 760)
        self.setModal(True)
        self._provider = snapshot_provider
        self.settings = settings
        self.started_config = None
        self._reading_pending = False
        self._settings_error = ""
        self._submission_error = ""
        self.workflow = TrackerWorkflow(snapshot_provider, admission_path=admission_path,
                                        fresh_snapshot_provider=fresh_snapshot_provider)
        self._hash_results = queue.Queue(maxsize=1)
        self._hash_cancel = threading.Event()
        self._hash_thread = None
        self._closing = False
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.addWidget(_label(tr('恢复既有 Windows 配对与地图'), "sectionTitle"))
        layout.addWidget(_label(tr('此向导复用已批准的 PyVUT 初始化脚本。只有点击“开始初始化”才会执行设备写入；不会配对、清地图或自动重试。请先关闭追踪器发布器，按步骤操作固定主机与两个从机。')))
        form = QFormLayout()
        self.fields = {}
        self.browse_buttons = []
        try:
            defaults = settings.defaults() if settings is not None else detect_defaults()
        except (ValueError, OSError, TypeError) as exc:
            defaults = {}
            self._settings_error = str(exc)
        for name, title in (("python", tr('批准的 Python')), ("script", tr('批准的初始化脚本')),
                            ("capture", tr('批准的 抓包文件')), ("bundle", tr('批准的 启动包'))):
            row = QWidget()
            horizontal = QHBoxLayout(row)
            horizontal.setContentsMargins(0, 0, 0, 0)
            edit = QLineEdit(defaults.get(name, ""))
            edit.setAccessibleName(title)
            edit.setPlaceholderText(tr('选择已批准的本地文件'))
            edit.setCursorPosition(0)
            edit.textChanged.connect(self._inputs_changed)
            button = QPushButton(tr('选择…'))
            button.clicked.connect(lambda checked=False, key=name: self._browse(key))
            horizontal.addWidget(edit, 1)
            horizontal.addWidget(button)
            self.fields[name] = edit
            self.browse_buttons.append(button)
            form.addRow(title, row)
        digest_row = QWidget()
        digest_layout = QHBoxLayout(digest_row)
        digest_layout.setContentsMargins(0, 0, 0, 0)
        self.fields["expected_sha256"] = QLineEdit(defaults.get("expected_sha256", ""))
        self.fields["expected_sha256"].setAccessibleName(tr('批准的抓包文件SHA-256'))
        self.fields["expected_sha256"].setPlaceholderText(tr('粘贴已批准指纹，或计算所选文件后人工核对'))
        self.fields["expected_sha256"].textChanged.connect(self._inputs_changed)
        self.hash_button = QPushButton(tr('计算指纹'))
        self.hash_button.clicked.connect(self._compute_hash)
        digest_layout.addWidget(self.fields["expected_sha256"], 1)
        digest_layout.addWidget(self.hash_button)
        form.addRow(tr('抓包文件指纹'), digest_row)
        layout.addLayout(form)
        self.hash_note = _label(tr('文件指纹用于核对既有抓包文件与启动包；计算指纹不会访问追踪器，也不代表批准。'))
        layout.addWidget(self.hash_note)
        saved_row = QHBoxLayout()
        self.save_button = QPushButton(tr('保存本机初始化配置'))
        self.save_button.clicked.connect(self._save_settings)
        self.save_note = _label(tr('已填入本机配置；保存后重启仍保留。每次初始化仍需手动确认。')
                                if settings is not None else tr('此窗口未配置本机存储；可直接完成本次初始化。'))
        if self._settings_error:
            self.save_note.setText(tr('本机配置读取失败，未覆盖原文件：') + display_message(self._settings_error))
        saved_row.addWidget(self.save_button)
        saved_row.addWidget(self.save_note, 1)
        layout.addLayout(saved_row)
        self.approval = QCheckBox(tr('我已核对以上文件及指纹已获批准，明确同意本次初始化写入设备'))
        self.approval.toggled.connect(self.refresh)
        layout.addWidget(self.approval)
        self.guard_label = _label("")
        layout.addWidget(self.guard_label)
        self.stage = _label(tr('尚未启动'), "sectionTitle")
        layout.addWidget(self.stage)
        self.instruction = _label("")
        self.instruction.setMinimumHeight(44)
        layout.addWidget(self.instruction)
        self.confirm_button = QPushButton(tr('等待脚本提示'))
        self.confirm_button.setObjectName("primary")
        self.confirm_button.clicked.connect(self._confirm)
        layout.addWidget(self.confirm_button)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(60)
        self.log.setPlaceholderText(tr('仅显示识别出的流程事件；不会输出私密抓包文件/启动包正文。'))
        layout.addWidget(self.log, 1)
        self.reading_button = QPushButton(tr('启动读取并查看追踪器'))
        self.reading_button.setObjectName("primary")
        self.reading_button.clicked.connect(self._request_reading)
        self.reading_button.hide()
        layout.addWidget(self.reading_button)
        self.reading_note = _label("")
        self.reading_note.hide()
        layout.addWidget(self.reading_note)
        actions = QHBoxLayout()
        self.start_button = QPushButton(tr('开始初始化 · 写入设备'))
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self._start)
        self.cancel_button = QPushButton(tr('取消本次初始化'))
        self.cancel_button.clicked.connect(self.workflow.cancel)
        self.close_button = QPushButton(tr('关闭'))
        self.close_button.clicked.connect(self.reject)
        for button in (self.start_button, self.cancel_button, self.close_button):
            actions.addWidget(button)
        layout.addLayout(actions)
        layout.addWidget(_label(tr('“完成”仅确认本次初始化流程；不等于位姿、频率或标定验收通过。关闭窗口不会停止其他设备。')))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.refresh()

    def _inputs_changed(self, *_):
        had_submission_error = bool(self._submission_error)
        self._submission_error = ""
        if hasattr(self, "approval"):
            self.approval.setChecked(False)
        if hasattr(self, "save_note") and self.settings is not None and not self._settings_error:
            self.save_note.setText(tr('配置已修改；点击保存，或在开始初始化时保存。'))
        if had_submission_error:
            self.refresh()

    def _values(self):
        return {key: edit.text().strip() for key, edit in self.fields.items()}

    def _save_settings(self):
        # Preferences do not carry consent. Opening the next dialog always
        # requires a fresh approval click and the actual physical prompts.
        if self.settings is None or self.workflow.snapshot()["state"] in {"RUNNING", "WAITING", "CANCELLING"}:
            return False
        try:
            if self._settings_error:
                raise ValueError(self._settings_error)
            self.settings.save(self._values())
        except (ValueError, OSError, TypeError) as exc:
            self.save_note.setText(tr('配置未保存，原文件保留：') + display_message(str(exc)))
            return False
        self.save_note.setText(tr('本机配置已保存；重启后自动填入，不会自动开始初始化。'))
        return True

    def set_reading_feedback(self, text, *, pending=False):
        self._reading_pending = pending
        self.reading_note.setText(display_message(text))
        self.reading_note.setVisible(bool(text))
        self.refresh()

    def _request_reading(self):
        if self.workflow.snapshot()["state"] == "COMPLETE" and not self._reading_pending:
            self.reading_requested.emit()

    def _browse(self, key):
        path, _ = QFileDialog.getOpenFileName(self, tr('选择已批准文件'), self.fields[key].text())
        if path:
            self.fields[key].setText(path)
            if key == "capture":
                self.fields["expected_sha256"].clear()

    def _compute_hash(self):
        if self._hash_thread and self._hash_thread.is_alive():
            return
        path = self.fields["capture"].text().strip()
        self.approval.setChecked(False)
        self._hash_cancel.clear()
        self.hash_note.setText(tr('正在本地计算指纹… 不访问设备。'))
        def work():
            try:
                value, error = capture_sha256(path, self._hash_cancel.is_set), ""
            except Exception as exc:
                value, error = "", str(exc)[:500]
            self._hash_results.put((path, value, error))
        self._hash_thread = threading.Thread(target=work, name="tracker-capture-digest", daemon=True)
        self._hash_thread.start()
        self.refresh()

    def _start(self):
        if not self.approval.isChecked():
            return
        try:
            config = BootstrapConfig(**self._values())
            config.argv()
            if self.settings is not None and not self._save_settings():
                self._submission_error = tr('初始化未启动；请先处理本机配置保存错误。')
                self.refresh()
                return
            self.workflow.start(config)
            self._submission_error = ""
            self.started_config = config
            self.set_reading_feedback("")
        except (ValueError, OSError) as exc:
            self._submission_error = tr('未启动：') + display_message(str(exc))
            self.refresh()
            return
        self.refresh()

    def _confirm(self):
        prompt = self.workflow.snapshot()["prompt"]
        try:
            self.workflow.answer(prompt)
        except ValueError as exc:
            self.instruction.setText(tr('未继续：') + display_message(str(exc)))
        self.refresh()

    def refresh(self, *_):
        try:
            path, digest, error = self._hash_results.get_nowait()
        except queue.Empty:
            pass
        else:
            if path != self.fields["capture"].text().strip():
                self.hash_note.setText(tr('所选抓包文件已变化，丢弃旧文件指纹。'))
            elif error:
                self.hash_note.setText(tr('指纹计算失败：') + display_message(error))
            else:
                self.fields["expected_sha256"].setText(digest)
                self.hash_note.setText(tr('指纹已计算。请确认这是已批准的 抓包文件；计算本身不是批准。'))
        view = self.workflow.snapshot()
        active = view["state"] in {"RUNNING", "WAITING", "CANCELLING"}
        hashing = self._hash_thread is not None and self._hash_thread.is_alive()
        try:
            blocker = initialization_blocker(self._provider())
        except Exception:
            blocker = tr('后台快照不可用；初始化已禁用。')
        self.guard_label.setText(display_message(blocker) if blocker else tr('安全检查通过：无活动 / 待核实录制，追踪器发布器未占用。'))
        self.start_button.setEnabled(not active and not hashing and not blocker and self.approval.isChecked())
        self.cancel_button.setEnabled(active and view["state"] != "CANCELLING")
        for field in self.fields.values():
            field.setEnabled(not active)
        for button in self.browse_buttons:
            button.setEnabled(not active)
        self.hash_button.setEnabled(not active and not hashing)
        self.save_button.setEnabled(self.settings is not None and not self._settings_error and not active and not hashing)
        self.approval.setEnabled(not active and not hashing)
        self.reading_button.setVisible(view["state"] == "COMPLETE")
        self.reading_button.setEnabled(view["state"] == "COMPLETE" and not self._reading_pending)
        self.stage.setText({"IDLE": tr('准备初始化'), "RUNNING": tr('初始化进行中'), "WAITING": tr('等待你的明确操作'),
                            "CANCELLING": tr('正在取消'), "CANCELLED": tr('已取消 · 未宣告完成'), "FAILED": tr('初始化未完成'),
                            "COMPLETE": tr('初始化完成')}[view["state"]])
        if self._submission_error:
            self.instruction.setText(self._submission_error)
        elif view["state"] == "IDLE":
            self.instruction.setText(tr('先确保三只追踪器全部关机；启动后只按界面明确提示开机。'))
        else:
            self.instruction.setText(display_message(view["detail"]))
        labels = dict(zip(PROMPTS, (tr('确认三只追踪器均已关闭'), tr('主机 已准备好，等待开机指令'),
                                   tr('两个从机已准备好，等待同时开机指令'))))
        self.confirm_button.setText(labels.get(view["prompt"], tr('等待脚本提示')))
        self.confirm_button.setEnabled(view["state"] == "WAITING" and not blocker)
        content = "\n".join(view["logs"])
        if self.log.toPlainText() != content:
            self.log.setPlainText(content)
        if self._closing and not active:
            self.timer.stop()
            super().reject()

    def reject(self):
        self._hash_cancel.set()
        self._closing = True
        if self.workflow.snapshot()["state"] in {"RUNNING", "WAITING", "CANCELLING"}:
            self.workflow.cancel()
            self.refresh()
            return
        self.timer.stop()
        super().reject()

    def closeEvent(self, event):
        # The close request is not acceptance that the child has stopped. Keep
        # this dialog alive until refresh observes the worker's terminal state;
        # only then does reject emit finished (the owner may delete the dialog).
        active = self.workflow.snapshot()["state"] in {"RUNNING", "WAITING", "CANCELLING"}
        if active:
            event.ignore()
        else:
            event.accept()
        self.reject()
