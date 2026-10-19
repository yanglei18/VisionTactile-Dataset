"""Explicit process-group commands; this widget never owns device processes."""

from __future__ import annotations

from ...i18n import tr
from ...i18n.demo import demo_group_label
from ...i18n.labels import builtin_label
from ...i18n.messages import message as display_message
from ...backend.operation_receipts import operation_for

import time

from PySide6.QtCore import Qt
from ...i18n.widgets import (
    QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)


PROCESS_STATES = {
    "DISABLED": tr('未启用'), "STOPPED": tr('已停止'), "STARTING": tr('启动中'),
    "RUNNING": tr('运行中'), "DEGRADED": tr('数据异常'), "STOPPING": tr('停止中'),
    "FAILED": tr('启动 / 运行失败'), "EXTERNAL": tr('外部进程'), "UNKNOWN": tr('状态未知'),
}
KINDS = {"camera": tr('相机'), "tracker": tr('追踪器'), "glove": tr('手势'), "tactile": tr('触觉')}


_DEFAULT_GROUP_LABELS = {
    "cameras": ("camera", "三相机与录制控制器"),
    "trackers": ("tracker", "三 Tracker ROS 读取"),
    "right_glove": ("glove", "右手姿态与标定窗口"),
    "left_glove": ("glove", "左手姿态与标定窗口"),
    "right_tactile": ("tactile", "右手触觉采集"),
    "left_tactile": ("tactile", "左手触觉采集"),
}


def _group_label(group, *, mode=None):
    """Translate only the exact built-in ID/kind/label combination."""
    if mode == "demo":
        return demo_group_label(group)
    expected = _DEFAULT_GROUP_LABELS.get(group.get("id"))
    label = group["label"]
    if expected == (group.get("kind"), label):
        return builtin_label(label)
    return label


def _text(value="", *, wrap=False):
    widget = QLabel(value)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setObjectName("muted")
    widget.setWordWrap(wrap)
    widget.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    return widget


class DeviceControls(QWidget):
    """Render authoritative flags, with an additional external-ownership guard."""

    def __init__(self, backend, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.groups = []
        self.selected_id = None
        self.pending_request = None
        self.pending_instance_id = None
        self._submitted_at = 0.0
        self._snapshot = {}
        self._available = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        heading = QHBoxLayout()
        title = QLabel(tr('设备进程组'))
        title.setObjectName("sectionTitle")
        heading.addWidget(title)
        self.availability = _text()
        self.availability.setAlignment(Qt.AlignmentFlag.AlignRight)
        heading.addWidget(self.availability, 1)
        layout.addLayout(heading)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels([tr('进程组'), tr('类型'), tr('状态'), tr('归属'), tr('进程号')])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.setMinimumHeight(0)
        self.table.itemSelectionChanged.connect(self._select)
        layout.addWidget(self.table, 1)
        actions = QHBoxLayout()
        self.start_button = QPushButton(tr('启动'))
        self.start_button.setObjectName("primary")
        self.stop_button = QPushButton(tr('停止'))
        self.restart_button = QPushButton(tr('重启'))
        for action, button in (("start", self.start_button), ("stop", self.stop_button),
                               ("restart", self.restart_button)):
            button.clicked.connect(lambda checked=False, action=action: self.submit(action))
            actions.addWidget(button)
        self.operation_notice = _text()
        self.operation_notice.setMaximumHeight(22)
        actions.addWidget(self.operation_notice, 1)
        layout.addLayout(actions)
        self.detail = _text(tr('选择进程组查看归属、状态和日志路径。'), wrap=True)
        self.detail.setMaximumHeight(38)
        self.detail.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.detail)
        logs = QHBoxLayout()
        logs.addWidget(QLabel(tr('日志路径')))
        self.log_path = QLineEdit()
        self.log_path.setReadOnly(True)
        self.log_path.setPlaceholderText(tr('暂无日志；本界面不自动打开或执行路径'))
        self.log_path.setAccessibleName(tr('所选进程组日志路径'))
        logs.addWidget(self.log_path, 1)
        layout.addLayout(logs)
        self._update_selection()

    @property
    def control_available(self) -> bool:
        """Whether the last refresh permits this backend's process controls."""
        return self._available

    def _notice(self, text):
        self.operation_notice.setText(text[:180])
        self.operation_notice.setToolTip(text)

    def refresh(self, snapshot):
        self._snapshot = snapshot
        control = snapshot.get("process_control")
        if control is None and self.backend.mode == "demo":
            # Keep DemoBackend.snapshot() a valid, unchanged v1 snapshot.
            control = getattr(self.backend, "process_control", {})
        control = control or {}
        self._available = bool(
            getattr(self.backend, "supports_device_control", False) and control.get("enabled")
            and (self.backend.mode == "demo" or snapshot.get("capabilities", {}).get("device_control")))
        self.availability.setText(tr('仅内存模拟 · 不启动硬件') if self.backend.mode == "demo"
                                  else (tr('控制已启用 · 仅管理本后台进程') if self._available
                                        else tr('只读 / 连接不可用 · 控制已禁用')))
        self.groups = control.get("groups", [])
        operation = operation_for(self.backend, self.pending_request, snapshot.get("management"))
        if self.pending_request:
            changed = self.pending_instance_id != snapshot.get("instance_id")
            if changed or not self._available:
                self._notice(tr('后台已更换或失联；操作结果待核实，不会自动重试。'))
                self.pending_request = None
            elif operation and operation.get("request_id") == self.pending_request:
                if operation.get("code") not in {"QUEUED", "queued"}:
                    prefix = tr('命令已受理，以进程组状态为准。') if operation.get("state") == "accepted" else tr('操作未确认完成。')
                    self._notice(prefix + tr('{v0}：{v1}', v0=display_message(operation.get('code', '')), v1=display_message(operation.get('detail', ''))))
                    self.pending_request = None
                else:
                    self._notice(tr('命令已排队，等待后台执行；不会自动重复提交。'))
            if self.pending_request and time.monotonic() - self._submitted_at > 15.0:
                # A manager may never complete its accepted intent. Report an
                # unknown outcome without waiting forever or resending it.
                self._notice(tr('未取得本次命令的最终回执；请核实进程组状态，不会自动重试。'))
                self.pending_request = None
        ids = [group["id"] for group in self.groups]
        if self.selected_id is None and ids:
            self.selected_id = ids[0]
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.groups))
        for row, group in enumerate(self.groups):
            ownership = (tr('外部 · 不接管') if group["state"] == "EXTERNAL" else
                         tr('本工作台后台') if group["owned"] else
                         tr('归属未确认') if group.get("pid") else tr('无托管进程'))
            values = (_group_label(group, mode=self.backend.mode), KINDS.get(group["kind"], group["kind"]),
                      PROCESS_STATES.get(group["state"], group["state"]), ownership,
                      str(group["pid"]) if group.get("pid") is not None else "—")
            for column, value in enumerate(values):
                item = self.table.item(row, column)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(row, column, item)
                item.setText(value)
                item.setToolTip(value)
                item.setData(Qt.ItemDataRole.UserRole, group["id"])
        if self.selected_id in ids:
            self.table.selectRow(ids.index(self.selected_id))
        else:
            # Do not silently retarget a destructive action when a group goes away.
            self.table.clearSelection()
            self.table.setCurrentCell(-1, -1)
        self.table.blockSignals(False)
        self._update_selection()

    def _select(self):
        row = self.table.currentRow()
        if self.table.selectionModel().hasSelection() and 0 <= row < len(self.groups):
            self.selected_id = self.groups[row]["id"]
        self._update_selection()

    def _selected(self):
        return next((group for group in self.groups if group["id"] == self.selected_id), None)

    def _allowed(self, group, action):
        if not group or not self._available or self.pending_request:
            return False
        if group["state"] == "EXTERNAL":
            return False
        if action in {"stop", "restart"} and not group["owned"]:
            return False
        return bool(group.get("can_" + action))

    def _update_selection(self):
        group = self._selected()
        for action, button in (("start", self.start_button), ("stop", self.stop_button),
                               ("restart", self.restart_button)):
            button.setEnabled(self._allowed(group, action))
            button.setToolTip(tr('仅向后台提交显式命令；进程运行不代表数据已有效') if button.isEnabled()
                              else tr('需要有效后台连接、允许的状态和归属；不能接管外部进程'))
        if group:
            text = tr('{v0} · {v1}：{v2}', v0=_group_label(group, mode=self.backend.mode), v1=PROCESS_STATES.get(group['state'], group['state']), v2=display_message(group['detail']))
            self.detail.setText(text)
            self.detail.setToolTip(text + "\n" + tr('数据主题') + "：" + ", ".join(group.get("topics", [])))
            path = group.get("log_path", "")
            if self.log_path.text() != path:
                self.log_path.setText(path)
                self.log_path.setCursorPosition(0)
            self.log_path.setToolTip(path)
        else:
            self.detail.setText(tr('没有已选进程组；后台未配置时仍可在“订阅数据流”页查看已有数据。'))
            self.detail.setToolTip("")
            self.log_path.clear()

    def submit(self, action):
        # Recheck live capability at click time, not only on the 100 ms refresh.
        if not getattr(self.backend, "supports_device_control", False):
            self._available = False
        group = self._selected()
        if not self._allowed(group, action):
            self._notice(tr('当前不能执行：请确认后台连接、进程状态和归属。'))
            self._update_selection()
            return
        try:
            instance = self._snapshot.get("instance_id", "")
            if not instance:
                raise ValueError(tr('后台实例身份未知'))
            self.pending_request = self.backend.submit("device." + action,
                {"group_id": group["id"], "expected_manager_instance_id": instance})
            self.pending_instance_id = instance
            self._submitted_at = time.monotonic()
            self._notice(tr('命令正在处理；不会自动重复提交。'))
        except (ValueError, RuntimeError, OSError) as exc:
            self._notice(tr('未提交：{v0}', v0=display_message(str(exc))))
        self._update_selection()
