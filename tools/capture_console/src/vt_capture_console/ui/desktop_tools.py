"""Workbench entry points for embedded calibration and existing offline tools."""
from __future__ import annotations

from ..i18n import tr
from ..i18n.messages import message as display_message

import time
from pathlib import Path

from PySide6.QtCore import QEvent, QObject, QTimer, QUrl
from PySide6.QtGui import QDesktopServices
from ..i18n.widgets import QAction
from ..i18n.widgets import QMenu, QMessageBox, QToolButton

from ..backend.operation_receipts import operation_for
from .widgets.workstation_settings import runtime_busy


class DesktopTools(QObject):
    def __init__(self, window, backend, runtime):
        super().__init__(window)
        self.window, self.backend, self.runtime = window, backend, runtime
        self.tracker_dialog = self.alignment_dialog = None
        self._tracker_read_request = self._tracker_read_instance = None
        self._tracker_read_started = 0.
        self._closing = False
        self.button = QToolButton(window)
        self.button.setText(tr('标定与离线工具'))
        self.button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(self.button)
        self.button.setMenu(menu)
        self.tracker_action = self._action(menu, tr('追踪器初始化向导'), self.open_tracker)
        self.right_glove_action = self._action(menu, tr('右手姿态标定工具'), lambda: window.open_glove_calibration("right"))
        self.left_glove_action = self._action(menu, tr('左手姿态标定工具'), lambda: window.open_glove_calibration("left"))
        self.calibration_action = self._action(menu, tr('相机–追踪器标定'), self.open_calibration)
        self.alignment_action = self._action(menu, tr('离线检查与时间对齐'), self.open_alignment)
        menu.addSeparator()
        self.folder_action = self._action(menu, tr('打开数据目录'), self.open_data)
        window.settings_actions.insertWidget(window.settings_actions.count() - 1, self.button)
        self.timer = QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self._poll_tools)
        self.timer.start()
        window.installEventFilter(self)

    def _tasks_active(self):
        return bool((self.alignment_dialog is not None and self.alignment_dialog.running)
                    or (self.tracker_dialog is not None and self.tracker_dialog.workflow.snapshot()["state"]
                        in {"RUNNING", "WAITING", "CANCELLING"}))

    def eventFilter(self, watched, event):
        if watched is self.window and event.type() == QEvent.Type.Close:
            if self._tasks_active():
                self._closing = True
                # Keep Qt alive to display cancellation and reap only owned
                # tool children. Sensor and Recorder lifetimes are unchanged.
                for dialog in (self.tracker_dialog, self.alignment_dialog):
                    if dialog is not None:
                        dialog.close()
                self.window.statusBar().showMessage(tr('正在结束本窗口的初始化／离线任务；采集节点不受影响。'))
                event.ignore()
                return True
            for dialog in (self.tracker_dialog, self.alignment_dialog):
                if dialog is not None:
                    dialog.close()
        return super().eventFilter(watched, event)

    def _action(self, menu, title, callback):
        action = QAction(title, menu)
        action.triggered.connect(callback)
        menu.addAction(action)
        return action

    def open_tracker(self):
        if self.tracker_dialog is None:
            if runtime_busy(self.runtime):
                self.window.show_operation_notice(tr('后台正在处理，请稍候。'))
                return
            from ..backend.tracker_settings import TrackerBootstrapSettings
            from .widgets.tracker_setup import TrackerSetupDialog
            self.tracker_dialog = TrackerSetupDialog(
                self.backend.snapshot, self.window, admission_path=self.runtime.admission_path,
                fresh_snapshot_provider=self.runtime.fresh_snapshot,
                settings=TrackerBootstrapSettings(repo_root=self.runtime.repo_root,
                                                  settings_dir=self.runtime.settings_path.parent),
            )
            self.tracker_dialog.reading_requested.connect(self._start_tracker_reading)
            self.tracker_dialog.finished.connect(self._tracker_closed)
        self.tracker_dialog.show()
        self.tracker_dialog.raise_()
        self.tracker_dialog.activateWindow()

    def _tracker_closed(self, *_):
        dialog, self.tracker_dialog = self.tracker_dialog, None
        self._tracker_read_request = self._tracker_read_instance = None
        if dialog is not None:
            dialog.deleteLater()

    def _tracker_feedback(self, message, *, pending=False):
        if self.tracker_dialog is not None:
            self.tracker_dialog.set_reading_feedback(message, pending=pending)

    def _show_trackers(self, message):
        dialog = self.tracker_dialog
        self._tracker_read_request = self._tracker_read_instance = None
        if dialog is not None:
            dialog.accept()
        self.window.show_tracker_preview(message)

    def _start_tracker_reading(self):
        """Explicit continuation only; initialization never auto-starts a reader."""
        dialog = self.tracker_dialog
        if dialog is None:
            return
        if dialog.workflow.snapshot().get("state") != "COMPLETE":
            self._tracker_feedback(tr('初始化尚未完整完成；未启动追踪器读取。'))
            return
        if self._tracker_read_request:
            self._tracker_feedback(tr('读取启动命令仍在等待回执；不会重复提交。'), pending=True)
            return
        try:
            snapshot = self.backend.snapshot()
        except (OSError, RuntimeError, ValueError):
            self._tracker_feedback(tr('后台状态不可用；未启动读取，请恢复连接后重试。'))
            return
        control = snapshot.get("process_control") or {}
        if (self.backend.mode != "live" or snapshot.get("mode") != "live"
                or not snapshot.get("instance_id") or not control.get("enabled")
                or not snapshot.get("capabilities", {}).get("device_control")
                or not getattr(self.backend, "supports_device_control", False)):
            self._tracker_feedback(tr('需要有效的真实设备管理连接；当前未启动追踪器读取。'))
            return
        groups = [group for group in control.get("groups", ()) if group.get("kind") == "tracker"]
        if len(groups) != 1:
            self._tracker_feedback(tr('追踪器进程组配置不唯一或不存在，请在设备页核对配置。'))
            return
        group = groups[0]
        if group.get("state") == "EXTERNAL":
            self._show_trackers(tr('追踪器由外部进程发布；仅查看数据，不接管或重复启动。0 Hz 不代表位姿有效。'))
            return
        if (group.get("owned") is True and type(group.get("pid")) is int and group["pid"] > 0
                and group.get("state") in {"STARTING", "RUNNING", "DEGRADED"}):
            self._show_trackers(tr('已存在本后台追踪器读取进程；仅查看，不重复启动。请核对接收与有效状态。'))
            return
        if (group.get("state") not in {"STOPPED", "FAILED"} or group.get("owned")
                or group.get("pid") is not None or not group.get("can_start")):
            self._tracker_feedback(tr('追踪器当前不可启动；请核对启用状态、进程归属和录制状态。'))
            return
        profile = next((entry for entry in self.runtime.configuration.get("process_groups", ())
                        if entry.get("id") == group["id"] and entry.get("preset") == "trackers"), None)
        configured_bundle = (profile.get("parameters") or {}).get("bundle_path") if profile else None
        initialized_bundle = getattr(getattr(dialog, "started_config", None), "bundle", None)
        bundle_matches = False
        if isinstance(configured_bundle, str) and configured_bundle and isinstance(initialized_bundle, str) and initialized_bundle:
            try:
                bundle_matches = Path(configured_bundle).expanduser().resolve() == Path(initialized_bundle).expanduser().resolve()
            except (OSError, RuntimeError, ValueError):
                pass
        if not bundle_matches:
            self._tracker_feedback(tr('本次初始化启动包与工作台追踪器读取配置不一致；未启动，请核对设备配置。'))
            return
        controls = self.window.device_controls
        controls.refresh(snapshot)
        if controls.pending_request:
            self._tracker_feedback(tr('已有设备命令等待完成；未重复提交追踪器启动。'))
            return
        controls.selected_id = group["id"]
        controls.submit("start")
        if not controls.pending_request:
            self._tracker_feedback(controls.operation_notice.text() or tr('追踪器启动请求未提交。'))
            return
        self._tracker_read_request = controls.pending_request
        self._tracker_read_instance = snapshot["instance_id"]
        self._tracker_read_started = time.monotonic()
        self._tracker_feedback(tr('读取启动已排队，等待后台回执；不代表位姿或频率验收通过。'), pending=True)
        self._poll_tracker_reading()

    def _poll_tracker_reading(self):
        if not self._tracker_read_request or self.tracker_dialog is None:
            return
        try:
            snapshot = self.backend.snapshot()
        except (OSError, RuntimeError, ValueError):
            snapshot = {}
        message = None
        if (snapshot.get("instance_id") != self._tracker_read_instance
                or snapshot.get("mode") != "live"
                or not getattr(self.backend, "supports_device_control", False)):
            message = tr('后台已更换或失联；读取启动结果待核实，不会自动重试。')
        else:
            operation = operation_for(self.backend, self._tracker_read_request, snapshot.get("management"))
            if operation is not None and str(operation.get("code", "")).lower() != "queued":
                if operation.get("state") == "accepted":
                    self._show_trackers(tr('追踪器读取启动已受理；请核对三个角色的数据与有效状态，初始化完成不等于姿态验收通过。'))
                    return
                message = (tr('追踪器读取启动未确认完成：') + display_message(str(operation.get("code", "")))
                           + " · " + display_message(str(operation.get("detail", ""))) + tr('；不会自动重试。'))
            elif time.monotonic() - self._tracker_read_started > 15.:
                message = tr('未取得读取启动的最终回执；请先在设备页核实进程状态，不会自动重试。')
        if message:
            self._tracker_read_request = self._tracker_read_instance = None
            self._tracker_feedback(message)

    def open_calibration(self):
        self.window.open_camera_calibration()

    def open_alignment(self):
        from .widgets.offline_alignment import OfflineAlignmentDialog
        if self.alignment_dialog is None:
            self.alignment_dialog = OfflineAlignmentDialog(
                self.runtime.repo_root, self.window, snapshot_provider=self.backend.snapshot)
        self.alignment_dialog.show()
        self.alignment_dialog.raise_()
        self.alignment_dialog.activateWindow()

    def open_data(self):
        from pathlib import Path
        configuration = self.runtime.configuration
        path = next((group["parameters"].get("output_root") for group in configuration.get("process_groups", ())
                     if group["preset"] == "cameras" and group["parameters"].get("output_root")), configuration.get("disk_path"))
        if not path or not Path(path).is_dir():
            QMessageBox.information(self.window, tr('数据目录'), tr('请在配置设备中选择已存在的数据输出目录。'))
        elif not QDesktopServices.openUrl(QUrl.fromLocalFile(str(path))):
            QMessageBox.warning(self.window, tr('数据目录'), tr('无法打开系统文件管理器。'))

    def _poll_tools(self):
        self._poll_tracker_reading()
        if self._closing and not self._tasks_active():
            self._closing = False
            self.window.close()
