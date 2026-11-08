"""Embedded hand panel for the existing workbench preview canvas.

Resources are checked in a worker. The UI subscribes to managed services and
requests lifecycle changes through the existing manager, never through Qt
window ownership or a second serial connection.
"""
from __future__ import annotations

from pathlib import Path

from PySide6 import QtCore, QtWidgets

from ...backend.hand_session import RemoteHandSession, TactileSubscription, HandResourceLoader, TactileIdentityResolver
from ...i18n import tr, language
from ...i18n.widgets import QLabel


class _NoPortDiscovery:
    def start(self): pass
    def stop(self): pass
    def refresh(self): pass
    def latest(self): return None
    def join(self, timeout=0): return True


class _Settings:
    def __init__(self, hand):
        self.hand = hand
        self.store = QtCore.QSettings(QtCore.QSettings.IniFormat, QtCore.QSettings.UserScope,
                                      "visiontactile", f"workbench-hand-{hand}")
        self.data_root = Path(self.store.fileName()).parent / f"workbench-hand-{hand}"

    @property
    def language(self):
        return language()

    def get(self, key, default=None):
        return self.store.value(key, default)

    def set(self, key, value):
        self.store.setValue(key, value)
        self.store.sync()
        if self.store.status() != QtCore.QSettings.NoError:
            raise OSError("Cannot save workbench hand presentation settings")

    def save_language(self, value):
        self.set("language", value)


class ManagedHandPane(QtWidgets.QWidget):
    def __init__(self, hand, backend, parent=None, *, resource_loader=None, panel_factory=None):
        super().__init__(parent)
        if hand not in ("left", "right"):
            raise ValueError("Unknown hand")
        self.hand, self.backend = hand, backend
        self._metadata, self._manager = {}, {}
        self._identity = None
        self._generation = 0
        self._loader = HandResourceLoader(hand, resource_loader)
        self._closed = False
        self._pending = {}
        self._calibration_visible = False
        self._resource_loader, self._panel_factory = resource_loader, panel_factory
        self.panel = self.session = self.tactile = self.subscription = None
        self._identity_resolver = self._identity_requested = None
        self._touch_sequence = 0
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.notice = QLabel(tr('正在准备手部显示资源；设备由工作台独立启停。'))
        self.notice.setWordWrap(True)
        self.notice.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        layout.addWidget(self.notice)
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        layout.addWidget(self.scroll, 1)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self._poll)
        self.timer.start()

    def set_calibration_visible(self, visible):
        self._calibration_visible = bool(visible)
        if self.panel is not None:
            self.panel.calibration_toggle.setChecked(visible)

    def update_manager(self, snapshot):
        self._manager = snapshot
        metadata = snapshot.get("hand_sessions", {}).get(self.hand, {})
        self._metadata = metadata
        identity = tuple(metadata.get(key, "") for key in
                         ("resources", "control_socket", "tactile_socket", "glove_python", "tactile_port"))
        if identity == self._identity:
            return
        self._identity = identity
        self._generation = self._loader.request(identity[0] if identity[1] else None,
                                                interpreter=identity[3], tactile_port=identity[4])
        self._detach()
        if not identity[0] or not identity[1]:
            self.notice.setText(tr('请在设备配置中选择本手的运行资源与手部服务；此页面不会打开串口。'))
            return
        self.notice.setText(tr('正在校验手部模型与显示资源…'))

    def _build(self, resources):
        from ...backend.tactile_managed import ManagedTactileSource
        from ...backend.tactile_reference_store import TactileReferenceStore
        from vt_tactile_capture.display_response import ResponseSettings
        from .window import HandPanel
        settings = _Settings(self.hand)
        reference_store = TactileReferenceStore(settings.data_root / "tactile-references",
                                                preset_path=resources.get("response_profile"), hand=self.hand)
        self.session = RemoteHandSession(self.hand, self._metadata["control_socket"], self.backend)
        self.tactile = ManagedTactileSource(hand=self.hand, reference_store=reference_store,
            device_identity=resources.get("tactile_device_identity"),
            default_response_settings=ResponseSettings(display_range_adc=3000.), session_scoped_identity=True)
        self._identity_resolver = TactileIdentityResolver()
        if self._metadata.get("tactile_socket"):
            self.subscription = TactileSubscription(self.hand, self._metadata["tactile_socket"])
            self.subscription.start()
        self.panel = (self._panel_factory or HandPanel)(
            settings, resources, self.session, self.tactile, owns_session=False,
            port_discovery=_NoPortDiscovery(), connection_request=self._toggle_device,
            connection_status=self._connection_status)
        self.panel.setMinimumSize(580, 600)
        self.panel.language_combo.hide()  # Workbench owns its global language selection.
        self.panel.title.hide()
        self.panel.calibration_toggle.setChecked(self._calibration_visible)
        self.scroll.setWidget(self.panel)
        self.panel.start()
        self.notice.setText(tr('IMU 与触觉可分别启停；关闭页面仅断开展示，后台采集与录制继续运行。'))

    def _group(self, kind):
        for group in self._manager.get("process_control", {}).get("groups", ()):
            if kind == "imu" and group.get("id") == self._metadata.get("group_id"):
                return group
            if kind == "tactile" and f"/gloves/{self.hand}/tactile/state" in group.get("topics", ()):
                return group
        return {}

    def _connection_status(self, kind):
        group = self._group(kind)
        stopping = group.get("can_stop", False)
        enabled = (not self._closed and kind not in self._pending
                   and bool(group.get("can_start") or stopping)
                   and self._manager.get("capabilities", {}).get("device_control", False)
                   and getattr(self.backend, "supports_device_control", False))
        port = self._metadata.get("port", "") if kind == "imu" else self._metadata.get("tactile_port", "")
        return dict(enabled=enabled, label=tr('停止') if stopping else tr('启动'),
                    state=group.get("state", ""), detail=group.get("detail", ""), port=port)

    def _toggle_device(self, kind):
        if not self._connection_status(kind)["enabled"]:
            raise RuntimeError(tr('当前不可启停此数据源，请先确认设备和录制状态。'))
        group = self._group(kind)
        command = "device.stop" if group.get("can_stop") else "device.start"
        self._pending[kind] = self.backend.submit(command, dict(group_id=group["id"],
                                    expected_manager_instance_id=self._manager["instance_id"]))

    def _poll(self):
        if self._closed:
            return
        result = self._loader.take_result()
        if result is not None:
            generation, resources, error = result
            if generation != self._generation:
                return
            if error:
                self.notice.setText(tr('手部资源加载失败：') + error)
            else:
                try:
                    self._build(resources)
                except (OSError, ValueError, RuntimeError, ImportError) as exc:
                    self._detach()
                    self.notice.setText(tr('手部面板无法准备：') + str(exc))
        for kind, identifier in tuple(self._pending.items()):
            receipt = self.backend.operation_result(identifier)
            if receipt and str(receipt.get("code", "")).lower() != "queued":
                self._pending.pop(kind)
                if receipt.get("state") in {"rejected", "unknown"}:
                    self.notice.setText(receipt.get("detail", ""))
        if self.subscription is not None and self.tactile is not None:
            sequence, sample = self.subscription.latest()
            if sample is not None and sequence != self._touch_sequence:
                self._touch_sequence = sequence
                try:
                    self.tactile.observe(sample)
                except (ValueError, TypeError, RuntimeError) as exc:
                    self.notice.setText(str(exc))
            if self.subscription.error:
                self.notice.setText(tr('触觉显示连接不可用：') + self.subscription.error)
        if self._identity_resolver is not None and self.tactile is not None:
            session = self.tactile.identity_request
            if session is not None and session != self._identity_requested:
                self._identity_requested = session
                self._identity_resolver.request(session, self._metadata.get("tactile_port", ""))
            result = self._identity_resolver.take_result()
            if result is not None:
                session, identity, error = result
                self.tactile.bind_device_identity(session, identity, detail=error)

    def _detach(self):
        if self._identity_resolver is not None:
            self._identity_resolver.close()
            self._identity_resolver = self._identity_requested = None
        if self.subscription is not None:
            self.subscription.close()
        if self.panel is not None:
            self.panel.shutdown()
            widget = self.scroll.takeWidget()
            if widget is not None:
                widget.deleteLater()
        else:
            if self.session is not None:
                self.session.close()
            if self.tactile is not None:
                self.tactile.close()
        self.panel = self.session = self.tactile = self.subscription = None
        self._touch_sequence = 0

    def shutdown(self):
        if not self._closed:
            self._closed = True
            self._generation += 1
            self._loader.close()
            self.timer.stop()
            self._detach()

    def closeEvent(self, event):
        self.shutdown()
        event.accept()
