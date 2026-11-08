"""One hand's shared presentation of managed ROS tactile acquisition.

This facade never opens a serial device. The acquisition service owns raw ADC
and recording; every embedded view shares this controller's reference revision.
Only the optional local display socket is owned here, not the ROS publisher.
"""
from collections import deque
from dataclasses import replace
from types import SimpleNamespace

from vt_tactile_capture import host_clock

from .tactile_source import TactileDisplayModel, TactileDisplayState
from .tactile_reference_store import TactileDeviceProfile, TactileDefaultProfile


class ManagedTactileSource:
    def __init__(self, *, hand="right", request_connect=None, request_disconnect=None,
                 reference_store=None, device_identity=None, socket_path=None,
                 receiver_factory=None, default_response_settings=None, session_scoped_identity=False):
        if reference_store is not None and reference_store.hand != hand:
            raise ValueError("Tactile reference store hand mismatch")
        self.hand = hand
        self._connect, self._disconnect = request_connect, request_disconnect
        self._store, self._device_identity = reference_store, device_identity
        self._receiver_factory, self.socket_path = receiver_factory, socket_path
        self._receiver = None
        self._closed = False
        self._accepting = True  # A new view may attach to an already running owner.
        self._connection_state = "disconnected"
        self._diagnostics = deque(maxlen=32)
        self._reference_profile = None
        self._pending_save = False
        self._persistence_error = ""
        self.port = ""
        self._default_response_settings = default_response_settings
        self._session_scoped_identity = session_scoped_identity
        self._transport_identity = self._identity_session = None
        self._identity_revision, self._identity_resolved = 0, False
        self._last_message = None
        self._model = self._new_model(device_identity)

    def _new_model(self, device_identity):
        self._reference_profile = None
        options = dict(hand=self.hand, response_enabled=True, response_settings=self._default_response_settings)
        if self._store is not None:
            try:
                selected = self._store.load(device_identity)
                self._diagnostics.extend(self._store.diagnostics)
                if selected is not None:
                    profile, source = selected
                    if profile.hand != self.hand:
                        raise ValueError("Tactile reference hand mismatch")
                    self._reference_profile = profile
                    options.update(response_reference=profile.reference,
                                   response_settings=profile.settings, reference_source=source)
            except (OSError, ValueError) as exc:
                self._diagnostics.append(f"reference_load: {exc}")
        return TactileDisplayModel(**options)

    @property
    def identity_request(self):
        """Opaque current-stream token for one read-only USB enumeration job."""
        if self._session_scoped_identity and not self._identity_resolved:
            return self._identity_session
        return None

    def _clear_identity(self):
        self._device_identity = self._transport_identity = self._identity_session = None
        self._identity_resolved = False
        self._pending_save, self._last_message = False, None
        self._persistence_error = ""
        self._model = self._new_model(None)

    def bind_device_identity(self, session, identity, *, detail="", now_ns=None):
        """Apply enumeration only to the still-current stream, never a prior one.

        A reference already being collected in this stream remains intact. A
        completed local measurement can now be saved without repeating it.
        """
        if self._closed or session is None or session != self.identity_request:
            return False
        self._identity_resolved = True
        self._device_identity = identity
        if detail:
            self._diagnostics.append(f"device_identity: {detail}")
        if identity is None:
            self._persistence_error = "未取得设备唯一标识，本次参考仅在当前连接使用。"
            return True
        self._persistence_error = ""
        status = self.response_status
        if status.phase == "collecting" or (status.phase == "ready" and status.reference_source == "measured"):
            if status.phase == "ready":
                self._pending_save = True
                self._save_reference()
        else:
            self._model = self._new_model(identity)
            if self._last_message is not None:
                self._model.observe(self._last_message, now_ns=now_ns)
        return True

    @property
    def connection_state(self):
        return self._connection_state

    @property
    def is_connected(self):
        return self._connection_state == "connected"

    @property
    def can_connect(self):
        return not self._closed and self._connection_state in ("disconnected", "error")

    @property
    def can_disconnect(self):
        return self._connection_state in ("connected", "connecting", "disconnecting")

    @property
    def response_status(self):
        return self._model.response_status

    @property
    def response_settings(self):
        return self._model.response_settings

    @property
    def reference_persistence_error(self):
        return self._persistence_error

    @property
    def diagnostic_log(self):
        return "\n".join(self._diagnostics)[-8192:]

    def start(self):
        """Bind only the private display socket, never start acquisition."""
        if self._closed:
            raise RuntimeError("Tactile view is closed")
        if self.socket_path is not None and self._receiver is None:
            from vt_tactile_capture.display_transport import LocalTactileReceiver
            factory = self._receiver_factory or LocalTactileReceiver
            self._receiver = factory(self.socket_path, hand=self.hand)

    def connect(self, port="", baudrate=115200):
        if not self.can_connect:
            raise RuntimeError("Wait for managed tactile acquisition to stop")
        if self._connect is None:
            raise RuntimeError("Start tactile acquisition from the workstation devices page")
        result = self._connect(port, baudrate)
        if result is False:
            raise RuntimeError("Managed tactile start request was rejected")
        self.port = port
        self._connection_state = "connecting"
        self._accepting = True
        return result

    def set_connection_state(self, state, *, detail="", port=None):
        """Apply a manager receipt; a start request alone is not a measurement."""
        if state not in ("disconnected", "connecting", "connected", "disconnecting", "error"):
            raise ValueError("Invalid managed tactile connection state")
        self._connection_state = state
        self._accepting = state in ("connecting", "connected")
        if port is not None:
            self.port = str(port)
        if state in ("disconnected", "disconnecting", "error"):
            if self._session_scoped_identity:
                self._clear_identity()
            self._model.invalidate("未连接", detail or "受管触觉数据流已停止。", reset_reference=True)
            self._pending_save = False

    def observe(self, message, *, now_ns=None):
        if self._closed or not self._accepting:
            return
        if isinstance(message, dict):
            message = SimpleNamespace(**message)
        # The model independently verifies hand, schema, clocks and ownership.
        self._model.observe(message, now_ns=now_ns)
        if self._session_scoped_identity:
            state = self._model.poll(now_ns=now_ns)
            if state.timing is not None:
                timing = state.timing
                transport = (timing.session_id, timing.stream_epoch, timing.transport_reset_generation)
                if transport != self._transport_identity:
                    # Never expose the previous USB device's retained reference
                    # while enumeration of this newly validated stream is pending.
                    self._clear_identity()
                    self._transport_identity = transport
                    self._identity_revision += 1
                    self._identity_session = (*transport, self._identity_revision)
                    self._model.observe(message, now_ns=now_ns)
                self._last_message = message
        if getattr(message, "hand", None) == self.hand:
            if getattr(message, "connected", None) is True:
                self._connection_state = "connected"
            elif getattr(message, "connected", None) is False:
                self._connection_state = "disconnected"
                self._pending_save = False
                if self._session_scoped_identity:
                    self._clear_identity()

    def _save_reference(self):
        status = self.response_status
        if not (self._pending_save and status.phase == "ready" and status.reference_source == "measured"):
            return
        self._pending_save = False
        if self._store is None or self._device_identity is None:
            self._persistence_error = "未取得设备唯一标识，本次参考仅在当前连接使用。"
            return
        profile = self._store.capture(self._device_identity, self._model.export_reference(), self.response_settings)
        self._reference_profile = profile
        try:
            self._store.save(profile)
        except (OSError, ValueError) as exc:
            self._diagnostics.append(f"reference_save: {exc}")
            self._persistence_error = "当前参考未能保存；本次连接继续使用新参考。"
            self._model.retain_reference(profile.reference, source="measured")
        else:
            self._model.retain_reference(profile.reference, source="saved")
            self._persistence_error = ""

    def poll(self, now_ns=None):
        if self._receiver is not None:
            try:
                message = self._receiver.receive_latest()
                if message is not None:
                    self.observe(message, now_ns=now_ns)
            except (OSError, ValueError) as exc:
                self._diagnostics.append(f"display_transport: {exc}")
                self._model.invalidate("数据无效", str(exc), reset_reference=True)
        now = host_clock.monotonic_ns() if now_ns is None else now_ns
        result = self._model.poll(now_ns=now)
        self._save_reference()
        if self._closed:
            return TactileDisplayState(None, "已关闭", "触觉视图已关闭；采集由工作台管理。")
        return result

    def start_reference(self, now_ns=None):
        if not self.is_connected:
            raise ValueError("需要新鲜触觉数据才能采集无载参考。")
        self._model.start_reference(now_ns=now_ns)
        self._pending_save = True
        self._persistence_error = ""

    def configure_response(self, *, display_range_adc=None, direction=None):
        self._model.configure_response(display_range_adc=display_range_adc, direction=direction)
        if self._store is None or self._reference_profile is None:
            return
        self._reference_profile = replace(self._reference_profile, settings=self.response_settings)
        try:
            if isinstance(self._reference_profile, TactileDeviceProfile):
                self._store.save(self._reference_profile)
            elif isinstance(self._reference_profile, TactileDefaultProfile):
                self._store.save_default_settings(self.response_settings)
        except (OSError, ValueError) as exc:
            self._diagnostics.append(f"settings_save: {exc}")
            self._persistence_error = "显示设置未能保存；当前设置仍有效。"

    def disconnect(self):
        if self._disconnect is None:
            raise RuntimeError("Stop tactile acquisition from the workstation devices page")
        result = self._disconnect()
        if result is False:
            raise RuntimeError("Managed tactile stop request was rejected")
        self.set_connection_state("disconnecting")
        return result

    def close(self):
        """Release this shared view only; do not stop or steal managed hardware."""
        self._closed = True
        if self._receiver is not None:
            self._receiver.close()
            self._receiver = None
        self._model.invalidate("已关闭", "显示已关闭", reset_reference=True)
        self._connection_state = "disconnected"
        return True
