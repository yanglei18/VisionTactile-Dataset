"""Independent PySide6 hand/touch UI; device and solver ownership stay outside Qt."""
from __future__ import annotations

import json
import math

from PySide6 import QtCore, QtWidgets

from vt_capture_console.ui.widgets.tactile_palette import heat_stops
from vt_capture_console.ui.widgets.tactile_surface_colors import surface_colors
from vt_fsglove12 import host_clock
from vt_fsglove12.tactile_surface import TactileSurfaceBinding

from .composition import DisplayComposer
from .port_identity import port_identity
from .port_metadata import list_port_candidates
from .serial_discovery import SerialPortDiscovery
from .translations import COMPOSITION_STATUS, text


STEPS = ("neutral", "fingers_down", "thumb_up")


def _set_text(widget, value):
    if widget.text() != value:
        widget.setText(value)


def _command_receipt(runtime, pending):
    """Only completed execution or explicit queue rejection releases a click latch."""
    if runtime.get("command_rejected_count", 0) > pending["rejected"] and runtime.get("last_rejected_command_name") == pending["name"]:
        return "rejected", runtime.get("last_rejected_command_error")
    if runtime.get("command_completed_generation", 0) > pending["generation"] and runtime.get("last_command_name") == pending["name"]:
        return "completed", runtime.get("last_command_error")
    return None, None


class HandTactileWindow(QtWidgets.QMainWindow):
    def __init__(self, settings, resources, pose_controller, tactile_source, *, composer=None,
                 view_factory=None, port_provider=None, port_discovery=None):
        super().__init__()
        self.settings, self.resources = settings, resources
        self.pose_controller, self.tactile_source = pose_controller, tactile_source
        self.composer = composer if composer is not None else DisplayComposer(resources["neutral_geometry"])
        self.port_discovery = (port_discovery if port_discovery is not None else
                               SerialPortDiscovery(port_provider or list_port_candidates))
        self._port_scan_error = ""
        self._port_identifying = False
        self._port_roles = {}
        self._applied_port_choices = {}
        self.language = settings.language
        self._bindings = []
        self._snapshot = {"runtime": {"connected": False}, "calibration": {}}
        self._touch_state = self._combined = None
        self._action_error = self._error_detail = ""
        self._imu_port_owned = ""
        self._pose_request = None
        self._pose_receipt = None
        self._pending_calibration = None
        self._step = STEPS[0]
        self._started = self._closing = self._closed = False
        self._last_geometry_key = None
        self._response_settings_seen = None
        self._details_updated_ns = None
        self.resize(1060, 900)
        self.setMinimumSize(700, 700)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(22, 18, 22, 16)
        layout.setSpacing(10)
        header = QtWidgets.QHBoxLayout()
        self.title = self._label("title", "title")
        header.addWidget(self.title)
        header.addStretch()
        self.language_combo = QtWidgets.QComboBox()
        self.language_combo.addItem("中文", "zh_CN")
        self.language_combo.addItem("English", "en")
        self.language_combo.setCurrentIndex(self.language_combo.findData(self.language))
        self.language_combo.activated.connect(lambda index: self.set_language(self.language_combo.itemData(index)))
        header.addWidget(self.language_combo)
        layout.addLayout(header)
        layout.addWidget(self._label("subtitle", "muted"))
        self.port_combos, self.connection_buttons, self.connection_statuses = {}, {}, {}
        for kind in ("imu", "tactile"):
            row = QtWidgets.QHBoxLayout()
            row.addWidget(self._label(kind + "_port"))
            combo = QtWidgets.QComboBox()
            combo.setEditable(True)
            combo.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
            combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            combo.setMinimumContentsLength(18)
            combo.addItem(self.tr("choose_port"), "")
            editor = combo.lineEdit()
            editor.editingFinished.connect(lambda edit=editor: edit.setModified(False))
            combo.activated.connect(lambda _index, edit=editor: edit.setModified(False))
            self.port_combos[kind] = combo
            row.addWidget(combo, 1)
            status = QtWidgets.QLabel()
            status.setObjectName("muted")
            status.setMinimumWidth(90)
            self.connection_statuses[kind] = status
            row.addWidget(status)
            button = QtWidgets.QPushButton()
            button.clicked.connect(lambda checked=False, channel=kind: self.toggle_connection(channel))
            self.connection_buttons[kind] = button
            row.addWidget(button)
            layout.addLayout(row)
        self.refresh_ports_button = self._button("refresh", self.refresh_ports)
        self.identify_ports_button = self._button("identify_ports", self.identify_ports)
        self.calibration_toggle = self._button("calibration")
        self.calibration_toggle.setCheckable(True)
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.addWidget(self.refresh_ports_button)
        toolbar.addWidget(self.identify_ports_button)
        toolbar.addWidget(self.calibration_toggle)
        toolbar.addStretch()
        self.mode_label = QtWidgets.QLabel()
        self.mode_label.setObjectName("topBadge")
        toolbar.addWidget(self.mode_label)
        layout.addLayout(toolbar)
        self.calibration_panel = self._make_calibration()
        self.calibration_panel.hide()
        self.calibration_toggle.toggled.connect(self.calibration_panel.setVisible)
        layout.addWidget(self.calibration_panel)
        view_toolbar = QtWidgets.QHBoxLayout()
        for label, palmar in (("palmar", True), ("dorsal", False), ("reset_view", True)):
            view_toolbar.addWidget(self._button(label, lambda checked=False, side=palmar: self.view.reset_camera(side)))
        self.joints_button = self._button("joints")
        self.joints_button.setCheckable(True)
        view_toolbar.addWidget(self.joints_button)
        view_toolbar.addStretch()
        layout.addLayout(view_toolbar)
        if view_factory is None:
            from vt_fsglove12.hand_view import HandView
            view_factory = HandView
        self.view = view_factory(resources["neutral_geometry"], resources["model_axes"], central)
        self.view.setMinimumSize(460, 300)
        self.view.set_show_joints(False)
        self.view.reset_camera(True)
        self.joints_button.toggled.connect(self.view.set_show_joints)
        self._binding = TactileSurfaceBinding(resources["neutral_geometry"], resources["model_axes"], hand="right")
        layout.addWidget(self.view, 1)
        self.metrics = QtWidgets.QLabel()
        self.metrics.setObjectName("muted")
        layout.addWidget(self.metrics)
        reference_row = QtWidgets.QHBoxLayout()
        self.reference_button = self._button("unloaded", self.start_reference)
        reference_row.addWidget(self.reference_button)
        self.reference_status = QtWidgets.QLabel()
        self.reference_status.setObjectName("muted")
        reference_row.addWidget(self.reference_status, 1)
        self.reference_progress = QtWidgets.QProgressBar()
        self.reference_progress.setRange(0, 100)
        self.reference_progress.setFixedSize(100, 8)
        self.reference_progress.setTextVisible(False)
        reference_row.addWidget(self.reference_progress)
        layout.addLayout(reference_row)
        bottom = QtWidgets.QHBoxLayout()
        self.options_toggle = self._button("options")
        self.options_toggle.setCheckable(True)
        bottom.addWidget(self.options_toggle)
        self.details_toggle = self._button("details")
        self.details_toggle.setCheckable(True)
        bottom.addWidget(self.details_toggle)
        bottom.addStretch()
        layout.addLayout(bottom)
        self.options_panel = self._make_options()
        self.options_panel.hide()
        self.options_toggle.toggled.connect(self.options_panel.setVisible)
        layout.addWidget(self.options_panel)
        self.notice = QtWidgets.QLabel()
        self.notice.setObjectName("operationNotice")
        self.notice.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.notice.setWordWrap(True)
        layout.addWidget(self.notice)
        self.details = QtWidgets.QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setMaximumBlockCount(150)
        self.details.setMaximumHeight(150)
        self.details.hide()
        self.details_toggle.toggled.connect(self.details.setVisible)
        self.details_toggle.toggled.connect(lambda visible: self._update_details(force=True) if visible else None)
        layout.addWidget(self.details)
        layout.addWidget(self._label("note", "muted"))
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self.refresh)
        for kind, combo in self.port_combos.items():
            selected = str(settings.get(kind + "_port", ""))
            if selected:
                combo.setEditText(selected)
        self._translate_static()
        self._update_controls()

    def tr(self, key, **values):
        return text(key, self.language, **values)

    def _label(self, key, object_name=""):
        label = QtWidgets.QLabel(self.tr(key))
        label.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        if object_name:
            label.setObjectName(object_name)
        self._bindings.append((label, key))
        return label

    def _button(self, key, callback=None):
        button = QtWidgets.QPushButton(self.tr(key))
        button.setAutoDefault(False)
        self._bindings.append((button, key))
        if callback is not None:
            button.clicked.connect(callback)
        return button

    def _make_calibration(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 2, 0, 4)
        row = QtWidgets.QHBoxLayout()
        self.step_buttons = {}
        for step in STEPS:
            button = QtWidgets.QPushButton()
            button.setCheckable(True)
            button.clicked.connect(lambda checked=False, value=step: self.select_step(value))
            self.step_buttons[step] = button
            row.addWidget(button)
        layout.addLayout(row)
        self.instructions = QtWidgets.QLabel()
        self.instructions.setWordWrap(True)
        layout.addWidget(self.instructions)
        buttons = QtWidgets.QHBoxLayout()
        self.capture_button = self._button("capture", lambda: self.send_calibration("capture", step=self._step))
        self.cancel_button = self._button("cancel", lambda: self.send_calibration("cancel_capture"))
        self.clear_button = self._button("clear", lambda: self.send_calibration("clear_calibration"))
        for button in (self.capture_button, self.cancel_button, self.clear_button):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.calibration_status = QtWidgets.QLabel()
        self.calibration_status.setWordWrap(True)
        layout.addWidget(self.calibration_status)
        self.functional_button = self._button("functional", lambda: self.send_calibration("capture_functional_hand"))
        layout.addWidget(self.functional_button)
        self.functional_status = QtWidgets.QLabel()
        self.functional_status.setObjectName("muted")
        self.functional_status.setWordWrap(True)
        layout.addWidget(self.functional_status)
        return panel

    def _make_options(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        self.response_range = QtWidgets.QDoubleSpinBox()
        self.response_range.setRange(1., 65535.)
        self.response_range.setDecimals(0)
        self.response_range.setValue(self.tactile_source.response_settings.display_range_adc)
        self.response_range.editingFinished.connect(self.configure_response)
        self.response_direction = QtWidgets.QComboBox()
        for value in ("both", "increase", "decrease"):
            self.response_direction.addItem(self.tr(value), value)
        self.response_direction.activated.connect(self.configure_response)
        self.tactile_baudrate = QtWidgets.QSpinBox()
        self.tactile_baudrate.setRange(1, 4_000_000)
        self.tactile_baudrate.setValue(int(self.settings.get("tactile_baudrate", 115200)))
        layout.addRow(self._label("range"), self.response_range)
        layout.addRow(self._label("direction"), self.response_direction)
        layout.addRow(self._label("baudrate"), self.tactile_baudrate)
        return panel

    def _translate_static(self):
        self.setWindowTitle(self.tr("title"))
        for widget, key in self._bindings:
            _set_text(widget, self.tr(key))
        self.language_combo.setAccessibleName(self.tr("language"))
        self.reference_button.setToolTip(self.tr("zero_help"))
        self.functional_button.setToolTip(self.tr("functional_help"))
        self.identify_ports_button.setToolTip(self.tr("identify_help"))
        for kind, combo in self.port_combos.items():
            combo.setAccessibleName(self.tr(kind + "_port"))
            if combo.count() and combo.itemData(0) == "":
                combo.setItemText(0, self.tr("choose_port"))
        for index in range(self.response_direction.count()):
            self.response_direction.setItemText(index, self.tr(self.response_direction.itemData(index)))

    def set_language(self, language):
        if language not in {"zh_CN", "en"}:
            return
        try:
            self.settings.save_language(language)
        except (OSError, ValueError) as exc:
            self._report_error(exc)
            return
        self.language = language
        with QtCore.QSignalBlocker(self.language_combo):
            self.language_combo.setCurrentIndex(self.language_combo.findData(language))
        self._translate_static()
        # Retranslation uses existing presentation state; never polls or resets devices.
        self._sync_ports()
        self._update_controls()

    def selected_port(self, kind):
        combo = self.port_combos[kind]
        index = combo.currentIndex()
        if index >= 0 and combo.currentText() == combo.itemText(index):
            return combo.itemData(index) or ""
        return combo.currentText().strip()

    def refresh_ports(self):
        """Request a scan; enumeration never runs on the rendering thread."""
        if not self._closing:
            self.port_discovery.refresh()

    def identify_ports(self):
        if self._closing:
            return
        # Keep ports reserved while connect/disconnect commands are in flight.
        excluded = [self._imu_port_owned]
        if self.tactile_source.can_disconnect:
            excluded.append(self.tactile_source.port)
        runtime = self._snapshot.get("runtime", {})
        if runtime.get("connected"):
            excluded.append(runtime.get("port", ""))
        rates = tuple(dict.fromkeys((115200, self.tactile_baudrate.value())))
        self.port_discovery.identify(excluded=excluded, baudrates=rates)
        self._sync_ports()
        self._update_controls()

    def _sync_ports(self):
        snapshot = self.port_discovery.latest()
        if snapshot is None:
            return
        self._port_scan_error = snapshot.error or ""
        self._port_identifying = snapshot.identifying
        classifications = {item.path: item for item in snapshot.classifications}
        identities = dict(snapshot.identities)
        self._port_roles = {port_identity(path): item.role for path, item in classifications.items()
                            if identities.get(path) is not None}
        choices = []
        for path, label in snapshot.choices:
            item = classifications.get(path)
            role = item.role if item else "unknown"
            role_label = self.tr("port_role_" + role)
            if role in {"imu", "tactile"} and identities.get(path) is None:
                role_label = self.tr("port_role_temporary", role=role_label)
            choices.append((path, role_label + " · " + label))
        choices = tuple(choices)
        for kind, combo in self.port_combos.items():
            if self._applied_port_choices.get(kind) == choices:
                continue
            editor = combo.lineEdit()
            # Apply the latest snapshot after a selection or edit finishes;
            # rebuilding an open popup would move the item under the pointer.
            if combo.view().isVisible() or (editor.hasFocus() and editor.isModified()):
                continue
            selected = self.selected_port(kind)
            cursor, selection = editor.cursorPosition(), editor.selectionStart()
            selection_length = len(editor.selectedText())
            with QtCore.QSignalBlocker(combo), QtCore.QSignalBlocker(editor):
                combo.clear()
                combo.addItem(self.tr("choose_port"), "")
                for path, label in choices:
                    combo.addItem(label, path)
                    combo.setItemData(combo.count() - 1, self.tr("identify_help"), QtCore.Qt.ItemDataRole.ToolTipRole)
                index = combo.findData(selected)
                if index >= 0:
                    combo.setCurrentIndex(index)
                else:
                    # A missing or manually entered path remains the explicit
                    # choice, but is not listed as a currently detected port.
                    combo.setCurrentIndex(-1)
                    combo.setEditText(selected)
                    editor.setCursorPosition(min(cursor, len(selected)))
                    if selection >= 0:
                        editor.setSelection(selection, selection_length)
            self._applied_port_choices[kind] = choices

    def _report_error(self, error, key="action_error"):
        self._action_error, self._error_detail = key, str(error)

    def _submit(self, name, **args):
        if not self.pose_controller.submit(name, **args):
            self._action_error = "queue_busy"
            return False
        return True

    def _pending(self, name):
        runtime = self._snapshot.get("runtime", {})
        return dict(name=name, generation=runtime.get("command_completed_generation", 0),
                    rejected=runtime.get("command_rejected_count", 0),
                    submitted_ns=host_clock.monotonic_ns(), session_id=runtime.get("session_id"))

    def toggle_connection(self, kind):
        if self._closing:
            return
        self._action_error = self._error_detail = ""
        try:
            if kind == "imu" and (self._snapshot.get("runtime", {}).get("connected") or self._pose_request == "connect"):
                if self._submit("disconnect"):
                    self._pose_request = "disconnect"
                    self._pose_receipt = self._pending("disconnect")
            elif kind == "tactile" and self.tactile_source.can_disconnect:
                self.tactile_source.disconnect()
            else:
                # Also guard direct calls, not only disabled buttons. Probes own
                # their serial handles until the worker reports completion.
                snapshot = self.port_discovery.latest()
                if snapshot is not None and snapshot.identifying:
                    self._action_error = "identify_wait"
                    self._update_controls()
                    return
                port = self.selected_port(kind)
                if not port:
                    self._action_error = "missing_port"
                else:
                    other = (self.tactile_source.port if self.tactile_source.can_disconnect else "") if kind == "imu" else self._imu_port_owned
                    if other and port_identity(port) == port_identity(other):
                        self._action_error = "same_port"
                    elif self._port_roles.get(port_identity(port)) in ({"imu", "tactile"} - {kind}):
                        self._action_error = "port_type_mismatch_" + kind
                    elif kind == "imu":
                        if self._submit("connect", port=port):
                            self._imu_port_owned, self._pose_request = port, "connect"
                            self._pose_receipt = self._pending("connect")
                            self.settings.set("imu_port", port)
                    else:
                        self.tactile_source.connect(port, self.tactile_baudrate.value())
                        self.settings.set("tactile_port", port)
                        self.settings.set("tactile_baudrate", self.tactile_baudrate.value())
        except (OSError, ValueError, RuntimeError) as exc:
            self._report_error(exc)
        self.refresh()

    def select_step(self, step):
        self._step = step
        self._update_controls()

    def send_calibration(self, name, **args):
        if self._closing:
            return
        self._action_error = self._error_detail = ""
        try:
            if self._submit(name, **args):
                self._pending_calibration = self._pending(name)
        except (ValueError, RuntimeError) as exc:
            self._report_error(exc)
        self.refresh()

    def start_reference(self):
        self._action_error = self._error_detail = ""
        try:
            self.tactile_source.start_reference()
        except (ValueError, RuntimeError) as exc:
            self._report_error(exc)
        self.refresh()

    def configure_response(self, *_):
        settings = self.tactile_source.response_settings
        value, direction = self.response_range.value(), self.response_direction.currentData()
        if (value, direction) == (settings.display_range_adc, settings.direction):
            return
        try:
            self.tactile_source.configure_response(display_range_adc=value, direction=direction)
        except (OSError, ValueError) as exc:
            self._report_error(exc)
        self.refresh()

    def start(self):
        if self._started or self._closing:
            return
        self.pose_controller.start()
        self.tactile_source.start()
        self.port_discovery.start()
        self._started = True
        self.timer.start()
        self.refresh()

    def refresh(self):
        if self._closed:
            return
        if self._closing:
            self._finish_shutdown()
            return
        try:
            self._sync_ports()
            self._snapshot = self.pose_controller.latest_snapshot()
            self._touch_state = self.tactile_source.poll()
            now = host_clock.monotonic_ns()
            runtime = self._snapshot.get("runtime", {})
            if runtime.get("connected"):
                self._imu_port_owned = runtime.get("port") or self._imu_port_owned
                if self._pose_request == "connect":
                    self._pose_request = self._pose_receipt = None
            if self._pose_receipt is not None:
                receipt, error = _command_receipt(runtime, self._pose_receipt)
                if receipt == "rejected" or error or (receipt == "completed" and self._pose_request == "disconnect"):
                    self._pose_request, self._pose_receipt, self._imu_port_owned = None, None, ""
                    if error:
                        self._report_error(error)
                elif receipt == "completed" and runtime.get("error") and not runtime.get("connected"):
                    # Connection setup can complete before the reader opens the device.
                    self._pose_request, self._pose_receipt, self._imu_port_owned = None, None, ""
            elif not runtime.get("connected") and runtime.get("error"):
                self._imu_port_owned = ""
            self._combined = self.composer.compose(self._snapshot, self._touch_state, now_ns=now)
            combined = self._combined
            if combined.geometry is None:
                self.view.set_hand_visible(False)
                self._last_geometry_key = None
            else:
                pose = self._snapshot.get("pose") or {}
                key = (combined.mode, runtime.get("session_id"), pose.get("profile_version"),
                       pose.get("timestamp_s"), runtime.get("ipc_sequence"))
                if key != self._last_geometry_key:
                    self.view.set_geometry(combined.geometry)
                    self._last_geometry_key = key
            frame = combined.tactile_frame
            colors = (surface_colors(self._binding, frame, self.view.colors,
                                     frame["levels"], heat_stops("tactile_response")) if frame else None)
            self.view.set_surface_colors(colors)
            self._update_controls(now)
        except (AttributeError, OSError, ValueError, TypeError, RuntimeError) as exc:
            self.view.set_surface_colors(None)
            self.view.set_hand_visible(False)
            self._last_geometry_key = None
            self._report_error(exc)
            self._update_controls()

    def _update_controls(self, now_ns=None):
        now_ns = host_clock.monotonic_ns() if now_ns is None else now_ns
        runtime, calibration = self._snapshot.get("runtime", {}), self._snapshot.get("calibration", {})
        connected = bool(runtime.get("connected"))
        imu_status = ("connecting" if self._pose_request == "connect" else
                      "disconnecting" if self._pose_request == "disconnect" else
                      "failed" if runtime.get("error") else "connected" if connected else "disconnected")
        touch_status = self.tactile_source.connection_state
        touch_status = "failed" if touch_status == "error" else touch_status
        if touch_status not in {"connected", "connecting", "disconnecting", "failed", "disconnected"}:
            touch_status = "disconnected"
        for kind, status, active in (("imu", imu_status, connected or self._pose_request == "connect"),
                                     ("tactile", touch_status, self.tactile_source.can_disconnect)):
            _set_text(self.connection_statuses[kind], self.tr(status))
            _set_text(self.connection_buttons[kind], self.tr("disconnect" if active else "connect"))
            self.connection_buttons[kind].setEnabled(not self._closing and status != "disconnecting"
                                                      and (active or not self._port_identifying))
            self.port_combos[kind].setEnabled(not self._closing and not active and status != "disconnecting")
        self.tactile_baudrate.setEnabled(not self.tactile_source.can_disconnect and not self._closing and not self._port_identifying)
        _set_text(self.identify_ports_button, self.tr("identifying_ports" if self._port_identifying else "identify_ports"))
        self.identify_ports_button.setEnabled(not self._closing and self._started and not self._port_identifying)
        self._update_calibration(runtime, calibration, now_ns)
        self._update_reference()
        self._update_display_status(runtime, calibration, touch_status)

    def _update_calibration(self, runtime, calibration, now_ns):
        connected = bool(runtime.get("connected"))
        active = calibration.get("active") or {}
        profile_status = str(calibration.get("profile_status", ""))
        busy = bool(active) or profile_status.startswith("fitting") or profile_status == "clearing"
        if self._pending_calibration:
            receipt, error = _command_receipt(runtime, self._pending_calibration)
            if receipt or not connected or runtime.get("session_id") != self._pending_calibration["session_id"]:
                self._pending_calibration = None
                if error:
                    self._report_error(error)
            elif now_ns - self._pending_calibration["submitted_ns"] > 5_000_000_000:
                self._action_error = "pending_timeout"
        pending = self._pending_calibration is not None
        captured = calibration.get("captured_steps", [])
        for step, button in self.step_buttons.items():
            _set_text(button, ("✓ " if step in captured else "") + self.tr(step))
            button.setChecked(step == self._step)
            button.setEnabled(not busy and not pending and not self._closing)
        _set_text(self.instructions, self.tr(self._step + "_help"))
        self.capture_button.setEnabled(connected and not busy and not pending and not self._closing)
        self.clear_button.setEnabled(connected and not busy and not pending and not self._closing)
        self.cancel_button.setEnabled(connected and (busy or pending) and not self._closing)
        self.functional_button.setEnabled(connected and bool(calibration.get("calibrated_nodes"))
                                           and not busy and not pending and not self._closing)
        if active:
            phase = active.get("phase", "collecting")
            phase = phase if phase in {"preparing", "collecting", "collecting_neutral", "moving"} else "collecting"
            remaining = active.get("remaining_s", 0.)
            seconds = max(0, math.ceil(remaining)) if isinstance(remaining, (int, float)) and math.isfinite(remaining) else 0
            calibration_text = self.tr(phase, seconds=seconds)
        elif profile_status.startswith("fitting"):
            calibration_text = self.tr("fitting")
        elif profile_status == "clearing":
            calibration_text = self.tr("clearing")
        elif pending:
            calibration_text = self.tr("pending")
        elif profile_status in {"fit_failed", "capture_failed"}:
            calibration_text = self.tr("calibration_failed")
        elif calibration.get("calibrated_nodes"):
            calibration_text = self.tr("calibration_partial" if calibration.get("rejected_nodes") else "calibration_ready")
        else:
            calibration_text = self.tr("calibration_steps", count=len(captured)) if captured else self.tr("calibration_idle")
        _set_text(self.calibration_status, calibration_text)
        functional = (calibration.get("functional_hand_result") or {}).get("status")
        functional_text = self.tr("functional_" + functional) if functional in {"complete", "partial", "failed", "cancelled"} else ""
        _set_text(self.functional_status, functional_text)
        self.functional_status.setVisible(bool(functional_text))

    def _update_reference(self):
        reference = self.tactile_source.response_status
        collecting = reference.phase == "collecting"
        self.reference_button.setEnabled(self.tactile_source.is_connected and self._touch_state is not None
                                         and self._touch_state.frame is not None and not collecting and not self._closing)
        if reference.phase == "ready":
            origin = reference.reference_source if reference.reference_source in {"default", "preset", "saved", "measured"} else "measured"
            reference_text = self.tr("reference_" + origin, count=reference.reference_cells)
        elif collecting:
            reference_text = self.tr("reference_collecting", percent=round(reference.progress * 100))
        else:
            reference_text = self.tr("reference_failed" if reference.phase == "failed" else "reference_missing")
        _set_text(self.reference_status, reference_text)
        self.reference_progress.setVisible(collecting)
        self.reference_progress.setValue(round(reference.progress * 100))
        settings = self.tactile_source.response_settings
        values = (settings.display_range_adc, settings.direction)
        if values != self._response_settings_seen and not self.response_range.hasFocus():
            with QtCore.QSignalBlocker(self.response_range), QtCore.QSignalBlocker(self.response_direction):
                self.response_range.setValue(values[0])
                self.response_direction.setCurrentIndex(self.response_direction.findData(values[1]))
            self._response_settings_seen = values
        self.response_range.setEnabled(not collecting and not self._closing)
        self.response_direction.setEnabled(not collecting and not self._closing)

    def _update_display_status(self, runtime, calibration, touch_status):
        combined = self._combined
        mode = combined.mode if combined else "waiting"
        mode_key = COMPOSITION_STATUS.get(combined.status, mode) if combined and mode != "reference" else mode
        _set_text(self.mode_label, self.tr(mode_key))
        def age(value):
            return f"{value:.0f} ms" if isinstance(value, (int, float)) and math.isfinite(value) else "—"
        _set_text(self.metrics, self.tr("metrics", pose=age(combined.pose_age_ms if combined else None),
                                      touch=age(combined.tactile_age_ms if combined else None),
                                      skew=age(combined.skew_ms if combined else None)))
        storage_error = self.tactile_source.reference_persistence_error or (runtime.get("calibration_storage") or {}).get("error")
        issue = self._action_error or ("storage_error" if storage_error else "backend_error" if runtime.get("error") or touch_status == "failed" else "port_scan_error" if self._port_scan_error else "")
        _set_text(self.notice, self.tr("closing") if self._closing else self.tr(issue) if issue else "")
        self.notice.setVisible(self._closing or bool(issue))
        self._update_details(force=bool(issue))

    def _update_details(self, *, force=False):
        now = host_clock.monotonic_ns()
        if not force and (self.details.isHidden() or (self._details_updated_ns is not None
                and 0 <= now - self._details_updated_ns < 200_000_000)):
            return
        self._details_updated_ns = now
        runtime, calibration, combined = self._snapshot.get("runtime", {}), self._snapshot.get("calibration", {}), self._combined
        details = dict(motion={key: runtime.get(key) for key in ("message", "error", "received_samples", "last_sample_age_s", "calibration_storage")},
                       calibration=dict(profile_status=calibration.get("profile_status", ""), rejected_nodes=calibration.get("rejected_nodes", {}), functional_result=calibration.get("functional_hand_result")),
                       tactile=dict(status=self._touch_state.status if self._touch_state else "", detail=self._touch_state.detail if self._touch_state else "", persistence_error=self.tactile_source.reference_persistence_error),
                       display=combined.status if combined else "", action=self._error_detail,
                       port_discovery=self._port_scan_error,
                       port_types=[dict(path=item.path, role=item.role, detail=item.detail)
                                   for item in self.port_discovery.latest().classifications]
                                  if self.port_discovery.latest() is not None else [])
        formatted = json.dumps(details, ensure_ascii=False, indent=2, default=str)[:16000]
        if self.details.toPlainText() != formatted:
            self.details.setPlainText(formatted)

    def shutdown(self):
        """Request closure once; the timer completes resource release without blocking."""
        if self._closed:
            return True
        if not self._closing:
            self._closing = True
            self.port_discovery.stop()
            self.pose_controller.stop()
            self.tactile_source.disconnect()
            self.centralWidget().setEnabled(False)
            self._update_controls()
            self.timer.start()
        return self._finish_shutdown()

    def _finish_shutdown(self):
        self.tactile_source.poll()
        pose_done = self.pose_controller.join(timeout=0)
        touch_done = self.tactile_source.can_connect
        discovery_done = self.port_discovery.join(timeout=0)
        if pose_done and touch_done and discovery_done:
            self.tactile_source.close()
            self._closed = True
            self.timer.stop()
            self.close()
            return True
        return False

    def closeEvent(self, event):
        if self._closed or self.shutdown():
            event.accept()
        else:
            event.ignore()
