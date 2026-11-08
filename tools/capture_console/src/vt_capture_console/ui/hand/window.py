"""Independent PySide6 hand/touch UI; device and solver ownership stay outside Qt."""
from __future__ import annotations

import json
import math
from functools import partial

from PySide6 import QtCore, QtWidgets

from vt_capture_console.ui.widgets.tactile_surface_colors import surface_colors
from vt_fsglove12.config import SLOT_NAMES
from vt_fsglove12 import host_clock
from vt_fsglove12.tactile_surface import TactileSurfaceBinding
from vt_fsglove12.upstream_functional import FINGER_SLOTS

from .composition import DisplayComposer
from .palm_guide import PalmGuide
from ...backend.hand_ports.port_identity import port_identity
from ...backend.hand_ports.port_metadata import list_port_candidates
from .progress_button import ProgressButton
from ...backend.hand_ports.serial_discovery import SerialPortDiscovery
from .translations import COMPOSITION_STATUS, text


STEPS = ("neutral", "fingers_down", "thumb_up")
CAPTURE_PHASES = {"preparing", "collecting", "collecting_neutral", "moving",
                  "moving_fingers", "preparing_thumb", "moving_thumb"}


PRESSURE_STOPS = ((.22, (251, 232, 165)), (.65, (244, 146, 64)), (1., (195, 42, 42)))
REFERENCE_MISSING_RGB = (229, 232, 237)
MEASURED_MISSING_RGB = (72, 101, 131)


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


class HandPanel(QtWidgets.QWidget):
    def __init__(self, settings, resources, pose_controller, tactile_source, *, composer=None,
                 view_factory=None, port_provider=None, port_discovery=None, demo_factory=None,
                 parent=None, owns_session=False, connection_request=None, connection_status=None):
        super().__init__(parent)
        self.owns_session = bool(owns_session)
        self.connection_request = connection_request
        self.connection_status = connection_status
        self.settings, self.resources = settings, resources
        self.hand = resources.get("hand", "right")
        self.pose_controller, self.tactile_source = pose_controller, tactile_source
        self.composer = composer if composer is not None else DisplayComposer(
            resources["neutral_geometry"], hand=self.hand, layout_path=resources.get("layout_path"))
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
        self.palm_guide = PalmGuide(resources)
        self._palm_preview = None
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
        central = self
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
        self.calibration_toggle.toggled.connect(self.calibration_panel.setVisible)
        self.calibration_toggle.setChecked(True)
        layout.addWidget(self.calibration_panel)
        view_toolbar = QtWidgets.QHBoxLayout()
        for label, palmar in (("palmar", True), ("dorsal", False), ("reset_view", True)):
            view_toolbar.addWidget(self._button(label, lambda checked=False, side=palmar: self._reset_views(side)))
        self.joints_button = self._button("joints")
        self.joints_button.setCheckable(True)
        view_toolbar.addWidget(self.joints_button)
        view_toolbar.addStretch()
        layout.addLayout(view_toolbar)
        if view_factory is None:
            from .display_view import DisplayHandView
            view_factory = DisplayHandView
        self.view = view_factory(resources["neutral_geometry"], resources["model_axes"], central)
        # The tracked hand uses the same anatomical material as the demo.
        self._reference_base = self.view.colors.copy()
        self._reference_base[:, :3] = 184 / 255.
        self.view.set_surface_colors(self._reference_base)
        self.view.setMinimumSize(240, 245)
        self.view.set_show_joints(False)
        self.joints_button.toggled.connect(self.view.set_show_joints)
        self._binding = TactileSurfaceBinding(resources["neutral_geometry"], resources["model_axes"],
                                              hand=self.hand, layout_path=resources.get("layout_path"))
        if demo_factory is None:
            from .mounting_demo import MountingDemo
            demo_factory = partial(MountingDemo, mano_assets=resources["mano_assets"],
                                   demo_clips=resources.get("demo_clips"))
        self.demo_card = QtWidgets.QWidget()
        demo_layout = QtWidgets.QVBoxLayout(self.demo_card)
        demo_layout.setContentsMargins(0, 0, 0, 0)
        self.demo_title = QtWidgets.QLabel()
        demo_layout.addWidget(self.demo_title)
        self.demo = demo_factory(resources["neutral_geometry"], resources["model_axes"], self.demo_card)
        # The display basis keeps fingertips along +Y for both models. Mirror
        # the observer across that axis so the thumb faces both cameras.
        azimuth = -20 if self.hand == "left" else -160
        for panel in (self.demo, self.view):
            panel.setCameraPosition(elevation=25, azimuth=azimuth, distance=.47)
        self.demo.setMinimumSize(240, 245)
        demo_layout.addWidget(self.demo, 1)
        live_card = QtWidgets.QWidget()
        live_layout = QtWidgets.QVBoxLayout(live_card)
        live_layout.setContentsMargins(0, 0, 0, 0)
        self.live_title = QtWidgets.QLabel()
        live_layout.addWidget(self.live_title)
        live_layout.addWidget(self.view, 1)
        comparison = QtWidgets.QHBoxLayout()
        comparison.addWidget(self.demo_card, 1)
        comparison.addWidget(live_card, 1)
        layout.addLayout(comparison, 1)
        self.calibration_toggle.toggled.connect(self.demo_card.setVisible)
        self.calibration_toggle.toggled.connect(self.live_title.setVisible)
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
        # Reserve the space of a joint notice, including wrapping at narrow
        # widths, so a status change cannot resize either 3D viewport.
        notice_row = QtWidgets.QGridLayout()
        self.notice, self._notice_space = QtWidgets.QLabel(), QtWidgets.QLabel()
        for label in (self.notice, self._notice_space):
            label.setObjectName("operationNotice")
            label.setTextFormat(QtCore.Qt.TextFormat.PlainText)
            label.setWordWrap(True)
            notice_row.addWidget(label, 0, 0)
        notice_policy = self._notice_space.sizePolicy()
        notice_policy.setRetainSizeWhenHidden(True)
        self._notice_space.setSizePolicy(notice_policy)
        self._notice_space.hide()
        layout.addLayout(notice_row)
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
        if not self.owns_session:
            self.refresh_ports_button.hide()
            self.identify_ports_button.hide()
            self.tactile_baudrate.setEnabled(False)
            for combo in self.port_combos.values():
                combo.setEnabled(False)
        self._translate_static()
        self._update_controls()

    def tr(self, key, **values):
        return text(key, self.language, hand=self.hand, **values)

    def _reset_views(self, palmar):
        self.view.reset_camera(palmar)
        self.demo.reset_camera(palmar)

    def _label(self, key, object_name=""):
        label = QtWidgets.QLabel(self.tr(key))
        label.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        if object_name:
            label.setObjectName(object_name)
        self._bindings.append((label, key))
        return label

    def _button(self, key, callback=None, *, progress=False):
        button = (ProgressButton if progress else QtWidgets.QPushButton)(self.tr(key))
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
            button = ProgressButton()
            button.clicked.connect(lambda checked=False, value=step: self.capture_step(value))
            self.step_buttons[step] = button
            row.addWidget(button)
        layout.addLayout(row)
        progress_row = QtWidgets.QHBoxLayout()
        progress_row.addWidget(self._label("motion_progress"))
        self.calibration_progress = QtWidgets.QProgressBar()
        self.calibration_progress.setStyleSheet(
            "QProgressBar {min-height: 18px; max-height: 18px; border: 1px solid #60666d;"
            " border-radius: 3px; background: #262a2e; color: white; text-align: center;}"
            "QProgressBar::chunk {background: #257281;}")
        progress_row.addWidget(self.calibration_progress, 1)
        layout.addLayout(progress_row)
        self.instructions = QtWidgets.QLabel()
        self.instructions.setWordWrap(True)
        layout.addWidget(self.instructions)
        buttons = QtWidgets.QHBoxLayout()
        self.cancel_button = self._button("cancel", lambda: self.send_calibration("cancel_capture"))
        self.clear_button = self._button("clear", lambda: self.send_calibration("clear_calibration"))
        self.heading_button = self._button("heading_reference", lambda: self.send_calibration("capture_heading_reference"),
                                           progress=True)
        for button in (self.cancel_button, self.heading_button, self.clear_button):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.heading_status = QtWidgets.QLabel()
        self.heading_status.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.heading_status.setWordWrap(True)
        layout.addWidget(self.heading_status)
        self.calibration_status = QtWidgets.QLabel()
        self.calibration_status.setWordWrap(True)
        layout.addWidget(self.calibration_status)
        self.functional_button = self._button("functional", lambda: self.send_calibration("capture_functional_hand"),
                                              progress=True)
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
        nodes = ", ".join(str(node) for node in range(1, 13))
        self._notice_space.setText(self.tr("joint_realign", nodes=nodes))
        for widget, key in self._bindings:
            _set_text(widget, self.tr(key))
        self.language_combo.setAccessibleName(self.tr("language"))
        self.reference_button.setToolTip(self.tr("zero_help"))
        self.calibration_progress.setAccessibleName(self.tr("motion_progress"))
        self.functional_button.setToolTip(self.tr("functional_help"))
        self.heading_button.setToolTip(self.tr("heading_reference_help"))
        for step, button in self.step_buttons.items():
            button.setToolTip(self.tr(step + "_help"))
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
        if self.connection_request is not None and not self._closing:
            try:
                self.connection_request(kind)
            except (ValueError, RuntimeError) as exc:
                self._report_error(exc)
            self.refresh()
            return
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

    def capture_step(self, step):
        if not self.step_buttons[step].isEnabled():
            return
        self._step = step
        self.send_calibration("capture", step=step)

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
            calibration = self._snapshot.get("calibration") or {}
            mounting = (calibration.get("active") or {}).get("kind") == "mounting"
            self._palm_preview = (self.palm_guide.update(self._snapshot, now_ns=now)
                                  if self.calibration_toggle.isChecked() and not combined.uses_tracked_geometry
                                  and (mounting or not calibration.get("calibrated_nodes")) else None)
            if combined.uses_tracked_geometry:
                geometry = combined.geometry
                key = combined.geometry_key
                if key is None:
                    pose = self._snapshot.get("pose") or {}
                    key = (combined.mode, runtime.get("session_id"), pose.get("profile_version"),
                           pose.get("timestamp_s"), runtime.get("ipc_sequence"))
            elif self._palm_preview is not None and self._palm_preview.geometry is not None:
                geometry = self._palm_preview.geometry
                key = self._palm_preview.key
            else:
                geometry = self.resources["neutral_geometry"]
                key = "reference"
            if key != self._last_geometry_key:
                self.view.set_geometry(geometry)
                self._last_geometry_key = key
            self._update_surface(combined)
            self._update_controls(now)
        except (AttributeError, OSError, ValueError, TypeError, RuntimeError) as exc:
            self.view.set_surface_colors(None)
            self.view.set_hand_visible(False)
            self._last_geometry_key = None
            self._report_error(exc)
            self._update_controls()

    def _update_surface(self, combined):
        # The same decision that selects measured geometry selects its material.
        # Losing touch clears pressure highlights without changing a live hand's color.
        base = self.view.colors if combined.uses_tracked_geometry else self._reference_base
        frame = combined.tactile_frame
        colors = base
        if frame is not None:
            missing = MEASURED_MISSING_RGB if combined.uses_tracked_geometry else REFERENCE_MISSING_RGB
            colors = surface_colors(self._binding, frame, base, frame["levels"], PRESSURE_STOPS, missing)
        self.view.set_surface_colors(colors)

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
        if not connected:
            calibration = {}
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
        retry_steps = [key for key in calibration.get("retry_steps", []) if key in self.step_buttons]
        age = runtime.get("last_sample_age_s")
        stale_after = self.resources.get("config", {}).get("stale_after_s", .15)
        input_ready = (runtime.get("received_samples") != 0 and
                       ("last_sample_age_s" not in runtime or
                        (type(age) in (float, int) and 0 <= age <= stale_after)))
        can_capture = connected and input_ready and not self._closing
        # A brief display hold keeps the guide on the same step. Starting a
        # capture still requires current input through can_capture.
        guide_ready = input_ready or (self._combined is not None and self._combined.mode == "held")
        for button in self.step_buttons.values():
            button.setEnabled(can_capture and not busy and not pending)
        profile_ready = (connected and bool(calibration.get("calibrated_nodes"))
                         and not runtime.get("clear_pending")
                         and profile_status not in {"not_calibrated", "steps_pending", "fit_failed", "capture_failed"})
        slots = calibration.get("functional_slots_by_finger")
        # Old workers did not publish pair coverage. Only their full profiles
        # are known to support functional reference collection.
        if slots is None:
            slots = FINGER_SLOTS if not retry_steps and not calibration.get("rejected_nodes") else {}
        available = [name for name in FINGER_SLOTS if slots.get(name)]
        limited = [name for name, required in FINGER_SLOTS.items() if set(required) - set(slots.get(name, ()))]
        base_ready = profile_ready and bool(available)
        report = calibration.get("functional_hand_result") or {}
        hand_pending = pending and self._pending_calibration["name"] == "capture_functional_hand"
        heading_pending = pending and self._pending_calibration["name"] == "capture_heading_reference"
        heading_active = active.get("kind") == "heading_reference"
        heading_available = calibration.get("heading_reference_available") is True
        self.heading_button.setVisible(heading_available or heading_active or heading_pending)
        self.heading_button.setEnabled(heading_available and profile_ready and can_capture and not busy and not pending)
        heading_result = calibration.get("heading_reference_status") or {}
        result_text = ""
        if not busy and not pending and heading_result.get("status") in {"updated", "rejected"}:
            result_text = self.tr("heading_" + heading_result["status"])
            if heading_result["status"] == "rejected" and heading_result.get("message"):
                result_text += " " + heading_result["message"]
        _set_text(self.heading_status, result_text)
        self.heading_status.setVisible(bool(result_text))
        hand_fitting = profile_status == "fitting_functional_hand"
        if heading_pending or heading_active:
            self.demo.set_step("neutral", hold=True)
            _set_text(self.demo_title, self.tr("mounting_demo", step=self.tr("heading_reference")))
            _set_text(self.instructions, self.tr("heading_reference_help"))
        elif base_ready and not guide_ready and not busy and not pending:
            self.demo.set_step("neutral", hold=True)
            _set_text(self.demo_title, self.tr("mounting_demo", step=self.tr("finger_open")))
            _set_text(self.instructions, self.tr("calibration_waiting_data"))
        elif connected and (active.get("kind") == "functional_hand" or hand_pending or hand_fitting
                            or (base_ready and not busy and not pending)):
            self._show_finger_guide(active, report, available, pending=hand_pending, fitting=hand_fitting)
        else:
            next_step = next((step for step in STEPS if step not in captured), self._step)
            guide_step = (active.get("step") if active else self._step if pending or busy else
                          retry_steps[0] if retry_steps else next_step)
            if guide_step not in STEPS:
                guide_step = self._step
            _set_text(self.instructions, self.tr(guide_step + "_help"))
            self.demo.set_step(guide_step, hold=bool(active and active.get("phase") == "collecting"))
            _set_text(self.demo_title, self.tr("mounting_demo", step=self.tr(guide_step)))
        self.clear_button.setEnabled(connected and not busy and not pending and not self._closing)
        self.cancel_button.setEnabled(connected and (busy or pending) and not self._closing)
        self.functional_button.setEnabled(can_capture and base_ready
                                           and not busy and not pending and not self._closing)
        if profile_status.startswith("fitting"):
            calibration_text = self.tr("fitting")
        elif profile_status == "clearing":
            calibration_text = self.tr("clearing")
        elif active:
            calibration_text = self._capture_phase_text(active)
        elif pending:
            calibration_text = self.tr("pending")
        elif profile_status in {"fit_failed", "capture_failed"}:
            calibration_text = self.tr("calibration_failed")
        elif connected and not guide_ready:
            calibration_text = self.tr("calibration_waiting_data")
        elif profile_ready and not base_ready:
            calibration_text = self.tr("finger_no_pairs")
        elif profile_ready:
            calibration_text = self.tr("calibration_partial" if calibration.get("rejected_nodes") else "calibration_ready")
        else:
            calibration_text = self.tr("calibration_steps", count=len(captured)) if captured else self.tr("calibration_idle")
        if profile_ready and limited and not busy and not pending and "functional_slots_by_finger" in calibration:
            if available:
                calibration_text += " " + self.tr("finger_available", fingers=self._finger_names(available))
            calibration_text += " " + self.tr("finger_missing_pairs", fingers=self._finger_names(limited))
        if retry_steps and not busy and not pending:
            retry_label = "mounting_optional_retry" if base_ready else "mounting_retry"
            labels = [self._mounting_retry_label(step, calibration) for step in retry_steps]
            calibration_text += " " + self.tr(retry_label, steps=" / ".join(labels))
            if calibration.get("rejected_nodes"):
                calibration_text += " " + self.tr("mounting_nodes", nodes=", ".join(calibration["rejected_nodes"]))
        retry_feedback = calibration.get("mounting_retry_feedback")
        if profile_ready and retry_feedback and not busy and not pending:
            kept = self.tr("mounting_retry_kept", count=retry_feedback["kept_nodes"])
            calibration_text = kept + " " + calibration_text
        _set_text(self.calibration_status, calibration_text)
        functional = report.get("status") if connected else None
        functional_text = self.tr("functional_" + functional) if functional in {"complete", "partial", "failed", "cancelled"} else ""
        if functional in {"partial", "failed"}:
            needs_retry = set(report.get("errors", {})) | set(report.get("partial_slots", {}))
            names = self._finger_names(name for name in available if name in needs_retry)
            if names:
                functional_text += " " + self.tr("finger_retry_list", fingers=names)
            if "reference" in needs_retry:
                functional_text += " " + self.tr("finger_reference_failed")
        _set_text(self.functional_status, functional_text)
        self.functional_status.setVisible(bool(functional_text))
        self._update_calibration_progress(connected, calibration, pending, base_ready and guide_ready)

    def _mounting_retry_label(self, step, calibration):
        label = self.tr(step)
        official_step = {"fingers_down": "z_rot_pose_1", "thumb_up": "z_rot_pose_2"}.get(step)
        motion = calibration.get("motion_diagnostics") or {}
        if not official_step or not motion:
            return label
        node_slots = self.resources.get("config", {}).get("node_to_slot", {})
        rejections = motion.get("rigid_rejections")
        affected = []
        for node, reason in calibration.get("rejected_nodes", {}).items():
            slot = node_slots.get(node)
            if type(slot) is not int or not 0 < slot < len(SLOT_NAMES):
                continue
            if rejections is None:
                # Older profiles also stored residuals for synthetic slots. Only
                # attribute an actual measured rejection, never the placeholder.
                if not str(reason).startswith(f"节点 {node} 的倾转与手掌不一致"):
                    continue
                residual = motion.get("by_slot", {}).get(str(slot), {}).get("rigid_residual_deg", {}).get(official_step)
                threshold = motion.get("max_rigid_residual_deg")
                if not all(type(v) in (float, int) and math.isfinite(v) for v in (residual, threshold)) or residual <= threshold:
                    continue
            elif official_step not in rejections.get(node, ()):
                continue
            finger = SLOT_NAMES[slot].split("_", 1)[0]
            affected.append(self.tr("finger_imu", finger=self.tr("finger_" + finger), node=node))
        return self.tr("mounting_retry_detail", step=label, sensors=", ".join(affected)) if affected else label

    def _finger_names(self, fingers):
        separator = "、" if self.language == "zh_CN" else ", "
        return separator.join(self.tr("finger_" + name) for name in fingers)

    def _show_finger_guide(self, active, report, available, *, pending, fitting):
        action, title = "neutral", "finger_open"
        if fitting:
            instruction = self.tr("fitting")
        elif pending:
            instruction = self.tr("pending")
        elif active:
            phase = active.get("phase")
            instruction = self.tr("finger_hold_open")
            if phase == "moving_fingers" and not active.get("reference_waiting"):
                action, title = "flex_fingers", "finger_four"
                instruction = self.tr("finger_flex_help")
            elif phase in {"preparing_thumb", "moving_thumb"} and not active.get("reference_waiting"):
                action, title = "flex_thumb", "finger_thumb_tip"
                instruction = self.tr("finger_prepare_thumb" if phase == "preparing_thumb" else "finger_thumb_help")
        elif report.get("status") == "complete":
            instruction, title = self.tr("finger_use_hand"), "finger_finished"
        else:
            if available == ["thumb"]:
                action, title = "flex_thumb", "finger_thumb_tip"
            else:
                action, title = "flex_fingers", "finger_four"
            instruction = self.tr("finger_retry_help" if report else "finger_next_help")
        self.demo.set_step(action, hold=action == "neutral")
        _set_text(self.instructions, instruction)
        _set_text(self.demo_title, self.tr("mounting_demo", step=self.tr(title)))

    def _capture_phase_text(self, active):
        if active.get("reference_waiting"):
            return self.tr("waiting_reference")
        phase = active.get("phase", "collecting")
        phase = phase if phase in CAPTURE_PHASES else "collecting"
        remaining = active.get("remaining_s", 0.)
        seconds = max(0, math.ceil(remaining)) if isinstance(remaining, (int, float)) and math.isfinite(remaining) else 0
        return self.tr(phase, seconds=seconds)

    def _update_calibration_progress(self, connected, calibration, pending, base_ready):
        """Render backend evidence without starting a second calibration clock."""
        # Build a complete presentation before touching widgets. Resetting then
        # reapplying a busy strip every GUI tick would restart its animation.
        states = {button: (None, False, self.tr(step + "_capture"))
                  for step, button in self.step_buttons.items()}
        states[self.functional_button] = (None, False, self.tr("functional"))
        states[self.heading_button] = (None, False, self.tr("heading_reference"))
        if not connected or self._closing:
            calibration, pending = {}, False
        for step in calibration.get("captured_steps", []):
            if step in self.step_buttons:
                states[self.step_buttons[step]] = (1., False, "✓ " + self.tr(step + "_recapture"))
        for step in calibration.get("retry_steps", []):
            if step in self.step_buttons:
                states[self.step_buttons[step]] = (None, False, "↻ " + self.tr(step + "_recapture"))
        if (calibration.get("functional_hand_result") or {}).get("status") == "complete":
            states[self.functional_button] = (1., False, "✓ " + self.tr("finger_recapture"))

        active = calibration.get("active") or {}
        status = str(calibration.get("profile_status", ""))
        completed = set(calibration.get("captured_steps", ())).intersection(STEPS)
        completed.difference_update(calibration.get("retry_steps", ()))
        overall = (len(completed) / len(STEPS), False, self.tr("calibration_steps", count=len(completed)))
        result = (calibration.get("functional_hand_result") or {}).get("status")
        if status in {"fit_failed", "capture_failed"}:
            overall = (len(completed) / len(STEPS) if len(completed) < len(STEPS) else 0.,
                       False, self.tr("calibration_failed"))
        elif base_ready:
            summary = "functional_" + result if result in {"partial", "complete", "failed", "cancelled"} else "finger_next"
            overall = (1. if result == "complete" else 0., False, self.tr(summary))
        if status.startswith("fitting"):
            button = self.functional_button if status == "fitting_functional_hand" else self.step_buttons[self._step]
            states[button] = (None, True, self.tr("fitting"))
            overall = states[button]
        elif active:
            if active.get("kind") == "heading_reference":
                button = self.heading_button
            elif active.get("kind") in {"functional", "functional_hand"}:
                button = self.functional_button
            else:
                button = self.step_buttons.get(active.get("step"), self.step_buttons[self._step])
            value = active.get("phase_progress")
            determinate = (not active.get("reference_waiting") and type(value) in (float, int)
                           and math.isfinite(value) and 0. <= value <= 1.)
            # A phase timer reaching its end is not an accepted capture. Reserve
            # a full strip/checkmark for backend completion evidence.
            progress = min(value, .99) if determinate else None
            label = self._capture_phase_text(active)
            if determinate:
                label += f" · {int(progress * 100)}%"
            states[button] = (progress, not determinate, label)
            overall = states[button]
            if active.get("kind", "mounting") == "mounting" and active.get("step") in self.step_buttons:
                step = active["step"]
                step_label = self.tr(step) + (f" · {int(progress * 100)}%" if determinate else "")
                states[self.step_buttons[step]] = (progress, not determinate, step_label)
        elif pending:
            name = self._pending_calibration["name"]
            pending_buttons = {"capture": self.step_buttons[self._step], "capture_functional_hand": self.functional_button,
                               "capture_heading_reference": self.heading_button}
            if name in pending_buttons:
                button = pending_buttons[name]
                states[button] = (None, True, self.tr("pending_short"))
                overall = states[button]
        progress, busy, label = overall
        self.calibration_progress.setRange(0, 0 if busy else 1000)
        if not busy:
            self.calibration_progress.setValue(round(1000 * progress))
        self.calibration_progress.setFormat(label)
        for button, (value, busy, label) in states.items():
            button.set_progress(value, busy=busy)
            _set_text(button, label)

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

    def _motion_notice(self, runtime):
        rejected = (self._snapshot.get("pose") or {}).get("calibration_rejections", {})
        if not runtime.get("connected") or not rejected:
            return ""
        realign = [node for node, reason in rejected.items()
                   if reason in ("filter_epoch_changed_recalibration_required", "node_not_calibrated")]
        if realign:
            return self.tr("joint_realign", nodes=", ".join(realign))
        return self.tr("joint_input_wait", nodes=", ".join(rejected))

    def _update_display_status(self, runtime, calibration, touch_status):
        combined = self._combined
        mode = combined.mode if combined else "waiting"
        mode_key = COMPOSITION_STATUS.get(combined.status, mode) if combined else mode
        label = self.tr(mode_key)
        if combined and mode == "waiting":
            if self._palm_preview is not None and self._palm_preview.geometry is not None:
                label = self.tr("palm_touch_waiting" if combined.tactile_frame is not None else "palm_waiting", reason=label)
            else:
                label = self.tr("reference_touch_waiting" if combined.tactile_frame is not None
                                else "reference_waiting", reason=label)
        _set_text(self.mode_label, label)
        source = self.tr({"live": "measured_hand", "held": "held_hand"}.get(mode, "fixed_hand"))
        if self._palm_preview is not None:
            if self._palm_preview.state == "tilt":
                source = self.tr("live_palm_pitch", angle=self._palm_preview.pitch_deg)
            elif self._palm_preview.state == "inclination":
                source = self.tr("live_palm_angle", angle=self._palm_preview.angle_deg)
            elif self._palm_preview.state == "await_neutral":
                source = self.tr("await_flat_hand")
        _set_text(self.live_title, self.tr("my_gesture", source=source))
        def age(value):
            return f"{value:.0f} ms" if isinstance(value, (int, float)) and math.isfinite(value) else "—"
        metrics = self.tr("metrics", pose=age(combined.pose_age_ms if combined else None),
                          touch=age(combined.tactile_age_ms if combined else None),
                          skew=age(combined.skew_ms if combined else None))
        if combined and combined.tactile_frame is not None:
            frame = combined.tactile_frame
            values = [v for v, valid in zip(frame["values"], frame["valid_mask"]) if valid]
            if values:
                metrics += "   |   " + self.tr("touch_reading", valid=len(values), total=len(frame["values"]),
                                               active=sum(v > 0 for v in values), peak=max(values))
        _set_text(self.metrics, metrics)
        storage_error = self.tactile_source.reference_persistence_error or (runtime.get("calibration_storage") or {}).get("error")
        issue = self._action_error or ("storage_error" if storage_error else "backend_error" if runtime.get("error") or touch_status == "failed" else "port_scan_error" if self._port_scan_error else "")
        notice = self.tr("closing") if self._closing else self.tr(issue) if issue else self._motion_notice(runtime)
        _set_text(self.notice, notice)
        self.notice.setVisible(bool(notice))
        self._update_details(force=bool(issue))
        if not self.owns_session:
            self.tactile_baudrate.setEnabled(False)
            for kind, combo in self.port_combos.items():
                combo.setEnabled(False)
                if self.connection_status is not None:
                    status = self.connection_status(kind)
                    self.connection_buttons[kind].setEnabled(bool(status.get("enabled")))
                    self.connection_buttons[kind].setText(status.get("label", self.tr("connect")))
                    self.connection_buttons[kind].setToolTip(status.get("detail", ""))
                    self.connection_statuses[kind].setText(status.get("state", ""))
                    port = status.get("port", "")
                    if combo.currentText() != port:
                        combo.setEditText(port)


    def _update_details(self, *, force=False):
        now = host_clock.monotonic_ns()
        if not force and (self.details.isHidden() or (self._details_updated_ns is not None
                and 0 <= now - self._details_updated_ns < 200_000_000)):
            return
        self._details_updated_ns = now
        runtime, calibration, combined = self._snapshot.get("runtime", {}), self._snapshot.get("calibration", {}), self._combined
        details = dict(motion={key: runtime.get(key) for key in ("message", "error", "received_samples", "last_sample_age_s", "calibration_storage")},
                       calibration=dict(profile_status=calibration.get("profile_status", ""), rejected_nodes=calibration.get("rejected_nodes", {}), functional_result=calibration.get("functional_hand_result")),
                       tactile=self._tactile_diagnostics(),
                       display=combined.status if combined else "", action=self._error_detail,
                       port_discovery=self._port_scan_error,
                       port_types=[dict(path=item.path, role=item.role, detail=item.detail)
                                   for item in self.port_discovery.latest().classifications]
                                  if self.port_discovery.latest() is not None else [])
        formatted = json.dumps(details, ensure_ascii=False, indent=2, default=str)[:16000]
        if self.details.toPlainText() != formatted:
            self.details.setPlainText(formatted)

    def _tactile_diagnostics(self):
        state = self._touch_state
        result = dict(status=state.status if state else "", detail=state.detail if state else "",
                      persistence_error=self.tactile_source.reference_persistence_error)
        if state is None:
            return result
        result.update(valid_cells=state.valid_cells, delivered_snapshots_hz=state.receive_hz)
        raw = state.raw_frame
        if raw is not None:
            from vt_tactile_capture.conversion import adc_to_resistance
            # Compact rows keep the optional diagnostics readable and bounded.
            # Missing cells remain distinct from real ADC zero values.
            values, valid = raw["values"], raw["valid_mask"]
            current = [value for value, present in zip(values, valid) if present]
            result["adc_range"] = [min(current), max(current)] if current else None
            resistance = [value for adc in current
                          if (value := adc_to_resistance(int(adc))) is not None]
            result["reference_circuit_resistance_ohm_range"] = (
                [min(resistance), max(resistance)] if resistance else None)
            result["adc_rows_0_to_19"] = [
                " ".join(str(int(values[index])) if valid[index] else "—"
                         for index in range(row * 20, (row + 1) * 20)) for row in range(20)]
        return result

    def shutdown(self):
        """Request closure once; the timer completes resource release without blocking."""
        if self._closed:
            return True
        if not self.owns_session:
            self._closing = self._closed = True
            self.timer.stop()
            self.port_discovery.stop()
            self.pose_controller.close()
            self.tactile_source.close()
            if hasattr(self.demo, "dispose"):
                self.demo.dispose()
            return True
        if not self._closing:
            self._closing = True
            self.port_discovery.stop()
            self.pose_controller.stop()
            self.tactile_source.disconnect()
            self.setEnabled(False)
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


class HandTactileWindow(HandPanel):
    """Standalone shell retaining ownership of its local sources."""

    def __init__(self, *args, **kwargs):
        kwargs["owns_session"] = True
        super().__init__(*args, **kwargs)
