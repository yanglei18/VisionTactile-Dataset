"""Twelve-node, six-axis IMU monitor for an STM32 USB-CDC interface.

Expected node block (14 bytes):
    byte 0      Node ID, 1..12
    byte 1      Ignored/status byte received from the node
    bytes 2..13 Six signed 16-bit values: GX, GY, GZ, AX, AY, AZ

Each 84-byte CDC frame contains six node blocks. The node IDs are at offsets
0, 14, 28, 42, 56, and 70. Frame type 1 contains IDs 1..6; frame type 2
contains IDs 7..12.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import struct
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

# This program uses PyQt5 explicitly. Letting pyqtgraph select an installed
# Qt6 first can mix incompatible bindings and fail before the window opens.
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PyQt5 import QtCore, QtGui, QtWidgets


BAUD_RATE = 115200  # USB CDC normally ignores this value.
BYTE_ORDER = "<"    # Current STM32 wire contract: little-endian signed int16.

NODE_COUNT = 12
NODES_PER_FRAME = 6
NODE_BLOCK_SIZE = 14
FRAME_SIZE = NODES_PER_FRAME * NODE_BLOCK_SIZE
NODE_ID_OFFSETS = tuple(range(0, FRAME_SIZE, NODE_BLOCK_SIZE))
FRAME_NODE_GROUPS = (
    tuple(range(1, NODES_PER_FRAME + 1)),
    tuple(range(NODES_PER_FRAME + 1, NODE_COUNT + 1)),
)

# Configured conversion assumptions for the user-reported LSM6DSRTR.
# This monitor does not read IMU registers or verify STM32 rescaling.
# Supported accelerometer values: 2, 4, 8, or 16 g.
# Supported gyroscope values: 125, 250, 500, 1000, 2000, or 4000 deg/s.
# They control both LSB-to-unit conversion and the fixed plot Y-axis limits.
ACCEL_FULL_SCALE_G = 4
GYRO_FULL_SCALE_DPS = 250

# LSM6DSR typical sensitivities from the ST datasheet (not register readback).
ACCEL_G_PER_LSB = {
    2: 0.061e-3,
    4: 0.122e-3,
    8: 0.244e-3,
    16: 0.488e-3,
}
GYRO_DPS_PER_LSB = {
    125: 4.375e-3,
    250: 8.75e-3,
    500: 17.50e-3,
    1000: 35.0e-3,
    2000: 70.0e-3,
    4000: 140.0e-3,
}

if ACCEL_FULL_SCALE_G not in ACCEL_G_PER_LSB:
    raise ValueError("ACCEL_FULL_SCALE_G must be 2, 4, 8, or 16")
if GYRO_FULL_SCALE_DPS not in GYRO_DPS_PER_LSB:
    raise ValueError(
        "GYRO_FULL_SCALE_DPS must be 125, 250, 500, 1000, 2000, or 4000"
    )

# The current STM32 protocol is interpreted as GX, GY, GZ, AX, AY, AZ.
# This is an input contract, not proof that firmware forwards registers unchanged.
AXIS_NAMES = ("Gyro X", "Gyro Y", "Gyro Z", "Accel X", "Accel Y", "Accel Z")
AXIS_SHORT = ("GX", "GY", "GZ", "AX", "AY", "AZ")
AXIS_UNITS = ("deg/s", "deg/s", "deg/s", "g", "g", "g")
AXIS_LSB_TO_UNIT = (
    GYRO_DPS_PER_LSB[GYRO_FULL_SCALE_DPS],
    GYRO_DPS_PER_LSB[GYRO_FULL_SCALE_DPS],
    GYRO_DPS_PER_LSB[GYRO_FULL_SCALE_DPS],
    ACCEL_G_PER_LSB[ACCEL_FULL_SCALE_G],
    ACCEL_G_PER_LSB[ACCEL_FULL_SCALE_G],
    ACCEL_G_PER_LSB[ACCEL_FULL_SCALE_G],
)
AXIS_Y_RANGES = (
    (-GYRO_FULL_SCALE_DPS, GYRO_FULL_SCALE_DPS),
    (-GYRO_FULL_SCALE_DPS, GYRO_FULL_SCALE_DPS),
    (-GYRO_FULL_SCALE_DPS, GYRO_FULL_SCALE_DPS),
    (-ACCEL_FULL_SCALE_G, ACCEL_FULL_SCALE_G),
    (-ACCEL_FULL_SCALE_G, ACCEL_FULL_SCALE_G),
    (-ACCEL_FULL_SCALE_G, ACCEL_FULL_SCALE_G),
)
PLOT_WINDOW_SECONDS = 5.0
MAX_POINTS_PER_CHANNEL = 3000
PLOT_REFRESH_MS = 50
SERIAL_BATCH_SECONDS = 0.020

Record = Tuple[float, float, int, Tuple[int, int, int, int, int, int]]


def node_unit_scales(config=None):
    """Read the pose tool's unit fields without requiring that package.

    Original raw records and CSV values are never converted in place. Empty
    configuration preserves the historical monitor's units.
    """
    config = {} if config is None else config
    if not isinstance(config, dict):
        raise ValueError("configuration must be a JSON object")
    gyro = config.get("gyro_dps_per_lsb", AXIS_LSB_TO_UNIT[0])
    accel = config.get("accel_g_per_lsb", AXIS_LSB_TO_UNIT[3])
    def valid(value, name, low, high):
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value) or not low <= value <= high):
            raise ValueError(f"{name} must be finite in [{low}, {high}]")
        return float(value)
    gyro = valid(gyro, "gyro_dps_per_lsb", 1e-6, 10)
    accel = valid(accel, "accel_g_per_lsb", 1e-7, 1)
    overrides = config.get("gyro_dps_per_lsb_by_node", {})
    if not isinstance(overrides, dict):
        raise ValueError("gyro_dps_per_lsb_by_node must be an object")
    if any(type(key) is not str or key not in {str(n) for n in range(1, 13)} for key in overrides):
        raise ValueError("gyro scale node keys must be canonical strings 1..12")
    overrides = {key: valid(value, f"node {key} gyro scale", 1e-6, 10)
                 for key, value in overrides.items()}
    accel_overrides = config.get("accel_g_per_lsb_by_node", {})
    if not isinstance(accel_overrides, dict):
        raise ValueError("accel_g_per_lsb_by_node must be an object")
    if any(type(key) is not str or key not in {str(n) for n in range(1, 13)} for key in accel_overrides):
        raise ValueError("accel scale node keys must be canonical strings 1..12")
    accel_overrides = {key: valid(value, f"node {key} accel scale", 1e-7, 1)
                       for key, value in accel_overrides.items()}
    return [((overrides.get(str(node), gyro),) * 3 + (accel_overrides.get(str(node), accel),) * 3)
            for node in range(1, 13)]


class ImuStreamParser:
    """Synchronize and decode an 84-byte, six-node frame."""

    LAYOUTS = (
        (
            FRAME_SIZE,
            NODE_ID_OFFSETS,
            "84-byte / 6 x 14-byte blocks",
        ),
    )
    GROUPS = FRAME_NODE_GROUPS

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames_ok = 0
        self.bytes_discarded = 0
        self.last_layout = "searching"

    def reset(self) -> None:
        self.buffer.clear()
        self.frames_ok = 0
        self.bytes_discarded = 0
        self.last_layout = "searching"

    @staticmethod
    def _matches(data: bytearray, start: int, offsets: Sequence[int],
                 group: Sequence[int]) -> bool:
        return all(data[start + offset] == node_id
                   for offset, node_id in zip(offsets, group))

    def feed(self, data: bytes) -> List[Record]:
        self.buffer.extend(data)
        output: List[Record] = []

        while len(self.buffer) >= FRAME_SIZE:
            candidate = None

            # Search for six correctly positioned node IDs. Requiring all six
            # IDs makes accidental synchronization on IMU payload bytes unlikely.
            max_start = len(self.buffer) - FRAME_SIZE
            for start in range(max_start + 1):
                for frame_size, offsets, layout_name in self.LAYOUTS:
                    if start + frame_size > len(self.buffer):
                        continue
                    for group in self.GROUPS:
                        if self._matches(self.buffer, start, offsets, group):
                            candidate = (start, frame_size, offsets,
                                         layout_name, group)
                            break
                    if candidate is not None:
                        break
                if candidate is not None:
                    break

            if candidate is None:
                # Retain enough trailing bytes to complete a partial frame.
                keep = min(len(self.buffer), FRAME_SIZE - 1)
                discard = len(self.buffer) - keep
                if discard:
                    del self.buffer[:discard]
                    self.bytes_discarded += discard
                break

            start, frame_size, offsets, layout_name, group = candidate
            if start:
                del self.buffer[:start]
                self.bytes_discarded += start

            frame = bytes(self.buffer[:frame_size])
            del self.buffer[:frame_size]

            monotonic_time = time.perf_counter()
            wall_time = time.time()
            for offset, expected_id in zip(offsets, group):
                node_id = frame[offset]
                if node_id != expected_id:
                    continue
                # frame[offset + 1] is intentionally ignored.
                values = struct.unpack(BYTE_ORDER + "6h",
                                       frame[offset + 2:offset + 14])
                output.append((monotonic_time, wall_time, node_id, values))

            self.frames_ok += 1
            self.last_layout = layout_name

        return output


class SerialWorker(QtCore.QThread):
    records_ready = QtCore.pyqtSignal(object)
    connection_changed = QtCore.pyqtSignal(bool, str)
    statistics_ready = QtCore.pyqtSignal(object)
    error_occurred = QtCore.pyqtSignal(str)

    def __init__(self, port_name: str, parent=None) -> None:
        super().__init__(parent)
        self.port_name = port_name
        self._running = True
        self._playing = False
        self._reset_requested = False

    def set_playing(self, enabled: bool) -> None:
        self._playing = enabled
        if enabled:
            self._reset_requested = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        parser = ImuStreamParser()
        port: Optional[serial.Serial] = None
        batch: List[Record] = []
        last_batch_time = time.perf_counter()
        last_stats_time = last_batch_time
        node_records_since_stats = 0

        try:
            port = serial.Serial(
                self.port_name,
                BAUD_RATE,
                timeout=0.020,
                write_timeout=0.2,
            )
            port.reset_input_buffer()
            self.connection_changed.emit(True, self.port_name)

            while self._running:
                if self._reset_requested:
                    port.reset_input_buffer()
                    parser.reset()
                    batch.clear()
                    self._reset_requested = False

                waiting = port.in_waiting
                chunk = port.read(waiting if waiting > 0 else 1)
                now = time.perf_counter()

                if chunk and self._playing:
                    records = parser.feed(chunk)
                    batch.extend(records)
                    node_records_since_stats += len(records)

                if batch and now - last_batch_time >= SERIAL_BATCH_SECONDS:
                    self.records_ready.emit(batch)
                    batch = []
                    last_batch_time = now

                stats_elapsed = now - last_stats_time
                if stats_elapsed >= 1.0:
                    per_node_rate = (
                        node_records_since_stats / (NODE_COUNT * stats_elapsed)
                    )
                    self.statistics_ready.emit({
                        "rate": per_node_rate,
                        "layout": parser.last_layout,
                        "frames": parser.frames_ok,
                        "discarded": parser.bytes_discarded,
                    })
                    node_records_since_stats = 0
                    last_stats_time = now

        except serial.SerialException as exc:
            self.error_occurred.emit(str(exc))
        except Exception as exc:  # Keep unexpected worker errors visible.
            self.error_occurred.emit(f"Unexpected serial error: {exc}")
        finally:
            if batch:
                self.records_ready.emit(batch)
            if port is not None and port.is_open:
                port.close()
            self.connection_changed.emit(False, self.port_name)


class ImuMainWindow(QtWidgets.QMainWindow):
    def __init__(self, *, config=None, port=None) -> None:
        super().__init__()
        self.node_scales = node_unit_scales(config)
        self.setWindowTitle("12-Node IMU Monitor")
        self.resize(1900, 950)

        self.worker: Optional[SerialWorker] = None
        self.connected = False
        self.playing = False
        self.first_sample_time: Optional[float] = None

        self.times: List[Deque[float]] = [
            deque(maxlen=MAX_POINTS_PER_CHANNEL) for _ in range(NODE_COUNT)
        ]
        self.values: List[List[Deque[float]]] = [
            [deque(maxlen=MAX_POINTS_PER_CHANNEL) for _ in range(6)]
            for _ in range(NODE_COUNT)
        ]

        self.csv_file = None
        self.csv_writer = None
        self.csv_path: Optional[Path] = None
        self.pending_scan: Dict[int, Record] = {}
        self.last_csv_flush = time.perf_counter()

        self.curves: List[List[pg.PlotDataItem]] = [
            [None for _ in range(6)] for _ in range(NODE_COUNT)  # type: ignore
        ]

        self._build_ui()
        self._scan_ports()
        if port:
            index = self.port_combo.findData(port)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

        self.port_scan_timer = QtCore.QTimer(self)
        self.port_scan_timer.timeout.connect(self._scan_ports)
        self.port_scan_timer.start(1500)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(PLOT_REFRESH_MS)

    def _build_ui(self) -> None:
        pg.setConfigOptions(antialias=False, background="#11151c", foreground="#d6d9df")

        central = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("USB CDC port:"))

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(280)
        controls.addWidget(self.port_combo)

        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.refresh_button.clicked.connect(self._scan_ports)
        controls.addWidget(self.refresh_button)

        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self._toggle_connection)
        controls.addWidget(self.connect_button)

        self.play_button = QtWidgets.QPushButton("Play")
        self.play_button.setEnabled(False)
        self.play_button.clicked.connect(self._toggle_play)
        controls.addWidget(self.play_button)

        self.save_button = QtWidgets.QPushButton("Save Data")
        self.save_button.clicked.connect(self._toggle_save)
        controls.addWidget(self.save_button)

        self.clear_button = QtWidgets.QPushButton("Clear")
        self.clear_button.clicked.connect(self._clear_history)
        controls.addWidget(self.clear_button)

        controls.addStretch(1)
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setMinimumWidth(430)
        controls.addWidget(self.status_label)
        outer.addLayout(controls)

        self.graphics = pg.GraphicsLayoutWidget()
        outer.addWidget(self.graphics, 1)

        colors = ("#50b7ff", "#61d095", "#ffd166", "#ef476f", "#c77dff", "#ff9f1c")
        for axis in range(6):
            for node in range(NODE_COUNT):
                plot = self.graphics.addPlot(row=axis, col=node)
                plot.showGrid(x=True, y=True, alpha=0.18)
                plot.setMouseEnabled(x=True, y=False)
                plot.setClipToView(True)
                plot.setDownsampling(auto=True, mode="peak")
                plot.getViewBox().setDefaultPadding(0.04)
                plot.getViewBox().setBorder(pg.mkPen("#7f8794", width=1))
                plot.disableAutoRange(axis=pg.ViewBox.YAxis)
                # Unit configuration can differ across chips (e.g. thumb
                # +/-500 dps, other fingers +/-250 dps). Scale each plot too.
                scale = self.node_scales[node][axis]
                bound = 32768 * scale
                y_min, y_max = -bound, bound
                plot.setYRange(y_min, y_max, padding=0)
                plot.setToolTip(f"Node {node + 1}: {scale:g} {AXIS_UNITS[axis]}/LSB; "
                                "configured conversion, not firmware verification")

                if axis == 0:
                    plot.setTitle(f"Node {node + 1}", size="10pt")
                if node == 0:
                    plot.setLabel("left", AXIS_NAMES[axis], units=AXIS_UNITS[axis])
                else:
                    plot.hideAxis("left")
                if axis == 5:
                    plot.setLabel("bottom", "Time", units="s")
                else:
                    plot.hideAxis("bottom")

                curve = plot.plot(pen=pg.mkPen(colors[axis], width=1.2))
                self.curves[node][axis] = curve

        self.setCentralWidget(central)

    @staticmethod
    def _port_score(port) -> int:
        text = f"{port.description} {port.manufacturer} {port.hwid}".lower()
        score = 0
        if port.vid is not None:
            score += 2
        for keyword, points in (
            ("stm", 10), ("cdc", 8), ("virtual com", 7),
            ("usb serial", 6), ("usb", 3),
        ):
            if keyword in text:
                score += points
        return score

    @QtCore.pyqtSlot()
    def _scan_ports(self) -> None:
        if self.connected:
            return

        previous = self.port_combo.currentData()
        ports = list(list_ports.comports())
        ports.sort(key=lambda p: (-self._port_score(p), p.device))

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for port in ports:
            label = f"{port.device} — {port.description}"
            self.port_combo.addItem(label, port.device)

        if previous is not None:
            index = self.port_combo.findData(previous)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        self.port_combo.blockSignals(False)
        self.connect_button.setEnabled(bool(ports))

        if not ports:
            self.status_label.setText("No serial/CDC port detected")
        elif not self.connected:
            selected = self.port_combo.currentData()
            self.status_label.setText(f"Ready: {selected}")

    @QtCore.pyqtSlot()
    def _toggle_connection(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.connect_button.setEnabled(False)
            self.status_label.setText("Disconnecting…")
            self.worker.stop()
            self.worker.wait(1000)
            return

        port_name = self.port_combo.currentData()
        if not port_name:
            QtWidgets.QMessageBox.warning(self, "No port", "No USB CDC port is selected.")
            return

        self.connect_button.setEnabled(False)
        self.status_label.setText(f"Connecting to {port_name}…")
        self.worker = SerialWorker(port_name, self)
        self.worker.records_ready.connect(self._receive_records)
        self.worker.connection_changed.connect(self._connection_changed)
        self.worker.statistics_ready.connect(self._show_statistics)
        self.worker.error_occurred.connect(self._show_serial_error)
        self.worker.finished.connect(self._worker_finished)
        self.worker.start()

    @QtCore.pyqtSlot(bool, str)
    def _connection_changed(self, connected: bool, port_name: str) -> None:
        self.connected = connected
        if connected:
            self.connect_button.setText("Disconnect")
            self.connect_button.setEnabled(True)
            self.port_combo.setEnabled(False)
            self.refresh_button.setEnabled(False)
            self.play_button.setEnabled(True)
            self.status_label.setText(f"Connected: {port_name}; press Play")
        else:
            self.playing = False
            self.play_button.setText("Play")
            self.play_button.setEnabled(False)
            self.connect_button.setText("Connect")
            self.connect_button.setEnabled(self.port_combo.count() > 0)
            self.port_combo.setEnabled(True)
            self.refresh_button.setEnabled(True)
            if self.status_label.text().startswith(("Connected", "Streaming", "Paused", "Disconnecting")):
                self.status_label.setText("Disconnected")

    @QtCore.pyqtSlot()
    def _worker_finished(self) -> None:
        self.worker = None
        self._scan_ports()

    @QtCore.pyqtSlot()
    def _toggle_play(self) -> None:
        if self.worker is None:
            return
        self.playing = not self.playing
        self.worker.set_playing(self.playing)
        self.play_button.setText("Pause" if self.playing else "Play")
        self.status_label.setText("Streaming…" if self.playing else "Paused")

    @QtCore.pyqtSlot(object)
    def _receive_records(self, records: List[Record]) -> None:
        if not records:
            return

        for record in records:
            monotonic_time, _wall_time, node_id, sample = record
            if self.first_sample_time is None:
                self.first_sample_time = monotonic_time
            elapsed = monotonic_time - self.first_sample_time
            node_index = node_id - 1
            self.times[node_index].append(elapsed)
            for axis, value in enumerate(sample):
                # Plot in physical units. The original LSB sample remains in
                # ``record`` and is written unchanged to the CSV file.
                self.values[node_index][axis].append(
                    value * self.node_scales[node_index][axis]
                )

            if self.csv_writer is not None:
                # Node 1 marks the beginning of a new complete 12-node scan.
                if node_id == 1:
                    self.pending_scan.clear()
                self.pending_scan[node_id] = record
                if len(self.pending_scan) == NODE_COUNT:
                    self._write_csv_scan(self.pending_scan)
                    self.pending_scan.clear()

        if self.csv_file is not None and time.perf_counter() - self.last_csv_flush >= 1.0:
            self.csv_file.flush()
            self.last_csv_flush = time.perf_counter()

    def _write_csv_scan(self, scan: Dict[int, Record]) -> None:
        first = scan[1]
        wall_time = first[1]
        elapsed = first[0] - (self.first_sample_time or first[0])
        row = [
            datetime.fromtimestamp(wall_time).isoformat(timespec="milliseconds"),
            f"{elapsed:.6f}",
        ]
        for node_id in range(1, NODE_COUNT + 1):
            row.extend(scan[node_id][3])
        self.csv_writer.writerow(row)

    @QtCore.pyqtSlot()
    def _refresh_plots(self) -> None:
        if not self.playing:
            return
        newest = 0.0
        for node_times in self.times:
            if node_times:
                newest = max(newest, node_times[-1])
        oldest = max(0.0, newest - PLOT_WINDOW_SECONDS)

        for node in range(NODE_COUNT):
            x = list(self.times[node])
            if not x:
                continue
            # All six axes for one node share identical timestamps.
            for axis in range(6):
                self.curves[node][axis].setData(x, list(self.values[node][axis]))
                self.curves[node][axis].getViewBox().setXRange(oldest, max(newest, 0.1), padding=0)

    @QtCore.pyqtSlot()
    def _clear_history(self) -> None:
        for node in range(NODE_COUNT):
            self.times[node].clear()
            for axis in range(6):
                self.values[node][axis].clear()
                self.curves[node][axis].clear()
        self.first_sample_time = None
        self.pending_scan.clear()

    @QtCore.pyqtSlot()
    def _toggle_save(self) -> None:
        if self.csv_file is not None:
            self._stop_saving()
            return

        default_name = datetime.now().strftime("imu_data_%Y%m%d_%H%M%S.csv")
        file_name, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save IMU data", default_name, "CSV files (*.csv)"
        )
        if not file_name:
            return

        try:
            self.csv_path = Path(file_name)
            self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            header = ["host_time", "elapsed_s"]
            header.extend(
                f"N{node}_{axis}_LSB"
                for node in range(1, NODE_COUNT + 1)
                for axis in AXIS_SHORT
            )
            self.csv_writer.writerow(header)
            self.csv_file.flush()
            self.pending_scan.clear()
            self.last_csv_flush = time.perf_counter()
            self.save_button.setText("Stop Saving")
            self.save_button.setStyleSheet("background-color: #a83232; color: white;")
        except OSError as exc:
            self.csv_file = None
            self.csv_writer = None
            QtWidgets.QMessageBox.critical(self, "Save error", str(exc))

    def _stop_saving(self) -> None:
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        saved_path = self.csv_path
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.pending_scan.clear()
        self.save_button.setText("Save Data")
        self.save_button.setStyleSheet("")
        if saved_path is not None:
            self.status_label.setText(f"Saved: {saved_path}")

    @QtCore.pyqtSlot(object)
    def _show_statistics(self, stats: dict) -> None:
        if self.playing:
            self.status_label.setText(
                f"Streaming: {stats['rate']:.1f} samples/s per node | "
                f"{stats['layout']} | discarded {stats['discarded']} bytes"
            )

    @QtCore.pyqtSlot(str)
    def _show_serial_error(self, message: str) -> None:
        self.status_label.setText(f"Serial error: {message}")
        QtWidgets.QMessageBox.critical(self, "Serial error", message)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._stop_saving()
        if self.worker is not None and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait(1000)
        event.accept()


def main() -> int:
    parser = argparse.ArgumentParser(description="12-node IMU monitor; connect manually")
    parser.add_argument("--config", type=Path, help="Pose JSON config, including per-node gyro scales")
    parser.add_argument("--port", help="Prefill only; click Connect then Play")
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text(encoding="utf-8")) if args.config else None
        node_unit_scales(config)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName("IMU Monitor")
    window = ImuMainWindow(config=config, port=args.port)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
