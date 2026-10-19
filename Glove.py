import sys
import csv
from pathlib import Path
from datetime import datetime

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
import pyqtgraph.exporters
from pyqtgraph.Qt import QtCore, QtWidgets


# =========================
# Settings
# =========================
BAUDRATE = 115200
# Physical display/export area. Serial packets retain their 80-row addressing
# and ten stride-8 ADC values; addresses outside this ROI are not touch cells.
ROWS = 20
COLS = 20
WIRE_ROWS = 80

FRAME_SIZE = 26
HEADER1 = 0xA5
HEADER2 = 0x5A

YMIN = 20000
YMAX = 24000

def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    ranked = []
    for p in ports:
        text = " ".join([
            str(p.device or ""),
            str(p.description or ""),
            str(p.manufacturer or ""),
            str(p.product or ""),
            str(p.hwid or "")
        ]).lower()

        score = 0
        for kw in ["stm32", "stmicro", "stlink", "usb serial", "cdc", "virtual com", "vcp", "usb"]:
            if kw in text:
                score += 1

        ranked.append((score, p.device))

    ranked.sort(reverse=True)
    if ranked and ranked[0][0] > 0:
        return ranked[0][1]

    return ports[0].device


class HeatmapViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tactile ADC Viewer — 20×20")

        # Store EXACTLY what MATLAB stores: raw ADC code
        self.data = np.full((ROWS, COLS), np.nan, dtype=np.float32)

        self.frame_count = 0
        self.bad_frame_count = 0
        self.byte_count = 0
        self.last_row = None
        self.last_col_start = None

        self.ser = None

        # Stream parser state machine
        # 0: wait for A5
        # 1: wait for 5A
        # 2: collect remaining 24 bytes
        self.rx_state = 0
        self.payload = bytearray()

        # Display window (inclusive indices from UI)
        self.view_row_start = 0
        self.view_row_end = ROWS - 1
        self.view_col_start = 0
        self.view_col_end = COLS - 1

        self._build_ui()
        self.open_auto_port()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_serial)
        self.timer.start(5)



    def _build_ui(self):
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)
        layout = QtWidgets.QVBoxLayout(cw)

        # Top controls
        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)

        self.port_label = QtWidgets.QLabel("Port: not connected")
        top.addWidget(self.port_label)

        self.reconnect_btn = QtWidgets.QPushButton("Reconnect")
        self.reconnect_btn.clicked.connect(self.reconnect)
        top.addWidget(self.reconnect_btn)

        self.save_btn = QtWidgets.QPushButton("Save PNG + CSV")
        self.save_btn.clicked.connect(self.save_both)
        top.addWidget(self.save_btn)

        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_data)
        top.addWidget(self.clear_btn)

        top.addSpacing(12)
        top.addWidget(QtWidgets.QLabel("Col:"))
        self.col_start_spin = QtWidgets.QSpinBox()
        self.col_start_spin.setRange(0, COLS - 1)
        self.col_start_spin.setValue(0)
        top.addWidget(self.col_start_spin)

        self.col_end_spin = QtWidgets.QSpinBox()
        self.col_end_spin.setRange(0, COLS)
        self.col_end_spin.setValue(COLS)
        top.addWidget(self.col_end_spin)

        top.addWidget(QtWidgets.QLabel("Row:"))
        self.row_start_spin = QtWidgets.QSpinBox()
        self.row_start_spin.setRange(0, ROWS - 1)
        self.row_start_spin.setValue(0)
        top.addWidget(self.row_start_spin)

        self.row_end_spin = QtWidgets.QSpinBox()
        self.row_end_spin.setRange(0, ROWS)
        self.row_end_spin.setValue(ROWS)
        top.addWidget(self.row_end_spin)

        self.apply_range_btn = QtWidgets.QPushButton("Apply Range")
        self.apply_range_btn.clicked.connect(self.apply_view_range)
        top.addWidget(self.apply_range_btn)

        self.full_range_btn = QtWidgets.QPushButton("Full Range")
        self.full_range_btn.clicked.connect(self.reset_view_range)
        top.addWidget(self.full_range_btn)

        top.addStretch()

        # Plot area
        self.graphics_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_layout)

        self.plot = self.graphics_layout.addPlot()
        self.plot.setLabel("left", "Row")
        self.plot.setLabel("bottom", "Column")
        self.plot.invertY(True)
        self.plot.setAspectLocked(True)

        self.img = pg.ImageItem()
        self.plot.addItem(self.img)
        self.img.setLevels((YMIN, YMAX))

        cmap = pg.colormap.get("plasma")
        self.img.setColorMap(cmap)

        self.cbar = pg.ColorBarItem(values=(YMIN, YMAX), colorMap=cmap, label="ADC Value")
        self.cbar.setImageItem(self.img)
        self.graphics_layout.addItem(self.cbar)

        # Status labels
        self.status_label = QtWidgets.QLabel("Waiting for data...")
        layout.addWidget(self.status_label)

        self.stats_label = QtWidgets.QLabel(
            "Frames: 0 | Bad: 0 | Bytes: 0 | Last row: - | Last col_start: -"
        )
        layout.addWidget(self.stats_label)

        self.refresh_image()

    def _normalize_range(self, start: int, end: int, limit: int):
        # End is treated as inclusive in UI. If set to limit, clamp to limit-1.
        s = int(np.clip(start, 0, limit - 1))
        e = int(np.clip(end, 0, limit))
        if e == limit:
            e = limit - 1
        if s > e:
            s, e = e, s
        return s, e

    def apply_view_range(self):
        r0, r1 = self._normalize_range(self.row_start_spin.value(), self.row_end_spin.value(), ROWS)
        c0, c1 = self._normalize_range(self.col_start_spin.value(), self.col_end_spin.value(), COLS)

        self.view_row_start, self.view_row_end = r0, r1
        self.view_col_start, self.view_col_end = c0, c1

        self.row_start_spin.setValue(r0)
        self.row_end_spin.setValue(r1)
        self.col_start_spin.setValue(c0)
        self.col_end_spin.setValue(c1)

        self.refresh_image()
        self.status_label.setText(f"View range set: row {r0}-{r1}, col {c0}-{c1}")

    def reset_view_range(self):
        self.view_row_start = 0
        self.view_row_end = ROWS - 1
        self.view_col_start = 0
        self.view_col_end = COLS - 1

        self.row_start_spin.setValue(0)
        self.row_end_spin.setValue(ROWS)
        self.col_start_spin.setValue(0)
        self.col_end_spin.setValue(COLS)

        self.refresh_image()
        self.status_label.setText("View range reset to full matrix")

    def open_auto_port(self):
        port = find_serial_port()
        if port is None:
            self.port_label.setText("Port: no serial port found")
            self.status_label.setText("No serial port found")
            return

        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(port, baudrate=BAUDRATE, timeout=0.01)
            self.port_label.setText(f"Port: {port}")
            self.status_label.setText(f"Connected to {port}")
        except Exception as e:
            self.ser = None
            self.port_label.setText("Port: open failed")
            self.status_label.setText(f"Failed to open {port}: {e}")

    def reconnect(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.rx_state = 0
        self.payload.clear()
        self.open_auto_port()

    def clear_data(self):
        self.data[:] = np.nan
        self.frame_count = 0
        self.bad_frame_count = 0
        self.byte_count = 0
        self.last_row = None
        self.last_col_start = None
        self.rx_state = 0
        self.payload.clear()

        self.refresh_image()
        self.update_stats_label()
        self.status_label.setText("Data cleared")

    def refresh_image(self):
        r0 = self.view_row_start
        r1 = self.view_row_end + 1
        c0 = self.view_col_start
        c1 = self.view_col_end + 1

        view = self.data[r0:r1, c0:c1]
        self.img.setImage(view, autoLevels=False)
        self.img.setRect(QtCore.QRectF(float(c0), float(r0), float(c1 - c0), float(r1 - r0)))
        self._sync_plot_view_range(r0, r1, c0, c1)

    def _sync_plot_view_range(self, r0: int, r1: int, c0: int, c1: int):
        # Lock viewport to selected matrix window so axes start from chosen row/col.
        self.plot.setXRange(float(c0), float(c1), padding=0.0)
        self.plot.setYRange(float(r0), float(r1), padding=0.0)

    def update_levels(self):
        valid = self.data[np.isfinite(self.data)]
        if valid.size == 0:
            return

        lo = float(np.min(valid))
        hi = float(np.max(valid))
        if hi <= lo:
            hi = lo + 1.0

        self.img.setLevels((lo, hi))
        self.cbar.setLevels((lo, hi))

    def update_stats_label(self):
        self.stats_label.setText(
            f"Frames: {self.frame_count} | Bad: {self.bad_frame_count} | "
            f"Bytes: {self.byte_count} | Last row: {self.last_row} | Last col_start: {self.last_col_start}"
        )

    def update_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            n = self.ser.in_waiting
            if n <= 0:
                return

            chunk = self.ser.read(n)
            self.byte_count += len(chunk)

            updated = False
            for b in chunk:
                if self.feed_byte(b):
                    updated = True

            if updated:
                self.refresh_image()

            self.update_stats_label()

        except Exception as e:
            self.status_label.setText(f"Serial error: {e}")

    def feed_byte(self, b: int) -> bool:
        """
        State machine parser:
        state 0: wait for 0xA5
        state 1: wait for 0x5A
        state 2: collect 24 payload bytes
        """
        if self.rx_state == 0:
            if b == HEADER1:
                self.rx_state = 1
            return False

        if self.rx_state == 1:
            if b == HEADER2:
                self.payload.clear()
                self.rx_state = 2
            elif b == HEADER1:
                # stay in state 1 in case repeated A5 comes
                self.rx_state = 1
            else:
                self.rx_state = 0
            return False

        if self.rx_state == 2:
            self.payload.append(b)

            if len(self.payload) < 24:
                return False

            frame = bytes([HEADER1, HEADER2]) + bytes(self.payload)
            self.rx_state = 0
            self.payload.clear()

            return self.process_frame(frame)

        self.rx_state = 0
        return False

    def process_frame(self, frame: bytes) -> bool:

        if len(frame) != FRAME_SIZE:
            self.bad_frame_count += 1
            return False

        if frame[0] != HEADER1 or frame[1] != HEADER2:
            self.bad_frame_count += 1
            return False

        row = frame[2] * 256 + frame[3]
        col_start = frame[5]

        if not (0 <= row < WIRE_ROWS):
            self.bad_frame_count += 1
            return False

        if not (0 <= col_start < 8):
            self.bad_frame_count += 1
            return False

        adc_values = []
        for i in range(10):
            hi = frame[6 + 2 * i]
            lo = frame[7 + 2 * i]
            adc = hi * 256 + lo
            adc_values.append(adc)

        for k, adc in enumerate(adc_values):
            col = col_start + 8 * k
            if row < ROWS and 0 <= col < COLS:
                self.data[row, col] = adc

        self.frame_count += 1
        self.last_row = row
        self.last_col_start = col_start
        self.status_label.setText(
            f"Frame #{self.frame_count}: row={row}, col_start={col_start}"
        )

        return True

    def save_both(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_base = f"heatmap_{timestamp}"

        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save PNG + CSV",
            default_base,
            "PNG Files (*.png)"
        )
        if not file_path:
            return

        path = Path(file_path)
        if path.suffix.lower() != ".png":
            path = path.with_suffix(".png")

        png_path = path
        csv_path = path.with_suffix(".csv")

        try:
            exporter = pg.exporters.ImageExporter(self.plot.scene())
            exporter.export(str(png_path))

            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["row\\col"] + list(range(COLS)))
                for r in range(ROWS):
                    vals = [("" if np.isnan(v) else int(v)) for v in self.data[r]]
                    writer.writerow([r] + vals)

            self.status_label.setText(f"Saved: {png_path.name} and {csv_path.name}")

        except Exception as e:
            self.status_label.setText(f"Save failed: {e}")

    def closeEvent(self, event):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        event.accept()


def main():
    pg.setConfigOptions(imageAxisOrder="row-major")

    app = QtWidgets.QApplication(sys.argv)
    win = HeatmapViewer()
    win.resize(1050, 860)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
