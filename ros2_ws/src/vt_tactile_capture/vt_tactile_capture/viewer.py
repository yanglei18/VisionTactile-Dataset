"""ROS-only heatmap consumer. Qt is imported only when opening the viewer.

The displayed ADC grid is a rolling state, not a synchronous tactile frame.
ROS source age and a local steady-clock arrival watchdog are kept separate so
recorded state can be viewed with --ros-args -p use_sim_time:=true and /clock.
"""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import sys
import time

from .grid import COLS, ROWS, SCHEMA_VERSION, normalize_state


YMIN, YMAX = 20000, 24000


def normalize_range(start: int, end: int, limit: int = ROWS) -> tuple[int, int]:
    """Inclusive UI bounds; upper sentinel 20 means the last physical row 19."""
    start, end = max(0, min(start, limit - 1)), max(0, min(end, limit - 1))
    return min(start, end), max(start, end)


class ViewerModel:
    """Single Qt-thread state store, also usable without Qt or ROS installed."""

    def __init__(self, *, monotonic_ns=time.monotonic_ns, stale_timeout_sec=2.0, expected_hand=None):
        if expected_hand not in (None, "left", "right"):
            raise ValueError("expected_hand must be left or right")
        self.expected_hand = expected_hand
        self._monotonic_ns = monotonic_ns
        self.stale_timeout_ns = int(stale_timeout_sec * 1e9)
        self.data = [math.nan] * (ROWS * COLS)
        self.display_block_realtime_ns = [0] * (ROWS * 8)
        self.display_block_monotonic_ns = [0] * (ROWS * 8)
        self.message = None
        self.arrival_ns = None
        self.identity = None
        self.rejected = 0
        self.state_listeners = []

    def clear_display(self) -> None:
        """Clear this view only; never command capture hardware or reset a stream."""
        self.data = [math.nan] * (ROWS * COLS)
        self.display_block_realtime_ns = [0] * (ROWS * 8)
        self.display_block_monotonic_ns = [0] * (ROWS * 8)
        self.message = None
        self.arrival_ns = None
        self.identity = None

    def observe(self, message) -> bool:
        count = ROWS * COLS
        try:
            message = normalize_state(message)
        except (AttributeError, KeyError, TypeError, ValueError):
            self.rejected += 1
            return False
        if (message.schema_version != SCHEMA_VERSION or message.rows != ROWS or message.cols != COLS
                or getattr(message, "hand", "") not in ("left", "right")
                or (self.expected_hand is not None and message.hand != self.expected_hand)
                or len(message.adc) != count or len(message.valid_mask) != count
                or len(message.updated_mask) != count
                or len(message.block_realtime_ns) != ROWS * 8
                or len(message.block_monotonic_ns) != ROWS * 8):
            self.rejected += 1
            return False
        identity = (message.session_id, message.stream_epoch)
        has_valid = any(message.valid_mask)
        # An empty reconnect event keeps the previous image visibly retained.
        # Once the new epoch contributes data, clear it before filling so the
        # displayed grid never mixes measurements across acquisition epochs.
        if self.identity != identity and has_valid:
            self.data = [math.nan] * count
            self.display_block_realtime_ns = [0] * (ROWS * 8)
            self.display_block_monotonic_ns = [0] * (ROWS * 8)
        # Keep the last measured image on disconnection or stale masks. Health
        # labels describe usability; an empty status update is not zero-touch.
        for index, valid in enumerate(message.valid_mask):
            if valid:
                self.data[index] = float(message.adc[index])
                row, column = divmod(index, COLS)
                block_index = row * 8 + column % 8
                self.display_block_realtime_ns[block_index] = int(message.block_realtime_ns[block_index])
                self.display_block_monotonic_ns[block_index] = int(message.block_monotonic_ns[block_index])
        if has_valid:
            self.identity = identity
        self.message = message
        self.arrival_ns = self._monotonic_ns()
        for listener in tuple(self.state_listeners):
            listener(message)
        return True

    def status(self, ros_now_ns: int, *, use_sim_time: bool = False) -> str:
        if self.message is None:
            return "Waiting for tactile state (no hardware connection in this viewer)."
        message = self.message
        arrival_age = max(0, self._monotonic_ns() - self.arrival_ns)
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        source_age = max(0, ros_now_ns - stamp_ns) if stamp_ns and ros_now_ns else None
        stale_arrival = arrival_age > self.stale_timeout_ns
        stale_source = source_age is not None and source_age > self.stale_timeout_ns
        valid_count = sum(bool(v) for v in message.valid_mask)
        unhealthy = (stale_arrival or stale_source or not message.connected
                     or not message.time_valid or not valid_count)
        prefix = "RETAINED / STALE" if unhealthy else "LIVE ROLLING ADC"
        if use_sim_time:
            prefix = "REPLAY (retained/stale)" if unhealthy else "REPLAY"
            if stale_arrival:
                prefix = "REPLAY — NO MESSAGES (paused/stopped?)"
        source_text = "unknown" if source_age is None else f"{source_age / 1e9:.2f}s"
        clock = "ROS playback" if use_sim_time else "ROS system"
        return (f"{prefix} | source age ({clock}): {source_text} | "
                f"arrival age (steady): {arrival_age / 1e9:.2f}s | "
                f"valid: {valid_count}/{ROWS * COLS} | "
                f"freshness limit: {message.max_age_ns / 1e9:.2f}s | "
                f"{message.reason or 'ok'}")

    def save_csv(self, path: str | Path) -> None:
        """Full 20x20 physical displayed ADC grid; missing values stay empty."""
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["row\\col"] + list(range(COLS)))
            for row in range(ROWS):
                values = self.data[row * COLS:(row + 1) * COLS]
                writer.writerow([row] + ["" if math.isnan(value) else int(value) for value in values])

    def save_metadata(self, path: str | Path) -> None:
        """Sidecar distinguishes historical visible pixels from currently valid data."""
        message = self.message
        metadata = {
            "schema_version": SCHEMA_VERSION,
            "csv_semantics": "displayed rolling ADC history, not a synchronous scan or archival sample stream",
            "unit": "adc_counts", "rows": ROWS, "cols": COLS,
            "hand": message.hand if message else self.expected_hand,
            "session_id": message.session_id if message else "",
            "stream_epoch": message.stream_epoch if message else 0,
            "display_session_id": self.identity[0] if self.identity else "",
            "display_stream_epoch": self.identity[1] if self.identity else 0,
            "source_sequence": message.source_sequence if message else 0,
            "source_realtime_ns": message.host_realtime_ns if message else 0,
            "source_monotonic_ns": message.host_monotonic_ns if message else 0,
            "snapshot_monotonic_ns": message.snapshot_monotonic_ns if message else 0,
            "connected": message.connected if message else False,
            "time_valid": message.time_valid if message else False,
            "reason": message.reason if message else "no_state",
            "current_valid_mask": [bool(value) for value in message.valid_mask] if message else [False] * (ROWS * COLS),
            "display_block_realtime_ns": self.display_block_realtime_ns,
            "display_block_monotonic_ns": self.display_block_monotonic_ns,
        }
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2)


def create_viewer_node(model: ViewerModel, **kwargs):
    """Build a lightweight ROS subscriber; importing this module needs no ROS."""
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from vt_tactile_msgs.msg import TactileState

    class TactileViewerNode(Node):
        def __init__(self):
            super().__init__("tactile_viewer", **kwargs)
            hand = self.declare_parameter("hand", "").value
            if not hand:
                parts = self.get_namespace().strip("/").split("/")
                hand = next((part for part in reversed(parts) if part in ("left", "right")), "right")
            if hand not in ("left", "right"):
                self.destroy_node()
                raise ValueError("hand must be left or right")
            model.expected_hand = hand
            self.state_subscription = self.create_subscription(
                TactileState, "state", model.observe,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE),
            )

    return TactileViewerNode()


def create_window(model: ViewerModel, node=None, *, hand_layout=None,
                  baseline_path=None, initial_view="hand", hand=None):
    """Load optional GUI dependencies without importing serial or capture code."""
    import numpy as np
    import pyqtgraph as pg
    import pyqtgraph.exporters
    from PyQt5 import QtCore, QtGui, QtWidgets
    from .display_baseline import DisplayBaseline
    from .hand_layout import load_layout, unmapped_layout
    from .presentation import delta_color_stops, region_summaries

    hand = hand or model.expected_hand or (hand_layout.hand if hand_layout is not None else "right")
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    if hand_layout is None:
        hand_layout = load_layout() if hand == "right" else unmapped_layout(hand)
    if hand_layout.hand != hand:
        raise ValueError("layout hand does not match viewer hand")
    if initial_view not in ("hand", "matrix"):
        raise ValueError("initial_view must be hand or matrix")
    if not hand_layout.points:
        initial_view = "matrix"
    hand_label = "左手" if hand == "left" else "右手"
    # Preserve the established right-hand seven-area summary for the original
    # profile; every custom profile uses its own explicit anatomical regions.
    summary_layout = (None if hand_layout.name == "Right glove — candidate tactile layout"
                      and dict(hand_layout.source).get("dataset") == "tactile_mapping_20261005_02"
                      and hand_layout == load_layout() else hand_layout)

    pg.setConfigOptions(imageAxisOrder="row-major")

    class HandCells(pg.GraphicsObject):
        """One cached picture of region-local cells; no invented interpolation."""

        def __init__(self, points):
            super().__init__()
            by_address = {(p.row, p.col): p for p in points}
            self.paths = []
            self.colors = np.full((len(points), 4), [65, 72, 80, 255], dtype=np.uint8)
            unit = QtGui.QPainterPath()
            unit.addRoundedRect(QtCore.QRectF(-.42, -.42, .84, .84), .12, .12)
            self._rect = QtCore.QRectF()
            for point in points:
                vectors = []
                for dr, dc in ((0, 1), (1, 0)):
                    vector = None
                    for sign in (1, -1):
                        neighbor = by_address.get((point.row + sign * dr, point.col + sign * dc))
                        if neighbor is not None and neighbor.region == point.region:
                            vector = (sign * (neighbor.x - point.x), sign * (neighbor.y - point.y))
                            break
                    vectors.append(vector or ((.8, 0) if dc else (0, .8)))
                u, v = vectors
                path = QtGui.QTransform(u[0], u[1], v[0], v[1], point.x, point.y).map(unit)
                self.paths.append(path)
                self._rect = self._rect.united(path.boundingRect())
            self.set_colors(self.colors)

        def set_colors(self, colors):
            self.colors = np.asarray(colors, dtype=np.uint8).copy()
            self.picture = QtGui.QPicture()
            painter = QtGui.QPainter(self.picture)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            painter.setPen(QtCore.Qt.NoPen)
            for path, color in zip(self.paths, self.colors):
                painter.setBrush(pg.mkBrush(*map(int, color)))
                painter.drawPath(path)
            painter.end()
            self.update()

        def paint(self, painter, *_):
            painter.drawPicture(0, 0, self.picture)

        def boundingRect(self):
            return self._rect

    class HeatmapViewer(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle(f"Tactile — {hand_label} · 手形触觉 / 原始矩阵 · ROS")
            self.view_range = (0, ROWS - 1, 0, COLS - 1)
            self._last_message = None
            self.baseline = DisplayBaseline(expected_hand=hand)
            self._reference_just_finished = False
            self._pending_baseline = baseline_path
            self.hand_layout = hand_layout
            self._hand_indices = np.asarray([p.row * COLS + p.col for p in hand_layout.points], dtype=np.int64)
            self._levels = (YMIN, YMAX)
            self._cmap = pg.colormap.get("plasma")
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            layout = QtWidgets.QVBoxLayout(central)
            controls = QtWidgets.QHBoxLayout()
            layout.addLayout(controls)
            controls.addWidget(QtWidgets.QLabel(f"{hand_label}触觉 · 相对变化，不是压力"))
            self.connection_label = QtWidgets.QLabel("等待触觉数据")
            controls.addWidget(self.connection_label)
            controls.addStretch()
            save = QtWidgets.QPushButton("Save PNG + CSV")
            save.clicked.connect(self.save_both)
            controls.addWidget(save)
            self.clear_button = QtWidgets.QPushButton("清除显示和参考")
            self.clear_button.clicked.connect(self.clear_display)
            controls.addWidget(self.clear_button)
            display_controls = QtWidgets.QHBoxLayout()
            layout.addLayout(display_controls)
            self.value_mode = QtWidgets.QComboBox()
            self.value_mode.addItem("原始 ADC", "raw")
            self.value_mode.addItem("相对静止参考 ΔADC", "delta")
            display_controls.addWidget(self.value_mode)
            self.range_preset = QtWidgets.QComboBox()
            self.range_preset.addItem("轻触显示（±3000 ADC）", 3000)
            self.range_preset.addItem("强响应（±16000 ADC）", 16000)
            self.range_preset.addItem("自定义色标", None)
            self.range_preset.setEnabled(False)
            display_controls.addWidget(self.range_preset)
            self.anchors_toggle = QtWidgets.QCheckBox("显示定位圈")
            display_controls.addWidget(self.anchors_toggle)
            self.details_toggle = QtWidgets.QCheckBox("高级 / 诊断")
            display_controls.addWidget(self.details_toggle)
            display_controls.addStretch()
            self.advanced_widget = QtWidgets.QWidget()
            advanced_layout = QtWidgets.QVBoxLayout(self.advanced_widget)
            advanced_layout.setContentsMargins(0, 0, 0, 0)
            advanced_controls = QtWidgets.QHBoxLayout()
            advanced_layout.addLayout(advanced_controls)
            advanced_controls.addWidget(QtWidgets.QLabel("自定义色标："))
            self.level_min = QtWidgets.QSpinBox()
            self.level_max = QtWidgets.QSpinBox()
            for spin, value in ((self.level_min, YMIN), (self.level_max, YMAX)):
                spin.setRange(-65535, 65535)
                spin.setValue(value)
                advanced_controls.addWidget(spin)
            levels_button = QtWidgets.QPushButton("应用色标")
            levels_button.clicked.connect(self.apply_levels)
            advanced_controls.addWidget(levels_button)
            advanced_controls.addWidget(QtWidgets.QLabel("显示中性区 ±ADC："))
            self.deadband = QtWidgets.QSpinBox()
            self.deadband.setRange(0, 5000)
            self.deadband.setValue(200)
            self.deadband.setToolTip("仅将参考附近的数值显示为中性色；不修改 ADC、差值、ROS 或导出数据。")
            advanced_controls.addWidget(self.deadband)
            self.baseline_button = QtWidgets.QPushButton("采集静止参考（3 秒）")
            self.baseline_button.clicked.connect(self.start_baseline)
            controls.addWidget(self.baseline_button)
            load_baseline = QtWidgets.QPushButton("载入参考 NPZ")
            load_baseline.clicked.connect(self.load_baseline_dialog)
            advanced_controls.addWidget(load_baseline)
            self.reference_label = QtWidgets.QLabel("尚无静止参考；可查看原始 ADC。")
            self.reference_label.setWordWrap(True)
            layout.addWidget(self.reference_label)
            self.matrix_controls = QtWidgets.QWidget()
            controls = QtWidgets.QHBoxLayout(self.matrix_controls)
            controls.setContentsMargins(0, 0, 0, 0)
            layout.addWidget(self.matrix_controls)
            controls.addWidget(QtWidgets.QLabel("原始矩阵裁剪："))
            self.bounds = {}
            for label in ("Row", "Col"):
                controls.addWidget(QtWidgets.QLabel(label + ":"))
                for edge, value in (("start", 0), ("end", ROWS)):
                    spin = QtWidgets.QSpinBox()
                    spin.setRange(0, ROWS - 1 if edge == "start" else ROWS)
                    spin.setValue(value)
                    self.bounds[(label, edge)] = spin
                    controls.addWidget(spin)
            apply_button = QtWidgets.QPushButton("Apply Range")
            apply_button.clicked.connect(self.apply_range)
            controls.addWidget(apply_button)
            full = QtWidgets.QPushButton("Full Range")
            full.clicked.connect(self.full_range)
            controls.addWidget(full)
            mapped = QtWidgets.QPushButton("查看 0–20 行/列")
            mapped.clicked.connect(self.mapped_range)
            controls.addWidget(mapped)
            self.tabs = QtWidgets.QTabWidget()
            layout.addWidget(self.tabs, 1)
            hand_page = QtWidgets.QWidget()
            hand_page_layout = QtWidgets.QHBoxLayout(hand_page)
            hand_page_layout.setContentsMargins(0, 0, 0, 0)
            self.hand_graphics = pg.GraphicsLayoutWidget()
            hand_page_layout.addWidget(self.hand_graphics, 1)
            sidebar = QtWidgets.QWidget()
            sidebar.setMinimumWidth(215)
            sidebar.setMaximumWidth(265)
            sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
            sidebar_layout.setSpacing(3)
            sidebar_layout.setContentsMargins(8, 6, 8, 6)
            self.region_title = QtWidgets.QLabel("区域最大变化 · ΔADC")
            sidebar_layout.addWidget(self.region_title)
            self.region_widgets = {}
            for summary in region_summaries(np.full((ROWS, COLS), np.nan), np.zeros((ROWS, COLS), dtype=bool), layout=summary_layout):
                title = QtWidgets.QLabel(summary["name"] + "  —")
                bar = QtWidgets.QProgressBar()
                bar.setRange(0, 1000)
                bar.setValue(0)
                bar.setTextVisible(False)
                bar.setMaximumHeight(13)
                detail = QtWidgets.QLabel("等待有效参考与数据")
                sidebar_layout.addWidget(title)
                sidebar_layout.addWidget(bar)
                sidebar_layout.addWidget(detail)
                sidebar_layout.addSpacing(5)
                self.region_widgets[summary["key"]] = (title, bar, detail)
            self.unmapped_label = QtWidgets.QLabel("未映射地址：请切换原始矩阵查看")
            self.unmapped_label.setWordWrap(True)
            sidebar_layout.addWidget(self.unmapped_label)
            self.legend = QtWidgets.QLabel()
            self.legend.setWordWrap(True)
            sidebar_layout.addWidget(self.legend)
            sidebar_layout.addStretch()
            sidebar_scroll = QtWidgets.QScrollArea()
            sidebar_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            sidebar_scroll.setWidgetResizable(True)
            sidebar_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            sidebar_scroll.setWidget(sidebar)
            sidebar_scroll.setMinimumWidth(235)
            sidebar_scroll.setMaximumWidth(285)
            hand_page_layout.addWidget(sidebar_scroll)
            self.tabs.addTab(hand_page, "手形触觉（候选布局）")
            self.graphics = pg.GraphicsLayoutWidget()
            self.tabs.addTab(self.graphics, "有效触觉矩阵（20×20 / 400 点）")
            self.tabs.setTabEnabled(0, bool(hand_layout.points))
            self.tabs.setTabToolTip(0, hand_layout.note)
            self.tabs.setCurrentIndex(0 if initial_view == "hand" else 1)
            self.matrix_controls.setVisible(initial_view == "matrix")
            self.tabs.currentChanged.connect(lambda index: self.matrix_controls.setVisible(index == 1))
            self.plot = self.graphics.addPlot()
            self.plot.setLabel("left", "Row")
            self.plot.setLabel("bottom", "Column")
            self.plot.invertY(True)
            self.plot.setAspectLocked(True)
            self.image = pg.ImageItem(axisOrder="row-major")
            self.plot.addItem(self.image)
            cmap = self._cmap
            self.image.setColorMap(cmap)
            self.image.setLevels((YMIN, YMAX))
            self.colorbar = pg.ColorBarItem(values=(YMIN, YMAX), colorMap=cmap,
                                           label="ADC counts", interactive=False)
            self.colorbar.setImageItem(self.image)
            self.colorbar.axis.setWidth(72)
            self.graphics.addItem(self.colorbar)
            self.hand_plot = self.hand_graphics.addPlot()
            self.hand_plot.hideAxis("left")
            self.hand_plot.hideAxis("bottom")
            self.hand_plot.setAspectLocked(True)
            self.hand_plot.setTitle(f"{hand_label} · 掌心朝向观察者 · 候选示意比例")
            outline = QtGui.QPainterPath()
            for index, (x, y) in enumerate(hand_layout.outline):
                if index == 0:
                    outline.moveTo(x, y)
                else:
                    outline.lineTo(x, y)
            outline.closeSubpath()
            self.hand_outline = QtWidgets.QGraphicsPathItem(outline)
            self.hand_outline.setPen(pg.mkPen("#5a7188", width=1.5))
            self.hand_outline.setBrush(pg.mkBrush("#172430"))
            self.hand_plot.addItem(self.hand_outline)
            self.hand_cells = HandCells(hand_layout.points)
            self.hand_plot.addItem(self.hand_cells)
            self.hand_scatter = pg.ScatterPlotItem(
                size=14, hoverable=True, hoverSize=18, pen=None, brush=pg.mkBrush(None),
                hoverPen=pg.mkPen("#ffffff", width=1.5))
            self.hand_scatter.setData([{"pos": (p.x, p.y), "data": p} for p in hand_layout.points])
            self.hand_scatter.sigHovered.connect(self.show_pixel_info)
            self.hand_plot.addItem(self.hand_scatter)
            verified = [p for p in hand_layout.points if p.verified_anchor]
            self.anchor_scatter = pg.ScatterPlotItem(
                x=[p.x for p in verified], y=[p.y for p in verified], size=15,
                pen=pg.mkPen("#61e4ee", width=1.8), brush=pg.mkBrush(None))
            self.hand_plot.addItem(self.anchor_scatter)
            self.anchor_scatter.setVisible(False)
            self.anchors_toggle.toggled.connect(self.anchor_scatter.setVisible)
            for text, x, y in hand_layout.labels:
                label = pg.TextItem(text, color="#d8e8ef", anchor=(.5, .5))
                label.setPos(x, y)
                self.hand_plot.addItem(label)
            xs, ys = zip(*hand_layout.outline)
            self._hand_extent = (min(xs) - .6, max(xs) + .6, min(ys) - 1.4, max(ys) + 1.4)
            self.hand_plot.getViewBox().sigResized.connect(self.fit_hand)
            self.fit_hand()
            self.hand_colorbar = pg.ColorBarItem(values=self._levels, colorMap=cmap,
                                                label="ADC counts", interactive=False)
            self.hand_colorbar.axis.setWidth(72)
            self.hand_graphics.addItem(self.hand_colorbar)
            anchor_count = sum(point.verified_anchor for point in hand_layout.points)
            self.layout_label = QtWidgets.QLabel(
                f"{hand_label}候选手形 · {anchor_count} 个已观测锚点，其余位置未验证 · 原始 ADC 不变"
                if hand_layout.points else f"{hand_label}尚未选择解剖布局 · 仅原始矩阵 · 不推断地址对应")
            if hand_layout.points and not anchor_count:
                self.layout_label.setText(self.layout_label.text() + " · 未验证候选，需实测确认")
            self.layout_label.setWordWrap(True)
            self.layout_label.setToolTip(hand_layout.note)
            layout.addWidget(self.layout_label)
            self.pixel_label = QtWidgets.QLabel("将鼠标移到触点，可查看原始行列、ADC、参考差值与有效性。")
            layout.addWidget(self.pixel_label)
            self.status_label = QtWidgets.QLabel()
            self.status_label.setWordWrap(True)
            advanced_layout.addWidget(self.status_label)
            self.stats_label = QtWidgets.QLabel("Not force/pressure; retained pixels may be stale. CSV exports the displayed history.")
            self.stats_label.setWordWrap(True)
            advanced_layout.addWidget(self.stats_label)
            layout.addWidget(self.advanced_widget)
            self.advanced_widget.setVisible(False)
            self.details_toggle.toggled.connect(self.advanced_widget.setVisible)
            self.value_mode.currentIndexChanged.connect(self.change_value_mode)
            self.range_preset.currentIndexChanged.connect(self.change_preset)
            self.deadband.valueChanged.connect(self.update_colormap)
            self.refresh_image()
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(33)
            model.state_listeners.append(self.observe_reference)
            self.resize(1200, 940)

        def observe_reference(self, message):
            # Every accepted ROS event, including brief disconnects between
            # rendered frames, reaches reference health handling on Qt's thread.
            collecting = self.baseline.collecting
            self.baseline.observe(message, time.monotonic_ns())
            if collecting and not self.baseline.collecting:
                self._reference_just_finished = True

        def closeEvent(self, event):
            self.timer.stop()
            if self.observe_reference in model.state_listeners:
                model.state_listeners.remove(self.observe_reference)
            super().closeEvent(event)

        def tick(self):
            if node is not None:
                import rclpy
                from rclpy.executors import ExternalShutdownException
                if not rclpy.ok(context=node.context):
                    self.close()
                    return
                # Bounded, non-blocking work on Qt's thread; no serial here.
                # Allow /clock and state callbacks in the same display tick.
                try:
                    for _ in range(4):
                        rclpy.spin_once(node, timeout_sec=0.0)
                except ExternalShutdownException:
                    self.close()
                    return
                now_ns = node.get_clock().now().nanoseconds
                use_sim_time = node.get_parameter("use_sim_time").value
            else:
                now_ns, use_sim_time = time.time_ns(), False
            was_collecting = self.baseline.collecting or self._reference_just_finished
            self._reference_just_finished = False
            if self._last_message is not model.message:
                self._last_message = model.message
                if model.message is not None:
                    self.baseline.observe(model.message, time.monotonic_ns())
                    if self._pending_baseline and model.identity is not None:
                        path, self._pending_baseline = self._pending_baseline, None
                        self.load_reference(path)
                self.refresh_image()
            self.baseline.poll(time.monotonic_ns())
            if was_collecting and not self.baseline.collecting:
                if self.baseline.values is not None:
                    self.value_mode.setCurrentIndex(1)
                self.refresh_image()
            self.baseline_button.setEnabled(not self.baseline.collecting)
            if self.baseline.collecting:
                reference_text = f"请保持全部触觉区域不受压，采集 3 秒静止参考中（{self.baseline.samples_count} 帧）。"
            elif self.baseline.values is not None:
                reference_text = "静止参考已就绪 · 暖色/蓝色表示相对参考的正/负变化，不是压力。"
            else:
                reference_text = "尚无静止参考：请让触觉面不受压，然后点击【采集静止参考（3 秒）】。"
            self.reference_label.setText(reference_text)
            self.reference_label.setToolTip(self.baseline.status)
            health = model.status(now_ns, use_sim_time=use_sim_time)
            self.status_label.setText(health)
            healthy = health.startswith("LIVE") or health == "REPLAY" or health.startswith("REPLAY |")
            if model.message is None:
                self.connection_label.setText("等待触觉数据")
            else:
                self.connection_label.setText("● 实时数据" if healthy else "历史画面 / 数据暂不可用")
            self.region_title.setText("区域最大变化 · ΔADC" if healthy else "区域变化 · 历史 / 未就绪")
            matching_hand = (model.message is None or
                             getattr(model.message, "hand", "") == hand_layout.hand)
            if not matching_hand:
                self.hand_plot.setTitle(f"手别不匹配：此候选布局用于{hand_label}；请核对订阅来源")
            elif "STALE" in health or "stale" in health or "NO MESSAGES" in health:
                self.hand_plot.setTitle(f"{hand_label} · 保留的历史画面（非新测量）")
            else:
                self.hand_plot.setTitle(f"{hand_label} · 掌心朝向观察者 · 候选示意比例")
            if model.message is not None:
                message = model.message
                self.stats_label.setText(
                    f"Session {message.session_id} | epoch {message.stream_epoch} | "
                    f"display origin {model.identity or 'none'} | "
                    f"source sequence {message.source_sequence} | dropped batches {message.producer_dropped_total} | "
                    f"rejected states {model.rejected}. Retained pixels may be stale; CSV is displayed history."
                )

        def refresh_image(self):
            r0, r1, c0, c1 = self.view_range
            matrix = np.asarray(model.data, dtype=np.float32).reshape(ROWS, COLS)
            if self.value_mode.currentData() == "delta":
                matrix = (self.baseline.subtract(matrix) if self.baseline.values is not None
                          else np.full((ROWS, COLS), np.nan, dtype=np.float32))
            self.image.setImage(matrix[r0:r1 + 1, c0:c1 + 1], autoLevels=False)
            self.image.setRect(QtCore.QRectF(c0, r0, c1 - c0 + 1, r1 - r0 + 1))
            self.plot.setXRange(c0, c1 + 1, padding=0)
            self.plot.setYRange(r0, r1 + 1, padding=0)
            values = matrix.ravel()[self._hand_indices].copy()
            if model.message is not None and getattr(model.message, "hand", "") != hand_layout.hand:
                values[:] = np.nan
            lo, hi = self._levels
            colors = self._cmap.map(np.clip(np.nan_to_num((values - lo) / (hi - lo), nan=0), 0, 1), mode="byte")
            colors[~np.isfinite(values)] = [65, 72, 80, 255]
            self.hand_cells.set_colors(colors)
            self.refresh_regions(matrix)

        def fit_hand(self):
            if not hasattr(self, "_hand_extent"):
                return
            x0, x1, y0, y1 = self._hand_extent
            self.hand_plot.setRange(xRange=(x0, x1), yRange=(y0, y1), padding=0,
                                    disableAutoRange=True)

        def refresh_regions(self, matrix):
            message = model.message
            if self.value_mode.currentData() == "delta":
                self.legend.setText("暖色：正变化 · 蓝色：负变化\n中性：参考附近 · 深灰：未就绪\n地址数不是接触面积，ΔADC不是压力。")
            else:
                self.legend.setText("当前显示原始 ADC，颜色对应色标，不表示正负变化。\n采集静止参考后可查看 ΔADC。")
            usable = (self.value_mode.currentData() == "delta" and self.baseline.values is not None
                      and message is not None and message.connected and message.time_valid
                      and getattr(message, "hand", "") == hand_layout.hand)
            valid = (np.asarray(message.valid_mask, dtype=bool).reshape(ROWS, COLS)
                     if usable else np.zeros((ROWS, COLS), dtype=bool))
            summaries = region_summaries(matrix, valid, threshold=self.deadband.value(), layout=summary_layout)
            for summary in summaries:
                title, bar, detail = self.region_widgets[summary["key"]]
                peak = summary["peak_delta"]
                scale = max(1, self._levels[1] if peak is not None and peak >= 0 else -self._levels[0])
                title.setText(summary["name"] + (f"  {peak:+.0f}" if peak is not None else "  —"))
                bar.setValue(min(1000, round(abs(peak) / scale * 1000)) if peak is not None else 0)
                color = ("#80909b" if peak is None or abs(peak) <= self.deadband.value()
                         else "#e98a36" if peak > 0 else "#378dd1")
                if bar.property("display_color") != color:
                    bar.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; }}")
                    bar.setProperty("display_color", color)
                if peak is None:
                    detail.setText("等待有效参考 / 数据" if self.value_mode.currentData() == "delta"
                                   else "切换 ΔADC 查看变化")
                else:
                    detail.setText(f"超过显示阈值 {summary['active_cells']}/{summary['valid_cells']} 地址"
                                   + (" · 超色标" if not self._levels[0] <= peak <= self._levels[1] else ""))
            outside = valid.copy().ravel()
            outside[self._hand_indices] = False
            candidates = np.flatnonzero(outside & np.isfinite(matrix.ravel()))
            if len(candidates):
                index = int(candidates[np.argmax(np.abs(matrix.ravel()[candidates]))])
                r, c = divmod(index, COLS)
                self.unmapped_label.setText(f"未映射地址最大变化：\n{matrix.ravel()[index]:+.0f} ΔADC · ({r}, {c})\n可在原始矩阵核对")
            else:
                self.unmapped_label.setText("有效区域：20×20 / 400 点；其余协议地址不参与显示")

        def change_value_mode(self):
            is_delta = self.value_mode.currentData() == "delta"
            self.range_preset.setEnabled(is_delta)
            self.deadband.setEnabled(is_delta)
            limit = self.range_preset.currentData() or 3000
            levels = (-limit, limit) if is_delta else (YMIN, YMAX)
            self.level_min.setValue(levels[0])
            self.level_max.setValue(levels[1])
            self.apply_levels(preset_change=True)

        def change_preset(self):
            limit = self.range_preset.currentData()
            if self.value_mode.currentData() != "delta" or limit is None:
                return
            self.level_min.setValue(-limit)
            self.level_max.setValue(limit)
            self.apply_levels(preset_change=True)

        def apply_levels(self, _checked=False, *, preset_change=False):
            lo, hi = self.level_min.value(), self.level_max.value()
            if lo >= hi:
                self.pixel_label.setText("色标上限必须大于下限；数据不受影响。")
                return
            self._levels = (lo, hi)
            if not preset_change and self.value_mode.currentData() == "delta":
                self.range_preset.setCurrentIndex(2)
            self.colorbar.setLevels((lo, hi))
            self.hand_colorbar.setLevels((lo, hi))
            unit = "ΔADC counts" if self.value_mode.currentData() == "delta" else "ADC counts"
            self.colorbar.getAxis("left").setLabel(unit)
            self.hand_colorbar.getAxis("left").setLabel(unit)
            self.update_colormap()

        def update_colormap(self):
            if self.value_mode.currentData() == "delta":
                positions, colors = delta_color_stops(self._levels, self.deadband.value())
                self._cmap = pg.ColorMap(positions, colors)
            else:
                self._cmap = pg.colormap.get("plasma")
            self.image.setColorMap(self._cmap)
            self.colorbar.setColorMap(self._cmap)
            self.hand_colorbar.setColorMap(self._cmap)
            self.refresh_image()

        def start_baseline(self):
            message = model.message
            if (message is None or model.identity is None or not message.connected or
                    not message.time_valid or model.arrival_ns is None or
                    not any(message.valid_mask) or
                    (message.session_id, message.stream_epoch) != model.identity or
                    model._monotonic_ns() - model.arrival_ns > 200_000_000):
                self.baseline.status = "需要新鲜、已连接的触觉数据才能采集参考。"
                self.reference_label.setText(self.baseline.status)
                return
            self.baseline.start(model.identity, time.monotonic_ns())
            self.baseline_button.setEnabled(False)
            self.reference_label.setText("请让全部触觉区域不受压，保持静止 3 秒。")
            self.refresh_image()

        def load_reference(self, path):
            if model.identity is None:
                self.baseline.status = "尚未收到数据，无法核对参考的采集身份。"
                return
            try:
                self.baseline.load(path, model.identity)
            except (ValueError, OSError, KeyError) as error:
                self.baseline.status = f"参考未加载：{error}"
            else:
                self.value_mode.setCurrentIndex(1)
            self.reference_label.setText(self.baseline.status)
            self.refresh_image()

        def load_baseline_dialog(self):
            path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "载入同一采集会话的静止参考", "", "NPZ Files (*.npz)")
            if path:
                self.load_reference(path)

        def show_pixel_info(self, _item, points, _event):
            if not points:
                return
            point = points[0].data()
            index = point.row * COLS + point.col
            value = model.data[index]
            delta = (value - self.baseline.values[point.row, point.col]
                     if self.baseline.values is not None else math.nan)
            valid = bool(model.message is not None and model.message.connected and
                         model.message.time_valid and model.message.valid_mask[index] and
                         model.arrival_ns is not None and
                         model._monotonic_ns() - model.arrival_ns <= model.stale_timeout_ns)
            self.pixel_label.setText(
                f"{point.region} · row={point.row}, col={point.col}（从 0 编号） · "
                f"ADC={value:.0f} · ΔADC={delta:.1f} · mask={'有效' if valid else '无效/历史'} · "
                f"{'实测定位点' if point.verified_anchor else '推断位置'}")

        def clear_display(self):
            model.clear_display()
            self.baseline.invalidate()
            self._last_message = None
            self.refresh_image()
            self.status_label.setText("Display cleared; capture continues unchanged.")
            self.reference_label.setText("显示及其静止参考已清除；采集和 ROS 数据保持不变。")

        def apply_range(self):
            r0, r1 = normalize_range(*(self.bounds[("Row", edge)].value() for edge in ("start", "end")))
            c0, c1 = normalize_range(*(self.bounds[("Col", edge)].value() for edge in ("start", "end")))
            self.view_range = r0, r1, c0, c1
            for label, lo, hi in (("Row", r0, r1), ("Col", c0, c1)):
                self.bounds[(label, "start")].setValue(lo)
                self.bounds[(label, "end")].setValue(hi)
            self.refresh_image()

        def full_range(self):
            for label in ("Row", "Col"):
                self.bounds[(label, "start")].setValue(0)
                self.bounds[(label, "end")].setValue(ROWS)
            self.apply_range()

        def mapped_range(self):
            for label in ("Row", "Col"):
                self.bounds[(label, "start")].setValue(0)
                self.bounds[(label, "end")].setValue(20)
            self.apply_range()
            self.tabs.setCurrentIndex(1)

        def export(self, path):
            path = Path(path).with_suffix(".png")
            plot = self.hand_plot if self.tabs.currentIndex() == 0 else self.plot
            pg.exporters.ImageExporter(plot.scene()).export(str(path))
            model.save_csv(path.with_suffix(".csv"))
            model.save_metadata(path.with_suffix(".json"))
            metadata_path = path.with_suffix(".json")
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["visualization"] = {
                "view": "hand" if self.tabs.currentIndex() == 0 else "matrix",
                "mode": self.value_mode.currentData(), "color_levels": self._levels,
                "display_deadband_adc": self.deadband.value(),
                "display_deadband_semantics": "Neutral color only; no data filtering or contact/pressure classification",
                "show_anchors": self.anchors_toggle.isChecked(),
                "layout_name": hand_layout.name, "layout_status": hand_layout.status,
                "layout_hand": hand_layout.hand, "layout_source": dict(hand_layout.source),
                "layout_note": hand_layout.note, "baseline": self.baseline.metadata(),
                "csv_values": "raw ADC history, never baseline-subtracted or remapped",
                "layout_points": [{"row": p.row, "col": p.col, "x": p.x, "y": p.y,
                                   "region": p.region, "verified_anchor": p.verified_anchor}
                                  for p in hand_layout.points],
                "baseline_adc_row_major": ([float(value) if np.isfinite(value) else None
                                             for value in self.baseline.values.ravel()]
                                            if self.baseline.values is not None else None),
            }
            metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

        def save_both(self):
            default = "heatmap_" + hand + "_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save PNG + CSV", default, "PNG Files (*.png)")
            if path:
                try:
                    self.export(path)
                except Exception as error:
                    QtWidgets.QMessageBox.warning(self, "Export failed", str(error))

    return HeatmapViewer()


def main(args=None):
    try:
        import pyqtgraph  # noqa: F401
        from PyQt5 import QtWidgets
    except ImportError as error:
        raise SystemExit("Viewer needs python3-pyqt5 and python3-pyqtgraph (capture works without GUI).") from error
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from .lifecycle import shutdown_ros

    # Qt owns the event loop: Python's default SIGINT handler would raise from
    # a Qt callback, where Qt prints a traceback instead of exiting cleanly.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_signals = {}
    try:
        app = QtWidgets.QApplication(remove_ros_args(args=sys.argv if args is None else args))
        previous_signals = {number: signal.signal(number, lambda *_: app.quit())
                            for number in (signal.SIGINT, signal.SIGTERM)}
        model = ViewerModel()
        node = create_viewer_node(model)
        from .hand_layout import load_layout
        layout_path = node.declare_parameter("hand_layout_path", "").value
        baseline_path = node.declare_parameter("baseline_path", "").value
        initial_view = node.declare_parameter("initial_view", "hand").value
        hand_layout = load_layout(layout_path, expected_hand=model.expected_hand) if layout_path else None
        window = create_window(model, node, hand_layout=hand_layout,
                               baseline_path=baseline_path or None, initial_view=initial_view)
        window.show()
        app.exec()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            shutdown_ros(node)
        finally:
            for number, handler in previous_signals.items():
                signal.signal(number, handler)
