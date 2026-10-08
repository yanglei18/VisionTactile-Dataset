"""Two independent sensor-orientation previews, not an anatomical hand model.

Launch with ``python -m vt_glove_pose.thumb_preview_gui PATH.json``. This
module never opens a serial port and never modifies the recorded experiment.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from copy import deepcopy
import json
import math
from pathlib import Path
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


COLORS = ("#2379bc", "#119c83")
ANGLE_FIELDS = ("rotation_prox_deg", "rotation_dist_deg", "gravity_prox_deg", "gravity_dist_deg")


def _quat(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4,) or not np.isfinite(value).all() or np.linalg.norm(value) < 1e-10:
        raise ValueError("orientation needs a finite nonzero quaternion [w,x,y,z]")
    return value/np.linalg.norm(value)


def _multiply(a, b):
    aw, av, bw, bv = a[0], a[1:], b[0], b[1:]
    return np.r_[aw*bw-np.dot(av, bv), aw*bv+bw*av+np.cross(av, bv)]


def _relative(reference, current):
    return _quat(_multiply(reference*np.array([1., -1., -1., -1.]), current))


def _matrix(q):
    w, x, y, z = _quat(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _angle(q):
    return math.degrees(2*math.acos(float(np.clip(abs(_quat(q)[0]), 0., 1.))))


def validate_preview(value):
    """Validate the display contract; not a calibration-quality acceptance."""
    if (not isinstance(value, dict) or value.get("kind") != "vt_glove_pose.thumb_orientation_preview"
            or value.get("schema_version") != 1 or value.get("recorded_only") is not True):
        raise ValueError("expected a recorded thumb_orientation_preview schema_version=1")
    runs = value.get("runs")
    if not isinstance(runs, list) or not runs or len(runs) > 64:
        raise ValueError("preview needs 1..64 recorded runs")
    for run in runs:
        if not isinstance(run, dict) or not isinstance(run.get("name"), str):
            raise ValueError("each run needs a name")
        for field in ("samples", "legacy_samples"):
            rows = run.get(field, [])
            if not isinstance(rows, list) or len(rows) > 100000 or (field == "samples" and not rows):
                raise ValueError("each run needs a bounded, nonempty samples array")
            previous = -math.inf
            for row in rows:
                if not isinstance(row, dict):
                    raise ValueError("preview samples must be objects")
                for key in ("time_s", *ANGLE_FIELDS):
                    item = row.get(key)
                    if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
                        raise ValueError(f"preview {key} must be finite")
                if row["time_s"] < previous:
                    raise ValueError("preview times must not run backwards")
                previous = row["time_s"]
                _quat(row.get("quat_prox"))
                _quat(row.get("quat_dist"))
    return deepcopy(value)


def load_preview(path):
    path = Path(path)
    if not path.is_file() or path.stat().st_size > 64*1024*1024:
        raise ValueError("preview JSON must be a regular file no larger than 64 MiB")
    with path.open(encoding="utf-8") as source:
        return validate_preview(json.load(source))


class SensorOrientationCanvas(QtWidgets.QWidget):
    """Lightweight perspective projection of independently oriented boards."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(620, 320)
        self.quaternions = [None, None]
        self.empty_message = "等待有效传感器数据\n未连接或数据已过期，不显示假姿态"
        self.node_ids = [5, 6]
        self.yaw, self.pitch = -.55, .55
        self._drag = None
        self.setToolTip("按住鼠标左键拖动视角；视角操作不改变传感器数据")

    def set_orientations(self, proximal, distal):
        self.quaternions = [_quat(proximal), _quat(distal)]
        self.update()

    def set_sample(self, row, reference=None):
        """Shared replay/live API; reference is two wxyz display quaternions."""
        if row is None:
            self.quaternions = [None, None]
            self.update()
            return
        refs = reference if reference is not None else ([1., 0., 0., 0.], [1., 0., 0., 0.])
        if len(refs) != 2:
            raise ValueError("display reference needs two independent sensor quaternions")
        self.set_orientations(*[_relative(_quat(ref), _quat(row[field]))
                               for ref, field in zip(refs, ("quat_prox", "quat_dist"))])

    def set_empty_message(self, message):
        """Explain why no pose is drawn without inventing an orientation."""
        if not isinstance(message, str) or not message:
            raise ValueError("empty-state message must be a nonempty string")
        if message != self.empty_message:
            self.empty_message = message
            self.update()

    def set_node_ids(self, proximal, distal):
        if any(type(node) is not int or not 1 <= node <= 12 for node in (proximal, distal)) or proximal == distal:
            raise ValueError("two distinct node IDs in 1..12 required")
        self.node_ids = [proximal, distal]
        self.update()

    def reset_view(self):
        self.yaw, self.pitch = -.55, .55
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._drag = event.pos()

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta = event.pos()-self._drag
            self.yaw += delta.x()*.008
            self.pitch = float(np.clip(self.pitch+delta.y()*.008, -1.3, 1.3))
            self._drag = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        self._drag = None

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), self.palette().base())
        cy, sy, cp, sp = math.cos(self.yaw), math.sin(self.yaw), math.cos(self.pitch), math.sin(self.pitch)
        camera = np.array([[cy, -sy, 0.], [sp*sy, sp*cy, -cp], [cp*sy, cp*cy, sp]])
        scale = min(self.width()*.19, self.height()*.38)
        for index, q in enumerate(self.quaternions):
            if q is None:
                p.setPen(QtGui.QColor(COLORS[index]))
                p.drawText(QtCore.QRectF(index*self.width()/2, 12, self.width()/2, 30), QtCore.Qt.AlignCenter,
                           f"{'近端' if index == 0 else '远端'} IMU · 节点 {self.node_ids[index]}")
                p.setPen(self.palette().color(QtGui.QPalette.Text))
                p.drawText(QtCore.QRectF(index*self.width()/2, 70, self.width()/2, self.height()-110),
                           QtCore.Qt.AlignCenter, self.empty_message)
                continue
            center = QtCore.QPointF(self.width()*(.25+.5*index), self.height()*.55)
            matrix = _matrix(q)
            def project(v):
                v = camera @ np.asarray(v)
                factor = 5./(5.+v[2])
                return QtCore.QPointF(center.x()+scale*v[0]*factor, center.y()+scale*v[1]*factor)
            p.setPen(QtGui.QPen(self.palette().color(QtGui.QPalette.Midlight), 1))
            for line in np.linspace(-1.2, 1.2, 7):
                p.drawLine(project((line, -1.2, -.3)), project((line, 1.2, -.3)))
                p.drawLine(project((-1.2, line, -.3)), project((1.2, line, -.3)))
            color = QtGui.QColor(COLORS[index])
            vertices = np.array([[-.80, -.42, -.06], [.80, -.42, -.06], [.80, .42, -.06], [-.80, .42, -.06],
                                 [-.80, -.42, .06], [.80, -.42, .06], [.80, .42, .06], [-.80, .42, .06]]) @ matrix.T
            faces = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
            faces.sort(key=lambda face: float((vertices[list(face)] @ camera.T)[:, 2].mean()), reverse=True)
            for face in faces:
                p.setBrush(color.lighter(145) if face == (4, 7, 6, 5) else color.darker(110))
                p.setPen(QtGui.QPen(color.darker(140), 1.4))
                p.drawPolygon(QtGui.QPolygonF([project(vertices[i]) for i in face]))
            # Asymmetry makes the board's positive x edge and upper side visible.
            p.setBrush(QtGui.QColor("#f3c759"))
            p.setPen(QtCore.Qt.NoPen)
            chip = np.array([[.25, -.17, .085], [.58, -.17, .085], [.58, .17, .085], [.25, .17, .085]]) @ matrix.T
            p.drawPolygon(QtGui.QPolygonF([project(v) for v in chip]))
            origin = project([0., 0., .1])
            for axis, axis_color, label in zip(np.eye(3), ("#e15c64", "#3da465", "#507bda"), ("X", "Y", "Z")):
                end = project(matrix @ (axis*1.05))
                p.setPen(QtGui.QPen(QtGui.QColor(axis_color), 3, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                p.drawLine(origin, end)
                p.drawText(end+QtCore.QPointF(5., -4.), label)
            p.setPen(color)
            font = p.font(); font.setPointSize(13); font.setBold(True); p.setFont(font)
            p.drawText(QtCore.QRectF(index*self.width()/2, 12, self.width()/2, 30), QtCore.Qt.AlignCenter,
                       f"{'近端' if index == 0 else '远端'} IMU · 节点 {self.node_ids[index]}")
            font.setPointSize(10); font.setBold(False); p.setFont(font)
            p.setPen(self.palette().color(QtGui.QPalette.Text))
            p.drawText(QtCore.QRectF(index*self.width()/2, 43, self.width()/2, 24), QtCore.Qt.AlignCenter,
                       f"相对各自显示参考的旋转幅值 {_angle(q):.1f}°")
        p.setPen(self.palette().color(QtGui.QPalette.Text))
        p.drawText(QtCore.QRectF(10, self.height()-29, self.width()-20, 22), QtCore.Qt.AlignCenter,
                   "左右仅为画面布局，不代表传感器实际位置或骨骼连接 · 拖动调整视角")


class AnglePlot(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(190)
        self.rows, self.legacy = [], []
        self.current_time = 0.
        self.show_legacy = True

    def set_run(self, rows, legacy):
        self.rows, self.legacy = rows, legacy
        self.update()

    def set_series(self, samples, legacy_samples=None):
        self.set_run(samples, legacy_samples if legacy_samples is not None else [])

    def set_time(self, time_s):
        self.current_time = float(time_s)
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), self.palette().base())
        area = QtCore.QRectF(48., 33., max(1., self.width()-65.), max(1., self.height()-62.))
        p.setPen(self.palette().color(QtGui.QPalette.Text))
        p.drawText(12, 20, "旋转幅值 / ° · 相对本段首帧各自参考；显示归零不改变曲线")
        if not self.rows:
            return
        start, end = self.rows[0]["time_s"], self.rows[-1]["time_s"]
        span = max(end-start, 1e-9)
        def point(t, angle):
            return QtCore.QPointF(area.left()+(t-start)/span*area.width(), area.bottom()-angle/180.*area.height())
        for value in (0, 45, 90, 135, 180):
            y = point(start, value).y()
            p.setPen(self.palette().color(QtGui.QPalette.Midlight))
            p.drawLine(QtCore.QPointF(area.left(), y), QtCore.QPointF(area.right(), y))
            p.setPen(self.palette().color(QtGui.QPalette.Text))
            p.drawText(8, int(y)+4, str(value))
        p.drawText(int(area.left()), self.height()-8, f"{start:.1f}s")
        p.drawText(int(area.right())-45, self.height()-8, f"{end:.1f}s")
        p.save(); p.setClipRect(area.adjusted(-1, -1, 1, 1))
        for rows, legacy in ((self.legacy, True), (self.rows, False)):
            if legacy and not self.show_legacy:
                continue
            for field, color in zip(("rotation_prox_deg", "rotation_dist_deg"), COLORS):
                pen = QtGui.QPen(QtGui.QColor(color), 1.4 if legacy else 2.2,
                                 QtCore.Qt.DashLine if legacy else QtCore.Qt.SolidLine)
                if legacy:
                    c = pen.color(); c.setAlpha(130); pen.setColor(c)
                p.setPen(pen)
                path = QtGui.QPainterPath()
                # A display-only subsample bounds rendering cost; JSON and seek
                # keep every sample and the final point is always retained.
                stride = max(1, len(rows)//max(1, self.width()*2))
                indices = list(range(0, len(rows), stride))
                if rows and (not indices or indices[-1] != len(rows)-1):
                    indices.append(len(rows)-1)
                for i, index in enumerate(indices):
                    sample = rows[index]
                    pt = point(sample["time_s"], sample[field])
                    path.moveTo(pt) if i == 0 else path.lineTo(pt)
                p.drawPath(path)
        p.setPen(QtGui.QPen(self.palette().color(QtGui.QPalette.Text), 1.3))
        p.drawLine(point(self.current_time, 0), point(self.current_time, 180))
        p.restore()


class ThumbPreviewWindow(QtWidgets.QWidget):
    def __init__(self, data, parent=None):
        super().__init__(parent)
        self.data = validate_preview(data)
        self.run_index, self.sample_index = 0, 0
        self.references = [np.array([1., 0., 0., 0.]) for _ in range(2)]
        self.playing = False
        self._last_tick = time.monotonic()
        self._position = 0.
        self.setWindowTitle("拇指双 IMU · 独立朝向与单位对比回放")
        self.resize(1100, 850)
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("拇指双传感器朝向预览")
        font = title.font(); font.setPointSize(17); font.setBold(True); title.setFont(font)
        layout.addWidget(title)
        self.warning = QtWidgets.QLabel("录制数据回放 · 不是解剖关节角，未保证无漂移；两块板分别相对自己的参考，不是两传感器之间的相对姿态。")
        self.warning.setWordWrap(True)
        layout.addWidget(self.warning)
        self.provenance = QtWidgets.QLabel("来源："+str(data.get("source_directory", "未注明")))
        self.provenance.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.provenance.setWordWrap(True)
        layout.addWidget(self.provenance)
        line = QtWidgets.QHBoxLayout()
        line.addWidget(QtWidgets.QLabel("录制段"))
        self.run_selector = QtWidgets.QComboBox()
        self.run_selector.addItems([run["name"] for run in self.data["runs"]])
        line.addWidget(self.run_selector, 1)
        self.zero_button = QtWidgets.QPushButton("以当前帧作为显示参考")
        self.restore_button = QtWidgets.QPushButton("恢复录制参考")
        self.view_button = QtWidgets.QPushButton("恢复视角")
        for button in (self.zero_button, self.restore_button, self.view_button):
            line.addWidget(button)
        layout.addLayout(line)
        self.canvas = SensorOrientationCanvas()
        layout.addWidget(self.canvas, 2)
        self.values = QtWidgets.QLabel()
        self.values.setWordWrap(True)
        layout.addWidget(self.values)
        legend = QtWidgets.QHBoxLayout()
        legend.addWidget(QtWidgets.QLabel("<span style='color:#2379bc'>━ 近端 5</span>　<span style='color:#119c83'>━ 远端 6</span>　实线：当前配置；虚线：旧单位配置"))
        legend.addStretch()
        self.legacy_checkbox = QtWidgets.QCheckBox("显示旧单位对比")
        self.legacy_checkbox.setChecked(True)
        legend.addWidget(self.legacy_checkbox)
        layout.addLayout(legend)
        self.plot = AnglePlot()
        layout.addWidget(self.plot, 1)
        controls = QtWidgets.QHBoxLayout()
        self.play_button = QtWidgets.QPushButton("播放")
        controls.addWidget(self.play_button)
        self.seek = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        controls.addWidget(self.seek, 1)
        self.clock_label = QtWidgets.QLabel()
        self.clock_label.setMinimumWidth(140)
        controls.addWidget(self.clock_label)
        self.speed = QtWidgets.QComboBox()
        for speed in (.25, .5, 1., 2.):
            self.speed.addItem(f"{speed:g}×", speed)
        self.speed.setCurrentIndex(2)
        controls.addWidget(self.speed)
        layout.addLayout(controls)
        self.reference_label = QtWidgets.QLabel("显示参考：本段首帧，各自独立显示参考。此窗口不进行转轴拟合或关节标定。")
        self.reference_label.setWordWrap(True)
        layout.addWidget(self.reference_label)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self._tick)
        self.run_selector.currentIndexChanged.connect(self.set_run)
        self.seek.valueChanged.connect(self._seek)
        self.play_button.clicked.connect(self.toggle_play)
        self.zero_button.clicked.connect(self.zero_display)
        self.restore_button.clicked.connect(self.restore_reference)
        self.view_button.clicked.connect(self.canvas.reset_view)
        self.legacy_checkbox.toggled.connect(self._legacy)
        self.set_run(0)

    @property
    def run(self):
        return self.data["runs"][self.run_index]

    def set_run(self, index):
        self.pause()
        self.run_index = int(index)
        self.times = [row["time_s"] for row in self.run["samples"]]
        self.references = [np.array([1., 0., 0., 0.]) for _ in range(2)]
        self.reference_label.setText("显示参考：本段首帧，各自独立显示参考。此窗口不进行转轴拟合或关节标定。")
        self.seek.blockSignals(True)
        self.seek.setRange(0, len(self.times)-1)
        self.seek.setValue(0)
        self.seek.blockSignals(False)
        self.plot.set_run(self.run["samples"], self.run.get("legacy_samples", []))
        self.legacy_checkbox.setEnabled(bool(self.run.get("legacy_samples")))
        self._position = self.times[0]
        self._show_sample(0)

    def _show_sample(self, index):
        self.sample_index = int(index)
        row = self.run["samples"][self.sample_index]
        self.canvas.set_orientations(*[_relative(reference, _quat(row[field])) for reference, field in
                                       zip(self.references, ("quat_prox", "quat_dist"))])
        self.values.setText(f"录制参考旋转幅值：近端 {row['rotation_prox_deg']:.1f}° / 远端 {row['rotation_dist_deg']:.1f}°"
                            f"　　加速度方向变化（含运动加速度）：近端 {row['gravity_prox_deg']:.1f}° / 远端 {row['gravity_dist_deg']:.1f}°")
        self.clock_label.setText(f"{row['time_s']-self.times[0]:.2f} / {self.times[-1]-self.times[0]:.2f} s")
        self.plot.current_time = row["time_s"]
        self.plot.update()
        self.seek.blockSignals(True); self.seek.setValue(self.sample_index); self.seek.blockSignals(False)

    def _seek(self, index):
        self._position = self.times[index]
        self._last_tick = time.monotonic()
        self._show_sample(index)

    def _legacy(self, checked):
        self.plot.show_legacy = checked
        self.plot.update()

    def toggle_play(self):
        if self.playing:
            self.pause()
        else:
            if self.sample_index >= len(self.times)-1:
                self._seek(0)
            self.playing = True
            self._last_tick = time.monotonic()
            self.play_button.setText("暂停")
            self.timer.start()

    def pause(self):
        self.playing = False
        self.timer.stop()
        self.play_button.setText("播放")

    def _tick(self):
        now = time.monotonic()
        self.advance(max(0., now-self._last_tick))
        self._last_tick = now

    def advance(self, elapsed_s):
        """Advance playback using elapsed time, independent of render frequency."""
        if not self.playing:
            return
        self._position = min(self.times[-1], self._position+elapsed_s*self.speed.currentData())
        self._show_sample(max(0, bisect_right(self.times, self._position)-1))
        if self._position >= self.times[-1]:
            self.pause()

    def zero_display(self):
        row = self.run["samples"][self.sample_index]
        self.references = [_quat(row[field]) for field in ("quat_prox", "quat_dist")]
        self.reference_label.setText(f"显示参考：本段 {row['time_s']-self.times[0]:.2f} s 帧。只改变板块显示；原始数据、观察器和曲线均未改动。")
        self._show_sample(self.sample_index)

    def restore_reference(self):
        self.references = [np.array([1., 0., 0., 0.]) for _ in range(2)]
        self.reference_label.setText("显示参考：本段首帧，各自独立显示参考。此窗口不进行转轴拟合或关节标定。")
        self._show_sample(self.sample_index)

    def closeEvent(self, event):
        self.pause()
        super().closeEvent(event)


OrientationPairWidget = SensorOrientationCanvas
RotationPlotWidget = AnglePlot


def run_replay(path):
    data = load_preview(path)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = ThumbPreviewWindow(data)
    window.show()
    return app.exec_()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    try:
        return run_replay(args.path)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
