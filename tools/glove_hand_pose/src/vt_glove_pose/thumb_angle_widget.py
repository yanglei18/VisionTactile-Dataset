"""Reusable conditional angle display; not a certified anatomical hand model."""
from __future__ import annotations

from copy import deepcopy
import math
from numbers import Real

from PyQt5 import QtCore, QtGui, QtWidgets


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, Real) and math.isfinite(value)


def angle_display_state(snapshot):
    snapshot = snapshot or {}
    angle = snapshot.get("angle_deg")
    if not snapshot:
        mode, label, color = "waiting", "等待候选角度数据", "#7c858b"
    elif not snapshot.get("paired_fresh", False):
        mode, label, color = "stale", "数据过期 / 未连接：不显示当前角度", "#ae5158"
    elif not _finite(angle):
        mode, label, color = "unavailable", "当前没有可用角度", "#8c7373"
        causes = snapshot.get("reasons") or []
        if "axis_gravity_mismatch" in causes:
            label = "数据正常；保存的运动轴与当前姿态不一致（重新连接或显示归零不能修复）"
        elif "low_gravity_projection" in causes:
            label = "数据正常；当前掌面朝向下，重力暂时无法约束这个弯曲角"
        elif "acc_norm" in causes:
            label = "数据正常；当前加速度不能作为稳定的重力参考"
    elif snapshot.get("current_angle_observed", False):
        mode, label, color = "observed", "当前有观测约束（不等于精度已验证）", "#197e8c"
    else:
        mode, label, color = "predicted", "当前仅预测：可能累计漂移", "#b57916"
    current = float(angle) if mode in {"observed", "predicted"} else None
    if snapshot.get("history_loss", False):
        label += " · 历史连续性已丢失"
    return dict(mode=mode, label=label, color=color, angle_deg=current)


def angle_segments(history, max_gap_s=.1):
    """Break missing/time-gap spans without breaking all latched-loss history."""
    previous = None
    segments = []
    for row in history:
        if (not isinstance(row, dict) or not _finite(row.get("time_s"))
                or not _finite(row.get("angle_deg")) or row.get("paired_fresh") is False):
            previous = None
            continue
        if previous is not None:
            dt = row["time_s"]-previous["time_s"]
            if 0. <= dt <= max_gap_s+1e-9:
                segments.append((previous, row, bool(row.get("current_angle_observed", False))))
        previous = row
    return segments


class _AngleCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 330)
        self.state, self.history = {}, []

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), self.palette().base())
        state = angle_display_state(self.state)
        color = QtGui.QColor(state["color"])
        p.setPen(self.palette().color(QtGui.QPalette.Text))
        p.drawText(15, 24, "两段局部示意 · 候选坐标正负，不表示真实骨骼形状或手掌空间朝向")
        angle = state["angle_deg"]
        joint = QtCore.QPointF(self.width()*.50, self.height()*.28)
        length = min(self.width()*.20, self.height()*.20)
        if angle is None:
            p.setPen(QtGui.QColor(state["color"]))
            p.drawText(QtCore.QRectF(15, 50, self.width()-30, self.height()*.34), QtCore.Qt.AlignCenter,
                       state["label"]+"\n旧轨迹仅保留在下方，不作为当前姿态")
        else:
            start = joint-QtCore.QPointF(length, 0.)
            theta = math.radians(angle)
            end = joint+QtCore.QPointF(length*math.cos(theta), -length*math.sin(theta))
            for a, b in ((start, joint), (joint, end)):
                p.setPen(QtGui.QPen(color.darker(125), 23, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                p.drawLine(a, b)
                p.setPen(QtGui.QPen(color.lighter(155), 15, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                p.drawLine(a-QtCore.QPointF(0., 2.), b-QtCore.QPointF(0., 2.))
            p.setBrush(QtGui.QColor("#f1ca60"))
            p.setPen(QtGui.QPen(color.darker(140), 3))
            p.drawEllipse(joint, 13., 13.)
            p.setPen(self.palette().color(QtGui.QPalette.Text))
            p.drawText(joint+QtCore.QPointF(-14., 32.), "关节示意")
        area = QtCore.QRectF(58., self.height()*.58, max(1., self.width()-78.), max(1., self.height()*.34))
        p.setPen(self.palette().color(QtGui.QPalette.Text))
        p.drawText(15, int(area.top())-14, "候选有符号角度 / ° · 实线有观测；虚线仅预测；缺失不连线")
        samples = [row for row in self.history if isinstance(row, dict) and _finite(row.get("time_s"))]
        if not samples:
            p.drawText(area, QtCore.Qt.AlignCenter, "尚无逐配对角度记录")
            return
        times = [float(row["time_s"]) for row in samples]
        start_time, end_time = min(times), max(times)
        span = max(.01, end_time-start_time)
        values = [abs(float(row["angle_deg"])) for row in samples if _finite(row.get("angle_deg"))]
        limit = max(90., math.ceil(max(values, default=0.)/45.)*45.)
        def point(row):
            return QtCore.QPointF(area.left()+(row["time_s"]-start_time)/span*area.width(),
                                 area.center().y()-row["angle_deg"]/limit*area.height()/2.)
        for value in (-limit, 0., limit):
            y = area.center().y()-value/limit*area.height()/2.
            p.setPen(self.palette().color(QtGui.QPalette.Midlight))
            p.drawLine(QtCore.QPointF(area.left(), y), QtCore.QPointF(area.right(), y))
            p.setPen(self.palette().color(QtGui.QPalette.Text))
            p.drawText(5, int(y)+4, f"{value:g}")
        paths = {True: QtGui.QPainterPath(), False: QtGui.QPainterPath()}
        last_row = {True: None, False: None}
        for previous, row, observed in angle_segments(self.history):
            path = paths[observed]
            if last_row[observed] is not previous:
                path.moveTo(point(previous))
            path.lineTo(point(row))
            last_row[observed] = row
        # Two batched paths rather than thousands of per-segment pen objects
        # and paint calls. Gaps and observation/prediction transitions stay split.
        for observed, path in paths.items():
            p.setPen(QtGui.QPen(QtGui.QColor("#197e8c" if observed else "#b57916"), 1.7,
                               QtCore.Qt.SolidLine if observed else QtCore.Qt.DashLine))
            p.drawPath(path)
        p.setPen(self.palette().color(QtGui.QPalette.Text))
        p.drawText(int(area.left()), int(area.bottom())+18, f"{start_time:.1f}s")
        p.drawText(int(area.right())-54, int(area.bottom())+18, f"{end_time:.1f}s")


class ThumbAngleWidget(QtWidgets.QWidget):
    """set_state(snapshot, history) works for live or recorded paired outputs.

    There is deliberately no zero/calibrate button. This viewer does not alter
    the candidate processor, profile, or the separate orientation reference.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.title = QtWidgets.QLabel("拇指弯曲（实验候选）")
        font = self.title.font(); font.setPointSize(15); font.setBold(True); self.title.setFont(font)
        layout.addWidget(self.title)
        self.warning = QtWidgets.QLabel("条件性标量角预览 · 不是已认证的解剖关节角。正负方向按候选坐标，精度和长期无漂移尚未验证。\n"
                                       "上方“显示参考”只影响独立朝向，不会将这里的角度归零，也不会重置候选观察器。")
        self.warning.setWordWrap(True)
        layout.addWidget(self.warning)
        self.readout = QtWidgets.QLabel("—")
        font = self.readout.font(); font.setPointSize(26); font.setBold(True); self.readout.setFont(font)
        layout.addWidget(self.readout)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.canvas = _AngleCanvas()
        layout.addWidget(self.canvas, 1)
        self.details = QtWidgets.QLabel()
        self.details.setWordWrap(True)
        self.details.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.details)
        self.snapshot, self.history = {}, []
        self.set_state(None, [])

    def set_state(self, snapshot, history):
        self.snapshot = deepcopy(snapshot or {})
        # A recorded row additionally carries observer diagnostics, lists and
        # provenance. The renderer only needs these scalar columns, not another
        # deep clone of the complete experiment on every display frame.
        keys = ("time_s", "angle_deg", "current_angle_observed", "paired_fresh",
                "history_loss", "pair_count", "quality")
        self.history = [{key: row[key] for key in keys if key in row}
                        for row in (history or []) if isinstance(row, dict)]
        view = angle_display_state(self.snapshot)
        self.mode = view["mode"]
        nodes = self.snapshot.get("node_ids") or [5, 6]
        self.title.setText("拇指弯曲（实验候选） · 节点 "+" / ".join(map(str, nodes)))
        self.readout.setText("—" if view["angle_deg"] is None else f"{view['angle_deg']:+.1f}°")
        self.readout.setStyleSheet("color: "+view["color"]+";")
        self.status.setText(view["label"])
        reasons = self.snapshot.get("reasons") or []
        if isinstance(reasons, str):
            reasons = [reasons]
        self.details.setText("状态："+str(self.snapshot.get("quality", "waiting"))+
                             ("　原因："+"；".join(map(str, reasons)) if reasons else ""))
        self.canvas.state, self.canvas.history = self.snapshot, self.history
        self.canvas.update()
