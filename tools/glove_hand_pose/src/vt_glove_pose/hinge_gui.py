"""Independent natural-motion joint experiment. Never replaces the hand GUI."""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import math
from pathlib import Path
import time
import uuid

from PyQt5 import QtCore, QtGui, QtWidgets

from .config import FINGERS, pair_gyro_scales
from .hinge_guide import guide_for
from .hinge_guide_widget import MotionGuideCanvas
from .hinge_worker import HingeWorker
from .stream import list_serial_ports


FINGER_NAMES = dict(thumb="拇指", index="食指", middle="中指", ring="无名指", little="小指")
STATE_NAMES = dict(disconnected="未连接", needs_neutral="需要参考姿势", prepare_neutral="准备静止参考",
    capturing_neutral="采集静止参考", neutral_ready="参考已采集，等待自然运动", prepare_motion="准备自然屈伸",
    capturing_motion="采集自然屈伸", fitting="后台拟合关节轴", candidate_ready="候选实时预览",
    prepare_direction="准备首次屈曲", capturing_direction="确认屈曲方向")


class LocalHingeCanvas(QtWidgets.QWidget):
    """Local two-link projection plus recent curve; no fake global hand pose."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 360)
        self.state = {}
        self.history = deque(maxlen=900)
        self._last_sample = None
        self._plot_direction = None
        self._last_angle = None
        self._last_signed = False

    def set_snapshot(self, state):
        self.state = state
        observer = state.get("observer") or {}
        key = (observer.get("samples"), state.get("state"))
        angle = state.get("anatomical_angle_deg")
        direction = (state.get("direction") or {}).get("multiplier") if angle is not None else None
        if direction != self._plot_direction:
            self.history.clear()  # Different coordinates must not create a fake jump.
            self._last_sample = None
            self._plot_direction = direction
        if angle is None:
            angle = state.get("math_angle_deg")
        if angle is not None:
            self._last_angle = float(angle)
            self._last_signed = state.get("anatomical_angle_deg") is not None
        if key != self._last_sample:
            self.history.append((time.monotonic(), float(angle) if angle is not None else 0.,
                                 bool(state.get("preview_valid")) and angle is not None))
            self._last_sample = key
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        palette = self.palette()
        painter.fillRect(self.rect(), palette.base())
        text_color = palette.color(QtGui.QPalette.Text)
        painter.setPen(text_color)
        painter.drawText(16, 25, "局部双骨段示意 · 近节固定显示，不代表整掌空间朝向")
        valid = bool(self.state.get("preview_valid"))
        angle = self.state.get("anatomical_angle_deg")
        signed = angle is not None
        if angle is None:
            angle = self.state.get("math_angle_deg")
        if angle is None and self._last_angle is not None:
            angle, signed = self._last_angle, self._last_signed
        if angle is None or not math.isfinite(angle):
            painter.drawText(20, 75, "尚无有效关节角；不播放模拟手势")
        else:
            width, height = self.width(), self.height()
            cx, cy, scale = width*.51, height*.33, min(width*.23, height*.26)
            theta = math.radians(angle)
            start = QtCore.QPointF(cx-scale, cy-scale*.18)
            joint = QtCore.QPointF(cx, cy)
            end = QtCore.QPointF(cx+scale*math.cos(theta), cy+scale*(.70*math.sin(theta)+.18*math.cos(theta)))
            observed = (self.state.get("observer") or {}).get("current_angle_observed", False)
            color = QtGui.QColor("#49aaaf" if observed else "#d49b45") if valid else QtGui.QColor("#a0a0a0")
            for a, b in ((start, joint), (joint, end)):
                painter.setPen(QtGui.QPen(color.darker(135), 30, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                painter.drawLine(a, b)
                painter.setPen(QtGui.QPen(color.lighter(155), 21, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                painter.drawLine(a-QtCore.QPointF(0, 2), b-QtCore.QPointF(0, 2))
            painter.setBrush(color.darker(140))
            painter.setPen(QtGui.QPen(palette.color(QtGui.QPalette.Base), 3))
            painter.drawEllipse(joint, 13, 13)
            painter.setPen(text_color)
            painter.drawText(20, 55, f"{'屈曲方向已声明' if signed else '数学角，尚未确认屈曲正方向'}：{angle:+.1f}°")
            if not valid:
                painter.drawText(20, 78, "预览无效／断流：灰色为最后状态，不是当前测量")
            elif not observed:
                painter.drawText(20, 78, "橙色：当前仅预测，缺少有效角度观测；可能漂移")
        # Fixed limits and unwrapped numeric readout avoid autoscale disguising drift.
        plot = QtCore.QRectF(48, self.height()*.62, self.width()-66, self.height()*.32)
        painter.setPen(palette.color(QtGui.QPalette.Mid))
        painter.drawRect(plot)
        for value in (-180, 0, 180):
            y = plot.center().y()-value/180.*plot.height()/2.
            painter.drawLine(QtCore.QPointF(plot.left(), y), QtCore.QPointF(plot.right(), y))
            painter.drawText(3, int(y)+4, f"{value}°")
        painter.setPen(text_color)
        painter.drawText(int(plot.left()), int(plot.top())-8, "最近 30 秒 · 图外角度保留于数值/原始回放，不作归零")
        now = time.monotonic()
        previous = None
        painter.save()
        painter.setClipRect(plot)
        for stamp, angle, good in self.history:
            if now-stamp > 30. or not good:
                previous = None
                continue
            point = QtCore.QPointF(plot.right()-(now-stamp)/30.*plot.width(),
                                  plot.center().y()-angle/180.*plot.height()/2.)
            if previous is not None:
                painter.setPen(QtGui.QPen(QtGui.QColor("#31989f"), 2))
                painter.drawLine(previous, point)
            previous = point
        painter.restore()


class HingeWindow(QtWidgets.QMainWindow):
    def __init__(self, config, *, finger="thumb", port=None, worker_factory=HingeWorker):
        super().__init__()
        if finger not in FINGERS:
            raise ValueError("unknown finger")
        pair = config["fingers"][finger]
        self.worker = worker_factory(config, pair["proximal"], pair["distal"])
        self._closing = False
        self._allow_unsaved_close = False
        self._finger_name = FINGER_NAMES[finger]
        self._stale_after_s = config["stale_after_s"]
        self._primary_command = None
        self.setWindowTitle(f"IMU 自然标定实验 — {FINGER_NAMES[finger]} {pair['proximal']}/{pair['distal']}")
        screen = QtWidgets.QApplication.primaryScreen()
        available = screen.availableGeometry() if screen else QtCore.QRect(0, 0, 1280, 960)
        self.resize(min(1080, available.width()-32), min(900, available.height()-60))
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        body = QtWidgets.QWidget()
        scroll.setWidget(body)
        self.setCentralWidget(scroll)
        layout = QtWidgets.QVBoxLayout(body)
        warning = QtWidgets.QLabel(f"本次只测 {self._finger_name} · 近端 {pair['proximal']} / 远端 {pair['distal']} · 实验向导，不覆盖旧标定")
        warning.setWordWrap(True)
        layout.addWidget(warning)
        scales = pair_gyro_scales(config, pair["proximal"], pair["distal"])
        unit_state = "配置标记已核实" if config["units_verified"] else "未核实固件量程"
        provenance = config.get("gyro_scale_provenance", {})
        if isinstance(provenance, dict) and provenance.get("kind") == "empirical_candidate_not_firmware_readback":
            unit_state = "实测推断候选；未核实固件量程"
        self.unit_label = QtWidgets.QLabel(
            f"陀螺比例：节点 {pair['proximal']} = {scales[0]:g}、"
            f"节点 {pair['distal']} = {scales[1]:g} °/s/LSB · {unit_state}")
        self.unit_label.setWordWrap(True)
        layout.addWidget(self.unit_label)
        row = QtWidgets.QHBoxLayout()
        self.port = QtWidgets.QComboBox()
        self.port.setEditable(True)
        if port:
            self.port.addItem(port)
        self.scan_button = QtWidgets.QPushButton("列出串口")
        self.connect_button = QtWidgets.QPushButton("连接")
        self.disconnect_button = QtWidgets.QPushButton("断开")
        row.addWidget(self.port, 1)
        for button in (self.scan_button, self.connect_button, self.disconnect_button):
            row.addWidget(button)
        layout.addLayout(row)
        steps = QtWidgets.QHBoxLayout()
        self.step_labels = []
        for index, title in enumerate(("连接", "参考姿势", "自然屈伸", "屈曲方向", "试跑", "导出")):
            label = QtWidgets.QLabel(f"{index+1}  {title}")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumHeight(30)
            steps.addWidget(label, 1)
            self.step_labels.append(label)
        layout.addLayout(steps)
        guide_row = QtWidgets.QHBoxLayout()
        guide_text = QtWidgets.QVBoxLayout()
        self.guide_title = QtWidgets.QLabel("先连接手套")
        self.guide_title.setWordWrap(True)
        self.guide_title.setStyleSheet("font-size: 23px; font-weight: 600;")
        guide_text.addWidget(self.guide_title)
        self.instructions = QtWidgets.QLabel()
        self.instructions.setWordWrap(True)
        self.instructions.setStyleSheet("font-size: 19px;")
        guide_text.addWidget(self.instructions)
        self.guide_detail = QtWidgets.QLabel()
        self.guide_detail.setWordWrap(True)
        guide_text.addWidget(self.guide_detail)
        self.guide_countdown = QtWidgets.QLabel()
        self.guide_countdown.setStyleSheet("font-size: 30px; font-weight: 600;")
        guide_text.addWidget(self.guide_countdown)
        self.guide_progress = QtWidgets.QProgressBar()
        self.guide_progress.setRange(0, 1000)
        self.guide_progress.setTextVisible(False)
        guide_text.addWidget(self.guide_progress)
        guide_text.addStretch(1)
        guide_row.addLayout(guide_text, 3)
        self.motion_guide = MotionGuideCanvas()
        # The teaching drawing currently depicts a thumb; do not mislabel it
        # as another finger. Text guidance still follows the selected pair.
        self.motion_guide.setVisible(finger == "thumb")
        guide_row.addWidget(self.motion_guide, 2)
        layout.addLayout(guide_row)
        primary_row = QtWidgets.QHBoxLayout()
        self.primary_button = QtWidgets.QPushButton("连接所选串口")
        self.primary_button.setMinimumHeight(52)
        self.primary_button.setStyleSheet("font-size: 19px; font-weight: 600;")
        self.primary_button.clicked.connect(self.advance_guide)
        primary_row.addWidget(self.primary_button, 3)
        self.guide_cancel_button = QtWidgets.QPushButton("取消本步，保留数据")
        self.guide_cancel_button.clicked.connect(lambda: self.send("cancel"))
        primary_row.addWidget(self.guide_cancel_button)
        self.guide_export_button = QtWidgets.QPushButton("保存已有数据…")
        self.guide_export_button.clicked.connect(self.export)
        primary_row.addWidget(self.guide_export_button)
        layout.addLayout(primary_row)
        self.message = QtWidgets.QLabel()
        self.message.setWordWrap(True)
        self.message.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.message)
        self.canvas = LocalHingeCanvas()
        self.canvas.setMinimumSize(480, 300)
        layout.addWidget(self.canvas, 1)
        self.advanced_toggle = QtWidgets.QCheckBox("展开高级操作、完整验证与诊断信息")
        layout.addWidget(self.advanced_toggle)
        self.advanced_panel = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(self.advanced_panel)
        self.advanced_panel.setVisible(False)
        self.advanced_toggle.toggled.connect(self.advanced_panel.setVisible)
        layout.addWidget(self.advanced_panel)
        actions = QtWidgets.QGridLayout()
        self.buttons = {}
        for index, (command, label) in enumerate((
            ("neutral", "① 参考姿势：准备 3 秒＋静止 5 秒"),
            ("motion", "② 自然屈伸：准备 3 秒＋运动 20 秒"),
            ("direction", "③ 先向掌心弯曲，确认正方向"),
            ("validation", "④ 独立验证录制 30 秒"),
            ("cancel", "取消当前动作（保留数据）"),
            ("export", "导出整个实验…"))):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, cmd=command: self.export() if cmd == "export" else self.send(cmd))
            self.buttons[command] = button
            actions.addWidget(button, index//2, index % 2)
        advanced_layout.addLayout(actions)
        protocol_row = QtWidgets.QHBoxLayout()
        protocol_row.addWidget(QtWidgets.QLabel("验证录制时长"))
        self.validation_duration = QtWidgets.QComboBox()
        for seconds in (30, 90, 180):
            self.validation_duration.addItem(f"{seconds} 秒", seconds)
        self.validation_duration.setCurrentIndex(2)
        protocol_row.addWidget(self.validation_duration)
        protocol_row.addWidget(QtWidgets.QLabel("180 秒可安排回位＋60 秒保持；标记只记录动作，不改变角度"), 1)
        advanced_layout.addLayout(protocol_row)
        markers = QtWidgets.QHBoxLayout()
        self.marker_buttons = {}
        for kind, label in (("reference", "参考手型 · 2 秒"), ("return_pose", "已回同一手型 · 2 秒"),
                            ("hold", "固定保持 · 60 秒"), ("common_rotation", "固定指形转掌 · 10 秒")):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, value=kind: self.send("marker", kind=value))
            self.marker_buttons[kind] = button
            markers.addWidget(button)
        self.cancel_marker_button = QtWidgets.QPushButton("取消本次标记")
        self.cancel_marker_button.clicked.connect(lambda: self.send("cancel_marker"))
        markers.addWidget(self.cancel_marker_button)
        advanced_layout.addLayout(markers)
        self.marker_status = QtWidgets.QLabel("先录制验证，再标记；点击后准备 1 秒，按对应时长保持所声明的动作。")
        self.marker_status.setWordWrap(True)
        advanced_layout.addWidget(self.marker_status)
        self.status = QtWidgets.QLabel("正在启动工作线程；不会自动连接")
        self.status.setWordWrap(True)
        advanced_layout.addWidget(self.status)
        self.metrics = QtWidgets.QLabel()
        self.metrics.setWordWrap(True)
        advanced_layout.addWidget(self.metrics)
        self.scan_button.clicked.connect(self.scan_ports)
        self.connect_button.clicked.connect(lambda: self.send("connect", port=self.port.currentText()))
        self.disconnect_button.clicked.connect(lambda: self.send("disconnect"))
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(33)
        self.worker.start()

    def advance_guide(self):
        command = self._primary_command
        self.primary_button.setEnabled(False)
        if command == "export":
            self.export()
        elif command == "connect":
            self.send("connect", port=self.port.currentText())
        elif command == "validation":
            self.send("validation", duration_s=30., guided_quickcheck=True)
        elif command is not None:
            self.send(command)

    def send(self, command, **args):
        if command == "validation" and "duration_s" not in args:
            args["duration_s"] = self.validation_duration.currentData()
        if not self.worker.submit(command, **args):
            self.message.setText("命令未提交：工作线程已停止或队列已满")

    def scan_ports(self):
        current = self.port.currentText()
        self.port.clear()
        for device, description in list_serial_ports():
            self.port.addItem(device)
            self.port.setItemData(self.port.count()-1, description, QtCore.Qt.ToolTipRole)
        # Enumeration must not silently select a different device.
        self.port.setEditText(current)

    def export(self):
        parent = QtWidgets.QFileDialog.getExistingDirectory(self, "选择实验导出父目录")
        if parent:
            name = "hinge_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")+"_"+uuid.uuid4().hex[:8]
            self.send("export", directory=str(Path(parent)/name))

    def refresh(self):
        state = self.worker.latest_snapshot()
        if not state:
            return
        runtime = state.get("runtime", {})
        connected = runtime.get("connected", False)
        name = state.get("state", "disconnected")
        busy = name.startswith(("prepare_", "capturing_")) or name == "fitting"
        validating = runtime.get("validation_remaining_s") is not None
        closing = self._closing
        self.connect_button.setEnabled(not connected and not closing)
        self.disconnect_button.setEnabled(connected and not closing)
        self.port.setEnabled(not connected and not closing)
        self.scan_button.setEnabled(not connected and not closing)
        self.validation_duration.setEnabled(not validating and not closing)
        active = next((marker for marker in runtime.get("validation_markers", []) if marker.get("status") == "recording"), None)
        quickcheck = runtime.get("validation_guided_quickcheck", False)
        for button in self.marker_buttons.values():
            button.setEnabled(validating and not quickcheck and active is None and not closing)
        self.cancel_marker_button.setEnabled(active is not None and not quickcheck and not closing)
        if active:
            now = runtime.get("validation_now_s", 0.)
            preparing = now < active["start_host_monotonic_s"]
            end = active["start_host_monotonic_s"] if preparing else active["end_host_monotonic_s"]
            names = dict(reference="参考手型", return_pose="回位", **{"return": "回位"}, hold="固定保持", common_rotation="固定指形转掌")
            self.marker_status.setText(f"{names.get(active['kind'], active['kind'])}：{'准备' if preparing else '采集'}剩余 {max(0., end-now):.1f} 秒；不自动归零")
        else:
            completed = sum(marker.get("kind") == "return" and marker.get("status") == "completed"
                            for marker in runtime.get("validation_markers", []))
            self.marker_status.setText(f"本次已完成 {completed} 段回位标记（不是通过次数）。点击标记后准备 1 秒，再保持相应动作。")
        enabled = dict(neutral=connected and not busy and not validating and not runtime.get("background_job"),
            motion=name in {"neutral_ready", "candidate_ready"} and not validating and not runtime.get("background_job"),
            direction=name == "candidate_ready" and not validating and state.get("preview_valid", False),
            validation=(name == "candidate_ready" and state.get("preview_valid", False)) or validating,
            cancel=busy or validating,
            export=not busy and not validating and not runtime.get("background_job"))
        for command, button in self.buttons.items():
            button.setEnabled(bool(enabled[command]) and not closing)
        self.buttons["validation"].setText("停止验证录制（保存已收数据）" if validating else "④ 开始独立验证录制")
        remaining = state.get("countdown_s") or state.get("capture_remaining_s") or 0.
        suffix = f" · 剩余 {remaining:.1f} 秒" if remaining > 0 else ""
        self.status.setText(("正在关闭串口并等待后台任务完成…" if closing else STATE_NAMES.get(name, name)+suffix)
                            + "\n" + str(state.get("status_message", "")))
        guide = guide_for(state, finger_name=self._finger_name, stale_after_s=self._stale_after_s)
        self._primary_command = guide["primary_command"]
        self.guide_title.setText(guide["title"])
        self.instructions.setText(guide["instruction"])
        self.guide_detail.setText(guide["detail"])
        seconds = guide["seconds_left"]
        self.guide_countdown.setText(f"{math.ceil(seconds)} 秒" if seconds is not None else "")
        self.guide_progress.setRange(0, 0 if guide["phase"] == "processing" else 1000)
        self.guide_progress.setValue(round(1000*guide["progress"]))
        self.guide_progress.setVisible(guide["phase"] in {"prepare", "capture", "processing"})
        self.primary_button.setText(guide["primary_text"])
        self.primary_button.setEnabled(guide["primary_enabled"] and not closing)
        self.guide_cancel_button.setEnabled((busy or validating) and not closing)
        self.guide_export_button.setEnabled(enabled["export"] and not closing)
        for index, label in enumerate(self.step_labels):
            label.setStyleSheet("font-weight: 600; border-bottom: 3px solid palette(highlight);"
                                if index == guide["step"] else "")
        self.motion_guide.set_instruction(guide["illustration"], guide["phase"])
        observer = state.get("observer") or {}
        age, rate = state.get("pair_age_s"), runtime.get("received_rate_hz")
        fmt = lambda x, digits=2: "—" if x is None else f"{x:.{digits}f}"
        self.metrics.setText(
            f"所选组原始样本 {runtime.get('selected_group_raw_samples', 0)} | 配对接收年龄 {fmt(age)} s | "
            f"近节主机接收率 {fmt(rate, 1)} Hz（非硬件同步证明）\n"
            f"当前角可观测：{observer.get('current_angle_observed', False)} | 历史缺失：{observer.get('history_loss', False)} | "
            f"残余偏置 {fmt(observer.get('relative_bias_dps'))} °/s | 质量：{observer.get('quality', '未初始化')}"
            + (f" | 验证剩余 {runtime['validation_remaining_s']:.1f} 秒\n"
               f"已记录实际在线输出 {runtime.get('validation_online_rows', 0)} 对 | "
               f"输出丢弃 {runtime.get('validation_online_discarded_rows', 0)} 对"
               if validating else ""))
        # Do not repeat a stale "please capture neutral" connection notice
        # underneath a successful-reference guide telling the user to move on.
        save_message = ("正在保存实验，请保持窗口开启" if runtime.get("background_job") == "export" else
                        f"最近保存：{runtime['last_export']}" if runtime.get("last_export") else "")
        self.message.setText("\n".join(filter(None, (runtime.get("error"), save_message,
            "当前有未保存记录；动作结束后可保存，失败数据也会保留。" if runtime.get("unsaved_experiment") else ""))))
        self.canvas.set_snapshot(state)

    def closeEvent(self, event):
        state = self.worker.latest_snapshot()
        if not self._closing and not self._allow_unsaved_close and state.get("runtime", {}).get("unsaved_experiment"):
            answer = QtWidgets.QMessageBox.question(self, "尚有未导出的实验数据",
                "关闭窗口将丢弃内存中的实验。仍要关闭吗？\n选择“否”后可点击“导出整个实验”。",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        self._closing = True
        self.worker.stop()
        if self.worker.isRunning():
            event.ignore()
            QtCore.QTimer.singleShot(100, self.close)
            return
        self.timer.stop()
        event.accept()


def launch_hinge_gui(config, *, finger="thumb", port=None, smoke_seconds=None):
    if smoke_seconds is not None and not 0 < smoke_seconds <= 3600:
        raise ValueError("smoke_seconds must be in (0, 3600]")
    existing = QtWidgets.QApplication.instance()
    app = existing or QtWidgets.QApplication(["vt-glove-pose-hinge"])
    window = HingeWindow(config, finger=finger, port=port)
    window.setAttribute(QtCore.Qt.WA_DeleteOnClose)
    window.show()
    if smoke_seconds is not None:
        QtCore.QTimer.singleShot(max(1, round(smoke_seconds*1000)), window.close)
    if existing is None:
        return app.exec_()
    loop = QtCore.QEventLoop()
    window.destroyed.connect(loop.quit)
    loop.exec_()
    return 0
