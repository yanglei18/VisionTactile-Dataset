"""Manual-connect quick whole-hand preview: one reader, latest-state delivery."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import queue
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from .canvas import FINGER_LABELS, HandCanvas
from .config import write_json_exclusive
from .stream import SerialSource, list_serial_ports


class QuickHandWorker(QtCore.QThread):
    """All serial and estimation operations stay in this one worker thread."""
    def __init__(self, config, *, engine_factory=None, source_factory=None,
                 clock=time.monotonic, parent=None):
        super().__init__(parent)
        self.config = deepcopy(config)
        self.engine_factory = engine_factory
        self.source_factory = source_factory or SerialSource
        self.clock = clock
        self.engine = self.source = None
        self.port = self.error = ""
        self.notice = ""
        self._profile = None
        self.pending = None
        self.message = "等待手动连接；不会自动打开串口"
        self.received_samples = 0
        self._commands = queue.Queue(maxsize=16)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = dict(model={}, nodes={}, calibration={}, warnings=[], quick_mode=True,
                              runtime=dict(connected=False, message=self.message, error=""))

    def _new_engine(self):
        factory = self.engine_factory
        if factory is None:
            from .quick_hand import QuickHandEngine
            factory = QuickHandEngine
        engine = factory(deepcopy(self.config))
        if self._profile is not None:
            engine.load_profile(deepcopy(self._profile))
        return engine

    def _remember_profile(self):
        exporter = getattr(self.engine, "export_profile", None)
        if exporter is not None:
            try:
                self._profile = deepcopy(exporter())
            except ValueError:
                # Before the first reference the engine has no complete mounts.
                # Keep any earlier explicitly loaded profile; never invent one.
                pass

    def submit(self, name, **args):
        try:
            self._commands.put_nowait((name, args))
            return True
        except queue.Full:
            return False

    def stop(self):
        self._stop.set()

    def latest_snapshot(self):
        with self._lock:
            return deepcopy(self._snapshot)

    def disconnect(self):
        self.pending = None
        source, self.source = self.source, None
        try:
            if source is not None:
                source.close()
        finally:
            if self.engine is not None:
                self.engine.cancel_calibration()
                self._remember_profile()
            self.engine = self._new_engine()
            self.message = "已断开；再次连接后重新设置本次参考"

    def command(self, name, args):
        if self.engine is None:
            self.engine = self._new_engine()
        if name in {"save_profile", "load_profile", "flip_direction"}:
            if self.pending or self.engine.get_snapshot(self.clock()).get("calibration", {}).get("active"):
                raise ValueError("请等待当前参考 / 学习完成，或先取消")
            if name == "save_profile":
                path = str(args.get("path", "")).strip()
                if not path:
                    raise ValueError("请选择保存文件路径")
                profile = self.engine.export_profile()
                write_json_exclusive(path, profile)
                self._profile = deepcopy(profile)
                self.notice = f"学习参数已保存：{path}（不包含本次张手参考）"
            elif name == "load_profile":
                path = Path(str(args.get("path", "")))
                if not path.is_file():
                    raise ValueError("请选择学习参数 JSON 文件")
                with path.open("rb") as stream:
                    data = stream.read(1024 * 1024 + 1)
                if len(data) > 1024 * 1024:
                    raise ValueError("学习参数文件不能超过 1 MiB")
                profile = json.loads(data.decode("utf-8"))
                if not isinstance(profile, dict) or profile.get("kind") != "vt_glove_pose.quick_hand_profile":
                    raise ValueError("只接受快速模式学习参数，不导入旧正式标定文件")
                self.engine.load_profile(profile)
                self._profile = deepcopy(self.engine.export_profile())
                self.notice = f"学习参数已加载：{path}；连接后请重新张手保持 2 秒设参考"
            else:
                finger, segment = str(args.get("finger", "")), str(args.get("segment", "both"))
                self.engine.flip_direction(finger, segment=segment)
                self._remember_profile()
                label = FINGER_LABELS.get(finger, finger)
                part = {"proximal": "近节", "distal": "中末节", "both": "整指"}.get(segment, segment)
                self.notice = f"已反转{label}的{part}方向；本次参考保持不变"
            self.error = ""
            return
        if name == "connect":
            if self.source is not None:
                raise ValueError("已经连接；请先断开")
            port = str(args.get("port", "")).strip()
            if not port:
                raise ValueError("请选择或输入串口，再点击连接")
            self._remember_profile()
            source = self.source_factory(port)
            try:
                source.open()
                engine = self._new_engine()
            except Exception:
                source.close()
                raise
            self.source, self.engine, self.port = source, engine, port
            self.received_samples = 0
            self.error = ""
            self.message = "已连接：点击步骤 1 设置本次参考，即可查看五指"
            return
        if name == "disconnect":
            self.disconnect()
            self.error = ""
            return
        if self.source is None:
            raise ValueError("请先连接串口")
        if name in {"reference", "align"}:
            if self.pending or self.engine.get_snapshot(self.clock()).get("calibration", {}).get("active"):
                raise ValueError("已有准备或采集正在进行，请先完成或取消")
            self.pending = dict(kind="reference" if name == "reference" else "frame_alignment",
                                start_at_s=self.clock() + 3., duration_s=2. if name == "reference" else 10.)
            self.message = "准备 3 秒：五指保持伸直，绿色提示出现后开始采集"
        elif name == "learn":
            if self.pending:
                raise ValueError("已有准备正在进行，请先完成或取消")
            self.engine.begin_motion_learning(duration=6)
            self.message = "手掌尽量不动，先握拳再张开，重复 2–3 次"
        elif name == "cancel":
            self.pending = None
            self.engine.cancel_calibration()
            self.message = "已取消准备 / 采集；现有参考状态见下方提示"
        else:
            raise ValueError(f"未知操作：{name}")
        self.error = ""

    def feed_samples(self, samples):
        if self.engine is None:
            self.engine = self._new_engine()
        for sample in samples:
            self.engine.process(sample)
            self.received_samples += 1

    def publish(self, now=None):
        now = self.clock() if now is None else now
        if self.engine is None:
            self.engine = self._new_engine()
        # Called only by the serial worker. Preparation never starts an engine
        # capture; the live pose keeps updating until this one-shot deadline.
        if self.pending is not None and now >= self.pending["start_at_s"]:
            pending, self.pending = self.pending, None
            try:
                if self.source is None:
                    raise ValueError("串口已断开，本次准备已取消")
                if pending["kind"] == "reference":
                    self.engine.begin_reference(duration=pending["duration_s"])
                    self.message = "正在采集参考：掌心朝下，五指自然伸直，保持不动 2 秒"
                else:
                    self.engine.begin_frame_alignment(duration=pending["duration_s"])
                    self.message = "现在开始转动，正在采集！整个手掌向下倾斜约 30°再回水平，重复；不要握拳 / 上下平移"
                self.error = ""
            except Exception as exc:
                self.error = f"采集未开始：{exc}"
        snapshot = self.engine.get_snapshot(now)
        calibration = snapshot.setdefault("calibration", {})
        if self.pending is not None:
            calibration["active"] = dict(kind=self.pending["kind"], phase="preparing",
                                         remaining_s=max(0., self.pending["start_at_s"] - now),
                                         capture_duration_s=self.pending["duration_s"])
        elif calibration.get("active"):
            calibration["active"] = dict(calibration["active"], phase="collecting")
        snapshot["runtime"] = dict(connected=self.source is not None, port=self.port,
                                   error=self.error, message=self.message, notice=self.notice,
                                   received_samples=self.received_samples,
                                   published_monotonic_s=now)
        with self._lock:
            self._snapshot = snapshot

    def run(self):
        last_publish = -math.inf
        try:
            self.engine = self._new_engine()
            while not self._stop.is_set():
                for _ in range(16):
                    try:
                        name, args = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self.command(name, args)
                    except Exception as exc:
                        self.error = str(exc)
                if self.source is not None:
                    try:
                        self.feed_samples(self.source.read())
                    except Exception as exc:
                        self.error = f"数据读取停止：{exc}"
                        try:
                            self.disconnect()
                        except Exception as close_error:
                            self.error += f"；关闭：{close_error}"
                else:
                    self._stop.wait(.01)
                now = self.clock()
                if now - last_publish >= 1 / 30:
                    self.publish(now)
                    last_publish = now
        except Exception as exc:
            self.error = f"预览停止：{exc}"
        finally:
            try:
                self.disconnect()
                self.publish()
            except Exception as exc:
                with self._lock:
                    self._snapshot["runtime"] = dict(connected=False, error=f"{self.error}；{exc}", message="预览已停止")


class QuickHandCanvas(HandCanvas):
    """Existing mesh/theme, with quick-mode labels instead of strict calibration UI."""
    def _header(self, painter):
        painter.setPen(QtGui.QColor("#edf2f9"))
        font = painter.font()
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(18, 27, "快速全手姿态 · 实验性估计 / 实体 3D")
        font.setBold(False)
        painter.setFont(font)
        painter.setPen(QtGui.QColor("#9caec6"))
        text = "整掌朝向跟随本次张手参考" if self._root_is_valid() else "等待本次参考 / 手掌数据；当前是示意外形"
        if not self.follow_root:
            text = "手掌局部锁定；仅改变观看方式"
        painter.drawText(18, 49, text)
        painter.drawText(18, 73, "青环：模型关节    金色：推算    灰色：该指暂无可用姿态")
        painter.setPen(QtGui.QColor("#e5edf5"))
        painter.drawText(18, 97, self.view_identity())

    def _footer(self, painter):
        painter.setPen(QtGui.QColor("#a4b5cc"))
        text = "已断开 / 数据停止，画面不是当前手姿" if self.stale else "快速估计，不是已验证的解剖关节角；DIP 由模型推算"
        painter.drawText(18, self.height() - 39, text)
        painter.setPen(QtGui.QColor("#899db9"))
        painter.drawText(18, self.height() - 17, "六轴航向会漂移；不估计全局位置。拖动旋转 · 滚轮缩放 · 双击复位")


def display_model(model):
    """Grey only unavailable fingers, without changing estimator state or handedness."""
    result = deepcopy(model or {})
    for finger in result.get("fingers", {}).values():
        if not finger.get("geometry_live", finger.get("valid", False)):
            finger["valid"] = False
            finger["geometry_live"] = False
            finger["quality"] = "unavailable"
    return result


class QuickHandWindow(QtWidgets.QMainWindow):
    def __init__(self, config, *, port="", worker=None, start_worker=True, parent=None):
        super().__init__(parent)
        self.config = deepcopy(config)
        self.worker = worker or QuickHandWorker(self.config)
        self.setWindowTitle("快速全手姿态 · IMU 实时估计")
        self.resize(1250, 780)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        ports = QtWidgets.QHBoxLayout()
        ports.addWidget(QtWidgets.QLabel("串口"))
        self.port_box = QtWidgets.QComboBox()
        self.port_box.setEditable(True)
        self.port_box.setMinimumWidth(270)
        self.port_box.setEditText(port)
        ports.addWidget(self.port_box, 1)
        self.refresh_ports_button = QtWidgets.QPushButton("刷新端口")
        self.connect_button = QtWidgets.QPushButton("连接")
        self.disconnect_button = QtWidgets.QPushButton("断开")
        for button in (self.refresh_ports_button, self.connect_button, self.disconnect_button):
            ports.addWidget(button)
        layout.addLayout(ports)
        self.status_label = QtWidgets.QLabel("等待手动连接；端口仅预填，不会自动连接")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        body = QtWidgets.QHBoxLayout()
        self.canvas = QuickHandCanvas(config=self.config)
        body.addWidget(self.canvas, 3)
        side = QtWidgets.QVBoxLayout()
        self.reference_button = QtWidgets.QPushButton("1  张手参考：准备 3 秒 + 保持 2 秒")
        self.learn_button = QtWidgets.QPushButton("2  整手坐标对齐：准备 3 秒 + 采集 10 秒")
        self.alignment_button = self.learn_button
        self.cancel_button = QtWidgets.QPushButton("取消当前参考 / 学习")
        for button in (self.reference_button, self.learn_button, self.cancel_button):
            button.setMinimumHeight(38)
            side.addWidget(button)
        self.guidance_label = QtWidgets.QLabel("连接 → 张手参考 → 整手坐标对齐\n点击后先准备 3 秒；绿色出现时立即转动，不要等采集结束才动作。")
        self.guidance_label.setWordWrap(True)
        guidance_policy = self.guidance_label.sizePolicy()
        guidance_policy.setHeightForWidth(False)
        guidance_policy.setVerticalPolicy(QtWidgets.QSizePolicy.Fixed)
        self.guidance_label.setSizePolicy(guidance_policy)
        self.guidance_label.setMinimumHeight(82)
        self.guidance_label.setStyleSheet("font-size: 17px; padding: 8px;")
        side.addWidget(self.guidance_label)
        self.angles_table = QtWidgets.QTableWidget(5, 6)
        self.angles_table.setHorizontalHeaderLabels(["手指", "MCP", "PIP/IP", "DIP*", "张开", "状态"])
        self.angles_table.verticalHeader().setVisible(False)
        self.angles_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.angles_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.angles_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.angles_table.setMinimumWidth(420)
        self.angles_table.setMaximumHeight(200)
        side.addWidget(self.angles_table)
        viewing = QtWidgets.QHBoxLayout()
        self.follow_box = QtWidgets.QCheckBox("整掌朝向跟随")
        self.follow_box.setChecked(True)
        viewing.addWidget(self.follow_box)
        for text, surface in (("看手背", "dorsal"), ("看手心", "palmar")):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda checked=False, s=surface: self.canvas.set_surface_view(s))
            viewing.addWidget(button)
        side.addLayout(viewing)
        self.nodes_label = QtWidgets.QLabel("实时节点：0 / 12")
        side.addWidget(self.nodes_label)
        self.parameters_group = QtWidgets.QGroupBox("方向修正 / 学习参数")
        self.parameters_group.setCheckable(True)
        self.parameters_group.setChecked(False)
        group_layout = QtWidgets.QVBoxLayout(self.parameters_group)
        self.parameters_body = QtWidgets.QWidget()
        controls = QtWidgets.QVBoxLayout(self.parameters_body)
        controls.setContentsMargins(0, 0, 0, 0)
        direction_row = QtWidgets.QHBoxLayout()
        self.finger_box = QtWidgets.QComboBox()
        for name, label in FINGER_LABELS.items():
            self.finger_box.addItem(label, name)
        self.segment_box = QtWidgets.QComboBox()
        for label, segment in (("近节", "proximal"), ("中末节", "distal"), ("整指", "both")):
            self.segment_box.addItem(label, segment)
        self.segment_box.setCurrentIndex(2)
        self.flip_button = QtWidgets.QPushButton("反转方向")
        for widget in (self.finger_box, self.segment_box, self.flip_button):
            direction_row.addWidget(widget)
        controls.addLayout(direction_row)
        file_row = QtWidgets.QHBoxLayout()
        self.save_profile_button = QtWidgets.QPushButton("保存学习参数 JSON")
        self.load_profile_button = QtWidgets.QPushButton("加载学习参数 JSON")
        file_row.addWidget(self.save_profile_button)
        file_row.addWidget(self.load_profile_button)
        controls.addLayout(file_row)
        parameter_note = QtWidgets.QLabel("仅在弯曲方向相反时手动修正。加载不连接串口，需重新张手设参考；不会覆盖已有文件。")
        parameter_note.setWordWrap(True)
        controls.addWidget(parameter_note)
        group_layout.addWidget(self.parameters_body)
        self.parameters_body.setVisible(False)
        self.parameters_group.toggled.connect(self.parameters_body.setVisible)
        side.addWidget(self.parameters_group)
        # Keep unfinished parameter tools out of this release's operator path.
        # Worker support remains for a later separately verified release.
        self.parameters_group.setVisible(False)
        self.notice_label = QtWidgets.QLabel()
        self.notice_label.setWordWrap(True)
        side.addWidget(self.notice_label)
        self.warnings_label = QtWidgets.QLabel()
        self.warnings_label.setWordWrap(True)
        side.addWidget(self.warnings_label)
        note = QtWidgets.QLabel("这是快速姿态估计，不是已认证的精确动捕。\nDIP* 为模型推算；六轴航向仍可能漂移，无全局位置。\n灰色只表示对应手指缺少可用数据，其余手指继续显示。")
        note.setWordWrap(True)
        side.addWidget(note)
        side.addStretch(1)
        body.addLayout(side, 2)
        layout.addLayout(body, 1)
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(lambda: self.send("connect", port=self.port_box.currentText()))
        self.disconnect_button.clicked.connect(lambda: self.send("disconnect"))
        self.reference_button.clicked.connect(lambda: self.send("reference"))
        self.learn_button.clicked.connect(lambda: self.send("align"))
        self.cancel_button.clicked.connect(lambda: self.send("cancel"))
        self.follow_box.toggled.connect(self.canvas.set_root_follow)
        self.flip_button.clicked.connect(lambda: self.send("flip_direction", finger=self.finger_box.currentData(), segment=self.segment_box.currentData()))
        self.save_profile_button.clicked.connect(self.save_profile_dialog)
        self.load_profile_button.clicked.connect(self.load_profile_dialog)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        if start_worker:
            self.worker.start()
        self.refresh()

    def send(self, name, **args):
        if not self.worker.submit(name, **args):
            self.status_label.setText("操作正在处理，请稍候")

    def refresh_ports(self):
        selected = self.port_box.currentText()
        try:
            ports = list_serial_ports()
            self.port_box.clear()
            for item in ports:
                device = item["device"] if isinstance(item, dict) else item[0] if isinstance(item, (tuple, list)) else str(item)
                self.port_box.addItem(device)
            self.port_box.setEditText(selected)
        except Exception as exc:
            self.status_label.setText(f"无法枚举端口，可直接输入路径：{exc}")

    def save_profile_dialog(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存快速模式学习参数（不覆盖）", "quick_hand_profile.json", "JSON (*.json)")
        if path:
            self.send("save_profile", path=path)

    def load_profile_dialog(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "加载快速模式学习参数", "", "JSON (*.json)")
        if path:
            self.send("load_profile", path=path)

    def refresh(self):
        snapshot = self.worker.latest_snapshot()
        runtime = snapshot.get("runtime", {})
        calibration = snapshot.get("calibration", {})
        connected = bool(runtime.get("connected"))
        active = calibration.get("active")
        ready = bool(calibration.get("neutral_ready"))
        nodes = snapshot.get("nodes", {})
        fresh_count = sum(bool(node.get("fresh")) for node in nodes.values())
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.port_box.setEnabled(not connected)
        self.reference_button.setEnabled(connected and not active)
        self.learn_button.setEnabled(connected and ready and not active)
        self.cancel_button.setEnabled(connected and bool(active))
        for widget in (self.flip_button, self.save_profile_button, self.load_profile_button, self.finger_box, self.segment_box):
            widget.setEnabled(not active)
        self.notice_label.setText(runtime.get("notice", ""))
        message = runtime.get("message", "等待手动连接")
        if connected and not active and ready:
            message = calibration.get("message") or "参考已设置，五指姿态正在显示；步骤 2 对齐全手坐标。"
        self.status_label.setText(runtime.get("error") or message)
        self.status_label.setStyleSheet("color: #b23b3b;" if runtime.get("error") else "")
        if active:
            is_reference = active.get("kind") in {"reference", "neutral"}
            remaining = max(0., float(active.get("remaining_s", 0)))
            if active.get("phase") == "preparing":
                instruction = "准备，请先保持伸直\n掌心朝下。绿色出现后保持不动。" if is_reference else "准备，请先保持伸直\n绿色出现时立即转动，不要等采集结束才动作。"
                self.guidance_label.setText(f"{instruction}\n准备剩余 {remaining:.1f} 秒（尚未采集）")
                self.guidance_label.setStyleSheet("font-size: 21px; font-weight: bold; padding: 10px; background: #fff1b8; color: #634600;")
            elif is_reference:
                self.guidance_label.setText(f"正在采集参考，请保持不动！\n掌心朝下，五指自然伸直\n采集剩余 {remaining:.1f} 秒")
                self.guidance_label.setStyleSheet("font-size: 21px; font-weight: bold; padding: 10px; background: #daf5df; color: #14552d;")
            elif active.get("kind") == "frame_alignment":
                self.guidance_label.setText(f"现在开始转动，正在采集！\n五指伸直，整个手掌向下倾斜约 30°再回水平，重复\n不要握拳 / 上下平移\n采集剩余 {remaining:.1f} 秒")
                self.guidance_label.setStyleSheet("font-size: 21px; font-weight: bold; padding: 10px; background: #daf5df; color: #14552d;")
            else:
                self.guidance_label.setText(f"正在采集\n采集剩余 {remaining:.1f} 秒")
                self.guidance_label.setStyleSheet("font-size: 21px; font-weight: bold; padding: 10px; background: #daf5df; color: #14552d;")
        elif ready:
            self.guidance_label.setText(calibration.get("message") or "参考已设置，五指姿态正在显示；步骤 2 对齐全手坐标。")
            self.guidance_label.setStyleSheet("font-size: 17px; padding: 8px;")
        else:
            self.guidance_label.setText(calibration.get("message") or "连接后点击步骤 1：准备 3 秒，掌心朝下，五指自然伸直，再保持 2 秒。")
            self.guidance_label.setStyleSheet("font-size: 17px; padding: 8px;")
        self.nodes_label.setText(f"实时节点：{fresh_count} / 12    已处理 {runtime.get('received_samples', 0)} 条")
        model = display_model(snapshot.get("model"))
        stale = not connected or (bool(nodes) and fresh_count == 0)
        self.canvas.set_model(model, mode="快速全手估计", stale=stale,
                              published_monotonic_s=runtime.get("published_monotonic_s"))
        for row, (name, label) in enumerate(FINGER_LABELS.items()):
            finger = model.get("fingers", {}).get(name, {})
            available = bool(finger.get("geometry_live", finger.get("valid", False))) and not stale
            values = [label]
            for key in ("mcp_deg", "ip_deg" if name == "thumb" else "pip_deg", "dip_deg", "splay_deg"):
                value = finger.get(key) if available else None
                values.append(f"{value:.1f}°" if isinstance(value, (int, float)) and math.isfinite(value) else "—")
            values.append("快速估计" if available else "暂无数据" if ready else "待参考")
            for col, text in enumerate(values):
                item = self.angles_table.item(row, col)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    self.angles_table.setItem(row, col, item)
                item.setText(text)
                item.setForeground(QtGui.QColor("#263d48" if available else "#7b858d"))
        warnings = snapshot.get("warnings", [])
        visible_warnings = warnings if len(warnings) <= 4 else [warnings[0], *warnings[-3:]]
        self.warnings_label.setText("\n".join(str(value) for value in visible_warnings))
        if self.isVisible():
            # Large phase instructions must not be squeezed by the angle table.
            # Recompute after wrapping at the actual sidebar width, not a fixed
            # height that hides the final "collection remaining" line.
            width = max(1, self.guidance_label.contentsRect().width() - 20)
            bounds = self.guidance_label.fontMetrics().boundingRect(
                QtCore.QRect(0, 0, width, 10000), QtCore.Qt.TextWordWrap,
                self.guidance_label.text())
            self.guidance_label.setFixedHeight(max(82, bounds.height() + 24))

    def closeEvent(self, event):
        self.timer.stop()
        self.worker.stop()
        if self.worker.isRunning() and not self.worker.wait(1500):
            self.status_label.setText("正在关闭串口，请稍候再次关闭窗口")
            self.timer.start()
            event.ignore()
            return
        event.accept()


def launch_quick_hand(config, port="", smoke_seconds=None):
    if smoke_seconds is not None and (not math.isfinite(smoke_seconds) or smoke_seconds <= 0):
        raise ValueError("smoke_seconds must be finite and positive")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = QuickHandWindow(config, port=port)
    window.show()
    if smoke_seconds is not None:
        QtCore.QTimer.singleShot(max(1, int(smoke_seconds * 1000)), window.close)
    return app.exec_()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="Explicit quick-hand config; no live config is changed")
    parser.add_argument("--port", default="", help="Prefill only; connection is always manual")
    parser.add_argument("--smoke-seconds", type=float, help="Close an unconnected UI after this duration")
    args = parser.parse_args(argv)
    from .config import load_config
    return launch_quick_hand(load_config(args.config), port=args.port, smoke_seconds=args.smoke_seconds)


if __name__ == "__main__":
    raise SystemExit(main())
