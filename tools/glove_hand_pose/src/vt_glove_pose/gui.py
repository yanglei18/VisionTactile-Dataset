"""Manual-only desktop operation, with bounded cross-thread snapshot delivery.

All source/engine/file operations belong to one worker. The GUI polls one latest
snapshot at 30 Hz; per-sample Qt signals are deliberately not generated.
"""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import queue
import threading
import time

from PyQt5 import QtCore, QtWidgets

from .canvas import FINGER_LABELS, HandCanvas
from .diagnostic_recording import DiagnosticRecorder, MARKERS
from .stream import RawRecorder, ReplaySource, SerialSource, list_serial_ports


def axis_progress(config: dict, calibration: dict, *, demo=False) -> tuple[str, str]:
    """Summarize the six operator steps from actual configured node IDs."""
    if demo:
        return "功能轴：模拟数据，不表示完成真实标定", "模拟模式不会建立或保存真机标定。"
    available = {str(node) for node in calibration.get("axis_nodes", [])}
    targets = {"手掌": [config.get("palm_node", 12)]}
    for name, label in FINGER_LABELS.items():
        finger = config.get("fingers", {}).get(name, {})
        targets[label] = [finger.get("proximal"), finger.get("distal")]
    items, details, complete = [], [], 0
    for label, required in targets.items():
        configured = all(node is not None for node in required)
        count = sum(str(node) in available for node in required if node is not None)
        ready = configured and count == len(required)
        complete += int(ready)
        state = "✓" if ready else f"{count}/{len(required)}" if configured else "待配置"
        items.append(f"{label} {state}")
        details.append(label + "：" + "、".join(
            "节点未配置" if node is None else f"Node {node} {'已有轴标定' if str(node) in available else '待标定'}"
            for node in required))
    text = f"功能轴 {complete}/6 完成：" + "；".join(items)
    details.append("每根手指必须两个配置节点均有轴标定；本次静止归零状态单独显示。")
    return text, "\n".join(details)


class PoseWorker(QtCore.QThread):
    def __init__(self, config: dict, *, mode="live", engine_factory=None,
                 serial_factory=None, replay_factory=None, recorder_factory=None,
                 demo_factory=None, calibration_path=None, diagnostic_factory=None, parent=None):
        super().__init__(parent)
        if mode not in {"live", "replay", "demo"}:
            raise ValueError("mode must be live, replay, or demo")
        self.config = deepcopy(config)
        self.mode = mode
        self._engine_factory = engine_factory
        self._serial_factory = serial_factory or SerialSource
        self._replay_factory = replay_factory or ReplaySource
        self._recorder_factory = recorder_factory or RawRecorder
        self._diagnostic_factory = diagnostic_factory or DiagnosticRecorder
        self._diagnostic = None
        self._diagnostic_error = ""
        self._demo_factory = demo_factory
        self._calibration_path = str(calibration_path) if calibration_path else None
        self._commands = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = {"runtime": {"mode": mode, "connected": False, "message": "等待手动操作", "error": ""}}
        self._engine = None
        self._source = None
        self._recorder = None
        self._recording_path = None
        self._source_path = None
        self._error = ""
        self._message = "模拟数据" if mode == "demo" else "等待手动连接" if mode == "live" else "请选择 CSV 后手动开始回放"
        self._eof = False
        self._last_axis_request_id = None
        self._reset_performance()

    def _reset_performance(self):
        # Scalar last/peak/counter observations only, never per-sample history.
        self._performance = {
            "basis": "host_observed_not_sensor_latency",
            "source_read_elapsed_ms": 0.0, "fusion_elapsed_ms": 0.0,
            "fusion_elapsed_ms_max": 0.0, "record_elapsed_ms": 0.0,
            "batch_samples": 0, "batch_samples_max": 0, "total_samples": 0,
            "read_batches": 0, "snapshot_build_elapsed_ms": 0.0,
            "snapshot_published_monotonic_s": None,
        }

    def submit(self, command: str, **arguments) -> bool:
        try:
            self._commands.put_nowait((command, arguments))
            return True
        except queue.Full:
            return False

    def latest_snapshot(self) -> dict:
        with self._lock:
            snapshot = deepcopy(self._snapshot)
        # The disk writer may finish after acquisition has stopped publishing.
        snapshot.setdefault("runtime", {})["diagnostic"] = self._diagnostic_status()
        return snapshot

    def stop(self) -> None:
        self._stop.set()

    def _new_engine(self):
        if self._engine_factory is None:
            from .engine import PoseEngine
            self._engine_factory = PoseEngine
        return self._engine_factory(self.config)

    def _close_recorder(self):
        recorder, self._recorder = self._recorder, None
        self._recording_path = None
        if recorder is not None:
            recorder.close()

    def _diagnostic_status(self):
        if self._diagnostic is None:
            return dict(state="idle", path=None, error=self._diagnostic_error)
        try:
            status = self._diagnostic.status()
            if not isinstance(status, dict):
                raise ValueError("invalid diagnostic writer status")
            if self._diagnostic_error:
                error = str(status.get("error", ""))
                if self._diagnostic_error not in error:
                    error = f"{error}；{self._diagnostic_error}" if error else self._diagnostic_error
                status = dict(status, state="failed", error=error)
            return status
        except Exception as exc:
            return dict(state="failed", path=None, error=f"诊断状态失败：{exc}")

    def _fail_diagnostic(self, message):
        self._diagnostic_error = str(message)
        if self._diagnostic is not None:
            try:
                self._diagnostic.fail(message)
            except Exception as exc:
                self._diagnostic_error += f"；停止诊断也失败：{exc}"

    def _emit_diagnostic(self, event_type, **payload):
        if self._diagnostic_status().get("state") != "recording":
            return
        try:
            self._diagnostic.offer(event_type, **payload)
        except Exception as exc:
            self._fail_diagnostic(f"诊断记录失败，采集继续：{exc}")

    def _diagnostic_batch(self, samples):
        if self._diagnostic_status().get("state") != "recording":
            return
        try:
            rows = [dict(node_id=sample.node_id, values=list(sample.values),
                         host_monotonic_s=sample.host_monotonic_s,
                         host_wall_s=sample.host_wall_s, status=sample.status) for sample in samples]
            self._emit_diagnostic("batch", samples=rows, performance=self._performance)
        except Exception as exc:
            self._fail_diagnostic(f"诊断批次失败，采集继续：{exc}")

    def _diagnostic_checkpoint(self):
        if self._diagnostic_status().get("state") == "recording":
            try:
                self._emit_diagnostic("checkpoint", checkpoint=self._engine.export_diagnostic_checkpoint())
            except Exception as exc:
                self._fail_diagnostic(f"诊断检查点失败，采集继续：{exc}")

    def _stop_diagnostic(self, reason):
        if self._diagnostic is not None:
            try:
                self._diagnostic.stop(reason)
            except Exception as exc:
                self._fail_diagnostic(f"停止诊断失败，采集继续：{exc}")

    def _close_source(self, reason="disconnect"):
        self._stop_diagnostic(reason)
        source, self._source = self._source, None
        try:
            self._close_recorder()
        finally:
            if source is not None:
                source.close()

    def _command(self, command: str, args: dict):
        if command == "disconnect":
            self._close_source()
            self._engine.cancel_calibration()
            self._message = "已断开；未关闭其他程序"
            return
        if command == "set_hand":
            if self._source is not None:
                raise ValueError("请先断开，再修改左右手；修改会清空本次标定")
            if args["hand"] not in {"left", "right"}:
                raise ValueError("无效的左右手选项")
            self.config["hand"] = args["hand"]
            self.config["hand_side_confirmed"] = bool(args.get("confirmed", False))
            self._engine = self._new_engine()
            self._reset_performance()
            self._message = "手模型已切换，本次标定已清空"
            return
        if self.mode == "demo":
            raise ValueError("模拟模式不连接硬件、不录制、不进行真机标定")
        if command == "connect":
            if self._source is not None:
                raise ValueError("已有连接，请先断开")
            path = str(args.get("source", "")).strip()
            if not path:
                raise ValueError("请明确选择串口或 CSV 文件")
            source = self._serial_factory(path) if self.mode == "live" else self._replay_factory(path, speed=1.0)
            try:
                source.open()
                engine = self._new_engine()
                if self._calibration_path:
                    engine.load_calibration(self._calibration_path)
            except Exception:
                source.close()
                raise
            self._source = source
            self._engine = engine
            self._source_path = path
            self._eof = False
            self._reset_performance()
            self._message = "已连接；请进行本次静止标定" if self.mode == "live" else "正在回放原始 CSV；非实时硬件"
            if self._calibration_path:
                self._message += "；已加载明确指定的标定文件，仍需重新归零"
        elif command == "neutral":
            if self._source is None or self._eof:
                raise ValueError("需要有效数据源才能开始静止标定")
            self._engine.begin_neutral(duration=5, confirm_mount=bool(args.get("confirm_mount", False)))
            self._emit_diagnostic("command", name="neutral", args=dict(confirm_mount=bool(args.get("confirm_mount", False)), duration=5))
            self._message = "保持平手、静止 5 秒；本步骤也用于重新归零"
        elif command == "axis":
            try:
                if self._source is None or self._eof:
                    raise ValueError("需要有效数据源才能开始功能轴标定")
                axis_args = dict(target=args["target"], duration=8)
                # Strict/default keeps the exact historical command shape.
                if args.get("calibration_method", "strict_v1") != "strict_v1":
                    axis_args["calibration_method"] = args["calibration_method"]
                self._engine.begin_axis(**axis_args)
                self._emit_diagnostic("command", name="axis", args=axis_args)
                self._message = (self._engine.message if args["target"] in {"thumb_common", "thumb_hinge"}
                                 else "轻柔地先弯曲再伸直；保持其他部位稳定")
            finally:
                # GUI acknowledgement only: not an engine/replay argument.
                self._last_axis_request_id = args.get("_gui_request_id")
        elif command == "cancel":
            self._engine.cancel_calibration()
            self._emit_diagnostic("command", name="cancel", args={})
            self._message = "已取消当前标定"
        elif command == "save":
            path = Path(args["path"])
            if path.exists():
                raise ValueError("文件已存在，请选择新文件名；不会覆盖")
            self._engine.save_calibration(str(path))
            self._message = f"已保存标定：{path}"
        elif command == "load":
            if self._source is None or self._eof:
                raise ValueError("请先连接有效数据源，再加载标定；加载后需要重新静止归零")
            self._engine.load_calibration(args["path"])
            self._diagnostic_checkpoint()
            self._message = "安装标定已读取；请确认本次佩戴并重新静止归零"
        elif command == "diagnostic_start":
            if self.mode != "live" or self._source is None or self._eof:
                raise ValueError("仅已连接的实时模式可手动开始诊断录制")
            if getattr(self._engine, "active", None) is not None:
                raise ValueError("请先完成或取消当前标定，再手动开始诊断录制")
            if self._diagnostic is not None and (self._diagnostic.is_alive or self._diagnostic_status().get("state") in {"recording", "stopping"}):
                raise ValueError("上一个诊断写入线程尚未结束，不能开始新录制")
            checkpoint = self._engine.export_diagnostic_checkpoint()
            self._diagnostic = self._diagnostic_factory(args["path"])
            self._diagnostic_error = ""
            try:
                self._diagnostic.start(checkpoint)
            except Exception as exc:
                self._fail_diagnostic(f"诊断启动失败，采集继续：{exc}")
            self._message = "已请求手动诊断录制；主机侧观测不代表传感器延迟"
        elif command == "diagnostic_stop":
            self._stop_diagnostic("manual")
            self._message = "已请求停止诊断录制；后台完成写入，采集继续"
        elif command == "diagnostic_marker":
            label = args.get("label")
            if label not in MARKERS:
                raise ValueError("诊断标记只能是 rest、moving 或 returned")
            if self._diagnostic_status().get("state") != "recording":
                raise ValueError("请先手动开始诊断录制")
            self._emit_diagnostic("marker", label=label)
        elif command == "record_start":
            if self.mode != "live" or self._source is None:
                raise ValueError("仅已连接的实时模式可录制原始数据")
            if self._recorder is not None:
                raise ValueError("正在录制，请先停止")
            recorder = self._recorder_factory(args["path"])
            try:
                recorder.open()
            except Exception:
                recorder.close()
                raise
            self._recorder = recorder
            self._recording_path = str(args["path"])
            self._message = "正在录制原始数据；不会自动保存或覆盖其他文件"
        elif command == "record_stop":
            self._close_recorder()
            self._message = "原始录制已关闭"
        else:
            raise ValueError(f"未知操作：{command}")

    def _publish(self, elapsed: float):
        build_started = time.perf_counter()
        if self.mode == "demo":
            if self._demo_factory is None:
                from .engine import demo_snapshot
                self._demo_factory = demo_snapshot
            snapshot = self._demo_factory(elapsed, self.config)
        else:
            now = None
            if self.mode == "replay":
                # Advance within the recording's clock domain during gaps.
                # ReplaySource freezes this presentation clock only at EOF.
                now = getattr(self._source, "timeline_s", None)
                if now is None:
                    now = getattr(self._engine, "last_time", None)
            if now is None:
                now = time.monotonic()
            snapshot = self._engine.get_snapshot(now=now)
        snapshot = deepcopy(snapshot)
        self._performance["snapshot_build_elapsed_ms"] = (time.perf_counter() - build_started) * 1000
        self._performance["snapshot_published_monotonic_s"] = time.monotonic()
        if self.mode != "demo":
            self._emit_diagnostic("snapshot", now_s=float(now), snapshot=snapshot, performance=self._performance)
        snapshot["runtime"] = {
            "mode": self.mode, "connected": self._source is not None,
            "source": self._source_path, "eof": self._eof,
            "recording": self._recording_path, "message": self._message,
            "error": self._error, "running": not self._stop.is_set(),
            "last_axis_request_id": self._last_axis_request_id,
            "performance": dict(self._performance),
            "diagnostic": self._diagnostic_status(),
        }
        with self._lock:
            self._snapshot = snapshot

    def run(self):
        started = time.monotonic()
        next_publish = started
        try:
            self._engine = self._new_engine()
            while not self._stop.is_set():
                for _ in range(16):
                    try:
                        command, args = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self._command(command, args)
                        self._error = ""
                    except Exception as exc:
                        self._error = f"{type(exc).__name__}: {exc}"
                if self._source is not None and not self._eof:
                    try:
                        read_started = time.perf_counter()
                        samples = self._source.read()
                        performance = self._performance
                        performance["source_read_elapsed_ms"] = (time.perf_counter() - read_started) * 1000
                        source_metrics = getattr(self._source, "performance", None)
                        if isinstance(source_metrics, dict):
                            performance.update(source_metrics)
                        performance["batch_samples"] = len(samples)
                        performance["batch_samples_max"] = max(performance["batch_samples_max"], len(samples))
                        performance["read_batches"] += 1
                        performance["record_elapsed_ms"] = 0.0
                        if self._recorder is not None:
                            record_started = time.perf_counter()
                            try:
                                self._recorder.write(samples)
                            except Exception as exc:
                                self._error = f"录制失败，采集继续：{exc}"
                                try:
                                    self._close_recorder()
                                except Exception as close_exc:
                                    # ENOSPC can make both flush/write and close
                                    # fail. Neither is a serial acquisition error.
                                    self._error += f"；关闭录制文件也失败：{close_exc}"
                            finally:
                                performance["record_elapsed_ms"] = (time.perf_counter() - record_started) * 1000
                        fusion_started = time.perf_counter()
                        try:
                            for sample in samples:
                                self._engine.process(sample)
                        except Exception as exc:
                            # A partially processed batch is not replayable evidence.
                            if self._diagnostic_status().get("state") == "recording":
                                self._fail_diagnostic(f"融合批次未完成，诊断不完整：{exc}")
                            raise
                        performance["fusion_elapsed_ms"] = (time.perf_counter() - fusion_started) * 1000
                        performance["fusion_elapsed_ms_max"] = max(performance["fusion_elapsed_ms_max"], performance["fusion_elapsed_ms"])
                        performance["total_samples"] += len(samples)
                        self._diagnostic_batch(samples)
                        if self.mode == "replay" and self._source.eof:
                            self._eof = True
                            self._message = "CSV 回放结束；显示最后一帧，不是实时数据"
                            if getattr(self._engine, "active", None) is not None:
                                self._engine.cancel_calibration()
                                self._message += "；未完成的标定已取消"
                    except Exception as exc:
                        self._error = f"数据源已停止：{type(exc).__name__}: {exc}"
                        self._close_source("source_error")
                        self._engine.cancel_calibration()
                current = time.monotonic()
                if current >= next_publish:
                    self._publish(current - started)
                    next_publish = current + 1 / 30
                self.msleep(4)
        except Exception as exc:
            self._error = f"工作线程停止：{type(exc).__name__}: {exc}"
        finally:
            try:
                self._close_source("worker_shutdown")
            except Exception as exc:
                self._error = self._error or f"关闭数据源失败：{exc}"
            if self._diagnostic is not None:
                try:
                    if not self._diagnostic.wait_closed(.5):
                        self._fail_diagnostic("诊断关闭等待超时，文件不完整；未等待慢盘阻塞采集退出")
                except Exception as exc:
                    self._fail_diagnostic(f"诊断关闭失败：{exc}")
            with self._lock:
                runtime = self._snapshot.setdefault("runtime", {})
                runtime.update(connected=False, recording=None, running=False, error=self._error)


class HandPoseWindow(QtWidgets.QMainWindow):
    def __init__(self, config: dict, *, mode="live", source=None, calibration_path=None, worker=None):
        super().__init__()
        self.mode = mode
        self._view_config = deepcopy(config)
        self._calibration_path = calibration_path
        self._closing = False
        self._ui_notice = ""
        self._thumb_preparation = None
        self._thumb_queued_at = None
        self._thumb_request_serial = 0
        self._thumb_prepare_timer = QtCore.QTimer(self)
        self._thumb_prepare_timer.setInterval(100)
        self._thumb_prepare_timer.timeout.connect(self._advance_thumb_preparation)
        self.setWindowTitle("12 IMU 手部姿态研究原型" + (" — 模拟数据" if mode == "demo" else ""))
        self.resize(1440, 1000)
        self.worker = worker or PoseWorker(config, mode=mode, calibration_path=calibration_path)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(8, 6, 8, 6)
        outer.setSpacing(4)
        palm_node, auxiliary_node = config.get("palm_node", 12), config.get("auxiliary_node", 11)
        self.explanation_label = QtWidgets.QLabel(f"研究原型：Node {palm_node} 是待验证的手掌参考，辅助 Node {auxiliary_node} 不预设为前臂。缺测骨段由模型估计；六轴航向可能漂移。")
        self.explanation_label.setWordWrap(True)
        outer.addWidget(self.explanation_label)

        connection = QtWidgets.QHBoxLayout()
        self.source_mode_label = QtWidgets.QLabel({
            "live": "实时串口（仅手动连接）", "replay": "原始 CSV 回放",
            "demo": "模拟数据 / DEMO · 不连接硬件，不代表真机精度",
        }[mode])
        if mode == "demo":
            self.source_mode_label.setStyleSheet("font-weight: 600; color: #255a82;")
        connection.addWidget(self.source_mode_label)
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.setEditable(True)
        self.source_combo.setMinimumWidth(300)
        connection.addWidget(self.source_combo, 1)
        self.refresh_button = QtWidgets.QPushButton("刷新串口" if mode == "live" else "选择 CSV")
        self.refresh_button.clicked.connect(self._refresh_or_browse)
        connection.addWidget(self.refresh_button)
        self.connect_button = QtWidgets.QPushButton("连接" if mode == "live" else "开始回放")
        self.connect_button.clicked.connect(lambda: self._submit("connect", source=self.source_combo.currentText()))
        connection.addWidget(self.connect_button)
        self.disconnect_button = QtWidgets.QPushButton("断开 / 停止回放")
        self.disconnect_button.clicked.connect(lambda: self._submit("disconnect"))
        connection.addWidget(self.disconnect_button)
        if mode == "demo":
            for widget in (self.source_combo, self.refresh_button, self.connect_button, self.disconnect_button):
                widget.hide()
            connection.addStretch(1)
        self.hand_combo = QtWidgets.QComboBox()
        self.hand_combo.addItem("右手模型（请核对）", "right")
        self.hand_combo.addItem("左手模型（请核对）", "left")
        self.hand_combo.setCurrentIndex(1 if config.get("hand") == "left" else 0)
        connection.addWidget(self.hand_combo)
        self.hand_check = QtWidgets.QCheckBox("已核对左右手")
        self.hand_check.setChecked(bool(config.get("hand_side_confirmed", False)))
        self.hand_combo.currentIndexChanged.connect(self._hand_changed)
        self.hand_check.toggled.connect(self._hand_changed)
        connection.addWidget(self.hand_check)
        self.hand_check.setVisible(mode != "demo")
        outer.addLayout(connection)

        self.calibration_group = calibration = QtWidgets.QGroupBox("手动标定：不自动推进动作、不自动写入文件")
        cal_layout = QtWidgets.QVBoxLayout(calibration)
        cal_layout.setContentsMargins(8, 6, 8, 6)
        cal_layout.setSpacing(4)
        self.mount_check = QtWidgets.QCheckBox(f"已确认手套固定、手掌参考 Node {palm_node} 贴合且平手")
        cal_layout.addWidget(self.mount_check)
        cal_actions = QtWidgets.QHBoxLayout()
        self.neutral_button = QtWidgets.QPushButton("静止 5 秒 / 重新归零")
        self.neutral_button.clicked.connect(lambda: self._submit("neutral", confirm_mount=self.mount_check.isChecked()))
        cal_actions.addWidget(self.neutral_button)
        self.axis_combo = QtWidgets.QComboBox()
        self.axis_combo.addItem("拇指 A · 先做：固定形状转掌", "thumb_common")
        self.axis_combo.addItem("拇指 B · A 成功后：扶近节屈伸末节", "thumb_hinge")
        for name, label in {"palm": "手掌参考", **FINGER_LABELS}.items():
            self.axis_combo.addItem("拇指（旧版独立轴，非联合 A/B）" if name == "thumb" else label, name)
        cal_actions.addWidget(self.axis_combo)
        self.axis_button = QtWidgets.QPushButton("开始所选功能轴")
        self.axis_button.clicked.connect(lambda: self._request_axis(self.axis_combo.currentData()))
        cal_actions.addWidget(self.axis_button)
        self.cancel_button = QtWidgets.QPushButton("取消标定")
        self.cancel_button.clicked.connect(lambda: self._submit("cancel"))
        cal_actions.addWidget(self.cancel_button)
        self.save_button = QtWidgets.QPushButton("保存标定 JSON")
        self.save_button.clicked.connect(self._save_calibration)
        cal_actions.addWidget(self.save_button)
        self.load_button = QtWidgets.QPushButton("加载标定 JSON")
        self.load_button.clicked.connect(self._load_calibration)
        cal_actions.addWidget(self.load_button)
        cal_layout.addLayout(cal_actions)
        thumb_actions = QtWidgets.QHBoxLayout()
        self.thumb_common_button = QtWidgets.QPushButton("① 拇指 A：准备 3 秒 → 采集 8 秒")
        self.thumb_hinge_button = QtWidgets.QPushButton("② 拇指 B：准备 3 秒 → 采集 8 秒")
        self.thumb_common_button.clicked.connect(lambda: self._request_axis("thumb_common"))
        self.thumb_hinge_button.clicked.connect(lambda: self._request_axis("thumb_hinge"))
        thumb_actions.addWidget(self.thumb_common_button)
        thumb_actions.addWidget(self.thumb_hinge_button)
        thumb_actions.addStretch(1)
        self.calibration_help_button = QtWidgets.QToolButton()
        self.calibration_help_button.setText("查看详细动作说明")
        self.calibration_help_button.setCheckable(True)
        self.calibration_help_button.setArrowType(QtCore.Qt.RightArrow)
        self.calibration_help_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        thumb_actions.addWidget(self.calibration_help_button)
        cal_layout.addLayout(thumb_actions)
        self.calibration_help = QtWidgets.QWidget()
        help_layout = QtWidgets.QVBoxLayout(self.calibration_help)
        help_layout.setContentsMargins(0, 0, 0, 0)
        instructions = QtWidgets.QLabel("四指 / 旧版独立轴：静止归零后轻柔地先弯曲、再伸直；手掌参考：五指伸直，手腕下弯再回正。拇指联合 A/B 按下面专门提示操作。加载旧标定时先加载，再完成本次静止归零。")
        instructions.setWordWrap(True)
        help_layout.addWidget(instructions)
        thumb_instructions = QtWidgets.QLabel("拇指先 A 后 B：A 保持拇指形状不变，上下翻掌并左右转掌。A 成功后手动点击 B，利用 3 秒准备时间用另一只手扶稳近节；看到“采集中”后，仅末节先弯再伸。准备期不采集标定动作；不会自动进入下一步，失败保留旧轴。")
        thumb_instructions.setWordWrap(True)
        help_layout.addWidget(thumb_instructions)
        cal_layout.addWidget(self.calibration_help)
        self.calibration_help.hide()
        self.calibration_help_button.toggled.connect(self.calibration_help.setVisible)
        self.calibration_help_button.toggled.connect(lambda expanded: self.calibration_help_button.setArrowType(
            QtCore.Qt.DownArrow if expanded else QtCore.Qt.RightArrow))
        self.thumb_capture_label = QtWidgets.QLabel("拇指联合标定：先完成静止归零，再手动开始 A。")
        self.thumb_capture_label.setWordWrap(True)
        self.thumb_capture_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        cal_layout.addWidget(self.thumb_capture_label)
        self.calibration_label = QtWidgets.QLabel("尚未标定")
        self.calibration_label.setWordWrap(True)
        self.calibration_label.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.calibration_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.calibration_result_scroll = QtWidgets.QScrollArea()
        self.calibration_result_scroll.setWidgetResizable(True)
        self.calibration_result_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.calibration_result_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.calibration_result_scroll.setMaximumHeight(56)
        self.calibration_result_scroll.setMinimumHeight(28)
        self.calibration_result_scroll.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        self.calibration_result_scroll.setWidget(self.calibration_label)
        self.calibration_result_scroll.setToolTip("完整标定结果；长内容可在此滚动、选择复制，也可悬停查看。")
        cal_layout.addWidget(self.calibration_result_scroll)
        self.axes_progress_label = QtWidgets.QLabel()
        self.axes_progress_label.setWordWrap(True)
        self.axes_progress_label.setMaximumHeight(45)
        cal_layout.addWidget(self.axes_progress_label)
        outer.addWidget(calibration)
        calibration.setVisible(mode != "demo")

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        visual_panel = QtWidgets.QWidget()
        visual_layout = QtWidgets.QVBoxLayout(visual_panel)
        visual_layout.setContentsMargins(0, 0, 0, 0)
        visual_controls = QtWidgets.QHBoxLayout()
        visual_controls.addWidget(QtWidgets.QLabel("显示方式"))
        self.render_combo = QtWidgets.QComboBox()
        self.render_combo.addItem("实体 3D", "solid")
        self.render_combo.addItem("骨架调试", "skeleton")
        visual_controls.addWidget(self.render_combo)
        self.root_follow_check = QtWidgets.QCheckBox("整掌朝向跟随")
        self.root_follow_check.setChecked(True)
        self.root_follow_check.setToolTip("勾选：跟随本次零位参考下的手掌朝向。取消：手掌局部锁定，用于检查手指。六轴航向仍会漂移。")
        visual_controls.addWidget(self.root_follow_check)
        visual_controls.addStretch(1)
        visual_layout.addLayout(visual_controls)
        self.canvas = HandCanvas(config=config)
        self.dorsal_view_button = QtWidgets.QPushButton("看手背")
        self.palmar_view_button = QtWidgets.QPushButton("看手心")
        for button, surface in ((self.dorsal_view_button, "dorsal"), (self.palmar_view_button, "palmar")):
            button.setToolTip("只调整观察视角，不切换左右手，不归零，不改变采集或标定；整掌跟随开启时仍随之后的手掌运动。")
            button.clicked.connect(lambda _checked=False, face=surface: self.canvas.set_surface_view(face))
            visual_controls.insertWidget(visual_controls.count()-1, button)
        self.render_combo.currentIndexChanged.connect(lambda: self.canvas.set_render_mode(self.render_combo.currentData()))
        self.root_follow_check.toggled.connect(self.canvas.set_root_follow)
        visual_layout.addWidget(self.canvas, 1)
        self.performance_label = QtWidgets.QLabel("主机侧诊断：等待数据；不是传感器端到端延迟")
        self.performance_label.setWordWrap(True)
        self.performance_label.setMaximumHeight(48)
        visual_layout.addWidget(self.performance_label)
        splitter.addWidget(visual_panel)
        side = QtWidgets.QWidget()
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.setContentsMargins(6, 4, 6, 4)
        side_layout.setSpacing(4)
        self.nodes_table = QtWidgets.QTableWidget(12, 5)
        self.nodes_table.setHorizontalHeaderLabels(["节点", "数据", "年龄 ms", "接收 Hz", "样本数"])
        self.nodes_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.nodes_table.verticalHeader().hide()
        self.nodes_table.verticalHeader().setMinimumSectionSize(20)
        self.nodes_table.verticalHeader().setDefaultSectionSize(22)
        self.nodes_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        self.nodes_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.nodes_table.setMinimumWidth(440)
        self.nodes_table.setMinimumHeight(12 * 22 + self.nodes_table.horizontalHeader().sizeHint().height() + 4)
        side_layout.addWidget(self.nodes_table)
        self.angles_table = QtWidgets.QTableWidget(5, 5)
        self.angles_table.setHorizontalHeaderLabels(["手指", "MCP °", "PIP/IP °", "DIP °*", "侧摆 °"])
        self.angles_table.verticalHeader().hide()
        self.angles_table.verticalHeader().setMinimumSectionSize(20)
        self.angles_table.verticalHeader().setDefaultSectionSize(22)
        self.angles_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        self.angles_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.angles_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.angles_table.setMinimumHeight(5 * 22 + self.angles_table.horizontalHeader().sizeHint().height() + 4)
        side_layout.addWidget(self.angles_table)
        self.thumb_quality_label = QtWidgets.QLabel("拇指抗漂移：等待配对观测")
        self.thumb_quality_label.setWordWrap(True)
        side_layout.addWidget(self.thumb_quality_label)
        legend = QtWidgets.QLabel("* 无直接观测的远节 / DIP 为模型推算。接收 Hz 是主机接收统计，不是已验证的硬件采样率。界面目标刷新率约 30 Hz。")
        legend.setWordWrap(True)
        side_layout.addWidget(legend)
        splitter.addWidget(side)
        splitter.setStretchFactor(0, 4 if mode == "demo" else 3)
        splitter.setStretchFactor(1, 1 if mode == "demo" else 2)
        if mode == "demo":
            splitter.setSizes([1000, 440])
        outer.addWidget(splitter, 1)
        self.recording_controls = QtWidgets.QWidget()
        recording = QtWidgets.QHBoxLayout(self.recording_controls)
        recording.setContentsMargins(0, 0, 0, 0)
        self.record_button = QtWidgets.QPushButton("手动录制原始 CSV")
        self.record_button.clicked.connect(self._start_recording)
        self.stop_record_button = QtWidgets.QPushButton("停止录制")
        self.stop_record_button.clicked.connect(lambda: self._submit("record_stop"))
        recording.addWidget(self.record_button)
        recording.addWidget(self.stop_record_button)
        self.record_label = QtWidgets.QLabel("未录制；不自动保存")
        recording.addWidget(self.record_label, 1)
        outer.addWidget(self.recording_controls)
        self.recording_controls.setVisible(mode != "demo")
        self.diagnostic_controls = QtWidgets.QWidget()
        diagnostic = QtWidgets.QHBoxLayout(self.diagnostic_controls)
        diagnostic.setContentsMargins(0, 0, 0, 0)
        self.diagnostic_start_button = QtWidgets.QPushButton("开始诊断")
        self.diagnostic_start_button.clicked.connect(self._start_diagnostic)
        self.diagnostic_stop_button = QtWidgets.QPushButton("停止诊断")
        self.diagnostic_stop_button.clicked.connect(self._stop_diagnostic)
        self.diagnostic_marker_combo = QtWidgets.QComboBox()
        for label, value in (("rest · 静止", "rest"), ("moving · 运动", "moving"), ("returned · 已伸直", "returned")):
            self.diagnostic_marker_combo.addItem(label, value)
        self.diagnostic_marker_button = QtWidgets.QPushButton("记录标记")
        self.diagnostic_marker_button.clicked.connect(self._mark_diagnostic)
        for widget in (self.diagnostic_start_button, self.diagnostic_stop_button,
                       self.diagnostic_marker_combo, self.diagnostic_marker_button):
            diagnostic.addWidget(widget)
        self.diagnostic_label = QtWidgets.QLabel("诊断未开始；仅手动记录现场，不调整算法")
        self.diagnostic_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        diagnostic.addWidget(self.diagnostic_label, 1)
        outer.addWidget(self.diagnostic_controls)
        self.diagnostic_controls.setVisible(mode == "live")
        self.status_label = QtWidgets.QLabel("等待手动操作")
        self.status_label.setWordWrap(True)
        self.status_label.setMaximumHeight(50)
        self.warning_label = QtWidgets.QLabel()
        self.warning_label.setWordWrap(True)
        self.warning_label.setMaximumHeight(48)
        outer.addWidget(self.status_label)
        outer.addWidget(self.warning_label)
        if mode == "live":
            self._refresh_or_browse()
        if source:
            self.source_combo.setEditText(str(source))
        self.axis_combo.currentIndexChanged.connect(self._refresh_snapshot)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh_snapshot)
        self.timer.start(33)
        self.worker.start()
        self._refresh_snapshot()

    def _submit(self, command, **arguments):
        if command in {"cancel", "disconnect", "neutral", "load", "set_hand"}:
            self._cancel_thumb_preparation()
        runtime = self.worker.latest_snapshot().get("runtime", {})
        if self._closing or runtime.get("running") is False:
            self._ui_notice = "后台处理已停止，请关闭本窗口后重新启动。" + str(runtime.get("error") or "")
            self.status_label.setText(self._ui_notice[:300])
            self.status_label.setToolTip(self._ui_notice)
            return False
        self._ui_notice = ""
        if not self.worker.submit(command, **arguments):
            self._ui_notice = "操作队列已满，请稍候再试"
            self.status_label.setText(self._ui_notice)
            return False
        return True

    def _cancel_thumb_preparation(self, notice=""):
        self._thumb_prepare_timer.stop()
        self._thumb_preparation = None
        self._thumb_queued_at = None
        if notice:
            self._ui_notice = notice

    def _thumb_readiness_error(self, snapshot, target):
        """Recheck the current snapshot, not the state when the button lit up."""
        runtime, calibration = snapshot.get("runtime", {}), snapshot.get("calibration", {})
        if self._closing or self.mode == "demo" or runtime.get("running") is not True:
            return "后台未就绪，不能开始拇指标定。"
        if not runtime.get("connected") or runtime.get("eof"):
            return "数据源已断开或回放结束，不能开始拇指标定。"
        if calibration.get("active"):
            return "已有标定正在采集，请先完成或取消。"
        if not calibration.get("neutral_ready"):
            return "请先完成本次静止归零。"
        nodes = snapshot.get("nodes", {})
        missing = [str(node) for node in range(1, 13)
                   if nodes.get(str(node), nodes.get(node, {})).get("fresh") is not True]
        if missing:
            return "12 路数据尚未全部就绪，未更新节点：" + "、".join(missing)
        # A stuck worker can leave a formerly fresh snapshot in the mailbox.
        published = runtime.get("performance", {}).get("snapshot_published_monotonic_s")
        if published is not None:
            stale_after = self._view_config.get("stale_after_s", .5)
            if (isinstance(published, bool) or not isinstance(published, (int, float))
                    or not math.isfinite(published)
                    or not 0 <= time.monotonic() - published <= stale_after):
                return "后台快照已过期，等待恢复新数据后再开始。"
        if target == "thumb_hinge" and calibration.get("quality_feedback", {}).get("status") != "ready_for_b":
            return "请先完成本版拇指 A；A 成功后再手动开始 B。"
        return ""

    def _request_axis(self, target):
        if self._thumb_preparation is not None or self._thumb_queued_at is not None or self._closing:
            return
        if target not in {"thumb_common", "thumb_hinge"}:
            self._submit("axis", target=target)
            return
        reason = self._thumb_readiness_error(self.worker.latest_snapshot(), target)
        if reason:
            self._ui_notice = reason
            self._refresh_snapshot()
            return
        self.axis_combo.setCurrentIndex(self.axis_combo.findData(target))
        self._ui_notice = ""
        self._thumb_preparation = dict(target=target, deadline=time.monotonic() + 3.0)
        self._thumb_prepare_timer.start()
        self._refresh_snapshot()

    def _advance_thumb_preparation(self):
        preparation = self._thumb_preparation
        if preparation is None:
            return
        reason = self._thumb_readiness_error(self.worker.latest_snapshot(), preparation["target"])
        if reason:
            self._cancel_thumb_preparation("准备已取消：" + reason)
        elif time.monotonic() >= preparation["deadline"]:
            self._thumb_prepare_timer.stop()
            self._thumb_preparation = None
            # Keep a local in-flight guard until a later worker publication
            # acknowledges the request, even when Qt receives a double click.
            self._thumb_queued_at = time.monotonic()
            self._thumb_request_serial += 1
            if not self._submit("axis", target=preparation["target"], calibration_method="robust_v2",
                                _gui_request_id=self._thumb_request_serial):
                self._thumb_queued_at = None
        self._refresh_snapshot()

    def _refresh_thumb_capture(self, snapshot):
        calibration = snapshot.get("calibration", {})
        feedback = calibration.get("quality_feedback") or {}
        active = calibration.get("active") or {}
        if self._thumb_preparation is not None:
            preparation = self._thumb_preparation
            seconds = max(1, math.ceil(preparation["deadline"] - time.monotonic()))
            stage = "A" if preparation["target"] == "thumb_common" else "B"
            action = ("保持拇指形状不变，准备上下翻掌并左右转掌" if stage == "A"
                      else "用另一只手扶稳近节；采集开始后仅末节先弯再伸")
            text = f"拇指 {stage} 准备 {seconds} 秒 · 尚未采集标定动作：{action}。"
        elif self._thumb_queued_at is not None:
            text = "准备完成，正在请求 8 秒采集；等待后台确认，请保持准备姿势。"
        elif active.get("target") in {"thumb_common", "thumb_hinge"}:
            stage = "A" if active["target"] == "thumb_common" else "B"
            text = f"拇指 {stage} 采集中 · 剩余 {self._number(active.get('remaining_s'))} / 8 秒。"
            if feedback:
                text += " " + str(feedback.get("message", ""))
        elif feedback.get("status") == "ready_for_b":
            text = str(feedback.get("message", "A 成功；请手动开始 B，不会自动推进。"))
        else:
            text = ("拇指联合 A/B：结果及数值原因见下方标定状态；不会自动进入下一步。"
                    if calibration.get("message") else "拇指联合 A/B：先完成静止归零，再手动开始 A。")
        if feedback and self._thumb_preparation is None and self._thumb_queued_at is None:
            samples = feedback.get("paired_samples")
            if samples is not None:
                text += f" 配对样本：{samples}。"
            if "proximal_speed_p95_dps" in feedback or "distal_speed_p95_dps" in feedback:
                text += (f" 近节/末节速度 P95：{self._number(feedback.get('proximal_speed_p95_dps'))} / "
                         f"{self._number(feedback.get('distal_speed_p95_dps'))} °/s（仅提示，非最终判定）。")
        self.thumb_capture_label.setText(text)
        self.thumb_capture_label.setToolTip(text + ("\n" + "\n".join(f"{key}: {value}" for key, value in feedback.items()) if feedback else ""))
        self.thumb_capture_label.setStyleSheet("color: #b05a16;" if feedback.get("status") == "needs_attention" else "")

    def _hand_changed(self, *_args):
        self._submit("set_hand", hand=self.hand_combo.currentData(), confirmed=self.hand_check.isChecked())

    def _refresh_or_browse(self):
        if self.mode == "live":
            previous = self.source_combo.currentText()
            try:
                ports = list_serial_ports()
                self.source_combo.clear()
                for port in ports:
                    if isinstance(port, str):
                        device, description = port, ""
                    elif isinstance(port, (list, tuple)):
                        device, description = port[0], port[1] if len(port) > 1 else ""
                    elif isinstance(port, dict):
                        device, description = port.get("device", ""), port.get("description", "")
                    else:
                        device, description = port.device, getattr(port, "description", "")
                    self.source_combo.addItem(device)
                    self.source_combo.setItemData(self.source_combo.count() - 1, description, QtCore.Qt.ToolTipRole)
                self.source_combo.setEditText(previous)
            except Exception as exc:
                self._ui_notice = f"无法列出串口，可手动输入路径：{exc}"
                self.status_label.setText(self._ui_notice)
        elif self.mode == "replay":
            path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "选择原始 CSV", self.source_combo.currentText(), "CSV (*.csv)")
            if path:
                self.source_combo.setEditText(path)

    def _save_calibration(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存为新标定文件（不覆盖）", self._calibration_path or "hand_calibration.json", "JSON (*.json)")
        if path:
            if Path(path).exists():
                self._ui_notice = "文件已存在；请使用新的标定文件名，不会覆盖"
                self.status_label.setText(self._ui_notice)
                return
            self._submit("save", path=path)

    def _load_calibration(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "加载标定（加载后重新静止归零）", self._calibration_path or "", "JSON (*.json)")
        if path:
            self._submit("load", path=path)

    def _start_recording(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "新建原始 CSV（不覆盖）", "imu_raw.csv", "CSV (*.csv)")
        if path:
            if Path(path).exists():
                self._ui_notice = "CSV 已存在；请选择新文件名，不会覆盖"
                self.status_label.setText(self._ui_notice)
                return
            self._submit("record_start", path=path)

    def _diagnostic_actions(self, snapshot):
        runtime = snapshot.get("runtime", {})
        diagnostic = runtime.get("diagnostic") or {}
        state = diagnostic.get("state", "idle")
        running_live = self.mode == "live" and runtime.get("running") is True
        can_start = (running_live and bool(runtime.get("connected"))
                     and not runtime.get("eof") and not snapshot.get("calibration", {}).get("active")
                     and self._thumb_preparation is None and self._thumb_queued_at is None
                     and not diagnostic.get("writer_alive", False)
                     and state in {"idle", "complete", "failed"})
        return can_start, running_live and state == "recording"

    def _start_diagnostic(self):
        if not self._diagnostic_actions(self.worker.latest_snapshot())[0]:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "新建现场诊断 JSONL（不覆盖，不调整算法）", "hand_diagnostic.jsonl", "JSONL (*.jsonl)",
            options=QtWidgets.QFileDialog.DontConfirmOverwrite)
        if not path:
            return
        target = Path(path)
        if not target.suffix:
            target = target.with_suffix(".jsonl")
        if target.suffix.lower() != ".jsonl":
            self._ui_notice = "诊断文件必须使用 .jsonl 后缀；请选择新文件名"
            self.status_label.setText(self._ui_notice)
            return
        if target.exists() or target.is_symlink():
            self._ui_notice = "诊断文件已存在；请选择新文件名，不会覆盖"
            self.status_label.setText(self._ui_notice)
            return
        # A native file dialog can run while connection/calibration state changes.
        if self._diagnostic_actions(self.worker.latest_snapshot())[0]:
            self._submit("diagnostic_start", path=str(target))

    def _stop_diagnostic(self):
        if self._diagnostic_actions(self.worker.latest_snapshot())[1]:
            self._submit("diagnostic_stop")

    def _mark_diagnostic(self):
        if self._diagnostic_actions(self.worker.latest_snapshot())[1]:
            self._submit("diagnostic_marker", label=self.diagnostic_marker_combo.currentData())

    def _refresh_diagnostic(self, snapshot):
        can_start, recording = self._diagnostic_actions(snapshot)
        self.diagnostic_start_button.setEnabled(can_start)
        for widget in (self.diagnostic_stop_button, self.diagnostic_marker_combo, self.diagnostic_marker_button):
            widget.setEnabled(recording)
        diagnostic = snapshot.get("runtime", {}).get("diagnostic") or {}
        state = diagnostic.get("state", "idle")
        path, error = str(diagnostic.get("path") or ""), str(diagnostic.get("error") or "")
        label = {
            "idle": "诊断未开始；仅手动记录现场，不调整算法",
            "recording": "诊断记录中", "stopping": "诊断停止中：等待缓冲写完，尚未完成",
            "complete": "诊断已完成", "failed": "诊断失败；不停止采集",
        }.get(state, "诊断状态未知")
        details = label + (f"\n文件：{path}" if path else "") + (f"\n错误：{error}" if error else "")
        if error:
            label += f"：{error[:100]}"
        elif path:
            label += f" · {Path(path).name[:64]}"
        self.diagnostic_label.setText(label)
        self.diagnostic_label.setToolTip(details)
        self.diagnostic_label.setStyleSheet("color: #c0392b;" if state == "failed" or error else "")

    @staticmethod
    def _number(value, decimals=1):
        try:
            return f"{float(value):.{decimals}f}" if value is not None else "—"
        except (ValueError, TypeError):
            return "—"

    @staticmethod
    def _cell(table, row, column, text):
        item = table.item(row, column)
        if item is None:
            item = QtWidgets.QTableWidgetItem()
            table.setItem(row, column, item)
        if item.text() != str(text):
            item.setText(str(text))

    def _refresh_snapshot(self):
        snapshot = self.worker.latest_snapshot()
        runtime = snapshot.get("runtime", {})
        calibration = snapshot.get("calibration", {})
        active = calibration.get("active")
        if self._thumb_preparation is not None:
            reason = self._thumb_readiness_error(snapshot, self._thumb_preparation["target"])
            if reason:
                self._cancel_thumb_preparation("准备已取消：" + reason)
        elif self._thumb_queued_at is not None:
            if (active or not runtime.get("connected")
                    or runtime.get("running") is not True or runtime.get("eof")
                    or runtime.get("last_axis_request_id") == self._thumb_request_serial):
                self._thumb_queued_at = None
        busy = bool(active) or self._thumb_preparation is not None or self._thumb_queued_at is not None
        # A missing running field is the initial startup snapshot, not failure.
        stopped = runtime.get("running") is False
        connected = bool(runtime.get("connected")) and not stopped
        eof = bool(runtime.get("eof"))
        demo = self.mode == "demo"
        nodes = snapshot.get("nodes", {})
        any_fresh = any(node.get("fresh") for node in nodes.values())
        performance = runtime.get("performance", {})
        published = performance.get("snapshot_published_monotonic_s")
        self.canvas.set_model(snapshot.get("model", {}), mode=self.mode,
                              stale=stopped or (not demo and (not connected or eof or not any_fresh)),
                              published_monotonic_s=published)
        snapshot_age = max(0.0, (time.monotonic()-published)*1000) if published is not None else None
        diagnostic_text = (
            f"主机侧：缓存 {self._number(performance.get('serial_backlog_before_bytes'), 0)}→"
            f"{self._number(performance.get('serial_backlog_after_bytes'), 0)} B；"
            f"融合/标定 {self._number(performance.get('fusion_elapsed_ms'))} ms；"
            f"快照年龄 {self._number(snapshot_age)} ms；绘制 {self._number(self.canvas.paint_elapsed_ms)} ms。"
            "非传感器端到端延迟"
        )
        self.performance_label.setText(diagnostic_text)
        details = ["主机侧观测；没有设备采样时间戳，不能据此证明传感器到屏幕总延迟。",
                   "快照年龄不含串口等待、融合、模型计算，也不含当前待执行绘制。",
                   f"上次绘制含网格构建：{self.canvas.paint_elapsed_ms:.2f} ms；绘制三角面：{self.canvas.drawn_triangles}"]
        details.extend(f"{key}: {value}" for key, value in performance.items())
        self.performance_label.setToolTip("\n".join(details))
        self.connect_button.setEnabled(not demo and not connected)
        self.disconnect_button.setEnabled(not demo and connected)
        self.source_combo.setEnabled(not demo and not connected)
        self.refresh_button.setEnabled(not demo and not connected)
        self.hand_combo.setEnabled(not connected)
        self.hand_check.setEnabled(not connected)
        can_calibrate = not demo and connected and not eof and not busy
        self.neutral_button.setEnabled(can_calibrate)
        axis_ready = (self.axis_combo.currentData() not in {"thumb_common", "thumb_hinge"}
                      or not self._thumb_readiness_error(snapshot, self.axis_combo.currentData()))
        self.axis_button.setEnabled(can_calibrate and axis_ready)
        self.thumb_common_button.setEnabled(can_calibrate and not self._thumb_readiness_error(snapshot, "thumb_common"))
        self.thumb_hinge_button.setEnabled(can_calibrate and not self._thumb_readiness_error(snapshot, "thumb_hinge"))
        self.thumb_hinge_button.setToolTip("A 成功后才可开始；点击后先用 3 秒扶稳近节，看到采集中再屈伸末节。")
        self.cancel_button.setEnabled(not demo)
        self.load_button.setEnabled(can_calibrate)
        self.mount_check.setEnabled(not demo)
        self.axis_combo.setEnabled(not demo and not busy)
        self.record_button.setEnabled(self.mode == "live" and connected and not runtime.get("recording"))
        self.stop_record_button.setEnabled(bool(runtime.get("recording")))
        self.record_label.setText(f"录制中：{runtime['recording']}" if runtime.get("recording") else "未录制；不自动保存")
        self._refresh_diagnostic(snapshot)
        self.save_button.setEnabled(not demo and bool(calibration.get("neutral_ready")) and not busy)
        if stopped:
            for widget in (self.connect_button, self.disconnect_button, self.source_combo,
                           self.refresh_button, self.hand_combo, self.hand_check,
                           self.neutral_button, self.axis_combo, self.axis_button,
                           self.thumb_common_button, self.thumb_hinge_button,
                           self.cancel_button, self.save_button, self.load_button,
                           self.mount_check, self.record_button, self.stop_record_button):
                widget.setEnabled(False)
        cal_text = "模拟数据；不含真实标定" if demo else ("静止标定已完成" if calibration.get("neutral_ready") else "静止标定未完成")
        if active:
            cal_text += f" | {active.get('kind', '')} {active.get('target') or ''} 剩余 {self._number(active.get('remaining_s'))} 秒"
        cal_text += " | " + str(calibration.get("message", ""))
        self.calibration_label.setText(cal_text)
        self.calibration_label.setToolTip(cal_text)
        self._refresh_thumb_capture(snapshot)
        progress_text, progress_details = axis_progress(self._view_config, calibration, demo=demo)
        self.axes_progress_label.setText(progress_text)
        self.axes_progress_label.setToolTip(progress_details)
        message = self._ui_notice or runtime.get("error") or runtime.get("message", "")
        if stopped:
            message = "后台处理已停止，请关闭本窗口后重新启动。" + str(runtime.get("error") or runtime.get("message", ""))
        self.status_label.setText(message[:300])
        self.status_label.setToolTip(message)
        self.status_label.setStyleSheet("color: #c0392b;" if stopped or runtime.get("error") else "")
        warnings = list(snapshot.get("warnings", [])) + list(snapshot.get("model", {}).get("warnings", []))
        warning_text = "；".join(dict.fromkeys(str(warning) for warning in warnings))
        self.warning_label.setText(warning_text[:300] + ("…（悬停查看全部）" if len(warning_text) > 300 else ""))
        self.warning_label.setToolTip(warning_text)
        for row in range(12):
            node = nodes.get(str(row + 1), nodes.get(row + 1, {}))
            age = node.get("age_s")
            state = "模拟" if demo else "回放结束" if eof else "正常" if node.get("fresh") and connected else "未收/断流"
            values = [str(row + 1), state, self._number(age * 1000 if age is not None else None), self._number(node.get("rate_hz")), str(node.get("samples", 0))]
            for column, value in enumerate(values):
                self._cell(self.nodes_table, row, column, value)
        for row, (name, label) in enumerate(FINGER_LABELS.items()):
            finger = snapshot.get("model", {}).get("fingers", {}).get(name, {})
            valid = finger.get("valid", False)
            angle_keys = ["mcp_deg", "ip_deg" if name == "thumb" else "pip_deg", "dip_deg", "splay_deg"]
            values = [label] + [self._number(finger.get(key)) if valid or demo or finger.get(key.replace("_deg", "_valid"), False) else "—" for key in angle_keys]
            for column, value in enumerate(values):
                self._cell(self.angles_table, row, column, value)
        thumb = snapshot.get("model", {}).get("fingers", {}).get("thumb", {})
        estimator = thumb.get("ip_estimator")
        quality_labels = {
            "uninitialized": "等待零位/配对功能轴", "reference_inconsistent": "参考轴不一致，需要联合 A/B 标定",
            "tracking": "配对陀螺仪 + 重力纠偏", "prediction_only": "仅预测，重力暂不可用",
            "uncertain": "不确定，请检查观测", "reacquiring": "可信重力重新约束中",
            "observed_after_loss": "当前重力重新约束，之前丢失轨迹不可恢复",
        }
        if estimator is not None:
            label = quality_labels.get(estimator.get("quality"), str(estimator.get("quality")))
            self.thumb_quality_label.setText("拇指抗漂移：" + label)
            self.thumb_quality_label.setToolTip("；".join(map(str, estimator.get("reasons", []))) +
                                               "。仅针对两段相对屈曲，不保证绝对航向/完整对掌模型。")
            self.thumb_quality_label.setStyleSheet("color: #b05a16;" if estimator.get("uncertain", True) else "")
        else:
            self.thumb_quality_label.setText("拇指：模拟姿态" if demo else "拇指：历史四元数算法")
            self.thumb_quality_label.setToolTip("")
            self.thumb_quality_label.setStyleSheet("")

    def closeEvent(self, event):
        self._closing = True
        self._cancel_thumb_preparation()
        self.timer.stop()
        self.worker.stop()
        if not self.worker.wait(1500):
            self.status_label.setText("正在等待数据源关闭，请稍候；不会强制终止线程")
            event.ignore()
            QtCore.QTimer.singleShot(200, self.close)
            return
        event.accept()


def launch_gui(config: dict, *, mode: str = "live", source: str | None = None,
               calibration_path: str | None = None, smoke_seconds: float | None = None) -> int:
    """Show a manual-control window; a supplied port never opens automatically."""
    if mode not in {"live", "replay", "demo"}:
        raise ValueError("mode must be live, replay, or demo")
    if smoke_seconds is not None and not (0 < smoke_seconds <= 3600):
        raise ValueError("smoke_seconds must be positive and no greater than 3600")
    existing = QtWidgets.QApplication.instance()
    app = existing or QtWidgets.QApplication(["vt-glove-pose"])
    window = HandPoseWindow(config, mode=mode, source=source, calibration_path=calibration_path)
    window.setAttribute(QtCore.Qt.WA_DeleteOnClose)
    window.show()
    if smoke_seconds is not None:
        QtCore.QTimer.singleShot(max(1, round(smoke_seconds * 1000)), window.close)
    if existing is None:
        return app.exec_()
    loop = QtCore.QEventLoop()
    window.destroyed.connect(loop.quit)
    loop.exec_()
    return 0
