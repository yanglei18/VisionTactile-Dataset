"""Explicit-connect live sensor preview, independent of hinge calibration.

Only the worker owns the serial port. Every received sample updates VQF; the
desktop consumes bounded, latest snapshots instead of queuing raw samples.
This does not estimate anatomical joint angles or save a calibration.
"""
from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
from pathlib import Path
import queue
import threading
import time

from PyQt5 import QtCore, QtWidgets

from .config import gyro_scale, load_config
from .stream import SerialSource
from .thumb_orientation import ThumbOrientationProcessor
from .thumb_preview import snapshot_row


def connection_status(state):
    """Describe connection, incoming data and display reference separately.

    This is presentation only: no serial command, reference reset, or state
    mutation occurs here. Historical notices must not hide a missing reference.
    """
    snapshot = state.get("snapshot", {})
    nodes = snapshot.get("node_ids", [5, 6])
    node_label = "/".join(map(str, nodes)) + " 号"
    if not state.get("connected", False):
        mode = "disconnected"
        status = "串口未连接。请选择手套串口后点击“连接”。"
        empty = "手套串口尚未连接\n请选择串口并点击“连接”"
    elif not snapshot.get("paired_fresh", False):
        mode = "waiting_for_data"
        status = f"串口已连接，正在等待 {node_label} 的新鲜配对数据。"
        empty = f"串口已连接\n正在等待 {node_label} 的新鲜数据"
    elif not snapshot.get("reference_ready", False):
        mode = "reference_required"
        missing = "display_reference_not_set" in snapshot.get("reasons", [])
        reason = "尚未设置显示参考" if missing else "显示参考已失效"
        status = (f"串口已连接，{node_label} 数据正常；{reason}。"
                  "请点击“以当前姿态为显示参考”，无需重新连接。")
        empty = (f"{node_label} 实时数据正常\n{reason}\n"
                 "请点击“以当前姿态为显示参考”")
    else:
        mode = "streaming"
        status = f"串口已连接，{node_label} 实时数据正常；正在显示两路独立朝向。"
        empty = "等待有效朝向画面"
        notice = state.get("notice")
        if notice:
            status += " " + str(notice)
    error = state.get("error")
    if error:
        status += "\n错误：" + str(error)
    return dict(mode=mode, status=status, empty_message=empty)


class OrientationWorker(QtCore.QThread):
    def __init__(self, config, *, source_factory=SerialSource, clock=time.monotonic, angle_candidate=None):
        super().__init__()
        self.config = copy.deepcopy(config)
        self.source_factory, self.clock = source_factory, clock
        self.processor = ThumbOrientationProcessor(config)
        self.source = None
        self.commands = queue.Queue(maxsize=16)
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.latest = {}
        self.history = deque(maxlen=900)
        self.started_at = None
        self.notice = "尚未连接。连接后可直接观察两个 IMU；显示参考不需要转轴标定。"
        self.error = ""
        self.trial = None
        self.trial_future = None
        self.last_trial = None
        self.save_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="thumb-trial-save")
        self.angle_candidate_profile = copy.deepcopy(angle_candidate)
        self.angle_processor = None
        self.angle_error = ""
        self.angle_history = deque(maxlen=3600)
        self._last_angle_pair_count = None
        self._angle_started_at = None
        self._reset_angle_candidate()

    def _reset_angle_candidate(self):
        """A candidate failure cannot prevent the original orientation stream."""
        self.angle_processor = None
        self.angle_error = ""
        self.angle_history.clear()
        self._last_angle_pair_count = None
        self._angle_started_at = None
        if self.angle_candidate_profile is not None:
            try:
                from .thumb_angle_stream import CandidateAngleProcessor
                self.angle_processor = CandidateAngleProcessor(copy.deepcopy(self.angle_candidate_profile), config=self.config)
            except Exception as exc:
                self.angle_error = str(exc)

    def _feed_angle_sample(self, sample):
        if self.angle_processor is None:
            return
        try:
            snapshot = self.angle_processor.process(sample)
            if snapshot is None:
                return
            count, stamp = snapshot["pair_count"], snapshot["host_monotonic_s"]
            if type(count) is not int or count < 1 or not isinstance(stamp, (float, int)) or not math.isfinite(stamp):
                raise ValueError("候选输出缺少有效配对计数或时间")
            if self._last_angle_pair_count is not None and count <= self._last_angle_pair_count:
                return
            if self._angle_started_at is None:
                self._angle_started_at = float(stamp)
            self._last_angle_pair_count = count
            row = {key: snapshot.get(key) for key in
                   ("angle_deg", "current_angle_observed", "paired_fresh", "history_loss", "pair_count", "quality")}
            row["time_s"] = float(stamp)-self._angle_started_at
            self.angle_history.append(row)
            return snapshot
        except Exception as exc:
            self.angle_error = str(exc)
            self.angle_processor = None

    def _angle_snapshot(self, now):
        if self.angle_candidate_profile is None:
            return None
        if self.angle_processor is not None:
            try:
                snapshot = self.angle_processor.snapshot(now=now)
                if self.source is not None:
                    return snapshot
            except Exception as exc:
                self.angle_error = str(exc)
                self.angle_processor = None
        return dict(angle_deg=None, paired_fresh=False, current_angle_observed=False,
                    quality="candidate_error" if self.angle_error else "disconnected",
                    history_loss=bool(self.angle_error), pair_count=self._last_angle_pair_count or 0,
                    node_ids=(self.angle_candidate_profile.get("node_ids", [5, 6])
                              if isinstance(self.angle_candidate_profile, dict) else [5, 6]),
                    reasons=[self.angle_error] if self.angle_error else ["not_connected"],
                    certified_calibration=False, anatomical_accuracy_verified=False)

    def submit(self, name, **kwargs):
        try:
            self.commands.put_nowait((name, kwargs))
            return True
        except queue.Full:
            return False

    def stop(self):
        self.stopping.set()

    def latest_snapshot(self):
        with self.lock:
            # Angle history contains only immutable scalar fields. Deep-copying
            # all 3600 dictionaries on every redraw needlessly eats the 33 ms
            # display budget; independent row copies preserve caller isolation.
            result = copy.deepcopy({k: v for k, v in self.latest.items() if k != "angle_history"})
            result["angle_history"] = [row.copy() for row in self.latest.get("angle_history", [])]
            return result

    def disconnect(self):
        self.finish_trial("disconnected")
        source, self.source = self.source, None
        try:
            if source is not None:
                source.close()
        except Exception as exc:
            self.error = f"释放串口时出现错误：{exc}；已采集测试仍会尝试保存。"
        finally:
            self.processor = ThumbOrientationProcessor(self.config)
            self.history.clear()
            self.started_at = None
            self._reset_angle_candidate()

    def finish_trial(self, reason="completed"):
        if self.trial is None:
            return
        trial, self.trial = self.trial, None
        now = self.clock()
        self.last_trial = dict(status="saving", directory=str(trial.directory),
                               reason=reason, summary="正在保存原始数据和测试结果；实时预览继续。")
        self.trial_future = self.save_pool.submit(trial.finish, now, reason=reason)

    def poll_trial_save(self):
        if self.trial_future is None or not self.trial_future.done():
            return
        future, self.trial_future = self.trial_future, None
        try:
            self.last_trial = future.result()
            report = self.last_trial.get("report") or {}
            self.last_trial["summary"] = report.get("summary", "已保存本次测试记录。")
            if self.last_trial.get("export_complete") is not True or self.last_trial.get("errors"):
                self.last_trial["status"] = "save_failed"
                self.last_trial["summary"] = "保存未完整完成；已写文件保留，请勿删除该目录。"
                self.error = "测试保存未完成：" + "; ".join(map(str, self.last_trial.get("errors", [])))
            else:
                self.notice = "测试数据已保存；请保留当前硬件连接，等待分析。"
        except Exception as exc:
            self.last_trial = dict(self.last_trial or {}, status="save_failed", error=str(exc),
                                   summary="测试保存未完成，已写入的文件保留；不要删除该目录。")
            self.error = f"测试保存未完成：{exc}"

    def feed_samples(self, samples):
        if self.trial is not None:
            try:
                self.trial.feed_raw(samples)
            except Exception as exc:
                self.error = f"原始记录已停止：{exc}"
                self.finish_trial("recording_error")
        for sample in samples:
            snapshot = self.processor.process(sample)
            angle_snapshot = self._feed_angle_sample(sample)
            if snapshot is not None and self.trial is not None:
                try:
                    if self.angle_candidate_profile is None:
                        self.trial.feed_snapshot(snapshot)
                    else:
                        self.trial.feed_snapshot(snapshot, angle_snapshot=angle_snapshot)
                except Exception as exc:
                    self.error = f"在线记录已停止：{exc}"
                    self.finish_trial("recording_error")

    def command(self, name, args):
        if name == "connect":
            if self.source is not None:
                raise ValueError("请先断开当前连接")
            source = self.source_factory(args["port"])
            try:
                source.open()
            except Exception:
                source.close()
                raise
            self.source = source
            self.processor = ThumbOrientationProcessor(self.config)
            self.history.clear()
            self.started_at = self.clock()
            self._reset_angle_candidate()
            self._angle_started_at = self.started_at
            self.notice = "已连接；收到数据后点“以当前姿态为显示参考”。不要求功能轴标定。"
        elif name == "disconnect":
            self.disconnect()
            self.notice = "已断开，串口已释放。"
        elif name == "reference":
            if self.trial is not None:
                raise ValueError("测试期间保持同一显示参考；如需调整，请先取消并保存本次测试")
            if self.source is None:
                raise ValueError("请先连接手套")
            if not self.processor.snapshot(now=self.clock()).get("paired_fresh"):
                raise ValueError("尚无新鲜配对数据，不能设置显示参考")
            self.processor.set_reference()
            self.history.clear()
            self.started_at = self.clock()
            self.notice = "显示参考已设置。两块 IMU 独立显示；不是关节角，也未执行零偏标定。"
            if self.angle_candidate_profile is not None:
                self.notice += " 弯曲候选使用独立参考，未归零或重置。"
        elif name == "start_trial":
            if self.trial is not None or self.trial_future is not None:
                raise ValueError("当前测试或保存尚未结束")
            snapshot = self.processor.snapshot(now=self.clock())
            if self.source is None or not snapshot.get("valid"):
                raise ValueError("请先连接并设置有效的显示参考")
            from .thumb_trial import ThumbTrial
            trial_options = ({} if self.angle_candidate_profile is None else
                             {"angle_candidate": self.angle_candidate_profile})
            self.trial = ThumbTrial(self.config, args["output_root"], self.clock(), snapshot, **trial_options)
            self.last_trial = None
            self.notice = "测试已开始；看大字提示操作，不需要手抄读数。"
        elif name == "cancel_trial":
            self.finish_trial("operator_cancelled")
            self.notice = "本次测试已取消，正在保留已采集的数据。"
        else:
            raise ValueError("未知预览命令")
        self.error = ""

    def publish(self, now):
        self.poll_trial_save()
        snapshot = self.processor.snapshot(now=now)
        if self.trial is not None:
            if not snapshot.get("paired_fresh"):
                self.finish_trial("data_gap")
            elif not snapshot.get("reference_ready"):
                self.finish_trial("reference_invalidated")
            elif self.trial.due(now):
                self.finish_trial("completed")
        current = None
        if snapshot.get("reference_ready") and snapshot.get("paired_fresh"):
            current = snapshot_row(snapshot, max(0.0, now - self.started_at))
            self.history.append(current)
        payload = dict(connected=self.source is not None, snapshot=snapshot,
                       current=current, history=list(self.history),
                       notice=self.notice, error=self.error,
                       trial=self.trial.status(now) if self.trial else None,
                       saving=self.trial_future is not None, last_trial=self.last_trial,
                       angle_candidate=self._angle_snapshot(now), angle_history=list(self.angle_history))
        with self.lock:
            self.latest = payload

    def run(self):
        deadline = 0.0
        try:
            while not self.stopping.is_set():
                for _ in range(16):
                    try:
                        name, args = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self.command(name, args)
                    except Exception as exc:
                        self.error = str(exc)
                try:
                    if self.source is not None:
                        self.feed_samples(self.source.read())
                    else:
                        self.stopping.wait(.02)
                except Exception as exc:
                    self.error = str(exc)
                    self.disconnect()
                    self.notice = "读取已停止；检查设备后重新连接。"
                now = self.clock()
                if now >= deadline:
                    self.publish(now)
                    deadline = now + 1.0/30
        finally:
            self.disconnect()
            self.save_pool.shutdown(wait=True)
            self.poll_trial_save()
            self.publish(self.clock())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--port", default="", help="Prefill only; never connect automatically")
    parser.add_argument("--angle-candidate", type=Path,
                        help="Explicit experimental scalar-angle candidate JSON; never a certified calibration")
    parser.add_argument("--output-root", type=Path,
                        default=Path.home()/"visiontactile-data"/"glove_trials",
                        help="Guided test destination; created only when you click Start")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    candidate = None
    if args.angle_candidate is not None:
        try:
            if not args.angle_candidate.is_file() or args.angle_candidate.stat().st_size > 4*1024*1024:
                raise ValueError("angle candidate JSON must be a file no larger than 4 MiB")
            with args.angle_candidate.open(encoding="utf-8") as stream:
                candidate = json.load(stream)
            from .thumb_angle_stream import CandidateAngleProcessor, validate_candidate
            validate_candidate(candidate)
            # Check the explicit acquisition configuration before creating UI.
            CandidateAngleProcessor(candidate, config=config)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            parser.error(str(exc))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = LiveOrientationWindow(config, port=args.port, output_root=args.output_root, angle_candidate=candidate)
    window.show()
    return app.exec_()


class LiveOrientationWindow(QtWidgets.QMainWindow):
    def __init__(self, config, port="", *, worker=None, output_root=None, angle_candidate=None):
        super().__init__()
        # Imported lazily so the worker can be tested without renderer state.
        from .thumb_preview_gui import OrientationPairWidget, RotationPlotWidget
        self.worker = worker or OrientationWorker(config, angle_candidate=angle_candidate)
        candidate_enabled = angle_candidate is not None or getattr(self.worker, "angle_candidate_profile", None) is not None
        self.output_root = Path(output_root) if output_root is not None else Path.home()/"visiontactile-data"/"glove_trials"
        self._trial_phase = None
        self.setWindowTitle("拇指实时弯曲（实验） — 单串口在线预览" if candidate_enabled else
                            "拇指实时朝向预览 — 无需转轴标定")
        self.resize(1120, 820)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        explanation = QtWidgets.QLabel(
            "拇指实时弯曲（实验） · 不是已认证的解剖关节角\n"
            "连接串口后直接观察弯曲页，不需要重新采集转轴或设置显示参考。\n"
            "如需独立朝向画面或 31 秒记录，再设置显示参考；此操作不会归零弯曲候选。"
            if candidate_enabled else
            "实时 IMU 朝向 · 不是解剖关节角\n"
            "1. 选择并连接串口　2. 设置当前显示参考　3. 自然移动观察。无需反复采集转轴。\n"
            "每个传感器使用独立参考；没有共同绝对航向，六轴航向仍可能漂移。")
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        nodes = [config["fingers"]["thumb"][k] for k in ("proximal", "distal")]
        layout.addWidget(QtWidgets.QLabel("有效角速度比例：" + "；".join(
            f"{n} 号 {gyro_scale(config,n):g} °/s/LSB" for n in nodes) + "（按配置，未读取固件量程）"))
        controls = QtWidgets.QHBoxLayout()
        self.port = QtWidgets.QComboBox()
        self.port.setEditable(True)
        self.port.setMinimumWidth(260)
        self.port.setEditText(port)
        controls.addWidget(self.port)
        refresh = QtWidgets.QPushButton("列出串口")
        refresh.clicked.connect(self.list_ports)
        controls.addWidget(refresh)
        self.connect_button = QtWidgets.QPushButton("连接")
        self.connect_button.clicked.connect(self.toggle_connection)
        controls.addWidget(self.connect_button)
        self.reference_button = QtWidgets.QPushButton("以当前姿态为显示参考")
        self.reference_button.clicked.connect(lambda: self.worker.submit("reference"))
        self.reference_button.setEnabled(False)
        controls.addWidget(self.reference_button)
        layout.addLayout(controls)
        self.status = QtWidgets.QLabel("尚未连接；本窗口不会自动占用串口。")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        test_controls = QtWidgets.QHBoxLayout()
        self.start_trial_button = QtWidgets.QPushButton("开始 31 秒引导测试（自动保存）")
        self.start_trial_button.clicked.connect(lambda: self.worker.submit("start_trial", output_root=str(self.output_root)))
        self.start_trial_button.setEnabled(False)
        test_controls.addWidget(self.start_trial_button)
        self.cancel_trial_button = QtWidgets.QPushButton("取消测试并保存已有数据")
        self.cancel_trial_button.clicked.connect(lambda: self.worker.submit("cancel_trial"))
        self.cancel_trial_button.setEnabled(False)
        test_controls.addWidget(self.cancel_trial_button)
        layout.addLayout(test_controls)
        self.guide = QtWidgets.QLabel("准备好后点击引导测试：放稳 → 慢弯伸 3 次 → 回到原姿态 → 保持。")
        self.guide.setWordWrap(True)
        self.guide.setStyleSheet("font-size: 18px; font-weight: 600; padding: 8px; background: #eaf4f2; color: #174e43;")
        layout.addWidget(self.guide)
        self.guide_progress = QtWidgets.QProgressBar()
        self.guide_progress.setRange(0, 31)
        self.guide_progress.setValue(0)
        layout.addWidget(self.guide_progress)
        self.trial_result = QtWidgets.QLabel(f"仅点击开始后才录制；保存目录：{self.output_root}")
        self.trial_result.setWordWrap(True)
        self.trial_result.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.trial_result)
        self.orientation = OrientationPairWidget()
        self.orientation.set_node_ids(*nodes)
        self.plot = RotationPlotWidget()
        self.angle_widget = None
        if candidate_enabled:
            from .thumb_angle_widget import ThumbAngleWidget
            self.tabs = QtWidgets.QTabWidget()
            orientation_page = QtWidgets.QWidget()
            orientation_layout = QtWidgets.QVBoxLayout(orientation_page)
            orientation_layout.addWidget(self.orientation, 3)
            orientation_layout.addWidget(self.plot, 2)
            self.tabs.addTab(orientation_page, "独立朝向")
            self.angle_widget = ThumbAngleWidget()
            self.tabs.addTab(self.angle_widget, "拇指弯曲（实验）")
            self.tabs.setCurrentWidget(self.angle_widget)
            layout.addWidget(self.tabs, 1)
            self.reference_button.setToolTip("只调整独立朝向显示；不会归零弯曲候选角或重置观察器")
        else:
            layout.addWidget(self.orientation, 3)
            layout.addWidget(self.plot, 2)
        self.connected = False
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(33)
        self.worker.start()

    def list_ports(self):
        from serial.tools import list_ports
        previous = self.port.currentText()
        self.port.clear()
        for item in list_ports.comports():
            self.port.addItem(item.device)
            self.port.setItemData(self.port.count()-1,
                f"{item.description} · serial={item.serial_number}", QtCore.Qt.ToolTipRole)
        self.port.setEditText(previous)

    def toggle_connection(self):
        self.worker.submit("disconnect" if self.connected else "connect", port=self.port.currentText().strip())

    def refresh(self):
        state = self.worker.latest_snapshot()
        self.connected = state.get("connected", False)
        self.connect_button.setText("断开" if self.connected else "连接")
        snapshot = state.get("snapshot", {})
        trial = state.get("trial")
        saving = state.get("saving", False)
        self.reference_button.setEnabled(self.connected and snapshot.get("paired_fresh", False) and not trial)
        self.start_trial_button.setEnabled(self.connected and snapshot.get("valid", False) and not trial and not saving)
        self.cancel_trial_button.setEnabled(bool(trial))
        self.port.setEnabled(not self.connected)
        display_state = connection_status(state)
        angle_state = state.get("angle_candidate") or {}
        if (self.angle_widget is not None and display_state["mode"] == "reference_required"
                and angle_state.get("paired_fresh") and isinstance(angle_state.get("angle_deg"), (int, float))
                and math.isfinite(angle_state["angle_deg"])):
            display_state["status"] = ("弯曲候选已有输出；独立朝向参考尚未设置或已失效（不影响弯曲页）。"
                                       "如需独立朝向画面或引导记录，再设置显示参考。")
            if state.get("error"):
                display_state["status"] += "\n错误："+str(state["error"])
        self.status.setText(display_state["status"])
        if trial:
            phase = trial.get("phase")
            if self._trial_phase is not None and phase != self._trial_phase:
                QtWidgets.QApplication.beep()
            self._trial_phase = phase
            self.guide.setText(f"{trial['label']} · 还剩 {max(0, int(trial['remaining_s']+.999))} 秒\n{trial['instruction']}")
            self.guide_progress.setValue(max(0, min(31, int(31-trial['total_remaining_s']))))
            self.trial_result.setText(f"正在保存原始数据：{trial['directory']}")
        elif saving:
            self._trial_phase = None
            self.guide.setText("本次采集已结束，正在生成结果。无需重复动作。")
        elif state.get("last_trial"):
            self._trial_phase = None
            result = state['last_trial']
            finished = result.get("status") == "completed" and result.get("export_complete") is True
            self.guide.setText("测试已完成并保存。请保持硬件连接，等待分析。" if finished else "本次测试已停止；请查看保存状态，无需立即重做。")
            if finished:
                self.guide_progress.setValue(31)
            self.trial_result.setText(str(result.get("summary", "数据已保留")) + "\n" + str(result.get("directory", "")))
        current = state.get("current")
        self.orientation.set_empty_message(display_state["empty_message"])
        self.orientation.set_sample(current)
        self.plot.set_series(state.get("history", []), [])
        if current:
            self.plot.set_time(current["time_s"])
        if self.angle_widget is not None:
            self.angle_widget.set_state(state.get("angle_candidate"), state.get("angle_history", []))

    def closeEvent(self, event):
        self.timer.stop()
        self.worker.stop()
        if self.worker.wait(2000):
            event.accept()
        else:
            self.status.setText("正在释放串口，请稍候再关闭。")
            event.ignore()
            self.timer.start(33)


if __name__ == "__main__":
    raise SystemExit(main())
