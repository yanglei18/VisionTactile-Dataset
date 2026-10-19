"""Single-owner serial worker for the experimental local-joint GUI."""
from __future__ import annotations

from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import math
import queue
import threading
import time

from PyQt5 import QtCore

from .hinge_export import export_hinge_session
from .hinge_replay import calibrate_hinge_pair
from .hinge_session import HingeSession
from .stream import SerialSource


class HingeWorker(QtCore.QThread):
    """Qt receives bounded latest snapshots, not an ever-growing sample queue.

    No serial operations before an explicit connect command. Numerical fitting
    and manual disk exports run outside the serial loop; session mutation is
    strictly confined to this worker. Cancelled fit results are token-checked.
    """

    def __init__(self, config, proximal=5, distal=6, *, source_factory=SerialSource,
                 session_factory=HingeSession, fit_function=calibrate_hinge_pair,
                 export_function=export_hinge_session, clock=time.monotonic):
        super().__init__()
        self.config = copy.deepcopy(config)
        self.proximal, self.distal = proximal, distal
        self.session = session_factory(config, proximal, distal)
        self._source_factory, self._fit_function = source_factory, fit_function
        self._export_function, self._clock = export_function, clock
        self._source = None
        self._commands = queue.Queue(maxsize=32)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = {}
        self._future = None
        self._job_kind = None
        self._job_token = None
        self._pool = None
        self._error = ""
        self._notice = "尚未连接；点击连接才会打开串口"
        self._last_export = None
        self._validation = None
        self._validation_attempts = []
        self._validation_for_current_candidate = False
        self._validation_samples = 0
        self._receive_times = deque(maxlen=4000)
        self._raw_count = 0
        self._export_revision = -1
        self._revision = 0

    def submit(self, command, **args):
        if self._stop.is_set():
            return False
        try:
            self._commands.put_nowait((command, args))
            return True
        except queue.Full:
            return False

    def latest_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._snapshot)

    def stop(self):
        self._stop.set()

    def _disconnect(self):
        source, self._source = self._source, None
        try:
            if source is not None:
                source.close()
        finally:
            self.session.disconnect()
            self._finish_validation("cancelled", "disconnect")
            self._validation_for_current_candidate = False

    def _finish_validation(self, status, reason):
        if self._validation is not None:
            self._advance_marker(self._clock())
            for marker in self._validation["markers"]:
                if marker["status"] in {"recording", "scheduled"}:
                    marker.update(status="interrupted", reason=reason, observed_end_s=self._clock())
            self._validation.update(status=status, reason=reason, ended_s=self._clock())
            self._validation_attempts.append(self._validation)
            self._validation_for_current_candidate = True
            self._validation = None
            self._revision += 1

    def _advance_marker(self, now):
        if self._validation is not None:
            for marker in self._validation["markers"]:
                if marker["status"] == "scheduled" and now >= marker["start_host_monotonic_s"]-1.:
                    marker["status"] = "recording"
                if marker["status"] == "recording" and now >= marker["end_host_monotonic_s"]:
                    marker.update(status="completed", observed_end_s=now)

    def _mark_validation(self, kind, now):
        if self._validation is None:
            raise ValueError("请先开始独立验证录制")
        if self._validation.get("guided_quickcheck"):
            raise ValueError("30 秒试跑按向导安排动作；手动标记请使用高级验证录制")
        durations = dict(reference=2., return_pose=2., hold=60., common_rotation=10.)
        if kind not in durations:
            raise ValueError("unknown validation marker")
        self._advance_marker(now)
        if any(marker["status"] == "recording" for marker in self._validation["markers"]):
            raise ValueError("当前标记仍在采集，请完成后再标下一段")
        if len(self._validation["markers"]) >= 256:
            raise ValueError("本次动作标记已满，请导出后开始新实验")
        start, end = now+1., now+1.+durations[kind]
        if end > self._validation["started_s"]+self._validation["duration_s"]:
            raise ValueError("剩余录制时间不足以完成该标记；可结束并开始更长的验证录制")
        self._validation["markers"].append(dict(
            kind="return" if kind == "return_pose" else kind,
            pose_id="reference_pose" if kind in {"reference", "return_pose"} else "held_pose",
            window_id=len(self._validation["markers"])+1, status="recording",
            requested_host_monotonic_s=now, start_host_monotonic_s=start, end_host_monotonic_s=end,
            label_source="operator_declared_pose_not_sensor_detected_truth"))
        self._revision += 1

    def _command(self, command, args):
        now = self._clock()
        if command == "connect":
            if self._source is not None:
                raise ValueError("已连接；请先断开")
            port = str(args.get("port", "")).strip()
            if not port:
                raise ValueError("请填写明确的串口路径")
            source = self._source_factory(port)
            try:
                source.open()
                self.session.connect(now)
            except Exception:
                source.close()
                raise
            self._source = source
            self._validation_for_current_candidate = False
            self._receive_times.clear()
            self._notice = "已连接；请先采集参考姿势"
        elif command == "disconnect":
            self._disconnect()
            self._notice = "已断开，原始窗口仍在内存中；可导出后关闭"
        elif command in {"neutral", "motion", "direction"}:
            if self._validation is not None:
                raise ValueError("请先结束验证录制")
            if command in {"neutral", "motion"} and self._future is not None:
                raise ValueError("后台拟合／导出尚未完成，请稍候")
            getattr(self.session, "start_" + command)(now)
            if command in {"neutral", "motion"}:
                self._validation_for_current_candidate = False
            self._revision += 1
        elif command == "cancel":
            self.session.cancel()
            self._finish_validation("cancelled", "operator_cancelled")
            self._validation_for_current_candidate = False
            self._notice = "已取消本次动作；已有数据保留，不强行通过标定"
            self._revision += 1
        elif command == "validation":
            if self._validation is not None:
                self._finish_validation("partial", "operator_stopped_early")
            else:
                state = self.session.snapshot(now)
                if state.get("state") != "candidate_ready" or not state.get("preview_valid"):
                    raise ValueError("请先获得有效候选预览，再单独录制验证动作")
                if self._validation_samples >= 180000 or len(self._validation_attempts) >= 256:
                    raise ValueError("本实验验证缓存已满，请导出并新开实验窗口")
                duration = args.get("duration_s", 30.)
                if isinstance(duration, bool) or not isinstance(duration, (float, int)) or not math.isfinite(duration) or not 5. <= duration <= 180.:
                    raise ValueError("验证时长必须为 5 至 180 秒")
                guided = args.get("guided_quickcheck", False)
                if type(guided) is not bool or (guided and duration != 30.):
                    raise ValueError("引导试跑固定为 30 秒；完整验证请使用高级操作")
                self._validation = dict(kind="validation", started_s=now, ended_s=None,
                                        duration_s=float(duration), status="recording", reason="",
                                        samples=[], series=[], markers=[], online=[], online_discarded_rows=0,
                                        guided_quickcheck=guided,
                                        **self.session.validation_context())
                if guided:
                    # These are timed instructions, NOT detection that the
                    # user reached either pose. They never reset the estimator.
                    for index, (kind, start, end) in enumerate((("reference", 3., 5.), ("return", 28., 30.)), 1):
                        self._validation["markers"].append(dict(kind=kind, pose_id="quickcheck_reference_pose",
                            window_id=index, status="scheduled", requested_host_monotonic_s=now,
                            start_host_monotonic_s=now+start, end_host_monotonic_s=now+end,
                            label_source="timed_guidance_not_detected_pose_or_independent_truth"))
                self._notice = f"独立验证录制 {duration:g} 秒：可标记参考／回位／保持／转掌；不会重设零位"
                self._revision += 1
        elif command == "marker":
            self._mark_validation(args.get("kind"), now)
        elif command == "cancel_marker":
            if self._validation is not None:
                self._advance_marker(now)
                for marker in self._validation["markers"]:
                    if marker["status"] == "recording":
                        marker.update(status="cancelled", reason="operator_cancelled_marker", observed_end_s=now)
                        self._revision += 1
        elif command == "export":
            if self._future is not None:
                raise ValueError("后台任务尚未完成，请稍候导出")
            state = self.session.snapshot(now)["state"]
            if self._validation is not None or state.startswith(("prepare_", "capturing_")):
                raise ValueError("请先结束或取消当前采集，再导出完整窗口")
            bundle = self.session.export_bundle()
            # Sample is frozen. Copy list containers, not every immutable object,
            # to avoid a long stop in the serial owner during manual export.
            # Completed validation rows are append-only snapshots and are never
            # mutated after closing an attempt. Only copy their list container;
            # duplicating 180 s of nested preview dictionaries stalls acquisition.
            validation = [dict(**{key: copy.deepcopy(value) for key, value in attempt.items()
                                 if key not in {"samples", "series", "online"}}, samples=list(attempt["samples"]),
                               series=list(attempt["series"]), online=list(attempt["online"]))
                          for attempt in self._validation_attempts]
            self._future = self._pool.submit(self._export_function, args["directory"], bundle,
                                             config=copy.deepcopy(self.config),
                                             validation_attempts=validation)
            self._job_kind = "export"
            self._job_token = self._revision
            self._notice = "正在后台导出；串口读取继续"
        else:
            raise ValueError(f"Unknown hinge command: {command}")

    def _jobs(self):
        if self._future is not None and self._future.done():
            job, kind, token = self._future, self._job_kind, self._job_token
            self._future, self._job_kind, self._job_token = None, None, None
            try:
                result = job.result()
                if kind == "fit":
                    self.session.accept_fit(token, result)
                    self._revision += 1
                else:
                    self._last_export = result
                    self._export_revision = token
                    self._notice = f"实验已导出：{result}"
            except Exception as exc:
                if kind == "fit":
                    self.session.reject_fit(token, str(exc))
                self._error = f"后台{kind}失败：{type(exc).__name__}: {exc}"
        if self._future is None:
            request = self.session.take_fit_request()
            if request is not None:
                self._future = self._pool.submit(
                    self._fit_function, request["neutral_capture"], request["motion_capture"],
                    accel_g_per_lsb=request["accel_g_per_lsb"])
                self._job_kind, self._job_token = "fit", request["token"]

    def _feed(self, samples, now):
        self.session.feed(samples, now_s=now)
        outputs = self.session.take_pair_outputs()
        selected = [sample for sample in samples if (sample.node_id-1)//6 == (self.proximal-1)//6]
        self._raw_count += len(selected)
        for sample in selected:
            if sample.node_id == self.proximal:
                self._receive_times.append(sample.host_monotonic_s)
        if self._validation is not None:
            deadline = self._validation["started_s"] + self._validation["duration_s"]
            online = [row for row in outputs["rows"]
                      if self._validation["started_s"] <= row["host_monotonic_s"] < deadline]
            available = 30000-len(self._validation["online"])
            self._validation["online"].extend(online[:available])
            self._validation["online_discarded_rows"] += outputs["discarded_rows"]+max(0, len(online)-available)
            in_window = [s for s in selected if self._validation["started_s"] <= s.host_monotonic_s < deadline]
            remaining = 180000-self._validation_samples
            retained = in_window[:remaining]
            self._validation["samples"].extend(retained)
            self._validation_samples += len(retained)
            if len(in_window) > remaining:
                self._finish_validation("partial", "raw_retention_limit")

    def _publish(self, now, *, running=True):
        state = self.session.snapshot(now)
        rate = None
        recent = [stamp for stamp in self._receive_times if now-stamp <= 2.]
        if len(recent) > 1 and recent[-1] > recent[0]:
            rate = (len(recent)-1)/(recent[-1]-recent[0])
        if self._validation is not None:
            # UI snapshots are separate from raw CSV and full-rate online rows.
            self._validation["series"].append(dict(host_monotonic_s=now,
                math_angle_deg=state.get("math_angle_deg"),
                anatomical_angle_deg=state.get("anatomical_angle_deg"),
                preview_valid=state.get("preview_valid"), observer=state.get("observer")))
        state["runtime"] = dict(running=running, connected=self._source is not None,
            background_job=self._job_kind, error=self._error, notice=self._notice,
            proximal=self.proximal, distal=self.distal, received_rate_hz=rate,
            selected_group_raw_samples=self._raw_count,
            validation_remaining_s=max(0., self._validation["started_s"]+self._validation["duration_s"]-now) if self._validation else None,
            validation_duration_s=self._validation["duration_s"] if self._validation else None,
            validation_markers=copy.deepcopy(self._validation["markers"]) if self._validation else [],
            validation_online_rows=len(self._validation["online"]) if self._validation else None,
            validation_online_discarded_rows=self._validation["online_discarded_rows"] if self._validation else None,
            validation_now_s=now,
            validation_attempts=len(self._validation_attempts), last_export=self._last_export,
            validation_for_current_candidate=self._validation_for_current_candidate,
            validation_guided_quickcheck=bool(self._validation and self._validation.get("guided_quickcheck")),
            unsaved_experiment=self._revision > self._export_revision and self._revision > 0,
            serial_performance=copy.deepcopy(getattr(self._source, "performance", {})))
        with self._lock:
            self._snapshot = state

    def run(self):
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hinge-background")
        next_publish = 0.
        try:
            while not self._stop.is_set():
                for _ in range(8):
                    try:
                        command, args = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self._command(command, args)
                        self._error = ""
                    except Exception as exc:
                        self._error = f"{type(exc).__name__}: {exc}"
                if self._source is not None:
                    try:
                        self._feed(self._source.read(), self._clock())
                    except Exception as exc:
                        self._error = f"采集停止：{type(exc).__name__}: {exc}"
                        self._disconnect()
                now = self._clock()
                self.session.tick(now)
                self._advance_marker(now)
                if self._validation and now >= self._validation["started_s"]+self._validation["duration_s"]:
                    status = "complete" if self._validation["samples"] else "no_data"
                    self._finish_validation(status, "duration_elapsed")
                self._jobs()
                if now >= next_publish:
                    self._publish(now)
                    next_publish = now+1./30.
                self.msleep(2)
        except Exception as exc:
            self._error = f"工作线程停止：{type(exc).__name__}: {exc}"
        finally:
            try:
                self._disconnect()
            except Exception as exc:
                self._error += f"；关闭串口：{exc}"
            self._pool.shutdown(wait=True, cancel_futures=True)
            self._publish(self._clock(), running=False)
