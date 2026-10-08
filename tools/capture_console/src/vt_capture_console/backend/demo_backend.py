"""Deterministic, in-memory demonstration. No ROS, files, sockets or devices."""

from __future__ import annotations

import math
import time
import uuid
from copy import deepcopy


class DemoBackend:
    mode = "demo"
    supports_details = True
    supports_device_control = True

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.origin = clock()
        self.started = None
        self.duration = 30.0
        self.state = "IDLE"
        self.sequence = 0
        self.scenario = "normal"
        self.hands = "both"
        self._process_groups = [
            {"id": "cameras", "label": "三相机 · 演示", "kind": "camera",
             "topics": [f"/{name}/color/image_raw" for name in ("d405_1", "d405_2", "d436")]},
            {"id": "trackers", "label": "Tracker · 演示", "kind": "tracker",
             "topics": [f"/vive/{role}/sample" for role in ("left_wrist", "right_wrist", "torso")]},
        ]
        for side, title in (("left", "左手"), ("right", "右手")):
            for kind, name, suffix in (("glove", "姿态", "state"), ("tactile", "触觉", "tactile/state")):
                self._process_groups.append({"id": f"{side}_{kind}", "label": f"{title} {name} · 演示",
                                             "kind": kind, "topics": [f"/gloves/{side}/{suffix}"]})
        for group in self._process_groups:
            group.update(state="RUNNING", owned=True, pid=None, log_path="",
                         detail="仅内存模拟，未启动任何硬件或进程；PID 和日志不存在。")
        self.trials = []
        self.batches = [{"id": "demo-batch", "label": "演示批次", "created_at": "2026-01-01T00:00:00Z"}]
        self.latest_operation = None
        self._operation_cache = {}
        self._details = {}
        self.latest_details = None
        self.details_status = {"trial_id": "", "request_id": "", "state": "idle", "error": ""}
        self.events = [{"id": "demo-ready", "level": "info", "message": "演示工作台已就绪；所有数据均为合成，未连接硬件。"}]

    def _event(self, message):
        self.events.insert(0, {"id": f"demo-{self.sequence}-{len(self.events)}", "level": "info", "message": message})
        self.events = self.events[:30]

    def start_recording(self, duration=30, *, batch_id="demo-batch", label="模拟试次", task_name="", operator_name=""):
        if self.state == "RECORDING":
            return
        if not 1 <= duration <= 300:
            raise ValueError("演示时长必须为 1–300 秒")
        self.duration = float(duration)
        self.started = self.clock()
        self.state = "RECORDING"
        self.trials.insert(0, {"id": f"DEMO-{len(self.trials) + 1:03d}", "state": "RECORDING", "duration": 0.0,
                              "batch_id": batch_id, "label": label, "duration_s": float(duration)})
        identifier = self.trials[0]["id"]
        self._details[identifier] = {
            "trial_id": identifier,
            "metadata": {"start": {"schema_version": 1, "demo": True, "label": label,
                                   "task_name": task_name, "operator_name": operator_name,
                                   "planned_duration_s": duration, "note": "仅演示，未连接传感器。"}, "finish": None},
            "quality": {"state": "NOT_ASSESSED", "verdict": "", "detail": "录制中不执行质检。",
                        "attempt": 0, "report_path": "", "report": None},
            "artifacts": {"start_metadata_path": "", "finish_metadata_path": ""},
        }
        self._event("模拟录制开始；不创建 bag，不写入采集数据目录。")

    def stop_recording(self):
        if self.state == "RECORDING":
            self.state = "COMPLETE"
            self.trials[0].update(state="COMPLETE", duration=min(self.duration, self.clock() - self.started))
            identifier = self.trials[0]["id"]
            self._details[identifier]["metadata"]["finish"] = {"demo": True, "elapsed_s": self.trials[0]["duration"],
                                                                  "state": "COMPLETE", "bag_created": False}
            self._demo_quality(identifier)
            self._event("模拟试次完成；详情中的质检报告为合成演示，未读取或生成真实 bag。")

    def _demo_quality(self, identifier):
        quality = self._details[identifier]["quality"]
        quality.update(state="COMPLETED", verdict="ok", attempt=quality["attempt"] + 1,
                       detail="仅演示报告，未执行真实质量检查。", report={
                           "schema_version": 1, "report_kind": "synthetic_demo", "status": "ok",
                           "summary": {"topic_count": 1, "inspected_topic_count": 0, "warning_count": 0},
                           "topics": {"/demo/image": {"recorded_message_count": 900,
                                                     "metadata_global_duration_rate_hz": 30.0}},
                           "warnings": [], "limitations": ["全部数字为合成示例，不代表硬件性能或数据质量。"],
                       })

    def request_details(self, trial_id):
        request_id = uuid.uuid4().hex
        self.latest_details = deepcopy(self._details.get(trial_id))
        self.details_status = {"trial_id": trial_id, "request_id": request_id,
                               "state": "ready" if self.latest_details else "error",
                               "error": "" if self.latest_details else "未找到演示试次"}
        return request_id

    def snapshot(self):
        self.sequence += 1
        phase = self.clock() - self.origin
        if self.state == "RECORDING" and self.clock() - self.started >= self.duration:
            self.stop_recording()
        devices = []

        def add(identifier, label, kind, topic, hz, preview, side=""):
            status = "online"
            if self.scenario == "stale" and kind in ("glove", "tactile"):
                status = "stale"
            elif self.scenario == "offline" and identifier == "d436":
                status = "offline"
            group = next((item for item in self._process_groups if topic in item["topics"]), None)
            if group and group["state"] == "STOPPED":
                status = "offline"
                preview = {**preview, "valid": False}
            devices.append({"id": identifier, "label": label, "kind": kind, "side": side,
                            "status": status, "topic": topic, "receive_hz": hz if status == "online" else 0.0,
                            "valid_hz": hz if kind in ("glove", "tracker") and status == "online" else None,
                            "age_s": 0.018 if status == "online" else 4.2,
                            "detail": "合成演示数据，不代表设备实测性能", "preview": preview})

        for i, camera in enumerate(("d405_1", "d405_2", "d436")):
            add(camera, ("D405 · 左视角", "D405 · 右视角", "D436 · 主视角")[i], "camera",
                f"/{camera}/color/image_raw", 30.0, {"demo": True, "phase": phase + i})
        for i, (role, label) in enumerate((("left_wrist", "左腕"), ("right_wrist", "右腕"), ("torso", "躯干"))):
            add(role, f"Tracker · {label}", "tracker", f"/vive/{role}/sample", 90.0,
                {"demo": True, "phase": phase, "valid": True,
                 "position": [0.2 * (i - 1), 0.07 * math.sin(phase * 0.4 + i), 0.8],
                 "orientation": [0.0, 0.0, math.sin(phase * 0.08), math.cos(phase * 0.08)]})
        for side in ("left", "right"):
            if self.hands not in ("both", side):
                continue
            label = "左手" if side == "left" else "右手"
            for kind, suffix, hz in (("glove", "姿态", 120.0), ("tactile", "触觉", 30.0)):
                topic = f"/gloves/{side}/" + ("state" if kind == "glove" else "tactile/state")
                add(f"{side}_{kind}", f"{label} · {suffix}", kind, topic, hz,
                    {"demo": True, "phase": phase, "valid": True}, side)
        elapsed = min(self.duration, self.clock() - self.started) if self.state == "RECORDING" else 0.0
        if self.state == "COMPLETE" and self.trials:
            elapsed = self.trials[0]["duration"]
        return {"mode": "demo", "instance_id": "demo-in-memory", "sequence": self.sequence,
                "recording": {"state": self.state, "session_id": self.trials[0]["id"] if self.trials else "",
                              "elapsed_s": elapsed, "duration_s": self.duration, "detail": "仅模拟流程，不生成 bag"},
                "devices": devices, "events": list(self.events), "resources": {"disk_free_bytes": None},
                "capabilities": {"record": False, "device_control": False}}

    def close(self):
        # A window disconnect is deliberately not an explicit device STOP.
        pass

    @property
    def process_control(self):
        """Additional presentation state without extending the strict v1 snapshot."""
        groups = deepcopy(self._process_groups)
        for group in groups:
            enabled = self.state != "RECORDING"
            group.update(can_start=enabled and group["state"] == "STOPPED",
                         can_stop=enabled and group["owned"] and group["state"] in {"RUNNING", "DEGRADED"},
                         can_restart=enabled and group["owned"] and group["state"] in {"RUNNING", "DEGRADED"})
            if not enabled:
                group["detail"] = "模拟录制期间禁用设备启停；先显式结束录制。"
        return {"enabled": True, "groups": groups}

    def _submit_device(self, command, params):
        if params.get("expected_manager_instance_id") != "demo-in-memory":
            raise ValueError("后台实例已变化，请刷新后重新确认")
        group = next((g for g in self.process_control["groups"] if g["id"] == params.get("group_id")), None)
        action = command.removeprefix("device.")
        if (not group or not group.get("can_" + action) or group["state"] == "EXTERNAL"
                or (action in {"stop", "restart"} and not group["owned"])):
            raise ValueError("模拟进程组当前状态或归属不允许此操作")
        target = next(g for g in self._process_groups if g["id"] == group["id"])
        target.update(state="STOPPED" if action == "stop" else "RUNNING", owned=action != "stop",
                      detail="仅内存模拟，未启动或停止任何真实设备；未创建日志。")
        title = {"start": "启动", "stop": "停止", "restart": "重启"}[action]
        self._event(f"模拟{title}：{group['label']}。未操作真实硬件或进程。")

    @property
    def management(self):
        trials = []
        for trial in self.trials:
            elapsed = min(self.duration, self.clock() - self.started) if trial["state"] == "RECORDING" else trial["duration"]
            trials.append({"id": trial["id"], "batch_id": trial["batch_id"], "label": trial["label"],
                           "state": trial["state"], "session_id": trial["id"], "output_path": "",
                           "duration_s": trial["duration_s"], "elapsed_s": elapsed,
                           "quality": ("OK" if self._details[trial["id"]]["quality"]["state"] == "COMPLETED"
                                       else self._details[trial["id"]]["quality"]["state"]),
                           "detail": "仅演示，无 bag；关闭窗口后清空。", "created_at": "2026-01-01T00:00:00Z"})
        return {"enabled": True, "controller": None, "batches": list(self.batches),
                "trials": trials, "operation": self.latest_operation}

    def submit(self, command, params, request_id=None):
        """Mirror the live command UI without sockets, ROS or persistent writes."""
        request_id = request_id or uuid.uuid4().hex
        if request_id in self._operation_cache:
            previous_command, previous_params, receipt = self._operation_cache[request_id]
            if (command, params) != (previous_command, previous_params):
                raise ValueError("演示请求 ID 对应参数冲突")
            self.latest_operation = receipt
            return request_id
        trial_id = ""
        if command in {"device.start", "device.stop", "device.restart"}:
            self._submit_device(command, params)
        elif command == "batch.create":
            name = params.get("label", "").strip()
            if not name or len(name) > 120:
                raise ValueError("批次名称须为 1–120 个字符")
            self.batches.insert(0, {"id": f"demo-batch-{len(self.batches)}", "label": name,
                                   "created_at": "2026-01-01T00:00:00Z"})
            self._event("已创建演示批次：" + name)
        elif command == "capture.start":
            if self.state == "RECORDING":
                raise ValueError("已有模拟录制正在进行")
            if not any(b["id"] == params.get("batch_id") for b in self.batches):
                raise ValueError("请选择演示批次")
            self.start_recording(params["duration_s"], batch_id=params["batch_id"], label=params["label"],
                                 task_name=params.get("task_name", ""), operator_name=params.get("operator_name", ""))
            trial_id = self.trials[0]["id"]
        elif command == "capture.stop":
            if not self.trials or params.get("trial_id") != self.trials[0]["id"]:
                raise ValueError("停止目标与当前模拟试次不符")
            self.stop_recording()
            trial_id = self.trials[0]["id"]
        elif command == "quality.retry":
            trial_id = params.get("trial_id", "")
            detail = self._details.get(trial_id)
            if detail is None or detail["quality"]["state"] not in {"ERROR", "INTERRUPTED"}:
                raise ValueError("仅可重试执行失败的演示检查")
            self._demo_quality(trial_id)
        else:
            raise ValueError("不支持的演示命令")
        receipt = {"request_id": request_id, "state": "accepted", "code": "DEMO_APPLIED",
                   "detail": "演示操作已完成，未操作真实设备或生成 bag。", "trial_id": trial_id}
        self.latest_operation = receipt
        self._operation_cache[request_id] = (command, dict(params), receipt)
        return request_id
