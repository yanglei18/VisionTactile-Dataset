"""Bounded hand-service observation and commands under recorder admission.

No serial or numerical imports. The manager owns process groups; closing this
adapter only stops observation, never the hand service or an active recording.
"""
from copy import deepcopy
import json
import queue
import threading
import time

from vt_capture_core.hand_paths import default_hand_control_socket
from vt_capture_core.hand_protocol import validate_hand_command


def _receipt(request_id, state, code, detail):
    return dict(request_id=request_id, state=state, code=code, detail=str(detail)[:1000], trial_id="")


class HandControl:
    def __init__(self, profiles, *, manager_instance_id, allowed, admission_lock,
                 process_snapshot, exchange=None, monotonic=time.monotonic):
        if exchange is None:
            from vt_fsglove12.hand_service_protocol import request
            exchange = request
        from vt_tactile_capture.display_transport import default_workbench_socket
        self._exchange, self._clock = exchange, monotonic
        self.instance_id, self._allowed = manager_instance_id, allowed
        self._admission, self._process_snapshot = admission_lock, process_snapshot
        self._lock, self._stop = threading.RLock(), threading.Event()
        self._queue = queue.Queue(maxsize=8)
        self._thread = None
        self._rows, self._seen, self._requests, self._pending = {}, {}, {}, set()
        self._retired = set()
        self._latest_id = None
        tactile_profiles = {p["parameters"]["hand"]: p for p in profiles if p["preset"] == "tactile"}
        tactile = {hand: profile["parameters"] for hand, profile in tactile_profiles.items()}
        motion = {p["parameters"]["hand"]: p for p in profiles if p["preset"] == "glove"}
        if len(motion) != sum(p["preset"] == "glove" for p in profiles):
            raise ValueError("each hand must have exactly one managed motion profile")
        # Presentation exists independently of motion acquisition. A tactile-only
        # hand publishes no motion endpoint, process identity or valid pose.
        for hand in sorted(motion.keys() | tactile.keys()):
            profile = motion.get(hand)
            # The settings form keeps disabled profiles. Enabling touch alone
            # must not inherit the disabled motion runtime or its resource bundle.
            if (profile is not None and profile.get("enabled") is False
                    and tactile_profiles.get(hand, {}).get("enabled", False)):
                profile = None
            params = profile["parameters"] if profile else tactile[hand]
            self._rows[hand] = dict(hand=hand, group_id=profile["id"] if profile else "",
                control_socket=(params.get("control_socket") or str(default_hand_control_socket(hand))) if profile else "",
                tactile_socket=tactile.get(hand, {}).get("display_socket") or str(default_workbench_socket(hand)),
                resources=params.get("resources", ""), port=params.get("port", "") if profile else "",
                tactile_port=tactile.get(hand, {}).get("port", ""),
                glove_python=params.get("glove_python", ""),
                service_instance_id="", service_state="unavailable", connected=False,
                raw_fresh=False, calibration_ready=False, pose_valid=False,
                session_id="", profile_version=0, busy=False, age_s=None,
                detail="手部服务尚未启动。" if profile else "纯触觉显示；未配置动捕服务。")

    def start(self):
        self._thread = threading.Thread(target=self._run, name="managed-hands", daemon=True)
        self._thread.start()
        return self

    @property
    def busy(self):
        with self._lock:
            return bool(self._pending or any(row["busy"] for row in self._rows.values()))

    def snapshot(self):
        now = self._clock()
        with self._lock:
            result = deepcopy(self._rows)
            for hand, row in result.items():
                row["age_s"] = max(0., now-self._seen[hand]) if hand in self._seen else None
                if row["age_s"] is None or row["age_s"] > 1.5:
                    row.update(service_state="unavailable", connected=False, raw_fresh=False, pose_valid=False)
            return result

    def ready(self, spec):
        """Service readiness is deliberately separate from a calibrated pose."""
        if spec.kind != "glove":
            return None
        return any(row["group_id"] == spec.id and row["service_state"] == "ready"
                   for row in self.snapshot().values())

    def operation(self, request_id):
        with self._lock:
            entry = self._requests.get(request_id)
            return deepcopy(entry[1]) if entry else None

    def command(self, params, request_id):
        validate_hand_command("hand.command", params)
        fingerprint = json.dumps(params, sort_keys=True, separators=(",", ":"))
        # Same lock as capture.start and device maintenance admission. Network
        # work is queued, never performed under this lock or the IPC selector.
        with self._admission:
            groups = self._process_snapshot().get("groups", [])
            permission = self._allowed()
            with self._lock:
                return self._command_locked(params, request_id, fingerprint, groups, permission)

    def _command_locked(self, params, request_id, fingerprint, groups, permission):
        prior = self._requests.get(request_id)
        if prior:
            return deepcopy(prior[1]) if prior[0] == fingerprint else _receipt(
                request_id, "rejected", "request_id_conflict", "请求编号已经用于其他操作。")
        if len(self._requests) >= 1024:
            return _receipt(request_id, "rejected", "hand_ledger_full", "手部操作记录已达上限；请在空闲时重新打开管理后台。")
        row = self.snapshot().get(params["hand"])
        reason = ""
        if params["expected_manager_instance_id"] != self.instance_id:
            reason = "后台已切换，请刷新手部状态后操作。"
        elif row is None or row["service_state"] != "ready":
            reason = "手部服务未就绪。"
        elif params["expected_service_instance_id"] != row["service_instance_id"]:
            reason = "手部服务已重启，请刷新后操作。"
        else:
            group = next((g for g in groups if g["id"] == row["group_id"]), None)
            if not group or not group.get("owned") or group.get("state") not in {"STARTING", "RUNNING", "DEGRADED"}:
                reason = "当前后台未持有此手的运行实例，不能修改标定。"
        if not reason:
            allowed, reason = permission
            if allowed:
                reason = ""
            else:
                reason = reason or "当前不允许修改手部标定。"
        if reason:
            result = _receipt(request_id, "rejected", "hand_not_admitted", reason)
        else:
            result = _receipt(request_id, "accepted", "hand_queued", "操作已排队，等待手部服务执行。")
            try:
                self._queue.put_nowait((request_id, deepcopy(params)))
                self._pending.add(request_id)
            except queue.Full:
                result = _receipt(request_id, "rejected", "hand_queue_full", "手部操作队列已满。")
        self._requests[request_id] = (fingerprint, result, params["hand"])
        self._latest_id = request_id
        return deepcopy(result)

    def _update_receipt(self, request_id, receipt):
        with self._lock:
            if request_id not in self._requests or request_id in self._retired:
                return
            state = receipt.get("state", "unknown")
            mapped = {"queued": "accepted", "executed": "accepted", "rejected": "rejected", "unknown": "unknown"}.get(state, "unknown")
            old = self._requests[request_id]
            result = _receipt(request_id, mapped, "hand_" + state,
                receipt.get("error") or ("手部命令已执行；标定生效及保存状态请查看当前手部面板。" if state == "executed" else "等待手部命令结果。"))
            self._requests[request_id] = (old[0], result, old[2])
            if state in {"executed", "rejected"}:
                self._pending.discard(request_id)

    def _poll(self, hand):
        with self._lock:
            path = self._rows[hand]["control_socket"]
        if not path:
            return  # Tactile-only presentation never probes or invents an IMU service.
        try:
            reply = self._exchange(path, dict(schema_version=1, op="snapshot", hand=hand), timeout=.25)
            if reply.get("hand") != hand or not reply.get("service_instance_id"):
                raise ValueError("手部服务身份不匹配")
            snap = reply.get("snapshot") or {}
            runtime, calibration, pose = snap.get("runtime") or {}, snap.get("calibration") or {}, snap.get("pose") or {}
            age = runtime.get("last_sample_age_s")
            with self._lock:
                row = self._rows[hand]
                changed = row["service_instance_id"] and row["service_instance_id"] != reply["service_instance_id"]
                if changed:
                    for rid, entry in self._requests.items():
                        if rid in self._pending and entry[2] == hand:
                            self._requests[rid] = (entry[0], _receipt(rid, "unknown", "hand_restarted", "服务已重启，旧请求结果不可确认；不会重发。"), hand)
                            self._pending.discard(rid)
                            self._retired.add(rid)
                row.update(service_instance_id=reply["service_instance_id"], service_state=reply["service_state"],
                    connected=runtime.get("connected") is True,
                    raw_fresh=runtime.get("connected") is True and type(age) in (int, float) and 0 <= age <= .25,
                    calibration_ready=(calibration.get("profile_version") or 0) > 0,
                    pose_valid=reply.get("pose_valid") is True,
                    session_id=runtime.get("session_id") or "", profile_version=calibration.get("profile_version") or 0,
                    busy=reply.get("calibration_busy", True) is not False,
                    detail=str(runtime.get("error") or runtime.get("message") or "")[:4096])
                self._seen[hand] = self._clock()
                for receipt in reply.get("receipts", []):
                    self._update_receipt(receipt.get("request_id"), receipt)
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
            with self._lock:
                self._rows[hand].update(service_state="unavailable", connected=False, raw_fresh=False,
                                        pose_valid=False, detail=str(exc)[:4096])

    def _retire_stopped(self):
        """A verified terminal process state ends uncertainty, never retries it.

        A socket timeout alone cannot prove that a calibration stopped. Read
        process ownership outside the hand lock, in the same admission order
        as command submission, to avoid inverting process/hand locks.
        """
        with self._admission:
            try:
                groups = self._process_snapshot().get("groups", [])
            except (OSError, ValueError, RuntimeError):
                return
            stopped = {g["id"] for g in groups if g.get("state") in {"STOPPED", "FAILED"}
                       and g.get("owned") is False and "pid" in g and g["pid"] is None}
            with self._lock:
                for hand, row in self._rows.items():
                    if row["group_id"] not in stopped:
                        continue
                    for rid in tuple(self._pending):
                        old = self._requests[rid]
                        if old[2] != hand:
                            continue
                        self._requests[rid] = (old[0], _receipt(rid, "unknown", "hand_stopped",
                            "手部进程已确认停止，旧请求结果不可确认；不会重发。"), hand)
                        self._pending.discard(rid)
                        self._retired.add(rid)
                    row.update(service_state="unavailable", connected=False, raw_fresh=False,
                               pose_valid=False, busy=False, detail="手部进程已停止；可以重新启动。")
                    self._seen.pop(hand, None)

    def _run(self):
        while not self._stop.is_set():
            self._retire_stopped()
            try:
                request_id, params = self._queue.get(timeout=.1)
            except queue.Empty:
                pass
            else:
                hand = params["hand"]
                try:
                    with self._lock:
                        pending = request_id in self._pending
                    if not pending:
                        continue
                    reply = self._exchange(self._rows[hand]["control_socket"], dict(schema_version=1,
                        op="command", hand=hand, expected_service_instance_id=params["expected_service_instance_id"],
                        request_id=request_id, command=params["name"], args=params["args"],
                        expected_session_id=params["expected_session_id"], expected_profile_version=params["expected_profile_version"]), timeout=.5)
                    self._update_receipt(request_id, reply.get("receipt") or {"state": "unknown"})
                except (OSError, ValueError, RuntimeError) as exc:
                    self._update_receipt(request_id, {"state": "unknown", "error": f"执行结果待核实：{exc}"})
                finally:
                    self._queue.task_done()
            for hand in self._rows:
                if self._stop.is_set():
                    break
                self._poll(hand)
            self._retire_stopped()
            self._stop.wait(.1)

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(2.)
            if self._thread.is_alive():
                raise RuntimeError("hand observation worker did not stop")
