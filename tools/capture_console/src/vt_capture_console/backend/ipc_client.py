"""Bounded background IPC with v4 device control and v3/v2/v1 fallback. No retries."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import queue
import socket
import threading
import time
import uuid

from vt_capture_core.protocol import (
    CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION, MAX_MESSAGE_BYTES, PROTOCOL_VERSION, decode_message, encode_message,
)


MAX_PENDING_COMMANDS = 8
DEVICE_COMMANDS = {"device.start", "device.stop", "device.restart"}
CONTROL_COMMANDS = {"batch.create", "capture.start", "capture.stop", "quality.retry"} | DEVICE_COMMANDS
MANAGEMENT_VERSIONS = (CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION)
DETAIL_VERSIONS = (DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION)


def default_socket_path():
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    root = Path(runtime) if runtime else Path.home() / ".local" / "state"
    return root / "visiontactile" / "manager-control.sock"


def _exchange(path, request, timeout=0.6):
    """One request/reply with a total deadline and an LF-inclusive size cap."""
    wire = encode_message(request)
    deadline = time.monotonic() + timeout
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(path))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("后台连接超时")
        connection.settimeout(remaining)
        connection.sendall(wire)
        data = bytearray()
        while b"\n" not in data:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("后台响应超时")
            connection.settimeout(remaining)
            chunk = connection.recv(min(65536, MAX_MESSAGE_BYTES + 1 - len(data)))
            if not chunk:
                raise ConnectionError("后台在完整响应前断开")
            data.extend(chunk)
            if len(data) > MAX_MESSAGE_BYTES:
                raise ValueError("后台响应超过协议大小上限")
        line, trailing = bytes(data).split(b"\n", 1)
        if trailing:
            raise ValueError("后台返回多个协议帧")
        message = decode_message(line + b"\n")
        if ("ok" not in message or message["request_id"] != request["request_id"]
                or message["version"] != request["version"]):
            raise ValueError("后台响应版本或请求不匹配")
        if message["snapshot"]["mode"] != "live":
            raise ValueError("真实模式拒绝演示后台数据")
        return message


def fetch_snapshot(path, timeout=0.6, *, version=PROTOCOL_VERSION):
    """Keep the original v1 API; newer versions explicitly request richer snapshots."""
    request = {"version": version, "request_id": uuid.uuid4().hex, "command": "snapshot"}
    if version in MANAGEMENT_VERSIONS:
        request["params"] = {}
    message = _exchange(path, request, timeout)
    if not message["ok"]:
        raise ValueError(message.get("error", "后台拒绝快照请求"))
    if message.get("trial_details") is not None:
        raise ValueError("快照轮询不能附带完整试次详情")
    return message["snapshot"]


def _empty_management():
    return {"enabled": False, "controller": None, "batches": [], "trials": [], "operation": None}


def _empty_process_control():
    return {"enabled": False, "groups": []}


def _operation(request_id, state, code, detail, trial_id=""):
    return {"request_id": request_id, "state": state, "code": code,
            "detail": detail[:1000], "trial_id": trial_id}


class IpcBackend:
    mode = "live"

    def __init__(self, path=None):
        self.path = Path(path) if path else default_socket_path()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = None
        self._received = None
        self._error = "正在连接后台"
        self._version = None
        self._next_v2_probe = 0.0
        self._commands = queue.Queue(maxsize=MAX_PENDING_COMMANDS)
        self._pending = {}
        self._latest_operation = None
        self._management_history = None
        self._process_history = None
        self._latest_details = None
        self._pending_details = None
        self._details_status = {"trial_id": "", "request_id": "", "state": "idle", "error": ""}
        self._details_instance = None
        self._thread = threading.Thread(target=self._run, name="capture-console-ipc", daemon=True)
        self._thread.start()

    @property
    def latest_operation(self):
        with self._lock:
            return copy.deepcopy(self._latest_operation)

    def _connected(self):
        return (not self._stop.is_set() and not self._error and self._snapshot is not None
                and self._received is not None and time.monotonic() - self._received <= 1.0)

    @property
    def supports_details(self):
        with self._lock:
            return bool(self._connected() and self._version in DETAIL_VERSIONS)

    @property
    def supports_device_control(self):
        with self._lock:
            return bool(self._connected() and self._version == DEVICE_PROTOCOL_VERSION
                        and self._snapshot.get("process_control", {}).get("enabled", False)
                        and self._snapshot["capabilities"]["device_control"])

    @property
    def latest_details(self):
        with self._lock:
            return copy.deepcopy(self._latest_details)

    @property
    def details_status(self):
        with self._lock:
            if self._details_status["state"] == "ready" and not self._connected():
                self._details_status.update(state="stale", error="后台状态已陈旧；显示历史详情")
            return dict(self._details_status)

    def clear_details(self):
        """Forget a cleared selection; any in-flight reply becomes irrelevant."""
        with self._lock:
            self._latest_details = self._pending_details = self._details_instance = None
            self._details_status = {"trial_id": "", "request_id": "", "state": "idle", "error": ""}

    def request_details(self, trial_id) -> str:
        """Request only the selected trial, coalescing unsent requests to one."""
        request = {"version": DETAIL_PROTOCOL_VERSION, "request_id": uuid.uuid4().hex,
                   "command": "trial.details", "params": {"trial_id": trial_id}}
        encode_message(request)
        with self._lock:
            if trial_id == self._details_status["trial_id"] and self._details_status["state"] == "loading":
                return self._details_status["request_id"]
            if trial_id != self._details_status["trial_id"]:
                self._latest_details = None
            self._pending_details = None
            identifier = request["request_id"]
            self._details_status = {"trial_id": trial_id, "request_id": identifier, "state": "loading", "error": ""}
            if not self._connected():
                self._details_status.update(state="stale" if self._latest_details is not None else "error",
                                            error="后台未连接或已关闭；未请求新详情")
            elif self._version not in DETAIL_VERSIONS:
                self._details_status.update(state="unsupported", error="当前后台不支持试次详情，需要 v3")
            else:
                request["version"] = self._version
                self._details_instance = self._snapshot["instance_id"]
                self._pending_details = (request, self._details_instance)
        self._wake.set()
        return identifier

    def _unavailable(self, command):
        """Called under _lock. No capability is inferred from a v1 response."""
        if self._stop.is_set():
            return "CLIENT_CLOSED", "窗口连接已关闭；未发送命令"
        if (self._error or self._snapshot is None or self._received is None
                or time.monotonic() - self._received > 1.0):
            return "BACKEND_UNAVAILABLE", "后台未连接或状态陈旧；未发送命令"
        if command in DEVICE_COMMANDS:
            if self._version != DEVICE_PROTOCOL_VERSION:
                return "UNSUPPORTED_FEATURE", "当前后台不支持设备进程控制，需要 v4；未发送命令"
            if not self._snapshot.get("process_control", {}).get("enabled", False):
                return "PROCESS_CONTROL_DISABLED", "后台未启用设备进程控制；未发送命令"
            if not self._snapshot["capabilities"]["device_control"]:
                return "CONTROL_UNAVAILABLE", "后台当前不可控制设备进程；未发送命令"
            return None
        if self._version not in MANAGEMENT_VERSIONS:
            return "READ_ONLY_PROTOCOL", "当前后台仅支持只读协议；未发送控制命令"
        if command == "quality.retry" and self._version not in DETAIL_VERSIONS:
            return "UNSUPPORTED_FEATURE", "当前后台不支持重新质检，需要 v3；未发送命令"
        if not self._snapshot.get("management", {}).get("enabled", False):
            return "MANAGEMENT_DISABLED", "后台未启用采集管理；未发送命令"
        if command not in {"batch.create", "quality.retry"} and not self._snapshot["capabilities"]["record"]:
            return "CONTROL_UNAVAILABLE", "后台当前不可控制录制；未发送命令"
        return None

    def submit(self, command, params, request_id=None) -> str:
        """Queue once without UI-thread IO; timeouts never trigger a retry.

        A caller may explicitly retry a completed request with the same ID.
        Identical in-flight IDs are locally deduplicated, conflicting ones fail.
        """
        if command not in CONTROL_COMMANDS:
            raise ValueError("仅允许批次、录制、重新质检和设备进程命令")
        request = {"version": DEVICE_PROTOCOL_VERSION,
                   "request_id": uuid.uuid4().hex if request_id is None else request_id,
                   "command": command, "params": copy.deepcopy(params)}
        encode_message(request)
        identifier = request["request_id"]
        with self._lock:
            if identifier in self._pending:
                previous = self._pending[identifier]
                if previous["command"] != command or previous["params"] != request["params"]:
                    raise ValueError("正在处理的 request_id 不能用于不同命令")
                return identifier
            unavailable = self._unavailable(command)
            if unavailable:
                self._latest_operation = _operation(identifier, "rejected", *unavailable)
                return identifier
            if self._version == CONTROL_PROTOCOL_VERSION and {"task_name", "operator_name"} & set(request["params"]):
                self._latest_operation = _operation(identifier, "rejected", "UNSUPPORTED_FEATURE",
                                                    "当前后台不支持任务/操作员元信息，需要 v3；未发送命令")
                return identifier
            request["version"] = self._version
            try:
                self._commands.put_nowait(request)
            except queue.Full:
                self._latest_operation = _operation(identifier, "rejected", "QUEUE_FULL", "待处理命令队列已满；未发送命令")
                return identifier
            self._pending[identifier] = request
            self._latest_operation = _operation(identifier, "unknown", "QUEUED", "已排队，等待后台确认；尚不代表操作已完成")
        self._wake.set()
        return identifier

    def _accept_snapshot(self, value, version, *, update_operation=True):
        with self._lock:
            replaced = self._snapshot is not None and self._snapshot["instance_id"] != value["instance_id"]
            interrupted = bool(self._error or (self._received is not None and time.monotonic() - self._received > 1.0))
            if self._latest_details is not None and (replaced or interrupted or version not in DETAIL_VERSIONS):
                self._details_status.update(state="stale", error="连接或后台已变化；显示历史详情，请重新获取")
            self._snapshot, self._received, self._error = value, time.monotonic(), ""
            self._version = version
            if version in MANAGEMENT_VERSIONS:
                self._management_history = copy.deepcopy(value["management"])
            if version == DEVICE_PROTOCOL_VERSION:
                self._process_history = copy.deepcopy(value.get("process_control", _empty_process_control()))
            operation = value.get("management", {}).get("operation")
            if update_operation and operation and (self._latest_operation is None
                              or operation["request_id"] == self._latest_operation["request_id"]):
                self._latest_operation = copy.deepcopy(operation)

    def _poll(self):
        now = time.monotonic()
        if self._version in (None, DEVICE_PROTOCOL_VERSION) or now >= self._next_v2_probe:
            versions = (DEVICE_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, CONTROL_PROTOCOL_VERSION, PROTOCOL_VERSION)
        elif self._version == DETAIL_PROTOCOL_VERSION:
            versions = (DETAIL_PROTOCOL_VERSION, CONTROL_PROTOCOL_VERSION, PROTOCOL_VERSION)
        elif self._version == CONTROL_PROTOCOL_VERSION:
            versions = (CONTROL_PROTOCOL_VERSION, PROTOCOL_VERSION)
        else:
            versions = (PROTOCOL_VERSION,)
        failure = None
        for version in versions:
            try:
                value = fetch_snapshot(self.path, version=version)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                failure = exc
                self._next_v2_probe = time.monotonic() + 5.0
            else:
                self._accept_snapshot(value, version)
                return
        raise failure

    def _send_details(self, request, instance_id):
        identifier = request["request_id"]
        with self._lock:
            if self._details_status["request_id"] != identifier:
                return
            available = self._connected() and self._version in DETAIL_VERSIONS
            if available:
                request = dict(request, version=self._version)
        try:
            if not available:
                raise ConnectionError("后台连接或详情协议不可用")
            reply = _exchange(self.path, request, timeout=.6)
            # A read-only details reply must never replace the pending control receipt.
            self._accept_snapshot(reply["snapshot"], request["version"], update_operation=False)
            if not reply["ok"]:
                raise ValueError(reply.get("error", "无法读取试次详情"))
            details = reply.get("trial_details")
            if not isinstance(details, dict) or details.get("trial_id") != request["params"]["trial_id"]:
                raise ValueError("后台详情缺失或试次身份不匹配")
            if reply["snapshot"]["instance_id"] != instance_id:
                raise ValueError("后台已更换，请重新获取详情")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            with self._lock:
                if self._details_status["request_id"] == identifier:
                    self._details_status.update(state="stale" if self._latest_details is not None else "error", error=str(exc)[:1000])
            return
        with self._lock:
            if self._details_status["request_id"] == identifier:
                self._latest_details = copy.deepcopy(details)
                self._details_status.update(state="ready", error="")

    def _send_command(self, request):
        identifier = request["request_id"]
        trial_id = request["params"].get("trial_id", "")
        with self._lock:
            unavailable = self._unavailable(request["command"])
            if unavailable is None and request["version"] > self._version:
                unavailable = ("PROTOCOL_CHANGED", "后台协议已降级；未发送此命令，请重新确认")
        try:
            if unavailable:
                operation = _operation(identifier, "rejected", *unavailable, trial_id=trial_id)
            else:
                reply = _exchange(self.path, request, timeout=5.0)
                self._accept_snapshot(reply["snapshot"], request["version"])
                operation = reply.get("operation")
                if operation is None:
                    operation = _operation(identifier, "unknown" if reply["ok"] else "rejected",
                                           "RESULT_UNAVAILABLE" if reply["ok"] else "COMMAND_REJECTED",
                                           reply.get("error", "后台未提供操作结果；请核对录制状态"), trial_id)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            operation = _operation(identifier, "unknown", "TRANSPORT_UNKNOWN",
                                   f"请求结果未知，未自动重试；请核对后台状态：{exc}", trial_id)
            with self._lock:
                self._error = f"后台请求未确认：{exc}"
        with self._lock:
            self._pending.pop(identifier, None)
            if self._latest_operation is None or self._latest_operation["request_id"] == identifier:
                self._latest_operation = copy.deepcopy(operation)

    def _run(self):
        while not self._stop.is_set():
            self._wake.clear()
            try:
                request = self._commands.get_nowait()
            except queue.Empty:
                request = None
            if request is not None:
                try:
                    self._send_command(request)
                finally:
                    self._commands.task_done()
                continue
            with self._lock:
                details = self._pending_details
                self._pending_details = None
            if details is not None:
                self._send_details(*details)
                continue
            delay = 0.15
            try:
                self._poll()
            except (OSError, ValueError, KeyError, TypeError) as exc:
                with self._lock:
                    self._error = f"后台未连接：{exc}"
                delay = 1.0
            self._wake.wait(delay)

    def snapshot(self):
        with self._lock:
            value = copy.deepcopy(self._snapshot)
            error, received = self._error, self._received
            latest = copy.deepcopy(self._latest_operation)
            version = self._version
            history = copy.deepcopy(self._management_history)
            process_history = copy.deepcopy(self._process_history)
        if value is None:
            return {"mode": "live", "instance_id": "", "sequence": 0,
                    "recording": {"state": "UNAVAILABLE", "session_id": "", "elapsed_s": 0.0,
                                  "duration_s": 0.0, "detail": error},
                    "devices": [], "events": [{"id": "ipc-disconnected", "level": "warning", "message": error}],
                    "resources": {"disk_free_bytes": None}, "capabilities": {"record": False, "device_control": False},
                    "management": {**_empty_management(), "operation": latest},
                    "process_control": _empty_process_control()}
        management = value.setdefault("management", history or _empty_management())
        process_control = value.setdefault("process_control", process_history or _empty_process_control())
        history_reason = None
        process_history_reason = None
        if latest is not None:
            management["operation"] = latest
        if version not in MANAGEMENT_VERSIONS:
            management.update(enabled=False, controller=None)
            value["capabilities"] = {"record": False, "device_control": False}
            history_reason = "只读协议，无法确认当前状态"
        if version != DEVICE_PROTOCOL_VERSION:
            value["capabilities"]["device_control"] = False
            process_history_reason = "协议已降级，当前设备进程状态未确认"
        if self._stop.is_set() or error or received is None or time.monotonic() - received > 1.0:
            value["resources"]["disk_free_bytes"] = None
            for device in value["devices"]:
                device["status"] = "stale"
                device["receive_hz"] = 0.0
                device["valid_hz"] = None
                if device["age_s"] is not None and received is not None:
                    device["age_s"] += max(0.0, time.monotonic() - received)
            value["recording"].update(state="UNAVAILABLE", detail="后台失联；录制状态未知，未发送任何停止命令")
            value["events"] = [{"id": "ipc-disconnected", "level": "warning", "message": error or "后台状态已陈旧"}] + value["events"][:29]
            value["capabilities"] = {"record": False, "device_control": False}
            management.update(enabled=False, controller=None)
            history_reason = "后台失联，当前状态未确认"
            process_history_reason = "后台失联，当前设备进程状态未确认"
        if history_reason:
            for trial in management["trials"]:
                if trial["state"] in {"STARTING", "RECORDING", "STOPPING"}:
                    trial["state"] = "UNKNOWN"
                trial["detail"] = (f"历史记录 · {history_reason}。 " + trial["detail"])[:1000]
        if process_history_reason:
            process_control["enabled"] = False
            for group in process_control["groups"]:
                group.update(state="UNKNOWN", can_start=False, can_stop=False, can_restart=False)
                group["detail"] = (f"历史记录 · {process_history_reason}。 " + group["detail"])[:1000]
        elif not process_control["enabled"] or not value["capabilities"]["device_control"]:
            for group in process_control["groups"]:
                group.update(can_start=False, can_stop=False, can_restart=False)
        return value

    def close(self):
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=1.5)
