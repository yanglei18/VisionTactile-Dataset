"""Single bounded, read-only scene client thread. No GUI or ROS imports."""
from __future__ import annotations

import copy
import os
from pathlib import Path
import socket
import struct
import threading
import time
import uuid

from .ipc_client import default_socket_path


MAX_SCENE_BYTES = 256 * 1024


class SceneBackend:
    mode = "live"

    def __init__(self, path=None, *, control_path=None, expected_instance_id=None, autostart=True):
        control_path = Path(control_path) if control_path else default_socket_path()
        self.path = Path(path) if path else control_path.with_name("manager-scene.sock")
        self.control_path = control_path
        self._expected = expected_instance_id
        self._verified = None
        self._peer_pid = None
        self._scene = None
        self._description = {}
        self._hello = {}
        self._hello_at = 0.
        from .cloud_backend import CloudBackend
        self.cloud_backend = CloudBackend(autostart=autostart)
        self._interests = {}
        self._viewer_id = uuid.uuid4().hex
        self._status = {"state": "waiting", "message": "正在等待管理后台身份"}
        self._lock = threading.RLock()
        self._stop, self._wake = threading.Event(), threading.Event()
        self._thread = None
        if autostart:
            self._thread = threading.Thread(target=self._run, name="capture-console-scene", daemon=True)
            self._thread.start()

    def set_manager_instance(self, instance_id):
        with self._lock:
            if instance_id != self._expected:
                self._expected = instance_id
                self._verified = self._peer_pid = self._scene = None
                self._description = {}
                self._hello = {}
                self._hello_at = 0.
                self.cloud_backend.configure(None, instance_id)
                self._status = {"state": "waiting", "message": "正在核验管理后台身份"}
                self._wake.set()

    def set_interest(self, viewer_id, layers):
        self.cloud_backend.set_interest(viewer_id, "cloud" in layers)
        with self._lock:
            if layers:
                self._interests[viewer_id] = frozenset(layers)
            else:
                self._interests.pop(viewer_id, None)
        self._wake.set()

    def latest(self):
        with self._lock:
            return copy.deepcopy(self._scene)

    def description(self):
        with self._lock:
            return copy.deepcopy(self._description)

    def hello(self):
        with self._lock:
            return copy.deepcopy(self._hello)

    def status(self):
        with self._lock:
            return dict(self._status)

    def _request(self, command, params):
        from vt_capture_core.spatial.scene_protocol import encode_scene_message, decode_scene_message
        request = dict(protocol="vt.scene", version=1, request_id=uuid.uuid4().hex, command=command, params=params)
        deadline = time.monotonic() + .5
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(.5)
            connection.connect(str(self.path))
            pid, uid, _gid = struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
            if uid != os.getuid():
                raise ValueError("场景后台用户身份不匹配")
            if self._peer_pid is None:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as control:
                    control.settimeout(max(.001, deadline-time.monotonic()))
                    control.connect(str(self.control_path))
                    control_pid, control_uid, _ = struct.unpack("3i", control.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                    if (pid, uid) != (control_pid, control_uid):
                        raise ValueError("场景与控制后台进程身份不匹配")
                self._peer_pid = pid
            elif pid != self._peer_pid:
                raise ValueError("场景后台进程已更换")
            connection.settimeout(max(.001, deadline-time.monotonic()))
            connection.sendall(encode_scene_message(request))
            data = bytearray()
            while b"\n" not in data:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("场景后台响应超时")
                connection.settimeout(remaining)
                part = connection.recv(min(65536, MAX_SCENE_BYTES+1-len(data)))
                if not part:
                    raise ConnectionError("场景后台响应不完整")
                data.extend(part)
                if len(data) > MAX_SCENE_BYTES:
                    raise ValueError("场景响应超过大小上限")
            line, trailing = bytes(data).split(b"\n", 1)
            if trailing:
                raise ValueError("场景后台返回多余数据")
            response = decode_scene_message(line+b"\n")
        if response.get("request_id") != request["request_id"] or not response.get("ok"):
            raise ValueError("场景响应错误或请求标识不匹配")
        return response

    def poll_once(self):
        with self._lock:
            expected = self._expected
            interest = sorted(set().union(*self._interests.values()) & {"tracker", "camera", "cloud", "trail"}) if self._interests else []
            verified = self._verified
            sequence = self._scene.get("sequence", 0) if self._scene else 0
        if not expected or not interest:
            return
        if verified != expected or time.monotonic()-self._hello_at > 1.:
            hello = self._request("hello", {}).get("hello", {})
            if hello.get("manager_instance_id") != expected:
                raise ValueError("场景与控制后台实例不匹配")
            if hello.get("manager_pid") is not None and self._peer_pid is not None and hello["manager_pid"] != self._peer_pid:
                raise ValueError("场景声明的后台进程身份不匹配")
            description = self._request("describe", {}).get("description", {})
            with self._lock:
                if expected != self._expected:
                    return
                self._hello, self._description, self._verified = hello, description, expected
                self._hello_at = time.monotonic()
                self.cloud_backend.configure(hello.get("cloud"), expected)
        response = self._request("latest", dict(after_sequence=sequence, viewer_id=self._viewer_id, interest=interest))
        scene = response.get("scene")
        with self._lock:
            if self._expected != expected:
                return
            if scene is not None:
                if scene.get("manager_instance_id") != expected or scene.get("mode") != "live":
                    # Synthetic frames are allowed only by the explicit DemoSceneBackend.
                    raise ValueError("场景后台实例或运行模式不匹配")
                self._scene = scene
            self._status = {"state": "ready", "message": "LIVE · 原生公共坐标 · 实时监看"}

    def _run(self):
        delay = .05
        while not self._stop.is_set():
            try:
                self.poll_once()
                delay = .05
            except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
                with self._lock:
                    self._verified = self._peer_pid = None
                    self._scene = None
                    self._hello = {}
                    self.cloud_backend.configure(None, self._expected)
                    message = ("空间服务未启动；请在“空间 → 空间配置”选择场景文件，并在后台空闲时保存并应用。" if isinstance(exc, FileNotFoundError)
                               else "联合空间暂不可用：" + str(exc)[:200])
                    self._status = {"state": "unavailable", "message": message}
                delay = min(2., max(.25, delay*2))
            self._wake.wait(delay)
            self._wake.clear()

    def close(self):
        self._stop.set()
        self._wake.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=.7)
        self.cloud_backend.close()
