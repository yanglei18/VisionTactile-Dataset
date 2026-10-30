"""Opt-in desktop VIVE Hub startup, followed by the managed read-only ROS bridge.

No Qt/ROS imports, pairing, map commands, process termination or restart loop.
Settings are desktop-local, separate from the manager's capture configuration.
"""
from __future__ import annotations

from copy import deepcopy
import json
import fcntl
import os
from pathlib import Path
import socket
import stat
import struct
import subprocess
import threading
import time
import uuid

from .ipc_client import _exchange
from .workstation_settings import atomic_json, read_private_json


def validate_settings(value):
    if (not isinstance(value, dict) or set(value) - {"enabled", "install_dir"}
            or type(value.get("enabled", False)) is not bool):
        raise ValueError("VIVE Hub 自动连接设置无效。")
    directory = value.get("install_dir", "")
    if (not isinstance(directory, str) or len(directory) > 4096
            or any(c in directory for c in ("\0", "\n", "\r"))):
        raise ValueError("请填写 VIVE Hub 安装目录。")
    if directory:
        path = Path(directory).expanduser()
        if not path.is_absolute():
            raise ValueError("VIVE Hub 安装目录必须为绝对路径。")
        directory = str(path)
    if value.get("enabled") and not directory:
        raise ValueError("启用自动连接前，请选择 VIVE Hub 安装目录。")
    return dict(enabled=value.get("enabled", False), install_dir=directory)


def hub_profile(configuration):
    groups = [g for g in configuration.get("process_groups", ())
              if g.get("enabled") and g.get("preset") == "trackers"]
    if len(groups) != 1 or groups[0].get("parameters", {}).get("backend") != "vivehub":
        raise ValueError("请在“配置设备”启用一个追踪器模块，并选择 VIVE Hub 官方 SDK 来源。")
    return groups[0]


def hub_ready(path, install_dir):
    """A stale socket file is not readiness; authenticate its listening process."""
    # VIVE Hub 1.0.3 uses packet-preserving Unix sockets, unlike manager IPC.
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
        connection.settimeout(.25)
        try:
            connection.connect(str(path))
        except (FileNotFoundError, ConnectionRefusedError):
            return False
        pid, uid, _ = struct.unpack("3i", connection.getsockopt(
            socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")))
        if uid != os.getuid() or Path(f"/proc/{pid}/exe").resolve() != (install_dir / "tracker_server").resolve():
            raise RuntimeError("Hub Socket 已由其他程序占用；请核对安装目录和 Socket 设置。")
    return True


def hub_processes(install_dir):
    """Recognize an official Hub still starting, without matching shell text."""
    executables = {(install_dir / name).resolve() for name in ("vivehub", "tracker_server")}
    found = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid == os.getuid() and (proc / "exe").resolve() in executables:
                found.append(int(proc.name))
        except (OSError, RuntimeError):
            continue
    return found


def legacy_reader_active():
    path = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")) / "vt_vive_tracker.lock"
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOFOLLOW)
    except FileNotFoundError:
        return False
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        return False
    finally:
        os.close(fd)


class ViveHubStartup:
    TIMEOUT_S = 35.
    POLL_S = .2

    def __init__(self, runtime):
        self.runtime = runtime
        self.path = runtime.settings_path.with_name("vivehub-startup.json")
        self.log_path = self.path.with_name("vivehub-startup.log")
        self.lock_path = self.path.with_name("vivehub-startup.lock")
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._process = None
        self._status = dict(state="idle", detail="VIVE Hub 自动连接尚未运行。")

    @property
    def settings(self):
        value = read_private_json(self.path) if self.path.exists() or self.path.is_symlink() else {}
        return validate_settings(value)

    @property
    def status(self):
        with self._lock:
            return deepcopy(self._status)

    def save(self, value):
        value = validate_settings(value)
        if value["enabled"]:
            hub_profile(self.runtime.configuration)
            self._installation(value)
        atomic_json(self.path, value)

    @staticmethod
    def _installation(settings):
        root = Path(settings["install_dir"])
        if not root.is_absolute() or any(not (root / name).is_file()
                                        for name in ("run-vivehub.sh", "vivehub", "tracker_server")):
            raise ValueError("所选目录缺少 run-vivehub.sh、vivehub 或 tracker_server，请选择完整安装目录。")
        if any(not os.access(root / name, os.X_OK) for name in ("vivehub", "tracker_server")):
            raise ValueError("VIVE Hub 程序没有执行权限。")
        return root

    def _update(self, state, detail):
        with self._lock:
            self._status = dict(state=state, detail=str(detail)[:2000])

    def start(self, *, automatic=False):
        with self._lock:
            if self._stop.is_set() or (self._thread and self._thread.is_alive()):
                return False
            try:
                settings = self.settings
                if automatic and not settings["enabled"]:
                    return False
                profile = hub_profile(self.runtime.configuration)
                root = self._installation(settings)
            except (OSError, ValueError) as exc:
                self._update("error", str(exc))
                return False
            self._update("connecting", "正在连接 VIVE Hub 和 Tracker 位姿桥…")
            self._thread = threading.Thread(target=self._run, args=(profile, root),
                                            name="desktop-vivehub-startup", daemon=True)
            self._thread.start()
            return True

    def _open_private(self, path):
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            os.close(fd)
            raise ValueError("Hub 启动日志和锁必须是当前用户的私有文件。")
        return fd

    def _launch_hub(self, root):
        if legacy_reader_active():
            raise RuntimeError("旧 PyVUT 读取器仍占用 Tracker；请先停止旧读取器，再连接 VIVE Hub。")
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("当前没有桌面显示连接，无法打开 VIVE Hub 窗口。")
        environment = dict(os.environ)
        for key in ("LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV",
                    "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"):
            environment.pop(key, None)
        fd = self._open_private(self.log_path)
        try:
            self._process = subprocess.Popen(
                ("/bin/bash", str(root / "run-vivehub.sh")), cwd=root, env=environment,
                stdin=subprocess.DEVNULL, stdout=fd, stderr=subprocess.STDOUT,
                close_fds=True, start_new_session=True)
        finally:
            os.close(fd)

    def _ensure_hub(self, profile, root):
        path = profile["parameters"]["hub_socket_path"]
        if hub_ready(path, root):
            return True
        if not hub_processes(root):
            if self._stop.is_set():
                return False
            self._launch_hub(root)
        deadline = time.monotonic() + self.TIMEOUT_S
        while not self._stop.is_set() and time.monotonic() < deadline:
            if hub_ready(path, root):
                return True
            if self._process and self._process.poll() is not None:
                raise RuntimeError(f"VIVE Hub 启动后退出，请查看日志：{self.log_path}")
            self._stop.wait(self.POLL_S)
        if not self._stop.is_set():
            raise RuntimeError(f"VIVE Hub 尚未就绪，请查看 Hub 窗口或日志：{self.log_path}")
        return False

    def _ensure_bridge(self, profile):
        deadline = time.monotonic() + self.TIMEOUT_S
        submitted = False
        expected = None
        while not self._stop.is_set() and time.monotonic() < deadline:
            snapshot = self._fresh_snapshot(deadline)
            if snapshot is None:
                return
            if expected is not None and snapshot["instance_id"] != expected:
                raise RuntimeError("工作台后台已更换；本次自动连接已停止，请重新连接。")
            expected = snapshot["instance_id"]
            group = next((g for g in snapshot.get("process_control", {}).get("groups", ())
                          if g["id"] == profile["id"]), None)
            if group is None:
                raise RuntimeError("当前后台没有对应的 Tracker 模块，请应用设备配置。")
            state = group["state"]
            if state in {"RUNNING", "EXTERNAL"} or group.get("owned"):
                devices = [d for d in snapshot.get("devices", ()) if d.get("topic") in group["topics"]]
                valid = sum(d.get("status") == "online" and (d.get("valid_hz") or 0) > 0 for d in devices)
                if valid == len(group["topics"]) and valid:
                    self._update("ready", f"VIVE Hub 已连接，工作台已收到 {valid} 路有效 Tracker 位姿。")
                    return
                self._update("waiting", "位姿桥已运行，正在等待 Tracker 有效位姿；请保持 Tracker 开机并在已建图区域内。")
            elif state == "STOPPED" and group.get("can_start") and not submitted:
                # Fresh manager admission includes recording/calibration and duplicate-publisher guards.
                if self._stop.is_set():
                    return
                request = dict(version=6, request_id=uuid.uuid4().hex, command="device.start",
                               params=dict(group_id=profile["id"], expected_manager_instance_id=expected))
                submitted = True  # An uncertain reply must never trigger a second start request.
                reply = _exchange(self.runtime.socket_path, request, timeout=3.)
                operation = reply.get("operation") or {}
                if not reply.get("ok") or operation.get("state") in {"rejected", "failed", "unknown"}:
                    raise RuntimeError("位姿桥启动请求未完成：" + str(operation.get("detail") or reply.get("error", "请查看设备页")))
            elif state in {"FAILED", "ERROR", "DISABLED", "STOPPING"}:
                raise RuntimeError("位姿桥未启动：" + group.get("detail", state))
            elif state == "STOPPED" and submitted:
                operation = snapshot.get("management", {}).get("operation") or {}
                if operation.get("state") in {"rejected", "failed"}:
                    raise RuntimeError("位姿桥未启动：" + str(operation.get("detail", "请查看设备页")))
            self._stop.wait(self.POLL_S)
        if not self._stop.is_set() and self.status["state"] != "waiting":
            raise RuntimeError("位姿桥暂不可启动，请在设备页检查状态后点击“立即连接”。")

    def _fresh_snapshot(self, deadline):
        # Read-only polls may retry during a busy frame; never retry a device command.
        while not self._stop.is_set():
            try:
                return self.runtime.fresh_snapshot()
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                self._stop.wait(self.POLL_S)
        return None

    def _run(self, profile, root):
        fd = None
        try:
            # Verify manager ownership before any Hub side effect.
            if self._fresh_snapshot(time.monotonic() + self.TIMEOUT_S) is None:
                return
            fd = self._open_private(self.lock_path)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("另一个工作台正在连接 VIVE Hub，请稍后查看设备状态。") from exc
            if self._ensure_hub(profile, root) and not self._stop.is_set():
                self._ensure_bridge(profile)
        except Exception as exc:
            self._update("error", str(exc))
        finally:
            if fd is not None:
                os.close(fd)
            try:
                result = dict(self.status, checked_at_ns=time.time_ns())
                atomic_json(self.path.with_name("vivehub-startup-status.json"), result)
                log = self._open_private(self.log_path)
                try:
                    os.write(log, (json.dumps(result, ensure_ascii=False) + "\n").encode())
                finally:
                    os.close(log)
            except (OSError, ValueError):
                pass

    def close(self):
        # Detach: both Hub and manager-owned sensors survive closing the GUI.
        self._stop.set()
