"""Asynchronous desktop-owned manager lifecycle, without Qt, ROS or hardware IO.

The fixed deployment script is the only executable recipe. Sensors are never
started here. Closing a window detaches; restart requires verified ownership,
a fresh idle snapshot, and a cross-process manager admission lock.
"""
from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
import fcntl
import json
import os
from pathlib import Path
import queue
import signal
import socket
import stat
import struct
import subprocess
import threading
import time
import uuid

from vt_capture_core.protocol import MAX_MESSAGE_BYTES, decode_message, encode_message
from .workstation_settings import (WorkstationSettings, atomic_json, private_directory,
                                   read_private_json, serial_ports, validate_configuration)


class _UnsupportedManagerProtocol(ValueError):
    """Only an explicit protocol mismatch permits a legacy probe."""


def _identity(pid):
    try:
        text = Path(f"/proc/{pid}/stat").read_text()
        fields = text[text.rindex(")") + 2:].split()
        if fields[0] in {"Z", "X"}:
            return None
        return dict(pid=int(pid), pgid=int(fields[2]), session=int(fields[3]), start_time=int(fields[19]))
    except (FileNotFoundError, ProcessLookupError):
        return None


def _process_groups(pid):
    """Inspect effective credentials, not the possibly newer account database."""
    groups, effective_gid = None, None
    for line in Path(f"/proc/{pid}/status").read_text().splitlines():
        if line.startswith("Groups:"):
            groups = {int(value) for value in line.split()[1:]}
        elif line.startswith("Gid:"):
            effective_gid = int(line.split()[2])
    if groups is None or effective_gid is None:
        raise RuntimeError("无法核对后台进程的实际串口组权限，未重启后台。")
    return groups | {effective_gid}


def _snapshot_problem(snapshot):
    if not isinstance(snapshot, dict) or snapshot.get("mode") != "live":
        return "需要真实后台的新鲜状态，不能依据离线或演示状态重启。"
    if snapshot.get("calibration", {}).get("busy"):
        return "标定任务正在运行，请先取消或等待完成后再应用设备配置。"
    management = snapshot.get("management")
    process_control = snapshot.get("process_control")
    if not isinstance(management, dict) or not isinstance(process_control, dict):
        return "后台未提供录制与设备生命周期状态，不能安全应用配置。"
    if any(trial.get("state") in {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"}
           for trial in management.get("trials", ())):
        return "存在录制中、过渡中或结果未确认的试次；请先完成或核对试次。"
    controller = management.get("controller")
    if controller and (controller.get("recorder_alive") or controller.get("state") in {1, 2, 3, 4}):
        return "录制仍在进行或切换中；请先停止录制。"
    if snapshot.get("recording", {}).get("state") == "RECORDING":
        return "录制仍在进行；请先停止录制。"
    for group in process_control.get("groups", ()):
        if group.get("owned") or group.get("state") in {"STARTING", "STOPPING"}:
            return "请先停止所有由工作台启动的设备，并等待设备命令完成后再应用配置。"
    operation = management.get("operation")
    if operation and operation.get("code") == "queued" and operation.get("state") in {"accepted", "unknown"}:
        return "后台仍有待处理命令；请等待完成后再应用配置。"
    return None


class DesktopRuntime:
    STARTUP_TIMEOUT_S = 15.
    STOP_TIMEOUT_S = 4.
    POLL_INTERVAL_S = .25
    AUXILIARY_SOCKET_NAMES = ("manager-scene.sock", "manager-cloud.sock")

    def __init__(self, repo_root=None, settings_dir=None):
        self.settings = WorkstationSettings(repo_root, settings_dir)
        self.repo_root = self.settings.repo_root
        self.settings_path = self.settings.path
        self.socket_path = self.settings.directory / "private-desktop.sock"
        self.log_path = self.settings.directory / "manager.log"
        self.state_dir = private_directory(self.settings.directory / "manager-state")
        self.apply_lock_path = self.state_dir / ".desktop-apply.lock"
        self.admission_path = self.apply_lock_path
        self._registry_path = self.settings.directory / "desktop-manager.json"
        self._operation_lock_path = self.settings.directory / "desktop-operation.lock"
        self._boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self._boot_started_ns = next(int(line.split()[1]) * 1_000_000_000
                                     for line in Path("/proc/stat").read_text().splitlines()
                                     if line.startswith("btime "))
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._tasks = queue.Queue(maxsize=1)
        self._thread = None
        self._process = None
        self._status = dict(state="stopped", detail="本地后台尚未启动。", last_apply=dict(state="idle", detail=""))

    @property
    def configuration(self):
        with self._lock:
            return self.settings.configuration

    @property
    def status(self):
        with self._lock:
            return deepcopy(self._status)

    def serial_ports(self):
        return serial_ports()

    def detect_environment(self, configuration):
        """Inspect the local filesystem without applying settings or touching hardware."""
        return self.settings.detect_environment(configuration)

    def fresh_snapshot(self):
        """Worker-only fresh ownership-checked snapshot; no cached UI evidence."""
        snapshot, pid = self._probe()
        if not self._is_owned(self._record(), snapshot, pid):
            raise RuntimeError("后台不属于本桌面运行时，不能执行本机配置或初始化操作。")
        return snapshot

    def _status_update(self, state, detail, apply_state=None):
        with self._lock:
            if self._stop.is_set():
                return
            self._status.update(state=state, detail=str(detail)[:2000])
            if apply_state is not None:
                self._status["last_apply"] = dict(state=apply_state, detail=str(detail)[:2000])

    def start(self):
        with self._lock:
            if self._stop.is_set():
                raise RuntimeError("已关闭的桌面运行时不能重新使用。")
            if self._status["state"] in {"starting", "applying", "ready"} or not self._tasks.empty():
                return
            self._status_update("starting", "正在连接或启动本地后台；不会自动启动任何设备。")
            self._tasks.put_nowait(("start", None, None))
            if self._thread is not None and self._thread.is_alive():
                return  # An explicit retry, never an automatic restart loop.
            self._thread = threading.Thread(target=self._run, name="desktop-manager-runtime", daemon=True)
            self._thread.start()

    def apply(self, configuration, snapshot):
        """Validate and enqueue once. Never wait on IPC, processes or disk here."""
        with self._lock:
            if self._status["state"] in {"starting", "applying"}:
                return False
        try:
            value = validate_configuration(configuration)
        except (ValueError, TypeError, UnicodeError) as exc:
            self._status_update("error", f"配置未保存：{exc}", "rejected")
            return False
        with self._lock:
            if self._stop.is_set() or self._thread is None:
                self._status_update("error", "后台运行时尚未启动或已关闭，配置未保存。", "rejected")
                return False
            if self._status["state"] in {"starting", "applying"} or not self._tasks.empty():
                return False
            if snapshot:
                problem = _snapshot_problem(snapshot)
                if problem:
                    self._status_update("error", problem, "rejected")
                    return False
            self._status_update("applying", "正在核对后台状态并保存配置；设备不会自动启动。", "pending")
            self._tasks.put_nowait(("apply", value, deepcopy(snapshot) if snapshot else None))
            return True

    @contextmanager
    def _file_lock(self, path):
        fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
                raise ValueError("拒绝使用非私有的后台控制锁。")
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("另一个工作台正在处理后台或配置，请稍后重试。") from exc
            yield
        finally:
            os.close(fd)

    def _record(self):
        if not self._registry_path.exists() and not self._registry_path.is_symlink():
            return None
        value = read_private_json(self._registry_path)
        if not isinstance(value, dict) or value.get("version") != 1 or value.get("socket_path") != str(self.socket_path):
            raise ValueError("后台所有权记录无效；不会接管或重复启动未知进程。")
        try:
            if not isinstance(value.get("boot_id"), str):
                raise ValueError
            uuid.UUID(value["boot_id"])
        except ValueError as exc:
            raise ValueError("后台启动会话记录无效；不会清理已有连接。") from exc
        identity = value.get("identity")
        if identity is not None and (not isinstance(identity, dict)
                or any(type(identity.get(k)) is not int or identity[k] <= 0 for k in ("pid", "pgid", "session", "start_time"))
                or identity["pgid"] != identity["pid"] or identity["session"] != identity["pid"]):
            raise ValueError("后台进程身份记录无效。")
        if "socket_inode" in value and (type(value["socket_inode"]) is not int or value["socket_inode"] <= 0):
            raise ValueError("后台 socket 身份记录无效。")
        auxiliary = value.get("auxiliary_socket_inodes", {})
        if (not isinstance(auxiliary, dict) or any(name not in self.AUXILIARY_SOCKET_NAMES
                or type(inode) is not int or inode <= 0 for name, inode in auxiliary.items())):
            raise ValueError("后台空间 socket 身份记录无效。")
        return value

    def _record_alive(self, record):
        # A PID and even its start tick can recur after reboot. Keep the old
        # registry only as evidence for its socket, never as process ownership.
        if record is None or record.get("boot_id") != self._boot_id:
            return False
        identity = record.get("identity")
        if identity is None:
            if record.get("pending_launch"):
                raise RuntimeError("上次后台启动结果不明确；不会自动接管或重复启动，请检查后台日志。")
            return False
        return _identity(identity["pid"]) == identity

    def _probe(self):
        """Probe calibration-aware admission, retaining explicit old-manager support."""
        try:
            return self._probe_version(6)
        except _UnsupportedManagerProtocol:
            return self._probe_version(4)

    def _probe_version(self, version):
        """One bounded request, authenticated to the peer PID on that socket."""
        request = dict(version=version, request_id=uuid.uuid4().hex, command="snapshot", params={})
        deadline = time.monotonic() + .6
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(.6)
            connection.connect(str(self.socket_path))
            pid, uid, _ = struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")))
            if uid != os.getuid():
                raise PermissionError("后台 IPC 不属于当前用户。")
            connection.sendall(encode_message(request))
            data = bytearray()
            while b"\n" not in data:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("后台快照超时。")
                connection.settimeout(remaining)
                chunk = connection.recv(min(65536, MAX_MESSAGE_BYTES + 1 - len(data)))
                if not chunk:
                    raise ConnectionError("后台未返回完整快照。")
                data.extend(chunk)
                if len(data) > MAX_MESSAGE_BYTES:
                    raise ValueError("后台消息超过限制。")
            line, trailing = bytes(data).split(b"\n", 1)
            if trailing:
                raise ValueError("后台返回多个消息帧。")
            reply = decode_message(line + b"\n")
            if reply.get("version") != version:
                raise _UnsupportedManagerProtocol("后台不支持所需的工作站协议。")
            if reply.get("request_id") != request["request_id"] or not reply.get("ok"):
                raise ValueError("后台不支持所需的工作站协议。")
            snapshot = reply["snapshot"]
            if snapshot.get("mode") != "live":
                raise ValueError("真实工作站拒绝演示后台。")
            return snapshot, pid

    def _is_owned(self, record, snapshot, pid):
        return bool(record and self._record_alive(record) and record["identity"]["pid"] == pid
                    and record.get("manager_instance_id", "") in ("", snapshot["instance_id"]))

    def _remember_ready(self, record, snapshot):
        record = deepcopy(record)
        record["manager_instance_id"] = snapshot["instance_id"]
        try:
            record["socket_inode"] = self.socket_path.lstat().st_ino
        except FileNotFoundError:
            raise RuntimeError("后台连接路径已变化，请重新连接。")
        record["auxiliary_socket_inodes"] = self._auxiliary_socket_inodes(record)
        atomic_json(self._registry_path, record)

    def _auxiliary_socket_inodes(self, record):
        """Record only endpoints authenticated to the live manager process group."""
        known = dict(record.get("auxiliary_socket_inodes", {}))
        if not self._record_alive(record):
            return known
        identity = record["identity"]
        for name in self.AUXILIARY_SOCKET_NAMES:
            path = self.socket_path.with_name(name)
            try:
                info = path.lstat()
                if (not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid()
                        or known.get(name) == info.st_ino):
                    continue
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                    connection.settimeout(.2)
                    connection.connect(str(path))
                    pid, uid, _ = struct.unpack("3i", connection.getsockopt(
                        socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")))
                    owned = pid == identity["pid"] if name == "manager-scene.sock" else os.getpgid(pid) == identity["pgid"]
                    if uid != os.getuid() or not owned:
                        continue
                fresh = path.lstat()
                if (self._record_alive(record) and stat.S_ISSOCK(fresh.st_mode)
                        and (fresh.st_dev, fresh.st_ino, fresh.st_uid) == (info.st_dev, info.st_ino, info.st_uid)):
                    known[name] = info.st_ino
            except OSError:
                continue  # Endpoints may appear after the control socket is ready.
        return known

    def _clean_owned_socket(self, record):
        self._clean_socket_path(self.socket_path, record, record.get("socket_inode") if record else None)
        for name in self.AUXILIARY_SOCKET_NAMES:
            path = self.socket_path.with_name(name)
            try:
                info = path.lstat()
            except FileNotFoundError:
                continue
            private_directory(path.parent)
            inode = record.get("auxiliary_socket_inodes", {}).get(name) if record else None
            if inode is None:
                # Older registries tracked only the control socket. Recover
                # those exact companion names only if the filesystem proves
                # they predate this boot; ctime prevents a backdated mtime alone.
                if (not record or info.st_mtime_ns >= self._boot_started_ns
                        or info.st_ctime_ns >= self._boot_started_ns):
                    raise RuntimeError("空间 socket 缺少所有权记录且无法确认来自上次启动；未清理已有连接。")
                inode = info.st_ino
            self._clean_socket_path(path, record, inode)

    def _remember_late_auxiliary_sockets(self, record):
        auxiliary = self._auxiliary_socket_inodes(record)
        if auxiliary == record.get("auxiliary_socket_inodes", {}):
            return
        try:
            with self._file_lock(self._operation_lock_path):
                # Another GUI may have replaced the manager while probing.
                if self._record() == record and self._record_alive(record):
                    updated = dict(record, auxiliary_socket_inodes=auxiliary)
                    atomic_json(self._registry_path, updated)
        except RuntimeError:
            return  # An explicit configuration operation has priority.

    def _clean_socket_path(self, path, record, inode):
        if not path.exists() and not path.is_symlink():
            return
        info = path.lstat()
        if (not record or self._record_alive(record) or not stat.S_ISSOCK(info.st_mode)
                or info.st_uid != os.getuid() or info.st_ino != inode):
            raise RuntimeError("后台 socket 仍存在且无法确认安全清理；不会覆盖外部后台。")
        # A failed snapshot is insufficient: an external listener may speak a
        # different protocol or be temporarily unresponsive. Only an explicit
        # refusal proves this exact path has no listener, including after reboot.
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(.2)
            try:
                connection.connect(str(path))
            except ConnectionRefusedError:
                pass
            except OSError as exc:
                raise RuntimeError("无法确认后台 socket 已停止监听；未清理已有连接。") from exc
            else:
                raise RuntimeError("后台 socket 仍有程序监听；不会覆盖外部后台。")
        # The operation lock prevents another desktop client racing the unlink;
        # never delete a changed inode or any ordinary file/symlink.
        fresh = path.lstat()
        if ((fresh.st_dev, fresh.st_ino, fresh.st_uid, fresh.st_mode)
                != (info.st_dev, info.st_ino, info.st_uid, info.st_mode)
                or self._record_alive(record)):
            raise RuntimeError("后台 socket 身份已变化；未清理已有连接。")
        path.unlink()

    def _manager_argv(self):
        script = self.repo_root / "tools/capture_deploy/start_manager.sh"
        if not script.is_file():
            raise ValueError("缺少仓库后台启动脚本，请修复桌面工作站安装。")
        return ("/bin/bash", str(script), "--enable-device-control", "--enable-recording",
                "--enable-calibration", "--calibration-python", str(self.repo_root / ".venv-calibration/bin/python"),
                "--config", str(self.settings.path), "--socket", str(self.socket_path),
                "--state-dir", str(self.state_dir), "--desktop-apply-lock", str(self.apply_lock_path),
                "--log-path", str(self.log_path))

    def _session_status(self):
        """Bounded worker-only account/session check; never opens a device."""
        script = self.repo_root / "tools/capture_deploy/serial_session.py"
        if not script.is_file():
            raise RuntimeError("缺少工作台串口权限检查组件，请修复本机部署。")
        result = subprocess.run(("/usr/bin/python3", "-I", str(script), "check"),
                                stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=3.)
        if result.returncode != 0 or len(result.stdout) > 16384:
            raise RuntimeError("无法检查串口会话权限，请查看后台日志并从桌面入口重新打开。")
        status = json.loads(result.stdout)
        if (not isinstance(status, dict)
                or any(type(status.get(key)) is not bool for key in
                       ("account_has_group", "process_has_group", "needs_refresh"))
                or (status.get("gid") is not None and type(status["gid"]) is not int)):
            raise RuntimeError("串口权限检查返回无效状态，未启动后台。")
        return status

    def _check_launch_session(self):
        if self._session_status()["needs_refresh"]:
            raise RuntimeError("当前窗口尚未继承账户已有的串口权限。请关闭并从桌面快捷方式重新打开工作台；"
                               "桌面入口会刷新权限，无需重装驱动或修改串口权限。")

    def _repair_idle_backend_session(self, record, pid):
        """Replace only an idle, owned stale-session manager, never its devices.

        Called with the desktop operation lock held. A newly opened GUI has
        already refreshed its authorized groups via the desktop entry; merely
        reusing the previous manager would otherwise retain the old denial.
        """
        session = self._session_status()
        if not session["account_has_group"] or session["gid"] in _process_groups(pid):
            return False
        if session["needs_refresh"]:
            self._status_update("permission_required", "旧后台与当前窗口尚未继承串口权限。请先停止相关设备，"
                                "关闭并从桌面快捷方式重新打开工作台以刷新权限。")
            return True
        with self._file_lock(self.admission_path):
            fresh, peer = self._probe()
            if not self._is_owned(record, fresh, peer) or peer != pid:
                raise RuntimeError("后台身份已变化，未执行串口权限恢复，请重试连接。")
            problem = _snapshot_problem(fresh)
            if problem:
                self._status_update("permission_required", "后台仍使用旧串口权限。" + problem + "随后点击“重试连接”恢复。")
                return True
            if self._stop_owned(record) and self._launch():
                self._status_update("ready", "已刷新空闲后台的串口权限；未自动启动设备，请手动启动需要的模块。")
        return True

    def _launch(self):
        if self._stop.is_set():
            return False
        self._check_launch_session()
        argv = self._manager_argv()
        record = dict(version=1, boot_id=self._boot_id, socket_path=str(self.socket_path), identity=None,
                      pending_launch=True, manager_instance_id="", nonce=uuid.uuid4().hex)
        fd = os.open(self.log_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NONBLOCK | os.O_NOFOLLOW, 0o600)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
                raise ValueError("后台日志必须为当前用户的私有普通文件。")
            # Persist possible-launch uncertainty only after local log setup,
            # and before the first external process side effect.
            atomic_json(self._registry_path, record)
            try:
                process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=fd, stderr=subprocess.STDOUT,
                                           cwd=self.repo_root, close_fds=True, start_new_session=True)
            except Exception:
                self._registry_path.unlink(missing_ok=True)
                raise
        finally:
            os.close(fd)
        self._process = process
        identity = _identity(process.pid)
        if identity is None:
            process.poll()
            self._registry_path.unlink(missing_ok=True)
            raise RuntimeError(f"后台启动后立即退出，请查看日志：{self.log_path}")
        record.update(identity=identity, pending_launch=False)
        atomic_json(self._registry_path, record)
        return self._wait_ready(record)

    def _wait_ready(self, record):
        deadline = time.monotonic() + self.STARTUP_TIMEOUT_S
        while not self._stop.is_set() and time.monotonic() < deadline:
            if self._process is not None:
                self._process.poll()
            if not self._record_alive(record):
                raise RuntimeError(f"后台已退出；请在工作台打开后台日志：{self.log_path}")
            try:
                snapshot, pid = self._probe()
            except (OSError, ValueError, KeyError):
                self._stop.wait(.1)
                continue
            if not self._is_owned(record, snapshot, pid):
                raise RuntimeError("后台身份与本次启动不一致；不会接管或重试。")
            self._remember_ready(record, snapshot)
            return True
        if not self._stop.is_set():
            raise RuntimeError("后台启动超时；进程未被自动终止或重试，请查看后台日志。")
        return False

    def _start_worker(self):
        with self._file_lock(self._operation_lock_path):
            record = self._record()
            try:
                snapshot, pid = self._probe()
            except (OSError, ValueError, KeyError):
                if self._record_alive(record):
                    if self._wait_ready(record):
                        snapshot, pid = self._probe()
                        if not self._is_owned(record, snapshot, pid):
                            raise RuntimeError("后台身份已变化，未执行串口权限恢复，请重试连接。")
                        if self._repair_idle_backend_session(record, pid):
                            return
                        self._status_update("ready", "已连接原有工作台后台；设备不会自动启动。")
                    return
                self._clean_owned_socket(record)
                # The task queue serializes configuration writers. Settings
                # publishes a new value only after atomic persistence, so UI
                # reads keep the previous complete value during slow discovery.
                self.settings.prepare_for_startup()
                if self._launch():
                    self._status_update("ready", "本地路径已自动准备，后台已就绪；可启动需要的设备。")
            else:
                if self._is_owned(record, snapshot, pid):
                    self._remember_ready(record, snapshot)
                    if self._repair_idle_backend_session(record, pid):
                        return
                    self._status_update("ready", "已复用原有工作台后台；未重复启动。")
                else:
                    self._status_update("external", "已连接已有后台；它不属于本桌面运行时，不能通过配置窗口重启。")

    def _signal_owned(self, record, sig):
        identity = record["identity"]
        if not self._record_alive(record):
            return
        # pidfds make signal delivery safe even if the numerical PID is reused
        # immediately after the final birth-identity check.
        if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
            raise RuntimeError("系统缺少安全的 pidfd 信号支持；未终止任何后台。")
        fd = os.pidfd_open(identity["pid"])
        try:
            if self._record_alive(record):
                signal.pidfd_send_signal(fd, sig)
        finally:
            os.close(fd)

    def _stop_owned(self, record):
        for sig in (signal.SIGINT, signal.SIGTERM):
            if self._stop.is_set():
                return False
            self._signal_owned(record, sig)
            deadline = time.monotonic() + self.STOP_TIMEOUT_S
            while time.monotonic() < deadline:
                if self._process is not None:
                    self._process.poll()
                if not self._record_alive(record):
                    self._clean_owned_socket(record)
                    return True
                if self._stop.wait(.05):
                    return False
        raise RuntimeError("后台没有在限定时间退出；未强制杀死或重复启动，请查看日志。")

    def _apply_worker(self, configuration, submitted_snapshot):
        with self._file_lock(self._operation_lock_path), self._file_lock(self.apply_lock_path):
            # Detect an old GUI session before saving configuration or stopping
            # the existing manager. A failed relaunch must not take it offline.
            self._check_launch_session()
            record = self._record()
            try:
                snapshot, pid = self._probe()
            except (OSError, ValueError, KeyError) as exc:
                if self._record_alive(record):
                    raise RuntimeError("后台仍存活但状态不可用；配置未保存，不能安全重启。") from exc
                self._clean_owned_socket(record)
            else:
                if not self._is_owned(record, snapshot, pid):
                    raise RuntimeError("已有后台不属于本桌面运行时；配置未保存，不会接管或终止它。")
                if submitted_snapshot and submitted_snapshot.get("instance_id") != snapshot["instance_id"]:
                    raise RuntimeError("后台实例已变化；请刷新后重新应用配置。")
                problem = _snapshot_problem(snapshot)
                if problem:
                    raise RuntimeError(problem)
                with self._lock:
                    # Retain any normalized serial identity across the backend
                    # restart; a reused tty number must not rebind this apply.
                    configuration = self.settings.save(configuration)
                if not self._stop_owned(record):
                    return
            if self._stop.is_set():
                return
            with self._lock:
                self.settings.save(configuration)
            if self._launch():
                self._status_update("ready", "配置已保存并应用；设备仍需手动启动，未自动打开任何硬件。", "applied")

    def _run(self):
        while not self._stop.is_set():
            try:
                action, configuration, snapshot = self._tasks.get(timeout=self.POLL_INTERVAL_S)
            except queue.Empty:
                with self._lock:
                    monitor = self._status["state"] == "ready"
                if monitor:
                    try:
                        record = self._record()
                        if self._process is not None:
                            self._process.poll()
                        if not self._record_alive(record):
                            self._status_update("error", "本地后台已退出；未自动重启，请查看后台日志。")
                        else:
                            self._remember_late_auxiliary_sockets(record)
                    except Exception as exc:
                        self._status_update("error", str(exc))
                continue
            try:
                if action == "start":
                    self._start_worker()
                else:
                    self._apply_worker(configuration, snapshot)
            except Exception as exc:
                self._status_update("error", str(exc), "error" if action == "apply" else None)
            finally:
                self._tasks.task_done()

    def close(self):
        # Never wait for or signal a backend process merely because the GUI
        # closes. In-flight explicit apply may already have sent a stop; its
        # worker sees this event before any next lifecycle action.
        self._stop.set()
        with self._lock:
            self._status.update(state="closed", detail="窗口已断开；后台与设备未被自动停止。")
