"""Linux desktop admission and activation, without importing Qt or opening devices."""
from __future__ import annotations

import fcntl
import hashlib
import os
from pathlib import Path
import socket
import stat
import struct
import sys
import threading
import time

from .backend.workstation_settings import private_directory


class DesktopSingleInstance:
    REQUEST = b"VT_CAPTURE_ACTIVATE/1\n"
    RESPONSE = b"VT_CAPTURE_ACTIVATE/1 OK\n"

    def __init__(self, directory=None):
        self.directory = private_directory(directory or Path.home() / ".config/visiontactile/workbench")
        key = hashlib.sha256(os.fsencode(self.directory.resolve())).hexdigest()[:32]
        # Linux releases abstract sockets on process exit; there is no stale
        # socket file to delete after a crash or reboot.
        self.address = f"\0vt-capture-desktop-{os.getuid()}-{key}"
        self.lock_path = self.directory / ".desktop-ui.lock"
        self._fd = None
        self._listener = None
        self._thread = None
        self._stop = threading.Event()
        self._activation = threading.Event()

    def acquire_or_activate(self, timeout=3.):
        """Return True for the primary, False after an existing GUI acknowledges."""
        if sys.platform != "linux":
            raise RuntimeError("桌面单实例启动需要 Linux 本地会话。")
        if self._fd is not None or self._stop.is_set():
            raise RuntimeError("桌面启动锁不能重复使用。")
        fd = os.open(self.lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        self._fd = fd
        info = os.fstat(fd)
        current = self.lock_path.lstat()
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077
                or info.st_nlink != 1 or (info.st_dev, info.st_ino) != (current.st_dev, current.st_ino)):
            self.close()
            raise RuntimeError("桌面启动锁必须是当前用户的私有普通文件。")
        deadline = time.monotonic() + timeout
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                try:
                    self._activate_existing(max(.01, min(.25, deadline - time.monotonic())))
                except (OSError, RuntimeError):
                    if time.monotonic() >= deadline:
                        self.close()
                        raise RuntimeError("已有工作台正在启动或暂时无法响应；未重复打开窗口，请稍后再试。")
                    self._stop.wait(.05)
                    continue
                self.close()
                return False
            try:
                listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self._listener = listener
                listener.bind(self.address)
                listener.listen(8)
                listener.settimeout(.2)
                self._thread = threading.Thread(target=self._serve, name="desktop-window-activation", daemon=True)
                self._thread.start()
                return True
            except BaseException:
                self.close()
                raise

    @staticmethod
    def _same_user(connection):
        return struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1] == os.getuid()

    @staticmethod
    def _read_exact(connection, expected, timeout=.25):
        data = bytearray()
        deadline = time.monotonic() + timeout
        while len(data) < len(expected):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("window activation message timed out")
            connection.settimeout(remaining)
            chunk = connection.recv(len(expected) + 1 - len(data))
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data) == expected

    def _activate_existing(self, timeout):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout)
            connection.connect(self.address)
            if not self._same_user(connection):
                raise RuntimeError("已有窗口不属于当前用户。")
            connection.sendall(self.REQUEST)
            if not self._read_exact(connection, self.RESPONSE, timeout):
                raise RuntimeError("已有窗口未确认激活请求。")

    def _serve(self):
        while not self._stop.is_set():
            try:
                connection, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with connection:
                try:
                    connection.settimeout(.25)
                    if self._same_user(connection) and self._read_exact(connection, self.REQUEST):
                        self._activation.set()
                        connection.sendall(self.RESPONSE)
                except OSError:
                    continue

    def consume_activation(self):
        if self._activation.is_set():
            self._activation.clear()
            return True
        return False

    def close(self):
        self._stop.set()
        if self._listener is not None:
            self._listener.close()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=.6)
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
