"""Small, bounded local datagram receiver independent of rclpy and GUI libraries."""
from __future__ import annotations

import errno
import fcntl
import os
from pathlib import Path
import socket
import stat


MAX_PACKET_BYTES = 60_000


def default_socket_path(hand: str = "right") -> str:
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    return f"/tmp/vt-glove-{os.getuid()}/{hand}.sock"


class LocalReceiver:
    """Own one private local endpoint; never take over an active listener."""

    def __init__(self, path: str):
        self.path = Path(path)
        self.socket = None
        self._lock_fd = None
        self._bound_identity = None
        if not self.path.is_absolute() or len(os.fsencode(self.path)) > 107:
            raise ValueError("socket_path must be absolute and at most 107 bytes")
        parent = self.path.parent
        parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        info = parent.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
                or info.st_mode & 0o077):
            raise ValueError("socket parent must be an owned private directory (chmod 700)")
        try:
            self._lock_fd = os.open(str(self.path) + ".lock",
                                    os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            lock_info = os.fstat(self._lock_fd)
            if not stat.S_ISREG(lock_info.st_mode) or lock_info.st_uid != os.getuid():
                raise ValueError("socket lock must be an owned regular file")
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._remove_stale_socket()
            self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.socket.setblocking(False)
            self.socket.bind(str(self.path))
            info = self.path.lstat()
            self._bound_identity = (info.st_dev, info.st_ino)
            os.chmod(self.path, 0o600)
        except BaseException:
            self.close()
            raise

    def _remove_stale_socket(self):
        try:
            info = self.path.lstat()
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
            raise ValueError("refusing to replace a non-socket or unowned socket path")
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            probe.connect(str(self.path))
        except OSError as exc:
            if exc.errno not in (errno.ECONNREFUSED, errno.ENOENT):
                raise
        else:
            raise ValueError("another receiver already owns this socket")
        finally:
            probe.close()
        # Directory is private and lock held; still avoid deleting a replaced inode.
        try:
            current = self.path.lstat()
            if (current.st_dev, current.st_ino) != (info.st_dev, info.st_ino):
                raise ValueError("socket changed while checking its owner")
            self.path.unlink()
        except FileNotFoundError:
            pass

    def receive(self):
        """Return next bytes or None. Oversized/truncated packets are never accepted."""
        try:
            payload, _ancillary, flags, _address = self.socket.recvmsg(MAX_PACKET_BYTES + 1)
        except BlockingIOError:
            return None
        if flags & socket.MSG_TRUNC or len(payload) > MAX_PACKET_BYTES:
            raise ValueError("glove datagram exceeds 60000 bytes")
        return payload

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        if self._bound_identity is not None:
            try:
                info = self.path.lstat()
                if (info.st_dev, info.st_ino) == self._bound_identity:
                    self.path.unlink()
            except FileNotFoundError:
                pass
            self._bound_identity = None
        if self._lock_fd is not None:
            os.close(self._lock_fd)
            self._lock_fd = None
