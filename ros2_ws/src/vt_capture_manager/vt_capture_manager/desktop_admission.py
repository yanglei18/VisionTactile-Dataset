"""Coordinate explicit desktop configuration apply with command admission.

The desktop holds an exclusive lock while observing idle state and replacing
its own backend. Read-only IPC remains available; new mutations fail promptly.
"""
import fcntl
import os
import stat
from pathlib import Path

from .storage import private_directory


class DesktopAdmission:
    def __init__(self, path):
        path = Path(path)
        private_directory(path.parent)
        self.fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        info = os.fstat(self.fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            os.close(self.fd)
            raise ValueError("desktop admission lock must be an owned private regular file")

    def acquire(self):
        try:
            fcntl.flock(self.fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
            return True
        except BlockingIOError:
            return False

    def release(self):
        fcntl.flock(self.fd, fcntl.LOCK_UN)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
