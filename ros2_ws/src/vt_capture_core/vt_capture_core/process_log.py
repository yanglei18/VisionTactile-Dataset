"""Bound an existing append log for the lifetime of its writing process.

The caller owns stdout/stderr. This service neither redirects output nor starts
another process. It verifies the output inode once and retains that descriptor,
so renaming or replacing its pathname cannot redirect subsequent truncations.
"""
from __future__ import annotations

import math
import fcntl
import os
import stat
import threading


class ProcessLogRetention:
    """Periodically keep a bounded tail, independent of any desktop connection.

    This is retention, not a filesystem quota: concurrent writes may briefly
    exceed the threshold, and bytes written during truncation may be dropped.
    The existing append descriptor remains valid, including after ``close``.
    """
    def __init__(self, path, *, output_fd, limit_bytes=5 * 1024 * 1024,
                 keep_bytes=1024 * 1024, interval_s=.25):
        if (type(limit_bytes) is not int or type(keep_bytes) is not int
                or not 0 < keep_bytes < limit_bytes <= 64 * 1024 * 1024
                or type(interval_s) not in (int, float) or not math.isfinite(interval_s)
                or not .01 <= interval_s <= 30):
            raise ValueError("invalid process log retention bounds")
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK | os.O_NOFOLLOW)
        try:
            info, output = os.fstat(fd), os.fstat(output_fd)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077
                    or (info.st_dev, info.st_ino) != (output.st_dev, output.st_ino)
                    or not fcntl.fcntl(output_fd, fcntl.F_GETFL) & os.O_APPEND):
                raise ValueError("process log must be the owned private output file")
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd
        self._limit, self._keep, self._interval = limit_bytes, keep_bytes, interval_s
        self._stop = threading.Event()
        self._thread = None
        self.last_error = None

    def start(self):
        if self._thread is not None or self._stop.is_set():
            raise RuntimeError("process log retention is single-use")
        self._thread = threading.Thread(target=self._run, name="process-log-retention", daemon=True)
        self._thread.start()
        return self

    def _trim(self):
        size = os.fstat(self._fd).st_size
        if size <= self._limit:
            return
        tail = os.pread(self._fd, self._keep, max(0, size - self._keep))
        written = 0
        while written < len(tail):
            count = os.pwrite(self._fd, tail[written:], written)
            if count == 0:
                raise OSError("process log tail write made no progress")
            written += count
        os.ftruncate(self._fd, len(tail))

    def _run(self):
        while not self._stop.is_set():
            try:
                self._trim()
                self.last_error = None
            except OSError as exc:
                # A transient full/unavailable filesystem must not kill the
                # manager or create a feedback loop by logging to this file.
                self.last_error = str(exc)
            self._stop.wait(self._interval)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.)
            if self._thread.is_alive():
                raise RuntimeError("process log retention did not stop")
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()
