"""Shared exclusive serial access for capture and explicit protocol discovery.

Linux detects accessible /proc owners before and after opening, then retains
flock and TIOCEXCL until close. Windows uses pyserial's exclusive Win32 handle.
These checks cannot rule out privileged or inaccessible competing processes.

``passive=True`` forbids application writes, buffer resets and control-line
updates, and suppresses pyserial's POSIX open-time flush/DTR/RTS operations.
Windows retains native pyserial initialization (including its queue purge and
DCB configuration). Opening/configuring/closing a device can have driver or
hardware side effects on either platform; passive is not physical isolation.
"""
from __future__ import annotations

import errno
import math
from numbers import Integral, Real
import os
import stat
import sys


class PortInUseError(OSError):
    """The requested port already has another visible owner."""


def _port_owners(port: str, exclude_fd: int | None = None) -> list[str]:
    target = os.stat(port)
    if not stat.S_ISCHR(target.st_mode):
        raise ValueError("serial port must be a character device")
    owners = []
    own_pid = os.getpid()
    with os.scandir("/proc") as processes:
        for process in processes:
            if not process.name.isdecimal():
                continue
            try:
                with os.scandir(os.path.join(process.path, "fd")) as descriptors:
                    for descriptor in descriptors:
                        if int(process.name) == own_pid and descriptor.name == str(exclude_fd):
                            continue
                        try:
                            info = os.stat(descriptor.path)
                        except (FileNotFoundError, PermissionError, ProcessLookupError):
                            continue
                        if stat.S_ISCHR(info.st_mode) and info.st_rdev == target.st_rdev:
                            owners.append(f"pid={process.name},fd={descriptor.name}")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
    return owners


def _assert_port_free(port: str, exclude_fd: int | None = None) -> None:
    owners = _port_owners(port, exclude_fd)
    if owners:
        raise PortInUseError(errno.EBUSY, "serial port already in use: " + ", ".join(owners), port)


def _passive_class(base):
    """Use platform initialization while preventing extra destructive actions."""
    class PassiveSerial(base):
        def _reset_input_buffer(self):
            # POSIX open calls this private hook; Windows open purges natively.
            pass

        def _update_dtr_state(self):
            pass

        def _update_rts_state(self):
            pass

        def write(self, _data):
            raise RuntimeError("Protocol discovery cannot write serial data")

        def reset_input_buffer(self):
            raise RuntimeError("Protocol discovery cannot discard serial data")

        def reset_output_buffer(self):
            raise RuntimeError("Protocol discovery cannot discard serial data")

        def send_break(self, duration=0.25):
            raise RuntimeError("Protocol discovery cannot send serial breaks")

    return PassiveSerial


def open_serial(port: str, baudrate: int, read_timeout: float = 0.05, *, passive: bool = False):
    """Open one explicitly requested port; ownership errors never trigger retries."""
    if not isinstance(port, str) or not port.strip():
        raise ValueError("an explicit nonempty serial port is required")
    if isinstance(baudrate, bool) or not isinstance(baudrate, Integral) or baudrate <= 0:
        raise ValueError("baudrate must be a positive integer")
    if (isinstance(read_timeout, bool) or not isinstance(read_timeout, Real)
            or not math.isfinite(read_timeout) or read_timeout <= 0):
        raise ValueError("read_timeout must be finite and positive")
    if not isinstance(passive, bool):
        raise ValueError("passive must be a boolean")
    import serial

    options = dict(port=port, baudrate=int(baudrate), timeout=read_timeout,
                   write_timeout=read_timeout)
    if sys.platform != "win32":
        options["exclusive"] = True
    linux = sys.platform.startswith("linux")
    if linux:
        _assert_port_free(port)
    constructor = _passive_class(serial.Serial) if passive else serial.Serial
    device = constructor(**options)
    if linux:
        import fcntl
        import termios
        try:
            fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(device.fileno(), termios.TIOCEXCL)
            _assert_port_free(port, exclude_fd=device.fileno())
        except BaseException:
            close_serial(device)
            raise
    return device


def close_serial(device) -> None:
    """Release Linux exclusive mode and close, including after an open race."""
    if sys.platform.startswith("linux"):
        import fcntl
        import termios
        try:
            fcntl.ioctl(device.fileno(), termios.TIOCNXCL)
        except (OSError, ValueError):
            pass
    try:
        device.close()
    except (OSError, ValueError):
        pass
