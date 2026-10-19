"""Host receive clocks shared by tactile acquisition and local presentation.

CPython <= 3.12 uses coarse Windows clocks for both ``time_ns`` and
``monotonic_ns`` (GetSystemTimeAsFileTime and GetTickCount64). Nanosecond units
do not make those clocks precise enough for millisecond serial reads. Use
precise UTC and the system-wide QPC counter on Windows; keep Linux's existing
time/CLOCK_MONOTONIC behavior. Never synthesize UTC from a fixed offset, clamp
timestamps, or hide real clock adjustments.

References:
https://docs.python.org/3.13/library/time.html#time.time
https://github.com/python/cpython/blob/v3.12.0/Python/pytime.c
https://learn.microsoft.com/en-us/windows/win32/sysinfo/acquiring-high-resolution-time-stamps
"""
from functools import lru_cache
import sys
import time


_FILETIME_UNIX_EPOCH_TICKS = 116_444_736_000_000_000


@lru_cache(maxsize=1)
def _windows_utc_reader():
    """Bind once, allocating a separate FILETIME for each thread/call.

    The API is available since Windows 8. Failure to bind is an installation
    error; falling back to a coarse clock would make valid samples unreliable.
    """
    import ctypes

    class FileTime(ctypes.Structure):
        # DWORD is always 32 bits, including 64-bit Windows. Fixed-width types
        # also let the conversion/ABI tests run on non-Windows build hosts.
        _fields_ = [("dwLowDateTime", ctypes.c_uint32),
                    ("dwHighDateTime", ctypes.c_uint32)]

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    read_filetime = kernel32.GetSystemTimePreciseAsFileTime
    read_filetime.argtypes = [ctypes.POINTER(FileTime)]
    read_filetime.restype = None

    def read():
        stamp = FileTime()
        read_filetime(ctypes.byref(stamp))
        ticks = (stamp.dwHighDateTime << 32) | stamp.dwLowDateTime
        return (ticks - _FILETIME_UNIX_EPOCH_TICKS) * 100

    return read


def realtime_ns() -> int:
    """Return precise UTC nanoseconds since the Unix epoch, without adjustment."""
    if sys.platform == "win32" and sys.version_info < (3, 13):
        return _windows_utc_reader()()
    return time.time_ns()


def monotonic_ns() -> int:
    """Return the local tactile monotonic domain; all local consumers use this.

    Windows QPC is shared across processes and unaffected by UTC corrections.
    CPython >= 3.13 also uses QPC for its native monotonic clock, while older
    supported versions expose it through perf_counter_ns instead.
    """
    if sys.platform == "win32":
        return time.perf_counter_ns()
    return time.monotonic_ns()
