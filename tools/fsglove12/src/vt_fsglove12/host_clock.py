"""One host monotonic domain for acquisition, solving, IPC and display.

Windows uses the system-wide QPC counter, including on Python versions whose
native monotonic clock is coarse. Linux keeps its existing CLOCK_MONOTONIC
behavior. UTC timestamps and sleeps remain the caller's standard time APIs.
No reference offset, timestamp repair or global monkey-patching is performed.
"""
from __future__ import annotations

import sys
import time


def monotonic_ns() -> int:
    return time.perf_counter_ns() if sys.platform == "win32" else time.monotonic_ns()


def monotonic() -> float:
    return time.perf_counter() if sys.platform == "win32" else time.monotonic()
