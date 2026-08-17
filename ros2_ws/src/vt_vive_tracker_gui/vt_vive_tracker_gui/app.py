"""Main-thread refresh orchestration for the tracker dashboard."""

from __future__ import annotations

import time
from collections import deque
from typing import Callable, Protocol

from .snapshot_store import LatestSnapshotStore, StoredSnapshot


FRAME_NS = 1_000_000_000 // 60
_FPS_WINDOW_NS = 1_000_000_000
_RENDER_ERROR_THROTTLE_NS = 5_000_000_000


class _Root(Protocol):
    def after(
        self,
        delay_ms: int,
        callback: Callable[[], None],
    ) -> object:
        ...

    def after_cancel(self, after_id: object) -> None:
        ...

    def destroy(self) -> None:
        ...


class _View(Protocol):
    def render(self, stored: StoredSnapshot | None, fps: float) -> None:
        ...

    def set_diagnostic(self, text: str) -> None:
        ...


class TrackerApplication:
    """Drive dashboard refreshes without owning or touching ROS objects."""

    def __init__(
        self,
        root: _Root,
        store: LatestSnapshotStore,
        view: _View,
        shutdown: Callable[[], None],
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.root = root
        self.store = store
        self.view = view
        self._shutdown = shutdown
        self._monotonic_ns = monotonic_ns
        self._closed = False
        self._started = False
        self._after_id: object | None = None
        self._next_frame_ns = 0
        self._frame_times_ns: deque[int] = deque()
        self._fps = 0.0
        self._diagnostic_version = 0
        self._last_render_error_ns: int | None = None

    def start(self) -> None:
        if self._closed or self._started:
            return
        self._started = True
        self._next_frame_ns = self._monotonic_ns()
        self._after_id = self.root.after(0, self.refresh)

    def _record_fps(self, now_ns: int) -> None:
        self._frame_times_ns.append(now_ns)
        cutoff_ns = now_ns - _FPS_WINDOW_NS
        while self._frame_times_ns and self._frame_times_ns[0] < cutoff_ns:
            self._frame_times_ns.popleft()
        self._fps = float(len(self._frame_times_ns))

    def _forward_latest_diagnostic(self) -> None:
        diagnostic = self.store.latest_diagnostic()
        if (
            diagnostic is None
            or diagnostic.version <= self._diagnostic_version
        ):
            return
        self._diagnostic_version = diagnostic.version
        self.view.set_diagnostic(diagnostic.text)

    def _report_render_error(self, error: Exception, now_ns: int) -> None:
        previous_ns = self._last_render_error_ns
        if (
            previous_ns is not None
            and now_ns - previous_ns < _RENDER_ERROR_THROTTLE_NS
        ):
            return
        self._last_render_error_ns = now_ns
        self.view.set_diagnostic(f"Render error: {error}")

    def refresh(self) -> None:
        if self._closed:
            return
        self._after_id = None
        now_ns = self._monotonic_ns()
        latest = self.store.latest()
        self._record_fps(now_ns)
        self._forward_latest_diagnostic()
        try:
            self.view.render(latest, self._fps)
        except Exception as error:
            self._report_render_error(error, now_ns)

        if self._closed:
            return
        self._next_frame_ns = max(
            self._next_frame_ns + FRAME_NS,
            now_ns,
        )
        delay_ms = max(
            0,
            (self._next_frame_ns - now_ns) // 1_000_000,
        )
        self._after_id = self.root.after(delay_ms, self.refresh)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        after_id = self._after_id
        self._after_id = None
        try:
            if after_id is not None:
                self.root.after_cancel(after_id)
        finally:
            try:
                self._shutdown()
            finally:
                self.root.destroy()
