from __future__ import annotations

import time
from typing import TYPE_CHECKING

from .viewer_model import PlaybackController, ViewerConfig

if TYPE_CHECKING:
    from .dataset import AlignedDataset


_SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0)


class _ViewerWindow:
    def __init__(
        self,
        root: object,
        dataset: AlignedDataset,
        *,
        start_index: int,
        speed: float,
        config: ViewerConfig,
    ) -> None:
        import tkinter as tk
        from PIL import ImageTk

        self._tk = tk
        self._image_tk = ImageTk
        self._root = root
        self._dataset = dataset
        self._config = config
        self._controller = PlaybackController(
            dataset.reference_times_ns,
            start_index=start_index,
            speed=speed,
        )
        self._label = tk.Label(root, borderwidth=0, highlightthickness=0)
        self._label.pack()
        self._photo = None
        self._last_rendered_index = -1
        self._dirty = True
        self._closed = False
        root.title("VisionTactile Aligned Dataset Viewer")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Key>", self._on_key)
        self._controller.play(now_ns=time.monotonic_ns())
        self._refresh()

    def _render(self) -> None:
        from .viewer_render import render_aligned_frame

        index = self._controller.index
        frame = self._dataset.frame(
            index,
            include_timing=False,
            additional_streams=(),
        )
        dashboard = render_aligned_frame(
            frame,
            camera_names=self._dataset.camera_names,
            total_frames=len(self._dataset),
            playing=self._controller.playing,
            speed=self._controller.speed,
            config=self._config,
        )
        self._photo = self._image_tk.PhotoImage(dashboard)
        self._label.configure(image=self._photo)
        self._last_rendered_index = index
        self._dirty = False

    def _refresh(self) -> None:
        if self._closed:
            return
        index = self._controller.tick(now_ns=time.monotonic_ns())
        if index != self._last_rendered_index or self._dirty:
            self._render()
        self._root.after(10, self._refresh)

    def _change_speed(self, direction: int, now_ns: int) -> None:
        current = self._controller.speed
        if direction > 0:
            choices = tuple(value for value in _SPEEDS if value > current)
            target = choices[0] if choices else _SPEEDS[-1]
        else:
            choices = tuple(value for value in _SPEEDS if value < current)
            target = choices[-1] if choices else _SPEEDS[0]
        self._controller.set_speed(target, now_ns=now_ns)
        self._dirty = True

    def _on_key(self, event: object) -> None:
        keysym = str(getattr(event, "keysym", ""))
        now_ns = time.monotonic_ns()
        if keysym in {"q", "Q", "Escape"}:
            self.close()
            return
        if keysym == "space":
            self._controller.toggle(now_ns=now_ns)
            self._dirty = True
            return
        if keysym in {"Left", "Right", "Home", "End"}:
            self._controller.pause(now_ns=now_ns)
            if keysym == "Left":
                target = self._controller.index - 1
            elif keysym == "Right":
                target = self._controller.index + 1
            elif keysym == "Home":
                target = 0
            else:
                target = len(self._dataset) - 1
            self._controller.seek(target, now_ns=now_ns)
            self._dirty = True
            return
        if keysym in {"plus", "equal", "KP_Add"}:
            self._change_speed(1, now_ns)
        elif keysym in {"minus", "underscore", "KP_Subtract"}:
            self._change_speed(-1, now_ns)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._root.destroy()


def run_interactive(
    dataset: AlignedDataset,
    *,
    start_index: int,
    speed: float,
    config: ViewerConfig,
) -> None:
    """Open the read-only Tk dashboard and run until the user closes it."""
    try:
        import tkinter as tk
    except ImportError as error:
        raise RuntimeError(
            "Tk viewer is unavailable; install python3-tk or use --export-frame"
        ) from error
    try:
        root = tk.Tk()
    except (RuntimeError, tk.TclError) as error:
        raise RuntimeError(
            "Tk viewer is unavailable; install python3-tk or use --export-frame"
        ) from error
    _ViewerWindow(
        root,
        dataset,
        start_index=start_index,
        speed=speed,
        config=config,
    )
    root.mainloop()
