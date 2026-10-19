"""Responsive, read-only Tk dashboard for tracker snapshots."""

from __future__ import annotations

import os
import tkinter as tk
from dataclasses import dataclass
from typing import Any

from vt_vive_tracker.visualization_model import RoleSnapshot, VisualHealth

from .canvas3d import HEALTH_COLORS, ROLE_COLORS, TrackerCanvasRenderer
from .display_model import (
    OverallState,
    TrackerCardModel,
    card_for_snapshot,
    overall_state,
)
from .snapshot_store import StoredSnapshot


_BACKGROUND = "#17191d"
_PANEL = "#23262c"
_TEXT = "#f3f4f6"
_MUTED = "#a6abb4"
_BORDER = "#353942"
_ORBIT_RADIANS_PER_PIXEL = 0.01
_STATE_COLORS = {
    OverallState.LIVE: HEALTH_COLORS[VisualHealth.FRESH],
    OverallState.DEGRADED: HEALTH_COLORS[VisualHealth.DELAYED],
    OverallState.DISCONNECTED: HEALTH_COLORS[VisualHealth.OFFLINE],
}


@dataclass(frozen=True)
class _CardLabels:
    tracker_id: tk.Label
    health: tk.Label
    position: tk.Label
    quaternion: tk.Label
    rpy: tk.Label
    rate: tk.Label
    age: tk.Label
    counters: tk.Label


class TrackerDashboard:
    """Render immutable tracker state with view-only camera controls."""

    def __init__(self, root: tk.Misc) -> None:
        self.root = root
        self._last_roles: tuple[RoleSnapshot, ...] = ()
        self._drag_position: tuple[int, int] | None = None
        self._cards: dict[str, _CardLabels] = {}

        root.title("VIVE Tracker Dashboard")
        root.configure(background=_BACKGROUND)
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(1, weight=1)

        self._build_top_bar()
        self._build_content()
        self._build_diagnostic_bar()

    def _label(
        self,
        parent: tk.Misc,
        text: str = "",
        *,
        foreground: str = _TEXT,
        font: tuple[str, int, str] | tuple[str, int] | None = None,
        anchor: str = "w",
    ) -> tk.Label:
        options: dict[str, Any] = {
            "text": text,
            "background": parent.cget("background"),
            "foreground": foreground,
            "anchor": anchor,
        }
        if font is not None:
            options["font"] = font
        return tk.Label(parent, **options)

    def _build_top_bar(self) -> None:
        top = tk.Frame(self.root, background=_PANEL, padx=14, pady=10)
        top.grid(row=0, column=0, sticky="ew")
        top.grid_columnconfigure(0, weight=1)

        self._label(
            top,
            "VIVE Tracker Dashboard",
            font=("TkDefaultFont", 15, "bold"),
        ).grid(row=0, column=0, sticky="w")
        self._overall_label = self._label(
            top,
            OverallState.DISCONNECTED.value,
            foreground=_STATE_COLORS[OverallState.DISCONNECTED],
            font=("TkDefaultFont", 10, "bold"),
        )
        self._overall_label.grid(row=0, column=1, padx=(12, 18))
        self._fps_label = self._label(top, "FPS 0.0", foreground=_MUTED)
        self._fps_label.grid(row=0, column=2, padx=(0, 18))
        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        self._label(
            top,
            f"ROS domain {domain_id}",
            foreground=_MUTED,
        ).grid(row=0, column=3, padx=(0, 18))
        self._label(
            top,
            "ROS 2 read-only",
            foreground=HEALTH_COLORS[VisualHealth.FRESH],
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=0, column=4)

    def _build_content(self) -> None:
        content = tk.Frame(self.root, background=_BACKGROUND, padx=10, pady=10)
        content.grid(row=1, column=0, sticky="nsew")
        content.grid_columnconfigure(0, weight=3, minsize=360)
        content.grid_columnconfigure(1, weight=2, minsize=330)
        content.grid_rowconfigure(0, weight=1)

        scene = tk.Frame(
            content,
            background=_PANEL,
            highlightthickness=1,
            highlightbackground=_BORDER,
        )
        scene.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        scene.grid_columnconfigure(0, weight=1)
        scene.grid_rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            scene,
            background="#111318",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.renderer = TrackerCanvasRenderer(self.canvas)
        self._bind_canvas()
        self._build_view_controls(scene)

        cards = tk.Frame(content, background=_BACKGROUND)
        cards.grid(row=0, column=1, sticky="nsew")
        cards.grid_columnconfigure(0, weight=1)
        for row, role in enumerate(("left_wrist", "right_wrist", "torso")):
            cards.grid_rowconfigure(row, weight=1)
            self._build_card(cards, role, row)

    def _build_view_controls(self, parent: tk.Frame) -> None:
        controls = tk.Frame(parent, background=_PANEL, padx=8, pady=8)
        controls.grid(row=1, column=0, sticky="ew")
        for column in range(5):
            controls.grid_columnconfigure(column, weight=1)
        definitions = (
            ("俯视", lambda: self._set_view("top")),
            ("前视", lambda: self._set_view("front")),
            ("侧视", lambda: self._set_view("side")),
            ("适应全部", self._fit_all),
            ("重置视角", self._reset_view),
        )
        for column, (text, command) in enumerate(definitions):
            tk.Button(
                controls,
                text=text,
                command=command,
                background="#30343b",
                foreground=_TEXT,
                activebackground="#414650",
                activeforeground=_TEXT,
                relief="flat",
                padx=6,
                pady=5,
            ).grid(row=0, column=column, sticky="ew", padx=2)

    def _build_card(self, parent: tk.Frame, role: str, row: int) -> None:
        card = tk.Frame(
            parent,
            background=_PANEL,
            padx=12,
            pady=10,
            highlightthickness=1,
            highlightbackground=_BORDER,
        )
        card.grid(
            row=row,
            column=0,
            sticky="nsew",
            pady=(0 if row == 0 else 4, 4),
        )
        card.grid_columnconfigure(0, weight=1)
        self._label(
            card,
            role,
            foreground=ROLE_COLORS[role],
            font=("TkDefaultFont", 12, "bold"),
        ).grid(row=0, column=0, sticky="w")
        health = self._label(
            card,
            VisualHealth.OFFLINE.value,
            foreground=HEALTH_COLORS[VisualHealth.OFFLINE],
            font=("TkDefaultFont", 10, "bold"),
            anchor="e",
        )
        health.grid(row=0, column=1, sticky="e")

        labels = []
        initial = (
            "ID —",
            "Position  x —   y —   z —",
            "Quaternion  x —   y —   z —   w —",
            "RPY  r —   p —   y —",
            "Rate  —",
            "Age  —",
            "valid — · invalid — · dropped —",
        )
        for index, text in enumerate(initial, start=1):
            label = self._label(card, text, foreground=_MUTED)
            label.grid(
                row=index,
                column=0,
                columnspan=2,
                sticky="ew",
                pady=(2, 0),
            )
            labels.append(label)
        self._cards[role] = _CardLabels(
            tracker_id=labels[0],
            health=health,
            position=labels[1],
            quaternion=labels[2],
            rpy=labels[3],
            rate=labels[4],
            age=labels[5],
            counters=labels[6],
        )

    def _build_diagnostic_bar(self) -> None:
        diagnostic = tk.Frame(self.root, background=_PANEL, padx=12, pady=6)
        diagnostic.grid(row=2, column=0, sticky="ew")
        diagnostic.grid_columnconfigure(0, weight=1)
        self._diagnostic_label = self._label(
            diagnostic,
            "Ready — waiting for ROS 2 tracker data",
            foreground=_MUTED,
        )
        self._diagnostic_label.grid(row=0, column=0, sticky="ew")

    def _bind_canvas(self) -> None:
        self.canvas.bind("<ButtonPress-1>", self._start_drag)
        self.canvas.bind("<B1-Motion>", self._drag)
        self.canvas.bind("<MouseWheel>", self._wheel)
        self.canvas.bind("<Button-4>", self._wheel)
        self.canvas.bind("<Button-5>", self._wheel)
        self.canvas.bind("<Double-Button-1>", self._double_click)
        self.canvas.bind("<Configure>", self._configure_canvas)

    def _start_drag(self, event: tk.Event) -> None:
        self._drag_position = (event.x, event.y)

    def _drag(self, event: tk.Event) -> None:
        if self._drag_position is None:
            self._drag_position = (event.x, event.y)
            return
        previous_x, previous_y = self._drag_position
        self._drag_position = (event.x, event.y)
        self.renderer.orbit(
            -(event.x - previous_x) * _ORBIT_RADIANS_PER_PIXEL,
            (event.y - previous_y) * _ORBIT_RADIANS_PER_PIXEL,
        )
        self._redraw()

    @staticmethod
    def _normalized_wheel_steps(event: tk.Event) -> float:
        number = getattr(event, "num", None)
        if number == 4:
            return 1.0
        if number == 5:
            return -1.0
        delta = float(getattr(event, "delta", 0.0))
        if delta == 0.0:
            return 0.0
        return delta / 120.0 if abs(delta) >= 120.0 else delta / abs(delta)

    def _wheel(self, event: tk.Event) -> str:
        steps = self._normalized_wheel_steps(event)
        if steps:
            self.renderer.zoom(steps)
            self._redraw()
        return "break"

    def _double_click(self, _event: tk.Event) -> str:
        self._reset_view()
        return "break"

    def _configure_canvas(self, event: tk.Event) -> None:
        self._redraw(event.width, event.height)

    def _set_view(self, name: str) -> None:
        self.renderer.set_view(name)
        self._redraw()

    def _fit_all(self) -> None:
        self.renderer.fit_all(self._last_roles)
        self._redraw()

    def _reset_view(self) -> None:
        self.renderer.reset_view()
        self._redraw()

    def _redraw(
        self,
        width: int | None = None,
        height: int | None = None,
    ) -> None:
        width = self.canvas.winfo_width() if width is None else width
        height = self.canvas.winfo_height() if height is None else height
        self.renderer.render(self._last_roles, max(1, width), max(1, height))

    def _render_card(self, model: TrackerCardModel) -> None:
        labels = self._cards[model.role]
        labels.health.configure(
            text=model.health.value,
            foreground=HEALTH_COLORS[model.health],
        )
        labels.tracker_id.configure(text=f"ID {model.tracker_id}")
        labels.position.configure(
            text="Position  x {}   y {}   z {}".format(*model.position)
        )
        labels.quaternion.configure(
            text="Quaternion  x {}   y {}   z {}   w {}".format(
                *model.quaternion
            )
        )
        labels.rpy.configure(
            text="RPY  r {}   p {}   y {}".format(*model.rpy_degrees)
        )
        labels.rate.configure(text=f"Rate  {model.rate}")
        labels.age.configure(text=f"Age  {model.age}")
        labels.counters.configure(text=model.counters)

    def render(self, stored: StoredSnapshot | None, fps: float) -> None:
        self._fps_label.configure(text=f"FPS {fps:.1f}")
        if stored is None:
            self._last_roles = ()
            state = OverallState.DISCONNECTED
        else:
            self._last_roles = stored.roles
            state = overall_state(stored.roles)
            for snapshot in stored.roles:
                self._render_card(card_for_snapshot(snapshot))
        self._overall_label.configure(
            text=state.value,
            foreground=_STATE_COLORS[state],
        )
        self._redraw()

    def set_diagnostic(self, text: str) -> None:
        self._diagnostic_label.configure(text=text, foreground=_MUTED)
