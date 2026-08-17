"""Deterministic Z-up tracker scene drawing for a Tk-compatible Canvas."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, TypeVar

from vt_vive_tracker.visualization_model import RoleSnapshot, VisualHealth

from .projection import (
    Camera,
    ProjectedPoint,
    camera_for_view,
    fit_camera,
    orientation_axes,
    project_point,
    project_points,
)


ROLE_COLORS = {
    "left_wrist": "#00d9ff",
    "right_wrist": "#ff00d9",
    "torso": "#ff8000",
}
HEALTH_COLORS = {
    VisualHealth.FRESH: "#34c759",
    VisualHealth.DELAYED: "#ffcc00",
    VisualHealth.OFFLINE: "#ff3b30",
}
OFFLINE_FILL = "#666666"

_GRID_COLOR = "#30343b"
_GRID_EXTENT = 2.0
_GRID_STEP = 0.5
_WORLD_AXIS_LENGTH = 1.0
_TRACKER_RADIUS = 6.0
_TRAIL_RADIUS = 2.0
_MAX_TRAIL_POINTS = 96
_T = TypeVar("_T")


@dataclass(frozen=True)
class _Primitive:
    depth: float
    method: str
    coordinates: tuple[float, ...]
    options: dict[str, Any]


def _evenly_sampled(
    points: list[_T], count: int
) -> list[_T]:
    if len(points) <= count:
        return points
    if count == 1:
        return [points[0]]
    final_index = len(points) - 1
    final_slot = count - 1
    return [
        points[(slot * final_index + final_slot // 2) // final_slot]
        for slot in range(count)
    ]


def _bounded_trail_runs(
    runs: list[list[ProjectedPoint]],
) -> list[list[ProjectedPoint]]:
    if sum(len(run) for run in runs) <= _MAX_TRAIL_POINTS:
        return runs

    minimums = [1 if len(run) == 1 else 2 for run in runs]
    if sum(minimums) > _MAX_TRAIL_POINTS:
        endpoints = []
        for run_index, run in enumerate(runs):
            endpoints.append((run_index, run[0]))
            if len(run) > 1:
                endpoints.append((run_index, run[-1]))
        selected = _evenly_sampled(endpoints, _MAX_TRAIL_POINTS)
        selected_runs = {}
        for run_index, point in selected:
            selected_runs.setdefault(run_index, []).append(point)
        return list(selected_runs.values())

    remaining = _MAX_TRAIL_POINTS - sum(minimums)
    capacities = [
        len(run) - minimum for run, minimum in zip(runs, minimums)
    ]
    total_capacity = sum(capacities)
    extras = [
        remaining * capacity // total_capacity for capacity in capacities
    ]
    leftover = remaining - sum(extras)
    remainders = [
        remaining * capacity % total_capacity for capacity in capacities
    ]
    for index in sorted(
        range(len(runs)), key=lambda value: (-remainders[value], value)
    )[:leftover]:
        extras[index] += 1

    return [
        _evenly_sampled(run, minimum + extra)
        for run, minimum, extra in zip(runs, minimums, extras)
    ]


class TrackerCanvasRenderer:
    """Project role snapshots and issue only Canvas drawing operations."""

    def __init__(self, canvas: Any, camera: Camera = Camera()) -> None:
        self.canvas = canvas
        self.camera = camera

    def orbit(self, dx: float, dy: float) -> None:
        self.camera = self.camera.orbit(dx, dy)

    def zoom(self, steps: float) -> None:
        self.camera = self.camera.zoom(steps)

    def set_view(self, name: str) -> None:
        self.camera = camera_for_view(name)

    def fit_all(self, roles: tuple[RoleSnapshot, ...]) -> None:
        points = tuple(
            role.pose.position for role in roles if role.pose is not None
        )
        self.camera = fit_camera(points, self.camera)

    def reset_view(self) -> None:
        self.camera = Camera()

    def render(
        self,
        roles: tuple[RoleSnapshot, ...],
        width: float,
        height: float,
    ) -> None:
        self.canvas.delete("all")
        primitives: list[_Primitive] = []
        self._gather_grid(primitives, width, height)
        self._gather_world_axes(primitives, width, height)
        for role in roles:
            if role.pose is not None:
                self._gather_role(primitives, role, width, height)

        for primitive in sorted(
            primitives, key=lambda item: item.depth, reverse=True
        ):
            getattr(self.canvas, primitive.method)(
                *primitive.coordinates, **primitive.options
            )

    def _projected_line(
        self,
        primitives: list[_Primitive],
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        viewport_width: float,
        viewport_height: float,
        **options: Any,
    ) -> None:
        projected_start = project_point(
            start, self.camera, viewport_width, viewport_height
        )
        projected_end = project_point(
            end, self.camera, viewport_width, viewport_height
        )
        if projected_start is None or projected_end is None:
            return
        primitives.append(
            _Primitive(
                (projected_start[2] + projected_end[2]) * 0.5,
                "create_line",
                (
                    projected_start[0],
                    projected_start[1],
                    projected_end[0],
                    projected_end[1],
                ),
                options,
            )
        )

    def _gather_grid(
        self,
        primitives: list[_Primitive],
        width: float,
        height: float,
    ) -> None:
        steps = round((2.0 * _GRID_EXTENT) / _GRID_STEP)
        for index in range(steps + 1):
            coordinate = -_GRID_EXTENT + index * _GRID_STEP
            self._projected_line(
                primitives,
                (coordinate, -_GRID_EXTENT, 0.0),
                (coordinate, _GRID_EXTENT, 0.0),
                width,
                height,
                fill=_GRID_COLOR,
                width=1,
                tags="grid",
            )
            self._projected_line(
                primitives,
                (-_GRID_EXTENT, coordinate, 0.0),
                (_GRID_EXTENT, coordinate, 0.0),
                width,
                height,
                fill=_GRID_COLOR,
                width=1,
                tags="grid",
            )

    def _gather_world_axes(
        self,
        primitives: list[_Primitive],
        width: float,
        height: float,
    ) -> None:
        origin = (0.0, 0.0, 0.0)
        for name, end, color in (
            ("x", (_WORLD_AXIS_LENGTH, 0.0, 0.0), "red"),
            ("y", (0.0, _WORLD_AXIS_LENGTH, 0.0), "green"),
            ("z", (0.0, 0.0, _WORLD_AXIS_LENGTH), "blue"),
        ):
            self._projected_line(
                primitives,
                origin,
                end,
                width,
                height,
                fill=color,
                width=2,
                arrow="last",
                tags=f"world-axis:{name}",
            )

    def _gather_role(
        self,
        primitives: list[_Primitive],
        role: RoleSnapshot,
        width: float,
        height: float,
    ) -> None:
        assert role.pose is not None
        role_color = ROLE_COLORS[role.role]
        geometry_color = (
            OFFLINE_FILL
            if role.health is VisualHealth.OFFLINE
            else role_color
        )
        stipple = "gray50" if role.health is VisualHealth.DELAYED else ""

        trail_runs = []
        current_run = []
        projected_trail = project_points(
            (trail_pose.position for trail_pose in role.trail),
            self.camera,
            width,
            height,
        )
        for point in projected_trail:
            if point is None:
                if current_run:
                    trail_runs.append(current_run)
                    current_run = []
                continue
            current_run.append(point)
        if current_run:
            trail_runs.append(current_run)
        trail_runs = _bounded_trail_runs(trail_runs)

        trail_options = {
            "fill": geometry_color,
            "stipple": stipple,
            "tags": f"trail:{role.role}",
        }
        for trail_points in trail_runs:
            if len(trail_points) >= 2:
                primitives.append(
                    _Primitive(
                        sum(point[2] for point in trail_points)
                        / len(trail_points),
                        "create_line",
                        tuple(
                            coordinate
                            for point in trail_points
                            for coordinate in point[:2]
                        ),
                        {**trail_options, "width": 2.0 * _TRAIL_RADIUS},
                    )
                )
                continue
            point = trail_points[0]
            primitives.append(
                _Primitive(
                    point[2],
                    "create_oval",
                    (
                        point[0] - _TRAIL_RADIUS,
                        point[1] - _TRAIL_RADIUS,
                        point[0] + _TRAIL_RADIUS,
                        point[1] + _TRAIL_RADIUS,
                    ),
                    {
                        **trail_options,
                        "outline": "",
                    },
                )
            )

        for index, (start, end, axis_color) in enumerate(
            orientation_axes(
                role.pose.position,
                role.pose.orientation_xyzw,
            )
        ):
            self._projected_line(
                primitives,
                start,
                end,
                width,
                height,
                fill=(
                    OFFLINE_FILL
                    if role.health is VisualHealth.OFFLINE
                    else axis_color
                ),
                width=1 if role.health is VisualHealth.DELAYED else 2,
                tags=f"local-axis:{role.role}:{'xyz'[index]}",
            )

        center = project_point(role.pose.position, self.camera, width, height)
        if center is None:
            return
        x, y, depth = center
        primitives.extend(
            (
                _Primitive(
                    depth,
                    "create_polygon",
                    (
                        x,
                        y - _TRACKER_RADIUS,
                        x + _TRACKER_RADIUS,
                        y,
                        x,
                        y + _TRACKER_RADIUS,
                        x - _TRACKER_RADIUS,
                        y,
                    ),
                    {
                        "fill": geometry_color,
                        "outline": "#ffffff",
                        "width": 1,
                        "stipple": stipple,
                        "tags": f"tracker:{role.role}",
                    },
                ),
                _Primitive(
                    depth,
                    "create_text",
                    (x + 11.0, y - 10.0),
                    {
                        "text": role.role.replace("_", " "),
                        "fill": geometry_color,
                        "anchor": "sw",
                        "tags": f"label:{role.role}",
                    },
                ),
                _Primitive(
                    depth,
                    "create_oval",
                    (x + 7.0, y - 16.0, x + 13.0, y - 10.0),
                    {
                        "fill": HEALTH_COLORS[role.health],
                        "outline": "",
                        "tags": f"health:{role.role}",
                    },
                ),
            )
        )
