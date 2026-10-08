"""Dependency-free perspective projection for the ROS ``vive_map`` frame.

World coordinates use an XY ground plane with positive Z pointing up.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
ProjectedPoint = tuple[float, float, float]
Axis = tuple[Vector3, Vector3, str]

_WORLD_UP: Vector3 = (0.0, 0.0, 1.0)
_MIN_PITCH = math.radians(-89.0)
_MAX_PITCH = math.radians(89.0)
_MIN_DISTANCE = 0.25
_NEAR_DEPTH = 0.05


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _scale(vector: Vector3, factor: float) -> Vector3:
    return tuple(value * factor for value in vector)  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _normalize(vector: Vector3) -> Vector3:
    magnitude = math.sqrt(_dot(vector, vector))
    if magnitude == 0.0:
        raise ValueError("cannot normalize a zero-length vector")
    return _scale(vector, 1.0 / magnitude)


@dataclass(frozen=True)
class Camera:
    """Immutable orbit camera looking at a target in ``vive_map``."""

    target: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = math.radians(-45.0)
    pitch: float = math.radians(30.0)
    distance: float = 5.0
    fov_y: float = math.radians(50.0)

    def __post_init__(self) -> None:
        if not math.isfinite(self.distance):
            raise ValueError("camera distance must be finite")
        object.__setattr__(self, "target", tuple(self.target))
        object.__setattr__(self, "pitch", max(_MIN_PITCH, min(_MAX_PITCH, self.pitch)))
        object.__setattr__(self, "distance", max(_MIN_DISTANCE, self.distance))

    def orbit(self, delta_yaw: float, delta_pitch: float) -> Camera:
        """Return a camera orbited by angular deltas in radians."""

        return replace(self, yaw=self.yaw + delta_yaw, pitch=self.pitch + delta_pitch)

    def zoom(self, steps: float) -> Camera:
        """Return a camera zoomed by wheel-like steps; positive moves closer."""

        exponent = max(-50.0, min(50.0, -0.12 * steps))
        return replace(self, distance=self.distance * math.exp(exponent))


def _view_frame(camera: Camera) -> tuple[Vector3, Vector3, Vector3, Vector3]:
    offset = (
        camera.distance * math.cos(camera.pitch) * math.cos(camera.yaw),
        camera.distance * math.cos(camera.pitch) * math.sin(camera.yaw),
        camera.distance * math.sin(camera.pitch),
    )
    eye = _add(camera.target, offset)
    forward = _normalize(_subtract(camera.target, eye))
    right = _normalize(_cross(forward, _WORLD_UP))
    up = _cross(right, forward)
    return eye, forward, right, up


def project_point(
    point: Vector3,
    camera: Camera,
    width: float,
    height: float,
) -> ProjectedPoint | None:
    """Project a world point to screen X/Y and positive camera depth."""

    return project_points((point,), camera, width, height)[0]


def project_points(
    points: Iterable[Vector3],
    camera: Camera,
    width: float,
    height: float,
) -> tuple[ProjectedPoint | None, ...]:
    """Project points using one cached camera frame and focal length."""

    eye, forward, right, up = _view_frame(camera)
    focal_length = (height * 0.5) / math.tan(camera.fov_y * 0.5)
    projected = []
    for point in points:
        relative = _subtract(point, eye)
        depth = _dot(relative, forward)
        if depth <= _NEAR_DEPTH:
            projected.append(None)
            continue

        perspective = focal_length / depth
        projected.append(
            (
                width * 0.5 + _dot(relative, right) * perspective,
                height * 0.5 - _dot(relative, up) * perspective,
                depth,
            )
        )
    return tuple(projected)


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(vector: Vector3, quaternion: Quaternion) -> Vector3:
    """Rotate a vector by an XYZW quaternion."""

    scale = max(abs(value) for value in quaternion)
    if scale == 0.0 or not math.isfinite(scale):
        raise ValueError("quaternion must have non-zero length")
    scaled = tuple(value / scale for value in quaternion)
    norm = math.sqrt(sum(value * value for value in scaled))
    x, y, z, w = (value / norm for value in scaled)
    unit_quaternion = (x, y, z, w)
    inverse = (-x, -y, -z, w)
    rotated = _quaternion_multiply(
        _quaternion_multiply(
            unit_quaternion,
            (vector[0], vector[1], vector[2], 0.0),
        ),
        inverse,
    )
    return rotated[0], rotated[1], rotated[2]


def orientation_axes(
    position: Vector3,
    quaternion: Quaternion,
    *,
    length: float = 0.2,
) -> tuple[Axis, Axis, Axis]:
    """Return red/green/blue local XYZ axes rooted at a world position."""

    return tuple(
        (position, _add(position, _scale(rotate_vector(axis, quaternion), length)), color)
        for axis, color in (
            ((1.0, 0.0, 0.0), "red"),
            ((0.0, 1.0, 0.0), "green"),
            ((0.0, 0.0, 1.0), "blue"),
        )
    )  # type: ignore[return-value]


def fit_camera(points: Iterable[Vector3], camera: Camera) -> Camera:
    """Center and distance a camera to contain the supplied world points."""

    points = tuple(points)
    if not points:
        return camera

    minimum = tuple(min(point[index] for point in points) for index in range(3))
    maximum = tuple(max(point[index] for point in points) for index in range(3))
    center = tuple((low + high) * 0.5 for low, high in zip(minimum, maximum))
    diagonal = math.sqrt(sum((high - low) ** 2 for low, high in zip(minimum, maximum)))
    return replace(
        camera,
        target=center,
        distance=max(1.0, diagonal * 1.5),
    )


def camera_for_view(view: str) -> Camera:
    """Return a front, side, or top Z-up camera preset."""

    views = {
        "front": Camera(yaw=-math.pi / 2.0, pitch=0.0),
        "side": Camera(yaw=0.0, pitch=0.0),
        "top": Camera(yaw=-math.pi / 2.0, pitch=_MAX_PITCH),
    }
    try:
        return views[view]
    except KeyError as error:
        raise ValueError(f"unknown camera view: {view!r}") from error
