from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


_EPSILON = 1e-12


def _as_vector(value: object, length: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},)")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    result = array.copy()
    result.setflags(write=False)
    return result


def _as_rotation(value: object) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    if not np.all(np.isfinite(array)):
        raise ValueError("rotation must contain only finite values")
    if not np.allclose(array.T @ array, np.eye(3), atol=1e-7):
        raise ValueError("rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(array)), 1.0, abs_tol=1e-7):
        raise ValueError("rotation determinant must be +1")
    result = array.copy()
    result.setflags(write=False)
    return result


def _normalized_quaternion_xyzw(value: object) -> np.ndarray:
    quaternion = _as_vector(value, 4, "quaternion_xyzw").copy()
    norm = float(np.linalg.norm(quaternion))
    if norm < _EPSILON:
        raise ValueError("quaternion norm must be non-zero")
    quaternion /= norm
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    quaternion.setflags(write=False)
    return quaternion


def quaternion_xyzw_to_rotation(value: object) -> np.ndarray:
    x, y, z, w = _normalized_quaternion_xyzw(value)
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return _as_rotation(rotation)


def rotation_to_quaternion_xyzw(value: object) -> np.ndarray:
    rotation = _as_rotation(value)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    return _normalized_quaternion_xyzw(quaternion)


def slerp_xyzw(left: object, right: object, fraction: float) -> np.ndarray:
    if not 0.0 <= fraction <= 1.0 or not math.isfinite(fraction):
        raise ValueError("fraction must be finite and within [0, 1]")
    first = _normalized_quaternion_xyzw(left).copy()
    second = _normalized_quaternion_xyzw(right).copy()
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second *= -1.0
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = first + fraction * (second - first)
        return _normalized_quaternion_xyzw(result)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    result = (
        math.sin((1.0 - fraction) * theta) / sin_theta * first
        + math.sin(fraction * theta) / sin_theta * second
    )
    return _normalized_quaternion_xyzw(result)


@dataclass(frozen=True, eq=False)
class Transform:
    """Rigid transform ``parent_from_child`` in metres."""

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "rotation", _as_rotation(self.rotation))
        object.__setattr__(
            self, "translation", _as_vector(self.translation, 3, "translation")
        )

    @classmethod
    def identity(cls) -> Transform:
        return cls(np.eye(3), np.zeros(3))

    @classmethod
    def from_quaternion_xyzw(
        cls, translation: object, quaternion_xyzw: object
    ) -> Transform:
        return cls(
            quaternion_xyzw_to_rotation(quaternion_xyzw),
            _as_vector(translation, 3, "translation"),
        )

    @classmethod
    def from_rvec_tvec(cls, rvec: object, tvec: object) -> Transform:
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
        return cls(rotation, np.asarray(tvec, dtype=np.float64).reshape(3))

    @classmethod
    def from_matrix(cls, matrix: object) -> Transform:
        value = np.asarray(matrix, dtype=np.float64)
        if value.shape != (4, 4):
            raise ValueError("homogeneous matrix must have shape (4, 4)")
        if not np.allclose(value[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
            raise ValueError("homogeneous matrix has an invalid final row")
        return cls(value[:3, :3], value[:3, 3])

    @property
    def quaternion_xyzw(self) -> np.ndarray:
        return rotation_to_quaternion_xyzw(self.rotation)

    @property
    def matrix(self) -> np.ndarray:
        result = np.eye(4)
        result[:3, :3] = self.rotation
        result[:3, 3] = self.translation
        return result

    def inverse(self) -> Transform:
        rotation = self.rotation.T
        return Transform(rotation, -(rotation @ self.translation))

    def __matmul__(self, other: object) -> Transform:
        if not isinstance(other, Transform):
            return NotImplemented
        return Transform(
            self.rotation @ other.rotation,
            self.rotation @ other.translation + self.translation,
        )

    def interpolate(self, other: Transform, fraction: float) -> Transform:
        if not isinstance(other, Transform):
            raise TypeError("other must be Transform")
        quaternion = slerp_xyzw(
            self.quaternion_xyzw, other.quaternion_xyzw, fraction
        )
        translation = (
            self.translation + fraction * (other.translation - self.translation)
        )
        return Transform.from_quaternion_xyzw(translation, quaternion)


def rotation_distance_rad(left: Transform, right: Transform) -> float:
    relative = left.rotation.T @ right.rotation
    cosine = (float(np.trace(relative)) - 1.0) / 2.0
    return math.acos(min(1.0, max(-1.0, cosine)))


def transform_mean(values: list[Transform] | tuple[Transform, ...]) -> Transform:
    if not values:
        raise ValueError("at least one transform is required")
    translation = np.mean([value.translation for value in values], axis=0)
    accumulator = np.zeros((4, 4), dtype=np.float64)
    reference = values[0].quaternion_xyzw
    for value in values:
        quaternion = value.quaternion_xyzw.copy()
        if float(np.dot(quaternion, reference)) < 0.0:
            quaternion *= -1.0
        accumulator += np.outer(quaternion, quaternion)
    _, eigenvectors = np.linalg.eigh(accumulator)
    quaternion = eigenvectors[:, -1]
    return Transform.from_quaternion_xyzw(translation, quaternion)
