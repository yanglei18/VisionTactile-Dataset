"""Small, dependency-free (apart from NumPy) scalar-first quaternion helpers.

Quaternions use ``[w, x, y, z]`` and active, right-handed rotations.  ``mul(a,
b)`` composes rotations so that ``b`` is applied first.  Inputs are deliberately
validated: a malformed IMU quaternion must not become plausible hand geometry.
"""

from __future__ import annotations

import numpy as np


def _vector(value, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite vector of length {size}")
    return result


def normalize(q) -> np.ndarray:
    """Return a unit quaternion without overflowing for large finite inputs."""
    q = _vector(q, 4, "quaternion")
    scale = float(np.max(np.abs(q)))
    if scale == 0.0:
        raise ValueError("a zero quaternion has no orientation")
    scaled = q / scale
    return scaled / np.linalg.norm(scaled)


def mul(a, b) -> np.ndarray:
    """Hamilton product (does not implicitly normalize the operands)."""
    a = _vector(a, 4, "left quaternion")
    b = _vector(b, 4, "right quaternion")
    return np.concatenate((
        [a[0] * b[0] - np.dot(a[1:], b[1:])],
        a[0] * b[1:] + b[0] * a[1:] + np.cross(a[1:], b[1:]),
    ))


def conj(q) -> np.ndarray:
    """Quaternion conjugate; also the inverse when the input is unit length."""
    q = _vector(q, 4, "quaternion")
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def rotate(q, v) -> np.ndarray:
    """Rotate one finite 3-vector using a normalized quaternion."""
    return to_matrix(q) @ _vector(v, 3, "vector")


def from_axis_angle(axis, angle_rad: float) -> np.ndarray:
    """Build a unit quaternion from a nonzero axis and angle in radians."""
    axis = _vector(axis, 3, "axis")
    angle_rad = float(angle_rad)
    if not np.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    scale = float(np.max(np.abs(axis)))
    if scale == 0.0:
        raise ValueError("rotation axis must be nonzero")
    axis = axis / scale
    axis /= np.linalg.norm(axis)
    half_angle = angle_rad / 2.0
    return normalize(np.concatenate(([np.cos(half_angle)], axis * np.sin(half_angle))))


def to_matrix(q) -> np.ndarray:
    """Convert a quaternion to a proper 3-by-3 rotation matrix."""
    w, x, y, z = normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _canonical(q) -> np.ndarray:
    q = normalize(q)
    # Deterministic also for half-turns, where w may be exactly zero.
    pivot = next((value for value in q if abs(value) > 1e-12), 1.0)
    return -q if pivot < 0.0 else q


def from_matrix(matrix) -> np.ndarray:
    """Convert a finite, orthonormal, right-handed rotation matrix.

    Reflection, scale, and shear are rejected, rather than silently projected
    onto a rotation.  This protects mounting-axis calibration from bad inputs.
    """
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation matrix must be finite and have shape (3, 3)")
    if not np.allclose(matrix.T @ matrix, np.eye(3), rtol=0.0, atol=1e-6):
        raise ValueError("rotation matrix must be orthonormal")
    if not np.isclose(np.linalg.det(matrix), 1.0, rtol=0.0, atol=1e-6):
        raise ValueError("rotation matrix must be right-handed (determinant +1)")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.array([s / 4.0, (matrix[2, 1] - matrix[1, 2]) / s,
                      (matrix[0, 2] - matrix[2, 0]) / s,
                      (matrix[1, 0] - matrix[0, 1]) / s])
    else:
        index = int(np.argmax(np.diag(matrix)))
        j, k = (index + 1) % 3, (index + 2) % 3
        s = np.sqrt(max(0.0, 1.0 + matrix[index, index] - matrix[j, j] - matrix[k, k])) * 2.0
        q = np.zeros(4)
        q[0] = (matrix[k, j] - matrix[j, k]) / s
        q[index + 1] = s / 4.0
        q[j + 1] = (matrix[j, index] + matrix[index, j]) / s
        q[k + 1] = (matrix[k, index] + matrix[index, k]) / s
    return _canonical(q)


def mean_quaternion(quats) -> np.ndarray:
    """Sign-invariant Markley mean of a nonempty collection of orientations.

    Equal-weight principal-eigenvector averaging avoids cancellation when a
    sensor alternates between the equivalent representations q and -q.
    """
    samples = np.asarray(list(quats), dtype=float)
    if samples.ndim != 2 or samples.shape[1:] != (4,) or len(samples) == 0:
        raise ValueError("quaternion samples must have nonempty shape (N, 4)")
    samples = np.array([normalize(q) for q in samples])
    _, vectors = np.linalg.eigh(samples.T @ samples / len(samples))
    return _canonical(vectors[:, -1])
