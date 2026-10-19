"""Immutable SE(3): T_A_B maps B to A, metres, quaternion xyzw.

Scalar Python floats provide double precision without importing NumPy into the
manager's small-data path. Quaternion acceptance matches the offline aligner.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Mapping


def finite_vector(value, length: int, name: str) -> tuple[float, ...]:
    try:
        values = tuple(value)
        if len(values) != length or any(isinstance(item, (bool, str, bytes)) for item in values):
            raise ValueError
        result = tuple(float(item) for item in values)
        if not all(math.isfinite(item) for item in result):
            raise ValueError
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must contain {length} finite numbers') from exc
    return result


def _quaternion(value):
    values = finite_vector(value, 4, 'quaternion_xyzw')
    norm = math.hypot(*values)
    if not .90 <= norm <= 1.10:
        raise ValueError('quaternion norm is outside the accepted range [0.90, 1.10]')
    return tuple(item / norm for item in values)


def _multiply(a, b):
    x, y, z, w = a
    u, v, r, s = b
    return (w*u+x*s+y*r-z*v, w*v-x*r+y*s+z*u,
            w*r+x*v-y*u+z*s, w*s-x*u-y*v-z*r)


def _rotate(q, point):
    x, y, z, w = q
    a, b, c = point
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx)


def slerp_xyzw(left, right, fraction: float) -> tuple[float, ...]:
    if isinstance(fraction, bool) or not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError('fraction must be finite and between zero and one')
    left, right = _quaternion(left), _quaternion(right)
    dot = sum(a*b for a, b in zip(left, right))
    if dot < 0:
        right, dot = tuple(-item for item in right), -dot
    dot = min(1., max(-1., dot))
    if dot > .9995:
        return _quaternion(tuple(a + fraction*(b-a) for a, b in zip(left, right)))
    theta = math.acos(dot)
    scale = math.sin(theta)
    a, b = math.sin((1-fraction)*theta)/scale, math.sin(fraction*theta)/scale
    return _quaternion(tuple(a*x+b*y for x, y in zip(left, right)))


@dataclass(frozen=True)
class RigidTransform:
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def __post_init__(self):
        object.__setattr__(self, 'translation_m', finite_vector(self.translation_m, 3, 'translation_m'))
        object.__setattr__(self, 'quaternion_xyzw', _quaternion(self.quaternion_xyzw))

    @classmethod
    def identity(cls) -> RigidTransform:
        return cls((0., 0., 0.), (0., 0., 0., 1.))

    def compose(self, child: RigidTransform) -> RigidTransform:
        if not isinstance(child, RigidTransform):
            raise TypeError('child must be RigidTransform')
        return RigidTransform(self.apply(child.translation_m), _multiply(self.quaternion_xyzw, child.quaternion_xyzw))

    def inverse(self) -> RigidTransform:
        x, y, z, w = self.quaternion_xyzw
        q = (-x, -y, -z, w)
        return RigidTransform(_rotate(q, tuple(-item for item in self.translation_m)), q)

    def apply(self, point) -> tuple[float, float, float]:
        rotated = _rotate(self.quaternion_xyzw, finite_vector(point, 3, 'point'))
        return tuple(a+b for a, b in zip(rotated, self.translation_m))

    def interpolate(self, other: RigidTransform, fraction: float) -> RigidTransform:
        if not isinstance(other, RigidTransform):
            raise TypeError('other must be RigidTransform')
        rotation = slerp_xyzw(self.quaternion_xyzw, other.quaternion_xyzw, fraction)
        translation = tuple(a+fraction*(b-a) for a, b in zip(self.translation_m, other.translation_m))
        return RigidTransform(translation, rotation)

    def as_dict(self) -> dict:
        return {'translation_m': list(self.translation_m), 'quaternion_xyzw': list(self.quaternion_xyzw)}

    @classmethod
    def from_dict(cls, value: Mapping) -> RigidTransform:
        if not isinstance(value, Mapping) or set(value) != {'translation_m', 'quaternion_xyzw'}:
            raise ValueError('transform requires translation_m and quaternion_xyzw')
        return cls(value['translation_m'], value['quaternion_xyzw'])
