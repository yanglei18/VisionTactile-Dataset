"""Validated host-timestamped raw IMU observations.

Timestamps are host parse times, not measurements of the device's sampling
clock. Several observations may therefore legitimately have the same time.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Integral, Real


@dataclass(frozen=True)
class Sample:
    """One node's signed 16-bit gyro/accelerometer values, in wire order."""

    node_id: int
    values: tuple[int, int, int, int, int, int]
    host_monotonic_s: float
    host_wall_s: float
    status: int | None = None

    def __post_init__(self) -> None:
        if isinstance(self.node_id, bool) or not isinstance(self.node_id, Integral):
            raise ValueError("node_id must be an integer in 1..12")
        if not 1 <= self.node_id <= 12:
            raise ValueError("node_id must be an integer in 1..12")
        try:
            values = tuple(self.values)
        except TypeError as exc:
            raise ValueError("values must contain six signed 16-bit integers") from exc
        if len(values) != 6 or any(
            isinstance(value, bool)
            or not isinstance(value, Integral)
            or not -32768 <= value <= 32767
            for value in values
        ):
            raise ValueError("values must contain six signed 16-bit integers")
        for name in ("host_monotonic_s", "host_wall_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"{name} must be a finite number")
            try:
                value = float(value)
            except (OverflowError, ValueError) as exc:
                raise ValueError(f"{name} must be a finite number") from exc
            if not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
            object.__setattr__(self, name, value)
        if self.status is not None and (
            isinstance(self.status, bool)
            or not isinstance(self.status, Integral)
            or not 0 <= self.status <= 255
        ):
            raise ValueError("status must be None or an integer in 0..255")
        object.__setattr__(self, "node_id", int(self.node_id))
        object.__setattr__(self, "values", tuple(int(value) for value in values))
        if self.status is not None:
            object.__setattr__(self, "status", int(self.status))
