from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ViewerConfig:
    canvas_width: int = 1600
    canvas_height: int = 900
    depth_min_m: float = 0.2
    depth_max_m: float = 3.0
    tracker_range_m: float = 2.0

    def __post_init__(self) -> None:
        if type(self.canvas_width) is not int or self.canvas_width < 800:
            raise ValueError("canvas_width must be an integer of at least 800")
        if type(self.canvas_height) is not int or self.canvas_height < 480:
            raise ValueError("canvas_height must be an integer of at least 480")
        values = (self.depth_min_m, self.depth_max_m, self.tracker_range_m)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("viewer metric ranges must be finite")
        if self.depth_min_m < 0.0:
            raise ValueError("depth_min_m must be non-negative")
        if self.depth_max_m <= self.depth_min_m:
            raise ValueError("depth_max_m must be greater than depth_min_m")
        if self.tracker_range_m <= 0.0:
            raise ValueError("tracker_range_m must be positive")


class PlaybackController:
    """Map monotonic wall time onto a strictly increasing data timeline."""

    def __init__(
        self,
        frame_times_ns: tuple[int, ...],
        *,
        start_index: int,
        speed: float,
    ) -> None:
        values = tuple(frame_times_ns)
        if not values:
            raise ValueError("frame_times_ns must not be empty")
        if any(type(value) is not int or value < 0 for value in values):
            raise ValueError("frame_times_ns must contain non-negative integers")
        if any(right <= left for left, right in zip(values, values[1:])):
            raise ValueError("frame_times_ns must be strictly increasing")
        if type(start_index) is not int or not 0 <= start_index < len(values):
            raise ValueError("start_index is outside the frame timeline")
        self._validate_speed(speed)
        self._frame_times_ns = values
        self._index = start_index
        self._speed = float(speed)
        self._playing = False
        self._anchor_wall_ns = 0
        self._anchor_data_ns = values[start_index]

    @staticmethod
    def _validate_speed(speed: float) -> None:
        if not isinstance(speed, (int, float)) or not math.isfinite(speed):
            raise ValueError("speed must be a finite positive number")
        if speed <= 0.0:
            raise ValueError("speed must be a finite positive number")

    @property
    def index(self) -> int:
        return self._index

    @property
    def playing(self) -> bool:
        return self._playing

    @property
    def speed(self) -> float:
        return self._speed

    def play(self, *, now_ns: int) -> None:
        if self._index >= len(self._frame_times_ns) - 1:
            self._playing = False
            return
        self._anchor_wall_ns = now_ns
        self._anchor_data_ns = self._frame_times_ns[self._index]
        self._playing = True

    def pause(self, *, now_ns: int) -> None:
        self.tick(now_ns=now_ns)
        self._playing = False

    def toggle(self, *, now_ns: int) -> None:
        if self._playing:
            self.pause(now_ns=now_ns)
        else:
            self.play(now_ns=now_ns)

    def tick(self, *, now_ns: int) -> int:
        if not self._playing:
            return self._index
        elapsed_ns = max(0, now_ns - self._anchor_wall_ns)
        target_ns = self._anchor_data_ns + int(elapsed_ns * self._speed)
        target_index = bisect_right(self._frame_times_ns, target_ns) - 1
        self._index = max(self._index, target_index)
        if self._index >= len(self._frame_times_ns) - 1:
            self._index = len(self._frame_times_ns) - 1
            self._playing = False
        return self._index

    def seek(self, index: int, *, now_ns: int) -> int:
        if type(index) is not int:
            raise TypeError("frame index must be an integer")
        self._index = min(max(index, 0), len(self._frame_times_ns) - 1)
        if self._playing and self._index < len(self._frame_times_ns) - 1:
            self._anchor_wall_ns = now_ns
            self._anchor_data_ns = self._frame_times_ns[self._index]
        elif self._index >= len(self._frame_times_ns) - 1:
            self._playing = False
        return self._index

    def set_speed(self, speed: float, *, now_ns: int) -> None:
        self._validate_speed(speed)
        if self._playing:
            self.tick(now_ns=now_ns)
            if self._playing:
                self._anchor_wall_ns = now_ns
                self._anchor_data_ns = self._frame_times_ns[self._index]
        self._speed = float(speed)
