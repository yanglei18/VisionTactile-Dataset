"""Uncompressed recording capacity estimates and finalization reserve."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .config import CaptureConfig


@dataclass(frozen=True)
class DiskGuardConfig:
    min_free_bytes: int = 2 * 1024**3
    check_interval_sec: float = 0.5
    additional_stream_bytes_per_second: int = 0

    def __post_init__(self) -> None:
        if type(self.min_free_bytes) is not int or self.min_free_bytes <= 0:
            raise ValueError("recording.disk_guard.min_free_bytes must be a positive integer")
        if (type(self.check_interval_sec) not in {int, float}
                or not math.isfinite(self.check_interval_sec)
                or not 0.1 <= self.check_interval_sec <= 5.0):
            raise ValueError("recording.disk_guard.check_interval_sec must be between 0.1 and 5 seconds")
        if (type(self.additional_stream_bytes_per_second) is not int
                or self.additional_stream_bytes_per_second < 0):
            raise ValueError("recording.disk_guard.additional_stream_bytes_per_second must be a nonnegative integer")


@dataclass(frozen=True)
class RecordingSpaceBudget:
    bytes_per_second: int
    stop_reserve_bytes: int
    required_start_bytes: int
    additional_streams_unbudgeted: bool

    def describe(self) -> str:
        scope = ("image streams only; additional topics have no configured size estimate"
                 if self.additional_streams_unbudgeted else "known image streams and configured additional-stream estimate")
        return (f"required_bytes={self.required_start_bytes}; "
                f"estimated_bytes_per_second={self.bytes_per_second}; "
                f"stop_reserve_bytes={self.stop_reserve_bytes}; budget_scope={scope}")


def recording_space_budget(config: CaptureConfig, duration_sec: int) -> RecordingSpaceBudget:
    """Estimate RGB8 + Z16 without assuming compression or observed delivery Hz.

    Ten percent covers message/container overhead, not arbitrary additional
    topics. The reserve includes the recorder cache and ingestion around a
    polling interval / normal graceful-stop response. It is not a guarantee
    against other processes exhausting the filesystem or a stalled recorder.
    """
    if config.color_encoding != "rgb8" or config.depth_encoding != "16UC1":
        raise ValueError("disk budget requires the configured RGB8 and Z16 capture contract")
    raw_rate = len(config.cameras) * config.width * config.height * config.fps * 5
    rate = (raw_rate * 110 + 99) // 100 + config.disk_guard.additional_stream_bytes_per_second
    reserve = max(config.disk_guard.min_free_bytes,
                  config.max_cache_size_bytes + math.ceil(rate * (2 + 2 * config.disk_guard.check_interval_sec)))
    return RecordingSpaceBudget(rate, reserve, reserve + rate * duration_sec,
                                bool(config.additional_streams) and config.disk_guard.additional_stream_bytes_per_second == 0)
