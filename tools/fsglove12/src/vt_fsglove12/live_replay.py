"""Paced, read-only CSV source for exercising the live pipeline without serial."""
from __future__ import annotations

import math
from pathlib import Path
import time

from . import host_clock
from .live_stream import RawSample, iter_csv_samples


class PacedCSVSource:
    """Replay once; never wrap EOF, synthesize samples, or claim live hardware.

    Source timestamps are shifted to this host's monotonic clock, retaining
    original spacing (divided by speed). Every raw value and sample is retained.
    The source CSV is never edited. EOF becomes a visible data gap in the UI.
    """

    input_kind = "csv_replay"

    def __init__(self, path, *, speed=1., clock=host_clock.monotonic, wall_clock=time.time):
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed <= 0:
            raise ValueError("Replay speed must be a finite positive number")
        self.path, self.speed, self.clock, self.wall_clock = path, speed, clock, wall_clock
        self.provenance = {"source_raw_path": str(Path(path).resolve()), "replay_speed": speed,
                           "host_timestamps_rebased": True, "hardware_accessed": False}
        self._iterator = None
        self._pending = None
        self.eof = False

    def open(self):
        if self._iterator is not None:
            raise RuntimeError("Replay source is already open")
        self._iterator = iter_csv_samples(self.path)
        try:
            self._pending = next(self._iterator)
        except StopIteration:
            self.close()
            raise ValueError("Raw CSV contains no samples") from None
        except Exception:
            self.close()
            raise
        self._origin = self._pending.host_monotonic_s
        self._start, self._wall = self.clock(), self.wall_clock()
        self.eof = False
        return self

    def read_samples(self):
        if self._iterator is None:
            raise RuntimeError("Replay source is not open")
        now = self.clock()
        result = []
        while self._pending is not None and len(result) < 2048:
            sample = self._pending
            offset = (sample.host_monotonic_s - self._origin) / self.speed
            if self._start + offset > now + 1e-9:
                break
            result.append(RawSample(sample.node_id, sample.values, self._start + offset,
                                    self._wall + offset, sample.status))
            try:
                self._pending = next(self._iterator)
            except StopIteration:
                self._pending, self.eof = None, True
        return result

    def close(self):
        iterator, self._iterator = self._iterator, None
        self._pending = None
        if iterator is not None:
            iterator.close()
