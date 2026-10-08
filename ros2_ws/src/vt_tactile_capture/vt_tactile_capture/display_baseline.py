"""Display-only tactile ADC reference; never modifies capture or ROS messages.

Reference subtraction is signed, unfiltered ADC arithmetic, not force/pressure
calibration.  A reference belongs to exactly one acquisition session and epoch.
The caller supplies local monotonic nanoseconds, not ROS or device timestamps.
"""

from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import warnings
from zipfile import BadZipFile, ZipFile

import numpy as np

from .grid import CELL_COUNT, SCHEMA_VERSION, SHAPE, normalize_state


COLLECTION_NS = 3_000_000_000
FRESH_NS = 200_000_000
MIN_SAMPLES = 10
MAX_SAMPLES = 120
MAX_FILE_FRAMES = 10_000


def _identity(value) -> tuple[str, int]:
    if (not isinstance(value, (tuple, list)) or len(value) != 2
            or not isinstance(value[0], str) or not value[0]
            or isinstance(value[1], (bool, np.bool_))
            or not isinstance(value[1], (int, np.integer)) or value[1] < 0):
        raise ValueError("Reference requires a nonempty session_id and nonnegative stream_epoch")
    return value[0], int(value[1])


def _masked_median(adc: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Bound temporary memory even for a 10,000-frame imported reference."""
    data = adc.reshape(len(adc), -1)
    masks = valid.reshape(len(valid), -1)
    result = np.full(SHAPE[0] * SHAPE[1], np.nan, dtype=np.float64)
    for offset in range(0, result.size, 128):
        chunk = data[:, offset:offset + 128].astype(np.float64)
        chunk[~masks[:, offset:offset + 128]] = np.nan
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="All-NaN slice encountered", category=RuntimeWarning)
            result[offset:offset + 128] = np.nanmedian(chunk, axis=0)
    return result.reshape(SHAPE)


def _array_header(archive: ZipFile, key: str):
    """Read only an NPY header before accepting potentially large NPZ arrays."""
    with archive.open(key + ".npy") as handle:
        version = np.lib.format.read_magic(handle)
        if version == (1, 0):
            return np.lib.format.read_array_header_1_0(handle)
        if version == (2, 0):
            return np.lib.format.read_array_header_2_0(handle)
        raise ValueError("Unsupported reference array format")


class DisplayBaseline:
    """Manual three-second zero-load reference for a ROS tactile viewer only."""

    def __init__(self, *, expected_hand=None):
        if expected_hand not in (None, "left", "right"):
            raise ValueError("expected_hand must be left or right")
        self.expected_hand = expected_hand
        self.values: np.ndarray | None = None
        self.identity: tuple[str, int] | None = None
        self.collecting = False
        self.status = "No ADC reference; this is not pressure calibration."
        self.samples_count = 0
        self._samples = deque(maxlen=MAX_SAMPLES)
        self._start_ns = None
        self._last_sample_ns = None
        self._last_sequence = None
        self._source = None

    def invalidate(self) -> None:
        """Clear local reference and pending capture; never reset the hardware."""
        self.values = None
        self.identity = None
        self.collecting = False
        self.samples_count = 0
        self._samples.clear()
        self._start_ns = None
        self._last_sample_ns = None
        self._last_sequence = None
        self._source = None
        self.status = "No ADC reference; acquire an unloaded reference (not pressure)."

    def start(self, identity, now_ns: int) -> None:
        """Refresh clears the previous reference immediately to prevent misuse."""
        self.invalidate()
        self.identity = _identity(identity)
        self._start_ns = int(now_ns)
        self.collecting = True
        self.status = "Collecting 3 s unloaded ADC reference; keep all tactile regions unpressed."

    def _cancel(self, reason: str) -> None:
        self.invalidate()
        self.status = "ADC reference cancelled: " + reason + "; acquire again."

    def observe(self, message, now_ns: int) -> None:
        """Accept each fresh valid source state at most once, in the UI thread."""
        now_ns = int(now_ns)
        try:
            message = normalize_state(message)
            if self.expected_hand is not None and getattr(message, "hand", "") != self.expected_hand:
                self.invalidate()
                self.status = "Hand mismatch; acquire a reference from the configured hand."
                return
            identity = _identity((message.session_id, message.stream_epoch))
            if message.schema_version != SCHEMA_VERSION or (message.rows, message.cols) != SHAPE:
                raise ValueError("unsupported state schema or shape")
            sequence = message.source_sequence
            connected, time_valid = message.connected, message.time_valid
            if not isinstance(connected, (bool, np.bool_)) or not isinstance(time_valid, (bool, np.bool_)):
                raise ValueError("invalid state health flags")
            if (isinstance(sequence, (bool, np.bool_))
                    or not isinstance(sequence, (int, np.integer)) or sequence < 0):
                raise ValueError("invalid source_sequence")
            adc = np.asarray(message.adc)
            mask = np.asarray(message.valid_mask)
            if adc.shape != (CELL_COUNT,) or mask.shape != (CELL_COUNT,):
                raise ValueError("invalid state array shape")
            if (adc.dtype.kind not in "iu" or np.any(adc < 0) or np.any(adc > 65535)
                    or mask.dtype.kind not in "biu" or np.any((mask != 0) & (mask != 1))):
                raise ValueError("invalid ADC or validity values")
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            if self.collecting:
                self._cancel("invalid state: " + str(exc))
            return

        if self.identity is not None and identity != self.identity:
            self.invalidate()
            self.status = "Session/epoch changed; acquire a new ADC reference (not pressure)."
            return
        if not connected or not time_valid:
            reason = "disconnected" if not connected else "invalid source time"
            if self.collecting:
                self._cancel(reason)
            elif self.values is not None:
                self.status = "ADC reference retained; " + reason + ". Not pressure calibration."
            return
        if not self.collecting:
            if self.values is not None:
                self.status = "ADC reference ready; signed ADC delta only, not pressure."
            return
        if now_ns < self._start_ns or (self._last_sample_ns is not None and now_ns < self._last_sample_ns):
            self._cancel("local monotonic clock moved backwards")
            return
        if self._last_sequence is not None and sequence <= self._last_sequence:
            self.poll(now_ns)
            return
        # A delayed new sample cannot retroactively make an expired window fresh.
        if now_ns > self._start_ns + COLLECTION_NS:
            self.poll(now_ns)
            return
        self._last_sequence = int(sequence)
        if np.any(mask):
            sample = adc.astype(np.float64).reshape(SHAPE)
            sample[~mask.astype(bool).reshape(SHAPE)] = np.nan
            self._samples.append(sample)
            self.samples_count = len(self._samples)
            self._last_sample_ns = now_ns
        self.poll(now_ns)

    def poll(self, now_ns: int) -> None:
        """Complete/fail collection even when the source has stopped publishing."""
        if not self.collecting:
            return
        now_ns = int(now_ns)
        if now_ns < self._start_ns:
            self._cancel("local monotonic clock moved backwards")
            return
        if now_ns - self._start_ns < COLLECTION_NS:
            return
        if self.samples_count < MIN_SAMPLES:
            self._cancel(f"need at least {MIN_SAMPLES} fresh source samples")
            return
        if self._last_sample_ns is None or not 0 <= now_ns - self._last_sample_ns <= FRESH_NS:
            self._cancel("no fresh valid sample in the last 200 ms")
            return
        samples = np.stack(self._samples)
        self.values = _masked_median(samples, np.isfinite(samples))
        self.collecting = False
        self._samples.clear()
        self._source = "manual_unloaded_3s"
        self.status = "ADC reference ready; signed ADC delta only, not pressure."

    def load(self, path, expected_identity) -> None:
        """Load a same-session capture archive; unrelated NPZ keys stay unread.

        Failure clears the old reference rather than silently displaying it as
        the newly requested one.  The JSON identity must cover every frame.
        """
        self.invalidate()
        expected = _identity(expected_identity)
        path = Path(path)
        try:
            metadata = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
            if not isinstance(metadata, dict):
                raise ValueError("Reference sidecar must be a JSON object")
            if (self.expected_hand is not None
                    and metadata.get("hand", self.expected_hand) != self.expected_hand):
                raise ValueError("Reference hand does not match the current stream")
            counts = metadata.get("session_epoch_frame_counts")
            if not isinstance(counts, dict) or len(counts) != 1:
                raise ValueError("Reference must contain exactly one session/epoch")
            key, count = next(iter(counts.items()))
            session, epoch_text = key.rsplit(":", 1)
            recorded = _identity((session, int(epoch_text)))
            if recorded != expected:
                raise ValueError("Reference session/epoch does not match the current stream")
            if isinstance(count, bool) or not isinstance(count, int) or not 0 < count <= MAX_FILE_FRAMES:
                raise ValueError("Reference frame count must be between 1 and 10000")
            with ZipFile(path) as archive:
                reference_shape = None
                for name, dtype in (("adc", np.dtype("uint16")), ("valid_mask", np.dtype("bool"))):
                    shape, _fortran_order, stored_dtype = _array_header(archive, name)
                    if shape not in ((count, *SHAPE), (count, 80, 80)) or stored_dtype != dtype:
                        raise ValueError(f"Reference {name} must have shape (frames,20,20) "
                                         f"or legacy (frames,80,80) and dtype {dtype}")
                    if reference_shape is not None and shape != reference_shape:
                        raise ValueError("Reference ADC and validity shapes must match")
                    reference_shape = shape
            with np.load(path, allow_pickle=False) as archive:
                values = _masked_median(archive["adc"][:, :SHAPE[0], :SHAPE[1]],
                                        archive["valid_mask"][:, :SHAPE[0], :SHAPE[1]])
            if not np.any(np.isfinite(values)):
                raise ValueError("Reference contains no valid ADC cells")
        except (OSError, ValueError, TypeError, KeyError, EOFError, BadZipFile) as exc:
            self.status = "ADC reference load failed: " + str(exc)
            raise ValueError(self.status) from exc
        self.values = values
        self.identity = expected
        self.samples_count = count
        self._source = str(path.resolve())
        self.status = "Loaded same-session ADC reference; signed ADC delta only, not pressure."
        if reference_shape[1:] == (80, 80):
            self.status += " Legacy 80x80 reference cropped to physical rows/columns 0..19."

    def subtract(self, values) -> np.ndarray:
        """Signed subtraction, retaining missing cells and negative responses."""
        if self.values is None:
            raise ValueError("No ADC reference available")
        values = np.asarray(values, dtype=np.float64)
        if values.shape != SHAPE:
            raise ValueError("Display values must have shape (20,20)")
        return values - self.values

    def metadata(self) -> dict:
        return {
            "schema_version": 2,
            "rows": SHAPE[0], "cols": SHAPE[1],
            "hand": self.expected_hand,
            "source": self._source,
            "identity": ({"session_id": self.identity[0], "stream_epoch": self.identity[1]}
                         if self.identity else None),
            "valid_cells": int(np.isfinite(self.values).sum()) if self.values is not None else 0,
            "samples_count": self.samples_count,
            "collecting": self.collecting,
            "status": self.status,
            "unit": "adc_counts",
            "semantics": "Display-only signed ADC reference subtraction, not pressure calibration",
        }
