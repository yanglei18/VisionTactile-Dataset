"""Unloaded-reference tactile response for display, independent of Qt and ROS.

This processor never changes acquisition data or estimates force/pressure.
References are explicit, belong to one stream identity, and remain fixed during
use. Per-cell source timestamps describe host receipt, not simultaneous hardware
sampling. Repeated rolling-cache values cannot become extra reference samples.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from numbers import Integral, Real

import numpy as np

from .grid import CELL_COUNT


COLLECTION_NS = 3_000_000_000
FRESH_NS = 200_000_000
MIN_SAMPLES = 10
MAX_SAMPLES = 120
MIN_NOISE_ADC = 3.0
NOISE_SIGMA_MULTIPLIER = 6.0
RELEASE_RATIO = 0.5
_MAX_TIME = np.iinfo(np.int64).max
# A noisy reference can intentionally make activation impossible within the ADC
# range. Retain that result when exporting; do not cap it and invent sensitivity.
_MAX_REFERENCE_THRESHOLD = 65535.0 * NOISE_SIGMA_MULTIPLIER * 1.4826


@dataclass(frozen=True)
class ReferenceProfile:
    """Portable detector coefficients, independent of devices and file paths.

    Schema 1 uses the current ADC units and hysteresis release ratio. Identity,
    model/layout compatibility, and the choice to reuse a reference belong to
    the application. Missing cells are explicit in ``valid_mask``; their finite
    numeric placeholders do not establish a reference for those cells.
    """

    baseline_adc: tuple[float, ...]
    activation_threshold_adc: tuple[float, ...]
    valid_mask: tuple[bool, ...]
    schema_version: int = 1

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("reference schema_version must be 1")

        def numbers(values, name, maximum):
            if not isinstance(values, (tuple, list, np.ndarray)):
                raise ValueError(f"{name} must contain 400 numeric values")
            try:
                result = tuple(values)
            except TypeError as exc:
                raise ValueError(f"{name} must contain 400 numeric values") from exc
            if (len(result) != CELL_COUNT
                    or any(isinstance(value, (bool, np.bool_))
                           or not isinstance(value, Real)
                           or not 0 <= value <= maximum for value in result)):
                raise ValueError(f"{name} must contain 400 finite values in [0, {maximum}]")
            return tuple(float(value) for value in result)

        baseline = numbers(self.baseline_adc, "baseline_adc", 65535.)
        threshold = numbers(self.activation_threshold_adc, "activation_threshold_adc",
                            _MAX_REFERENCE_THRESHOLD)
        if not isinstance(self.valid_mask, (tuple, list, np.ndarray)):
            raise ValueError("reference valid_mask must contain 400 booleans")
        try:
            mask = tuple(self.valid_mask)
        except TypeError as exc:
            raise ValueError("reference valid_mask must contain 400 booleans") from exc
        if (len(mask) != CELL_COUNT
                or any(not isinstance(value, (bool, np.bool_)) for value in mask)
                or not any(mask)):
            raise ValueError("reference valid_mask needs 400 booleans and at least one valid cell")
        if any(valid and value < MIN_NOISE_ADC for valid, value in zip(mask, threshold)):
            raise ValueError(f"valid reference thresholds must be at least {MIN_NOISE_ADC} ADC")
        object.__setattr__(self, "baseline_adc", baseline)
        object.__setattr__(self, "activation_threshold_adc", threshold)
        object.__setattr__(self, "valid_mask", tuple(bool(value) for value in mask))

    def to_dict(self):
        """Return an independent JSON-compatible object without NaN placeholders."""
        return {"schema_version": self.schema_version,
                "baseline_adc": list(self.baseline_adc),
                "activation_threshold_adc": list(self.activation_threshold_adc),
                "valid_mask": list(self.valid_mask)}

    @classmethod
    def from_dict(cls, value):
        fields = {"schema_version", "baseline_adc", "activation_threshold_adc", "valid_mask"}
        if not isinstance(value, dict) or value.keys() != fields:
            raise ValueError("reference profile fields do not match schema 1")
        return cls(**value)


@dataclass(frozen=True)
class ResponseSettings:
    """Display range and signal polarity, not a physical pressure calibration."""

    display_range_adc: float = 1000.0
    direction: str = "both"

    def __post_init__(self):
        value = self.display_range_adc
        if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
                or not np.isfinite(value) or not 0 < value <= 65535):
            raise ValueError("display_range_adc must be finite in (0, 65535]")
        if self.direction not in ("both", "increase", "decrease"):
            raise ValueError("direction must be both, increase, or decrease")
        object.__setattr__(self, "display_range_adc", float(value))


@dataclass(frozen=True)
class ResponseStatus:
    phase: str = "unreferenced"
    progress: float = 0.0
    reference_cells: int = 0
    active_cells: int = 0
    reason: str = ""
    reference_source: str = ""


def _clock(value):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral)
            or not 0 <= value <= _MAX_TIME):
        raise ValueError("monotonic time must be a nonnegative signed 64-bit integer")
    return int(value)


def _identity(value):
    if (not isinstance(value, (tuple, list)) or len(value) < 2
            or not isinstance(value[0], str) or not value[0]):
        raise ValueError("identity requires a session ID and nonnegative epoch/generations")
    return (value[0], *(_clock(part) for part in value[1:]))


def _inputs(adc, valid_mask):
    values, mask = np.asarray(adc), np.asarray(valid_mask)
    if values.shape != (CELL_COUNT,) or values.dtype.kind not in "iuf":
        raise ValueError("adc must contain 400 numeric raw values")
    if (mask.shape != (CELL_COUNT,) or mask.dtype.kind not in "biu"
            or np.any((mask != 0) & (mask != 1))):
        raise ValueError("valid_mask must contain 400 boolean values")
    mask = mask.astype(bool, copy=True)
    values = values.astype(np.float64, copy=False)
    if np.any(mask & (~np.isfinite(values) | (values < 0) | (values > 65535))):
        raise ValueError("valid adc values must be finite in [0, 65535]")
    return values, mask


class TactileResponseProcessor:
    """Manual reference acquisition followed by fixed, noise-gated response.

    Each cell contributes at most one sample in each of 120 time bins over the
    three-second acquisition. This bounds memory and retains coverage of the
    whole reference window even for high-rate sources. At least ten occupied
    bins and a source sample within the final 200 ms are required per cell.

    Activation threshold = max(3 ADC, 6 * robust sigma, 99th-percentile idle
    deviation). Release is at half that threshold. Active response subtracts
    the release threshold, so these values are noise-gated ADC changes, not
    untouched signed deltas. None of these constants claim a hardware noise
    specification or a calibrated touch-detection accuracy.
    """

    def __init__(self, settings=None):
        self._settings = settings if settings is not None else ResponseSettings()
        if not isinstance(self._settings, ResponseSettings):
            raise ValueError("settings must be ResponseSettings")
        self.reset()

    @property
    def settings(self):
        return self._settings

    @property
    def status(self):
        return ResponseStatus(self._phase, self._progress,
                              int(self._reference_mask.sum()), int(self._active.sum()),
                              self._reason, self._reference_source)

    def reset(self, reason=""):
        self._identity = None
        self._phase, self._progress, self._reason = "unreferenced", 0.0, str(reason)
        self._reference_source = ""
        self._start_ns = self._last_clock = None
        self._samples = None
        self._last_samples = np.zeros(CELL_COUNT, dtype=np.int64)
        self._reference = np.full(CELL_COUNT, np.nan)
        self._threshold = np.full(CELL_COUNT, np.nan)
        self._reference_mask = np.zeros(CELL_COUNT, dtype=bool)
        self._active = np.zeros(CELL_COUNT, dtype=bool)

    def export_reference(self):
        """Export only a completed reference, without current contact or clocks."""
        if self._phase != "ready":
            raise ValueError("a completed reference is required for export")
        return ReferenceProfile(
            np.where(self._reference_mask, self._reference, 0.),
            np.where(self._reference_mask, self._threshold, 0.),
            self._reference_mask,
        )

    def load_reference(self, profile, identity, now_ns, *, source="saved"):
        """Explicitly bind a validated profile to the current stream.

        No measurements are collected or inferred here. Validate everything
        before replacing the current state, including an in-progress reference.
        """
        if not isinstance(profile, ReferenceProfile):
            raise ValueError("profile must be a ReferenceProfile")
        identity, now_ns = _identity(identity), _clock(now_ns)
        if source not in ("measured", "preset", "saved", "default"):
            raise ValueError("reference source must be measured, preset, saved, or default")
        if self._last_clock is not None and now_ns < self._last_clock:
            raise ValueError("reference load time must not move backwards")
        baseline = np.asarray(profile.baseline_adc, dtype=np.float64)
        threshold = np.asarray(profile.activation_threshold_adc, dtype=np.float64)
        mask = np.asarray(profile.valid_mask, dtype=bool)
        self.reset()
        self._reference, self._threshold, self._reference_mask = baseline, threshold, mask
        self._identity, self._last_clock = identity, now_ns
        self._phase, self._progress, self._reference_source = "ready", 1., source

    def configure(self, *, display_range_adc=None, direction=None):
        changes = {}
        if display_range_adc is not None:
            changes["display_range_adc"] = display_range_adc
        if direction is not None:
            changes["direction"] = direction
        updated = replace(self._settings, **changes)
        if updated.direction != self._settings.direction:
            self.clear_activity()
        self._settings = updated

    def clear_activity(self):
        """Forget live hysteresis after an outage, retaining the fixed reference."""
        self._active.fill(False)

    def begin_reference(self, identity, now_ns):
        identity, now_ns = _identity(identity), _clock(now_ns)
        self.reset()
        self._identity = identity
        self._start_ns = self._last_clock = now_ns
        self._phase = "collecting"
        self._samples = np.full((MAX_SAMPLES, CELL_COUNT), np.nan)

    def cancel_reference(self, reason="interrupted"):
        """Cancel an unfinished reference; completed references stay fixed."""
        if self._phase == "collecting":
            self._phase, self._reason = "failed", str(reason)
            self._samples = None
            self._active.fill(False)

    def _advance(self, now_ns):
        now_ns = _clock(now_ns)
        if self._last_clock is not None and now_ns < self._last_clock:
            self.reset("clock_backwards")
            self._last_clock = now_ns
            return False
        self._last_clock = now_ns
        if self._phase == "collecting":
            self._progress = min(1., (now_ns - self._start_ns) / COLLECTION_NS)
        return True

    def _bind(self, identity):
        identity = _identity(identity)
        if self._identity is not None and identity != self._identity:
            now = self._last_clock
            self.reset("stream_changed")
            self._last_clock = now
        self._identity = identity

    def observe(self, adc, valid_mask, sample_ns, identity, now_ns):
        """Observe newly delivered raw data, independent of display polling."""
        values, mask = _inputs(adc, valid_mask)
        stamps = np.asarray(sample_ns)
        if (stamps.shape != (CELL_COUNT,) or stamps.dtype.kind not in "iu"
                or np.any(stamps < 0) or np.any(stamps > _MAX_TIME)):
            raise ValueError("sample_ns must contain 400 nonnegative source timestamps")
        identity, now_ns = _identity(identity), _clock(now_ns)
        if not self._advance(now_ns):
            return
        self._bind(identity)
        if self._phase != "collecting":
            return
        if now_ns > self._start_ns + COLLECTION_NS:
            self.poll(now_ns)
            return  # A late sample cannot retroactively refresh the reference.
        stamps = stamps.astype(np.int64, copy=False)
        usable = (mask & (stamps > self._start_ns) & (stamps <= now_ns)
                  & (stamps >= max(0, now_ns - FRESH_NS))
                  & (stamps > self._last_samples))
        cells = np.flatnonzero(usable)
        if cells.size:
            bins = ((stamps[cells] - self._start_ns - 1) * MAX_SAMPLES // COLLECTION_NS)
            self._samples[bins, cells] = values[cells]
            self._last_samples[cells] = stamps[cells]
        self.poll(now_ns)

    def poll(self, now_ns):
        if not self._advance(now_ns) or self._phase != "collecting":
            return self.status
        if now_ns < self._start_ns + COLLECTION_NS:
            return self.status
        enough = np.isfinite(self._samples).sum(axis=0) >= MIN_SAMPLES
        ready = enough & (self._last_samples >= max(1, now_ns - FRESH_NS))
        if not np.any(ready):
            self.cancel_reference("stale_reference" if np.any(enough) else "insufficient_samples")
            return self.status
        samples = self._samples[:, ready]
        baseline = np.nanmedian(samples, axis=0)
        deviations = np.abs(samples - baseline)
        sigma = 1.4826 * np.nanmedian(deviations, axis=0)
        envelope = np.nanquantile(deviations, .99, axis=0)
        self._reference[ready] = baseline
        self._threshold[ready] = np.maximum.reduce([
            np.full(baseline.shape, MIN_NOISE_ADC),
            NOISE_SIGMA_MULTIPLIER * sigma, envelope,
        ])
        self._reference_mask = ready
        self._samples = None
        self._phase, self._progress, self._reason = "ready", 1., ""
        self._reference_source = "measured"
        return self.status

    def transform(self, adc, valid_mask, identity, now_ns):
        """Produce a fresh display frame without sampling or changing raw ADC."""
        values, mask = _inputs(adc, valid_mask)
        identity, now_ns = _identity(identity), _clock(now_ns)
        if self._advance(now_ns):
            self._bind(identity)
            self.poll(now_ns)
        response = np.zeros(CELL_COUNT, dtype=np.float64)
        ready = self._phase == "ready"
        if ready:
            mask &= self._reference_mask
            delta = values - self._reference
            if self._settings.direction == "both":
                amplitude = np.abs(delta)
            elif self._settings.direction == "increase":
                amplitude = np.maximum(delta, 0.)
            else:
                amplitude = np.maximum(-delta, 0.)
            off = RELEASE_RATIO * self._threshold
            self._active &= mask & (amplitude > off)
            self._active |= mask & (amplitude > self._threshold)
            response[self._active] = amplitude[self._active] - off[self._active]
        else:
            self._active.fill(False)
        return {"values": response.tolist(), "valid_mask": mask.tolist(),
                "levels": [0., self._settings.display_range_adc],
                "value_kind": "tactile_response", "reference_ready": ready}
