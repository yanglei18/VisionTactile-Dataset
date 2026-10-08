"""Bounded per-joint continuity for signed hinge-angle representations.

Only integer multiples of 2*pi are added. This is not smoothing, drift
correction, an anatomical constraint, or evidence of the true joint angle.
Choosing the nearest branch assumes less than pi of motion between updates.
"""
from dataclasses import dataclass
import math
from numbers import Integral, Real
from typing import Hashable


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number")
    try:
        value = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite real number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite real number")
    return value


def _slot(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value < 16:
        raise ValueError("slot must be an integer from 0 to 15")
    return int(value)


@dataclass(frozen=True)
class HingeAngleResult:
    angle_rad: float
    branch_corrected: bool
    branch_turns: int
    reset_reason: str | None = None


@dataclass(frozen=True)
class _State:
    timestamp_s: float
    binding_token: Hashable
    raw_angle_rad: float
    result: HingeAngleResult


class HingeAngleContinuity:
    """Keep one sample per MANO slot, scoped by binding and time continuity.

    ``binding_token`` must be immutable and hashable; callers should include
    their session, calibration/profile and relevant sensor epochs in it.
    A changed token, backward timestamp, or gap greater than ``max_gap_s``
    starts from the supplied raw angle, including an initially negative one.

    Repeating the same binding, timestamp and raw angle returns the cached
    result. A different raw angle at the same timestamp is processed normally
    against the latest branch; it must not freeze a newly solved observation.
    Inputs are always validated first.
    ``branch_corrected`` means this result differs from the raw angle by a
    nonzero whole-turn offset, not necessarily that a crossing occurred now.
    """

    def __init__(self, *, max_gap_s=0.25):
        self.max_gap_s = _finite(max_gap_s, "max_gap_s")
        if self.max_gap_s <= 0:
            raise ValueError("max_gap_s must be positive")
        self._states: list[_State | None] = [None] * 16

    def reset(self, slot=None):
        """Forget all branches, or just one explicitly selected slot."""
        if slot is None:
            self._states = [None] * 16
        else:
            self._states[_slot(slot)] = None

    def update(self, slot, raw_angle_rad, timestamp_s, binding_token):
        """Return the nearest continuous 2*pi branch and its diagnostics."""
        slot = _slot(slot)
        raw_angle = angle = _finite(raw_angle_rad, "raw_angle_rad")
        timestamp = _finite(timestamp_s, "timestamp_s")
        try:
            hash(binding_token)
        except TypeError as exc:
            raise ValueError("binding_token must be immutable and hashable") from exc
        previous = self._states[slot]
        reason = None
        if previous is None:
            reason = "initialized"
        elif binding_token != previous.binding_token:
            reason = "binding_changed"
        elif timestamp == previous.timestamp_s and raw_angle == previous.raw_angle_rad:
            return previous.result
        elif timestamp < previous.timestamp_s:
            reason = "time_reversed"
        elif timestamp - previous.timestamp_s > self.max_gap_s:
            reason = "gap_exceeded"

        turns = 0
        if reason is None:
            # Scale before subtraction to avoid overflowing opposite finite
            # inputs. Exact half-turn ties are ambiguous; round is deterministic.
            turns = round(previous.result.angle_rad / math.tau - angle / math.tau)
            angle += turns * math.tau
            if not math.isfinite(angle):
                raise ValueError("unwrapped angle must remain finite")
        result = HingeAngleResult(angle, bool(turns), turns, reason)
        self._states[slot] = _State(timestamp, binding_token, raw_angle, result)
        return result
