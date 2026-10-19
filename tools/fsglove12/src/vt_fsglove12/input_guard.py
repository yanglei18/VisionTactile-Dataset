"""Causal gyro-component and acceleration-vector rejection before VQF.

This is an engineering guard, not a physical saturation detector. Accepted
measurements pass unchanged: there is no smoothing, interpolation, or lookahead.
Raw records remain the caller's original data. A missing ``input_filter``
section disables the guard for compatibility with historical configurations.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from numbers import Integral, Real

import numpy as np


DEFAULTS = dict(enabled=True, boundary_lsb=32760, gyro_jump_dps=120.0,
                accel_jump_g=1.5, confirmation_samples=2, max_hold_samples=2,
                max_invalid_s=0.25)
REVISION = "per_axis_gyro_continuity_v2"
_GRAVITY = 9.80665
_MIN_ACCEL_NORM_M_S2 = 1e-6
_AXES = ("x", "y", "z")
_PASS_ACTIONS = ("pass", "pass", "pass")
_NO_REASONS = (None, None, None)


def _positive_number(value):
    try:
        return (not isinstance(value, bool) and isinstance(value, Real)
                and math.isfinite(value) and value > 0)
    except OverflowError:
        return False


def guard_settings(config):
    """Return independent, validated effective ``input_filter`` settings.

    ``confirmation_samples`` is deliberately fixed at two: a first candidate
    is rejected and a consistent next finite sample confirms a reported step.
    A gyro coding boundary is diagnostic evidence, not proof of a corrupt value
    or zero angular velocity. Smoothly approaching it retains the reported rate.
    ``max_hold_samples`` may be zero to disable gyro hold-last fallback.
    """
    if not isinstance(config, dict):
        raise ValueError("input guard config must be an object")
    options = config.get("input_filter", {})
    if not isinstance(options, dict) or set(options) - set(DEFAULTS):
        raise ValueError("input_filter requires supported option fields")
    settings = {**DEFAULTS, **options}
    if "input_filter" not in config:
        settings["enabled"] = False
    if type(settings["enabled"]) is not bool:
        raise ValueError("input_filter enabled must be boolean")
    boundary = settings["boundary_lsb"]
    if (isinstance(boundary, bool) or not isinstance(boundary, Integral)
            or not 1 <= boundary <= 32768):
        raise ValueError("input_filter boundary_lsb must be an integer in [1, 32768]")
    confirmation = settings["confirmation_samples"]
    if (isinstance(confirmation, bool) or not isinstance(confirmation, Integral)
            or confirmation != 2):
        raise ValueError("input_filter confirmation_samples must be integer 2")
    hold = settings["max_hold_samples"]
    if isinstance(hold, bool) or not isinstance(hold, Integral) or hold < 0:
        raise ValueError("input_filter max_hold_samples must be a nonnegative integer")
    for key in ("gyro_jump_dps", "accel_jump_g", "max_invalid_s"):
        if not _positive_number(settings[key]):
            raise ValueError(f"input_filter {key} must be finite and positive")
        settings[key] = float(settings[key])
    for key in ("boundary_lsb", "confirmation_samples", "max_hold_samples"):
        settings[key] = int(settings[key])
    return settings


def _node_key(node):
    if isinstance(node, str) and node in {str(n) for n in range(1, 13)}:
        return node
    if not isinstance(node, bool) and isinstance(node, Integral) and 1 <= node <= 12:
        return str(int(node))
    raise ValueError("input guard node ID must be in 1..12")


def _vector(values, name):
    vector = np.array(values, dtype=float, copy=True)
    if vector.shape != (3,):
        raise ValueError(f"input guard {name} must contain three values")
    return vector


@dataclass
class _VectorState:
    accepted: np.ndarray | None = None
    candidate: np.ndarray | None = None
    invalid_run: int = 0
    rejected_samples: int = 0


@dataclass
class _AxisState:
    accepted: float | None = None
    candidate: float | None = None
    invalid_run: int = 0
    rejected_samples: int = 0
    held_samples: int = 0
    zero_motion_samples: int = 0
    boundary_samples: int = 0
    boundary_now: bool = False


@dataclass
class _GyroState:
    axes: tuple[_AxisState, ...] = field(default_factory=lambda: tuple(_AxisState() for _ in _AXES))
    # Compatibility counters: any rejected component rejects one whole input
    # row for accounting/validity, but must not replace its good components.
    invalid_run: int = 0
    rejected_samples: int = 0


@dataclass
class _NodeState:
    gyro: _GyroState = field(default_factory=_GyroState)
    accel: _VectorState = field(default_factory=_VectorState)
    raw_samples: int = 0
    rejected_samples: int = 0


class ImuInputGuard:
    """Bounded gyro substitution and acceleration skipping, before six-axis VQF.

    Rejection runs count nominal per-node sample periods, including every row
    in a USB batch. Call ``reset`` whenever the corresponding VQF is reset by a
    clock rollback or long gap. No host timestamps or device writes are used.
    For long gyro-axis rejection runs, return that component of the current
    VQF bias (or zero if none was supplied). Other gyro components pass through
    unchanged; an invalid axis must not erase their real angular motion. Rejected
    acceleration is zero, VQF's sentinel for skipping its acceleration update;
    holding a previous gravity vector would introduce fictitious observations.
    """
    def __init__(self, config):
        self.settings = guard_settings(config)
        rate = config.get("sample_rate_hz", 120.0)
        if not _positive_number(rate):
            raise ValueError("input guard sample_rate_hz must be finite and positive")
        self.sample_period = 1.0 / float(rate)
        self._gyro_jump = math.radians(self.settings["gyro_jump_dps"])
        self._accel_jump = self.settings["accel_jump_g"] * _GRAVITY
        self._states = {}

    def reset(self, node=None):
        """Clear accepted inputs, pending candidates, runs, and epoch counters."""
        if node is None:
            self._states.clear()
        else:
            self._states.pop(_node_key(node), None)

    @staticmethod
    def _decide(state, vector, raw, threshold, boundary, name):
        reason = None
        # Convert before abs to avoid overflow at np.int16(-32768).
        if any(abs(int(value)) >= boundary for value in raw):
            reason = f"{name}_boundary"
        elif not np.isfinite(vector).all():
            reason = f"{name}_nonfinite"
        elif name == "accel" and np.linalg.norm(vector) <= _MIN_ACCEL_NORM_M_S2:
            reason = "accel_zero_norm"
        if reason is not None:
            # Hard-invalid input can never become a confirmed step, and also
            # interrupts any previous candidate's consecutive confirmation.
            state.candidate = None
        elif (state.accepted is not None
              and np.max(np.abs(vector - state.accepted)) > threshold):
            if (state.candidate is None
                    or np.max(np.abs(vector - state.candidate)) > threshold):
                state.candidate = vector.copy()
                reason = f"{name}_jump"
        if reason is None:
            state.accepted = vector.copy()
            state.candidate = None
            state.invalid_run = 0
        else:
            state.invalid_run += 1
            state.rejected_samples += 1
        return reason

    def _decide_gyro(self, state, vector, raw):
        values = tuple(float(value) for value in vector)
        boundary, threshold = self.settings["boundary_lsb"], self._gyro_jump
        for index, axis in enumerate(state.axes):
            axis.boundary_now = abs(int(raw[index])) >= boundary
            axis.boundary_samples += int(axis.boundary_now)
        # The common all-good case avoids candidate/fallback allocations and
        # NumPy reductions. A continuous approach to a boundary is not a spike.
        # An initial boundary with no accepted baseline still needs confirmation.
        if all(math.isfinite(value)
               and ((axis.accepted is None and not axis.boundary_now)
                    or axis.accepted is not None and abs(value - axis.accepted) <= threshold)
               for axis, value in zip(state.axes, values)):
            for axis, value in zip(state.axes, values):
                axis.accepted, axis.candidate, axis.invalid_run = value, None, 0
            state.invalid_run = 0
            return _NO_REASONS
        reasons = []
        for axis, value in zip(state.axes, values):
            reason = None
            if not math.isfinite(value):
                reason = "gyro_nonfinite"
                axis.candidate = None
            elif ((axis.accepted is None and axis.boundary_now)
                  or axis.accepted is not None and abs(value - axis.accepted) > threshold):
                if (axis.candidate is None or abs(value - axis.candidate) > threshold
                        or axis.boundary_now and value * axis.candidate <= 0.):
                    axis.candidate = value
                    reason = "gyro_boundary" if axis.boundary_now else "gyro_jump"
                # A consistent subsequent finite value confirms only the
                # reported rate, NOT absence of saturation. Do not erase a
                # sustained rotation by replacing its boundary plateau with bias.
            if reason is None:
                axis.accepted, axis.candidate, axis.invalid_run = value, None, 0
            else:
                axis.invalid_run += 1
                axis.rejected_samples += 1
            reasons.append(reason)
        if any(reasons):
            state.invalid_run += 1
            state.rejected_samples += 1
        else:
            state.invalid_run = 0
        return reasons

    def _status(self, state, gyro_action, accel_action, reasons,
                gyro_actions=_PASS_ACTIONS, gyro_reasons=_NO_REASONS):
        gyro, accel = state.gyro, state.accel
        enabled = self.settings["enabled"]
        valid = not enabled or (
            all(axis.accepted is not None for axis in gyro.axes) and accel.accepted is not None
            and max(gyro.invalid_run, accel.invalid_run) * self.sample_period
            <= self.settings["max_invalid_s"])
        return dict(enabled=enabled, revision=REVISION, gyro_action=gyro_action, accel_action=accel_action,
                    gyro_rejected=gyro_action != "pass", accel_rejected=accel_action != "pass",
                    reasons=reasons, raw_samples=state.raw_samples,
                    rejected_samples=state.rejected_samples,
                    gyro_rejected_samples=gyro.rejected_samples,
                    accel_rejected_samples=accel.rejected_samples,
                    gyro_run=gyro.invalid_run, accel_run=accel.invalid_run,
                    gyro_rejected_axes=[name for name, action in zip(_AXES, gyro_actions) if action != "pass"],
                    gyro_action_by_axis=dict(zip(_AXES, gyro_actions)),
                    gyro_reasons_by_axis=dict(zip(_AXES, gyro_reasons)),
                    gyro_run_by_axis={name: axis.invalid_run for name, axis in zip(_AXES, gyro.axes)},
                    gyro_rejected_samples_by_axis={name: axis.rejected_samples for name, axis in zip(_AXES, gyro.axes)},
                    gyro_held_samples_by_axis={name: axis.held_samples for name, axis in zip(_AXES, gyro.axes)},
                    gyro_zero_motion_samples_by_axis={name: axis.zero_motion_samples for name, axis in zip(_AXES, gyro.axes)},
                    gyro_boundary_axes=[name for name, axis in zip(_AXES, gyro.axes) if axis.boundary_now],
                    gyro_boundary_integrated_axes=[name for name, axis, action in zip(_AXES, gyro.axes, gyro_actions)
                                                   if axis.boundary_now and action == "pass"],
                    gyro_boundary_samples_by_axis={name: axis.boundary_samples for name, axis in zip(_AXES, gyro.axes)},
                    boundary_is_physical_saturation_proof=False,
                    orientation_valid=bool(valid))

    def process(self, node_id, raw_values, gyro_rad_s, accel_m_s2, *, gyro_bias_rad_s=None):
        """Return detached SI vectors for VQF plus a raw-value-free status dict."""
        key = _node_key(node_id)
        gyro = _vector(gyro_rad_s, "gyro_rad_s")
        accel = _vector(accel_m_s2, "accel_m_s2")
        # Disabled mode must not introduce rejection or alter legacy values,
        # even for rail events, zero acceleration, or nonfinite SI vectors.
        if not self.settings["enabled"]:
            state = self._states.get(key)
            if state is None:
                state = self._states[key] = _NodeState()
            state.raw_samples += 1
            return gyro, accel, self._status(state, "pass", "pass", [])
        raw = tuple(raw_values)
        if len(raw) != 6 or any(isinstance(value, bool) or not isinstance(value, Integral)
                                or not -32768 <= value <= 32767 for value in raw):
            raise ValueError("input guard raw_values must contain six int16 integers")
        state = self._states.get(key)
        if state is None:
            state = self._states[key] = _NodeState()
        state.raw_samples += 1
        gyro_reasons = self._decide_gyro(state.gyro, gyro, raw[:3])
        # Retain the legacy aggregate reason precedence; the per-axis field
        # records additional simultaneous failures without hiding any of them.
        gyro_reason = next((reason for reason in ("gyro_boundary", "gyro_nonfinite", "gyro_jump")
                            if reason in gyro_reasons), None)
        accel_reason = self._decide(state.accel, accel, raw[3:], self._accel_jump,
                                    self.settings["boundary_lsb"], "accel")
        gyro_action, accel_action = "pass", "pass"
        gyro_actions = _PASS_ACTIONS
        if gyro_reason is not None:
            gyro_actions = list(_PASS_ACTIONS)
            bias = None
            for index, (axis, reason) in enumerate(zip(state.gyro.axes, gyro_reasons)):
                if reason is None:
                    continue
                if axis.accepted is not None and axis.invalid_run <= self.settings["max_hold_samples"]:
                    gyro[index] = axis.accepted
                    gyro_actions[index] = "held"
                    axis.held_samples += 1
                else:
                    if bias is None:
                        bias = (np.zeros(3) if gyro_bias_rad_s is None
                                else _vector(gyro_bias_rad_s, "gyro_bias_rad_s"))
                    gyro[index] = bias[index] if math.isfinite(bias[index]) else 0.
                    gyro_actions[index] = "zero_motion"
                    axis.zero_motion_samples += 1
            # Legacy consumers understand one severity/action for the frame;
            # its scope is now explicitly described by gyro_action_by_axis.
            gyro_action = "zero_motion" if "zero_motion" in gyro_actions else "held"
        if accel_reason is not None:
            accel = np.zeros(3)
            accel_action = "skip_accel"
        reasons = [reason for reason in (gyro_reason, accel_reason) if reason is not None]
        if reasons:
            state.rejected_samples += 1
        return gyro, accel, self._status(state, gyro_action, accel_action, reasons, gyro_actions, gyro_reasons)
