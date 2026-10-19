"""Calibrated current projection plus independently validated heading evidence.

QMT estimates heading from excited backward windows, never a single idealized
hinge pose. No evidence means keeping the calibrated increment, not freezing
the current angle or bootstrapping a new yaw from installation/model error.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .config import digest
from .heading_correction import CausalHeadingCorrector, HINGES, _finite, _rotation, _wrap
from .heading_window import FunctionalHeadingWindow, WINDOW_DEFAULTS
from .upstream_functional import FunctionalDependencyError, KIND, _project, _same_binding


METHOD = "qmt_excited_window_functional_hinge_v3"
DEFAULTS = dict(max_age_s=.15, max_pair_skew_s=.03, max_gap_s=.25,
                rating_min=.2, residual_max_deg=15.,
                smoothing_tau_s=.75, max_correction_rate_deg_s=5.,
                max_parameter_angle_rate_deg_s=10., **WINDOW_DEFAULTS)
_FINGERS = {slot: finger for finger, slot in HINGES.items()}
_ANCHORS = ("session_id", "upstream_state_sha256", "config_sha256", "rest_sha256")


@dataclass
class _HingeState:
    context: str
    geometry: tuple
    identity: tuple | None = None
    source_time: float | None = None
    frame_time: float | None = None
    delta: float = 0.
    observer: FunctionalHeadingWindow | None = None
    angle: float | None = None
    angle_time: float | None = None
    quaternions: tuple | None = None


class _Unavailable(ValueError):
    def __init__(self, reason, *, hard=True):
        super().__init__(reason)
        self.hard = hard


class _FrameProjection:
    """Reuse QMT projections only within ONE current sensor pair.

    QMT supports batches natively. Projecting the old/proposed references
    together removes Python overhead without changing the heading estimator,
    its acceptance tests, or the angle definition. Never reuse across frames.
    """

    def __init__(self, q1, q2, axis):
        self.parent_inverse = q1.inv()
        self.child, self.axis = q2, axis
        self.cache = {}

    def one(self, delta):
        if delta not in self.cache:
            self.cache[delta] = tuple(float(v) for v in _project(
                self.parent_inverse * Rotation.from_rotvec([0., 0., delta]) * self.child,
                self.axis))
        return self.cache[delta]

    def pair(self, old, proposed):
        missing = list(dict.fromkeys(v for v in (old, proposed) if v not in self.cache))
        if len(missing) == 1:
            self.one(missing[0])
        elif missing:
            rotations = np.zeros((len(missing), 3))
            rotations[:, 2] = missing
            angles, residuals = _project(
                self.parent_inverse * Rotation.from_rotvec(rotations) * self.child, self.axis)
            self.cache.update((delta, (float(angle), float(residual)))
                              for delta, angle, residual in zip(missing, angles, residuals))
        return self.cache[old], self.cache[proposed]


class FunctionalHingeEstimator:
    """Estimate slots 2/5/8/11/15 without corrected-to-raw fallback.

    ``valid`` requires recent independent window evidence and usable input.
    ``display_available`` allows a fresh calibrated projection even without
    that evidence, explicitly degraded. Heading confidence expiry does not
    turn fresh sensor data into an outage. No absolute heading is observed.
    """

    def __init__(self, config=None):
        if config is None:
            config = {}
        if not isinstance(config, dict) or set(config) - set(DEFAULTS):
            raise ValueError("functional hinge requires supported option fields")
        self.settings = {**DEFAULTS, **config}
        if any(not _finite(v) or v <= 0 for v in self.settings.values()):
            raise ValueError("functional hinge options must be finite and positive")
        if self.settings["rating_min"] > 1 or self.settings["residual_max_deg"] >= 180:
            raise ValueError("invalid hinge observability/residual threshold")
        if (self.settings["heading_min_window_s"] > self.settings["heading_window_s"]
                or self.settings["heading_estimate_rate_hz"] > self.settings["heading_sample_rate_hz"]
                or self.settings["heading_sample_rate_hz"] > 30
                or self.settings["heading_window_s"] > 10):
            raise ValueError("invalid causal heading window/rate settings")
        self._states: dict[int, _HingeState] = {}

    def reset(self, slot=None):
        if slot is None:
            self._states.clear()
        else:
            self._states.pop(slot, None)

    @staticmethod
    def _project(q1, q2, axis, delta):
        return tuple(float(v) for v in _project(
            q1.inv() * Rotation.from_rotvec([0., 0., delta]) * q2, axis))

    @staticmethod
    def _continuous(state, raw_angle, stamp, max_gap, reset_reason):
        reason = reset_reason
        if state.angle is None:
            reason = reason or "initialized"
        elif state.angle_time is not None and stamp - state.angle_time > max_gap:
            reason = "gap_exceeded"
        turns = 0 if reason else round((state.angle - raw_angle) / math.tau)
        angle = raw_angle + turns * math.tau
        state.angle, state.angle_time = angle, stamp
        return angle, dict(branch_corrected=bool(turns), branch_turns=int(turns),
                           reset_reason=reason)

    def apply(self, slot, frame, profile, calibrator, binding):
        detail = dict(method=METHOD, slot=slot, valid=False, display_available=False,
                      quality="unavailable", status="unavailable", applied=False,
                      delta_rad=None, residual_deg=None, rating=None,
                      continuous_angle_rad=None,
                      continuity=dict(branch_corrected=False, branch_turns=0, reset_reason=None),
                      absolute_heading_observed=False, anatomical_accuracy_verified=False,
                      raw_values_modified=False, calibration_modified=False,
                      motion_smoothed=False, correction_only_smoothing=True,
                      qmt_optimization_steps=5,
                      fit_cost_semantics="qmt_pre_final_update_heading_alignment_not_anatomical_error",
                      heading_reference_origin="bound_installation_and_functional_neutral",
                      heading_quality="calibration_relative", heading_window={},
                      heading_updated=False)
        state = self._states.get(slot) if type(slot) is int else None
        try:
            if type(slot) is not int or slot not in _FINGERS:
                raise _Unavailable("unsupported_functional_hinge")
            finger = _FINGERS[slot]
            if not isinstance(profile, dict) or not isinstance(binding, dict):
                raise _Unavailable("missing_profile_or_binding")
            entry = profile.get("functional_fingers", {}).get(finger, {})
            reference = entry.get("slot_refs", {}).get(str(slot), {})
            if (profile.get("functional_enabled", True) is False
                    or entry.get("kind") != KIND or entry.get("finger") != finger
                    or reference.get("mode") != "hinge_projection"
                    or not _same_binding(entry.get("binding", {}), binding, slot)):
                raise _Unavailable("functional_binding_unavailable")
            dependencies = binding["dependency_nodes_by_slot"][str(slot)]
            if reference.get("dependency_nodes") != dependencies:
                raise _Unavailable("functional_dependency_mismatch")
            if (frame.get("session_id") != binding["session_id"]
                    or profile.get("session_id") != binding["session_id"]):
                raise _Unavailable("session_mismatch")
            now = frame.get("timestamp_s")
            if not _finite(now) or now < 0:
                raise _Unavailable("invalid_frame_time")
            # Only this reference and its installation/session/dependencies
            # identify the state. Another finger's capture/profile version is
            # not a physical change to this hinge's reference.
            context = digest(dict(reference=reference,
                anchors={name: binding.get(name) for name in _ANCHORS},
                epochs={node: binding["nodes_epochs"][node] for node in dependencies}))
            reset_reason = None
            if state is not None and context != state.context:
                self.reset(slot)
                state = None
                reset_reason = "binding_changed"
            if state is not None and state.frame_time is not None and now < state.frame_time:
                self.reset(slot)
                state = None
                reset_reason = "time_reversed"
            detail.update(parent_node=dependencies[0], child_node=dependencies[1])
            pair = [frame.get("nodes", {}).get(node) for node in dependencies]
            protected = False
            for node, data in zip(dependencies, pair):
                if not isinstance(data, dict):
                    raise _Unavailable("missing_parent_or_child", hard=False)
                if data.get("epoch") != binding["nodes_epochs"][node]:
                    raise _Unavailable("filter_epoch_changed")
                if data.get("orientation_valid") is False:
                    raise _Unavailable("input_orientation_invalid")
                age, index = data.get("age_s"), data.get("sample_index")
                if not _finite(age) or not 0 <= age <= self.settings["max_age_s"]:
                    raise _Unavailable("stale_parent_or_child", hard=False)
                if type(index) is not int or index <= 0:
                    raise _Unavailable("invalid_sample_index")
                protection = data.get("input_filter", {})
                protected |= bool(protection.get("gyro_rejected") or protection.get("accel_rejected"))
            if abs(pair[0]["age_s"] - pair[1]["age_s"]) > self.settings["max_pair_skew_s"]:
                raise _Unavailable("parent_child_source_time_skew", hard=False)
            raw = tuple(_rotation(data["quat_xyzw"]) for data in pair)
            stamp = float(now - max(data["age_s"] for data in pair))
            identity = tuple(data["sample_index"] for data in pair)
            if state is not None and state.identity is not None:
                if any(a < b for a, b in zip(identity, state.identity)):
                    raise _Unavailable("sample_index_reversed")
                if stamp < state.source_time - 1e-9:
                    raise _Unavailable("source_time_reversed")
                # A clock/age claim cannot turn repeated sensor identities
                # into a new reference. Only a genuinely new pair can start
                # a new continuity segment after a forward observation gap.
                if (all(a > b for a, b in zip(identity, state.identity))
                        and stamp - state.source_time > self.settings["max_gap_s"]):
                    self.reset(slot)
                    state = None
                    reset_reason = "gap_exceeded"
            if state is None:
                geometry = CausalHeadingCorrector._geometry(calibrator, slot - 1, slot, deepcopy(reference))
                state = _HingeState(context=context, geometry=geometry,
                                    observer=FunctionalHeadingWindow(self.settings))
                self._states[slot] = state
            hp, hc, rp, rc, axis = state.geometry
            q1, q2 = hp * raw[0] * rp, hc * raw[1] * rc
            quaternions = (q1.as_quat(), q2.as_quat())
            if state.quaternions is not None:
                quaternions = tuple(-q if np.dot(q, old) < 0 else q
                                    for q, old in zip(quaternions, state.quaternions))
                q1, q2 = (Rotation.from_quat(q) for q in quaternions)
            state.quaternions = quaternions
            projection = _FrameProjection(q1, q2, axis)
            fresh = state.identity is None or all(a > b for a, b in zip(identity, state.identity))
            dt = 0. if state.source_time is None else max(0., stamp - state.source_time)
            if fresh:
                state.identity, state.source_time = identity, stamp
            state.frame_time = float(now)
            v1, v2 = q1.apply(axis), q2.apply(axis)
            rating = float(np.linalg.norm(v1[:2]) * np.linalg.norm(v2[:2]))
            observable = rating >= self.settings["rating_min"]
            detail.update(rating=rating, source_timestamp_s=stamp, source_dt_s=dt,
                          fresh_pair=fresh, reset_reason=reset_reason)
            status = "calibrated_relative_projection"
            candidate = None
            try:
                projection.one(state.delta)
            except ValueError as exc:
                raise _Unavailable("current_hinge_projection_unidentifiable", hard=False) from exc
            if protected:
                state.observer.invalidate_window()
                status = "input_protection_using_calibrated_heading"
            elif fresh:
                try:
                    candidate, window = state.observer.observe(stamp, q1, q2, axis, state.delta)
                    detail["heading_window"] = window
                    status = window["status"]
                except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
                    state.observer.invalidate_window()
                    status = "heading_window_fit_rejected"
                    detail["heading_window"] = dict(status=status, reason=str(exc), accepted=False)
            else:
                status = "repeated_pair_using_calibrated_heading"
            if candidate is not None and observable and fresh and not protected:
                detail["target_delta_rad"] = candidate
                innovation = _wrap(candidate - state.delta)
                limit = math.radians(self.settings["max_correction_rate_deg_s"]) * dt
                step = float(np.clip(innovation * -math.expm1(-dt / self.settings["smoothing_tau_s"]), -limit, limit))
                proposed = _wrap(state.delta + step)
                try:
                    (old_angle, _), (new_angle, _) = projection.pair(state.delta, proposed)
                    parameter_step = abs(_wrap(new_angle - old_angle))
                except ValueError:
                    parameter_step = math.inf
                detail["parameter_angle_step_rad"] = parameter_step if math.isfinite(parameter_step) else None
                if parameter_step <= math.radians(self.settings["max_parameter_angle_rate_deg_s"]) * dt + 1e-12:
                    state.delta = proposed
                    detail["heading_updated"] = abs(step) > 1e-12
                else:
                    status = "parameter_innovation_rejected_using_last_heading"
            detail.update(delta_rad=state.delta, status=status)
            try:
                angle, residual = projection.one(state.delta)
            except ValueError as exc:
                raise _Unavailable("current_hinge_projection_unidentifiable", hard=False) from exc
            residual_ok = residual <= self.settings["residual_max_deg"] + 1e-9
            detail["residual_deg"] = residual
            angle, continuity = self._continuous(state, angle, stamp, self.settings["max_gap_s"], reset_reason)
            detail.update(continuity=continuity, continuous_angle_rad=float(angle))
            reference_age = (math.inf if state.observer.evidence_time is None else
                             max(0., stamp - state.observer.evidence_time))
            detail["heading_reference_age_s"] = reference_age if math.isfinite(reference_age) else None
            heading_recent = reference_age <= self.settings["heading_evidence_age_s"]
            # A fit can be accepted long before the bounded applied correction
            # reaches it. Do not mark the intermediate angle as verified.
            converged = (state.observer.target is not None and
                         abs(_wrap(state.observer.target - state.delta)) <= math.radians(.5))
            valid = bool(heading_recent and converged and residual_ok and fresh and not protected
                         and status != "parameter_innovation_rejected_using_last_heading")
            detail["heading_quality"] = ("observed_window" if valid else
                "calibration_relative" if state.observer.evidence_time is None else "held_unobservable")
            if not residual_ok:
                detail["status"] = "high_residual_using_last_heading"
            detail.update(applied=True, display_available=True, valid=valid,
                          quality="valid" if valid else "degraded", continuity=continuity)
            return float(angle), detail
        except (ValueError, KeyError, TypeError, AttributeError, IndexError,
                ImportError, FunctionalDependencyError, np.linalg.LinAlgError) as exc:
            hard = not isinstance(exc, _Unavailable) or exc.hard
            if hard:
                self.reset(slot)
            elif state is not None and state.observer is not None:
                state.observer.invalidate_window()
            detail.update(status="unavailable", reason=str(exc), hard_reset=hard,
                          delta_rad=None if hard or state is None else state.delta)
            return None, detail
