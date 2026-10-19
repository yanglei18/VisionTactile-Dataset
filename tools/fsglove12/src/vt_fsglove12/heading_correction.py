"""Causal QMT relative-heading adapter for five observed finger hinges only.

No VQF state, raw sample, installation profile, MCP or CMC is changed. Only a
copy of the measured child's quaternion used for pose inference is corrected.
The parent remains the heading reference, not an absolute-heading observation.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import hashlib
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .upstream_functional import KIND, _same_binding


HINGES = {"index": 2, "middle": 5, "little": 8, "ring": 11, "thumb": 15}
DEFAULTS = dict(enabled=False, window_s=3., min_window_s=2., sample_rate_hz=10.,
                estimate_rate_hz=2., rating_min=.2, smoothing_tau_s=.75,
                max_correction_rate_deg_s=5., max_pair_skew_s=.03, max_age_s=.15)
METHOD = "qmt_estimateDelta1d_1d_corr_causal_relative_heading_v1"
_VERTICAL = np.array([0., 0., 1.])
# Official Raw2MANO uses its +Y as vertical; QMT assumes world +Z.
_WORLD = Rotation.from_euler("x", np.pi / 2)


def _finite(value):
    try:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        return False


def _wrap(value):
    return float((value + np.pi) % (2 * np.pi) - np.pi)


def _rotation(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1., atol=1e-5):
        raise ValueError("invalid unit quaternion")
    return Rotation.from_quat(q)


class CausalHeadingCorrector:
    """Config is this adapter's options dict; omitted/disabled is passthrough.

    Windows contain actual fresh paired observations, sampled at *at most*
    10 Hz by default, not synthetic/resampled device data. QMT's 1D estimator
    does not require an equidistant time array. Heading updates are causal at
    most 2 Hz. Only the slowly varying correction is smoothed and rate limited;
    the current hand motion quaternion is never temporally filtered here.
    """
    def __init__(self, config=None):
        if config is None:
            config = {}
        if not isinstance(config, dict) or set(config) - set(DEFAULTS):
            raise ValueError("heading correction requires supported option fields")
        self.settings = {**DEFAULTS, **config}
        if type(self.settings["enabled"]) is not bool:
            raise ValueError("heading enabled must be boolean")
        for key, value in self.settings.items():
            if key != "enabled" and (not _finite(value) or value <= 0):
                raise ValueError(f"heading {key} must be finite and positive")
        if not (2 <= self.settings["window_s"] <= 4 and
                0 < self.settings["min_window_s"] <= self.settings["window_s"] and
                self.settings["sample_rate_hz"] <= 30 and
                self.settings["estimate_rate_hz"] <= self.settings["sample_rate_hz"] and
                self.settings["rating_min"] <= 1):
            raise ValueError("invalid heading window/rate/rating options")
        self._states = {}
        self._context = None
        self._last_time = None

    def reset(self):
        self._states.clear()
        self._context = self._last_time = None

    @staticmethod
    def _geometry(calibrator, parent, child, reference):
        """Map different sensor axes into the same virtual hinge coordinates.

        F_i = K_i Q_i L_i^-1, K_i = G_i B_i^-1 is upstream's global segment
        orientation. A_child = (F_parent D_child)^-1 F_child. The functional
        fit supplies A0 and j for A0^-1 A_child = Rot(j, theta).
        Thus q1 = W F_parent D_child A0, q2 = W F_child share the SAME hinge
        axis j, as required by QMT estimateDelta1d (which uses j for both).
        """
        base = Rotation.from_quat(np.asarray(calibrator._base_quat, dtype=float))
        global_error = Rotation.from_rotvec(np.asarray(calibrator._global_installation_error, dtype=float))
        local_error = Rotation.from_rotvec(np.asarray(calibrator._local_installation_error, dtype=float))
        rest = Rotation.from_rotvec(np.asarray(calibrator._delta_rot_fa_rotvec, dtype=float))
        if int(calibrator._joint_fa[child]) != parent:
            raise ValueError("incompatible upstream parent hierarchy")
        hp = _WORLD * global_error[parent] * base[parent].inv()
        hc = _WORLD * global_error[child] * base[child].inv()
        if any(np.linalg.norm(h.apply(_VERTICAL) - _VERTICAL) > 1e-4 for h in (hp, hc)):
            raise ValueError("installation frame does not preserve world vertical")
        a0 = _rotation(reference["neutral_local_xyzw"])
        axis = np.asarray(reference["axis_local"], dtype=float)
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
            raise ValueError("invalid functional hinge axis")
        axis /= np.linalg.norm(axis)
        rp = local_error[parent].inv() * rest[child] * a0
        rc = local_error[child].inv()
        return hp, hc, rp, rc, axis

    def apply(self, frame, profile, calibrator, binding):
        output = deepcopy(frame)
        diagnostic = dict(enabled=self.settings["enabled"], method=METHOD,
            implementation="qmt.functions.heading_correction.estimateDelta1d", constraint="1d_corr",
            applied_slots=[], pairs={}, raw_values_modified=False, input_frame_modified=False,
            filter_state_modified=False, installation_profile_modified=False,
            absolute_heading_observed=False, anatomical_accuracy_verified=False,
            uses_future_samples=False, correction_only_smoothing=True,
            scope="five_measured_PIP_or_thumb_IP_child_segments_not_MCP_CMC",
            settings=dict(self.settings))
        if not isinstance(profile, dict) or not isinstance(binding, dict):
            self.reset()
            diagnostic["reason"] = "missing_profile_or_binding"
            return output, diagnostic
        if not self.settings["enabled"] or profile.get("functional_enabled", True) is False:
            self.reset()
            diagnostic["reason"] = "disabled" if not self.settings["enabled"] else "functional_profile_disabled"
            return output, diagnostic
        now = frame.get("timestamp_s")
        if not _finite(now) or now < 0:
            self.reset()
            diagnostic["reason"] = "invalid_frame_time"
            return output, diagnostic
        if frame.get("session_id") != binding.get("session_id") or profile.get("session_id") != binding.get("session_id"):
            self.reset()
            diagnostic["reason"] = "session_mismatch"
            return output, diagnostic
        entries = profile.get("functional_fingers", {})
        if not isinstance(entries, dict):
            self.reset()
            diagnostic["reason"] = "invalid_functional_entries"
            return output, diagnostic
        try:
            context = hashlib.sha256(json.dumps(dict(binding=binding, entries=entries,
                profile_version=profile.get("profile_version")),
                sort_keys=True, allow_nan=False, separators=(",", ":")).encode()).hexdigest()
        except (ValueError, TypeError):
            self.reset()
            diagnostic["reason"] = "invalid_profile_binding"
            return output, diagnostic
        if context != self._context:
            self._states.clear()
            diagnostic["reset_reason"] = "profile_or_binding_changed"
        elif self._last_time is not None and (now < self._last_time or
                now - self._last_time > max(.5, 2 * self.settings["max_age_s"])):
            self._states.clear()
            diagnostic["reset_reason"] = "clock_reversed_or_observation_gap"
        self._context, self._last_time = context, now
        for finger, child in HINGES.items():
            key = str(child)
            entry = entries.get(finger, {})
            if not isinstance(entry, dict):
                entry = {}
            reference = entry.get("slot_refs", {}).get(key)
            detail = dict(status="missing_functional_hinge", applied=False, delta_rad=0., target_delta_rad=0.,
                          rating=None, current_rating=None, window_samples=0, window_span_s=0.)
            diagnostic["pairs"][key] = detail
            if reference is None:
                self._states.pop(key, None)
                continue
            try:
                dependencies = binding.get("dependency_nodes_by_slot", {}).get(key)
                if (entry.get("kind") != KIND or entry.get("finger") != finger or
                        reference.get("mode") != "hinge_projection" or
                        not _same_binding(entry.get("binding", {}), binding, child) or
                        reference.get("dependency_nodes") != dependencies):
                    raise ValueError("functional_binding_mismatch")
                parent_node, child_node = dependencies
                detail.update(parent_node=parent_node, child_node=child_node)
                first, second = (frame.get("nodes", {}).get(n) for n in dependencies)
                if not isinstance(first, dict) or not isinstance(second, dict):
                    raise ValueError("missing_parent_or_child")
                for node, data in zip(dependencies, (first, second)):
                    protection = data.get("input_filter", {})
                    if (data.get("orientation_valid") is False or protection.get("gyro_rejected")
                            or protection.get("accel_rejected")):
                        raise ValueError("input_protection_active")
                    age, index = data.get("age_s"), data.get("sample_index")
                    if data.get("epoch") != binding["nodes_epochs"][node]:
                        raise ValueError("filter_epoch_changed")
                    if not _finite(age) or not 0 <= age <= self.settings["max_age_s"]:
                        raise ValueError("stale_parent_or_child")
                    if type(index) is not int or index <= 0:
                        raise ValueError("invalid_sample_index")
                if abs(first["age_s"] - second["age_s"]) > self.settings["max_pair_skew_s"]:
                    raise ValueError("parent_child_source_time_skew")
                qp, qc = _rotation(first["quat_xyzw"]), _rotation(second["quat_xyzw"])
                identity = (first["sample_index"], second["sample_index"])
                stamp = now - max(first["age_s"], second["age_s"])
                state = self._states.get(key)
                if state is None:
                    hp, hc, rp, rc, axis = self._geometry(calibrator, child - 1, child, reference)
                    state = dict(geometry=(hp, hc, rp, rc, axis), history=deque(), last_sample=None,
                                 last_estimate=None, last_apply=now, identity=None, delta=0., target=0., rating=None)
                    self._states[key] = state
                hp, hc, rp, rc, axis = state["geometry"]
                q1, q2 = hp * qp * rp, hc * qc * rc
                v1, v2 = q1.apply(axis), q2.apply(axis)
                current_rating = float(np.linalg.norm(v1[:2]) * np.linalg.norm(v2[:2]))
                detail["current_rating"] = current_rating
                fresh = state["identity"] is None or all(a != b for a, b in zip(identity, state["identity"]))
                if state["identity"] is not None and any(a < b for a, b in zip(identity, state["identity"])):
                    raise ValueError("sample_index_reversed")
                if fresh and (state["last_sample"] is None or
                              stamp - state["last_sample"] >= 1. / self.settings["sample_rate_hz"] - 1e-9):
                    state["history"].append((stamp, q1.as_quat()[[3, 0, 1, 2]], q2.as_quat()[[3, 0, 1, 2]]))
                    state["last_sample"] = stamp
                if fresh:
                    state["identity"] = identity
                history = state["history"]
                while history and stamp - history[0][0] > self.settings["window_s"] + 1e-9:
                    history.popleft()
                span = history[-1][0] - history[0][0] if history else 0.
                detail.update(window_samples=len(history), window_span_s=float(span))
                observable = current_rating >= self.settings["rating_min"]
                ready = len(history) >= 3 and span >= self.settings["min_window_s"] - 1e-9
                if fresh and ready and observable and (state["last_estimate"] is None or
                        now - state["last_estimate"] >= 1. / self.settings["estimate_rate_hz"] - 1e-9):
                    from qmt.functions.heading_correction import estimateDelta1d
                    delta, rating, cost = estimateDelta1d(
                        np.asarray([h[1] for h in history]), np.asarray([h[2] for h in history]),
                        axis, state["target"], "1d_corr", 5)
                    state["last_estimate"] = now
                    if not all(np.isfinite(v) for v in (delta, rating, cost)):
                        raise ValueError("nonfinite_qmt_estimate")
                    state["rating"] = float(rating)
                    detail["estimate_updated"] = False
                    if rating >= self.settings["rating_min"]:
                        state["target"] = _wrap(float(delta))
                        detail["estimate_updated"] = True
                dt = max(0., now - state["last_apply"])
                state["last_apply"] = now
                if fresh and observable and state["rating"] is not None and state["rating"] >= self.settings["rating_min"]:
                    step = _wrap(state["target"] - state["delta"]) * (-math.expm1(-dt / self.settings["smoothing_tau_s"]))
                    limit = math.radians(self.settings["max_correction_rate_deg_s"]) * dt
                    state["delta"] = _wrap(state["delta"] + float(np.clip(step, -limit, limit)))
                detail.update(delta_rad=state["delta"], target_delta_rad=state["target"], rating=state["rating"])
                if not observable:
                    detail["status"] = "currently_unobservable_passthrough"
                elif not ready or state["rating"] is None:
                    detail["status"] = "collecting_causal_window"
                elif state["rating"] < self.settings["rating_min"]:
                    detail["status"] = "window_rating_low_passthrough"
                else:
                    detail["status"] = "tracking_relative_heading"
                    if abs(state["delta"]) > 1e-12:
                        # q2_corr = Z(delta) q2 => Q_child_corr = H_child^-1 Z H_child Q_child.
                        # Do NOT mutate QMT input history with already corrected quaternions.
                        correction = hc.inv() * Rotation.from_rotvec(_VERTICAL * state["delta"]) * hc
                        output["nodes"][child_node]["quat_xyzw"] = (correction * qc).as_quat().tolist()
                        diagnostic["applied_slots"].append(child)
                        detail["applied"] = True
            except (ValueError, TypeError, KeyError, IndexError, AttributeError, ImportError, np.linalg.LinAlgError) as exc:
                self._states.pop(key, None)
                detail.update(status="passthrough", reason=str(exc), applied=False)
        diagnostic["applied_slots"].sort()
        return output, diagnostic
