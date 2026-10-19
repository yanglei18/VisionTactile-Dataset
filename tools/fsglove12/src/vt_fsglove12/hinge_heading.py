"""QMT heading-invariant little-finger hinge projection.

Six-axis filters have independent, drifting yaw references. Projecting their
uncorrected relative quaternion onto a hinge mixes that yaw error into flexion
when the hand turns. Use QMT's existing 1d_corr estimator BEFORE projection,
in the FSGlove installation and functional-neutral coordinates. This changes
only the inferred little PIP, not the input, VQF, calibration, palm or MCP.

There is no window or motion low-pass filter. At a vertical hinge, yaw is not
observable: retain the last identifiable yaw correction but still project the
CURRENT quaternions. Never retain a previous finger angle as a measurement.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .config import digest
from .heading_correction import CausalHeadingCorrector, _finite, _rotation, _wrap
from .upstream_functional import FunctionalDependencyError, KIND, _project, _same_binding


class LittleFingerHeading:
    """Reuse the saved little-finger axis; no new calibration or unit changes."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._context = None
        self._geometry_cache = None
        self._last_time = None
        self._identity = None
        self._delta = None

    def apply(self, frame, profile, calibrator, binding):
        detail = dict(method="qmt_estimateDelta1d_current_pair_1d_corr_v1",
                      slot=8, applied=False, status="passthrough",
                      absolute_heading_observed=False,
                      anatomical_accuracy_verified=False,
                      raw_values_modified=False, calibration_modified=False,
                      motion_smoothed=False)
        try:
            entry = profile.get("functional_fingers", {}).get("little", {})
            reference = entry.get("slot_refs", {}).get("8", {})
            if (profile.get("functional_enabled", True) is False
                    or entry.get("kind") != KIND or entry.get("finger") != "little"
                    or reference.get("mode") != "hinge_projection"
                    or not _same_binding(entry.get("binding", {}), binding, 8)):
                raise ValueError("little_functional_binding_unavailable")
            dependencies = binding["dependency_nodes_by_slot"]["8"]
            if reference.get("dependency_nodes") != dependencies:
                raise ValueError("little_dependency_mismatch")
            if (frame.get("session_id") != binding["session_id"]
                    or profile.get("session_id") != binding["session_id"]):
                raise ValueError("session_mismatch")
            now = frame.get("timestamp_s")
            if not _finite(now) or now < 0:
                raise ValueError("invalid_frame_time")
            context = digest(dict(binding=binding, reference=reference,
                                  profile_version=profile.get("profile_version")))
            if (context != self._context or self._last_time is not None
                    and (now < self._last_time or now - self._last_time > .25)):
                self.reset()
                self._context = context
            first, second = (frame.get("nodes", {}).get(n) for n in dependencies)
            protected = False
            for node, data in zip(dependencies, (first, second)):
                if not isinstance(data, dict):
                    raise ValueError("missing_parent_or_child")
                protection = data.get("input_filter", {})
                if data.get("orientation_valid") is False:
                    raise ValueError("input_orientation_invalid")
                protected |= bool(protection.get("gyro_rejected") or protection.get("accel_rejected"))
                if data.get("epoch") != binding["nodes_epochs"][node]:
                    raise ValueError("filter_epoch_changed")
                if not _finite(data.get("age_s")) or not 0 <= data["age_s"] <= .15:
                    raise ValueError("stale_parent_or_child")
                if type(data.get("sample_index")) is not int or data["sample_index"] <= 0:
                    raise ValueError("invalid_sample_index")
            if abs(first["age_s"] - second["age_s"]) > .03:
                raise ValueError("parent_child_source_time_skew")
            identity = (first["sample_index"], second["sample_index"])
            if self._identity is not None and any(a < b for a, b in zip(identity, self._identity)):
                raise ValueError("sample_index_reversed")
            fresh = self._identity is None or all(a != b for a, b in zip(identity, self._identity))
            if self._geometry_cache is None:
                self._geometry_cache = CausalHeadingCorrector._geometry(calibrator, 7, 8, reference)
            hp, hc, rp, rc, axis = self._geometry_cache
            q1 = hp * _rotation(first["quat_xyzw"]) * rp
            q2 = hc * _rotation(second["quat_xyzw"]) * rc
            v1, v2 = q1.apply(axis), q2.apply(axis)
            rating = float(np.linalg.norm(v1[:2]) * np.linalg.norm(v2[:2]))
            detail["rating"] = rating
            self._last_time = now
            candidate = self._delta
            observable = rating >= .2
            if observable and fresh and not protected:
                from qmt.functions.heading_correction import estimateDelta1d
                delta, fitted_rating, cost = estimateDelta1d(
                    q1.as_quat()[None, [3, 0, 1, 2]],
                    q2.as_quat()[None, [3, 0, 1, 2]],
                    axis, 0. if candidate is None else candidate, "1d_corr", 5)
                if not all(np.isfinite(x) for x in (delta, fitted_rating, cost)):
                    raise ValueError("nonfinite_qmt_estimate")
                candidate = _wrap(float(delta))
                detail["fit_cost"] = float(cost)
                self._identity = identity
            if candidate is None:
                detail["reason"] = ("input_protection_without_reference" if protected else
                                    "heading_unobservable_without_reference")
                return None, detail
            corrected = q1.inv() * Rotation.from_rotvec([0., 0., candidate]) * q2
            angle, residual = _project(corrected, axis)
            detail.update(delta_rad=candidate, residual_deg=float(residual))
            # QMT's 1d_corr cost alone only tests horizontal axis alignment.
            # Check the full corrected rotation too, including axis inclination.
            if float(residual) > 15.:
                raise ValueError("corrected_hinge_residual_too_large")
            self._delta = candidate
            detail.update(applied=True, status=("input_protection_using_last_heading" if protected else
                "corrected" if observable else "unobservable_using_last_heading"))
            return float(angle), detail
        except (ValueError, KeyError, TypeError, AttributeError, IndexError,
                ImportError, FunctionalDependencyError, np.linalg.LinAlgError) as exc:
            self.reset()
            detail.update(applied=False, status="passthrough", reason=str(exc))
            return None, detail
