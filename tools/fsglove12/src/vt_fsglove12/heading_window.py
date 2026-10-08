"""Causal evidence for a slowly varying relative heading, using real QMT.

A single projected joint axis cannot distinguish mounting/model error from
yaw drift. Only an excited history whose independent subsets agree may change
the calibrated heading reference. This observer never outputs a finger angle,
resamples sensor data, or claims anatomical/absolute-heading ground truth.
"""
from collections import deque
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .heading_correction import _wrap
from .upstream_functional import _project


WINDOW_DEFAULTS = dict(heading_window_s=4., heading_min_window_s=2.5,
    heading_sample_rate_hz=10., heading_estimate_rate_hz=1.,
    heading_min_flexion_deg=12., heading_min_tilt_deg=15.,
    heading_consistency_deg=2., heading_evidence_age_s=10.,
    heading_model_variation_slack_deg=.1)


class FunctionalHeadingWindow:
    """Use backward-only real sample pairs; QMT operates on quaternion batches.

    ``target`` is an increment relative to the bound installation/neutral
    calibration, not a newly measured absolute heading. A target is offered
    for gradual application only while there is recent local articulation.
    """
    def __init__(self, settings):
        self.settings = settings
        self.history = deque()
        self.last_sample = self.last_fit = self.last_source = None
        self.target = self.evidence_time = None
        self.diagnostic = dict(status="collecting_heading_window", window_samples=0,
                               window_span_s=0., accepted=False)

    def invalidate_window(self):
        # A protected/invalid interval must not help fit a new reference. The
        # estimator owns and retains the previously accepted yaw separately.
        self.history.clear()
        self.last_sample = self.last_fit = self.last_source = None
        self.target = None
        self.diagnostic = dict(status="heading_window_interrupted", window_samples=0,
                               window_span_s=0., accepted=False)

    @staticmethod
    def _span(angles):
        angles = np.unwrap(np.asarray(angles))
        return float(np.rad2deg(np.percentile(angles, 90) - np.percentile(angles, 10)))

    def observe(self, stamp, q1, q2, axis, delta):
        s = self.settings
        if self.last_source is not None and stamp - self.last_source > s["max_gap_s"]:
            self.invalidate_window()
        self.last_source = stamp
        if self.last_sample is None or stamp - self.last_sample >= 1. / s["heading_sample_rate_hz"] - 1e-8:
            self.history.append((stamp, q1.as_quat(), q2.as_quat()))
            self.last_sample = stamp
        while self.history and stamp - self.history[0][0] > s["heading_window_s"] + 1e-8:
            self.history.popleft()
        h = list(self.history)
        span = h[-1][0] - h[0][0] if h else 0.
        self.diagnostic = {**self.diagnostic, "window_samples": len(h),
                           "window_span_s": float(span), "accepted": False}
        # Do not keep chasing an old target after the finger stops moving.
        recent = [row for row in h if stamp - row[0] <= .3 + 1e-8]
        active = False
        if len(recent) >= 2:
            # Reproject ALL recent samples using the SAME reference. Storing
            # each frame's differently corrected angle lets parameter motion
            # masquerade as new articulation and sustains its own feedback.
            first = Rotation.from_quat(np.asarray([row[1] for row in recent]))
            second = Rotation.from_quat(np.asarray([row[2] for row in recent]))
            recent_angles, _ = _project(first.inv() * Rotation.from_rotvec([0., 0., delta]) * second, axis)
            active = self._span(recent_angles) >= .5
        if (len(h) >= 20 and span >= s["heading_min_window_s"] - 1e-8
                and (self.last_fit is None or stamp - self.last_fit >= 1. / s["heading_estimate_rate_hz"] - 1e-8)):
            self.last_fit = stamp
            target, diagnostic = self._fit(h, axis, delta)
            self.diagnostic.update(diagnostic)
            if target is not None:
                self.target, self.evidence_time = target, stamp
            else:
                self.target = None
        self.diagnostic["recent_articulation"] = active
        evidence_age = None if self.evidence_time is None else max(0., stamp - self.evidence_time)
        self.diagnostic["evidence_age_s"] = evidence_age
        # An old batch is not an indefinitely renewable source of corrections.
        live_target = (self.target is not None and active and evidence_age is not None
                       and evidence_age <= 1.5 / s["heading_estimate_rate_hz"])
        return self.target if live_target else None, dict(self.diagnostic)

    def _fit(self, history, axis, delta):
        from qmt.functions.heading_correction import estimateDelta1d

        s = self.settings
        q1 = Rotation.from_quat(np.asarray([h[1] for h in history]))
        q2 = Rotation.from_quat(np.asarray([h[2] for h in history]))
        v1, v2 = q1.apply(axis), q2.apply(axis)
        weights = np.linalg.norm(v1[:, :2], axis=1) * np.linalg.norm(v2[:, :2], axis=1)
        keep = weights >= s["rating_min"]
        q1, q2, v1, v2 = q1[keep], q2[keep], v1[keep], v2[keep]
        d = dict(status="insufficient_heading_geometry", accepted=False,
                 observable_samples=int(np.count_nonzero(keep)))
        if len(q1) < 16:
            return None, d
        old_relative = q1.inv() * Rotation.from_rotvec([0., 0., delta]) * q2
        angles, old_residual = _project(old_relative, axis)
        flex_span = self._span(angles)
        d["flexion_span_deg"] = flex_span
        if flex_span < s["heading_min_flexion_deg"]:
            d["status"] = "insufficient_local_articulation"
            return None, d
        # Pure world-yaw rotation provides no new inclination evidence.
        gravity = q1.inv().apply([0., 0., 1.])
        distances = np.arccos(np.clip(gravity @ gravity.T, -1., 1.))
        end1, end2 = np.unravel_index(np.argmax(distances), distances.shape)
        tilt_span = float(np.rad2deg(distances[end1, end2]))
        d["parent_tilt_span_deg"] = tilt_span
        if tilt_span < s["heading_min_tilt_deg"]:
            d["status"] = "insufficient_parent_tilt_diversity"
            return None, d
        n = len(q1)
        halves = (np.arange(n // 2), np.arange(n // 2, n))
        if any(self._span(angles[group]) < s["heading_min_flexion_deg"] / 2 for group in halves):
            d["status"] = "insufficient_split_window_articulation"
            return None, d
        # These are the analytical zeroes of QMT's 1d_corr residual, used only
        # to test consistency; the actual fit still calls upstream QMT below.
        point_targets = np.arctan2(v1[:, 1], v1[:, 0]) - np.arctan2(v2[:, 1], v2[:, 0])
        offsets = (point_targets - delta + np.pi) % (2 * np.pi) - np.pi
        spread = float(np.rad2deg(np.percentile(np.abs(offsets - np.median(offsets)), 90)))
        d["candidate_spread_deg"] = spread
        if spread > s["heading_consistency_deg"]:
            d["status"] = "heading_model_inconsistent"
            return None, d
        order = np.argsort(gravity @ (gravity[end1] - gravity[end2]))
        tilt_groups = (order[:n // 2], order[n // 2:])
        qa, qb = q1.as_quat()[:, [3, 0, 1, 2]], q2.as_quat()[:, [3, 0, 1, 2]]

        def fit(group):
            value, rating, cost = estimateDelta1d(qa[group], qb[group], axis, delta, "1d_corr", 5)
            if not all(math.isfinite(float(v)) for v in (value, rating, cost)):
                raise ValueError("nonfinite QMT heading window fit")
            return _wrap(float(value))

        candidate = fit(np.arange(n))
        split = [fit(group) for group in halves]
        tilts = [fit(group) for group in tilt_groups]
        split_difference = abs(math.degrees(_wrap(split[0] - split[1])))
        tilt_difference = abs(math.degrees(_wrap(tilts[0] - tilts[1])))
        d.update(split_difference_deg=split_difference, tilt_difference_deg=tilt_difference)
        if max(split_difference, tilt_difference) > s["heading_consistency_deg"]:
            d["status"] = "heading_subwindows_disagree"
            return None, d
        relative = q1.inv() * Rotation.from_rotvec([0., 0., candidate]) * q2
        new_angles, residual = _project(relative, axis)
        # A constant non-hinge component is model/mounting mismatch, not yaw
        # evidence. A candidate must not turn it into a palm-orientation-
        # dependent residual merely because forcing its mean towards zero
        # decreases the ideal-hinge fitting cost. Use QMT's projected angles
        # to compare dispersion of the remaining rotations, not raw error
        # magnitude. No new pose solver or anatomical-angle prior is fitted.
        old_remaining = Rotation.from_rotvec(-angles[:, None] * axis) * old_relative
        new_remaining = Rotation.from_rotvec(-new_angles[:, None] * axis) * relative

        def variation(rotations):
            return float(np.rad2deg(np.sqrt(np.mean((rotations.mean().inv() * rotations).magnitude() ** 2))))

        old_variation, new_variation = variation(old_remaining), variation(new_remaining)
        d.update(model_variation_before_deg=old_variation, model_variation_after_deg=new_variation)
        if new_variation > old_variation + s["heading_model_variation_slack_deg"]:
            d["status"] = "heading_increases_model_variation"
            return None, d
        if np.percentile(residual, 90) > s["residual_max_deg"]:
            d["status"] = "heading_window_residual_high"
            return None, d
        if any(np.sqrt(np.mean(residual[group] ** 2)) >
               np.sqrt(np.mean(old_residual[group] ** 2)) + .05 for group in halves):
            d["status"] = "heading_window_does_not_improve_both_halves"
            return None, d
        d.update(status="accepted_heading_window", accepted=True,
                 candidate_delta_rad=candidate, residual_p90_deg=float(np.percentile(residual, 90)))
        return candidate, d
