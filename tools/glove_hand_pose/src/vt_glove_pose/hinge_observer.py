"""Experimental offline angle/bias observer for a calibrated 1-DoF hinge.

This does not replace ThumbFlexionEstimator or install a live calibration.
Functional x is opposite the consistently signed hinge axis. Angle prediction
uses pg.x - dg.x minus the estimated *relative* residual gyro bias. Gravity and
transverse angular velocity can independently observe relative rotation modulo
360 degrees. Neither observes an anatomical straight pose or global heading.

All noise values and thresholds are experimental model parameters, not measured
sensor specifications or product accuracy bounds. In particular angle_std_deg
is a filter-model uncertainty, NOT an empirically calibrated confidence bound.
Correct units, paired timing, rigid mounting and hinge applicability are caller
requirements. Consistent sustained linear accelerations can be indistinguishable
from gravity using these measurements alone; the rejection checks are not a
proof of gravity-only motion. Noise between samples/sources can be correlated.
"""

from __future__ import annotations

import math
from numbers import Real

import numpy as np


# Experimental hypotheses, deliberately separate from hardware acceptance.
_GYRO_SAMPLE_STD_DPS = .8
_BIAS_RANDOM_WALK_DPS_SQRT_S = .05
_GRAVITY_ANGLE_STD_DEG = 1.5
_TRANSVERSE_COMPONENT_STD_DPS = .8
_MIN_TRANSVERSE_DPS = 12.
_CONFIRM_OBSERVATION_S = .15
_SHORT_OBSERVATION_GAP_S = .10
_LARGE_RECOVERY_GAP_S = .5
_UNKNOWN_MOTION_VARIANCE_PER_SAMPLE = 25.


def _number(value, name, low, high):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f"{name} must be finite in [{low}, {high}]")
    return float(value)


def _boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f"{name} must be boolean")
    return value


def _vector(value, name, bound):
    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "ifu" or raw.shape != (3,):
            raise ValueError
        result = raw.astype(float)
        if not np.all(np.isfinite(result)) or np.any(np.abs(result) > bound):
            raise ValueError
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain three finite real values within +/-{bound}") from exc
    return result


def _wrap(value):
    return (float(value) + 180.) % 360. - 180.


def _gravity(pa, da):
    pn, dn = float(np.linalg.norm(pa)), float(np.linalg.norm(da))
    if not .85 <= pn <= 1.15 or not .85 <= dn <= 1.15 or abs(pn - dn) > .10:
        return None, ["acc_norm"]
    p, d = pa / pn, da / dn
    if abs(float(p[0] - d[0])) > .12:
        return None, ["axis_gravity_mismatch"]
    pp, dp = float(np.linalg.norm(p[1:])), float(np.linalg.norm(d[1:]))
    if min(pp, dp) < .30:
        return None, ["low_gravity_projection"]
    beta = math.degrees(math.atan2(p[1] * d[2] - p[2] * d[1], p[1:] @ d[1:]))
    # Near-axis gravity has poorer angular sensitivity even above the gate.
    sigma = _GRAVITY_ANGLE_STD_DEG / min(pp, dp)
    return (beta, sigma), []


def _transverse(pg, dg):
    p, d = pg[1:], dg[1:]
    pn, dn = float(np.linalg.norm(p)), float(np.linalg.norm(d))
    if min(pn, dn) < _MIN_TRANSVERSE_DPS:
        return None, ["transverse_gyro_too_small"]
    if abs(pn - dn) > 3. + .12 * max(pn, dn):
        return None, ["transverse_gyro_magnitude_mismatch"]
    beta = math.degrees(math.atan2(p[0] * d[1] - p[1] * d[0], p @ d))
    sigma = max(.7, math.degrees(_TRANSVERSE_COMPONENT_STD_DPS * math.sqrt(1. / pn**2 + 1. / dn**2)))
    return (beta, sigma), []


class ConstrainedHingeObserver:
    """Causal angle/bias Kalman candidate with conditional hinge observations.

    ``reset`` and ``update`` have the same input convention as the existing
    ThumbFlexionEstimator. ``rate_dps`` in snapshots is bias-corrected relative
    angular rate; ``raw_relative_rate_dps`` preserves pg.x-dg.x. The correction
    rate is the numerical measurement update / dt, not physical angular speed.
    ``current_angle_observed`` describes this sample, separately from latched
    ``history_loss``. Rejecting acceleration does not itself lose gyro history.
    ``observation_rejection_count`` counts samples with any rejected observation;
    ``rejection_counts`` counts individual reasons and may sum to a larger value.

    An observation gap needs coherent confirmation before reuse, except for a
    short dropout whose established anchor and uninterrupted gyro history still
    explain the returning observation within the unchanged motion bound.
    After a real gap, large corrections may re-establish the *currently observed*
    angle quickly, with history_loss latched and bias learning disabled for that
    reacquisition update. A continuous observation cannot jump just because its
    new value persists. Recovery cannot reconstruct a missing trajectory or
    resolve whole rotations lost during a gap. There is no automatic straight
    reset. No files, hardware, VQF, GUI or live parameters are accessed.
    """

    def __init__(self, sample_rate_hz):
        self.sample_rate_hz = _number(sample_rate_hz, "sample_rate_hz", 10., 2000.)
        self._initialized = False
        self._x = np.zeros(2)  # angle degrees, relative gyro bias degrees/s
        self._p = np.diag([1., 9.])
        self._neutral = None
        self._phase = 0.
        self._time = 0.
        self._samples = 0
        self._history_loss = False
        self._history_epoch = 0
        self._history_loss_causes = []
        self._rejection_counts = {}
        self._rejected_observation_samples = 0
        self._last_observation_time = 0.
        self._sources = {}
        self._details = dict(rate_dps=0., raw_relative_rate_dps=0., acc_angle_deg=None,
                             innovation_deg=None, correction_dps=0., acc_observable=False,
                             correction_active=False, quality="uninitialized",
                             reasons=["neutral_required"], observation_sources=[],
                             observation_rejection_active=False)

    @staticmethod
    def _source_state(*, beta=None, sigma=None, anchor_epoch=None):
        return dict(beta=beta, sigma=sigma, phase=0., gap=beta is None, gap_start=0., gap_end=None,
                    anchor_epoch=anchor_epoch,
                    pending_beta=None, pending_phase=None, pending_start=None, pending_sigma=None)

    def reset(self, proximal_acc, distal_acc, *, current_proximal_acc=None, current_distal_acc=None):
        """Use an explicit caller-verified static reference, possibly bent.

        Geometry and norm checks cannot prove static or acceleration-free data.
        Optional current vectors initialize the presently observed angle without
        redefining the reference. Validation is transactional.
        """
        pa = _vector(proximal_acc, "proximal_acc", 100.)
        da = _vector(distal_acc, "distal_acc", 100.)
        neutral, errors = _gravity(pa, da)
        if errors:
            raise ValueError("neutral gravity is not observable/consistent: " + ", ".join(errors))
        if (current_proximal_acc is None) != (current_distal_acc is None):
            raise ValueError("both current acceleration vectors are required")
        current = neutral
        if current_proximal_acc is not None:
            cp = _vector(current_proximal_acc, "current_proximal_acc", 100.)
            cd = _vector(current_distal_acc, "current_distal_acc", 100.)
            current, errors = _gravity(cp, cd)
            if errors:
                raise ValueError("current gravity is not observable/consistent: " + ", ".join(errors))
        fresh = type(self)(self.sample_rate_hz)
        fresh._initialized = True
        fresh._neutral = float(neutral[0])
        fresh._x[0] = _wrap(current[0] - neutral[0])
        fresh._p[0, 0] = float(current[1] ** 2)
        fresh._sources = {"gravity": self._source_state(beta=current[0], sigma=current[1],
                                                      anchor_epoch=fresh._history_epoch),
                          "transverse_gyro": self._source_state()}
        fresh._details.update(acc_angle_deg=float(fresh._x[0]), innovation_deg=0.,
                              acc_observable=True, quality="tracking", reasons=[],
                              observation_sources=["gravity"])
        self.__dict__.update(fresh.__dict__)
        return self.snapshot()

    def _mark_gap(self, name):
        s = self._sources[name]
        if not s["gap"] or s["gap_end"] is not None:
            s["gap_start"] = self._time
        s.update(gap=True, gap_end=None, pending_beta=None, pending_phase=None,
                 pending_start=None, pending_sigma=None)

    def _mark_history_loss(self, cause):
        self._history_loss = True
        if cause in ("missing_pair", "gyro_saturation"):
            self._history_epoch += 1
        if cause not in self._history_loss_causes:
            self._history_loss_causes.append(cause)

    def _qualify(self, name, observation):
        """Check temporal evidence without replacing an accepted anchor."""
        beta, sigma = observation
        s = self._sources[name]
        if s["gap"]:
            if s["gap_end"] is None:
                # End the actual unavailability interval on the FIRST returning
                # observation, even if it is rejected. Waiting on an incorrect
                # value cannot manufacture a longer missing-observation gap.
                s["gap_end"] = self._time
            # A short observation-only dropout is not a missing trajectory.
            # Compare with the last ACCEPTED anchor, not a just-seen pending
            # value; a latched earlier loss does not invalidate a newer anchor.
            # Keep this anchor constraint during repeated inconsistent returns.
            # Another genuine long dropout or a new history epoch can require
            # confirmation again, but mere persistence cannot bless a jump.
            if (s["beta"] is not None and s["anchor_epoch"] == self._history_epoch
                    and s["gap_end"] - s["gap_start"] <= _SHORT_OBSERVATION_GAP_S + 1e-12):
                motion_error = _wrap(beta - s["beta"] - (self._phase - s["phase"]))
                bound = max(3., 3. * math.hypot(sigma, s["sigma"]))
                if abs(motion_error) <= bound:
                    return True, "short_observation_gap_recovered"
                return False, "continuous_observation_jump"
            if s["pending_beta"] is None:
                s.update(pending_beta=beta, pending_phase=self._phase,
                         pending_start=self._time, pending_sigma=sigma)
                return False, "observation_confirming"
            motion_error = _wrap(beta - s["pending_beta"] - (self._phase - s["pending_phase"]))
            bound = max(3., 3. * math.hypot(sigma, s["pending_sigma"]))
            if abs(motion_error) > bound:
                s.update(pending_beta=beta, pending_phase=self._phase,
                         pending_start=self._time, pending_sigma=sigma)
                return False, "observation_motion_inconsistent"
            if self._time - s["pending_start"] + 1e-12 < _CONFIRM_OBSERVATION_S:
                return False, "observation_confirming"
            return True, None
        motion_error = _wrap(beta - s["beta"] - (self._phase - s["phase"]))
        bound = max(3., 3. * math.hypot(sigma, s["sigma"]))
        if abs(motion_error) > bound:
            return False, "continuous_observation_jump"
        return True, None

    def _accept_anchor(self, name, observation):
        beta, sigma = observation
        self._sources[name].update(beta=beta, sigma=sigma, phase=self._phase, gap=False,
                                   anchor_epoch=self._history_epoch, gap_end=None,
                                   pending_beta=None, pending_phase=None, pending_start=None,
                                   pending_sigma=None)

    def update(self, proximal_gyro_dps, proximal_acc_g, distal_gyro_dps, distal_acc_g,
               *, dt_s=None, proximal_saturated=False, distal_saturated=False, pair_valid=True):
        nominal = 1. / self.sample_rate_hz
        dt = nominal if dt_s is None else _number(dt_s, "dt_s", .25 * nominal, 4. * nominal)
        proximal_saturated = _boolean(proximal_saturated, "proximal_saturated")
        distal_saturated = _boolean(distal_saturated, "distal_saturated")
        pair_valid = _boolean(pair_valid, "pair_valid")
        if not self._initialized:
            raise ValueError("reset with an explicit observable neutral pose before update")
        if pair_valid:
            pg = _vector(proximal_gyro_dps, "proximal_gyro_dps", 10000.)
            dg = _vector(distal_gyro_dps, "distal_gyro_dps", 10000.)
            pa = _vector(proximal_acc_g, "proximal_acc_g", 100.)
            da = _vector(distal_acc_g, "distal_acc_g", 100.)
        # No mutations occur above this line if an input is invalid.
        self._samples += 1
        self._time += dt
        self._details.update(rate_dps=0., raw_relative_rate_dps=0., acc_angle_deg=None,
                             innovation_deg=None, correction_dps=0., acc_observable=False,
                             correction_active=False, reasons=[], observation_sources=[])
        reasons = self._details["reasons"]
        q_bias = _BIAS_RANDOM_WALK_DPS_SQRT_S ** 2
        process = np.array([[_GYRO_SAMPLE_STD_DPS**2 * dt**2 + q_bias * dt**3 / 3., -q_bias * dt**2 / 2.],
                            [-q_bias * dt**2 / 2., q_bias * dt]])
        if not pair_valid:
            self._p += process + np.diag([_UNKNOWN_MOTION_VARIANCE_PER_SAMPLE, 0.])
            self._mark_history_loss("missing_pair")
            reasons.append("pair_unavailable")
            for name in self._sources:
                self._mark_gap(name)
            return self._finish()
        raw_rate = float(pg[0] - dg[0])
        predicted_rate = raw_rate - float(self._x[1])
        self._x[0] += predicted_rate * dt
        self._phase += predicted_rate * dt
        transition = np.array([[1., -dt], [0., 1.]])
        self._p = transition @ self._p @ transition.T + process
        self._details.update(raw_relative_rate_dps=raw_rate, rate_dps=predicted_rate)
        if proximal_saturated or distal_saturated:
            self._p[0, 0] += _UNKNOWN_MOTION_VARIANCE_PER_SAMPLE
            self._mark_history_loss("gyro_saturation")
            reasons.append("saturated_input")
            for name in self._sources:
                self._mark_gap(name)
            return self._finish()
        gravity, gravity_errors = _gravity(pa, da)
        transverse, gyro_errors = _transverse(pg, dg)
        observations = {"gravity": gravity, "transverse_gyro": transverse}
        reasons.extend(gravity_errors + gyro_errors)
        if gravity is not None:
            self._details["acc_angle_deg"] = float(self._x[0] + _wrap(gravity[0] - self._neutral - self._x[0]))
        if "transverse_gyro_magnitude_mismatch" in gyro_errors:
            # The supplied two-link motion violates the hinge model. Gravity
            # alone must not hide that failure and turn it into a bias update.
            observations = dict(gravity=None, transverse_gyro=None)
            reasons.append("hinge_motion_inconsistent")
        qualified = {}
        short_gap_sources = set()
        continuous_jump = False
        for name, observation in observations.items():
            if observation is None:
                self._mark_gap(name)
                continue
            passed, reason = self._qualify(name, observation)
            if passed:
                qualified[name] = observation
                if reason == "short_observation_gap_recovered":
                    short_gap_sources.add(name)
            else:
                reasons.append(f"{name}_{reason}")
                if reason == "continuous_observation_jump":
                    continuous_jump = True
        if continuous_jump:
            # A newly available gyro-angle observation cannot retroactively
            # justify a simultaneous jump in the established gravity source.
            # Growing prediction covariance is not independent evidence that
            # the discontinuous observation became physically valid.
            for name in list(qualified):
                if self._sources[name]["beta"] is None:
                    del qualified[name]
                    reasons.append(f"{name}_unestablished_during_observation_jump")
        if gravity is not None and transverse is not None:
            disagreement = abs(_wrap(gravity[0] - transverse[0]))
            bound = max(8., 3. * math.hypot(gravity[1], transverse[1]))
            if disagreement > bound:
                reasons.append("gravity_transverse_disagreement")
                # One source may be independently rejected as a discontinuity.
                # If both are temporally plausible, the conflict is ambiguous:
                # neither is silently preferred because it has a lower variance.
                if len(qualified) == 2:
                    qualified.clear()
                elif len(qualified) == 1:
                    name = next(iter(qualified))
                    innovation = abs(_wrap(qualified[name][0] - self._neutral - self._x[0]))
                    if innovation > max(6., 3. * qualified[name][1]):
                        qualified.clear()
        if not qualified:
            return self._finish()
        # Treat repeated measurements as at most approximately 30 Hz worth of
        # independent evidence; this is an explicit conservative model, not an
        # estimated autocorrelation time or a sensor specification.
        sampling_factor = max(1., self.sample_rate_hz / 30.)
        variances = {name: observation[1] ** 2 * sampling_factor for name, observation in qualified.items()}
        inverse_variance = sum(1. / value for value in variances.values())
        measurement_variance = 1. / inverse_variance
        innovation = sum(_wrap(obs[0] - self._neutral - self._x[0]) / variances[name]
                         for name, obs in qualified.items()) * measurement_variance
        self._details["innovation_deg"] = float(innovation)
        gap_sources = [name for name in qualified
                       if self._sources[name]["gap"] and name not in short_gap_sources]
        innovation_bound = max(6., 4. * math.sqrt(self._p[0, 0] + measurement_variance))
        large_recovery = False
        if abs(innovation) > innovation_bound:
            recoverable = any(self._sources[name]["beta"] is not None
                              and ((self._time if self._sources[name]["anchor_epoch"] != self._history_epoch
                                    else self._sources[name]["gap_end"])
                                   - self._sources[name]["gap_start"] >= _LARGE_RECOVERY_GAP_S)
                              for name in gap_sources)
            if not recoverable:
                reasons.append("innovation_inconsistent")
                return self._finish()
            # A confirmed current angle after a real gap is new information,
            # not proof that its difference was all gyro bias. Enlarge the
            # explicitly uncertain angle state, preserve missing-history status.
            self._mark_history_loss("large_observation_reacquisition")
            self._p[0, 0] = max(self._p[0, 0], innovation**2)
            large_recovery = True
        # A newly appearing second source is not a loss of the still-tracked
        # first source, and must not repeatedly erase learned bias confidence.
        continuously_observed = any(name not in gap_sources for name in qualified)
        reacquiring = large_recovery or (bool(gap_sources) and self._history_loss and not continuously_observed)
        if reacquiring:
            self._p[0, 1] = self._p[1, 0] = 0.
            self._p[1, 1] = max(self._p[1, 1], 4.)
        gain = self._p[:, 0] / (self._p[0, 0] + measurement_variance)
        if reacquiring:
            gain[1] = 0.
        angle_before = float(self._x[0])
        self._x += gain * innovation
        # Joseph form maintains a symmetric nonnegative covariance even when
        # the bias update has deliberately been disabled during reacquisition.
        residual_map = np.eye(2) - np.outer(gain, [1., 0.])
        self._p = residual_map @ self._p @ residual_map.T + np.outer(gain, gain) * measurement_variance
        self._p = .5 * (self._p + self._p.T)
        for name, observation in qualified.items():
            self._accept_anchor(name, observation)
            if name in short_gap_sources:
                reasons.append(f"{name}_short_observation_gap_recovered")
        self._details.update(correction_active=True, acc_observable="gravity" in qualified,
                             observation_sources=list(qualified),
                             correction_dps=float((self._x[0] - angle_before) / dt),
                             rate_dps=float(raw_rate - self._x[1]))
        if reacquiring:
            reasons.append("conditional_observation_reacquisition")
        return self._finish(reacquiring=reacquiring)

    def _finish(self, *, reacquiring=False):
        observed = bool(self._details["observation_sources"])
        # A rejected acceleration observation is not a missing gyro trajectory.
        # Count it and express current prediction-only quality; do not make
        # history_loss permanent unless samples were lost/clipped or a large
        # reacquisition actually established past trajectory uncertainty.
        rejection_names = {
            "acc_norm", "axis_gravity_mismatch", "transverse_gyro_magnitude_mismatch",
            "hinge_motion_inconsistent", "gravity_transverse_disagreement", "innovation_inconsistent",
        }
        rejected = [reason for reason in self._details["reasons"]
                    if reason in rejection_names or reason.endswith(("continuous_observation_jump",
                        "observation_motion_inconsistent", "unestablished_during_observation_jump"))]
        if rejected:
            self._rejected_observation_samples += 1
            for reason in set(rejected):
                self._rejection_counts[reason] = self._rejection_counts.get(reason, 0) + 1
        self._details["observation_rejection_active"] = bool(rejected)
        if observed:
            self._last_observation_time = self._time
        if self._history_loss:
            self._details["reasons"].append("past_unobserved_motion")
        if reacquiring:
            quality = "reacquiring"
        elif observed:
            quality = "observed_after_loss" if self._history_loss else "tracking"
        else:
            quality = "uncertain" if self._history_loss else "prediction_only"
        self._details["quality"] = quality
        self._details["reasons"] = list(dict.fromkeys(self._details["reasons"]))
        return self.snapshot()

    def snapshot(self):
        """Return independent JSON-safe diagnostics; no state is advanced."""
        details = dict(self._details)
        details["reasons"] = list(details["reasons"])
        details["observation_sources"] = list(details["observation_sources"])
        return {
            **details,
            "angle_deg": float(self._x[0]) if self._initialized else None,
            "angle_std_deg": float(math.sqrt(max(0., self._p[0, 0]))) if self._initialized else None,
            "relative_bias_dps": float(self._x[1]),
            "relative_bias_std_dps": float(math.sqrt(max(0., self._p[1, 1]))),
            "neutral_ready": self._initialized,
            "valid": self._details["quality"] in ("tracking", "observed_after_loss"),
            "uncertain": self._history_loss or self._details["quality"] != "tracking",
            "history_loss": self._history_loss,
            "history_loss_causes": list(self._history_loss_causes),
            "current_angle_observed": bool(details["observation_sources"]),
            "time_since_angle_observation_s": float(self._time - self._last_observation_time) if self._initialized else None,
            "observation_rejection_count": self._rejected_observation_samples,
            "rejection_counts": dict(self._rejection_counts),
            "samples": self._samples,
            "observer": "experimental_constrained_hinge_angle_bias_v1",
            "recovery_policy": "bounded_short_gap_anchor_v2",
            "gyro_history_continuity_basis": "software_detected_pair_and_saturation_events_not_verified_device_sampling",
            "uncertainty_is_model_based_not_validated_accuracy": True,
        }
