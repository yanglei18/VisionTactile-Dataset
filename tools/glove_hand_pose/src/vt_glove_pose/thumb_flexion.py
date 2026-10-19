"""Conditional relative-1DOF thumb estimator, independent of VQF/Qt.

Inputs are simultaneous, bias-corrected vectors in calibrated functional
frames: positive flexion is rotation about -x. The two physical hinge axes
must be parallel and the sensors must straddle the intended joint. Independent
PCA calibration does NOT establish either assumption. The gates below detect
some violations; they cannot certify anatomy, reject all linear acceleration,
or recover a trajectory lost to sensor saturation.

Gyro prediction is immediate. Observable gravity corrects the current relative
angle, never toward an assumed straight hand. A stable gravity observation may
reacquire the current angle after a disturbance, while history_loss remains
latched. Gravity only observes the angle modulo 360 degrees. State restoration
is for offline replay/continuous processing, not automatic live zero reuse.
"""
from __future__ import annotations

import math

import numpy as np


_KIND = "vt_glove_pose.thumb_flexion_state"
_SCHEMA_VERSION = 1
_MAX_CORRECTION_DPS = 8.0
_QUALITIES = {"uninitialized", "tracking", "prediction_only", "uncertain",
              "reacquiring", "observed_after_loss"}
_REASONS = {"neutral_required", "pair_unavailable", "saturated_input",
            "acc_norm", "axis_gravity_mismatch", "low_gravity_projection",
            "off_axis_motion_mismatch", "innovation_too_large",
            "acc_motion_inconsistent", "gravity_reacquisition",
            "past_unobserved_motion"}
_STATE_FIELDS = {
    "initialized", "neutral_angle_deg", "angle_deg", "gyro_phase_deg",
    "acc_reference_beta_deg", "acc_reference_phase_deg",
    "still_reference_beta_deg", "still_time_s", "rate_dps", "acc_angle_deg",
    "innovation_deg", "correction_dps", "acc_observable", "correction_active",
    "history_loss", "samples", "quality", "reasons",
}


def _number(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
    return float(value)


def _boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f"{name} must be boolean")
    return value


def _vector(value, name, bound):
    try:
        raw = np.asarray(value)
        if raw.shape != (3,) or raw.dtype.kind not in "ifu":
            raise ValueError
        result = raw.astype(float)
        if not np.all(np.isfinite(result)) or np.any(np.abs(result) > bound):
            raise ValueError
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain three finite numeric values within +/-{bound}") from exc
    return result


def _wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _gravity(proximal, distal):
    """Return functional-frame angle beta and geometry gate failure reasons."""
    np_, nd = float(np.linalg.norm(proximal)), float(np.linalg.norm(distal))
    if not .85 <= np_ <= 1.15 or not .85 <= nd <= 1.15 or abs(np_ - nd) > .10:
        return None, ["acc_norm"]
    p, d = proximal / np_, distal / nd
    reasons = []
    if abs(float(p[0] - d[0])) > .12:
        reasons.append("axis_gravity_mismatch")
    if min(math.hypot(p[1], p[2]), math.hypot(d[1], d[2])) < .30:
        reasons.append("low_gravity_projection")
    if reasons:
        return None, reasons
    beta = math.degrees(math.atan2(p[1] * d[2] - p[2] * d[1],
                                  p[1] * d[1] + p[2] * d[2]))
    return _wrap(beta), []


class ThumbFlexionEstimator:
    """Predict from gyro differences; correct only with gated relative gravity.

    ``update`` consumes one paired IMU sample, never one UI refresh. Gyros are
    degrees/s and accelerations are g, after mounting and static-bias correction.
    Passing an invalid number or dt raises ValueError without changing state.
    ``pair_valid=False`` records a known missing pair and does not invent motion.
    Saturated gyro values are still integrated as reported, explicitly uncertain.
    """

    def __init__(self, sample_rate_hz):
        self.sample_rate_hz = _number(sample_rate_hz, "sample_rate_hz", 10., 2000.)
        self._state = {
            "initialized": False, "neutral_angle_deg": None, "angle_deg": None,
            "gyro_phase_deg": 0., "acc_reference_beta_deg": None,
            "acc_reference_phase_deg": None, "still_reference_beta_deg": None,
            "still_time_s": 0., "rate_dps": 0., "acc_angle_deg": None,
            "innovation_deg": None, "correction_dps": 0.,
            "acc_observable": False, "correction_active": False,
            "history_loss": False, "samples": 0, "quality": "uninitialized",
            "reasons": ["neutral_required"],
        }

    def reset(self, proximal_acc, distal_acc, *, current_proximal_acc=None,
              current_distal_acc=None):
        """Explicitly declare this accepted gravity pose the new relative zero.

        The positional vectors always describe the explicit neutral capture.
        When first enabling the estimator or changing axes after that capture,
        pass both current vectors to initialize at the observed *current* angle,
        not at a fictional zero. They do not replace the neutral reference.
        Without them this is an explicit zero at the neutral capture itself.

        Caller must establish a genuine neutral/static capture, hinge-axis
        applicability, and a suitable current observation. A single pair of
        vectors cannot prove the absence of all linear acceleration. Missing,
        unobservable or inconsistent gravity is rejected transactionally.
        """
        p = _vector(proximal_acc, "proximal_acc", 100.)
        d = _vector(distal_acc, "distal_acc", 100.)
        beta, reasons = _gravity(p, d)
        if reasons:
            raise ValueError("neutral gravity is not observable/consistent: " + ", ".join(reasons))
        current_beta = beta
        if (current_proximal_acc is None) != (current_distal_acc is None):
            raise ValueError("both current acceleration vectors are required")
        if current_proximal_acc is not None:
            cp = _vector(current_proximal_acc, "current_proximal_acc", 100.)
            cd = _vector(current_distal_acc, "current_distal_acc", 100.)
            current_beta, reasons = _gravity(cp, cd)
            if reasons:
                raise ValueError("current gravity is not observable/consistent: " + ", ".join(reasons))
        initial_angle = _wrap(current_beta - beta)
        state = ThumbFlexionEstimator(self.sample_rate_hz)._state
        state.update(initialized=True, neutral_angle_deg=beta, angle_deg=initial_angle,
                     acc_reference_beta_deg=current_beta, acc_reference_phase_deg=0.,
                     still_reference_beta_deg=current_beta, acc_angle_deg=initial_angle,
                     innovation_deg=0., acc_observable=True, correction_active=True,
                     quality="tracking", reasons=[])
        self._state = state
        return self.snapshot()

    def update(self, proximal_gyro_dps, proximal_acc_g, distal_gyro_dps, distal_acc_g,
               *, dt_s=None, proximal_saturated=False, distal_saturated=False,
               pair_valid=True):
        nominal = 1. / self.sample_rate_hz
        dt = nominal if dt_s is None else _number(dt_s, "dt_s", .25 * nominal, 4 * nominal)
        proximal_saturated = _boolean(proximal_saturated, "proximal_saturated")
        distal_saturated = _boolean(distal_saturated, "distal_saturated")
        pair_valid = _boolean(pair_valid, "pair_valid")
        if not self._state["initialized"]:
            raise ValueError("reset with an explicit observable neutral pose before update")
        # Check everything before mutation. A known unavailable pair may have
        # None vectors; it is represented as data loss, not a zero-gyro sample.
        if pair_valid:
            pg = _vector(proximal_gyro_dps, "proximal_gyro_dps", 10000.)
            dg = _vector(distal_gyro_dps, "distal_gyro_dps", 10000.)
            pa = _vector(proximal_acc_g, "proximal_acc_g", 100.)
            da = _vector(distal_acc_g, "distal_acc_g", 100.)
        s = dict(self._state)
        s.update(samples=s["samples"] + 1, acc_observable=False, correction_active=False,
                 correction_dps=0., acc_angle_deg=None, innovation_deg=None, reasons=[])
        if not pair_valid:
            s.update(history_loss=True, quality="uncertain", rate_dps=0.,
                     still_time_s=0., still_reference_beta_deg=None,
                     reasons=["pair_unavailable", "past_unobserved_motion"])
            self._state = s
            return self.snapshot()
        rate = float(pg[0] - dg[0])
        s["rate_dps"] = rate
        s["gyro_phase_deg"] += rate * dt
        predicted = s["angle_deg"] + rate * dt
        s["angle_deg"] = predicted
        if proximal_saturated or distal_saturated:
            s.update(history_loss=True, quality="uncertain", still_time_s=0.,
                     still_reference_beta_deg=None,
                     reasons=["saturated_input", "past_unobserved_motion"])
            self._state = s
            return self.snapshot()

        beta, reasons = _gravity(pa, da)
        max_speed = max(float(np.linalg.norm(pg)), float(np.linalg.norm(dg)))
        if beta is not None:
            # For q_distal = q_proximal Rx(-beta), body angular velocities
            # satisfy gyro_distal.yz = Rx(+beta) gyro_proximal.yz. Directly
            # subtracting yz would incorrectly reject normal flexion.
            r = math.radians(beta)
            c, sn = math.cos(r), math.sin(r)
            aligned_yz = np.array([c * pg[1] - sn * pg[2], sn * pg[1] + c * pg[2]])
            if np.linalg.norm(aligned_yz - dg[1:]) > 8. + .12 * max_speed:
                reasons.append("off_axis_motion_mismatch")
        s["acc_observable"] = beta is not None and not reasons
        if s["acc_observable"]:
            previous = s["still_reference_beta_deg"]
            stable = max_speed < 3. and (previous is None or abs(_wrap(beta - previous)) <= .5)
            s["still_time_s"] = min(10., s["still_time_s"] + dt) if stable else 0.
            # Bound change across the whole stability interval, not merely
            # adjacent samples: a steadily moving angle is not a stable hold.
            if previous is None or not stable:
                s["still_reference_beta_deg"] = beta
            innovation = _wrap(beta - s["neutral_angle_deg"] - predicted)
            s["acc_angle_deg"] = predicted + innovation
            s["innovation_deg"] = innovation
            phase_delta = s["gyro_phase_deg"] - s["acc_reference_phase_deg"]
            motion_error = _wrap(beta - s["acc_reference_beta_deg"] - phase_delta)
            if abs(motion_error) > 2. + min(8., .1 * abs(phase_delta)):
                reasons.append("acc_motion_inconsistent")
            if abs(innovation) > 12.:
                reasons.append("innovation_too_large")
            reacquire = bool(reasons) and s["still_time_s"] >= .5
            if not reasons or reacquire:
                # Quiet observable gravity converges faster, without assuming
                # the joint is straight. Reacquisition is rate-limited too.
                tau = .75 + 1.75 * min(max_speed / 90., 1.)
                change = float(np.clip(-math.expm1(-dt / tau) * innovation,
                                       -_MAX_CORRECTION_DPS * dt, _MAX_CORRECTION_DPS * dt))
                s["angle_deg"] += change
                s["correction_dps"] = change / dt
                s["correction_active"] = True
                s["acc_reference_beta_deg"] = beta
                s["acc_reference_phase_deg"] = s["gyro_phase_deg"]
                if reacquire or (abs(innovation) > 2. and s["history_loss"]):
                    s["quality"] = "reacquiring"
                    reasons.append("gravity_reacquisition")
                else:
                    s["quality"] = "observed_after_loss" if s["history_loss"] else "tracking"
            else:
                s["quality"] = "uncertain" if s["history_loss"] else "prediction_only"
        else:
            s.update(still_time_s=0., still_reference_beta_deg=None,
                     quality="uncertain" if s["history_loss"] else "prediction_only")
        if s["history_loss"]:
            reasons.append("past_unobserved_motion")
        s["reasons"] = reasons
        self._state = s
        return self.snapshot()

    def snapshot(self):
        """Read-only JSON-safe result; reading it never advances the estimator."""
        s = self._state
        return {"angle_deg": s["angle_deg"], "rate_dps": s["rate_dps"],
                "acc_angle_deg": s["acc_angle_deg"], "innovation_deg": s["innovation_deg"],
                "correction_dps": s["correction_dps"], "acc_observable": s["acc_observable"],
                "correction_active": s["correction_active"], "neutral_ready": s["initialized"],
                "valid": s["quality"] in ("tracking", "observed_after_loss"),
                "uncertain": s["history_loss"] or s["quality"] != "tracking",
                "quality": s["quality"], "reasons": list(s["reasons"]),
                "history_loss": s["history_loss"], "samples": s["samples"]}

    def export_state(self):
        """Return a deep-independent strict JSON checkpoint, offline only."""
        state = dict(self._state)
        state["reasons"] = list(state["reasons"])
        return {"kind": _KIND, "schema_version": _SCHEMA_VERSION, "offline_only": True,
                "sample_rate_hz": self.sample_rate_hz, "state": state}

    @classmethod
    def from_state(cls, data):
        """Validate a complete checkpoint before constructing a new estimator."""
        if type(data) is not dict or set(data) != {"kind", "schema_version", "offline_only", "sample_rate_hz", "state"}:
            raise ValueError("unexpected or missing thumb checkpoint fields")
        if data["kind"] != _KIND or type(data["schema_version"]) is not int or data["schema_version"] != _SCHEMA_VERSION or data["offline_only"] is not True:
            raise ValueError("unsupported thumb checkpoint kind/version/offline flag")
        sample_rate = _number(data["sample_rate_hz"], "sample_rate_hz", 10., 2000.)
        raw = data["state"]
        if type(raw) is not dict or set(raw) != _STATE_FIELDS:
            raise ValueError("unexpected or missing thumb runtime fields")
        s = {}
        for key in ("initialized", "history_loss", "acc_observable", "correction_active"):
            s[key] = _boolean(raw[key], key)
        for key in ("neutral_angle_deg", "acc_reference_beta_deg", "still_reference_beta_deg"):
            s[key] = None if raw[key] is None else _number(raw[key], key, -180., 180.)
        for key in ("angle_deg", "acc_reference_phase_deg", "acc_angle_deg"):
            s[key] = None if raw[key] is None else _number(raw[key], key, -1e9, 1e9)
        s["gyro_phase_deg"] = _number(raw["gyro_phase_deg"], "gyro_phase_deg", -1e9, 1e9)
        s["still_time_s"] = _number(raw["still_time_s"], "still_time_s", 0., 10.)
        s["rate_dps"] = _number(raw["rate_dps"], "rate_dps", -20000., 20000.)
        s["correction_dps"] = _number(raw["correction_dps"], "correction_dps", -_MAX_CORRECTION_DPS - 1e-12, _MAX_CORRECTION_DPS + 1e-12)
        s["innovation_deg"] = None if raw["innovation_deg"] is None else _number(raw["innovation_deg"], "innovation_deg", -180., 180.)
        if type(raw["samples"]) is not int or not 0 <= raw["samples"] <= 2**53 - 1:
            raise ValueError("invalid thumb sample count")
        s["samples"] = raw["samples"]
        if type(raw["quality"]) is not str or raw["quality"] not in _QUALITIES:
            raise ValueError("invalid thumb quality")
        s["quality"] = raw["quality"]
        reasons = raw["reasons"]
        if type(reasons) is not list or len(reasons) > len(_REASONS) or any(type(v) is not str or v not in _REASONS for v in reasons) or len(set(reasons)) != len(reasons):
            raise ValueError("invalid thumb quality reasons")
        s["reasons"] = list(reasons)
        required = ("neutral_angle_deg", "angle_deg", "acc_reference_beta_deg", "acc_reference_phase_deg")
        if s["initialized"]:
            if any(s[key] is None for key in required) or s["quality"] == "uninitialized":
                raise ValueError("initialized thumb state is incomplete")
        elif s != cls(sample_rate)._state:
            raise ValueError("uninitialized thumb state contains runtime history")
        if s["quality"] in ("tracking", "observed_after_loss", "reacquiring") and not (s["acc_observable"] and s["correction_active"]):
            raise ValueError("thumb quality contradicts observation state")
        if s["correction_active"] and (not s["acc_observable"] or s["acc_angle_deg"] is None or s["innovation_deg"] is None):
            raise ValueError("thumb correction lacks an observation")
        observation_fields = (s["acc_angle_deg"], s["innovation_deg"])
        if ((s["acc_observable"] and any(v is None for v in observation_fields))
                or (not s["acc_observable"] and any(v is not None for v in observation_fields))):
            raise ValueError("thumb observation fields are inconsistent")
        if not s["correction_active"] and s["correction_dps"] != 0.:
            raise ValueError("inactive thumb correction must be zero")
        if ((s["quality"] in ("observed_after_loss", "uncertain") and not s["history_loss"])
                or (s["quality"] in ("tracking", "prediction_only") and s["history_loss"])):
            raise ValueError("thumb quality contradicts history-loss state")
        if s["quality"] in ("prediction_only", "uncertain", "uninitialized") and s["correction_active"]:
            raise ValueError("unobserved thumb quality cannot claim active correction")
        if ((s["quality"] == "tracking" and s["reasons"])
                or (s["quality"] == "observed_after_loss" and s["reasons"] != ["past_unobserved_motion"])):
            raise ValueError("confident thumb observation retains rejected measurement gates")
        if (s["quality"] == "reacquiring") != ("gravity_reacquisition" in s["reasons"]):
            raise ValueError("thumb reacquisition provenance is inconsistent")
        if s["history_loss"] != ("past_unobserved_motion" in s["reasons"]):
            raise ValueError("thumb loss provenance is incomplete")
        if any(reason in s["reasons"] for reason in ("saturated_input", "pair_unavailable")) and not (s["history_loss"] and s["quality"] == "uncertain" and not s["acc_observable"]):
            raise ValueError("thumb input-loss flags contradict its quality")
        if ("neutral_required" in s["reasons"]) != (not s["initialized"]):
            raise ValueError("thumb neutral provenance is inconsistent")
        if s["acc_observable"] and any(reason in s["reasons"] for reason in ("acc_norm", "axis_gravity_mismatch", "low_gravity_projection", "off_axis_motion_mismatch")):
            raise ValueError("failed gravity gates cannot claim observability")
        if any(reason in s["reasons"] for reason in ("acc_motion_inconsistent", "innovation_too_large")) and not s["acc_observable"]:
            raise ValueError("thumb innovation gate lacks an observation")
        if s["still_time_s"] > 0 and s["still_reference_beta_deg"] is None:
            raise ValueError("thumb stability history lacks a reference")
        instance = cls(sample_rate)
        instance._state = s
        return instance
