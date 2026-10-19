"""Conditional scalar thumb preview from raw pairs, not a certified calibration.

The stored neutral remains the reference. Initializing while bent uses current
gravity, never a fabricated zero. No PoseEngine state or serial port is owned.
"""
from __future__ import annotations

import json
import math

import numpy as np

from .config import pair_gyro_scales, validate_config
from .hinge_observer import ConstrainedHingeObserver, _gravity
from .records import Sample


KIND = "vt_glove_pose.thumb_angle_candidate"


def _vector(value, size, name):
    raw = np.asarray(value)
    has_boolean = any(isinstance(v, (bool, np.bool_)) for v in np.asarray(value, dtype=object).flat)
    if has_boolean or raw.dtype.kind not in "iuf" or raw.shape != (size,) or not np.all(np.isfinite(raw)):
        raise ValueError(f"{name} must be a finite numeric vector of size {size}")
    return raw.astype(float)


def validate_candidate(profile):
    """Copy an explicit experimental profile, retaining rejected fit evidence."""
    if not isinstance(profile, dict):
        raise ValueError("candidate profile must be a JSON object")
    # Validate before JSON copying can silently coerce integer mapping keys.
    validate_config(profile.get("config"))
    if isinstance(profile.get("bias_dps"), dict) and any(type(k) is not str for k in profile["bias_dps"]):
        raise ValueError("bias_dps node keys must be canonical strings")
    try:
        data = json.loads(json.dumps(profile, allow_nan=False))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("candidate profile must be finite JSON data") from exc
    if (not isinstance(data, dict) or data.get("kind") != KIND
            or type(data.get("schema_version")) is not int or data["schema_version"] != 1
            or data.get("scope") != "conditional_scalar_preview"
            or data.get("certified_calibration") is not False
            or data.get("anatomical_accuracy_verified") is not False):
        raise ValueError("expected an explicitly uncertified conditional_scalar_preview candidate")
    data["config"] = validate_config(data.get("config"))
    nodes = data.get("node_ids")
    if (not isinstance(nodes, list) or len(nodes) != 2
            or any(type(n) is not int or not 1 <= n <= 12 for n in nodes)
            or nodes[0] == nodes[1] or (nodes[0]-1)//6 != (nodes[1]-1)//6):
        raise ValueError("candidate node_ids must be distinct nodes in one wire group")
    bias = data.get("bias_dps")
    if not isinstance(bias, dict) or set(bias) != {str(n) for n in nodes}:
        raise ValueError("bias_dps must contain exactly the selected node IDs")
    for node in nodes:
        if np.linalg.norm(_vector(bias[str(node)], 3, "bias_dps")) > 10:
            raise ValueError("bias_dps norm must be <=10 deg/s")
    frames = data.get("frames")
    if not isinstance(frames, dict) or frames.get("status") != "frames_ready":
        raise ValueError("candidate needs frames_ready functional frames")
    neutral = []
    for role in ("prox", "dist"):
        key = role+"_sensor_to_functional"
        raw = np.asarray(frames.get(key))
        has_boolean = any(isinstance(v, (bool, np.bool_))
                          for v in np.asarray(frames.get(key), dtype=object).flat)
        if has_boolean or raw.dtype.kind not in "iuf" or raw.shape != (3, 3) or not np.all(np.isfinite(raw)):
            raise ValueError(f"{key} must be a finite SO(3) matrix")
        matrix = raw.astype(float)
        if (not np.allclose(matrix@matrix.T, np.eye(3), rtol=0., atol=1e-6)
                or not np.isclose(np.linalg.det(matrix), 1., rtol=0., atol=1e-6)):
            raise ValueError(f"{key} must be a proper SO(3) rotation")
        neutral.append(_vector(frames.get("functional_neutral_"+role+"_acc_g"), 3, "functional neutral"))
    # Confirm that the stored reference is usable by the unchanged observer.
    # This is geometric/input validation, not certification of anatomy/axes.
    ConstrainedHingeObserver(data["config"]["sample_rate_hz"]).reset(*neutral)
    if not isinstance(data.get("axis_fit"), dict):
        raise ValueError("axis_fit must preserve the native fitter evidence")
    return data


def _node_roles(config, nodes):
    roles = {}
    for name, finger in config["fingers"].items():
        for role in ("proximal", "distal"):
            if finger[role] in nodes:
                roles[finger[role]] = (name, role)
    for role in ("palm_node", "auxiliary_node"):
        if config[role] in nodes:
            roles[config[role]] = (role,)
    return roles


class CandidateAngleProcessor:
    """Consume ordered wire samples and expose an experimental angle snapshot.

    A completed pair returns a snapshot even while initialization waits for
    observable gravity. Other samples return None. A missing pair marks loss
    once per outage; no missing time is integrated or interpolated. Equal USB
    timestamps do not merge frames. Snapshot polling never integrates gyro.
    """

    def __init__(self, profile, config=None):
        self.profile = validate_candidate(profile)
        recorded = self.profile["config"]
        self.config = validate_config(recorded if config is None else config)
        self.node_ids = tuple(self.profile["node_ids"])
        if (_node_roles(recorded, self.node_ids) != _node_roles(self.config, self.node_ids)
                or any(not math.isclose(recorded[k], self.config[k], rel_tol=1e-12)
                       for k in ("sample_rate_hz", "accel_g_per_lsb"))
                or pair_gyro_scales(recorded, *self.node_ids) != pair_gyro_scales(self.config, *self.node_ids)):
            raise ValueError("runtime selected nodes, effective units or sample rate differ from candidate")
        self._scales = pair_gyro_scales(self.config, *self.node_ids)
        self._frames = [np.array(self.profile["frames"][r+"_sensor_to_functional"], float)
                        for r in ("prox", "dist")]
        self._neutral = [np.array(self.profile["frames"]["functional_neutral_"+r+"_acc_g"], float)
                         for r in ("prox", "dist")]
        self._bias = [np.array(self.profile["bias_dps"][str(n)], float) for n in self.node_ids]
        self._observer = None
        self._pending = None
        self._last_node_time = {}
        self._last_pair_time = None
        self._latest_input_time = None
        self._awaiting_pair = True
        self._loss_episode = False
        self._history_causes = []
        self._reasons = ["waiting_for_current_gravity"]
        self.pair_count = 0
        self._gap_limit = max(.1, 5./self.config["sample_rate_hz"])

    def _mark_loss(self, reason):
        if reason not in self._history_causes:
            self._history_causes.append(reason)
        if self._observer is not None and not self._loss_episode:
            self._observer.update(None, None, None, None, pair_valid=False)
        self._loss_episode = True
        self._awaiting_pair = True
        self._reasons = [reason]

    def process(self, sample):
        if not isinstance(sample, Sample):
            raise ValueError("process expects a validated Sample")
        low, high = sorted(self.node_ids)
        node, stamp = sample.node_id, sample.host_monotonic_s
        if node in self.node_ids:
            previous = self._last_node_time.get(node)
            if previous is not None and stamp < previous:
                self._pending = None
                self._mark_loss("backward_host_time")
                return None
            self._last_node_time[node] = stamp
            self._latest_input_time = stamp if self._latest_input_time is None else max(stamp, self._latest_input_time)
        if self._pending is not None:
            first, previous_node = self._pending
            if (node != previous_node+1 or node > high or stamp != first.host_monotonic_s
                    or sample.host_wall_s != first.host_wall_s):
                self._pending = None
                self._mark_loss("missing_or_mismatched_pair")
            else:
                self._pending = (first, node)
                if node == high:
                    self._pending = None
                    pair = (first, sample) if self.node_ids[0] == low else (sample, first)
                    return self._process_pair(pair)
                return None
        if node == low:
            self._pending = (sample, node)
        elif node == high:
            self._mark_loss("missing_or_mismatched_pair")
        return None

    def _process_pair(self, pair):
        stamp = pair[0].host_monotonic_s
        if self._last_pair_time is not None and stamp-self._last_pair_time > self._gap_limit:
            self._mark_loss("host_gap")
        self._last_pair_time = stamp
        self.pair_count += 1
        self._awaiting_pair = False
        self._loss_episode = False
        self._reasons = []
        raw = [np.asarray(sample.values, float) for sample in pair]
        near = [bool(np.any(np.abs(values[:3]) >= 32700)) for values in raw]
        gyros = [frame@(values[:3]*scale-bias) for frame,values,scale,bias in
                 zip(self._frames,raw,self._scales,self._bias)]
        accels = [frame@(values[3:]*self.config["accel_g_per_lsb"]) for frame,values in zip(self._frames,raw)]
        if self._observer is None:
            if any(near):
                if "gyro_saturation" not in self._history_causes:
                    self._history_causes.append("gyro_saturation")
                self._reasons = ["initial_gyro_saturated"]
                return self.snapshot()
            observer = ConstrainedHingeObserver(self.config["sample_rate_hz"])
            try:
                observer.reset(*self._neutral, current_proximal_acc=accels[0], current_distal_acc=accels[1])
            except ValueError:
                # Retain the actual geometric cause rather than making a
                # healthy incoming stream look like a connection problem.
                _, causes = _gravity(*accels)
                self._reasons = ["waiting_for_observable_current_gravity", *causes]
                return self.snapshot()
            self._observer = observer
            if self._history_causes:
                # reset() clears the observer's history. Preserve a loss that
                # preceded initialization without integrating this pair twice.
                observer.update(None, None, None, None, pair_valid=False)
            return self.snapshot()
        try:
            self._observer.update(gyros[0], accels[0], gyros[1], accels[1],
                                  proximal_saturated=near[0], distal_saturated=near[1])
        except ValueError:
            self._mark_loss("invalid_converted_input")
        if any(near) and "gyro_saturation" not in self._history_causes:
            self._history_causes.append("gyro_saturation")
        return self.snapshot()

    def snapshot(self, now=None):
        if now is None:
            now = self._latest_input_time
        if now is not None and (type(now) not in (int, float) or not math.isfinite(now)):
            raise ValueError("snapshot time must be finite")
        age = None if now is None or self._last_pair_time is None else now-self._last_pair_time
        if age is not None and age < 0:
            self._pending = None
            self._mark_loss("backward_snapshot_time")
        elif age is not None and age > self.config["stale_after_s"]:
            self._pending = None
            self._mark_loss("stale_data")
        fresh = age is not None and 0 <= age <= self.config["stale_after_s"] and not self._awaiting_pair
        state = self._observer.snapshot() if self._observer is not None else dict(
            angle_deg=None, quality="uninitialized", current_angle_observed=False,
            valid=False, history_loss=False, reasons=[], samples=0)
        reasons = list(dict.fromkeys(state["reasons"]+self._reasons))
        loss = bool(self._history_causes) or state["history_loss"]
        if loss and "past_unobserved_motion" not in reasons:
            reasons.append("past_unobserved_motion")
        state.update(angle_deg=state["angle_deg"] if fresh else None,
                     quality=state["quality"] if fresh else "unavailable",
                     current_angle_observed=bool(fresh and state["current_angle_observed"]),
                     valid=bool(fresh and state["valid"]), paired_fresh=fresh,
                     history_loss=loss, reasons=reasons,
                     history_loss_causes=list(dict.fromkeys(state.get("history_loss_causes", [])+self._history_causes)),
                     host_monotonic_s=self._last_pair_time, age_s=age,
                     pair_count=self.pair_count, node_ids=list(self.node_ids),
                     kind="vt_glove_pose.thumb_angle_candidate_snapshot", scope="conditional_scalar_preview",
                     certified_calibration=False, anatomical_accuracy_verified=False,
                     anatomical_flexion_sign_verified=False, anatomical_zero_verified=False,
                     native_axis_fit_status=self.profile["axis_fit"].get("candidate_status"),
                     time_basis="configured_nominal_period_no_device_timestamps",
                     physical_synchronization_verified=False)
        return state
