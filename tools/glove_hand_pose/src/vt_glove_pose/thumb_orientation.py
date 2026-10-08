"""Two independent sensor-orientation previews, without a hinge calibration.

This deliberately does not subtract the two VQF quaternions: independent
six-axis filters have an unknown inter-frame heading. A display reference is
not an anatomical zero, nor evidence that the hand is stationary. Every valid
sample advances its own filter by the configured nominal sample period.
"""
from __future__ import annotations

import math

import numpy as np

from .config import gyro_scale, validate_config
from .engine import PoseEngine
from .math3d import conj, mul, normalize
from .records import Sample


class ThumbOrientationProcessor:
    """No serial access, mounting axes, joint-angle fitting, or automatic zero.

    ``process(sample)`` returns a new snapshot only when a matched selected
    pair is completed, otherwise None. ``set_reference()`` explicitly stores
    that pair as a *display* reference. ``snapshot(now=...)`` checks freshness;
    omitted ``now`` uses the latest processed sample time for offline replay.

    ``bias_dps`` may contain integer or canonical string node keys. It is an
    explicit previously estimated bias, not estimated from arbitrary motion.
    A missing interval must not be filled by feeding unrelated CSV segments
    as if continuous: make a new processor per motion recording and declare
    its first matched pair as a display reference, if that is the intention.
    """

    def __init__(self, config, *, bias_dps=None, node_ids=None):
        self.config = validate_config(config)
        finger = self.config["fingers"]["thumb"]
        nodes = (finger["proximal"], finger["distal"]) if node_ids is None else tuple(node_ids)
        if (len(nodes) != 2 or any(type(n) is not int or not 1 <= n <= 12 for n in nodes)
                or nodes[0] == nodes[1] or (nodes[0]-1)//6 != (nodes[1]-1)//6):
            raise ValueError("node_ids must select two distinct nodes in one wire group")
        self.node_ids = nodes
        self.engine = PoseEngine(self.config, algorithm_mode="legacy_quaternion_v1")
        if bias_dps is not None:
            if not isinstance(bias_dps, dict):
                raise ValueError("bias_dps must map node IDs to three finite deg/s values")
            seen = set()
            for node, value in bias_dps.items():
                if type(node) is str and node in {str(n) for n in range(1, 13)}:
                    node = int(node)
                if type(node) is not int or not 1 <= node <= 12 or node in seen:
                    raise ValueError("bias_dps node keys must be unique node IDs 1..12")
                vector = np.asarray(value, dtype=float)
                if vector.shape != (3,) or not np.all(np.isfinite(vector)) or np.linalg.norm(vector) > 10:
                    raise ValueError("bias_dps vectors must be finite, norm <= 10 deg/s")
                self.engine.bias[node] = vector.copy()
                seen.add(node)
        self._pending = None
        self._latest_pair = None
        self._reference = None
        self._reference_source = None
        self._reference_host_s = None
        self._reason = "display_reference_not_set"
        self.pair_count = 0

    def _invalidate_reference(self, reason, *, discard_pair=False):
        self._reference = None
        self._reference_source = None
        self._reference_host_s = None
        self._reason = reason
        if discard_pair:
            self._latest_pair = None
            self._pending = None

    def process(self, sample):
        if not isinstance(sample, Sample):
            raise ValueError("process expects a validated Sample")
        node = sample.node_id
        if node not in self.node_ids:
            return None
        before_invalid, before_gap = self.engine.invalid_count, self.engine.gap_count
        self.engine.process(sample)
        if self.engine.invalid_count != before_invalid:
            self._invalidate_reference("invalid_sample_or_backward_time", discard_pair=True)
            return None
        if self.engine.gap_count != before_gap:
            self._invalidate_reference("sample_gap", discard_pair=True)
        if any(abs(value) >= 32700 for value in sample.values[:3]):
            self._invalidate_reference("raw_gyro_near_limit", discard_pair=True)
            return None
        low, high = sorted(self.node_ids)
        if node == low:
            if self._pending is not None:
                self._invalidate_reference("unmatched_pair", discard_pair=True)
            self._pending = sample.host_monotonic_s
            return None
        if self._pending is None or self._pending != sample.host_monotonic_s:
            if self._latest_pair is not None:
                self._invalidate_reference("unmatched_pair", discard_pair=True)
            self._pending = None
            return None
        self._pending = None
        self._latest_pair = {}
        for n in self.node_ids:
            state = self.engine.states[n]
            self._latest_pair[n] = {key: state[key].copy() for key in ("quat", "gyro", "acc")}
            self._latest_pair[n]["time"] = state["time"]
        self.pair_count += 1
        return self.snapshot()

    def set_reference(self, *, source="operator_display_reference"):
        """Store the latest matched pose; does not infer stillness or zero bias."""
        if not isinstance(source, str) or not source or len(source) > 256:
            raise ValueError("reference source must be a nonempty short string")
        if self._latest_pair is None or self._pending is not None:
            raise ValueError("a complete valid paired observation is required before setting reference")
        latest_time = max(row["time"] for row in self._latest_pair.values())
        if self.engine.last_time is not None and self.engine.last_time - latest_time > self.config["stale_after_s"]:
            self._invalidate_reference("sample_gap", discard_pair=True)
            raise ValueError("latest paired observation is stale")
        self._reference = {n: {key: row[key].copy() for key in ("quat", "acc")}
                           for n, row in self._latest_pair.items()}
        self._reference_source = source
        self._reference_host_s = latest_time
        self._reason = None
        return self.snapshot()

    def snapshot(self, now=None):
        if now is None:
            now = self.engine.last_time
        if now is not None:
            if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
                raise ValueError("snapshot time must be finite")
        latest = self._latest_pair or {}
        ages = {n: max(0., now-row["time"]) if now is not None else 0.
                for n, row in latest.items()}
        stale = any(age > self.config["stale_after_s"] for age in ages.values())
        if stale:
            self._invalidate_reference("stale_data")
        nodes = {}
        for n in self.node_ids:
            row = latest.get(n)
            fresh = row is not None and ages[n] <= self.config["stale_after_s"]
            reference = None if self._reference is None else self._reference[n]
            relative = magnitude = gravity_change = None
            if fresh and reference is not None:
                relative = normalize(mul(conj(reference["quat"]), row["quat"]))
                # q and -q encode the same rotation; use the principal 0..180° magnitude.
                magnitude = float(np.degrees(2*np.arccos(np.clip(abs(relative[0]), 0., 1.))))
                old = reference["acc"] / np.linalg.norm(reference["acc"])
                current = row["acc"] / np.linalg.norm(row["acc"])
                gravity_change = float(np.degrees(np.arccos(np.clip(old @ current, -1., 1.))))
            nodes[str(n)] = dict(
                fresh=fresh, age_s=ages.get(n),
                sensor_quat_wxyz=None if row is None else row["quat"].tolist(),
                relative_quat_wxyz=None if relative is None else relative.tolist(),
                rotation_magnitude_deg=magnitude,
                gravity_direction_change_deg=gravity_change,
                gyro_dps=None if row is None else row["gyro"].tolist(),
                accel_g=None if row is None else row["acc"].tolist(),
                accel_norm_g=None if row is None else float(np.linalg.norm(row["acc"])),
                gyro_dps_per_lsb=gyro_scale(self.config, n))
        valid = self._reference is not None and len(latest) == 2 and not stale
        return dict(kind="thumb_sensor_orientation_preview", node_ids=list(self.node_ids),
                    pair_count=self.pair_count, host_monotonic_s=(None if not latest else
                    max(row["time"] for row in latest.values())),
                    paired_fresh=len(latest) == 2 and not stale,
                    reference_set=self._reference is not None, reference_ready=self._reference is not None,
                    reference_source=self._reference_source, reference_host_s=self._reference_host_s,
                    valid=valid, reasons=[] if valid else [self._reason or "waiting_for_pair"],
                    nodes=nodes, joint_angle_deg=None,
                    anatomical_pose_estimated=False, heading_drift_corrected=False,
                    hardware_units_verified=self.config["units_verified"],
                    physical_synchronization_verified=False,
                    time_basis="configured_nominal_period_no_device_timestamps",
                    reference_scope="independent_sensor_display_references_not_anatomical_zero",
                    limitations=["The two sensor frames are independent; no cross-sensor joint angle is computed.",
                                 "Six-axis heading can drift; no absolute heading or position is measured.",
                                 "Acceleration direction includes linear acceleration, not only gravity."])
