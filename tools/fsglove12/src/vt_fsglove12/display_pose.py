"""Display-only local rotations, never a replacement measurement or solver.

Missing joints retain their most recently displayable *local* rotations, so
valid parent motion still moves them. This is a bounded 16-slot visual cache,
not interpolation, estimation, or re-calibration. Every held value keeps its
original time and quality. Source data and ROS validity are never modified.
"""
from __future__ import annotations

from copy import deepcopy
import math

import numpy as np

from .joint_quality import joint_quality_state


class DisplayPoseCache:
    """Compose a complete, explicitly provisional MANO display pose.

The owner supplies a reference ``context`` (session, clear generation,
calibration, model/config, algorithm and functional switch). Changing it, or
moving source time backwards, discards all cached local rotations. A cached
degraded angle remains degraded history: it never enters an estimator or
becomes an observation. Keeping that last displayed angle avoids a reverse
jump to an older valid angle when the measurement becomes unavailable.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self._context = None
        self._stamp = None
        self._slots = [None] * 16

    @staticmethod
    def _array(value):
        array = np.asarray(value, dtype=float)
        if array.shape != (16, 3) or not np.isfinite(array).all():
            raise ValueError("display rotations must be finite (16, 3)")
        return array.copy()

    def update(self, pose, *, context, now):
        """Return new display metadata/rotations, or None (keep the old image).

No new source time is invented. Rendering may finish after the measurement
deadline; the UI must show that age instead of calling the result live. The
output is deliberately not a valid measurement pose and must not be passed to
the ROS exporter or written over a source pose in ``poses.jsonl``.
        """
        if context != self._context:
            self.reset()
            self._context = deepcopy(context)
        if not isinstance(pose, dict) or not pose.get("pose_available") or not pose.get("solver_success"):
            return None
        stamp = pose.get("timestamp_s")
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in (stamp, now)) or stamp > now:
            return None
        if self._stamp is not None and stamp < self._stamp:
            self.reset()
            self._context = deepcopy(context)
        try:
            mano = self._array(pose.get("mano_pose_rotvec"))
            angles = self._array(pose.get("anatomical_euler_rad"))
            valid, available, quality = joint_quality_state(pose)
        except (ValueError, TypeError, OverflowError):
            return None
        # A current root must belong to this exact computed pose. Do not mix a
        # raw IMU root from the independent input monitor into calibrated MANO.
        if not valid[0] or not available[0]:
            return None

        new_sample = self._stamp is None or stamp > self._stamp
        next_slots = list(self._slots)
        held, unknown, provisional = [], [], []
        source_stamps, ages, origins = {}, {}, {}
        for slot in range(16):
            key = str(slot)
            if available[slot]:
                # Store displayable estimates, not only valid estimates: when
                # the next frame is absent, freeze what was actually shown.
                if new_sample or next_slots[slot] is None:
                    next_slots[slot] = (mano[slot].copy(), angles[slot].copy(), stamp, quality[slot])
                source_stamps[key] = stamp
                if not valid[slot]:
                    provisional.append(slot)
            elif next_slots[slot] is not None:
                local, anatomical, previous_stamp, previous_quality = next_slots[slot]
                mano[slot], angles[slot] = local, anatomical
                source_stamps[key] = previous_stamp
                origins[key] = previous_quality
                held.append(slot)
            else:
                # No historical pose exists for this joint. A MANO-neutral
                # visual outline is explicitly unknown, never observed zero.
                mano[slot] = 0.
                angles[slot] = 0.
                source_stamps[key] = None
                unknown.append(slot)
            ages[key] = None if source_stamps[key] is None else max(0., now - source_stamps[key])

        self._slots, self._stamp = next_slots, stamp
        result = {key: deepcopy(pose[key]) for key in
                  ("session_id", "frame_index", "profile_version", "config_sha256",
                   "model_rest_sha256", "shape_params", "root_quat_xyzw",
                   "source_monotonic_ns", "source_reference_node") if key in pose}
        result.update(display_only=True, measurement_valid=False,
                      timestamp_s=stamp, source_timestamp_s=stamp,
                      held_slots=held, unknown_slots=unknown, provisional_slots=provisional,
                      source_timestamp_by_slot=source_stamps, source_age_by_slot=ages,
                      held_source_quality_by_slot=origins,
                      mano_pose_rotvec=mano.tolist(), anatomical_euler_rad=angles.tolist(),
                      source_joint_valid_mask=valid, source_joint_quality=quality,
                      history_semantics="last_displayed_local_rotation_not_new_measurement",
                      unknown_semantics="neutral_mano_outline_not_observed_extension")
        return result
