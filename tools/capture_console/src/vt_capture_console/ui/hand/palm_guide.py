"""Read-only palm inclination during official mounting calibration.

A flat hold determines the palm normal in the mounted IMU; it cannot
determine which way the fingers point. An accepted fingertips-up hold fixes
that remaining direction. Gravity never supplies an absolute heading or
individual finger angles. The UI must label the rigid hand accordingly.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import digest
from vt_fsglove12.live_display import display_basis
from vt_fsglove12.palm_inclination import observed_up


@dataclass(frozen=True)
class PalmPreview:
    state: str
    angle_deg: float | None = None
    pitch_deg: float | None = None
    geometry: dict | None = None
    key: tuple | None = None


class PalmGuide:
    def __init__(self, resources):
        self.resources = resources
        config = resources.get("config") or {}
        self.node = next((n for n, slot in config.get("node_to_slot", {}).items() if slot == 0), None)
        self.hand = resources.get("hand", "right")
        self.config_hash = digest(config)
        self.stale_s = config.get("stale_after_s", .15)
        self._last = None

    def update(self, snapshot, *, now_ns):
        """Read the input mailbox; no clocks, serial access, captures or writes."""
        runtime, cal = snapshot.get("runtime") or {}, snapshot.get("calibration") or {}
        input_state = snapshot.get("input_snapshot") or {}
        live, frame = input_state.get("runtime") or {}, input_state.get("orientation") or {}
        session, generation = runtime.get("session_id"), cal.get("clear_generation")
        node = (frame.get("nodes") or {}).get(self.node) or {}
        stamp = node.get("host_monotonic_ns")
        if (not self.node or self.resources["config"].get("hand") != self.hand
                or not runtime.get("connected") or not live.get("connected") or live.get("input_snapshot_stale")
                or not session or session != live.get("session_id") or session != frame.get("session_id")
                or frame.get("config_sha256") != self.config_hash
                or generation is None or live.get("clear_generation") != generation
                or cal.get("profile_status") == "clearing" or runtime.get("clear_pending")
                or type(stamp) is not int or not 0 < stamp <= now_ns <= stamp + self.stale_s * 1e9):
            return PalmPreview("unavailable")
        current = observed_up(node, self.stale_s)
        if current is None:
            return PalmPreview("unavailable")

        def reference(step):
            ref = (cal.get("palm_tilt_references") or {}).get(step) or {}
            if (ref.get("node"), ref.get("epoch"), ref.get("session_id"), ref.get("clear_generation"),
                    ref.get("config_sha256"), ref.get("hand")) != (
                    self.node, node["epoch"], session, generation, self.config_hash, self.hand):
                return None
            return ref

        flat, lifted = reference("neutral"), reference("fingers_down")
        if flat is None:
            return PalmPreview("await_neutral")
        z = np.asarray(flat["up"])
        angle = float(np.rad2deg(np.arccos(np.clip(current @ z, -1., 1.))))
        if lifted is None:
            return PalmPreview("inclination", angle)
        key = ("measured_palm_tilt", session, generation, node["epoch"], stamp,
               flat["capture_id"], lifted["capture_id"])
        if self._last is not None and self._last.key == key:
            return self._last
        # In a labelled fingertips-up hold, world up has a positive
        # component along the fingers in the palm's own coordinate frame.
        up = np.asarray(lifted["up"])
        y = up - (up @ z) * z
        if np.linalg.norm(y) < np.sin(np.deg2rad(12.)):
            return PalmPreview("inclination", angle)
        y /= np.linalg.norm(y)
        local_up = np.array([current @ np.cross(y, z), current @ y, current @ z])
        # Shortest tilt that maps the measured local up vector to world up.
        # At an exact inversion gravity alone cannot choose a rotation axis.
        if local_up[2] < -.999:
            return PalmPreview("inclination", angle)
        rotation = Rotation.from_quat([local_up[1], -local_up[0], 0., 1. + local_up[2]])
        axes = self.resources.get("model_axes") or {}
        neutral = self.resources.get("neutral_geometry") or {}
        if (not {"roll_axis", "pitch_axis"} <= axes.keys() or not {"joints_m", "vertices_m"} <= neutral.keys()
                or axes.get("hand", "right") != self.hand
                or neutral.get("provenance", {}).get("hand", "right") != self.hand):
            return PalmPreview("inclination", angle)
        basis = display_basis(axes)
        model = basis.T @ rotation.as_matrix() @ basis
        wrist = np.asarray(neutral["joints_m"][0])
        geometry = dict(neutral, display_only=True, palm_tilt_only=True)
        for field in ("vertices_m", "joints_m"):
            geometry[field] = (np.asarray(neutral[field]) - wrist) @ model.T + wrist
        pitch = float(np.rad2deg(np.arcsin(np.clip(local_up[1], -1., 1.))))
        self._last = PalmPreview("tilt", angle, pitch, geometry, key)
        return self._last
