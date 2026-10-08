"""Conservative root-local hand reconstruction from paired segment IMUs.

The four fingers have a measured proximal and middle segment; their final
segment is optional DIP model output, never a third IMU measurement.  The thumb
has only two phalanges, with explicitly provisional anatomical interpretation.
There is no world-position or wrist-translation estimate in this model.
"""

from __future__ import annotations

import math

import numpy as np

from .math3d import conj, from_axis_angle, mul, normalize, rotate


_FORWARD = np.array([0.0, 1.0, 0.0])
_FLEX_AXIS = np.array([-1.0, 0.0, 0.0])
_DEFAULT_BASES = {
    "thumb": [38.0, 16.0, 0.0], "index": [25.0, 65.0, 0.0],
    "middle": [7.0, 70.0, 0.0], "ring": [-12.0, 66.0, 0.0],
    "little": [-29.0, 56.0, 0.0],
}


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _node(value):
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError("node mappings must be nonnegative integer IDs or null")
    return int(value)


class HandModel:
    """A small geometric model, independent of serial, VQF, and Qt.

    ``base_mm`` and ``neutral_splay_deg`` describe a canonical right-hand
    template.  Setting ``hand='left'`` mirrors final x coordinates, including
    the palm; calibrated angle conventions remain the same on either hand.
    Input quaternions must already describe calibrated segment axes in a common
    canonical right-hand orientation frame.  The engine reflects real left-hand
    rotations into that frame before relative-angle extraction; merely mirroring
    the final geometry is not sufficient for real left-hand sensor inputs.
    Neutral calibration alone does not establish these
    axes; ``calibrated_axes`` explicitly carries that separate assertion.

    This is the historical numeric template, not an anatomically certified
    mesh. The canvas uses explicit config metadata to correct its mirrored
    palm/finger-base layout without changing angles or physical root attitude.
    """

    def __init__(self, config: dict):
        self.palm_node = _node(config.get("palm_node", 12))
        self.hand = config.get("hand", "right")
        if self.hand not in ("right", "left"):
            raise ValueError("hand must be 'right' or 'left'")
        self.dip_ratio = float(config.get("dip_ratio", 0.65))
        if not math.isfinite(self.dip_ratio) or not 0.0 <= self.dip_ratio <= 1.0:
            raise ValueError("dip_ratio must be finite and between 0 and 1")
        self.estimate_dip = bool(config.get("estimate_dip", True))
        self.estimate_dip &= config.get("dip_mode", "estimated") != "disabled"
        self.estimate_dip &= self.dip_ratio > 0.0
        self.fingers = {}
        for name, raw in config.get("fingers", {}).items():
            thumb = name == "thumb"
            lengths = np.asarray(raw.get("lengths_mm", [35.0, 25.0] if thumb else [45.0, 25.0, 18.0]), dtype=float)
            count = 2 if thumb else 3
            if lengths.shape != (count,) or not np.all(np.isfinite(lengths)) or np.any(lengths <= 0):
                raise ValueError(f"{name}: lengths_mm must contain {count} finite positive lengths")
            base = np.asarray(raw.get("base_mm", _DEFAULT_BASES.get(name, [0.0, 60.0, 0.0])), dtype=float)
            if base.shape != (3,) or not np.all(np.isfinite(base)):
                raise ValueError(f"{name}: base_mm must contain three finite coordinates")
            splay = float(raw.get("neutral_splay_deg", -50.0 if thumb else 0.0))
            if not math.isfinite(splay):
                raise ValueError(f"{name}: neutral_splay_deg must be finite")
            self.fingers[name] = {
                "proximal": _node(raw.get("proximal")),
                "distal": _node(raw.get("distal")),
                "lengths": lengths, "base": base, "neutral_splay": splay,
                "anatomy_verified": bool(raw.get("anatomy_verified", False)),
            }

    def _mirror(self, points: np.ndarray) -> list:
        result = np.array(points, dtype=float, copy=True)
        if self.hand == "left":
            result[:, 0] *= -1.0
        return result.tolist()

    def _points(self, finger: dict, mcp: float, pip: float, splay: float, dip: float) -> list:
        q = from_axis_angle([0, 0, 1], math.radians(finger["neutral_splay"] + splay))
        q = mul(q, from_axis_angle(_FLEX_AXIS, math.radians(mcp)))
        points = [finger["base"].copy()]
        bends = [0.0, pip, dip]
        for index, length in enumerate(finger["lengths"]):
            if index:
                q = mul(q, from_axis_angle(_FLEX_AXIS, math.radians(bends[index])))
            points.append(points[-1] + rotate(q, _FORWARD) * length)
        return self._mirror(points)

    def _palm_points(self) -> list:
        # Closed outline: wrist -> radial edge -> knuckles -> ulnar wrist.
        bases = [self.fingers[name]["base"] for name in ("thumb", "index", "middle", "ring", "little") if name in self.fingers]
        if not bases:
            bases = [np.array([35.0, 60.0, 0.0]), np.array([-35.0, 60.0, 0.0])]
        points = [[30.0, 0.0, 0.0], *bases, [-30.0, 0.0, 0.0], [30.0, 0.0, 0.0]]
        return self._mirror(points)

    def solve(self, orientations: dict[int, np.ndarray], valid_nodes: set[int],
              calibrated_axes: set[int], neutral_ready: bool, *, thumb_flexion=None) -> dict:
        """Return JSON-safe angles, local geometry, and a separate root attitude.

        Missing/stale/unverified data never reuse the previous frame: their
        complete finger geometry returns to a neutral placeholder.  A fresh
        pair can still yield a valid PIP/IP value while the palm is unavailable,
        but the placeholder remains marked ``valid=False`` / ``geometry_live=False``.
        """
        valid_nodes, calibrated_axes = set(valid_nodes), set(calibrated_axes)
        quats, invalid = {}, set()
        for node, q in orientations.items():
            if node not in valid_nodes:
                continue
            try:
                quats[node] = normalize(q)
            except (TypeError, ValueError, OverflowError):
                invalid.add(node)
        # An uncalibrated but actively receiving sensor has no common-frame
        # quaternion yet.  Do not confuse that absence with a stale source.
        palm_fresh = self.palm_node is not None and self.palm_node in valid_nodes
        palm_available = self.palm_node is not None and self.palm_node in quats
        palm_axes = self.palm_node is not None and self.palm_node in calibrated_axes
        root_valid = bool(neutral_ready and palm_fresh and palm_available and palm_axes)
        root_orientation = quats[self.palm_node].copy() if root_valid else np.array([1., 0., 0., 0.])
        if self.hand == "left":
            # Points below are already physically mirrored.  Undo the engine's
            # canonicalization for the root attitude so the renderer can apply
            # this proper rotation directly to those physical left-hand points.
            root_orientation *= [1., 1., -1., -1.]
        result = {"fingers": {}, "palm_points": self._palm_points(), "warnings": [],
                  "palm_valid": root_valid, "root_orientation_wxyz": root_orientation.tolist(),
                  "root_orientation_valid": root_valid, "root_reference": "session_neutral",
                  "root_translation_mm": [0., 0., 0.], "root_heading_observable": False,
                  "root_fixed": False, "root_translation_fixed": True,
                  "hand": self.hand, "units": "mm"}
        if not neutral_ready:
            result["warnings"].append("Neutral reference is not ready; geometry is a neutral placeholder.")
        if not palm_fresh:
            result["warnings"].append("Palm orientation is unavailable or stale; MCP and splay are unavailable.")
        elif self.palm_node in invalid:
            result["warnings"].append("Palm orientation is invalid; MCP and splay are unavailable.")
        elif not palm_axes:
            result["warnings"].append("Palm segment axes are unverified; MCP and splay are unavailable.")
        elif not palm_available and neutral_ready:
            result["warnings"].append("Calibrated palm orientation is unavailable; MCP and splay are unavailable.")
        for name, finger in self.fingers.items():
            proximal, distal = finger["proximal"], finger["distal"]
            thumb = name == "thumb"
            mapped = proximal is not None and distal is not None
            prox_fresh, dist_fresh = proximal in valid_nodes, distal in valid_nodes
            prox_available, dist_available = proximal in quats, distal in quats
            prox_axes, dist_axes = proximal in calibrated_axes, distal in calibrated_axes
            mcp_valid = bool(neutral_ready and prox_fresh and prox_available and prox_axes and palm_fresh and palm_available and palm_axes)
            pair_valid = bool(neutral_ready and mapped and prox_fresh and dist_fresh and prox_available and dist_available and prox_axes and dist_axes)
            warnings, clipped = [], []
            raw_angles = {"mcp_deg": None, "pip_deg": None, "splay_deg": None, "dip_deg": None, "ip_deg": None}

            def limited(key: str, angle: float, low: float, high: float) -> float:
                raw_angles[key] = float(angle)
                if angle < low - 1e-6 or angle > high + 1e-6:
                    clipped.append(key)
                    warnings.append(f"{key}={angle:.1f} is outside [{low:g}, {high:g}]; displayed geometry is clipped.")
                return float(np.clip(angle, low, high))

            mcp = splay = pip = dip = ip = None
            quaternion_ip = None
            if mcp_valid:
                relative = mul(conj(quats[self.palm_node]), quats[proximal])
                direction = rotate(relative, _FORWARD)
                mcp = limited("mcp_deg", math.degrees(math.atan2(-direction[2], math.hypot(direction[0], direction[1]))), -20.0, 90.0)
                if math.hypot(direction[0], direction[1]) < 1e-7:
                    warnings.append("Splay is undefined when the proximal segment points perpendicular to the palm.")
                    splay_valid = False
                else:
                    deviation = _wrap_degrees(math.degrees(math.atan2(-direction[0], direction[1])) - finger["neutral_splay"])
                    splay = limited("splay_deg", deviation, -60.0 if thumb else -35.0, 60.0 if thumb else 35.0)
                    splay_valid = True
            else:
                splay_valid = False
            if pair_valid and thumb and thumb_flexion is not None:
                # Preserve the old output as a diagnostic, never silently fall
                # back to independently drifting orientations in the new mode.
                relative = normalize(mul(conj(quats[proximal]), quats[distal]))
                if math.hypot(relative[0], relative[1]) >= 1e-8:
                    quaternion_ip = _wrap_degrees(math.degrees(2.0 * math.atan2(-relative[1], relative[0])))
                estimate = thumb_flexion.get("angle_deg")
                # Fresh paired prediction remains visible (with uncertainty),
                # instead of snapping to a fake straight pose each time gravity
                # is temporarily unobservable. Numerical availability is NOT
                # equivalent to a trustworthy anatomical observation.
                pair_valid = type(estimate) in (int, float) and math.isfinite(estimate)
                if pair_valid:
                    # Estimator angles are continuous. Do not wrap +/-180 and
                    # create an artificial +90 to -10 display reversal.
                    ip = limited("ip_deg", estimate, -10.0, 90.0)
                if thumb_flexion.get("uncertain", True) or not pair_valid:
                    warnings.append("Thumb flexion estimate is uncertain: " +
                                    str(thumb_flexion.get("quality", "unavailable")) + "; " +
                                    "; ".join(map(str, thumb_flexion.get("reasons", []))))
            elif pair_valid:
                relative = normalize(mul(conj(quats[proximal]), quats[distal]))
                if math.hypot(relative[0], relative[1]) < 1e-8:
                    pair_valid = False
                    warnings.append("Relative flexion is undefined for this segment orientation.")
                else:
                    bend = _wrap_degrees(math.degrees(2.0 * math.atan2(-relative[1], relative[0])))
                    if thumb:
                        ip = limited("ip_deg", bend, -10.0, 90.0)
                    else:
                        pip = limited("pip_deg", bend, -10.0, 120.0)
            if not thumb and pair_valid and self.estimate_dip:
                dip = limited("dip_deg", self.dip_ratio * max(0.0, pip), 0.0, 80.0)
            valid = bool(mapped and mcp_valid and splay_valid and pair_valid)
            if not neutral_ready:
                quality = "no_neutral"
            elif not mapped:
                quality = "unmapped"
            elif proximal in invalid or distal in invalid or self.palm_node in invalid:
                quality = "invalid_orientation"
            elif not prox_fresh or not dist_fresh:
                quality = "stale"
            elif not prox_axes or not dist_axes:
                quality = "axes_unverified"
            elif not palm_fresh:
                quality = "palm_stale"
            elif not palm_axes:
                quality = "palm_axes_unverified"
            elif not prox_available or not dist_available or not palm_available:
                quality = "orientation_unavailable"
            elif not pair_valid or not splay_valid:
                quality = "singular"
            elif clipped:
                quality = "clipped"
            elif not finger["anatomy_verified"]:
                quality = "anatomy_unverified"
            elif thumb:
                quality = "anatomical_provisional"
            else:
                quality = "ok"
            if not mapped:
                warnings.append("Sensor pair is not mapped.")
            if not prox_axes or not dist_axes:
                warnings.append("Finger segment axes are unverified; neutral calibration does not establish anatomical axes.")
            if not finger["anatomy_verified"]:
                warnings.append("Sensor-to-phalanx mapping is unverified; anatomical labels are provisional.")
            if thumb:
                warnings.append("Thumb MCP/IP/splay labels are provisional; only two phalanges are modeled.")
            points = self._points(finger, mcp, ip if thumb else pip, splay, dip or 0.0) if valid else self._points(finger, 0.0, 0.0, 0.0, 0.0)
            item = {
                "points": points, "mcp_deg": mcp, "pip_deg": pip, "splay_deg": splay,
                "dip_deg": dip, "ip_deg": ip, "valid": valid, "geometry_live": valid,
                "mcp_valid": mcp_valid, "pip_valid": bool(pair_valid and not thumb),
                "ip_valid": bool(pair_valid and thumb), "splay_valid": splay_valid,
                "dip_valid": bool(dip is not None), "dip_measured": False,
                "dip_source": "not_applicable" if thumb else "model_estimate" if self.estimate_dip else "disabled",
                "estimated_segments": [] if thumb else [2],
                "segment_sources": ["imu", "imu"] + ([] if thumb else ["dip_model" if self.estimate_dip else "unmeasured_straight"]),
                "anatomy_verified": finger["anatomy_verified"],
                "anatomical_provisional": bool(thumb or not finger["anatomy_verified"]),
                "quality": quality, "clipped_angles": clipped,
                "raw_angles_deg": raw_angles, "warnings": warnings,
            }
            result["fingers"][name] = item
            if thumb and thumb_flexion is not None:
                item["ip_estimator"] = dict(thumb_flexion)
                item["ip_quaternion_deg"] = quaternion_ip
                item["ip_confident"] = bool(pair_valid and thumb_flexion.get("valid") is True and not thumb_flexion.get("uncertain", True))
                if (quality not in {"stale", "palm_stale", "invalid_orientation", "unmapped", "no_neutral",
                                    "axes_unverified", "palm_axes_unverified", "orientation_unavailable"}
                        and neutral_ready and prox_axes and dist_axes and prox_fresh and dist_fresh
                        and (not pair_valid or thumb_flexion.get("uncertain", True))):
                    item["quality"] = "thumb_" + str(thumb_flexion.get("quality", "unavailable"))
            result["warnings"].extend(f"{name}: {warning}" for warning in warnings)
        return result
