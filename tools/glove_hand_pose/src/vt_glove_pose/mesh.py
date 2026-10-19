"""Original procedural hand solids; no downloaded mesh or anatomical assets.

The surfaces follow the model's measured/inferred skeleton. Their thickness,
nails, and wrist stub are illustrative geometry, not additional measurements.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


SKIN = np.array([231.0, 239.0, 240.0])
UNCERTAIN = np.array([234.0, 232.0, 218.0])
UNCALIBRATED = np.array([178.0, 195.0, 205.0])
STALE = np.array([220.0, 160.0, 166.0])
NAIL = np.array([141.0, 213.0, 218.0])
# Hand-side cues are decorative materials, separate from measurement quality.
# Multiply the current state color rather than replacing it: a stale palm
# remains red and an uncalibrated palm remains blue-grey on either side.
PALMAR_MATERIAL_MULTIPLIER = np.array([0.76, 0.87, 0.87])
UNAVAILABLE_NAIL_MULTIPLIER = 0.70
ESTIMATE_ACCENT = np.array([216.0, 177.0, 106.0])
JOINT_ROOT = np.array([46.0, 139.0, 162.0])
JOINT_INTERMEDIATE = np.array([77.0, 177.0, 185.0])
JOINT_UNAVAILABLE = np.array([125.0, 149.0, 165.0])
JOINT_STALE = np.array([185.0, 99.0, 111.0])
_THETA = np.arange(12) * (2 * math.pi / 12)
_LATITUDE = np.linspace(-math.pi / 2, math.pi / 2, 5)
_SPHERE = np.stack((np.cos(_LATITUDE)[:, None]*np.cos(_THETA)[None, :],
                    np.broadcast_to(np.sin(_LATITUDE)[:, None], (5, len(_THETA))),
                    np.cos(_LATITUDE)[:, None]*np.sin(_THETA)[None, :]), axis=2)


@dataclass(frozen=True)
class HandMesh:
    vertices: np.ndarray
    normals: np.ndarray
    triangles: np.ndarray
    colors: np.ndarray
    estimated: np.ndarray
    parts: tuple[str, ...]
    center: np.ndarray
    radius: float


def _unit(vector, fallback):
    vector = np.asarray(vector, dtype=float)
    length = float(np.linalg.norm(vector))
    return vector / length if length > 1e-8 else np.asarray(fallback, dtype=float)


def orientation_matrix(quaternion) -> np.ndarray:
    """Validated active rotation matrix for physical-frame [w,x,y,z]."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError("root orientation must be a finite, nonzero quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2 * (x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2 * (x*x + y*y)],
    ])


def display_root_matrix(model: dict, follow: bool = True) -> np.ndarray:
    """Missing/invalid root data never cause a held-last or invented attitude."""
    if not follow or not model.get("root_orientation_valid", False):
        return np.eye(3)
    try:
        return orientation_matrix(model.get("root_orientation_wxyz"))
    except (ValueError, TypeError):
        return np.eye(3)


class _Builder:
    def __init__(self):
        self.vertices, self.faces, self.colors = [], [], []
        self.normals = []
        self.estimated, self.parts = [], []

    def surface(self, rings, color, part, estimated=False, *, caps=False,
                reverse_winding=True, normals=None):
        rings = np.asarray(rings, dtype=float)
        if rings.ndim != 3 or rings.shape[2] != 3:
            raise ValueError("surface rings must have shape (rings, points, 3)")
        count, sides, _ = rings.shape
        offset = len(self.vertices)
        self.vertices.extend(rings.reshape(-1, 3).tolist())
        if normals is None:
            tangent = np.roll(rings, -1, axis=1)-np.roll(rings, 1, axis=1)
            axial = np.gradient(rings, axis=0)
            normals = np.cross(tangent, axial)
            if reverse_winding:
                normals = -normals
        normals = np.asarray(normals, dtype=float)
        normals /= np.maximum(np.linalg.norm(normals, axis=2, keepdims=True), 1e-12)
        self.normals.extend(normals.reshape(-1, 3).tolist())
        faces = []
        for row in range(count - 1):
            for side in range(sides):
                other = (side + 1) % sides
                a, b = offset + row*sides + side, offset + row*sides + other
                c, d = a + sides, b + sides
                faces.extend(((a, b, c), (b, d, c)))
        if caps:
            for row, reverse in ((0, True), (count - 1, False)):
                center = len(self.vertices)
                cap_center = rings[row].mean(axis=0)
                self.vertices.append(cap_center.tolist())
                a, b = rings[row, 0]-cap_center, rings[row, 1]-cap_center
                normal = np.cross(b, a) if reverse else np.cross(a, b)
                if reverse_winding:
                    normal = -normal
                self.normals.append(_unit(normal, [0, 0, 1]).tolist())
                for side in range(sides):
                    a, b = offset + row*sides + side, offset + row*sides + (side + 1) % sides
                    faces.append((center, b, a) if reverse else (center, a, b))
        self.faces.extend((a, c, b) for a, b, c in faces) if reverse_winding else self.faces.extend(faces)
        self.colors.extend([np.asarray(color).tolist()] * len(faces))
        self.estimated.extend([bool(estimated)] * len(faces))
        self.parts.extend([part] * len(faces))

    def ellipsoid(self, center, basis, radii, color, part, estimated=False, *, band_color=None):
        # Five latitude rings / twelve meridians with smooth analytic normals.
        basis = np.asarray(basis)
        rings = np.asarray(center) + (_SPHERE*radii) @ basis.T
        normals = (_SPHERE/radii) @ basis.T
        first_face = len(self.faces)
        self.surface(rings, color, part, estimated, normals=normals)
        if band_color is not None:
            # Recolor the existing equatorial latitude bands. No intersecting
            # torus/extra shell or additional triangles are introduced.
            faces_per_band = len(_THETA)*2
            start = first_face+faces_per_band
            stop = first_face+3*faces_per_band
            self.colors[start:stop] = [np.asarray(band_color).tolist()]*(stop-start)

    def tube(self, start, end, width, dorsal, radius_a, radius_b, color, part, estimated=False,
             *, start_color=None, end_color=None, start_length=0.0, end_length=0.0):
        cross_section = np.outer(np.cos(_THETA), width) + np.outer(np.sin(_THETA), dorsal * 0.85)
        length = max(float(np.linalg.norm(np.asarray(end)-start)), 1e-8)
        # Keep material boundaries outside the overlapping knuckle surfaces.
        # Both adjacent tubes and that knuckle share a color inside the joint;
        # only the tube's complete latitude ring changes back to the body.
        fractions = np.array([0.0, min(0.42, max(0.22, start_length/length)),
                              max(0.58, min(0.72, 1-end_length/length)), 1.0])
        radii = (radius_a*(1-fractions)+radius_b*fractions)*[1, 1.04, 1.01, 1]
        centers = np.asarray(start)[None, :]*(1-fractions[:, None])+np.asarray(end)[None, :]*fractions[:, None]
        rings = centers[:, None, :]+cross_section[None, :, :]*radii[:, None, None]
        first_face = len(self.faces)
        self.surface(rings, color, part, estimated)
        faces_per_band = len(_THETA)*2
        for index, band_color in ((0, start_color), (2, end_color)):
            if band_color is not None:
                first = first_face+index*faces_per_band
                self.colors[first:first+faces_per_band] = [np.asarray(band_color).tolist()]*faces_per_band


def _color(finger, stale):
    quality = str(finger.get("quality", ""))
    if stale or "stale" in quality:
        return STALE
    if not finger.get("valid", False):
        return UNCALIBRATED
    if (quality in {"clipped", "anatomy_unverified", "anatomical_provisional", "singular"}
            or finger.get("ip_confident") is False or bool(finger.get("clipped_angles"))):
        return UNCERTAIN
    return SKIN


def _joint_color(name, segment, finger, stale):
    """Joint-position markers do not assert an independently measured joint."""
    quality = str(finger.get("quality", ""))
    if stale or "stale" in quality:
        return JOINT_STALE
    if not finger.get("valid", False):
        return JOINT_UNAVAILABLE
    if quality == "clipped" or bool(finger.get("clipped_angles")) or finger.get("ip_confident") is False:
        return ESTIMATE_ACCENT
    if name != "thumb" and segment == 2:
        inferred = finger.get("dip_source") == "model_estimate" and bool(finger.get("dip_valid", False))
        return ESTIMATE_ACCENT if inferred else JOINT_UNAVAILABLE
    return JOINT_ROOT if segment == 0 else JOINT_INTERMEDIATE


def _rounded_outline(points):
    points = np.asarray(points, dtype=float)
    if len(points) > 1 and np.linalg.norm(points[-1] - points[0]) < 1e-8:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("palm outline requires at least three points")
    # Two Chaikin corner-cutting passes create a rounded convex silhouette.
    for _ in range(2):
        following = np.roll(points, -1, axis=0)
        points = np.stack((0.75*points + 0.25*following,
                           0.25*points + 0.75*following), axis=1).reshape(-1, 3)
    return points


def build_hand_mesh(model: dict, *, stale: bool = False) -> HandMesh:
    """Create a tapered volumetric hand without changing its kinematic points."""
    builder = _Builder()
    palm_points = model.get("palm_points", [])
    fingers = model.get("fingers", {})
    if palm_points:
        palm_array = np.asarray(palm_points, dtype=float)
        if palm_array.ndim != 2 or palm_array.shape[1] != 3 or not np.all(np.isfinite(palm_array)):
            raise ValueError("palm geometry must be finite xyz points")
        outline = _rounded_outline(palm_array)
        center = (palm_array.min(axis=0) + palm_array.max(axis=0)) * 0.5
        center[2] = 0.0
        palm_color = STALE if stale else SKIN if model.get("palm_valid", False) else UNCALIBRATED
        thumb_points = fingers.get("thumb", {}).get("points", [])
        thumb_side = -1.0 if thumb_points and thumb_points[0][0] < 0 else 1.0
        rings = []
        # Dorsal dome, rounded edge and gently concave palmar thickness.
        for radius, height in ((0.015, 8.0), (0.50, 7.3), (0.92, 4.3),
                               (1.0, 0.0), (0.94, -4.5), (0.50, -6.5), (0.015, -6.0)):
            ring = center + (outline-center)*radius
            ring[:, 2] = height
            if height < -0.1:
                # Model the thenar fullness within the same palm surface;
                # intersecting decorative shells create painter-order shards.
                thenar = np.exp(-((ring[:, 0]-thumb_side*18)/12)**2-((ring[:, 1]-25)/20)**2)
                ring[:, 2] -= thenar*1.1
            rings.append(ring)
        signed_area = np.sum(outline[:, 0]*np.roll(outline[:, 1], -1)-outline[:, 1]*np.roll(outline[:, 0], -1))
        first_palm_face = len(builder.faces)
        builder.surface(rings, palm_color, "palm", caps=True, reverse_winding=signed_area > 0)
        # The complete -Z half has a distinct material, including its cap.
        # Reuse the existing surface, avoiding extra layers, painter-order
        # artifacts or geometry that might look like measured palm creases.
        palmar_color = (palm_color * PALMAR_MATERIAL_MULTIPLIER).tolist()
        faces_per_band = 2 * len(outline)
        for row in range(len(rings)-1):
            if (rings[row][:, 2].mean()+rings[row+1][:, 2].mean()) < 0:
                start = first_palm_face + row * faces_per_band
                builder.colors[start:start+faces_per_band] = [palmar_color] * faces_per_band
        bottom_cap = first_palm_face + (len(rings)-1) * faces_per_band + len(outline)
        builder.colors[bottom_cap:bottom_cap+len(outline)] = [palmar_color] * len(outline)
        # The wrist stub is decorative hand-attached geometry, not a measured
        # forearm or an independently moving wrist joint.
        theta = np.arange(16) * (2 * math.pi / 16)
        wrist_rings = [np.column_stack((np.cos(theta)*width, np.full(16, y), np.sin(theta)*depth))
                       for y, width, depth in ((8, 27, 6.5), (-7, 25.5, 7.0), (-17, 24, 6.8), (-21, 22, 5.0), (-22, 20, 3.0))]
        builder.surface(wrist_rings, palm_color * 0.99, "wrist_stub", caps=True, reverse_winding=False)
    else:
        center = np.array([0.0, 40.0, 0.0])
    radius_bound = 90.0
    radii = {"thumb": 8.3, "index": 7.3, "middle": 7.7, "ring": 7.2, "little": 6.1}
    for name, finger in fingers.items():
        points = np.asarray(finger.get("points", []), dtype=float)
        if points.size == 0:
            continue
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.all(np.isfinite(points)):
            raise ValueError(f"{name}: finger geometry must be finite xyz points")
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        radius_bound = max(radius_bound, float(np.linalg.norm(points[0] - center) + lengths.sum() + 13))
        base_radius = radii.get(name, 7.0)
        color = _color(finger, stale)
        expected = set(finger.get("estimated_segments", []))
        first_direction = points[1] - points[0]
        horizontal = first_direction.copy()
        horizontal[2] = 0
        fallback = [0.8 if points[0, 0] >= 0 else -0.8, 0.6, 0] if name == "thumb" else [0, 1, 0]
        horizontal = _unit(horizontal, fallback)
        width_hint = _unit(np.cross(horizontal, [0, 0, 1]), [1, 0, 0])
        for segment, (start, end) in enumerate(zip(points, points[1:])):
            axis = _unit(end-start, [0, 1, 0])
            width = _unit(width_hint - np.dot(width_hint, axis)*axis, [1, 0, 0])
            dorsal = _unit(np.cross(width, axis), [0, 0, 1])
            basis = np.column_stack((width, axis, dorsal))
            r_a = base_radius * (1.0 - 0.14*segment)
            r_b = base_radius * (0.86 - 0.14*segment)
            estimated = segment in expected
            segment_color = color
            if estimated and finger.get("valid") and not stale and "stale" not in str(finger.get("quality", "")):
                segment_color = color*0.62 + UNCERTAIN*0.38
            part = f"{name}:{segment}"
            final = segment == len(points)-2
            tube_end = end-axis*r_b*0.70 if final else end
            joint_color = _joint_color(name, segment, finger, stale)
            next_color = None if final else _joint_color(name, segment+1, finger, stale)
            builder.tube(start, tube_end, width, dorsal, r_a*0.95, r_b, segment_color, part, estimated,
                         start_color=joint_color, end_color=next_color,
                         start_length=r_a*0.88+1.0, end_length=0 if final else r_b*0.88+1.0)
            joint_inferred = name != "thumb" and segment == 2 and finger.get("dip_source") == "model_estimate" and bool(finger.get("dip_valid", False))
            builder.ellipsoid(start, basis, [r_a*1.09, r_a*0.88, r_a*0.99], joint_color,
                              f"{name}:knuckle", joint_inferred)
            if final:
                tip_center = end-axis*r_b*0.70
                builder.ellipsoid(tip_center, basis, [r_b, r_b*0.70, r_b*0.85], segment_color, f"{name}:tip", estimated)
                # A low-domed oval nail follows the distal bone's dorsal side.
                nail_center = end-axis*min(lengths[segment]*0.36, r_b*1.4) + dorsal*r_b*0.79
                nail_available = finger.get("valid") and not stale and "stale" not in str(finger.get("quality", ""))
                # A darker same-state nail stays visible even before axes are
                # calibrated or after disconnection; never imply valid data.
                nail_color = NAIL if nail_available else color*UNAVAILABLE_NAIL_MULTIPLIER
                if nail_available and (estimated or finger.get("ip_confident") is False
                                       or bool(finger.get("clipped_angles"))
                                       or str(finger.get("quality", "")) in {"clipped", "anatomy_unverified", "anatomical_provisional"}):
                    nail_color = nail_color*0.85 + ESTIMATE_ACCENT*0.15
                builder.ellipsoid(nail_center, basis, [r_b*0.65, min(lengths[segment]*0.34, r_b*0.98), 0.75],
                                  nail_color, f"{name}:nail", estimated)
    vertices = np.asarray(builder.vertices, dtype=float).reshape(-1, 3)
    triangles = np.asarray(builder.faces, dtype=np.int32).reshape(-1, 3)
    return HandMesh(vertices, np.asarray(builder.normals, dtype=float).reshape(-1, 3), triangles,
                    np.clip(np.asarray(builder.colors, dtype=float).reshape(-1, 3), 0, 255),
                    np.asarray(builder.estimated, dtype=bool), tuple(builder.parts), center, radius_bound)
