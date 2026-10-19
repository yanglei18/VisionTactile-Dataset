"""Candidate tactile addresses bound once to a neutral MANO surface.

NumPy and pure layout parsing only: no ROS, Qt, serial, pose solver, model loading
or pressure inference. Left-hand binding requires an explicitly selected layout.
The 20x20 addresses are physically valid, NOT 400 verified skin coordinates.
Bindings and color supports never follow world-space nearest neighbours after
the hand moves, so bringing two fingers together cannot exchange their colors.
"""
from __future__ import annotations

import hashlib
import math

import numpy as np


_FINGERS = (("thumb", 1), ("index", 5), ("middle", 9),
            ("ring", 13), ("little", 17))
_NAMES = ("palm", "thumb", "index", "middle", "ring", "little")


def _array(value, shape, name):
    try:
        result = np.array(value, dtype=np.float64, copy=True)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite array shaped {shape}") from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite array shaped {shape}")
    return result


def _unit(vector, name):
    value = _array(vector, (3,), name)
    length = np.linalg.norm(value)
    if not np.isfinite(length) or length <= 1e-12:
        raise ValueError(f"{name} must have nonzero finite length")
    return value / length


def _readonly(value, dtype=None):
    result = np.array(value, dtype=dtype, copy=True, order="C")
    result.setflags(write=False)
    return result


def _segments(joints):
    # Same anatomical grouping as live_display.neutral_geometry_slot_map,
    # kept local to avoid importing its SciPy/pose presentation dependencies.
    starts, ends, regions = [], [], []
    for region, (_, base) in enumerate(_FINGERS, start=1):
        starts.append(joints[0]); ends.append(joints[base]); regions.append(0)
        for offset in range(3):
            starts.append(joints[base + offset])
            ends.append(joints[base + offset + 1]); regions.append(region)
    return np.asarray(starts), np.asarray(ends), np.asarray(regions, dtype=np.int64)


def _nearest_segments(points, starts, ends):
    directions = ends - starts
    lengths2 = np.einsum("ij,ij->i", directions, directions)
    offsets = points[:, None, :] - starts[None, :, :]
    t = np.clip(np.einsum("nsi,si->ns", offsets, directions) / np.maximum(lengths2, 1e-20), 0., 1.)
    distance2 = np.sum((offsets - t[..., None] * directions[None]) ** 2, axis=2)
    distance2[:, lengths2 <= 1e-20] = np.inf
    return np.argmin(distance2, axis=1)


def _closest_triangle(point, triangles):
    """Exact nearest point over nondegenerate triangles, vectorized per face."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac, ap = b - a, c - a, point - a
    aa = np.einsum("ij,ij->i", ab, ab)
    bb = np.einsum("ij,ij->i", ac, ac)
    mixed = np.einsum("ij,ij->i", ab, ac)
    ra, rb = np.einsum("ij,ij->i", ap, ab), np.einsum("ij,ij->i", ap, ac)
    determinant = aa * bb - mixed * mixed
    v = (bb * ra - mixed * rb) / determinant
    w = (aa * rb - mixed * ra) / determinant
    plane_weights = np.column_stack((1 - v - w, v, w))
    candidates = [np.einsum("ni,nij->nj", plane_weights, triangles)]
    weights = [plane_weights]
    for first, second in ((0, 1), (1, 2), (2, 0)):
        origin, direction = triangles[:, first], triangles[:, second] - triangles[:, first]
        t = np.clip(np.einsum("ij,ij->i", point - origin, direction)
                    / np.einsum("ij,ij->i", direction, direction), 0., 1.)
        edge_weights = np.zeros((len(triangles), 3))
        edge_weights[:, first], edge_weights[:, second] = 1 - t, t
        candidates.append(origin + t[:, None] * direction)
        weights.append(edge_weights)
    distances2 = np.sum((np.stack(candidates, axis=1) - point) ** 2, axis=2)
    distances2[np.any(plane_weights < -1e-10, axis=1), 0] = np.inf
    face, method = np.unravel_index(np.argmin(distances2), distances2.shape)
    barycentric = np.maximum(weights[method][face], 0.)
    barycentric /= barycentric.sum()
    return face, barycentric, float(np.sqrt(distances2[face, method]))


class TactileSurfaceBinding:
    """Fixed face/barycentric candidates in the caller's neutral model basis.

    ``positions`` returns NaN for unsupported addresses; ``valid`` is explicit.
    Color supports only touch vertices of the same neutral finger/palm region
    with palmar normals. Model topology changes require a new binding; a new
    hand shape should also rebuild/review the neutral candidate mapping.
    """

    def __init__(self, neutral_geometry, model_axes, *, hand="right", layout_path=None):
        from vt_tactile_capture.hand_layout import load_layout

        self.layout = load_layout(layout_path, expected_hand=hand)
        self.hand = hand
        try:
            raw_vertices = np.asarray(neutral_geometry["vertices_m"])
            faces = np.asarray(neutral_geometry["faces"])
            joints = _array(neutral_geometry["joints_m"], (21, 3), "joints_m")
            up = _unit(model_axes["palm_up_axis"], "palm_up_axis")
            forward = _unit(model_axes["roll_axis"], "roll_axis")
            thumbward = _unit(model_axes["pitch_axis"], "pitch_axis")
        except (KeyError, TypeError) as exc:
            raise ValueError("neutral geometry and model_axes are required") from exc
        if raw_vertices.ndim != 2 or raw_vertices.shape[1:] != (3,) or not len(raw_vertices):
            raise ValueError("vertices_m must be a nonempty (V, 3) array")
        vertices = _array(raw_vertices, raw_vertices.shape, "vertices_m")
        if (faces.ndim != 2 or faces.shape[1:] != (3,) or faces.dtype.kind not in "iu"
                or (faces.size and (faces.min() < 0 or faces.max() >= len(vertices)))):
            raise ValueError("faces must be integer (F, 3) indices within vertices")
        if abs(np.dot(up, forward)) > .1 or abs(np.dot(up, thumbward)) > .1 or abs(np.dot(forward, thumbward)) > .1:
            raise ValueError("model_axes must be approximately perpendicular")
        provenance = neutral_geometry.get("provenance", {})
        if not isinstance(provenance, dict) or provenance.get("hand", "right") != hand:
            raise ValueError(f"neutral geometry must belong to the {hand} hand; cannot cross-bind hands")
        self._vertices = _readonly(vertices)
        self._faces = _readonly(faces, np.int64)
        self._vertex_count = len(vertices)
        digest = hashlib.sha256()
        digest.update(np.asarray([len(vertices), len(faces)], dtype="<u8").tobytes())
        digest.update(np.asarray(faces, dtype="<u8").tobytes())
        self.topology_id = "sha256:" + digest.hexdigest()
        self.neutral_geometry_id = "sha256:" + hashlib.sha256(np.asarray(vertices, dtype="<f8").tobytes()).hexdigest()
        self.addresses = _readonly([(r, c) for r in range(20) for c in range(20)], np.int64)
        anchors = {(point.row, point.col) for point in self.layout.points if point.verified_anchor}
        self.verified_anchor = _readonly([tuple(address) in anchors for address in self.addresses], bool)
        self._raw_indices = _readonly(self.addresses[:, 0] * 20 + self.addresses[:, 1])
        regions = np.full(400, -1, dtype=np.int64)
        along_fractions = np.full(400, np.nan)
        across_fractions = np.full(400, np.nan)
        for grid in self.layout.regions:
            rows, cols = self.addresses.T
            selected = ((rows >= grid.rows[0]) & (rows < grid.rows[1])
                        & (cols >= grid.cols[0]) & (cols < grid.cols[1]))
            regions[selected] = _NAMES.index(grid.region)
            along = (rows[selected] - grid.rows[0] + .5) / (grid.rows[1] - grid.rows[0])
            across = (cols[selected] - grid.cols[0] + .5) / (grid.cols[1] - grid.cols[0])
            along_fractions[selected] = along if grid.row_direction == 1 else 1 - along
            across_fractions[selected] = across if grid.col_direction == 1 else 1 - across
        self.regions = _readonly(regions)
        self.region_names = tuple(_NAMES[region] if region >= 0 else "unmapped" for region in regions)

        starts, ends, segment_regions = _segments(joints)
        if np.any(np.linalg.norm(ends - starts, axis=1) <= 1e-10):
            raise ValueError("neutral joints must define nondegenerate palm and finger segments")
        directions = ends - starts
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        segment_up = up - (directions @ up)[:, None] * directions
        segment_up[segment_regions == 0] = up
        lengths = np.linalg.norm(segment_up, axis=1)
        if np.any(lengths < .1):
            raise ValueError("neutral finger direction cannot be parallel to the palm normal")
        segment_up /= lengths[:, None]
        nearest_vertex = _nearest_segments(vertices, starts, ends)
        vertex_regions = segment_regions[nearest_vertex]
        triangles = vertices[self._faces]
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        areas2 = np.linalg.norm(normals, axis=1)
        face_normals = np.divide(normals, areas2[:, None], out=np.zeros_like(normals), where=areas2[:, None] > 1e-16)
        vertex_normals = np.zeros_like(vertices)
        for corner in range(3):
            np.add.at(vertex_normals, self._faces[:, corner], normals)
        lengths = np.linalg.norm(vertex_normals, axis=1)
        vertex_normals = np.divide(vertex_normals, lengths[:, None], out=np.zeros_like(vertex_normals), where=lengths[:, None] > 1e-16)
        vertex_palmar = np.einsum("ij,ij->i", vertex_normals, segment_up[nearest_vertex]) < -.1
        self.vertex_regions = _readonly(vertex_regions)
        self.palmar_vertices = _readonly(vertex_palmar)
        nearest_face = _nearest_segments(triangles.mean(axis=1), starts, ends)
        face_regions = segment_regions[nearest_face]
        eligible = ((areas2 > 1e-14)
                    & (np.einsum("ij,ij->i", face_normals, segment_up[nearest_face]) < -.15)
                    & np.all(vertex_palmar[self._faces], axis=1)
                    & np.all(vertex_regions[self._faces] == face_regions[:, None], axis=1))

        face_ids = np.full(400, -1, dtype=np.int64)
        barycentric = np.zeros((400, 3), dtype=np.float64)
        distances = np.full(400, np.nan)
        for region in range(6):
            candidates = np.flatnonzero(eligible & (face_regions == region))
            if not len(candidates):
                continue
            region_vertices = vertices[(vertex_regions == region) & vertex_palmar]
            for index in np.flatnonzero(regions == region):
                if region == 0:
                    offsets = region_vertices - joints[0]
                    progress = offsets @ forward
                    lower, upper = np.percentile(progress, [2, 98])
                    length = upper - lower
                    along = lower + along_fractions[index] * length
                    band = np.abs(progress - along) <= max(length / 6, .002)
                    band_offsets = offsets[band] if np.count_nonzero(band) >= 4 else offsets
                    left, right = np.percentile(band_offsets @ thumbward, [5, 95])
                    across = left + across_fractions[index] * (right - left)
                    target = joints[0] + forward * along + thumbward * across + up * np.median(band_offsets @ up)
                    max_distance = max(.002, length * .3)
                else:
                    _, base = _FINGERS[region - 1]
                    chain = joints[base:base + 4]
                    lengths = np.linalg.norm(np.diff(chain, axis=0), axis=1)
                    progress = along_fractions[index] * lengths.sum()
                    cumulative = np.r_[0., np.cumsum(lengths)]
                    segment = min(int(np.searchsorted(cumulative[1:], progress, side="right")), 2)
                    fraction = (progress - cumulative[segment]) / lengths[segment]
                    center = chain[segment] * (1 - fraction) + chain[segment + 1] * fraction
                    direction = (chain[segment + 1] - chain[segment]) / lengths[segment]
                    local_up = _unit(up - np.dot(up, direction) * direction, "local palmar normal")
                    lateral = _unit(np.cross(direction, local_up), "local lateral")
                    if np.dot(lateral, thumbward) < 0:
                        lateral = -lateral
                    offset = region_vertices - center
                    near = np.abs(offset @ direction) <= max(lengths[segment] * .6, .006)
                    local = offset[near] if np.count_nonzero(near) >= 4 else offset
                    low, high = np.percentile(local @ lateral, [8, 92])
                    across = low + across_fractions[index] * (high - low)
                    target = center + lateral * across + local_up * np.median(local @ local_up)
                    max_distance = max(.003, lengths.sum() * .2)
                local_face, weights, distance = _closest_triangle(target, triangles[candidates])
                if distance <= max_distance:
                    face_ids[index] = candidates[local_face]
                    barycentric[index] = weights
                    distances[index] = distance
        self.face_ids = _readonly(face_ids)
        self.weights = _readonly(barycentric)
        self.valid = _readonly(face_ids >= 0)
        self.projection_distance_m = _readonly(distances)
        vertex_weights = np.zeros((len(vertices), 400), dtype=np.float64)
        indices = np.flatnonzero(self.valid)
        for corner in range(3):
            vertex_weights[self._faces[face_ids[indices], corner], indices] += barycentric[indices, corner]
        self.vertex_weights = _readonly(vertex_weights)
        self._support_vertices, self._support_addresses = np.nonzero(vertex_weights)
        self._support_weights = vertex_weights[self._support_vertices, self._support_addresses]

    def positions(self, current_vertices):
        vertices = _array(current_vertices, (self._vertex_count, 3), "current_vertices")
        positions = np.full((len(self.addresses), 3), np.nan)
        selected = self._faces[self.face_ids[self.valid]]
        positions[self.valid] = np.einsum("ni,nij->nj", self.weights[self.valid], vertices[selected])
        return positions

    def vertex_colors(self, delta, valid_mask, base_colors, limit=3000, deadband=200):
        """Local signed weighted means; neutral/unknown restores original skin.

        Display limits change blending only, never source values. Negative
        changes are blue, positive warm; alpha remains the base surface alpha.
        """
        for name, value in (("limit", limit), ("deadband", deadband)):
            if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if limit <= 0 or deadband < 0 or deadband >= limit:
            raise ValueError("display values must satisfy 0 <= deadband < limit")
        values = np.asarray(delta, dtype=np.float64)
        mask = np.asarray(valid_mask)
        if values.shape != (20, 20) or mask.shape != (20, 20) or mask.dtype.kind != "b":
            raise ValueError("delta and boolean valid_mask must have shape (20, 20)")
        colors = _array(base_colors, (self._vertex_count, 4), "base_colors")
        if np.any((colors < 0) | (colors > 1)):
            raise ValueError("base_colors must be within 0..1")
        values = values.ravel()[self._raw_indices]
        usable = mask.ravel()[self._raw_indices] & self.valid & np.isfinite(values)
        supported = usable[self._support_addresses]
        vertices = self._support_vertices[supported]
        indices = self._support_addresses[supported]
        weights = self._support_weights[supported]
        total = np.bincount(vertices, weights=weights, minlength=self._vertex_count)
        sums = np.bincount(vertices, weights=weights * values[indices], minlength=self._vertex_count)
        means = np.divide(sums, total, out=np.zeros(self._vertex_count), where=total > 1e-15)
        active = (total > 1e-15) & (np.abs(means) > deadband)
        strength = np.clip((np.abs(means[active]) - deadband) / (limit - deadband), 0., 1.)
        blend = (.9 * np.sqrt(strength))[:, None]
        targets = np.where((means[active] > 0)[:, None], [1., .30, .055], [.055, .40, 1.])
        colors[active, :3] = colors[active, :3] * (1 - blend) + targets * blend
        return colors.astype(np.float32)

    def metadata(self):
        return {
            "schema_version": 1, "status": "candidate", "hand": self.hand,
            "rows": 20, "cols": 20, "tactile_state_schema_version": 2,
            "topology_id": self.topology_id, "neutral_geometry_id": self.neutral_geometry_id,
            "address_count": len(self.addresses), "bound_count": int(self.valid.sum()),
            "unbound_count": int((~self.valid).sum()), "observed_anchor_count": int(self.verified_anchor.sum()),
            "verified_skin_coordinates": False, "unit": "ADC difference, not force or pressure",
            "source": dict(self.layout.source), "layout_name": self.layout.name,
            "layout_note": self.layout.note,
            "region_counts": {name: int(np.count_nonzero(self.valid & (self.regions == region)))
                              for region, name in enumerate(_NAMES)},
            "note": "400 physically valid addresses; skin locations remain inferred, not individually calibrated. Fixed neutral face/barycentric binding. The display uses the compact 20x20 state, not the full wire address space.",
        }
