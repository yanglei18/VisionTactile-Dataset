"""Head-camera measurements of up to two identical, 30 mm tag36h11 cubes.

The origin is the centre of the cube. +X is right when facing the front,
+Y goes from front to back, +Z from bottom to top. Printed tags are assumed
centred on the orthogonal faces. The portable template includes the top and
front from the reference photo. A verified layout supplies the other faces.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from itertools import combinations
import json
import os
import tempfile
from math import cos, sin
from pathlib import Path

import cv2
import numpy as np


# For each face, the printed coordinate axes point right and down as seen
# from the outside. On top, printed "up" is toward the back; on bottom it is
# toward the front. On the four vertical faces printed "up" is cube +Z.
_FACE_AXES = {
    "top": ((1, 0, 0), (0, -1, 0)),
    "bottom": ((1, 0, 0), (0, 1, 0)),
    "front": ((1, 0, 0), (0, 0, -1)),
    "right": ((0, 1, 0), (0, 0, -1)),
    "back": ((-1, 0, 0), (0, 0, -1)),
    "left": ((0, -1, 0), (0, 0, -1)),
}


@dataclass(frozen=True)
class CubeLayout:
    # Rows use decoded tag ID, face name, and index of the canonical first
    # AprilTag corner in that face's physical TL,TR,BR,BL order.
    faces: tuple[tuple[int, str, int], ...]
    family = 'tag36h11'
    cube_edge_m = 0.030
    tag_edge_m = 0.024

    def __post_init__(self):
        if not 1 <= len(self.faces) <= 6:
            raise ValueError("A cube layout requires one to six known faces")
        used_ids, used_faces = set(), set()
        for tag, face, corner in self.faces:
            if type(tag) is not int or not 0 <= tag <= 5 or tag in used_ids:
                raise ValueError("Expected unique tag IDs in 0..5")
            if not isinstance(face, str) or face not in _FACE_AXES or face in used_faces:
                raise ValueError("Expected unique cube face names")
            if type(corner) is not int or not 0 <= corner <= 3:
                raise ValueError("corner0 must be an integer in 0..3")
            used_ids.add(tag)
            used_faces.add(face)

    @classmethod
    def default(cls):
        return cls(((2, "top", 2), (5, "front", 3)))

    @classmethod
    def from_file(cls, path):
        document = json.loads(Path(path).read_text(encoding="utf-8"))
        if (not isinstance(document, dict) or set(document) != {'family', 'cube_edge_m', 'tag_edge_m', 'faces'}
                or document['family'] != cls.family or document['cube_edge_m'] != cls.cube_edge_m
                or document['tag_edge_m'] != cls.tag_edge_m or not isinstance(document['faces'], list)):
            raise ValueError("Expected a tag36h11 30 mm cube layout with 24 mm markers")
        rows = []
        for row in document['faces']:
            if not isinstance(row, dict) or set(row) != {'id', 'face', 'corner0'}:
                raise ValueError("Each face requires id, face and corner0")
            rows.append((row['id'], row['face'], row['corner0']))
        return cls(tuple(rows))

    def save(self, path, *, replace=False):
        """Commit a complete JSON file; the CLI creates, the save dialog can replace."""
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(self.as_dict(), sort_keys=True, indent=2, allow_nan=False) + "\n"
        stream = tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=target.parent, delete=False)
        temporary = Path(stream.name)
        try:
            with stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            if replace:
                os.replace(temporary, target)
            else:
                os.link(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)

    @property
    def sha256(self):
        """Bind output to physical tag corners in metres, including their winding."""
        geometry = {str(k): v.round(12).tolist() for k, v in sorted(self.object_points().items())}
        document = dict(family=self.family, cube_edge_m=self.cube_edge_m,
                        tag_edge_m=self.tag_edge_m, corners_m=geometry)
        return hashlib.sha256(json.dumps(document, separators=(',', ':'), sort_keys=True).encode()).hexdigest()

    def as_dict(self):
        return dict(family=self.family, cube_edge_m=self.cube_edge_m, tag_edge_m=self.tag_edge_m,
                    faces=[dict(id=tag, face=face, corner0=corner)
                           for tag, face, corner in sorted(self.faces)])

    @classmethod
    def face_corner_options(cls):
        return {(face, corner): cls.face_corners(face, corner)
                for face in _FACE_AXES for corner in range(4)}

    @classmethod
    def face_corners(cls, face, corner):
        right, down = np.asarray(_FACE_AXES[face], dtype=np.float64)
        centre = np.cross(down, right) * (cls.cube_edge_m / 2)
        offsets = np.array([-right - down, right - down, right + down, -right + down])
        points = centre + offsets * (cls.tag_edge_m / 2)
        return np.roll(points, -corner, axis=0)

    def object_points(self):
        return {tag: self.face_corners(face, corner) for tag, face, corner in self.faces}



@dataclass(frozen=True)
class CubeObservation:
    center_px: tuple[float, float]
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    tag_ids: tuple[int, ...]
    reprojection_px: float


@dataclass(frozen=True)
class _Fit:
    indices: frozenset[int]
    observation: CubeObservation


def _measure(indices, points, pixels, tag_ids, rvec, tvec, camera, distortion):
    """Check all face corners, positive depth and visible exterior normals."""
    rvec, tvec = rvec.reshape(3), tvec.reshape(3)
    rotation = cv2.Rodrigues(rvec)[0]
    corners = points @ rotation.T + tvec
    if not np.isfinite(corners).all() or np.min(corners[:, 2]) <= 0:
        return None
    groups = corners.reshape(-1, 4, 3)
    # Detected corners wind clockwise as seen from outside the printed face.
    inward = np.cross(groups[:, 1] - groups[:, 0], groups[:, 2] - groups[:, 0])
    if np.any((inward * groups.mean(axis=1)).sum(axis=1) <= 0):
        return None
    predicted = cv2.projectPoints(points, rvec, tvec, camera, distortion)[0].reshape(-1, 2)
    residual_sq = np.square(predicted-pixels).sum(axis=1).reshape(-1, 4)
    pixel_groups = pixels.reshape(-1, 4, 2)
    edge = np.linalg.norm(pixel_groups-np.roll(pixel_groups, 1, axis=1), axis=2).mean(axis=1)
    if np.any(residual_sq.mean(axis=1)**.5 > np.clip(.025 * edge, 1.25, 5.)):
        return None
    angle = np.linalg.norm(rvec)
    axis = rvec * (sin(angle/2) / angle if angle else .5)
    centre = cv2.projectPoints(np.zeros((1, 3)), rvec, tvec, camera, distortion)[0].reshape(2)
    obs = CubeObservation(tuple(centre), tuple(tvec), (*axis, cos(angle/2)),
                          tuple(sorted(tag_ids)), float(residual_sq.mean()**.5))
    return _Fit(frozenset(indices), obs)


def _single_face(index, points, pixels, tag, camera, distortion):
    # IPPE_SQUARE requires a centred, canonical square, regardless of which
    # face it belongs to. Convert each returned pose back to the cube frame.
    h = CubeLayout.tag_edge_m / 2
    local = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    x = (points[1]-points[0]) / (2*h)
    y = (points[0]-points[3]) / (2*h)
    cube_from_tag = np.column_stack((x, y, np.cross(x, y)))
    count, rotations, translations, _ = cv2.solvePnPGeneric(
        local, pixels, camera, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    fits = []
    for i in range(count):
        camera_from_cube = cv2.Rodrigues(rotations[i])[0] @ cube_from_tag.T
        tvec = translations[i].reshape(3) - camera_from_cube @ points.mean(axis=0)
        value = _measure((index,), points, pixels, (tag,), cv2.Rodrigues(camera_from_cube)[0],
                         tvec, camera, distortion)
        if value:
            fits.append(value)
    fits.sort(key=lambda f: f.observation.reprojection_px)
    if len(fits) == 2:
        a, b = (f.observation for f in fits)
        similarity = abs(np.dot(a.quaternion_xyzw, b.quaternion_xyzw))
        if similarity < np.cos(np.deg2rad(5.) / 2) and b.reprojection_px - a.reprojection_px < .65:
            return None  # Plausible, distinct planar solutions.
    return fits[0] if fits else None


def _choose_fits(fits):
    """Assign disjoint face sightings, retain only unambiguous cube groups."""
    alternatives = []
    for size in (1, 2):
        for partition in combinations(fits, size):
            if size == 2:
                a, b = partition
                if a.indices & b.indices:
                    continue
                # Two solid cubes cannot have nearly coincident centres.
                if np.linalg.norm(np.subtract(a.observation.translation_m, b.observation.translation_m)) < CubeLayout.cube_edge_m:
                    continue
            coverage = sum(len(f.indices) for f in partition)
            # At equal coverage two agreeing faces are stronger than two
            # isolated, independently fitted planar tags.
            score = (coverage, coverage-size)
            error = sum(f.observation.reprojection_px*len(f.indices) for f in partition)/coverage
            alternatives.append((score, error, partition))
    if not alternatives:
        return []
    score = max(item[0] for item in alternatives)
    plausible = [item for item in alternatives if item[0] == score]
    best = min(plausible, key=lambda item: item[1])
    common = {fit.indices for fit in best[2]}
    for _, error, group in plausible:
        if error <= best[1] + .65:
            common.intersection_update(fit.indices for fit in group)
    return [fit.observation for fit in best[2] if fit.indices in common]


class CubeDetector:
    def __init__(self, layout: CubeLayout):
        self.layout = layout
        self._points = layout.object_points()
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        parameters = (aruco.DetectorParameters_create() if hasattr(aruco, 'DetectorParameters_create')
                      else aruco.DetectorParameters())
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        if hasattr(aruco, 'ArucoDetector'):
            self._detect = aruco.ArucoDetector(dictionary, parameters).detectMarkers
        else:
            self._detect = lambda image: aruco.detectMarkers(image, dictionary, parameters=parameters)

    def read_corners(self, image_bgr_or_gray):
        """Decode all rig IDs; preserve separate sightings when IDs repeat."""
        image = np.asarray(image_bgr_or_gray)
        if image.dtype != np.uint8 or (image.ndim != 2 and not (image.ndim == 3 and image.shape[2] == 3)):
            raise ValueError("Expected an 8-bit grayscale or BGR frame")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return []
        return [(int(tag), np.asarray(points, dtype=np.float64).reshape(4, 2))
                for tag, points in zip(ids.ravel(), corners) if 0 <= tag <= 5]

    def detect(self, image_bgr_or_gray, K, D) -> list[CubeObservation]:
        return self.from_corners(self.read_corners(image_bgr_or_gray), K, D)

    def from_corners(self, sightings, K, D) -> list[CubeObservation]:
        camera = np.asarray(K, dtype=np.float64)
        distortion = np.asarray(D, dtype=np.float64).reshape(-1) if D is not None else np.empty(0)
        if (camera.shape != (3, 3) or not np.isfinite(camera).all() or min(camera[0,0], camera[1,1]) <= 0
                or len(distortion) not in (0, 4, 5, 8, 12, 14) or not np.isfinite(distortion).all()):
            raise ValueError("Invalid camera matrix or distortion coefficients")
        tags = []
        pixels = []
        for tag, sighting in sightings:
            if tag in self._points:
                tags.append(int(tag))
                pixels.append(np.asarray(sighting, dtype=np.float64).reshape(4, 2))
        if any(tags.count(tag) > 2 for tag in set(tags)):
            return []  # The rig contains at most two physical copies of a tag.
        fits = []
        centres = [pts.mean(axis=0) for pts in pixels]
        spans = [float(np.linalg.norm(pts-np.roll(pts, 1, axis=0), axis=1).max()) for pts in pixels]
        # For a convex cube at most three faces can be visible at once. Never
        # index observations by ID alone: both hands carry the same six IDs.
        for size in range(1, min(3, len(tags))+1):
            for indices in combinations(range(len(tags)), size):
                members = tuple(tags[i] for i in indices)
                if len(set(members)) != size:
                    continue
                if any(np.linalg.norm(centres[i]-centres[j]) > 2 * max(spans[i], spans[j])
                       for i, j in combinations(indices, 2)):
                    continue
                points = np.concatenate([self._points[t] for t in members])
                corners2d = np.concatenate([pixels[i] for i in indices])
                if size == 1:
                    fit = _single_face(indices[0], points, corners2d, members[0], camera, distortion)
                else:
                    ok, rvec, tvec = cv2.solvePnP(points, corners2d, camera, distortion, flags=cv2.SOLVEPNP_SQPNP)
                    if not ok:
                        continue
                    rvec, tvec = cv2.solvePnPRefineLM(points, corners2d, camera, distortion, rvec, tvec)
                    fit = _measure(indices, points, corners2d, members, rvec, tvec, camera, distortion)
                if fit:
                    fits.append(fit)
        return _choose_fits(fits)
