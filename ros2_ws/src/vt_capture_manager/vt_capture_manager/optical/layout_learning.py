"""Recover the face names and printed quarter-turns of one 30 mm cube.

The two known faces anchor the right-handed frame. An unknown tag has only
four possible rotations on each of the remaining faces. Compare their printed
corners with the image, using a pose estimated exclusively from other tags.
Discovery and independent validation consume separate, diverse source views.
"""
from __future__ import annotations


import cv2
import numpy as np

from .cube import CubeDetector, CubeLayout

_ALL_FACES = CubeLayout.face_corner_options()


def _outside_previous_views(direction, views):
    """View direction is in cube coordinates; stationary/new image IDs do not count."""
    return all(direction @ saved < np.cos(np.deg2rad(8.)) for saved in views)


def _select_face(observation, measured, camera, distortion):
    """Return a unique corner layout and view direction in cube coordinates."""
    q = np.asarray(observation.quaternion_xyzw)
    length = np.linalg.norm(q[:3])
    rvec = q[:3] * (2 * np.arctan2(length, q[3]) / length if length else 0.)
    rotation = cv2.Rodrigues(rvec)[0]
    tvec = np.asarray(observation.translation_m)
    eye = -rotation.T @ tvec
    direction = eye / np.linalg.norm(eye)
    errors = []
    for key, corners in _ALL_FACES.items():
        normal = corners.mean(axis=0)  # exterior normal times half width
        if normal @ (eye-normal) <= 0:
            continue
        pred = cv2.projectPoints(corners, rvec, tvec, camera, distortion)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.square(pred-measured).sum(axis=1).mean()))
        errors.append((rms, key))
    errors.sort()
    span = np.linalg.norm(measured-np.roll(measured, 1, axis=0), axis=1).mean()
    if (len(errors) < 2 or errors[0][0] > np.clip(.04 * span, 1.5, 5.)
            or errors[1][0] - errors[0][0] < max(2., .08 * span)):
        return None
    return errors[0][1], direction


class CubeLayoutLearner:
    """Non-blocking single-frame capture. Keep just one cube in the image."""
    def __init__(self, initial=None):
        self.layout = initial or CubeLayout.default()
        self._detector = CubeDetector(self.layout)
        self._known = {tag: (face, corner) for tag, face, corner in self.layout.faces}
        if len(self._known) < 2:
            raise ValueError("Learning requires two known reference tags")
        self._discovery = {}
        self._checks = {}
        self._status = 'show_adjacent_tags'
        self._conflict = False

    @property
    def complete(self):
        return not self._conflict and sum(len(x) >= 2 for x in self._checks.values()) == 6

    def snapshot(self):
        validated = sorted(tag for tag, views in self._checks.items() if len(views) >= 2)
        return dict(known_ids=sorted(self._known), validated_ids=validated, complete=self.complete,
                    provisional_faces=len(self._known), verified_faces=len(validated), status=self._status)

    def update(self, image_bgr_or_gray, K, D):
        if not self.complete and not self._conflict:
            self._observe(self._detector.read_corners(image_bgr_or_gray), np.asarray(K, dtype=np.float64),
                          np.asarray(D, dtype=np.float64).reshape(-1) if D is not None else None)
        return self.snapshot()

    def _observe(self, sightings, camera, distortion):
        visible = dict(sightings)
        if len(visible) != len(sightings) or len(visible) > 3:
            self._status = 'multiple_cubes'
            return
        if len(visible) < 2:
            self._status = 'show_adjacent_tags'
            return
        verifying = len(self._known) == 6
        targets = sorted(visible if verifying else visible.keys()-self._known.keys())
        self._status = 'verify_from_another_angle' if verifying else 'show_unknown_with_known'
        proposed = []
        for tag in targets:
            # Leave the target OUT of the pose even during validation.
            anchors = [(other, pts) for other, pts in sightings if other != tag and other in self._known]
            if not anchors:
                continue
            estimates = self._detector.from_corners(anchors, camera, distortion)
            if len(estimates) != 1 or len(estimates[0].tag_ids) != len(anchors):
                continue
            match = _select_face(estimates[0], visible[tag], camera, distortion)
            if match is not None:
                proposed.append((tag, *match))
        # All hypotheses use only faces that were known before this frame.
        for tag, key, direction in proposed:
            if (tag in self._known and self._known[tag] != key
                    or any(t != tag and value[0] == key[0] for t, value in self._known.items())):
                self._status = 'layout_conflict'
                self._conflict = True
                return
            if verifying:
                # A hold-out recording of an already seen static viewpoint is
                # not independent geometry evidence for newly learned tags.
                training = self._discovery.get(tag, (None, []))[1]
                checks = self._checks.setdefault(tag, [])
                if len(checks) < 2 and _outside_previous_views(direction, training + checks):
                    checks.append(direction)
            else:
                previous_key, views = self._discovery.get(tag, (key, []))
                if key != previous_key:
                    # Neither a single misread nor alternating hypotheses can
                    # establish a face binding. Restart this tag's votes only.
                    views = []
                if _outside_previous_views(direction, views):
                    views.append(direction)
                self._discovery[tag] = (key, views)
                if len(views) >= 3:
                    self._known[tag] = key
                    self.layout = CubeLayout(tuple((t, f, c) for t, (f, c) in sorted(self._known.items())))
                    self._detector = CubeDetector(self.layout)
                self._status = 'rotate_for_more_views'
        if len(self._known) == 6:
            self._status = 'complete' if self.complete else 'verify_from_another_angle'

    def save(self, path):
        """Publish only an independently verified map, with atomic replacement."""
        if not self.complete:
            raise ValueError("All six faces must pass independent validation before saving")
        self.layout.save(path, replace=True)
