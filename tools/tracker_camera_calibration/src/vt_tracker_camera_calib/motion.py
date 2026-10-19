"""Shared geometric excitation diagnostics for online guidance and solving."""
from __future__ import annotations
import math
import cv2
import numpy as np

def motion_coverage(pairs) -> dict:
    """The same geometric thresholds as the existing hand-eye excitation gate."""
    if not pairs:
        return {'translation_span_m': 0., 'rotation_span_deg': 0., 'rotation_axis_score': 0.}
    reference = pairs[0].world_from_tracker
    translation, rotation, axes = 0., 0., []
    for pair in pairs[1:]:
        vector, _ = cv2.Rodrigues(reference.rotation.T @ pair.world_from_tracker.rotation)
        angle = float(np.linalg.norm(vector))
        translation = max(translation, float(np.linalg.norm(pair.world_from_tracker.translation - reference.translation)))
        rotation = max(rotation, math.degrees(angle))
        if angle >= math.radians(5):
            axes.append(vector.reshape(3) / angle)
    singular = np.linalg.svd(np.stack(axes), compute_uv=False) if len(axes) >= 2 else []
    return {'translation_span_m': translation, 'rotation_span_deg': rotation,
            'rotation_axis_score': float(singular[1]) if len(singular) > 1 else 0.}




_EXCITATION_REQUIREMENTS = (
    ('rotation_span_deg', 15., 'INCREASE_ROTATION',
     'pose set needs at least 15 degrees of rotational span'),
    ('translation_span_m', .05, 'INCREASE_TRANSLATION',
     'pose set needs at least 50 mm of translational span'),
    ('rotation_axis_score', .15, 'ROTATE_SECOND_AXIS',
     'pose set must rotate around at least two non-collinear axes'),
)


def excitation_failure(coverage: dict) -> tuple[str, str] | None:
    """Return the same actionable reason for online guidance and solver checks."""
    for key, minimum, hint, message in _EXCITATION_REQUIREMENTS:
        if coverage[key] < minimum:
            return hint, message
    return None
