"""Pinhole frustum in optical coordinates (x right, y down, z forward)."""

import math


FRUSTUM_EDGES = ((0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7))


def frustum_vertices(intrinsics=None, *, width=None, height=None, fx=None, fy=None,
                     cx=None, cy=None, near_m=.05, far_m=.5,
                     distortion_model='none', distortion=()):
    """Return near TL/TR/BR/BL then far TL/TR/BR/BL image-edge corners.

Distorted input must be rectified explicitly by its owner. Zero-coefficient
plumb_bob/rational models reduce to pinhole and are supported.
"""
    if intrinsics is not None:
        width, height = intrinsics['width'], intrinsics['height']
        fx, fy, cx, cy = (intrinsics[name] for name in ('fx', 'fy', 'cx', 'cy'))
        distortion_model = intrinsics.get('distortion_model', distortion_model)
        distortion = intrinsics.get('distortion', intrinsics.get('D', intrinsics.get('d', distortion)))
    if type(width) is not int or type(height) is not int or not 0 < width <= 65536 or not 0 < height <= 65536:
        raise ValueError('image dimensions must be positive bounded integers')
    values = (fx, fy, cx, cy, near_m, far_m)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('intrinsics and distances must be finite')
    if fx <= 0 or fy <= 0 or not 0 < near_m < far_m:
        raise ValueError('positive focal lengths and 0 < near < far are required')
    if distortion_model not in ('none', '', 'plumb_bob', 'rational_polynomial'):
        raise ValueError('unsupported distortion model')
    if any(isinstance(v, bool) or not math.isfinite(v) or v != 0 for v in distortion):
        raise ValueError('frustum requires zero distortion or rectified intrinsics')
    corners = ((0, 0), (width, 0), (width, height), (0, height))
    return tuple(((u-cx)/fx*z, (v-cy)/fy*z, z) for z in (near_m, far_m) for u, v in corners)
