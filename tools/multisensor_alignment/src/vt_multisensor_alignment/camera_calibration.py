"""Bounded observations of recorded camera calibration, independent of world poses.

Values remain in their message conventions: CameraInfo matrices are row-major;
RealSense Extrinsics rotation is column-major and translation is in metres.
This validates numeric structure, not physical accuracy or image association.
"""
from __future__ import annotations

import math
from numbers import Integral, Real


CAMERA_INFO_TYPE = "sensor_msgs/msg/CameraInfo"
EXTRINSICS_TYPE = "realsense2_camera_msgs/msg/Extrinsics"
CALIBRATION_TYPES = frozenset({CAMERA_INFO_TYPE, EXTRINSICS_TYPE})
_DISTORTION_LENGTHS = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}


def _vector(value, length, name):
    if len(value) != length:
        raise ValueError(f"{name} must have {length} values")
    if any(isinstance(item, bool) or not isinstance(item, Real) or not math.isfinite(item)
           for item in value):
        raise ValueError(f"{name} must contain finite numbers")
    return [float(item) for item in value]


def _unsigned(value, name, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, Integral)
            or not (int(positive) <= value <= 0xffffffff)):
        raise ValueError(f"{name} must be a {'positive' if positive else 'nonnegative'} uint32")
    return int(value)


def _rotation(values):
    # Orthogonality and determinant are layout-invariant. The source layout is
    # preserved in the report; no implicit transpose or direction inversion.
    rows = [values[i:i + 3] for i in (0, 3, 6)]
    for row in range(3):
        for column in range(3):
            dot = sum(rows[row][i] * rows[column][i] for i in range(3))
            if not math.isclose(dot, float(row == column), abs_tol=1e-4, rel_tol=0.):
                raise ValueError("rotation is not orthonormal within 1e-4")
    a, b, c = rows
    determinant = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (
        b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
    if not math.isclose(determinant, 1., abs_tol=1e-4, rel_tol=0.):
        raise ValueError("rotation determinant must be +1 within 1e-4")


def _intrinsics(message):
    frame = message.header.frame_id
    if not isinstance(frame, str) or not frame.strip() or len(frame) > 512 or any(ord(c) < 32 for c in frame):
        raise ValueError("CameraInfo frame_id must be a nonempty bounded frame")
    width = _unsigned(message.width, "width", positive=True)
    height = _unsigned(message.height, "height", positive=True)
    model = message.distortion_model
    if not isinstance(model, str) or len(model) > 128:
        raise ValueError("distortion_model must be a bounded string")
    if len(message.d) > 64:
        raise ValueError("distortion coefficient count exceeds 64")
    d = _vector(message.d, len(message.d), "D")
    k, r, p = (_vector(getattr(message, field), size, field.upper())
               for field, size in (("k", 9), ("r", 9), ("p", 12)))
    if min(k[0], k[4], p[0], p[5]) <= 0:
        raise ValueError("K and P focal lengths must be positive (zero K means uncalibrated)")
    # CameraInfo defines standard homogeneous rows. Permit only numeric
    # roundoff here; this checks representation, not physical calibration.
    for name, row, expected in (("K", k[6:], (0., 0., 1.)),
                                ("P", p[8:], (0., 0., 1., 0.))):
        if any(not math.isclose(value, target, abs_tol=1e-9, rel_tol=0.)
               for value, target in zip(row, expected)):
            raise ValueError(f"{name} homogeneous row must be {expected} within 1e-9")
    _rotation(r)
    supported = model in _DISTORTION_LENGTHS
    if supported and len(d) != _DISTORTION_LENGTHS[model]:
        raise ValueError(f"{model} requires {_DISTORTION_LENGTHS[model]} distortion coefficients")
    return {
        "frame_id": frame, "width": width, "height": height,
        "distortion_model": model, "d": d, "k": k, "r": r, "p": p,
        "matrix_layout": "row_major", "projection_supported": supported,
        "binning_x": _unsigned(message.binning_x, "binning_x"),
        "binning_y": _unsigned(message.binning_y, "binning_y"),
        "roi": {**{field: _unsigned(getattr(message.roi, field), f"roi.{field}")
                   for field in ("x_offset", "y_offset", "height", "width")},
                "do_rectify": bool(message.roi.do_rectify)},
    }


def _extrinsics(message):
    rotation = _vector(message.rotation, 9, "rotation")
    translation = _vector(message.translation, 3, "translation")
    _rotation(rotation)
    return {"rotation": rotation, "rotation_layout": "column_major",
            "translation_m": translation,
            "transform_convention": "p_target = R * p_source + t; source and target are encoded by the topic"}


class CalibrationQualityAccumulator:
    """Inspect every sample while retaining only the first and previous valid one."""

    def __init__(self, type_name):
        if type_name not in CALIBRATION_TYPES:
            raise ValueError(f"unsupported calibration type: {type_name}")
        self.kind = "intrinsics" if type_name == CAMERA_INFO_TYPE else "extrinsics"
        self.parse = _intrinsics if self.kind == "intrinsics" else _extrinsics
        self.messages = self.decoded = self.malformed = self.valid = self.invalid = 0
        self.unsupported = self.changes = 0
        self.first_decode_error = self.first_invalid_reason = None
        self.first = self.previous = None

    def decode_failure(self, receipt_ns, error):
        self.messages += 1
        self.malformed += 1
        if self.first_decode_error is None:
            self.first_decode_error = f"{type(error).__name__}: {error}"[:400]

    def add(self, message, receipt_ns):
        self.messages += 1
        self.decoded += 1
        try:
            sample = self.parse(message)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self.invalid += 1
            if self.first_invalid_reason is None:
                self.first_invalid_reason = str(error)[:400]
            return
        self.valid += 1
        self.unsupported += sample.get("projection_supported") is False
        if self.first is None:
            self.first = sample
        elif self.previous != sample:
            self.changes += 1
        self.previous = sample

    def document(self):
        return {
            "inspection": "calibration_messages", "calibration_kind": self.kind,
            "observed_messages": self.messages, "decoded_messages": self.decoded,
            "malformed_messages": self.malformed, "first_decode_error": self.first_decode_error,
            "valid_calibration_messages": self.valid, "invalid_calibration_messages": self.invalid,
            "unsupported_distortion_messages": self.unsupported, "calibration_change_events": self.changes,
            "first_invalid_reason": self.first_invalid_reason, "first_valid_calibration": self.first,
            "inspection_limitations": "Numeric structure and changes only; physical accuracy, image-to-calibration association and world/Tracker transforms are not certified. Unsupported distortion models are preserved, not converted.",
        }
