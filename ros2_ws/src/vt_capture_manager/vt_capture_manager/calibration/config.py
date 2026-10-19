"""Build the existing offline contract from configured devices and measured board."""
from __future__ import annotations

import math
import re

from vt_capture_core.calibration_policy import acceptance_defaults, board_detection_defaults, pairing_defaults

_NAME = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')


def _number(value, low, high, label):
    if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{label} must be within [{low}, {high}]')
    return value


def build_config(camera: dict, tracker_role: str, board: dict, *, tracker_frame_id: str | None = None,
                 world_frame: str = 'vive_map') -> dict:
    """Return schema 1; caller supplies authoritative camera serial/model binding."""
    name = camera.get('name', camera.get('id'))
    if not isinstance(name, str) or not _NAME.fullmatch(name) or len(name) > 128:
        raise ValueError('invalid camera name')
    if tracker_role not in ('torso', 'left_wrist', 'right_wrist'):
        raise ValueError('unsupported Tracker role')
    for key in ('serial', 'model'):
        if not isinstance(camera.get(key), str) or not camera[key] or len(camera[key]) > 256:
            raise ValueError(f'camera {key} is required')
    for key in ('squares_x', 'squares_y'):
        if type(board.get(key)) is not int:
            raise ValueError(f'{key} must be an integer')
        _number(board[key], 3, 100, key)
    square = _number(board.get('square_length_mm'), .1, 1000, 'square_length_mm') / 1000
    marker = _number(board.get('marker_length_mm'), .1, 1000, 'marker_length_mm') / 1000
    if marker >= square:
        raise ValueError('marker length must be smaller than square length')
    dictionary = board.get('dictionary')
    if not isinstance(dictionary, str) or not re.fullmatch(r'DICT_(?:[4-7]X[4-7]_(?:50|100|250|1000)|ARUCO_ORIGINAL)', dictionary):
        raise ValueError('unsupported ChArUco dictionary')
    if (board['squares_x'] - 1) * (board['squares_y'] - 1) < 12:
        raise ValueError('board must support at least 12 interior corners')
    return dict(schema_version=1, camera=dict(name=name, serial=camera['serial'], model=camera['model'],
        frame_id=camera.get('color_frame_id', camera.get('frame_id', f'{name}_color_optical_frame')),
        image_topic=camera.get('color_topic', f'/{name}/color/image_raw'),
        camera_info_topic=camera.get('camera_info_topic', f'/{name}/color/camera_info'),
        timing_topic=camera.get('timing_topic', f'/{name}/frame_timing')),
        tracker=dict(role=tracker_role, frame_id=tracker_frame_id or f'vive_tracker_{tracker_role}',
                     sample_topic=f'/vive/{tracker_role}/sample'),
        frames=dict(world=world_frame, board='charuco_board'),
        board=dict(squares_x=board['squares_x'], squares_y=board['squares_y'], square_length_m=square,
                   marker_length_m=marker, dictionary=dictionary, **board_detection_defaults()),
        pairing=pairing_defaults(), acceptance=acceptance_defaults())
