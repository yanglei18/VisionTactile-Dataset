"""Bounded VTPC frame: prefix, JSON header, planar xyz float32 then RGBA8.

The prefix is little-endian ``4s, uint32 version, uint32 header_bytes,
uint32 payload_bytes``. Validate it before allocating/reading the body.
"""

from collections import namedtuple
import json
import math
import struct

try:
    import numpy as _numpy
except ImportError:  # The transport remains usable by standard-library-only clients.
    _numpy = None

from .scene_protocol import (_bounded_json, _generation, _integer, _keys,
                             _load_json, _string, _transform, SceneProtocolError)

PROTOCOL = 'vt.cloud'
VERSION = 1
MAGIC = b'VTPC'
PREFIX = struct.Struct('<4sIII')
PREFIX_BYTES = PREFIX.size
MAX_HEADER_BYTES = 16 * 1024
MAX_POINTS = 2_097_152  # Full 1280x720 / 1920x1080 frames, still bounded.
MAX_FRAME_BYTES = PREFIX_BYTES + MAX_HEADER_BYTES + MAX_POINTS * 16
CloudFrame = namedtuple('CloudFrame', ['header', 'xyz_bytes', 'rgba_bytes'])


class CloudProtocolError(ValueError):
    pass


_REQUIRED = {'protocol', 'version', 'camera_id', 'frame_key', 'source_realtime_ns',
             'time_epoch', 'coordinate_epoch', 'geometry_revision', 'geometry_sha256',
             'point_count', 'world_from_color', 'worker_instance_id', 'cloud_sequence'}
_OPTIONAL = {'manager_instance_id', 'scene_epoch', 'sync_epoch', 'color_frame_id',
             'time_residual_ns', 'color_mode', 'dropped_frames', 'sampling', 'origin',
             'matching_mode', 'tracker_pose'}


def validate_cloud_header(header):
    try:
        _bounded_json(header)
        _keys(header, _REQUIRED | _OPTIONAL, _REQUIRED, 'cloud header')
        if header['protocol'] != PROTOCOL or type(header['version']) is not int or header['version'] != VERSION:
            raise CloudProtocolError('unsupported cloud protocol/version')
        for key in ('camera_id', 'worker_instance_id'):
            _string(header[key], key)
        for key in ('source_realtime_ns', 'cloud_sequence', 'point_count'):
            _integer(header[key], key)
        if header['point_count'] > MAX_POINTS:
            raise CloudProtocolError('point count exceeds limit')
        for key in ('time_epoch', 'coordinate_epoch', 'geometry_revision'):
            _generation(header[key], key)
        for key in ('scene_epoch', 'sync_epoch'):
            if key in header:
                _generation(header[key], key)
        digest = header['geometry_sha256']
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdefABCDEF' for c in digest):
            raise CloudProtocolError('geometry_sha256 must be a SHA-256 hex digest')
        frame_key = header['frame_key']
        if isinstance(frame_key, str):
            _string(frame_key, 'frame_key', 1024)
        elif not isinstance(frame_key, dict) or not frame_key:
            raise CloudProtocolError('frame_key must be nonempty')
        _transform(header['world_from_color'], 'world_from_color')
        if header.get('matching_mode','synchronized') not in ('synchronized','independent'):
            raise CloudProtocolError('invalid cloud matching_mode')
        if header.get('matching_mode') == 'independent':
            pose=header.get('tracker_pose')
            fields={'role','tracker_id','world_from_tracker','before_ns','after_ns','gap_ns'}
            _keys(pose, fields, fields, 'tracker_pose')
            _string(pose['role'],'tracker role'); _string(pose['tracker_id'],'tracker_id')
            _transform(pose['world_from_tracker'],'world_from_tracker')
            for key in ('before_ns','after_ns','gap_ns'): _integer(pose[key],key)
            if not (pose['before_ns']<=header['source_realtime_ns']<=pose['after_ns']
                    and pose['after_ns']-pose['before_ns']==pose['gap_ns']):
                raise CloudProtocolError('tracker pose does not bracket cloud source time')
            if (not isinstance(frame_key,dict) or frame_key.get('scene_time_ns')!=header['source_realtime_ns']
                    or frame_key.get('reference_camera')!=header['camera_id']):
                raise CloudProtocolError('independent cloud requires its own source time')
        elif 'tracker_pose' in header:
            raise CloudProtocolError('tracker_pose requires independent matching')
        if 'origin' in header:
            origin = header['origin']
            _keys(origin, {'source','topic','native_frame_id','input_points','invalid_points'},
                  {'source','topic','native_frame_id','input_points','invalid_points'}, 'cloud origin')
            if origin['source'] != 'realsense' or 'sampling' in header:
                raise CloudProtocolError('invalid driver cloud origin')
            _string(origin['topic'], 'cloud source topic')
            _string(origin['native_frame_id'], 'cloud native frame')
            for key in ('input_points','invalid_points'):
                _integer(origin[key], key)
            if (origin['input_points'] > MAX_POINTS or
                    origin['input_points'] - origin['invalid_points'] != header['point_count']):
                raise CloudProtocolError('driver cloud counts disagree')
        if 'sampling' in header:
            s = header['sampling']
            if not isinstance(s, dict) or set(s) != {'width','height','stride','min_depth_m','max_depth_m'}:
                raise CloudProtocolError('invalid cloud sampling description')
            for key, limit in (('width',4096),('height',4096),('stride',4096)):
                if type(s[key]) is not int or not 1 <= s[key] <= limit:
                    raise CloudProtocolError('invalid cloud sampling dimensions')
            if not all(type(s[k]) in (int,float) and 0 < s[k] <= 100 for k in ('min_depth_m','max_depth_m')) or s['min_depth_m'] >= s['max_depth_m']:
                raise CloudProtocolError('invalid cloud sampling range')
        return header
    except SceneProtocolError as exc:
        raise CloudProtocolError(str(exc)) from exc


def inspect_cloud_prefix(prefix) -> tuple[int, int]:
    _validate_byte_buffer(prefix)
    if len(prefix) != PREFIX_BYTES:
        raise CloudProtocolError('truncated cloud prefix')
    magic, version, header_length, payload_length = PREFIX.unpack(prefix)
    if magic != MAGIC or version != VERSION:
        raise CloudProtocolError('invalid cloud magic/version')
    if not 1 <= header_length <= MAX_HEADER_BYTES or payload_length > MAX_POINTS*16 or payload_length % 16:
        raise CloudProtocolError('cloud length exceeds limit or point layout')
    if PREFIX_BYTES + header_length + payload_length > MAX_FRAME_BYTES:
        raise CloudProtocolError('cloud frame exceeds size limit')
    return header_length, payload_length


def _validate_byte_buffer(value):
    """Only flat contiguous byte buffers have unambiguous protocol lengths."""
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise CloudProtocolError('cloud blocks must be bytes')
    if isinstance(value, memoryview):
        try:
            flat_bytes = value.ndim == 1 and value.itemsize == 1 and value.c_contiguous
        except ValueError as exc:
            raise CloudProtocolError('cloud buffer is unavailable') from exc
        if not flat_bytes:
            raise CloudProtocolError('cloud buffers must be contiguous one-dimensional bytes')


def _validate_xyz(xyz):
    # The wire format is always little-endian, including on big-endian hosts.
    # NumPy views the existing bounded buffer; it does not copy coordinates or
    # turn each float into a Python object. Keep the dependency optional because
    # vt-capture-core is also installed independently of the visualization stack.
    finite = (bool(_numpy.isfinite(_numpy.frombuffer(xyz, dtype='<f4')).all())
              if _numpy is not None else
              all(math.isfinite(value[0]) for value in struct.iter_unpack('<f', xyz)))
    if not finite:
        raise CloudProtocolError('cloud coordinates must be finite')


def pack_cloud_frame(header, xyz_bytes, rgba_bytes) -> bytes:
    if not isinstance(header, dict):
        raise CloudProtocolError('cloud header must be an object')
    header = dict(header)
    header.setdefault('protocol', PROTOCOL)
    header.setdefault('version', VERSION)
    validate_cloud_header(header)
    _validate_byte_buffer(xyz_bytes)
    _validate_byte_buffer(rgba_bytes)
    count = header['point_count']
    if len(xyz_bytes) != count*12 or len(rgba_bytes) != count*4:
        raise CloudProtocolError('point count does not match data lengths')
    _validate_xyz(xyz_bytes)
    try:
        encoded = json.dumps(header, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8')
    except (TypeError, ValueError, UnicodeError) as exc:
        raise CloudProtocolError(str(exc)) from exc
    prefix = PREFIX.pack(MAGIC, VERSION, len(encoded), 16*count)
    inspect_cloud_prefix(prefix)
    # Allocate the multi-MiB frame once, not a growing XYZ-sized intermediate.
    return b''.join((prefix, encoded, xyz_bytes, rgba_bytes))


def unpack_cloud_frame(frame, *, copy_payload=True) -> CloudFrame:
    _validate_byte_buffer(frame)
    if len(frame) > MAX_FRAME_BYTES:
        raise CloudProtocolError('invalid or oversized cloud frame')
    header_length, payload_length = inspect_cloud_prefix(frame[:PREFIX_BYTES])
    if len(frame) != PREFIX_BYTES + header_length + payload_length:
        raise CloudProtocolError('cloud frame length mismatch')
    start = PREFIX_BYTES+header_length
    try:
        header = _load_json(bytes(frame[PREFIX_BYTES:start]), MAX_HEADER_BYTES)
    except SceneProtocolError as exc:
        raise CloudProtocolError(str(exc)) from exc
    validate_cloud_header(header)
    count = header['point_count']
    if count*16 != payload_length:
        raise CloudProtocolError('cloud payload differs from point count')
    if copy_payload:
        xyz = bytes(frame[start:start+count*12])
        rgba = bytes(frame[start+count*12:])
    else:
        # Preview readers own an immutable wire buffer. Keep bounded views of
        # it instead of copying the full point array again. Mutable caller
        # buffers are frozen first; a read-only view alone does not freeze its
        # bytearray owner. Public/default decoding still returns bytes.
        owner = frame if isinstance(frame, bytes) else bytes(frame)
        view = memoryview(owner)
        xyz, rgba = view[start:start+count*12], view[start+count*12:]
    _validate_xyz(xyz)
    return CloudFrame(header, xyz, rgba)
