"""Bounded, independently versioned read-only scene JSON/LF protocol.

No control-protocol decoder or command dispatcher is shared with this module.
Entity extension fields remain finite bounded JSON; known geometry/state fields
are validated even when supplied by a newer source adapter.
"""

from collections.abc import Mapping
import json
import math

from .transforms import RigidTransform, finite_vector

PROTOCOL = 'vt.scene'
VERSION = 1
MAX_SCENE_BYTES = 256 * 1024


class SceneProtocolError(ValueError):
    pass


def _bounded_json(value, depth=0):
    if depth > 24:
        raise SceneProtocolError('JSON nesting exceeds limit')
    if value is None or type(value) is bool:
        return
    if type(value) is int:
        if not -(2**63) <= value <= 2**63-1:
            raise SceneProtocolError('integer exceeds 64-bit range')
        return
    if type(value) is float:
        if not math.isfinite(value):
            raise SceneProtocolError('nonfinite number')
        return
    if isinstance(value, str):
        if len(value) > 8192:
            raise SceneProtocolError('string exceeds limit')
        return
    if isinstance(value, (list, tuple)):
        if len(value) > 2048:
            raise SceneProtocolError('array exceeds limit')
        for child in value:
            _bounded_json(child, depth+1)
        return
    if isinstance(value, dict):
        if len(value) > 128:
            raise SceneProtocolError('object exceeds limit')
        for key, child in value.items():
            if not isinstance(key, str) or not key or len(key) > 256:
                raise SceneProtocolError('invalid object key')
            _bounded_json(child, depth+1)
        return
    raise SceneProtocolError('value is not JSON-compatible')


def _string(value, name, limit=256):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise SceneProtocolError(f'{name} must be a nonempty bounded string')


def _integer(value, name, minimum=0):
    if type(value) is not int or not minimum <= value <= 2**63-1:
        raise SceneProtocolError(f'{name} must be an integer >= {minimum}')


def _generation(value, name):
    if type(value) is int:
        _integer(value, name)
    else:
        _string(value, name)


def _keys(value, allowed, required, name):
    if not isinstance(value, dict) or set(value)-set(allowed) or set(required)-set(value):
        raise SceneProtocolError(f'{name} has missing or unknown fields')


def _transform(value, name):
    try:
        RigidTransform.from_dict(value)
    except (TypeError, ValueError) as exc:
        raise SceneProtocolError(f'{name}: {exc}') from exc


def _vector(value, length, name):
    try:
        finite_vector(value, length, name)
    except (TypeError, ValueError) as exc:
        raise SceneProtocolError(str(exc)) from exc


def _entity(value):
    if not isinstance(value, dict):
        raise SceneProtocolError('entity must be an object')
    _string(value.get('id'), 'id')
    if 'label' in value:
        _string(value['label'], 'label')
    for key in ('role', 'camera_id', 'tracker_id'):
        if key in value and value[key] is not None:
            _string(value[key], key)
    for key in ('position_m', 'translation_m'):
        if key in value and value[key] is not None:
            _vector(value[key], 3, key)
    if value.get('quaternion_xyzw') is not None:
        _transform({'translation_m': [0, 0, 0], 'quaternion_xyzw': value['quaternion_xyzw']}, 'quaternion_xyzw')
    for key in ('world_from_color', 'world_from_tracker', 'pose'):
        if value.get(key) is not None:
            _transform(value[key], key)
    if 'state' in value and value['state'] not in ('waiting', 'fresh', 'delayed', 'invalid', 'offline'):
        raise SceneProtocolError('invalid entity state')
    for key in ('valid', 'pose_valid'):
        if key in value and type(value[key]) is not bool:
            raise SceneProtocolError(f'{key} must be boolean')
    if value.get('age_s') is not None:
        age = value['age_s']
        if type(age) not in (int, float) or not math.isfinite(age) or age < 0:
            raise SceneProtocolError('age_s must be nonnegative and finite')
    for key in ('source_realtime_ns', 'before_ns', 'after_ns', 'gap_ns'):
        if value.get(key) is not None:
            _integer(value[key], key)
    vertices = value.get('frustum_vertices')
    if vertices is not None:
        if not isinstance(vertices, (list, tuple)) or len(vertices) != 8:
            raise SceneProtocolError('frustum_vertices must contain 8 points')
        for point in vertices:
            _vector(point, 3, 'frustum vertex')
    trails = value.get('trail_segments')
    if trails is not None:
        if not isinstance(trails, (list, tuple)) or len(trails) > 90:
            raise SceneProtocolError('trail_segments exceeds limit')
        point_count = 0
        for segment in trails:
            if not isinstance(segment, (list, tuple)):
                raise SceneProtocolError('trail segment must be an array')
            point_count += len(segment)
            for point in segment:
                _vector(point, 3, 'trail point')
        if point_count > 512:
            raise SceneProtocolError('trail point count exceeds limit')


_SCENE_FIELDS = {
    'manager_instance_id', 'scene_epoch', 'sequence', 'mode', 'world_frame',
    'time_epoch', 'coordinate_epoch', 'sync_epoch', 'map_identity', 'time_mode',
    'scene_time_ns', 'time_basis', 'config_revision', 'geometry_revision',
    'generated_realtime_ns', 'active_reference_camera', 'display_from_world',
    'trackers', 'cameras', 'diagnostics', 'preview_notice',
}


def validate_scene(value):
    _keys(value, _SCENE_FIELDS, {'manager_instance_id', 'scene_epoch', 'sequence',
                               'mode', 'world_frame', 'trackers', 'cameras', 'diagnostics'}, 'scene')
    for key in ('manager_instance_id', 'world_frame'):
        _string(value[key], key)
    if 'preview_notice' in value:
        notice = value['preview_notice']
        if not isinstance(notice, str) or len(notice) > 1024:
            raise SceneProtocolError('preview_notice must be a string of at most 1024 characters')
    for key in ('scene_epoch', 'time_epoch', 'coordinate_epoch', 'sync_epoch', 'config_revision', 'geometry_revision'):
        if key in value:
            _generation(value[key], key)
    _integer(value['sequence'], 'sequence')
    if value['mode'] not in ('live', 'demo', 'replay'):
        raise SceneProtocolError('invalid scene mode')
    if value.get('time_mode', 'monitor') not in ('monitor', 'aligned', 'independent'):
        raise SceneProtocolError('invalid time_mode')
    if value.get('time_mode')=='independent' and (value.get('scene_time_ns') is not None or value.get('mode')!='live'):
        raise SceneProtocolError('independent live preview has no common scene time')
    for key in ('generated_realtime_ns', 'scene_time_ns'):
        if value.get(key) is not None:
            _integer(value[key], key)
    if value.get('active_reference_camera') is not None:
        _string(value['active_reference_camera'], 'active_reference_camera')
    if 'display_from_world' in value:
        _transform(value['display_from_world'], 'display_from_world')
    for key in ('trackers', 'cameras'):
        entities = value[key]
        if not isinstance(entities, (list, tuple)) or len(entities) > 32:
            raise SceneProtocolError(f'{key} exceeds entity count limit')
        for entity in entities:
            _entity(entity)
    if not isinstance(value['diagnostics'], (list, tuple)) or len(value['diagnostics']) > 128:
        raise SceneProtocolError('diagnostics exceeds limit')


def validate_scene_message(value):
    _bounded_json(value)
    if not isinstance(value, dict):
        raise SceneProtocolError('message must be an object')
    if value.get('protocol') != PROTOCOL or type(value.get('version')) is not int or value['version'] != VERSION:
        raise SceneProtocolError('unsupported scene protocol/version')
    _string(value.get('request_id'), 'request_id')
    base = {'protocol', 'version', 'request_id'}
    if 'command' in value:
        _keys(value, base | {'command', 'params'}, base | {'command', 'params'}, 'request')
        command, params = value['command'], value['params']
        if command not in ('hello', 'describe', 'latest'):
            raise SceneProtocolError('unknown read-only command')
        allowed = {'after_sequence', 'viewer_id', 'interest'} if command == 'latest' else set()
        _keys(params, allowed, set(), 'params')
        if 'after_sequence' in params:
            _integer(params['after_sequence'], 'after_sequence')
        if 'viewer_id' in params:
            _string(params['viewer_id'], 'viewer_id')
        if 'interest' in params:
            interest = params['interest']
            if not isinstance(interest, (list, tuple)) or len(interest) > 8 or any(item not in ('tracker', 'camera', 'cloud', 'trail') for item in interest):
                raise SceneProtocolError('invalid display interest')
    else:
        fields = {'hello', 'description', 'scene', 'no_change'}
        _keys(value, base | {'ok', 'error'} | fields, base | {'ok', 'error'}, 'response')
        if type(value['ok']) is not bool:
            raise SceneProtocolError('ok must be boolean')
        present = set(value) & fields
        if value['ok']:
            if value['error'] is not None or len(present) != 1:
                raise SceneProtocolError('success requires one payload and no error')
            payload = next(iter(present))
            content = value[payload]
            if payload == 'scene':
                validate_scene(content)
            elif payload == 'no_change':
                if content is not True and not isinstance(content, dict):
                    raise SceneProtocolError('invalid no_change payload')
            else:
                if not isinstance(content, dict):
                    raise SceneProtocolError('payload must be an object')
                if payload == 'hello':
                    _string(content.get('manager_instance_id'), 'manager_instance_id')
                    if 'manager_pid' in content:
                        _integer(content['manager_pid'], 'manager_pid', 1)
                elif 'display_from_world' in content:
                    _transform(content['display_from_world'], 'display_from_world')
        else:
            if present:
                raise SceneProtocolError('error cannot carry stale success payload')
            error = value['error']
            _keys(error, {'code', 'message'}, {'code', 'message'}, 'error')
            _string(error['code'], 'error.code', 64)
            _string(error['message'], 'error.message', 1024)
    return value


def _object_pairs(pairs):
    value = {}
    for key, child in pairs:
        if key in value:
            raise SceneProtocolError('duplicate JSON key')
        value[key] = child
    return value


def _nonfinite(value):
    raise SceneProtocolError(f'nonfinite JSON number: {value}')


def _load_json(frame, max_bytes):
    try:
        if isinstance(frame, str):
            if len(frame) > max_bytes:
                raise SceneProtocolError('frame exceeds size limit')
            frame = frame.encode('utf-8')
        if not isinstance(frame, (bytes, bytearray)) or len(frame) > max_bytes:
            raise SceneProtocolError('invalid or oversized frame')
        value = json.loads(frame.decode('utf-8'), object_pairs_hook=_object_pairs, parse_constant=_nonfinite)
        _bounded_json(value)
        return value
    except (UnicodeError, json.JSONDecodeError, RecursionError, OverflowError, ValueError) as exc:
        raise SceneProtocolError(str(exc)) from exc


def encode_scene_message(value) -> bytes:
    if not isinstance(value, dict) and hasattr(value, 'to_dict'):
        value = value.to_dict()
    validate_scene_message(value)
    try:
        wire = json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode('utf-8') + b'\n'
    except (TypeError, ValueError, UnicodeError, RecursionError) as exc:
        raise SceneProtocolError(str(exc)) from exc
    if len(wire) > MAX_SCENE_BYTES:
        raise SceneProtocolError('frame exceeds size limit')
    return wire


def decode_scene_message(frame) -> dict:
    return validate_scene_message(_load_json(frame, MAX_SCENE_BYTES))
