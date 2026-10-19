"""Hash-verified, recorded geometry selection with explicit epoch boundaries.

Never opens current machine configuration or substitutes a live calibration.
Legacy bags without a sidecar return an explicit unavailable selection.
"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from .model import Transform

MAX_DOCUMENT_BYTES = 512*1024
MAX_ARTIFACT_BYTES = 32*1024*1024
_GEOMETRY_CONTENT = ('config', 'extrinsic', 'color_intrinsics', 'depth_intrinsics',
                     'depth_to_color', 'depth_units_m', 'depth_units_source')


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate geometry JSON key')
        result[key] = value
    return result


def _read(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError('geometry document must be a regular file')
    if path.stat().st_size > MAX_DOCUMENT_BYTES:
        raise ValueError('geometry document exceeds 512 KiB')
    data = path.read_bytes()
    if len(data) > MAX_DOCUMENT_BYTES:
        raise ValueError('geometry document exceeds 512 KiB')
    try:
        value = json.loads(data, object_pairs_hook=_pairs)
        json.dumps(value, allow_nan=False)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise ValueError('invalid geometry JSON') from exc
    if not isinstance(value, dict):
        raise ValueError('geometry document must be an object')
    return value, data


@dataclass(frozen=True)
class GeometrySelection:
    camera_id: str
    geometry: dict | None
    status: str
    reason: str = ''
    valid_from_ns: int | None = None
    valid_until_ns: int | None = None
    time_epoch: str | int | None = None
    coordinate_epoch: str | int | None = None
    scene_epoch: str | None = None


class GeometryMaterials:
    def __init__(self, bag_path, *, sidecar_path=None, expected_session_id=None):
        bag = Path(bag_path).absolute()
        self.directory = Path(sidecar_path).absolute() if sidecar_path else bag.with_name(bag.name+'.spatial')
        self.manifest = None
        self._entries = {}
        self._identity_intervals = None
        self.note = 'No recorded geometry sidecar; metric point clouds unavailable.'
        path = self.directory/'manifest.json'
        if not path.exists():
            return
        if any(item.is_symlink() for item in (self.directory, *self.directory.parents)):
            raise ValueError('geometry directory cannot contain symlinks')
        manifest, _ = _read(path)
        if type(manifest.get('schema_version')) is not int or manifest['schema_version'] != 1 or manifest.get('kind') != 'vt.spatial.geometry':
            raise ValueError('unsupported geometry sidecar schema')
        if manifest.get('bag_path') != str(bag):
            raise ValueError('geometry sidecar bag identity mismatch')
        if expected_session_id is not None and manifest.get('session_id') != expected_session_id:
            raise ValueError('geometry sidecar session identity mismatch')
        revisions = manifest.get('revisions')
        if not isinstance(revisions, list) or len(revisions) > 512:
            raise ValueError('invalid geometry revision count')
        if 'identity_intervals' in manifest:
            intervals = manifest['identity_intervals']
            if not isinstance(intervals, list) or len(intervals) > 512:
                raise ValueError('invalid identity interval count')
            self._identity_intervals = deepcopy(intervals)
            for interval in intervals:
                if not isinstance(interval, dict):
                    raise ValueError('invalid identity interval')
                for key in ('camera_id', 'serial', 'scene_epoch', 'geometry_sha256'):
                    if not isinstance(interval.get(key), str) or not interval[key] or len(interval[key]) > 256:
                        raise ValueError('invalid identity interval identity')
                for key in ('start_realtime_ns', 'end_realtime_ns', 'last_observed_ns', 'identity_revision', 'geometry_revision'):
                    if type(interval.get(key)) is not int or not 0 <= interval[key] <= 2**63-1:
                        raise ValueError('invalid identity interval time/revision')
                for key in ('time_epoch', 'coordinate_epoch'):
                    if type(interval.get(key)) not in (str, int) or isinstance(interval.get(key), bool):
                        raise ValueError('invalid identity interval epoch')
                if (type(interval.get('verified')) is not bool or interval['start_realtime_ns'] > interval['end_realtime_ns']
                        or interval['start_realtime_ns'] > interval['last_observed_ns']
                        or interval['end_realtime_ns'] > interval['last_observed_ns']+5_000_000_000):
                    raise ValueError('invalid identity interval lease')
        artifacts, total_bytes = {}, 0
        for revision in revisions:
            if not isinstance(revision, dict):
                raise ValueError('invalid geometry revision')
            name = revision.get('artifact')
            if not isinstance(name, str) or not name or Path(name).name != name or name in ('.', '..'):
                raise ValueError('invalid geometry artifact path')
            if name not in artifacts:
                geometry, data = _read(self.directory/name)
                total_bytes += len(data)
                if total_bytes > MAX_ARTIFACT_BYTES:
                    raise ValueError('geometry artifact bytes exceed bound')
                artifacts[name] = (geometry, hashlib.sha256(data).hexdigest())
            geometry, digest = artifacts[name]
            if digest != revision.get('artifact_sha256'):
                raise ValueError('geometry artifact hash mismatch')
            camera = revision.get('camera_id')
            stamp = revision.get('observed_realtime_ns')
            if not isinstance(camera, str) or not camera or type(stamp) is not int or stamp < 0:
                raise ValueError('invalid geometry camera/time')
            if geometry.get('geometry_sha256') != revision.get('geometry_sha256') or geometry.get('geometry_revision') != revision.get('geometry_revision'):
                raise ValueError('geometry revision/hash mismatch')
            if all(key in geometry for key in _GEOMETRY_CONTENT):
                canonical = json.dumps({key: geometry[key] for key in _GEOMETRY_CONTENT}, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
                if hashlib.sha256(canonical).hexdigest() != geometry.get('geometry_sha256'):
                    raise ValueError('geometry content hash mismatch')
            self._entries.setdefault(camera, []).append((deepcopy(revision), geometry))
        self.manifest = manifest
        self.note = 'Recorded geometry validated by artifact SHA-256; epoch and segment bounds still apply.'

    def select(self, camera_id, source_realtime_ns, *, time_epoch=None, coordinate_epoch=None, scene_epoch=None, require_depth=True):
        unavailable = lambda reason: GeometrySelection(camera_id, None, 'unavailable', reason)
        if type(source_realtime_ns) is not int or source_realtime_ns < 0:
            raise ValueError('source_realtime_ns must be nonnegative')
        if self.manifest is None:
            return unavailable('missing_geometry_sidecar')
        candidates = self._entries.get(camera_id, [])
        if not candidates:
            return unavailable('missing_camera_geometry')
        supplied = {'time_epoch': time_epoch, 'coordinate_epoch': coordinate_epoch, 'scene_epoch': scene_epoch}
        if any(any(key not in revision for key in supplied) for revision, _ in candidates):
            return unavailable('missing_generation_boundary')
        candidates = [(rev, geom) for rev, geom in candidates
                      if all(value is None or str(rev[key]) == str(value) for key, value in supplied.items())]
        if not candidates:
            return unavailable('generation_mismatch')
        generations = {tuple(str(rev[key]) for key in supplied) for rev, _ in candidates}
        if len(generations) != 1:
            return unavailable('ambiguous_generation')
        candidates.sort(key=lambda item: item[0]['observed_realtime_ns'])
        eligible = [(rev, geom) for rev, geom in candidates if rev['observed_realtime_ns'] <= source_realtime_ns]
        if not eligible:
            return unavailable('before_geometry_segment')
        revision, geometry = eligible[-1]
        start = revision['observed_realtime_ns']
        if sum(rev['observed_realtime_ns'] == start for rev, _ in candidates) > 1:
            return unavailable('ambiguous_geometry_boundary')
        future = [rev['observed_realtime_ns'] for rev, _ in candidates if rev['observed_realtime_ns'] > start]
        end = min(future) if future else self.manifest.get('updated_realtime_ns')
        if type(end) is not int or end < start:
            return unavailable('missing_geometry_end_boundary')
        if source_realtime_ns > end:
            return unavailable('after_recorded_geometry')
        required = _GEOMETRY_CONTENT if require_depth else ('config', 'extrinsic')
        if not all(geometry.get(key) for key in required):
            return unavailable('incomplete_geometry')
        try:
            config, identity, extrinsic = geometry['config'], geometry.get('identity', {}), geometry['extrinsic']
            if config.get('id', config.get('name')) != camera_id:
                return unavailable('camera_identity_mismatch')
            if identity.get('verified') is not True or identity.get('serial') != config.get('serial'):
                return unavailable('identity_unverified')
            if self._identity_intervals is None:
                # Compatibility with first-version sidecars: a single DeviceInfo
                # observation can only prove its original bounded lease.
                observed = identity.get('observed_ns')
                if type(observed) is not int or not 0 <= source_realtime_ns-observed <= 5_000_000_000:
                    return unavailable('identity_observation_expired')
                start, end = max(start, observed), min(end, observed+5_000_000_000)
            else:
                identity_revision = geometry.get('identity_revision', 0)
                intervals = [value for value in self._identity_intervals
                             if value['camera_id'] == camera_id and value['verified'] is True
                             and value['serial'] == config['serial']
                             and value['identity_revision'] == identity_revision
                             and value['geometry_revision'] == geometry['geometry_revision']
                             and value['geometry_sha256'] == geometry['geometry_sha256']
                             and all(str(value[key]) == str(revision[key]) for key in supplied)
                             and value['start_realtime_ns'] <= source_realtime_ns < value['end_realtime_ns']]
                if len(intervals) != 1:
                    return unavailable('identity_interval_unverified' if not intervals else 'ambiguous_identity_interval')
                start = max(start, intervals[0]['start_realtime_ns'])
                end = min(end, intervals[0]['end_realtime_ns'])
            if (extrinsic.get('status') != 'VALID' or extrinsic['camera']['name'] != camera_id
                    or extrinsic['camera']['serial'] != config['serial']
                    or extrinsic['camera']['frame_id'] != config['color_frame_id']
                    or extrinsic['tracker']['role'] != config['tracker_role']
                    or (config.get('model') and extrinsic['camera'].get('model') != config['model'])
                    or (config.get('tracker_frame_id') and extrinsic['tracker'].get('frame_id') != config['tracker_frame_id'])):
                return unavailable('extrinsic_identity_mismatch')
            transform = extrinsic['transform']
            if (transform.get('semantics') != 'parent_from_child'
                    or transform.get('parent_frame') != extrinsic['tracker']['frame_id']
                    or transform.get('child_frame') != config['color_frame_id']):
                return unavailable('extrinsic_direction_mismatch')
            if extrinsic.get('frames', {}).get('world', self.manifest.get('world_frame')) != self.manifest.get('world_frame'):
                return unavailable('world_frame_mismatch')
            transforms = (geometry['depth_to_color'], transform) if require_depth else (transform,)
            for transform in transforms:
                Transform(transform['translation_m'], transform['quaternion_xyzw'])
            if require_depth:
                units = geometry['depth_units_m']
                if type(units) not in (int, float) or not math.isfinite(units) or units <= 0:
                    return unavailable('invalid_depth_units')
        except (KeyError, TypeError, ValueError):
            return unavailable('invalid_geometry')
        return GeometrySelection(camera_id, deepcopy(geometry), 'ready', '', start, end,
                                 revision['time_epoch'], revision['coordinate_epoch'], revision['scene_epoch'])
