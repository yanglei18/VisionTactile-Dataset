"""Explicit geometry configuration; loading never opens a sensor."""
from dataclasses import dataclass, field
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_core.spatial.cloud_protocol import MAX_POINTS


@dataclass(frozen=True)
class SceneConfig:
    enabled: bool = True
    world_frame: str = 'vive_map'
    scene_hz: float = 20.
    map_identity: dict | None = None
    config_revision: str = 'default'
    display_from_world: RigidTransform = field(default_factory=RigidTransform.identity)
    cameras: tuple = ()
    reference_camera: str | None = None
    cloud: dict = field(default_factory=lambda: {'enabled': False})
    timing: dict = field(default_factory=dict)
    source_path: str = ''
    alignment_document: dict | None = None
    # Presentation-only provenance; never changes calibration acceptance.
    preview_notice: str = ''


def _document(path):
    path = Path(path)
    if path.stat().st_size > 1024 * 1024:
        raise ValueError('geometry configuration exceeds 1 MiB')
    import yaml
    class UniqueLoader(yaml.SafeLoader):
        def construct_mapping(self, node, deep=False):
            seen = set()
            for key_node, _ in node.value:
                key = self.construct_object(key_node, deep=deep)
                if not isinstance(key, str) or key in seen:
                    raise ValueError('configuration contains duplicate or non-string keys')
                seen.add(key)
            return super().construct_mapping(node, deep=deep)
    try:
        value = yaml.load(path.read_text(encoding='utf-8'), Loader=UniqueLoader)
    except (yaml.YAMLError, RecursionError) as exc:
        raise ValueError(f'invalid geometry configuration: {exc}') from exc
    if not isinstance(value, dict):
        raise ValueError('geometry configuration must be a mapping')
    # Reject YAML dates, NaN and other values that cannot travel over IPC.
    json.dumps(value, allow_nan=False)
    return value


def _bounded(value, lo, hi, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{name} must be in [{lo}, {hi}]')
    return value


def load_scene_config(path=None):
    if path is None:
        return SceneConfig()
    path = Path(path).resolve()
    raw = _document(path)
    allowed = {'schema_version', 'enabled', 'world_frame', 'map_identity', 'alignment_config_path',
               'extrinsics_directory', 'camera_device_info_services', 'scene_hz', 'timing', 'display', 'cloud', 'depth_units',
               'preview_notice'}
    if (set(raw) - allowed or type(raw.get('schema_version', 1)) is not int
            or raw.get('schema_version', 1) != 1):
        raise ValueError('unknown scene configuration fields or version')
    enabled = raw.get('enabled', True)
    if type(enabled) is not bool:
        raise ValueError('enabled must be boolean')
    preview_notice = raw.get('preview_notice', '')
    if not isinstance(preview_notice, str) or len(preview_notice) > 1024:
        raise ValueError('preview_notice must be a string of at most 1024 characters')
    world = raw.get('world_frame', 'vive_map')
    if not isinstance(world, str) or not world or len(world) > 128:
        raise ValueError('world_frame must be a bounded frame name')
    identity = raw.get('map_identity')
    if identity is not None and (not isinstance(identity, dict) or set(identity) != {'id', 'verification'}
                                or any(not isinstance(v, str) or not v or len(v) > 256 for v in identity.values())):
        raise ValueError('map_identity requires id and verification')
    display = raw.get('display', {})
    if not isinstance(display, dict) or set(display) - {'display_from_world'}:
        raise ValueError('unknown display setting')
    transform = RigidTransform.from_dict(display['display_from_world']) if 'display_from_world' in display else RigidTransform.identity()
    cloud = raw.get('cloud', {'enabled': False})
    if not isinstance(cloud, dict) or set(cloud) - {'enabled', 'hz', 'max_points_per_camera', 'stride', 'min_depth_m', 'max_depth_m', 'color_mode', 'source', 'topics', 'matching_mode'}:
        raise ValueError('unknown cloud setting')
    cloud = dict(enabled=False, hz=10., max_points_per_camera=50000, stride=4,
                 min_depth_m=.1, max_depth_m=3., color_mode='source', source='rgbd', topics={},
                 matching_mode='synchronized') | cloud
    if cloud['matching_mode'] not in ('synchronized', 'independent'):
        raise ValueError('invalid cloud matching_mode')
    if cloud['source'] not in ('rgbd','realsense') or not isinstance(cloud['topics'],dict):
        raise ValueError('invalid cloud source/topics')
    from ..config import _topic
    for name, topic in cloud['topics'].items():
        if not isinstance(name,str): raise ValueError('invalid cloud camera name')
        _topic(topic)
    if type(cloud['enabled']) is not bool or cloud['color_mode'] not in ('source', 'rgb'):
        raise ValueError('invalid cloud mode')
    for key, lo, hi in [('hz', 1, 15), ('max_points_per_camera', 1, MAX_POINTS), ('stride', 1, 64),
                        ('min_depth_m', .001, 100), ('max_depth_m', .001, 100)]:
        _bounded(cloud[key], lo, hi, key)
    if any(type(cloud[key]) is not int for key in ('max_points_per_camera', 'stride')) or cloud['min_depth_m'] >= cloud['max_depth_m']:
        raise ValueError('invalid cloud sampling bounds')
    timing = raw.get('timing', {})
    limits = {'max_camera_delta_ms': (0, 100), 'max_tracker_gap_ms': (1, 100), 'wait_ms': (0, 500),
              'reference_stale_ms': (50, 5000), 'reference_recovery_ms': (100, 10000)}
    if not isinstance(timing, dict) or set(timing) - limits.keys():
        raise ValueError('unknown timing setting')
    for key, value in timing.items():
        _bounded(value, *limits[key], key)
    def resolve(value):
        if not isinstance(value, str) or not value or len(value) > 4096:
            raise ValueError('configuration path must be a bounded string')
        return (path.parent / value).resolve()
    depth_units = raw.get('depth_units', {})
    if not isinstance(depth_units, dict):
        raise ValueError('depth_units must be an explicit per-camera mapping')
    for name, contract in depth_units.items():
        if (not isinstance(contract, dict) or set(contract) != {'meters_per_unit', 'source'}
                or contract['source'] not in ('realsense_ros_z16_mm', 'rep118_16uc1_mm', 'rep118_32fc1_m')):
            raise ValueError('depth units require a supported publisher contract')
        expected = 1. if contract['source'] == 'rep118_32fc1_m' else .001
        if type(contract['meters_per_unit']) not in (int, float) or contract['meters_per_unit'] != expected:
            raise ValueError('depth unit disagrees with publisher encoding contract')
    cameras = []
    reference = None
    alignment = None
    alignment_path = raw.get('alignment_config_path')
    services = raw.get('camera_device_info_services', {})
    if not isinstance(services, dict):
        raise ValueError('camera_device_info_services must be a mapping')
    for value in services.values():
        from ..config import _topic
        _topic(value)
    if alignment_path is not None:
        alignment = _document(resolve(alignment_path))
        if alignment.get('frames', {}).get('world', world) != world:
            raise ValueError('scene/alignment world frames disagree')
        tracker_entries = alignment.get('trackers', [])
        if not isinstance(tracker_entries, list) or any(not isinstance(t, dict) for t in tracker_entries):
            raise ValueError('alignment trackers must be explicit role/frame mappings')
        tracker_frames = {t.get('role'): t.get('frame_id') for t in tracker_entries}
        if len(tracker_frames) != len(tracker_entries):
            raise ValueError('duplicate tracker role in alignment configuration')
        entries = alignment.get('cameras', [])
        if not isinstance(entries, list) or not 1 <= len(entries) <= 3:
            raise ValueError('scene supports one to three configured cameras')
        extrinsics = resolve(raw['extrinsics_directory']) if raw.get('extrinsics_directory') else None
        for entry in entries:
            camera = dict(entry)
            name = camera['name']
            if not isinstance(name, str) or not name.replace('_', '').isalnum() or len(name) > 128:
                raise ValueError('invalid camera name')
            camera['id'] = name
            tracker_frame = tracker_frames.get(camera.get('tracker_role'))
            if not isinstance(tracker_frame, str) or not tracker_frame or len(tracker_frame) > 128:
                raise ValueError('camera requires an explicitly configured Tracker role/frame')
            camera['tracker_frame_id'] = tracker_frame
            camera['color_frame_id'] = camera.get('color_frame_id', camera.get('frame_id', ''))
            camera['depth_frame_id'] = camera.get('depth_frame_id', f'{name}_depth_optical_frame')
            camera['timing_frame_id'] = camera.get('timing_frame_id', f'{name}_frame_group')
            camera['depth_camera_info_topic'] = camera.get('depth_camera_info_topic', f'/{name}/depth/camera_info')
            camera['device_info_service'] = services.get(name, '')
            camera['depth_to_color_topic'] = f'/{name}/extrinsics/depth_to_color'
            if cloud['source'] == 'realsense':
                camera['cloud_topic'] = cloud['topics'].get(name, f'/{name}/depth/color/points')
            if name in depth_units:
                camera['depth_units_m'] = depth_units[name]['meters_per_unit']
                camera['depth_units_source'] = depth_units[name]['source']
            if camera['device_info_service']:
                from ..config import _topic
                _topic(camera['device_info_service'])
            if extrinsics:
                camera['extrinsic_path'] = str(extrinsics / f'{name}.yaml')
            cameras.append(camera)
        if len({v['id'] for v in cameras}) != len(cameras):
            raise ValueError('duplicate camera identity')
        if set(cloud['topics']) - {v['id'] for v in cameras}:
            raise ValueError('point cloud topic configured for unknown camera')
        reference = alignment.get('reference_camera', cameras[0]['id'])
        if reference not in {c['id'] for c in cameras}:
            raise ValueError('unknown reference camera')
        # Freeze the explicit live frame contract for replay. Legacy alignment
        # files remain untouched; their single-frame offline semantics cannot
        # describe the distinct raw depth and frame-group messages observed here.
        alignment = deepcopy(alignment)
        alignment['schema_version'] = 2
        for archived, camera in zip(alignment['cameras'], cameras):
            archived.pop('frame_id', None)
            for key in ('color_frame_id', 'depth_frame_id', 'timing_frame_id', 'depth_camera_info_topic'):
                archived[key] = camera[key]
    digest = hashlib.sha256(json.dumps(dict(scene=raw, cameras=cameras), sort_keys=True).encode()).hexdigest()
    return SceneConfig(enabled, world, _bounded(raw.get('scene_hz', 20.), 1, 60, 'scene_hz'), identity,
                       digest, transform, tuple(cameras), reference, cloud, dict(timing), str(path), alignment,
                       preview_notice)
