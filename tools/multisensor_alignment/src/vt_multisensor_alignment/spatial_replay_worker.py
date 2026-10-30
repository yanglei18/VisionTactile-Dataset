"""Read-only ROS bag replay worker, isolated from the Qt process.

stdout is bounded JSON-lines IPC. ROS deserialization, image reads and point
cloud construction happen here; diagnostic/log output belongs on stderr.
"""
from bisect import bisect_left
from collections import defaultdict
import base64
import json
import sys
from types import SimpleNamespace
import uuid

from vt_capture_core.spatial.transforms import RigidTransform, finite_vector
from vt_capture_core.spatial.types import PoseSample, PoseSelection
from vt_capture_core.spatial.timeline import PoseBuffer
from vt_capture_core.spatial.frustum import frustum_vertices
from vt_capture_core.spatial.scene_protocol import validate_scene
from vt_capture_core.spatial.cloud_protocol import pack_cloud_frame

from .bag_input import absolute_path, inventory
from .config import load_config_document
from .bag_reader import (read_unified_bag, _open_reader, _message_types, _deserializer,
                         _pose_from_message, stamp_to_nanoseconds)
from .geometry_materials import GeometryMaterials
from .image_reader import BagImageReader
from .model import MessageRef

MAX_REPLY_BYTES = 4*1024*1024
ROLES = ('left_wrist', 'right_wrist', 'torso')


def _inventory(path):
    return inventory(path)[1]


def _assert_closed_bag(path, materials):
    if not (path/'metadata.yaml').is_file():
        raise ValueError('replay requires a closed bag with metadata.yaml')
    if materials.manifest is not None and materials.manifest.get('complete') is not True:
        raise ValueError('geometry sidecar indicates the bag is not completely closed')
    if any(path.glob('*.active')):
        raise ValueError('replay requires a closed bag')


def _read_replay_dataset(path, materials, cancelled):
    before = _inventory(path)
    document = materials.manifest.get('alignment_config') if materials.manifest else None
    if document is not None:
        config = load_config_document(document)
        dataset = read_unified_bag(path, config, cancelled=cancelled, allow_partial_cameras=True)
    else:
        # Legacy bags expose recorded Tracker poses even without any geometry
        # materials. Camera bindings are never guessed from topic/name order.
        reader = _open_reader(path)
        types = _message_types(reader)
        topics = {topic for topic, kind in types.items() if kind == 'vt_tracker_msgs/msg/TrackerSample'}
        if not topics:
            raise ValueError('bag contains no recorded TrackerSample stream')
        deserialize = _deserializer('vt_tracker_msgs/msg/TrackerSample')
        poses, invalid, counts, identities = defaultdict(list), defaultdict(list), defaultdict(int), {}
        world = None
        total = 0
        while reader.has_next():
            if cancelled():
                raise InterruptedError('bag indexing cancelled')
            topic, payload, bag_ns = reader.read_next()
            total += 1
            if total > 10_000_000:
                raise ValueError('bag index exceeds 10 million messages')
            if topic not in topics:
                continue
            message = deserialize(payload)
            role = message.role
            if role not in ROLES:
                continue
            world = world or message.header.frame_id
            reference = MessageRef(topic, counts[topic], int(bag_ns), stamp_to_nanoseconds(message.header.stamp))
            counts[topic] += 1
            pose = _pose_from_message(message, role=role, world_frame=world, reference=reference)
            if role in identities and identities[role] != message.tracker_id:
                raise ValueError('legacy bag changes Tracker identity without recorded epoch boundaries')
            identities[role] = message.tracker_id
            if pose is None:
                invalid[role].append(int(message.host_realtime_ns))
            else:
                if poses[role] and pose.host_realtime_ns <= poses[role][-1].host_realtime_ns:
                    raise ValueError('legacy Tracker clock is not continuous; explicit replay segments required')
                poses[role].append(pose)
        config = SimpleNamespace(world_frame=world or 'vive_map', cameras=(), reference_camera=None,
                                 thresholds=SimpleNamespace(max_camera_delta_ms=20., max_tracker_gap_ms=50.))
        dataset = SimpleNamespace(tracker_poses=dict(poses), camera_frames={}, invalid_tracker_times=dict(invalid))
    if _inventory(path) != before:
        raise ValueError('bag changed during indexing; close recording before replay')
    return config, dataset


class BagReplaySource:
    def __init__(self, path, cancelled=None):
        self.path = absolute_path(path)
        self.cancelled = cancelled or (lambda: False)
        self.materials = GeometryMaterials(self.path)
        manifest = self.materials.manifest or {}
        self._display_from_world = (RigidTransform.from_dict(manifest['display_from_world'])
                                    if 'display_from_world' in manifest else RigidTransform.identity())
        _assert_closed_bag(self.path, self.materials)
        self.config, self.dataset = _read_replay_dataset(self.path, self.materials, self.cancelled)
        self.manager_instance_id = 'replay-'+uuid.uuid4().hex
        self.scene_epoch = uuid.uuid4().hex
        self.worker_instance_id = uuid.uuid4().hex
        self._pose_times = {role: [pose.host_realtime_ns for pose in poses] for role, poses in self.dataset.tracker_poses.items()}
        self._camera_times = {name: [frame.host_realtime_ns for frame in frames] for name, frames in self.dataset.camera_frames.items()}
        reference = self.config.reference_camera
        if self._camera_times.get(reference):
            self._times = self._camera_times[reference]
        elif any(self._camera_times.values()):
            # Replay keeps its configured reference identity; the recorded
            # timeline may still drive partial frames while that camera is absent.
            self._times = sorted({stamp for times in self._camera_times.values() for stamp in times})
        else:
            reference_role = next((role for role in ROLES if self._pose_times.get(role)), None)
            self._times = self._pose_times.get(reference_role, [])
        if not self._times:
            raise ValueError('bag contains no usable replay frames')
        self.frame_count = len(self._times)
        self.note = 'Recorded ROS bag; '+self.materials.note
        self.last_clouds = []
        self._images = None

    def _pose(self, role, timestamp):
        times = self._pose_times.get(role, [])
        if not times:
            return PoseSelection(None, 'unavailable', 'missing_tracker')
        index = bisect_left(times, timestamp)
        selected = (self.dataset.tracker_poses[role][index:index+1]
                    if index < len(times) and times[index] == timestamp else
                    self.dataset.tracker_poses[role][max(0, index-1):min(len(times), index+1)])
        if not selected:
            return PoseSelection(None, 'unavailable', 'no_after')
        start, end = selected[0].host_realtime_ns, selected[-1].host_realtime_ns
        if any(start <= invalid <= end for invalid in self.dataset.invalid_tracker_times.get(role, ())):
            return PoseSelection(None, 'unavailable', 'invalid_pose', start, end, end-start)
        buffer = PoseBuffer()
        for sample in selected:
            buffer.append(PoseSample(role, sample.tracker_id, self.config.world_frame, sample.host_realtime_ns,
                                     sample.host_monotonic_ns, 'replay', 'replay',
                                     RigidTransform(sample.transform.translation, sample.transform.quaternion_xyzw), True))
        return buffer.select(timestamp, max_gap_ns=int(self.config.thresholds.max_tracker_gap_ms*1e6))

    def _nearest_camera(self, name, timestamp):
        times = self._camera_times.get(name, [])
        index = bisect_left(times, timestamp)
        choices = self.dataset.camera_frames.get(name, ())[max(0, index-1):min(len(times), index+1)]
        if not choices:
            return None
        selected = min(choices, key=lambda frame: abs(frame.host_realtime_ns-timestamp))
        return selected if abs(selected.host_realtime_ns-timestamp) <= self.config.thresholds.max_camera_delta_ms*1e6 else None

    def frame(self, index):
        self.last_clouds = []
        if type(index) is not int or not 0 <= index < self.frame_count:
            raise IndexError('replay frame index is out of range')
        if self.cancelled():
            raise InterruptedError('replay frame cancelled')
        timestamp = self._times[index]
        trackers = []
        for role in ROLES:
            selection = self._pose(role, timestamp)
            pose = selection.pose
            samples = self.dataset.tracker_poses.get(role, ())
            trackers.append(dict(id=role, role=role, label=role, tracker_id=samples[0].tracker_id if samples else None,
                frame_id=self.config.world_frame, source_realtime_ns=timestamp if pose else None,
                position_m=list(pose.translation_m) if pose else None, quaternion_xyzw=list(pose.quaternion_xyzw) if pose else None,
                state='fresh' if pose else 'offline', age_s=0. if pose else None, valid=pose is not None,
                reason=selection.reason, trail_segments=[], before_ns=selection.before_ns, after_ns=selection.after_ns, gap_ns=selection.gap_ns))
        cameras, clouds, diagnostics = [], [], []
        if self.config.reference_camera and self._nearest_camera(self.config.reference_camera, timestamp) is None:
            diagnostics.append('reference_missing')
        generations = set()
        for camera in self.config.cameras:
            if self.cancelled():
                raise InterruptedError('replay frame cancelled')
            selected = self._nearest_camera(camera.name, timestamp)
            entity = dict(id=camera.name, label=camera.name, camera_id=camera.name, serial=camera.serial,
                          tracker_role=camera.tracker_role, position_m=None, quaternion_xyzw=None, state='waiting',
                          reason='missing_frame', frustum_vertices=None, source_realtime_ns=None)
            cameras.append(entity)
            if selected is None:
                continue
            material = self.materials.select(camera.name, selected.host_realtime_ns, require_depth=False)
            entity['reason'] = material.reason
            if material.status != 'ready':
                continue
            geometry = material.geometry
            tracker = self._pose(camera.tracker_role, selected.host_realtime_ns)
            expected_id = geometry['extrinsic']['tracker']['tracker_id']
            samples = self.dataset.tracker_poses.get(camera.tracker_role, ())
            if tracker.pose is None or not samples or samples[0].tracker_id != expected_id:
                entity['reason'] = 'tracker_unavailable_or_identity_mismatch'
                continue
            transform = geometry['extrinsic']['transform']
            world = tracker.pose.compose(RigidTransform(transform['translation_m'], transform['quaternion_xyzw']))
            frame_key = dict(camera_id=camera.name, shared_ros_timestamp_ns=selected.source_timestamp_ns,
                             bag_timestamp_ns=selected.color.bag_timestamp_ns)
            entity.update(position_m=list(world.translation_m), quaternion_xyzw=list(world.quaternion_xyzw),
                          source_realtime_ns=selected.host_realtime_ns, state='fresh', reason='', frame_key=frame_key,
                          geometry_revision=geometry['geometry_revision'], geometry_sha256=geometry['geometry_sha256'],
                          time_residual_ns=selected.host_realtime_ns-timestamp, intrinsics=geometry.get('color_intrinsics'))
            info = geometry.get('color_intrinsics')
            recorded_frustum = geometry.get('color_frustum')
            if recorded_frustum is not None:
                if not isinstance(recorded_frustum, (list, tuple)) or len(recorded_frustum) != 8:
                    raise ValueError('recorded color_frustum must contain eight vertices')
                entity['frustum_vertices'] = [list(finite_vector(point, 3, 'recorded frustum vertex'))
                                              for point in recorded_frustum]
            elif info:
                try:
                    entity['frustum_vertices'] = [list(point) for point in frustum_vertices(info, distortion=info.get('d', ()))]
                except (ValueError, KeyError, TypeError):
                    pass
            generation = (material.time_epoch, material.coordinate_epoch)
            generations.add(generation)
            metric = self.materials.select(camera.name, selected.host_realtime_ns)
            if metric.status != 'ready':
                diagnostics.append(f'{camera.name}: {metric.reason}')
                continue
            try:
                from vt_capture_manager.spatial.cloud_builder import build_cloud
                if self._images is None:
                    self._images = BagImageReader(self.path, cancelled=lambda: self.cancelled())
                images = self._images.read_many((selected.color, selected.depth), expected_frames={
                    camera.color_topic: camera.color_frame_id, camera.depth_topic: camera.depth_frame_id})
                def message(image):
                    return SimpleNamespace(width=image.width, height=image.height, step=image.step, encoding=image.encoding,
                        is_bigendian=image.is_bigendian, data=image.data, header=SimpleNamespace(frame_id=image.frame_id))
                xyz, rgba = build_cloud(message(images[selected.depth]), message(images[selected.color]), metric.geometry,
                                        stride=4, max_points=50_000, color_mode='rgb')
                header = dict(camera_id=camera.name, frame_key=frame_key, source_realtime_ns=selected.host_realtime_ns,
                              time_epoch=material.time_epoch, coordinate_epoch=material.coordinate_epoch,
                              geometry_revision=geometry['geometry_revision'], geometry_sha256=geometry['geometry_sha256'],
                              point_count=len(xyz)//12, world_from_color=world.as_dict(), worker_instance_id=self.worker_instance_id,
                              cloud_sequence=index+1, manager_instance_id=self.manager_instance_id, scene_epoch=self.scene_epoch, sync_epoch=0)
                clouds.append((header, xyz, rgba))
            except (ImportError, ValueError, KeyError, TypeError, AttributeError) as exc:
                diagnostics.append(f'{camera.name}: point cloud unavailable: {str(exc)[:160]}')
        if len(generations) > 1:
            clouds.clear()
            diagnostics.append('Camera geometry generations disagree; point clouds withheld.')
        time_epoch, coordinate_epoch = next(iter(generations)) if len(generations) == 1 else ('replay', 'replay')
        manifest = self.materials.manifest or {}
        scene = dict(manager_instance_id=self.manager_instance_id, scene_epoch=self.scene_epoch, sequence=index+1,
                     mode='replay', world_frame=self.config.world_frame, time_epoch=time_epoch, coordinate_epoch=coordinate_epoch,
                     sync_epoch=0, time_mode='aligned', scene_time_ns=timestamp, generated_realtime_ns=timestamp,
                     config_revision=manifest.get('config_revision', 'recorded'), geometry_revision='recorded',
                     active_reference_camera=self.config.reference_camera, map_identity=None,
                     preview_notice=manifest.get('preview_notice', ''),
                     display_from_world=self._display_from_world.as_dict(), trackers=trackers, cameras=cameras,
                     diagnostics=diagnostics or ([] if manifest else ['Recorded geometry unavailable; Tracker-only replay.']))
        validate_scene(scene)
        self.last_clouds = [base64.b64encode(pack_cloud_frame(*cloud)).decode('ascii') for cloud in clouds]
        return scene

    def close(self):
        if self._images is not None:
            self._images.close()


def _reply(value):
    encoded = json.dumps(value, separators=(',', ':'), allow_nan=False).encode()+b'\n'
    if len(encoded) > MAX_REPLY_BYTES:
        raise ValueError('replay reply exceeds 4 MiB')
    sys.stdout.buffer.write(encoded)
    sys.stdout.buffer.flush()


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        _reply({'ok': False, 'error': 'Expected one closed ROS bag directory'})
        return 2
    source = None
    try:
        source = BagReplaySource(args[0])
        _reply({'ok': True, 'metadata': {'frame_count': source.frame_count, 'note': source.note}})
        while True:
            line = sys.stdin.buffer.readline(4097)
            if not line:
                break
            if len(line) > 4096:
                raise ValueError('replay request exceeds bound')
            request_id = index = None
            try:
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise ValueError('invalid replay request')
                request_id, index = request.get('request_id'), request.get('index')
                if (set(request) != {'command', 'index', 'request_id'} or request['command'] != 'frame'
                        or type(request_id) is not int or request_id <= 0 or type(index) is not int):
                    raise ValueError('only identified frame(index) requests are supported')
                scene = source.frame(index)
                _reply({'ok': True, 'request_id': request_id, 'index': index,
                        'scene': scene, 'clouds': source.last_clouds})
            except (ValueError, KeyError, IndexError, TypeError, RuntimeError, OSError) as exc:
                _reply({'ok': False, 'request_id': request_id, 'index': index, 'error': str(exc)[:512]})
        return 0
    except (ValueError, TypeError, KeyError, OSError, RuntimeError) as exc:
        _reply({'ok': False, 'error': str(exc)[:512]})
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == '__main__':
    raise SystemExit(main())
