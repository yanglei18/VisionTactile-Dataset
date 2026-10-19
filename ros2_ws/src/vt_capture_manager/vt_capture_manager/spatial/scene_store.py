"""Thread-safe, source-time scene cache. Never re-zero individual devices."""
from collections import deque
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import threading
import time
import uuid

from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_core.spatial.timeline import PoseBuffer
from vt_capture_core.spatial.types import PoseSample, timestamp_ns

ROLES = ('left_wrist', 'right_wrist', 'torso')
LABELS = ('左腕 Tracker', '右腕 Tracker', '躯干 Tracker')


class SceneStore:
    def __init__(self, manager_instance_id, config, *, realtime_ns=time.time_ns, monotonic_ns=time.monotonic_ns):
        self.manager_instance_id, self.config = manager_instance_id, config
        self.realtime_ns, self.monotonic_ns = realtime_ns, monotonic_ns
        self.scene_epoch = uuid.uuid4().hex
        self.time_epoch = self.coordinate_epoch = self.sync_epoch = self.sequence = 0
        self._lock = threading.RLock()
        self._states = {role: self._empty() for role in ROLES}
        self._retired_identities = {role: deque(maxlen=32) for role in ROLES}
        self.buffers = {role: PoseBuffer() for role in ROLES}
        self.camera_registry = None
        self.cloud_supervisor = None
        self.active_reference_camera = None
        self._diagnostics = deque(maxlen=32)
        self._interests = {}
        self._base_config_revision = config.config_revision
        self._calibration_digests = {}

    def apply_camera_calibration(self, camera_id, document, *, tracker_sample_topic=None):
        """Change live geometry and future replay bindings as one scene operation.

        The manager must hold its admission mutex and exclude formal recordings.
        Loaded configuration objects and already queued recording snapshots remain
        immutable; the active scene gets a new configuration revision.
        """
        with self._lock:
            if self.camera_registry is None:
                raise ValueError('scene has no configured camera geometry')
            cameras = deepcopy(list(self.config.cameras))
            camera = next((value for value in cameras if value['id'] == camera_id), None)
            if camera is None:
                raise ValueError('camera is not configured in this scene')
            tracker = document['tracker']
            camera.update(tracker_role=tracker['role'], tracker_frame_id=tracker['frame_id'])
            camera.pop('extrinsic_path', None)
            alignment = deepcopy(self.config.alignment_document)
            if alignment is not None:
                archived = next((value for value in alignment['cameras'] if value['name'] == camera_id), None)
                if archived is None:
                    raise ValueError('camera is missing from recorded alignment contract')
                archived['tracker_role'] = tracker['role']
                tracker_binding = next((value for value in alignment['trackers'] if value['role'] == tracker['role']), None)
                if tracker_binding is None or tracker_binding['frame_id'] != tracker['frame_id']:
                    raise ValueError('calibrated Tracker frame differs from recorded alignment contract')
                if tracker_sample_topic is not None and tracker_binding['sample_topic'] != tracker_sample_topic:
                    raise ValueError('calibrated Tracker topic differs from recorded alignment contract')
            canonical = json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
            digests = dict(self._calibration_digests, **{camera_id: hashlib.sha256(canonical).hexdigest()})
            revision = hashlib.sha256(json.dumps(dict(base=self._base_config_revision, cameras=cameras,
                alignment=alignment, calibrations=digests), sort_keys=True, allow_nan=False).encode()).hexdigest()
            config = replace(self.config, cameras=tuple(cameras), alignment_document=alignment, config_revision=revision)
            changed = self.camera_registry.replace_calibration_binding(camera_id, document)
            if config == self.config and not changed:
                return False
            self.config, self._calibration_digests = config, digests
            self.sync_epoch += 1
            self.active_reference_camera = None
            return True

    def geometry_snapshot(self):
        """Capture scene, configuration and camera revisions under one scene lock."""
        with self._lock:
            config = deepcopy(self.config)
            geometries = {camera['id']: self.camera_registry.geometry(camera['id']) if self.camera_registry else {}
                          for camera in config.cameras}
            return config, self.snapshot(), geometries

    @staticmethod
    def _empty():
        return dict(tracker_id=None, source_ns=None, source_monotonic_ns=None, received_ns=None,
                    frame_id=None, pose=None, valid=False, trail=deque(maxlen=90), broken=True, reason='等待数据')

    def _reset(self, *, coordinate=False):
        if coordinate:
            self.coordinate_epoch += 1
        else:
            self.time_epoch += 1
        self.sync_epoch += 1
        for role in ROLES:
            self.buffers[role].reset(time_epoch=str(self.time_epoch), coordinate_epoch=str(self.coordinate_epoch))
            state = self._states[role]
            state['trail'].clear()
            state.update(pose=None, valid=False, source_ns=None, source_monotonic_ns=None,
                         received_ns=None, broken=True, reason='坐标或时钟代次变化，等待新样本')

    def observe_tracker(self, role, message):
        if role not in self._states:
            return False
        with self._lock:
            state = self._states[role]
            now, monotonic = self.realtime_ns(), self.monotonic_ns()
            try:
                source = timestamp_ns(message.host_realtime_ns, 'host_realtime_ns')
                source_mono = timestamp_ns(message.host_monotonic_ns, 'host_monotonic_ns')
                identity = message.tracker_id
                frame = message.header.frame_id
                if not isinstance(identity, str) or not identity.strip() or len(identity) > 256 or source <= 0:
                    raise ValueError('invalid tracker identity/time')
                if (identity, frame) in self._retired_identities[role]:
                    return False
                if message.role != role or frame != self.config.world_frame or source > now + 500_000_000:
                    # Bad metadata must not reset every device's coordinate epoch.
                    self.buffers[role] = PoseBuffer()
                    raise ValueError('无效角色、坐标系或源时间')
                if state['tracker_id'] and (identity != state['tracker_id'] or frame != state['frame_id']):
                    self._retired_identities[role].append((state['tracker_id'], state['frame_id']))
                    self._reset(coordinate=True)
                # Network reordering may fill the interpolation buffer but cannot
                # advance freshness or display time. Exact duplicates are ignored.
                if state['source_ns'] == source:
                    return False
                if state['source_ns'] is not None:
                    jump = (source - state['source_ns']) - (source_mono - state['source_monotonic_ns'])
                    if source_mono <= state['source_monotonic_ns']:
                        # A packet from a retired realtime mapping can arrive
                        # after a clock step; it must not restore the old epoch.
                        if abs(jump) > 5_000_000 or source >= state['source_ns']:
                            return False
                    elif abs(jump) > 5_000_000:
                        self._reset()
                valid = bool(message.pose_valid)
                reason = '' if valid else '无效位姿、坐标系或源时间'
                pose = None
                if valid:
                    try:
                        p, q = message.pose.position, message.pose.orientation
                        pose = RigidTransform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
                    except (ValueError, TypeError, AttributeError):
                        valid, reason = False, '非有限位姿或无效四元数'
                observation = PoseSample(role, identity, frame, source, monotonic, str(self.time_epoch),
                                         str(self.coordinate_epoch), pose, valid)
                result = self.buffers[role].append(observation)
                if not result.accepted:
                    return False
                if state['source_ns'] is not None and source < state['source_ns']:
                    return True
                previous_source = state['source_ns']
                state.update(tracker_id=identity, frame_id=frame, source_ns=source, source_monotonic_ns=source_mono,
                             received_ns=monotonic, valid=valid, reason=reason)
                if valid:
                    state['pose'] = pose
                    trace = state['trail']
                    while trace and source - trace[0][0] > 3_000_000_000:
                        trace.popleft()
                    if not trace or state['broken'] or source-trace[-1][0] >= 33_000_000:
                        state['broken'] = state['broken'] or (previous_source is not None and source-previous_source > 250_000_000)
                        trace.append((source, list(pose.translation_m), state['broken']))
                    state['broken'] = False
                else:
                    state['broken'] = True
                return True
            except (ValueError, TypeError, AttributeError, OverflowError) as exc:
                state.update(valid=False, broken=True, reason=str(exc)[:256])
                return False

    def set_interest(self, viewer_id, interest):
        with self._lock:
            now = self.monotonic_ns()
            self._interests = {k: v for k, v in self._interests.items() if now-v[0] < 2_000_000_000}
            if len(self._interests) < 32 or viewer_id in self._interests:
                self._interests[viewer_id] = (now, set(interest))

    def has_interest(self, layer):
        with self._lock:
            now = self.monotonic_ns()
            return any(now-stamp < 2_000_000_000 and layer in layers for stamp, layers in self._interests.values())

    def description(self):
        return dict(world_frame=self.config.world_frame, map_identity=self.config.map_identity,
                    config_revision=self.config.config_revision, display_from_world=self.config.display_from_world.as_dict(),
                    cameras=self.camera_registry.describe() if self.camera_registry else [],
                    tracker_roles=list(ROLES), time_mode='monitor', units='m', quaternion_order='xyzw')

    def _align_cloud_scene(self, trackers, cameras, now):
        if self.cloud_supervisor is None or not self.has_interest('cloud'):
            return None, None
        headers = self.cloud_supervisor.latest_headers()
        generation = dict(manager_instance_id=self.manager_instance_id, scene_epoch=self.scene_epoch,
                          time_epoch=self.time_epoch, coordinate_epoch=self.coordinate_epoch, sync_epoch=self.sync_epoch)
        matching = {}
        for camera in cameras:
            header = headers.get(camera['id'])
            if (not header or any(header.get(k) != v for k, v in generation.items())
                    or header.get('geometry_revision') != camera.get('geometry_revision')
                    or header.get('geometry_sha256') != camera.get('geometry_sha256')
                    or camera.get('registration_state') != 'ready'
                    or not isinstance(header.get('frame_key'), dict)):
                continue
            frame_time = header['frame_key'].get('scene_time_ns')
            if type(frame_time) is not int or not 0 <= now-frame_time <= 1_000_000_000:
                continue
            matching[camera['id']] = header
        if not matching:
            return None, None
        scene_time = max(h['frame_key']['scene_time_ns'] for h in matching.values())
        matching = {key: header for key, header in matching.items() if header['frame_key']['scene_time_ns'] == scene_time}
        reference = next(iter(matching.values()))['frame_key'].get('reference_camera')
        for camera in cameras:
            header = matching.get(camera['id'])
            if header is None:
                camera.update(position_m=None, quaternion_xyzw=None, state='waiting', reason='当前时间组缺少可用图像或几何')
                continue
            pose = RigidTransform.from_dict(header['world_from_color'])
            age = max(0., (now-header['source_realtime_ns'])/1e9)
            camera.update(position_m=list(pose.translation_m), quaternion_xyzw=list(pose.quaternion_xyzw),
                          source_realtime_ns=header['source_realtime_ns'], age_s=age,
                          frame_key=deepcopy(header['frame_key']), state='fresh' if age <= .25 else 'delayed')
        for tracker in trackers:
            selected = self.buffers[tracker['role']].select(scene_time)
            pose = selected.pose
            tracker.update(position_m=list(pose.translation_m) if pose else None,
                           quaternion_xyzw=list(pose.quaternion_xyzw) if pose else None,
                           source_realtime_ns=scene_time, age_s=max(0., (now-scene_time)/1e9), valid=pose is not None,
                           state=('fresh' if now-scene_time <= 250_000_000 else 'delayed') if pose else 'invalid',
                           reason=selected.reason, before_ns=selected.before_ns, after_ns=selected.after_ns,
                           gap_ns=selected.gap_ns)
            segments = []
            for stamp, point, broken in self._states[tracker['role']]['trail']:
                if not scene_time-3_000_000_000 <= stamp <= scene_time:
                    continue
                if broken or not segments:
                    segments.append([])
                segments[-1].append(point)
            tracker['trail_segments'] = segments
        return scene_time, reference

    def snapshot(self):
        with self._lock:
            now, mono = self.realtime_ns(), self.monotonic_ns()
            trackers = []
            for role, label in zip(ROLES, LABELS):
                s = self._states[role]
                age = (max(0, mono-s['received_ns'], now-s['source_ns'])/1e9
                       if s['received_ns'] is not None else None)
                state = ('waiting' if age is None else 'offline' if age > 1. else
                         'invalid' if not s['valid'] else 'delayed' if age > .25 else 'fresh')
                segments = []
                for stamp, point, broken in s['trail']:
                    if s['source_ns'] is not None and s['source_ns']-stamp > 3_000_000_000:
                        continue
                    if broken or not segments:
                        segments.append([])
                    segments[-1].append(point)
                pose = s['pose']
                trackers.append(dict(id=role, role=role, label=label, tracker_id=s['tracker_id'],
                    frame_id=s['frame_id'] or self.config.world_frame, source_realtime_ns=s['source_ns'],
                    position_m=list(pose.translation_m) if pose else None,
                    quaternion_xyzw=list(pose.quaternion_xyzw) if pose else None,
                    state=state, age_s=age, valid=state in ('fresh', 'delayed'),
                    reason=s['reason'], trail_segments=segments))
            cameras = self.camera_registry.camera_entities(trackers, now_ns=now) if self.camera_registry else []
            scene_time, reference = self._align_cloud_scene(trackers, cameras, now)
            diagnostics = list(self._diagnostics)
            if self.has_interest('cloud') and scene_time is None:
                status = self.cloud_supervisor.status() if self.cloud_supervisor and hasattr(self.cloud_supervisor, 'status') else {}
                diagnostics.append('点云：'+(status.get('error') or '等待同帧图像、完整几何与有效插值位姿'))
            self.sequence += 1
            return deepcopy(dict(manager_instance_id=self.manager_instance_id, scene_epoch=self.scene_epoch,
                sequence=self.sequence, mode='live', world_frame=self.config.world_frame,
                time_epoch=self.time_epoch, coordinate_epoch=self.coordinate_epoch, sync_epoch=self.sync_epoch,
                time_mode='aligned' if scene_time is not None else 'monitor', scene_time_ns=scene_time, generated_realtime_ns=now,
                config_revision=self.config.config_revision, geometry_revision=self.coordinate_epoch,
                active_reference_camera=reference, map_identity=self.config.map_identity,
                display_from_world=self.config.display_from_world.as_dict(), trackers=trackers,
                cameras=cameras, diagnostics=diagnostics))
