"""Live identity and typed calibration publication, independent of scene visibility."""
from __future__ import annotations

from copy import deepcopy
import re
import threading
import time
from types import SimpleNamespace

from vt_capture_core.spatial.transforms import RigidTransform
from ..spatial.ros_adapter import SpatialRosAdapter


class TrackerSourceGraph:
    """Bound graph inspection to four checks per second for each role topic."""
    def __init__(self, query, *, clock=time.monotonic, refresh_s=.25):
        self.query, self.clock, self.refresh_s = query, clock, refresh_s
        self._cache = {}
        self._lock = threading.RLock()

    def valid(self, topic):
        with self._lock:
            now = self.clock()
            cached = self._cache.get(topic)
            if cached is not None and 0 <= now - cached[0] < self.refresh_s:
                return cached[1]
            try:
                valid = len(self.query(topic)) == 1
            except (RuntimeError, ValueError, TypeError, AttributeError):
                valid = False
            if topic not in self._cache and len(self._cache) >= 64:
                self._cache.pop(next(iter(self._cache)))
            self._cache[topic] = (now, valid)
            return valid


class CalibrationIdentity:
    """Require recent DeviceInfo proof and a matching, healthy Tracker stream."""
    def __init__(self, registry, trackers, *, clock=time.time_ns, tracker_ttl_s=1., world_frame='vive_map'):
        self.registry, self.clock = registry, clock
        self.trackers = {entry['role']: dict(entry) for entry in trackers}
        self.world_frame, self.tracker_ttl_ns = world_frame, int(tracker_ttl_s * 1e9)
        self._observations = {}
        self._lock = threading.RLock()

    def observe_tracker(self, role, sample, *, source_valid=True):
        now = self.clock()
        valid = False
        try:
            identifier = sample.tracker_id
            valid = (source_valid and role in self.trackers and sample.role == role
                     and isinstance(identifier, str) and re.fullmatch(r'[0-9a-f]{64}', identifier) is not None
                     and sample.header.frame_id == self.world_frame and sample.pose_valid
                     and int(sample.tracking_status) & 0x0F == 2 and sample.host_monotonic_ns > 0
                     and 0 <= now - sample.host_realtime_ns <= self.tracker_ttl_ns)
            if valid:
                p, q = sample.pose.position, sample.pose.orientation
                RigidTransform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        except (ValueError, TypeError, AttributeError):
            valid = False
        with self._lock:
            if valid:
                self._observations[role] = dict(tracker_id=identifier, observed_ns=now, source_ns=sample.host_realtime_ns)
            else:
                self._observations.pop(role, None)

    def invalidate_tracker(self, role):
        with self._lock:
            self._observations.pop(role, None)

    def expected_tracker_id(self, role):
        with self._lock:
            value = self._observations.get(role)
            if value and 0 <= self.clock() - value['source_ns'] <= self.tracker_ttl_ns:
                return value['tracker_id']
            return ''

    def identity(self, document):
        try:
            camera, tracker = document['camera'], document['tracker']
            observed = self.registry.geometry(camera['name'])
            proof, binding = observed['identity'], observed['config']
            if (not proof['verified'] or proof['observed_ns'] is None
                    or not 0 <= self.clock() - proof['observed_ns'] <= self.registry.identity_ttl_ns):
                return False, 'camera_identity_unverified'
            if camera['serial'] != proof['serial'] or camera['model'] != binding.get('model') or camera['frame_id'] != binding['color_frame_id']:
                return False, 'camera_identity_mismatch'
            current_tracker = self.expected_tracker_id(tracker['role'])
            if not current_tracker:
                return False, 'tracker_unavailable'
            if current_tracker != tracker['tracker_id']:
                return False, 'tracker_identity_mismatch'
            if tracker['frame_id'] != self.trackers[tracker['role']]['frame_id']:
                return False, 'tracker_frame_mismatch'
            if document['frames']['world'] != self.world_frame:
                return False, 'world_frame_mismatch'
            return True, ''
        except (KeyError, TypeError, ValueError):
            return False, 'unknown_calibration_identity'


def calibration_message(message_type, document, manifest, *, identity_valid, reason, now_ns=None, instance_id=''):
    now_ns = time.time_ns() if now_ns is None else now_ns
    message = message_type()
    message.schema_version = 1
    message.manager_instance_id = instance_id
    camera, tracker, transform = document['camera'], document['tracker'], document['transform']
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(now_ns, 1_000_000_000)
    message.header.frame_id = tracker['frame_id']
    message.camera_name, message.camera_model, message.camera_serial = camera['name'], camera['model'], camera['serial']
    message.camera_frame_id = camera['frame_id']
    message.tracker_role, message.tracker_id = tracker['role'], tracker['tracker_id']
    message.world_frame_id = document['frames']['world']
    message.revision, message.sha256 = manifest['revision'], manifest['sha256']
    message.activated_ns = manifest['activated_ns']
    message.valid, message.reason = bool(identity_valid), str(reason)[:512]
    # A late subscriber must never treat an old identity assertion as timeless.
    message.valid_until_ns = now_ns + 2_000_000_000 if identity_valid else 0
    message.quality_grade = document.get('quality', {}).get('grade', '')
    p, q = message.tracker_from_camera.translation, message.tracker_from_camera.rotation
    p.x, p.y, p.z = map(float, transform['translation_m'])
    q.x, q.y, q.z, q.w = map(float, transform['quaternion_xyzw'])
    return message


class CalibrationRosAdapter:
    """Reuse audited camera source verification without starting a spatial view."""
    def __init__(self, node, cameras, trackers, *, world_frame='vive_map', instance_id=''):
        from vt_tracker_msgs.msg import TrackerSample
        from vt_camera_msgs.msg import CameraTrackerCalibration
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.node, self.instance_id = node, instance_id
        cameras = [deepcopy(camera) for camera in cameras]
        for camera in cameras:
            camera.setdefault('id', camera.get('name'))
            camera.setdefault('tracker_role', 'torso')
        store = SimpleNamespace(config=SimpleNamespace(cameras=cameras, world_frame=world_frame), camera_registry=None)
        self._camera_adapter = SpatialRosAdapter(node, store)
        self._identity = CalibrationIdentity(store.camera_registry, trackers, world_frame=world_frame)
        self._message_type = CameraTrackerCalibration
        self._source_graph = TrackerSourceGraph(node.get_publishers_info_by_topic)
        self._tracker_topics = {}
        self._publishers = {}
        self._subscriptions = []
        self._last_publications = {}
        self._lock = threading.RLock()
        self._closed = False
        self._latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for tracker in trackers:
            role = tracker['role']
            topic = tracker.get('sample_topic', f'/vive/{role}/sample')
            self._tracker_topics[role] = topic
            self._subscriptions.append(node.create_subscription(TrackerSample, topic,
                lambda message, r=role, t=topic: self._sample(r, t, message), qos))
        self._timer = node.create_timer(1., self._refresh)

    @property
    def registry(self):
        return self._camera_adapter.registry

    def _sample(self, role, topic, message):
        self._identity.observe_tracker(role, message, source_valid=self._source_graph.valid(topic))

    def expected_tracker_id(self, role):
        topic = self._tracker_topics.get(role)
        if topic is None or not self._source_graph.valid(topic):
            self._identity.invalidate_tracker(role)
        return self._identity.expected_tracker_id(role)

    def identity(self, document):
        self.expected_tracker_id(document.get('tracker', {}).get('role'))
        return self._identity.identity(document)

    def publish(self, document, manifest, identity_valid, reason):
        with self._lock:
            if self._closed:
                raise RuntimeError('calibration publisher is closed')
            name = document['camera']['name']
            if name not in self._publishers:
                self._publishers[name] = self.node.create_publisher(self._message_type,
                    f'/calibration/{name}/tracker_from_camera', self._latched_qos)
            message = calibration_message(self._message_type, document, manifest,
                                          identity_valid=identity_valid, reason=reason, instance_id=self.instance_id)
            self._publishers[name].publish(message)
            self._last_publications[name] = (deepcopy(document), deepcopy(manifest))

    def _refresh(self):
        with self._lock:
            if self._closed:
                return
            for document, manifest in list(self._last_publications.values()):
                valid, reason = self.identity(document)
                self.publish(document, manifest, valid, reason)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._timer.cancel()
            self._camera_adapter.close()
            self.node.destroy_timer(self._timer)
            self.node.destroy_timer(self._camera_adapter.timer)
            for subscription in self._camera_adapter.subscriptions:
                self.node.destroy_subscription(subscription)
            for client in self._camera_adapter.clients.values():
                self.node.destroy_client(client)
            for subscription in self._subscriptions:
                self.node.destroy_subscription(subscription)
            for publisher in self._publishers.values():
                self.node.destroy_publisher(publisher)
            self._subscriptions.clear()
            self._publishers.clear()
