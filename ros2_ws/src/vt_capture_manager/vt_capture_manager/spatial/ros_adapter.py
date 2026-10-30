"""Read-only subscriptions and asynchronous camera identity verification."""
import time
import math
import threading

from vt_capture_core.spatial.transforms import RigidTransform, finite_vector

class SpatialRosAdapter:
    def __init__(self, node, store):
        from .camera_geometry import CameraGeometryRegistry
        from sensor_msgs.msg import CameraInfo
        from realsense2_camera_msgs.msg import Extrinsics
        from realsense2_camera_msgs.srv import DeviceInfo
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.node, self.store = node, store
        # Future done callbacks are executor tasks, outside client callback groups.
        self._identity_lock = threading.RLock()
        self._closed = False
        self._verified_signatures = {}
        self.registry = CameraGeometryRegistry(store.config.cameras, world_frame=store.config.world_frame)
        store.camera_registry = self.registry
        self.subscriptions, self.clients, self.pending = [], {}, {}
        self.cameras = {c['id']: c for c in store.config.cameras}
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        for name, camera in self.cameras.items():
            for stream, key in [('color', 'camera_info_topic'), ('depth', 'depth_camera_info_topic')]:
                if camera.get(key):
                    self.subscriptions.append(node.create_subscription(CameraInfo, camera[key],
                        lambda msg, n=name, s=stream: self._info(n, msg, s), qos))
            if camera.get('depth_to_color_topic'):
                latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.subscriptions.append(node.create_subscription(Extrinsics, camera['depth_to_color_topic'],
                    lambda msg, n=name: self._extrinsics(n, msg), latched))
            service = camera.get('device_info_service')
            if service:
                self.clients[name] = node.create_client(DeviceInfo, service)
        self.timer = node.create_timer(2., self._identity_tick)

    def _info(self, name, message, stream):
        try:
            self.registry.observe_camera_info(name, message, stream=stream)
        except (ValueError, TypeError, AttributeError) as exc:
            self.node.get_logger().warning(f'Spatial {name} {stream} CameraInfo rejected: {exc}', throttle_duration_sec=5.)

    def _extrinsics(self, name, message):
        try:
            camera = self.cameras[name]
            color = self.node.get_publishers_info_by_topic(camera['color_topic'])
            extrinsics = self.node.get_publishers_info_by_topic(camera['depth_to_color_topic'])
            if (len(color) != 1 or len(extrinsics) != 1
                    or (color[0].node_name, color[0].node_namespace) !=
                       (extrinsics[0].node_name, extrinsics[0].node_namespace)):
                self.registry.invalidate_identity(name, 'depth_extrinsics_source_mismatch')
                return
            transform = RigidTransform(message.translation, rotation_from_column_major(message.rotation))
            self.registry.observe_depth_to_color(name, transform.as_dict())
        except (ValueError, TypeError, AttributeError) as exc:
            self.registry.invalidate_depth_geometry(name, str(exc)[:256])
            self.node.get_logger().warning(f'Spatial {name} depth extrinsics rejected: {exc}', throttle_duration_sec=5.)

    def _source_matches(self, camera):
        """A serial response must belong to the node publishing this RGB-D pair."""
        return self._source_signature(camera) is not None

    def _source_signature(self, camera):
        owners = []
        signatures = []
        topics = [camera.get('color_topic'), camera.get('depth_topic'), camera.get('camera_info_topic'), camera.get('depth_camera_info_topic')]
        if 'cloud_topic' in camera:
            topics.append(camera['cloud_topic'])
        for topic in topics:
            if not topic:
                return None
            endpoints = self.node.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1:
                return None
            endpoint = endpoints[0]
            gid = tuple(getattr(endpoint, 'endpoint_gid', ()))
            if not gid or len(gid) > 64:
                return None
            owner = (endpoint.node_name, endpoint.node_namespace)
            owners.append(owner)
            signatures.append((topic, owner, gid))
        if len(set(owners)) != 1:
            return None
        nodes = self.node.get_node_names_and_namespaces()
        if len(nodes) > 2048 or nodes.count(owners[0]) != 1:
            return None
        service_owners = []
        for name, namespace in nodes:
            services = self.node.get_service_names_and_types_by_node(name, namespace)
            if any(service == camera['device_info_service']
                   and 'realsense2_camera_msgs/srv/DeviceInfo' in types for service, types in services):
                service_owners.append((name, namespace))
        # An unrelated node advertising the same service could answer the
        # request. Checking only that the expected node advertises it is unsafe.
        if service_owners != [owners[0]]:
            return None
        return tuple(signatures)

    def _discard_request(self, name):
        previous = self.pending.pop(name, None)
        if previous is not None:
            self.clients[name].remove_pending_request(previous[0])

    def _identity_tick(self):
        with self._identity_lock:
            self._identity_tick_locked()

    def _identity_tick_locked(self):
        from realsense2_camera_msgs.srv import DeviceInfo
        if self._closed:
            return
        for name, client in self.clients.items():
            try:
                signature = self._source_signature(self.cameras[name])
                if signature is None:
                    self._discard_request(name)
                    self._verified_signatures.pop(name, None)
                    self.registry.invalidate_identity(name, 'source_graph_mismatch')
                    continue
                if name in self._verified_signatures and signature != self._verified_signatures[name]:
                    self.registry.invalidate_identity(name, 'source_graph_changed')
                    self._verified_signatures.pop(name, None)
                if name in self.pending:
                    future, started, requested_signature = self.pending[name]
                    if time.monotonic()-started > 4. or requested_signature != signature:
                        self._discard_request(name)
                        self.registry.invalidate_identity(name, 'identity_request_expired')
                    continue
                if client.service_is_ready():
                    future = client.call_async(DeviceInfo.Request())
                    self.pending[name] = (future, time.monotonic(), signature)
                    future.add_done_callback(lambda result, n=name: self._identity_done(n, result))
                else:
                    self.registry.invalidate_identity(name, 'identity_service_unavailable')
            except (RuntimeError, ValueError, AttributeError, TypeError) as exc:
                self.registry.invalidate_identity(name, str(exc)[:256])

    def _identity_done(self, name, future):
        with self._identity_lock:
            self._identity_done_locked(name, future)

    def _identity_done_locked(self, name, future):
        pending = self.pending.get(name)
        if self._closed or pending is None or pending[0] is not future:
            return
        self.pending.pop(name, None)
        try:
            signature = self._source_signature(self.cameras[name])
            if future.cancelled() or signature is None or signature != pending[2]:
                self.registry.invalidate_identity(name, 'source_graph_changed')
                return
            self.registry.observe_device_info(name, future.result(), observed_ns=time.time_ns())
            self._verified_signatures[name] = signature
        except (ValueError, RuntimeError, AttributeError, TypeError) as exc:
            self.registry.invalidate_identity(name, str(exc)[:256])

    def close(self):
        with self._identity_lock:
            self._closed = True
            self.timer.cancel()
            for name in list(self.pending):
                self._discard_request(name)


def rotation_from_column_major(values):
    """Validate the RealSense Extrinsics SO(3) matrix and convert to xyzw."""
    values = finite_vector(values, 9, 'rotation')
    r = [[values[col*3+row] for col in range(3)] for row in range(3)]
    for a in range(3):
        for b in range(3):
            if abs(sum(r[i][a]*r[i][b] for i in range(3))-(1. if a == b else 0.)) > 1e-5:
                raise ValueError('extrinsics rotation must be orthonormal')
    determinant = (r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
                   -r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
                   +r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]))
    if abs(determinant-1.) > 1e-5:
        raise ValueError('extrinsics rotation must have determinant +1')
    trace = r[0][0]+r[1][1]+r[2][2]
    if trace > 0:
        scale = math.sqrt(trace+1.)*2
        return ((r[2][1]-r[1][2])/scale, (r[0][2]-r[2][0])/scale, (r[1][0]-r[0][1])/scale, scale/4)
    index = max(range(3), key=lambda i: r[i][i])
    j, k = (index+1)%3, (index+2)%3
    scale = math.sqrt(1.+r[index][index]-r[j][j]-r[k][k])*2
    q = [0., 0., 0., 0.]
    q[index], q[j], q[k], q[3] = scale/4, (r[j][index]+r[index][j])/scale, (r[k][index]+r[index][k])/scale, (r[k][j]-r[j][k])/scale
    return tuple(q)
