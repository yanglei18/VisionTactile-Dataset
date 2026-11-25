"""On-demand, source-matched RGB-D samples for points on a frozen glove image."""
from collections import OrderedDict
from dataclasses import dataclass, field
from functools import partial
from threading import RLock
import time

import cv2
import numpy as np

from vt_capture_core.spatial.transforms import RigidTransform
from ..spatial.camera_geometry import _intrinsics
from ..spatial.cloud_builder import build_cloud, project_color
from ..spatial.ros_adapter import rotation_from_column_major
from .hand_fusion import color_time, stamp_ns as header_stamp_ns


_MAX_PIXELS = 2_097_152
_HISTORY = 10
_PAIR_WAIT = '等待与彩色画面同一帧的深度，请保持手不动。'
_NO_DEPTH = '这个位置没有可靠深度，请稍微挪到同一处手套的内侧再点。'


@dataclass
class FrozenDepth:
    """One retained exact frame. Build its color-camera Z buffer only when clicked."""
    serial: str
    depth_image: object
    geometry: dict
    matrix: np.ndarray
    distortion: np.ndarray
    size: tuple
    _color_z: np.ndarray = field(default=None, init=False, repr=False)

    def _register(self):
        width, height = self.size
        try:
            xyz, _ = build_cloud(self.depth_image, None, self.geometry, stride=1, max_points=_MAX_PIXELS,
                                 min_depth_m=.05, max_depth_m=5.)
            points = np.frombuffer(xyz, dtype='<f4').reshape(-1, 3)
            points = points[points[:, 2] > 0]
            uv = project_color(points, self.matrix, self.distortion)
            good = (np.isfinite(uv).all(axis=1) & (uv[:, 0] > -.5) & (uv[:, 0] < width-.5)
                    & (uv[:, 1] > -.5) & (uv[:, 1] < height-.5))
            uv = np.rint(uv[good]).astype(np.intp)
            color_z = np.full((height, width), np.inf, dtype=np.float32)
            # Distinct depth pixels can project onto the same color pixel. Keep the front surface.
            np.minimum.at(color_z, (uv[:, 1], uv[:, 0]), points[good, 2])
        except (TypeError, ValueError, KeyError, cv2.error) as exc:
            raise ValueError('当前深度图的单位、格式或相机参数无法使用，请重新采集画面。') from exc
        self._color_z = color_z

    def point_at(self, pixel):
        click = np.asarray(pixel, dtype=np.float64)
        width, height = self.size
        if (click.shape != (2,) or not np.isfinite(click).all()
                or not (0 <= click[0] < width and 0 <= click[1] < height)):
            raise ValueError('请点在相机画面内。')
        if self._color_z is None:
            self._register()
        x, y = np.rint(click).astype(int)
        x0, y0 = max(0, x-3), max(0, y-3)
        patch = self._color_z[y0:y+4, x0:x+4]
        yy, xx = np.indices(patch.shape)
        radius_sq = (xx + x0 - click[0])**2 + (yy + y0 - click[1])**2
        values = patch[np.isfinite(patch) & (radius_sq <= 9)]
        if len(values) < 8 or not np.any(np.isfinite(patch) & (radius_sq <= 1.5**2)):
            raise ValueError(_NO_DEPTH)
        z = float(np.median(values))
        # A second surface in the neighborhood indicates an occlusion boundary.
        lo, hi = np.percentile(values, (10, 90))
        if hi - lo > max(.008, .012*z):
            raise ValueError('这个点落在深度边缘，请点在手套表面中央。')
        ray = cv2.undistortPoints(click.reshape(1, 1, 2), self.matrix, self.distortion).ravel()
        if not np.isfinite(ray).all():
            raise ValueError(_NO_DEPTH)
        return np.array([ray[0]*z, ray[1]*z, z])


class MountDepthCapture:
    """Temporary subscriptions attached to the running optical node; owns no physical device."""

    def __init__(self, node, camera):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from realsense2_camera_msgs.msg import Extrinsics
        from realsense2_camera_msgs.srv import DeviceInfo
        from sensor_msgs.msg import CameraInfo, Image
        from vt_camera_msgs.msg import CameraFrameTiming

        self.node, self.camera = node, camera
        self._lock = RLock()
        self._closed = False
        self._topics = {key: f'/{camera}/{suffix}' for key, suffix in {
            'rgb': 'color/image_raw', 'image': 'depth/image_rect_raw', 'info': 'depth/camera_info',
            'transform': 'extrinsics/depth_to_color', 'timing': 'frame_timing'}.items()}
        self._rows = {key: OrderedDict() for key in ('image', 'info', 'timing')}
        self._static_info = self._transform = None
        self._device = self._pending = self._timer = self._client = None
        self._last_probe = 0.
        self._service = f'/{camera}/device_info'
        self._subscriptions = {}
        self._latched_info = False
        streaming = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        try:
            for key, msg_type in (('image', Image), ('info', CameraInfo), ('timing', CameraFrameTiming),
                                  ('transform', Extrinsics)):
                qos = latched if key == 'transform' else streaming
                sub = node.create_subscription(msg_type, self._topics[key], partial(self._receive, key), qos)
                self._subscriptions[key] = sub
            self._client = node.create_client(DeviceInfo, self._service)
            self._timer = node.create_timer(.25, self._probe_device)
        except Exception:
            self.close()
            raise

    def _endpoint(self, key):
        endpoints = self.node.get_publishers_info_by_topic(self._topics[key])
        if len(endpoints) != 1:
            return None
        endpoint = endpoints[0]
        return (endpoint.node_name, endpoint.node_namespace, bytes(endpoint.endpoint_gid))

    def _receive(self, key, msg):
        owner = self._endpoint(key)
        with self._lock:
            if self._closed or owner is None:
                return
            if key == 'transform':
                try:
                    transform = RigidTransform(msg.translation, rotation_from_column_major(msg.rotation))
                    self._transform = (owner, transform.as_dict())
                except (ValueError, TypeError):
                    self._transform = None
                return
            stamp = header_stamp_ns(msg.header)
            if key == 'info' and (stamp == 0 or self._latched_info):
                self._static_info = (owner, msg, time.monotonic())
                return
            if stamp <= 0:
                return
            if key == 'image' and (msg.width * msg.height > _MAX_PIXELS or len(msg.data) > _MAX_PIXELS*8):
                return
            rows = self._rows[key]
            if rows and stamp < next(reversed(rows)):
                # Stamp reuse after a driver/clock restart. A single stream must not
                # match packets retained from an older collection interval.
                for history in self._rows.values():
                    history.clear()
            if stamp in rows:
                rows[stamp] = None  # A repeated frame stamp cannot identify a unique observation.
                return
            rows[stamp] = (owner, msg, time.monotonic())
            if len(rows) > _HISTORY:
                rows.popitem(last=False)

    def _refresh_info_subscription(self):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo

        endpoints = self.node.get_publishers_info_by_topic(self._topics['info'])
        if len(endpoints) != 1:
            return
        offered = endpoints[0].qos_profile
        latched = offered.durability == DurabilityPolicy.TRANSIENT_LOCAL
        if latched != self._latched_info:
            self._latched_info = latched
            self._static_info = None
            self._rows['info'].clear()
            self.node.destroy_subscription(self._subscriptions['info'])
            qos = QoSProfile(depth=3, durability=offered.durability,
                             reliability=offered.reliability if latched else ReliabilityPolicy.BEST_EFFORT)
            self._subscriptions['info'] = self.node.create_subscription(
                CameraInfo, self._topics['info'], partial(self._receive, 'info'), qos)

    def _probe_device(self):
        from realsense2_camera_msgs.srv import DeviceInfo
        from rclpy.node import NodeNameNonExistentError

        with self._lock:
            if self._closed:
                return
            self._refresh_info_subscription()
            source = self._endpoint('rgb')
            if source is None:
                self._device = None
                return
            now = time.monotonic()
            if self._pending is not None:
                pending_source, future = self._pending
                if not future.done() and now - self._last_probe <= 2. and source == pending_source:
                    return
                self._pending = None
                if future.done() and not future.cancelled() and source == pending_source:
                    try:
                        reply = future.result()
                        if reply and reply.serial_number and 'realsense' in reply.device_name.lower():
                            self._device = (source, reply.serial_number, reply.device_name, now)
                            return
                    except (AttributeError, RuntimeError):
                        pass
                elif not future.done():
                    self._client.remove_pending_request(future)
                self._device = None
            if self._device and self._device[0] == source and now-self._device[3] < 2.:
                return
            if not self._client.service_is_ready():
                return
            # Confirm that this RealSense-specific service belongs to the image publisher.
            try:
                matches = [owner for owner in self.node.get_node_names_and_namespaces()
                           if any(name == self._service and 'realsense2_camera_msgs/srv/DeviceInfo' in types
                                  for name, types in self.node.get_service_names_and_types_by_node(*owner))]
            except (RuntimeError, NodeNameNonExistentError):
                # A node may have gone away between the two graph queries.
                return
            if matches == [source[:2]]:
                self._last_probe = now
                self._pending = (source, self._client.call_async(DeviceInfo.Request()))

    def freeze(self, stamp_ns, camera_matrix, distortion, size):
        """Freeze only a matched, still-fresh observation. Its geometry survives stream teardown."""
        with self._lock:
            if self._closed:
                raise ValueError('本次深度采集已结束。')
            sources = {key: self._endpoint(key) for key in self._topics}
            if any(source is None for source in sources.values()):
                raise ValueError('等候头部相机的彩色、深度、时序和相机参数。')
            driver = sources['rgb'][:2]
            if any(sources[key][:2] != driver for key in ('image', 'info', 'transform')):
                raise ValueError('彩色和深度的数据来源不一致，请检查相机连接。')
            if not self._transform or self._transform[0] != sources['transform']:
                raise ValueError('等待相机深度与彩色画面的对齐参数。')
            rows = {key: history.get(stamp_ns) for key, history in self._rows.items()}
            if (stamp_ns not in self._rows['info'] and self._static_info
                    and header_stamp_ns(self._static_info[1].header) <= stamp_ns):
                rows['info'] = self._static_info
            if any(row is None or row[0] != sources[key] for key, row in rows.items()):
                raise ValueError(_PAIR_WAIT)
            if any(time.monotonic()-rows[key][2] > .55 for key in ('image', 'timing')):
                raise ValueError(_PAIR_WAIT)
            depth, info, timing = (rows[key][1] for key in ('image', 'info', 'timing'))
            flags = timing.VALID_HOST_REALTIME | timing.VALID_FRAME_NUMBER
            if (color_time(timing, self.camera) is None or timing.depth_validity_flags & flags != flags
                    or timing.depth_host_realtime_ns <= 0):
                raise ValueError(_PAIR_WAIT)
            if not self._device or self._device[0] != sources['rgb'] or time.monotonic()-self._device[3] > 4.:
                raise ValueError('正在确认头部相机的设备身份和深度单位。')
            _, serial, model, _ = self._device
            if serial != timing.serial_number or not timing.camera_model or timing.camera_model.lower() not in model.lower():
                raise ValueError('彩色与深度帧的相机型号或序列号不匹配。')
            frame_id = self.camera + '_depth_optical_frame'
            try:
                lens = _intrinsics(info, frame_id)
                if (depth.header.frame_id != frame_id or (lens['width'], lens['height']) != (depth.width, depth.height)
                        or depth.encoding not in ('16UC1', '32FC1')):
                    raise ValueError()
                width, height = size
                k, d = np.asarray(camera_matrix, dtype=float), np.asarray(distortion, dtype=float).reshape(-1)
                if (min(width, height) < 1 or width*height > _MAX_PIXELS or k.shape != (3, 3)
                        or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
                        or d.size not in (0, 4, 5, 8, 12, 14) or not np.isfinite(d).all()):
                    raise ValueError()
            except (ValueError, TypeError, AttributeError) as exc:
                raise ValueError('彩色或深度的分辨率、内参或图像格式无效。') from exc
            # The verified RealSense ROS driver publishes 16UC1 in normalized
            # millimetres, independently of the physical device's depth_scale.
            geometry = dict(config={'depth_frame_id': frame_id, 'depth_image_rectified': True},
                            depth_intrinsics=lens, depth_to_color=self._transform[1],
                            depth_units_m=.001, depth_units_source='realsense_ros_z16_mm')
            return FrozenDepth(serial, depth, geometry, k.copy(), d.copy(), (width, height))

    def close(self):
        with self._lock:
            self._closed = True
            for sub in self._subscriptions.values():
                self.node.destroy_subscription(sub)
            self._subscriptions.clear()
            if self._timer:
                self.node.destroy_timer(self._timer)
                self._timer = None
            if self._client:
                self.node.destroy_client(self._client)
                self._client = None
            self._device = self._pending = self._transform = self._static_info = None
            for history in self._rows.values():
                history.clear()
