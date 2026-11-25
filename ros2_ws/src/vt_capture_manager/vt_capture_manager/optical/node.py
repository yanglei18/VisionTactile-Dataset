"""Observe metric hand-mounted cubes from existing raw ROS images and intrinsics."""
from collections import deque
import hashlib
import json
import re
import threading
import time
import uuid

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from vt_camera_msgs.msg import HandCubeObservation

from ..spatial.camera_geometry import _intrinsics
from .cube import CubeDetector, CubeLayout
from .identity import BilateralTracker
from .lease import domain_lease

HANDS = ("left", "right")
PREFIX = "/hand_cube_tracking"
# Bound waiting for a current image, including decode/detection, in host monotonic time.
BIND_MAX_AGE_S = 0.25


def _calibration(msg, frame):
    geometry = _intrinsics(msg, frame)
    model, coefficients = geometry["distortion_model"], geometry["d"]
    valid_sizes = {"plumb_bob": (4, 5), "rational_polynomial": (8, 12, 14)}
    if len(coefficients) not in valid_sizes.get(model, ()):
        raise ValueError("unsupported raw RGB distortion model or coefficient count")
    digest = hashlib.sha256(json.dumps(geometry, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return digest, msg.width, msg.height, np.reshape(geometry["k"], (3, 3)), np.asarray(coefficients)


def _stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class HandCubeNode(Node):
    def __init__(self, *, frame_listener=None, **kwargs):
        context = kwargs.get("context") or rclpy.get_default_context()
        self._lease = domain_lease(context.get_domain_id(), "hand_cube_tracking")
        try:
            super().__init__("hand_cube_tracking", **kwargs)
        except BaseException:
            self._lease.close()
            raise
        self._worker = None
        self._condition = threading.Condition()
        self._closed = False
        try:
            self._configure(frame_listener)
        except BaseException:
            self.destroy_node()
            raise

    @property
    def instance_id(self):
        return self._instance

    def _configure(self, listener):
        self.camera_name = self.declare_parameter("camera_name", "d436").value
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", self.camera_name):
            raise ValueError("camera_name must be a single ROS camera name")
        self.optical_frame = self.camera_name + "_color_optical_frame"
        layout_path = self.declare_parameter("layout_path", "").value
        layout = CubeLayout.from_file(layout_path) if layout_path else CubeLayout.default()
        self._layout_hash = layout.sha256
        self._detector = CubeDetector(layout)
        self._tracker = BilateralTracker()
        self._bridge = CvBridge()
        self._listener = listener
        self._instance = uuid.uuid4().hex
        self._camera = None
        self._info_stamps = deque(maxlen=8)
        self._static_since = None
        self._info_owner = self._image_owner = None
        self._last_arrival = self._last_stamp = None
        self._image_index = self._skipped = self._epoch = 0
        self._processed = self._ready_until = 0
        self._pending = None
        self._outputs = {hand: self.create_publisher(HandCubeObservation, f"{PREFIX}/{hand}/observation",
                                                     QoSProfile(depth=10)) for hand in HANDS}
        for hand in HANDS:
            self.create_service(Trigger, f"{PREFIX}/bind_{hand}", self._bind_service(hand))
        self.create_service(Trigger, f"{PREFIX}/reset", self._reset_service)
        # Head-camera drivers publish CameraInfo with their raw color images.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        camera = f"/{self.camera_name}/color"
        self._info_topic, self._image_topic = camera + "/camera_info", camera + "/image_raw"
        self._latched_info_subscription = False
        self._info_subscription = self.create_subscription(CameraInfo, self._info_topic, self._receive_info, qos)
        self.create_timer(.2, self._refresh_info_qos)
        self.create_subscription(Image, self._image_topic, self._receive_image, qos)
        self._worker = threading.Thread(target=self._run, name="cube-image-detection", daemon=True)
        self._worker.start()

    def _invalidate(self):
        """All callers hold the condition. Discard any work from the old binding epoch."""
        self._epoch += 1
        self._tracker.reset()
        self._processed = self._ready_until = 0
        if self._pending is not None:
            self._skipped += 1
            self._pending = None
        self._condition.notify_all()

    def _refresh_info_qos(self):
        entries = self.get_publishers_info_by_topic(self._info_topic)
        if len(entries) != 1:
            return
        offered = entries[0].qos_profile
        latched = offered.durability == DurabilityPolicy.TRANSIENT_LOCAL
        if self._latched_info_subscription == latched:
            return
        # A volatile subscription cannot retrieve a one-time CameraInfo
        # published before the workbench opened. Match the actual publisher.
        qos = QoSProfile(depth=1, durability=offered.durability,
                         reliability=offered.reliability if latched else ReliabilityPolicy.BEST_EFFORT)
        self.destroy_subscription(self._info_subscription)
        self._info_subscription = self.create_subscription(CameraInfo, self._info_topic, self._receive_info, qos)
        self._latched_info_subscription = latched

    def _source_owner(self, topic):
        # Jazzy rclpy MessageInfo omits publisher_gid. With a unique graph
        # endpoint we can detect driver restarts and reject mixed publishers.
        entries = self.get_publishers_info_by_topic(topic)
        if len(entries) == 1:
            entry = entries[0]
            return ((entry.node_namespace, entry.node_name), bytes(entry.endpoint_gid),
                    entry.qos_profile.durability)
        return None

    def _receive_info(self, msg):
        try:
            camera = _calibration(msg, self.optical_frame)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"Rejected color CameraInfo: {exc}", throttle_duration_sec=5.)
            camera = None
        owner = self._source_owner(self._info_topic)
        if owner is None:
            camera = None
        with self._condition:
            old = self._camera[0] if self._camera is not None else None
            new = camera[0] if camera is not None else None
            if self._info_owner != owner or old != new:
                if self._camera is not None:
                    self._invalidate()
                self._info_stamps.clear()
                self._static_since = None
            self._camera, self._info_owner = camera, owner
            stamp = _stamp_ns(msg.header)
            if stamp == 0 or (owner and owner[2] == DurabilityPolicy.TRANSIENT_LOCAL):
                self._static_since = stamp if self._static_since is None else min(stamp, self._static_since)
            else:
                self._static_since = None
            self._info_stamps.append(stamp)
            self._condition.notify_all()

    def _matched_camera(self, owner, stamp):
        if (owner and self._info_owner and owner[0] == self._info_owner[0]
                and ((self._static_since is not None and stamp >= self._static_since) or stamp in self._info_stamps)):
            return self._camera
        return None

    def _receive_image(self, msg):
        arrival = time.monotonic()
        stamp = _stamp_ns(msg.header)
        owner = self._source_owner(self._image_topic)
        with self._condition:
            changed_owner = self._image_owner is not None and self._image_owner != owner
            if changed_owner:
                # Require fresh intrinsics after the camera publisher restarts.
                self._camera = None
            if changed_owner or (self._last_arrival is not None and arrival - self._last_arrival > BIND_MAX_AGE_S):
                self._invalidate()
                if changed_owner:
                    self._last_stamp = None
            if self._last_stamp is not None and stamp < self._last_stamp:
                self._invalidate()
                self._camera = None
            if stamp == self._last_stamp:
                return
            self._last_arrival, self._last_stamp, self._image_owner = arrival, stamp, owner
            self._image_index += 1
            if self._pending is not None:
                self._skipped += 1
            self._pending = (msg, owner, self._epoch, self._image_index, arrival)
            self._condition.notify_all()

    def _source_is_current(self, stamp):
        # RGB Image.header uses the ROS clock. During bag playback, use the
        # bag's /clock. Monotonic arrival bounds still apply if ROS time pauses.
        age_ns = self.get_clock().now().nanoseconds - stamp
        return -100_000_000 <= age_ns <= int(BIND_MAX_AGE_S * 1e9)

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                msg, owner, epoch, index, arrival = self._pending
                self._pending = None
                stamp = _stamp_ns(msg.header)
                # Images and per-frame intrinsics use two DDS topics. Join by
                # source stamp even when the CameraInfo callback comes later.
                if self._matched_camera(owner, stamp) is None:
                    self._condition.wait_for(lambda: self._closed or self._epoch != epoch
                        or self._pending is not None or self._matched_camera(owner, stamp) is not None,
                        timeout=.035)
                if self._closed:
                    return
                camera = self._matched_camera(owner, stamp)
                if self._epoch != epoch:
                    self._skipped += 1
                    continue
            image, observations, failure = None, [], ""
            if camera is None:
                failure = "intrinsics_unavailable"
            elif (msg.header.frame_id != self.optical_frame or (msg.width, msg.height) != camera[1:3]
                  or _stamp_ns(msg.header) <= 0):
                failure = "image_geometry_mismatch"
            elif not self._source_is_current(_stamp_ns(msg.header)):
                failure = "source_time_stale"
            else:
                try:
                    image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    observations = self._detector.detect(image, camera[3], camera[4])
                except (CvBridgeError, cv2.error, RuntimeError, TypeError, ValueError) as exc:
                    self.get_logger().warning(f"Cube detection failed: {exc}", throttle_duration_sec=5.)
                    failure = "detection_failed"
            with self._condition:
                if self._closed:
                    return
                if epoch != self._epoch:
                    continue
                if (time.monotonic() - arrival > BIND_MAX_AGE_S or
                        (not failure and not self._source_is_current(_stamp_ns(msg.header)))):
                    failure = "processing_late"
                if failure:
                    self._tracker.reset()
                    observations = []
                    result = None
                else:
                    result = self._tracker.update(_stamp_ns(msg.header), observations)
                self._ready_until = arrival + BIND_MAX_AGE_S if result is not None else 0
                self._processed = index
                for hand, output in self._outputs.items():
                    record = self._record(msg, camera, index, len(observations), hand,
                                          result[hand] if result is not None else None, failure)
                    if self.context.ok():
                        output.publish(record)
                self._condition.notify_all()
            if self._listener is not None and result is not None:
                try:
                    self._listener(image, camera[3], camera[4], observations, result, _stamp_ns(msg.header))
                except Exception as exc:
                    self.get_logger().warning(f"Cube preview failed: {exc}", throttle_duration_sec=5.)

    def _record(self, image, camera, index, count, hand, estimate, failure):
        message = HandCubeObservation(schema_version=1, header=image.header, hand=hand, camera_name=self.camera_name,
            instance_id=self._instance, layout_sha256=self._layout_hash, intrinsics_sha256=camera[0] if camera else "",
            status=failure or estimate.status, visible_cube_count=count, source_image_index=index, skipped_image_total=self._skipped)
        observation = estimate.observation if estimate is not None else None
        if observation is not None:
            message.valid = True
            message.center_px = list(observation.center_px)
            message.tag_ids = list(observation.tag_ids)
            message.reprojection_px = observation.reprojection_px
            message.pose.position.x, message.pose.position.y, message.pose.position.z = observation.translation_m
            q = message.pose.orientation
            q.x, q.y, q.z, q.w = observation.quaternion_xyzw
        return message

    def _bind_current(self, operation, *args):
        with self._condition:
            # A service thread may arrive while an image newer than the displayed
            # one is being decoded. Wait only for work already received at request.
            target, epoch = self._image_index, self._epoch
            if not target or self._closed:
                raise ValueError("等待新鲜的头部相机画面")
            self._condition.wait_for(lambda: self._processed >= target or self._closed or epoch != self._epoch,
                                     timeout=BIND_MAX_AGE_S)
            if (self._closed or epoch != self._epoch or self._processed < target
                    or time.monotonic() >= self._ready_until):
                raise ValueError("当前相机观测不可用或已过期，请查看最新画面后绑定")
            return operation(*args)

    def bind_hand(self, hand):
        return self._bind_current(self._tracker.bind, hand)

    def bind_hand_at(self, hand, center_px, stamp_ns):
        return self._bind_current(self._tracker.bind_at, hand, center_px, stamp_ns)

    def reset_bindings(self):
        with self._condition:
            self._invalidate()

    def _bind_service(self, hand):
        def callback(_request, response):
            try:
                self.bind_hand(hand)
            except ValueError as exc:
                response.success, response.message = False, str(exc)
            else:
                response.success, response.message = True, f"Bound {hand} cube"
            return response
        return callback

    def _reset_service(self, _request, response):
        self.reset_bindings()
        response.success, response.message = True, "Cleared both cube bindings"
        return response

    def destroy_node(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        if self._worker is not None:
            self._worker.join()
        try:
            return super().destroy_node()
        finally:
            if self._lease is not None:
                self._lease.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HandCubeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
