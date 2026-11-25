"""Opt-in ROS source for metric wrists and IMU joint shapes in the camera frame."""
from functools import partial
import re
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vt_camera_msgs.msg import CameraFrameTiming, HandCubeObservation, OpticalHandPose
from vt_glove_msgs.msg import GloveState

from .hand_fusion import FusionJoin, read_mounts
from .identity import HANDS
from .lease import domain_lease


def output_message(result):
    cube = result.cube
    msg = OpticalHandPose(header=cube.header, schema_version=1, camera_name=cube.camera_name,
        hand=cube.hand, status=result.status, cube_instance_id=cube.instance_id,
        cube_source_image_index=cube.source_image_index, layout_sha256=cube.layout_sha256,
        intrinsics_sha256=cube.intrinsics_sha256, mount_sha256=result.mount.sha256 if result.mount else "")
    if result.timing is not None:
        msg.color_host_realtime_ns = result.timing.realtime
        msg.color_frame_number = result.timing.number
        msg.camera_serial = result.timing.serial
    if result.wrist is not None:
        msg.wrist_valid = True
        p, q = msg.wrist.position, msg.wrist.orientation
        p.x, p.y, p.z = result.wrist.translation_m
        q.x, q.y, q.z, q.w = result.wrist.quaternion_xyzw
    msg.glove_before_ns, msg.glove_after_ns = result.before, result.after
    if result.shape is not None:
        sample = result.shape
        msg.joints_valid = True
        msg.joint_positions_m = result.joints
        msg.glove_session_id, msg.glove_config_sha256 = sample.session_id, sample.config_sha256
        msg.glove_model_rest_sha256 = sample.model_rest_sha256
        msg.glove_calibration_version, msg.glove_frame_index = sample.profile, sample.frame
        msg.glove_host_realtime_ns = sample.realtime
        msg.glove_minus_camera_ns = sample.realtime - result.timing.realtime
    return msg


class OpticalHandFusionNode(Node):
    def __init__(self, *, result_listener=None, **kwargs):
        context = kwargs.get("context") or rclpy.get_default_context()
        self._lease = domain_lease(context.get_domain_id(), "optical_hand_fusion")
        try:
            super().__init__("optical_hand_fusion", **kwargs)
        except BaseException:
            self._lease.close()
            raise
        try:
            camera = self.declare_parameter("camera_name", "d436").value
            if not isinstance(camera, str) or not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_]*", camera):
                raise ValueError("camera_name must be one ROS name")
            path = self.declare_parameter("mount_path", "").value
            serial = self.declare_parameter("camera_serial", "").value
            self.join = FusionJoin(camera, read_mounts(path) if path else {}, serial)
            self._listener = result_listener
            stream = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CameraFrameTiming, f"/{camera}/frame_timing", self._timing, stream)
            self._outputs = {}
            for hand in HANDS:
                prefix = f"/hand_cube_tracking/{hand}"
                self._outputs[hand] = self.create_publisher(OpticalHandPose, prefix + "/hand_pose", 10)
                self.create_subscription(HandCubeObservation, prefix + "/observation", partial(self._cube, hand), 32)
                self.create_subscription(GloveState, f"/gloves/{hand}/state",
                                         partial(self._shape, hand), stream)
            # Default mutually-exclusive ROS callback group serialises cache updates.
            self.create_timer(.010, self._drain)
        except BaseException:
            self.destroy_node()
            raise

    def _timing(self, msg):
        self.join.add_timing(msg, time.monotonic_ns())

    def _shape(self, hand, msg):
        self.join.shapes[hand].add(msg, hand, time.monotonic_ns())

    def _cube(self, hand, msg):
        # Do not route a payload for an unexpected side onto the other hand's output topic.
        if msg.hand == hand:
            self._publish(self.join.add_cube(hand, msg, time.monotonic_ns()))

    def _drain(self):
        self._publish(self.join.drain(time.monotonic_ns()))

    def _publish(self, results):
        for result in results:
            msg = output_message(result)
            self._outputs[msg.hand].publish(msg)
            if self._listener is not None:
                try:
                    self._listener(msg)
                except Exception as exc:
                    self.get_logger().warning(f"Hand fusion preview failed: {exc}", throttle_duration_sec=5.)

    def destroy_node(self):
        try:
            return super().destroy_node()
        finally:
            if self._lease is not None:
                self._lease.close()
                self._lease = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpticalHandFusionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
