#!/usr/bin/python3
"""TEST ONLY: publish the nine topics needed by the Recorder smoke test."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vt_camera_msgs.msg import CameraFrameTiming


CAMERA_IDENTITIES = {
    'd405_1': ('D405', '260322278433'),
    'd405_2': ('D405', '260322276463'),
    'd436': ('D436', '408322071716'),
}
WIDTH = 16
HEIGHT = 12


def make_timing_message(
    camera_name: str, stamp, stamp_ns: int, *, frame_number: int
) -> CameraFrameTiming:
    """Build timing metadata without claiming unpopulated callback clocks."""

    camera_model, serial_number = CAMERA_IDENTITIES[camera_name]
    message = CameraFrameTiming()
    message.header.stamp = stamp
    message.camera_name = camera_name
    message.camera_model = camera_model
    message.serial_number = serial_number
    message.shared_ros_timestamp_ns = stamp_ns
    message.color_frame_number = frame_number
    message.depth_frame_number = frame_number
    message.color_timestamp_domain = message.DOMAIN_HARDWARE_CLOCK
    message.depth_timestamp_domain = message.DOMAIN_HARDWARE_CLOCK
    message.color_validity_flags = (
        message.VALID_FRAME_NUMBER | message.VALID_CLOCK_DOMAIN
    )
    message.depth_validity_flags = message.color_validity_flags
    message.group_validity_flags = (
        message.GROUP_VALID_COMMON_STAMP
        | message.GROUP_VALID_IDENTITY
        | message.GROUP_VALID_DOMAINS
        | message.GROUP_VALID_UNIQUE
    )
    return message


class SyntheticCaptureSupport(Node):
    def __init__(self) -> None:
        super().__init__('synthetic_capture_support')
        sensor_qos = qos_profile_sensor_data
        self._color_publishers = {}
        self._depth_publishers = {}
        self._timing_publishers = {}
        for camera_name in CAMERA_IDENTITIES:
            self._color_publishers[camera_name] = self.create_publisher(
                Image, f'/{camera_name}/color/image_raw', sensor_qos
            )
            self._depth_publishers[camera_name] = self.create_publisher(
                Image, f'/{camera_name}/depth/image_rect_raw', sensor_qos
            )
            self._timing_publishers[camera_name] = self.create_publisher(
                CameraFrameTiming, f'/{camera_name}/frame_timing', sensor_qos
            )
        self._frame_number = 0
        self._timer = self.create_timer(1.0 / 30.0, self._publish_cycle)

    @staticmethod
    def _image(stamp, *, depth: bool) -> Image:
        message = Image()
        message.header.stamp = stamp
        message.height = HEIGHT
        message.width = WIDTH
        message.is_bigendian = False
        if depth:
            message.encoding = '16UC1'
            message.step = WIDTH * 2
            message.data = bytes(HEIGHT * message.step)
        else:
            message.encoding = 'rgb8'
            message.step = WIDTH * 3
            message.data = bytes(HEIGHT * message.step)
        return message

    def _timing(
        self, camera_name: str, stamp, stamp_ns: int
    ) -> CameraFrameTiming:
        return make_timing_message(
            camera_name,
            stamp,
            stamp_ns,
            frame_number=self._frame_number,
        )

    def _publish_cycle(self) -> None:
        self._frame_number += 1
        now = self.get_clock().now()
        stamp = now.to_msg()
        stamp_ns = now.nanoseconds
        color = self._image(stamp, depth=False)
        depth = self._image(stamp, depth=True)
        for camera_name in CAMERA_IDENTITIES:
            self._color_publishers[camera_name].publish(color)
            self._depth_publishers[camera_name].publish(depth)
            self._timing_publishers[camera_name].publish(
                self._timing(camera_name, stamp, stamp_ns)
            )


def main() -> int:
    rclpy.init()
    node = SyntheticCaptureSupport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
