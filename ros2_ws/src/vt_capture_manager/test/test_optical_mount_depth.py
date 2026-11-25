"""Frozen color pixels sample physical depth after depth-to-color registration."""
from types import SimpleNamespace as NS
import os
import time

import numpy as np
import pytest

from vt_capture_manager.optical.mount_depth import FrozenDepth


WIDTH, HEIGHT = 96, 72
K = np.array([[200., 0., 47.5], [0., 200., 35.5], [0., 0., 1.]])
D = np.zeros(5)
DEPTH_FRAME = 'mount_depth_test_depth_optical_frame'


def frozen(depth_mm, shift_x=.0):
    k = K.ravel().tolist()
    depth = NS(encoding='16UC1', is_bigendian=False, width=WIDTH, height=HEIGHT,
               step=WIDTH*2, data=depth_mm.tobytes(), header=NS(frame_id=DEPTH_FRAME))
    info = dict(width=WIDTH, height=HEIGHT, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2],
                k=k, r=np.eye(3).ravel().tolist(), p=np.column_stack((K, np.zeros(3))).ravel().tolist())
    geometry = dict(config={'depth_frame_id': DEPTH_FRAME, 'depth_image_rectified': True},
                    depth_intrinsics=info, depth_units_m=.001, depth_units_source='realsense_ros_z16_mm',
                    depth_to_color=dict(translation_m=[shift_x, 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]))
    return FrozenDepth('test', depth, geometry, K.copy(), D.copy(), (WIDTH, HEIGHT))


def test_color_click_matches_registered_depth_and_foreground_z_buffer():
    depth = np.full((HEIGHT, WIDTH), 1000, dtype='<u2')
    depth[15:60, 5:35] = 500
    snap = frozen(depth, .05)
    # The foreground moves 20 pixels right after registration; background moves 10.
    assert depth[35, 44] == 1000
    np.testing.assert_allclose(snap.point_at((44, 35)), [(.5*(44-47.5)/200), .5*(35-35.5)/200, .5])
    with pytest.raises(ValueError, match='边缘'):
        snap.point_at((54, 35))
    with pytest.raises(ValueError, match='画面内'):
        snap.point_at((-3, 35))
    depths = np.full((HEIGHT, WIDTH), 500, dtype='<u2')
    depths[32:39, 41:48] = 0
    with pytest.raises(ValueError, match='可靠深度'):
        frozen(depths).point_at((44, 35))


def test_depth_units_must_be_verified_and_holes_do_not_pick_nearby_background():
    snap = frozen(np.full((HEIGHT, WIDTH), 450, dtype='<u2'))
    snap.geometry['depth_units_source'] = ''
    with pytest.raises(ValueError, match='深度图'):
        snap.point_at((30, 30))
    depth = np.full((HEIGHT, WIDTH), 750, dtype='<u2')
    depth[29:32, 29:32] = 0
    with pytest.raises(ValueError, match='可靠深度'):
        frozen(depth).point_at((30, 30))


@pytest.mark.skipif(os.getenv('VT_CAPTURE_MANAGER_ROS_TEST') != '1', reason='isolated real DDS transport')
def test_rgb_depth_serial_and_group_identity_over_dds():
    from cv_bridge import CvBridge
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from realsense2_camera_msgs.msg import Extrinsics
    from realsense2_camera_msgs.srv import DeviceInfo
    from vt_camera_msgs.msg import CameraFrameTiming
    from vt_capture_manager.optical.mount_depth import MountDepthCapture

    camera = 'mount_depth_test'
    prefix = '/' + camera + '/'
    ctx = Context()
    ctx.init(args=[])
    driver = Node('mount_depth_driver', context=ctx)
    reader = Node('mount_depth_receiver', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    for node in (driver, reader):
        executor.add_node(node)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    driver.create_publisher(Image, prefix+'color/image_raw', 1)
    depth_pub = driver.create_publisher(Image, prefix+'depth/image_rect_raw', 1)
    info_pub = driver.create_publisher(CameraInfo, prefix+'depth/camera_info', 1)
    timing_pub = driver.create_publisher(CameraFrameTiming, prefix+'frame_timing', 1)
    ext_pub = driver.create_publisher(Extrinsics, prefix+'extrinsics/depth_to_color', latched)
    ext_pub.publish(Extrinsics(rotation=np.eye(3).ravel(), translation=[0., 0., 0.]))

    def identify(request, response):
        response.serial_number = 'physical-SN'
        response.device_name = 'Intel RealSense D436'
        return response

    driver.create_service(DeviceInfo, prefix+'device_info', identify)
    cap = MountDepthCapture(reader, camera)

    def wait(condition):
        deadline = time.monotonic() + 4.
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert condition()

    depth = CvBridge().cv2_to_imgmsg(np.full((HEIGHT, WIDTH), 520, dtype=np.uint16), encoding='16UC1')
    depth.header.frame_id = DEPTH_FRAME
    info = CameraInfo(width=WIDTH, height=HEIGHT, k=K.ravel(), r=np.eye(3).ravel(),
                      p=np.column_stack((K, np.zeros(3))).ravel(), distortion_model='plumb_bob', d=D.tolist())
    info.header.frame_id = DEPTH_FRAME

    def publish(serial='physical-SN', lag=0):
        tick = time.time_ns()
        depth.header.stamp.sec, depth.header.stamp.nanosec = divmod(tick, 10**9)
        info.header.stamp = depth.header.stamp
        tm = CameraFrameTiming(camera_name=camera, camera_model='D436', serial_number=serial,
                               color_host_realtime_ns=tick, depth_host_realtime_ns=tick,
                               color_validity_flags=65, depth_validity_flags=65, group_validity_flags=27,
                               color_frame_number=1, depth_frame_number=1, shared_ros_timestamp_ns=tick+lag)
        tm.header.frame_id = camera+'_frame_group'
        tm.header.stamp.sec, tm.header.stamp.nanosec = divmod(tick+lag, 10**9)
        depth_pub.publish(depth)
        info_pub.publish(info)
        timing_pub.publish(tm)
        wait(lambda: tick in cap._rows['image'] and tick in cap._rows['info'] and tick+lag in cap._rows['timing'])
        return tick

    try:
        wait(lambda: cap._transform and cap._device and depth_pub.get_subscription_count() == 1)
        original = cap.freeze(publish(), K, D, (WIDTH, HEIGHT))
        assert original.serial == 'physical-SN'
        with pytest.raises(ValueError, match='序列号'):
            cap.freeze(publish('other'), K, D, (WIDTH, HEIGHT))
        with pytest.raises(ValueError, match='同一帧'):
            cap.freeze(publish(lag=1), K, D, (WIDTH, HEIGHT))
        cap.close()
        assert original.point_at((40, 35))[2] == pytest.approx(.52)
    finally:
        cap.close()
        executor.shutdown()
        reader.destroy_node()
        driver.destroy_node()
        ctx.try_shutdown()
