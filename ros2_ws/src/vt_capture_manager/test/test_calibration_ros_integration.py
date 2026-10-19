"""Synthetic isolated-domain publication test; no physical sensors.

VT_CALIBRATION_ROS_TEST=1 ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
"""
import os
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('VT_CALIBRATION_ROS_TEST') != '1',
                                reason='explicit isolated calibration ROS test opt-in required')


def test_typed_latched_publication_identity_invalidation_and_adapter_restart():
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from realsense2_camera_msgs.srv import DeviceInfo
    from vt_tracker_msgs.msg import TrackerSample
    from vt_camera_msgs.msg import CameraTrackerCalibration
    from vt_capture_manager.calibration.ros_adapter import CalibrationRosAdapter

    context = Context()
    context.init(args=[])
    source = Node('calibration_identity_source', context=context)
    manager = Node('calibration_publish_test', context=context)
    observer = Node('calibration_late_subscriber', context=context)
    executor = SingleThreadedExecutor(context=context)
    for node in (source, manager, observer):
        executor.add_node(node)
    camera = dict(id='cam', name='cam', model='D405', serial='123', color_frame_id='cam_color_optical_frame',
                  depth_frame_id='cam_depth_optical_frame', tracker_role='torso', color_topic='/cam/color/image_raw',
                  depth_topic='/cam/depth/image_rect_raw', camera_info_topic='/cam/color/camera_info',
                  depth_camera_info_topic='/cam/depth/camera_info', device_info_service='/cam/device_info')
    keepalive = [source.create_publisher(kind, camera[key], 1) for kind, key in
                 ((Image, 'color_topic'), (Image, 'depth_topic'), (CameraInfo, 'camera_info_topic'),
                  (CameraInfo, 'depth_camera_info_topic'))]
    def device_info(request, response):
        response.serial_number = '123'
        response.device_name = 'Intel RealSense D405'
        return response
    info_service = source.create_service(DeviceInfo, '/cam/device_info', device_info)
    tracker_publisher = source.create_publisher(TrackerSample, '/vive/torso/sample', 10)
    identifier = ['a' * 64]
    def publish_tracker():
        sample = TrackerSample()
        sample.header.frame_id = 'vive_map'
        sample.role = 'torso'
        sample.tracker_id = identifier[0]
        sample.host_realtime_ns, sample.host_monotonic_ns = time.time_ns(), time.monotonic_ns()
        sample.tracking_status, sample.pose_valid = 0x12, True
        sample.pose.orientation.w = 1.
        tracker_publisher.publish(sample)
    timer = source.create_timer(.05, publish_tracker)
    document = dict(schema_version=1, status='VALID', camera=dict(name='cam', serial='123', model='D405',
                    frame_id='cam_color_optical_frame'), tracker=dict(role='torso', tracker_id='a' * 64,
                    frame_id='vive_tracker_torso'), frames=dict(world='vive_map'),
                    transform=dict(semantics='parent_from_child', parent_frame='vive_tracker_torso',
                    child_frame='cam_color_optical_frame', translation_m=[.1, 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]),
                    quality=dict(grade='TARGET'))
    manifest = dict(revision='b' * 32, sha256='c' * 64, activated_ns=time.time_ns())
    trackers = [dict(role='torso', frame_id='vive_tracker_torso', sample_topic='/vive/torso/sample')]
    adapter = CalibrationRosAdapter(manager, [camera], trackers, instance_id='manager-one')
    def spin_until(predicate, seconds=10.):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert predicate()
    received = []
    subscription = None
    try:
        spin_until(lambda: adapter.identity(document)[0])
        adapter.publish(document, manifest, True, '')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = observer.create_subscription(CameraTrackerCalibration,
                        '/calibration/cam/tracker_from_camera', received.append, qos)
        spin_until(lambda: bool(received))
        assert received[-1].revision == manifest['revision']
        assert received[-1].schema_version == 1
        assert received[-1].manager_instance_id == 'manager-one'
        assert received[-1].valid and received[-1].valid_until_ns > time.time_ns()
        assert received[-1].tracker_from_camera.translation.x == .1
        identifier[0] = 'd' * 64
        spin_until(lambda: bool(received) and not received[-1].valid)
        assert received[-1].reason == 'tracker_identity_mismatch'
        adapter.close()
        identifier[0] = 'a' * 64
        adapter = CalibrationRosAdapter(manager, [camera], trackers, instance_id='manager-two')
        spin_until(lambda: adapter.identity(document)[0])
        adapter.publish(document, manifest, True, '')
        spin_until(lambda: bool(received) and received[-1].manager_instance_id == 'manager-two')
        assert received[-1].revision == manifest['revision']
    finally:
        adapter.close()
        executor.shutdown()
        for node in (observer, manager, source):
            node.destroy_node()
        context.try_shutdown()
