"""Exercise the actual optical node over DDS without cameras or gloves."""
import os
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.getenv("VT_CAPTURE_MANAGER_ROS_TEST") != "1",
                                reason="isolated ROS integration test")


def test_two_identical_cubes_bind_publish_and_lose_visibility():
    import numpy as np
    from cv_bridge import CvBridge
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import CameraInfo, Image
    from std_srvs.srv import Trigger
    from vt_camera_msgs.msg import HandCubeObservation
    from vt_capture_manager.optical.node import HandCubeNode, PREFIX
    from test_optical_cube import image_for, K, D

    ctx = Context()
    ctx.init(args=[])
    observer = Node("cube_test_camera", context=ctx)
    previews = []
    tracker = HandCubeNode(frame_listener=lambda *args: previews.append(args), context=ctx,
                           parameter_overrides=[Parameter("camera_name", value="cube_test")])
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(tracker)
    executor.add_node(observer)
    raw = observer.create_publisher(Image, "/cube_test/color/image_raw", 1)
    intrinsics = observer.create_publisher(CameraInfo, "/cube_test/color/camera_info", 1)
    received = {"left": [], "right": []}
    for hand, messages in received.items():
        observer.create_subscription(HandCubeObservation, f"{PREFIX}/{hand}/observation", messages.append, 10)
    bind_left = observer.create_client(Trigger, f"{PREFIX}/bind_left")
    bind_right = observer.create_client(Trigger, f"{PREFIX}/bind_right")
    reset = observer.create_client(Trigger, f"{PREFIX}/reset")
    grey = np.full((600, 960), 255, np.uint8)
    left_pose, right_pose = ((-.058, 0., .27), 33., (2,5)), ((.058, .01, .29), 55., (2,5))
    left_image = image_for([left_pose])
    both_image = image_for([left_pose, right_pose])
    info = CameraInfo(width=grey.shape[1], height=grey.shape[0], k=list(K.flatten()), d=list(D),
                      distortion_model="plumb_bob")
    info.header.frame_id = "cube_test_color_optical_frame"
    stamps = {}
    bridge = CvBridge()

    def wait(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate()

    def send(arr, index, *, stamp_offset_ns=0):
        message = bridge.cv2_to_imgmsg(arr, encoding="mono8")
        message.header.frame_id = info.header.frame_id
        stamp = stamps[index] = time.time_ns() + stamp_offset_ns
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        raw.publish(message)
        wait(lambda: all(rows and rows[-1].header == message.header for rows in received.values()))
        return {hand: rows[-1] for hand, rows in received.items()}

    def call(client):
        fut = client.call_async(Trigger.Request())
        wait(fut.done)
        return fut.result()

    try:
        wait(lambda: bind_left.service_is_ready() and bind_right.service_is_ready() and reset.service_is_ready()
             and raw.get_subscription_count() == intrinsics.get_subscription_count() == 1
             and all(pub.get_subscription_count() for pub in tracker._outputs.values()))
        with pytest.raises(RuntimeError, match="already has a local hand_cube_tracking"):
            HandCubeNode(context=ctx)
        # Source frames before usable intrinsics publish invalid, carrying the original source header.
        before = send(left_image, 1)
        assert before["left"].status == "intrinsics_unavailable"
        intrinsics.publish(info)
        wait(lambda: tracker._camera is not None)
        first = send(left_image, 2)
        assert first["left"].visible_cube_count == 1 and not first["left"].valid
        assert call(bind_left).success
        second = send(both_image, 3)
        assert second["left"].valid and not second["right"].valid
        assert call(bind_right).success
        both = send(both_image, 4)
        for hand, expected in (("left", left_pose[0]), ("right", right_pose[0])):
            msg = both[hand]
            assert msg.valid and msg.status == "tracked" and msg.visible_cube_count == 2
            assert len(msg.layout_sha256) == len(msg.intrinsics_sha256) == 64
            assert msg.reprojection_px < 1.25 and list(msg.tag_ids) == [2, 5]
            assert msg.instance_id == tracker.instance_id
            assert msg.hand == hand and msg.camera_name == "cube_test" and msg.source_image_index == 4
            pos = msg.pose.position
            np.testing.assert_allclose([pos.x, pos.y, pos.z], expected, atol=.003)
        assert previews[-1][0].shape == (600, 960, 3)
        assert previews[-1][-1] == stamps[4]
        assert all(v.status == "tracked" for v in previews[-1][4].values())
        lost = send(grey, 5)
        assert all(not msg.valid and msg.visible_cube_count == 0 for msg in lost.values())
        assert not call(bind_left).success
        assert call(reset).success
        # A stale buffer cannot authorize binding after the camera stops.
        send(left_image, 6)
        time.sleep(.27)
        assert not call(bind_left).success
        stale = send(left_image, 7, stamp_offset_ns=2_000_000_000)
        assert all(msg.status == "source_time_stale" and not msg.valid for msg in stale.values())
        assert not call(bind_left).success
        # A mismatched optical frame invalidates the current geometry.
        info.header.frame_id = "other_optical_frame"
        intrinsics.publish(info)
        wait(lambda: tracker._camera is None)
        assert all(not msg.valid for msg in send(left_image, 8).values())
        info.header.frame_id = "cube_test_color_optical_frame"
        intrinsics.publish(info)
        wait(lambda: tracker._camera is not None)
        send(left_image, 9)
        assert call(bind_left).success
        # Do not transfer a previous binding or old intrinsics to a new image endpoint.
        observer.destroy_publisher(raw)
        raw = observer.create_publisher(Image, "/cube_test/color/image_raw", 1)
        wait(lambda: raw.get_subscription_count() == 1)
        assert send(left_image, 10)["left"].status == "intrinsics_unavailable"
        intrinsics.publish(info)
        wait(lambda: tracker._camera is not None)
        assert send(left_image, 11)["left"].status == "unbound"

        # Simulate CPU pressure inside detection. A delayed result cannot label
        # the current hand, and only the newest pending source image survives.
        entered, release = threading.Event(), threading.Event()
        original = tracker._detector.detect

        def delayed(*args):
            entered.set()
            release.wait(1)
            return original(*args)

        tracker._detector.detect = delayed
        def enqueue():
            msg = bridge.cv2_to_imgmsg(left_image, encoding="mono8")
            msg.header.frame_id = info.header.frame_id
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
            raw.publish(msg)
            return msg.header
        previous = tracker._image_index
        enqueue()
        wait(entered.is_set)
        # Arrange overlapping waits older than the live observation bound.
        time.sleep(.13)
        enqueue()
        wait(lambda: tracker._image_index == previous + 2)
        time.sleep(.15)
        last = enqueue()
        wait(lambda: tracker._image_index == previous + 3)
        release.set()
        wait(lambda: received["left"][-1].header == last)
        assert any(msg.status == "processing_late" and not msg.valid for msg in received["left"])
        assert received["left"][-1].skipped_image_total > 0
        assert received["left"][-1].status == "unbound"
    finally:
        executor.shutdown()
        tracker.destroy_node()
        observer.destroy_node()
        ctx.try_shutdown()


def test_late_latched_info_and_exact_volatile_image_info_pair():
    import numpy as np
    from cv_bridge import CvBridge
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from vt_camera_msgs.msg import HandCubeObservation
    from vt_capture_manager.optical.node import HandCubeNode, PREFIX

    ctx = Context()
    ctx.init(args=[])
    camera = Node("cube_static_calibration", context=ctx)
    raw_name = "/hotplug_cube/color/image_raw"
    info_name = "/hotplug_cube/color/camera_info"
    raw = camera.create_publisher(Image, raw_name, 1)
    latched = camera.create_publisher(CameraInfo, info_name,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    info = CameraInfo(width=64, height=48, k=[100.,0.,32.,0.,100.,24.,0.,0.,1.],
                      d=[0.] * 5, distortion_model="plumb_bob")
    info.header.frame_id = "hotplug_cube_color_optical_frame"
    info.header.stamp.sec = int(time.time()) - 60
    latched.publish(info)  # One nonzero-stamped profile, before any subscriber.
    node = HandCubeNode(context=ctx, parameter_overrides=[Parameter("camera_name", value="hotplug_cube")])
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(camera)
    executor.add_node(node)
    out = []
    camera.create_subscription(HandCubeObservation, f"{PREFIX}/left/observation", out.append, 10)

    def wait(condition):
        deadline = time.monotonic() + 5
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.003)
        assert condition()

    def image():
        msg = CvBridge().cv2_to_imgmsg(np.full((48, 64, 3), 255, np.uint8), encoding="bgr8")
        msg.header.frame_id = info.header.frame_id
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(time.time_ns(), 10**9)
        return msg

    try:
        wait(lambda: node._camera is not None and node._latched_info_subscription
             and node._outputs["left"].get_subscription_count() == 1)
        first = image()
        raw.publish(first)
        wait(lambda: len(out) == 1)
        assert out[0].header == first.header and out[0].status == "unbound" and len(out[0].intrinsics_sha256) == 64
        camera.destroy_publisher(latched)
        volatile = camera.create_publisher(CameraInfo, info_name, 1)
        wait(lambda: not node._latched_info_subscription and volatile.get_subscription_count() == 1)
        wrong_time = image()
        info.header.stamp = wrong_time.header.stamp
        info.header.stamp.nanosec -= 1 if info.header.stamp.nanosec else 0
        volatile.publish(info)
        wait(lambda: node._static_since is None)
        missing = image()
        raw.publish(missing)
        wait(lambda: len(out) == 2)
        assert out[-1].status == "intrinsics_unavailable"
        paired = image()
        raw.publish(paired)
        wait(lambda: node._image_index == 3)
        info.header.stamp = paired.header.stamp
        volatile.publish(info)
        wait(lambda: len(out) == 3)
        assert out[-1].status == "unbound" and out[-1].header == paired.header
    finally:
        executor.shutdown()
        node.destroy_node()
        camera.destroy_node()
        ctx.try_shutdown()
