"""Record an actual optical node operating on two identical synthetic cubes.

Opt in using VT_CAPTURE_MANAGER_ROS_TEST=1. Set VT_HAND_CUBE_MCAP_REPORT to
export a compact JSON result after a successful independent read of closed MCAP.
"""
from collections import Counter
from hashlib import sha256
import json
import os
from pathlib import Path
import time

import pytest

pytestmark = pytest.mark.skipif(os.getenv("VT_CAPTURE_MANAGER_ROS_TEST") != "1",
                                reason="isolated ROS integration test")


def test_bilateral_camera_source_and_occlusion_survive_closed_mcap(tmp_path, monkeypatch):
    import numpy as np
    import cv2
    import rosbag2_py
    from cv_bridge import CvBridge
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image
    from std_srvs.srv import Trigger
    from vt_camera_msgs.msg import HandCubeObservation
    from vt_capture_manager.optical.node import HandCubeNode, PREFIX
    from vt_realsense_capture.controller import RecorderStartSpec
    from vt_realsense_capture.controller_node import ManagedRecorder
    from vt_realsense_capture.recorder import required_topics, write_qos_overrides
    from test_optical_cube import image_for, K, D

    domain = 150 + os.getpid()%25
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    context = Context()
    context.init(args=[], domain_id=domain)
    camera = "cube_source_test"
    src = Node("cube_recording_test_camera", context=context, enable_rosout=False)
    tracker = HandCubeNode(context=context, parameter_overrides=[Parameter("camera_name", value=camera)])
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(src)
    executor.add_node(tracker)
    hands = ("left", "right")
    optical = {side: f"{PREFIX}/{side}/observation" for side in hands}
    rgb, intrinsics = f"/{camera}/color/image_raw", f"/{camera}/color/camera_info"
    topics = required_topics((camera, "other_camera_2", "other_camera_3"), tuple(optical.values()))
    directory = tmp_path/"bilateral-cube"
    directory.mkdir()
    bag = directory/"bag"
    qos = write_qos_overrides(directory/"qos.yaml", topics,
                              reliable_topics=(rgb,*optical.values()), latched_topics=(intrinsics,))
    storage = Path(__file__).resolve().parents[2]/"vt_realsense_capture/config/mcap_writer_options.yaml"
    recorder = ManagedRecorder(storage)
    spec = RecorderStartSpec(directory.name, directory, bag, qos, topics)
    pub = src.create_publisher(Image, rgb, 10)
    ipub = src.create_publisher(CameraInfo, intrinsics,
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clients = {hand: src.create_client(Trigger, f"{PREFIX}/bind_{hand}") for hand in hands}
    outputs = {hand: [] for hand in hands}
    for hand in hands:
        src.create_subscription(HandCubeObservation, optical[hand], outputs[hand].append, 10)
    bridge = CvBridge()
    lp = ((-.058, 0., .27), 33., (2,5))
    rp = ((.058, .01, .29), 55., (2,5))
    pictures = {name: cv2.cvtColor(image_for(poses), cv2.COLOR_GRAY2RGB)
                for name, poses in (("L",[lp]), ("B",[lp,rp]), ("R",[rp]))}
    info = CameraInfo(width=960,height=600,k=K.flatten().tolist(),d=D.tolist(),distortion_model="plumb_bob")
    info.header.frame_id = f"{camera}_color_optical_frame"
    stamps = []
    source_hashes = {}

    def wait(predicate):
        deadline = time.monotonic()+8.
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate(), "Timed out waiting for camera/solver/recorder"

    def send(name):
        message = bridge.cv2_to_imgmsg(pictures[name], encoding="rgb8")
        stamp = time.time_ns()
        message.header.frame_id = info.header.frame_id
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp,10**9)
        stamps.append(stamp)
        source_hashes[stamp] = sha256(bytes(message.data)).hexdigest()
        pub.publish(message)
        wait(lambda: all(ms and ms[-1].header == message.header for ms in outputs.values()))

    def bind(hand):
        future = clients[hand].call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success

    try:
        wait(lambda: all(x.service_is_ready() for x in clients.values()) and
             pub.get_subscription_count() and ipub.get_subscription_count() and
             all(src.get_publishers_info_by_topic(t) for t in optical.values()) and
             all(p.get_subscription_count() for p in tracker._outputs.values()))
        # Complete DDS recorder discovery before starting the live source; the
        # initial operator binding frames are part of this test recording too.
        recorder.start(spec)
        wait(lambda: min(pub.get_subscription_count(), ipub.get_subscription_count()) >= 2 and
             all(p.get_subscription_count() >= 2 for p in tracker._outputs.values()))
        ipub.publish(info)
        wait(lambda: tracker._camera is not None)
        send("L")
        bind("left")
        send("B")
        bind("right")
        for name in ("B", "R", "B"):
            send(name)
        expected = {rgb:5, intrinsics:1, optical["left"]:5, optical["right"]:5}

        def storage_ready():
            metrics = recorder.metrics()
            counters = {row["topic"]:row["storage_returned_messages"]
                        for row in (metrics or {}).get("topics", [])}
            return all(counters.get(topic)==count for topic,count in expected.items())

        wait(storage_ready)
        recorder.stop()
        final = json.loads((directory/"recorder-metrics.json").read_text())
        assert final["closed"] and final["writer_errors"] == final["storage_errors"] == 0
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id="mcap"), rosbag2_py.ConverterOptions("",""))
        observed = {topic:[] for topic in expected}
        classes = {rgb:Image,intrinsics:CameraInfo,**{t:HandCubeObservation for t in optical.values()}}
        while reader.has_next():
            topic, raw, _received = reader.read_next()
            observed[topic].append(deserialize_message(raw, classes[topic]))
        reader.close()
        assert {topic:len(rows) for topic,rows in observed.items()} == expected
        metrics = {row["topic"]:row for row in final["topics"]}
        for topic,count in expected.items():
            row = metrics[topic]
            assert row["writer_received_messages"] == row["storage_returned_messages"] == count
        assert observed[intrinsics] == [info]
        by_stamp = {}
        for frame in observed[rgb]:
            stamp = frame.header.stamp.sec * 10**9 + frame.header.stamp.nanosec
            assert sha256(bytes(frame.data)).hexdigest() == source_hashes[stamp]
            by_stamp[stamp] = frame
        assert list(by_stamp) == stamps
        statuses = {}
        for hand in hands:
            rows = observed[optical[hand]]
            statuses[hand] = [row.status for row in rows]
            assert all(row.hand == hand and row.header == by_stamp[stamp].header for row,stamp in zip(rows,stamps))
            assert [row.visible_cube_count for row in rows] == [1,2,2,1,2]
            assert [row.source_image_index for row in rows] == [1,2,3,4,5]
            assert all(row.skipped_image_total == 0 for row in rows)
            for row in rows:
                assert tuple(row.tag_ids) == ((2,5) if row.valid else ())
                if row.valid:
                    vec = row.pose.position
                    np.testing.assert_allclose([vec.x,vec.y,vec.z], (lp if hand=="left" else rp)[0], atol=.003)
            assert [row.valid for row in rows] == ([False,True,True,False,False] if hand == "left" else [False,False,True,True,True])
        assert statuses == {"left":["unbound","tracked","tracked","lost","lost"], "right":["unbound","unbound","tracked","tracked","tracked"]}
        report = dict(synthetic=True,camera_size=[960,600],source_frames=5,
                      frame_intervals_ns=np.diff(stamps).tolist(), source_rgb_payloads_verified=5, shared_parent_headers_verified=10,
                      repeated_ids=[2,5],visible_cube_counts=[1,2,2,1,2],status_by_hand=statuses,
                      writer_storage_mcap_counts=expected,closed=True,writer_errors=0,storage_errors=0)
        output = os.getenv("VT_HAND_CUBE_MCAP_REPORT")
        if output:
            Path(output).write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    finally:
        recorder.stop()
        executor.shutdown()
        tracker.destroy_node()
        src.destroy_node()
        context.try_shutdown()
