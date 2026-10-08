"""Synthetic ROS only. Run with an isolated ROS_DOMAIN_ID and local discovery."""
import os
import socket
import time
import uuid

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("VT_CAPTURE_MANAGER_ROS_TEST") != "1",
                              reason="opt-in synthetic ROS test requires an isolated ROS domain")


def test_synthetic_ros_to_unix_snapshot(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from vt_camera_msgs.msg import CaptureStatus
    from vt_glove_msgs.msg import GloveState
    from vt_tactile_msgs.msg import TactileState
    from vt_tracker_msgs.msg import TrackerSample
    from vt_capture_core import encode_message, decode_message
    from vt_capture_manager.config import DeviceConfig, ManagerConfig
    from vt_capture_manager.node import build_node
    from vt_capture_manager.server import SnapshotServer

    prefix = "/manager_synthetic_" + uuid.uuid4().hex
    kinds = ("camera", "tracker", "glove", "tactile")
    ids = ("camera", "right_wrist", "glove", "tactile")
    config = ManagerConfig(tuple(DeviceConfig(identity, identity, kind, "right", prefix+"/"+kind)
                                 for identity, kind in zip(ids, kinds)), capture_status_topic=prefix+"/status")
    context = Context()
    rclpy.init(context=context)
    manager = build_node(config, context=context)
    publisher_node = Node("synthetic_manager_sources", context=context, enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(manager)
    executor.add_node(publisher_node)
    types = (Image, TrackerSample, GloveState, TactileState)
    publishers = []
    for kind, cls in zip(kinds, types):
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE if kind == "glove" else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        publishers.append(publisher_node.create_publisher(cls, prefix+"/"+kind, qos))
    status_publisher = publisher_node.create_publisher(CaptureStatus, prefix+"/status", QoSProfile(depth=5,
        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
    server = SnapshotServer(manager.model, tmp_path/"private"/"manager.sock").start()
    try:
        deadline = time.monotonic()+8
        while any(p.get_subscription_count() != 1 for p in publishers+[status_publisher]) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert all(p.get_subscription_count() == 1 for p in publishers+[status_publisher])
        realtime = time.time_ns()
        mono = time.monotonic_ns()

        def stamp(message):
            message.header.stamp.sec = realtime//1_000_000_000
            message.header.stamp.nanosec = realtime%1_000_000_000
            return message

        image = stamp(Image(width=8, height=8, step=24, encoding="rgb8", data=bytes([255, 0, 0])*64))
        tracker = stamp(TrackerSample(role="right_wrist", tracker_id="synthetic", host_realtime_ns=realtime,
                                      host_monotonic_ns=mono, pose_valid=True, tracking_status=2))
        tracker.header.frame_id = "vive_map"
        tracker.pose.position.x = 1.
        tracker.pose.orientation.w = 1.
        glove = stamp(GloveState(schema_version=1, hand="right", session_id="synthetic", pose_valid=True,
                                time_valid=True, host_realtime_ns=realtime, host_monotonic_ns=mono,
                                mano_pose_rotvec=[0.]*48, shape_params=[0.]*10, root_quat_xyzw=[0.,0.,0.,1.],
                                joint_positions_m=[.01]*63, quality="valid", frame_index=1))
        glove.header.frame_id = "glove_right_reference"
        tactile = stamp(TactileState(schema_version=2, hand="right", session_id="synthetic", rows=20, cols=20,
            adc=[0]*400, valid_mask=[True]*400, block_realtime_ns=[realtime]*160, connected=True, time_valid=True,
            host_realtime_ns=realtime, host_monotonic_ns=mono, max_age_ns=1_000_000_000, stream_epoch=1, source_sequence=1))
        capture = stamp(CaptureStatus(state=CaptureStatus.IDLE, recorder_alive=False, session_id="", detail="synthetic idle"))
        for publisher, message in zip(publishers, (image, tracker, glove, tactile)):
            publisher.publish(message)
        status_publisher.publish(capture)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            snapshot = manager.model.snapshot()
            if (all(d["receive_hz"] >= 1 for d in snapshot["devices"]) and
                    snapshot["devices"][0]["preview"] and snapshot["recording"]["state"] == "IDLE"):
                break
        assert all(d["status"] == "online" for d in snapshot["devices"])
        assert snapshot["devices"][0]["preview"]["format"] == "jpeg"
        assert snapshot["devices"][1]["preview"]["position"] == [1., 0., 0.]
        assert len(snapshot["devices"][2]["preview"]["joints"]) == 21
        assert all(snapshot["devices"][3]["preview"]["valid_mask"])
        assert snapshot["recording"]["state"] == "IDLE"
        with socket.socket(socket.AF_UNIX) as client:
            client.settimeout(2)
            client.connect(str(server.path))
            client.sendall(encode_message(dict(version=1, request_id="e2e", command="snapshot")))
            data = bytearray()
            while not data.endswith(b"\n"):
                data.extend(client.recv(65536))
        result = decode_message(bytes(data))
        assert result["ok"] and result["snapshot"]["capabilities"] == dict(record=False, device_control=False)
        # Manager has no device/recorder publishers or control services.
        assert all(p.topic_name == "/parameter_events" for p in manager.publishers)
        assert all(service.srv_name.endswith("/get_type_description") for service in manager.services)
    finally:
        server.close()
        executor.remove_node(manager)
        executor.remove_node(publisher_node)
        manager.destroy_node()
        publisher_node.destroy_node()
        executor.shutdown()
        context.shutdown()
