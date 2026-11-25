"""Exercise timing, mount, TF-like composition, and real output over local DDS."""
import json
from math import sqrt
import time

import pytest


def test_real_dds_fusion_joins_correct_timestamps_and_emits_current_invalidity(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from vt_camera_msgs.msg import CameraFrameTiming, HandCubeObservation, OpticalHandPose
    from vt_glove_msgs.msg import GloveState
    from vt_capture_manager.optical.fusion_node import OpticalHandFusionNode

    ns = "fusiontest"
    base = "/hand_cube_tracking/right"
    source_stamp = 20_000_000_000
    host = time.time_ns()
    config = tmp_path / "mount.json"
    config.write_text(json.dumps({"schema_version":1, "hands":{"right":{
        "layout_sha256":"e"*64, "cube_from_model_wrist":{
            "translation_m":[.02,0.,0.], "quaternion_xyzw":[0,0,0,1]}}}}))

    def at(header, frame, ns):
        header.frame_id = frame
        header.stamp.sec, header.stamp.nanosec = divmod(ns, 10**9)

    ctx = Context()
    rclpy.init(context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    listened, received = [], []
    fusion = OpticalHandFusionNode(context=ctx, result_listener=listened.append, parameter_overrides=[
        Parameter("camera_name", value=ns), Parameter("mount_path", value=str(config)),
        Parameter("camera_serial", value="serial")])
    src = Node("fusion_source", context=ctx)
    try:
        executor.add_node(fusion)
        executor.add_node(src)
        pub_c = src.create_publisher(HandCubeObservation, base+"/observation", 10)
        pub_g = src.create_publisher(GloveState, "/gloves/right/state", 10)
        pub_t = src.create_publisher(CameraFrameTiming, "/fusiontest/frame_timing", 10)
        sub = src.create_subscription(OpticalHandPose, base+"/hand_pose", received.append, 10)

        def until(pred, limit=5):
            deadline = time.monotonic()+limit
            while not pred() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert pred()

        until(lambda: min(p.get_subscription_count() for p in (pub_t, pub_c, pub_g)) and sub.get_publisher_count())
        cube = HandCubeObservation(schema_version=1, camera_name=ns, instance_id="cube-worker",
            layout_sha256="e"*64, intrinsics_sha256="a"*64, hand="right", status="tracked",
            valid=True, source_image_index=1)
        at(cube.header, ns+"_color_optical_frame", source_stamp)
        cube.pose.position.z = .5
        cube.pose.orientation.z = cube.pose.orientation.w = sqrt(.5)
        timing = CameraFrameTiming(camera_name=ns, serial_number="serial", shared_ros_timestamp_ns=source_stamp,
            color_host_realtime_ns=host, color_frame_number=1, group_validity_flags=31, color_validity_flags=65)
        at(timing.header, ns+"_frame_group", source_stamp)
        pose = GloveState(schema_version=1, hand="right", source_mode="live", session_id="real-glove",
            config_sha256="b"*64, model_rest_sha256="c"*64, calibration_version=4, frame_index=15,
            pose_valid=True, time_valid=True, clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
            host_realtime_ns=host, host_monotonic_ns=30_000_000_000, root_quat_xyzw=[0.,0.,0.,1.],
            joint_positions_m=[0.,0.,0.]+[.04,0.,0.]*20)
        at(pose.header, "glove_right_reference", host)
        pub_c.publish(cube)
        pub_g.publish(pose)
        pub_t.publish(timing)
        until(lambda: len(received) >= 1)
        measurement = received[0]
        assert listened[0] == measurement
        assert measurement.joints_valid and measurement.wrist_valid
        assert measurement.header == cube.header and measurement.color_host_realtime_ns == host
        assert measurement.glove_session_id == "real-glove" and measurement.glove_frame_index == 15
        assert measurement.glove_minus_camera_ns == 0 and measurement.camera_serial == "serial"
        assert len(measurement.mount_sha256) == 64 and measurement.glove_model_rest_sha256 == "c"*64
        assert list(measurement.joint_positions_m[-3:]) == pytest.approx([0.,.06,.5])

        cube.valid, cube.status, cube.source_image_index = False, "lost", 2
        at(cube.header, ns+"_color_optical_frame", source_stamp+33_000_000)
        pub_c.publish(cube)
        until(lambda: len(received) >= 2)
        assert received[-1].status == "cube_lost"
        assert not received[-1].wrist_valid and not received[-1].joints_valid and not received[-1].joint_positions_m
    finally:
        executor.remove_node(fusion)
        executor.remove_node(src)
        fusion.destroy_node()
        src.destroy_node()
        executor.shutdown()
        ctx.shutdown()
