"""Opt-in PTY -> serial worker -> DDS -> MCAP -> offline grid, no hardware.

Run after sourcing built ROS packages with VT_TACTILE_E2E=1. The only serial
device opened here is a newly allocated pseudo-terminal created by this test.
"""
import json
import os
from pathlib import Path
import struct
import sys
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("VT_TACTILE_E2E") != "1",
    reason="opt-in synthetic PTY/DDS/MCAP test; source ROS and set VT_TACTILE_E2E=1",
)


def test_serial_dds_bag_and_offline_roundtrip(tmp_path):
    import numpy as np
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    import rosbag2_py
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from vt_tactile_msgs.msg import TactileSampleArray, TactileState
    from vt_tactile_capture.node import TactileNode
    from vt_tactile_capture.viewer import ViewerModel

    # rosbag transport resolves message types through the ament index, unlike
    # the in-process writer below. Importability alone is not sufficient.
    message_share = Path(get_package_share_directory("vt_tactile_msgs"))
    assert (message_share / "msg/TactileSampleArray.msg").is_file()

    root = Path(__file__).resolve().parents[4]
    sys.path.insert(0, str(root / "tools/multisensor_alignment/src"))
    from vt_multisensor_alignment.tactile_export import export_tactile

    master, slave = os.openpty()
    port = os.ttyname(slave)
    os.close(slave)  # The test must not itself hold the sole-reader port open.
    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("VT_TACTILE_TEST_DOMAIN_ID", "156")), args=[])
    executor = SingleThreadedExecutor(context=context)
    observer = Node("tactile_e2e_observer", context=context)
    samples, states = [], []
    topic = "/vt_tactile_e2e/samples"
    observer.create_subscription(TactileSampleArray, topic, samples.append, 30)
    observer.create_subscription(
        TactileState, "/vt_tactile_e2e/state", states.append,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    executor.add_node(observer)
    driver = None

    def wait_for(predicate, seconds=8):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), "synthetic tactile pipeline timed out"

    def packet(row, group, values, status=0):
        return b"\xa5\x5a" + struct.pack(">HBB10H", row, status, group, *values)

    try:
        driver = TactileNode(context=context, namespace="/vt_tactile_e2e",
                             parameter_overrides=[
                                 Parameter("port", value=port),
                                 Parameter("max_cell_age_ms", value=500.0),
                                 Parameter("stale_timeout_sec", value=5.0),
                             ])
        executor.add_node(driver)
        wait_for(lambda: driver.sample_publisher.get_subscription_count() > 0)
        # A connected snapshot confirms the new PTY was opened before writing.
        wait_for(lambda: any(message.connected for message in states))
        values = tuple(range(10))  # Include valid ADC zero.
        os.write(master, packet(17, 3, values, status=173)
                 + packet(79, 7, (65535,) * 10))
        wait_for(lambda: sum(len(message.blocks) for message in samples) >= 2)
        wait_for(lambda: any(sum(message.valid_mask) == 3 for message in states))
        captured = [message for message in samples if message.blocks]
        assert captured
        for message in captured:
            assert message.time_valid and message.connected
            assert message.clock_domain == "CLOCK_MONOTONIC"
            assert message.timing_source == "host_receive"
            assert message.header.stamp.sec * 10**9 + message.header.stamp.nanosec == message.host_realtime_ns
        assert captured[0].blocks[0].status_raw == 173
        latest = next(message for message in reversed(states) if sum(message.valid_mask) == 3)
        assert latest.schema_version == 2 and (latest.rows, latest.cols) == (20, 20)
        assert latest.adc[17 * 20 + 3] == 0
        assert latest.valid_mask[17 * 20 + 3]
        assert len(latest.adc) == 400
        assert any(block.row == 79 for message in captured for block in message.blocks)
        model = ViewerModel()
        assert model.observe(latest)
        assert model.data[17 * 20 + 3] == 0.0

        bag = tmp_path / "recording"
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("", ""))
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=1, name=topic, type="vt_tactile_msgs/msg/TactileSampleArray",
            serialization_format="cdr"))
        for message in samples:
            writer.write(topic, serialize_message(message), message.host_realtime_ns)
        writer.close()
        target_ns = captured[-1].host_realtime_ns + 1_000_000
        targets = tmp_path / "aligned_frames.jsonl"
        targets.write_text(json.dumps({"frame_index": 0, "reference_time_ns": target_ns}) + "\n")
        output = export_tactile(bag_path=bag, aligned_frames=targets,
                                output_directory=tmp_path / "tactile", topic=topic,
                                max_age_ms=500.0)
        manifest = json.loads((output / "manifest.json").read_text())
        assert manifest["frame_count"] == 1
        with np.load(output / "frames/000000000.npz", allow_pickle=False) as grid:
            assert grid["valid_mask"].sum() == 3
            assert grid["valid_mask"][17, 3]
            np.testing.assert_array_equal(grid["adc"][17, 3::8], values[:3])
            assert grid["adc"].shape == (20, 20)
            assert int(grid["timestamp_ns"]) == target_ns
    finally:
        if driver is not None:
            executor.remove_node(driver)
            driver.destroy_node()
        os.close(master)
        executor.remove_node(observer)
        observer.destroy_node()
        executor.shutdown()
        context.shutdown()
