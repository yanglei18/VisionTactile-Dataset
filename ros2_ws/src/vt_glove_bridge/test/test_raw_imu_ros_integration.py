"""Real generated messages and DDS with synthetic wire bytes; no hardware/model."""
import json
import os
from pathlib import Path
import struct
import tempfile
import time

import pytest

from test_protocol import packet

pytestmark = pytest.mark.skipif(os.environ.get("VT_GLOVE_ROS_INTEGRATION") != "1",
    reason="set VT_GLOVE_ROS_INTEGRATION=1 after building and sourcing the ROS workspace")


def test_dual_hand_parser_fusion_transport_dds_preserves_pose_source_references():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.serialization import serialize_message, deserialize_message
    from vt_glove_msgs.msg import GloveState, RawImuBatch
    from vt_glove_bridge.node import GloveBridge
    from vt_fsglove12.live_stream import ImuParser, OnlineFusion
    from vt_fsglove12.pose_export import PoseDatagramExporter
    from vt_fsglove12.raw_imu_export import RawImuDatagramExporter, raw_sample_document

    context = Context()
    rclpy.init(context=context, domain_id=148, args=[])
    executor = SingleThreadedExecutor(context=context)
    observer = Node("raw_imu_synthetic_observer", context=context)
    executor.add_node(observer)
    bridges, exporters, subscriptions = [], [], []
    raw, poses = {hand: [] for hand in ("left", "right")}, {hand: [] for hand in ("left", "right")}

    def spin_until(predicate):
        end = time.monotonic() + 8.
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), "synthetic raw IMU DDS timed out"

    try:
        with tempfile.TemporaryDirectory(prefix="vtraw-dds-") as directory:
            for hand in raw:
                endpoint = str(Path(directory) / f"{hand}.sock")
                bridge = GloveBridge(context=context, namespace=f"/gloves/{hand}", parameter_overrides=[
                    Parameter("hand", value=hand), Parameter("socket_path", value=endpoint)])
                bridges.append(bridge)
                executor.add_node(bridge)
                subscriptions.append(observer.create_subscription(RawImuBatch, f"/gloves/{hand}/imu/raw", raw[hand].append, 30))
                subscriptions.append(observer.create_subscription(GloveState, f"/gloves/{hand}/state", poses[hand].append, 30))
                raw_exporter, pose_exporter = RawImuDatagramExporter(endpoint), PoseDatagramExporter(endpoint)
                exporters.extend((raw_exporter, pose_exporter))
                spin_until(lambda: bridge.raw_publisher.get_subscription_count() and bridge.publisher.get_subscription_count())
                real, mono = time.time_ns(), time.monotonic_ns()
                wire = b"".join(struct.pack("<BB6h", node, node + 128, 0, 0, 0, 0, 0, 8192) for node in range(1, 13))
                samples = ImuParser().feed(wire, host_realtime_ns=real, host_monotonic_ns=mono)
                fusion, rows = OnlineFusion({}), []
                fusion.update(samples, raw_observer=lambda *args: rows.append(raw_sample_document(*args)))
                raw_exporter.publish(rows, hand=hand, session_id="shared-session", config_sha256="a" * 64, source_mode="live")
                palm = 11 if hand == "left" else 12
                pose_exporter.publish(json.loads(packet(hand=hand, frame_id=f"glove_{hand}_reference",
                    session_id="shared-session", config_sha256="a" * 64, reference_node_id=palm,
                    source_node_ids=[5, palm], source_sample_indices=[1, 1], source_filter_epochs=[0, 0],
                    host_realtime_ns=real, host_monotonic_ns=mono, source_realtime_ns=[real, real],
                    source_monotonic_ns=[mono, mono], solve_completed_monotonic_ns=mono)))
            spin_until(lambda: all(len(raw[h]) == len(poses[h]) == 1 for h in raw))
            for hand in raw:
                batch, pose = raw[hand][0], poses[hand][0]
                assert batch.session_id == pose.session_id == "shared-session"
                assert len(batch.samples) == 12 and batch.batch_sequence == 1
                assert batch.producer_dropped_samples_total == batch.bridge_sequence_gaps_total == 0
                by_node = {row.node_id: row for row in batch.samples}
                for index, node in enumerate(pose.source_node_ids):
                    row = by_node[node]
                    assert row.sample_index == pose.source_sample_indices[index]
                    assert row.filter_epoch == pose.source_filter_epochs[index]
                    assert row.host_realtime_ns == pose.source_realtime_ns[index]
                    assert row.host_monotonic_ns == pose.source_monotonic_ns[index]
                    assert list(row.raw_values) == [0, 0, 0, 0, 0, 8192]
                    assert row.status == node + 128
                copied = deserialize_message(serialize_message(batch), RawImuBatch)
                assert copied == batch
    finally:
        for exporter in exporters:
            exporter.close()
        for bridge in bridges:
            executor.remove_node(bridge)
            bridge.destroy_node()
        for subscription in subscriptions:
            observer.destroy_subscription(subscription)
        executor.remove_node(observer)
        observer.destroy_node()
        executor.shutdown()
        context.shutdown()
