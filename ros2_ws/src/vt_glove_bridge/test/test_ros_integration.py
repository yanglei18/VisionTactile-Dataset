"""Opt-in actual ROS/DDS loopback, synthetic solved states only; no serial or GUI."""
import os
from pathlib import Path
import socket
import tempfile
import time

import pytest

from test_protocol import packet


pytestmark = pytest.mark.skipif(
    os.environ.get("VT_GLOVE_ROS_INTEGRATION") != "1",
    reason="set VT_GLOVE_ROS_INTEGRATION=1 after sourcing built ROS workspace")


def test_two_bridges_keep_same_session_indices_and_single_hand_restart_isolated():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from vt_glove_msgs.msg import GloveState
    from vt_glove_bridge.node import GloveBridge

    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("VT_GLOVE_TEST_DOMAIN_ID", "147")), args=[])
    executor = SingleThreadedExecutor(context=context)
    observer = Node("two_hand_observer", context=context)
    executor.add_node(observer)
    received = {hand: [] for hand in ("left", "right")}
    subscriptions = [observer.create_subscription(
        GloveState, f"/gloves/{hand}/test_state", received[hand].append, 30)
        for hand in received]
    nodes = {}
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def wait_for(predicate):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not predicate():
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), "dual-hand ROS loopback timed out"

    def data(hand, index):
        mono, real = time.monotonic_ns(), time.time_ns()
        palm = 11 if hand == "left" else 12
        return packet(hand=hand, frame_id=f"glove_{hand}_reference", frame_index=index,
                      reference_node_id=palm, source_node_ids=[5, palm],
                      host_realtime_ns=real, host_monotonic_ns=mono,
                      source_realtime_ns=[real - 1, real], source_monotonic_ns=[mono - 1, mono],
                      solve_completed_monotonic_ns=mono)

    try:
        with tempfile.TemporaryDirectory(prefix="vtg-dual-") as directory:
            for hand in received:
                nodes[hand] = GloveBridge(context=context, namespace=f"/gloves/{hand}", parameter_overrides=[
                    Parameter("hand", value=hand),
                    Parameter("socket_path", value=str(Path(directory) / f"{hand}.sock")),
                    Parameter("topic", value=f"/gloves/{hand}/test_state"),
                ])
                executor.add_node(nodes[hand])
            wait_for(lambda: all(node.publisher.get_subscription_count() for node in nodes.values()))
            for hand in received:
                sender.sendto(data(hand, 1), str(Path(directory) / f"{hand}.sock"))
            wait_for(lambda: all(len(values) == 1 for values in received.values()))
            assert received["left"][0].reference_node_id == 11
            assert received["right"][0].reference_node_id == 12
            assert all(values[0].pose_valid for values in received.values())
            # Misrouting must reject just this packet, not take down either hand.
            sender.sendto(data("left", 2), str(Path(directory) / "right.sock"))
            wait_for(lambda: nodes["right"].rejected_total == 1)
            assert len(received["right"]) == 1
            left = nodes.pop("left")
            executor.remove_node(left)
            left.destroy_node()
            sender.sendto(data("right", 2), str(Path(directory) / "right.sock"))
            wait_for(lambda: len(received["right"]) == 2)
            assert received["right"][-1].pose_valid
            assert received["right"][-1].bridge_sequence_gaps_total == 0
            assert len(received["left"]) == 1
    finally:
        sender.close()
        for node in nodes.values():
            executor.remove_node(node)
            node.destroy_node()
        for subscription in subscriptions:
            observer.destroy_subscription(subscription)
        executor.remove_node(observer)
        observer.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_actual_ros_generated_message_dds_delivery_and_bridge_restart():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from vt_glove_msgs.msg import GloveState
    from vt_glove_bridge.node import GloveBridge

    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("VT_GLOVE_TEST_DOMAIN_ID", "147")), args=[])
    executor = SingleThreadedExecutor(context=context)
    observer = Node("glove_synthetic_test_observer", context=context)
    received = []
    subscription = observer.create_subscription(GloveState, "/gloves/right/test_state", received.append, 30)
    executor.add_node(observer)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    bridge = None

    def spin_until(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not predicate():
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "synthetic ROS loopback timed out"

    try:
        with tempfile.TemporaryDirectory(prefix="vtg-ros-") as directory:
            endpoint = str(Path(directory) / "g.sock")

            def start_bridge():
                node = GloveBridge(context=context, parameter_overrides=[
                    Parameter("socket_path", value=endpoint),
                    Parameter("topic", value="/gloves/right/test_state"),
                ])
                executor.add_node(node)
                spin_until(lambda: node.publisher.get_subscription_count() > 0)
                return node

            bridge = start_bridge()
            mono, real = time.monotonic_ns(), time.time_ns()
            kwargs = dict(host_realtime_ns=real, host_monotonic_ns=mono,
                          source_realtime_ns=[real - 1, real], source_monotonic_ns=[mono - 1, mono],
                          solve_completed_monotonic_ns=mono + 1)
            sender.sendto(packet(**kwargs), endpoint)
            spin_until(lambda: len(received) == 1)
            result = received[0]
            assert result.pose_valid and result.time_valid
            assert result.host_realtime_ns == real
            assert result.header.stamp.sec * 1_000_000_000 + result.header.stamp.nanosec == real
            assert list(result.mano_pose_rotvec) == [0.] * 48
            assert list(result.source_sample_indices) == [4, 8]
            assert result.bridge_received_monotonic_ns >= mono
            assert bridge.published_total == 1

            # Duplicate is rejected; no ROS message fabricated to replace it.
            sender.sendto(packet(**kwargs), endpoint)
            spin_until(lambda: bridge.rejected_total == 1)
            assert bridge.published_total == 1

            # Restart only the bridge; the same producer socket still works.
            executor.remove_node(bridge)
            bridge.destroy_node()
            bridge = start_bridge()
            mono, real = time.monotonic_ns(), time.time_ns()
            kwargs.update(host_realtime_ns=real, host_monotonic_ns=mono,
                          source_realtime_ns=[real - 1, real], source_monotonic_ns=[mono - 1, mono],
                          solve_completed_monotonic_ns=mono + 1, frame_index=2)
            sender.sendto(packet(**kwargs), endpoint)
            spin_until(lambda: len(received) == 2)
            assert received[-1].pose_valid and received[-1].frame_index == 2
    finally:
        sender.close()
        if bridge is not None:
            executor.remove_node(bridge)
            bridge.destroy_node()
        observer.destroy_subscription(subscription)
        executor.remove_node(observer)
        observer.destroy_node()
        executor.shutdown()
        context.shutdown()
