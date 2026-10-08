"""The ROS process consumes solved-state datagrams and never accesses hardware."""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from vt_glove_msgs.msg import GloveState

from .protocol import SequenceTracker, decode_state, populate_message
from .transport import LocalReceiver, default_socket_path


class GloveBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("vt_glove_bridge", **kwargs)
        self.declare_parameter("hand", "right")
        self.hand = self.get_parameter("hand").value
        if self.hand not in ("left", "right"):
            raise ValueError("hand must be left or right")
        self.declare_parameter("socket_path", default_socket_path(self.hand))
        self.declare_parameter("topic", f"/gloves/{self.hand}/state")
        self.declare_parameter("reference_node_id", 0)
        reference = self.get_parameter("reference_node_id").value
        if type(reference) is not int or not 0 <= reference <= 12:
            raise ValueError("reference_node_id must be 0 (packet-defined) or in 1..12")
        self.reference_node_id = reference or None
        topic = self.get_parameter("topic").value
        other_hand = "left" if self.hand == "right" else "right"
        if topic.startswith(f"/gloves/{other_hand}/"):
            raise ValueError("topic namespace does not match configured hand")
        self.declare_parameter("max_input_age_s", 0.15)
        self.declare_parameter("allow_replay", False)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        if self.get_parameter("use_sim_time").value:
            raise ValueError("live bridge uses host time; use_sim_time must be false")
        age = self.get_parameter("max_input_age_s").value
        if not isinstance(age, (float, int)) or not math.isfinite(age) or age <= 0:
            raise ValueError("max_input_age_s must be finite and positive")
        self.max_age_ns = int(age * 1e9)
        self.allow_replay = self.get_parameter("allow_replay").value
        qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(GloveState, self.get_parameter("topic").value, qos)
        self.receiver = LocalReceiver(self.get_parameter("socket_path").value)
        self.sequences = SequenceTracker()
        self.rejected_total = 0
        self.published_total = 0
        self.last_received_ns = None
        self.started_ns = time.monotonic_ns()
        self._last_warning_ns = 0
        self.timer = self.create_timer(0.002, self._poll)
        self.health_timer = self.create_timer(5.0, self._health)
        self.get_logger().info(f"Solved-state bridge listening on {self.receiver.path}; no serial access, waiting for GUI")

    def _poll(self):
        for _ in range(128):
            try:
                payload = self.receiver.receive()
                if payload is None:
                    return
                now_mono, now_real = time.monotonic_ns(), time.time_ns()
                state = decode_state(payload, allow_replay=self.allow_replay,
                                     expected_hand=self.hand,
                                     reference_node_id=self.reference_node_id)
                self.sequences.accept(state)
                message = populate_message(GloveState(), state, realtime_ns=now_real,
                                           monotonic_ns=now_mono,
                                           gaps_total=self.sequences.gaps_total,
                                           rejected_total=self.rejected_total,
                                           max_input_age_ns=self.max_age_ns)
            except (ValueError, OverflowError, TypeError) as exc:
                self.rejected_total += 1
                now = time.monotonic_ns()
                if now - self._last_warning_ns > 2_000_000_000:
                    self.get_logger().warning(f"Rejected glove datagram ({self.rejected_total} total): {exc}")
                    self._last_warning_ns = now
                continue
            self.publisher.publish(message)
            self.published_total += 1
            self.last_received_ns = now_mono

    def _health(self):
        now = time.monotonic_ns()
        age = now - (self.last_received_ns or self.started_ns)
        if age > 5_000_000_000:
            self.get_logger().warning(
                f"No new solved state for {age / 1e9:.1f}s; published={self.published_total}, "
                f"rejected={self.rejected_total}, observed_sequence_gaps={self.sequences.gaps_total}. "
                "No synthetic pose/heartbeat is published; check GUI connection/calibration.")

    def destroy_node(self):
        if hasattr(self, "receiver"):
            self.receiver.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GloveBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
