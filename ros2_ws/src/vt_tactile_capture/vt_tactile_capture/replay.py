"""Rebuild rolling state from recorded decoded samples without serial hardware."""

from dataclasses import fields
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vt_tactile_msgs.msg import TactileSampleArray, TactileState

from .assembler import Assembler
from .lifecycle import shutdown_ros
from .protocol import Block
from .ros_messages import heatmap_message, state_message
from .serial_source import Batch


class TactileReplayNode(Node):
    def __init__(self, **kwargs):
        super().__init__("tactile_replay", **kwargs)
        self.declare_parameters("", [("hand", "right"), ("frame_id", ""),
                                     ("state_rate_hz", 30.0), ("max_cell_age_ms", 100.0),
                                     ("publish_heatmap", False)])
        self.hand = self.get_parameter("hand").value
        self.frame_id = self.get_parameter("frame_id").value or f"{self.hand}_tactile"
        rate = self.get_parameter("state_rate_hz").value
        max_age = self.get_parameter("max_cell_age_ms").value
        if not math.isfinite(rate) or rate <= 0 or not math.isfinite(max_age) or max_age <= 0:
            super().destroy_node()
            raise ValueError("state_rate_hz and max_cell_age_ms must be finite and positive")
        self.max_age_ns = int(max_age * 1e6)
        self.assembler = Assembler(hand=self.hand)
        self._last_batch = None
        self._last_clock_ns = None
        self.rejected_samples = 0
        self.state_publisher = self.create_publisher(
            TactileState, "state", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                             durability=DurabilityPolicy.VOLATILE))
        self.heatmap_publisher = self.create_publisher(Image, "heatmap", 1) if self.get_parameter("publish_heatmap").value else None
        self.sample_subscription = self.create_subscription(
            TactileSampleArray, "samples", self.on_samples,
            QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        # ROS-time timer intentionally pauses with the bag clock. The viewer's
        # separate steady arrival watchdog then explains pause/stopped playback.
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        if not self.get_parameter("use_sim_time").value:
            self.get_logger().warning("Replay should use use_sim_time:=true with ros2 bag play --clock.")

    def on_samples(self, message):
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (message.schema_version != 1 or message.hand != self.hand
                or stamp_ns != message.host_realtime_ns
                or message.clock_domain != "CLOCK_MONOTONIC" or message.timing_source != "host_receive"
                or any(block.row >= 80 or block.column_group >= 8 or len(block.adc) != 10
                       for block in message.blocks)):
            self.rejected_samples += 1
            self.assembler = Assembler(hand=self.hand)
            self._last_batch = None
            return
        # A bag seek/loop must not mix the previous playback position's grid.
        previous = self._last_batch
        now_ros_ns = self.get_clock().now().nanoseconds
        clock_rewound = self._last_clock_ns is not None and now_ros_ns < self._last_clock_ns
        if clock_rewound or (previous is not None and (
            message.session_id == previous.session_id and message.sequence <= previous.sequence
        )):
            self.assembler = Assembler(hand=self.hand)
            self._last_clock_ns = now_ros_ns
        # Generated fixed-size ROS arrays may contain numpy.uint16 rather than
        # Python int. Normalize at the wire boundary for the pure core validator.
        values = {field.name: getattr(message, field.name) for field in fields(Batch)
                  if field.name != "blocks"}
        values["blocks"] = tuple(Block(row=int(block.row), column_group=int(block.column_group),
                                       status_raw=int(block.status_raw), adc=tuple(map(int, block.adc)))
                                 for block in message.blocks)
        self.assembler.apply(Batch(**values))
        self._last_batch = message

    def publish_state(self):
        if self._last_batch is None:
            return
        batch = self._last_batch
        now_ros_ns = self.get_clock().now().nanoseconds
        if self._last_clock_ns is not None and now_ros_ns < self._last_clock_ns:
            # Wait for data at the new playback position instead of restamping
            # data from the future after a clock rewind.
            self.assembler = Assembler(hand=self.hand)
            self._last_batch = None
            self._last_clock_ns = now_ros_ns
            return
        self._last_clock_ns = now_ros_ns
        now_recorded_monotonic_ns = batch.host_monotonic_ns + max(0, now_ros_ns - batch.host_realtime_ns)
        state = state_message(self.assembler.snapshot(now_recorded_monotonic_ns, self.max_age_ns), self.frame_id)
        self.state_publisher.publish(state)
        if self.heatmap_publisher is not None:
            self.heatmap_publisher.publish(heatmap_message(state))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TactileReplayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown_ros(node)
