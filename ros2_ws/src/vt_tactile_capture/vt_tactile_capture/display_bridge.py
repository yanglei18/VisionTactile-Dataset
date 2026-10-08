"""One-way ROS tactile state to local display IPC; no hardware ownership."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from vt_tactile_msgs.msg import TactileState

from .display_transport import LocalTactileSender, default_socket_path
from .lifecycle import shutdown_ros


class TactileDisplayBridge(Node):
    def __init__(self, *, sender_factory=LocalTactileSender, **kwargs):
        super().__init__("tactile_display_bridge", **kwargs)
        self.sender = None
        self.received = self.rejected = 0
        self._last_health = None
        try:
            self.declare_parameters("", [
                ("topic", ""), ("socket_path", ""), ("hand", "right"),
            ])
            self.config = {name: self.get_parameter(name).value for name in ("topic", "socket_path", "hand")}
            self._startup_parameters = self.config.copy()
            if self.get_parameter("use_sim_time").value:
                raise ValueError("Live tactile display bridge rejects use_sim_time; replay is not supported here.")
            if self.config["hand"] not in ("left", "right"):
                raise ValueError("hand must be left or right")
            namespace = self.get_namespace().rstrip("/") or f"/gloves/{self.config['hand']}/tactile"
            self.config["topic"] = self.config["topic"] or namespace + "/state"
            if not self.config["topic"].startswith("/") or self.config["topic"] == "/":
                raise ValueError("topic must be an absolute ROS topic name")
            self.config["socket_path"] = (self.config["socket_path"] or
                default_socket_path(self.config["hand"], self.config["topic"].rsplit("/", 1)[0]))
            self.sender = sender_factory(self.config["socket_path"])
            self.subscription = self.create_subscription(
                TactileState, self.config["topic"], self.on_state,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE),
            )
            self.add_on_set_parameters_callback(self._validate_parameter_change)
            self.health_timer = self.create_timer(5.0, self._health)
            self.get_logger().info(
                f"Read-only tactile display bridge: {self.config['topic']} -> {self.config['socket_path']}; "
                "best-effort display only, no serial access or capture changes.")
        except BaseException:
            self.destroy_node()
            raise

    def _validate_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name == "use_sim_time" and parameter.value:
                return SetParametersResult(successful=False, reason="Live display bridge requires system time.")
            if parameter.name in self._startup_parameters and parameter.value != self._startup_parameters[parameter.name]:
                return SetParametersResult(successful=False, reason="Display bridge parameters are startup-only.")
        return SetParametersResult(successful=True)

    def on_state(self, message):
        self.received += 1
        if (message.hand != self.config["hand"] or message.clock_domain != "CLOCK_MONOTONIC"
                or message.timing_source != "host_receive"):
            self.rejected += 1
            return
        self.sender.send(message)

    def _health(self):
        health = (self.received, self.rejected, self.sender.sent, self.sender.dropped, self.sender.rejected)
        if health != self._last_health:
            self.get_logger().info(
                f"Display IPC totals: ROS={self.received}, sent={self.sender.sent}, "
                f"GUI absent/full={self.sender.dropped}, invalid={self.rejected + self.sender.rejected}. "
                "Display drops do not alter acquisition or recorded ROS data.")
            self._last_health = health

    def destroy_node(self):
        if self.sender is not None:
            self.sender.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TactileDisplayBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown_ros(node)
