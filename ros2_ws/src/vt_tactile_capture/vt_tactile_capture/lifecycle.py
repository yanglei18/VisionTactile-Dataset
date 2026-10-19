"""Orderly CLI cleanup under repeated ros2 launch termination signals."""

import signal

import rclpy


def shutdown_ros(node):
    # A terminal sends SIGINT to the process group, and launch can forward it
    # again while the first signal is already stopping the serial worker/Qt.
    previous = {number: signal.signal(number, signal.SIG_IGN)
                for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)
