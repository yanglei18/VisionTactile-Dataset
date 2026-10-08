"""ROS snapshots with optional managed recording; ROS imports remain lazy."""
from __future__ import annotations

import argparse
import time

from .config import load_config
from .model import SnapshotModel


def build_node(config, *, context=None, model=None):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from vt_tracker_msgs.msg import TrackerSample
    from vt_glove_msgs.msg import GloveState
    from vt_tactile_msgs.msg import TactileState
    from vt_camera_msgs.msg import CaptureStatus
    from .image_preview import ImagePreviewWorker

    class CaptureManagerNode(Node):
        def __init__(self):
            super().__init__("vt_capture_manager", context=context, enable_rosout=False,
                             start_parameter_services=False)
            self.model = model or SnapshotModel(config)
            self._preview_worker = ImagePreviewWorker(self.model).start()
            self._subscriptions_owned = []
            types = dict(camera=Image, tracker=TrackerSample, glove=GloveState, tactile=TactileState)
            for device in config.devices:
                reliability = ReliabilityPolicy.RELIABLE if device.kind == "glove" else ReliabilityPolicy.BEST_EFFORT
                qos = QoSProfile(depth=5, reliability=reliability, durability=DurabilityPolicy.VOLATILE)
                self._subscriptions_owned.append(self.create_subscription(
                    types[device.kind], device.topic,
                    lambda message, identity=device.id, kind=device.kind: self._observe(identity, kind, message), qos))
            self._subscriptions_owned.append(self.create_subscription(
                CaptureStatus, config.capture_status_topic, self.model.observe_capture,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)))
            self.model.event("info", "ROS stream monitor: no camera, tracker or serial device ownership.")

        def _observe(self, identity, kind, message):
            self.model.observe(identity, message)
            if kind == "camera":
                self._preview_worker.submit(identity, message)

        def destroy_node(self):
            self._preview_worker.close()
            return super().destroy_node()

    return CaptureManagerNode()


def main(argv=None):
    parser = argparse.ArgumentParser(description="ROS capture manager; recording and device control are separate explicit opt-ins")
    parser.add_argument("--config", help="Manager JSON/YAML subscription configuration")
    parser.add_argument("--socket", help="Unix socket in an owned private directory")
    parser.add_argument("--enable-recording", action="store_true", help="Explicitly enable durable managed recorder commands; default is read-only")
    parser.add_argument("--enable-device-control", action="store_true", help="Enable manual launch-profile start/stop; never autostart or take over external nodes")
    parser.add_argument("--state-dir", help="Private management directory; recording DB and/or process journal are created only when enabled")
    parser.add_argument("--desktop-apply-lock", help="Private command-admission lock used by the desktop configuration launcher")
    args, ros_args = parser.parse_known_args(argv)
    config = load_config(args.config)
    import rclpy
    from .server import SnapshotServer
    from .management import ManagementService
    rclpy.init(args=ros_args)
    node = None
    server = None
    management = None
    processes = None
    try:
        node = build_node(config)
        adapter = None
        if args.enable_recording or args.enable_device_control:
            from .recorder_adapter import ManagedRosAdapter
            adapter = ManagedRosAdapter(node)
        management = ManagementService(node.model, adapter, enabled=args.enable_recording, state_dir=args.state_dir,
                                       admission_lock=args.desktop_apply_lock)
        if args.enable_device_control:
            from .device_processes import ProcessSupervisor
            from .launch_profiles import build_specs
            from .management import default_state_dir
            from pathlib import Path
            specs = build_specs(config.process_groups)
            discovered_at = time.monotonic()

            def controller_present():
                if time.monotonic() - discovered_at < 1.:
                    raise RuntimeError("initial ROS discovery")
                return node.count_publishers(config.capture_status_topic) > 0

            def observe():
                if time.monotonic() - discovered_at < 1.:
                    raise RuntimeError("initial ROS discovery")
                fresh = node.model.topic_health()
                return {topic: dict(publishers=node.count_publishers(topic), fresh=fresh.get(topic, False))
                        for spec in specs for topic in spec.topics}

            management.controller_present = controller_present
            processes = ProcessSupervisor(specs, instance_id=node.model.instance_id,
                state_dir=Path(args.state_dir or default_state_dir()) / "processes", observe=observe,
                allowed=management.lifecycle_allowed, event=node.model.event)
            management.processes = processes
            processes.start()
        management.start()
        server = SnapshotServer(node.model, args.socket, management=management).start()
        node.get_logger().info(f"Capture snapshots (managed recording {'enabled' if args.enable_recording else 'disabled'}): {server.path}")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.close()
        if processes is not None:
            processes.close()  # Detach, never stop sensors just because the manager exits.
        if management is not None:
            management.close()  # Never sends STOP to an external recorder.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
