"""ROS snapshots with optional managed recording; ROS imports remain lazy."""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import os
import sys
import time

from .config import load_config
from .model import SnapshotModel


def build_node(config, *, context=None, model=None):
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from vt_tracker_msgs.msg import TrackerSample
    from vt_glove_msgs.msg import GloveState
    from vt_tactile_msgs.msg import TactileSampleArray, TactileState
    from vt_camera_msgs.msg import CaptureStatus
    from .image_preview import ImagePreviewWorker

    class CaptureManagerNode(Node):
        def __init__(self):
            super().__init__("vt_capture_manager", context=context, enable_rosout=False,
                             start_parameter_services=False)
            self.model = model or SnapshotModel(config)
            self.scene_store = None
            self.spatial_adapter = None
            if config.scene_config_path:
                from .spatial.config import load_scene_config
                from .spatial.scene_store import SceneStore
                scene_config = load_scene_config(config.scene_config_path)
                if scene_config.enabled:
                    self.scene_store = SceneStore(self.model.instance_id, scene_config)
                    if scene_config.cameras:
                        from .spatial.ros_adapter import SpatialRosAdapter
                        self.spatial_adapter = SpatialRosAdapter(self, self.scene_store)
            self._preview_worker = ImagePreviewWorker(self.model).start()
            self._subscriptions_owned = []
            self._image_callback_groups = []
            types = dict(camera=Image, tracker=TrackerSample, glove=GloveState, tactile=TactileState)
            for device in config.devices:
                reliability = ReliabilityPolicy.RELIABLE if device.kind == "glove" else ReliabilityPolicy.BEST_EFFORT
                group = None
                depth = 5
                if device.kind == "camera":
                    reliability = (ReliabilityPolicy.RELIABLE if device.image_reliability == "reliable"
                                   else ReliabilityPolicy.BEST_EFFORT)
                    # A bounded DDS history permits retransmission of large images without
                    # building an unbounded stale queue. Each stream stays ordered while
                    # the default callback group remains available for control/status.
                    depth = 8
                    group = MutuallyExclusiveCallbackGroup()
                    self._image_callback_groups.append(group)
                qos = QoSProfile(depth=depth, reliability=reliability, durability=DurabilityPolicy.VOLATILE)
                self._subscriptions_owned.append(self.create_subscription(
                    types[device.kind], device.topic,
                    lambda message, identity=device.id, kind=device.kind: self._observe(identity, kind, message),
                    qos, callback_group=group))
            for topic in self.model.sample_activity_topics():
                self._subscriptions_owned.append(self.create_subscription(
                    TactileSampleArray, topic,
                    lambda _message, name=topic: self.model.observe_topic_activity(name),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE), raw=True))
            self._subscriptions_owned.append(self.create_subscription(
                CaptureStatus, config.capture_status_topic, self.model.observe_capture,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)))
            self.model.event("info", "ROS stream monitor: no camera, tracker or serial device ownership.")

        def _observe(self, identity, kind, message):
            self.model.observe(identity, message)
            if kind == "tracker" and self.scene_store is not None:
                self.scene_store.observe_tracker(identity, message)
            if kind == "camera":
                self._preview_worker.submit(identity, message)

        def destroy_node(self):
            if self.spatial_adapter is not None:
                self.spatial_adapter.close()
            self._preview_worker.close()
            return super().destroy_node()

    return CaptureManagerNode()


def build_executor(node):
    """Keep image callbacks independent of recorder responses with bounded workers."""
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4, context=node.context)
    executor.add_node(node)
    return executor


def main(argv=None):
    parser = argparse.ArgumentParser(description="ROS capture manager; recording and device control are separate explicit opt-ins")
    parser.add_argument("--config", help="Manager JSON/YAML subscription configuration")
    parser.add_argument("--socket", help="Unix socket in an owned private directory")
    parser.add_argument("--enable-recording", action="store_true", help="Explicitly enable durable managed recorder commands; default is read-only")
    parser.add_argument("--enable-device-control", action="store_true", help="Enable manual launch-profile start/stop; never autostart or take over external nodes")
    parser.add_argument("--enable-calibration", action="store_true", help="Enable user-started guided/offline calibration sessions and retained result publication")
    parser.add_argument("--calibration-python", default=sys.executable, help="Python executable in the separately installed calibration environment")
    parser.add_argument("--calibration-root", help="External directory for calibration sessions and versioned active results")
    parser.add_argument("--state-dir", help="Private management directory; recording DB and/or process journal are created only when enabled")
    parser.add_argument("--desktop-apply-lock", help="Private command-admission lock used by the desktop configuration launcher")
    parser.add_argument("--log-path", help="Retain the private file already receiving this process's stdout; no output redirection")
    args, ros_args = parser.parse_known_args(argv)
    from vt_capture_core.process_log import ProcessLogRetention
    retention = (ProcessLogRetention(args.log_path, output_fd=sys.stdout.fileno())
                 if args.log_path else nullcontext())
    with retention:
        return _run_manager(args, ros_args)


def _run_manager(args, ros_args):
    config = load_config(args.config)
    from vt_capture_core.ros_transport import capture_transport_environment
    environment = capture_transport_environment(os.environ)
    if environment != dict(os.environ):
        print("Capture transport: bundled Fast DDS profile (64 MiB SHM per participant, queue 4096).", flush=True)
    else:
        print("Capture transport: inherited middleware and transport configuration.", flush=True)
    os.environ.update(environment)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from .server import SnapshotServer
    from .management import ManagementService
    rclpy.init(args=ros_args)
    node = None
    server = None
    scene_server = None
    geometry_recorder = None
    cloud_supervisor = None
    management = None
    processes = None
    calibration_adapter = None
    executor = None
    try:
        node = build_node(config)
        adapter = None
        if args.enable_recording or args.enable_device_control or getattr(args, "enable_calibration", False):
            from .recorder_adapter import ManagedRosAdapter
            adapter = ManagedRosAdapter(node)
        management = ManagementService(node.model, adapter, enabled=args.enable_recording, state_dir=args.state_dir,
                                       admission_lock=args.desktop_apply_lock)
        if args.enable_recording and node.scene_store is not None:
            from .spatial.geometry_recording import GeometryRecorder
            geometry_recorder = GeometryRecorder(node.scene_store, event=node.model.event)
            management.geometry_recorder = geometry_recorder
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
        if getattr(args, "enable_calibration", False):
            from pathlib import Path
            from .calibration_runtime import attach_calibration
            output = next((p["parameters"].get("output_root") for p in config.process_groups
                           if p["preset"] == "cameras" and p["parameters"].get("output_root")), None)
            root = (getattr(args, "calibration_root", None)
                    or (Path(output) if output else Path.home() / "visiontactile-data") / "calibration")
            try:
                calibration_adapter = attach_calibration(node, management, root_dir=root,
                    worker_python=getattr(args, "calibration_python", sys.executable))
            except (ValueError, OSError, ImportError, KeyError, RuntimeError) as exc:
                management.calibration_catalog["detail"] = f"Calibration unavailable: {exc}"[:1024]
                node.model.event("warning", f"Calibration unavailable: {exc}", key="calibration-setup")
        management.start()
        server = SnapshotServer(node.model, args.socket, management=management).start()
        if node.scene_store is not None:
            from .spatial.scene_server import SceneServer
            if node.scene_store.config.cloud.get('enabled') and node.scene_store.config.cameras:
                from .spatial.cloud_supervisor import CloudSupervisor
                cloud_supervisor = CloudSupervisor(node.scene_store, server.path.parent / "manager-cloud.sock")
                node.scene_store.cloud_supervisor = cloud_supervisor
                cloud_supervisor.start()
            scene_server = SceneServer(node.scene_store, server.path.parent / "manager-scene.sock").start()
        node.get_logger().info(f"Capture snapshots (managed recording {'enabled' if args.enable_recording else 'disabled'}): {server.path}")
        executor = build_executor(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            # Finish callbacks before destroying subscriptions or the preview worker.
            executor.shutdown()
        if scene_server is not None:
            scene_server.close()
        if cloud_supervisor is not None:
            cloud_supervisor.close()
        if server is not None:
            server.close()
        if processes is not None:
            processes.close()  # Detach, never stop sensors just because the manager exits.
        if management is not None:
            management.close()  # Never sends STOP to an external recorder.
        if calibration_adapter is not None:
            calibration_adapter.close()
        if geometry_recorder is not None:
            geometry_recorder.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
