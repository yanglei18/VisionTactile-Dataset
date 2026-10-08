"""One serial owner, bounded worker queue, loss-aware ROS tactile output.

Only decoded sample batches are archival measurements. The timer-driven state
and optional RGB heatmap retain the acquisition timestamp of their source.
"""

from __future__ import annotations

import math
from numbers import Integral
import time
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vt_tactile_msgs.msg import TactileSampleArray, TactileState

from .assembler import Assembler
from .grid import CELL_COUNT, COLS, ROWS, SCHEMA_VERSION
from .lifecycle import shutdown_ros
from .serial_source import Batch, MAX_READ_BYTES, SerialWorker
from .ros_messages import heatmap_message, sample_message, state_message


class TactileNode(Node):
    def __init__(self, *, source_factory=SerialWorker, monotonic_ns=time.monotonic_ns,
                 context=None, parameter_overrides=None, **kwargs):
        super().__init__("tactile_capture", context=context,
                         parameter_overrides=parameter_overrides, **kwargs)
        defaults = {
            "port": "", "baudrate": 115200, "hand": "right", "frame_id": "",
            "queue_capacity": 512, "sample_qos_depth": 128,
            "read_batch_bytes": 1024, "read_timeout_sec": 0.002,
            "state_rate_hz": 30.0, "diagnostics_rate_hz": 1.0,
            "stale_timeout_sec": 2.0, "max_cell_age_ms": 100.0, "reconnect_interval_sec": 1.0,
            "publish_heatmap": False,
        }
        self.declare_parameters("", list(defaults.items()))
        self.config = {name: self.get_parameter(name).value for name in defaults}
        try:
            if self.get_parameter("use_sim_time").value:
                raise ValueError("Live tactile acquisition rejects use_sim_time; use the independent viewer for replay.")
            for name in ("queue_capacity", "sample_qos_depth", "baudrate"):
                if self.config[name] < 1:
                    raise ValueError(f"{name} must be positive")
            read_batch_bytes = self.config["read_batch_bytes"]
            if (isinstance(read_batch_bytes, bool) or not isinstance(read_batch_bytes, Integral)
                    or not 1 <= read_batch_bytes <= MAX_READ_BYTES):
                raise ValueError(f"read_batch_bytes must be an integer in [1, {MAX_READ_BYTES}]")
            for name in ("state_rate_hz", "diagnostics_rate_hz", "stale_timeout_sec", "max_cell_age_ms", "reconnect_interval_sec", "read_timeout_sec"):
                if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                    raise ValueError(f"{name} must be finite and positive")
            if self.config["hand"] not in ("left", "right"):
                raise ValueError("hand must be 'left' or 'right'")
        except Exception:
            super().destroy_node()
            raise
        self.add_on_set_parameters_callback(self._validate_parameter_change)
        self._monotonic_ns = monotonic_ns
        self.frame_id = self.config["frame_id"] or f"{self.config['hand']}_tactile"
        self.assembler = Assembler(hand=self.config["hand"])
        self._last_batch = None
        self._last_snapshot = None
        self._source_stopped = False
        self._last_diagnostic_summary = None
        self._rate_window_start_ns = self._monotonic_ns()
        self._rate_window_batches = 0
        self._rate_window_packets = 0
        self._batch_rate_hz = 0.0
        self._packet_rate_hz = 0.0
        self.sample_publisher = self.create_publisher(
            TactileSampleArray, "samples",
            QoSProfile(depth=self.config["sample_qos_depth"],
                       reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
        )
        self.state_publisher = self.create_publisher(
            TactileState, "state",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, "diagnostics", 1)
        self.heatmap_publisher = self.create_publisher(Image, "heatmap", 1) if self.config["publish_heatmap"] else None
        self.source = source_factory(
            port=self.config["port"], baudrate=self.config["baudrate"], hand=self.config["hand"],
            queue_capacity=self.config["queue_capacity"], read_timeout=self.config["read_timeout_sec"],
            read_batch_bytes=self.config["read_batch_bytes"],
            no_data_timeout=self.config["stale_timeout_sec"],
            reconnect_interval_sec=self.config["reconnect_interval_sec"],
        ) if self.config["port"].strip() else None
        self._pending_events = []
        self.drain_timer = self.create_timer(0.005, self.drain_samples)
        self.state_timer = self.create_timer(1.0 / self.config["state_rate_hz"], self.publish_state)
        self.diagnostic_timer = self.create_timer(1.0 / self.config["diagnostics_rate_hz"], self.publish_diagnostics)
        if not self.config["port"].strip():
            self.get_logger().warning("No tactile port configured: disconnected. Set an explicit port; automatic port discovery is disabled.")
        if self.source is not None:
            self.source.start()
        else:
            self._pending_events.append(Batch(hand=self.config["hand"], session_id=str(uuid.uuid4()),
                                              reason="port_not_configured"))

    def _validate_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name == "use_sim_time" and parameter.value:
                return SetParametersResult(successful=False, reason="Live tactile acquisition requires system time.")
            if parameter.name in self.config and parameter.value != self.config[parameter.name]:
                return SetParametersResult(successful=False, reason="Capture parameters are startup-only; restart the node to change them.")
        return SetParametersResult(successful=True)

    def drain_samples(self):
        # This queue is independent of the DDS KEEP_LAST history. A blocked DDS
        # publish never makes the serial thread wait on the ROS executor.
        batches = self._pending_events
        self._pending_events = []
        if self.source is not None:
            batches = batches + self.source.drain(limit=128)
        for batch in batches:
            self._last_batch = batch
            self.assembler.apply(batch)
            self.sample_publisher.publish(sample_message(batch, self.frame_id))
            self._rate_window_batches += 1
            self._rate_window_packets += len(batch.blocks)

    def publish_state(self):
        self._last_snapshot = self.assembler.snapshot(
            self._monotonic_ns(), int(self.config["max_cell_age_ms"] * 1e6))
        message = state_message(self._last_snapshot, self.frame_id)
        self.state_publisher.publish(message)
        if self.heatmap_publisher is not None:
            self.heatmap_publisher.publish(heatmap_message(message))

    def publish_diagnostics(self):
        snapshot = self._last_snapshot
        batch = self._last_batch
        now_ns = self._monotonic_ns()
        elapsed_ns = now_ns - self._rate_window_start_ns
        if elapsed_ns > 0:
            self._batch_rate_hz = self._rate_window_batches * 1e9 / elapsed_ns
            self._packet_rate_hz = self._rate_window_packets * 1e9 / elapsed_ns
            self._rate_window_start_ns = now_ns
            self._rate_window_batches = self._rate_window_packets = 0
        valid_cell_count = sum(snapshot.valid_mask) if snapshot else 0
        source_age_ns = (max(0, now_ns - snapshot.host_monotonic_ns)
                         if snapshot and snapshot.host_monotonic_ns else -1)
        reason = snapshot.reason if snapshot else "waiting_for_state"
        if not snapshot or not snapshot.connected:
            level = DiagnosticStatus.ERROR
        elif not snapshot.time_valid or not any(snapshot.valid_mask):
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.OK
        dropped = batch.producer_dropped_total if batch else 0
        if dropped:
            level = max(level, DiagnosticStatus.WARN)
        summary = (level, reason, dropped)
        status = DiagnosticStatus(
            level=level, name=f"tactile/{self.config['hand']}",
            hardware_id=self.config["port"] or "unconfigured", message=reason or "ok",
            values=[KeyValue(key=key, value=str(value)) for key, value in {
                "port": self.config["port"], "unit": "adc_counts",
                "source_sequence": snapshot.source_sequence if snapshot else 0,
                "producer_dropped_total": dropped,
                "discarded_bytes_total": batch.discarded_bytes_total if batch else 0,
                "malformed_packets_total": batch.malformed_packets_total if batch else 0,
                "max_age_ns": snapshot.max_age_ns if snapshot else 0,
                "batch_rate_hz": f"{self._batch_rate_hz:.6f}",
                "packet_rate_hz": f"{self._packet_rate_hz:.6f}",
                "rate_scope": "host_publish; batches_include_status_events; not_hardware_scan_rate",
                "valid_cell_count": valid_cell_count,
                "cell_count": CELL_COUNT, "rows": ROWS, "cols": COLS,
                "state_schema_version": SCHEMA_VERSION,
                "source_age_ns": source_age_ns,
                "state_is_rolling": True,
                "checksum_verified": False,
            }.items()],
        )
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diagnostic_publisher.publish(array)
        if summary != self._last_diagnostic_summary:
            if level != DiagnosticStatus.OK:
                self.get_logger().warning(f"Tactile status: {status.message}; dropped batches={dropped}")
            self._last_diagnostic_summary = summary

    def destroy_node(self):
        if not self._source_stopped:
            self._source_stopped = True
            if self.source is not None and not self.source.stop(timeout=1.0):
                self.get_logger().error("Tactile serial worker did not stop within 1 second.")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TactileNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown_ros(node)
