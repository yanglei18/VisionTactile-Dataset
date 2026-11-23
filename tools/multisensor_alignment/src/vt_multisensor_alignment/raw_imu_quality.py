"""Streaming observations of raw IMU samples actually read from a closed bag."""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

from .hand_identity import HandStreamIdentity
from .source_counter import SourceCounter


RAW_IMU_TYPE = "vt_glove_msgs/msg/RawImuBatch"


@dataclass
class _Node:
    samples: int = 0
    unobserved: int = 0
    out_of_order: int = 0
    host_regressions: int = 0
    timed_intervals: int = 0
    elapsed_ns: int = 0
    longest_gap_ns: int = 0
    previous_index: int | None = None
    previous_stamp: int | None = None

    def boundary(self):
        self.previous_index = self.previous_stamp = None

    def observe(self, index, stamp):
        self.samples += 1
        if self.previous_index is not None:
            if index <= self.previous_index:
                self.out_of_order += 1
                return
            self.unobserved += index - self.previous_index - 1
        self.previous_index = index
        if stamp is not None and self.previous_stamp is not None and stamp >= self.previous_stamp:
            # A serial read can contain several samples sharing a host stamp.
            delta = stamp - self.previous_stamp
            self.timed_intervals += 1
            self.elapsed_ns += delta
            self.longest_gap_ns = max(self.longest_gap_ns, delta)
        if stamp is not None and self.previous_stamp is not None and stamp < self.previous_stamp:
            self.host_regressions += 1
        self.previous_stamp = stamp

    def report(self):
        return dict(samples=self.samples, unobserved_sample_indices=self.unobserved,
                    repeated_or_reordered_indices=self.out_of_order, host_timestamp_regressions=self.host_regressions,
                    observed_sample_hz=(self.timed_intervals * 1e9 / self.elapsed_ns if self.elapsed_ns else None),
                    longest_host_gap_ns=self.longest_gap_ns if self.timed_intervals else None)


class RawImuQuality:
    def __init__(self, topic):
        self.topic = topic
        self.identity = HandStreamIdentity(pose=False)
        self.nodes = {}
        self.context = None
        self.previous_batch = None
        self.counts = Counter()
        self.first_error = None
        self.transport = {name: SourceCounter() for name in (
            "producer_dropped_batches_total", "producer_dropped_samples_total",
            "bridge_sequence_gaps_total", "bridge_rejected_total")}

    def decode_failure(self, receipt_ns, error):
        self.counts["observed"] += 1
        self.counts["malformed"] += 1
        if self.first_error is None:
            self.first_error = str(error)[:400]

    def add(self, message, receipt_ns):
        try:
            hand, session = message.hand, message.session_id
            if (message.schema_version != 1 or hand not in ("left", "right") or not session
                    or self.topic != f"/gloves/{hand}/imu/raw" or message.batch_sequence < 1):
                raise ValueError("Invalid raw IMU schema, source identity or sequence")
            if (message.clock_domain != "CLOCK_MONOTONIC" or message.timing_source != "host_receive"
                    or not message.config_sha256 or not message.source_mode):
                raise ValueError("Unsupported raw IMU clock, timing source or configuration")
            counters = {name: getattr(message, name) for name in self.transport}
            if any(type(value) is not int or value < 0 for value in counters.values()):
                raise ValueError("Invalid raw IMU transport counters")
            if not 1 <= len(message.samples) <= 64 or any(s.node_id not in range(1, 13) or s.sample_index < 1
                    or len(s.raw_values) != 6 for s in message.samples):
                raise ValueError("Invalid raw IMU sample batch")
            ctx = (hand, session, message.config_sha256, message.source_mode)
            if ctx != self.context:
                self.counts["segments"] += 1
                self.context = ctx
                self.previous_batch = None
                for node in self.nodes.values():
                    node.boundary()
            sequence = message.batch_sequence
            if self.previous_batch is not None:
                self.counts["batch_gaps"] += max(0, sequence - self.previous_batch - 1)
                self.counts["reordered_batches"] += sequence <= self.previous_batch
            self.previous_batch = max(self.previous_batch or 0, sequence)
            for name, value in counters.items():
                self.transport[name].add(value, ctx)
            for sample in message.samples:
                node = self.nodes.get(sample.node_id)
                if node is None:
                    node = self.nodes[sample.node_id] = _Node()
                stamp = sample.host_monotonic_ns if sample.time_valid and sample.host_monotonic_ns > 0 else None
                node.observe(sample.sample_index, stamp)
                self.counts["unknown_time"] += stamp is None
                self.counts["gyro_rejected"] += bool(sample.gyro_rejected)
                self.counts["accel_rejected"] += bool(sample.accel_rejected)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self.decode_failure(receipt_ns, exc)
            return
        self.identity.observe(message)
        self.counts["observed"] += 1

    def document(self):
        return dict(inspection="raw_imu_samples", hand_source=self.identity.document(), observed_messages=self.counts["observed"],
                    malformed_messages=self.counts["malformed"], first_decode_error=self.first_error,
                    recorded_samples=sum(node.samples for node in self.nodes.values()),
                    source_segments=self.counts["segments"], unobserved_batches=self.counts["batch_gaps"],
                    repeated_or_reordered_batches=self.counts["reordered_batches"],
                    unknown_source_time_samples=self.counts["unknown_time"],
                    rejected_gyro_samples=self.counts["gyro_rejected"],
                    rejected_accel_samples=self.counts["accel_rejected"],
                    cumulative_counters={k: counter.document() for k, counter in self.transport.items()},
                    nodes={str(n): state.report() for n, state in sorted(self.nodes.items())})
