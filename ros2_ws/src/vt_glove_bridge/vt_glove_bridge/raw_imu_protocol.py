"""Validate original IMU batches without importing the numerical hand runtime."""
from collections import OrderedDict
import json
import re

from .transport import MAX_PACKET_BYTES

SAMPLE_UINTS = {"node_id": 8, "sample_index": 64, "filter_epoch": 32,
                "host_realtime_ns": 64, "host_monotonic_ns": 64, "status": 8}
SAMPLE_BOOLS = ("time_valid", "status_known", "gyro_rejected", "accel_rejected", "fusion_updated")
BATCH_UINTS = ("batch_sequence", "producer_dropped_batches_total", "producer_dropped_samples_total")


def _uint(value, bits, name):
    if type(value) is not int or not 0 <= value < 2**bits:
        raise ValueError(f"{name} must be uint{bits}")


def decode_batch(payload, *, expected_hand, allow_replay=False):
    if not isinstance(payload, bytes) or len(payload) > MAX_PACKET_BYTES:
        raise ValueError("invalid raw IMU packet size")
    try:
        batch = json.loads(payload)
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ValueError("invalid raw IMU JSON") from exc
    if not isinstance(batch, dict) or batch.get("kind") != "raw_imu_batch" or type(batch.get("schema_version")) is not int or batch["schema_version"] != 1:
        raise ValueError("unsupported raw IMU schema")
    if batch.get("hand") != expected_hand or expected_hand not in ("left", "right"):
        raise ValueError("raw IMU hand mismatch")
    if batch.get("source_mode") not in (("live", "replay") if allow_replay else ("live",)):
        raise ValueError("raw IMU source mode is not enabled")
    if not isinstance(batch.get("session_id"), str) or not 1 <= len(batch["session_id"].encode("utf-8")) <= 256:
        raise ValueError("invalid raw IMU session")
    if not isinstance(batch.get("config_sha256"), str) or not re.fullmatch("[0-9a-f]{64}", batch["config_sha256"]):
        raise ValueError("invalid raw IMU configuration digest")
    if batch.get("clock_domain") != "CLOCK_MONOTONIC" or batch.get("timing_source") != "host_receive":
        raise ValueError("unsupported raw IMU clock semantics")
    for key in BATCH_UINTS:
        _uint(batch.get(key), 64, key)
    if not batch["batch_sequence"]:
        raise ValueError("raw IMU sequence must start at 1")
    samples = batch.get("samples")
    if not isinstance(samples, list) or not 1 <= len(samples) <= 64:
        raise ValueError("raw IMU batch must contain 1..64 samples")
    for sample in samples:
        if not isinstance(sample, dict) or set(sample) != set(SAMPLE_UINTS) | set(SAMPLE_BOOLS) | {"raw_values"}:
            raise ValueError("invalid raw IMU sample fields")
        for key, bits in SAMPLE_UINTS.items():
            _uint(sample[key], bits, key)
        if not 1 <= sample["node_id"] <= 12 or not sample["sample_index"]:
            raise ValueError("invalid raw IMU node/sample identity")
        if any(type(sample[key]) is not bool for key in SAMPLE_BOOLS):
            raise ValueError("invalid raw IMU flags")
        values = sample["raw_values"]
        if not isinstance(values, list) or len(values) != 6 or any(type(v) is not int or not -32768 <= v <= 32767 for v in values):
            raise ValueError("raw IMU values must be six original int16 values")
        valid_time = bool(sample["host_realtime_ns"] and sample["host_monotonic_ns"])
        if sample["time_valid"] != valid_time or (batch["source_mode"] == "live" and not valid_time):
            raise ValueError("raw IMU source clocks are missing or inconsistent")
        if not sample["status_known"] and sample["status"] != 0:
            raise ValueError("unknown raw IMU status must have zero placeholder")
        if sample["host_realtime_ns"] // 1_000_000_000 >= 2**31:
            raise ValueError("raw IMU realtime exceeds ROS stamp range")
    return batch


class RawSequenceTracker:
    def __init__(self):
        self.sessions = OrderedDict()
        self.gaps_total = 0

    def accept(self, batch):
        key = (batch["hand"], batch["session_id"])
        previous, indices = self.sessions.get(key, (0, {}))
        sequence = batch["batch_sequence"]
        if sequence <= previous:
            raise ValueError("duplicate or out-of-order raw IMU batch")
        indices = dict(indices)
        for sample in batch["samples"]:
            node, index = sample["node_id"], sample["sample_index"]
            if index <= indices.get(node, 0):
                raise ValueError("duplicate or out-of-order raw IMU node sample")
            indices[node] = index
        self.gaps_total += sequence - previous - 1
        self.sessions[key] = (sequence, indices)
        self.sessions.move_to_end(key)
        if len(self.sessions) > 64:
            self.sessions.popitem(last=False)


def populate_batch(message, batch, sample_type, *, realtime_ns, monotonic_ns, gaps_total, rejected_total):
    for key in ("schema_version", "hand", "source_mode", "session_id", "config_sha256", "clock_domain", "timing_source", *BATCH_UINTS):
        setattr(message, key, batch[key])
    stamp = batch["samples"][-1]["host_realtime_ns"]
    message.header.frame_id = f"glove_{batch['hand']}_raw_imu"
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.samples = []
    for row in batch["samples"]:
        sample = sample_type()
        for key, value in row.items():
            setattr(sample, key, value)
        message.samples.append(sample)
    message.bridge_received_realtime_ns = realtime_ns
    message.bridge_received_monotonic_ns = monotonic_ns
    message.bridge_sequence_gaps_total = gaps_total
    message.bridge_rejected_total = rejected_total
    return message
