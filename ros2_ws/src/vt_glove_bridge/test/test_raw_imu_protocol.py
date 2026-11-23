"""Raw observations keep their own identity; pose validity cannot rewrite them."""
from copy import deepcopy
import json

import pytest

from vt_glove_bridge.raw_imu_protocol import RawSequenceTracker, decode_batch


def batch(sequence=1, index=1, hand="left"):
    return dict(kind="raw_imu_batch", schema_version=1, hand=hand, source_mode="live",
                session_id="session", config_sha256="a" * 64, batch_sequence=sequence,
                clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
                producer_dropped_batches_total=0, producer_dropped_samples_total=0,
                samples=[dict(node_id=9, sample_index=index, filter_epoch=2,
                    host_realtime_ns=1_800_000_000_000_000_123, host_monotonic_ns=10_000_000_123,
                    time_valid=True, raw_values=[32767, -32768, 0, 1, 2, 3], status=255,
                    status_known=True, gyro_rejected=True, accel_rejected=False, fusion_updated=False)])


def decode(value, **kwargs):
    return decode_batch(json.dumps(value).encode(), expected_hand="left", **kwargs)


def test_rejected_raw_values_and_integer_clocks_survive_wire_validation():
    value = batch()
    assert decode(value) == value
    with pytest.raises(ValueError, match="hand mismatch"):
        decode(batch(hand="right"))


@pytest.mark.parametrize("key,value", [("node_id", 0), ("sample_index", True),
    ("host_realtime_ns", 1.8e18), ("raw_values", [0] * 5), ("raw_values", [32768] * 6),
    ("status", 256), ("gyro_rejected", 1), ("time_valid", False)])
def test_malformed_samples_are_rejected(key, value):
    document = batch()
    document["samples"][0][key] = value
    with pytest.raises(ValueError):
        decode(document)


def test_replay_does_not_invent_integer_precision_or_enable_itself():
    value = batch()
    value["source_mode"] = "replay"
    value["samples"][0].update(host_realtime_ns=0, host_monotonic_ns=0, time_valid=False,
                                status_known=False, status=0)
    with pytest.raises(ValueError, match="source mode"):
        decode(value)
    assert decode(value, allow_replay=True) == value


def test_batch_and_per_node_order_fail_atomically_and_count_missing_batches():
    tracker = RawSequenceTracker()
    tracker.accept(decode(batch(sequence=3)))
    assert tracker.gaps_total == 2
    bad = batch(sequence=4, index=2)
    bad["samples"].append(deepcopy(bad["samples"][0]))
    with pytest.raises(ValueError, match="node sample"):
        tracker.accept(decode(bad))
    tracker.accept(decode(batch(sequence=4, index=2)))
    assert tracker.gaps_total == 2
    with pytest.raises(ValueError, match="batch"):
        tracker.accept(decode(batch(sequence=4, index=3)))
