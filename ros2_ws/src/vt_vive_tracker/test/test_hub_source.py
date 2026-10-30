import json
from copy import deepcopy

import pytest

from vt_vive_tracker.hub_source import HubDecoder, load_bindings, source_info, AnchoredHostClock, HubClockError


@pytest.fixture
def bindings(tmp_path):
    path = tmp_path / "hub.yaml"
    path.write_text("schema_version: 1\nroles:\n  torso: HEAD\n  right_wrist: RIGHT\n  left_wrist: LEFT\n")
    path.chmod(0o600)
    return path


@pytest.fixture
def event():
    return dict(type="pose", serial="RIGHT", device_id=4, state=18, timestamp_ns=1_000_000_000,
                receive_monotonic_ns=1_000_000_100, receive_realtime_ns=9_000_000_100, receive_clock_span_ns=100,
                position=[1, 2, 3], orientation=[0.5, 0.5, 0.5, 0.5], angular_velocity=[0, 0, 0])


def test_serial_identity_and_time_translation(bindings, event):
    serials, roles = load_bindings(bindings)
    decoder = HubDecoder(serials, roles)
    sample = decoder.decode(event, 1_001_000_000)
    assert sample.role == "right_wrist" and sample.tracker_id == roles.by_role["right_wrist"]
    assert sample.host_monotonic_ns == 1_000_000_000
    assert sample.host_realtime_ns == 9_000_000_000
    assert sample.pose_valid and sample.tracking_status == 18
    event.update(device_id=1, timestamp_ns=1_000_000_001)
    assert decoder.decode(event, 1_001_000_000).role == "right_wrist"


@pytest.mark.parametrize("state,valid", [(2, True), (18, True), (3, False), (4, False),
                                        (5, False), (6, False), (19, False), (255, False)])
def test_ready_only(bindings, event, state, valid):
    event["state"] = state
    assert HubDecoder(*load_bindings(bindings)).decode(event, 1_001_000_000).pose_valid == valid


def test_quaternion_is_xyzw_not_reversed(bindings, event):
    event["orientation"] = [0, 0, 0.6, 0.8]
    sample = HubDecoder(*load_bindings(bindings)).decode(event, 1_001_000_000)
    assert sample.quaternion_xyzw == (0.0, 0.0, 0.6, 0.8)


def test_reject_unknown_stale_duplicate_and_backward(bindings, event):
    decoder = HubDecoder(*load_bindings(bindings))
    unknown = dict(event, serial="OTHER")
    assert decoder.decode(unknown, 1_001_000_000) is None
    assert decoder.decode(event, 2_000_000_000) is None
    assert decoder.decode(event, 900_000_000) is None
    assert decoder.decode(event, 1_001_000_000) is not None
    assert decoder.decode(event, 1_001_000_000) is None
    assert decoder.decode(dict(event, timestamp_ns=999_999_999), 1_001_000_000) is None


@pytest.mark.parametrize("key,value", [("orientation", [0, 0, 0, 0]),
                                       ("position", [float("nan"), 0, 0]),
                                       ("position", [None, 0, 0]),
                                       ("angular_velocity", [0, 0]), ("state", -1),
                                       ("timestamp_ns", True)])
def test_malformed_data_not_published(bindings, event, key, value):
    event[key] = value
    with pytest.raises(ValueError):
        HubDecoder(*load_bindings(bindings)).decode(event, 1_001_000_000)


def test_binding_uniqueness(bindings):
    bindings.write_text("schema_version: 1\nroles:\n  torso: A\n  left_wrist: A\n  right_wrist: B\n")
    with pytest.raises(ValueError, match="unique"):
        load_bindings(bindings)


def test_missing_fields_explicit():
    info = source_info()
    assert info["unavailable_fields"] == ["linear_acceleration", "raw_buttons"]
    assert "bridge counter" in info["packet_index"]
    json.dumps(info, allow_nan=False)


def test_recorded_2104_ns_reversal_is_removed_without_changing_source_interval(bindings, event):
    # These are the exact two adjacent clocks from failed round 2 (bag indices
    # 40731/40732). Receive pairs were not recorded; reconstruct their observed
    # offsets to replay the old decoder's mapping, not claim raw SDK capture.
    values = [(9357450672327, 1793024403726563313),
              (9357450677535, 1793024403726561209)]
    decoder = HubDecoder(*load_bindings(bindings))
    samples = []
    for mono, old_real in values:
        receive = mono + 1_000_000
        e = dict(event, timestamp_ns=mono, receive_monotonic_ns=receive,
                 receive_realtime_ns=old_real + receive - mono)
        samples.append(decoder.decode(e, receive + 1000))
    assert values[1][1] - values[0][1] == -2104
    assert samples[1].host_realtime_ns - samples[0].host_realtime_ns == 5208
    assert samples[1].host_monotonic_ns - samples[0].host_monotonic_ns == 5208
    assert samples[1].position == samples[0].position == tuple(event['position'])


def test_burst_jitter_uses_one_offset_for_all_roles(bindings, event):
    decoder = HubDecoder(*load_bindings(bindings))
    previous = {}
    offsets = set()
    for i in range(3000):
        serial = ('HEAD', 'RIGHT', 'LEFT')[i % 3]
        stamp = 1_000_000_000 + i * 1000
        recv = stamp + 100_000
        e = dict(event, serial=serial, timestamp_ns=stamp, receive_monotonic_ns=recv,
                 receive_realtime_ns=recv + 8_000_000_000 + (i % 7 - 3) * 7000)
        sample = decoder.decode(e, recv)
        offsets.add(sample.host_realtime_ns - stamp)
        if serial in previous:
            old = previous[serial]
            assert sample.host_realtime_ns - old.host_realtime_ns == stamp - old.host_monotonic_ns
        previous[serial] = sample
    assert len(offsets) == 1


@pytest.mark.parametrize('step', [-6_000_000, 6_000_000, 1_000_000_000])
def test_real_clock_step_is_latched_not_smoothed_or_reanchored(step):
    clock = AnchoredHostClock()
    assert clock.map(1_000_000, 2_000_000, 9_000_000_000, 100) == 8_999_000_000
    with pytest.raises(HubClockError, match='more than 5 ms'):
        clock.map(2_000_000, 3_000_000, 9_001_000_000 + step, 100)
    # Even apparent recovery requires an explicit bridge restart.
    with pytest.raises(HubClockError):
        clock.map(3_000_000, 4_000_000, 9_002_000_000, 100)


@pytest.mark.parametrize('span', [None, True, -1, 100_001, 10_000_000])
def test_uncertain_clock_pair_cannot_set_or_move_anchor(span):
    clock = AnchoredHostClock()
    with pytest.raises(ValueError, match='sampling span'):
        clock.map(1, 100, 1_000_000, span)
    assert clock.offset_ns is None
    clock.map(1, 100, 1_000_000, 100)
    with pytest.raises(ValueError):
        clock.map(2, 101, 100_000_000, span)
    assert clock.map(3, 102, 1_000_002, 100) == 999_903
