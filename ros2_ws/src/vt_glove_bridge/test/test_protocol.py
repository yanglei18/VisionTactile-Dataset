import json
from types import SimpleNamespace

import pytest

from vt_glove_bridge.protocol import decode_state, populate_message, SequenceTracker
from vt_glove_bridge.transport import MAX_PACKET_BYTES


def packet(**overrides):
    state = dict(
        schema_version=1, kind="glove_state", hand="right", frame_id="glove_right_reference",
        source_mode="live", session_id="session-a", frame_index=1, calibration_version=2,
        config_sha256="abc", model_rest_sha256="def", algorithm="official_fsglove_sparse_adapter",
        quality="experimental", reason="", pose_valid=True, time_valid=True,
        host_realtime_ns=1_791_000_000_123_456_789, host_monotonic_ns=10_000_000_000,
        reference_node_id=12, solve_completed_monotonic_ns=10_010_000_000,
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive", source_node_ids=[5, 12],
        source_realtime_ns=[1_791_000_000_120_456_789, 1_791_000_000_123_456_789],
        source_monotonic_ns=[9_997_000_000, 10_000_000_000], source_sample_indices=[4, 8],
        source_filter_epochs=[0, 0], mano_pose_rotvec=[0.] * 48, root_quat_xyzw=[0., 0., 0., 1.],
        shape_params=[0.] * 10, joint_positions_m=[0.] * 63,
        segment_observed_mask=[True] * 16, joint_observed_mask=[False] * 16,
        inferred_mask=[True] * 16, source_labels=["coupled"] * 16,
        units_verified=False, sample_rate_verified=False, anatomical_accuracy_verified=False,
        yaw_drift_corrected=False, producer_dropped_total=3, quality_json="{}", metadata_json="{}",
    )
    state.update(overrides)
    return json.dumps(state).encode()


def message():
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(), frame_id=""))


def joint_quality(**overrides):
    quality = dict(joint_valid_mask=[True] * 16,
                   joint_display_available_mask=[True] * 16,
                   joint_quality=["valid"] * 16,
                   joint_estimation={"8": {"method": "hinge", "residual_deg": 2.5}})
    quality.update(overrides)
    return quality


def test_preserves_source_stamps_and_pose_exactly():
    state = decode_state(packet())
    result = populate_message(message(), state, realtime_ns=state["host_realtime_ns"] + 99,
                              monotonic_ns=10_020_000_000, gaps_total=4, rejected_total=2)
    assert result.header.stamp.sec == 1_791_000_000
    assert result.header.stamp.nanosec == 123_456_789
    assert result.host_realtime_ns != result.bridge_received_realtime_ns
    assert result.mano_pose_rotvec == state["mano_pose_rotvec"]
    assert result.pose_valid and result.bridge_sequence_gaps_total == 4
    assert result.bridge_rejected_total == 2 and result.producer_dropped_total == 3


def test_left_hand_and_non12_palm_preserve_independent_identity():
    data = packet(hand="left", frame_id="glove_left_reference", reference_node_id=11,
                  source_node_ids=[5, 11])
    state = decode_state(data, expected_hand="left", reference_node_id=11)
    assert state["hand"] == "left" and state["reference_node_id"] == 11
    with pytest.raises(ValueError, match="bridge instance"):
        decode_state(data, expected_hand="right")
    with pytest.raises(ValueError, match="configured mapping"):
        decode_state(data, expected_hand="left", reference_node_id=12)
    with pytest.raises(ValueError, match="coordinate frame"):
        decode_state(packet(hand="left"))


def test_same_session_sequence_is_independent_across_hands():
    sequence = SequenceTracker()
    sequence.accept(decode_state(packet()))
    sequence.accept(decode_state(packet(hand="left", frame_id="glove_left_reference")))
    assert len(sequence.sessions) == 2 and sequence.gaps_total == 0


@pytest.mark.parametrize("change", [
    {"schema_version": 2}, {"hand": "left"}, {"frame_id": "world"},
    {"host_realtime_ns": 1.2}, {"host_realtime_ns": True}, {"host_realtime_ns": -1},
    {"pose_valid": 1}, {"clock_domain": "CLOCK_MONOTONIC_RAW"}, {"reference_node_id": 11},
    {"source_node_ids": [12, 12]}, {"source_filter_epochs": []},
    {"root_quat_xyzw": [0., 0., 0., 0.]}, {"mano_pose_rotvec": []},
    {"mano_pose_rotvec": [float("nan")] * 48}, {"time_valid": False},
    {"source_realtime_ns": [1, 2]}, {"source_labels": []},
    {"inferred_mask": [1] * 16}, {"quality_json": "[]"},
    {"anatomical_accuracy_verified": True}, {"yaw_drift_corrected": True},
])
def test_rejects_invalid_or_misleading_packets(change):
    with pytest.raises(ValueError):
        decode_state(packet(**change))


@pytest.mark.parametrize("payload", [b"", b"\xff", b"[]", b"{", b"{}", b"x" * 60001,
                                      b'{"kind":"glove_state","kind":"other"}'])
def test_bad_json(payload):
    with pytest.raises(ValueError):
        decode_state(payload)


def test_unavailable_state_can_have_zero_time_and_empty_pose():
    state = decode_state(packet(pose_valid=False, time_valid=False, host_realtime_ns=0,
                                host_monotonic_ns=0, mano_pose_rotvec=[], root_quat_xyzw=[],
                                shape_params=[], joint_positions_m=[], source_node_ids=[],
                                source_realtime_ns=[], source_monotonic_ns=[],
                                source_sample_indices=[], source_filter_epochs=[]))
    result = populate_message(message(), state, realtime_ns=123, monotonic_ns=456)
    assert result.header.stamp.sec == result.header.stamp.nanosec == 0
    assert not result.pose_valid


def test_stale_input_invalidated_but_not_restamped():
    state = decode_state(packet())
    result = populate_message(message(), state, realtime_ns=123, monotonic_ns=11_000_000_000)
    assert not result.pose_valid and result.time_valid
    assert result.reason == "bridge_input_stale"
    assert result.host_realtime_ns == state["host_realtime_ns"]


def test_different_boot_monotonic_time_is_not_fresh_pose():
    result = populate_message(message(), decode_state(packet()), realtime_ns=123, monotonic_ns=1)
    assert not result.pose_valid and not result.time_valid
    assert result.reason == "bridge_input_clock_mismatch"


def test_replay_requires_explicit_opt_in():
    with pytest.raises(ValueError, match="replay input disabled"):
        decode_state(packet(source_mode="replay"))
    assert decode_state(packet(source_mode="replay"), allow_replay=True)["source_mode"] == "replay"


def test_sequence_gap_duplicate_and_session_restart():
    sequence = SequenceTracker()
    sequence.accept(decode_state(packet(frame_index=10)))
    sequence.accept(decode_state(packet(frame_index=13)))
    assert sequence.gaps_total == 2
    for index in (13, 12):
        with pytest.raises(ValueError):
            sequence.accept(decode_state(packet(frame_index=index)))
    sequence.accept(decode_state(packet(session_id="new", frame_index=0)))
    assert sequence.gaps_total == 2


def test_session_tracking_is_bounded():
    sequence = SequenceTracker()
    for index in range(100):
        sequence.accept(decode_state(packet(session_id=str(index))))
    assert len(sequence.sessions) == 64


def test_all_valid_joint_quality_preserves_schema_v1_and_raw_extensions():
    quality_json = json.dumps(joint_quality(extra={"note": "keep this extension"}), indent=2)
    metadata_json = '{ "custom": {"run": "joint-quality"} }'
    state = decode_state(packet(quality_json=quality_json, metadata_json=metadata_json))
    result = populate_message(message(), state, realtime_ns=123, monotonic_ns=10_020_000_000)
    assert result.schema_version == 1 and result.pose_valid
    assert result.quality_json == quality_json
    assert result.metadata_json == metadata_json


@pytest.mark.parametrize("status,display", [("degraded", True), ("unavailable", False)])
def test_partial_pose_retains_provisional_arrays_and_joint_quality(status, display):
    quality = joint_quality()
    quality["joint_valid_mask"][8] = False
    quality["joint_display_available_mask"][8] = display
    quality["joint_quality"][8] = status
    quality_json = json.dumps(quality)
    rotations = [index / 100. for index in range(48)]
    positions = [index / 1000. for index in range(63)]
    state = decode_state(packet(pose_valid=False, quality_json=quality_json,
                                mano_pose_rotvec=rotations, joint_positions_m=positions))
    result = populate_message(message(), state, realtime_ns=123, monotonic_ns=10_020_000_000)
    assert not result.pose_valid
    assert result.mano_pose_rotvec == rotations
    assert result.joint_positions_m == positions
    assert result.root_quat_xyzw == [0., 0., 0., 1.]
    assert result.shape_params == [0.] * 10
    assert result.quality_json == quality_json


@pytest.mark.parametrize("status,display", [("degraded", True), ("unavailable", False)])
def test_nonvalid_joint_quality_cannot_claim_valid_pose(status, display):
    quality = joint_quality()
    quality["joint_valid_mask"][8] = False
    quality["joint_display_available_mask"][8] = display
    quality["joint_quality"][8] = status
    with pytest.raises(ValueError, match="valid pose requires all joint_quality"):
        decode_state(packet(quality_json=json.dumps(quality)))


@pytest.mark.parametrize("keys", [
    ("joint_valid_mask",), ("joint_display_available_mask",), ("joint_quality",),
    ("joint_valid_mask", "joint_display_available_mask"),
    ("joint_valid_mask", "joint_quality"),
    ("joint_display_available_mask", "joint_quality"),
])
def test_joint_quality_extension_requires_complete_triplet(keys):
    complete = joint_quality()
    quality = {key: complete[key] for key in keys}
    with pytest.raises(ValueError, match="requires both masks and joint_quality"):
        decode_state(packet(pose_valid=False, quality_json=json.dumps(quality)))


@pytest.mark.parametrize("key", ["joint_valid_mask", "joint_display_available_mask"])
@pytest.mark.parametrize("values", [None, {}, "true", [], [True] * 15, [True] * 17,
                                     [1] * 16, [0] * 16, [None] * 16, ["true"] * 16])
def test_joint_quality_masks_require_exact_boolean_arrays(key, values):
    quality = joint_quality(**{key: values})
    with pytest.raises(ValueError, match=f"{key} must contain 16 bool values"):
        decode_state(packet(pose_valid=False, quality_json=json.dumps(quality)))


@pytest.mark.parametrize("values", [None, {}, "valid", [], ["valid"] * 15,
                                     ["valid"] * 17, ["invalid"] * 16,
                                     [True] * 16, [None] * 16, [[]] * 16, [{}] * 16])
def test_joint_quality_requires_exact_enum_array(values):
    with pytest.raises(ValueError, match="joint_quality must contain 16"):
        decode_state(packet(pose_valid=False,
                            quality_json=json.dumps(joint_quality(joint_quality=values))))


@pytest.mark.parametrize("status,valid,display", [
    ("valid", False, False), ("valid", False, True), ("valid", True, False),
    ("degraded", True, True), ("degraded", True, False), ("degraded", False, False),
    ("unavailable", True, True), ("unavailable", True, False), ("unavailable", False, True),
])
def test_joint_quality_masks_must_match_each_slots_quality(status, valid, display):
    quality = joint_quality()
    quality["joint_valid_mask"][8] = valid
    quality["joint_display_available_mask"][8] = display
    quality["joint_quality"][8] = status
    with pytest.raises(ValueError, match="joint quality masks disagree at slot 8"):
        decode_state(packet(pose_valid=False, quality_json=json.dumps(quality)))


def test_joint_estimation_alone_is_optional_object_extension():
    quality_json = '{"joint_estimation":{"8":{"reason":"collecting"}}}'
    assert decode_state(packet(quality_json=quality_json))["quality_json"] == quality_json


@pytest.mark.parametrize("estimation", [None, [], "pending", 1, True])
def test_joint_estimation_requires_object(estimation):
    with pytest.raises(ValueError, match="joint_estimation must be an object"):
        decode_state(packet(quality_json=json.dumps({"joint_estimation": estimation})))


def test_all_unavailable_joints_allow_empty_pose():
    quality = joint_quality(joint_valid_mask=[False] * 16,
                            joint_display_available_mask=[False] * 16,
                            joint_quality=["unavailable"] * 16)
    state = decode_state(packet(pose_valid=False, quality_json=json.dumps(quality),
                                mano_pose_rotvec=[], root_quat_xyzw=[], shape_params=[],
                                joint_positions_m=[]))
    assert not state["pose_valid"] and state["mano_pose_rotvec"] == []


def test_provisional_pose_still_rejects_nonfinite_arrays():
    with pytest.raises(ValueError):
        decode_state(packet(pose_valid=False, mano_pose_rotvec=[float("inf")] * 48))


@pytest.mark.parametrize("field", ["quality_json", "metadata_json"])
def test_optional_extensions_remain_subject_to_packet_size_limit(field):
    extension = json.dumps({"joint_estimation": {"note": "x" * MAX_PACKET_BYTES}})
    with pytest.raises(ValueError, match="oversized glove packet"):
        decode_state(packet(**{field: extension}))


def test_joint_quality_extension_rejects_duplicate_nested_keys():
    quality_json = '{"joint_estimation":{},"joint_estimation":{}}'
    with pytest.raises(ValueError, match="duplicate JSON key"):
        decode_state(packet(quality_json=quality_json))
