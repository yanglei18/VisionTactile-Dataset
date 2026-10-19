"""Pure display tests: in-memory data, no ROS, GUI, serial, or hardware."""

from collections import deque
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

# The optional ROS package's pure display modules work without a ROS install.
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "ros2_ws/src/vt_tactile_capture"))

from vt_fsglove12.tactile_overlay import MAX_HISTORY, TactileOverlayController


NS = 1_000_000_000


def state(stamp=10 * NS, sequence=1, adc=100, **changes):
    values = dict(schema_version=2, rows=20, cols=20, hand="right", unit="adc_counts",
        session_id="one", stream_epoch=0, source_sequence=sequence,
        host_monotonic_ns=stamp, snapshot_monotonic_ns=stamp,
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        connected=True, time_valid=True, max_age_ns=250_000_000,
        adc=np.full(400, adc, dtype=np.uint16), valid_mask=np.ones(400, dtype=bool),
        block_monotonic_ns=np.full(160, stamp, dtype=np.uint64))
    values.update(changes)
    return SimpleNamespace(**values)


class Receiver:
    def __init__(self, *messages):
        self.messages = deque(messages)
        self.reset_generation = self.health_generation = 0
        self.closed = False
        self.calls = 0

    def receive_latest(self):
        self.calls += 1
        return self.messages.popleft() if self.messages else None

    def close(self):
        self.closed = True


def controller(*messages):
    receiver = Receiver(*messages)
    return TactileOverlayController(receiver=receiver), receiver


def install_reference(display, value=100):
    # Most timing tests use an already collected reference; collection itself
    # is exercised independently below using the real DisplayBaseline.
    display.baseline.values = np.full((20, 20), value, dtype=float)
    display.baseline.identity = ("one", 0)


def show(display, stamp=10 * NS, **kwargs):
    return display.frame(stamp / NS, now_ns=stamp, **kwargs)


def test_signed_adc_zero_valid_and_no_source_mutation():
    message = state(adc=0)
    display, _ = controller(message)
    display.poll(10 * NS)
    install_reference(display)
    frame = show(display)
    assert frame.delta.shape == frame.valid_mask.shape == (20, 20)
    assert frame.valid_mask.all()
    assert (frame.delta == -100).all()
    assert not frame.delta.flags.writeable and not frame.valid_mask.flags.writeable
    assert (message.adc == 0).all() and message.valid_mask.all()
    assert "非同时扫描" in frame.status and "非压力" in frame.status
    assert frame.sequence == 1 and frame.age_ms == frame.skew_ms == 0
    message.adc[:] = 900
    message.valid_mask[:] = False
    assert (show(display).delta == -100).all()  # independent history storage


def test_left_overlay_requires_matching_layout_and_keeps_reference_isolated(tmp_path):
    import vt_tactile_capture.hand_layout as layout_module
    path = Path(layout_module.__file__).parent / "layouts/left_glove_unverified_candidate.json"
    with pytest.raises(ValueError, match="explicit left layout"):
        TactileOverlayController(receiver=Receiver(), expected_hand="left")
    left_receiver, right_receiver = Receiver(state(hand="left", adc=101)), Receiver(state(adc=202))
    left = TactileOverlayController(receiver=left_receiver, expected_hand="left", layout_path=path)
    right = TactileOverlayController(receiver=right_receiver)
    for display in (left, right):
        display.poll(10 * NS)
        install_reference(display)
    assert np.all(show(left).delta == 1) and np.all(show(right).delta == 102)
    assert left._latest.hand == "left"
    assert show(left, metadata={"hand": "right"}).delta is None
    left_receiver.messages.append(state(sequence=2, hand="right"))
    left.poll(10 * NS)
    assert left.baseline.values is None and not left._history
    assert np.all(show(right).delta == 102)
    left.close()
    assert left_receiver.closed and not right_receiver.closed


def test_left_held_finger_mask_uses_explicit_anatomical_layout_not_right_columns(tmp_path):
    import vt_tactile_capture.hand_layout as layout_module
    path = Path(layout_module.__file__).parent / "layouts/left_glove_unverified_candidate.json"
    config = json.loads(path.read_text())
    regions = {region["region"]: region for region in config["regions"]}
    regions["index"]["cols"], regions["little"]["cols"] = regions["little"]["cols"], regions["index"]["cols"]
    path = tmp_path / "independent_left.json"
    path.write_text(json.dumps(config))
    display = TactileOverlayController(receiver=Receiver(state(hand="left", adc=123)), expected_hand="left", layout_path=path)
    display.poll(10 * NS)
    install_reference(display)
    frame = show(display, metadata={"held_slots": [1], "hand": "left"})
    assert not frame.valid_mask[8:20, 0:4].any()
    assert frame.valid_mask[8:20, 12:16].all()
    assert frame.valid_mask.sum() == 352
    assert np.all(display._latest.adc == 123)


def test_legacy_state_crops_physical_addresses_without_retiming_or_rescaling():
    message = state(schema_version=1, rows=80, cols=80,
        valid_mask=np.ones(6400, dtype=bool),
        block_monotonic_ns=np.full(640, 10 * NS, dtype=np.uint64))
    message.adc = np.arange(6400, dtype=np.uint16)
    # Values/clocks outside the physical region must never affect the overlay.
    message.valid_mask[19 * 80 + 19] = False
    message.block_monotonic_ns[20 * 8:] = 11 * NS
    display, _ = controller(message)
    display.poll(10 * NS)
    install_reference(display, value=0)
    frame = show(display)
    assert frame.delta.shape == (20, 20) and frame.valid_mask.sum() == 399
    assert frame.delta[1, 0] == 80 and frame.delta[19, 18] == 1538
    assert np.isnan(frame.delta[19, 19])
    assert display._latest.block_monotonic_ns.shape == (160,)
    assert display._latest.host_monotonic_ns == display._latest.snapshot_monotonic_ns == 10 * NS
    assert message.adc.shape == (6400,) and message.rows == 80


def test_latest_causal_source_chosen_not_future_or_latest_arrival():
    display, receiver = controller(state(sequence=1, adc=101))
    display.poll(10 * NS)
    receiver.messages.append(state(10 * NS + 100_000_000, sequence=2, adc=102))
    display.poll(10 * NS + 100_000_000)
    install_reference(display)
    frame = display.frame(10.05, now_ns=10 * NS + 100_000_000)
    assert frame.sequence == 1 and np.all(frame.delta == 1)
    assert frame.skew_ms == 50 and frame.age_ms == 100
    assert show(display, 10 * NS + 100_000_000).sequence == 2
    assert display.frame(9.99, now_ns=10 * NS + 100_000_000).delta is None


def test_snapshot_is_causal_too():
    display, _ = controller(state(snapshot_monotonic_ns=10 * NS + 20_000_000))
    display.poll(10 * NS + 20_000_000)
    install_reference(display)
    assert display.frame(10.01, now_ns=10 * NS + 20_000_000).delta is None
    assert show(display, 10 * NS + 20_000_000).delta is not None


@pytest.mark.parametrize("changes,phrase", [
    ({"connected": False}, "断开"),
    ({"time_valid": False}, "时间无效"),
    ({"clock_domain": "CLOCK_REALTIME"}, "时钟不匹配"),
    ({"clock_domain": "ros_time"}, "时钟不匹配"),
    ({"timing_source": "device"}, "时钟不匹配"),
    ({"host_monotonic_ns": 11 * NS, "snapshot_monotonic_ns": 11 * NS}, "未来"),
    ({"snapshot_monotonic_ns": 11 * NS}, "未来"),
    ({"snapshot_monotonic_ns": 9 * NS}, "早于"),
    ({"host_monotonic_ns": 9 * NS}, "过期"),
])
def test_unhealthy_clock_future_and_old_source_cannot_start_reference(changes, phrase):
    display, _ = controller(state(**changes))
    display.poll(10 * NS)
    assert not display.start_reference(10 * NS)
    frame = show(display)
    assert frame.delta is None and phrase in frame.status
    assert display.baseline.values is None


def test_fresh_snapshot_and_arrival_never_rescue_old_source():
    display, _ = controller(state(9 * NS, snapshot_monotonic_ns=10 * NS))
    display.poll(10 * NS)
    install_reference(display)
    assert show(display).delta is None


def test_source_age_and_pose_skew_have_independent_limits():
    display, _ = controller(state())
    display.poll(10 * NS)
    install_reference(display)
    boundary = display.frame(10.15, now_ns=10 * NS + 250_000_000)
    assert boundary.delta is not None
    assert boundary.age_ms == 250 and boundary.skew_ms == 150
    assert display.frame(10.151, now_ns=10 * NS + 151_000_000).delta is None
    assert display.frame(10.15, now_ns=10 * NS + 250_000_001).delta is None


@pytest.mark.parametrize("pose,kwargs", [
    (10., {"pose_current": False}), (None, {}), (True, {}),
    (float("nan"), {}), (float("inf"), {}), (11., {}), (9., {}),
    (10., {"metadata": {"demo": True}}),
    (10., {"metadata": {"is_demo": True}}),
    (10., {"metadata": {"calibrated": False}}),
    (10., {"metadata": {"realtime": False}}),
])
def test_pose_gating_clears_only_overlay_and_preserves_tactile_reference(pose, kwargs):
    display, _ = controller(state())
    display.poll(10 * NS)
    install_reference(display)
    assert display.frame(pose, now_ns=10 * NS, **kwargs).delta is None
    assert display.baseline.values is not None
    assert show(display).delta is not None


def test_per_block_stride_eight_old_future_zero_and_original_invalid():
    message = state()
    message.block_monotonic_ns[2 * 8 + 3] = 9 * NS  # row2 cols3,11,19
    message.block_monotonic_ns[3 * 8 + 4] = 11 * NS
    message.block_monotonic_ns[4 * 8 + 5] = 0
    message.valid_mask[5 * 20 + 6] = False
    display, _ = controller(message)
    display.poll(10 * NS)
    install_reference(display)
    frame = show(display)
    expected = np.ones((20, 20), dtype=bool)
    expected[2, 3::8] = expected[3, 4::8] = expected[4, 5::8] = False
    expected[5, 6] = False
    np.testing.assert_array_equal(frame.valid_mask, expected)
    assert np.isnan(frame.delta[~expected]).all()
    assert message.valid_mask.sum() == 399


def test_producer_shorter_max_age_is_honored():
    message = state(max_age_ns=20_000_000)
    message.block_monotonic_ns[0] -= np.uint64(21_000_000)
    display, _ = controller(message)
    display.poll(10 * NS)
    install_reference(display)
    assert not show(display).valid_mask[0, ::8].any()
    assert display.frame(10.021, now_ns=10 * NS + 21_000_000).delta is None


@pytest.mark.parametrize("slot,column", [(1, 12), (3, 12), (4, 8), (6, 8),
    (7, 0), (9, 0), (10, 4), (12, 4), (13, 16), (15, 16)])
@pytest.mark.parametrize("field", ["held_slots", "unknown_slots"])
def test_any_held_or_unknown_slot_masks_entire_finger_not_neighbors(slot, column, field):
    display, _ = controller(state())
    display.poll(10 * NS)
    install_reference(display)
    metadata = {field: [slot]}
    frame = show(display, metadata=metadata)
    expected = np.ones((20, 20), dtype=bool)
    expected[8:20, column:column + 4] = False
    np.testing.assert_array_equal(frame.valid_mask, expected)
    assert metadata == {field: [slot]}
    assert "已屏蔽" in frame.status


@pytest.mark.parametrize("field", ["held_slots", "unknown_slots"])
def test_frozen_root_clears_all_tactile_but_not_reference(field):
    display, _ = controller(state())
    display.poll(10 * NS)
    install_reference(display)
    assert show(display, metadata={field: [0]}).delta is None
    assert display.baseline.values is not None


def test_reference_is_manual_three_seconds_and_distinct_source_states():
    display, receiver = controller(state())
    assert display.start_reference(10 * NS)
    assert display.baseline.collecting and display.baseline.samples_count == 1
    for i in range(1, 31):
        stamp = 10 * NS + i * 100_000_000
        receiver.messages.append(state(stamp, sequence=i + 1, adc=100 + i))
        display.poll(stamp)
        if i < 30:
            assert display.baseline.collecting
            assert show(display, stamp).delta is None
    assert not display.baseline.collecting
    assert display.baseline.samples_count == 31
    np.testing.assert_array_equal(display.baseline.values, 115)
    np.testing.assert_array_equal(show(display, 13 * NS).delta, 15)


def test_duplicate_states_and_silence_cannot_complete_reference():
    display, receiver = controller(state())
    assert display.start_reference(10 * NS)
    for i in range(1, 10):
        receiver.messages.append(state(10 * NS + i * 100_000_000, sequence=1))
        display.poll(10 * NS + i * 100_000_000)
    assert display.baseline.samples_count == 1
    display.poll(13 * NS)
    assert not display.baseline.collecting and display.baseline.values is None
    assert "cancelled" in display.baseline.status


def test_ten_samples_but_silent_deadline_or_late_arrival_still_fails():
    for late in (False, True):
        display, receiver = controller(state())
        assert display.start_reference(10 * NS)
        for i in range(1, 11):
            stamp = 10 * NS + i * 250_000_000
            receiver.messages.append(state(stamp, sequence=i + 1))
            display.poll(stamp)
        assert display.baseline.samples_count == 11
        if late:
            receiver.messages.append(state(13 * NS + 1, sequence=12))
        display.poll(13 * NS + int(late))
        assert not display.baseline.collecting and display.baseline.values is None


def test_reference_progress_ready_and_failure_remain_visible_without_imu_pose():
    display, receiver = controller(state())
    assert display.start_reference(10 * NS)
    frame = display.frame(None, pose_current=False, now_ns=10 * NS)
    assert frame.delta is None and "正在采集" in frame.status
    display.poll(13 * NS)
    failed = display.frame(None, pose_current=False, now_ns=13 * NS)
    assert failed.delta is None and "已取消" in failed.status
    install_reference(display)
    ready = display.frame(None, pose_current=False, now_ns=13 * NS)
    assert ready.delta is None and "参考就绪" in ready.status


def test_all_invalid_source_cannot_start_and_missing_reference_cells_stay_masked():
    message = state()
    message.valid_mask[:] = False
    display, receiver = controller(message)
    assert not display.start_reference(10 * NS)
    receiver.messages.append(state(10 * NS + 1, sequence=2))
    display.poll(10 * NS + 1)
    install_reference(display)
    display.baseline.values[3, 4] = np.nan
    frame = show(display, 10 * NS + 1)
    assert not frame.valid_mask[3, 4] and np.isnan(frame.delta[3, 4])
    assert frame.valid_mask.sum() == 399


def test_reference_rejects_stale_blocks_and_can_finish_without_imu():
    display, receiver = controller(state())
    assert display.start_reference(10 * NS)
    for i in range(1, 31):
        stamp = 10 * NS + i * 100_000_000
        message = state(stamp, sequence=i + 1, adc=200)
        message.block_monotonic_ns[0] = 9 * NS
        receiver.messages.append(message)
        display.poll(stamp)
    assert display.baseline.values is not None
    # Block0's only valid observation was the initial value 100, not stale 200s.
    np.testing.assert_array_equal(display.baseline.values[0, ::8], 100)
    assert display.baseline.values[1, 0] == 200


@pytest.mark.parametrize("changes", [{"session_id": "two"}, {"stream_epoch": 1},
    {"connected": False}, {"time_valid": False}, {"clock_domain": "CLOCK_REALTIME"}])
@pytest.mark.parametrize("collecting", [False, True])
def test_reference_and_history_clear_at_identity_or_health_boundary(changes, collecting):
    display, receiver = controller(state())
    display.poll(10 * NS)
    if collecting:
        assert display.start_reference(10 * NS)
    else:
        install_reference(display)
    receiver.messages.append(state(10 * NS + 10_000_000, sequence=2, **changes))
    display.poll(10 * NS + 10_000_000)
    assert display.baseline.values is None and not display.baseline.collecting
    assert len(display._history) <= 1
    assert show(display, 10 * NS + 10_000_000).delta is None


@pytest.mark.parametrize("generation", ["reset_generation", "health_generation"])
def test_recovery_hidden_in_receiver_drain_still_cancels_reference(generation):
    display, receiver = controller(state())
    display.poll(10 * NS)
    install_reference(display)
    setattr(receiver, generation, 1)
    receiver.messages.append(state(10 * NS + 10_000_000, sequence=2))
    display.poll(10 * NS + 10_000_000)
    assert display.baseline.values is None
    assert len(display._history) == 1
    assert display.start_reference(10 * NS + 10_000_000)


def test_generation_change_with_no_message_also_invalidates():
    display, receiver = controller(state())
    display.poll(10 * NS)
    install_reference(display)
    receiver.health_generation += 1
    display.poll(10 * NS)
    assert display.baseline.values is None and not display._history


def test_history_and_receiver_work_are_bounded_and_close_is_idempotent():
    display, receiver = controller()
    for i in range(200):
        stamp = 10 * NS + i * 1_000_000
        receiver.messages.append(state(stamp, sequence=i))
        display.poll(stamp)
    assert len(display._history) == MAX_HISTORY and receiver.calls == 200
    display.close()
    display.close()
    assert receiver.closed and not display._history
    assert show(display).delta is None
    assert not display.start_reference(11 * NS)


def test_absent_stream_and_status_do_not_accumulate_text():
    display, _ = controller()
    display.poll(10 * NS)
    first = show(display).status
    for _ in range(10):
        assert show(display).status == first
    assert not display.start_reference(10 * NS)


@pytest.mark.parametrize("changes", [{"adc": [0] * 10}, {"valid_mask": [2] * 400},
    {"block_monotonic_ns": [0] * 10}, {"connected": 1}, {"rows": 80},
    {"source_sequence": -1}, {"hand": "left"}])
def test_malformed_injected_state_does_not_crash_or_retain_reference(changes):
    display, receiver = controller(state())
    display.poll(10 * NS)
    install_reference(display)
    message = state(sequence=2)
    for name, value in changes.items():
        setattr(message, name, value)
    receiver.messages.append(message)
    display.poll(10 * NS)
    assert display.baseline.values is None
    assert "数据不可用" in show(display).status


def test_local_clock_reversal_cancels_reference():
    display, _ = controller(state())
    assert display.start_reference(10 * NS)
    display.poll(9 * NS)
    assert not display.baseline.collecting and display.baseline.values is None
