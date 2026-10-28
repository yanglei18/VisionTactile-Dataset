"""Source-time association and product modes without devices, Qt or models."""
from dataclasses import replace

import numpy as np
import pytest

from vt_capture_console.backend.tactile_source import TactileDisplayState, TactileTiming
from vt_hand_tactile_viewer.composition import DisplayComposer, MAX_HISTORY
from vt_tactile_capture.display_response import ResponseStatus


NOW = 10_000_000_000


def geometry():
    return dict(vertices_m=[[0., 0., 0.], [0.01, 0., 0.], [0., 0.01, 0.]],
                joints_m=[[0., 0., 0.]] * 21, faces=[[0, 1, 2]])


def pose(stamp=NOW, *, session="pose-session", profile=1, source=None, **changes):
    value = dict(session_id=session, profile_version=profile, config_sha256="config", model_rest_sha256="model",
                 timestamp_s=stamp / 1e9, source_monotonic_ns=stamp if source is None else source,
                 frame_index=stamp, hand="right", pose_available=True, solver_success=True,
                 quality="not_accuracy_validated", algorithm="official_fsglove_raw2mano_with_explicit_sparse_adapter_v1",
                 upstream_fsglove_solver_executed=True, hardware_accessed=True)
    value.update(changes)
    return dict(runtime=dict(connected=True, session_id=session, last_sample_age_s=0.,
                             published_monotonic_s=stamp / 1e9),
                pose=value, geometry=geometry(), calibration=dict(profile_status="ready", clear_generation=0))


def tactile(stamp=NOW, *, sequence=1, session="touch-session", snapshot=None, generation=0, value=100.):
    return TactileDisplayState(
        dict(values=[value] * 400, valid_mask=[True] * 400, levels=[0., 1000.],
             reference_ready=True, value_kind="tactile_response"), "实时", "",
        response=ResponseStatus(phase="ready", reference_cells=400, reference_source="default"),
        timing=TactileTiming(session, 1, sequence, stamp, stamp if snapshot is None else snapshot,
                             (stamp,) * 160, 100_000_000, display_generation=generation))


def test_synchronized_live_hand_and_response_are_not_modified_by_composition():
    composer = DisplayComposer(geometry())
    motion, touch = pose(), tactile()
    result = composer.compose(motion, touch, now_ns=NOW)
    assert result.mode == "live" and result.pose_current
    assert result.geometry is motion["geometry"]
    assert result.tactile_frame["values"] == [100.] * 400
    assert result.tactile_frame["valid_mask"] == [True] * 400
    assert result.pose_age_ms == result.tactile_age_ms == result.skew_ms == 0
    result.tactile_frame["values"][0] = 0
    touch.frame["values"][1] = 0
    # History is independent of source mutation and of previous output lists.
    future = tactile(NOW + 20_000_000, sequence=2, value=200.)
    paired = composer.compose(pose(NOW + 10_000_000), future, now_ns=NOW + 20_000_000)
    assert paired.tactile_frame["values"][:2] == [100., 100.]
    assert paired.skew_ms == 10


def test_causal_pair_uses_real_palm_receive_time_not_processing_or_arrival_time():
    composer = DisplayComposer(geometry())
    composer.compose(pose(), tactile(value=10.), now_ns=NOW)
    # A 60 ms processing delay must not admit a tactile sample 50 ms newer
    # than the actual palm measurement, even though the solver tick is later.
    result = composer.compose(pose(NOW + 60_000_000, source=NOW + 10_000_000),
                              tactile(NOW + 50_000_000, sequence=2, value=20.),
                              now_ns=NOW + 60_000_000)
    assert result.tactile_frame["values"] == [10.] * 400
    assert result.pose_age_ms == 50 and result.skew_ms == 10


def test_future_snapshot_is_not_retimed_even_if_its_latest_packet_is_older():
    result = DisplayComposer(geometry()).compose(
        pose(NOW + 10_000_000), tactile(snapshot=NOW + 20_000_000), now_ns=NOW + 20_000_000)
    assert result.pose_current and result.geometry is not None
    assert result.tactile_frame is None and "时间对应" in result.status


@pytest.mark.parametrize("context", ["session", "profile", "generation"])
def test_pose_or_display_context_change_drops_old_tactile_but_admits_current_input(context):
    composer = DisplayComposer(geometry())
    composer.compose(pose(), tactile(value=10.), now_ns=NOW)
    current = pose(NOW + 10_000_000, session="new" if context == "session" else "pose-session",
                   profile=2 if context == "profile" else 1)
    touch = tactile(NOW + 20_000_000, sequence=2, generation=int(context == "generation"), value=20.)
    result = composer.compose(current, touch, now_ns=NOW + 20_000_000)
    assert result.tactile_frame is None
    later = pose(NOW + 30_000_000, session=current["pose"]["session_id"], profile=current["pose"]["profile_version"])
    resumed = composer.compose(later, touch, now_ns=NOW + 30_000_000)
    assert resumed.tactile_frame["values"] == [20.] * 400


def test_tactile_device_change_and_disconnect_never_resurrect_previous_hotspots():
    composer = DisplayComposer(geometry())
    composer.compose(pose(), tactile(), now_ns=NOW)
    new = tactile(NOW + 20_000_000, session="new-board")
    assert composer.compose(pose(NOW + 10_000_000), new, now_ns=NOW + 20_000_000).tactile_frame is None
    disconnected = TactileDisplayState(None, "未连接", "")
    frame = composer.compose(pose(NOW + 30_000_000), disconnected, now_ns=NOW + 30_000_000)
    assert frame.geometry is not None and frame.pose_current and frame.tactile_frame is None
    assert composer.history_size == 0


def test_pure_tactile_uses_explicit_reference_mode_but_connected_uncalibrated_is_waiting():
    composer = DisplayComposer(geometry())
    result = composer.compose(dict(runtime=dict(connected=False)), tactile(), now_ns=NOW)
    assert result.mode == "reference" and not result.pose_current and result.tactile_frame is not None
    assert "固定参考" in result.status
    snapshot = pose()
    snapshot["pose"].update(pose_available=False, reason="calibration_required")
    snapshot["calibration"]["profile_status"] = "not_calibrated"
    waiting = composer.compose(snapshot, tactile(), now_ns=NOW)
    assert waiting.mode == "waiting" and waiting.geometry is None and waiting.tactile_frame is None


@pytest.mark.parametrize("mutation", ["session", "profile", "stale", "missing_source", "future_source", "root_old", "bad_mesh"])
def test_unavailable_pose_never_receives_fresh_tactile(mutation):
    snapshot = pose(NOW + 20_000_000)
    if mutation == "session":
        snapshot["pose"]["session_id"] = "wrong"
    elif mutation == "profile":
        snapshot["calibration"]["profile_version"] = 3
    elif mutation == "stale":
        snapshot["pose"]["timestamp_s"] = (NOW - 300_000_000) / 1e9
    elif mutation == "missing_source":
        del snapshot["pose"]["source_monotonic_ns"]
    elif mutation == "future_source":
        snapshot["pose"]["source_monotonic_ns"] = NOW + 20_000_001
    elif mutation == "root_old":
        snapshot["pose"]["source_monotonic_ns"] = NOW - 300_000_000
    else:
        snapshot["geometry"]["vertices_m"][0][0] = float("nan")
    result = DisplayComposer(geometry()).compose(snapshot, tactile(), now_ns=NOW + 20_000_000)
    assert result.tactile_frame is None
    if mutation != "missing_source":
        assert not result.pose_current and result.geometry is None


def test_held_or_unknown_fingers_lose_only_their_tactile_colors():
    snapshot = pose()
    displayed = dict(snapshot["pose"], display_only=True, held_slots=[1], unknown_slots=[13])
    snapshot.update(display_pose=displayed, display_geometry=dict(geometry(), display_only=True), geometry=None)
    result = DisplayComposer(geometry()).compose(snapshot, tactile(), now_ns=NOW)
    assert result.pose_current and result.geometry is not None
    mask = np.asarray(result.tactile_frame["valid_mask"]).reshape(20, 20)
    # The delivered right-hand layout assigns those physical ADC bands to
    # index/thumb. These expectations do not read the compositor's own masks.
    assert not mask[8:20, 12:16].any()
    assert not mask[8:20, 16:20].any()
    assert mask[8:20, 8:12].all()
    assert "部分" in result.status


def test_old_display_mesh_from_a_different_profile_or_frame_is_not_a_current_pose():
    snapshot = pose()
    snapshot.update(display_pose=dict(snapshot["pose"], profile_version=0),
                    display_geometry=geometry(), geometry=None)
    result = DisplayComposer(geometry()).compose(snapshot, tactile(), now_ns=NOW)
    assert result.mode == "waiting" and result.tactile_frame is None


def test_sample_age_skew_and_per_block_ages_have_independent_bounds():
    touch = tactile()
    # Permit 250 ms only in this synthetic transport to test the 150 ms skew limit.
    touch = replace(touch, timing=replace(touch.timing, max_age_ns=250_000_000))
    composer = DisplayComposer(geometry())
    at_limit = composer.compose(pose(NOW + 150_000_000), touch, now_ns=NOW + 150_000_000)
    assert at_limit.tactile_frame is not None
    assert composer.compose(pose(NOW + 150_000_001), touch, now_ns=NOW + 150_000_001).tactile_frame is None
    expired = composer.compose(pose(NOW + 250_000_001), touch, now_ns=NOW + 250_000_001)
    assert expired.geometry is not None and expired.tactile_frame is None
    blocks = list(touch.timing.block_monotonic_ns)
    blocks[3 * 8 + 2] = NOW - 100_000_001
    partial = replace(tactile(), timing=replace(tactile().timing, block_monotonic_ns=blocks))
    result = DisplayComposer(geometry()).compose(pose(), partial, now_ns=NOW)
    assert [i for i, valid in enumerate(result.tactile_frame["valid_mask"]) if not valid] == [62, 70, 78]


def test_polling_does_not_grow_history_or_renew_source_and_storage_is_bounded():
    composer = DisplayComposer(geometry())
    for sequence in range(1, MAX_HISTORY + 20):
        stamp = NOW + sequence * 1_000_000
        composer.compose(pose(stamp), tactile(stamp, sequence=sequence), now_ns=stamp)
    assert composer.history_size == MAX_HISTORY
    for _ in range(100):
        result = composer.compose(pose(stamp), tactile(stamp, sequence=sequence), now_ns=stamp + 1)
    assert composer.history_size == MAX_HISTORY
    assert result.tactile_age_ms == 1e-6
    expired = composer.compose(pose(stamp + 100_000_001), tactile(stamp, sequence=sequence), now_ns=stamp + 100_000_001)
    assert expired.tactile_frame is None


def test_clock_reversal_and_reference_collection_clear_cached_colors():
    composer = DisplayComposer(geometry())
    composer.compose(pose(), tactile(), now_ns=NOW)
    result = composer.compose(pose(), tactile(), now_ns=NOW - 1)
    assert result.mode == "waiting" and result.tactile_frame is None
    assert composer.history_size == 0
    collecting = replace(tactile(), response=ResponseStatus(phase="collecting"))
    result = composer.compose(pose(), collecting, now_ns=NOW)
    assert result.geometry is not None and result.tactile_frame is None


def test_synthetic_or_shape_preview_does_not_mix_with_live_tactile():
    snapshot = pose(hardware_accessed=False)
    assert DisplayComposer(geometry()).compose(snapshot, tactile(), now_ns=NOW).tactile_frame is None
    snapshot = pose()
    snapshot.update(display_pose=dict(snapshot["pose"], shape_preview_mode="candidate"),
                    display_geometry=geometry(), geometry=None)
    assert DisplayComposer(geometry()).compose(snapshot, tactile(), now_ns=NOW).geometry is None
