"""Source-time association and product modes without devices, Qt or models."""
from copy import deepcopy
from dataclasses import replace
from importlib.resources import files

import pytest

from vt_capture_console.backend.tactile_source import TactileDisplayState, TactileTiming
from vt_hand_tactile_viewer.composition import DisplayComposer, MAX_HISTORY, MAX_SHAPE_AGE_NS
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
                             (stamp,) * 20, 100_000_000, display_generation=generation))


def missing_palm(snapshot):
    # The aging runtime retains source metadata and can omit the hand/solver
    # metadata. The last mesh carried in display_geometry is not current.
    gap = deepcopy(snapshot)
    pose = gap["pose"]
    pose.pop("hand")
    pose.update(pose_available=False, solver_success=False, quality="unavailable",
                reason="missing_or_stale_root", source_monotonic_ns=None)
    gap["display_geometry"], gap["geometry"] = gap["geometry"], None
    return gap


def asynchronous_preview_snapshot():
    latest = pose(NOW + 50_000_000, shape_params=[0.] * 10)
    old = pose(NOW, shape_params=[0.] * 10)['pose']
    latest['geometry'] = {'joints_m': [[9., 0., 0.]] * 21}  # Current measurement, no surface.
    display = {**old, 'display_only': True, 'shape_preview_mode': 'current'}
    latest['surface_preview'] = dict(
        binding=('pose-session', 0, 1, 1, None, True, 'config', 'model', 'right', 0, 'current'),
        pose=display, measurement_pose=old, orientation={'nodes': {}}, geometry=geometry())
    return latest


def test_async_surface_uses_its_own_pose_time_for_age_and_tactile_skew():
    composer = DisplayComposer(geometry())
    motion = asynchronous_preview_snapshot()
    before = deepcopy(motion)
    rendered = composer.compose(motion, tactile(NOW + 50_000_000), now_ns=NOW + 50_000_000)
    assert rendered.pose_current and rendered.geometry is motion['surface_preview']['geometry']
    assert rendered.pose_age_ms == rendered.skew_ms == 50.
    assert motion == before  # Latest numeric measurement remains untouched.


@pytest.mark.parametrize('change', ['profile', 'clear', 'functional', 'shape', 'hand', 'epoch', 'unavailable'])
def test_async_surface_does_not_cross_current_measurement_context(change):
    composer = DisplayComposer(geometry())
    motion = asynchronous_preview_snapshot()
    if change == 'profile':
        motion['pose']['profile_version'] = 2
    elif change == 'clear':
        motion['calibration']['clear_generation'] = 2
    elif change == 'functional':
        motion['calibration']['functional_enabled'] = False
    elif change == 'shape':
        motion['pose']['shape_params'] = [1.] * 10
    elif change == 'hand':
        motion['surface_preview']['measurement_pose']['hand'] = 'left'
    elif change == 'epoch':
        motion['surface_preview']['orientation']['nodes'] = {'11': {'epoch': 1}}
        motion['orientation'] = {'nodes': {'11': {'epoch': 2}}}
    else:
        motion['pose']['pose_available'] = False
    rendered = composer.compose(motion, None, now_ns=NOW + 50_000_000)
    assert not rendered.uses_tracked_geometry


@pytest.fixture(params=[("left", "left_glove_from_right_initial.json"),
                        ("right", "right_glove_candidate.json")], ids=["left", "right"])
def native_composer(request):
    hand, name = request.param
    neutral = dict(geometry(), provenance={"hand": hand})
    layout = files("vt_tactile_capture") / "layouts" / name
    return hand, neutral, DisplayComposer(neutral, hand=hand, layout_path=layout)


def test_hand_bridges_only_a_bounded_source_time_gap_with_independent_fresh_touch(native_composer):
    hand, neutral, composer = native_composer
    # The actual palm sample was already 100ms old at first presentation.
    start = NOW + 100_000_000
    motion = dict(pose(start, source=NOW, hand=hand), geometry=deepcopy(neutral))
    first = composer.compose(motion, None, now_ns=start)
    assert first.uses_tracked_geometry and first.pose_current and first.geometry_key is not None

    gap = missing_palm(motion)
    before = deepcopy(gap)
    stamp = NOW + 200_000_000
    touch = tactile(stamp, value=350.)
    touch.frame["hand"] = hand
    rows = (stamp - 100_000_001,) + (stamp,) * 19
    touch = replace(touch, timing=replace(touch.timing, row_monotonic_ns=rows))
    held = composer.compose(gap, touch, now_ns=stamp)
    assert held.mode == "held" and not held.pose_current and held.uses_tracked_geometry
    assert held.geometry is first.geometry and held.geometry_key == first.geometry_key
    assert held.pose_age_ms == 200 and held.tactile_age_ms == 0 and held.skew_ms is None
    assert held.tactile_frame["valid_mask"] == [False] * 20 + [True] * 380
    assert held.tactile_frame["values"][100] == 350.
    assert gap == before

    # Even new worker timestamps/IPC polls cannot prolong an old measurement.
    deadline = NOW + MAX_SHAPE_AGE_NS
    gap["pose"]["timestamp_s"] = deadline / 1e9
    final = composer.compose(gap, touch, now_ns=deadline)
    assert final.mode == "held" and final.pose_age_ms == 500 and final.tactile_frame is None
    expired = composer.compose(gap, touch, now_ns=deadline + 1)
    assert expired.mode == "waiting" and not expired.uses_tracked_geometry
    assert expired.geometry is None and expired.geometry_key is None

    resumed = dict(pose(deadline + 2, hand=hand), geometry=deepcopy(neutral))
    live = composer.compose(resumed, None, now_ns=deadline + 2)
    assert live.pose_current and live.uses_tracked_geometry and live.geometry is resumed["geometry"]
    assert live.geometry_key != first.geometry_key


@pytest.mark.parametrize("change", [
    {"pose": {"reason": "input_health_invalid"}},
    {"pose": {"reason": "process_unresponsive"}},
    {"pose": {"calibration_rejections": {"11": "filter_epoch_changed_recalibration_required"}}},
    {"input_snapshot": {"orientation": {"input_health": {"enabled": True, "recovering_nodes": ["11"]}}}},
    {"input_snapshot": {"orientation": {"nodes": {"11": {"epoch": 4}}}}},
    {"input_snapshot": {"orientation": {"nodes": {"11": {"orientation_valid": False}}}}},
    {"runtime": {"connected": False}},
    {"runtime": {"session_id": "new-session"}, "pose": {"session_id": "new-session"}},
    {"runtime": {"error": "serial read failed"}},
    {"pose": {"profile_version": 2}},
    {"pose": {"hand": "left"}},
    {"pose": {"source_monotonic_ns": NOW + 2}},
    {"pose": {"source_monotonic_ns": NOW - 1}},
    {"calibration": {"clear_generation": 1}},
    {"calibration": {"profile_status": "fitting"}},
    {"calibration": {"active": {"kind": "mounting"}}},
])
def test_fault_or_context_change_forgets_previous_shape_even_if_a_timing_gap_follows(change):
    composer = DisplayComposer(geometry())
    motion = pose()
    motion["orientation"] = {"nodes": {"11": {"epoch": 3}}}
    composer.compose(motion, None, now_ns=NOW)
    fault = missing_palm(motion)
    for block, fields in change.items():
        fault.setdefault(block, {}).update(fields)
    for snapshot in (fault, missing_palm(motion)):
        frame = composer.compose(snapshot, tactile(), now_ns=NOW + 1)
        assert not frame.uses_tracked_geometry and not frame.pose_current
        assert frame.tactile_frame is not None and frame.skew_ms is None


def test_short_gap_before_any_measured_hardware_shape_uses_reference_hand():
    composer = DisplayComposer(geometry())
    for motion in (pose(hardware_accessed=False), pose()):
        if not motion["pose"]["hardware_accessed"]:
            assert composer.compose(motion, None, now_ns=NOW).pose_current
        frame = composer.compose(missing_palm(motion), tactile(), now_ns=NOW + 1)
        assert frame.mode == "waiting" and not frame.pose_current and frame.geometry is None


@pytest.mark.parametrize("override, expected", [
    ({}, "held"),
    ({"faulted": True}, "waiting"),
    ({"state": "recovering"}, "waiting"),
    ({"reasons": ["frozen_bank"]}, "waiting"),
    ({"integration_allowed": False}, "waiting"),
    ({"age_s": -0.1}, "waiting"),
])
def test_only_time_expiry_in_input_health_can_bridge_a_missing_palm(override, expected):
    composer = DisplayComposer(geometry())
    motion = pose()
    composer.compose(motion, None, now_ns=NOW)
    gap = missing_palm(motion)
    # Real receive-health snapshots keep aged nodes after removing their
    # orientation. "healthy": False alone is not a hardware fault.
    node = dict(state="stale", healthy=False, faulted=False, integration_allowed=True, reasons=[], age_s=.18)
    node.update(override)
    gap["input_snapshot"] = {"orientation": {"nodes": {}, "input_health": {
        "enabled": True, "faulted_nodes": [], "recovering_nodes": [], "nodes": {"11": node}}}}
    frame = composer.compose(gap, None, now_ns=NOW + 180_000_000)
    assert frame.mode == expected and not frame.pose_current


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
    assert touch.frame["values"][:2] == [100., 0.]
    assert result.tactile_frame["values"][:2] == [0., 100.]
    # A newer press is visible even if the next motion source time lags behind.
    future = tactile(NOW + 20_000_000, sequence=2, value=200.)
    paired = composer.compose(pose(NOW + 10_000_000), future, now_ns=NOW + 20_000_000)
    assert paired.tactile_frame["values"][:2] == [200., 200.]
    assert paired.skew_ms == 10


@pytest.mark.parametrize("phase", ["collecting_neutral", "moving_fingers", "preparing_thumb", "moving_thumb"])
def test_finger_capture_uses_current_measured_motion_until_fitting(phase):
    motion, touch = pose(), tactile()
    motion["calibration"].update(active={"kind": "functional_hand", "step": "neutral", "phase": phase},
                                 calibrated_nodes=["11"], profile_status="collecting_functional_hand")
    composer = DisplayComposer(geometry())
    live = composer.compose(motion, touch, now_ns=NOW)
    assert live.pose_current and live.geometry is motion["geometry"] and live.tactile_frame is not None
    assert not composer.compose(motion, touch, now_ns=NOW + 300_000_000).pose_current
    motion["calibration"]["profile_status"] = "fitting_functional_hand"
    assert not composer.compose(motion, touch, now_ns=NOW + 300_000_001).pose_current
    # In particular, even a still-fresh previous pose cannot escape a fit boundary.
    assert not DisplayComposer(geometry()).compose(motion, touch, now_ns=NOW).pose_current
    motion["calibration"].update(active={"kind": "mounting", "step": "neutral"}, profile_status="collecting")
    assert not DisplayComposer(geometry()).compose(motion, touch, now_ns=NOW).pose_current


def test_skew_uses_real_palm_receive_time_while_showing_the_newest_pressure():
    composer = DisplayComposer(geometry())
    composer.compose(pose(), tactile(value=10.), now_ns=NOW)
    # Independent palm and tactile samples are both current at display time.
    result = composer.compose(pose(NOW + 60_000_000, source=NOW + 10_000_000),
                              tactile(NOW + 50_000_000, sequence=2, value=20.),
                              now_ns=NOW + 60_000_000)
    assert result.tactile_frame["values"] == [20.] * 400
    assert result.pose_age_ms == 50 and result.tactile_age_ms == 10 and result.skew_ms == 40


def test_snapshot_after_pose_preserves_actual_row_receive_time():
    result = DisplayComposer(geometry()).compose(
        pose(NOW + 10_000_000), tactile(snapshot=NOW + 20_000_000), now_ns=NOW + 20_000_000)
    assert result.pose_current and result.geometry is not None
    assert result.tactile_frame["valid_mask"] == [True] * 400
    assert result.tactile_age_ms == 20 and result.skew_ms == 10


@pytest.mark.parametrize("context", ["session", "profile", "generation"])
def test_pose_or_display_context_change_drops_old_tactile_but_admits_current_input(context):
    composer = DisplayComposer(geometry())
    composer.compose(pose(), tactile(value=10.), now_ns=NOW)
    current = pose(NOW + 10_000_000, session="new" if context == "session" else "pose-session",
                   profile=2 if context == "profile" else 1)
    touch = tactile(NOW + 20_000_000, sequence=2, generation=int(context == "generation"), value=20.)
    result = composer.compose(current, touch, now_ns=NOW + 20_000_000)
    assert result.tactile_frame["values"] == [20.] * 400
    later = pose(NOW + 30_000_000, session=current["pose"]["session_id"], profile=current["pose"]["profile_version"])
    resumed = composer.compose(later, touch, now_ns=NOW + 30_000_000)
    assert resumed.tactile_frame["values"] == [20.] * 400


def test_tactile_device_change_and_disconnect_never_resurrect_previous_hotspots():
    composer = DisplayComposer(geometry())
    composer.compose(pose(), tactile(), now_ns=NOW)
    new = tactile(NOW + 20_000_000, session="new-board", value=0.)
    assert composer.compose(pose(NOW + 10_000_000), new, now_ns=NOW + 20_000_000).tactile_frame["values"] == [0.] * 400
    disconnected = TactileDisplayState(None, "未连接", "")
    frame = composer.compose(pose(NOW + 30_000_000), disconnected, now_ns=NOW + 30_000_000)
    assert frame.geometry is not None and frame.pose_current and frame.tactile_frame is None
    assert composer.history_size == 0


def test_pure_tactile_uses_explicit_reference_mode_but_connected_uncalibrated_is_waiting():
    composer = DisplayComposer(geometry())
    result = composer.compose(dict(runtime=dict(connected=False)), tactile(), now_ns=NOW)
    assert result.mode == "reference" and not result.pose_current and result.tactile_frame is not None
    assert result.geometry is composer.neutral_geometry and result.skew_ms is None
    assert "固定参考" in result.status
    opened = dict(runtime=dict(connected=True, session_id="pose-session", received_samples=0),
                  pose=None, calibration=dict(profile_status="not_calibrated"))
    first = composer.compose(opened, tactile(), now_ns=NOW)
    assert first.status == "等待 IMU 数据" and not first.pose_current
    assert first.mode == "waiting" and first.geometry is None
    assert first.tactile_frame["valid_mask"] == [True] * 400
    assert first.tactile_age_ms == 0 and first.skew_ms is None
    snapshot = pose()
    snapshot["pose"].update(pose_available=False, reason="calibration_required")
    snapshot["calibration"]["profile_status"] = "not_calibrated"
    waiting = composer.compose(snapshot, tactile(), now_ns=NOW + 5_000_000)
    assert waiting.mode == "waiting" and waiting.geometry is None and not waiting.pose_current
    assert waiting.status == "等待手势标定" and waiting.tactile_frame["values"] == [100.] * 400
    assert waiting.tactile_age_ms == 5 and waiting.skew_ms is None


@pytest.mark.parametrize("mutation", ["session", "profile", "stale", "missing_source", "future_source", "root_old", "bad_mesh"])
def test_touch_remains_current_during_motion_faults_without_claiming_a_time_match(mutation):
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
    assert result.skew_ms is None
    if mutation == "missing_source":
        assert result.pose_current and result.geometry is snapshot["geometry"]
        assert result.tactile_frame is None and result.tactile_age_ms is None
    else:
        assert result.mode == "waiting" and not result.pose_current and result.geometry is None
        assert result.tactile_frame["values"] == [100.] * 400 and result.tactile_age_ms == 20


def test_missing_display_geometry_cannot_show_unavailable_joints_as_straight():
    snapshot = pose()
    flags = [True] * 16
    flags[2] = False
    snapshot["pose"].update(joint_valid_mask=flags, joint_display_available_mask=flags,
                            joint_quality=["valid" if good else "unavailable" for good in flags])
    result = DisplayComposer(geometry()).compose(snapshot, tactile(), now_ns=NOW)
    assert not result.pose_current and result.geometry is None


@pytest.mark.parametrize("hand", ["left", "right"])
def test_finger_pose_quality_never_hides_valid_touch_or_revives_missing_readings(hand):
    mesh = dict(geometry(), provenance={"hand": hand})
    layout = files("vt_tactile_capture") / "layouts" / "left_glove_from_right_initial.json" if hand == "left" else None
    composer = DisplayComposer(mesh, hand=hand, layout_path=layout)
    # Each finger has a joint without a fresh pose. A display mesh still exists.
    flags = [slot not in (1, 4, 7, 10, 13) for slot in range(16)]
    snapshot = pose(hand=hand, joint_valid_mask=flags, joint_display_available_mask=flags,
                    joint_quality=["valid" if good else "unavailable" for good in flags])
    displayed = dict(snapshot["pose"], display_only=True, held_slots=[1, 7], unknown_slots=[4, 13])
    snapshot.update(display_pose=displayed, display_geometry=dict(mesh, display_only=True), geometry=None)
    touch = tactile(NOW - 10_000_000)
    touch.frame.update(hand=hand, values=list(range(400)))
    result = composer.compose(snapshot, touch, now_ns=NOW)
    assert result.pose_current and result.geometry is snapshot["display_geometry"]
    assert result.tactile_frame["valid_mask"] == [True] * 400
    assert result.tactile_frame["values"] == list(range(400))
    assert result.status == "实时手势与触觉"

    # The next packet has one invalid channel and one expired scan row.
    partial = tactile(NOW, sequence=2)
    partial.frame["hand"] = hand
    partial.frame["valid_mask"][211] = False
    rows = list(partial.timing.row_monotonic_ns)
    rows[3] = NOW - partial.timing.max_age_ns - 1
    partial = replace(partial, timing=replace(partial.timing, row_monotonic_ns=tuple(rows)))
    result = composer.compose(snapshot, partial, now_ns=NOW)
    missing = {i for i, valid in enumerate(result.tactile_frame["valid_mask"]) if not valid}
    assert missing == {*range(60, 80), 211}
    assert "部分" in result.status


def test_old_display_mesh_from_a_different_profile_or_frame_is_not_a_current_pose():
    snapshot = pose()
    snapshot.update(display_pose=dict(snapshot["pose"], profile_version=0),
                    display_geometry=geometry(), geometry=None)
    result = DisplayComposer(geometry()).compose(snapshot, tactile(), now_ns=NOW)
    assert result.mode == "waiting" and result.geometry is None and not result.pose_current
    assert result.status == "手势模型暂不可用" and result.skew_ms is None
    assert result.tactile_frame["values"] == [100.] * 400


def test_reported_skew_does_not_hide_a_fresh_row_and_old_rows_expire():
    touch = tactile()
    # The transport governs row expiry independently of the reported source skew.
    touch = replace(touch, timing=replace(touch.timing, max_age_ns=250_000_000))
    composer = DisplayComposer(geometry())
    at_limit = composer.compose(pose(NOW + 150_000_000), touch, now_ns=NOW + 150_000_000)
    assert at_limit.tactile_frame is not None
    later = composer.compose(pose(NOW + 200_000_000), touch, now_ns=NOW + 200_000_000)
    assert later.skew_ms == 200. and later.tactile_frame["valid_mask"] == [True] * 400
    expired = composer.compose(pose(NOW + 250_000_001), touch, now_ns=NOW + 250_000_001)
    assert expired.geometry is not None and expired.tactile_frame is None
    rows = list(touch.timing.row_monotonic_ns)
    rows[3] = NOW - 100_000_001
    partial = replace(tactile(), timing=replace(tactile().timing, row_monotonic_ns=rows))
    result = DisplayComposer(geometry()).compose(pose(), partial, now_ns=NOW)
    assert [i for i, valid in enumerate(result.tactile_frame["valid_mask"]) if not valid] == list(range(60, 80))


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


def test_live_and_reference_show_identical_rolling_rows_under_pose_latency(native_composer):
    hand, neutral, live = native_composer
    reference = deepcopy(live)
    # At 115200 baud, 44 bytes (8N1) take ~3.82 ms per row, ~76 ms for 20 rows.
    row_period = round(44 * 10 * 1e9 / 115200)
    for tick in range(30):
        now = NOW + tick * 16_666_667
        row_id = now // row_period
        row_stamps = tuple((row_id - (row_id - row) % 20) * row_period for row in range(20))
        last_row = row_id * row_period
        press = tactile(last_row, sequence=row_id, snapshot=now, value=800.)
        press.frame["hand"] = hand
        press = replace(press, timing=replace(press.timing, row_monotonic_ns=row_stamps))
        # Alternate between a normal solver update and a longer scheduling delay.
        source = now - (45_000_000 if tick % 8 < 4 else 95_000_000)
        # Retain the latest measured palm until the next scheduled update arrives.
        if tick == 0 or source > motion["pose"]["source_monotonic_ns"]:
            motion = dict(pose(now, source=source, hand=hand), geometry=neutral)
        fixed = reference.compose({"runtime": {"connected": False}}, press, now_ns=now)
        tracked = live.compose(motion, press, now_ns=now)
        assert tracked.pose_current and tracked.geometry is neutral
        assert fixed.tactile_frame["valid_mask"] == [True] * 400
        assert tracked.tactile_frame == fixed.tactile_frame
        assert tracked.skew_ms == abs(motion["pose"]["source_monotonic_ns"] - last_row) / 1e6


def test_repeated_out_of_order_touch_cannot_reintroduce_an_old_press():
    composer = DisplayComposer(geometry())
    old = tactile(value=500.)
    released = tactile(NOW + 10_000_000, sequence=2, value=0.)
    now = NOW + 15_000_000
    assert composer.compose(pose(now), released, now_ns=now).tactile_frame["values"] == [0.] * 400
    for _ in range(3):
        assert composer.compose(pose(now), old, now_ns=now).tactile_frame is None
    assert composer.compose(pose(now), released, now_ns=now).tactile_frame["values"] == [0.] * 400


def test_press_and_release_update_while_measured_pose_is_between_ticks(native_composer):
    hand, neutral, composer = native_composer
    motion = dict(pose(hand=hand), geometry=neutral)
    for i, level in enumerate((0., 770., 0.)):
        stamp = NOW + i * 25_000_000
        touch = tactile(stamp, sequence=i + 1, value=0.)
        touch.frame["hand"] = hand
        touch.frame["values"][201] = level
        frame = composer.compose(motion, touch, now_ns=stamp)
        assert frame.geometry is neutral and frame.pose_current
        assert frame.tactile_frame["valid_mask"][201] and frame.tactile_frame["values"][201] == level
        assert frame.pose_age_ms == frame.skew_ms == i * 25 and frame.tactile_age_ms == 0.


def test_future_touch_clock_does_not_color_a_measured_pose():
    composer = DisplayComposer(geometry())
    for touch in (tactile(NOW + 1), tactile(snapshot=NOW + 1)):
        frame = composer.compose(pose(), touch, now_ns=NOW)
        assert frame.pose_current and frame.tactile_frame is None and frame.skew_ms is None


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


def test_left_composer_checks_motion_touch_and_geometry_handedness(monkeypatch):
    from types import SimpleNamespace
    from vt_capture_console.ui.hand import composition
    layout_path = "left-layout.json"
    def checked_layout(path, *, expected_hand):
        assert path == layout_path and expected_hand == "left"
        return SimpleNamespace(points=[])
    monkeypatch.setattr(composition, "load_layout", checked_layout)
    left_geometry = dict(geometry(), provenance={"hand": "left"})
    composer = DisplayComposer(left_geometry, hand="left", layout_path=layout_path)
    motion = dict(pose(hand="left"), geometry=left_geometry)
    touch = tactile()
    touch.frame["hand"] = "left"
    assert composer.compose(motion, touch, now_ns=NOW).tactile_frame is not None
    for wrong_hand in ("right", None):
        touch.frame["hand"] = wrong_hand
        assert composer.compose(motion, touch, now_ns=NOW).tactile_frame is None
        assert composer.history_size == 0
        assert composer.compose(dict(motion, pose=dict(motion["pose"], hand=wrong_hand)),
                                touch, now_ns=NOW).geometry is None
        assert composer.compose(dict(motion, geometry=dict(geometry(), provenance={"hand": wrong_hand})),
                                touch, now_ns=NOW).geometry is None


def test_touch_updates_independently_across_calibration_and_recovers_after_expiry(native_composer):
    hand, neutral, composer = native_composer
    pending = pose(hand=hand, pose_available=False, reason="calibration_required")
    pending["calibration"]["profile_status"] = "not_calibrated"
    unavailable = TactileDisplayState(None, "未连接", "")
    waiting = composer.compose(pending, unavailable, now_ns=NOW)
    assert waiting.status == "等待手势标定" and waiting.tactile_frame is None

    press = tactile(value=0.)
    press.frame["hand"] = hand
    press.frame["values"][93] = 750.
    colored = composer.compose(pending, press, now_ns=NOW + 12_000_000)
    assert colored.mode == "waiting" and colored.geometry is None and not colored.pose_current
    assert colored.tactile_frame["values"][93] == 750. and colored.tactile_frame["valid_mask"][93]
    assert colored.tactile_age_ms == 12 and colored.skew_ms is None
    expired = composer.compose(pending, press, now_ns=NOW + press.timing.max_age_ns + 1)
    assert expired.tactile_frame is None and expired.status == "等待手势标定"

    next_press = tactile(NOW + 140_000_000, sequence=2, value=0.)
    next_press.frame["hand"] = hand
    next_press.frame["values"][211] = 900.
    live_geometry = dict(neutral, vertices_m=[[0., 0.001, 0.], *neutral["vertices_m"][1:]])
    motion = dict(pose(NOW + 135_000_000, hand=hand), geometry=live_geometry)
    displayed = composer.compose(motion, next_press, now_ns=NOW + 145_000_000)
    assert displayed.pose_current and displayed.geometry is live_geometry and displayed.skew_ms == 5
    assert displayed.tactile_frame["values"][211] == 900. and displayed.tactile_frame["values"][93] == 0.
    motion = dict(pose(NOW + 150_000_000, hand=hand), geometry=live_geometry)
    paired = composer.compose(motion, next_press, now_ns=NOW + 152_000_000)
    assert paired.pose_current and paired.status == "实时手势与触觉"
    assert paired.tactile_frame["values"][211] == 900. and paired.tactile_frame["values"][93] == 0.
    assert paired.tactile_age_ms == 12 and paired.skew_ms == 10


@pytest.mark.parametrize("invalid", ["wrong_hand", "reference_lost", "clock_reversed"])
def test_waiting_for_motion_drops_invalid_touch_and_old_hotspots(invalid):
    composer = DisplayComposer(geometry())
    pending = pose(reason="calibration_required")
    press = tactile()
    assert composer.compose(pending, press, now_ns=NOW).tactile_frame is not None
    now = NOW
    if invalid == "wrong_hand":
        press.frame["hand"] = "left"
    elif invalid == "reference_lost":
        press = replace(press, response=ResponseStatus(phase="collecting"))
    else:
        now -= 1
    result = composer.compose(pending, press, now_ns=now)
    assert result.mode == "waiting" and result.geometry is None and not result.pose_current
    assert result.tactile_frame is None and result.skew_ms is None and composer.history_size == 0
