"""Physical-window calibration coverage and fail-closed input health gates.

All calls are synchronous, using synthetic snapshots: no device, worker,
licensed model, file writer or full pipeline benchmark is started.
"""
from copy import deepcopy
import io
import json
from types import SimpleNamespace
import threading

import pytest

from vt_fsglove12.live_runtime import LiveController


def controller(active=None, *, health=False):
    owner = LiveController.__new__(LiveController)
    owner._init_shape()  # This focused fixture bypasses the full constructor.
    owner.config = dict(node_to_slot={"12": 0}, input_health={"enabled": health})
    owner._active = active
    owner._profile = {"status": "saved", "profile_version": 3}
    owner._profile_status = "saved"
    owner._functional_hand_cancel = active.get("cancel") if active else None
    owner._functional_hand_result = None
    owner.messages, owner.events = [], []
    owner._message = lambda *args: owner.messages.append(args)
    owner._event = lambda *args, **kwargs: owner.events.append((args, kwargs))
    owner._clock = lambda: 10.
    owner.fits = []
    owner._fit_functional_hand_profile = lambda *args: owner.fits.append(args)
    return owner


def frame(stamp, *, index=None, state=None, invalid=False):
    data = dict(epoch=0, sample_index=round(stamp * 120) + 1 if index is None else index,
                age_s=0., orientation_valid=not invalid,
                quat_xyzw=[0., 0., 0., 1.], late_motion=stamp >= 8.)
    result = dict(timestamp_s=stamp, nodes={"12": data})
    if state is not None:
        status = dict(state=state, healthy=state == "healthy")
        data["input_health"] = deepcopy(status)
        result["input_health"] = dict(enabled=True, nodes={"12": status})
    return result


def regular(kind=None):
    active = dict(step="neutral", collect_at=0., end_at=2., seen=None, rows=[])
    if kind:
        active["kind"] = kind
    if kind == "functional":
        active.update(finger="little", neutral_end_at=2., end_at=10., motion_rows=[])
    return active


def whole_hand():
    return dict(step="neutral", kind="functional_hand", collect_at=3., neutral_end_at=5.,
                fingers_end_at=13., thumb_start_at=16., end_at=24., seen=None,
                capture_phase=None, rows=[], finger_rows=[], thumb_rows=[], cancel=threading.Event())


def assert_window(rows, start, end, cap):
    assert rows and len(rows) <= cap
    assert start <= rows[0]["timestamp_s"] < start + .06
    assert end - .06 < rows[-1]["timestamp_s"] < end
    assert all(start <= row["timestamp_s"] < end for row in rows)
    indices = [row["nodes"]["12"]["sample_index"] for row in rows]
    assert indices == sorted(set(indices))


@pytest.mark.parametrize("rate", [30, 120])
@pytest.mark.parametrize("kind", [None, "neutral_reference", "functional"])
def test_all_regular_capture_paths_cover_the_entire_physical_window(rate, kind):
    active = regular(kind)
    owner = controller(active)
    for tick in range(round(active["end_at"] * rate)):
        stamp = tick / rate
        current = frame(stamp)  # SAME 120 Hz raw stream sampled at each solve rate.
        owner._capture(None, current, stamp)
        owner._capture(None, current, stamp)  # Polling is not a new source sample.
    assert_window(active["rows"], 0., 2., 120)
    if kind == "functional":
        assert_window(active["motion_rows"], 2., 10., 300)
        assert any(row["nodes"]["12"]["late_motion"] for row in active["motion_rows"])
    if rate == 30:
        assert len(active["rows"]) == 59  # Preserve ordinary 30 Hz sample coverage.


@pytest.mark.parametrize("rate", [30, 120])
def test_full_hand_each_phase_covers_start_and_end_without_boundary_carryover(rate):
    active = whole_hand()
    owner = controller(active)
    for tick in range(24 * rate):
        stamp = tick / rate
        owner._capture(None, frame(stamp), stamp)
    assert_window(active["rows"], 3., 5., 120)
    assert_window(active["finger_rows"], 5., 13., 300)
    assert_window(active["thumb_rows"], 16., 24., 300)


def test_latest_bin_snapshot_preserves_async_new_nodes_without_inventing_ages_or_aliases():
    active = regular()
    owner = controller(active)
    first = frame(0., index=1)
    first["nodes"]["3"] = deepcopy(first["nodes"]["12"])
    owner._capture(None, first, 0.)
    second = deepcopy(first)
    second.update(timestamp_s=.001)
    second["nodes"]["12"]["sample_index"] = 2
    owner._capture(None, second, .001)
    third = deepcopy(second)
    third.update(timestamp_s=.002)
    third["nodes"]["12"]["age_s"] = .001
    third["nodes"]["3"]["sample_index"] = 2
    owner._capture(None, third, .002)
    assert len(active["rows"]) == 1
    row = active["rows"][0]
    assert row == third
    third["nodes"]["3"]["quat_xyzw"][0] = 999.
    assert row["nodes"]["3"]["quat_xyzw"][0] == 0.
    # Unchanged asynchronous observations are not counted in the NEXT bin.
    fourth = deepcopy(second)
    fourth.update(timestamp_s=.04)
    fourth["nodes"]["12"]["sample_index"] = 3
    fourth["nodes"]["3"]["sample_index"] = 2
    owner._capture(None, fourth, .04)
    assert set(active["rows"][1]["nodes"]) == {"12"}


@pytest.mark.parametrize("bad", ["orientation", "gyro", "accel", "missing", "index"])
def test_invalid_or_rejected_sample_is_not_saved_or_rehabilitated_by_repolling(bad):
    active = regular()
    owner = controller(active)
    owner._capture(None, frame(0.), 0.)
    current = frame(.05)
    if bad == "orientation":
        current["nodes"]["12"]["orientation_valid"] = False
    elif bad in {"gyro", "accel"}:
        current["nodes"]["12"]["input_filter"] = {bad + "_rejected": True}
    elif bad == "missing":
        current["nodes"] = {}
    else:
        current["nodes"]["12"]["sample_index"] = 0
    owner._capture(None, current, .05)
    assert active["rows"] == []
    if bad in {"orientation", "gyro", "accel"}:
        owner._capture(None, frame(.05), .05)
        assert active["rows"] == []
    owner._capture(None, frame(.1), .1)
    assert len(active["rows"]) == 1


@pytest.mark.parametrize("state", [None, "warming_up", "stale", "faulted", "recovering"])
@pytest.mark.parametrize("command", ["capture", "reanchor_neutral", "capture_functional", "capture_functional_hand"])
def test_health_unqualified_input_cannot_start_any_capture_or_erase_previous_profile(state, command):
    owner = controller(health=True)
    current = frame(10., state=state)
    fusion = SimpleNamespace(snapshot=lambda now: current)
    owner._session = SimpleNamespace(opened=True, done=threading.Event(), lock=threading.Lock(), fusion=fusion)
    before = deepcopy(owner._profile)
    with pytest.raises(ValueError, match="输入完整性尚未通过"):
        owner._command(command, {"step": "neutral", "finger": "little"})
    assert owner._active is None and owner._profile == before
    assert owner.events == []


@pytest.mark.parametrize("kind", [None, "neutral_reference", "functional", "functional_hand"])
def test_health_fault_after_good_rows_aborts_whole_capture_including_prepare_and_end(kind):
    active = whole_hand() if kind == "functional_hand" else regular(kind)
    owner = controller(active, health=True)
    start = active["collect_at"]
    owner._capture(None, frame(start, state="healthy"), start)
    owner._capture(None, frame(start + .05, state="healthy"), start + .05)
    assert active["rows"]
    saved = deepcopy(owner._profile)
    owner._capture(None, frame(active["end_at"], state="faulted"), active["end_at"])
    assert owner._active is None and owner._profile == saved and owner.fits == []
    assert owner.events[-1][1]["reason"] == "input_health"
    owner._capture(None, frame(active["end_at"] + .05, state="healthy"), active["end_at"] + .05)
    assert owner._active is None and owner.fits == []  # Recovery never auto-restarts capture.


def test_healthy_monitor_does_not_launder_missing_or_guard_rejected_orientation():
    owner = controller(health=True)
    current = frame(1., state="healthy")
    assert owner._capture_health(current) == {}
    current["nodes"]["12"]["input_filter"] = {"gyro_rejected": True}
    assert owner._capture_health(current) == {}  # A single rejected sample is skipped, not a failed capture.
    current["nodes"]["12"]["orientation_valid"] = False
    assert owner._capture_health(current) == {"12": "invalid_orientation"}
    current["nodes"] = {}
    assert owner._capture_health(current) == {"12": "missing_or_stale_node"}


@pytest.mark.parametrize("interruption", ["gyro", "accel", "stale", "missing", "warming_up"])
def test_transient_capture_interruption_skips_sample_then_keeps_whole_window(interruption):
    active = regular()
    owner = controller(active, health=True)
    for index in range(60):
        stamp = index / 30.
        current = frame(stamp, state="healthy")
        if index == 30:
            if interruption in {"gyro", "accel"}:
                current["nodes"]["12"]["input_filter"] = {interruption + "_rejected": True}
            elif interruption == "missing":
                current["nodes"] = {}
            else:
                current["input_health"]["nodes"]["12"] = dict(state=interruption, healthy=False)
                current["nodes"]["12"]["orientation_valid"] = False
        owner._capture(None, current, stamp)
        assert owner._active is active
    assert_window(active["rows"], 0., 2., 120)
    assert all(row["timestamp_s"] != 1. for row in active["rows"])
    assert len(active["rows"]) == 58
    assert owner.events == []


def test_health_is_checked_again_before_saving_after_a_slow_fit():
    owner = controller(regular(), health=True)
    current = frame(10., state="faulted")
    session = SimpleNamespace(lock=threading.Lock(), fusion=SimpleNamespace(snapshot=lambda now: current))
    saved = deepcopy(owner._profile)
    assert not owner._capture_save_healthy(session)
    assert owner._active is None and owner._profile == saved


def test_fault_and_recovery_between_ticks_cannot_launder_capture_epochs():
    active = regular()
    owner = controller(active, health=True)
    owner._capture(None, frame(0., state="healthy"), 0.)
    owner._capture(None, frame(.05, state="healthy"), .05)
    recovered = frame(.1, state="healthy")
    recovered["nodes"]["12"]["epoch"] = 1
    owner._capture(None, recovered, .1)
    assert owner._active is None
    assert owner.events[-1][1]["nodes"] == {"12": "filter_epoch_changed_during_capture"}


def test_fault_and_recovery_during_fit_cannot_save_old_epoch_profile():
    owner = controller(health=True)
    recovered = frame(10., state="healthy")
    recovered["nodes"]["12"]["epoch"] = 1
    session = SimpleNamespace(lock=threading.Lock(), fusion=SimpleNamespace(snapshot=lambda now: recovered))
    fitted = dict(nodes={"12": {"epoch": 0}})
    assert not owner._capture_save_healthy(session, fitted)
    assert owner.events[-1][1]["nodes"] == {"12": "filter_epoch_changed_during_capture"}


def test_full_hand_cancellation_discards_binned_capture_without_fitting():
    active = whole_hand()
    owner = controller(active)
    owner._capture(None, frame(3.), 3.)
    owner._capture(None, frame(3.05), 3.05)
    assert active["rows"]
    active["cancel"].set()
    owner._capture(None, frame(3.1), 3.1)
    assert owner._active is None and owner.fits == []
    assert owner._functional_hand_result["status"] == "cancelled"


@pytest.mark.parametrize("state, blocked", [("faulted", True), ("recovering", True), ("stale", False)])
def test_tick_confirmed_nonroot_fault_cannot_be_hidden_by_sparse_prior_or_counted_valid(
        tmp_path, state, blocked):
    from vt_fsglove12.config import PARENTS, load_config
    from vt_fsglove12.live_runtime import _Session

    config = load_config()
    config["input_health"] = {"enabled": True}
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    calls = []

    def solve(*args, **kwargs):
        calls.append("solve")
        return dict(pose_available=True, solver_success=True, mano_pose_rotvec=[[0., 0., 0.]] * 16)

    def render(*args):
        calls.append("render")
        return {"synthetic": True}

    owner = LiveController(config, rest, {}, output_root=tmp_path, clock=lambda: 10.,
        pose_backend=SimpleNamespace(solve=solve), geometry_backend=SimpleNamespace(render_mano=render))
    owner._display_pose_cache = SimpleNamespace(update=lambda *args, **kwargs: None)
    owner._profile = dict(status="saved", profile_version=1)
    current = frame(10., state="healthy")
    current["nodes"]["3"] = deepcopy(current["nodes"]["12"])
    current["input_health"].update(faulted_nodes=[], recovering_nodes=[])
    fusion = SimpleNamespace(snapshot=lambda now: current)
    session = _Session("synthetic-health", tmp_path, "NO_DEVICE", object(), fusion, object(), 10.)
    session.opened = session.metadata_written = True
    session.last_flush = 10.
    session.streams = {name: io.StringIO() for name in ("orientations", "poses", "events")}
    owner._session = session
    owner._tick(session)
    assert calls == ["solve", "render"]
    assert session.fresh_input_results == 1
    current["input_health"]["nodes"]["3"] = dict(state=state, healthy=False)
    if state != "stale":
        current["input_health"][state + "_nodes"] = ["3"]
    for data in current["nodes"].values():
        data["sample_index"] += 1
    owner._tick(session)
    pose = json.loads(session.streams["poses"].getvalue().splitlines()[-1])
    assert pose["input_health"] == current["input_health"]
    assert session.frames == 2  # Raw/orientation evidence still recorded.
    if blocked:
        assert calls == ["solve", "render"]
        assert pose["reason"] == "input_health_invalid" and not pose["pose_available"]
        assert session.fresh_input_results == session.available_poses == 1
        assert session.invalid_results == 1
    else:
        assert calls == ["solve", "render", "solve", "render"]
        assert session.fresh_input_results == 2


def test_confirmed_fault_bypasses_duplicate_suppression_even_if_source_identity_unchanged(tmp_path):
    from vt_fsglove12.config import PARENTS, load_config
    from vt_fsglove12.live_runtime import _Session

    config = load_config()
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    owner = LiveController(config, rest, {}, output_root=tmp_path, clock=lambda: 10.,
        pose_backend=SimpleNamespace(solve=lambda *args, **kwargs:
            dict(pose_available=True, solver_success=True)))
    owner._display_pose_cache = SimpleNamespace(update=lambda *args, **kwargs: None)
    owner._profile = dict(status="saved", profile_version=1)
    current = frame(10., state="healthy")
    current["input_health"].update(faulted_nodes=[], recovering_nodes=[])
    fusion = SimpleNamespace(snapshot=lambda now: current)
    session = _Session("synthetic-duplicate", tmp_path, "NO_DEVICE", object(), fusion, object(), 10.)
    session.opened = session.metadata_written = True
    session.last_flush = 10.
    session.streams = {name: io.StringIO() for name in ("orientations", "poses", "events")}
    owner._session = session
    owner._tick(session)
    current["input_health"]["faulted_nodes"] = ["3"]
    owner._tick(session)
    assert session.frames == 2 and session.duplicate_inputs_skipped == 0
    assert owner.latest_pose_snapshot()["pose"]["reason"] == "input_health_invalid"
