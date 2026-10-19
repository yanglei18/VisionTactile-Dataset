"""High-rate scheduling/provenance and bounded I/O; no hardware or timing benchmark."""
import io
import json
import threading

import pytest

from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.live_recording import AsyncResultRecorder
from vt_fsglove12.live_runtime import LiveController, _Session
from vt_fsglove12.live_stream import OnlineFusion, RawSample

from test_live_runtime import Clock, Fusion, Solver, harness, wait_for


def test_configured_120_hz_has_no_hidden_30_hz_cap(harness):
    controller, clock = harness.controller, harness.clock
    controller.config["output_rate_hz"] = 120.
    starts = []

    def tick(_session):
        starts.append(clock())
        clock.advance(.002)

    controller._tick = tick
    harness.connect()
    wait_for(lambda: len(starts) == 1)
    for index in range(1, 5):
        clock.value = starts[-1] + 1. / 120. + .00001
        wait_for(lambda: len(starts) == index + 1)
    assert starts[-1] - starts[0] < .034
    processing = controller.latest_snapshot()["runtime"]["processing"]
    assert processing["target_rate_hz"] == 120.
    assert processing["cycles"] == 5


def test_scheduler_jitter_does_not_shift_future_deadlines(harness):
    controller, clock = harness.controller, harness.clock
    controller.config["output_rate_hz"] = 120.
    starts = []
    controller._tick = lambda _session: starts.append(clock())
    harness.connect()
    wait_for(lambda: len(starts) == 1)
    anchor = starts[0]
    clock.value = anchor + 1. / 120. + .002
    wait_for(lambda: len(starts) == 2)
    # The second wake has less jitter. Its deadline must remain on the
    # original grid, not move 2 ms later because of the previous wake.
    clock.value = anchor + 2. / 120. + .001
    wait_for(lambda: len(starts) == 3)


def test_overrun_resumes_latest_input_without_waiting_another_full_grid_slot(harness):
    controller, clock = harness.controller, harness.clock
    controller.config["output_rate_hz"] = 120.
    starts = []

    def tick(_session):
        starts.append(clock())
        if len(starts) < 4:
            clock.advance(.009)

    controller._tick = tick
    harness.connect()
    wait_for(lambda: len(starts) >= 4)
    assert starts[3] - starts[0] == pytest.approx(.027)


def setup_direct(tmp_path):
    clock, config = Clock(), load_config()
    config["output_rate_hz"] = 120.
    # These scheduling/deduplication fixtures inject only a few sparse samples;
    # acquisition qualification and its warmup have separate health regressions.
    config["input_health"] = {"enabled": False}
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)

    class Backend:
        def __init__(self):
            self.calls = []

        def solve(self, frame, profile, timestamp_s):
            self.calls.append(frame)
            available = "12" in frame["nodes"]
            return dict(timestamp_s=timestamp_s, pose_available=available, solver_success=available,
                        mano_pose_rotvec=[[0., 0., 0.]] * 16,
                        anatomical_euler_rad=[[0., 0., 0.]] * 16,
                        root_quat_xyzw=[0., 0., 0., 1.], shape_params=[0.] * 10)

    backend, fusion = Backend(), Fusion(config)
    controller = LiveController(config, rest, {}, output_root=tmp_path, clock=clock,
                                pose_backend=backend, solver_factory=Solver)
    session = _Session("TEST", tmp_path, "NO_DEVICE", object(), fusion, object(), clock())
    session.opened = session.metadata_written = True
    session.streams = {key: io.StringIO() for key in ("orientations", "poses", "events")}
    controller._session = session
    controller._profile = dict(profile_version=1)
    return controller, session, backend, clock


def feed(session, clock, *nodes):
    session.fusion.update([RawSample(node, (0, 0, 0, 0, 0, 8192), clock(), 1234.) for node in nodes])


def test_pose_mailbox_snapshot_does_not_copy_independent_input_monitor(tmp_path, monkeypatch):
    controller, session, _backend, clock = setup_direct(tmp_path)
    feed(session, clock, 12)
    controller._tick(session)
    reads = []
    monkeypatch.setattr(controller, "latest_input_snapshot", lambda: reads.append(1) or {"separate": True})
    pose = controller.latest_pose_snapshot()
    assert not reads and "input_snapshot" not in pose
    assert pose["pose"]["pose_available"]
    pose["pose"]["root_quat_xyzw"][0] = 99.
    assert controller.latest_pose_snapshot()["pose"]["root_quat_xyzw"][0] == 0.
    assert controller.latest_snapshot()["input_snapshot"] == {"separate": True}
    assert reads == [1]


def test_duplicate_sample_never_gets_new_solve_timestamp_or_record(tmp_path):
    controller, session, backend, clock = setup_direct(tmp_path)
    feed(session, clock, 12, 3)
    assert controller._tick(session)
    original = controller.latest_snapshot()["pose"]
    clock.advance(1 / 120.)
    assert not controller._tick(session)
    assert len(backend.calls) == session.frames == 1
    assert controller.latest_snapshot()["pose"] == original
    assert session.duplicate_inputs_skipped == 1
    assert len(session.streams["poses"].getvalue().splitlines()) == 1
    # Updating reception bookkeeping without advancing a source sample index
    # is not new data, even when it supplies a different integer timestamp.
    session.fusion.nodes["12"]["host_monotonic_ns"] = 999_000_000
    assert not controller._tick(session)
    # Auxiliary node 11 is not a MANO observation and cannot manufacture a new pose.
    feed(session, clock, 11)
    assert not controller._tick(session)
    # A genuinely new mapped finger observation is useful even when the palm's
    # source timestamp is unchanged; provenance remains per-node.
    feed(session, clock, 3)
    assert controller._tick(session)
    assert len(backend.calls) == session.frames == 2
    # Same counter after a fusion epoch reset must also be a new input identity.
    session.fusion.nodes["12"]["epoch"] += 1
    assert controller._tick(session)
    # Explicit profile changes are not swallowed by input deduplication.
    controller._profile = dict(profile_version=2)
    assert controller._tick(session)
    assert len(backend.calls) == 4
    assert not controller.latest_snapshot()["pose"]["source_fresh"]
    assert session.fresh_input_results == 3
    assert session.state_only_results == 1


def test_dedup_cannot_hide_stale_status_from_custom_fusion(tmp_path):
    controller, session, _backend, clock = setup_direct(tmp_path)
    feed(session, clock, 12)
    assert controller._tick(session)
    frame = session.fusion.snapshot(clock())
    session.fusion.snapshot = lambda now: {**frame, "timestamp_s": now,
        "nodes": {"12": {**frame["nodes"]["12"], "age_s": .5}}}
    clock.advance(.2)
    assert controller._tick(session)
    assert not controller.latest_snapshot()["pose"]["pose_available"]
    assert not controller._snapshot["pose"]["source_fresh"]


def test_duplicate_auxiliary_sample_does_not_swallow_one_shot_gap_warning(tmp_path):
    controller, session, _backend, clock = setup_direct(tmp_path)
    session.fusion = OnlineFusion(controller.config)
    # Node 11 is deliberately outside the MANO mapping. A gap on it must
    # survive the duplicate suppression of the unchanged mapped palm sample.
    session.fusion.update([RawSample(11, (0, 0, 0, 0, 0, 8192), clock() - .2, 1234.),
                           RawSample(12, (0, 0, 0, 0, 0, 8192), clock(), 1234.)])
    assert controller._tick(session)
    clock.advance(.01)
    session.fusion.update([RawSample(11, (0, 0, 0, 0, 0, 8192), clock(), 1234.)])
    warning = "node_11_short_gap_filter_preserved"
    assert warning in session.fusion._pending_warnings
    assert controller._tick(session)
    frame = json.loads(session.streams["orientations"].getvalue().splitlines()[-1])
    assert warning in frame["warnings"]
    assert not controller._snapshot["pose"]["source_fresh"]
    assert not session.fusion._pending_warnings
    assert not controller._tick(session)  # Constant baseline warnings do not force more output.


def test_solved_rate_counts_only_new_available_input_and_ages_to_zero(tmp_path):
    controller, session, _backend, clock = setup_direct(tmp_path)
    feed(session, clock, 12)
    assert controller._tick(session)
    assert controller._processing_status(session)["solved_rate_hz"] == 0.
    clock.advance(1. / 120.)
    feed(session, clock, 12)
    assert controller._tick(session)
    assert controller._processing_status(session)["solved_rate_hz"] == pytest.approx(120.)
    clock.advance(.001)
    controller._profile = dict(profile_version=2)
    assert controller._tick(session)
    assert not controller._tick(session)
    status = controller._processing_status(session)
    assert status["fresh_input_results"] == 2
    assert status["state_only_results"] == 1
    assert status["duplicate_inputs_skipped"] == 1
    assert status["solved_rate_hz"] == pytest.approx(120.)
    clock.advance(.3)
    assert controller._tick(session)
    status = controller._processing_status(session)
    assert status["fresh_input_results"] == 2
    assert status["invalid_results"] == 1
    assert status["solved_rate_hz"] == 0.


def test_duplicate_filter_keeps_calibration_wait_and_stale_heartbeats(tmp_path):
    controller, session, backend, clock = setup_direct(tmp_path)
    controller._profile = None
    feed(session, clock, 12)
    assert controller._tick(session)
    clock.advance(.01)
    assert controller._tick(session)
    assert len(backend.calls) == 0 and session.frames == 2
    controller._profile = dict(profile_version=1)
    assert controller._tick(session)
    clock.advance(.2)
    assert controller._tick(session)
    assert not controller.latest_snapshot()["pose"]["pose_available"]
    assert controller._tick(session)  # Missing-root status still has an explicit result.


def test_async_result_writer_drains_flushes_and_preserves_pair_order(tmp_path):
    paths = {key: tmp_path / f"{key}.jsonl" for key in ("orientations", "poses")}
    writer = AsyncResultRecorder({key: path.open("w") for key, path in paths.items()})
    for index in range(25):
        writer.submit(dict(frame=index), dict(pose=index))
    assert writer.close(2.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["written_pairs"] == 25
    assert status["pending_pairs"] == status["failed_pairs"] == status["rejected_pairs"] == 0
    assert not status["writer_alive"] and status["error"] is None
    for key, path in paths.items():
        field = "frame" if key == "orientations" else "pose"
        assert [json.loads(row)[field] for row in path.read_text().splitlines()] == list(range(25))


class GatedStream(io.StringIO):
    def __init__(self, *, fail=False):
        super().__init__()
        self.entered, self.release = threading.Event(), threading.Event()
        self.fail = fail

    def write(self, value):
        self.entered.set()
        if not self.release.wait(2.):
            raise OSError("test gate timeout")
        if self.fail:
            raise OSError("test disk full")
        return super().write(value)


def test_queue_saturation_is_explicit_and_previously_accepted_pairs_are_drained():
    stream = GatedStream()
    writer = AsyncResultRecorder(dict(orientations=stream, poses=io.StringIO()), capacity=1)
    try:
        writer.submit(dict(index=0), dict(index=0))
        assert stream.entered.wait(1.)
        writer.submit(dict(index=1), dict(index=1))
        with pytest.raises(OSError, match="queue full"):
            writer.submit(dict(index=2), dict(index=2))
        status = writer.snapshot()
        assert status["accepted_pairs"] == 2 and status["rejected_pairs"] == 1
        assert status["queue_high_water_pairs"] == 1
        assert not writer.close(0.)  # A blocked disk writer is never killed or hidden.
    finally:
        stream.release.set()
        assert writer.close(2.)
    assert writer.snapshot()["written_pairs"] == 2
    with pytest.raises(OSError, match="queue full"):
        writer.check()


def test_async_disk_error_accounts_for_all_accepted_but_failed_pairs():
    stream = GatedStream(fail=True)
    writer = AsyncResultRecorder(dict(orientations=stream, poses=io.StringIO()), capacity=2)
    try:
        writer.submit(dict(index=0), dict(index=0))
        assert stream.entered.wait(1.)
        writer.submit(dict(index=1), dict(index=1))
    finally:
        stream.release.set()
        assert writer.close(2.)
    status = writer.snapshot()
    assert status["accepted_pairs"] == status["failed_pairs"] == 2
    assert status["pending_pairs"] == status["written_pairs"] == 0
    with pytest.raises(OSError, match="test disk full"):
        writer.check()


def test_async_recording_failure_marks_live_summary_incomplete(harness):
    harness.connect()
    session = harness.controller._session
    session.result_recorder._fail(OSError("test result disk full"))
    harness.clock.advance(.05)
    wait_for(lambda: session.recording_disabled.is_set())
    assert harness.controller.latest_snapshot()["runtime"]["connected"]
    assert harness.controller.submit("disconnect")
    wait_for(lambda: (session.directory / "summary.json").exists())
    summary = json.loads((session.directory / "summary.json").read_text())
    assert not summary["recording_complete"]
    assert "test result disk full" in summary["result_recording"]["error"]
    assert any("test result disk full" in error for error in summary["errors"])
