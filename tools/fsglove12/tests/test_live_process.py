"""Spawn/lifecycle and serial mesh scheduling; fake sources, no real device."""
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import threading
import time

import pytest

from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.live_process import ProcessLiveController, _idle, _publish
from vt_fsglove12.live_runtime import LiveController, _Session


def rest():
    return dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)


def wait_for(predicate, timeout=5.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError("spawn test did not reach expected state")


class Backend:
    def __init__(self, assets, hand="right"):
        self.pid, self.loaded = os.getpid(), False
        self.hand = hand

    def export_rest(self):
        self.loaded = True
        return rest()


class Runtime:
    """Top-level spawn-picklable fake, not a simulator used by product code."""
    def __init__(self, config, rest, model_axes, *, output_root, geometry_backend, **kwargs):
        self.backend, self.output = geometry_backend, Path(output_root)
        self.connected, self.stopped = False, False
        self.sequence, self.mode = 0, ""
        self.factory_kwargs = sorted(kwargs)
        assert geometry_backend.loaded

    def start(self):
        return self

    def submit(self, name, **args):
        if name == "connect":
            self.connected, self.mode = True, args["port"]
        elif name == "disconnect":
            # Artificial asynchronous disconnect to exercise local suppression.
            self.disconnect_at = time.monotonic() + .12
        elif name == "capture":
            self.mode = "HANG"
        return True

    def latest_snapshot(self):
        if self.mode == "EXIT":
            os._exit(7)
        if self.mode == "ERROR":
            self.mode = ""
            raise RuntimeError("deliberate snapshot failure")
        if self.mode == "HANG":
            self.mode = ""
            time.sleep(.4)
        if hasattr(self, "disconnect_at") and time.monotonic() >= self.disconnect_at:
            self.connected = False
        now = time.monotonic()
        self.sequence += 1
        value = _idle()
        value["runtime"].update(connected=self.connected, child_pid=os.getpid(), backend_pid=self.backend.pid,
                                backend_loaded=self.backend.loaded, factory_kwargs=self.factory_kwargs,
                                last_sample_age_s=0., sequence=self.sequence, hand=self.backend.hand)
        value["orientation"] = dict(timestamp_s=now, nodes={"12": dict(age_s=0., sample_index=self.sequence, epoch=0)})
        value["pose"].update(timestamp_s=now, pose_available=self.connected, solver_success=self.connected)
        value["geometry"] = dict(vertices_m=[[float(self.sequence), 0., 0.]], joints_m=[], faces=[])
        if self.mode == "OVERSIZE":
            value["geometry"]["oversize_test"] = "x" * 2_000_000
        return value

    def stop(self):
        self.stopped, self.connected = True, False
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / "closed.json").write_text(json.dumps({"pid": os.getpid()}))

    def join(self, timeout=None):
        return self.stopped


class RevisionRuntime(Runtime):
    def snapshot_revision(self):
        return 0  # Idle state never changes; command acknowledgement is separate.


class InputRevisionRuntime(RevisionRuntime):
    """Input keeps updating independently of frozen/slow pose publication."""
    def latest_input_snapshot(self):
        now = time.monotonic()
        self.input_sequence = getattr(self, "input_sequence", 0) + 1
        return dict(runtime=dict(connected=self.connected, session_id="input-test",
            published_monotonic_s=now, received_samples=self.input_sequence,
            last_sample_age_s=0., clear_generation=0, input_diagnostics=None),
            orientation=dict(timestamp_s=now, session_id="input-test", nodes={
                "12": dict(age_s=0., sample_index=self.input_sequence, epoch=0)}))


@pytest.fixture
def controller(tmp_path):
    created = []

    def make(**kwargs):
        value = ProcessLiveController(load_config(), rest(), {}, output_root=tmp_path,
                                      mano_assets="UNUSED_FAKE_ASSETS", backend_factory=Backend,
                                      runtime_factory=kwargs.pop("runtime_factory", Runtime), **kwargs)
        created.append(value)
        return value

    yield make
    for value in created:
        value.stop()
        assert value.join(3.)


def test_spawn_idle_then_explicit_connect_same_geometry_newest_only(controller, tmp_path):
    value = controller().start()
    snapshot = wait_for(lambda: (s if (s := value.latest_snapshot())["runtime"].get("backend_loaded") else None))
    assert snapshot["runtime"]["child_pid"] == snapshot["runtime"]["backend_pid"] != os.getpid()
    assert value.renders_geometry and not snapshot["runtime"]["connected"]
    assert snapshot["geometry"] is None and not list(tmp_path.iterdir())
    assert value.submit("connect", port="FAKE")
    snapshot = wait_for(lambda: (s if (s := value.latest_snapshot())["pose"]["pose_available"] else None))
    first = snapshot["runtime"]["sequence"]
    time.sleep(.1)  # Don't poll; mailbox must overwrite, not queue ten old frames.
    later = value.latest_snapshot()
    assert later["runtime"]["sequence"] > first + 2
    assert later["geometry"]["vertices_m"][0][0] == later["runtime"]["sequence"]
    later["geometry"]["vertices_m"].clear()
    assert value.latest_snapshot()["geometry"]["vertices_m"]
    assert value.submit("disconnect")
    for _ in range(10):
        snapshot = value.latest_snapshot()
        assert snapshot["geometry"] is None and not snapshot["pose"]["pose_available"]
        time.sleep(.005)
    wait_for(lambda: not value.latest_snapshot()["runtime"]["connected"])
    value.stop()
    assert value.latest_snapshot()["geometry"] is None
    assert value.join(2.) and (tmp_path / "closed.json").is_file()


def test_two_hand_workers_are_independent_and_stopping_left_preserves_right(tmp_path):
    workers = {}
    try:
        for hand in ("left", "right"):
            workers[hand] = ProcessLiveController(load_config(hand=hand), rest(), {},
                output_root=tmp_path / hand, mano_assets="UNUSED_FAKE_ASSETS",
                backend_factory=Backend, runtime_factory=Runtime).start()
        snapshots = {hand: wait_for(lambda worker=worker: (
            s if (s := worker.latest_snapshot())["runtime"].get("backend_loaded") else None))
            for hand, worker in workers.items()}
        assert snapshots["left"]["runtime"]["child_pid"] != snapshots["right"]["runtime"]["child_pid"]
        for hand, worker in workers.items():
            assert snapshots[hand]["runtime"]["hand"] == hand
            assert not snapshots[hand]["runtime"]["connected"]
            assert worker.submit("connect", port="FAKE_" + hand)
            wait_for(lambda worker=worker: worker.latest_snapshot()["pose"]["pose_available"])
        workers["left"].stop()
        assert workers["left"].join(3.)
        assert workers["right"].latest_snapshot()["pose"]["pose_available"]
        assert not workers["right"]._stop.is_set()
        assert (tmp_path / "left" / "closed.json").is_file()
        assert not (tmp_path / "right" / "closed.json").exists()
    finally:
        for worker in workers.values():
            worker.stop()
            assert worker.join(3.)


@pytest.mark.skipif(not (os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") and os.environ.get("FSGLOVE12_MANO_ASSETS")),
                    reason="explicit official source and MANO resources required")
@pytest.mark.parametrize("output_rate_hz,gc_experiment", [(30., "0"), (120., "0"), (120., "1")])
def test_real_official_child_starts_idle_without_hardware_or_recording(
        tmp_path, monkeypatch, output_rate_hz, gc_experiment):
    import gc
    monkeypatch.setenv("VT_FSGLOVE_STARTUP_GC_FREEZE", gc_experiment)
    from vt_fsglove12.mano import ManoBackend
    from vt_fsglove12.upstream_core import load_upstream_core
    assets = os.environ["FSGLOVE12_MANO_ASSETS"]
    checkout = os.environ["FSGLOVE12_UPSTREAM_CHECKOUT"]
    core = load_upstream_core(checkout, assets)
    config = load_config()
    config.update(pose_backend="official_fsglove_sparse_adapter", upstream_provenance=core.provenance,
                  output_rate_hz=output_rate_hz)
    model = ManoBackend(assets)
    parent_gc = (gc.isenabled(), gc.get_threshold(), gc.get_freeze_count())
    proxy = ProcessLiveController(config, model.export_rest(), {}, mano_assets=assets,
        output_root=tmp_path, upstream_checkout=checkout).start()
    try:
        def ready():
            snapshot = proxy.latest_snapshot()
            return snapshot if proxy._sequence > 0 else None
        snapshot = wait_for(ready, timeout=15.)
        assert proxy.is_alive()
        assert not snapshot["runtime"]["error"]
        assert not snapshot["runtime"]["connected"]
        assert snapshot["runtime"]["received_samples"] == 0
        assert not snapshot["pose"]["pose_available"] and snapshot["geometry"] is None
        assert not list(tmp_path.iterdir())
        if output_rate_hz > 30.:
            assert snapshot["runtime"]["result_recording_mode"] == "bounded_process"
            if gc_experiment == "1":
                policy = snapshot["runtime"]["startup_gc_policy"]
                assert policy["applied"], policy
                assert policy["automatic_gc_enabled_after"]
                assert policy["thresholds_before"] == policy["thresholds_after"]
                assert policy["owner_pid"] == proxy.pid != os.getpid()
            else:
                assert "startup_gc_policy" not in snapshot["runtime"]
        else:
            assert "startup_gc_policy" not in snapshot["runtime"]
    finally:
        proxy.stop()
        assert proxy.join(5.)
    assert (gc.isenabled(), gc.get_threshold(), gc.get_freeze_count()) == parent_gc


def test_hung_worker_hides_stale_pose_and_read_is_nonblocking(controller):
    value = controller().start()
    assert value.submit("connect", port="FAKE")
    wait_for(lambda: value.latest_snapshot()["pose"]["pose_available"])
    assert value.submit("capture", step="neutral")
    time.sleep(.2)
    started = time.perf_counter()
    snapshot = value.latest_snapshot()
    assert time.perf_counter() - started < .05
    assert snapshot["geometry"] is None and not snapshot["pose"]["pose_available"]
    wait_for(lambda: value.latest_snapshot()["pose"]["pose_available"])


def test_clear_hides_old_calibration_until_execution_not_just_queue_ack(controller):
    value = controller()  # Drive its real mailbox deterministically, without a child.
    now = time.monotonic()
    old = _idle()
    old["runtime"].update(connected=True, published_monotonic_s=now)
    old["orientation"] = dict(timestamp_s=now, nodes={"12": dict(age_s=0.)})
    old["pose"].update(timestamp_s=now, pose_available=True, solver_success=True)
    old["geometry"] = dict(vertices_m=[[1., 2., 3.]])
    old["calibration"].update(clear_generation=3, captured_steps=["neutral"],
                               calibrated_nodes=["12"], functional_fingers=["little"])
    _publish(value._mailbox, old, 0)
    assert value.latest_snapshot()["pose"]["pose_available"]
    assert value.submit("clear_calibration")
    for seen in (0, value._command_id):
        _publish(value._mailbox, old, seen)
        pending = value.latest_snapshot()
        assert pending["runtime"]["connected"]
        assert pending["calibration"]["profile_status"] == "clearing"
        assert pending["calibration"]["captured_steps"] == []
        assert pending["calibration"]["functional_fingers"] == []
        assert not pending["pose"]["pose_available"] and pending["geometry"] is None
    cleared = deepcopy(old)
    cleared["calibration"] = {**_idle()["calibration"], "clear_generation": 4}
    cleared["pose"].update(pose_available=False, solver_success=False)
    cleared["geometry"] = None
    _publish(value._mailbox, cleared, value._command_id)
    actual = value.latest_snapshot()
    assert actual["runtime"]["connected"]
    assert actual["calibration"]["profile_status"] == "not_calibrated"
    assert value._pending_clear_generation is None


def test_manual_reference_and_functional_commands_are_explicit_and_validated(controller):
    value = controller()
    assert not value.submit("capture_functional", finger="index")
    assert not value.submit("capture_functional")
    assert not value.submit("reanchor_neutral", finger="thumb")
    assert value.submit("reanchor_neutral")
    assert value.submit("capture_functional", finger="thumb")
    assert value.submit("capture_functional", finger="little")
    assert not value.submit("set_functional_enabled", enabled=True)


def test_official_five_finger_and_switch_commands_never_enable_legacy_reference(controller):
    value = controller(upstream_checkout="EXPLICIT_MOCK_NOT_STARTED")
    assert not value.submit("reanchor_neutral")
    for finger in ("index", "middle", "ring", "little", "thumb"):
        assert value.submit("capture_functional", finger=finger)
    assert not value.submit("capture_functional", finger="invalid")
    assert not value.submit("set_functional_enabled")
    for invalid in (1, "false", None):
        assert not value.submit("set_functional_enabled", enabled=invalid)
    assert value.submit("set_functional_enabled", enabled=False)
    assert value.submit("set_functional_enabled", enabled=True)
    assert value.pid is None  # Submission never opens hardware or starts a process.


def test_whole_hand_command_is_official_only_and_has_no_arguments(controller):
    legacy = controller()
    assert not legacy.submit("capture_functional_hand")
    official = controller(upstream_checkout="EXPLICIT_MOCK_NOT_STARTED")
    assert not official.submit("capture_functional_hand", finger="thumb")
    assert official.submit("capture_functional_hand")
    assert official.submit("cancel_capture")
    assert official.pid is None


def test_child_death_hides_previous_pose_and_never_restarts(controller):
    value = controller().start()
    assert value.submit("connect", port="FAKE")
    wait_for(lambda: value.latest_snapshot()["pose"]["pose_available"])
    assert value.submit("connect", port="EXIT")
    wait_for(lambda: not value.is_alive())
    snapshot = value.latest_snapshot()
    assert snapshot["geometry"] is None and snapshot["pose"]["reason"] == "process_exited"
    assert not value.submit("connect", port="FAKE")
    assert "7" in snapshot["runtime"]["error"]


def test_fatal_runtime_error_survives_clean_shutdown_publish(controller):
    value = controller().start()
    assert value.submit("connect", port="ERROR")
    wait_for(lambda: not value.is_alive())
    snapshot = value.latest_snapshot()
    assert "deliberate snapshot failure" in snapshot["runtime"]["error"]
    assert snapshot["geometry"] is None


def test_oversize_snapshot_is_explicit_and_stop_is_not_a_queue_command(controller):
    value = controller(mailbox_capacity=4096).start()
    assert value.submit("connect", port="OVERSIZE")
    snapshot = wait_for(lambda: (s if "snapshot_capacity_exceeded" in str((s := value.latest_snapshot())["runtime"].get("error")) else None))
    assert snapshot["geometry"] is None
    value.stop()
    assert value.join(2.)


def test_mailbox_lock_contention_never_waits_and_bounded_commands(controller):
    value = controller()
    _publish(value._mailbox, _idle(), 0)
    value.latest_snapshot()
    value._mailbox[3].acquire()
    try:
        started = time.perf_counter()
        assert value.latest_snapshot()["geometry"] is None
        assert time.perf_counter() - started < .05
    finally:
        value._mailbox[3].release()
    assert all(value.submit("cancel_capture") for _ in range(16))
    assert not value.submit("cancel_capture")
    value.stop()
    assert not value.submit("connect", port="FAKE")
    assert value.join(.1)
    with pytest.raises(RuntimeError):
        value.start()


def test_replay_factory_supplied_without_autoconnect(controller):
    value = controller(replay_raw="NOT_OPENED.csv", replay_speed=2.).start()
    snapshot = wait_for(lambda: (s if (s := value.latest_snapshot())["runtime"].get("backend_loaded") else None))
    assert snapshot["runtime"]["factory_kwargs"] == ["source_factory"]
    assert not snapshot["runtime"]["connected"]


def test_unchanged_revision_never_recopies_snapshot_and_idle_is_not_hung(controller):
    value = controller(runtime_factory=RevisionRuntime).start()
    snapshot = wait_for(lambda: (s if (s := value.latest_snapshot())["runtime"].get("backend_loaded") else None))
    sequence, ipc_sequence = snapshot["runtime"]["sequence"], snapshot["runtime"]["ipc_sequence"]
    time.sleep(.4)
    unchanged = value.latest_snapshot()
    assert unchanged["runtime"]["sequence"] == sequence
    assert unchanged["runtime"]["ipc_sequence"] == ipc_sequence
    assert not unchanged["runtime"]["error"]
    assert unchanged["pose"]["reason"] != "process_unresponsive"
    assert value.submit("cancel_capture")
    wait_for(lambda: value.latest_snapshot()["runtime"]["ipc_sequence"] > ipc_sequence)


def runtime_harness(tmp_path):
    config = load_config()
    calls, clock = [], [100.]

    class Fusion:
        sample_index = 1

        def snapshot(self, now):
            return dict(timestamp_s=now, nodes={"12": dict(quat_xyzw=[0., 0., 0., 1.], age_s=0., epoch=0,
                                                         sample_index=self.sample_index)})

    class Solver:
        def __init__(self, *_):
            pass

        def solve(self, observations, timestamp_s):
            calls.append(("solve", threading.get_ident()))
            return dict(timestamp_s=timestamp_s, pose_available=True, solver_success=True,
                        anatomical_euler_rad=[[0., 0., 0.]] * 16, root_quat_xyzw=[0., 0., 0., 1.])

    class Geometry:
        fail, delay = False, 0.

        def render(self, angles, root):
            calls.append(("render", threading.get_ident()))
            clock[0] += self.delay
            if self.fail:
                raise ValueError("mesh test failure")
            return dict(vertices_m=[[1., 2., 3.]], joints_m=[], faces=[])

    geometry = Geometry()
    controller = LiveController(config, rest(), {}, output_root=tmp_path, solver_factory=Solver,
                                geometry_backend=geometry, clock=lambda: clock[0])
    session = _Session("FAKE_SESSION", tmp_path, "NO_DEVICE", object(), Fusion(), object(), clock[0])
    session.opened, session.metadata_written, session.last_sample = True, True, clock[0]
    session.streams = {name: io.StringIO() for name in ("orientations", "poses", "events")}
    controller._session = session
    controller._profile = dict(profile_version=1, stale_after_s=config["stale_after_s"], nodes={
        "12": dict(epoch=0, world_alignment_xyzw=[0., 0., 0., 1.], sensor_from_bone_xyzw=[0., 0., 0., 1.])})
    return controller, session, geometry, calls, clock


def test_runtime_renders_serially_same_frame_and_does_not_log_geometry(tmp_path):
    controller, session, geometry, calls, clock = runtime_harness(tmp_path)
    controller._tick(session)
    snapshot = controller.latest_snapshot()
    assert calls == [("solve", threading.get_ident()), ("render", threading.get_ident())]
    assert snapshot["geometry"]["vertices_m"] == [[1., 2., 3.]]
    timing = snapshot["runtime"]["timing_ms"]
    assert set(timing) == {"solve", "calibration", "mesh", "total"}
    assert all(value >= 0 for value in timing.values())
    assert timing["total"] >= sum(timing[key] for key in ("solve", "calibration", "mesh"))
    logged = json.loads(session.streams["poses"].getvalue())
    assert "geometry" not in logged and "vertices_m" not in logged
    clock[0] += .2
    assert controller.latest_snapshot()["geometry"] is None


def test_render_failure_does_not_disconnect_or_stop_pose_recording(tmp_path):
    controller, session, geometry, calls, clock = runtime_harness(tmp_path)
    geometry.fail = True
    controller._tick(session)
    snapshot = controller.latest_snapshot()
    assert snapshot["runtime"]["connected"] and snapshot["pose"]["pose_available"]
    assert snapshot["geometry"] is None and "mesh test failure" in snapshot["geometry_error"]
    assert session.frames == 1
    geometry.fail = False
    session.fusion.sample_index += 1  # Recovery is evaluated on a genuinely new input.
    controller._tick(session)
    assert controller.latest_snapshot()["geometry"] and session.frames == 2
    assert controller.submit("disconnect")
    assert controller.latest_snapshot()["geometry"] is None


def test_slow_mesh_cannot_publish_expired_root(tmp_path):
    controller, session, geometry, calls, clock = runtime_harness(tmp_path)
    geometry.delay = .2
    controller._tick(session)
    snapshot = controller.latest_snapshot()
    assert snapshot["geometry"] is None and not snapshot["pose"]["pose_available"]
    assert not session.available_poses


def test_independent_input_mailbox_updates_without_republishing_frozen_mesh(controller):
    value = controller(runtime_factory=InputRevisionRuntime).start()
    assert value.submit("connect", port="FAKE")
    first = wait_for(lambda: (s if (s := value.latest_snapshot())["input_snapshot"]["runtime"].get("received_samples", 0) > 0
                             and s["runtime"].get("connected") else None))
    pose_sequence = first["runtime"]["ipc_sequence"]
    input_sequence = first["input_snapshot"]["runtime"]["input_ipc_sequence"]
    time.sleep(.35)
    current = value.latest_snapshot()
    assert current["runtime"]["ipc_sequence"] == pose_sequence
    assert current["input_snapshot"]["runtime"]["input_ipc_sequence"] >= input_sequence + 3
    assert current["runtime"]["connected"] and current["input_snapshot"]["runtime"]["connected"]
    assert current["input_snapshot"]["orientation"]["nodes"]["12"]["age_s"] < .12
    assert not current["pose"]["pose_available"] and current["geometry"] is None
    assert set(value.latest_input_snapshot()) == {"runtime", "orientation"}
    assert value._input_mailbox[1].value < 8192  # No 778-vertex mesh in this mailbox.


def test_input_publisher_keeps_running_while_pose_snapshot_blocks_and_exit_disconnects(controller):
    value = controller(runtime_factory=InputRevisionRuntime).start()
    assert value.submit("connect", port="FAKE")
    wait_for(lambda: value.latest_snapshot()["runtime"]["connected"])
    before = value.latest_input_snapshot()["runtime"]["input_ipc_sequence"]
    assert value.submit("capture", step="neutral")
    time.sleep(.3)
    current = value.latest_snapshot()
    assert current["input_snapshot"]["runtime"]["input_ipc_sequence"] >= before + 3
    assert current["runtime"]["connected"]
    assert not current["pose"]["pose_available"]
    assert value.submit("connect", port="EXIT")
    wait_for(lambda: not value.is_alive())
    assert not value.latest_input_snapshot()["runtime"]["connected"]
    assert value.latest_snapshot()["input_snapshot"]["orientation"] is None


def test_input_transport_ages_once_and_input_lock_never_blocks_gui(controller, monkeypatch):
    import vt_fsglove12.live_process as module
    value = controller()
    clock = [100.]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    light = dict(runtime=dict(connected=True, session_id="one", published_monotonic_s=100.,
        last_sample_age_s=.02, input_diagnostics=dict(last_byte_age_s=.03, last_sample_age_s=.02)),
        orientation=dict(timestamp_s=100., nodes={"12": dict(age_s=.01, sample_index=1)}))
    _publish(value._input_mailbox, light, 0, input_only=True)
    clock[0] = 100.1
    first = value.latest_input_snapshot()
    assert first["runtime"]["last_sample_age_s"] == pytest.approx(.12)
    assert first["orientation"]["nodes"]["12"]["age_s"] == pytest.approx(.11)
    assert first["runtime"]["input_diagnostics"]["last_byte_age_s"] == pytest.approx(.13)
    value._input_mailbox[3].acquire()
    try:
        clock[0] = 100.12
        started = time.perf_counter()
        second = value.latest_input_snapshot()
        assert time.perf_counter() - started < .05
        assert second["orientation"]["nodes"]["12"]["age_s"] == pytest.approx(.13)
    finally:
        value._input_mailbox[3].release()


def test_proxy_clear_and_true_disconnect_clear_display_but_stale_pose_does_not(controller):
    value = controller()
    current = _idle()
    current["runtime"]["connected"] = True
    current["orientation"] = dict(timestamp_s=time.monotonic() - 1., nodes={})
    current["display_pose"] = dict(display_only=True, source_timestamp_s=10.)
    current["display_geometry"] = dict(vertices_m=[[1., 2., 3.]])
    _publish(value._mailbox, current, 0)
    assert value.latest_snapshot()["display_geometry"] is not None
    assert value.submit("clear_calibration")
    cleared = value.latest_snapshot()
    assert cleared["display_pose"] is None and cleared["display_geometry"] is None
    assert cleared["runtime"]["connected"]
    assert value.submit("disconnect")
    assert not value.latest_input_snapshot()["runtime"]["connected"]


def display_runtime_harness(tmp_path):
    controller, session, geometry, calls, clock = runtime_harness(tmp_path)

    class OfficialPose:
        mode = "valid"

        def solve(self, frame, profile, *, timestamp_s):
            if self.mode == "missing_root":
                return dict(timestamp_s=timestamp_s, pose_available=False, solver_success=False,
                            algorithm="base_without_functional_revision", reason="missing_or_stale_root")
            angles = [[.1, .2, .3] for _ in range(16)]
            angles[8] = [.8, .1, .2]
            valid = [True] * 16
            if self.mode == "missing_joint":
                valid[8] = False
                angles[8] = [0., 0., 0.]
            return dict(timestamp_s=timestamp_s, pose_available=True, solver_success=True,
                algorithm="official", pose_processing_revision="functional_test", mano_pose_rotvec=deepcopy(angles),
                anatomical_euler_rad=angles, shape_params=[0.] * 10,
                joint_valid_mask=valid, joint_display_available_mask=list(valid),
                joint_quality=["valid" if v else "unavailable" for v in valid])

    class OfficialGeometry:
        delay = 0.

        def __init__(self):
            self.rendered = []

        def render_mano(self, angles, shape):
            self.rendered.append(deepcopy(angles))
            clock[0] += self.delay
            return dict(vertices_m=deepcopy(angles), joints_m=deepcopy(angles), faces=[])

    controller._pose_backend = OfficialPose()
    controller._geometry_backend = OfficialGeometry()
    return controller, session, controller._pose_backend, controller._geometry_backend, clock


def test_display_holds_local_angle_without_replacing_measurement_or_export_geometry(tmp_path):
    controller, session, backend, geometry, clock = display_runtime_harness(tmp_path)
    controller._tick(session)
    initial = controller.latest_snapshot()
    assert initial["display_pose"]["source_timestamp_s"] == 100.
    assert initial["geometry"] == initial["display_geometry"]
    assert len(geometry.rendered) == 1
    backend.mode = "missing_joint"
    clock[0] += .05
    session.fusion.sample_index += 1
    controller._tick(session)
    current = controller.latest_snapshot()
    assert current["pose"]["mano_pose_rotvec"][8] == [0., 0., 0.]
    assert current["pose"]["joint_valid_mask"][8] is False
    assert current["display_pose"]["mano_pose_rotvec"][8] == [.8, .1, .2]
    assert current["display_pose"]["held_slots"] == [8]
    assert current["display_pose"]["source_timestamp_by_slot"]["8"] == 100.
    assert current["display_geometry"]["display_only"]
    assert current["geometry"] is None
    assert len(geometry.rendered) == 2  # One display render, no second measurement render.
    logged = json.loads(session.streams["poses"].getvalue().splitlines()[-1])
    assert logged["mano_pose_rotvec"][8] == [0., 0., 0.]
    assert "display_pose" not in logged and "display_only" not in logged


def test_display_survives_missing_root_algorithm_omission_and_late_render_is_not_measurement(tmp_path):
    controller, session, backend, geometry, clock = display_runtime_harness(tmp_path)
    geometry.delay = .2
    controller._tick(session)
    late = controller.latest_snapshot()
    assert not late["pose"]["pose_available"] and late["geometry"] is None
    assert late["display_geometry"] is not None
    assert late["display_pose"]["source_timestamp_s"] == 100.
    geometry.delay = 0.
    backend.mode = "missing_root"
    clock[0] += .05
    controller._tick(session)
    missing = controller.latest_snapshot()
    assert missing["display_pose"]["source_timestamp_s"] == 100.
    backend.mode = "missing_joint"
    clock[0] += .05
    controller._tick(session)
    recovered = controller.latest_snapshot()
    assert recovered["display_pose"]["held_slots"] == [8]
    assert recovered["display_pose"]["mano_pose_rotvec"][8] == [.8, .1, .2]
    assert recovered["geometry"] is None
    controller._clear_calibration()
    cleared = controller.latest_snapshot()
    assert cleared["display_pose"] is None and cleared["display_geometry"] is None
