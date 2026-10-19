"""Thread/lifecycle contracts with fake sources, never hardware or MANO."""
from collections import Counter
from copy import deepcopy
import csv
import io
import json
import queue
import threading
import time
from types import SimpleNamespace

import pytest

from vt_fsglove12.config import PARENTS, digest, load_config
from vt_fsglove12.guided_calibration import GuidedCalibrationError
from vt_fsglove12.live_runtime import COMMAND_CAPACITY, LiveController, STEP_KEYS, _Session, _input_wait_message
from vt_fsglove12.live_stream import OnlineFusion, RawRecorder, RawSample


@pytest.mark.parametrize("diagnostics, expected", [
    (None, None),
    ({"source_kind": "csv_replay", "bytes_received": 0}, None),
    ({"source_kind": "raw_serial", "bytes_received": 0}, "尚未收到任何字节"),
    ({"source_kind": "raw_serial", "bytes_received": 20, "samples_decoded": 0,
      "bytes_discarded": 0}, "等待完整 IMU 帧"),
    ({"source_kind": "raw_serial", "bytes_received": 2000, "samples_decoded": 0,
      "bytes_discarded": 1950}, "尚未解析到有效 IMU 帧"),
    ({"source_kind": "raw_serial", "bytes_received": 2000, "samples_decoded": 6,
      "bytes_discarded": 1900}, None),
])
def test_input_wait_message_distinguishes_absent_bytes_and_undecodable_input(diagnostics, expected):
    message = _input_wait_message(diagnostics)
    assert message is None if expected is None else expected in message


def test_results_export_is_optional_and_bridge_errors_do_not_stop_acquisition(harness):
    h = harness

    class Exporter:
        def __init__(self):
            self.documents, self.errors = [], []

        def publish(self, document):
            self.documents.append(document)
            raise OSError("deliberate absent bridge")

        def record_error(self, exc):
            self.errors.append(str(exc))

        def snapshot(self):
            return dict(enabled=True, sent=0, dropped=len(self.errors), last_error=self.errors[-1])

    exporter = Exporter()
    h.controller._pose_exporter = exporter
    h.connect()
    h.pump(nodes=tuple(range(1, 13)))
    wait_for(lambda: bool(exporter.errors))
    snapshot = h.controller.latest_snapshot()
    assert snapshot["runtime"]["connected"]
    assert not snapshot["runtime"]["error"]
    assert snapshot["runtime"]["ros_export"]["dropped"] >= 1
    assert exporter.documents[-1]["kind"] == "glove_state"
    assert not exporter.documents[-1]["pose_valid"]
    assert "values" not in exporter.documents[-1]
    assert h.controller._profile is None


def test_passive_input_diagnostics_reach_status_recording_and_summary(harness):
    h = harness
    diagnostics = dict(source_kind="raw_serial", opened=True, bytes_received=0,
                       samples_decoded=0, frames_decoded=0, bytes_discarded=0,
                       buffered_bytes=0, last_byte_age_s=None, last_sample_age_s=None)
    reads = []

    def setup(source):
        def snapshot_diagnostics(now=None):
            reads.append(now)
            return {**deepcopy(diagnostics), "opened": source.opened and not source.closed}
        source.snapshot_diagnostics = snapshot_diagnostics

    h.controls.source_setup = setup
    h.connect()
    h.pump(nodes=())
    snapshot = h.controller.latest_snapshot()
    assert "尚未收到任何字节" in snapshot["runtime"]["message"]
    diagnostics.update(bytes_received=300, bytes_discarded=250, buffered_bytes=50,
                       last_byte_age_s=.01)
    h.pump(nodes=())
    snapshot = h.controller.latest_snapshot()
    assert "尚未解析到有效 IMU 帧" in snapshot["runtime"]["message"]
    assert snapshot["runtime"]["input_diagnostics"]["bytes_received"] == 300
    assert snapshot["orientation"]["input_diagnostics"]["bytes_discarded"] == 250
    snapshot["runtime"]["input_diagnostics"]["bytes_received"] = -1
    assert h.controller.latest_snapshot()["runtime"]["input_diagnostics"]["bytes_received"] == 300

    diagnostics.update(bytes_received=468, frames_decoded=2, samples_decoded=12,
                       buffered_bytes=50, last_sample_age_s=.01)
    h.pump(nodes=tuple(range(1, 13)))
    snapshot = h.controller.latest_snapshot()
    assert len(snapshot["orientation"]["nodes"]) == 12
    assert "尚未解析" not in snapshot["runtime"]["message"]
    assert snapshot["runtime"]["received_samples"] == 12
    assert not snapshot["pose"]["pose_available"]  # No calibration is invented.
    directory = h.output / snapshot["runtime"]["session_id"]
    assert h.controller.submit("disconnect")
    wait_for(lambda: (directory / "summary.json").exists())
    summary = json.loads((directory / "summary.json").read_text())
    assert summary["input_diagnostics"]["bytes_discarded"] == 250
    assert summary["input_diagnostics"]["opened"] is False
    wait_for(lambda: h.controller.latest_snapshot()["runtime"]["input_diagnostics"]["opened"] is False)
    assert h.controller.latest_snapshot()["runtime"]["input_diagnostics"] == summary["input_diagnostics"]
    frames = [json.loads(line) for line in (directory / "orientations.jsonl").read_text().splitlines()]
    assert any(f["input_diagnostics"]["bytes_received"] == 300 for f in frames)
    with (directory / "raw.csv").open() as stream:
        assert len(list(csv.DictReader(stream))) == 12
    assert reads and all(isinstance(now, float) for now in reads)


def wait_for(predicate, timeout=2.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.003)
    raise AssertionError("fake controller did not reach the expected state")


class Clock:
    def __init__(self):
        self.value = 100.

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class Source:
    def __init__(self, port):
        self.port, self.batches = port, queue.Queue(maxsize=64)
        self.opened, self.closed = False, False
        self.open_thread = self.close_thread = None
        self.open_error = None
        self.open_gate = None

    def open(self):
        self.open_thread = threading.get_ident()
        if self.open_gate is not None:
            self.open_gate.wait(3.)
        if self.open_error:
            raise self.open_error
        self.opened = True

    def read_samples(self):
        assert threading.get_ident() == self.open_thread
        try:
            value = self.batches.get(timeout=.01)
        except queue.Empty:
            return []
        if isinstance(value, BaseException):
            raise value
        return value

    def close(self):
        self.close_thread = threading.get_ident()
        self.closed = True


class Fusion:
    def __init__(self, config):
        self.config, self.nodes, self.counts = config, {}, Counter()

    def update(self, samples):
        for sample in samples:
            node = str(sample.node_id)
            self.counts[node] += 1
            self.nodes[node] = dict(quat_xyzw=[0., 0., 0., 1.], epoch=0,
                                    sample_index=self.counts[node], stamp=sample.host_monotonic_s)

    def snapshot(self, now):
        return dict(timestamp_s=now, nodes={
            node: {key: value for key, value in {**data, "age_s": now - data["stamp"]}.items() if key != "stamp"}
            for node, data in self.nodes.items() if 0 <= now - data["stamp"] <= self.config["stale_after_s"]})


class Solver:
    def __init__(self, *_):
        self.calls, self.entered, self.release = [], threading.Event(), threading.Event()
        self.block = False

    def solve(self, observations, timestamp_s):
        self.calls.append((observations, timestamp_s))
        if self.block:
            self.entered.set()
            self.release.wait(3.)
        root = "12" in observations
        return dict(timestamp_s=timestamp_s, pose_available=root, solver_success=root,
                    quality="experimental_partial" if root else "unavailable",
                    anatomical_euler_rad=[[0., 0., 0.]] * 16 if root else None,
                    root_quat_xyzw=[0., 0., 0., 1.] if root else None,
                    source_labels=["mock_only"] * 16)


@pytest.mark.parametrize("sample_offset, expected_warning", [
    (0., None),
    (.01, "node_12_future_host_stamp"),
    (-.2, "node_12_stale"),
])
def test_tick_timestamps_locked_state_without_hiding_future_or_stale_samples(
        tmp_path, sample_offset, expected_warning):
    clock, config = Clock(), load_config()
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    controller = LiveController(config, rest, {}, output_root=tmp_path,
                                clock=clock, solver_factory=Solver)
    fusion = OnlineFusion(config)

    class ReaderCommitsBeforeLockAcquired:
        """Deterministic reader/worker handoff, without threads or devices."""
        def __enter__(self):
            clock.advance(.05)
            sample = RawSample(12, (0, 0, 0, 0, 0, 8192),
                               clock() + sample_offset, 1234.)
            fusion.update([sample])

        def __exit__(self, *_):
            pass

    session = _Session("LOCK_HANDOFF", tmp_path, "NO_DEVICE", object(), fusion,
                       object(), clock(), lock=ReaderCommitsBeforeLockAcquired())
    session.opened = session.metadata_written = True
    session.streams = {name: io.StringIO() for name in ("orientations", "poses", "events")}
    controller._session = session

    controller._tick(session)

    frame = controller.latest_snapshot()["orientation"]
    assert frame["timestamp_s"] == clock()
    if expected_warning is None:
        assert frame["nodes"]["12"]["age_s"] == 0.
        assert frame["nodes"]["12"]["sample_index"] == 1
        assert "node_12_future_host_stamp" not in frame["warnings"]
    else:
        assert "12" not in frame["nodes"]
        assert expected_warning in frame["warnings"]
    timing = controller.latest_snapshot()["runtime"]["timing_ms"]
    assert timing["total"] >= sum(timing[key] for key in ("solve", "calibration", "mesh"))


@pytest.fixture
def harness(tmp_path):
    clock, sources, fit_calls = Clock(), [], []
    config = load_config()
    config["input_health"] = {"enabled": False}  # FakeFusion tests do not model acquisition health.
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    solver = Solver()
    controls = SimpleNamespace(fail_fit=False, source_setup=None)

    def source_factory(port):
        source = Source(port)
        if controls.source_setup:
            controls.source_setup(source)
        sources.append(source)
        return source

    def fit(captures, rest, config, *, session_id, model_axes):
        fit_calls.append(captures)
        if controls.fail_fit:
            raise GuidedCalibrationError("第二步转动不足，请重采指尖下倾。", redo_steps=("fingers_down",))
        nodes, rejected = {}, {}
        for node, slot in config["node_to_slot"].items():
            counts = [sum(node in frame["nodes"] for frame in captures[step]) for step in STEP_KEYS]
            if min(counts) >= 3:
                nodes[node] = dict(slot=slot, epoch=0, world_alignment_xyzw=[0., 0., 0., 1.],
                                   sensor_from_bone_xyzw=[0., 0., 0., 1.])
            else:
                rejected[node] = "本节点的有效样本不足。"
        if "12" not in nodes:
            raise GuidedCalibrationError("手掌样本不足。", redo_steps=STEP_KEYS)
        return dict(status="provisional_not_accuracy_validated", session_id=session_id,
                    config_sha256=digest(config), rest_sha256=digest(rest),
                    stale_after_s=config["stale_after_s"], nodes=nodes, rejected_nodes=rejected,
                    warnings=["实验标定，不校正航向漂移。"])

    controller = LiveController(config, rest, {}, output_root=tmp_path, source_factory=source_factory,
                                clock=clock, fusion_factory=Fusion, solver_factory=lambda *_: solver,
                                calibration_fn=fit)
    state = SimpleNamespace(controller=controller, clock=clock, sources=sources, solver=solver,
                            fits=fit_calls, controls=controls, output=tmp_path)

    def connect(port="FAKE"):
        controller.start()
        assert controller.submit("connect", port=port)
        wait_for(lambda: bool(sources) and controller.latest_snapshot()["runtime"]["connected"])
        return sources[-1]

    def pump(nodes=(3, 12), count=1, delta=.05):
        before = controller.latest_snapshot()["runtime"]["received_samples"]
        batch = [RawSample(node, (0, 0, 0, 0, 0, 8192), clock(), 1234.)
                 for _ in range(count) for node in nodes]
        if batch:
            sources[-1].batches.put_nowait(batch)
            wait_for(lambda: controller.latest_snapshot()["runtime"]["received_samples"] >= before + len(batch))
        clock.advance(delta)
        wait_for(lambda: controller.latest_snapshot()["runtime"]["published_monotonic_s"] >= clock())

    def capture(step):
        assert controller.submit("capture", step=step)
        wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
        assert controller.latest_snapshot()["calibration"]["active"]["phase"] == "preparing"
        clock.advance(3.)
        pump()  # Enter collection and establish sample_index baseline.
        for _ in range(4):
            pump()
        clock.advance(2.)
        pump(nodes=())
        wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)

    state.connect, state.pump, state.capture = connect, pump, capture
    yield state
    solver.release.set()
    for source in sources:
        if source.open_gate:
            source.open_gate.set()
    controller.stop()
    assert controller.join(2.)


def test_default_idle_no_source_or_output_and_snapshot_deepcopy(harness):
    controller = harness.controller.start()
    snapshot = controller.latest_snapshot()
    assert not snapshot["runtime"]["connected"]
    snapshot["calibration"]["warnings"].append("mutated")
    assert not controller.latest_snapshot()["calibration"]["warnings"]
    assert not harness.sources and not list(harness.output.iterdir())


def test_input_quality_is_exported_without_stopping_acquisition(harness, monkeypatch):
    quality = {"gyro_near_rail": {"threshold_lsb": 32760, "recent_window_s": 2.,
        "active_nodes": ["3"], "nodes": {"3": {"near_rail_samples": 2}}}}
    original = Fusion.snapshot

    def snapshot(fusion, now):
        return dict(original(fusion, now), sensor_quality=deepcopy(quality),
                    warnings=["independent_initial_heading", "node_3_gyro_near_rail"])

    monkeypatch.setattr(Fusion, "snapshot", snapshot)
    harness.connect()
    harness.pump()
    result = harness.controller.latest_snapshot()
    assert result["runtime"]["connected"] and result["runtime"]["error"] is None
    assert result["pose"]["sensor_quality"] == result["orientation"]["sensor_quality"] == quality
    assert result["pose"]["input_warnings"] == ["node_3_gyro_near_rail"]
    result["pose"]["sensor_quality"]["gyro_near_rail"]["active_nodes"].clear()
    assert harness.controller.latest_snapshot()["pose"]["sensor_quality"] == quality
    directory = harness.output / result["runtime"]["session_id"]
    harness.controller.stop()
    assert harness.controller.join(2.)
    poses = [json.loads(line) for line in (directory / "poses.jsonl").read_text().splitlines()]
    assert poses and all(pose["sensor_quality"] == quality for pose in poses)
    assert any(pose["input_warnings"] == ["node_3_gyro_near_rail"] for pose in poses)


def test_official_mode_never_calls_legacy_fit_solver_or_retarget(harness, monkeypatch):
    from vt_fsglove12 import live_runtime
    h = harness
    calls = []

    def forbidden(*args, **kwargs):
        raise AssertionError("official path called a legacy algorithm")

    class Official:
        def fit_calibration(self, captures, *, session_id):
            assert set(captures) == {"base_pose", "z_rot_pose_1", "z_rot_pose_2"}
            calls.append("fit")
            return dict(session_id=session_id, nodes={"12": {"epoch": 0}},
                        status="official_core_sparse_adapter_not_accuracy_validated",
                        calibration_motion_diagnostics={"blocking": False, "warnings": ["示例转轴偏差"]},
                        config_sha256=h.controller._config_sha, rest_sha256=h.controller._rest_sha)

        def solve(self, frame, profile, *, timestamp_s):
            calls.append("solve")
            assert frame["session_id"] == profile["session_id"]
            return dict(timestamp_s=timestamp_s, pose_available=True, solver_success=True,
                        quality="not_accuracy_validated", mano_pose_rotvec=[[0., 0., 0.]] * 16,
                        shape_params=[.1] * 10, calibration_rejections={})

    class Geometry:
        render = staticmethod(forbidden)

        def render_mano(self, pose, shape):
            calls.append("render_mano")
            assert shape == [.1] * 10
            return {"official_mock_geometry": True}

    h.controller._pose_backend = Official()
    h.controller._geometry_backend = Geometry()
    h.controller._calibration_fn = forbidden
    monkeypatch.setattr(live_runtime, "apply_calibration", forbidden)
    monkeypatch.setattr(live_runtime, "apply_functional_axes", forbidden)
    h.controller._solver.solve = forbidden
    assert not h.controller.submit("reanchor_neutral")
    assert not h.controller.submit("capture_functional", finger="invalid")
    h.connect()
    for step in STEP_KEYS:
        h.capture(step)
    h.pump()
    assert "fit" in calls and "solve" in calls and "render_mano" in calls
    assert h.controller.latest_snapshot()["geometry"] == {"official_mock_geometry": True}
    snapshot = h.controller.latest_snapshot()
    assert snapshot["calibration"]["motion_diagnostics"]["warnings"] == ["示例转轴偏差"]
    assert snapshot["calibration"]["motion_diagnostics"]["blocking"] is False
    assert set(snapshot["calibration"]["captured_steps"]) == set(STEP_KEYS)
    assert snapshot["runtime"]["error"] is None
    assert "结果保留" in snapshot["runtime"]["message"]


def test_connect_records_all_samples_and_disconnect_writes_complete_summary(harness):
    source = harness.connect()
    harness.pump(count=20)
    snapshot = harness.controller.latest_snapshot()
    directory = harness.output / snapshot["runtime"]["session_id"]
    assert snapshot["runtime"]["received_samples"] == 40
    assert snapshot["pose"]["pose_available"] is False
    assert harness.controller.submit("disconnect")
    wait_for(lambda: (directory / "summary.json").exists())
    summary = json.loads((directory / "summary.json").read_text())
    assert summary["recording_complete"] and summary["received_samples"] == summary["recorded_samples"] == 40
    with (directory / "raw.csv").open() as stream:
        assert len(list(csv.DictReader(stream))) == 40
    assert source.closed and source.close_thread == source.open_thread
    assert not harness.controller.latest_snapshot()["pose"]["pose_available"]
    assert all((directory / name).exists() for name in ("orientations.jsonl", "poses.jsonl", "events.jsonl", "config.json", "rest.json"))


def test_three_capture_steps_partial_profile_and_exclusive_versions(harness):
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    snapshot = harness.controller.latest_snapshot()
    assert snapshot["calibration"]["captured_steps"] == list(STEP_KEYS)
    assert snapshot["calibration"]["calibrated_nodes"] == ["3", "12"]
    assert "4" in snapshot["calibration"]["rejected_nodes"]
    directory = harness.output / snapshot["runtime"]["session_id"]
    original = (directory / "profile.json").read_bytes()
    assert json.loads(original)["profile_version"] == 1
    harness.pump()
    pose = harness.controller.latest_snapshot()["pose"]
    assert pose["pose_available"] and pose["quality"] == "experimental_partial"
    assert pose["model_rest_sha256"] and pose["config_sha256"] and pose["profile_version"] == 1
    harness.capture("neutral")
    assert (directory / "profile.v002.json").exists()
    assert (directory / "profile.json").read_bytes() == original
    assert (directory / "profile.v001.json").read_bytes() == original


def test_fit_failure_retains_steps_for_selective_retry(harness):
    harness.controls.fail_fit = True
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    snapshot = harness.controller.latest_snapshot()
    assert snapshot["calibration"]["profile_status"] == "fit_failed"
    assert snapshot["calibration"]["captured_steps"] == list(STEP_KEYS)
    assert "指尖下倾" in snapshot["runtime"]["error"]
    assert "fingers_down" in snapshot["calibration"]["warnings"][0]
    assert not snapshot["pose"]["pose_available"]
    harness.controls.fail_fit = False
    harness.capture("fingers_down")
    assert harness.controller.latest_snapshot()["calibration"]["calibrated_nodes"] == ["3", "12"]


def test_manual_reference_keeps_existing_mounts_and_saves_new_profile(harness):
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    controller = harness.controller
    directory = harness.output / controller.latest_snapshot()["runtime"]["session_id"]
    original = (directory / "profile.v001.json").read_bytes()
    assert controller.submit("reanchor_neutral")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    assert controller.latest_snapshot()["calibration"]["active"]["kind"] == "neutral_reference"
    harness.clock.advance(3.)
    harness.pump()
    for _ in range(4):
        harness.pump()
    harness.clock.advance(2.)
    harness.pump(nodes=())
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    updated = json.loads((directory / "profile.v002.json").read_text())
    assert updated["manual_neutral_reference"]["palm_orientation_preserved"]
    assert set(updated["manual_neutral_reference"]["updated_nodes"]) == {"3"}
    assert updated["nodes"]["12"] == json.loads(original)["nodes"]["12"]
    assert (directory / "profile.v001.json").read_bytes() == original
    assert (directory / "profile.json").read_bytes() == original
    harness.pump()
    assert controller.latest_snapshot()["pose"]["profile_version"] == 2


def test_manual_reference_failure_and_cancel_keep_old_profile(harness):
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    controller = harness.controller
    assert controller.submit("reanchor_neutral")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    assert controller.submit("cancel_capture")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    harness.pump()
    assert controller.latest_snapshot()["pose"]["profile_version"] == 1
    assert controller.submit("reanchor_neutral")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    harness.clock.advance(3.)
    harness.pump(nodes=(3,))
    for _ in range(4):
        harness.pump(nodes=(3,))
    harness.clock.advance(2.)
    harness.pump(nodes=())
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    harness.pump()
    assert "原标定保留" in controller.latest_snapshot()["runtime"]["message"]
    assert controller.latest_snapshot()["pose"]["profile_version"] == 1


def test_functional_capture_has_separate_reference_and_motion_and_keeps_versions(harness, monkeypatch):
    from vt_fsglove12 import live_runtime
    calls = []

    def fit(neutral, motion, profile, rest, config, **kwargs):
        calls.append((neutral, motion, kwargs))
        return {"session_id": kwargs["session_id"], "slot_refs": {}, "test_only": True}

    monkeypatch.setattr(live_runtime, "fit_functional_finger", fit)
    monkeypatch.setattr(live_runtime, "apply_functional_axes", lambda obs, profile, rest:
                        (obs, {"applied": bool(profile.get("functional_fingers"))}))
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    controller = harness.controller
    assert not controller.submit("capture_functional", finger="invalid")
    assert controller.submit("capture_functional", finger="little")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    harness.clock.advance(3.)
    harness.pump()
    for _ in range(4):
        harness.pump()
    assert controller.latest_snapshot()["calibration"]["active"]["phase"] == "collecting_neutral"
    harness.clock.advance(2.)
    for _ in range(4):
        harness.pump()
    assert controller.latest_snapshot()["calibration"]["active"]["phase"] == "moving"
    harness.clock.advance(8.)
    harness.pump()
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    assert len(calls) == 1 and calls[0][0] and calls[0][1]
    assert calls[0][0][-1]["timestamp_s"] < calls[0][1][0]["timestamp_s"]
    assert calls[0][2]["finger"] == "little"
    snapshot = controller.latest_snapshot()
    assert snapshot["pose"]["profile_version"] == 2
    assert snapshot["pose"]["functional_axis_retargeting"]["applied"]
    assert snapshot["calibration"]["functional_fingers"] == ["little"]
    directory = harness.output / snapshot["runtime"]["session_id"]
    assert (directory / "profile.v001.json").exists()
    assert json.loads((directory / "profile.v002.json").read_text())["functional_fingers"]["little"]["test_only"]


def _official_functional_backend(harness, monkeypatch):
    """Fake the official numerical backend, retaining real worker/recording flow."""
    from vt_fsglove12 import live_runtime
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    calls, control = [], SimpleNamespace(fail=False)

    def forbidden(*args, **kwargs):
        raise AssertionError("official functional capture must not call the legacy fit")

    class Official:
        def fit_functional_finger(self, neutral, motion, profile, *, finger, session_id):
            calls.append((neutral, motion, finger, session_id))
            assert neutral and motion and neutral[-1]["timestamp_s"] < motion[0]["timestamp_s"]
            if control.fail:
                profile["nodes"].clear()  # Failed backend work cannot mutate the live profile.
                raise ValueError("mock insufficient movement; previous profile must remain")
            return dict(kind="qmt_functional_axis_adapter_v1", finger=finger, slot_refs={},
                        diagnostics={"15" if finger == "thumb" else "1": {
                            "status": "fitted", "holdout_p95_residual_deg": 17., "warnings": ["mock soft warning"]}},
                        warnings=["mock soft warning"])

        def solve(self, frame, profile, *, timestamp_s):
            pose = harness.solver.solve(frame["nodes"], timestamp_s)
            pose.update(mano_pose_rotvec=[[0., 0., 0.]] * 16,
                        algorithm="official_fsglove_with_qmt_functional_adapter_v1",
                        pose_postprocessing="qmt_functional_axis_and_joint_constraint",
                        functional_adapter={"enabled": profile.get("functional_enabled", True)})
            return pose

    harness.controller._pose_backend = Official()
    monkeypatch.setattr(live_runtime, "fit_functional_finger", forbidden)
    monkeypatch.setattr(live_runtime, "apply_functional_axes", forbidden)
    return calls, control


def _capture_functional(harness, finger):
    controller = harness.controller
    assert controller.submit("capture_functional", finger=finger)
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    assert controller.latest_snapshot()["calibration"]["active"]["phase"] == "preparing"
    harness.clock.advance(3.)
    for _ in range(4):
        harness.pump()
    assert controller.latest_snapshot()["calibration"]["active"]["phase"] == "collecting_neutral"
    harness.clock.advance(2.)
    for _ in range(4):
        harness.pump()
    assert controller.latest_snapshot()["calibration"]["active"]["phase"] == "moving"
    harness.clock.advance(8.)
    harness.pump()
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)


def test_official_all_five_functional_captures_keep_base_and_other_fingers(harness, monkeypatch):
    calls, _ = _official_functional_backend(harness, monkeypatch)
    controller = harness.controller
    base_nodes = deepcopy(controller._profile["nodes"])
    before_raw = controller.latest_snapshot()["runtime"]["received_samples"]
    for finger in ("index", "middle", "ring", "little", "thumb"):
        _capture_functional(harness, finger)
        assert controller._profile["nodes"] == base_nodes
    snapshot = controller.latest_snapshot()
    assert [call[2] for call in calls] == ["index", "middle", "ring", "little", "thumb"]
    assert snapshot["calibration"]["functional_fingers"] == ["index", "little", "middle", "ring", "thumb"]
    assert snapshot["calibration"]["functional_diagnostics"]["thumb"]["15"]["status"] == "fitted"
    assert "IP" in snapshot["runtime"]["message"] and "MCP / CMC" in snapshot["runtime"]["message"]
    assert "mock soft warning" in snapshot["calibration"]["warnings"]
    assert snapshot["runtime"]["connected"] and snapshot["runtime"]["received_samples"] > before_raw
    assert controller._profile["profile_version"] == 6 and controller._profile["functional_enabled"] is True
    directory = harness.output / snapshot["runtime"]["session_id"]
    assert len(list(directory.glob("profile.v*.json"))) == 6
    assert not controller.submit("reanchor_neutral")


def test_official_failed_and_cancelled_finger_keep_profile_without_writing(harness, monkeypatch):
    calls, control = _official_functional_backend(harness, monkeypatch)
    _capture_functional(harness, "index")
    controller = harness.controller
    previous = deepcopy(controller._profile)
    control.fail = True
    _capture_functional(harness, "little")
    assert controller._profile == previous
    assert controller.latest_snapshot()["runtime"]["connected"]
    assert "mock insufficient" in controller.latest_snapshot()["runtime"]["error"]
    assert controller.submit("capture_functional", finger="thumb")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    assert controller.submit("cancel_capture")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    assert controller._profile == previous and len(calls) == 2
    directory = harness.output / previous["session_id"]
    assert len(list(directory.glob("profile.v*.json"))) == 2


def test_official_switch_is_boolean_versioned_and_never_changes_base(harness, monkeypatch):
    _official_functional_backend(harness, monkeypatch)
    _capture_functional(harness, "index")
    controller = harness.controller
    previous = deepcopy(controller._profile)
    for bad in (1, "false", None):
        assert not controller.submit("set_functional_enabled", enabled=bad)
    assert controller.submit("set_functional_enabled", enabled=False)
    wait_for(lambda: controller.latest_snapshot()["calibration"]["functional_enabled"] is False)
    expected = {**previous, "profile_version": 3, "functional_enabled": False}
    assert controller._profile == expected
    assert controller.submit("set_functional_enabled", enabled=False)  # No-op must not manufacture another version.
    harness.pump()
    assert controller._profile == expected
    assert controller.submit("capture_functional", finger="ring")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    assert controller.submit("set_functional_enabled", enabled=True)
    wait_for(lambda: bool(controller.latest_snapshot()["runtime"]["error"]))
    assert controller._profile == expected
    assert controller.submit("cancel_capture")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    assert controller.submit("set_functional_enabled", enabled=True)
    wait_for(lambda: controller.latest_snapshot()["calibration"]["functional_enabled"] is True)
    assert controller._profile == {**previous, "profile_version": 4}


def test_official_postprocessed_geometry_has_matching_provenance(harness, monkeypatch):
    _official_functional_backend(harness, monkeypatch)
    geometry = {"provenance": {"pose_input": "upstream_mano_axis_angle_direct"}, "vertices_m": []}
    harness.controller._geometry_backend = SimpleNamespace(render_mano=lambda *args: geometry)
    harness.pump()
    result = harness.controller.latest_snapshot()["geometry"]
    assert result["provenance"]["pose_input"] == "upstream_axis_layer_composed_functional_constraint"
    assert result["provenance"]["pose_postprocessing"] == "qmt_functional_axis_and_joint_constraint"
    assert result["provenance"]["pose_algorithm"] == "official_fsglove_with_qmt_functional_adapter_v1"
    assert geometry["provenance"]["pose_input"] == "upstream_mano_axis_angle_direct"


def _hand_backend(harness, monkeypatch):
    _official_functional_backend(harness, monkeypatch)
    calls = []
    control = SimpleNamespace(errors={}, rejected={}, fail=False, block=False,
                              entered=threading.Event(), release=threading.Event())

    def fit(neutral, fingers, thumb, profile, *, session_id):
        calls.append((deepcopy(neutral), deepcopy(fingers), deepcopy(thumb), session_id))
        control.entered.set()
        if control.block:
            control.release.wait(2.)
        if control.fail:
            profile["nodes"].clear()
            raise ValueError("mock whole-hand failure")
        entries = {finger: dict(kind="qmt_functional_axis_adapter_v1", finger=finger,
                                slot_refs={"1": {"test_only": True}, "2": {"test_only": True}},
                                diagnostics={}, rejected_slots=control.rejected.get(finger, {}))
                   for finger in ("index", "middle", "ring", "little", "thumb") if finger not in control.errors}
        return dict(entries=entries, errors=deepcopy(control.errors))

    harness.controller._pose_backend.fit_functional_hand = fit
    return calls, control


def _start_hand_capture(harness):
    start = harness.clock()
    assert harness.controller.submit("capture_functional_hand")
    wait_for(lambda: harness.controller.latest_snapshot()["calibration"]["active"] is not None)
    return start


def _hand_capture_phases(harness, *, finish=True):
    start = _start_hand_capture(harness)
    for offset, phase, duration in ((.01, "preparing", 3.), (3.01, "collecting_neutral", 2.),
                                     (5.01, "moving_fingers", 8.), (13.01, "preparing_thumb", 3.),
                                     (16.01, "moving_thumb", 8.)):
        harness.clock.value = start + offset
        for _ in range(4):
            harness.pump()
        active = harness.controller.latest_snapshot()["calibration"]["active"]
        assert active["kind"] == "functional_hand" and active["step"] == "neutral"
        assert active["phase"] == phase and 0 < active["remaining_s"] <= duration
    if finish:
        harness.clock.value = start + 24.01
        harness.pump()
        wait_for(lambda: harness.controller.latest_snapshot()["calibration"]["active"] is None)
    return start


def test_hand_capture_only_official_after_basic_profile(harness):
    assert not harness.controller.submit("capture_functional_hand")
    harness.controller._pose_backend = object()
    assert not harness.controller.submit("capture_functional_hand", finger="index")
    harness.connect()
    assert harness.controller.submit("capture_functional_hand")
    wait_for(lambda: bool(harness.controller.latest_snapshot()["runtime"]["error"]))
    assert "三姿势" in harness.controller.latest_snapshot()["runtime"]["error"]
    assert harness.controller.latest_snapshot()["calibration"]["active"] is None


def test_whole_hand_split_capture_single_save_and_preserved_enabled(harness, monkeypatch):
    calls, _ = _hand_backend(harness, monkeypatch)
    controller = harness.controller
    controller._profile["functional_enabled"] = False
    old_nodes = deepcopy(controller._profile["nodes"])
    start = _hand_capture_phases(harness)
    assert len(calls) == 1
    for rows, lo, hi in zip(calls[0][:3], (3., 5., 16.), (5., 13., 24.)):
        assert rows and all(start + lo <= row["timestamp_s"] < start + hi for row in rows)
        assert len({row["nodes"]["12"]["sample_index"] for row in rows}) == len(rows)
    result = controller.latest_snapshot()["calibration"]["functional_hand_result"]
    assert result["status"] == "complete" and len(result["updated_fingers"]) == 5
    assert controller._profile["functional_enabled"] is False and controller._profile["nodes"] == old_nodes
    assert controller._profile["profile_version"] == 2
    directory = harness.output / controller._profile["session_id"]
    assert len(list(directory.glob("profile.v*.json"))) == 2
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
    assert sum(event["event"] == "functional_hand_updated" for event in events) == 1
    assert controller.latest_snapshot()["runtime"]["connected"]


def test_hand_partial_and_total_failure_preserve_original_parameters(harness, monkeypatch):
    _, control = _hand_backend(harness, monkeypatch)
    controller = harness.controller
    old_index = {"old_test_only": True}
    controller._profile["functional_fingers"] = {"index": deepcopy(old_index)}
    control.errors = {"index": "mock missing index motion"}
    control.rejected = {"little": {"2": "mock PIP kept previous"}}
    _hand_capture_phases(harness)
    result = controller.latest_snapshot()["calibration"]["functional_hand_result"]
    assert result["status"] == "partial" and len(result["updated_fingers"]) == 4
    assert result["fingers"]["index"]["status"] == "failed"
    assert result["fingers"]["little"]["status"] == "partial"
    assert result["fingers"]["little"]["updated_slots"] == ["1"]
    assert result["partial_slots"] == control.rejected
    assert controller._profile["functional_fingers"]["index"] == old_index
    previous = deepcopy(controller._profile)
    control.fail = True
    _hand_capture_phases(harness)
    assert controller._profile == previous
    result = controller.latest_snapshot()["calibration"]["functional_hand_result"]
    assert result["status"] == "failed" and not result["updated_fingers"] and len(result["errors"]) == 5
    assert controller.latest_snapshot()["runtime"]["connected"]


def test_resampling_basic_pose_clears_previous_whole_hand_result(harness, monkeypatch):
    _hand_backend(harness, monkeypatch)
    _hand_capture_phases(harness)
    controller = harness.controller
    assert controller.latest_snapshot()["calibration"]["functional_hand_result"]["status"] == "complete"
    assert controller.submit("capture", step="neutral")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is not None)
    assert controller.latest_snapshot()["calibration"]["functional_hand_result"] is None
    assert controller._profile is None


@pytest.mark.parametrize("offset", [.1, 3.1, 5.1, 13.1, 16.1])
def test_cancel_in_each_whole_hand_phase_never_fits_or_saves(harness, monkeypatch, offset):
    calls, _ = _hand_backend(harness, monkeypatch)
    controller = harness.controller
    previous = deepcopy(controller._profile)
    start = _start_hand_capture(harness)
    harness.clock.value = start + offset
    harness.pump()
    assert controller.submit("cancel_capture")
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    assert controller._profile == previous and not calls
    assert controller.latest_snapshot()["calibration"]["functional_hand_result"]["status"] == "cancelled"


def test_cancel_during_whole_hand_fit_keeps_reader_running_and_does_not_save(harness, monkeypatch):
    calls, control = _hand_backend(harness, monkeypatch)
    controller = harness.controller
    previous = deepcopy(controller._profile)
    start = _hand_capture_phases(harness, finish=False)
    control.block = True
    harness.clock.value = start + 24.01
    controller._wake.set()
    assert control.entered.wait(1.)
    assert controller.latest_snapshot()["calibration"]["profile_status"] == "fitting_functional_hand"
    assert controller.submit("cancel_capture")
    assert controller._functional_hand_cancel.is_set()
    before = controller.latest_snapshot()["runtime"]["received_samples"]
    harness.sources[-1].batches.put([RawSample(12, (0, 0, 0, 0, 0, 8192), harness.clock(), 1234.)])
    wait_for(lambda: controller.latest_snapshot()["runtime"]["received_samples"] > before)
    control.release.set()
    wait_for(lambda: controller.latest_snapshot()["calibration"]["active"] is None)
    assert calls and controller._profile == previous
    assert controller.latest_snapshot()["calibration"]["functional_hand_result"]["status"] == "cancelled"
    assert len(list((harness.output / previous["session_id"]).glob("profile.v*.json"))) == 1


def test_hand_phase_buffers_are_bounded_and_do_not_repeat_polled_nodes(harness):
    controller = harness.controller  # Unstarted: pure staged capture, no device/thread.
    active = dict(kind="functional_hand", collect_at=3., neutral_end_at=5., fingers_end_at=13.,
                  thumb_start_at=16., end_at=24., cancel=threading.Event(), capture_phase=None,
                  rows=[], finger_rows=[], thumb_rows=[], seen=None)
    controller._active = active
    index = 0
    for start, end, key, cap in ((3., 5., "rows", 120), (5., 13., "finger_rows", 300),
                                (13., 16., None, 0), (16., 24., "thumb_rows", 300)):
        for tick in range(1000):
            now = start + (end - start) * tick / 1000
            index += 1
            frame = {"timestamp_s": now, "nodes": {"12": {"epoch": 0, "sample_index": index}}}
            controller._capture_functional_hand(None, frame, now)
            controller._capture_functional_hand(None, frame, now)  # Same snapshot is not another sample.
        if key:
            assert len(active[key]) == cap
            assert len({row["nodes"]["12"]["sample_index"] for row in active[key]}) == cap
    assert all(not 13. <= row["timestamp_s"] < 16.
               for key in ("rows", "finger_rows", "thumb_rows") for row in active[key])


def test_frozen_samples_do_not_become_capture_rows(harness):
    harness.connect()
    assert harness.controller.submit("capture", step="neutral")
    wait_for(lambda: harness.controller.latest_snapshot()["calibration"]["active"] is not None)
    harness.clock.advance(3.)
    harness.pump()  # Exactly one fresh state, then snapshots only.
    for _ in range(3):
        harness.pump(nodes=(), delta=.04)
    harness.clock.advance(2.)
    harness.pump(nodes=())
    snapshot = harness.controller.latest_snapshot()
    assert snapshot["calibration"]["profile_status"] == "capture_failed"
    assert snapshot["calibration"]["captured_steps"] == []
    assert not harness.fits


def test_cancel_and_clear_do_not_automatically_recalibrate(harness):
    harness.connect()
    harness.capture("neutral")
    assert harness.controller.submit("capture", step="thumb_up")
    wait_for(lambda: harness.controller.latest_snapshot()["calibration"]["active"] is not None)
    assert harness.controller.submit("cancel_capture")
    wait_for(lambda: harness.controller.latest_snapshot()["calibration"]["active"] is None)
    assert harness.controller.latest_snapshot()["calibration"]["captured_steps"] == ["neutral"]
    assert harness.controller.submit("clear_calibration")
    wait_for(lambda: not harness.controller.latest_snapshot()["calibration"]["captured_steps"])
    assert not harness.controller.latest_snapshot()["pose"]["pose_available"]
    assert not harness.fits


def test_disconnect_and_reconnect_never_reuses_profile(harness):
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    old = harness.controller.latest_snapshot()["runtime"]["session_id"]
    assert harness.controller.submit("disconnect")
    wait_for(lambda: (harness.output / old / "summary.json").exists())
    harness.connect("FAKE-2")
    harness.pump()
    snapshot = harness.controller.latest_snapshot()
    assert snapshot["runtime"]["session_id"] != old
    assert snapshot["calibration"]["captured_steps"] == []
    assert snapshot["calibration"]["calibrated_nodes"] == []
    assert not snapshot["pose"]["pose_available"]


def test_latest_snapshot_hides_stale_root_independently_of_worker(harness):
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    harness.pump()
    assert harness.controller.latest_snapshot()["pose"]["pose_available"]
    harness.clock.advance(1.)
    snapshot = harness.controller.latest_snapshot()
    assert not snapshot["pose"]["pose_available"]
    assert snapshot["pose"]["reason"] == "missing_or_stale_root"
    assert "12" not in snapshot["orientation"]["nodes"]
    harness.pump(nodes=(3,))
    assert not harness.controller.latest_snapshot()["pose"]["pose_available"]
    harness.pump()
    assert harness.controller.latest_snapshot()["pose"]["pose_available"]


def test_slow_solver_does_not_block_reader_or_create_pose_backlog(harness):
    harness.connect()
    for step in STEP_KEYS:
        harness.capture(step)
    harness.solver.block = True
    source = harness.sources[-1]
    before = harness.controller.latest_snapshot()["runtime"]["received_samples"]
    source.batches.put([RawSample(12, (0, 0, 0, 0, 0, 8192), harness.clock(), 1234.)])
    wait_for(lambda: harness.controller.latest_snapshot()["runtime"]["received_samples"] == before + 1)
    harness.clock.advance(.05)
    assert harness.solver.entered.wait(1.)
    calls = len(harness.solver.calls)
    batch = [RawSample(12, (0, 0, 0, 0, 0, 8192), harness.clock(), 1234.) for _ in range(500)]
    source.batches.put(batch)
    wait_for(lambda: harness.controller.latest_snapshot()["runtime"]["received_samples"] == before + 501)
    assert len(harness.solver.calls) == calls
    harness.clock.advance(1.)
    assert not harness.controller.latest_snapshot()["pose"]["pose_available"]
    harness.solver.release.set()
    wait_for(lambda: harness.controller.latest_snapshot()["runtime"]["published_monotonic_s"] >= harness.clock())
    assert not harness.controller.latest_snapshot()["pose"]["pose_available"]
    harness.solver.block = False
    harness.pump(nodes=(12,))
    assert len(harness.solver.calls) <= calls + 2
    assert harness.controller.latest_snapshot()["orientation"]["nodes"]["12"]["sample_index"] > 500


def test_independent_input_snapshot_remains_fresh_while_solver_is_blocked(harness):
    h = harness
    h.connect()
    for step in STEP_KEYS:
        h.capture(step)
    h.solver.block = True
    source = h.sources[-1]
    source.batches.put([RawSample(12, (0, 0, 0, 0, 0, 8192), h.clock(), 1234.)])
    h.clock.advance(.05)
    assert h.solver.entered.wait(1.)
    old_revision = h.controller.snapshot_revision()
    h.clock.advance(.5)
    source.batches.put([RawSample(12, (123, 0, 0, 0, 0, 8192), h.clock(), 1234.)])
    def fresh_input():
        value = h.controller.latest_input_snapshot()
        return value["orientation"] and "12" in value["orientation"]["nodes"]
    wait_for(fresh_input)
    value = h.controller.latest_input_snapshot()
    assert set(value) == {"runtime", "orientation"}
    assert value["runtime"]["connected"]
    assert value["runtime"]["last_sample_age_s"] == pytest.approx(0.)
    assert value["orientation"]["timestamp_s"] == h.clock()
    assert h.controller.snapshot_revision() == old_revision
    measurement = h.controller.latest_snapshot()
    assert not measurement["pose"]["pose_available"]
    assert "12" not in measurement["orientation"]["nodes"]
    assert "12" in measurement["input_snapshot"]["orientation"]["nodes"]
    value["orientation"]["nodes"].clear()
    assert h.controller.latest_input_snapshot()["orientation"]["nodes"]


def test_input_monitor_uses_raw_source_time_and_retains_connection_during_silence(harness):
    h = harness
    h.connect()
    before = h.controller.latest_input_snapshot()["runtime"]["received_samples"]
    # An old receive timestamp is not refreshed when fusion/recording finishes.
    h.sources[-1].batches.put([RawSample(12, (0, 0, 0, 0, 0, 8192), h.clock() - .1, 1234.)])
    wait_for(lambda: h.controller.latest_input_snapshot()["runtime"]["received_samples"] > before)
    assert h.controller.latest_input_snapshot()["runtime"]["last_sample_age_s"] == pytest.approx(.1)
    h.clock.advance(.5)
    quiet = h.controller.latest_input_snapshot()
    assert quiet["runtime"]["connected"]
    assert quiet["runtime"]["last_sample_age_s"] == pytest.approx(.6)
    assert "12" not in quiet["orientation"]["nodes"]
    assert h.controller.submit("disconnect")
    assert not h.controller.latest_input_snapshot()["runtime"]["connected"]


def test_input_monitor_lock_contention_returns_bounded_cached_state(harness):
    h = harness
    h.connect()
    h.pump()
    initial = h.controller.latest_input_snapshot()
    session = h.controller._session
    session.lock.acquire()
    try:
        h.clock.advance(.02)
        started = time.perf_counter()
        current = h.controller.latest_input_snapshot()
        assert time.perf_counter() - started < .05
        assert current["orientation"]["nodes"]["12"]["sample_index"] == initial["orientation"]["nodes"]["12"]["sample_index"]
        assert current["orientation"]["nodes"]["12"]["age_s"] == pytest.approx(
            initial["orientation"]["nodes"]["12"]["age_s"] + .02)
    finally:
        session.lock.release()


def test_occupied_port_failure_is_visible_without_retries(harness):
    harness.controls.source_setup = lambda source: setattr(source, "open_error", RuntimeError("端口已被其他进程占用"))
    harness.controller.start()
    assert harness.controller.submit("connect", port="BUSY")
    wait_for(lambda: bool(list(harness.output.glob("*/summary.json"))))
    snapshot = harness.controller.latest_snapshot()
    assert not snapshot["runtime"]["connected"]
    assert "占用" in snapshot["runtime"]["error"]
    assert len(harness.sources) == 1 and harness.sources[0].closed
    summary = json.loads(next(harness.output.glob("*/summary.json")).read_text())
    assert summary["input_kind"] == "raw_serial" and not summary["hardware_accessed"]


@pytest.mark.parametrize("exception,complete,reason", [
    (EOFError(), True, "source_eof"), (OSError("拔出设备"), False, "reader_error")])
def test_source_ends_or_fails_without_stale_pose_or_reconnection(harness, exception, complete, reason):
    source = harness.connect()
    harness.pump()
    directory = harness.output / harness.controller.latest_snapshot()["runtime"]["session_id"]
    source.batches.put(exception)
    wait_for(lambda: (directory / "summary.json").exists())
    summary = json.loads((directory / "summary.json").read_text())
    assert summary["recording_complete"] is complete and summary["stop_reason"] == reason
    assert not harness.controller.latest_snapshot()["pose"]["pose_available"]
    assert len(harness.sources) == 1


def test_raw_recording_failure_cannot_announce_success(harness):
    class FailingRecorder(RawRecorder):
        def write(self, _samples):
            raise OSError("磁盘已满")

    harness.controller._recorder_factory = FailingRecorder
    source = harness.connect()
    directory = harness.output / harness.controller.latest_snapshot()["runtime"]["session_id"]
    source.batches.put([RawSample(12, (0, 0, 0, 0, 0, 8192), harness.clock(), 1234.)])
    wait_for(lambda: harness.controller._session.recording_disabled.is_set())
    assert harness.controller.latest_snapshot()["runtime"]["connected"]
    assert "磁盘已满" in harness.controller.latest_snapshot()["runtime"]["local_recording"]["warning"]
    assert harness.controller.submit("disconnect")
    wait_for(lambda: (directory / "summary.json").exists())
    summary = json.loads((directory / "summary.json").read_text())
    assert not summary["recording_complete"] and summary["recorded_samples"] == 0
    assert "磁盘已满" in harness.controller.latest_snapshot()["runtime"]["error"]


def test_command_queue_is_bounded_and_stop_does_not_wait_for_it(harness):
    controller = harness.controller
    assert not controller.submit("capture", step="made_up")
    assert not controller.submit("connect", port="")
    assert not controller.submit("disconnect", port="unwanted")
    for _ in range(COMMAND_CAPACITY):
        assert controller.submit("disconnect")
    assert not controller.submit("disconnect")
    controller.stop()
    assert controller.join(.01)
    assert not controller.submit("connect", port="FAKE")


def test_bounded_join_does_not_close_a_source_from_another_thread(harness):
    gate = threading.Event()
    harness.controls.source_setup = lambda source: setattr(source, "open_gate", gate)
    harness.controller.start()
    assert harness.controller.submit("connect", port="SLOW-OPEN")
    wait_for(lambda: harness.sources and harness.sources[0].open_thread is not None)
    harness.controller.stop()
    start = time.monotonic()
    assert not harness.controller.join(.03)
    assert time.monotonic() - start < .2
    assert not harness.sources[0].closed
    assert not harness.controller.latest_snapshot()["pose"]["pose_available"]
    gate.set()
    assert harness.controller.join(2.)
    assert harness.sources[0].close_thread == harness.sources[0].open_thread


def test_processing_period_includes_computation_time_without_backlog(harness):
    controller, clock = harness.controller, harness.clock
    original = controller._tick
    starts = []

    class ObservedWake:
        def __init__(self):
            self.event = threading.Event()
            self.waited_after_ticks = 0

        def set(self):
            self.event.set()

        def clear(self):
            self.event.clear()

        def wait(self, timeout):
            # The main thread must not advance fake time while _tick or the
            # next-deadline update is still running. Observe the loop boundary.
            self.waited_after_ticks = len(starts)
            return self.event.wait(timeout)

    wake = controller._wake = ObservedWake()

    def timed_tick(session):
        starts.append(clock())
        original(session)
        clock.advance(.022)  # Simulate 22 ms of processing, without hardware.

    controller._tick = timed_tick
    harness.connect()
    wait_for(lambda: wake.waited_after_ticks == 1)
    clock.value = starts[0] + 1. / 30. + .001
    wake.set()
    wait_for(lambda: wake.waited_after_ticks == 2)
    assert starts[1] - starts[0] < .04  # Not 22 ms + another complete 33 ms.
    clock.advance(3.)
    wake.set()
    wait_for(lambda: wake.waited_after_ticks == 3)
    time.sleep(.025)
    assert len(starts) == 3  # No catch-up processing of missed historical ticks.


def test_jsonl_is_flushed_periodically_before_disconnect(harness):
    harness.connect()
    harness.pump()
    directory = harness.output / harness.controller.latest_snapshot()["runtime"]["session_id"]
    initial = len((directory / "orientations.jsonl").read_text().splitlines())
    harness.pump()
    harness.clock.advance(1.)
    harness.pump()
    wait_for(lambda: len((directory / "orientations.jsonl").read_text().splitlines()) > initial)
    wait_for(lambda: bool((directory / "poses.jsonl").read_text().strip()))


def test_csv_source_provenance_is_retained_without_claiming_hardware(harness):
    provenance = dict(source_raw_path="/fixture/old_raw.csv", replay_speed=1.5,
                      host_timestamps_rebased=True, hardware_accessed=False)

    def setup(source):
        source.input_kind, source.provenance = "csv_replay", provenance

    harness.controls.source_setup = setup
    harness.connect("/fixture/old_raw.csv")
    harness.pump()
    snapshot = harness.controller.latest_snapshot()
    directory = harness.output / snapshot["runtime"]["session_id"]
    assert snapshot["orientation"]["input_kind"] == snapshot["pose"]["input_kind"] == "csv_replay"
    assert not snapshot["orientation"]["hardware_accessed"]
    metadata = json.loads((directory / "session_metadata.json").read_text())
    assert metadata["source_provenance"] == provenance and not metadata["hardware_accessed"]
    harness.controller.submit("disconnect")
    wait_for(lambda: (directory / "summary.json").exists())
    summary = json.loads((directory / "summary.json").read_text())
    assert summary["input_kind"] == "csv_replay" and not summary["hardware_accessed"]
    assert summary["source_provenance"] == provenance
    for filename in ("orientations.jsonl", "poses.jsonl", "events.jsonl"):
        records = [json.loads(line) for line in (directory / filename).read_text().splitlines()]
        assert records and all(row["input_kind"] == "csv_replay" and not row["hardware_accessed"] for row in records)


def test_input_snapshot_clear_after_fusion_read_discards_old_epoch_without_relabelling(tmp_path):
    config, clock = load_config(), Clock()
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    controller = LiveController(config, rest, {}, output_root=tmp_path, clock=clock, solver_factory=Solver)
    fusion = OnlineFusion(config)
    fusion.update([RawSample(12, (0, 0, 0, 0, 0, 8192), clock(), 1234.)])

    class ClearsDuringDiagnostics:
        cleared = False

        def snapshot_diagnostics(self, now=None):
            if not self.cleared:
                self.cleared = True
                controller._clear_calibration()
            return dict(source_kind="raw_serial", samples_decoded=1)

    session = _Session("clear-race", tmp_path, "NO_DEVICE", ClearsDuringDiagnostics(), fusion, object(), clock())
    session.opened = True
    controller._session = session
    published = controller.latest_input_snapshot()
    assert controller._clear_generation == published["runtime"]["clear_generation"] == 1
    assert published["orientation"] is None
    assert published["runtime"]["input_reset_race_discarded"]
    assert controller._input_snapshot["orientation"] is None
    assert fusion.input_snapshot(clock())["nodes"] == {}
    assert not list(tmp_path.iterdir())  # No source/thread/recording was opened.


def test_clear_generation_is_advanced_before_releasing_fusion_reset_lock(tmp_path):
    config, clock = load_config(), Clock()
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    controller = LiveController(config, rest, {}, output_root=tmp_path, clock=clock, solver_factory=Solver)
    observed_generations = []

    class ResetBoundary:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            observed_generations.append(controller._clear_generation)

    fusion = OnlineFusion(config)
    session = _Session("reset-boundary", tmp_path, "NO_DEVICE", object(), fusion, object(), clock(), lock=ResetBoundary())
    controller._session = session
    controller._clear_calibration()
    assert observed_generations == [1]
    assert controller._input_snapshot["runtime"]["clear_generation"] == 1
