"""Clear/recalibrate within one reader session; no real device or GUI."""
from copy import deepcopy
import io
import json
from types import SimpleNamespace

from test_live_runtime import harness, Clock, Solver, wait_for
from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.live_runtime import LiveController, STEP_KEYS, _Session
from vt_fsglove12.live_stream import OnlineFusion, RawSample


def test_clear_completed_profile_and_recalibrate_without_closing_reader(harness):
    h = harness
    source = h.connect()
    for step in STEP_KEYS:
        h.capture(step)
    original = h.controller.latest_snapshot()
    session = h.controller._session
    directory = h.output / session.session_id
    first_file = directory / "profile.v001.json"
    first_bytes = first_file.read_bytes()
    h.controller._profile["functional_fingers"] = {"little": {"diagnostics": {}}}
    h.controller._functional_hand_result = {"status": "complete"}
    received = session.received
    assert h.controller.submit("clear_calibration")
    wait_for(lambda: h.controller.latest_snapshot()["calibration"]["clear_generation"] == 1)
    cleared = h.controller.latest_snapshot()
    assert cleared["runtime"]["connected"] and not source.closed
    assert cleared["runtime"]["session_id"] == original["runtime"]["session_id"]
    assert cleared["calibration"]["captured_steps"] == []
    assert cleared["calibration"]["functional_fingers"] == []
    assert cleared["calibration"]["functional_hand_result"] is None
    assert cleared["calibration"]["calibrated_nodes"] == []
    assert not cleared["pose"]["pose_available"] and cleared["geometry"] is None
    assert first_file.read_bytes() == first_bytes
    h.pump()
    assert session.received > received
    for step in STEP_KEYS:
        h.capture(step)
    h.pump()
    actual = h.controller.latest_snapshot()
    assert actual["runtime"]["connected"] and not source.closed
    assert actual["pose"]["pose_available"]
    assert actual["calibration"]["captured_steps"] == list(STEP_KEYS)
    assert h.controller._profile["profile_version"] == 2
    assert len(h.sources) == 1 and h.controller._session is session
    assert first_file.read_bytes() == first_bytes
    assert (directory / "profile.v002.json").is_file()


def test_clear_reinitializes_real_fusion_epochs_and_backend_but_not_raw_history(tmp_path):
    clock, config = Clock(), load_config()
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    resets = []
    backend = SimpleNamespace(reset_calibration=lambda: resets.append(True))
    controller = LiveController(config, rest, {}, output_root=tmp_path, clock=clock, pose_backend=backend)
    fusion = OnlineFusion(config)
    fusion.update([RawSample(node, (10, 20, 30, 0, 0, 8192), clock(), 1234.) for node in (9, 10, 12)])
    old_nodes = fusion.snapshot(clock())["nodes"]
    session = _Session("SAME_SESSION", tmp_path, "NO_DEVICE", object(), fusion, object(), clock())
    session.opened = session.metadata_written = True
    session.streams = {name: io.StringIO() for name in ("orientations", "poses", "events")}
    session.received = session.recorded = 3
    session.profiles = ["profile.v001.json"]
    controller._session = session
    controller._profile = dict(profile_version=1)
    controller._captures = {"neutral": ["old"]}
    controller._snapshot["runtime"]["connected"] = True
    before_config = deepcopy(config)
    controller._command("clear_calibration", {})
    assert resets == [True]
    assert fusion.snapshot(clock())["nodes"] == {}
    assert controller._profile is None and not controller._captures
    assert controller._session is session and session.fusion is fusion
    assert not session.stop.is_set() and not session.done.is_set()
    assert session.received == session.recorded == 3 and len(session.profiles) == 1
    clock.advance(.01)
    fusion.update([RawSample(node, (0, 0, 0, 0, 0, 8192), clock(), 1234.) for node in (9, 10, 12)])
    new_nodes = fusion.snapshot(clock())["nodes"]
    for node in old_nodes:
        assert new_nodes[node]["epoch"] == old_nodes[node]["epoch"] + 1
        assert new_nodes[node]["sample_index"] == old_nodes[node]["sample_index"] + 1
    assert config == before_config
    event = json.loads(session.streams["events"].getvalue())
    assert event["event"] == "clear_calibration" and event["clear_generation"] == 1
    assert event["historical_profiles_preserved"] and event["fusion_reset"]["reset_nodes"] == ["9", "10", "12"]


def test_clear_during_capture_cancels_it_and_allows_first_step_again(harness):
    h = harness
    source = h.connect()
    h.capture("neutral")
    assert h.controller.submit("capture", step="thumb_up")
    wait_for(lambda: h.controller.latest_snapshot()["calibration"]["active"] is not None)
    assert h.controller.submit("clear_calibration")
    wait_for(lambda: h.controller.latest_snapshot()["calibration"]["clear_generation"] == 1)
    assert h.controller._active is None and h.controller._captures == {}
    assert not source.closed and h.controller.latest_snapshot()["runtime"]["connected"]
    h.capture("neutral")
    assert h.controller.latest_snapshot()["calibration"]["captured_steps"] == ["neutral"]
