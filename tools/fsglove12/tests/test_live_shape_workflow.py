"""Live shape workflow boundaries with fake inputs/models/jobs, no hardware."""
from copy import deepcopy
import json
from types import SimpleNamespace

import numpy as np
import pytest

from test_live_runtime import Fusion
from test_live_runtime_high_rate import feed, setup_direct
from test_shape_calibration import PROVENANCE, profile as shape_profile
from vt_fsglove12 import shape_calibration


MODEL = dict(hand="right", sha256="b" * 64, path="/mock/model.pkl")


class Job:
    def __init__(self, contacts, options, directory):
        self.contacts, self.options, self.directory = contacts, options, directory
        self.running, self.cancelled, self.result = True, False, None

    def is_alive(self):
        return self.running

    def cancel(self):
        self.cancelled = True

    def join(self, timeout=None):
        pass


class Geometry:
    def __init__(self):
        self.calls = []

    def render_mano(self, pose, params):
        self.calls.append((deepcopy(pose), list(params)))
        return dict(vertices_m=[[params[0], 0., 0.]], joints_m=[[params[0], 0., 0.]] * 21,
                    faces=[], provenance={"source": "fake_geometry_test_only"})


class Exporter:
    def __init__(self):
        self.documents, self.errors = [], []

    def publish(self, value):
        self.documents.append(deepcopy(value))

    def record_error(self, error):
        self.errors.append(str(error))

    def snapshot(self):
        return dict(sent=len(self.documents), errors=len(self.errors))


@pytest.fixture
def workflow(tmp_path, monkeypatch):
    controller, session, backend, clock = setup_direct(tmp_path)
    controller._fusion_factory = Fusion
    controller._shape_job_factory = Job
    controller._profile.update(status="provisional_not_accuracy_validated",
        session_id=session.session_id, config_sha256=controller._config_sha,
        rest_sha256=controller._rest_sha, nodes={"12": {"mounting": "unchanged"}},
        upstream_state_sha256="c" * 64, upstream_calibration={"matrix": [[1., 0.], [0., 1.]]},
        functional_fingers={"index": {"axis": [1., 0., 0.]}}, functional_enabled=True,
        shape_params=[.15] * 10, shape_calibrated=False,
        warnings=["fixture: 未声明完成手形触指优化"])
    controller._profile_status = controller._profile["status"]
    backend.core = SimpleNamespace(provenance=deepcopy(PROVENANCE), mano_assets_root="NO_MODEL_FILES")
    backend.shape_job_options = lambda: dict(checkout="NO_CHECKOUT", mano_assets="NO_MODEL_FILES")
    original_solve = backend.solve
    def solve(frame, profile, timestamp_s):
        pose = original_solve(frame, profile, timestamp_s)
        pose.update(shape_params=list(profile.get("shape_params", [0.] * 10)),
                    source_fresh=True, virtual_input_slots=[])
        return pose
    backend.solve = solve
    monkeypatch.setattr(shape_calibration, "model_provenance", lambda path, hand="right": {**deepcopy(MODEL), "hand": hand})
    geometry, exporter = Geometry(), Exporter()
    controller._geometry_backend, controller._pose_exporter = geometry, exporter
    h = SimpleNamespace(controller=controller, session=session, backend=backend, clock=clock,
                        geometry=geometry, exporter=exporter, directory=tmp_path)
    fresh(h)
    yield h
    controller.stop()


def fresh(h):
    feed(h.session, h.clock, *range(1, 13))
    for node in h.session.fusion.nodes.values():
        node.update(host_monotonic_ns=int(h.clock() * 1e9), host_realtime_ns=int((h.clock() + 1000) * 1e9))
    return h.session.fusion.snapshot(h.clock())


def current_pose(h):
    return h.backend.solve(h.session.fusion.snapshot(h.clock()), h.controller._profile, h.clock())


def start(h, category=2):
    fresh(h)
    h.controller._command("capture_shape_contact", dict(category=category, contact_confirmed=True))
    capture = h.controller._shape_capture
    assert capture is not None, h.controller._shape_error
    return capture


def complete(h, category=2, *, count=20):
    capture = start(h, category)
    for offset in np.linspace(.05, 1.95, count):
        h.clock.value = capture["collect_at"] + float(offset)
        fresh(h)
        assert h.controller._tick(h.session)
    h.clock.value = capture["end_at"]
    h.controller._poll_shape()
    return capture


def fill_four(h):
    for category in (2, 3, 4, 5):
        complete(h, category)
    assert set(h.controller._shape_contacts) == {2, 3, 4, 5}


def document(beta=.7):
    return {**shape_profile(), "shape_params": [beta] * 10, "mano_model": deepcopy(MODEL)}


def candidate(h, *, beta=.7):
    result = dict(status="completed", candidate_available=True, shape_profile=document(beta),
                  shape_path=str(h.directory / "candidate.json"), before_shape_params=[.15] * 10,
                  metrics={"physical_accuracy_verified": False})
    h.controller._shape_candidate = deepcopy(result)
    return result


def test_four_explicit_labels_prepare_hold_and_background_job_frozen_inputs(workflow):
    h, c = workflow, workflow.controller
    assert not c.submit("capture_shape_contact", category=2, contact_confirmed=False)
    assert not c.submit("capture_shape_contact", category=True, contact_confirmed=True)
    capture = start(h)
    assert c._shape_state(h.clock())["phase"] == "preparing"
    h.clock.value = capture["collect_at"] - .01
    c._shape_tick(h.session, fresh(h), current_pose(h), h.clock())
    assert capture["rows"] == []
    c._command("cancel_shape", {})
    fill_four(h)
    assert c._shape_state(h.clock())["can_optimize"]
    original = deepcopy(c._profile)
    c._command("optimize_shape", {})
    job = c._shape_job
    assert job is not None and job.running and c._session is h.session
    assert not h.session.stop.is_set() and c._profile == original
    assert job.contacts["contact_labels_confirmed"] is True
    assert job.contacts["contact_labels_source"] == "user_explicit_not_inferred_from_imu"
    assert set(job.contacts["finger_category"]) == {2, 3, 4, 5}
    assert 40 <= len(job.contacts["mano_pose_rotvec"]) <= 120
    assert all(row["contact_confirmed"] and row["input_identity"]
               for row in job.contacts["capture_evidence"])
    assert job.options["iterations"] == 500 and job.options["seed"] == 0
    assert c._snapshot["runtime"]["shape_command_generation"] == 7


@pytest.mark.parametrize("fault", ["display_only", "held_pose", "state_only", "unavailable", "failed",
    "calibration_rejection", "virtual_thumb", "virtual_contact", "malformed", "nonfinite",
    "missing_required", "invalid_orientation", "stale", "gyro_rejected", "accel_rejected"])
def test_capture_rejects_nonmeasurement_invalid_virtual_or_unhealthy_input(workflow, fault):
    h, c = workflow, workflow.controller
    capture = start(h)
    h.clock.value = capture["collect_at"] + .1
    frame, pose = fresh(h), current_pose(h)
    required_node = next(node for node, slot in c.config["node_to_slot"].items() if slot == 14)
    if fault == "display_only": pose["display_only"] = True
    elif fault == "held_pose": pose["held_slots"] = [14]
    elif fault == "state_only": pose["source_fresh"] = False
    elif fault == "unavailable": pose["pose_available"] = False
    elif fault == "failed": pose["solver_success"] = False
    elif fault == "calibration_rejection": pose["calibration_rejections"] = {required_node: "bad"}
    elif fault == "virtual_thumb": pose["virtual_input_slots"] = [14]
    elif fault == "virtual_contact": pose["virtual_input_slots"] = [1]
    elif fault == "malformed": pose["mano_pose_rotvec"] = [[0., 0., 0.]] * 15
    elif fault == "nonfinite": pose["mano_pose_rotvec"][14] = [float("nan"), 0., 0.]
    elif fault == "missing_required": frame["nodes"].pop(required_node)
    elif fault == "invalid_orientation": frame["nodes"][required_node]["orientation_valid"] = False
    elif fault == "stale": frame["nodes"][required_node]["age_s"] = c.config["stale_after_s"] + .01
    else: frame["nodes"][required_node]["input_filter"] = {fault: True}
    c._shape_tick(h.session, frame, pose, h.clock())
    assert capture["rows"] == [], fault


def test_fresh_identity_not_reused_and_high_rate_capture_is_time_bounded(workflow):
    h, c = workflow, workflow.controller
    capture = start(h)
    for index in range(240):
        h.clock.value = capture["collect_at"] + index / 120.
        frame = fresh(h)
        pose = current_pose(h)
        c._shape_tick(h.session, frame, pose, h.clock())
        before = deepcopy(capture["rows"])
        c._shape_tick(h.session, frame, pose, h.clock())
        assert capture["rows"] == before
    assert 10 <= len(capture["rows"]) <= 30
    identities = [tuple(tuple(x) for x in row["input_identity"]) for row in capture["rows"]]
    assert len(identities) == len(set(identities))
    h.clock.value = capture["end_at"]
    c._poll_shape()
    assert 2 in c._shape_contacts


def test_minimum_ten_frames_and_cancel_or_failed_recapture_keep_previous_group(workflow):
    h, c = workflow, workflow.controller
    original = complete(h)
    assert 2 in c._shape_contacts
    start(h)
    c._command("cancel_shape", {})
    assert c._shape_contacts[2] is original and c._shape_capture is None
    complete(h, count=9)
    assert c._shape_contacts[2] is original
    assert c._shape_state(h.clock())["status"] == "failed"
    assert "不足" in c._shape_error
    complete(h, category=3, count=9)
    assert set(c._shape_contacts) == {2}


@pytest.mark.parametrize("operation", ["cancel", "clear", "disconnect", "reference_change"])
def test_cancelled_or_stale_completed_job_cannot_reintroduce_candidate(workflow, operation):
    h, c = workflow, workflow.controller
    fill_four(h)
    c._command("optimize_shape", {})
    job = c._shape_job
    original = deepcopy(c._shape_contacts)
    if operation == "cancel": c._command("cancel_shape", {})
    elif operation == "clear": c._command("clear_calibration", {})
    elif operation == "disconnect": c._command("disconnect", {})
    else:
        changed = deepcopy(c._profile)
        changed["functional_enabled"] = False
        c._save_profile(h.session, changed, event="functional_enabled_changed")
    assert job.cancelled
    job.result = dict(report=candidate(h), error=None)
    c._shape_candidate = None  # candidate helper constructs the simulated report only.
    job.running = False
    c._poll_shape()
    assert c._shape_candidate is None and c._shape_job is None
    assert not c._shape_state(h.clock())["candidate_available"]
    if operation == "cancel":
        assert c._shape_contacts == original
    else:
        assert not c._shape_contacts


def test_completed_job_requires_explicit_apply_and_shape_only_transaction_can_revert(workflow):
    h, c = workflow, workflow.controller
    fill_four(h)
    original = deepcopy(c._profile)
    fusion = h.session.fusion
    c._command("optimize_shape", {})
    job = c._shape_job
    report = candidate(h)
    c._shape_candidate = None
    job.result, job.running = dict(report=report, error=None), False
    c._poll_shape()
    assert c._profile == original and c._shape_state(h.clock())["candidate_available"]
    c._command("apply_shape_candidate", {})
    assert c._shape_error is None
    assert c._profile["shape_params"] == [.7] * 10 and c._profile["shape_calibrated"]
    for key in ("nodes", "upstream_calibration", "upstream_state_sha256", "functional_fingers", "functional_enabled"):
        assert c._profile[key] == original[key]
    assert h.session.fusion is fusion and not h.session.stop.is_set()
    assert c._shape_state(h.clock())["can_revert"]
    assert (h.directory / "profile.v001.json").is_file()
    c._command("revert_shape", {})
    assert c._profile["shape_params"] == original["shape_params"]
    assert c._profile["shape_calibrated"] == original["shape_calibrated"]
    assert c._profile["functional_fingers"] == original["functional_fingers"]
    assert h.session.fusion is fusion


@pytest.mark.parametrize("mode,beta", [("current", .15), ("candidate", .7), ("baseline", .15)])
def test_preview_switch_only_changes_display_never_pose_recording_or_ros(workflow, mode, beta):
    h, c = workflow, workflow.controller
    candidate(h)
    original = deepcopy(c._profile)
    c._command("preview_shape", dict(mode=mode))
    h.clock.advance(.1)
    fresh(h)
    c._tick(h.session)
    result = c.latest_snapshot()
    assert c._profile == original
    assert result["pose"]["shape_params"] == [.15] * 10
    assert result["geometry"]["vertices_m"][0][0] == .15
    assert result["display_geometry"]["vertices_m"][0][0] == beta
    assert result["display_pose"]["shape_preview_mode"] == mode
    assert h.exporter.documents[-1]["shape_params"] == [.15] * 10
    assert h.exporter.documents[-1]["joint_positions_m"][0] == .15
    assert not h.exporter.errors
    recorded = json.loads(h.session.streams["poses"].getvalue().splitlines()[-1])
    assert recorded["shape_params"] == [.15] * 10
    assert not recorded.get("display_only") and "shape_preview_mode" not in recorded


def test_loading_before_connection_does_not_connect_and_next_base_fit_uses_saved_shape(workflow):
    h, c = workflow, workflow.controller
    path = h.directory / "import.json"
    path.write_text(json.dumps(document(.8)))
    installed = deepcopy(c._profile)
    c._session, c._profile = None, None
    c._command("load_shape_profile", dict(path=str(path)))
    assert c._shape_error is None and c._shape_applied["shape_params"] == [.8] * 10
    assert c._session is None and c._profile is None
    assert not c.latest_snapshot()["runtime"]["connected"]
    c._session = h.session
    h.backend.fit_calibration = lambda captures, session_id: deepcopy(installed)
    c._fit_profile(h.session)
    assert c._profile["shape_params"] == [.8] * 10
    assert c._profile["functional_fingers"] == installed["functional_fingers"]
    assert c._profile["upstream_calibration"] == installed["upstream_calibration"]


def test_preview_change_without_new_input_cannot_trigger_a_new_measurement(workflow):
    h, c = workflow, workflow.controller
    candidate(h)
    assert c._tick(h.session)
    pose = deepcopy(c.latest_snapshot()["pose"])
    exported = deepcopy(h.exporter.documents)
    frames = h.session.frames
    c._command("preview_shape", dict(mode="candidate"))
    h.clock.advance(.01)  # Change only display mode, not the sensor identity.
    assert c._tick(h.session) is False
    assert c.latest_snapshot()["pose"] == pose
    assert h.exporter.documents == exported and h.session.frames == frames
    fresh(h)
    assert c._tick(h.session)
    assert c.latest_snapshot()["display_pose"]["shape_preview_mode"] == "candidate"


def test_export_creates_new_file_but_cannot_overwrite_existing_profile(workflow):
    h, c = workflow, workflow.controller
    candidate(h)
    original = deepcopy(c._profile)
    path = h.directory / "new-export.json"
    c._command("save_shape_profile", dict(path=str(path)))
    assert c._shape_error is None and json.loads(path.read_text())["shape_params"] == [.7] * 10
    saved = path.read_bytes()
    c._shape_candidate["shape_profile"]["shape_params"] = [.9] * 10
    c._command("save_shape_profile", dict(path=str(path)))
    assert c._shape_error and path.read_bytes() == saved
    assert c._profile == original and c._session is h.session
    assert not h.session.stop.is_set()


def test_export_failure_does_not_stop_live_pose_or_discard_prior_shape(workflow, monkeypatch):
    from vt_fsglove12 import calibration_artifacts
    h, c = workflow, workflow.controller
    candidate(h)
    original = deepcopy(c._profile)
    def fail(path, value):
        raise OSError("fixture full disk")
    monkeypatch.setattr(calibration_artifacts, "write_artifact", fail)
    c._command("save_shape_profile", dict(path=str(h.directory / "full.json")))
    assert "full disk" in c._shape_error
    assert c._snapshot["runtime"]["shape_command_generation"] == 1
    assert c._profile == original and c._session is h.session and not h.session.stop.is_set()
    h.clock.advance(.1)
    fresh(h)
    c._tick(h.session)
    assert c.latest_snapshot()["pose"]["pose_available"] and h.exporter.documents


def test_failed_profile_import_preserves_applied_and_mounting_data(workflow):
    h, c = workflow, workflow.controller
    original = deepcopy(c._profile)
    c._shape_applied = document(.6)
    bad = document(.9)
    bad["mano_model"]["sha256"] = "d" * 64
    path = h.directory / "wrong-model.json"
    path.write_text(json.dumps(bad))
    c._command("load_shape_profile", dict(path=str(path)))
    assert "different MANO model" in c._shape_error
    assert c._profile == original and c._shape_applied["shape_params"] == [.6] * 10
    assert not h.session.stop.is_set()


@pytest.mark.parametrize("command,args", [("capture", {"step": "neutral"}),
    ("capture_functional", {"finger": "index"}), ("capture_functional_hand", {}),
    ("set_functional_enabled", {"enabled": False})])
def test_mounting_or_functional_changes_cannot_overlap_active_shape_capture(workflow, command, args):
    h, c = workflow, workflow.controller
    capture = start(h)
    original = deepcopy(c._profile)
    with pytest.raises(ValueError, match="手形"):
        c._command(command, args)
    assert c._shape_capture is capture and c._profile == original
    assert c._active is None and not h.session.stop.is_set()


def test_cancelled_job_cleanup_blocks_new_fit_until_old_process_has_finished(workflow):
    h, c = workflow, workflow.controller
    fill_four(h)
    c._command("optimize_shape", {})
    job = c._shape_job
    c._command("cancel_shape", {})
    assert job.cancelled and c._shape_state(h.clock())["busy"]
    c._command("optimize_shape", {})
    assert c._shape_job is job and c._shape_error
    job.running, job.result = False, {"report": None, "error": "cancelled fixture"}
    c._poll_shape()
    assert c._shape_job is None and c._shape_state(h.clock())["can_optimize"]
    c._command("optimize_shape", {})
    assert c._shape_job is not None and c._shape_job is not job
    assert c._shape_error is None


def test_profile_write_failure_keeps_applied_memory_and_live_acquisition(workflow, monkeypatch):
    from vt_fsglove12 import live_runtime
    h, c = workflow, workflow.controller
    candidate(h)
    original_installation = deepcopy(c._profile["upstream_calibration"])
    def fail(path, value):
        raise OSError("fixture profile filesystem full")
    monkeypatch.setattr(live_runtime, "write_artifact", fail)
    c._command("apply_shape_candidate", {})
    assert c._profile["shape_params"] == [.7] * 10
    assert c._profile["upstream_calibration"] == original_installation
    assert not h.session.recording_disabled.is_set()
    assert c._calibration_storage_status(h.session)["pending_count"] >= 1
    assert not h.session.stop.is_set() and c._session is h.session
    h.clock.advance(.1)
    fresh(h)
    c._tick(h.session)
    result = c.latest_snapshot()
    assert result["runtime"]["connected"] and result["pose"]["pose_available"]
    assert result["pose"]["shape_params"] == [.7] * 10
    assert h.exporter.documents[-1]["shape_params"] == [.7] * 10


def test_snapshot_exposes_only_contact_counts_not_pose_training_blob(workflow):
    h, c = workflow, workflow.controller
    fill_four(h)
    value = c.latest_snapshot()["calibration"]["shape_calibration"]
    assert set(value["contacts"]) == {"2", "3", "4", "5"}
    assert all(set(group) == {"frames", "confirmed", "saved"} for group in value["contacts"].values())
    assert all(group["frames"] >= 10 and group["confirmed"] for group in value["contacts"].values())
    value["contacts"].clear()
    assert len(c.latest_snapshot()["calibration"]["shape_calibration"]["contacts"]) == 4


@pytest.mark.parametrize("operation", ["apply", "load"])
@pytest.mark.parametrize("previous_personalized", [False, True])
def test_duplicate_apply_or_load_preserves_original_rollback_and_does_not_save_again(
        workflow, operation, previous_personalized):
    h, c = workflow, workflow.controller
    if previous_personalized:
        c._shape_applied = document(.4)
        c._shape_applied_path = str(h.directory / "previous.json")
        c._profile = c._shape_with_applied(c._profile)
    original_profile = deepcopy(c._profile)
    original_document, original_path = deepcopy(c._shape_applied), c._shape_applied_path
    report = candidate(h)
    if operation == "load":
        path = h.directory / "repeat-load.json"
        path.write_text(json.dumps(report["shape_profile"]))
        command, args = "load_shape_profile", {"path": str(path)}
    else:
        command, args = "apply_shape_candidate", {}
    assert c._shape_state(h.clock())["can_apply"]
    c._command(command, args)
    assert c._shape_error is None
    rollback = c._shape_previous
    applied = deepcopy(c._profile)
    saved = {path.name: path.read_bytes() for path in h.directory.glob("profile*.json")}
    assert rollback["document"] == original_document
    assert rollback["fields"]["shape_params"] == original_profile["shape_params"]
    assert not c._shape_state(h.clock())["can_apply"]
    assert c._shape_state(h.clock())["can_revert"]

    c._command(command, args)
    assert c._shape_error is None and c._shape_previous is rollback
    assert c._profile == applied
    assert {path.name: path.read_bytes() for path in h.directory.glob("profile*.json")} == saved
    assert c._snapshot["runtime"]["shape_command_generation"] == 2
    assert not c._shape_state(h.clock())["can_apply"]
    c._command("revert_shape", {})
    assert c._profile["shape_params"] == original_profile["shape_params"]
    assert c._profile["shape_calibrated"] == original_profile["shape_calibrated"]
    assert c._profile["warnings"] == original_profile["warnings"]
    assert c._shape_applied == original_document and c._shape_applied_path == original_path
    assert c._shape_previous is None and not h.session.stop.is_set()
