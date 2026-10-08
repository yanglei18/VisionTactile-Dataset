"""Opt-in durable capture management, independent of ROS and Qt.

The adapter returns concurrent.futures.Future objects. One control worker waits
for them; ROS callbacks continue in their own executor. A command timeout means
UNKNOWN, never permission to invent a new request ID and retry automatically.
"""
from __future__ import annotations

from concurrent.futures import TimeoutError
from copy import deepcopy
import json
from pathlib import Path
import queue
import threading
import time
import uuid

from vt_capture_core.validation import validate_controller, validate_message

from .storage import IntentStore, fingerprint, utc_now, bounded_text
from .metadata import MetadataCollector, bounded_document


def default_state_dir():
    return Path.home() / ".local" / "state" / "visiontactile" / "capture-manager"


def empty_management():
    return dict(enabled=False, controller=None, batches=[], trials=[], operation=None)


def receipt(request_id, state, code, detail, trial_id=""):
    return dict(request_id=request_id, state=state, code=code, detail=bounded_text(detail), trial_id=trial_id)


class ManagementService:
    """Owns only an intent database and recorder service client, never a recorder."""
    def __init__(self, model, adapter=None, *, enabled=False, state_dir=None,
                 timeout_s=2., poll_interval_s=.5, context_ttl_s=2.5, monotonic=time.monotonic,
                 quality_runner=None, admission_lock=None):
        if not .01 <= timeout_s <= 30 or not .01 <= poll_interval_s <= 30 or context_ttl_s <= 0:
            raise ValueError("invalid managed capture timing bounds")
        self.model, self.adapter, self.enabled = model, adapter, bool(enabled)
        self.timeout_s, self.poll_interval_s, self.context_ttl_s = timeout_s, poll_interval_s, context_ttl_s
        self._clock = monotonic
        self._mutex = threading.RLock()
        self._stop = threading.Event()
        self._commands = queue.Queue(maxsize=16)
        self._thread = None
        self._controller = None
        self._controller_at = None
        self._controller_available = False
        self.store = IntentStore(state_dir or default_state_dir()) if self.enabled else None
        self._metadata = MetadataCollector(model.config) if self.enabled else None
        self._quality_runner = quality_runner
        self._quality = None
        # Attached explicitly by node.main only with --enable-device-control.
        self.processes = None
        self.controller_present = lambda: False
        self._device_request_id = None
        self._desktop_admission = None
        if admission_lock is not None:
            from .desktop_admission import DesktopAdmission
            self._desktop_admission = DesktopAdmission(admission_lock)
        if self.store:
            self.store.recover_unresolved()
            for trial in self.store.active_trials():
                self._manifest(trial["id"])

    def start(self):
        if self._thread is not None or self._stop.is_set():
            raise RuntimeError("management service is single-use")
        if self.adapter is not None:
            self._thread = threading.Thread(target=self._run, name="capture-control", daemon=True)
            self._thread.start()
        if self.enabled and self.model.config.quality_enabled:
            self._create_quality_scheduler().start()
        return self

    def lifecycle_allowed(self, action):
        """Device maintenance and recording are distinct mutually exclusive work.

        A controller need not exist to start the camera/controller group, but an
        existing or previously observed controller must have a fresh idle state.
        """
        with self._mutex:
            if self._stop.is_set():
                return False, "后台正在退出，未发送设备命令。"
            if self.store and self.store.active_trials():
                return False, "存在录制中或状态待核实的试次；请先结束录制并确认状态。"
            if self._fresh_context():
                if self._controller["recorder_alive"] or self._controller["state"] not in (0, 5, 6):
                    return False, "录制尚未结束，暂不启停或重启设备。"
                return True, ""
            try:
                present = self.controller_present()
            except Exception:
                return False, "ROS 状态尚未确认，请等待设备发现完成。"
            last_active = self._controller is not None and (
                self._controller["recorder_alive"] or self._controller["state"] not in (0, 5, 6))
            if present or last_active:
                return False, "录制控制器状态未确认，请等待重新连接。"
            if self.model.snapshot()["recording"]["state"] == "RECORDING":
                return False, "检测到外部录制，暂不启停设备。"
            return True, ""

    def _create_quality_scheduler(self):
        if self._quality is None:
            from .quality_scheduler import QualityScheduler
            if self._quality_runner is None:
                from .quality_runner import QualityRunner
                self._quality_runner = QualityRunner(timeout_s=self.model.config.quality_timeout_s)
            self._quality = QualityScheduler(self.store, self._quality_runner,
                                            can_run=self._quality_can_run, event=self.model.event)
        return self._quality

    def _quality_can_run(self):
        with self._mutex:
            return (not self._stop.is_set() and self.enabled and self.model.config.quality_enabled
                    and self._fresh_context() and not self._controller["recorder_alive"]
                    and self._controller["state"] in (0, 5, 6) and not self.store.active_trials())

    def _fresh_context(self):
        return (self._controller is not None and self._controller_available and self._controller_at is not None
                and 0 <= self._clock()-self._controller_at <= self.context_ttl_s)

    def _owned(self, context):
        if not self.store or not context or not context["session_id"]:
            return None
        for trial in self.store.active_trials():
            if (trial["controller_instance_id"] == context["controller_instance_id"]
                    and trial["start_request_id"] == context["start_request_id"]
                    and trial["session_id"] == context["session_id"]):
                return trial
        return None

    def snapshot(self, version=1):
        value = self.model.snapshot()
        if version == 1:
            # A v1 client is always read-only, even beside an enabled manager.
            value["capabilities"]["record"] = False
            return value
        with self._mutex:
            management = empty_management()
            management["enabled"] = self.enabled
            if self.store:
                management.update(self.store.view())
            fresh = self._fresh_context()
            management["controller"] = deepcopy(self._controller) if fresh else None
            value["management"] = management
            context = self._controller if fresh else None
            can_manage = False
            if context is not None and self.enabled:
                active = self.store.active_trials()
                if context["state"] in (1, 2, 3, 4) or context["recorder_alive"]:
                    owned = self._owned(context)
                    can_manage = owned is not None and owned["state"] == "RECORDING" and context["state"] == 3
                else:
                    can_manage = not active
                value["recording"].update(
                    state={0: "IDLE", 3: "RECORDING", 5: "COMPLETE"}.get(context["state"], "UNAVAILABLE"),
                    session_id=context["session_id"], elapsed_s=float(context["elapsed_sec"]),
                    duration_s=float(context["planned_duration_sec"]), detail=context["detail"])
            value["capabilities"]["record"] = bool(can_manage and not (self.processes and self.processes.busy))
            if version >= 4:
                value["process_control"] = self.processes.snapshot() if self.processes else dict(enabled=False, groups=[])
                value["capabilities"]["device_control"] = self.processes is not None
                if self.processes is not None and self._device_request_id:
                    operation = self.processes.operation(self._device_request_id)
                    if operation is not None:
                        management["operation"] = operation
            return value

    def dispatch(self, request):
        validate_message(request)
        readonly = request["command"] in {"snapshot", "ping", "trial.details"}
        if self._desktop_admission is None or readonly:
            return self._dispatch(request)
        if not self._desktop_admission.acquire():
            operation = receipt(request["request_id"], "rejected", "desktop_configuration_applying",
                                "正在应用工作台配置，请等待后台重新连接后再操作。")
            return dict(version=request["version"], request_id=request["request_id"], ok=False,
                        snapshot=self.snapshot(request["version"]), operation=operation, error=operation["detail"])
        try:
            return self._dispatch(request)
        finally:
            self._desktop_admission.release()

    def _dispatch(self, request):
        version, request_id, command = request["version"], request["request_id"], request["command"]
        operation = None
        trial_details = None
        if command.startswith("device."):
            with self._mutex:
                if self.processes is None:
                    operation = receipt(request_id, "rejected", "device_control_disabled", "Start manager with --enable-device-control and explicit launch profiles.")
                else:
                    operation = self.processes.command(command, request["params"], request_id)
                    self._device_request_id = request_id
        elif command == "trial.details":
            trial_details = self.store.details(request["params"]["trial_id"]) if self.store else None
            if trial_details is None:
                operation = receipt(request_id, "rejected", "trial_not_found", "This trial has no local management record.", request["params"]["trial_id"])
        elif command not in {"snapshot", "ping"}:
            with self._mutex:
                self._device_request_id = None
            try:
                operation = self._intent(request_id, command, request.get("params", {}))
            except Exception as exc:
                # A failure after COMMIT cannot truthfully be reported as "not
                # sent". Preserve the ID and force explicit reconciliation.
                previous = self.store.operation(request_id) if self.store else None
                operation = receipt(request_id, "unknown", "manager_error",
                                    f"Operation outcome must be checked with the same request ID: {exc}",
                                    previous["trial_id"] if previous else "")
                self.model.event("error", operation["detail"], key="management-error")
        result = dict(version=version, request_id=request_id,
                      ok=operation is None or operation["state"] != "rejected", snapshot=self.snapshot(version))
        if version >= 2:
            result["operation"] = operation
        if version >= 3 and command == "trial.details":
            result["trial_details"] = trial_details
        if operation is not None and operation["state"] == "rejected":
            result["error"] = operation["detail"]
        return result

    def _intent(self, request_id, command, params):
        # Reference hashing/copy preparation is bounded and outside the control
        # lock. Software/git observations were cached once at manager startup.
        prepared = alignment_content = None
        if self.enabled and command == "capture.start":
            try:
                prepared, alignment_content = self._metadata.prepare()
                prepared["device_observations"] = self.model.provenance()
            except Exception as exc:
                prepared = dict(references=[], alignment_config=None, observation_error=bounded_text(exc),
                                prepared_at=utc_now(), content_observation_not_solver_load=True)
        with self._mutex:
            if not self.enabled:
                return receipt(request_id, "rejected", "read_only", "Recording management is disabled; start manager with --enable-recording.")
            prior = self.store.operation(request_id)
            digest, _ = fingerprint(command, params)
            if prior:
                if prior["fingerprint"] != digest:
                    return receipt(request_id, "rejected", "request_id_conflict", "This request ID already identifies a different durable operation.")
                return self.store.receipt(prior)

            def reject(code, detail, trial_id=""):
                with self.store.transaction() as db:
                    self.store.insert_operation(db, request_id, command, params, state="rejected", code=code,
                                                detail=detail, trial_id=trial_id)
                return receipt(request_id, "rejected", code, detail, trial_id)

            if command == "batch.create":
                batch_id = uuid.uuid4().hex
                with self.store.transaction() as db:
                    db.execute("INSERT INTO batches VALUES(?,?,?)", (batch_id, params["label"], utc_now()))
                    self.store.insert_operation(db, request_id, command, params, state="accepted", code="batch_created",
                                                detail="Batch metadata committed; no recording has been started.")
                return self.store.receipt(self.store.operation(request_id))
            if command == "quality.retry":
                trial_id = params["trial_id"]
                if not self.model.config.quality_enabled:
                    return reject("quality_disabled", "Quality execution is disabled in manager configuration.", trial_id)
                details = self.store.details(trial_id)
                if details is None or details["metadata"]["finish"] is None:
                    return reject("quality_not_eligible", "No verified frozen finish exists; legacy trials are not automatically imported or assessed.", trial_id)
                job = self.store.quality_job(trial_id)
                if job is not None and job["state"] in {"QUEUED", "RUNNING"}:
                    return reject("quality_busy", "A quality attempt is already queued or running.", trial_id)
                with self.store.transaction() as db:
                    self.store.retry_quality(db, trial_id)
                    self.store.insert_operation(db, request_id, command, params, state="accepted", code="quality_queued",
                                                detail="Read-only quality retry durably queued; recording state is unchanged.", trial_id=trial_id)
                if self._quality:
                    self._quality.wake()
                self._manifest(trial_id)
                return self.store.receipt(self.store.operation(request_id))
            if not self._fresh_context():
                return reject("controller_unavailable", "A fresh managed controller snapshot is required; no command was sent.")
            context = deepcopy(self._controller)
            if params["expected_controller_instance_id"] != context["controller_instance_id"]:
                return reject("controller_instance_mismatch", "Controller instance changed; refresh context before submitting a new operation.")
            if self._commands.full():
                return reject("queue_full", "Control queue is full; no command was sent.")
            if command == "capture.start":
                if self.processes and self.processes.busy:
                    return reject("device_transition", "设备启停正在进行，请等待操作结束后再开始录制。")
                if params["expected_state_revision"] != context["state_revision"]:
                    return reject("state_revision_mismatch", "Controller state revision changed; no command was sent.")
                if context["state"] not in (0, 5, 6) or context["recorder_alive"]:
                    return reject("active_session", "Controller already has an active or transitioning session.")
                if self.store.active_trials():
                    return reject("unresolved_local_trial", "A local trial is active or unresolved; snapshot reconciliation is required before another START.")
                if params["duration_s"] > context["max_duration_sec"]:
                    return reject("duration_out_of_range", "Requested duration exceeds the controller limit.")
                trial_id = uuid.uuid4().hex
                with self.store.transaction() as db:
                    if db.execute("SELECT 1 FROM batches WHERE id=?", (params["batch_id"],)).fetchone() is None:
                        self.store.insert_operation(db, request_id, command, params, state="rejected", code="batch_not_found",
                                                    detail="Select an existing local batch before START.")
                        return self.store.receipt(self.store.operation(request_id))
                    db.execute("INSERT INTO trials(id,batch_id,label,state,duration_s,created_at,controller_instance_id,start_request_id,detail) "
                               "VALUES(?,?,?,'STARTING',?,?,?,?,?)",
                               (trial_id, params["batch_id"], params["label"], float(params["duration_s"]), utc_now(),
                                context["controller_instance_id"], request_id, "START intent committed; awaiting asynchronous controller receipt."))
                    self.store.insert_operation(db, request_id, command, params, state="accepted", code="queued",
                                                detail="START intent committed and queued; accepted does not mean recording has begun.",
                                                trial_id=trial_id, phase="INTENT")
                    try:
                        metadata = self._metadata.start_document(request_id, params, context, prepared)
                    except Exception as exc:
                        metadata = dict(schema_version=1, request_id=request_id, frozen_at=utc_now(), planned=params,
                            controller_before_start=context, metadata_status="unknown", metadata_error=bounded_text(exc),
                            content_observation_not_solver_load=True)
                    self.store.insert_start_metadata(db, trial_id, metadata, alignment_content)
                if self._quality:
                    self._quality.interrupt()
                wire = dict(command=1, session_label=params["label"], planned_duration_sec=params["duration_s"])
            elif command == "capture.stop":
                trial_id = params["trial_id"]
                trial = self.store.trial(trial_id)
                owned = self._owned(context)
                if (trial is None or owned is None or trial_id != owned["id"]
                        or not params["expected_session_id"] or params["expected_session_id"] != context["session_id"]):
                    return reject("session_not_owned", "STOP is only allowed for the current session owned by this local trial.", trial_id)
                if trial["state"] not in {"STARTING", "RECORDING", "UNKNOWN"} or context["state"] not in (1, 2, 3):
                    return reject("not_stoppable", "This trial is not an active stoppable recording.", trial_id)
                with self.store.transaction() as db:
                    db.execute("UPDATE trials SET state='STOPPING',detail=? WHERE id=?",
                               ("STOP intent committed; finalization is not yet confirmed.", trial_id))
                    self.store.insert_operation(db, request_id, command, params, state="accepted", code="queued",
                                                detail="STOP intent committed and queued; finalization is not yet confirmed.",
                                                trial_id=trial_id, phase="INTENT")
                wire = dict(command=2, session_label="", planned_duration_sec=0)
            else:
                return reject("unsupported_command", "Unsupported managed operation.")
            wire.update(request_id=request_id, expected_controller_instance_id=context["controller_instance_id"],
                        expected_session_id=context["session_id"], expected_state_revision=context["state_revision"])
            # The SQLite COMMIT above precedes queue insertion and every external
            # adapter call. A crash in between is reconciled, never replayed.
            self._commands.put_nowait((request_id, trial_id, wire))
            self._manifest(trial_id)
            return self.store.receipt(self.store.operation(request_id))

    def _manifest(self, trial_id):
        try:
            self.store.write_manifest(trial_id)
        except (OSError, ValueError) as exc:
            self.model.event("error", f"Trial manifest unavailable: {exc}", key="manifest")

    def _run(self):
        next_poll = 0.
        while not self._stop.is_set():
            if self._clock() >= next_poll:
                self.poll_once()
                next_poll = self._clock()+self.poll_interval_s
            try:
                item = self._commands.get(timeout=.05)
            except queue.Empty:
                continue
            if self._stop.is_set():
                return
            try:
                self._execute(*item)
            except Exception as exc:
                self._unknown(item[0], item[1], f"Unexpected control failure: {exc}")
            finally:
                self._commands.task_done()

    def poll_once(self):
        """One read-only reconciliation, usable with a fake adapter in tests."""
        if self.adapter is None:
            return False
        try:
            future = self.adapter.get_snapshot()
            try:
                context = future.result(timeout=self.timeout_s)
            except TimeoutError:
                future.cancel()  # Abandon local wait only; no ROS command cancellation.
                raise
            self.reconcile(context)
            return True
        except Exception as exc:
            with self._mutex:
                self._controller_available = False
            self.model.event("warning", f"Managed recorder snapshot unavailable: {exc}", key="managed-controller")
            return False

    def reconcile(self, context):
        validate_controller(context)
        if context["state"] not in range(7):
            raise ValueError("unsupported controller state")
        context = deepcopy(context)
        with self._mutex:
            old = self._controller
            if (old is not None and old["controller_instance_id"] == context["controller_instance_id"]
                    and context["state_revision"] < old["state_revision"]):
                return
            self._controller, self._controller_at, self._controller_available = context, self._clock(), True
            if not self.store:
                return
            for trial in self.store.active_trials():
                if trial["controller_instance_id"] != context["controller_instance_id"]:
                    with self.store.transaction() as db:
                        db.execute("UPDATE trials SET state='UNKNOWN',detail=? WHERE id=?",
                                   ("Controller instance changed; session ownership is unresolved, no command was retried.", trial["id"]))
                    continue
                if trial["start_request_id"] != context["start_request_id"]:
                    # Labels or paths never establish ownership of an external session.
                    continue
                if trial["session_id"] and trial["session_id"] != context["session_id"]:
                    with self.store.transaction() as db:
                        db.execute("UPDATE trials SET state='UNKNOWN',detail=? WHERE id=?",
                                   ("Controller reported a different session for the same START; confirmed ownership was preserved.", trial["id"]))
                    self._manifest(trial["id"])
                    continue
                state = {0: "UNKNOWN", 1: "STARTING", 2: "STARTING", 3: "RECORDING", 4: "STOPPING",
                         5: "COMPLETE", 6: "FAILED"}[context["state"]]
                detail = context["detail"]
                if context["state"] == 6 and context["recorder_alive"]:
                    state = "UNKNOWN"
                    detail = "Controller state is INVALID but the recorder is still alive; termination is not confirmed. " + detail
                with self.store.transaction() as db:
                    if state == "RECORDING":
                        stop = db.execute("SELECT state FROM operations WHERE trial_id=? AND command='capture.stop' "
                                          "AND state IN ('accepted','unknown') ORDER BY created_at DESC,rowid DESC LIMIT 1",
                                          (trial["id"],)).fetchone()
                        if stop is not None:
                            state = "UNKNOWN" if stop["state"] == "unknown" else "STOPPING"
                    db.execute("UPDATE trials SET state=?,session_id=?,output_path=?,elapsed_s=?,detail=? WHERE id=?",
                               (state, context["session_id"], context["output_path"], float(context["elapsed_sec"]),
                                bounded_text(detail, 4096), trial["id"]))
                    db.execute("UPDATE operations SET state='accepted',code='reconciled',detail=?,phase='TERMINAL',updated_at=? "
                               "WHERE request_id=? AND state='unknown'",
                               ("Controller snapshot confirms this START request; no retry was sent.", utc_now(), trial["start_request_id"]))
                    if state in {"STOPPING", "COMPLETE", "FAILED"}:
                        db.execute("UPDATE operations SET state='accepted',code='reconciled',detail=?,phase='TERMINAL',updated_at=? "
                                   "WHERE trial_id=? AND command='capture.stop' AND state='unknown'",
                                   ("Owned session is finalizing or terminal; STOP effect observed without retry.", utc_now(), trial["id"]))
                    if context["state"] in (5, 6) and not context["recorder_alive"] and context["session_id"]:
                        self.store.freeze_finish(db, trial["id"], bounded_document(dict(schema_version=1,
                            frozen_at=utc_now(), controller=context, ownership_confirmed=True,
                            recorder_stopped_confirmed=True)), quality_enabled=self.model.config.quality_enabled)
                self._manifest(trial["id"])
            if self._quality:
                self._quality.wake()

    def _execute(self, request_id, trial_id, wire):
        self.store.update_operation(request_id, state="accepted", code="dispatching",
                                    detail="Durable intent dispatched once; awaiting controller response.", phase="DISPATCHED")
        try:
            future = self.adapter.command(deepcopy(wire))
            try:
                response = future.result(timeout=self.timeout_s)
            except TimeoutError:
                future.cancel()
                raise
        except Exception as exc:
            self._unknown(request_id, trial_id, f"Controller command outcome unknown: {exc}. No automatic retry.")
            return
        try:
            if type(response) is not dict or type(response.get("accepted")) is not bool:
                raise ValueError("invalid controller response")
            validate_controller(response["context"])
            accepted = response["accepted"]
            self.store.update_operation(request_id, state="accepted" if accepted else "rejected",
                                        code=bounded_text(response["code"], 64) or "controller_response", detail=str(response["detail"]))
            if not accepted:
                context = response["context"]
                confirmed_start = (wire["command"] == 1 and bool(context["session_id"])
                                   and context["controller_instance_id"] == wire["expected_controller_instance_id"]
                                   and context["start_request_id"] == request_id)
                with self.store.transaction() as db:
                    if wire["command"] == 1 and not confirmed_start:
                        db.execute("UPDATE trials SET state='FAILED',detail=? WHERE id=?", (bounded_text(response["detail"]), trial_id))
                    elif wire["command"] == 2:
                        db.execute("UPDATE trials SET state='UNKNOWN',detail=? WHERE id=?", (bounded_text(response["detail"]), trial_id))
            self.reconcile(response["context"])
            self._manifest(trial_id)
        except Exception as exc:
            self._unknown(request_id, trial_id, f"Invalid controller receipt; outcome unknown: {exc}. No automatic retry.")

    def _unknown(self, request_id, trial_id, detail):
        with self._mutex:
            self.store.update_operation(request_id, state="unknown", code="outcome_unknown", detail=detail, phase="UNKNOWN")
            with self.store.transaction() as db:
                db.execute("UPDATE trials SET state='UNKNOWN',detail=? WHERE id=?", (bounded_text(detail), trial_id))
            self._manifest(trial_id)

    def close(self):
        """Stop observing, never send STOP and never terminate an external recorder."""
        self._stop.set()
        if self._quality is not None:
            self._quality.close()
        if self._thread is not None:
            self._thread.join(self.timeout_s+2.)
            if self._thread.is_alive():
                raise RuntimeError("Control worker did not shut down within the bounded timeout")
        if self.store:
            self.store.close()
        if self._desktop_admission is not None:
            self._desktop_admission.close()
