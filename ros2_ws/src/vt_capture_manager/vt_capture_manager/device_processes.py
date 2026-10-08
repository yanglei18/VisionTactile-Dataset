"""Opt-in, Linux process-group supervision without ROS or GUI dependencies.

Only trusted configuration supplies argv. A new manager never adopts a process
from the registry: those entries are safety hints, not authority to signal it.
Closing this service deliberately leaves devices running. No automatic launch,
restart, retry, or stop is performed.
"""
from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass, field
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time
import uuid

from .serial_access import serial_start_problem
from .storage import bounded_text, private_directory


@dataclass(frozen=True)
class ProcessSpec:
    id: str
    label: str
    kind: str
    argv: tuple[str, ...]
    topics: tuple[str, ...]
    resources: tuple[str, ...]
    enabled: bool = True
    startup_timeout_s: float = 30.

    def __post_init__(self):
        if not isinstance(self.id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", self.id):
            raise ValueError("invalid device process group id")
        if not isinstance(self.label, str) or not self.label.strip() or len(self.label.encode("utf-8")) > 512:
            raise ValueError("invalid device process label")
        if self.kind not in ("camera", "tracker", "glove", "tactile"):
            raise ValueError("invalid device process kind")
        for name in ("argv", "topics", "resources"):
            values = getattr(self, name)
            if not isinstance(values, tuple) or any(not isinstance(v, str) or not v or "\0" in v for v in values):
                raise ValueError(f"{name} must be a tuple of nonempty strings")
        if not self.argv or len(self.argv) > 128 or sum(map(len, self.argv)) > 32768:
            raise ValueError("invalid device argv")
        if (len(self.topics) > 128 or len(self.resources) > 128 or len(set(self.topics)) != len(self.topics)
                or any(not t.startswith("/") or len(t.encode("utf-8")) > 512 for t in self.topics)):
            raise ValueError("invalid device topics/resources")
        if type(self.enabled) is not bool or not isinstance(self.startup_timeout_s, (int, float)) or isinstance(self.startup_timeout_s, bool):
            raise ValueError("invalid device process timing/enabled flag")
        if not math.isfinite(self.startup_timeout_s) or not .01 <= self.startup_timeout_s <= 600:
            raise ValueError("device startup timeout must be between .01 and 600 seconds")


@dataclass
class _Group:
    spec: ProcessSpec
    process: subprocess.Popen | None = None
    identity: dict | None = None
    hint: dict | None = None
    state: str = "STOPPED"
    detail: str = "Stopped; no process has been launched."
    log_path: str = ""
    started_at: float = 0.
    ready_once: bool = False
    pending: bool = False
    stopping: bool = False
    explicitly_stopped: bool = False
    uncertain: bool = False
    members: tuple = ()
    completed_logs: list = field(default_factory=list)
    reported_state: str = ""
    launch_error: str = ""


def _proc_info(pid):
    """Read Linux birth identity, not just the reusable numeric PID."""
    try:
        value = Path(f"/proc/{pid}/stat").read_text()
        tail = value[value.rindex(")") + 2:].split()
        return dict(pid=int(pid), state=tail[0], pgid=int(tail[2]),
                    session=int(tail[3]), start_time=int(tail[19]))
    except (OSError, ValueError, IndexError):
        return None


def _same_process(info, identity):
    return info is not None and all(info.get(k) == identity.get(k)
                                    for k in ("pid", "pgid", "session", "start_time"))


def _session_members(identity):
    """Return evidence of live members, or None if identity is untrusted.

    A process group remains reserved while descendants exist, even after its
    leader exits. Check session as well as pgid, and reject a reused leader PID.
    A matching live leader is sufficient evidence; normal monitoring therefore
    reads one proc stat instead of scanning every process for every group. Only
    after the leader exits do we enumerate descendants. The result is not a
    complete process-tree inventory. Zombies are not live signal targets.
    """
    leader = _proc_info(identity["pid"])
    if leader is not None and not _same_process(leader, identity):
        return None
    if leader is not None and leader["state"] not in ("Z", "X"):
        return (leader,)
    members = []
    try:
        entries = os.scandir("/proc")
        with entries:
            for entry in entries:
                if not entry.name.isdigit():
                    continue
                info = _proc_info(int(entry.name))
                if (info is not None and info["state"] not in ("Z", "X")
                        and info["pgid"] == identity["pgid"] and info["session"] == identity["session"]):
                    if info["start_time"] < identity["start_time"]:
                        return None
                    members.append(info)
    except OSError:
        return None
    return tuple(members)


class ProcessSupervisor:
    """Single-worker commands and observational readiness, with explicit ownership.

    ``observe`` returns ``{topic: {publishers: int, fresh: bool}}``. ``allowed``
    receives the full device command and must fail closed unless the recorder is
    known idle. Callbacks run outside this service's lock to avoid lock inversion
    with the capture management service. ``event`` accepts model.event's API.
    """
    LOG_LIMIT_BYTES = 2 * 1024 * 1024
    LOG_KEEP_BYTES = 1024 * 1024
    LOG_RUNS_PER_GROUP = 4
    RECEIPT_LIMIT = 256

    def __init__(self, specs, *, instance_id, state_dir, observe, allowed, event,
                 poll_interval_s=.1, interrupt_timeout_s=3., terminate_timeout_s=2., kill_timeout_s=1.):
        specs = tuple(specs)
        if len(specs) > 32 or any(not isinstance(spec, ProcessSpec) for spec in specs) or len({s.id for s in specs}) != len(specs):
            raise ValueError("device specs must have unique ids")
        if not isinstance(instance_id, str) or not instance_id:
            raise ValueError("manager instance id is required")
        timings = (poll_interval_s, interrupt_timeout_s, terminate_timeout_s, kill_timeout_s)
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) or not .01 <= v <= 30 for v in timings):
            raise ValueError("invalid device supervisor timing bounds")
        self.instance_id = instance_id
        self._observe, self._allowed, self._event = observe, allowed, event
        self._poll_interval = poll_interval_s
        self._timeouts = (interrupt_timeout_s, terminate_timeout_s, kill_timeout_s)
        self._mutex = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._commands = queue.Queue(maxsize=16)
        self._receipts = OrderedDict()
        self._groups = {spec.id: _Group(spec) for spec in specs}
        self._active = False
        self._lock_file = None
        self._registry = {}
        self._boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.directory = private_directory(state_dir)
        self.logs = private_directory(self.directory / "device-logs")
        self._registry_path = self.directory / "device-processes.json"
        try:
            fd = os.open(self.directory / "device-supervisor.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            self._lock_file = os.fdopen(fd, "r+")
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("Another device supervisor already owns this state directory") from exc
            self._load_registry()
        except BaseException:
            if self._lock_file is not None:
                self._lock_file.close()
                self._lock_file = None
            raise

    def _load_registry(self):
        if not self._registry_path.exists():
            return
        if self._registry_path.is_symlink() or self._registry_path.stat().st_size > 1024 * 1024:
            raise ValueError("Refusing unsafe device ownership registry")
        try:
            document = json.loads(self._registry_path.read_text())
            if document["version"] != 1 or not isinstance(document["groups"], dict):
                raise ValueError("Unsupported device ownership registry")
            if document["boot_id"] != self._boot_id:
                return
            for group_id, hint in document["groups"].items():
                if not isinstance(hint, dict) or not isinstance(hint.get("resources"), list):
                    raise ValueError("Invalid device ownership hint")
                identity = hint.get("identity")
                if identity is not None:
                    if (not isinstance(identity, dict) or any(type(identity.get(k)) is not int or identity[k] <= 0
                            for k in ("pid", "pgid", "session", "start_time"))
                            or identity["pid"] != identity["pgid"] or identity["pid"] != identity["session"]):
                        raise ValueError("Invalid device process identity")
                    members = _session_members(identity)
                    if members == ():
                        continue
                elif not hint.get("pending_launch"):
                    continue
                self._registry[group_id] = hint
                if group_id in self._groups:
                    group = self._groups[group_id]
                    group.hint = hint
                    group.log_path = str(hint.get("log_path", ""))
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise ValueError("Device registry is unreadable; reconcile it before launching devices") from exc

    def _save_registry(self):
        path = self.directory / (".device-processes-" + uuid.uuid4().hex + ".tmp")
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        try:
            with os.fdopen(fd, "w") as stream:
                json.dump(dict(version=1, boot_id=self._boot_id, groups=self._registry), stream, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(path, self._registry_path)
            directory_fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            path.unlink(missing_ok=True)

    def start(self):
        with self._mutex:
            if self._thread is not None or self._stop.is_set():
                raise RuntimeError("device supervisor is single-use")
            self._thread = threading.Thread(target=self._run, name="device-process-control", daemon=True)
            self._thread.start()
        return self

    @property
    def busy(self):
        with self._mutex:
            # Readiness is observational, not a recording quality gate. Only OS
            # lifecycle mutations must serialize against recording START.
            return bool(self._active or any(g.pending or g.stopping for g in self._groups.values()))

    def _observations(self):
        try:
            value = self._observe()
            if not isinstance(value, dict):
                return None
            for topic in {topic for group in self._groups.values() for topic in group.spec.topics}:
                sample = value.get(topic)
                if (not isinstance(sample, dict) or type(sample.get("publishers")) is not int
                        or sample["publishers"] < 0 or type(sample.get("fresh")) is not bool):
                    return None
            return value
        except Exception:
            return None

    def _permission(self, action):
        try:
            allowed, reason = self._allowed(action)
            return allowed is True, bounded_text(reason)
        except Exception as exc:
            return False, bounded_text(f"Recorder-idle guard unavailable: {exc}")

    def _emit(self, level, detail, group_id):
        try:
            self._event(level, bounded_text(detail), key="device-process-" + group_id)
        except Exception:
            pass  # Logging must not turn a known process side effect into a retry.

    @staticmethod
    def _publishers(group, observations):
        return observations is not None and any(observations[t]["publishers"] > 0 for t in group.spec.topics)

    def _refresh(self, observations):
        for group in self._groups.values():
            if group.process is not None:
                code = group.process.poll()
                if group.identity is not None:
                    members = _session_members(group.identity)
                    group.uncertain = members is None
                    group.members = members or ()
                else:
                    group.uncertain = code is None
                    group.members = ()
                if group.uncertain:
                    group.state, group.detail = "UNKNOWN", "Process identity cannot be verified; no signals or new launch are allowed."
                elif group.members:
                    if group.stopping:
                        group.state, group.detail = "STOPPING", "Waiting for the owned process group to exit."
                    elif code is not None:
                        group.state, group.detail = "FAILED", f"Launcher exited with code {code}; owned descendants remain. Explicit STOP is available."
                    elif observations is not None and all(observations[t]["publishers"] > 0 and observations[t]["fresh"] for t in group.spec.topics):
                        group.ready_once = True
                        group.state, group.detail = "RUNNING", "Owned process is alive and all expected topics are fresh."
                    elif not group.ready_once and time.monotonic() - group.started_at < group.spec.startup_timeout_s:
                        group.state, group.detail = "STARTING", "Process launched; waiting for fresh expected topics."
                    else:
                        group.state = "DEGRADED"
                        group.detail = "Owned process is alive but expected topics are not fresh; no automatic restart or kill."
                elif code is None:
                    group.state, group.detail = "UNKNOWN", "Launcher is alive but its owned process group cannot be verified."
                    group.uncertain = True
                elif group.explicitly_stopped:
                    group.state, group.detail = "STOPPED", "Owned process group stopped."
                else:
                    group.state, group.detail = "FAILED", f"Owned process exited with code {code}; no automatic restart."
            elif group.hint is not None:
                identity = group.hint.get("identity")
                members = _session_members(identity) if identity else None
                if members == ():
                    group.hint = None
                    self._registry.pop(group.spec.id, None)
                else:
                    group.state = "EXTERNAL" if self._publishers(group, observations) else "UNKNOWN"
                    group.detail = "A previous manager may have left this group running; it is not owned and will not be adopted or signalled."
            if group.process is None and group.hint is None:
                if group.state != "FAILED":
                    group.state, group.detail = "STOPPED", "Stopped; no owned process."
            if group.launch_error and not group.members and not group.uncertain and group.hint is None:
                group.state, group.detail = "FAILED", bounded_text("Configured process could not be launched: " + group.launch_error)
            if not group.members and not group.uncertain and group.hint is None:
                if observations is None and not group.launch_error:
                    group.state, group.detail = "UNKNOWN", "Topic observations are unavailable; launch is blocked."
                elif self._publishers(group, observations):
                    group.state, group.detail = "EXTERNAL", "Expected topics already have publishers; no owned process may be launched over them."
            if not group.spec.enabled:
                group.state, group.detail = "DISABLED", "This process group is disabled in manager configuration."

    def _start_problem(self, group, observations, *, ignore_pending=False):
        if observations is None:
            return "observations_unavailable", "Fresh graph observations are required before launching a process."
        if group.uncertain or group.hint is not None:
            return "ownership_unknown", "A prior or uncertain process cannot be adopted or relaunched."
        if group.members or (group.process is not None and group.process.poll() is None):
            return "already_running", "This manager already owns a live process group."
        if group.pending and not ignore_pending:
            return "device_busy", "An operation for this group is already queued."
        if self._publishers(group, observations):
            return "external_publishers", "Expected topics already have publishers; refusing a duplicate launch."
        resources = set(group.spec.resources)
        for other in self._groups.values():
            if other is group or not resources.intersection(other.spec.resources):
                continue
            if (other.pending or other.members or other.uncertain or other.hint is not None
                    or self._publishers(other, observations)):
                return "resource_conflict", f"Resources are already reserved by group {other.spec.id}."
        for group_id, hint in self._registry.items():
            if group_id in self._groups or not resources.intersection(hint.get("resources", ())):
                continue
            identity = hint.get("identity")
            if identity is None or _session_members(identity) != ():
                return "resource_conflict", f"Resources may still belong to previous group {group_id}."
        return serial_start_problem(group.spec.resources)

    def snapshot(self):
        observations = self._observations()
        permissions = {action: self._permission("device." + action)[0] for action in ("start", "stop", "restart")}
        with self._mutex:
            self._refresh(observations)
            available = self._thread is not None and not self._stop.is_set()
            groups = []
            for group in self._groups.values():
                owned = group.process is not None and bool(group.members) and not group.uncertain
                usable = available and group.spec.enabled and not group.pending
                problem = self._start_problem(group, observations) if usable and permissions["start"] else None
                state, detail = group.state, group.detail
                if problem is not None and problem[0] in ("serial_permission_denied", "serial_unavailable"):
                    # Present current admission evidence, not a sticky failure:
                    # reconnecting/fixing access makes the next snapshot usable.
                    # Owned groups return already_running before serial checks,
                    # so STOP and the stopping phase of RESTART stay available.
                    state, detail = "FAILED", problem[1]
                pid = group.identity["pid"] if owned and group.identity else None
                if group.hint is not None and group.hint.get("identity"):
                    pid = group.hint["identity"]["pid"]
                groups.append(dict(id=group.spec.id, label=group.spec.label, kind=group.spec.kind,
                                   state=("STOPPING" if owned else "STARTING") if group.pending else state,
                                   owned=owned, pid=pid, detail=bounded_text(detail),
                                   log_path=group.log_path, can_start=bool(usable and permissions["start"] and problem is None),
                                   can_stop=bool(usable and permissions["stop"] and owned),
                                   can_restart=bool(usable and permissions["restart"] and owned), topics=list(group.spec.topics)))
            return dict(enabled=True, groups=groups)

    @staticmethod
    def _receipt(request_id, state, code, detail):
        return dict(request_id=request_id, state=state, code=code, detail=bounded_text(detail), trial_id="")

    def _remember(self, request_id, fingerprint, receipt):
        self._receipts[request_id] = (fingerprint, receipt)
        while len(self._receipts) > self.RECEIPT_LIMIT:
            candidate = next((key for key, (_, value) in self._receipts.items() if value["code"] != "queued"), None)
            if candidate is None:
                break
            del self._receipts[candidate]
        return deepcopy(receipt)

    def command(self, command, params, request_id):
        if not isinstance(request_id, str) or not request_id or len(request_id) > 256:
            return self._receipt(str(request_id)[:256], "rejected", "invalid_request", "A bounded nonempty request id is required.")
        try:
            fingerprint = json.dumps([command, params], sort_keys=True, allow_nan=False)
        except (TypeError, ValueError):
            return self._receipt(request_id, "rejected", "invalid_request", "Invalid device command payload.")
        # Do not call recorder callbacks while holding the process lock.
        observations = self._observations()
        permitted, reason = self._permission(command)
        with self._mutex:
            prior = self._receipts.get(request_id)
            if prior is not None:
                if prior[0] != fingerprint:
                    return self._receipt(request_id, "rejected", "request_id_conflict", "This request id identifies a different device command.")
                return deepcopy(prior[1])

            def reject(code, detail):
                return self._remember(request_id, fingerprint, self._receipt(request_id, "rejected", code, detail))

            if command not in ("device.start", "device.stop", "device.restart"):
                return reject("unsupported_command", "Unknown device process command.")
            if not isinstance(params, dict) or set(params) != {"group_id", "expected_manager_instance_id"}:
                return reject("invalid_request", "Device commands accept only group_id and expected_manager_instance_id.")
            if params["expected_manager_instance_id"] != self.instance_id:
                return reject("manager_instance_mismatch", "Manager instance changed; refresh before issuing a new command.")
            group = self._groups.get(params["group_id"]) if isinstance(params["group_id"], str) else None
            if group is None:
                return reject("group_not_found", "Unknown configured device group.")
            if self._thread is None or self._stop.is_set():
                return reject("supervisor_unavailable", "Device supervisor is not running.")
            if not group.spec.enabled:
                return reject("device_disabled", "This group is disabled in manager configuration.")
            if not permitted:
                return reject("recorder_not_idle", reason or "Recorder must be known idle before changing devices.")
            self._refresh(observations)
            if group.pending:
                return reject("device_busy", "An operation for this group is already queued.")
            if command == "device.start":
                problem = self._start_problem(group, observations)
                if problem:
                    return reject(*problem)
            elif group.process is None or group.uncertain or not group.members:
                return reject("not_owned", "Only a verified process group owned by this manager may be stopped or restarted.")
            if self._commands.full():
                return reject("queue_full", "Device command queue is full.")
            group.pending = True
            self._commands.put_nowait((command, group.spec.id, request_id))
            return self._remember(request_id, fingerprint, self._receipt(request_id, "accepted", "queued", "Device command queued; process readiness is reported separately."))

    def _finish_receipt(self, request_id, state, code, detail):
        with self._mutex:
            previous = self._receipts.get(request_id)
            if previous:
                self._receipts[request_id] = (previous[0], self._receipt(request_id, state, code, detail))

    def operation(self, request_id):
        """Return the latest bounded-cache receipt, without issuing a command."""
        with self._mutex:
            previous = self._receipts.get(request_id)
            return deepcopy(previous[1]) if previous is not None else None

    def _run(self):
        while not self._stop.is_set():
            try:
                item = self._commands.get(timeout=self._poll_interval)
            except queue.Empty:
                item = None
            if item is not None:
                try:
                    self._execute(*item)
                finally:
                    self._commands.task_done()
            observations = self._observations()
            with self._mutex:
                self._refresh(observations)
                changes = [(g.spec.id, g.state, g.detail) for g in self._groups.values() if g.state != g.reported_state]
                for group in self._groups.values():
                    group.reported_state = group.state
            for group_id, state, detail in changes:
                self._emit("error" if state == "FAILED" else "warning" if state in ("DEGRADED", "UNKNOWN") else "info",
                           f"Device group {group_id}: {state}. {detail}", group_id)
            self._trim_logs()

    def _execute(self, command, group_id, request_id):
        group = self._groups[group_id]
        with self._mutex:
            self._active = True
        try:
            if self._stop.is_set():
                self._finish_receipt(request_id, "rejected", "supervisor_closed", "Supervisor closed before this command was dispatched.")
                return
            permitted, reason = self._permission(command)
            if not permitted:
                self._finish_receipt(request_id, "rejected", "recorder_not_idle", reason or "Recorder is no longer idle; no command dispatched.")
                return
            if command in ("device.stop", "device.restart"):
                self._stop_group(group)
                if self._stop.is_set():
                    self._finish_receipt(request_id, "unknown", "supervisor_closed", "Supervisor closed while stopping; inspect current processes.")
                    return
                if command == "device.stop":
                    self._finish_receipt(request_id, "accepted", "stopped", "Owned process group exited.")
                    self._emit("info", f"Device group {group_id} stopped.", group_id)
                    return
                # DDS graph removal can lag process exit. Wait a bounded period;
                # this is the same explicit restart, never an automatic retry.
                deadline = time.monotonic() + self._timeouts[1]
                while self._publishers(group, self._observations()) and time.monotonic() < deadline:
                    if self._stop.wait(self._poll_interval):
                        break
                permitted, reason = self._permission(command)
                if not permitted:
                    self._finish_receipt(request_id, "rejected", "recorder_not_idle", "Process stopped, but restart blocked: " + reason)
                    return
            observations = self._observations()
            with self._mutex:
                self._refresh(observations)
                problem = self._start_problem(group, observations, ignore_pending=True)
                if problem or self._stop.is_set():
                    code, detail = problem or ("supervisor_closed", "Supervisor closed before launch.")
                    self._finish_receipt(request_id, "rejected", code, detail)
                    return
                self._launch(group)
            self._finish_receipt(request_id, "accepted", "started", "Owned process launched; fresh expected topics are still required for RUNNING.")
            self._emit("info", f"Device group {group_id} launched (pid {group.process.pid}).", group_id)
        except Exception as exc:
            with self._mutex:
                if command in ("device.start", "device.restart") and not group.members and (group.process is None or group.process.poll() is not None):
                    group.launch_error = bounded_text(exc)
                    group.state, group.detail = "FAILED", "Configured process could not be launched: " + group.launch_error
            self._finish_receipt(request_id, "unknown", "process_error", f"Inspect process state before retrying: {exc}")
            self._emit("error", f"Device group {group_id}: {exc}", group_id)
        finally:
            with self._mutex:
                group.pending = False
                group.stopping = False
                self._active = False

    def _launch(self, group):
        if group.log_path and group.process is not None:
            group.completed_logs.append(group.log_path)
        log_path = self.logs / f"{group.spec.id}-{time.time_ns()}-{uuid.uuid4().hex[:8]}.log"
        hint = dict(manager_instance_id=self.instance_id, identity=None, pending_launch=True,
                    resources=list(group.spec.resources), log_path=str(log_path))
        # Persist uncertainty before Popen, covering a crash between fork and the
        # post-launch identity write. Such a hint never authorizes a future kill.
        self._registry[group.spec.id] = hint
        self._save_registry()
        fd = os.open(log_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        group.log_path = str(log_path)
        try:
            try:
                process = subprocess.Popen(group.spec.argv, stdin=subprocess.DEVNULL, stdout=fd, stderr=subprocess.STDOUT,
                                           close_fds=True, start_new_session=True)
            except Exception:
                self._registry.pop(group.spec.id, None)
                self._save_registry()
                group.state, group.detail = "FAILED", "The configured process could not be launched."
                raise
        finally:
            os.close(fd)
        group.process = process
        group.identity = _proc_info(process.pid)
        group.hint = None
        group.log_path = str(log_path)
        group.started_at = time.monotonic()
        group.ready_once = group.explicitly_stopped = group.uncertain = False
        group.launch_error = ""
        group.state, group.detail = "STARTING", "Process launched; waiting for fresh expected topics."
        hint.update(identity=group.identity, pending_launch=group.identity is None)
        self._save_registry()

    def _stop_group(self, group):
        with self._mutex:
            if group.process is None or group.identity is None or group.uncertain:
                raise RuntimeError("No verified owned process group is available to stop")
            group.stopping = True
            group.state = "STOPPING"
        for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), self._timeouts):
            if self._stop.is_set():
                return
            # Recheck birth identity before *each* escalation. A registry hint
            # or external publisher can never reach this signalling path.
            members = _session_members(group.identity)
            if members is None:
                with self._mutex:
                    group.uncertain = True
                raise RuntimeError("Process identity changed; refusing to signal a reused PID/group")
            if not members:
                break
            try:
                os.killpg(group.identity["pgid"], sig)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and not self._stop.is_set():
                group.process.poll()
                members = _session_members(group.identity)
                if members is None:
                    with self._mutex:
                        group.uncertain = True
                    raise RuntimeError("Process identity became uncertain during stop")
                if not members:
                    break
                self._stop.wait(min(self._poll_interval, max(0., deadline - time.monotonic())))
            if not members:
                break
        if self._stop.is_set():
            return
        if _session_members(group.identity) != ():
            raise RuntimeError("Owned process group did not exit within bounded stop time")
        group.process.poll()
        with self._mutex:
            group.explicitly_stopped = True
            group.members = ()
            group.state, group.detail = "STOPPED", "Owned process group stopped."
            self._registry.pop(group.spec.id, None)
            self._save_registry()

    def _trim_logs(self):
        """Cap active logs while attached; keep durable append fds on detach.

        No pipe is used, so a surviving device cannot receive SIGPIPE when the
        manager exits. Detached children can keep appending until stopped by
        their operator; their files are never deleted by a later manager.
        """
        with self._mutex:
            paths = tuple(g.log_path for g in self._groups.values() if g.process is not None and g.log_path)
        for path in paths:
            try:
                if Path(path).stat().st_size <= self.LOG_LIMIT_BYTES:
                    continue
                fd = os.open(path, os.O_RDWR | os.O_NOFOLLOW)
                with os.fdopen(fd, "r+b") as stream:
                    stream.seek(-self.LOG_KEEP_BYTES, os.SEEK_END)
                    tail = stream.read(self.LOG_KEEP_BYTES)
                    stream.seek(0)
                    stream.write(tail)
                    stream.truncate(self.LOG_KEEP_BYTES)
            except OSError:
                pass
        # Only collect this manager's completed-run files, never registry hints
        # or arbitrary existing logs that could still have external writers.
        for group in self._groups.values():
            known = group.completed_logs
            while len(known) > self.LOG_RUNS_PER_GROUP:
                try:
                    Path(known.pop(0)).unlink(missing_ok=True)
                except OSError:
                    pass

    def close(self):
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=sum(self._timeouts) + 2.)
            if thread.is_alive():
                raise RuntimeError("Device supervisor worker did not detach; state directory remains locked")
        with self._mutex:
            while True:
                try:
                    _, group_id, request_id = self._commands.get_nowait()
                except queue.Empty:
                    break
                self._groups[group_id].pending = False
                self._finish_receipt(request_id, "rejected", "supervisor_closed", "Supervisor closed before dispatch.")
                self._commands.task_done()
            if self._lock_file is not None:
                self._lock_file.close()
                self._lock_file = None
