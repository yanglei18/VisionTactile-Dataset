"""Managed, headless owner of one hand's existing spawned solver.

Control requests are local JSON, never pickle. A receipt confirms command
execution, while calibration completion and persistence remain in the original
snapshot. No timeout, reconnect or service restart silently repeats an action.
"""
from __future__ import annotations

from copy import deepcopy
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import socketserver
import stat
import struct
import threading
import time
from uuid import uuid4

from .hand_service_protocol import SCHEMA_VERSION, MAX_REQUEST_BYTES, MAX_RESPONSE_BYTES, encode, receive


MAX_RECEIPTS = 1024
VISIBLE_RECEIPTS = 64


def _json_default(value):
    if type(value).__module__.startswith("numpy"):
        return value.tolist()
    raise TypeError(f"Unsupported hand snapshot value: {type(value).__name__}")


def _token(value, name):
    if not isinstance(value, str) or not 1 <= len(value) <= 128 or not value.isascii():
        raise ValueError(f"Invalid {name}")
    return value


class HandService:
    """One serialized command owner; snapshots retain source clocks and quality."""

    def __init__(self, controller, *, hand, clock=time.monotonic, instance_id=None):
        if hand not in ("left", "right"):
            raise ValueError("Invalid hand")
        self.controller, self.hand, self.clock = controller, hand, clock
        self.instance_id = instance_id or uuid4().hex
        self._lock = threading.Lock()
        self._snapshot = {}
        self._receipts = {}
        self._fingerprints = {}
        self._pending = None
        self._initialized = False
        self._stopping = False
        self._service_state = "starting"

    def refresh(self):
        with self._lock:
            self._refresh()

    def _refresh(self):
        self._snapshot = self.controller.latest_snapshot()
        runtime = self._snapshot.get("runtime") or {}
        self._initialized |= runtime.get("ipc_sequence", 0) > 0
        alive = self.controller.is_alive()
        self._service_state = ("stopping" if self._stopping else "failed" if not alive
                               else "ready" if self._initialized else "starting")
        if self._pending is None:
            return
        receipt = self._receipts[self._pending["request_id"]]
        name = receipt["command"]
        # Input mailbox acknowledgements keep working while mesh generation is
        # slow. They come from the same command executor and carry its counters.
        candidates = [runtime, (self._snapshot.get("input_snapshot") or {}).get("runtime") or {}]
        for observed in candidates:
            if (observed.get("command_rejected_count", 0) > self._pending["rejected"]
                    and observed.get("last_rejected_command_name") == name):
                receipt.update(state="rejected", error=observed.get("last_rejected_command_error"))
                break
            if (observed.get("command_completed_generation", 0) > self._pending["generation"]
                    and observed.get("last_command_name") == name):
                error = observed.get("last_command_error")
                if error is None:
                    from .live_process import SHAPE_COMMAND_ARGS
                    if name in SHAPE_COMMAND_ARGS:
                        error = observed.get("shape_command_error")
                receipt.update(state="rejected" if error else "executed", error=error)
                break
        if receipt["state"] != "queued":
            receipt["completed_monotonic_ns"] = time.monotonic_ns()
            self._pending = None
        elif not alive:
            receipt.update(state="unknown", error="Compute process exited before execution could be confirmed")
            self._pending = None

    def _envelope(self):
        valid = False
        config = getattr(self.controller, "config", None)
        if config is not None:
            from .pose_export import build_state
            try:
                root = next(node for node, slot in config["node_to_slot"].items() if slot == 0)
                state = build_state(self._snapshot.get("orientation") or {}, self._snapshot.get("pose") or {},
                    None, config, root_node=root, solve_completed_monotonic_ns=0)
                valid = bool(state["pose_valid"] and self._snapshot.get("runtime", {}).get("connected"))
            except (KeyError, StopIteration, TypeError, ValueError):
                valid = False
        activities = [runtime.get("calibration_activity") for runtime in
            [self._snapshot.get("runtime") or {},
             (self._snapshot.get("input_snapshot") or {}).get("runtime") or {}]]
        activities = [value for value in activities if isinstance(value, dict)]
        activity = max(activities, key=lambda value: value.get("observed_monotonic_s", 0), default=None)
        busy = (self._service_state != "ready" or self._pending is not None
                or activity is None or activity.get("busy") is not False)
        return dict(schema_version=SCHEMA_VERSION, service_instance_id=self.instance_id,
                    hand=self.hand, service_state=self._service_state, pose_valid=valid,
                    calibration_busy=busy, calibration_activity=deepcopy(activity),
                    snapshot_monotonic_ns=time.monotonic_ns())

    def handle(self, document):
        with self._lock:
            self._refresh()
            response = {}
            try:
                if document.get("schema_version") != SCHEMA_VERSION or type(document.get("schema_version")) is not int:
                    raise ValueError("Unsupported hand service request version")
                if document.get("hand") != self.hand:
                    raise ValueError("Hand service request belongs to another hand")
                op = document.get("op")
                if op == "snapshot":
                    if set(document) - {"schema_version", "hand", "op", "expected_service_instance_id"}:
                        raise ValueError("Unknown snapshot fields")
                    expected = document.get("expected_service_instance_id", self.instance_id)
                    if expected != self.instance_id:
                        raise ValueError("Hand service instance changed")
                    response.update(snapshot=deepcopy(self._snapshot),
                        receipts=deepcopy(list(self._receipts.values())[-VISIBLE_RECEIPTS:]))
                elif op in {"command", "receipt"}:
                    if document.get("expected_service_instance_id") != self.instance_id:
                        raise ValueError("Hand service instance changed; inspect status before submitting")
                    request_id = _token(document.get("request_id"), "request_id")
                    if op == "receipt":
                        if set(document) != {"schema_version", "hand", "op", "expected_service_instance_id", "request_id"}:
                            raise ValueError("Invalid receipt request fields")
                        if request_id not in self._receipts:
                            raise ValueError("Unknown request_id in this service instance")
                        receipt = self._receipts[request_id]
                    else:
                        receipt = self._command(document, request_id)
                    response["receipt"] = deepcopy(receipt)
                else:
                    raise ValueError("Unsupported hand service operation")
                response["ok"] = True
            except (TypeError, ValueError) as exc:
                response.update(ok=False, error=str(exc)[:4096])
            response.update(self._envelope())
            return response

    def _command(self, document, request_id):
        required = {"schema_version", "hand", "op", "expected_service_instance_id", "request_id",
                    "command", "args", "expected_session_id", "expected_profile_version"}
        if set(document) != required:
            raise ValueError("Command requires exact service, session and profile bindings")
        name = _token(document["command"], "command")
        args = document["args"]
        if not isinstance(args, dict):
            raise ValueError("Command args must be an object")
        session, version = document["expected_session_id"], document["expected_profile_version"]
        if session is not None:
            _token(session, "expected_session_id")
        if version is not None and (type(version) is not int or version < 1):
            raise ValueError("Invalid expected_profile_version")
        fingerprint = hashlib.sha256(json.dumps(document, sort_keys=True, allow_nan=False).encode()).hexdigest()
        if request_id in self._receipts:
            if fingerprint != self._fingerprints[request_id]:
                raise ValueError("request_id was already used with different arguments")
            return self._receipts[request_id]
        if len(self._receipts) >= MAX_RECEIPTS:
            raise ValueError("Command receipt capacity reached; no new operation was submitted")
        runtime = self._snapshot.get("runtime") or {}
        calibration = self._snapshot.get("calibration") or {}
        receipt = dict(request_id=request_id, command=name, state="rejected", error=None,
                       submitted_monotonic_ns=time.monotonic_ns())
        # Retain rejected IDs too: replaying a rejected old request cannot turn
        # it into a new action after a session or calibration has changed.
        self._fingerprints[request_id] = fingerprint
        self._receipts[request_id] = receipt
        if self._service_state != "ready":
            receipt["error"] = "Hand service is not ready"
        elif self._pending is not None:
            receipt["error"] = "A previous command is awaiting execution; inspect its receipt"
        elif document["expected_session_id"] != runtime.get("session_id"):
            receipt["error"] = "Input session changed"
        elif document["expected_profile_version"] != calibration.get("profile_version"):
            receipt["error"] = "Calibration version changed"
        elif not self.controller.submit_bound(name, args, expected_session_id=session, expected_profile_version=version):
            receipt["error"] = "Command unavailable or bounded command queue full"
        else:
            receipt["state"] = "queued"
            input_runtime = (self._snapshot.get("input_snapshot") or {}).get("runtime") or {}
            self._pending = dict(request_id=request_id,
                generation=max(runtime.get("command_completed_generation", 0), input_runtime.get("command_completed_generation", 0)),
                rejected=max(runtime.get("command_rejected_count", 0), input_runtime.get("command_rejected_count", 0)))
        return receipt

    def stop(self):
        with self._lock:
            self._stopping = True
            self.controller.stop()


class _RequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(.5)
        try:
            if hasattr(socket, "SO_PEERCRED"):
                _, uid, _ = struct.unpack("3i", self.request.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                if uid != os.getuid():
                    return
            document = receive(self.request, MAX_REQUEST_BYTES)
            response = self.server.service.handle(document)
            # NumPy arrays are only numerical display buffers. Wire JSON still
            # rejects nonfinite numbers and arbitrary Python objects.
            response = json.loads(json.dumps(response, allow_nan=False, default=_json_default))
            self.request.sendall(encode(response, MAX_RESPONSE_BYTES))
        except (OSError, ValueError, TypeError, ConnectionError):
            return


class HandServiceServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    request_queue_size = 4

    def __init__(self, path, service):
        self.service = service
        self._slots = threading.BoundedSemaphore(4)
        super().__init__(str(path), _RequestHandler)

    def process_request(self, request, client_address):
        if not self._slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self._slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._slots.release()


def _claim_socket(path):
    """Lock one endpoint before removing only an owned, abandoned socket."""
    if not path.is_absolute() or len(os.fsencode(path)) > 100:
        raise ValueError("Control socket must be absolute and at most 100 bytes")
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    lock = os.open(str(path) + ".lock", os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(lock)
        if info.st_uid != os.getuid() or not stat.S_ISREG(info.st_mode):
            raise ValueError("Control socket lock is not an owned regular file")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if path.exists() or path.is_symlink():
            info = path.lstat()
            if info.st_uid != os.getuid() or not stat.S_ISSOCK(info.st_mode):
                raise ValueError("Refusing to replace a non-socket control path")
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                probe.settimeout(.2)
                try:
                    probe.connect(str(path))
                except OSError as exc:
                    if exc.errno not in (errno.ECONNREFUSED, errno.ENOENT):
                        raise
                else:
                    raise ValueError("Control socket already has a live owner")
            path.unlink(missing_ok=True)
        return lock
    except BaseException:
        os.close(lock)
        raise


def run_service(args):
    from .hand_service_resources import load_service_resources
    from .live_process import ProcessLiveController

    resources = load_service_resources(hand=args.hand, resources=args.resources, config=args.config,
        rest=args.rest, mano_assets=args.mano_assets, fsglove_source=args.fsglove_source,
        output_rate_hz=args.output_rate_hz)
    controller = ProcessLiveController(**resources, output_root=args.output_root,
        ros_socket=args.ros_socket, record_streams=args.record_streams,
        replay_raw=args.replay_raw, replay_speed=args.replay_speed)
    path = Path(args.control_socket)
    lock = _claim_socket(path)
    server = None
    worker = None
    stop = threading.Event()
    previous_signals = {}
    service = HandService(controller, hand=args.hand)
    try:
        server = HandServiceServer(path, service)
        os.chmod(path, 0o600)
        controller.start()
        worker = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .05}, daemon=True)
        worker.start()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_signals[signum] = signal.signal(signum, lambda *_: stop.set())
        autoconnect = bool(args.port)
        while not stop.wait(1. / 60.):
            service.refresh()
            if autoconnect and service._service_state == "ready":
                state = service.handle(dict(schema_version=1, op="snapshot", hand=args.hand))
                snapshot = state["snapshot"]
                service.handle(dict(schema_version=1, op="command", hand=args.hand,
                    expected_service_instance_id=service.instance_id, request_id="startup-connect",
                    command="connect", args={"port": args.port},
                    expected_session_id=snapshot["runtime"].get("session_id"),
                    expected_profile_version=snapshot.get("calibration", {}).get("profile_version")))
                autoconnect = False
            if service._service_state == "failed":
                # Keep a final observable failure briefly; no invisible restart.
                stop.wait(.25)
                return 1
        return 0
    finally:
        service.stop()
        if server is not None:
            if worker is not None and worker.is_alive():
                server.shutdown()
            server.server_close()
        # Keep ownership while the original reader and recorder close. Signal
        # handling stays cooperative, as in the standalone application.
        while not controller.join(timeout=.1):
            pass
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)
        path.unlink(missing_ok=True)
        os.close(lock)
