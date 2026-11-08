"""Bounded read-only hand subscriptions and manager-admitted mutations.

This client owns no acquisition process or serial port. Closing it only stops
its local polling. Each hand has one reader shared by its embedded panel.
"""
from __future__ import annotations

from copy import deepcopy
import threading
import time


def _idle(detail="Waiting for the managed hand service"):
    return {"runtime": {"connected": False, "message": detail, "error": "",
                        "remote_stale": True}, "calibration": {}, "pose": None,
            "geometry": None, "input_snapshot": None}


class RemoteHandSession:
    """ProcessLiveController's presentation API, backed by a private service."""

    renders_geometry = True

    def __init__(self, hand, socket_path, backend, *, request=None, timeout=.25,
                 interval=1 / 30, stale_after=.5):
        if hand not in ("left", "right"):
            raise ValueError("hand must be left or right")
        if request is None:
            from vt_fsglove12.hand_service_protocol import request
        self.hand, self.socket_path, self.backend = hand, str(socket_path), backend
        self._request, self.timeout = request, timeout
        self.interval, self.stale_after = interval, stale_after
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._snapshot, self._received, self._instance = _idle(), None, None
        self._error, self._state = "", "unavailable"
        self._pending = {}
        self._rejected = 0
        self._last_rejected = None

    def start(self):
        if self._stop.is_set():
            raise RuntimeError("Closed hand client cannot restart")
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name=f"hand-preview-{self.hand}", daemon=True)
            self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                reply = self._request(self.socket_path, {"schema_version": 1, "op": "snapshot", "hand": self.hand},
                                      timeout=self.timeout)
                if (reply.get("hand") != self.hand or not reply.get("service_instance_id")
                        or not isinstance(reply.get("snapshot"), dict)):
                    raise ValueError("Hand service identity or snapshot is invalid")
                snapshot = reply["snapshot"]
                if not isinstance(snapshot.get("runtime"), dict) or not isinstance(snapshot.get("calibration"), dict):
                    raise ValueError("Hand service runtime/calibration is invalid")
                with self._lock:
                    self._snapshot = snapshot
                    self._instance = reply["service_instance_id"]
                    self._received = time.monotonic()
                    self._state, self._error = reply.get("service_state", "unknown"), ""
            except (OSError, ValueError, TypeError, KeyError, RuntimeError) as exc:
                with self._lock:
                    self._error = str(exc)[:1000]
                    self._received = None
            self._stop.wait(max(.005, self.interval - (time.monotonic() - started)))

    @property
    def service_instance_id(self):
        with self._lock:
            return self._instance

    def latest_snapshot(self):
        with self._lock:
            fresh = (not self._stop.is_set() and self._received is not None
                     and 0 <= time.monotonic() - self._received <= self.stale_after)
            snapshot = deepcopy(self._snapshot) if fresh else _idle(self._error or "Hand service is not available")
            instance = self._instance
        # Manager rejection must release the original UI's click latch even
        # when the compute worker never received the command.
        runtime = snapshot["runtime"]
        for identifier, pending in tuple(self._pending.items()):
            result = self.backend.operation_result(identifier)
            code = str((result or {}).get("code", "")).lower()
            unknown = result and result.get("state") == "unknown" and code not in {"queued", "pending"}
            instance_changed = instance is not None and pending["instance"] != instance
            if instance_changed or (result and (result.get("state") == "rejected" or unknown)):
                self._rejected += 1
                detail = "Hand session changed" if instance_changed else result.get("detail", "Command rejected")
                self._last_rejected = (pending["name"], detail,
                                       runtime.get("command_rejected_count", 0))
                self._pending.pop(identifier)
            elif (runtime.get("command_completed_generation", 0) > pending["generation"]
                  and runtime.get("last_command_name") == pending["name"]):
                self._pending.pop(identifier)
        worker_rejected = runtime.get("command_rejected_count", 0)
        runtime["command_rejected_count"] = worker_rejected + self._rejected
        if self._last_rejected is not None and worker_rejected <= self._last_rejected[2]:
            runtime["last_rejected_command_name"], runtime["last_rejected_command_error"] = self._last_rejected[:2]
        runtime["remote_service_instance_id"] = instance
        return snapshot

    def latest_input_snapshot(self):
        return self.latest_snapshot().get("input_snapshot")

    def submit(self, name, **args):
        if name in {"connect", "disconnect"} or len(self._pending) >= 8:
            return False
        if not getattr(self.backend, "supports_hand_control", False):
            return False
        with self._lock:
            if (self._stop.is_set() or self._received is None
                    or time.monotonic() - self._received > self.stale_after):
                return False
            instance, snapshot = self._instance, deepcopy(self._snapshot)
        manager = self.backend.snapshot()
        metadata = manager.get("hand_sessions", {}).get(self.hand, {})
        if metadata.get("service_instance_id") != instance:
            return False
        runtime, calibration = snapshot["runtime"], snapshot["calibration"]
        params = dict(hand=self.hand, name=name, args=args,
                      expected_manager_instance_id=manager["instance_id"],
                      expected_service_instance_id=instance,
                      expected_session_id=runtime.get("session_id"),
                      expected_profile_version=calibration.get("profile_version"))
        identifier = self.backend.submit("hand.command", params)
        self._pending[identifier] = dict(name=name, instance=instance,
                                        generation=runtime.get("command_completed_generation", 0))
        return True

    def close(self):
        self._stop.set()

    def is_alive(self):
        return self._thread is not None and self._thread.is_alive()


class TactileSubscription:
    """One private ROS display receiver; never an alternative serial reader."""

    def __init__(self, hand, socket_path, *, receiver_factory=None):
        if receiver_factory is None:
            from vt_tactile_capture.display_transport import LocalTactileReceiver
            receiver_factory = LocalTactileReceiver
        self.hand, self.socket_path, self._factory = hand, socket_path, receiver_factory
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._thread, self._latest, self._sequence = None, None, 0
        self.error = ""

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name=f"hand-touch-{self.hand}", daemon=True)
            self._thread.start()

    def _run(self):
        receiver = None
        try:
            receiver = self._factory(self.socket_path, hand=self.hand)
            while not self._stop.is_set():
                sample = receiver.receive_latest()
                if sample is not None:
                    if getattr(sample, "hand", None) != self.hand:
                        raise ValueError("Tactile state belongs to another hand")
                    with self._lock:
                        self._latest, self._sequence = sample, self._sequence + 1
                self._stop.wait(.01)
        except (OSError, ValueError, RuntimeError) as exc:
            self.error = str(exc)[:1000]
        finally:
            if receiver is not None:
                receiver.close()

    def latest(self):
        with self._lock:
            return self._sequence, deepcopy(self._latest)

    def close(self):
        self._stop.set()


class TactileIdentityResolver:
    """Resolve one port's USB metadata off Qt, coalescing session changes."""

    def __init__(self, resolver=None):
        if resolver is None:
            from .tactile_reference_store import device_identity_for_port
            resolver = device_identity_for_port
        self._resolver = resolver
        self._condition = threading.Condition()
        self._job = self._result = None
        self._closed = False
        self._generation = 0
        self._thread = threading.Thread(target=self._run, name="tactile-usb-identity", daemon=True)
        self._thread.start()

    def request(self, session, port):
        with self._condition:
            self._generation += 1
            self._job = (self._generation, session, port)
            self._result = None
            self._condition.notify()

    def take_result(self):
        with self._condition:
            result, self._result = self._result, None
            return result

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._job is not None)
                if self._closed:
                    return
                generation, session, port = self._job
                self._job = None
            try:
                identity, error = self._resolver(port) if port else None, ""
            except (OSError, ValueError, RuntimeError) as exc:
                identity, error = None, str(exc)[:1000]
            with self._condition:
                if not self._closed and generation == self._generation:
                    self._result = (session, identity, error)

    def close(self):
        with self._condition:
            self._closed = True
            self._job = self._result = None
            self._condition.notify()


class HandResourceLoader:
    """Serialize model checks and coalesce configuration changes to one job."""

    def __init__(self, hand, loader=None):
        self.hand, self.loader = hand, loader
        self._condition = threading.Condition()
        self._closed, self._request, self._result = False, None, None
        self._generation = 0
        self._cancelled = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"hand-resources-{hand}", daemon=True)
        self._thread.start()

    def request(self, root, interpreter=None, tactile_port=None):
        with self._condition:
            self._generation += 1
            self._cancelled.set()
            self._cancelled = threading.Event()
            self._request = (self._generation, root, interpreter, tactile_port, self._cancelled) if root else None
            self._result = None
            self._condition.notify()
            return self._generation

    def take_result(self):
        with self._condition:
            result, self._result = self._result, None
            return result

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._request is not None)
                if self._closed:
                    return
                generation, root, interpreter, tactile_port, cancelled = self._request
                self._request = None
            try:
                loader = self.loader
                if loader is None:
                    from .hand_resources import load_resources
                    loader = load_resources
                resources, error = loader(root, hand=self.hand), ""
                if tactile_port is not None:
                    from .tactile_reference_store import device_identity_for_port
                    resources["tactile_device_identity"] = (device_identity_for_port(tactile_port)
                                                           if tactile_port else None)
                if interpreter is not None:
                    from .hand_demonstration import load_demonstration
                    resources["demo_clips"] = load_demonstration(interpreter, root, self.hand,
                        resources["neutral_geometry"], cancelled=cancelled)
            except Exception as exc:
                resources, error = None, str(exc)[:2000]
            with self._condition:
                if not self._closed and generation == self._generation:
                    self._result = (generation, resources, error)

    def close(self):
        with self._condition:
            self._closed = True
            self._cancelled.set()
            self._request = self._result = None
            self._condition.notify()
