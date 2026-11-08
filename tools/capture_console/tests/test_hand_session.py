"""No-hardware checks of the embedded hand's process and command boundaries."""
from copy import deepcopy
from types import SimpleNamespace
import threading
import time

import pytest

from vt_capture_console.backend.hand_session import RemoteHandSession, TactileSubscription


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.005)
    assert predicate()


class Manager:
    supports_hand_control = True

    def __init__(self):
        self.value = dict(instance_id="manager", hand_sessions={"left": dict(service_instance_id="service")})
        self.commands, self.results = [], {}

    def snapshot(self):
        return deepcopy(self.value)

    def submit(self, command, params):
        key = f"request-{len(self.commands)}"
        self.commands.append((command, params))
        self.results[key] = dict(state="unknown", code="QUEUED")
        return key

    def operation_result(self, key):
        return self.results.get(key)


def reply():
    return dict(hand="left", service_instance_id="service", service_state="running", snapshot=dict(
        runtime=dict(connected=True, session_id="capture-session", command_completed_generation=3),
        calibration=dict(profile_version=4), pose=dict(valid=True), geometry=dict(vertices_m=[[0., 1., 2.]])))


def test_slow_service_io_runs_outside_the_gui_and_close_only_detaches():
    entered, release = threading.Event(), threading.Event()
    calls = []
    def request(path, document, timeout):
        calls.append((threading.get_ident(), document))
        entered.set()
        assert release.wait(2)
        return reply()
    client = RemoteHandSession("left", "/test.sock", Manager(), request=request)
    client.start()
    assert entered.wait(2)
    assert client.latest_snapshot()["runtime"]["connected"] is False
    client.close()
    assert calls[0][0] != threading.get_ident()
    assert calls[0][1]["op"] == "snapshot"
    release.set()
    wait_for(lambda: not client.is_alive())
    assert len(calls) == 1


def test_commands_only_enter_manager_with_current_service_session_and_profile():
    manager, calls = Manager(), []
    def request(path, document, timeout):
        calls.append(document)
        return reply()
    client = RemoteHandSession("left", "/test.sock", manager, request=request)
    client.start()
    try:
        wait_for(lambda: client.service_instance_id == "service")
        assert client.submit("capture", step="neutral")
        command, params = manager.commands[0]
        assert command == "hand.command"
        assert params == dict(hand="left", name="capture", args=dict(step="neutral"),
            expected_manager_instance_id="manager", expected_service_instance_id="service",
            expected_session_id="capture-session", expected_profile_version=4)
        assert not client.submit("connect", port="/dev/forbidden")
        assert not client.submit("disconnect")
        assert {item["op"] for item in calls} == {"snapshot"}
        manager.results["request-0"] = dict(state="rejected", code="recording_busy", detail="Recording in progress")
        snapshot = client.latest_snapshot()
        assert snapshot["runtime"]["command_rejected_count"] == 1
        assert snapshot["runtime"]["last_rejected_command_name"] == "capture"
        assert snapshot["runtime"]["last_rejected_command_error"] == "Recording in progress"
    finally:
        client.close()
        wait_for(lambda: not client.is_alive())


def test_service_failure_hides_old_geometry_and_service_mismatch_rejects_command():
    manager, fail = Manager(), threading.Event()
    def request(path, document, timeout):
        if fail.is_set():
            raise ConnectionError("service stopped")
        return reply()
    client = RemoteHandSession("left", "/test.sock", manager, request=request, interval=.01)
    client.start()
    try:
        wait_for(lambda: client.service_instance_id == "service")
        snapshot = client.latest_snapshot()
        snapshot["geometry"]["vertices_m"][0][0] = 999
        assert client.latest_snapshot()["geometry"]["vertices_m"][0][0] == 0
        manager.value["hand_sessions"]["left"]["service_instance_id"] = "replacement"
        assert not client.submit("capture", step="neutral")
        fail.set()
        wait_for(lambda: client.latest_snapshot()["runtime"].get("remote_stale"))
        assert client.latest_snapshot()["geometry"] is None
        assert not client.submit("clear_calibration")
        assert not manager.commands
    finally:
        client.close()
        wait_for(lambda: not client.is_alive())


def test_service_restart_releases_pending_command_without_a_manager_receipt():
    manager, restart, unavailable = Manager(), threading.Event(), threading.Event()
    def request(path, document, timeout):
        if unavailable.is_set():
            raise ConnectionError("transport unavailable")
        value = reply()
        if restart.is_set():
            value["service_instance_id"] = "replacement"
        return value
    client = RemoteHandSession("left", "/test.sock", manager, request=request, interval=.01)
    client.start()
    try:
        wait_for(lambda: client.service_instance_id == "service")
        assert client.submit("capture", step="neutral")
        manager.results.clear()  # Manager restart discarded old operation IDs.
        unavailable.set()
        wait_for(lambda: client.latest_snapshot()["runtime"].get("remote_stale"))
        assert len(client._pending) == 1  # No new identity: outcome remains unknown.
        restart.set()
        unavailable.clear()
        wait_for(lambda: client.service_instance_id == "replacement")
        snapshot = client.latest_snapshot()
        assert not client._pending
        assert snapshot["runtime"]["command_rejected_count"] == 1
        assert snapshot["runtime"]["last_rejected_command_name"] == "capture"
        assert snapshot["runtime"]["last_rejected_command_error"] == "Hand session changed"
        assert len(manager.commands) == 1  # No automatic mutation replay.
    finally:
        client.close()
        wait_for(lambda: not client.is_alive())


def test_tactile_display_receiver_rejects_other_hand_and_releases_only_socket():
    closed = []
    class Receiver:
        def __init__(self, path, hand):
            assert (path, hand) == ("/private.sock", "left")
        def receive_latest(self): return SimpleNamespace(hand="right")
        def close(self): closed.append(True)
    subscription = TactileSubscription("left", "/private.sock", receiver_factory=Receiver)
    subscription.start()
    wait_for(lambda: bool(closed))
    assert "another hand" in subscription.error
    assert subscription.latest() == (0, None)
    subscription.close()


def test_resource_reload_coalesces_and_discards_obsolete_result():
    from vt_capture_console.backend.hand_session import HandResourceLoader
    entered, release, completed = threading.Event(), threading.Event(), threading.Event()
    calls = []
    def load(root, hand):
        calls.append(root)
        if root == "old":
            entered.set()
            assert release.wait(2)
        if root == "latest":
            completed.set()
        return dict(root=root, hand=hand)
    loader = HandResourceLoader("left", load)
    try:
        loader.request("old")
        assert entered.wait(2)
        loader.request("intermediate")
        generation = loader.request("latest")
        release.set()
        assert completed.wait(2)
        wait_for(lambda: loader._result is not None)
        assert loader.take_result() == (generation, dict(root="latest", hand="left"), "")
        assert calls == ["old", "latest"]
    finally:
        release.set()
        loader.close()


def test_tactile_usb_identity_is_resolved_only_in_the_resource_worker(monkeypatch):
    from vt_capture_console.backend.hand_session import HandResourceLoader
    from vt_capture_console.backend import tactile_reference_store
    identity = tactile_reference_store.TactileDeviceIdentity(123, 456, "test-left")
    calls = []
    def resolve(port):
        calls.append((port, threading.get_ident()))
        return identity
    monkeypatch.setattr(tactile_reference_store, "device_identity_for_port", resolve)
    loader = HandResourceLoader("left", lambda root, hand: {"hand": hand})
    try:
        generation = loader.request("bundle", tactile_port="/configured/tactile")
        wait_for(lambda: loader._result is not None)
        assert loader.take_result() == (generation, {"hand": "left", "tactile_device_identity": identity}, "")
        assert calls == [("/configured/tactile", loader._thread.ident)]
        assert calls[0][1] != threading.get_ident()
    finally:
        loader.close()
