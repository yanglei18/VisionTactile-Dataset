"""Scene acquisition is read-only and bound to the control Manager identity."""
import pytest
import socket

from vt_capture_console.backend.scene_backend import SceneBackend
from vt_capture_console.backend.scene_demo import demo_scene


def test_backend_rejects_old_manager_and_clears_cache_on_change(monkeypatch, tmp_path):
    backend = SceneBackend(tmp_path / "manager-scene.sock", autostart=False)
    backend.set_interest("main", frozenset({"tracker", "camera"}))
    backend.set_manager_instance("demo-manager")
    commands = []

    def exchange(command, params):
        commands.append(command)
        frame = demo_scene(0)
        frame["mode"] = "live"
        return {"hello": {"manager_instance_id": "demo-manager"}} if command == "hello" else {
            "description": {}} if command == "describe" else {"scene": frame}

    monkeypatch.setattr(backend, "_request", exchange)
    backend.poll_once()
    assert backend.latest()["manager_instance_id"] == "demo-manager"
    assert set(commands) <= {"hello", "describe", "latest"}
    backend.set_manager_instance("different-manager")
    assert backend.latest() is None
    with pytest.raises(ValueError, match="实例"):
        backend.poll_once()
    backend.close()


def test_without_interest_or_verified_control_no_poll(monkeypatch, tmp_path):
    backend = SceneBackend(tmp_path / "manager-scene.sock", autostart=False)
    monkeypatch.setattr(backend, "_request", lambda *args: pytest.fail("unexpected poll"))
    backend.poll_once()
    backend.set_interest("main", frozenset({"tracker"}))
    backend.poll_once()
    backend.close()


@pytest.mark.parametrize("capabilities", [[], ["hand", "camera"]])
def test_hand_interest_negotiation_and_expiry_on_stalled_source(monkeypatch, tmp_path, capabilities):
    from vt_capture_console.backend import scene_backend
    backend = SceneBackend(tmp_path / "scene.sock", expected_instance_id="demo-manager", autostart=False)
    frame = demo_scene(0)
    frame.update(mode="live", generated_realtime_ns=1_100_000_000)
    hand = dict(id="left", hand="left", label="左手", camera_id="head", valid=True,
                state="fresh", reason="", source_realtime_ns=1_000_000_000,
                position_m=[0., 0., .5], quaternion_xyzw=[0., 0., 0., 1.], joint_positions_m=[])
    clock = [10_000_000_000]
    monkeypatch.setattr(scene_backend.time, "monotonic_ns", lambda: clock[0])
    def exchange(command, params):
        if command == "hello":
            return {"hello": {"manager_instance_id": "demo-manager", "capabilities": capabilities}}
        if command == "describe":
            return {"description": {}}
        assert ("hand" in params["interest"]) == ("hand" in capabilities)
        return {"scene": dict(frame, hands=[hand]) if "hand" in capabilities else frame}
    monkeypatch.setattr(backend, "_request", exchange)
    try:
        backend.set_interest("main", {"camera", "hand"})
        backend.poll_once()
        if "hand" in capabilities:
            assert backend.latest()["hands"][0]["valid"]
            clock[0] += 151_000_000
            expired = backend.latest()["hands"][0]
            assert not expired["valid"] and expired["position_m"] is None and not expired["joint_positions_m"]
            assert hand["valid"]
        else:
            assert "hands" not in backend.latest()
    finally:
        backend.close()


def test_real_scene_wire_and_same_process_peer_identity(tmp_path):
    from vt_capture_manager.spatial.config import SceneConfig
    from vt_capture_manager.spatial.scene_server import SceneServer
    from vt_capture_manager.spatial.scene_store import SceneStore
    scene_path, control_path = tmp_path / "manager-scene.sock", tmp_path / "manager-control.sock"
    control = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    control.bind(str(control_path))
    control.listen(2)
    store = SceneStore("manager-test", SceneConfig())
    server = SceneServer(store, scene_path).start()
    backend = SceneBackend(scene_path, control_path=control_path, autostart=False)
    try:
        backend.set_manager_instance("manager-test")
        backend.set_interest("test", frozenset({"tracker", "camera", "axes", "grid", "trail", "frustum"}))
        backend.poll_once()
        assert backend.latest()["manager_instance_id"] == "manager-test"
        assert len(backend.latest()["trackers"]) == 3
    finally:
        backend.close()
        server.close()
        control.close()
