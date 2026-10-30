import copy
import json
import os
import socket
import threading
from types import SimpleNamespace

import pytest

from vt_capture_console.backend import vivehub_startup as module
from vt_capture_console.backend.vivehub_startup import ViveHubStartup


def snapshot(state="STOPPED", *, valid=0, instance="manager-one", owned=False, can_start=True):
    topics = [f"/vive/{role}/sample" for role in ("torso", "left_wrist", "right_wrist")]
    return dict(instance_id=instance, process_control=dict(groups=[dict(
        id="trackers", state=state, owned=owned, can_start=can_start, topics=topics)]),
        devices=[dict(topic=topic, status="online", valid_hz=125. if i < valid else 0.)
                 for i, topic in enumerate(topics)], management=dict(operation=None))


@pytest.fixture
def startup(tmp_path, monkeypatch):
    # No test may launch a real device or touch the user's configuration.
    monkeypatch.setattr(module.subprocess, "Popen", lambda *a, **k: pytest.fail("unexpected real process"))
    root = tmp_path / "VIVE Hub"
    root.mkdir()
    for name in ("run-vivehub.sh", "vivehub", "tracker_server"):
        (root / name).touch(mode=0o700)
    profile = dict(id="trackers", preset="trackers", enabled=True, parameters=dict(
        backend="vivehub", hub_socket_path=str(tmp_path / "hub.sock")))
    runtime = SimpleNamespace(settings_path=tmp_path / "workstation.json",
                              socket_path=tmp_path / "manager.sock",
                              configuration=dict(process_groups=[profile]),
                              fresh_snapshot=lambda: snapshot())
    startup = ViveHubStartup(runtime)
    startup.TIMEOUT_S = .04
    startup.POLL_S = .001
    startup.save(dict(enabled=True, install_dir=str(root)))
    startup.root, startup.profile = root, profile
    yield startup
    startup.close()
    if startup._thread:
        startup._thread.join(2)


def finish(startup):
    startup._thread.join(2)
    assert not startup._thread.is_alive()
    return startup.status


def test_default_and_disabled_never_connect(startup, monkeypatch):
    startup.path.unlink()
    monkeypatch.setattr(startup, "_run", lambda *a: pytest.fail("must remain opt-in"))
    assert startup.settings == dict(enabled=False, install_dir="")
    assert startup.start(automatic=True) is False
    startup.save(dict(enabled=False, install_dir=str(startup.root)))
    assert startup.start(automatic=True) is False


def test_cold_start_hub_before_one_bridge_command(startup, monkeypatch):
    actions, ready = [], [False]
    state = [snapshot()]
    monkeypatch.setattr(module, "hub_ready", lambda *a: ready[0])
    monkeypatch.setattr(module, "hub_processes", lambda *a: [])
    def launch(root):
        actions.append("hub")
        ready[0] = True
    monkeypatch.setattr(startup, "_launch_hub", launch)
    startup.runtime.fresh_snapshot = lambda: copy.deepcopy(state[0])
    def exchange(path, request, timeout):
        assert ready[0] and request["command"] == "device.start"
        assert request["params"] == dict(group_id="trackers", expected_manager_instance_id="manager-one")
        actions.append("bridge")
        state[0] = snapshot("RUNNING", valid=3, owned=True)
        return dict(ok=True, operation=dict(state="accepted", code="queued"))
    monkeypatch.setattr(module, "_exchange", exchange)
    assert startup.start(automatic=True)
    assert finish(startup)["state"] == "ready"
    assert actions == ["hub", "bridge"]
    assert json.loads(startup.path.with_name("vivehub-startup-status.json").read_text())["state"] == "ready"


@pytest.mark.parametrize("state,owned", [("RUNNING", True), ("EXTERNAL", False)])
def test_reuse_existing_hub_and_bridge_without_spawn(startup, monkeypatch, state, owned):
    monkeypatch.setattr(module, "hub_ready", lambda *a: True)
    monkeypatch.setattr(module, "_exchange", lambda *a, **k: pytest.fail("duplicate bridge"))
    startup.runtime.fresh_snapshot = lambda: snapshot(state, valid=3, owned=owned)
    assert startup.start(automatic=True)
    assert finish(startup)["state"] == "ready"


def test_existing_hub_starting_is_not_duplicated(startup, monkeypatch):
    calls = [False, False, True]
    monkeypatch.setattr(module, "hub_ready", lambda *a: calls.pop(0))
    monkeypatch.setattr(module, "hub_processes", lambda *a: [123])
    assert startup._ensure_hub(startup.profile, startup.root)
    assert not calls


def test_hub_timeout_does_not_start_bridge_or_restart_hub(startup, monkeypatch):
    actions = []
    monkeypatch.setattr(module, "hub_ready", lambda *a: False)
    monkeypatch.setattr(module, "hub_processes", lambda *a: [])
    monkeypatch.setattr(startup, "_launch_hub", lambda *a: actions.append("hub"))
    monkeypatch.setattr(module, "_exchange", lambda *a, **k: pytest.fail("Hub not ready"))
    assert startup.start()
    assert finish(startup)["state"] == "error"
    assert actions == ["hub"]


def test_unknown_tracker_validity_does_not_restart_running_bridge(startup, monkeypatch):
    monkeypatch.setattr(module, "hub_ready", lambda *a: True)
    monkeypatch.setattr(module, "_exchange", lambda *a, **k: pytest.fail("duplicate bridge"))
    startup.runtime.fresh_snapshot = lambda: snapshot("DEGRADED", owned=True)
    assert startup.start()
    assert finish(startup)["state"] == "waiting"


def test_unknown_graph_or_recording_guard_blocks_launch(startup, monkeypatch):
    monkeypatch.setattr(module, "hub_ready", lambda *a: True)
    monkeypatch.setattr(module, "_exchange", lambda *a, **k: pytest.fail("not admitted"))
    startup.runtime.fresh_snapshot = lambda: snapshot(can_start=False)
    assert startup.start()
    assert finish(startup)["state"] == "error"


def test_discovery_delay_then_start(startup, monkeypatch):
    monkeypatch.setattr(module, "hub_ready", lambda *a: True)
    states = [snapshot("UNKNOWN", can_start=False), snapshot("UNKNOWN", can_start=False), snapshot()]
    final = [snapshot("STARTING", owned=True)]
    startup.runtime.fresh_snapshot = lambda: states.pop(0) if states else final[0]
    calls = []
    def exchange(*args, **kwargs):
        calls.append(1)
        final[0] = snapshot("RUNNING", valid=3, owned=True)
        return dict(ok=True, operation=dict(state="accepted"))
    monkeypatch.setattr(module, "_exchange", exchange)
    assert startup.start()
    assert finish(startup)["state"] == "ready" and calls == [1]


def test_uncertain_command_reply_is_never_retried(startup, monkeypatch):
    monkeypatch.setattr(module, "hub_ready", lambda *a: True)
    calls = []
    def exchange(*args, **kwargs):
        calls.append(1)
        raise TimeoutError("response lost")
    monkeypatch.setattr(module, "_exchange", exchange)
    assert startup.start()
    assert finish(startup)["state"] == "error" and calls == [1]


def test_manager_change_after_submission_stops_sequence(startup, monkeypatch):
    monkeypatch.setattr(module, "hub_ready", lambda *a: True)
    def exchange(*a, **k):
        startup.runtime.fresh_snapshot = lambda: snapshot(instance="replacement")
        return dict(ok=True, operation=dict(state="accepted"))
    monkeypatch.setattr(module, "_exchange", exchange)
    assert startup.start()
    assert "已更换" in finish(startup)["detail"]


def test_external_manager_blocks_hub_start(startup, monkeypatch):
    def snapshot_error():
        raise RuntimeError("not owned")
    startup.runtime.fresh_snapshot = snapshot_error
    monkeypatch.setattr(module, "hub_ready", lambda *a: pytest.fail("not owned"))
    assert startup.start()
    assert finish(startup)["detail"] == "not owned"


def test_duplicate_start_and_close_detach_without_bridge_request(startup, monkeypatch):
    entered = threading.Event()
    def ensure(*args):
        entered.set()
        startup._stop.wait(1)
        return True
    monkeypatch.setattr(startup, "_ensure_hub", ensure)
    monkeypatch.setattr(startup, "_ensure_bridge", lambda *a: pytest.fail("closed"))
    assert startup.start()
    assert entered.wait(1)
    assert startup.start() is False
    startup.close()
    finish(startup)


def test_launcher_uses_fixed_recipe_clean_environment_and_detaches(startup, monkeypatch):
    monkeypatch.setattr(module, "legacy_reader_active", lambda: False)
    monkeypatch.setenv("DISPLAY", ":test")
    monkeypatch.setenv("LD_LIBRARY_PATH", "/another/ros/lib")
    monkeypatch.setenv("LD_PRELOAD", "/another/qt/lib.so")
    calls = []
    monkeypatch.setattr(module.subprocess, "Popen", lambda *a, **k: calls.append((a, k)))
    startup._launch_hub(startup.root)
    args, kw = calls[0]
    assert args == (("/bin/bash", str(startup.root / "run-vivehub.sh")),)
    assert kw["cwd"] == startup.root and kw["start_new_session"]
    assert "LD_PRELOAD" not in kw["env"] and "LD_LIBRARY_PATH" not in kw["env"]
    assert kw["env"]["DISPLAY"] == ":test"


def test_active_legacy_reader_blocks_hub_launch(startup, monkeypatch):
    monkeypatch.setattr(module, "legacy_reader_active", lambda: True)
    with pytest.raises(RuntimeError, match="PyVUT"):
        startup._launch_hub(startup.root)


def test_missing_or_foreign_socket_is_not_ready(startup, tmp_path):
    path = tmp_path / "s.sock"
    assert not module.hub_ready(path, startup.root)
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as server:
        server.bind(str(path))
        server.listen(1)
        with pytest.raises(RuntimeError, match="其他程序"):
            module.hub_ready(path, startup.root)
    assert not module.hub_ready(path, startup.root)  # stale file


def test_live_seqpacket_peer_readiness(startup, tmp_path):
    path = tmp_path / "seq.sock"
    executable = startup.root / "tracker_server"
    executable.unlink()
    executable.symlink_to(f"/proc/{os.getpid()}/exe")
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as server:
        server.bind(str(path))
        server.listen(1)
        assert module.hub_ready(path, startup.root)


def test_transient_snapshot_disconnect_does_not_duplicate_hub(startup, monkeypatch):
    monkeypatch.setattr(module, "hub_ready", lambda *a: True)
    calls = []
    def fresh():
        calls.append(1)
        if len(calls) < 3:
            raise ConnectionError("busy")
        return snapshot("RUNNING", valid=3, owned=True)
    startup.runtime.fresh_snapshot = fresh
    assert startup.start()
    assert finish(startup)["state"] == "ready"


def test_bad_settings_do_not_change_saved_configuration(startup):
    before = startup.path.read_bytes()
    with pytest.raises(ValueError):
        startup.save(dict(enabled=True, install_dir="relative"))
    with pytest.raises(ValueError):
        startup.save(dict(enabled=True, install_dir=str(startup.root / "missing")))
    startup.runtime.configuration["process_groups"][0]["parameters"]["backend"] = "pyvut"
    with pytest.raises(ValueError, match="官方 SDK"):
        startup.save(dict(enabled=True, install_dir=str(startup.root)))
    assert startup.path.read_bytes() == before


def test_initial_manager_connection_triggers_saved_autoconnect_once(tmp_path, monkeypatch):
    from vt_capture_console.backend.local_runtime import DesktopRuntime
    runtime = DesktopRuntime(tmp_path, tmp_path / "settings")
    called = threading.Event()
    calls = []
    monkeypatch.setattr(runtime, "_start_worker", lambda: runtime._status_update("ready", "ready"))
    monkeypatch.setattr(runtime, "_record_alive", lambda record: True)
    monkeypatch.setattr(runtime, "_remember_late_auxiliary_sockets", lambda record: None)
    def connect(*, automatic):
        calls.append(automatic)
        called.set()
    monkeypatch.setattr(runtime.vivehub, "start", connect)
    try:
        runtime.start()
        assert called.wait(1)
        runtime.start()
        assert calls == [True]
    finally:
        runtime.close()
        runtime._thread.join(1)
    assert runtime.vivehub._stop.is_set()


def test_dialog_save_is_only_a_preference_and_connect_is_explicit(startup, qapp, monkeypatch):
    from vt_capture_console.ui.widgets.vivehub_connection import ViveHubConnectionDialog
    calls = []
    monkeypatch.setattr(startup, "start", lambda: calls.append(1))
    dialog = ViveHubConnectionDialog(startup)
    dialog.enabled.setChecked(False)
    dialog.save_button.click()
    assert startup.settings["enabled"] is False and calls == []
    dialog.connect_button.click()
    assert calls == [1]
    dialog.close()
    assert not dialog.timer.isActive()
