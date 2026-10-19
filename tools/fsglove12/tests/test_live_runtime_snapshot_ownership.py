"""Snapshot ownership/lock contracts; no devices, threads or timing benchmark."""
from copy import deepcopy
import pickle
from types import MethodType

import numpy as np
import pytest

from vt_fsglove12 import live_runtime
from vt_fsglove12.config import load_config
from vt_fsglove12.live_stream import OnlineFusion, RawSample
from vt_fsglove12.mano import _readonly_geometry_array

from test_live_runtime_high_rate import feed, setup_direct


def published_controller(tmp_path):
    controller, session, backend, clock = setup_direct(tmp_path)
    feed(session, clock, 12, 3)
    controller._tick(session)
    geometry = dict(vertices_m=_readonly_geometry_array([[1., 2., 3.]]),
                    faces=_readonly_geometry_array([[0, 0, 0]], dtype=np.int64),
                    provenance=dict(source="owned_test_geometry"))
    controller._snapshot.update(geometry=geometry, display_geometry=geometry)
    return controller, session, backend, clock


@pytest.mark.parametrize("method", ["latest_snapshot", "latest_pose_snapshot"])
def test_public_snapshots_copy_payloads_outside_state_lock(tmp_path, monkeypatch, method):
    controller, _session, _backend, _clock = published_controller(tmp_path)
    original = controller._snapshot
    copies = []

    def checked_copy(value, *args):
        assert not controller._state_lock.locked()
        copies.append(value)
        return deepcopy(value, *args)

    monkeypatch.setattr(live_runtime, "deepcopy", checked_copy)
    result = getattr(controller, method)()
    assert copies
    assert result["geometry"] is result["display_geometry"]
    assert not np.shares_memory(result["geometry"]["vertices_m"], original["geometry"]["vertices_m"])
    result["runtime"]["timing_ms"]["solve"] = -1
    result["pose"]["root_quat_xyzw"][0] = 99.
    result["orientation"]["nodes"]["12"]["quat_xyzw"][0] = 99.
    result["calibration"]["warnings"].append("caller_only")
    result["geometry"]["provenance"]["source"] = "caller_only"
    assert original["runtime"]["timing_ms"]["solve"] >= 0.
    assert original["pose"]["root_quat_xyzw"][0] == 0.
    assert original["orientation"]["nodes"]["12"]["quat_xyzw"][0] == 0.
    assert "caller_only" not in original["calibration"]["warnings"]
    assert original["geometry"]["provenance"]["source"] == "owned_test_geometry"


def test_private_publication_detaches_only_aging_and_overlay_headers(tmp_path, monkeypatch):
    controller, _session, _backend, clock = published_controller(tmp_path)
    original = controller._snapshot
    clock.advance(.04)

    def unexpected_copy(*_args):
        raise AssertionError("trusted synchronous publication must not deep-copy payloads")

    monkeypatch.setattr(live_runtime, "deepcopy", unexpected_copy)
    result = controller._latest_pose_publication()
    assert result is not original
    assert result["runtime"] is not original["runtime"]
    for key in ("pose", "calibration", "geometry", "display_pose", "display_geometry"):
        assert result[key] is original[key]
    assert result["orientation"] is not original["orientation"]
    assert result["orientation"]["nodes"] is not original["orientation"]["nodes"]
    result_node = result["orientation"]["nodes"]["12"]
    original_node = original["orientation"]["nodes"]["12"]
    assert result_node is not original_node
    assert result_node["quat_xyzw"] is original_node["quat_xyzw"]
    assert result_node["age_s"] == pytest.approx(.04)
    result["runtime"]["error"] = "child_command_overlay"
    result_node["age_s"] = 999.
    assert original["runtime"]["error"] is None
    assert original_node["age_s"] == 0.


def test_private_publication_pickle_owns_values_and_old_payload_survives_lifecycle(tmp_path):
    controller, session, _backend, clock = published_controller(tmp_path)
    publication = controller._latest_pose_publication()
    encoded = pickle.dumps(publication, protocol=pickle.HIGHEST_PROTOCOL)
    received = pickle.loads(encoded)
    received["pose"]["root_quat_xyzw"][0] = 50.
    received["orientation"]["nodes"]["12"]["quat_xyzw"][0] = 60.
    received["calibration"]["warnings"].append("receiver_only")
    received["geometry"]["provenance"]["source"] = "receiver_only"
    assert not np.shares_memory(received["geometry"]["vertices_m"], publication["geometry"]["vertices_m"])
    assert not received["geometry"]["vertices_m"].flags.writeable
    assert received["geometry"] is received["display_geometry"]
    clock.advance(.01)
    feed(session, clock, 12, 3)
    controller._tick(session)
    assert pickle.dumps(publication, protocol=pickle.HIGHEST_PROTOCOL) == encoded
    controller._clear_calibration()
    assert pickle.dumps(publication, protocol=pickle.HIGHEST_PROTOCOL) == encoded
    assert controller._latest_pose_publication()["geometry"] is None
    controller._invalidate("test_disconnect")
    assert pickle.dumps(publication, protocol=pickle.HIGHEST_PROTOCOL) == encoded
    disconnected = controller._latest_pose_publication()
    assert disconnected["pose"]["reason"] == "disconnected"
    assert disconnected["display_geometry"] is disconnected["display_pose"] is None


def test_private_publication_disconnect_after_header_capture_hides_payloads(tmp_path):
    controller, session, _backend, _clock = published_controller(tmp_path)

    class DisconnectDuringStatus:
        def snapshot(self):
            session.stop.set()
            controller._invalidate("test_disconnect")
            return {}

    session.result_recorder = DisconnectDuringStatus()
    publication = controller._latest_pose_publication()
    assert not publication["runtime"]["connected"]
    assert publication["pose"]["reason"] == "disconnected"
    assert publication["geometry"] is publication["display_geometry"] is publication["display_pose"] is None


def test_input_success_does_not_copy_unused_cached_frame(tmp_path, monkeypatch):
    controller, _session, _backend, _clock = published_controller(tmp_path)
    previous = controller._input_snapshot
    copied = []

    def checked_copy(value, *args):
        assert value is not previous
        assert not controller._state_lock.locked()
        copied.append(value)
        return deepcopy(value, *args)

    monkeypatch.setattr(live_runtime, "deepcopy", checked_copy)
    result = controller.latest_input_snapshot()
    assert result["orientation"]["nodes"]["12"]["sample_index"] == 1
    assert copied
    result["orientation"]["nodes"]["12"]["quat_xyzw"][0] = 99.
    assert controller._input_snapshot["orientation"]["nodes"]["12"]["quat_xyzw"][0] == 0.


def test_input_busy_cache_is_aged_once_and_caller_cannot_mutate_cache(tmp_path, monkeypatch):
    controller, session, _backend, clock = published_controller(tmp_path)
    controller.latest_input_snapshot()
    cached = controller._input_snapshot
    clock.advance(.04)

    def checked_copy(value, *args):
        assert not controller._state_lock.locked()
        return deepcopy(value, *args)

    monkeypatch.setattr(live_runtime, "deepcopy", checked_copy)
    with session.lock:
        result = controller.latest_input_snapshot()
        assert result["orientation"]["nodes"]["12"]["age_s"] == pytest.approx(.04)
        assert cached["orientation"]["nodes"]["12"]["age_s"] == 0.
        result["orientation"]["nodes"]["12"]["quat_xyzw"][0] = 99.
        clock.advance(.02)
        again = controller.latest_input_snapshot()
        assert again["orientation"]["nodes"]["12"]["age_s"] == pytest.approx(.06)
        assert again["orientation"]["nodes"]["12"]["quat_xyzw"][0] == 0.
        clock.advance(controller.config["stale_after_s"])
        assert controller.latest_input_snapshot()["orientation"]["nodes"] == {}


@pytest.mark.parametrize("action", ["clear", "disconnect", "replace_session"])
def test_input_cache_copy_race_revalidates_epoch_and_connection(tmp_path, monkeypatch, action):
    controller, session, _backend, _clock = published_controller(tmp_path)
    controller.latest_input_snapshot()
    previous = controller._input_snapshot
    previous_bytes = pickle.dumps(previous)
    triggered = False

    def racing_copy(value, *args):
        nonlocal triggered
        result = deepcopy(value, *args)
        if not triggered and isinstance(value, dict) and set(value) == {"runtime", "orientation"}:
            triggered = True
            if action == "clear":
                controller._clear_calibration()
            elif action == "disconnect":
                session.stop.set()
                controller._invalidate("test_disconnect")
            else:
                controller._session = None
        return result

    monkeypatch.setattr(live_runtime, "deepcopy", racing_copy)
    result = controller.latest_input_snapshot()
    assert triggered and result["orientation"] is None
    assert pickle.dumps(previous) == previous_bytes
    if action == "clear":
        assert result["runtime"]["clear_generation"] == 1
        assert result["runtime"]["input_reset_race_discarded"]
        assert controller._input_snapshot["orientation"] is None
        assert controller._input_snapshot["runtime"]["clear_generation"] == 1
    else:
        assert not result["runtime"]["connected"]
        if action == "disconnect":
            assert not controller._input_snapshot["runtime"]["connected"]


@pytest.mark.parametrize("input_only", [False, True])
def test_original_online_fusion_snapshot_already_owns_filter_leaves(monkeypatch, input_only):
    config = load_config()
    fusion = OnlineFusion(config)
    fusion.update([RawSample(12, (0, 0, 0, 0, 0, 8192), 100., 1234.)])
    expected = fusion.input_snapshot(100.)

    def unexpected_copy(*_args):
        raise AssertionError("OnlineFusion already owns mutable leaves")

    monkeypatch.setattr(live_runtime, "deepcopy", unexpected_copy)
    frame = live_runtime._fusion_snapshot(fusion, 100., input_only=input_only)
    assert frame == expected
    frame["nodes"]["12"]["quat_xyzw"][0] = 99.
    frame["nodes"]["12"]["raw_values"][0] = 99
    frame["nodes"]["12"]["fusion_state"]["gyro_bias_rad_s"][0] = 99.
    frame["sensor_quality"]["gyro_near_rail"]["nodes"]["12"]["total_by_axis"]["x"] = 99
    assert fusion.input_snapshot(100.)["nodes"] == expected["nodes"]
    assert fusion.input_snapshot(100.)["sensor_quality"] == expected["sensor_quality"]


@pytest.mark.parametrize("override", ["subclass", "instance_snapshot", "instance_input", "class_snapshot", "class_input"])
@pytest.mark.parametrize("input_only", [False, True])
def test_injected_or_monkeypatched_fusion_keeps_defensive_copy(monkeypatch, override, input_only):
    config = load_config()
    borrowed = dict(timestamp_s=100., nodes={"12": dict(quat_xyzw=[0., 0., 0., 1.])})

    def snapshot(self, now, **_kwargs):
        return borrowed

    class Subclass(OnlineFusion):
        def snapshot(self, now, **_kwargs):
            return borrowed

    fusion = Subclass(config) if override == "subclass" else OnlineFusion(config)
    if override == "instance_snapshot":
        fusion.snapshot = MethodType(snapshot, fusion)
    elif override == "instance_input":
        fusion.input_snapshot = MethodType(snapshot, fusion)
    elif override == "class_snapshot":
        monkeypatch.setattr(OnlineFusion, "snapshot", snapshot)
    elif override == "class_input":
        monkeypatch.setattr(OnlineFusion, "input_snapshot", snapshot)
    if override.endswith("input") and not input_only:
        # Replacing the monitor does not change the original logging reader.
        assert live_runtime._fusion_snapshot(fusion, 100.)["nodes"] == {}
        return
    frame = live_runtime._fusion_snapshot(fusion, 100., input_only=input_only)
    assert frame == borrowed and frame is not borrowed
    frame["nodes"]["12"]["quat_xyzw"][0] = 99.
    assert borrowed["nodes"]["12"]["quat_xyzw"][0] == 0.
