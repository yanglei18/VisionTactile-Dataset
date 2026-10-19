"""Parser and mocked UI contract tests, not real OpenGL/MANO validation."""
import copy
import json
from types import SimpleNamespace

import pytest

from vt_fsglove12 import viewer
from vt_fsglove12.config import digest
from vt_fsglove12.mano import DependencyError


REST = {"rest_global_xyzw": [[0, 0, 0, 1]] * 16, "provenance": "test mock only"}


def pose(timestamp=0.0, **changes):
    result = {
        "timestamp_s": timestamp,
        "anatomical_euler_rad": [[0, 0, 0] for _ in range(16)],
        "root_quat_xyzw": [0, 0, 0.7071067811865475, 0.7071067811865476],
        "quality": "experimental",
        "source_labels": ["measured_root"] + ["observed_relative_constrained_fit"] * 15,
        "model_rest_sha256": digest(REST),
        "solver_success": True,
    }
    result.update(changes)
    return result


def parse(*frames):
    return viewer.parse_pose_frames(json.dumps(frame) for frame in frames)


def test_parse_is_pure_preserves_explicit_gaps_and_counts_finger_priors():
    frame = pose()
    for slot in (3, 6, 9, 12, 13):
        frame["source_labels"][slot] = "inferred_missing_segment_prior"
    original = copy.deepcopy(frame)
    parsed = parse(frame, pose(1.0, quality="unavailable", anatomical_euler_rad=None,
                               root_quat_xyzw=None, solver_success=False,
                               source_labels=["unavailable_missing_root"] * 16))
    assert frame == original
    assert parsed[0]["inferred_joint_count"] == 5
    assert parsed[0]["renderable"] is True
    assert parsed[1]["renderable"] is False
    assert parsed[1]["anatomical_euler_rad"] is None


@pytest.mark.parametrize("changes", [
    {"timestamp_s": float("nan")}, {"timestamp_s": True},
    {"quality": "live_success"}, {"quality": []},
    {"source_labels": []}, {"source_labels": [0] * 16},
    {"model_rest_sha256": "missing"}, {"model_rest_sha256": "G" * 64},
    {"anatomical_euler_rad": [[0, 0, 0]] * 15},
    {"anatomical_euler_rad": [[float("nan"), 0, 0]] * 16},
    {"root_quat_xyzw": [0, 0, 0, 0]}, {"root_quat_xyzw": [0, 0, 1]},
    {"root_quat_xyzw": [0, 0, 0, float("inf")]},
])
def test_parse_rejects_invalid_available_frame_with_line_number(changes):
    with pytest.raises(ValueError, match="Pose JSONL line 1:"):
        parse(pose(**changes))


@pytest.mark.parametrize("lines", [[], ["\n"], ["{"], ["null"], ["[]"]])
def test_parse_rejects_empty_or_invalid_json(lines):
    with pytest.raises(ValueError):
        viewer.parse_pose_frames(lines)


def test_parse_rejects_non_monotonic_and_mixed_models():
    for frame in (pose(0), pose(-1), pose(1, model_rest_sha256="a" * 64)):
        with pytest.raises(ValueError, match="line 2"):
            parse(pose(), frame)


@pytest.mark.parametrize("changes", [{"quality": "experimental_not_converged"}, {"solver_success": False}])
def test_unsuccessful_solution_never_renderable(changes):
    assert not parse(pose(**changes))[0]["renderable"]


def test_headless_rejected_without_model_or_file_access(monkeypatch):
    monkeypatch.setattr(viewer.sys, "platform", "linux")
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    monkeypatch.setattr(viewer, "ManoBackend", lambda *_: pytest.fail("model constructed"))
    with pytest.raises(viewer.ViewerError, match="DISPLAY"):
        viewer.run_viewer("missing.jsonl", "missing_assets")


@pytest.mark.parametrize("speed", [0, -1, float("inf"), float("nan"), True, "1"])
def test_invalid_speed_fails_early(speed):
    with pytest.raises(ValueError, match="speed"):
        viewer.run_viewer("missing.jsonl", "missing_assets", speed=speed)


@pytest.fixture
def fake_player(monkeypatch, tmp_path):
    """A scripted fake UI; no installed Polyscope, torch or model is exercised."""
    monkeypatch.setenv("DISPLAY", ":mock")
    state = SimpleNamespace(steps=[], step={}, time=0.0, init_count=0, callback=None,
                            meshes=[], snapshots=[], text=[], renders=[], exports=0,
                            fail_render=False, fail_init=False, constructed=0)

    class Backend:
        def __init__(self, _assets_root, hand="right"):
            state.constructed += 1
            state.hand = hand

        def export_rest(self):
            state.exports += 1
            return REST

        def render(self, angles, root):
            state.renders.append((angles, root))
            if state.fail_render:
                raise RuntimeError("mock model failure")
            return {"vertices_m": [[0, 0, 0], [.1, 0, 0], [0, .1, 0]], "faces": [[0, 1, 2]]}

    class Mesh:
        def __init__(self):
            self.enabled = True
            self.updates = 0

        def set_enabled(self, value):
            self.enabled = value

        def update_vertex_positions(self, _vertices):
            self.updates += 1

    def init():
        state.init_count += 1
        if state.fail_init:
            raise RuntimeError("mock OpenGL failure")

    def register(*args, **kwargs):
        mesh = Mesh()
        state.meshes.append(mesh)
        return mesh

    def show():
        for step in state.steps:
            state.step = step
            state.time = step.get("time", 0.0)
            state.fail_render = step.get("fail_render", False)
            state.callback()
            state.snapshots.append([mesh.enabled for mesh in state.meshes])

    ps = SimpleNamespace(init=init, register_surface_mesh=register, show=show,
                         set_user_callback=lambda cb: setattr(state, "callback", cb),
                         clear_user_callback=lambda: setattr(state, "callback", None))
    ui = SimpleNamespace(Button=lambda _: state.step.get("button", False),
                         SliderInt=lambda _label, value, _lo, _hi: ("select" in state.step, state.step.get("select", value)),
                         TextUnformatted=state.text.append)
    monkeypatch.setattr(viewer, "ManoBackend", Backend)
    monkeypatch.setattr(viewer.importlib, "import_module", lambda name: {"polyscope": ps, "polyscope.imgui": ui}[name])
    monkeypatch.setattr(viewer.time, "monotonic", lambda: state.time)

    def write(*frames):
        path = tmp_path / "poses.jsonl"
        path.write_text("\n".join(json.dumps(frame) for frame in frames), encoding="utf-8")
        return path

    state.write = write
    return state


def test_model_digest_mismatch_before_opening_window(fake_player):
    path = fake_player.write(pose(model_rest_sha256="a" * 64))
    with pytest.raises(viewer.ViewerError, match="model_rest_sha256 mismatch"):
        viewer.run_viewer(path, "mock_assets")
    assert fake_player.exports == 1
    assert fake_player.init_count == 0
    assert not fake_player.renders


def test_missing_gui_dependency_does_not_load_model(fake_player, monkeypatch):
    path = fake_player.write(pose())

    def missing(_name):
        raise ModuleNotFoundError("polyscope")

    monkeypatch.setattr(viewer.importlib, "import_module", missing)
    with pytest.raises(DependencyError, match="Polyscope.*unavailable"):
        viewer.run_viewer(path, "mock_assets")
    assert fake_player.exports == 0


def test_missing_model_has_readable_error_before_gui_import(fake_player, monkeypatch):
    path = fake_player.write(pose())

    def missing(_assets, hand="right"):
        raise DependencyError("MANO model is missing: models/MANO_RIGHT.pkl")

    monkeypatch.setattr(viewer, "ManoBackend", missing)
    monkeypatch.setattr(viewer.importlib, "import_module", lambda *_: pytest.fail("GUI imported"))
    with pytest.raises(DependencyError, match="MANO_RIGHT.pkl"):
        viewer.run_viewer(path, "missing_assets")


def test_bad_opengl_is_readable(fake_player):
    fake_player.fail_init = True
    with pytest.raises(viewer.ViewerError, match="OpenGL.*mock OpenGL failure"):
        viewer.run_viewer(fake_player.write(pose()), "mock_assets")


def test_play_pause_slider_and_gaps_never_hold_stale_mesh(fake_player):
    path = fake_player.write(pose(), pose(1, quality="unavailable", anatomical_euler_rad=None,
                                         root_quat_xyzw=None, reason="missing_or_invalid_root"), pose(2))
    fake_player.steps = [{}, {"button": True}, {"time": .4}, {"time": 1.0}, {"select": 2}]
    viewer.run_viewer(path, "mock_assets")
    assert fake_player.snapshots == [[True], [True], [False], [False], [True]]
    assert len(fake_player.renders) == 2
    assert fake_player.renders[0][1] == pose()["root_quat_xyzw"]
    assert any("EXPERIMENTAL / OFFLINE" in text for text in fake_player.text)
    assert any("Recording gap" in text for text in fake_player.text)
    assert any("missing_or_invalid_root" in text for text in fake_player.text)
    assert fake_player.callback is None


def test_all_unavailable_never_creates_synthetic_mesh(fake_player):
    fake_player.steps = [{}]
    viewer.run_viewer(fake_player.write(pose(quality="unavailable")), "mock_assets")
    assert not fake_player.meshes


def test_left_viewer_passes_side_to_model(fake_player):
    fake_player.steps = [{}]
    viewer.run_viewer(fake_player.write(pose(quality="unavailable")), "mock_assets", hand="left")
    assert fake_player.hand == "left"
    assert not fake_player.renders


def test_render_failure_disables_previous_mesh_and_is_reported(fake_player):
    fake_player.steps = [{}, {"select": 1, "fail_render": True}]
    with pytest.raises(viewer.ViewerError, match="mock model failure"):
        viewer.run_viewer(fake_player.write(pose(), pose(1)), "mock_assets")
    assert fake_player.snapshots == [[True], [False]]
    assert fake_player.callback is None


def test_speed_and_end_of_recording(fake_player):
    fake_player.steps = [{"button": True}, {"time": .06}, {"time": 1.0}]
    viewer.run_viewer(fake_player.write(pose(), pose(.1), pose(.2)), "mock_assets", speed=2)
    assert len(fake_player.renders) == 3
    assert "Paused / offline frame inspection" == fake_player.text[-1]
