"""Offline benchmark contracts; no hardware, GUI or licensed model needed."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_fsglove12.config import PARENTS, digest, load_config


source = Path(__file__).parents[1] / "scripts" / "benchmark_upstream_runtime.py"
spec = importlib.util.spec_from_file_location("benchmark_upstream_runtime_test", source)
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


def recording(tmp_path):
    config = load_config()
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    metadata = dict(session_id="recorded-session", config_sha256=digest(config), model_rest_sha256=digest(rest))
    profile = dict(session_id=metadata["session_id"], config_sha256=digest(config), rest_sha256=digest(rest),
                   upstream_state_sha256="state", profile_version=2, functional_enabled=True,
                   functional_fingers={finger: {} for finger in benchmark.FINGERS},
                   nodes={node: dict(epoch=0) for node in config["node_to_slot"]})
    frames = [dict(**metadata, frame_index=index, timestamp_s=100. + index / 30.,
                   orientation_source="vqf_6d", orientation_frame="independent_initial_heading",
                   nodes={str(node): dict(epoch=0, sample_index=index + 1, quat_xyzw=[0., 0., 0., 1.],
                                          age_s=0.) for node in range(1, 13)}) for index in range(8)]
    for name, value in (("config.json", config), ("rest.json", rest), ("profile.json", profile),
                        ("session_metadata.json", metadata)):
        (tmp_path / name).write_text(json.dumps(value))
    (tmp_path / "orientations.jsonl").write_text("".join(json.dumps(frame) + "\n" for frame in frames))
    return config, profile, frames


def test_tail_ignores_partial_boundaries_and_never_reads_full_file(tmp_path):
    path = tmp_path / "orientations.jsonl"
    lines = [json.dumps({"frame_index": index, "padding": "x" * 100}) + "\n" for index in range(100)]
    path.write_text("".join(lines) + '{"frame_index":100')
    frames, detail = benchmark.complete_tail(path, count=3, max_bytes=800)
    assert [frame["frame_index"] for frame in frames] == [97, 98, 99]
    assert detail["read_bytes"] == 800 < detail["file_size_at_read_bytes"]
    with pytest.raises(ValueError, match="bounded tail"):
        benchmark.complete_tail(path, count=100, max_bytes=800)


def test_latest_version_not_stale_profile_json_and_explicit_override(tmp_path):
    for name in ("profile.json", "profile.v002.json", "profile.v010.json", "profile.v9.json", "profile.vx.json"):
        (tmp_path / name).touch()
    assert benchmark.select_profile(tmp_path).name == "profile.v010.json"
    assert benchmark.select_profile(tmp_path, "profile.v002.json").name == "profile.v002.json"


@pytest.mark.parametrize("first", [0, 1, 50, 97])
def test_binary_range_recovers_exact_frames_without_scanning_prefix(tmp_path, first):
    path = tmp_path / "orientations.jsonl"
    path.write_text("".join(json.dumps(dict(frame_index=i, padding="x" * (100 + i))) + "\n"
                            for i in range(100)) + '{"frame_index":100')
    frames, detail = benchmark.bounded_frame_range(path, start_frame=first, count=3, max_bytes=6000)
    assert [frame["frame_index"] for frame in frames] == list(range(first, first + 3))
    assert detail["read_bytes"] <= 6000 < detail["file_size_at_read_bytes"]
    with pytest.raises(ValueError):
        benchmark.bounded_frame_range(path, start_frame=first, count=3, max_bytes=50)


def test_loading_and_snapshot_preserve_real_binding_and_source(tmp_path):
    config, profile, frames = recording(tmp_path)
    original = (tmp_path / "orientations.jsonl").read_bytes()
    loaded = benchmark.load_recording(tmp_path, count=6)
    assert loaded["frames"] == frames[-6:]
    target = tmp_path / "frozen"
    benchmark.freeze_recording(loaded, target)
    frozen = benchmark.load_recording(target, count=6)
    assert frozen["frames"] == loaded["frames"]
    assert frozen["profile"] == profile
    assert (tmp_path / "orientations.jsonl").read_bytes() == original
    with pytest.raises(FileExistsError):
        benchmark.freeze_recording(loaded, target)


@pytest.mark.parametrize("change,match", [("session", "bindings"), ("functional", "functional"),
                                        ("channel", "twelve"), ("epoch", "epoch")])
def test_loader_rejects_unrepresentative_or_mismatched_input(tmp_path, change, match):
    _, profile, frames = recording(tmp_path)
    if change == "session":
        profile["session_id"] = "another-session"
    elif change == "functional":
        profile["functional_enabled"] = False
    elif change == "channel":
        del frames[-1]["nodes"]["11"]
    elif change == "epoch":
        frames[-1]["nodes"]["12"]["epoch"] += 1
    (tmp_path / "profile.json").write_text(json.dumps(profile))
    (tmp_path / "orientations.jsonl").write_text("".join(json.dumps(frame) + "\n" for frame in frames))
    with pytest.raises(ValueError, match=match):
        benchmark.load_recording(tmp_path, count=6)


def test_source_counts_do_not_turn_duplicate_or_30hz_frames_into_120hz(tmp_path):
    config, _, frames = recording(tmp_path)
    frames[-1]["nodes"] = deepcopy(frames[-2]["nodes"])
    summary = benchmark.source_summary(frames, config)
    assert summary["recorded_publication_hz"] == pytest.approx(30.)
    assert summary["unique_recorded_frame_indices"] == 8
    assert summary["unique_source_vectors"] == summary["unique_root_samples"] == 7
    assert summary["duplicate_source_vectors"] == 1
    assert summary["input_channel_count"] == 12
    assert not summary["is_120hz_hardware_validation"]
    assert not summary["timestamps_rebased"]


def test_latency_percentiles_and_deadline_include_tail_outliers():
    stats = benchmark.latency_stats([1., 2., 3., 4., 100.], deadline_ms=1000. / 120.)
    assert stats["median_ms"] == 3.
    assert stats["p95_ms"] == pytest.approx(80.8)
    assert stats["p99_ms"] == pytest.approx(96.16)
    assert stats["max_ms"] == 100.
    assert stats["deadline_misses"] == 1
    assert stats["deadline_miss_fraction"] == .2
    assert benchmark.latency_stats([], deadline_ms=8.33)["median_ms"] is None


def test_benchmark_excludes_warmup_and_counts_only_rendered_meshes(tmp_path):
    _, profile, frames = recording(tmp_path)
    calls = []

    def solve(frame, supplied):
        calls.append(frame["frame_index"])
        return dict(pose_available=True, solver_success=True, upstream_fsglove_solver_executed=True,
                    mano_pose_rotvec=[[0., 0., 0.]] * 16, functional_adapter=dict(enabled=True),
                    pose_postprocessing="qmt_functional_axis_and_joint_constraint")

    backend = SimpleNamespace(solve=solve, reset_calibration=lambda: None)
    meshes = []
    geometry = SimpleNamespace(render_mano=lambda *args: meshes.append(args))
    display = SimpleNamespace(update=lambda *args, **kwargs: None, reset=lambda: None)
    before = deepcopy((frames, profile))
    result = benchmark.benchmark_frames(backend, geometry, display, frames, profile,
                                       warmup=2, mesh_every=4, deadline_ms=1000. / 120.)
    assert result["measured_frames"] == 6
    assert result["rendered_mesh_count"] == result["latency_ms"]["mesh"]["count"] == 2
    assert result["latency_ms"]["total_with_mesh"]["count"] == 2
    assert result["latency_ms"]["total_without_mesh"]["count"] == 4
    assert result["latency_ms"]["solve"]["count"] == 6
    assert calls == [0] + list(range(8))  # Untimed initialization, then ordered real frames.
    assert (frames, profile) == before


def test_benchmark_rejects_silent_functional_pipeline_fallback(tmp_path):
    _, profile, frames = recording(tmp_path)
    backend = SimpleNamespace(solve=lambda *_: dict(pose_available=True, solver_success=True,
                                                    upstream_fsglove_solver_executed=True))
    with pytest.raises(ValueError, match="functional pose pipeline"):
        benchmark.benchmark_frames(backend, None, None, frames, profile,
                                   warmup=2, mesh_every=1, deadline_ms=1000. / 120.)


def test_gc_pause_recorder_preserves_policy_and_attributes_generation_to_stage():
    gc = benchmark.gc
    enabled, thresholds, callbacks = gc.isenabled(), gc.get_threshold(), list(gc.callbacks)
    clock = iter([100., 100.010, 100.063])
    trace = benchmark.GcPauseRecorder(clock=lambda: next(clock))
    trace.start()
    trace.frame_index, trace.stage = 45166, "mesh"
    trace._callback("start", {"generation": 2})
    trace._callback("stop", {"generation": 2, "collected": 41, "uncollectable": 0})
    report = trace.finish()
    assert gc.callbacks == callbacks
    assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
    assert report["gc_policy_changed"] is False and report["manual_collect_called"] is False
    assert report["by_generation"]["2"]["max_ms"] == pytest.approx(53.)
    assert report["events"][0]["frame_index"] == 45166
    assert report["events"][0]["stage"] == "mesh"
    assert report["events"][0]["collected"] == 41


def test_gc_callback_is_removed_when_measured_backend_fails(tmp_path):
    _, profile, frames = recording(tmp_path)
    callbacks = list(benchmark.gc.callbacks)

    def solve(frame, supplied):
        if frame["frame_index"] == 2:
            raise RuntimeError("intentional measured failure")
        return dict(pose_available=True, solver_success=True, upstream_fsglove_solver_executed=True,
                    mano_pose_rotvec=[[0., 0., 0.]] * 16, functional_adapter=dict(enabled=True),
                    pose_postprocessing="qmt_functional_axis_and_joint_constraint")

    backend = SimpleNamespace(solve=solve, reset_calibration=lambda: None)
    geometry = SimpleNamespace(render_mano=lambda *_: None)
    display = SimpleNamespace(update=lambda *args, **kwargs: None, reset=lambda: None)
    with pytest.raises(RuntimeError, match="intentional measured failure"):
        benchmark.benchmark_frames(backend, geometry, display, frames, profile,
                                   warmup=2, mesh_every=1, deadline_ms=1000. / 120., trace_gc=True)
    assert benchmark.gc.callbacks == callbacks
