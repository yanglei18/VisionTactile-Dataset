"""Passive live-tick diagnostics contracts; no serial, GUI or MANO assets."""
from copy import deepcopy
import gc
import importlib.util
from pathlib import Path
import queue
import sys
from types import SimpleNamespace

import pytest


scripts = Path(__file__).parents[1] / "scripts"
spec = importlib.util.spec_from_file_location("benchmark_live_tick_test", scripts / "benchmark_live_tick.py")
benchmark = importlib.util.module_from_spec(spec)
sys.path.insert(0, str(scripts))
try:
    spec.loader.exec_module(benchmark)
finally:
    sys.path.pop(0)


def test_gc_trace_correlates_real_intervals_without_changing_gc_policy():
    policy = (gc.isenabled(), gc.get_threshold(), list(gc.callbacks))
    clock = SimpleNamespace(now=1_000_000_000)
    trace = benchmark.LiveGcTrace(clock=lambda: clock.now)
    trace.start()
    tick = trace.begin("tick", 45210, measured=True)
    clock.now += 2_000_000
    writer = trace.begin("writer_pair", 45209)
    trace._callback("start", dict(generation=2))
    clock.now += 87_000_000
    trace._callback("stop", dict(generation=2, collected=3, uncollectable=0))
    trace.end(writer)
    clock.now += 1_000_000
    trace.end(tick, emitted=True)
    report = trace.finish()
    assert trace.finish() is report
    assert (gc.isenabled(), gc.get_threshold(), list(gc.callbacks)) == policy
    assert report["gc_policy_changed"] is report["manual_collect_called"] is False
    event = next(event for event in report["events"] if event["generation"] == 2)
    assert event["duration_ns"] == 87_000_000
    assert event["collector_interval"] == "writer_pair"
    assert event["collector_frame_index"] == 45209
    assert event["overlapping_measured_ticks"][0]["frame_index"] == 45210
    assert event["overlapping_measured_ticks"][0]["overlap_ns"] == 87_000_000
    slow = report["slowest_ticks"][0]
    assert slow["duration_ms"] == 90.
    assert slow["gc_events"][0]["duration_ms"] == 87.
    assert slow["overlapping_interval_counts"] == {"writer_pair": 1}


def test_trace_bounds_storage_but_keeps_total_gc_counters():
    clock = SimpleNamespace(now=0)
    trace = benchmark.LiveGcTrace(clock=lambda: clock.now, max_events=1, max_intervals=1)
    trace.start()
    for index in range(3):
        token = trace.begin("tick", index, measured=True)
        trace._callback("start", dict(generation=2))
        clock.now += 1_000_000
        trace._callback("stop", dict(generation=2, collected=0, uncollectable=0))
        trace.end(token, emitted=True)
    report = trace.finish(writer_process=True)
    assert len(report["events"]) == report["event_limit"] == 1
    assert report["dropped_events"] == report["dropped_intervals"] == 2
    assert report["by_generation"]["2"] == dict(count=3, total_ms=3., max_ms=1.)
    assert report["interval_counts"] == {"tick": 1}
    assert report["writer_process_gc_observed"] is False


def test_queue_instrumentation_preserves_identity_and_completion():
    original = queue.Queue(maxsize=1)
    pair = ({"frame_index": 8}, {"frame_index": 8}, False)
    trace = benchmark.LiveGcTrace()
    wrapped = benchmark._TracedQueue(original, trace)
    trace.start()
    try:
        wrapped.put_nowait(pair)
        assert wrapped.get(timeout=.01) is pair
        wrapped.task_done()
        assert wrapped.empty() and original.unfinished_tasks == 0
    finally:
        report = trace.finish()
    assert report["interval_counts"] == {"writer_pair": 1}
    assert report["slowest_writer_pairs"][0]["frame_index"] == 8


def test_traced_thread_recorder_preserves_order_pair_counts_and_payloads(tmp_path):
    paths = {name: tmp_path / f"{name}.jsonl" for name in ("orientations", "poses")}
    trace = benchmark.LiveGcTrace()
    recorder, streams = benchmark.make_result_recorder(paths, trace=trace)
    trace.start()
    frames = [{"frame_index": index, "timestamp_s": 100. + index / 30.} for index in range(3)]
    before = deepcopy(frames)
    try:
        for frame in frames:
            recorder.submit(frame, dict(frame, source_fresh=True, hardware_accessed=False))
        assert recorder.close(2.)
    finally:
        recorder.close(2.)
        report = trace.finish()
    assert frames == before
    assert recorder.snapshot()["written_pairs"] == 3
    logs = benchmark.verify_logs(paths, [0, 1, 2])
    assert logs["poses"]["source_fresh_results"] == 3
    assert all(stream.closed for stream in streams.values())
    assert report["interval_counts"]["writer_pair"] == 3
    assert report["interval_counts"]["writer_write"] == 6
    assert report["interval_counts"]["recorder_submit"] == 3


def test_process_factory_does_not_open_parent_streams(monkeypatch):
    paths = {"orientations": object(), "poses": object()}
    writer = SimpleNamespace(check=lambda: None)
    constructor_calls = []

    def constructor(supplied):
        constructor_calls.append(supplied)
        return writer

    monkeypatch.setitem(sys.modules, "vt_fsglove12.live_recording_process",
                        SimpleNamespace(ProcessResultRecorder=constructor))
    returned, streams = benchmark.make_result_recorder(paths, mode="process")
    assert returned is writer and streams == {}
    assert constructor_calls == [paths]


def test_publication_reader_really_pickles_production_envelope(monkeypatch):
    calls, snapshot = [], {"pose": {"frame_index": 7}, "geometry": [1., 2.]}
    original = deepcopy(snapshot)

    def read():
        reader.stop.set()  # One synchronous read, no scheduler timing dependence.
        return snapshot

    real_dumps = benchmark.pickle.dumps

    def dumps(envelope, *, protocol):
        calls.append((envelope, protocol))
        return real_dumps(envelope, protocol=protocol)

    monkeypatch.setattr(benchmark.pickle, "dumps", dumps)
    reader = benchmark.SnapshotReader(SimpleNamespace(_latest_pose_publication=read), 0,
                                      mode="publication")
    reader.hz = 60
    reader.measure.set()
    reader._run()
    result = reader.close()
    assert not result["errors"] and result["reads"] == 1
    assert result["public_copy_calls"] == 0 and result["borrowed_publication_calls"] == 1
    assert result["pickle_encoded_snapshots"] == 1 and result["pickle_bytes"] > 0
    assert result["pickle_latency_ms"]["count"] == 1
    assert calls[0][0][1:] == (0, snapshot)
    assert calls[0][1] == benchmark.pickle.HIGHEST_PROTOCOL
    assert snapshot == original


def test_default_copy_reader_does_not_encode_or_borrow():
    def read():
        reader.stop.set()
        return {"pose": None}

    reader = benchmark.SnapshotReader(SimpleNamespace(latest_pose_snapshot=read), 0)
    reader.hz = 60
    reader.measure.set()
    reader._run()
    result = reader.close()
    assert result["mode"] == "copy" and result["public_copy_calls"] == 1
    assert result["borrowed_publication_calls"] == result["pickle_encoded_snapshots"] == 0


def test_snapshot_failure_closes_trace_intervals_and_reports_error():
    def fail():
        raise RuntimeError("read failed")

    trace = benchmark.LiveGcTrace()
    reader = benchmark.SnapshotReader(SimpleNamespace(latest_pose_snapshot=fail), 0, trace=trace)
    reader.hz = 60
    trace.start()
    try:
        reader._run()
        assert reader.close()["errors"] == ["RuntimeError: read failed"]
        assert all(not stack for stack in trace.active.values())
    finally:
        trace.finish()


def test_unknown_modes_and_missing_publication_are_explicit():
    with pytest.raises(ValueError, match="Unknown result recorder"):
        benchmark.make_result_recorder({}, mode="missing")
    with pytest.raises(ValueError, match="requires _latest_pose_publication"):
        benchmark.SnapshotReader(SimpleNamespace(), 0, mode="publication")


def test_semantic_digest_includes_values_quality_presence_and_original_source_times():
    records = [dict(frame_index=3, timestamp_s=12., root_quat_xyzw=[0., 0., 0., 1.],
                    quality={"valid": True}, nodes={"1": {"host_monotonic_ns": 77, "sample_index": 5}},
                    solve_completed_monotonic_ns=100)]
    reference = benchmark.semantic_digest(records)
    changed = deepcopy(records)
    changed[0]["solve_completed_monotonic_ns"] = 200
    assert benchmark.semantic_digest(changed) == reference
    assert "solve_completed_monotonic_ns" not in reference["included_top_level_fields"]
    for key, value in (("timestamp_s", 13.), ("quality", {"valid": False}),
                       ("root_quat_xyzw", [0., 0., 1., 0.]), ("new_field", None),
                       ("nodes", {"1": {"host_monotonic_ns": 78, "sample_index": 5}})):
        changed = deepcopy(records)
        changed[0][key] = value
        assert benchmark.semantic_digest(changed)["sha256"] != reference["sha256"]


def _prepared_input_fakes(monkeypatch, tmp_path, *, freeze):
    import vt_fsglove12.calibration as calibration
    import vt_fsglove12.config as config_module
    calls = []
    (tmp_path / "config.json").write_text('{"hand":"right"}')
    (tmp_path / "rest.json").write_text('{}')
    monkeypatch.setattr(config_module, "validate_config", lambda value: value)
    monkeypatch.setattr(calibration, "validate_rest", lambda value: None)

    class Backend:
        def __init__(self, config, rest, **kwargs):
            calls.append("backend")
            assert config == {"hand": "right"} and rest == {}

    class Geometry:
        def __init__(self, assets, **kwargs):
            calls.append("geometry")
            assert kwargs == {"hand": "right", "compact_live_geometry": True}

        def export_rest(self):
            calls.append("models_ready")

    class Policy:
        def __init__(self, *, scope, allow_bootstrap_permanent):
            assert scope == "offline_benchmark"
            assert allow_bootstrap_permanent is True
            assert calls == []
            calls.append("policy_token")

        def start(self):
            assert calls[-1] == "models_ready"
            calls.append("policy_start")

        def close(self):
            calls.append("policy_close")

    def load(*args, **kwargs):
        assert calls[-1] == ("policy_start" if freeze else "models_ready")
        calls.append("recording")
        return dict(config={"hand": "right"}, rest={}, profile={}, frames=[{"frame_index": 1}])

    monkeypatch.setitem(sys.modules, "vt_fsglove12.mano", SimpleNamespace(ManoBackend=Geometry))
    monkeypatch.setitem(sys.modules, "vt_fsglove12.upstream_backend",
                        SimpleNamespace(OfficialFSGloveBackend=Backend))
    monkeypatch.setitem(sys.modules, "vt_fsglove12.live_gc", SimpleNamespace(StartupGcFreeze=Policy))
    monkeypatch.setattr(benchmark, "_configure_tensor_threads", lambda: calls.append("tensor_threads"))
    monkeypatch.setattr(benchmark, "load_recording", load)
    args = SimpleNamespace(session=tmp_path, fsglove_source=tmp_path, mano_assets=tmp_path,
        compact_live_geometry=True, startup_gc_freeze=freeze, result_recorder="thread",
        frames=2, warmup=1, tail_mib=1)
    return args, calls


@pytest.mark.parametrize("freeze", [False, True])
def test_static_model_initialization_order_is_matched_and_frames_are_loaded_after_policy(
        monkeypatch, tmp_path, freeze):
    args, calls = _prepared_input_fakes(monkeypatch, tmp_path, freeze=freeze)
    with benchmark.prepared_benchmark_inputs(args) as (recording, backend, geometry, policy):
        calls.append("body")
        assert recording["frames"] == [{"frame_index": 1}]
        assert (policy is not None) == freeze
    assert calls == [*(["policy_token"] if freeze else []), "tensor_threads", "backend", "geometry",
                     "models_ready", *(["policy_start"] if freeze else []),
                     "recording", "body", *(["policy_close"] if freeze else [])]


@pytest.mark.parametrize("failure_stage", ["load", "tick"])
def test_startup_policy_is_cleaned_up_after_dynamic_load_or_benchmark_failure(
        monkeypatch, tmp_path, failure_stage):
    args, calls = _prepared_input_fakes(monkeypatch, tmp_path, freeze=True)
    if failure_stage == "load":
        def fail(*args, **kwargs):
            raise RuntimeError("load failed")
        monkeypatch.setattr(benchmark, "load_recording", fail)
    with pytest.raises(RuntimeError, match=failure_stage):
        with benchmark.prepared_benchmark_inputs(args):
            raise RuntimeError("tick failed")
    assert calls[-1] == "policy_close"
