#!/usr/bin/env python3
"""Real-GL recorded-orientation performance replay; NEVER opens a serial port.

Run with the package/live/model dependencies installed, for example::

    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python \
      tools/fsglove12/scripts/benchmark_live_ui.py --session SESSION_DIR \
      --mano-assets ASSETS_ROOT --duration 8 --output NEW_RESULT.json

Reads only a bounded, complete tail of orientations.jsonl and its session-bound
profile.v001.json. Recorded values/epochs are unchanged; scheduling timestamps
are explicitly rebased for performance replay, not physical latency analysis.
The output is created exclusively. No existing application is controlled.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from functools import partial
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np


def _read_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def _complete_tail(path, *, count=120, max_bytes=2 * 1024 * 1024):
    with Path(path).open("rb") as stream:
        end = stream.seek(0, 2)
        start = max(0, end - max_bytes)
        stream.seek(start)
        lines = stream.read(end - start).splitlines(keepends=True)
    if start:
        lines = lines[1:]  # May be a partial leading record.
    rows = [json.loads(line) for line in lines if line.endswith(b"\n") and line.strip()]
    rows = rows[-count:]
    if len(rows) < 3:
        raise ValueError("Need at least three complete recorded orientation frames")
    return rows


def load_recording(session, *, start_frame=None, frame_count=120):
    """Validate configuration/model/session/epoch binding before any replay."""
    from vt_fsglove12.calibration import validate_rest
    from vt_fsglove12.config import digest, validate_config

    session = Path(session).resolve()
    config = validate_config(_read_json(session / "config.json"))
    rest = _read_json(session / "rest.json")
    validate_rest(rest)
    profile = _read_json(session / "profile.v001.json")
    metadata = _read_json(session / "session_metadata.json")
    identity = metadata["session_id"]
    config_hash, rest_hash = digest(config), digest(rest)
    if (profile.get("session_id") != identity or profile.get("config_sha256") != config_hash
            or profile.get("rest_sha256") != rest_hash
            or metadata.get("config_sha256") != config_hash or metadata.get("model_rest_sha256") != rest_hash):
        raise ValueError("Recorded profile/config/rest/session binding does not match")
    if start_frame is None:
        frames = _complete_tail(session / "orientations.jsonl", count=frame_count)
    else:
        # Explicit previously verified range, useful after a live session's
        # newest frames have changed epoch. Never relabel mismatched epochs.
        frames = []
        with (session / "orientations.jsonl").open(encoding="utf-8") as stream:
            for line in stream:
                if len(line) > 1024 * 1024:
                    raise ValueError("Oversized orientation record")
                if not line.endswith("\n"):
                    break
                frame = json.loads(line)
                index = frame.get("frame_index", -1)
                if index >= start_frame + frame_count:
                    break
                if index >= start_frame:
                    frames.append(frame)
        if [frame.get("frame_index") for frame in frames] != list(range(start_frame, start_frame + frame_count)):
            raise ValueError("Requested complete contiguous recorded frame range is unavailable")
    previous = -math.inf
    for frame in frames:
        if (frame.get("session_id") != identity or frame.get("config_sha256") != config_hash
                or frame.get("model_rest_sha256") != rest_hash
                or frame.get("orientation_source") != "vqf_6d"
                or frame.get("orientation_frame") != "independent_initial_heading"):
            raise ValueError("Recorded orientation identity does not match the supplied profile")
        timestamp = frame["timestamp_s"]
        if not isinstance(timestamp, (float, int)) or not math.isfinite(timestamp) or timestamp < previous:
            raise ValueError("Recorded orientation timestamps are invalid or decreasing")
        previous = timestamp
        for node, params in profile["nodes"].items():
            data = frame["nodes"].get(node)
            if data is not None and data.get("epoch") != params["epoch"]:
                raise ValueError(f"Recorded node {node} no longer matches the calibration epoch")
    return config, rest, profile, metadata, frames


class RecordedRuntime:
    """No-device adapter matching production's serial processing-thread cadence.

    latest_snapshot only copies the current state; the process IPC poll interval
    must not become an extra delay after each solve/render computation.
    """

    def __init__(self, config, rest, axes, *, output_root, geometry_backend, session_path,
                 start_frame=None, frame_count=120):
        from vt_fsglove12.config import digest
        from vt_fsglove12.solver import SparseHandSolver

        recorded_config, recorded_rest, self.profile, metadata, self.frames = load_recording(
            session_path, start_frame=start_frame, frame_count=frame_count)
        if digest(config) != digest(recorded_config) or digest(rest) != digest(recorded_rest):
            raise ValueError("Worker configuration/rest differ from recorded bindings")
        self.backend, self.solver = geometry_backend, SparseHandSolver(config, rest["rest_global_xyzw"])
        self.config, self.session_path = config, str(Path(session_path).resolve())
        self.rest = deepcopy(rest)
        self.source_session_id = metadata["session_id"]
        self.session_id = "performance-replay:" + self.source_session_id
        self.connected = self.stopped = False
        self.index, self.next_tick = 0, 0.
        self._lock = threading.Lock()
        self._stop_event, self._wake = threading.Event(), threading.Event()
        self._thread = None
        self._revision = 0
        self.state = dict(runtime=dict(connected=False, session_id=self.session_id,
            input_kind="recorded_orientation_performance_replay", hardware_accessed=False,
            published_monotonic_s=time.monotonic(), last_sample_age_s=None,
            received_samples=0, message="录制数据性能回放，非现场设备", error=None,
            output_dir=self.session_path), orientation=None, pose=None, geometry=None,
            geometry_error="", calibration=dict(active=None, captured_steps=["neutral", "fingers_down", "thumb_up"],
                calibrated_nodes=sorted(self.profile["nodes"], key=int), profile_status=self.profile["status"],
                rejected_nodes=self.profile.get("rejected_nodes", {}), warnings=[
                    "仅离线性能回放；循环记录窗口，不代表动作准确度、现场数据年龄或硬件延迟。"] ))

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="recorded-performance-processing", daemon=True)
            self._thread.start()
        return self

    def snapshot_revision(self):
        """Cheap change notification; polling never copies or ages geometry."""
        with self._lock:
            return self._revision

    def submit(self, name, **args):
        if name == "connect" and not self.stopped:
            if str(Path(args.get("port", "")).resolve()) != str(Path(self.session_path) / "orientations.jsonl"):
                raise ValueError("This adapter only accepts its recorded orientations.jsonl; no devices")
            with self._lock:
                self.connected = True
                self._revision += 1
            self._wake.set()
            return True
        if name == "disconnect":
            with self._lock:
                self.connected = False
                self.state["runtime"]["connected"] = False
                self.state.update(pose=None, geometry=None)
                self._revision += 1
            self._wake.set()
            return True
        return False

    def latest_snapshot(self):
        with self._lock:
            state = deepcopy(self.state)
        frame = state.get("orientation")
        if frame is not None:
            elapsed = max(0., time.monotonic() - frame["timestamp_s"])
            state["runtime"]["last_sample_age_s"] = elapsed
            frame["nodes"] = {node: {**data, "age_s": data.get("age_s", 0.) + elapsed}
                              for node, data in frame["nodes"].items()}
        return state

    def _run(self):
        period = 1. / min(30., self.config["output_rate_hz"])
        while not self._stop_event.is_set():
            if not self.connected:
                self._wake.wait(.01)
                self._wake.clear()
                continue
            started = time.monotonic()
            try:
                self._compute()
            except Exception as exc:
                with self._lock:
                    self.connected = False
                    self.state["runtime"].update(connected=False, error=f"Recorded replay failed: {exc}")
                    self.state.update(pose=None, geometry=None)
                    self._revision += 1
            # Same no-catch-up contract as the production processing thread:
            # computing consumes the period, rather than adding a full wait.
            self._wake.wait(max(0., started + period - time.monotonic()))
            self._wake.clear()

    def _compute(self):
        from vt_fsglove12.calibration import apply_calibration
        from vt_fsglove12.functional_axes import apply_functional_axes

        now = time.monotonic()
        started = time.perf_counter()
        source = self.frames[self.index % len(self.frames)]
        frame = deepcopy(source)
        frame.update(timestamp_s=now, recorded_timestamp_s=source["timestamp_s"],
                     source_session_id=self.source_session_id, session_id=self.session_id,
                     frame_index=self.index, input_kind="recorded_orientation_performance_replay",
                     hardware_accessed=False, timestamps_rebased_for_performance=True)
        before = time.perf_counter()
        observations, rejected = apply_calibration(frame, self.profile)
        observations, functional = apply_functional_axes(observations, self.profile, self.rest)
        calibration_ms = (time.perf_counter() - before) * 1000.
        before = time.perf_counter()
        pose = self.solver.solve(observations, now)
        pose["functional_axis_retargeting"] = functional
        solve_ms = (time.perf_counter() - before) * 1000.
        pose.update(session_id=self.session_id, frame_index=self.index, profile_version=1,
                    calibration_rejections=rejected, input_kind="recorded_orientation_performance_replay",
                    hardware_accessed=False, recorded_timestamp_s=source["timestamp_s"])
        before = time.perf_counter()
        geometry = self.backend.render(pose["anatomical_euler_rad"], pose["root_quat_xyzw"]) if pose["pose_available"] else None
        mesh_ms = (time.perf_counter() - before) * 1000.
        finished = time.monotonic()
        with self._lock:
            if not self.connected or self.stopped:
                return
            self.state.update(orientation=frame, pose=pose, geometry=geometry)
            self.state["runtime"].update(connected=True, published_monotonic_s=finished,
                last_sample_age_s=finished - now, received_samples=(self.index + 1) * len(frame["nodes"]),
                timing_ms=dict(calibration=calibration_ms, solve=solve_ms, mesh=mesh_ms,
                               total=(time.perf_counter() - started) * 1000.))
            self._revision += 1
        self.index += 1

    def stop(self):
        self._stop_event.set()
        self._wake.set()
        with self._lock:
            self.stopped, self.connected = True, False
            self.state["runtime"]["connected"] = False
            self.state.update(pose=None, geometry=None)
            self._revision += 1

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout)
        return self._thread is None or not self._thread.is_alive()


def _stats(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"count": 0}
    return dict(count=len(values), mean=float(values.mean()), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), maximum=float(values.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mano-assets", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=8.)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-frame", type=int, help="Explicit verified source frame range instead of the newest tail")
    parser.add_argument("--frames", type=int, default=120, help="Number of complete orientation records to loop (3..500)")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error("duration must be between 1 and 60 seconds")
    if args.output.exists():
        parser.error("output already exists; refusing to replace it")
    if not 3 <= args.frames <= 500 or args.start_frame is not None and args.start_frame < 0:
        parser.error("frames must be 3..500 and start-frame nonnegative")
    for variable in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        if os.environ.get(variable) != "1":
            parser.error(f"Set {variable}=1 before launching this benchmark")
    config, rest, profile, metadata, frames = load_recording(
        args.session, start_frame=args.start_frame, frame_count=args.frames)

    from PyQt5 import QtCore, QtWidgets
    from vt_fsglove12.guided_calibration import model_motion_axes
    from vt_fsglove12.live_process import ProcessLiveController
    from vt_fsglove12.live_ui import LiveWindow
    from vt_fsglove12.mano import ManoBackend

    backend = ManoBackend(args.mano_assets)
    exported = backend.export_rest()
    # Provenance file paths may differ across installs; compare the actual model frames.
    if exported["parents"] != rest["parents"] or not np.allclose(exported["rest_global_xyzw"], rest["rest_global_xyzw"], atol=1e-6):
        raise ValueError("Supplied MANO asset does not match the recorded rest frames")
    axes = model_motion_axes(backend.render(np.zeros((16, 3)))["joints_m"])
    controller = ProcessLiveController(config, rest, axes, output_root=args.output.parent,
        mano_assets=args.mano_assets,
        runtime_factory=partial(RecordedRuntime, session_path=str(args.session.resolve()),
                                # Freeze the selected range even when the source
                                # recording is still growing during child startup.
                                start_frame=frames[0]["frame_index"], frame_count=len(frames)))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    measurements = dict(start=None, first_pose=None, done=False, heartbeat=[], refresh=[], pose_age=[],
                        worker_solve=[], worker_mesh=[], worker_total=[], paint=[], swaps=[], updates=0,
                        previous_heartbeat=None, seen_worker=set(), gl_valid=False)

    class MeasuredWindow(LiveWindow):
        def refresh(self):
            previous_token = getattr(self, "_last_pose", None)
            before = time.perf_counter()
            super().refresh()
            now = time.monotonic()
            if self._last_pose is not None and measurements["first_pose"] is None:
                measurements["first_pose"] = now
            if measurements["start"] is not None and not measurements["done"]:
                measurements["refresh"].append((time.perf_counter() - before) * 1000.)
                if self._last_pose is not None and self._last_pose != previous_token:
                    measurements["updates"] += 1
                    measurements["pose_age"].append(self._pose_age_ms)
                state = self.controller.latest_snapshot()
                token = (state.get("pose") or {}).get("frame_index")
                if token is not None and token not in measurements["seen_worker"]:
                    measurements["seen_worker"].add(token)
                    timing = state["runtime"].get("timing_ms", {})
                    for key in ("solve", "mesh", "total"):
                        if key in timing:
                            measurements["worker_" + key].append(timing[key])

    window = MeasuredWindow(config, backend, output_root=args.output.parent,
        replay_raw=args.session / "orientations.jsonl", controller=controller)
    window.mode.setText("录制数据性能回放，非现场设备；不连接串口")
    window.setWindowTitle("离线性能测量 · 不操作现场设备")
    paint_original = type(window.view).paintGL
    def measured_paint(view, *paint_args, **paint_kwargs):
        before = time.perf_counter()
        value = paint_original(view, *paint_args, **paint_kwargs)
        if measurements["start"] is not None and not measurements["done"]:
            measurements["paint"].append((time.perf_counter() - before) * 1000.)
        return value
    type(window.view).paintGL = measured_paint
    window.view.frameSwapped.connect(lambda: measurements["swaps"].append(time.monotonic())
        if measurements["start"] is not None and not measurements["done"] else None)
    window.show()
    launched = time.monotonic()
    end_reason = {"reason": "window_closed_early"}
    heartbeat = QtCore.QTimer()
    def tick():
        now = time.monotonic()
        if measurements["start"] is None:
            first = measurements["first_pose"]
            if first is not None and now - first >= 2.:
                measurements["start"] = now
                measurements["previous_heartbeat"] = now
                measurements["gl_valid"] = bool(window.view.isValid())
            elif now - launched > 25:
                end_reason["reason"] = "no_fresh_pose_within_25_seconds"
                heartbeat.stop(); window.close(); app.quit()
        elif not measurements["done"]:
            measurements["heartbeat"].append((now - measurements["previous_heartbeat"]) * 1000.)
            measurements["previous_heartbeat"] = now
            if now - measurements["start"] >= args.duration:
                measurements["done"] = True
                measurements["elapsed"] = now - measurements["start"]
                end_reason["reason"] = "completed"
                heartbeat.stop(); window.close(); app.quit()
    heartbeat.timeout.connect(tick)
    heartbeat.start(10)
    # This explicit benchmark action clicks only its own replay button.
    QtCore.QTimer.singleShot(0, window.connect_button.click)
    app.exec_()
    controller.stop()
    worker_ended = controller.join(5.) and not controller.is_alive()
    elapsed = measurements.get("elapsed", 0.)
    report = dict(schema_version=1, kind="recorded_orientation_live_ui_performance",
        replay_scheduler="serial_processing_thread_with_latest_only_snapshot",
        status=end_reason["reason"], session=str(args.session.resolve()), source_session_id=metadata["session_id"],
        input_frames=len(frames), input_frame_indices=[frames[0]["frame_index"], frames[-1]["frame_index"]],
        profile="profile.v001.json", warmup_after_first_pose_s=2., measured_duration_s=elapsed,
        hardware_accessed=False, original_live_process_untouched=True, worker_ended=worker_ended,
        real_gl_context_valid=measurements["gl_valid"], qt_platform=app.platformName(),
        ui_heartbeat_interval_ms=_stats(measurements["heartbeat"]),
        ui_refresh_ms=_stats(measurements["refresh"]),
        submitted_replay_pose_age_ms=_stats(measurements["pose_age"]),
        distinct_mesh_submissions=measurements["updates"],
        distinct_mesh_submission_hz=measurements["updates"] / elapsed if elapsed else 0.,
        observed_worker_solve_ms=_stats(measurements["worker_solve"]),
        observed_worker_mesh_ms=_stats(measurements["worker_mesh"]),
        observed_worker_total_ms=_stats(measurements["worker_total"]),
        paintGL_wall_ms=_stats(measurements["paint"]),
        frame_swapped_hz=len(measurements["swaps"]) / elapsed if elapsed else 0.,
        limitations=["Recorded orientations are looped with rebased timestamps; this is not physical sensor latency.",
                     "Mesh submissions and frameSwapped are not photon-level presentation measurements.",
                     "paintGL wall time includes CPU/driver work and is not an isolated GPU timer.",
                     "Worker timings cover observed distinct snapshots; latest-only delivery can skip intermediate poses.",
                     "No physical accuracy, serial ingestion, live fusion, or serial-recording cost is tested."])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 0 if report["status"] == "completed" and worker_ended and report["real_gl_context_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
