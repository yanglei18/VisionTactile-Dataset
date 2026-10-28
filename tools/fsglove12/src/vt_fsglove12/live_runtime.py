"""Manual-only threaded acquisition controller, independent of any GUI.

The reader owns its source and raw recorder. Fusion has a short shared lock;
the processing thread samples its latest value, never queues historical poses.
Sources must implement bounded reads so a stop request can finish promptly.
No device is opened until an explicit connect command is submitted.
"""
from __future__ import annotations

from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import tempfile
import threading
import time
from typing import Any
from uuid import uuid4

from . import host_clock
from .calibration import apply_calibration, validate_rest
from .calibration_artifacts import artifact_size, resume_artifact, write_artifact
from .config import digest, validate_config
from .display_pose import DisplayPoseCache
from .input_contract import describe_input_contract
from .live_recording import AsyncResultRecorder
from .recording_policy import ResultRecordingPolicy, SessionStorageBudget
from .live_shape import ShapeCalibrationMixin, SHAPE_COMMANDS, valid_shape_command
from .scale_capture_buffer import ScaleCaptureBuffer, ScaleHistoryEvictedError
from .live_stream import OnlineFusion as _OnlineFusion
from .neutral_reference import reanchor_neutral_reference
from .functional_axes import apply_functional_axes, fit_functional_finger
from .functional_capture import FunctionalReferenceCapture
from .upstream_functional import FINGER_SLOTS as OFFICIAL_FUNCTIONAL_SLOTS
from .solver import SparseHandSolver

STEP_KEYS = ("neutral", "fingers_down", "thumb_up")
PREPARE_SECONDS, CAPTURE_SECONDS = 3.0, 2.0
COMMAND_CAPACITY, CAPTURE_FRAME_LIMIT = 16, 120
CALIBRATION_PENDING_LIMIT, CALIBRATION_PENDING_BYTES = 12, 32 * 1024 * 1024
# Only used while an official mounting-calibration trial is in progress.
# RawSample is immutable; this ring holds references, never filtered values.
INPUT_SCALE_MAX_SAMPLES, INPUT_SCALE_HISTORY_S = 180000, 90.
FUNCTIONAL_FINGERS = ("thumb", "little")
OFFICIAL_FUNCTIONAL_FINGERS = ("index", "middle", "ring", "little", "thumb")
FINGER_NAMES = dict(index="食指", middle="中指", ring="无名指", little="小指", thumb="拇指")
FUNCTIONAL_MOTION_SECONDS = 8.0
# Capture the audited implementations, not a potentially monkeypatched class
# attribute. Injected/subclass readers retain the conservative copy boundary.
_OWNED_FUSION_SNAPSHOT = _OnlineFusion.snapshot
_OWNED_FUSION_INPUT_SNAPSHOT = _OnlineFusion.input_snapshot


def _sample_capture_frame(active, key, frame, now, start, end, limit):
    """Keep the latest real snapshot in each fixed, whole-phase time bin.

    The bin budget belongs to physical time, not solver ticks. Replacing a bin
    uses a deep copy of the CURRENT snapshot, never ages/merges old node data.
    Identities in earlier bins are excluded, while asynchronous nodes newly
    observed within this bin remain eligible in its latest snapshot.
    """
    indices = {node: (data.get("epoch"), data.get("sample_index"))
               for node, data in frame["nodes"].items()}
    if active["seen"] is None:
        active["seen"] = indices
        return
    previous_seen = dict(active["seen"])
    active["seen"].update(indices)

    def usable(node, data):
        health = frame.get("input_health", {})
        status = health.get("nodes", {}).get(node, data.get("input_health", {}))
        qualified = (not health.get("enabled", False) and not status
                     or status.get("state") == "healthy" and status.get("healthy") is True)
        return (type(data.get("sample_index")) is int and data["sample_index"] > 0
                and qualified
                and data.get("orientation_valid") is not False
                and not data.get("input_filter", {}).get("gyro_rejected")
                and not data.get("input_filter", {}).get("accel_rejected"))

    if not any(usable(node, data) and indices[node] != previous_seen.get(node)
               for node, data in frame["nodes"].items()):
        return
    bucket = min(limit - 1, max(0, int((now - start) * limit / (end - start))))
    state = active.setdefault("sampling", {}).setdefault(
        key, dict(bucket=None, baseline=previous_seen, row_index=None))
    rows = active[key]
    if bucket != state["bucket"]:
        if state["row_index"] is not None:
            state["baseline"].update({node: (data.get("epoch"), data["sample_index"])
                                      for node, data in rows[state["row_index"]]["nodes"].items()})
        state.update(bucket=bucket, row_index=None)
    nodes = {node: data for node, data in frame["nodes"].items()
             if usable(node, data) and indices[node] != state["baseline"].get(node)}
    if not nodes:
        return
    row = deepcopy({**frame, "nodes": nodes})
    if state["row_index"] is None:
        if len(rows) >= limit:
            return  # Defensive bound if a caller supplies nonmonotonic time.
        state["row_index"] = len(rows)
        rows.append(row)
    else:
        rows[state["row_index"]] = row


def _source(port):
    from .live_stream import SerialSource
    return SerialSource(port)


def _fusion(config):
    from .live_stream import OnlineFusion
    return OnlineFusion(config)


def _recorder(path):
    from .live_stream import RawRecorder
    return RawRecorder(path)


def _fit(*args, **kwargs):
    from .guided_calibration import fit_guided_calibration
    return fit_guided_calibration(*args, **kwargs)


def _write_json(path, value):
    """Atomically create complete JSON, never overwriting an existing file."""
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write("\n")
        # Closing flushes the document before exposing its final name. A hard
        # link atomically publishes it with the original open('x') no-clobber
        # semantics; an existing destination still raises FileExistsError.
        os.link(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _unavailable(reason, timestamp):
    return dict(timestamp_s=timestamp, quality="unavailable", reason=reason,
                pose_available=False, solver_success=False, anatomical_euler_rad=None,
                root_quat_xyzw=None, predicted_global_xyzw=None,
                source_labels=["unavailable"] * 16, inferred_mask=[False] * 16,
                observed_mask=[False] * 16,
                segment_observed_mask=[False] * 16, joint_observed_mask=[False] * 16,
                anatomical_accuracy_verified=False, yaw_drift_corrected=False)


def _input_wait_message(diagnostics):
    """Explain receive/parse evidence without guessing the attached hardware."""
    if not diagnostics or diagnostics.get("source_kind") != "raw_serial":
        return None
    if diagnostics.get("bytes_received", 0) == 0:
        return "串口已打开，但尚未收到任何字节；请检查 IMU 采集板供电与连接，暂不标定。"
    if diagnostics.get("samples_decoded", 0) == 0:
        if diagnostics.get("bytes_discarded", 0) == 0:
            return "已收到串口字节，正在等待完整 IMU 帧；暂不标定。"
        return "已收到串口字节，但尚未解析到有效 IMU 帧；请核对 IMU 串口与数据协议，暂不标定。"
    return None


def _input_diagnostics(source, now):
    """Optional passive counters; file replay and legacy test sources need none."""
    reader = getattr(source, "snapshot_diagnostics", None)
    if not callable(reader):
        return None
    return {**reader(now=now), "observed_monotonic_s": now}


def _fusion_snapshot(fusion, now, *, input_only=False):
    """Read under the caller's fusion lock, retaining injected-reader ownership.

    OnlineFusion creates new containers and copies every mutable filter leaf.
    Its input reader also delegates to snapshot, so both bound implementations
    must be the audited originals before omitting the redundant deep copy.
    """
    snapshot = fusion.snapshot
    reader = getattr(fusion, "input_snapshot", snapshot) if input_only else snapshot
    owns_values = (type(fusion) is _OnlineFusion
                   and getattr(snapshot, "__func__", None) is _OWNED_FUSION_SNAPSHOT
                   and (not input_only
                        or getattr(reader, "__func__", None) is _OWNED_FUSION_INPUT_SNAPSHOT))
    frame = reader(now)
    return frame if owns_values else deepcopy(frame)


def _age_input_snapshot(snapshot, elapsed, stale_after_s):
    """Age an owned, lightweight input copy exactly once across a transport."""
    runtime = snapshot["runtime"]
    if runtime.get("last_sample_age_s") is not None:
        runtime["last_sample_age_s"] += elapsed
    diagnostics = runtime.get("input_diagnostics")
    if isinstance(diagnostics, dict):
        for key in ("last_byte_age_s", "last_sample_age_s"):
            if diagnostics.get(key) is not None:
                diagnostics[key] += elapsed
    frame = snapshot.get("orientation")
    if frame is not None:
        frame["nodes"] = {node: {**data, "age_s": data.get("age_s", 0.) + elapsed}
            for node, data in frame.get("nodes", {}).items()
            if data.get("age_s", float("inf")) + elapsed <= stale_after_s}
        if diagnostics is not None:
            frame["input_diagnostics"] = deepcopy(diagnostics)
    return snapshot


@dataclass
class _Session:
    session_id: str
    directory: Path
    port: str
    source: Any
    fusion: Any
    recorder: Any
    started: float
    streams: dict = field(default_factory=dict)
    lock: Any = field(default_factory=threading.Lock)
    stop: Any = field(default_factory=threading.Event)
    done: Any = field(default_factory=threading.Event)
    reader: Any = None
    opened: bool = False
    source_opened: bool = False
    error: str | None = None
    eof: bool = False
    received: int = 0
    recorded: int = 0
    last_sample: float | None = None
    node_counts: Any = field(default_factory=Counter)
    frames: int = 0
    available_poses: int = 0
    profiles: list = field(default_factory=list)
    last_flush: float = 0.0
    metadata_written: bool = False
    result_recorder: Any = None
    last_input_signature: Any = None
    last_sample_ids: dict = field(default_factory=dict)
    last_pose_context: Any = None
    last_pose_available: bool = False
    duplicate_inputs_skipped: int = 0
    processed_cycles: int = 0
    overrun_cycles: int = 0
    max_cycle_ms: float = 0.
    fresh_input_results: int = 0
    state_only_results: int = 0
    invalid_results: int = 0
    solve_times: Any = field(default_factory=deque)
    scale_samples: Any = None
    recording_disabled: Any = field(default_factory=threading.Event)
    recording_lock: Any = field(default_factory=threading.RLock)
    recording_errors: list = field(default_factory=list)
    recording_policy: Any = None
    storage_budget: Any = None
    profile_version: int = 0
    profile_saved: Any = None
    calibration_pending: list = field(default_factory=list)
    calibration_directory: Path | None = None
    calibration_storage: dict = field(default_factory=lambda: dict(
        saved_count=0, last_saved_path=None, error=None, evicted_count=0))


@dataclass
class _DetachedCalibrationSaves:
    """Only bounded persistence state survives disconnect, never acquisition."""
    session_id: str
    directory: Path
    profile_version: int
    profile_saved: Any
    calibration_pending: list
    calibration_directory: Path | None
    calibration_storage: dict
    profiles: list = field(default_factory=list)


class LiveController(ShapeCalibrationMixin):
    """Start idle; submit explicit commands and poll deep-copied snapshots.

    Optional factories are dependency-injection seams, not alternate synthetic
    models. ``join`` returns True only when processing and reader threads ended.
    A timed-out reader is never killed, and blocks any subsequent connection.
    ``record_streams=False`` keeps configuration, bounded calibration evidence
    and events, without creating raw/pose/orientation continuous recorders.
    """

    def __init__(self, config, rest, model_axes, *, output_root,
                 source_factory=None, clock=host_clock.monotonic, fusion_factory=None,
                 solver_factory=None, recorder_factory=None, calibration_fn=None,
                 geometry_backend=None, pose_backend=None, pose_exporter=None,
                 result_recorder_factory=None, input_scale_job_factory=None, shape_job_factory=None,
                 record_streams=True):
        if type(record_streams) is not bool:
            raise ValueError("record_streams must be a boolean")
        self.record_streams = record_streams
        self.config = validate_config(deepcopy(config))
        ResultRecordingPolicy(self.config)  # Validate logging settings before opening hardware.
        validate_rest(rest)
        from .config import check_model_hand
        check_model_hand(self.config, rest)
        self.rest, self.model_axes = deepcopy(rest), deepcopy(model_axes)
        self.output_root = Path(output_root)
        self._clock = clock
        self._source_factory, self._fusion_factory = source_factory or _source, fusion_factory or _fusion
        self._recorder_factory = recorder_factory or _recorder
        # Optional writer owns unopened result paths; legacy/injected runtimes
        # keep the thread writer. Raw CSV ownership is unchanged.
        self._result_recorder_factory = result_recorder_factory
        self._calibration_fn = calibration_fn or _fit
        self._geometry_backend = geometry_backend
        self._pose_backend = pose_backend
        self._pose_exporter = pose_exporter
        self.renders_geometry = geometry_backend is not None
        # The official path never constructs or falls back to the legacy solver.
        self._solver = None if pose_backend is not None else (solver_factory or SparseHandSolver)(
            self.config, self.rest["rest_global_xyzw"])
        self._config_sha, self._rest_sha = digest(self.config), digest(self.rest)
        self._root_node = next(node for node, slot in self.config["node_to_slot"].items() if slot == 0)
        self._commands = queue.Queue(maxsize=COMMAND_CAPACITY)
        self._stop, self._wake = threading.Event(), threading.Event()
        self._state_lock = threading.Lock()
        self._thread = None
        self._session = None
        self._calibration_detached = None
        self._readers = []
        self._writers = []
        self._captures, self._active, self._profile = {}, None, None
        self._functional_hand_cancel, self._functional_hand_result = None, None
        self._profile_status, self._warnings, self._rejected = "not_calibrated", [], {}
        self._clear_generation = 0
        self._input_scale_generation = 0
        self._scale_token = 0
        self._scale_holds, self._scale_pending = {}, None
        self._scale_job, self._scale_job_context = None, None
        self._scale_job_factory = input_scale_job_factory
        self._scale_report = None
        self._scale_state = dict(status="waiting", candidate_available=False,
            message="完成三姿态后自动检查输入尺度；不需要手长或精确角度。")
        self._display_pose_cache, self._display_context = DisplayPoseCache(), None
        self._revision = 0
        self._init_shape(shape_job_factory)
        self._snapshot = dict(
            runtime=dict(connected=False, port=None, message="未连接；请手动选择端口并连接。", error=None,
                         session_id=None, output_dir=None, received_samples=0,
                         input_diagnostics=None,
                         active_input_contract=describe_input_contract(self.config),
                         input_scale_generation=0,
                         command_completed_generation=0, last_command_name=None,
                         last_command_error=None,
                         shape_command_generation=0, shape_command_error=None,
                         calibration_storage=self._calibration_storage_status(None),
                         last_sample_age_s=None, published_monotonic_s=self._clock(),
                         timing_ms=dict(solve=0., calibration=0., mesh=0., total=0.)),
            orientation=None, pose=None, geometry=None, geometry_error="",
            display_pose=None, display_geometry=None,
            calibration=self._calibration_state(self._clock()))
        self._input_snapshot = dict(runtime=deepcopy(self._snapshot["runtime"]), orientation=None)

    def start(self):
        if self._stop.is_set():
            raise RuntimeError("已停止的 LiveController 不能重新启动；请创建新的控制器。")
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="fsglove12-processing", daemon=True)
            self._thread.start()
        return self

    def submit(self, name, **args) -> bool:
        valid = {"connect": {"port"}, "disconnect": set(), "capture": {"step"},
                 "cancel_capture": set(), "clear_calibration": set(), "reanchor_neutral": set(),
                 "capture_functional": {"finger"}, "set_functional_enabled": {"enabled"},
                 "capture_functional_hand": set(), "use_input_scale_candidate": set()}
        valid.update(SHAPE_COMMANDS)
        if name not in valid or set(args) != valid[name] or self._stop.is_set():
            return False
        if name in SHAPE_COMMANDS and (self._pose_backend is None or not valid_shape_command(name, args)):
            return False
        if self._pose_backend is not None and name == "reanchor_neutral":
            return False  # The official matrices do not support legacy reanchoring.
        if name == "set_functional_enabled" and (self._pose_backend is None or type(args["enabled"]) is not bool):
            return False
        if name == "capture_functional_hand" and self._pose_backend is None:
            return False
        if name == "connect" and (not isinstance(args["port"], str) or not args["port"].strip()):
            return False
        if name == "capture" and args["step"] not in STEP_KEYS:
            return False
        supported_fingers = OFFICIAL_FUNCTIONAL_FINGERS if self._pose_backend is not None else FUNCTIONAL_FINGERS
        if name == "capture_functional" and args["finger"] not in supported_fingers:
            return False
        try:
            self._commands.put_nowait((name, deepcopy(args)))
        except queue.Full:
            return False
        if name in {"cancel_capture", "clear_calibration", "disconnect"}:
            # The processing thread may be inside a slow fit. This Event belongs
            # only to the current whole-hand capture; other flows are unchanged.
            cancel = self._functional_hand_cancel
            if cancel is not None:
                cancel.set()
        if name in {"cancel_shape", "clear_calibration", "disconnect"} and self._shape_job is not None:
            self._shape_job.cancel()
        if name == "disconnect" and self._session is not None:
            self._session.stop.set()
            self._invalidate("断开请求已提交；正在关闭当前会话。")
        self._wake.set()
        return True

    def latest_pose_snapshot(self):
        """Pose IPC already has an independent input mailbox; do not recopy it."""
        return self.latest_snapshot(include_input=False)

    def latest_snapshot(self, *, include_input=True):
        """Return deep-independent values, even while fitting/solving is busy."""
        result = deepcopy(self._latest_pose_publication())
        if include_input:
            result["input_snapshot"] = self.latest_input_snapshot()
        return result

    def _latest_pose_publication(self):
        """Borrow replacement-only payloads solely for synchronous child IPC.

        PRIVATE, READ-ONLY: pose/calibration/geometry and nested sensor values
        belong to the controller. The trusted child publisher may only pickle
        this result synchronously, not expose it to a GUI or mutate its leaves.
        Pickle creates the independent receiving-process ownership boundary.
        Public latest[_pose]_snapshot callers continue to receive deep copies.

        Only top-level/runtime/frame/node headers are detached, because aging
        and the publisher's command-error overlay modify those headers. The
        controller replaces all published payloads; none are edited in place.
        """
        now = self._clock()
        with self._state_lock:
            # Published payloads are replacement-only. Detach the two mutable
            # headers while locked, then copy potentially large geometry and
            # diagnostics without blocking reader counters or worker commits.
            result = {**self._snapshot, "runtime": dict(self._snapshot["runtime"])}
            session = self._session
            last = session.last_sample if session else None
        runtime = result["runtime"]
        if session is not None and session.result_recorder is not None:
            runtime["result_recording"] = session.result_recorder.snapshot()
        runtime["last_sample_age_s"] = max(0.0, now - last) if last is not None else None
        frame = result["orientation"]
        if frame is not None:
            frame = result["orientation"] = dict(frame)
            elapsed = max(0.0, now - frame["timestamp_s"])
            frame["nodes"] = {node: {**data, "age_s": data.get("age_s", 0.) + elapsed}
                              for node, data in frame["nodes"].items()
                              if data.get("age_s", float("inf")) + elapsed <= self.config["stale_after_s"]}
        disconnected = (self._stop.is_set() or not runtime["connected"]
                        or (session is not None and (session.stop.is_set() or session.done.is_set())))
        if disconnected or not frame or self._root_node not in frame["nodes"]:
            reason = "disconnected" if disconnected else "missing_or_stale_root"
            metadata = {key: value for key, value in (result.get("pose") or {}).items()
                        if key in {"schema_version", "session_id", "frame_index", "config_sha256",
                                   "model_rest_sha256", "profile_version", "input_kind", "hardware_accessed",
                                   "source_monotonic_ns", "source_reference_node"}}
            result["pose"] = {**metadata, **_unavailable(reason, now)}
            result["geometry"] = None
            if reason == "disconnected":
                runtime["connected"] = False
                result["display_pose"] = result["display_geometry"] = None
            if reason == "missing_or_stale_root":
                runtime["message"] = (_input_wait_message(runtime.get("input_diagnostics")) or
                                      "手掌数据缺失或已过期；隐藏旧姿态，等待新数据。")
        if not (result.get("pose") or {}).get("pose_available"):
            result["geometry"] = None
        return result

    def latest_input_snapshot(self):
        """Reader/fusion monitoring, independent of solving, fitting and MANO.

        Only bounded sensor state is copied. If fusion is committing a batch,
        return the last input snapshot aged to now rather than block a GUI or
        IPC publisher on that batch. This never opens a source or runs a filter.
        """
        now = self._clock()
        with self._state_lock:
            session = self._session
            runtime = {key: value for key, value in self._snapshot["runtime"].items()
                       if key not in {"timing_ms", "ros_export"}}
            # This cache and its runtime header are replaced, never modified.
            # A successful fusion read need not copy the unused old frame.
            cached = self._input_snapshot
            frame_generation = self._clear_generation
            # Compatibility for injected context-manager-only lock fixtures.
            fallback = (self._snapshot.get("orientation")
                        if session is not None and not callable(getattr(session.lock, "acquire", None))
                        else None)
        runtime = deepcopy(runtime)
        connected = bool(session is not None and session.opened and not session.stop.is_set()
                         and not session.done.is_set() and not self._stop.is_set())
        frame, diagnostics = None, runtime.get("input_diagnostics")
        if connected:
            acquire = getattr(session.lock, "acquire", None)
            if callable(acquire) and acquire(False):
                try:
                    now = self._clock()
                    frame = _fusion_snapshot(session.fusion, now, input_only=True)
                    frame_generation = self._clear_generation
                finally:
                    session.lock.release()
                diagnostics = _input_diagnostics(session.source, now)
            elif (cached["runtime"].get("session_id") == session.session_id
                  and cached["runtime"].get("clear_generation", 0) == self._clear_generation):
                aged = _age_input_snapshot(deepcopy(cached),
                    max(0., now - cached["runtime"].get("published_monotonic_s", now)),
                    self.config["stale_after_s"])
                frame, diagnostics = aged["orientation"], aged["runtime"].get("input_diagnostics")
                frame_generation = aged["runtime"].get("clear_generation", 0)
            if not callable(acquire) and fallback is not None:
                frame = _age_input_snapshot(dict(runtime={}, orientation=deepcopy(fallback)),
                    max(0., now - fallback["timestamp_s"]), self.config["stale_after_s"])["orientation"]
            if frame is not None:
                frame.update(session_id=session.session_id, config_sha256=self._config_sha,
                             model_rest_sha256=self._rest_sha, **self._source_tags(session))
                if diagnostics is not None:
                    frame["input_diagnostics"] = deepcopy(diagnostics)
        # A solve/mesh can be stale while the sole reader is still connected.
        runtime.update(connected=connected, published_monotonic_s=now,
            clear_generation=frame_generation, input_diagnostics=diagnostics,
            session_id=session.session_id if session is not None else runtime.get("session_id"),
            received_samples=session.received if session is not None else runtime.get("received_samples", 0),
            last_sample_monotonic_s=session.last_sample if session is not None else None,
            last_sample_age_s=(max(0., now - session.last_sample)
                if session is not None and session.last_sample is not None else None))
        result = dict(runtime=runtime, orientation=frame)
        # Callers retain a fully independent mutable result. Cache its owned
        # copy outside the state lock, then validate the epoch/session at the
        # same commit boundary used by clear_calibration.
        cached_result = deepcopy(result)
        with self._state_lock:
            if session is self._session:
                if frame_generation != self._clear_generation:
                    # A clear can execute after the fusion lock is released
                    # (e.g. while copying passive source diagnostics). Never
                    # relabel an old epoch's frame as the completed new clear.
                    result["orientation"] = None
                    result["runtime"].update(clear_generation=self._clear_generation,
                        input_reset_race_discarded=True)
                    cached_result["orientation"] = None
                    cached_result["runtime"].update(clear_generation=self._clear_generation,
                        input_reset_race_discarded=True)
                if (session is None or not session.opened or session.stop.is_set()
                        or session.done.is_set() or self._stop.is_set()):
                    result["orientation"] = cached_result["orientation"] = None
                    result["runtime"]["connected"] = cached_result["runtime"]["connected"] = False
                self._input_snapshot = cached_result
            else:
                # Never combine one session's sensor values with another ID.
                result["orientation"] = None
                result["runtime"].update(connected=False, message="输入会话正在切换。")
        return result

    def snapshot_revision(self):
        """Cheap display-state token; raw receive counters piggyback on frames.

        Polling this never copies geometry or ages a frame. Reader packet counts
        alone intentionally do not publish the same mesh at the IMU packet rate.
        """
        with self._state_lock:
            return self._revision

    def stop(self):
        self._stop.set()
        if self._shape_job is not None:
            self._shape_job.cancel()
        if self._scale_job is not None:
            self._scale_job.cancel()
        session = self._session
        if session is not None:
            session.stop.set()
        self._wake.set()
        self._invalidate("控制器停止中。")

    def join(self, timeout=None) -> bool:
        deadline = None if timeout is None else host_clock.monotonic() + max(0., timeout)
        # The processor may still be registering a job when stop() arrives.
        # Join it first, then inventory workers; a pre-join snapshot can omit
        # exactly the child we need to drain before reporting shutdown success.
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(None if deadline is None else max(0., deadline - host_clock.monotonic()))
        workers = [self._thread, *self._readers, *self._writers, self._scale_job, self._shape_job]
        for thread in workers:
            if thread is not None and thread is not threading.current_thread():
                remaining = None if deadline is None else max(0., deadline - host_clock.monotonic())
                thread.join(remaining)
        return not any(thread and thread.is_alive() for thread in workers)

    def _calibration_state(self, now):
        active = None
        if self._active is not None:
            preparing = now < self._active["collect_at"]
            active = dict(step=self._active["step"], phase="preparing" if preparing else "collecting",
                          kind=self._active.get("kind", "mounting"),
                          remaining_s=max(0., (self._active["collect_at"] if preparing else self._active["end_at"]) - now))
            if self._active.get("kind") == "functional":
                moving = now >= self._active["neutral_end_at"]
                active.update(finger=self._active["finger"],
                              phase="preparing" if preparing else "moving" if moving else "collecting_neutral",
                              remaining_s=max(0., (self._active["collect_at"] if preparing else
                                                  self._active["end_at"] if moving else self._active["neutral_end_at"]) - now))
            elif self._active.get("kind") == "functional_hand":
                phase, boundary = self._functional_hand_phase(self._active, now)
                active.update(phase=phase, remaining_s=max(0., boundary - now))
            reference = self._active.get("reference_capture")
            if reference is not None and reference.selected is None and not preparing:
                status = reference.summary(now)
                active.update(phase="collecting_neutral", reference_waiting=True,
                    remaining_s=max(0., reference.deadline_s - now),
                    reference_wait_elapsed_s=status["wait_elapsed_s"],
                    reference_wait_limit_s=status["wait_limit_s"],
                    reference_status=status["status"], unstable_slots=status["unstable_slots"])
        return dict(active=active, captured_steps=[key for key in STEP_KEYS if key in self._captures],
                    shape_calibration=self._shape_state(now),
                    clear_generation=self._clear_generation,
                    input_scale_check=deepcopy(self._scale_state),
                    functional_fingers=sorted((self._profile or {}).get("functional_fingers", {})),
                    functional_enabled=(self._profile or {}).get("functional_enabled", True),
                    functional_hand_result=deepcopy(self._functional_hand_result),
                    functional_diagnostics={finger: deepcopy(entry.get("diagnostics", {}))
                                            for finger, entry in (self._profile or {}).get("functional_fingers", {}).items()},
                    profile_status=self._profile_status,
                    motion_diagnostics=deepcopy((self._profile or {}).get("calibration_motion_diagnostics", {})),
                    calibrated_nodes=sorted((self._profile or {}).get("nodes", {}), key=int),
                    rejected_nodes=deepcopy(self._rejected), warnings=list(self._warnings))

    def _invalidate(self, message, error=None):
        self._display_pose_cache.reset()
        self._display_context = None
        with self._state_lock:
            self._snapshot["runtime"].update(connected=False, message=message, error=error,
                                              published_monotonic_s=self._clock())
            self._snapshot["pose"] = _unavailable("disconnected", self._clock())
            self._snapshot["orientation"] = None
            self._snapshot["geometry"] = None
            self._snapshot["display_pose"] = self._snapshot["display_geometry"] = None
            self._snapshot["geometry_error"] = ""
            self._revision += 1

    def _message(self, message, error=None):
        with self._state_lock:
            self._snapshot["runtime"].update(message=message, error=error)
            self._revision += 1

    @staticmethod
    def _line(stream, value):
        stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")

    def _local_recording_status(self, session):
        with session.recording_lock:
            errors = list(session.recording_errors)
            failed = session.recording_disabled.is_set()
            enabled = self.record_streams and not failed
        return dict(enabled=enabled, requested=self.record_streams,
            mode="streams" if self.record_streams else "calibration_only", errors=errors,
            warning=("" if not failed else "本地记录已暂停；实时采集、解算和 ROS 发布继续，"
                     "暂停后的数据未写入本地文件。原因：" + "；".join(errors)),
            received_samples=session.received, recorded_samples=session.recorded,
            profile_saved=session.profile_saved,
            policy=session.recording_policy.snapshot() if session.recording_policy else None,
            storage=session.storage_budget.snapshot() if session.storage_budget else None)

    @staticmethod
    def _calibration_storage_status(session):
        """Small calibration files have no dependency on the bulk-log latch."""
        state = (session.calibration_storage if session is not None else
                 dict(saved_count=0, last_saved_path=None, error=None, evicted_count=0))
        pending = session.calibration_pending if session is not None else ()
        error = state.get("error")
        warning = ("标定结果已保留在内存，但部分标定文件未保存；实时采集、解算和 ROS 发布继续。"
                   "可释放磁盘空间或选择其他目录后重试保存。原因：" + error if error else "")
        if state["evicted_count"]:
            warning += f" 待保存缓存已达到上限，{state['evicted_count']} 份旧文件未保留用于重试；当前标定仍在内存中。"
        return dict(**state, status="save_failed" if error else "saved" if state["saved_count"] else "ready",
                    pending_count=len(pending), pending_bytes=sum(row["bytes"] for row in pending),
                    source_session_id=session.session_id if session is not None else None,
                    detached=isinstance(session, _DetachedCalibrationSaves),
                    destination_dir=(str(session.calibration_directory or session.directory)
                                     if session is not None else None),
                    retry_available=bool(pending), warning=warning)

    def _publish_calibration_storage(self, session):
        status = self._calibration_storage_status(self._calibration_retry_session())
        with self._state_lock:
            if session is self._session or session is self._calibration_detached:
                self._snapshot["runtime"]["calibration_storage"] = status
                if self._session is not None:
                    self._snapshot["runtime"]["local_recording"] = self._local_recording_status(self._session)
                elif "local_recording" in self._snapshot["runtime"]:
                    self._snapshot["runtime"]["local_recording"]["profile_saved"] = session.profile_saved
                self._revision += 1

    def _calibration_retry_session(self):
        """Return the actual bounded owner behind the published retry action."""
        if self._session is not None and self._session.calibration_pending:
            return self._session
        if self._calibration_detached is not None and self._calibration_detached.calibration_pending:
            return self._calibration_detached
        return self._session if self._session is not None else self._calibration_detached

    def _retain_calibration_saves(self, session):
        previous = self._calibration_detached
        if previous is not None and previous.calibration_pending and not session.calibration_pending:
            return  # A clean subsequent session must not discard older evidence.
        state = dict(session.calibration_storage)
        if previous is not None and previous.calibration_pending:
            state["evicted_count"] += len(previous.calibration_pending) + previous.calibration_storage["evicted_count"]
        self._calibration_detached = _DetachedCalibrationSaves(
            session.session_id, session.directory, session.profile_version, session.profile_saved,
            session.calibration_pending, session.calibration_directory, state)
        session.calibration_pending = []  # Transfer, do not duplicate, the bounded documents.

    def _calibration_artifact_saved(self, session, entry, resolved):
        state = session.calibration_storage
        state.update(saved_count=state["saved_count"] + 1, last_saved_path=resolved)
        if entry["kind"] == "profile":
            path = Path(resolved)
            filename = path.name if path.parent == session.directory.resolve() else str(path)
            if filename not in session.profiles:
                session.profiles.append(filename)
            if entry.get("profile_version") == session.profile_version:
                session.profile_saved = True

    @staticmethod
    def _calibration_document_bytes(document):
        """Bound queued evidence as well as the on-disk serialization."""
        return artifact_size(document)

    @staticmethod
    def _calibration_artifact_destination(session, path):
        """Keep logical session-relative paths across explicitly chosen roots."""
        path = Path(path)
        if session.calibration_directory is None:
            return path
        try:
            relative = (path.parent.resolve() / path.name).relative_to(session.directory.resolve())
        except ValueError:
            relative = Path(path.name)
        return session.calibration_directory / relative

    def _persist_calibration_artifact(self, session, path, document, *, kind, profile_version=None):
        """Attempt a calibration-only write, retaining bounded explicit retries.

        Never enables/disables bulk recording, invalidates poses, or closes a
        reader. Successful files are immutable; failed attempts keep their exact
        document and destination, rather than implicitly retrying every tick.
        """
        logical_path = Path(path)
        path = self._calibration_artifact_destination(session, logical_path)
        entry = dict(path=logical_path, document=document, kind=kind, profile_version=profile_version)
        try:
            resolved = write_artifact(path, document)
        except (OSError, ValueError) as exc:
            session.calibration_storage["error"] = f"{path.name}: {exc}"
            try:
                size = self._calibration_document_bytes(document)
            except (TypeError, ValueError, OverflowError):
                # An invalid/oversized document cannot consume the retry queue.
                # The active profile/capture remains owned by the controller.
                size = None
            if size is not None and size <= CALIBRATION_PENDING_BYTES:
                pending = session.calibration_pending
                while pending and (len(pending) >= CALIBRATION_PENDING_LIMIT or
                                   sum(row["bytes"] for row in pending) + size > CALIBRATION_PENDING_BYTES):
                    pending.pop(0)
                    session.calibration_storage["evicted_count"] += 1
                pending.append({**entry, "document": deepcopy(document), "bytes": size})
            self._publish_calibration_storage(session)
            return False
        self._calibration_artifact_saved(session, entry, resolved)
        if not session.calibration_pending:
            session.calibration_storage["error"] = None
        self._publish_calibration_storage(session)
        return True

    def _retry_calibration_saves(self, session, directory=None):
        """Explicitly retry bounded artifacts, optionally beneath a new root."""
        if directory is not None:
            session.calibration_directory = Path(directory).expanduser().resolve()
            if self._session is not None:
                self._session.calibration_directory = session.calibration_directory
        remaining = []
        for entry in session.calibration_pending:
            path = self._calibration_artifact_destination(session, entry["path"])
            try:
                resolved = resume_artifact(path, entry["document"])
            except (OSError, ValueError) as exc:
                session.calibration_storage["error"] = f"{path.name}: {exc}"
                remaining.append(entry)
            else:
                self._calibration_artifact_saved(session, entry, resolved)
        session.calibration_pending = remaining
        if not remaining:
            session.calibration_storage["error"] = None
        self._publish_calibration_storage(session)
        return not remaining and not any(owner is not None and owner.calibration_pending
                                          for owner in (self._session, self._calibration_detached))

    def _disable_recording(self, session, reason, detail):
        # This may run on the reader. Never join a writer or reset fusion here.
        with session.recording_lock:
            if session.recording_disabled.is_set():
                return
            session.recording_errors.append(f"{reason}: {detail}")
            session.recording_disabled.set()
            if session.storage_budget is not None:
                session.storage_budget.disable(reason, detail, now=self._clock())
            if session.result_recorder is not None:
                session.result_recorder.abort(session.recording_errors[0])
        status = self._local_recording_status(session)
        with self._state_lock:
            if self._session is session:
                self._snapshot["runtime"]["local_recording"] = status
                self._revision += 1

    def _check_recording(self, session):
        if session.recording_disabled.is_set():
            return False
        if session.storage_budget is not None and not session.storage_budget.check(now=self._clock()):
            state = session.storage_budget.snapshot()
            self._disable_recording(session, state["disabled_reason"], state["disabled_detail"])
            return False
        try:
            if session.result_recorder is not None:
                session.result_recorder.check()
        except OSError as exc:
            self._disable_recording(session, "result_recording_failed", str(exc))
        return not session.recording_disabled.is_set()

    def _event(self, session, kind, **data):
        if session.recording_disabled.is_set():
            return
        try:
            self._line(session.streams["events"], dict(event=kind, timestamp_s=self._clock(),
                                                      session_id=session.session_id,
                                                      **self._source_tags(session), **data))
            session.streams["events"].flush()
        except OSError as exc:
            self._disable_recording(session, "event_recording_failed", str(exc))

    @staticmethod
    def _source_tags(session):
        kind = getattr(session.source, "input_kind", "raw_serial")
        return dict(input_kind=kind, hardware_accessed=bool(kind == "raw_serial" and session.source_opened))

    def _ensure_metadata(self, session):
        if not session.metadata_written and not session.recording_disabled.is_set():
            try:
                self._write_metadata(session)
            except OSError as exc:
                self._disable_recording(session, "metadata_recording_failed", str(exc))

    def _write_metadata(self, session):
        _write_json(session.directory / "session_metadata.json", dict(
            schema_version=1, session_id=session.session_id, port=session.port,
            started_monotonic_s=session.started, **self._source_tags(session),
            source_provenance=deepcopy(getattr(session.source, "provenance", {})),
            input_contract=describe_input_contract(self.config),
            config_sha256=self._config_sha, model_rest_sha256=self._rest_sha,
            startup_gc_policy=deepcopy(self._snapshot["runtime"].get("startup_gc_policy")),
            result_recording_mode=(session.result_recorder.snapshot()["mode"]
                                   if session.result_recorder is not None else
                                   "synchronous" if self.record_streams else "calibration_only")))
        session.metadata_written = True

    @staticmethod
    def _retain_scale_samples(session, samples):
        """Reader-owned raw ring. Caller holds session.lock; no fitting or I/O."""
        if session.scale_samples is None:
            return
        session.scale_samples.extend(samples)
        if isinstance(session.scale_samples, ScaleCaptureBuffer):
            return  # Preserve the whole trial; freeze explicitly refuses cap eviction.
        if samples:
            cutoff = samples[-1].host_monotonic_s - INPUT_SCALE_HISTORY_S
            while session.scale_samples and session.scale_samples[0].host_monotonic_s < cutoff:
                session.scale_samples.popleft()

    def _invalidate_scale_check(self, *, status="waiting"):
        self._scale_token += 1
        if self._scale_job is not None:
            self._scale_job.cancel()
        self._scale_pending = self._scale_report = None
        self._scale_state = dict(status=status, candidate_available=False,
            message=("正在记录三姿态的原始静止片段及中间转动；完成后自动检查，不改变当前参数。"
                     if status == "collecting" else
                     "完成三姿态后自动检查输入尺度；不需要手长或精确角度。"))

    def _queue_scale_check(self, session):
        if session.recording_disabled.is_set():
            self._scale_state = dict(status="unavailable", candidate_available=False,
                message="本地记录已暂停，暂不启动需要保存文件的输入尺度检查；实时标定和当前参数不受影响。")
            return
        # Snapshot the immutable sample references once, not a growing raw.csv.
        with session.lock:
            try:
                samples = (session.scale_samples.freeze() if isinstance(session.scale_samples, ScaleCaptureBuffer)
                           else tuple(session.scale_samples or ()))
            except ScaleHistoryEvictedError:
                self._scale_state = dict(status="failed", candidate_available=False,
                    message="本次采集超过有界原始缓存容量，早期证据已不完整；不推断尺度、不改变参数。",
                    retention=session.scale_samples.snapshot())
                return
        holds = sorted(deepcopy(list(self._scale_holds.values())), key=lambda w: w["start_s"])
        if not samples:
            self._scale_state = dict(status="failed", candidate_available=False,
                message="本次三姿态没有原始样本可供检查；未改变参数。")
            return
        directory = session.directory / ("input_scale_" + uuid4().hex[:12])
        self._scale_pending = dict(samples=samples, config=deepcopy(self.config), holds=holds,
            directory=directory, token=self._scale_token, session_id=session.session_id,
            config_sha256=self._config_sha)
        self._scale_state = dict(status="running", candidate_available=False,
            message="后台选择静止片段并逐路拟合/独立复核；实时采集和当前配置保持不变。")

    def _poll_scale_check(self):
        changed = False
        if self._session is not None and self._session.recording_disabled.is_set():
            if (self._scale_pending is not None or self._scale_job is not None) and self._scale_state.get("status") != "recording_unavailable":
                self._invalidate_scale_check(status="recording_unavailable")
                self._scale_state["message"] = "本地记录已暂停，输入尺度文件检查取消；当前参数不变。"
                changed = True
        if self._scale_job is not None and not self._scale_job.is_alive():
            result, context = self._scale_job.result, self._scale_job_context
            self._scale_job.join(0.)
            self._scale_job = self._scale_job_context = None
            session = self._session
            current = (context["token"] == self._scale_token and session is not None
                       and context["session_id"] == session.session_id and not session.stop.is_set()
                       and context["config_sha256"] == self._config_sha)
            if current:
                if not result or result.get("error") or not isinstance(result.get("report"), dict):
                    self._scale_state = dict(status="failed", candidate_available=False,
                        message="输入尺度检查未完成；当前参数和采集不受影响。",
                        error=(result or {}).get("error", "后台没有返回结果。"))
                else:
                    from .input_scale_check import scale_check_summary
                    report = result["report"]
                    self._scale_state = scale_check_summary(report)
                    self._scale_state["config_sha256"] = self._config_sha
                    self._scale_report = deepcopy(report)
                    self._scale_state["candidate_available"] = bool(
                        self._scale_state.get("candidate_available")
                        and callable(getattr(self._pose_backend, "with_config", None)))
                self._event(session, "input_scale_check_finished", result=self._scale_state)
                changed = True
        # Never overlap two numerical checks. A newer capture cancels the old
        # generation; its queued snapshot waits for cooperative cleanup.
        if self._scale_job is None and self._scale_pending is not None:
            pending, self._scale_pending = self._scale_pending, None
            if not self._stop.is_set() and pending["token"] == self._scale_token:
                try:
                    if self._scale_job_factory is None:
                        from .input_scale_job import InputScaleJob
                        factory = InputScaleJob
                    else:
                        factory = self._scale_job_factory
                    self._scale_job = factory(pending["samples"], pending["config"],
                                              pending["holds"], pending["directory"])
                    self._scale_job_context = {key: pending[key] for key in
                                              ("token", "session_id", "config_sha256")}
                except Exception as exc:
                    self._scale_state = dict(status="failed", candidate_available=False,
                        message="后台检查无法启动；当前参数和采集不受影响。", error=str(exc))
                    changed = True
        if changed:
            with self._state_lock:
                self._snapshot["calibration"] = self._calibration_state(self._clock())
                self._revision += 1

    def _use_input_scale_candidate(self):
        """Explicit transaction: validate, disconnect, replace config/backend.

        Never change a running fusion, overwrite source config, reuse the old
        calibration, or auto-open a source. Persisted candidate files are for
        repeatable later launches; this command only selects the next session.
        """
        if self._active is not None:
            raise ValueError("请等当前动作采集结束，再采用输入尺度候选。")
        state, report = deepcopy(self._scale_state), deepcopy(self._scale_report)
        if (state.get("status") != "completed" or not state.get("candidate_available")
                or not isinstance(report, dict) or not report.get("recommended_nodes")
                or state.get("config_sha256") != self._config_sha):
            raise ValueError("没有绑定当前配置、经独立复核的尺度候选可采用。")
        candidate = validate_config(deepcopy(report["candidate_config"]))
        allowed = {"gyro_dps_per_lsb_by_node", "units_verified", "input_contract",
                   "input_unit_fit_evidence", "provenance"}
        if ({key: value for key, value in candidate.items() if key not in allowed}
                != {key: value for key, value in self.config.items() if key not in allowed}):
            raise ValueError("候选改变了尺度以外的运行配置，未应用。")
        clone = getattr(self._pose_backend, "with_config", None)
        if not callable(clone):
            raise ValueError("当前后端不支持更换输入配置；请用候选文件重新启动。")
        backend = clone(candidate)  # Prepare before closing the live session.
        previous_sha = self._config_sha
        if self._session is not None:
            self._event(self._session, "input_scale_candidate_selected",
                        candidate_path=state.get("candidate_path"), candidate_sha256=digest(candidate))
        self._disconnect("input_scale_candidate_selected")
        if any(thread.is_alive() for thread in [*self._readers, *self._writers]):
            raise ValueError("旧读取/记录尚未退出，未应用候选；请等待关闭完成。候选文件已保留。")
        self.config, self._pose_backend = candidate, backend
        self._config_sha = digest(candidate)
        contract = describe_input_contract(candidate)
        self._scale_state = {**state, "status": "applied", "candidate_available": False,
            "config_sha256": self._config_sha, "previous_config_sha256": previous_sha,
            "applied_input_contract": contract,
            "message": "候选已用于下次连接；当前已断开，请连接并重新完成三姿态及功能轴标定。原配置文件未覆盖。"}
        with self._state_lock:
            self._clear_generation += 1
            self._snapshot["runtime"]["active_input_contract"] = contract
            self._snapshot["calibration"] = self._calibration_state(self._clock())
            self._input_snapshot = dict(runtime={**self._snapshot["runtime"],
                "clear_generation": self._clear_generation}, orientation=None)
            self._revision += 1
        self._message(self._scale_state["message"])

    def _reset_calibration(self):
        self._shape_invalidate('连接或基础标定已重新开始；已有手形文件保留，接触采集需使用本次有效姿态。')
        self._invalidate_scale_check()
        self._scale_holds = {}
        if self._session is not None and self._session.scale_samples is not None:
            with self._session.lock:
                self._session.scale_samples = None
        self._display_pose_cache.reset()
        self._display_context = None
        if self._functional_hand_cancel is not None:
            self._functional_hand_cancel.set()
        self._functional_hand_cancel, self._functional_hand_result = None, None
        self._captures, self._active, self._profile = {}, None, None
        self._profile_status, self._warnings, self._rejected = "not_calibrated", [], {}
        reset = getattr(self._pose_backend, "reset_calibration", None)
        if callable(reset):
            reset()

    def _clear_calibration(self):
        """Explicit full software reset; keep the reader and raw record alive.

        A calibration clear must be equivalent to fresh filter/pose references,
        not merely remove the three captured pose dictionaries. Sensor epochs
        change so no saved old binding can accidentally become valid again.
        """
        session = self._session
        previous_version = (self._profile or {}).get("profile_version")
        self._reset_calibration()
        fusion_reset = None
        if session is not None:
            with session.lock:
                reset = getattr(session.fusion, "reset_for_calibration", None)
                if callable(reset):
                    fusion_reset = reset()
                else:
                    # Compatibility for injected, stateless test/third-party
                    # fusion factories. Production OnlineFusion retains its
                    # session counters and increments filter epochs in place.
                    session.fusion = self._fusion_factory(self.config)
                    fusion_reset = dict(method="replacement_fusion_factory")
                # Snapshot generation and filter reset share one boundary.
                # The state lock also serializes the final input publication.
                with self._state_lock:
                    self._clear_generation += 1
        else:
            with self._state_lock:
                self._clear_generation += 1
        now = self._clock()
        with self._state_lock:
            self._snapshot.update(orientation=None, pose=_unavailable("calibration_required", now),
                                  geometry=None, geometry_error="", display_pose=None, display_geometry=None,
                                  calibration=self._calibration_state(now))
            self._snapshot["runtime"].update(message="标定、滤波参考和解算缓存已全部清除；串口保持连接，请从第 1 步重新标定。",
                                              error=None, published_monotonic_s=now)
            self._input_snapshot = dict(runtime={**self._input_snapshot["runtime"],
                "clear_generation": self._clear_generation, "published_monotonic_s": now},
                orientation=None)
            self._revision += 1
        if session is not None and session.streams:
            self._event(session, "clear_calibration", clear_generation=self._clear_generation,
                        previous_profile_version=previous_version, fusion_reset=fusion_reset,
                        historical_profiles_preserved=True)

    def _connect(self, port):
        if self._stop.is_set():
            raise ValueError("控制器已请求停止，不再连接数据源。")
        if self._session is not None or any(thread.is_alive() for thread in [*self._readers, *self._writers]):
            raise ValueError("已有连接或上次读取/记录线程尚未结束；请先断开，不能抢占或自动重连。")
        self._readers.clear()
        self._writers.clear()
        session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:12]
        directory = self.output_root / session_id
        directory.mkdir(parents=True, exist_ok=False)
        session = _Session(session_id, directory, port, self._source_factory(port),
                           self._fusion_factory(self.config),
                           self._recorder_factory(directory / "raw.csv") if self.record_streams else None,
                           self._clock())
        session.recording_policy = ResultRecordingPolicy(self.config)
        session.storage_budget = SessionStorageBudget(directory, self.config.get("local_recording"))
        self._session = session
        self._reset_calibration()
        try:
            names = (("events",) if not self.record_streams or self._result_recorder_factory is not None
                     else ("orientations", "poses", "events"))
            for name in names:
                session.streams[name] = (directory / f"{name}.jsonl").open("x", encoding="utf-8")
            if self.record_streams:
                if self._result_recorder_factory is None:
                    session.result_recorder = AsyncResultRecorder(session.streams)
                else:
                    session.result_recorder = self._result_recorder_factory(
                        {name: directory / f"{name}.jsonl" for name in ("orientations", "poses")})
                self._writers.append(session.result_recorder.worker)
                # Track a possibly timed-out process before testing readiness:
                # setup failures cannot orphan it or open a serial source first.
                session.result_recorder.check()
            _write_json(directory / "config.json", self.config)
            _write_json(directory / "input_contract.json", describe_input_contract(self.config))
            _write_json(directory / "rest.json", self.rest)
            self._event(session, "connecting", port=port)
            with self._state_lock:
                self._snapshot.update(orientation=None, pose=None, geometry=None, geometry_error="",
                                      display_pose=None, display_geometry=None,
                                      calibration=self._calibration_state(self._clock()))
                self._snapshot["runtime"].update(connected=False, port=port, session_id=session_id,
                    output_dir=str(directory), received_samples=0, last_sample_age_s=None,
                    input_diagnostics=None,
                    local_recording=self._local_recording_status(session),
                    calibration_storage=self._calibration_storage_status(self._calibration_retry_session()),
                    message="正在连接；不会抢占被占用端口。", error=None)
                self._revision += 1
            session.reader = threading.Thread(target=self._read, args=(session,), name="fsglove12-reader", daemon=True)
            self._readers.append(session.reader)
            session.reader.start()
        except Exception:
            self._disconnect("connection_setup_failed", "连接准备或文件创建失败。")
            raise

    def _read(self, session):
        raw_open = False
        try:
            if session.stop.is_set() or self._stop.is_set():
                return
            session.source.open()
            session.source_opened = True
            if session.recorder is not None and not session.recording_disabled.is_set():
                try:
                    session.recorder.open()
                    raw_open = True
                except OSError as exc:
                    self._disable_recording(session, "raw_recording_open_failed", str(exc))
            session.opened = True
            with self._state_lock:
                if self._session is session and not session.stop.is_set():
                    self._snapshot["runtime"].update(connected=True, message="已连接；请完成本次会话的三步标定。", error=None)
                    self._revision += 1
            while not session.stop.is_set() and not self._stop.is_set():
                samples = session.source.read_samples()
                if not samples:
                    session.stop.wait(.002)
                    continue
                session.received += len(samples)
                # Record receive/source time before disk I/O or fusion work.
                # A slow update must not make old packets appear newly arrived.
                session.last_sample = max(getattr(sample, "host_monotonic_s", self._clock())
                                          for sample in samples)
                if raw_open and not session.recording_disabled.is_set():
                    try:
                        session.recorder.write(samples)
                        session.recorded += len(samples)
                    except OSError as exc:
                        self._disable_recording(session, "raw_recording_failed", str(exc))
                if raw_open and session.recording_disabled.is_set():
                    raw_open = False
                    try:
                        session.recorder.close()
                    except OSError:
                        pass  # First failure is latched; do not retry disk I/O on every batch.
                with session.lock:
                    self._retain_scale_samples(session, samples)
                    session.fusion.update(samples)
                    session.node_counts.update(str(sample.node_id) for sample in samples)
                with self._state_lock:
                    if self._session is session:
                        self._snapshot["runtime"]["received_samples"] = session.received
        except EOFError:
            session.eof = True
        except Exception as exc:
            session.error = f"数据源读取或融合失败：{exc}"
            if self._session is session:
                self._invalidate("连接已停止；不会自动重连。", session.error)
        finally:
            session.opened = False
            try:
                if session.recorder is not None:
                    session.recorder.close()
            except OSError as exc:
                self._disable_recording(session, "raw_recording_close_failed", str(exc))
            for resource in (session.source,):
                try:
                    resource.close()
                except Exception as exc:
                    session.error = f"{session.error or ''} 关闭资源失败：{exc}".strip()
            if self._session is session:
                self._invalidate("数据源已结束；不会自动重连。", session.error)
            session.done.set()
            self._wake.set()

    def _disconnect(self, reason="manual_disconnect", error=None):
        session = self._session
        self._invalidate("已断开；重新连接必须重新标定。", error)
        self._reset_calibration()
        if session is None:
            return
        session.stop.set()
        if session.reader is not None:
            session.reader.join(.75)
        errors = [value for value in (error, session.error) if value]
        reader_done = session.reader is None or session.done.is_set()
        if not reader_done:
            errors.append("读取线程未在 0.75 秒内退出；记录不完整，禁止重连直到线程结束。")
        try:
            self._ensure_metadata(session)
            if "events" in session.streams:
                self._event(session, "disconnected", reason=reason, errors=errors)
        except Exception as exc:
            errors.append(f"写入会话元数据或事件失败：{exc}")
        if session.result_recorder is not None:
            if not session.result_recorder.close():
                errors.append("结果记录线程未在 0.75 秒内退出；记录不完整，禁止重连直到线程结束。")
            result_recording = session.result_recorder.snapshot()
            if result_recording["error"]:
                errors.append(f"结果记录失败：{result_recording['error']}")
        else:
            result_recording = None
        for name, stream in session.streams.items():
            if session.result_recorder is not None and name in {"orientations", "poses"}:
                continue  # Only their writer may close these streams.
            try:
                stream.close()
            except Exception as exc:
                errors.append(f"关闭记录文件失败：{exc}")
        final_input = _input_diagnostics(session.source, self._clock())
        errors.extend(value for value in session.recording_errors if value not in errors)
        summary = dict(schema_version=1, session_id=session.session_id, port=session.port,
                       **self._source_tags(session),
                       source_provenance=deepcopy(getattr(session.source, "provenance", {})),
                       started_monotonic_s=session.started, ended_monotonic_s=self._clock(),
                       stop_reason=reason, recording_requested=self.record_streams,
                       session_complete=reader_done and not errors,
                       recording_complete=self.record_streams and reader_done and not errors,
                       reader_stopped=reader_done, errors=errors, received_samples=session.received,
                       recorded_samples=session.recorded, samples_per_node=dict(session.node_counts),
                       output_frames=session.frames, available_poses=session.available_poses,
                       profile_files=session.profiles, config_sha256=self._config_sha,
                       model_rest_sha256=self._rest_sha,
                       input_diagnostics=final_input,
                       result_recording=result_recording,
                       local_recording=self._local_recording_status(session),
                       calibration_storage=self._calibration_storage_status(session),
                       raw_recording_complete=self.record_streams and reader_done and session.received == session.recorded
                           and not session.recording_disabled.is_set(),
                       processing=self._processing_status(session),
                       input_contract=describe_input_contract(self.config),
                       anatomical_accuracy_verified=False, yaw_drift_corrected=False)
        try:
            _write_json(session.directory / "summary.json", summary)
        except Exception as exc:
            errors.append(f"写入 summary.json 失败：{exc}")
        self._retain_calibration_saves(session)
        self._session = None
        with self._state_lock:
            self._snapshot["runtime"]["input_diagnostics"] = final_input
            self._snapshot["runtime"]["result_recording"] = result_recording
            self._snapshot["runtime"]["local_recording"] = self._local_recording_status(session)
            self._snapshot["runtime"]["calibration_storage"] = self._calibration_storage_status(self._calibration_retry_session())
            self._snapshot["calibration"] = self._calibration_state(self._clock())
            self._revision += 1
        if errors:
            self._message("会话结束，但记录存在错误，不能视为完整成功。", "；".join(errors))

    def _command(self, name, args):
        if name in SHAPE_COMMANDS:
            self._shape_command(name, args)
            return
        if name in {"capture", "reanchor_neutral", "capture_functional", "capture_functional_hand",
                    "set_functional_enabled", "use_input_scale_candidate"} and self._shape_busy():
            raise ValueError("触指采集或手形优化正在进行，请先完成或取消本次手形操作。")
        if name == "use_input_scale_candidate":
            try:
                self._use_input_scale_candidate()
            finally:
                # A submit-queue acknowledgement is not an application commit.
                # Both IPC paths wait for this actual execution generation.
                with self._state_lock:
                    self._input_scale_generation += 1
                    self._snapshot["runtime"]["input_scale_generation"] = self._input_scale_generation
                    self._input_snapshot = dict(runtime={**self._input_snapshot["runtime"],
                        "input_scale_generation": self._input_scale_generation}, orientation=None)
                    self._revision += 1
            return
        if name == "connect":
            self._connect(args["port"])
            return
        if name == "disconnect":
            self._disconnect()
            return
        if name == "clear_calibration":
            self._clear_calibration()
            return
        session = self._session
        if session is None or not session.opened or session.done.is_set():
            raise ValueError("请先手动连接有效数据源。")
        capture_epochs = None
        if name in {"capture", "reanchor_neutral", "capture_functional", "capture_functional_hand"}:
            failures = {}
            if self.config.get("input_health", {}).get("enabled", False):
                with session.lock:
                    capture_frame = _fusion_snapshot(session.fusion, self._clock(), input_only=True)
                failures = self._capture_health(capture_frame)
                if not failures:
                    capture_epochs = {node: capture_frame["nodes"][node].get("epoch")
                                      for node in self.config["node_to_slot"]}
            if failures:
                raise ValueError("输入完整性尚未通过，不能开始标定；请先恢复数据流，不必反复重做校准。"
                                 + self._capture_health_text(failures))
        if name == "cancel_capture":
            if self._active is not None and self._active.get("kind") == "functional_hand":
                self._cancel_functional_hand(session)
            self._active = None
            self._profile_status = "not_calibrated" if self._profile is None else self._profile["status"]
            self._message("已取消当前采集；已完成步骤保留。")
        elif name == "set_functional_enabled":
            if self._active is not None:
                raise ValueError("正在采集；请先取消或等待完成，再切换单指校正。")
            if self._profile is None:
                raise ValueError("请先完成本次会话的三步标定。")
            if self._profile.get("functional_enabled", True) != args["enabled"]:
                profile = deepcopy(self._profile)
                profile["functional_enabled"] = args["enabled"]
                self._save_profile(session, profile, event="functional_enabled_changed")
            self._message("已启用单指轴校正 + 关节约束；非实测精度。" if args["enabled"] else
                          "已关闭单指轴校正，显示官方原始输出作对比；基础标定与单指参数保留。")
        elif name == "reanchor_neutral":
            if self._active is not None:
                raise ValueError("正在采集；请先取消或等待完成。")
            if self._profile is None:
                raise ValueError("请先完成三步安装标定，再重设伸直参考。")
            now = self._clock()
            self._active = dict(step="neutral", kind="neutral_reference", collect_at=now + PREPARE_SECONDS,
                                end_at=now + PREPARE_SECONDS + CAPTURE_SECONDS, seen=None, rows=[])
            self._profile_status = "collecting_neutral_reference"
            self._message("准备 3 秒：五指自然伸直，再静止 2 秒；保留安装轴和手掌朝向，不重启滤波器。")
        elif name == "capture_functional":
            if self._active is not None:
                raise ValueError("正在采集；请先取消或等待完成。")
            if self._profile is None:
                raise ValueError("请先完成三步安装标定，再校准手指弯曲方向。")
            now = self._clock()
            neutral_end = now + PREPARE_SECONDS + CAPTURE_SECONDS
            self._active = dict(step="neutral", kind="functional", finger=args["finger"],
                                collect_at=now + PREPARE_SECONDS, neutral_end_at=neutral_end,
                                end_at=neutral_end + FUNCTIONAL_MOTION_SECONDS,
                                seen=None, rows=[], motion_rows=[])
            if self._pose_backend is not None:
                self._active["reference_capture"] = FunctionalReferenceCapture(
                    OFFICIAL_FUNCTIONAL_SLOTS[args["finger"]], now + PREPARE_SECONDS)
            self._profile_status = "collecting_functional_axis"
            self._message("先自然张手：准备 3 秒后等待取得连续 2 秒稳定参考；看到“现在弯伸”后，再弯伸所选手指 2–3 次。")
        elif name == "capture_functional_hand":
            if self._pose_backend is None:
                raise ValueError("全手功能轴采集仅用于官方 FSGlove 模式。")
            if self._active is not None:
                raise ValueError("正在采集；请先取消或等待完成。")
            if self._profile is None:
                raise ValueError("请先完成三姿势基础标定，再开始全手功能轴采集。")
            now = self._clock()
            neutral_end = now + PREPARE_SECONDS + CAPTURE_SECONDS
            fingers_end = neutral_end + FUNCTIONAL_MOTION_SECONDS
            thumb_start = fingers_end + PREPARE_SECONDS
            self._functional_hand_cancel = threading.Event()
            self._functional_hand_result = None
            self._active = dict(step="neutral", kind="functional_hand", collect_at=now + PREPARE_SECONDS,
                                neutral_end_at=neutral_end, fingers_end_at=fingers_end,
                                thumb_start_at=thumb_start, end_at=thumb_start + FUNCTIONAL_MOTION_SECONDS,
                                seen=None, capture_phase=None, rows=[], finger_rows=[], thumb_rows=[],
                                cancel=self._functional_hand_cancel)
            self._active["reference_capture"] = FunctionalReferenceCapture(
                [slot for slots in OFFICIAL_FUNCTIONAL_SLOTS.values() for slot in slots],
                now + PREPARE_SECONDS)
            self._profile_status = "collecting_functional_hand"
            self._message("全手通常约 24 秒，等待稳定时延长：准备 3 秒、取得伸直参考、四指慢弯伸 8 秒；"
                          "再准备 3 秒、仅拇指 IP 慢弯伸 8 秒。按当前提示动作，不必逐指点击。")
        elif name == "capture":
            if self._active is not None:
                raise ValueError("正在采集一个步骤；请先取消或等待完成。")
            now = self._clock()
            if self._pose_backend is not None:
                self._invalidate_scale_check(status="collecting")
                self._scale_holds.pop(args["step"], None)
                with session.lock:
                    if session.scale_samples is None:
                        session.scale_samples = ScaleCaptureBuffer()
            self._active = dict(step=args["step"], collect_at=now + PREPARE_SECONDS,
                                end_at=now + PREPARE_SECONDS + CAPTURE_SECONDS, seen=None, rows=[])
            self._profile = None
            self._shape_invalidate()
            self._functional_hand_result = None
            self._profile_status = "collecting"
            self._warnings, self._rejected = [], {}
            self._message("准备 3 秒后采集 2 秒；保持张手，按当前步骤整体转动手掌。")
        if capture_epochs is not None and self._active is not None:
            self._active["health_epochs"] = capture_epochs
        self._event(session, name, **args)
        with self._state_lock:
            self._snapshot["calibration"] = self._calibration_state(self._clock())
            if self._profile is None:
                self._snapshot["pose"] = _unavailable("calibration_required", self._clock())
                self._snapshot["geometry"] = None
            self._revision += 1

    def _capture(self, session, frame, now):
        active = self._active
        if active is not None:
            failures = self._capture_health(frame, strict=False)
            if failures:
                self._fail_capture_health(session, failures)
                return
        if active is not None and active.get("kind") == "functional_hand":
            self._capture_functional_hand(session, frame, now)
            return
        if active is not None and self._capture_functional_reference(session, frame, now):
            return
        if active is None or now < active["collect_at"]:
            return
        if now >= active["end_at"]:
            self._active = None
            if not active["rows"]:
                self._profile_status = self._profile["status"] if self._profile is not None else "capture_failed"
                self._message("本步骤没有新的有效样本，请检查数据流后单独重采。", "采集失败：没有新增 sample_index。")
                self._event(session, "capture_failed", step=active["step"], reason="no_new_samples")
                return
            if active.get("kind") == "neutral_reference":
                self._reanchor_profile(session, active["rows"])
                return
            if active.get("kind") == "functional":
                self._fit_functional_profile(session, active)
                return
            self._captures[active["step"]] = active["rows"]
            capture_id = uuid4().hex
            self._persist_calibration_artifact(session,
                session.directory / "calibration" / f"capture.{active['step']}.{capture_id}.json",
                dict(schema_version=1, artifact_type="mounting_capture", capture_id=capture_id,
                     session_id=session.session_id, step=active["step"],
                     config_sha256=self._config_sha, rest_sha256=self._rest_sha,
                     **self._source_tags(session),
                     started_monotonic_s=active["collect_at"], ended_monotonic_s=active["end_at"],
                     frames=active["rows"]), kind="mounting_capture")
            self._profile_status = "steps_pending"
            self._event(session, "capture_complete", step=active["step"], frames=len(active["rows"]))
            if self._pose_backend is not None:
                self._scale_holds[active["step"]] = dict(label=active["step"],
                    start_s=active["collect_at"], end_s=active["end_at"],
                    time_basis="absolute_monotonic")
                if all(key in self._scale_holds for key in STEP_KEYS):
                    self._queue_scale_check(session)
            self._message("本步骤已记录；可继续下一步，或单独重新采集任一步骤。")
            if all(key in self._captures for key in STEP_KEYS):
                self._fit_profile(session)
            return
        moving = active.get("kind") == "functional" and now >= active["neutral_end_at"]
        start = active["neutral_end_at"] if moving else active["collect_at"]
        end = active["end_at"] if moving else active.get("neutral_end_at", active["end_at"])
        _sample_capture_frame(active, "motion_rows" if moving else "rows", frame, now,
                              start, end, 300 if moving else CAPTURE_FRAME_LIMIT)

    def _capture_health(self, frame, *, epochs=None, strict=True):
        configured = self.config.get("input_health", {}).get("enabled", False)
        health = frame.get("input_health", {})
        if not configured and not health.get("enabled", False):
            return {}
        if not health.get("enabled", False):
            return {node: "missing_input_health" for node in self.config["node_to_slot"]}
        active = self._active
        if epochs is None and active is not None:
            epochs = active.get("health_epochs")
        failures = {}
        for node in self.config["node_to_slot"]:
            data = frame.get("nodes", {}).get(node)
            status = health.get("nodes", {}).get(node, {})
            state = status.get("state")
            if state in {"faulted", "recovering"}:
                failures[node] = state
            elif strict and (state != "healthy" or status.get("healthy") is not True):
                failures[node] = state or "missing_input_health"
            elif state not in {"healthy", "warming_up", "stale"}:
                failures[node] = "missing_input_health"
            elif isinstance(data, dict) and epochs is not None and data.get("epoch") != epochs.get(node):
                failures[node] = "filter_epoch_changed_during_capture"
            elif strict and not isinstance(data, dict):
                failures[node] = "missing_or_stale_node"
            elif (isinstance(data, dict) and state == "healthy"
                  and data.get("orientation_valid") is False):
                # A healthy stream with an invalid orientation is the guard's
                # prolonged-invalid output, not its one-sample rejection flag.
                failures[node] = "invalid_orientation"
        if (not failures and active is not None and "health_epochs" not in active
                and all(node in frame.get("nodes", {}) and health["nodes"][node].get("healthy") is True
                        for node in self.config["node_to_slot"])):
            active["health_epochs"] = {node: frame["nodes"][node].get("epoch")
                                       for node in self.config["node_to_slot"]}
        return failures

    def _current_capture_health(self, session, *, epochs=None):
        if not self.config.get("input_health", {}).get("enabled", False):
            return {}
        with session.lock:
            frame = _fusion_snapshot(session.fusion, self._clock(), input_only=True)
        return self._capture_health(frame, epochs=epochs)

    @staticmethod
    def _capture_health_text(failures):
        return "节点 " + "、".join(f"{node}: {reason}" for node, reason in sorted(failures.items()))

    def _fail_capture_health(self, session, failures):
        active = self._active
        if self._functional_hand_cancel is not None:
            self._functional_hand_cancel.set()
        if active is not None and active.get("kind") == "functional_hand":
            self._functional_hand_result = dict(status="failed", updated_fingers=[],
                errors={"input_health": self._capture_health_text(failures)}, partial_slots={}, fingers={})
        self._active, self._functional_hand_cancel = None, None
        self._profile_status = self._profile["status"] if self._profile is not None else "capture_failed"
        self._message("输入完整性异常，本次采集未保存；已有标定保留，请先恢复数据流。",
                      self._capture_health_text(failures))
        self._event(session, "capture_failed", reason="input_health", nodes=failures,
                    step=active.get("step") if active else None)

    def _capture_save_healthy(self, session, profile=None):
        # Fitting may block solver ticks while input continues. A fault and
        # recovery during that interval must not launder old reference epochs.
        epochs = ({node: data.get("epoch") for node, data in profile.get("nodes", {}).items()}
                  if profile is not None else None)
        failures = self._current_capture_health(session, epochs=epochs)
        if failures:
            self._fail_capture_health(session, failures)
            return False
        return True

    @staticmethod
    def _functional_hand_phase(active, now):
        reference = active.get("reference_capture")
        if reference is not None and reference.selected is None:
            return (("preparing", active["collect_at"]) if now < active["collect_at"]
                    else ("collecting_neutral", reference.deadline_s))
        for phase, key in (("preparing", "collect_at"), ("collecting_neutral", "neutral_end_at"),
                           ("moving_fingers", "fingers_end_at"), ("preparing_thumb", "thumb_start_at")):
            if now < active[key]:
                return phase, active[key]
        return "moving_thumb", active["end_at"]

    def _capture_functional_reference(self, session, frame, now):
        """Select a real stable reference before scheduling any motion phase."""
        active = self._active
        reference = active.get("reference_capture")
        if reference is None or reference.selected is not None:
            return False
        if now < active["collect_at"]:
            return True
        rotations = (self._pose_backend.functional_reference_rotations(
            frame, self._profile, [int(slot) for slot in reference.slots])
            if reference.needs_sample(now) else {})
        status = reference.observe(frame, rotations, now)
        if status not in {"accepted", "partial", "failed"}:
            return True
        selection = reference.summary(now)
        if status == "failed":
            reason = "尚未取得连续 2 秒稳定的伸直参考；本次已结束，已有标定保留。请伸直手指后重试或单指补采。"
            if active["kind"] == "functional_hand":
                self._functional_hand_result = dict(status="failed", reason=reason,
                    updated_fingers=[], errors={"reference": reason}, partial_slots={}, fingers={})
            self._event(session, "functional_reference_failed", capture_kind=active["kind"],
                        reference_selection=selection)
            self._active, self._functional_hand_cancel = None, None
            self._profile_status = self._profile["status"]
            self._message(reason, reason)
            return True
        active["rows"] = reference.selected
        active["neutral_end_at"] = now
        active["seen"] = {node: (data.get("epoch"), data.get("sample_index"))
                          for node, data in frame["nodes"].items()}
        if active["kind"] == "functional_hand":
            active["fingers_end_at"] = now + FUNCTIONAL_MOTION_SECONDS
            active["thumb_start_at"] = active["fingers_end_at"] + PREPARE_SECONDS
            active["end_at"] = active["thumb_start_at"] + FUNCTIONAL_MOTION_SECONDS
        else:
            active["end_at"] = now + FUNCTIONAL_MOTION_SECONDS
        active["reference_selection"] = selection
        self._event(session, "functional_reference_selected", capture_kind=active["kind"],
                    finger=active.get("finger"), reference_selection=selection,
                    motion_start_s=now, motion_end_s=active["end_at"],
                    fingers_end_s=active.get("fingers_end_at"), thumb_start_s=active.get("thumb_start_at"))
        self._message("已取得伸直参考；现在按提示弯伸。" if status == "accepted" else
                      "已取得部分关节的伸直参考；现在按提示弯伸，未稳定关节保留原参数并可单指补采。")
        return True

    def _cancel_functional_hand(self, session):
        if self._functional_hand_cancel is not None:
            self._functional_hand_cancel.set()
        self._functional_hand_result = dict(status="cancelled", updated_fingers=[], errors={},
                                           partial_slots={}, fingers={})
        self._profile_status = self._profile["status"] if self._profile else "not_calibrated"
        self._message("全手功能轴采集已取消；没有保存新参数，原标定保留。")
        self._event(session, "functional_hand_cancelled")
        self._active, self._functional_hand_cancel = None, None

    def _capture_functional_hand(self, session, frame, now):
        active = self._active
        failures = self._capture_health(frame, strict=False)
        if failures:
            self._fail_capture_health(session, failures)
            return
        if active["cancel"].is_set():
            self._cancel_functional_hand(session)
            return
        if self._capture_functional_reference(session, frame, now):
            return
        if now >= active["end_at"]:
            try:
                self._fit_functional_hand_profile(session, active)
            finally:
                self._active, self._functional_hand_cancel = None, None
            return
        phase, _ = self._functional_hand_phase(active, now)
        indices = {node: (data.get("epoch"), data.get("sample_index")) for node, data in frame["nodes"].items()}
        if phase != active["capture_phase"]:
            # Rebase at every boundary: no last-frame carry-over from another
            # phase, even when the same IMU state is polled more than once.
            active["capture_phase"], active["seen"] = phase, indices
            if session is not None:
                self._event(session, "functional_capture_phase", capture_kind=active["kind"], phase=phase,
                            phase_started_s=now, phase_end_s=self._functional_hand_phase(active, now)[1])
            return
        if phase in {"preparing", "preparing_thumb"}:
            active["seen"] = indices
            return
        key = {"collecting_neutral": "rows", "moving_fingers": "finger_rows", "moving_thumb": "thumb_rows"}[phase]
        start, end = {"collecting_neutral": (active["collect_at"], active["neutral_end_at"]),
                      "moving_fingers": (active["neutral_end_at"], active["fingers_end_at"]),
                      "moving_thumb": (active["thumb_start_at"], active["end_at"])}[phase]
        limit = CAPTURE_FRAME_LIMIT if phase == "collecting_neutral" else 300
        _sample_capture_frame(active, key, frame, now, start, end, limit)

    def _fit_functional_hand_profile(self, session, active):
        self._profile_status = "fitting_functional_hand"
        with self._state_lock:
            self._snapshot["calibration"] = self._calibration_state(self._clock())
            self._snapshot["pose"] = _unavailable("calibration_fitting", self._clock())
            self._snapshot["geometry"] = None
            self._revision += 1
        try:
            result = self._pose_backend.fit_functional_hand(
                active["rows"], active["finger_rows"], active["thumb_rows"], deepcopy(self._profile),
                session_id=session.session_id, reference_selection=active.get("reference_selection"))
            entries, errors = result["entries"], result["errors"]
            failures = result.get("failures", {})
            if not isinstance(entries, dict) or not isinstance(errors, dict) or any(
                    finger not in OFFICIAL_FUNCTIONAL_FINGERS for finger in (*entries, *errors)):
                raise ValueError("全手功能轴结果格式无效；原标定保留。")
        except ValueError as exc:
            entries, errors = {}, {finger: str(exc) for finger in OFFICIAL_FUNCTIONAL_FINGERS}
            failures = {}
        if active["cancel"].is_set() or self._stop.is_set() or session.stop.is_set():
            self._cancel_functional_hand(session)
            return
        errors = {**{finger: "本次没有可更新结果，保留原参数。" for finger in OFFICIAL_FUNCTIONAL_FINGERS
                     if finger not in entries and finger not in errors}, **errors}
        updated = [finger for finger in OFFICIAL_FUNCTIONAL_FINGERS if finger in entries]
        partial = {finger: deepcopy(entry.get("rejected_slots", {})) for finger, entry in entries.items()
                   if entry.get("rejected_slots")}
        fingers = {finger: dict(status="partial" if finger in partial else "updated",
                                updated_slots=sorted((slot for slot in entry.get("slot_refs", {})
                                                      if slot not in entry.get("rejected_slots", {})), key=int),
                                rejected_slots=deepcopy(entry.get("rejected_slots", {})))
                   for finger, entry in entries.items()}
        fingers.update({finger: dict(status="failed", error=reason, updated_slots=[],
                                     **failures.get(finger, dict(rejected_slots={})))
                        for finger, reason in errors.items() if finger not in entries})
        report = dict(status="partial" if errors or partial else "complete", updated_fingers=updated,
                      errors=deepcopy(errors), partial_slots=partial, fingers=fingers)
        if not entries:
            report["status"] = "failed"
            self._functional_hand_result = report
            self._profile_status = self._profile["status"]
            self._message("本次全手功能轴未更新；原标定全部保留，可查看每指原因。", "；".join(errors.values()))
            self._event(session, "functional_hand_failed", result=report)
            return
        profile = deepcopy(self._profile)
        if active.get("reference_selection"):
            for entry in entries.values():
                entry["reference_selection"] = deepcopy(active["reference_selection"])
        profile.setdefault("functional_fingers", {}).update(deepcopy(entries))
        profile.setdefault("functional_enabled", True)
        if active["cancel"].is_set() or self._stop.is_set() or session.stop.is_set():
            self._cancel_functional_hand(session)
            return
        if not self._capture_save_healthy(session, profile):
            return
        self._save_profile(session, profile, event="functional_hand_updated")
        self._functional_hand_result = report
        for entry in entries.values():
            self._warnings.extend(entry.get("warnings", []))
        names = "、".join(FINGER_NAMES[finger] for finger in updated)
        self._message(f"全手功能轴已更新：{names}；只应用了本次成功部分，失败部分保留旧参数。"
                      "拇指仅 IP，不改变 MCP / CMC；非实测精度。")
        self._event(session, "functional_hand_result", result=report)

    def _fit_profile(self, session):
        self._profile_status = "fitting"
        with self._state_lock:
            self._snapshot["calibration"] = self._calibration_state(self._clock())
            self._snapshot["pose"] = _unavailable("calibration_fitting", self._clock())
            self._snapshot["geometry"] = None
            self._revision += 1
        try:
            if self._pose_backend is not None:
                official_keys = dict(zip(STEP_KEYS, ("base_pose", "z_rot_pose_1", "z_rot_pose_2")))
                captures = {official_keys[key]: deepcopy(rows) for key, rows in self._captures.items()}
                profile = self._pose_backend.fit_calibration(captures, session_id=session.session_id)
            else:
                profile = self._calibration_fn(deepcopy(self._captures), self.rest, self.config,
                                               session_id=session.session_id, model_axes=self.model_axes)
            if profile.get("session_id") != session.session_id or self._root_node not in profile.get("nodes", {}):
                raise ValueError("标定结果缺少当前会话绑定或有效手掌节点。")
            if profile.get("config_sha256") != self._config_sha or profile.get("rest_sha256") != self._rest_sha:
                raise ValueError("标定结果的配置或模型摘要不匹配，拒绝混用。")
        except ValueError as exc:
            self._profile_status = "fit_failed"
            self._warnings = [f"建议重采：{', '.join(getattr(exc, 'redo_steps', ()))}"] if getattr(exc, "redo_steps", ()) else []
            self._message("标定失败；已采步骤保留，可单独重采。", str(exc))
            self._event(session, "calibration_failed", error=str(exc), redo_steps=list(getattr(exc, "redo_steps", ())))
            return
        if self._stop.is_set() or session.stop.is_set():
            return
        if not self._capture_save_healthy(session, profile):
            return
        self._save_profile(session, profile, event="calibration_fitted")
        if self._pose_backend is not None and profile.get("calibration_motion_diagnostics", {}).get("warnings"):
            self._message("官方标定已执行，但转掌动作与算法假设存在偏差；结果保留，请看转轴提示，可单独重采第 2/3 步。")
            return
        self._message("官方 FSGlove 标定已执行；12 路适配的缺测骨段为显式先验，尚未验证实物精度。"
                      if self._pose_backend is not None else
                      "标定拟合完成（实验性，非实物精度验收）；缺失手指节点仍用显式先验。")

    def _reanchor_profile(self, session, rows):
        try:
            profile = reanchor_neutral_reference(rows, self._profile, self.rest, session_id=session.session_id)
        except ValueError as exc:
            self._profile_status = self._profile["status"]
            self._message("伸直参考未更新；原标定保留，不必从头重做。", str(exc))
            self._event(session, "neutral_reference_failed", error=str(exc))
            return
        if self._stop.is_set() or session.stop.is_set():
            return
        if not self._capture_save_healthy(session, profile):
            return
        self._save_profile(session, profile, event="neutral_reference_updated")
        reference = profile["manual_neutral_reference"]
        rejected = reference["rejected_nodes"]
        self._warnings.extend(f"伸直参考节点 {node} 未更新：{reason}" for node, reason in rejected.items())
        self._message(f"已更新 {len(reference['updated_nodes'])} 个手指节点的伸直参考；"
                      "保留手掌朝向和安装轴，未校正持续漂移。" + ("部分节点保留旧参考，详见反馈。" if rejected else "")
                      + ("坐标参考已改变，原弯曲方向校准已清除，请按需重采拇指/小指。"
                         if reference.get("functional_axes_cleared") else ""))

    def _save_profile(self, session, profile, *, event):
        if not event.startswith("shape_"):
            self._shape_invalidate()
            profile = self._shape_with_applied(profile)
        version = max(session.profile_version, len(session.profiles)) + 1
        session.profile_version = version
        profile = {**profile, "profile_version": version}
        filename = f"profile.v{version:03d}.json"
        session.profile_saved = False
        self._profile, self._rejected = profile, profile.get("rejected_nodes", {})
        self._warnings = list(profile.get("warnings", []))
        self._profile_status = profile.get("status", "provisional_not_accuracy_validated")
        self._persist_calibration_artifact(session, session.directory / filename, profile,
                                           kind="profile", profile_version=version)
        if version == 1:
            self._persist_calibration_artifact(session, session.directory / "profile.json", profile,
                                               kind="profile_alias", profile_version=version)
        self._event(session, event, profile_file=filename if session.profile_saved else None,
                    profile_saved=session.profile_saved, rejected_nodes=self._rejected)

    def _fit_functional_profile(self, session, active):
        try:
            if self._pose_backend is not None:
                entry = self._pose_backend.fit_functional_finger(
                    active["rows"], active["motion_rows"], deepcopy(self._profile),
                    finger=active["finger"], session_id=session.session_id,
                    reference_selection=active.get("reference_selection"))
            else:
                entry = fit_functional_finger(active["rows"], active["motion_rows"], self._profile,
                                              self.rest, self.config, finger=active["finger"],
                                              session_id=session.session_id)
        except ValueError as exc:
            self._profile_status = self._profile["status"]
            self._message("弯曲方向未更新；原标定与其他手指保留，只需重采这根手指。", str(exc))
            self._event(session, "functional_axis_failed", finger=active["finger"], error=str(exc),
                        diagnostics=deepcopy(getattr(exc, "diagnostics", {})),
                        rejected_slots=deepcopy(getattr(exc, "rejected_slots", {})),
                        reference_selection=active.get("reference_selection"))
            return
        if self._stop.is_set() or session.stop.is_set():
            return
        if not self._capture_save_healthy(session, self._profile):
            return
        profile = deepcopy(self._profile)
        if active.get("reference_selection"):
            entry["reference_selection"] = deepcopy(active["reference_selection"])
        profile.setdefault("functional_fingers", {})[active["finger"]] = entry
        if self._pose_backend is not None:
            profile.setdefault("functional_enabled", True)
        self._save_profile(session, profile, event="functional_axis_updated")
        name = FINGER_NAMES[active["finger"]]
        self._warnings.extend(entry.get("warnings", []))
        self._warnings.extend(f"{name}关节 {slot} 方向未更新：{reason}" for slot, reason in entry.get("rejected_slots", {}).items())
        if self._pose_backend is not None:
            scope = " IP（不改变 MCP / CMC）" if active["finger"] == "thumb" else ""
            disabled = "当前校正开关仍关闭，可手动开启对比。" if not profile["functional_enabled"] else ""
            saved = "已保存" if session.profile_saved else "已在内存中生效，但未保存到文件"
            self._message(f"{name}{scope}校准{saved}；单指轴校正 + 关节约束，非实测精度。" + disabled)
        else:
            self._message(f"{name}弯曲方向已更新；保持运动幅度，不增加平滑。属于模型轴重定向，非人体精度或极限认证。")

    def _tick(self, session):
        pipeline_start = time.perf_counter()
        timing = dict(solve=0., calibration=0., mesh=0., total=0.)
        self._check_recording(session)
        self._ensure_metadata(session)
        with session.lock:
            # The reader may commit newer samples while this tick waits for
            # the lock. Timestamp the protected state, not the earlier wait.
            now = self._clock()
            # OnlineFusion consumes this one-shot set in snapshot(). Preserve
            # its presence under the SAME lock so a later duplicate suppression
            # cannot swallow a gap/reset event from an unmapped auxiliary node.
            # Persistent quality warnings are not members of this event set.
            has_pending_input_events = bool(getattr(session.fusion, "_pending_warnings", ()))
            frame = _fusion_snapshot(session.fusion, now)
        input_diagnostics = _input_diagnostics(session.source, now)
        if input_diagnostics is not None:
            frame["input_diagnostics"] = input_diagnostics
        frame.update(schema_version=1, session_id=session.session_id, frame_index=session.frames,
                     config_sha256=self._config_sha, model_rest_sha256=self._rest_sha,
                     **self._source_tags(session))
        calibration_start = time.perf_counter()
        self._capture(session, frame, now)
        timing["calibration"] = (time.perf_counter() - calibration_start) * 1000.
        # Scheduling time is not a new measurement. Only mapped source sample
        # identity (or a changed calibration context) may produce a new valid
        # result; stale/missing-root and calibration-required heartbeats remain.
        signature = tuple((node, data.get("epoch"), data.get("sample_index"), data.get("orientation_valid"))
                          for node, data in sorted(frame["nodes"].items())
                          if node in self.config["node_to_slot"])
        source_ids = {node: (data.get("epoch"), data["sample_index"])
                      for node, data in frame["nodes"].items()
                      if node in self.config["node_to_slot"]
                      and type(data.get("sample_index")) is int and data["sample_index"] > 0}
        changed_nodes = sorted(node for node, identity in source_ids.items()
                               if session.last_sample_ids.get(node) != identity)
        source_fresh = bool(changed_nodes)
        pose_context = (self._clear_generation, id(self._profile),
                        (self._profile or {}).get("profile_version"),
                        (self._profile or {}).get("functional_enabled", True))
        root = frame["nodes"].get(self._root_node)
        health = frame.get("input_health", {})
        blocked_nodes = (set(health.get("faulted_nodes", ()))
                         | set(health.get("recovering_nodes", ()))) & set(self.config["node_to_slot"])
        input_health_blocked = bool(health.get("enabled") and blocked_nodes)
        if (self._profile is not None and session.last_pose_available
                and not input_health_blocked
                and not has_pending_input_events
                and root is not None and root.get("orientation_valid") is not False
                and 0. <= root.get("age_s", float("inf")) <= self.config["stale_after_s"]
                and signature == session.last_input_signature and pose_context == session.last_pose_context):
            session.duplicate_inputs_skipped += 1
            with self._state_lock:
                self._snapshot["calibration"] = self._calibration_state(now)
                self._snapshot["runtime"].update(duplicate_inputs_skipped=session.duplicate_inputs_skipped,
                                                  published_monotonic_s=now)
                if self._active is not None or self._shape_capture is not None:
                    self._revision += 1  # Progress only; pose/source timestamps stay unchanged.
            return False
        rejected = {}
        if input_health_blocked:
            pose = _unavailable("input_health_invalid", now)
        elif self._profile is None:
            pose = _unavailable("calibration_required", now)
        elif self._pose_backend is not None:
            solve_start = time.perf_counter()
            pose = self._pose_backend.solve(frame, self._profile, timestamp_s=now)
            rejected = pose.get("calibration_rejections", {})
            timing["solve"] = (time.perf_counter() - solve_start) * 1000.
        else:
            calibration_start = time.perf_counter()
            observations, rejected = apply_calibration(frame, self._profile)
            observations, functional = apply_functional_axes(observations, self._profile, self.rest)
            timing["calibration"] += (time.perf_counter() - calibration_start) * 1000.
            solve_start = time.perf_counter()
            pose = self._solver.solve(observations, timestamp_s=now)
            if rejected.get(self._root_node) == "filter_epoch_changed_recalibration_required":
                pose["reason"] = "calibration_epoch_changed"
            pose["functional_axis_retargeting"] = functional
            timing["solve"] = (time.perf_counter() - solve_start) * 1000.
        solve_completed_ns = host_clock.monotonic_ns()
        geometry, geometry_error = None, ""
        display_pose, display_geometry = None, None
        root = frame["nodes"].get(self._root_node)
        source_stamp = root.get("host_monotonic_ns") if root is not None else None
        if (type(source_stamp) is not int or source_stamp <= 0
                or root.get("orientation_valid") is False):
            source_stamp = None

        # Give display metadata the SAME source identity before a slow result
        # is invalidated for measurement/export below. No source time changes.
        # timestamp_s remains the solver tick; the explicit root source time
        # anchors display-level association and is never synthesized from it.
        pose.update(session_id=session.session_id, frame_index=session.frames,
                    config_sha256=self._config_sha, model_rest_sha256=self._rest_sha,
                    profile_version=(self._profile or {}).get("profile_version"),
                    source_monotonic_ns=source_stamp, source_reference_node=self._root_node)
        # Per-frame unavailable results may omit/change their algorithm tag.
        # Only the owning implementation/reference may invalidate this cache.
        owner = self._pose_backend if self._pose_backend is not None else self._solver
        revision = (type(owner).__module__, type(owner).__qualname__)
        context = (session.session_id, self._clear_generation, pose.get("profile_version"),
            self._config_sha, self._rest_sha, (self._profile or {}).get("upstream_state_sha256"),
            revision,
            (self._profile or {}).get("functional_enabled", True))
        if context != self._display_context:
            self._display_context = context
            with self._state_lock:
                self._snapshot["display_pose"] = self._snapshot["display_geometry"] = None
        if (self._pose_backend is not None and not self._stop.is_set() and not session.stop.is_set()
                and not input_health_blocked
                and not session.done.is_set() and root is not None
                and root.get("orientation_valid") is not False):
            display_pose = self._display_pose_cache.update(pose, context=context, now=self._clock())

        def source_current():
            return (not self._stop.is_set() and not session.stop.is_set() and not session.done.is_set()
                    and root is not None
                    and root.get("orientation_valid") is not False
                    and root.get("age_s", float("inf")) + max(0., self._clock() - now) <= self.config["stale_after_s"])

        if (self._geometry_backend is not None and (source_current() or display_pose is not None)
                and pose.get("pose_available") and pose.get("solver_success")):
            mesh_start = time.perf_counter()
            try:
                # Intentionally the same processing thread as solve. Neither the
                # reader nor GUI invokes MANO, and geometry is not a pose record.
                if self._pose_backend is not None:
                    render_pose = display_pose if display_pose is not None else pose
                    rendered = self._geometry_backend.render_mano(
                        render_pose["mano_pose_rotvec"], render_pose.get("shape_params"))
                    if pose.get("pose_postprocessing"):
                        # Copy metadata only, not the same mesh a second time.
                        rendered = {**rendered, "provenance": {**rendered.get("provenance", {}),
                            "pose_postprocessing": pose["pose_postprocessing"], "pose_algorithm": pose.get("algorithm"),
                            "pose_input": "upstream_axis_layer_composed_functional_constraint"}}
                    if display_pose is not None:
                        substituted = bool(display_pose["held_slots"] or display_pose["unknown_slots"])
                        display_geometry = ({**rendered, "display_only": True,
                            "provenance": {**rendered.get("provenance", {}), "display_only": True,
                                "source": "display_local_rotation_cache_not_measurement"}}
                            if substituted else rendered)
                        if not substituted:
                            geometry = rendered  # One render, shared only when no visual substitution exists.
                    else:
                        geometry = rendered
                else:
                    geometry = self._geometry_backend.render(pose["anatomical_euler_rad"], pose["root_quat_xyzw"])
            except Exception as exc:
                geometry_error = f"手模型生成失败（采集继续）：{type(exc).__name__}: {exc}"
            timing["mesh"] = (time.perf_counter() - mesh_start) * 1000.
        # A slow fit, solver or render cannot publish an expired root as fresh.
        finished = self._clock()
        if not source_current():
            pose = _unavailable("input_health_invalid" if input_health_blocked else "missing_or_stale_root", now)
            geometry = None
        pose.update(schema_version=1, session_id=session.session_id, frame_index=session.frames,
                    config_sha256=self._config_sha, model_rest_sha256=self._rest_sha,
                    source_monotonic_ns=source_stamp, source_reference_node=self._root_node,
                    **self._source_tags(session),
                    source_fresh=source_fresh, source_changed_nodes=changed_nodes,
                    profile_version=(self._profile or {}).get("profile_version"), calibration_rejections=rejected)
        # Input quality is evidence attached to this frame, not a solver failure
        # or a reason to reset filters/hide a fresh hand. Preserve it in exports
        # as well as the GUI; do not label absence of clipping as accuracy.
        pose["sensor_quality"] = deepcopy(frame.get("sensor_quality", {}))
        if "input_filter" in frame:
            pose["input_filter"] = deepcopy(frame["input_filter"])
        if "input_health" in frame:
            pose["input_health"] = deepcopy(frame["input_health"])
        pose["input_warnings"] = [warning for warning in frame.get("warnings", [])
                                  if warning.endswith("_gyro_near_rail")]
        self._shape_tick(session, frame, pose, finished)
        display_pose, display_geometry = self._shape_render_preview(display_pose, display_geometry, pose, finished)
        flush = finished - session.last_flush >= 1.
        if session.recording_policy is None:
            session.recording_policy = ResultRecordingPolicy(self.config)
        if (self.record_streams and not session.recording_disabled.is_set() and session.recording_policy.should_record(
                frame, pose, now=finished, context=dict(clear_generation=self._clear_generation,
                    profile_status=self._profile_status, active=self._active),
                force=self._active is not None or self._shape_capture is not None)):
            try:
                if session.result_recorder is not None:
                    session.result_recorder.submit(frame, pose, flush=flush)
                else:
                    self._line(session.streams["orientations"], frame)
                    self._line(session.streams["poses"], pose)
                    if flush:
                        session.streams["orientations"].flush()
                        session.streams["poses"].flush()
            except OSError as exc:
                self._disable_recording(session, "result_recording_failed", str(exc))
        # Results-only side channel: no ROS import, no second reader/solver,
        # no GUI polling. A missing/slow bridge must never reset calibration.
        export_status = None
        if self._pose_exporter is not None:
            from .pose_export import build_state
            try:
                self._pose_exporter.publish(build_state(
                    frame, pose, geometry, self.config, root_node=self._root_node,
                    solve_completed_monotonic_ns=solve_completed_ns))
            except Exception as exc:
                self._pose_exporter.record_error(exc)
            export_status = self._pose_exporter.snapshot()
        if flush:
            session.last_flush = finished
        session.frames += 1
        session.available_poses += bool(pose.get("pose_available") and pose.get("solver_success"))
        session.last_input_signature, session.last_pose_context = signature, pose_context
        session.last_sample_ids.update(source_ids)
        session.last_pose_available = bool(pose.get("pose_available") and pose.get("solver_success"))
        session.state_only_results += not source_fresh
        session.invalid_results += not session.last_pose_available
        if source_fresh and session.last_pose_available:
            session.fresh_input_results += 1
            session.solve_times.append(self._clock())
        timing["total"] = (time.perf_counter() - pipeline_start) * 1000.
        with self._state_lock:
            connected = not session.done.is_set() and not session.stop.is_set() and not self._stop.is_set()
            self._snapshot.update(orientation=frame, pose=pose, geometry=geometry if connected else None,
                                  geometry_error=geometry_error, calibration=self._calibration_state(finished))
            if not connected:
                self._snapshot["display_pose"] = self._snapshot["display_geometry"] = None
            elif display_pose is not None and (display_geometry is not None or self._geometry_backend is None):
                self._snapshot.update(display_pose=display_pose, display_geometry=display_geometry)
            self._snapshot["runtime"].update(connected=connected, received_samples=session.received,
                                              input_diagnostics=input_diagnostics,
                                              local_recording=self._local_recording_status(session),
                                              calibration_storage=self._calibration_storage_status(self._calibration_retry_session()),
                                              duplicate_inputs_skipped=session.duplicate_inputs_skipped,
                                              published_monotonic_s=finished, timing_ms=timing)
            if session.result_recorder is not None:
                self._snapshot["runtime"]["result_recording"] = session.result_recorder.snapshot()
            if export_status is not None:
                self._snapshot["runtime"]["ros_export"] = export_status
            self._revision += 1
        return True

    def _processing_status(self, session, *, cycle_ms=None):
        now = self._clock()
        while session.solve_times and session.solve_times[0] < now - 2.:
            session.solve_times.popleft()
        recent = session.solve_times
        rate = ((len(recent) - 1) / (recent[-1] - recent[0])
                if len(recent) >= 2 and recent[-1] > recent[0] and now - recent[-1] <= .25 else 0.)
        return dict(target_rate_hz=self.config["output_rate_hz"], cycle_ms=cycle_ms,
                    cycles=session.processed_cycles, overrun_cycles=session.overrun_cycles,
                    max_cycle_ms=session.max_cycle_ms, solved_rate_hz=rate,
                    fresh_input_results=session.fresh_input_results,
                    state_only_results=session.state_only_results, invalid_results=session.invalid_results,
                    duplicate_inputs_skipped=session.duplicate_inputs_skipped)

    def _run(self):
        next_tick = self._clock()
        period = 1. / self.config["output_rate_hz"]
        try:
            while not self._stop.is_set():
                for _ in range(COMMAND_CAPACITY):
                    if self._stop.is_set():
                        break
                    try:
                        name, args = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    command_error = None
                    try:
                        self._command(name, args)
                    except ValueError as exc:
                        command_error = str(exc)
                        self._message("操作未完成。", str(exc))
                    except Exception as exc:
                        command_error = str(exc)
                        self._disconnect("command_failed", f"命令或记录失败：{exc}")
                    finally:
                        # Processing the request is distinct from receiving it
                        # in a queue, and from finishing any asynchronous job
                        # it starts. Rejections also release the UI request latch.
                        with self._state_lock:
                            runtime = self._snapshot["runtime"]
                            runtime.update(command_completed_generation=
                                runtime["command_completed_generation"] + 1,
                                last_command_name=name, last_command_error=command_error)
                            self._revision += 1
                session = self._session
                try:
                    self._poll_scale_check()
                    self._poll_shape()
                    if session is not None and session.done.is_set():
                        self._disconnect("source_eof" if session.eof else "reader_error", session.error)
                    elif session is not None and session.opened and self._clock() >= next_tick:
                        tick_started = self._clock()
                        cycle_started = time.perf_counter()
                        self._tick(session)
                        cycle_ms = (time.perf_counter() - cycle_started) * 1000.
                        session.processed_cycles += 1
                        session.overrun_cycles += cycle_ms > period * 1000.
                        session.max_cycle_ms = max(session.max_cycle_ms, cycle_ms)
                        with self._state_lock:
                            self._snapshot["runtime"]["processing"] = self._processing_status(session, cycle_ms=cycle_ms)
                        # Keep an absolute cadence: scheduler wake-up jitter
                        # must not accumulate into a permanently slower rate.
                        # On overrun resume with the latest input immediately:
                        # a 9 ms computation should not become a 60 Hz stream
                        # just because it missed an 8.33 ms grid point. Do not
                        # replay any historical slot or accumulate catch-up debt.
                        next_tick += period
                        if next_tick <= tick_started:
                            # A long pause/open/calibration already missed this
                            # slot before solving even began. Resume from this
                            # fresh tick, not with one extra catch-up iteration.
                            next_tick = tick_started + period
                        next_tick = max(next_tick, self._clock())
                except Exception as exc:
                    self._disconnect("processing_failed", f"姿态处理或记录失败：{exc}")
                delay = min(.01, max(0., next_tick - self._clock())) if self._session and self._session.opened else .01
                self._wake.wait(delay)
                self._wake.clear()
        finally:
            self._disconnect("controller_stopped")
