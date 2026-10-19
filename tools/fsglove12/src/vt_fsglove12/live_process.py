"""Spawn-isolated live runtime with a bounded, latest-value GUI mailbox.

Only the child owns MANO, the processing/reader threads, and any open device.
Construction/start never connects. Pickle is used solely between this process
and its own trusted child; no pickle files or external IPC are accepted.
"""
from __future__ import annotations

import atexit
from copy import deepcopy
from functools import partial
import math
import multiprocessing as mp
import os
import pickle
import queue
import threading
import time

from .calibration import validate_rest
from .config import validate_config
from .live_runtime import (COMMAND_CAPACITY, STEP_KEYS, FUNCTIONAL_FINGERS,
                           OFFICIAL_FUNCTIONAL_FINGERS, _unavailable, _age_input_snapshot)

MAILBOX_CAPACITY = 1024 * 1024
INPUT_MAILBOX_CAPACITY = 128 * 1024
INPUT_PUBLISH_PERIOD_S = .05
GUI_PUBLISH_PERIOD_S = 1. / 60.
SHAPE_COMMAND_ARGS = {
    "capture_shape_contact": {"category", "contact_confirmed"},
    "optimize_shape": set(), "cancel_shape": set(), "preview_shape": {"mode"},
    "apply_shape_candidate": set(), "load_shape_profile": {"path"},
    "save_shape_profile": {"path"}, "revert_shape": set(),
    "retry_calibration_save": set(), "save_shape_task": {"path"},
    "load_shape_task": {"path"},
    "retry_calibration_save_to": {"directory"},
}


def _input_scale_completed(runtime):
    """Executed requests plus explicit child-queue rejections, never receipts."""
    return runtime.get("input_scale_generation", 0) + runtime.get("input_scale_rejected_count", 0)


def _idle(message="独立计算进程准备中；连接仍需手动操作。", error=None):
    now = time.monotonic()
    return dict(runtime=dict(connected=False, port=None, message=message, error=error,
                             session_id=None, output_dir=None, received_samples=0,
                             last_sample_age_s=None, published_monotonic_s=now,
                             timing_ms=dict(solve=0., calibration=0., mesh=0., total=0.)),
                orientation=None, pose=_unavailable("disconnected", now),
                geometry=None, geometry_error="", display_pose=None, display_geometry=None,
                calibration=dict(active=None, captured_steps=[], profile_status="not_calibrated",
                                 calibrated_nodes=[], rejected_nodes={}, warnings=[], clear_generation=0))


def _hide(snapshot, reason, message=None, *, disconnect=False, error=None):
    """Invalidate an owned snapshot, preserving its provenance and session IDs."""
    pose = snapshot.get("pose") or {}
    pose.update(_unavailable(reason, time.monotonic()))
    snapshot["pose"], snapshot["geometry"] = pose, None
    if disconnect:
        snapshot["runtime"]["connected"] = False
        snapshot["orientation"] = None
        snapshot["display_pose"] = snapshot["display_geometry"] = None
        if isinstance(snapshot.get("input_snapshot"), dict):
            snapshot["input_snapshot"]["runtime"]["connected"] = False
            snapshot["input_snapshot"]["orientation"] = None
    if message is not None:
        snapshot["runtime"]["message"] = message
    if error is not None:
        snapshot["runtime"]["error"] = error
    if reason in {"calibration_clear_requested", "input_session_changed", "disconnected"}:
        snapshot["display_pose"] = snapshot["display_geometry"] = None
    if disconnect or reason in {"calibration_clear_requested", "input_session_changed"}:
        shape = snapshot.get("calibration", {}).get("shape_calibration")
        if isinstance(shape, dict):
            # This is local visual suppression, never a completed worker ACK.
            # Keep the applied profile's metadata and revision. In particular,
            # ordinary disconnected polling must not erase a profile the user
            # explicitly loads while disconnected.
            shape.update(status="invalidated", candidate_available=False,
                         contacts={}, active_category=None, busy=False, phase=None,
                         preview_mode="current", invalidation_reason=reason)
    return snapshot


def _publish(mailbox, snapshot, command_seen, *, input_only=False):
    buffer, length, sequence, lock = mailbox
    envelope = (time.monotonic(), command_seen, snapshot)
    try:
        payload = pickle.dumps(envelope, protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload) > len(buffer):
            raise ValueError(f"snapshot_capacity_exceeded: {len(payload)} > {len(buffer)} bytes")
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        if input_only:
            runtime = snapshot.get("runtime", {})
            snapshot = dict(runtime=dict(connected=bool(runtime.get("connected")),
                session_id=runtime.get("session_id"), published_monotonic_s=time.monotonic(),
                input_monitor_error=error), orientation=None)
        else:
            snapshot = _idle("计算快照无法传输；隐藏模型，采集控制仍可停止。", error)
            snapshot["geometry_error"] = snapshot["runtime"]["error"]
        payload = pickle.dumps((time.monotonic(), command_seen, snapshot), protocol=pickle.HIGHEST_PROTOCOL)
    if not lock.acquire(False):
        return False  # Never queue an older snapshot behind the current one.
    try:
        memoryview(buffer).cast("B")[:len(payload)] = payload
        length.value = len(payload)
        sequence.value += 1
    finally:
        lock.release()
    return True


def _replay_source(_port, *, path, speed):
    from .live_replay import PacedCSVSource
    return PacedCSVSource(path, speed=speed)


def _child_main(config, rest, model_axes, options, commands, stop, mailbox, input_mailbox,
                runtime_factory, backend_factory):
    runtime, command_seen, fatal_error = None, 0, None
    input_scale_rejected_count, input_scale_error = 0, None
    shape_rejected_count, shape_error = 0, None
    input_thread, input_stop = None, threading.Event()
    pose_exporter = None
    high_rate = (runtime_factory is None and options.get("upstream_checkout") is not None
                 and config["output_rate_hz"] > 30.)
    gc_policy = None
    try:
        # Experimental only: the measured whole-window maximum did not improve
        # consistently. Normal high-rate mode keeps Python's GC policy intact.
        if high_rate and os.environ.get("VT_FSGLOVE_STARTUP_GC_FREEZE", "0") == "1":
            from .live_gc import StartupGcFreeze
            # Capture the clean, fresh-child interpreter baseline BEFORE model
            # loading. Never run this in the GUI or an existing live session.
            gc_policy = StartupGcFreeze(scope="compute_child", allow_bootstrap_permanent=True)
        if backend_factory is None:
            from .mano import ManoBackend
            backend = ManoBackend(options["mano_assets"], hand=options["hand"], compact_live_geometry=True)
        else:
            backend = backend_factory(options["mano_assets"], hand=options["hand"])
        backend.export_rest()  # Explicit model initialization, before any device can be opened.
        torch = getattr(backend, "_torch", None)
        if torch is not None:
            torch.set_num_threads(1)  # Tiny CPU inference only; never mutate GUI-process settings.
        if stop.is_set():
            return
        if runtime_factory is None:
            from .live_runtime import LiveController
            runtime_factory = LiveController
        kwargs = dict(output_root=options["output_root"], geometry_backend=backend)
        if high_rate:
            from .live_recording_process import ProcessResultRecorder
            kwargs["result_recorder_factory"] = ProcessResultRecorder
        if options.get("ros_socket"):
            from .pose_export import PoseDatagramExporter
            pose_exporter = PoseDatagramExporter(options["ros_socket"])
            kwargs["pose_exporter"] = pose_exporter
        if options.get("upstream_checkout") is not None:
            from .upstream_backend import OfficialFSGloveBackend
            kwargs["pose_backend"] = OfficialFSGloveBackend(
                config, rest, checkout=options["upstream_checkout"], mano_assets_root=options["mano_assets"])
        if options["replay_raw"] is not None:
            kwargs["source_factory"] = partial(_replay_source, path=options["replay_raw"], speed=options["replay_speed"])
        if gc_policy is not None:
            gc_policy.start()  # Models ready; no runtime, session or frames yet.
        runtime = runtime_factory(config, rest, model_axes, **kwargs)
        if high_rate:
            runtime._snapshot["runtime"]["result_recording_mode"] = "bounded_process"
        if gc_policy is not None:
            runtime._snapshot["runtime"]["startup_gc_policy"] = gc_policy.snapshot()
        runtime.start()
        parent = mp.parent_process()
        input_reader = getattr(runtime, "latest_input_snapshot", None)
        if callable(input_reader):
            def publish_input():
                # Independent of pose publication and the processing worker.
                # This thread only reads already-fused state; never serial I/O.
                last_runtime = dict(connected=False, session_id=None)
                while not input_stop.is_set() and not stop.is_set():
                    try:
                        value = input_reader()
                        light = {key: value.get(key) for key in ("runtime", "orientation")}
                        light["runtime"] = {**light["runtime"],
                            "input_scale_rejected_count": input_scale_rejected_count,
                            "input_scale_command_error": input_scale_error,
                            "shape_rejected_count": shape_rejected_count,
                            "shape_command_error": shape_error or light["runtime"].get("shape_command_error")}
                        last_runtime = dict(light["runtime"])
                        _publish(input_mailbox, light, command_seen, input_only=True)
                    except Exception as exc:
                        light = dict(runtime=dict(last_runtime,
                            published_monotonic_s=time.monotonic(), input_monitor_error=str(exc)), orientation=None)
                        _publish(input_mailbox, light, command_seen, input_only=True)
                    input_stop.wait(INPUT_PUBLISH_PERIOD_S)
            input_thread = threading.Thread(target=publish_input, name="fsglove12-input-publisher", daemon=True)
            input_thread.start()
        revision_reader = getattr(runtime, "snapshot_revision", None)
        # Trusted child-only path: borrow immutable published payloads until
        # synchronous pickle completes. The public GUI API still returns a
        # deep-independent decoded snapshot. Do not mutate borrowed payloads.
        pose_reader = getattr(runtime, "_latest_pose_publication", None)
        if not callable(pose_reader):
            pose_reader = getattr(runtime, "latest_pose_snapshot", runtime.latest_snapshot)
        last_revision, published_command = object(), -1
        rejected = None
        while not stop.is_set() and (parent is None or parent.is_alive()):
            for _ in range(COMMAND_CAPACITY):
                if stop.is_set():
                    break
                try:
                    command_id, name, args = commands.get_nowait()
                except queue.Empty:
                    break
                command_seen = command_id
                if not runtime.submit(name, **args):
                    rejected = f"操作队列繁忙或命令不可用：{name}；未自动重试。"
                    if name == "use_input_scale_candidate":
                        # No runtime command will execute its finally block.
                        # Report rejection separately; do not falsify an actual
                        # configuration-execution generation or retry silently.
                        input_scale_rejected_count += 1
                        input_scale_error = rejected
                    elif name in SHAPE_COMMAND_ARGS:
                        # A rejected queue receipt is not a successful worker
                        # execution. The UI can release only its click latch.
                        shape_rejected_count += 1
                        shape_error = rejected
                elif name == "use_input_scale_candidate":
                    input_scale_error = None
                elif name in SHAPE_COMMAND_ARGS:
                    shape_error = None
            # Legacy injected runtimes without a token retain compatibility.
            # Production avoids deepcopy/pickle of an unchanged 778-vertex mesh.
            revision = revision_reader() if callable(revision_reader) else object()
            if revision != last_revision or command_seen != published_command or rejected:
                snapshot = dict(pose_reader())
                snapshot["runtime"] = {**snapshot["runtime"],
                    "input_scale_rejected_count": input_scale_rejected_count,
                    "input_scale_command_error": input_scale_error,
                    "shape_rejected_count": shape_rejected_count,
                    "shape_command_error": shape_error or snapshot["runtime"].get("shape_command_error")}
                snapshot.pop("input_snapshot", None)  # Its own bounded mailbox owns this cadence.
                if rejected or input_scale_error or shape_error:
                    snapshot["runtime"]["error"] = rejected or input_scale_error or shape_error
                if _publish(mailbox, snapshot, command_seen):
                    last_revision, published_command, rejected = revision, command_seen, None
            # GUI consumes latest values at at most 60 Hz; ROS is sent directly
            # by the solver, so a 120 Hz result stream does not require copying
            # and pickling 120 full meshes for the display.
            stop.wait(GUI_PUBLISH_PERIOD_S)
    except Exception as exc:
        fatal_error = f"{type(exc).__name__}: {exc}"
        _publish(mailbox, _idle("独立计算进程发生错误；不会自动连接或重启。",
                               fatal_error), command_seen)
    finally:
        try:
            input_stop.set()
            if input_thread is not None:
                input_thread.join(.5)
            if runtime is not None:
                runtime.stop()
                # Never abandon source/recording ownership on a short timeout.
                while not runtime.join(.1):
                    snapshot = _idle(error=fatal_error) if fatal_error else runtime.latest_snapshot()
                    _publish(mailbox, _hide(snapshot, "stopping",
                                           "正在等待采集线程关闭；不会强制终止。", disconnect=True), command_seen)
                snapshot = _idle(error=fatal_error) if fatal_error else runtime.latest_snapshot()
                _publish(mailbox, _hide(snapshot, "stopped",
                                       "独立计算进程已关闭。", disconnect=True), command_seen)
            if pose_exporter is not None:
                pose_exporter.close()
        finally:
            if gc_policy is not None:
                gc_policy.close()


class ProcessLiveController:
    """Nonblocking GUI proxy; all factories execute only in a spawn child.

    ``runtime_factory`` has LiveController's constructor/API and must be a
    spawn-picklable top-level callable. ``backend_factory`` is a test seam with
    ManoBackend's constructor/export_rest/render API. A caller must use a guarded
    ``if __name__ == '__main__'`` entry point and stop/join on normal shutdown.
    No terminate/kill fallback exists, and reconnect is always an explicit command.
    """

    renders_geometry = True

    def __init__(self, config, rest, model_axes, *, output_root, mano_assets,
                 hand=None, replay_raw=None, replay_speed=1., runtime_factory=None,
                 backend_factory=None, mailbox_capacity=MAILBOX_CAPACITY, upstream_checkout=None,
                 ros_socket=None):
        self.config = validate_config(deepcopy(config))
        hand = self.config["hand"] if hand is None else hand
        if hand != self.config["hand"]:
            raise ValueError("backend hand does not match configuration hand")
        from .config import check_model_hand
        check_model_hand(self.config, rest)
        self._official = upstream_checkout is not None
        if ros_socket is not None:
            from .pose_export import validate_socket_path
            ros_socket = validate_socket_path(ros_socket)
            if not self._official:
                raise ValueError("ROS results export requires the official FSGlove backend")
        validate_rest(rest)
        if hand not in ("right", "left"):
            raise ValueError("hand must be right or left")
        if not isinstance(mailbox_capacity, int) or isinstance(mailbox_capacity, bool) or not 4096 <= mailbox_capacity <= MAILBOX_CAPACITY:
            raise ValueError("mailbox_capacity must be between 4096 and 1048576 bytes")
        if isinstance(replay_speed, bool) or not isinstance(replay_speed, (int, float)) or not math.isfinite(replay_speed) or replay_speed <= 0:
            raise ValueError("replay_speed must be finite and positive")
        self._root_node = next(node for node, slot in self.config["node_to_slot"].items() if slot == 0)
        self._context = mp.get_context("spawn")
        self._mailbox = (self._context.RawArray("B", mailbox_capacity),
                         self._context.RawValue("I", 0), self._context.RawValue("Q", 0), self._context.Lock())
        self._input_mailbox = (self._context.RawArray("B", INPUT_MAILBOX_CAPACITY),
            self._context.RawValue("I", 0), self._context.RawValue("Q", 0), self._context.Lock())
        self._commands = self._context.Queue(maxsize=COMMAND_CAPACITY)
        self._stop = self._context.Event()
        self._process = None
        self._args = (self.config, deepcopy(rest), deepcopy(model_axes),
                      dict(output_root=str(output_root), mano_assets=str(mano_assets), hand=hand,
                           replay_raw=None if replay_raw is None else str(replay_raw), replay_speed=replay_speed,
                           upstream_checkout=None if upstream_checkout is None else str(upstream_checkout),
                           ros_socket=ros_socket),
                      self._commands, self._stop, self._mailbox, self._input_mailbox, runtime_factory, backend_factory)
        self._cached, self._sequence, self._published, self._command_seen = _idle(), 0, None, 0
        self._input_cached, self._input_sequence, self._input_published, self._input_command_seen = None, 0, None, 0
        self._command_id, self._invalidate_id, self._disconnect_pending = 0, 0, False
        self._pending_clear_generation = None
        self._pending_input_scale_generation = None
        self._input_scale_generation_floor = 0
        self._started_at = None
        self._queue_closed = False

    @property
    def pid(self):
        return self._process.pid if self._process is not None else None

    def is_alive(self):
        return self._process is not None and self._process.is_alive()

    def start(self):
        if self._stop.is_set():
            raise RuntimeError("已停止的 ProcessLiveController 不能重新启动。")
        if self._process is None:
            self._process = self._context.Process(target=_child_main, args=self._args,
                                                 name="fsglove12-compute", daemon=False)
            self._process.start()
            self._started_at = time.monotonic()
            atexit.register(self._at_exit)
        return self

    def submit(self, name, **args):
        valid = {"connect": {"port"}, "disconnect": set(), "capture": {"step"},
                 "cancel_capture": set(), "clear_calibration": set(), "reanchor_neutral": set(),
                 "capture_functional": {"finger"}, "set_functional_enabled": {"enabled"},
                 "capture_functional_hand": set(), "use_input_scale_candidate": set(),
                 **SHAPE_COMMAND_ARGS}
        if self._stop.is_set() or self._queue_closed or name not in valid or set(args) != valid[name]:
            return False
        if self._process is not None and not self.is_alive():
            return False
        if name == "connect" and (not isinstance(args["port"], str) or not args["port"].strip() or len(args["port"]) > 4096):
            return False
        if name == "connect" and self._pending_input_scale_generation is not None:
            return False  # Finish the explicit reconfiguration before reconnecting.
        if name == "capture" and args["step"] not in STEP_KEYS:
            return False
        if name == "reanchor_neutral" and self._official:
            return False
        if name == "capture_functional_hand" and not self._official:
            return False
        if name == "set_functional_enabled" and (not self._official or type(args["enabled"]) is not bool):
            return False
        if name in SHAPE_COMMAND_ARGS:
            if not self._official:
                return False
            if name == "capture_shape_contact" and (type(args["category"]) is not int
                    or args["category"] not in (2, 3, 4, 5)
                    or args["contact_confirmed"] is not True):
                return False
            if name == "preview_shape" and (type(args["mode"]) is not str
                    or args["mode"] not in ("current", "candidate", "baseline")):
                return False
            if name in {"load_shape_profile", "save_shape_profile", "load_shape_task", "save_shape_task"} and (
                    not isinstance(args["path"], str) or not args["path"].strip()
                    or len(args["path"]) > 4096):
                return False
            if name == "retry_calibration_save_to" and (
                    not isinstance(args["directory"], str) or not args["directory"].strip()
                    or len(args["directory"]) > 4096):
                return False
        supported_fingers = OFFICIAL_FUNCTIONAL_FINGERS if self._official else FUNCTIONAL_FINGERS
        if name == "capture_functional" and args["finger"] not in supported_fingers:
            return False
        command_id = self._command_id + 1
        try:
            self._commands.put_nowait((command_id, name, deepcopy(args)))
        except queue.Full:
            return False
        self._command_id = command_id
        if name == "disconnect":
            self._invalidate_id, self._disconnect_pending = command_id, True
            self._pending_clear_generation = None
        elif name == "connect":
            self._disconnect_pending = False  # Explicit user action only.
            self._pending_clear_generation = None
        elif name == "clear_calibration":
            current = self._cached.get("calibration", {}).get("clear_generation", 0)
            self._pending_clear_generation = max(current, self._pending_clear_generation or 0) + 1
        elif name == "use_input_scale_candidate":
            current = _input_scale_completed(self._cached.get("runtime", {}))
            if self._input_cached is not None:
                current = max(current, _input_scale_completed(self._input_cached.get("runtime", {})))
            expected = max(current, self._input_scale_generation_floor) + 1
            self._input_scale_generation_floor = expected
            self._pending_input_scale_generation = expected
            self._pending_clear_generation = None
        return True

    def latest_snapshot(self):
        buffer, length, sequence, lock = self._mailbox
        payload, new_sequence = None, self._sequence
        if lock.acquire(False):
            try:
                if sequence.value != self._sequence and 0 < length.value <= len(buffer):
                    payload = bytes(memoryview(buffer).cast("B")[:length.value])
                    new_sequence = sequence.value
            finally:
                lock.release()
        if payload is not None:
            self._published, self._command_seen, self._cached = pickle.loads(payload)
            self._sequence = new_sequence
        result, now = deepcopy(self._cached), time.monotonic()
        result["input_snapshot"] = self.latest_input_snapshot()
        alive = self.is_alive()
        runtime = result["runtime"]
        runtime.update(process_alive=alive, process_pid=self.pid, ipc_sequence=self._sequence)
        if self._input_cached is not None:
            runtime["connected"] = bool(result["input_snapshot"]["runtime"].get("connected"))
        if self._stop.is_set():
            return _hide(result, "stopping", "独立计算进程停止中。", disconnect=True)
        if self._process is not None and not alive:
            return _hide(result, "process_exited", "独立计算进程已退出；不会自动重启或重连。", disconnect=True,
                         error=runtime.get("error") or f"计算进程退出码：{self._process.exitcode}")
        if self._input_scale_generation_floor:
            completed = _input_scale_completed(runtime)
            input_completed = _input_scale_completed(result["input_snapshot"]["runtime"])
            expected = self._input_scale_generation_floor
            if min(completed, input_completed) >= expected:
                self._pending_input_scale_generation = None
                runtime["input_scale_pending"] = False
            else:
                # Receiving a command is not execution acknowledgement. Both
                # independently published mailboxes must confirm completion;
                # neither old fused inputs nor old calibrated meshes may leak.
                runtime["input_scale_pending"] = True
                generation = result.get("calibration", {}).get("clear_generation", 0)
                shape = result.get("calibration", {}).get("shape_calibration")
                result["calibration"] = dict(_idle()["calibration"],
                    profile_status="input_scale_pending", clear_generation=generation)
                if isinstance(shape, dict):
                    result["calibration"]["shape_calibration"] = shape
                return _hide(result, "input_scale_change_requested",
                    "正在切换输入换算并清除旧标定；完成后请手动连接。", disconnect=True)
        if self._disconnect_pending and self._command_seen >= self._invalidate_id and not runtime.get("connected"):
            self._disconnect_pending = False
        if self._disconnect_pending or self._command_seen < self._invalidate_id:
            return _hide(result, "disconnect_requested", "断开请求已提交；隐藏旧姿态。", disconnect=True)
        if self._pending_clear_generation is not None:
            # Acknowledgement of queue.submit is not acknowledgement that a
            # slow calibration worker actually executed the clear. Wait for
            # the runtime's completed-clear generation before showing poses.
            completed = result.get("calibration", {}).get("clear_generation", 0)
            if completed >= self._pending_clear_generation:
                self._pending_clear_generation = None
            else:
                shape = result.get("calibration", {}).get("shape_calibration")
                result["calibration"] = dict(active=None, captured_steps=[], functional_fingers=[],
                    functional_enabled=True, functional_hand_result=None, functional_diagnostics={},
                    profile_status="clearing", motion_diagnostics={}, calibrated_nodes=[],
                    rejected_nodes={}, warnings=[], clear_generation=completed)
                if isinstance(shape, dict):
                    result["calibration"]["shape_calibration"] = shape
                return _hide(result, "calibration_clear_requested",
                             "正在清除标定与滤波参考；串口保持连接，请稍候。")
        input_session = result["input_snapshot"]["runtime"].get("session_id")
        if (input_session is not None and runtime.get("session_id") is not None
                and input_session != runtime["session_id"]):
            return _hide(result, "input_session_changed", "输入会话已切换；等待同会话姿态结果。")
        elapsed = max(0., now - self._published) if self._published is not None else 0.
        if (runtime.get("connected") and self._published is not None and elapsed > max(.25, self.config["stale_after_s"] * 2.)
                or self._published is None and self._started_at is not None and now - self._started_at > 30.):
            return _hide(result, "process_unresponsive", "姿态计算暂未更新；输入连接状态由独立监视显示，不自动重连。")
        if runtime.get("last_sample_age_s") is not None:
            runtime["last_sample_age_s"] += elapsed
        frame = result.get("orientation")
        if frame is not None:
            # Child's latest_snapshot already aged nodes when sending; only add
            # IPC elapsed time here, not the frame age for a second time.
            frame["nodes"] = {node: {**data, "age_s": data.get("age_s", 0.) + elapsed}
                              for node, data in frame.get("nodes", {}).items()
                              if data.get("age_s", float("inf")) + elapsed <= self.config["stale_after_s"]}
        pose = result.get("pose") or {}
        if (not runtime.get("connected") or not frame or self._root_node not in frame["nodes"]
                or pose.get("pose_available") and now - pose.get("timestamp_s", -math.inf) > self.config["stale_after_s"]):
            return _hide(result, "missing_or_stale_root" if runtime.get("connected") else "disconnected")
        if not pose.get("pose_available") or not pose.get("solver_success"):
            result["geometry"] = None
        return result

    def latest_input_snapshot(self):
        """Nonblocking, geometry-free reader status with one IPC-age increment."""
        buffer, length, sequence, lock = self._input_mailbox
        payload, new_sequence = None, self._input_sequence
        if lock.acquire(False):
            try:
                if sequence.value != self._input_sequence and 0 < length.value <= len(buffer):
                    payload = bytes(memoryview(buffer).cast("B")[:length.value])
                    new_sequence = sequence.value
            finally:
                lock.release()
        if payload is not None:
            self._input_published, self._input_command_seen, self._input_cached = pickle.loads(payload)
            self._input_sequence = new_sequence
        now = time.monotonic()
        if self._input_cached is None:
            # Old injected runtimes need not implement the new monitoring API.
            result = dict(runtime=deepcopy(self._cached["runtime"]),
                          orientation=deepcopy(self._cached.get("orientation")))
            published = self._published
        else:
            result, published = deepcopy(self._input_cached), self._input_published
        elapsed = max(0., now - published) if published is not None else 0.
        _age_input_snapshot(result, elapsed, self.config["stale_after_s"])
        runtime = result["runtime"]
        runtime.update(process_alive=self.is_alive(), process_pid=self.pid,
            input_ipc_sequence=self._input_sequence, transport_age_s=elapsed,
            ages_as_of_monotonic_s=now,
            input_snapshot_stale=bool(published is not None and elapsed > max(.25, 2 * self.config["stale_after_s"])))
        if (self._stop.is_set() or self._process is not None and not self.is_alive()
                or self._disconnect_pending or self._input_command_seen < self._invalidate_id):
            runtime["connected"] = False
            result["orientation"] = None
        if (self._pending_clear_generation is not None
                and runtime.get("clear_generation", -1) < self._pending_clear_generation):
            result["orientation"] = None
            runtime["clear_pending"] = True
        if _input_scale_completed(runtime) < self._input_scale_generation_floor:
            result["orientation"] = None
            runtime["connected"] = False
            runtime["input_scale_pending"] = True
        else:
            runtime["input_scale_pending"] = False
        return result

    def stop(self):
        self._stop.set()  # Independent of either bounded command queue.

    def join(self, timeout=None):
        if self._process is not None:
            self._process.join(timeout)
        ended = not self.is_alive()
        if ended and not self._queue_closed:
            self._commands.cancel_join_thread()
            self._commands.close()
            self._queue_closed = True
            atexit.unregister(self._at_exit)
        return ended

    def _at_exit(self):
        self.stop()
        self.join()  # Never abandon an active reader/recorder or force-kill it.
