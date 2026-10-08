"""Read-only ROS preview. No acquisition, hardware control, or calibration solve.

ROS imports are deliberately lazy. Ages and rates describe local subscription
arrival, not device-clock accuracy. Timing joins use the exact shared ROS stamp
and validated host callback mapping, just as the offline bag reader does.
"""
from __future__ import annotations

from collections import OrderedDict, deque
import re
import threading
import time
from types import SimpleNamespace
import uuid

import numpy as np

from .bag_reader import (
    decode_image_message,
    intrinsics_from_message,
    stamp_to_nanoseconds,
    _same_intrinsics,
)
from .charuco import detect_board_pose
from .config import CalibrationConfig
from .transforms import Transform


_STALE_S = 1.0
_DETECTION_PERIOD_S = 0.2
_IMAGE_CACHE_SIZE = 8  # A bounded timing join, never a detection work queue.
_TIMING_CACHE_SIZE = 64
_TRACKER_ID = re.compile(r"[0-9a-f]{64}")


def _load_ros() -> SimpleNamespace:
    try:
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import (
            QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
        )
        from sensor_msgs.msg import Image, CameraInfo
        from vt_camera_msgs.msg import CameraFrameTiming
        from vt_tracker_msgs.msg import TrackerSample
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "ROS 2 预览依赖不可用；请先 source /opt/ros/jazzy/setup.bash 和 "
            "ros2_ws/install/setup.bash，再使用 .venv-calibration 启动。"
        ) from error
    return SimpleNamespace(**locals())


def _image_rgb(message: object, config: CalibrationConfig) -> tuple[np.ndarray, int]:
    if str(message.header.frame_id) != config.camera_frame:
        raise ValueError("Image frame_id 与配置不一致")
    stamp = stamp_to_nanoseconds(message.header.stamp)
    image, encoding = decode_image_message(message)
    if encoding == "mono8":
        rgb = np.repeat(image[:, :, None], 3, axis=2)
    elif encoding in {"bgr8", "bgra8"}:
        rgb = image[:, :, :3][:, :, ::-1].copy()
    else:
        rgb = image[:, :, :3].copy()
    rgb.setflags(write=False)
    return rgb, stamp


def _timing_pair(message: object, config: CalibrationConfig) -> tuple[int, int]:
    for field, expected in (("camera_name", config.camera_name),
                            ("camera_model", config.camera_model),
                            ("serial_number", config.camera_serial)):
        if str(getattr(message, field)) != expected:
            raise ValueError(f"CameraFrameTiming {field} 与配置不一致")
    required = (int(message.GROUP_VALID_COMMON_STAMP)
                | int(message.GROUP_VALID_IDENTITY)
                | int(message.GROUP_VALID_CALLBACK_CLOCKS))
    if int(message.group_validity_flags) & required != required:
        raise ValueError("CameraFrameTiming 缺少有效共同时间戳/身份/主机回调时钟")
    stamp, host = int(message.shared_ros_timestamp_ns), int(message.group_host_realtime_ns)
    if stamp < 0 or host <= 0:
        raise ValueError("CameraFrameTiming 主机时间映射无效")
    return stamp, host


def _tracker_pose(message: object, config: CalibrationConfig) -> tuple[str, Transform | None]:
    if str(message.role) != config.tracker_role:
        raise ValueError("TrackerSample role 与配置不一致")
    if str(message.header.frame_id) != config.world_frame:
        raise ValueError("TrackerSample world frame 与配置不一致")
    if not bool(message.pose_valid) or int(message.tracking_status) & 0x0F != 2:
        return "", None
    identity = str(message.tracker_id)
    if _TRACKER_ID.fullmatch(identity) is None:
        raise ValueError("TrackerSample tracker_id 不是有效的 64 位身份指纹")
    if int(message.host_realtime_ns) <= 0:
        raise ValueError("TrackerSample host_realtime_ns 无效")
    p, q = message.pose.position, message.pose.orientation
    transform = Transform.from_quaternion_xyzw(
        (float(p.x), float(p.y), float(p.z)),
        (float(q.x), float(q.y), float(q.z), float(q.w)),
    )
    return identity, transform


def _age(now: float, stamp: float | None) -> float | None:
    return None if stamp is None else max(0.0, now - stamp)


def _rate(arrivals: deque, now: float) -> float:
    recent = [stamp for stamp in arrivals if now - stamp <= 2.0]
    if len(recent) < 2 or now - recent[-1] > _STALE_S:
        return 0.0
    span = recent[-1] - recent[0]
    return (len(recent) - 1) / span if span > 0.0 else 0.0


class LiveMonitor:
    """Latest-value monitor with independent ROS and bounded-rate vision workers.

    ``board_valid`` means only a recent board detection passed the configured
    corner/reprojection checks. It never certifies recorded data or extrinsics.
    ``stop`` is bounded; a stuck native detector cannot keep the ROS node alive.
    A monitor whose old worker has not exited cannot be restarted.
    """

    def __init__(self, config: CalibrationConfig):
        self.config = config
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._vision_wake = threading.Event()
        self._ready = threading.Event()
        self._thread = self._vision_thread = None
        self._executor = None
        self._clock = time.monotonic
        self._reset()

    def _reset(self) -> None:
        self._running = False
        self._errors: dict[str, str] = {}
        self._received = dict.fromkeys(("image", "info", "timing", "tracker"))
        self._arrivals = {key: deque(maxlen=240) for key in ("image", "tracker")}
        self._images: OrderedDict[int, tuple[np.ndarray, float]] = OrderedDict()
        self._timings: OrderedDict[int, int | None] = OrderedDict()
        self._intrinsics = self.config.fallback_intrinsics
        self._tracker_identity: str | None = None
        self._tracker: Transform | None = None
        self._board = None
        self._board_received = None
        self._generation = 0
        self._last_detection_key = None
        self._last_detection_started = -float("inf")

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive() and self._running:
                return
            if any(thread is not None and thread.is_alive()
                   for thread in (self._thread, self._vision_thread)):
                raise RuntimeError("上一预览线程仍在退出，暂不能重新启动。")
            with self._lock:
                self._reset()
            try:
                ros = _load_ros()
            except Exception as error:
                with self._lock:
                    self._errors["ROS"] = str(error)
                raise RuntimeError(str(error)) from error
            self._stop_event.clear()
            self._vision_wake.clear()
            self._ready.clear()
            self._thread = threading.Thread(target=self._run_ros, args=(ros,),
                                            name="calibration-preview-ros", daemon=True)
            self._vision_thread = threading.Thread(target=self._run_vision,
                                                   name="calibration-preview-board", daemon=True)
            self._thread.start()
            self._vision_thread.start()
        if not self._ready.wait(3.0):
            self.stop()
            raise RuntimeError("ROS 2 预览初始化超时；未启动采集或设备控制。")
        with self._lock:
            running, error = self._running, self._errors.get("ROS", "ROS 2 预览启动失败")
        if not running:
            self.stop()
            raise RuntimeError(error)

    def stop(self) -> None:
        with self._lifecycle_lock:
            self._stop_event.set()
            self._vision_wake.set()
            with self._lock:
                executor = self._executor
                self._running = False
            if executor is not None:
                try:
                    executor.wake()
                except Exception:
                    pass  # It may already have been destroyed by its owner.
            deadline = time.monotonic() + 2.0
            for thread in (self._thread, self._vision_thread):
                if thread is not None and thread is not threading.current_thread():
                    thread.join(max(0.0, deadline - time.monotonic()))
            if any(thread is not None and thread.is_alive()
                   for thread in (self._thread, self._vision_thread)):
                with self._lock:
                    self._errors["ROS"] = "预览已停止显示；后台线程尚未退出，不能重新启动。"

    def _run_ros(self, ros: SimpleNamespace) -> None:
        context = node = executor = None
        try:
            context = ros.Context()
            context.init(args=[])
            node = ros.Node("tracker_camera_preview_" + uuid.uuid4().hex[:8], context=context,
                            enable_rosout=False, start_parameter_services=False)
            executor = ros.SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = ros.QoSProfile(depth=1, history=ros.HistoryPolicy.KEEP_LAST,
                                 reliability=ros.ReliabilityPolicy.BEST_EFFORT,
                                 durability=ros.DurabilityPolicy.VOLATILE)
            for message_type, topic, callback in (
                (ros.Image, self.config.image_topic, self._on_image),
                (ros.CameraInfo, self.config.camera_info_topic, self._on_info),
                (ros.CameraFrameTiming, self.config.timing_topic, self._on_timing),
                (ros.TrackerSample, self.config.tracker_sample_topic, self._on_tracker),
            ):
                node.create_subscription(message_type, topic, callback, qos)
            with self._lock:
                self._executor = executor
                self._running = not self._stop_event.is_set()
            self._ready.set()
            while not self._stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.05)
        except Exception as error:
            with self._lock:
                self._errors["ROS"] = f"ROS 2 只读预览失败：{error}"
        finally:
            self._stop_event.set()
            self._vision_wake.set()
            # Each resource is released independently, even after partial init.
            for cleanup in (
                lambda: executor.shutdown(timeout_sec=0.2) if executor is not None else None,
                lambda: node.destroy_node() if node is not None else None,
                lambda: context.try_shutdown() if context is not None else None,
            ):
                try:
                    cleanup()
                except Exception as error:
                    with self._lock:
                        self._errors["ROS cleanup"] = str(error)
            with self._lock:
                self._executor = None
                self._running = False
            self._ready.set()

    def _receive(self, kind: str) -> float:
        now = self._clock()
        self._received[kind] = now
        if kind in self._arrivals:
            self._arrivals[kind].append(now)
        return now

    def _invalidate_board(self) -> None:
        self._generation += 1
        self._board = self._board_received = None

    def _on_image(self, message: object) -> None:
        try:
            rgb, stamp = _image_rgb(message, self.config)
            error = None
        except (AttributeError, TypeError, ValueError, OverflowError) as caught:
            error = str(caught)
        with self._lock:
            now = self._receive("image")
            if error is not None:
                self._errors["Image"] = error
                self._images.clear()
                self._invalidate_board()
            else:
                self._errors.pop("Image", None)
                self._images[stamp] = (rgb, now)
                self._images.move_to_end(stamp)
                while len(self._images) > _IMAGE_CACHE_SIZE:
                    self._images.popitem(last=False)
        self._vision_wake.set()

    def _on_info(self, message: object) -> None:
        try:
            if str(message.header.frame_id) != self.config.camera_frame:
                raise ValueError("CameraInfo frame_id 与配置不一致")
            intrinsics, error = intrinsics_from_message(message), None
        except (AttributeError, TypeError, ValueError, OverflowError) as caught:
            intrinsics, error = None, str(caught)
        with self._lock:
            self._receive("info")
            if error is not None:
                self._errors["CameraInfo"] = error
            else:
                self._errors.pop("CameraInfo", None)
            if (intrinsics is None or self._intrinsics is None
                    or not _same_intrinsics(intrinsics, self._intrinsics)):
                self._invalidate_board()
            self._intrinsics = intrinsics
        self._vision_wake.set()

    def _on_timing(self, message: object) -> None:
        stamp = None
        try:
            stamp, host = _timing_pair(message, self.config)
            error = None
        except (AttributeError, TypeError, ValueError, OverflowError) as caught:
            error = str(caught)
        with self._lock:
            self._receive("timing")
            if error is None and stamp in self._timings and self._timings[stamp] != host:
                self._timings[stamp] = None  # Poison conflicts until evicted.
                error = "同一图像时间戳存在冲突的主机 timing 映射"
            if error is not None:
                self._errors["Timing"] = error
                self._invalidate_board()
                if stamp is None:
                    self._timings.clear()
            else:
                self._errors.pop("Timing", None)
                self._timings[stamp] = host
                self._timings.move_to_end(stamp)
                while len(self._timings) > _TIMING_CACHE_SIZE:
                    self._timings.popitem(last=False)
        self._vision_wake.set()

    def _on_tracker(self, message: object) -> None:
        try:
            identity, transform = _tracker_pose(message, self.config)
            error = None
        except (AttributeError, TypeError, ValueError, OverflowError) as caught:
            identity, transform, error = "", None, str(caught)
        with self._lock:
            self._receive("tracker")
            if transform is not None:
                if self._tracker_identity not in {None, identity}:
                    error, transform = "Tracker 身份在本次预览中发生变化；请检查角色映射。", None
                else:
                    self._tracker_identity = identity
            self._tracker = transform
            if error is not None:
                self._errors["Tracker"] = error
            else:
                self._errors.pop("Tracker", None)

    def _matched_image(self, now: float):
        for stamp, (image, received) in reversed(self._images.items()):
            host = self._timings.get(stamp)
            if host is not None and now - received <= _STALE_S:
                return stamp, image, received, host
        return None

    def _detect_latest(self) -> None:
        now = self._clock()
        with self._lock:
            match = self._matched_image(now)
            intrinsics, generation = self._intrinsics, self._generation
            if (self._stop_event.is_set() or match is None or intrinsics is None
                    or now - self._last_detection_started < _DETECTION_PERIOD_S):
                return
            stamp, image, received, host = match
            key = (stamp, host, generation)
            if key == self._last_detection_key:
                return
            self._last_detection_key = key
            self._last_detection_started = now
        try:
            board = detect_board_pose(image, encoding="rgb8", intrinsics=intrinsics,
                                      config=self.config.board, timestamp_ns=host,
                                      source_stamp_ns=stamp)
            error = None
        except Exception as caught:
            board, error = None, f"棋盘检测失败：{caught}"
        with self._lock:
            if self._stop_event.is_set() or generation != self._generation:
                return
            self._board, self._board_received = board, received
            if error is not None:
                self._errors["Board"] = error
            else:
                self._errors.pop("Board", None)

    def _run_vision(self) -> None:
        while not self._stop_event.is_set():
            self._vision_wake.wait(_DETECTION_PERIOD_S)
            self._vision_wake.clear()
            remaining = self._last_detection_started + _DETECTION_PERIOD_S - self._clock()
            if remaining > 0 and self._stop_event.wait(remaining):
                return
            self._detect_latest()

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            ages = {kind + "_age_s": _age(now, received)
                    for kind, received in self._received.items()}
            image = next(reversed(self._images.values()))[0] if self._images else None
            tracker_valid = bool(self._running and self._tracker is not None
                                 and ages["tracker_age_s"] <= _STALE_S)
            matched = self._matched_image(now)
            board_age = _age(now, self._board_received)
            board_valid = bool(self._running and self._board is not None
                               and board_age <= _STALE_S and matched is not None)
            errors = "; ".join(f"{key}: {value}" for key, value in self._errors.items())
            if not self._running:
                guidance = "预览未连接；不会启动设备或自动采集。"
            elif errors:
                guidance = "请先处理输入错误；当前仅为预览。"
            elif image is None or ages["image_age_s"] > _STALE_S:
                guidance = "等待新鲜相机图像。"
            elif self._intrinsics is None:
                guidance = "等待有效 CameraInfo。"
            elif matched is None:
                guidance = "等待与图像时间戳精确匹配的有效 CameraFrameTiming。"
            elif not tracker_valid:
                guidance = "等待所选 Tracker 的有效定位。"
            elif not board_valid:
                guidance = "请将完整棋盘放入相机视野，避免遮挡与反光。"
            else:
                guidance = "棋盘可见且输入正常；仅预览，尚未验证采集或标定精度。"
            return dict(
                image_rgb=None if image is None else image.copy(), **ages,
                image_hz=_rate(self._arrivals["image"], now),
                tracker_hz=_rate(self._arrivals["tracker"], now),
                tracker_valid=tracker_valid,
                tracker_position=self._tracker.translation.tolist() if tracker_valid else None,
                tracker_quaternion=self._tracker.quaternion_xyzw.tolist() if tracker_valid else None,
                board_corners=0 if self._board is None else self._board.corner_count,
                board_rms_px=None if self._board is None else self._board.reprojection_rms_px,
                board_valid=board_valid, board_age_s=board_age,
                board_source_stamp_ns=None if self._board is None else self._board.source_stamp_ns,
                timing_matched=bool(self._running and matched is not None),
                running=self._running, error=errors, guidance=guidance,
                quality="preview_only" if self._running else "disconnected",
                age_basis="local_subscription_arrival_monotonic",
            )
