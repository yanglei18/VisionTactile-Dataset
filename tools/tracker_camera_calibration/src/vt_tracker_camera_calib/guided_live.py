"""ROS input adapter for guided sampling; independent from any GUI event loop."""
from __future__ import annotations

import base64
from collections import OrderedDict
from dataclasses import replace
import threading
import time
import uuid

import cv2

from .bag_reader import intrinsics_from_message, same_intrinsics
from .charuco import detect_board_pose
from .clock import HostClockContinuity
from .guided_quality import IncrementalSampler, StableGuidance, collection_target, guidance
from .live_inputs import image_rgb, load_ros, timing_pair, tracker_pose
from .model import TimedTransform


class GuidedMonitor:
    """Independent ROS and vision threads, with bounded latest-value caches.

    All data-domain errors are sticky for this round. A later good frame cannot
    erase an identity change, intrinsic change, clock step, or timing conflict.
    """
    def __init__(self, config, expected_tracker_id=None):
        self.config = config
        self.tracker_id = expected_tracker_id or None
        self.error = ''
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._started = threading.Event()
        self._thread = None
        self._vision_thread = None
        self._vision_wake = threading.Event()
        self._detection_lock = threading.Lock()
        self._images = OrderedDict()
        self._timings = OrderedDict()
        self._intrinsics = config.fallback_intrinsics
        self._sampler = IncrementalSampler(config.pairing)
        self._guidance = StableGuidance()
        self._camera_clock = HostClockContinuity('CameraFrameTiming')
        self._tracker_clock = HostClockContinuity('TrackerSample')
        self._arrived = {}
        self._last_detection = -1
        self._last_detection_started = -float('inf')
        self._board_at = None
        self._preview = None
        self._preview_at = 0.

    def _fail(self, error):
        self.error = self.error or str(error)

    def on_image(self, message):
        try:
            image, stamp = image_rgb(message, self.config)
            with self._lock:
                self._images[stamp] = (image, time.monotonic())
                while len(self._images) > 8:
                    self._images.popitem(last=False)
                self._arrived['image'] = time.monotonic()
        except Exception as error:
            with self._lock:
                self._fail(error)

    def on_info(self, message):
        try:
            if str(message.header.frame_id) != self.config.camera_frame:
                raise ValueError('CameraInfo frame mismatch')
            candidate = intrinsics_from_message(message)
            with self._lock:
                if self._intrinsics is not None and not same_intrinsics(candidate, self._intrinsics):
                    raise ValueError('CameraInfo changed within calibration round')
                self._intrinsics = candidate
                self._arrived['info'] = time.monotonic()
        except Exception as error:
            with self._lock:
                self._fail(error)

    def on_timing(self, message):
        try:
            stamp, host = timing_pair(message, self.config)
            monotonic = int(message.group_host_monotonic_raw_ns)
            with self._lock:
                value = host, monotonic
                if stamp in self._timings:
                    if self._timings[stamp] != value:
                        raise ValueError('conflicting timing observations for one image stamp')
                else:
                    self._camera_clock.observe(*value)
                    self._timings[stamp] = value
                while len(self._timings) > 128:
                    self._timings.popitem(last=False)
                self._arrived['timing'] = time.monotonic()
        except Exception as error:
            with self._lock:
                self._fail(error)

    def on_tracker(self, message):
        try:
            identity, pose = tracker_pose(message, self.config)
            host, monotonic = int(message.host_realtime_ns), int(message.host_monotonic_ns)
            with self._lock:
                self._tracker_clock.observe(host, monotonic)
                if pose is not None:
                    if self.tracker_id not in {None, identity}:
                        raise ValueError('Tracker identity changed or differs from selected identity')
                    self.tracker_id = identity
                    self._sampler.add_tracker(TimedTransform(host, pose, monotonic))
                    self._sampler.advance()
                    self._arrived['tracker'] = time.monotonic()
        except Exception as error:
            with self._lock:
                self._fail(error)

    def start(self):
        if self._thread is not None:
            raise RuntimeError('Monitor instances are single-use')
        ros = load_ros()
        self._thread = threading.Thread(target=self._run_ros, args=(ros,), daemon=True,
                                        name='calibration-guided-ros')
        self._thread.start()
        if not self._started.wait(5):
            self.stop()
            raise RuntimeError('Guided ROS initialization timed out')
        if self.error:
            self.stop()
            raise RuntimeError(self.error)
        self._vision_thread = threading.Thread(target=self._run_vision, daemon=True,
                                               name='calibration-guided-vision')
        self._vision_thread.start()

    def request_detection(self):
        """Coalesce requests; never wait for native vision code on the owner loop."""
        self._vision_wake.set()

    def _run_vision(self):
        while not self._stop.is_set():
            self._vision_wake.wait(.2)
            self._vision_wake.clear()
            if not self._stop.is_set():
                self.detect_latest()

    def _run_ros(self, ros):
        context = node = executor = None
        try:
            context = ros.Context()
            context.init(args=[])
            node = ros.Node('guided_calibration_' + uuid.uuid4().hex[:8], context=context,
                            enable_rosout=False, start_parameter_services=False)
            executor = ros.SingleThreadedExecutor(context=context)
            executor.add_node(node)
            # The sampler needs Tracker history; a depth-one latest-value reader
            # can discard the endpoints of a stability window during detection.
            qos = ros.QoSProfile(depth=256, history=ros.HistoryPolicy.KEEP_LAST,
                                 reliability=ros.ReliabilityPolicy.BEST_EFFORT,
                                 durability=ros.DurabilityPolicy.VOLATILE)
            image_qos = ros.QoSProfile(depth=2, history=ros.HistoryPolicy.KEEP_LAST,
                                       reliability=ros.ReliabilityPolicy.BEST_EFFORT,
                                       durability=ros.DurabilityPolicy.VOLATILE)
            for kind, topic, callback in (
                (ros.Image, self.config.image_topic, self.on_image),
                (ros.CameraInfo, self.config.camera_info_topic, self.on_info),
                (ros.CameraFrameTiming, self.config.timing_topic, self.on_timing),
                (ros.TrackerSample, self.config.tracker_sample_topic, self.on_tracker),
            ):
                node.create_subscription(kind, topic, callback, image_qos if kind is ros.Image else qos)
            self._started.set()
            while not self._stop.is_set() and context.ok():
                executor.spin_once(timeout_sec=.05)
        except Exception as error:
            with self._lock:
                self._fail(error)
        finally:
            self._started.set()
            if executor is not None:
                executor.shutdown(timeout_sec=.5)
            if node is not None:
                node.destroy_node()
            if context is not None:
                context.try_shutdown()

    def stop(self, timeout_s: float = 3.):
        self._stop.set()
        self._vision_wake.set()
        deadline = time.monotonic() + timeout_s
        for thread in (self._thread, self._vision_thread):
            if thread is not None:
                thread.join(max(0., deadline - time.monotonic()))
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError('Guided ROS worker did not stop')
        if self._vision_thread is not None and self._vision_thread.is_alive():
            raise RuntimeError('Guided vision worker did not stop; Recorder has been closed')

    def detect_latest(self):
        """Run at most one detection; normally invoked only by the vision thread."""
        if not self._detection_lock.acquire(blocking=False):
            return
        try:
            self._detect_latest()
        finally:
            self._detection_lock.release()

    def _detect_latest(self):
        now = time.monotonic()
        with self._lock:
            if self._stop.is_set() or now - self._last_detection_started < .2 or not self._images or self.error:
                return
            stamp, (image, arrived) = next(reversed(self._images.items()))
            if now - arrived > 1 or stamp == self._last_detection:
                return
            self._last_detection_started = now
            clocks, intrinsics = self._timings.get(stamp), self._intrinsics
            # Wait for an exact timing join instead of permanently consuming a
            # frame whose timing callback has not yet arrived.
            if clocks is None or intrinsics is None:
                matched = [(key, value) for key, value in self._images.items()
                           if key in self._timings and key > self._last_detection]
                if matched and intrinsics is not None:
                    stamp, (image, arrived) = matched[-1]
                    clocks = self._timings[stamp]
            self._last_detection = stamp if clocks is not None and intrinsics is not None else self._last_detection
        try:
            board = None
            if clocks is not None and intrinsics is not None:
                board = detect_board_pose(image, encoding='rgb8', intrinsics=intrinsics,
                    config=self.config.board, timestamp_ns=clocks[0], source_stamp_ns=stamp)
            rendered = image.copy()
            if board is not None:
                board = replace(board, host_monotonic_raw_ns=clocks[1])
                # Project physical board corners using this frame's pose. This
                # overlay never combines a cached detection with a later image.
                from .charuco import board_corner_points, create_board
                rvec, _ = cv2.Rodrigues(board.camera_from_board.rotation)
                points, _ = cv2.projectPoints(board_corner_points(create_board(self.config.board)),
                    rvec, board.camera_from_board.translation, intrinsics.camera_matrix, intrinsics.distortion)
                for x, y in points.reshape(-1, 2):
                    if 0 <= x < rendered.shape[1] and 0 <= y < rendered.shape[0]:
                        cv2.circle(rendered, (round(float(x)), round(float(y))), 3, (80, 220, 100), 1)
            scale = min(1., 800 / rendered.shape[1], 600 / rendered.shape[0])
            if scale < 1:
                rendered = cv2.resize(rendered, None, fx=scale, fy=scale)
            success, jpeg = cv2.imencode('.jpg', cv2.cvtColor(rendered, cv2.COLOR_RGB2BGR),
                                         [cv2.IMWRITE_JPEG_QUALITY, 65])
            with self._lock:
                if self._stop.is_set():
                    return
                if board is not None:
                    self._sampler.add_observation(board)
                    self._sampler.advance()
                    self._board_at = arrived
                else:
                    self._board_at = None
                if success and len(jpeg) < 400_000:
                    self._preview = {'jpeg_base64': base64.b64encode(jpeg).decode('ascii'),
                                     'source_stamp_ns': stamp}
                    self._preview_at = arrived
        except Exception as error:
            with self._lock:
                self._fail(error)

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            pairs = self._sampler.accepted
            ready, hint, metrics = guidance(pairs, self.config.acceptance)
            fresh = all(key in self._arrived and now - self._arrived[key] <= 1
                        for key in ('image', 'timing', 'tracker')) and self._intrinsics is not None
            if not fresh:
                hint = 'WAITING_INPUT'
            elif self._board_at is None or now - self._board_at > 1:
                hint = 'SHOW_BOARD'
            elif self._sampler.hint_code == 'HOLD_STILL':
                hint = 'HOLD_STILL'
            metrics.update(tracker_id=self.tracker_id)
            return {'accepted': len(pairs), 'target': collection_target(self.config.acceptance),
                    'ready': ready and fresh and not self.error, 'hint_code': self._guidance.update(hint, now),
                    'detail': self.error, 'metrics': metrics,
                    'preview': self._preview if now - self._preview_at <= 1 else None}
