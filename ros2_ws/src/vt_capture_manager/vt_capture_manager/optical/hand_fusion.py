"""Metric camera-frame wrist and hand shape, with an explicit physical mount.

The common matching clock is host CLOCK_REALTIME. Its meaning is camera colour
callback versus glove host receive, not simultaneous hardware exposure.
"""
from bisect import bisect_left
from collections import OrderedDict, deque
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re

from vt_capture_core.spatial.transforms import RigidTransform, finite_vector

from .identity import HANDS


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


@dataclass(frozen=True)
class Mount:
    layout_sha256: str
    cube_from_wrist: RigidTransform
    sha256: str


def read_mounts(path):
    """A per-hand mount maps model wrist/rest axes to measured cube axes."""
    with Path(path).open("rb") as stream:
        raw = stream.read(65537)
    if len(raw) > 65536:
        raise ValueError("Mount document exceeds 64 KiB")
    doc = json.loads(raw)
    if (not isinstance(doc, dict) or set(doc) != {"schema_version", "hands"}
            or type(doc["schema_version"]) is not int or doc["schema_version"] != 1
            or not isinstance(doc["hands"], dict) or not set(doc["hands"]) <= set(HANDS)):
        raise ValueError("Expected schema_version=1 and left/right hand mounts")
    result = {}
    for hand, item in doc["hands"].items():
        if (not isinstance(item, dict) or set(item) != {"layout_sha256", "cube_from_model_wrist"}
                or not isinstance(item["layout_sha256"], str)
                or not re.fullmatch("[0-9a-f]{64}", item["layout_sha256"])):
            raise ValueError(f"Invalid {hand} mount/layout identity")
        transform = RigidTransform.from_dict(item["cube_from_model_wrist"])
        binding = json.dumps([hand, item], sort_keys=True, allow_nan=False, separators=(",", ":")).encode()
        result[hand] = Mount(item["layout_sha256"], transform, hashlib.sha256(binding).hexdigest())
    return result


@dataclass(frozen=True)
class Shape:
    hand: str
    session_id: str
    config_sha256: str
    model_rest_sha256: str
    profile: int
    frame: int
    realtime: int
    monotonic: int
    root: RigidTransform
    joints: tuple

    @classmethod
    def read(cls, message, hand):
        if (message.schema_version != 1 or message.hand != hand or message.source_mode != "live"
                or not message.pose_valid or not message.time_valid or not message.session_id
                or message.header.frame_id != f"glove_{hand}_reference"
                or message.host_realtime_ns <= 0 or message.host_realtime_ns != stamp_ns(message.header)
                or message.host_monotonic_ns <= 0 or message.clock_domain != "CLOCK_MONOTONIC"
                or message.timing_source != "host_receive" or message.calibration_version <= 0
                or not re.fullmatch("[0-9a-f]{64}", message.config_sha256)
                or not re.fullmatch("[0-9a-f]{64}", message.model_rest_sha256)):
            raise ValueError("Unusable glove pose, clock, or calibration")
        points = finite_vector(message.joint_positions_m, 63, "wrist-relative joints (m)")
        # GloveState measurements use the wrist itself as origin. Reject an
        # incompatible model convention rather than applying two translations.
        if max(abs(c) for c in points[:3]) > 0.0001:
            raise ValueError("Glove geometry has a nonzero wrist origin")
        return cls(hand, message.session_id, message.config_sha256, message.model_rest_sha256,
                   message.calibration_version, message.frame_index, message.host_realtime_ns,
                   message.host_monotonic_ns, RigidTransform((0., 0., 0.), message.root_quat_xyzw), points)

    @property
    def identity(self):
        return (self.session_id, self.config_sha256, self.model_rest_sha256, self.profile)


class ShapeBuffer:
    """Bounded, time-ordered valid and invalid glove observations for one hand."""

    MAX_SAMPLES = 128
    MAX_SPAN_NS = 500_000_000
    MAX_BRACKET_NS = 40_000_000
    CLOCK_STEP_TOLERANCE_NS = 5_000_000

    def __init__(self):
        self.times, self.samples, self.arrivals = [], [], []

    def add(self, message, hand, arrival_ns=None):
        t = message.host_realtime_ns
        if t <= 0:
            # This disconnect or clock failure has no position on our timeline.
            self.times.clear()
            self.samples.clear()
            self.arrivals.clear()
            return
        try:
            sample = Shape.read(message, hand)
        except (AttributeError, TypeError, ValueError, OverflowError):
            sample = None
        pos = bisect_left(self.times, t)
        if pos < len(self.times) and self.times[pos] == t:
            if self.samples[pos] != sample:
                self.samples[pos] = None
            return
        self.times.insert(pos, t)
        self.samples.insert(pos, sample)
        self.arrivals.insert(pos, arrival_ns)
        cut = max(len(self.times) - self.MAX_SAMPLES, bisect_left(self.times, self.times[-1] - self.MAX_SPAN_NS))
        del self.times[:cut]
        del self.samples[:cut]
        del self.arrivals[:cut]

    def nearest_in_bracket(self, target, now_ns=None):
        i = bisect_left(self.times, target)
        before = self.times[i-1] if i else 0
        after = self.times[i] if i < len(self.times) else 0
        edges = (i,) if after == target else tuple(j for j in (i-1, i) if 0 <= j < len(self.times))
        if now_ns is not None and any(self.arrivals[j] is not None and
                                     not 0 <= now_ns - self.arrivals[j] <= self.MAX_SPAN_NS for j in edges):
            return "glove_arrival_expired", None, before, after
        if after == target:
            shape = self.samples[i]
            return ("tracked" if shape else "invalid_glove", shape, target, target)
        if not before or not after:
            return "glove_not_bracketed", None, before, after
        a, b = self.samples[i-1:i+1]
        if a is None or b is None:
            return "invalid_glove", None, before, after
        if a.identity != b.identity:
            return "glove_source_changed", None, before, after
        if after - before > self.MAX_BRACKET_NS:
            return "glove_gap", None, before, after
        if (a.frame >= b.frame or a.monotonic >= b.monotonic or
                abs((b.monotonic - a.monotonic) - (after - before)) > self.CLOCK_STEP_TOLERANCE_NS):
            return "glove_clock_discontinuity", None, before, after
        return "tracked", a if target - before <= after - target else b, before, after


@dataclass(frozen=True)
class FrameTime:
    serial: str
    number: int
    realtime: int


def color_time(message, camera, serial=""):
    """Map an image's exact ROS stamp to the colour callback's host clock."""
    group_flags = (message.GROUP_VALID_COMMON_STAMP | message.GROUP_VALID_IDENTITY
                   | message.GROUP_VALID_CALLBACK_CLOCKS | message.GROUP_VALID_UNIQUE)
    flags = message.VALID_HOST_REALTIME | message.VALID_FRAME_NUMBER
    if (message.camera_name != camera or not message.serial_number
            or (serial and message.serial_number != serial)
            or message.header.frame_id != camera + "_frame_group"
            or message.shared_ros_timestamp_ns != stamp_ns(message.header)
            or message.shared_ros_timestamp_ns <= 0 or message.color_host_realtime_ns <= 0
            or message.group_validity_flags & group_flags != group_flags
            or message.color_validity_flags & flags != flags):
        return None
    return FrameTime(message.serial_number, message.color_frame_number, message.color_host_realtime_ns)


@dataclass
class FusionResult:
    cube: object
    status: str
    mount: Mount | None = None
    timing: FrameTime | None = None
    wrist: RigidTransform | None = None
    shape: Shape | None = None
    joints: tuple = ()
    before: int = 0
    after: int = 0


def fuse(cube, hand, camera, mounts, timing, history, now_ns=None):
    result = FusionResult(cube, "", mount=mounts.get(hand), timing=timing)
    if (cube.schema_version != 1 or cube.hand != hand or cube.camera_name != camera
            or cube.header.frame_id != camera + "_color_optical_frame" or stamp_ns(cube.header) <= 0
            or not cube.instance_id or cube.source_image_index < 1):
        result.status = "invalid_cube_source"
    elif not cube.valid or cube.status != "tracked":
        result.status = "cube_" + cube.status if cube.status != "tracked" else "invalid_cube"
    elif result.mount is None:
        result.status = "mount_missing"
    elif cube.layout_sha256 != result.mount.layout_sha256:
        result.status = "layout_mismatch"
    elif not re.fullmatch("[0-9a-f]{64}", cube.intrinsics_sha256):
        result.status = "invalid_intrinsics"
    elif timing is None:
        result.status = "camera_timing_unavailable"
    if result.status:
        return result

    p, q = cube.pose.position, cube.pose.orientation
    try:
        camera_from_cube = RigidTransform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        if p.z <= 0:
            raise ValueError("Cube behind camera")
    except (TypeError, ValueError, OverflowError):
        result.status = "invalid_cube_pose"
        return result
    result.wrist = camera_from_cube.compose(result.mount.cube_from_wrist)
    result.status, result.shape, result.before, result.after = history.nearest_in_bracket(timing.realtime, now_ns)
    if result.shape is not None:
        # The recorded 21 points already contain this glove frame's root rotation.
        transform = result.wrist.compose(result.shape.root.inverse())
        points = result.shape.joints
        result.joints = tuple(v for i in range(0, 63, 3) for v in transform.apply(points[i:i+3]))
    return result


class FusionJoin:
    """Briefly reorder the three DDS inputs and finish each cube frame once."""

    SETTLE_NS = 60_000_000
    MAX_PENDING = 16
    TIMING_LIFETIME_NS = 750_000_000

    def __init__(self, camera, mounts, serial=""):
        self.camera, self.mounts, self.serial = camera, mounts, serial
        self.shapes = {h: ShapeBuffer() for h in HANDS}
        self.timings = OrderedDict()
        self.pending = {h: {} for h in HANDS}
        self.instance = dict.fromkeys(HANDS, "")
        self.retired = {h: deque(maxlen=16) for h in HANDS}
        self.finished = dict.fromkeys(HANDS, 0)
        self.last_timing = None

    def add_timing(self, msg, arrival_ns):
        stamp = stamp_ns(msg.header)
        value = color_time(msg, self.camera, self.serial)
        if stamp in self.timings:
            earlier, received = self.timings[stamp]
            self.timings[stamp] = (value if earlier == value else None, received)
            return
        if value is not None and self.last_timing is not None:
            previous_stamp, previous = self.last_timing
            # Modest, internally consistent reordering is normal DDS delivery.
            # Changing serial, backtracking over the wait window, reversing a
            # camera counter, or a host clock step starts a new timing epoch.
            delta = stamp - previous_stamp
            changed = (value.serial != previous.serial or delta < -self.SETTLE_NS or
                       (delta > 0 and (value.number <= previous.number or value.realtime <= previous.realtime)) or
                       (delta < 0 and (value.number >= previous.number or value.realtime >= previous.realtime)))
            if changed:
                self._break_timing_epoch()
        if value is not None and (self.last_timing is None or stamp > self.last_timing[0]):
            self.last_timing = (stamp, value)
        self.timings[stamp] = (value, arrival_ns)
        while len(self.timings) > 96:
            self.timings.popitem(last=False)

    def _break_timing_epoch(self):
        self.timings.clear()
        self.last_timing = None
        for hand, pending in self.pending.items():
            self.shapes[hand] = ShapeBuffer()
            for index, (cube, arrived, _) in pending.items():
                pending[index] = (cube, arrived, "camera_clock_discontinuity")

    def add_cube(self, hand, cube, arrival_ns):
        """Return terminal failures for frames displaced by a restart or overflow."""
        queue = self.pending[hand]
        index, source = cube.source_image_index, cube.instance_id
        if not source or index < 1:
            return [FusionResult(cube, "invalid_cube_source")]
        if source in self.retired[hand]:
            return []
        discarded = []
        if self.instance[hand] != source:
            if self.instance[hand]:
                self.retired[hand].append(self.instance[hand])
            discarded = [FusionResult(row[0], "cube_source_changed") for row in queue.values()]
            queue.clear()
            self.instance[hand], self.finished[hand] = source, 0
        if index <= self.finished[hand]:
            return discarded
        if index in queue:
            old, arrival, status = queue[index]
            if old != cube:
                queue[index] = (old, arrival, "cube_frame_conflict")
            return discarded
        queue[index] = (cube, arrival_ns, "")
        if len(queue) > self.MAX_PENDING:
            oldest = min(queue)
            discarded.append(FusionResult(queue.pop(oldest)[0], "fusion_queue_overflow"))
            self.finished[hand] = oldest
        return discarded

    def drain(self, now_ns):
        results = []
        for hand, queue in self.pending.items():
            for index in sorted(queue):
                cube, arrival, status = queue[index]
                if now_ns - arrival < self.SETTLE_NS:
                    break
                del queue[index]
                self.finished[hand] = index
                timing, time_arrived = self.timings.get(stamp_ns(cube.header), (None, 0))
                if now_ns - time_arrived > self.TIMING_LIFETIME_NS:
                    timing = None
                if status:
                    results.append(FusionResult(cube, status))
                elif now_ns - arrival > self.TIMING_LIFETIME_NS:
                    results.append(FusionResult(cube, "processing_late"))
                else:
                    results.append(fuse(cube, hand, self.camera, self.mounts, timing, self.shapes[hand], now_ns))
        return results
