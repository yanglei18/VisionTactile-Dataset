"""Read-only adapter for VIVE Hub's bundled vut::Client SDK.

No pyvut import, USB access, initialization or device control. The external C++
helper uses the vendor SDK and streams bounded NDJSON over a private pipe.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
import threading
import time
from pathlib import Path

import yaml

from .model import StampedTrackerSample
from .roles import EXPECTED_ROLES, RoleMap


FRAME_ID = "vive_hub_map"
CONVENTION = "vivehub_openvr_xyzw_v1"
CLOCK_POLICY = "anchored_linux_realtime_monotonic_v2"
MAX_CLOCK_PAIR_SPAN_NS = 100_000
MAX_CLOCK_OFFSET_ERROR_NS = 5_000_000


def load_bindings(path):
    """Explicit physical roles by serial, never by transient dongle slot."""
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if (type(doc) is not dict or set(doc) != {"schema_version", "roles"}
            or doc["schema_version"] != 1 or type(doc["roles"]) is not dict
            or set(doc["roles"]) != EXPECTED_ROLES):
        raise ValueError("Hub bindings require schema_version=1 and all three roles")
    serials = doc["roles"]
    if any(type(v) is not str or not re.fullmatch(r"[A-Za-z0-9_-]{1,31}", v) for v in serials.values()):
        raise ValueError("Hub roles must contain explicit tracker serial numbers")
    if len(set(serials.values())) != 3:
        raise ValueError("Hub tracker serials must be unique")
    ids = {role: hashlib.sha256(("vivehub_serial_v1:" + serial).encode("ascii")).hexdigest()
           for role, serial in serials.items()}
    return {serial: role for role, serial in serials.items()}, RoleMap(ids, {v: k for k, v in ids.items()})


def source_info():
    return dict(backend="vivehub", convention=CONVENTION, frame_id=FRAME_ID,
                identity="sha256(vivehub_serial_v1:SERIAL)", quaternion="xyzw", position_units="m",
                monotonic_clock="CLOCK_MONOTONIC (daemon pose timestamp; not MONOTONIC_RAW)",
                realtime_clock="source monotonic + one shared, fixed REALTIME-MONOTONIC anchor",
                clock_policy=CLOCK_POLICY, clock_pair_max_span_ns=MAX_CLOCK_PAIR_SPAN_NS,
                clock_offset_max_error_ns=MAX_CLOCK_OFFSET_ERROR_NS,
                clock_discontinuity="latched rejection until explicit software bridge restart; no automatic re-anchor",
                timestamp_scope="daemon timestamp, not a verified sensor exposure timestamp",
                ready_states=[2, 18], ready_18="v1.0.3 observed raw 0x12 while Hub reports Ready",
                unavailable_fields=["linear_acceleration", "raw_buttons"],
                unavailable_fill=0, packet_index="bridge counter per role modulo 65536",
                angular_velocity="SDK forwarded; zero may mean unavailable",
                extrinsics="old pyvut extrinsics are NOT automatically compatible")


class HubClockError(ValueError):
    """A confirmed clock-domain change, not an ordinary invalid pose."""


class AnchoredHostClock:
    """One Linux host, one constant clock-domain offset for all Tracker roles.

    CLOCK_MONOTONIC and CLOCK_REALTIME share frequency adjustments on Linux.
    Pair-reading jitter must not become per-pose timestamp jitter. A bounded
    bracket establishes the offset once; subsequent accurate pairs only check
    it. Suspend/wall-clock steps are not silently absorbed into a new anchor.
    """

    def __init__(self):
        self.offset_ns = None
        self.fault = None

    def map(self, stamp, receive_monotonic_ns, receive_realtime_ns, pair_span_ns):
        if self.fault:
            raise HubClockError(self.fault)
        if type(pair_span_ns) is not int or not 0 <= pair_span_ns <= MAX_CLOCK_PAIR_SPAN_NS:
            raise ValueError('Hub clock pair missing or sampling span exceeds 100 us; pose rejected')
        observed_offset = receive_realtime_ns - receive_monotonic_ns
        if self.offset_ns is None:
            self.offset_ns = observed_offset
        elif abs(observed_offset - self.offset_ns) > MAX_CLOCK_OFFSET_ERROR_NS:
            self.fault = ('Hub host clock offset changed by more than 5 ms; '
                          'poses stopped. Restart the software bridge before a new calibration round')
            raise HubClockError(self.fault)
        mapped = stamp + self.offset_ns
        if mapped <= 0:
            raise ValueError('Invalid mapped Hub realtime timestamp')
        return mapped


class HubDecoder:
    def __init__(self, serial_roles, role_map, *, max_age_ns=500_000_000):
        self.serial_roles, self.role_map = serial_roles, role_map
        self.max_age_ns = max_age_ns
        self.last_stamp = {}
        self.counters = {}
        self.clock = AnchoredHostClock()

    def decode(self, event, now_ns):
        if event.get("type") != "pose":
            return None
        role = self.serial_roles.get(event.get("serial"))
        if role is None:
            return None
        stamp, recv, real = (event[k] for k in ("timestamp_ns", "receive_monotonic_ns", "receive_realtime_ns"))
        if any(type(v) is not int or v <= 0 for v in (stamp, recv, real)):
            raise ValueError("invalid Hub clock fields")
        if not 0 <= now_ns - stamp <= self.max_age_ns or not 0 <= recv - stamp <= self.max_age_ns:
            return None  # Never replay buffered poses after a stall/reconnect.
        if stamp <= self.last_stamp.get(role, 0):
            return None

        def vector(key, count):
            values = event[key]
            if not isinstance(values, list) or len(values) != count or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise ValueError(f"invalid Hub {key}")
            return tuple(float(v) for v in values)

        position = vector("position", 3)
        q = vector("orientation", 4)  # Already xyzw: NEVER reverse as pyvut wzyx.
        angular = vector("angular_velocity", 3)
        norm = math.sqrt(sum(v * v for v in q))
        if not 0.9 <= norm <= 1.1:
            raise ValueError("invalid Hub quaternion norm")
        state = event["state"]
        if type(state) is not int or not 0 <= state <= 255:
            raise ValueError("invalid Hub state")
        mapped_realtime = self.clock.map(stamp, recv, real, event.get('receive_clock_span_ns'))
        index = self.counters.get(role, 0)
        sample = StampedTrackerSample(
            role, self.role_map.by_role[role], stamp, mapped_realtime,
            index, state, 0, state in (2, 18), position,
            tuple(v / norm for v in q), (0.0, 0.0, 0.0), angular)
        self.last_stamp[role] = stamp
        self.counters[role] = (index + 1) & 0xFFFF
        return sample


class HubPoseSource:
    def __init__(self, bindings_path, reader_path, socket_path, *, monotonic_ns=time.monotonic_ns):
        serial_roles, role_map = load_bindings(bindings_path)
        self.decoder = HubDecoder(serial_roles, role_map)
        self.reader_path = str(Path(reader_path).resolve(strict=True))
        self.socket_path = str(socket_path)
        self.monotonic_ns = monotonic_ns
        self.process = self.thread = None
        self.stopping = threading.Event()

    def start(self, on_sample, on_invalid, on_error):
        if self.process is not None:
            raise RuntimeError("Hub source already started")
        self.process = subprocess.Popen([self.reader_path, self.socket_path],
                                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        text=True, encoding="utf-8", bufsize=1)

        def receive():
            last_error = -10_000_000_000
            try:
                while not self.stopping.is_set():
                    line = self.process.stdout.readline(8193)
                    if not line:
                        if not self.stopping.is_set():
                            on_error(RuntimeError("VIVE Hub SDK reader exited; no poses will be replayed"))
                        break
                    now = self.monotonic_ns()
                    try:
                        if len(line) > 8192 or not line.endswith("\n"):
                            raise ValueError("oversized SDK event")
                        event = json.loads(line)
                        sample = self.decoder.decode(event, now)
                        if sample is not None:
                            on_sample(sample)
                    except (ValueError, KeyError, TypeError, AttributeError) as exc:
                        # Do not advance health clocks with receive time: the next
                        # source timestamp may be slightly earlier than that.
                        if now - last_error > 5_000_000_000:
                            on_error(ValueError(f"Rejected Hub event: {exc}"))
                            last_error = now
            finally:
                self.process.stdout.close()

        self.thread = threading.Thread(target=receive, name="vivehub-sdk-source", daemon=True)
        self.thread.start()

    def stop(self):
        self.stopping.set()
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            if self.thread is not None:
                self.thread.join(timeout=3)
