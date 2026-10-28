"""Independent, manual-only raw IMU input and incremental six-axis VQF.

The observed wire layout has six 14-byte blocks per 84-byte frame. ID order
helps recover framing; the uninterpreted status byte is NOT a verified checksum
or device timestamp. This module never imports the legacy glove application,
writes device commands, automatically connects, or automatically reconnects.
"""
from __future__ import annotations

import csv
from copy import deepcopy
from dataclasses import dataclass
import math
from numbers import Integral, Real
import os
from pathlib import Path
import struct
import sys
import threading
import time

import numpy as np

from . import host_clock
from .replay import _configuration, _node_key, _record_gap, _WARNINGS
from .sensor_quality import GyroNearRailMonitor, quality_warnings
from .input_contract import effective_input_scales, raw_to_si, validate_input_contract
from .input_diagnostics import InputObservationMonitor, describe_filter_state
from .input_guard import ImuInputGuard, REVISION as INPUT_GUARD_REVISION
from .input_health import InputHealthMonitor
from .vqf_settings import vqf_parameters

FRAME_SIZE = 84
NODE_BLOCK_SIZE = 14
MAX_READ_BYTES = 4096
CSV_FIELDS = ("host_parse_monotonic_s", "host_parse_wall_s", "node_id",
              "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb", "status")


def _timestamp(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be finite")
    value = float(value)
    if not math.isfinite(value) or (name == "host_monotonic_s" and value < 0):
        raise ValueError(f"{name} must be finite and monotonic time nonnegative")
    return value


def _timestamp_ns(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral) or (name == "host_monotonic_ns" and value < 0):
        raise ValueError(f"{name} must be integer nanoseconds and monotonic time nonnegative")
    return int(value)


def _timestamp_pair(seconds, nanoseconds, seconds_name, ns_name, clock_ns):
    # Float-only legacy inputs deliberately retain unknown nanosecond precision.
    if seconds is None and nanoseconds is None:
        nanoseconds = clock_ns()
    if nanoseconds is not None:
        nanoseconds = _timestamp_ns(nanoseconds, ns_name)
        derived_seconds = _timestamp(nanoseconds / 1_000_000_000, seconds_name)
        if seconds is None:
            seconds = derived_seconds
        elif _timestamp(seconds, seconds_name) != derived_seconds:
            raise ValueError(f"{seconds_name} and {ns_name} must describe the same timestamp")
    return _timestamp(seconds, seconds_name), nanoseconds


@dataclass(frozen=True)
class RawSample:
    """One observed node with host receive time, never a device timestamp.

    Optional integer clocks preserve live source precision. Legacy float-only
    inputs leave these fields None; converting their floats cannot recover it.
    status=None preserves an unknown legacy CSV field.
    """
    node_id: int
    values: tuple[int, int, int, int, int, int]
    host_monotonic_s: float
    host_wall_s: float
    status: int | None = 0
    host_monotonic_ns: int | None = None
    host_realtime_ns: int | None = None

    def __post_init__(self):
        if isinstance(self.node_id, bool) or not isinstance(self.node_id, Integral) or not 1 <= self.node_id <= 12:
            raise ValueError("node_id must be an integer in 1..12")
        try:
            values = tuple(self.values)
        except TypeError as exc:
            raise ValueError("values must contain six int16 values") from exc
        if len(values) != 6 or any(isinstance(x, bool) or not isinstance(x, Integral) or not -32768 <= x <= 32767 for x in values):
            raise ValueError("values must contain six int16 values")
        if self.status is not None and (isinstance(self.status, bool) or not isinstance(self.status, Integral) or not 0 <= self.status <= 255):
            raise ValueError("status must be an unsigned byte or None")
        object.__setattr__(self, "node_id", int(self.node_id))
        object.__setattr__(self, "values", tuple(int(x) for x in values))
        object.__setattr__(self, "host_monotonic_s", _timestamp(self.host_monotonic_s, "host_monotonic_s"))
        object.__setattr__(self, "host_wall_s", _timestamp(self.host_wall_s, "host_wall_s"))
        for seconds_name, ns_name in (("host_monotonic_s", "host_monotonic_ns"),
                                      ("host_wall_s", "host_realtime_ns")):
            value = getattr(self, ns_name)
            if value is not None:
                value = _timestamp_ns(value, ns_name)
                if getattr(self, seconds_name) != value / 1_000_000_000:
                    raise ValueError(f"{seconds_name} and {ns_name} must describe the same timestamp")
                object.__setattr__(self, ns_name, value)
        if self.status is not None:
            object.__setattr__(self, "status", int(self.status))


class ImuParser:
    """Bounded rolling synchronization; every decoded raw row is retained."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.buffer = bytearray()
        self.frames_ok = 0
        self.bytes_discarded = 0

    def feed(self, data, host_monotonic_s=None, host_wall_s=None, *,
             host_monotonic_ns=None, host_realtime_ns=None):
        stamp, monotonic_ns = _timestamp_pair(host_monotonic_s, host_monotonic_ns,
                                             "host_monotonic_s", "host_monotonic_ns", host_clock.monotonic_ns)
        wall, realtime_ns = _timestamp_pair(host_wall_s, host_realtime_ns,
                                           "host_wall_s", "host_realtime_ns", time.time_ns)
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("data must be bytes-like")
        view = memoryview(data).cast("B")
        samples = []
        for start_chunk in range(0, len(view), MAX_READ_BYTES):
            self.buffer.extend(view[start_chunk:start_chunk + MAX_READ_BYTES])
            cursor, last_start = 0, len(self.buffer) - FRAME_SIZE
            while cursor <= last_start:
                starts = [p for p in (self.buffer.find(b"\x01", cursor, last_start + 1),
                                      self.buffer.find(b"\x07", cursor, last_start + 1)) if p >= 0]
                if not starts:
                    self.bytes_discarded += last_start + 1 - cursor
                    cursor = last_start + 1
                    break
                start = min(starts)
                self.bytes_discarded += start - cursor
                first = self.buffer[start]
                if not all(self.buffer[start + i * NODE_BLOCK_SIZE] == first + i for i in range(6)):
                    self.bytes_discarded += 1
                    cursor = start + 1
                    continue
                for i in range(6):
                    offset = start + i * NODE_BLOCK_SIZE
                    samples.append(RawSample(first + i, struct.unpack_from("<6h", self.buffer, offset + 2),
                                             stamp, wall, self.buffer[offset + 1], monotonic_ns, realtime_ns))
                self.frames_ok += 1
                cursor = start + FRAME_SIZE
            if cursor:
                del self.buffer[:cursor]
        return samples


class StreamError(RuntimeError):
    """Source/dependency failure suitable for displaying without reconnecting."""


def _serial_module():
    try:
        import serial
    except ImportError as exc:
        raise StreamError("Live serial input requires pyserial; offline CSV/VQF remains available") from exc
    return serial


def list_serial_ports():
    """Return (device, description) choices; never select or open a device."""
    _serial_module()
    from serial.tools import list_ports
    return sorted((item.device, item.description or "") for item in list_ports.comports())


def _find_serial_port_holders(port, *, proc_root=Path("/proc")):
    """Best-effort visible Linux FD check; permissions and open races remain."""
    if not sys.platform.startswith("linux"):
        return []
    target, holders = os.path.realpath(port), []
    try:
        processes = list(Path(proc_root).iterdir())
    except OSError:
        return holders
    for process in processes:
        if not process.name.isdecimal():
            continue
        try:
            descriptors = list((process / "fd").iterdir())
        except OSError:
            continue
        for descriptor in descriptors:
            if not descriptor.name.isdecimal():
                continue
            try:
                link = os.readlink(descriptor)
                if link.startswith("/") and os.path.realpath(link) == target:
                    holders.append((int(process.name), int(descriptor.name)))
            except OSError:
                continue
    return sorted(holders)


def _claim_tty_exclusive(port):
    if os.name != "posix":
        return False
    import fcntl
    import termios
    command = getattr(termios, "TIOCEXCL", None)
    if command is None:
        return False
    fcntl.ioctl(port.fileno(), command)
    return True


def _release_tty_exclusive(port):
    import fcntl
    import termios
    command = getattr(termios, "TIOCNXCL", None)
    if command is not None:
        try:
            fcntl.ioctl(port.fileno(), command)
        except OSError:
            pass


class SerialSource:
    """Explicit open/read/close, with no writes or reconnect policy."""
    def __init__(self, port, baud=115200):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("an explicit serial port is required")
        if isinstance(baud, bool) or not isinstance(baud, Integral) or baud <= 0:
            raise ValueError("baud must be a positive integer")
        self.port, self.baud = port, int(baud)
        self.parser = ImuParser()
        self._serial = None
        self._tty_exclusive = False
        self._diagnostics_lock = threading.Lock()
        self._opened = False
        self._bytes_received = self._samples_decoded = 0
        self._last_byte = self._last_sample = None

    def open(self):
        if self._serial is not None:
            return self
        holders = _find_serial_port_holders(self.port)
        if holders:
            owners = ", ".join(f"PID {pid} (fd {fd})" for pid, fd in holders)
            raise StreamError(f"Serial port is already held by {owners}; close its reader first. No process was changed.")
        serial = _serial_module()
        kwargs = dict(timeout=.020, write_timeout=.2)
        if os.name == "posix":
            kwargs["exclusive"] = True
        try:
            self._serial = serial.Serial(self.port, self.baud, **kwargs)
            self._tty_exclusive = _claim_tty_exclusive(self._serial)
        except (OSError, serial.SerialException) as exc:
            self.close()
            raise StreamError(f"Cannot open serial port {self.port!r}: {exc}") from exc
        with self._diagnostics_lock:
            self.parser.reset()
            self._bytes_received = self._samples_decoded = 0
            self._last_byte = self._last_sample = None
            self._opened = True
        return self

    def read_samples(self):
        if self._serial is None:
            raise StreamError("SerialSource is not open; connect explicitly")
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(min(MAX_READ_BYTES, max(1, waiting)))
            # Capture the two host clocks after read completion, before waiting
            # for the diagnostics lock. These are not simultaneous IMU times.
            monotonic_ns, realtime_ns = host_clock.monotonic_ns(), time.time_ns()
        except (OSError, _serial_module().SerialException) as exc:
            raise StreamError(f"Serial read failed for {self.port!r}: {exc}") from exc
        # The sole reader performs all I/O outside the snapshot/parse lock.
        # read() is capped at 4 KiB, keeping the locked parser work bounded.
        with self._diagnostics_lock:
            stamp = monotonic_ns / 1_000_000_000
            self._bytes_received += len(data)
            if data:
                self._last_byte = stamp
            samples = self.parser.feed(data, host_monotonic_ns=monotonic_ns,
                                       host_realtime_ns=realtime_ns)
            self._samples_decoded += len(samples)
            if samples:
                self._last_sample = samples[-1].host_monotonic_s
            return samples

    def snapshot_diagnostics(self, now=None):
        """Copy passive reader counters; decoded framing is not CRC validation.

        No serial properties are queried and no I/O is performed. Ages are in
        the host monotonic clock domain; None means no corresponding observation.
        Counts and ages remain available after close until the next real open.
        """
        with self._diagnostics_lock:
            stamp = _timestamp(host_clock.monotonic() if now is None else now, "host_monotonic_s")
            return dict(source_kind="raw_serial", opened=self._opened,
                        bytes_received=self._bytes_received,
                        frames_decoded=self.parser.frames_ok,
                        samples_decoded=self._samples_decoded,
                        bytes_discarded=self.parser.bytes_discarded,
                        buffered_bytes=len(self.parser.buffer),
                        last_byte_age_s=None if self._last_byte is None else max(0., stamp - self._last_byte),
                        last_sample_age_s=None if self._last_sample is None else max(0., stamp - self._last_sample))

    def close(self):
        opened, self._serial = self._serial, None
        with self._diagnostics_lock:
            self._opened = False
        try:
            if opened is not None:
                try:
                    if self._tty_exclusive:
                        _release_tty_exclusive(opened)
                finally:
                    opened.close()
        finally:
            self._tty_exclusive = False


def iter_csv_samples(path):
    """Read legacy CSV without reordering, deduplicating or hiding clock resets."""
    with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream, strict=True)
        required = set(CSV_FIELDS[:-1])
        try:
            fields = reader.fieldnames
            if (fields is None or len(fields) != len(set(fields)) or not required <= set(fields)
                    or set(fields) - required - {"status", "phase"}):
                raise ValueError("CSV requires timestamps, node_id, six raw IMU fields, optional status/phase")
            for row in reader:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("wrong CSV field count")
                text = row.get("status", "").strip()
                yield RawSample(_node_key(row["node_id"]), tuple(int(row[key]) for key in CSV_FIELDS[3:9]),
                                float(row[CSV_FIELDS[0]]), float(row[CSV_FIELDS[1]]), int(text) if text else None)
        except (ValueError, TypeError, csv.Error) as exc:
            raise ValueError(f"invalid raw CSV at line {reader.line_num}: {exc}") from exc


class RawRecorder:
    """Explicit exclusive CSV output, preserving every supplied raw value."""
    def __init__(self, path):
        self.path = Path(path)
        self._stream = self._writer = None
        self._closed = False

    def open(self):
        if self._closed:
            raise RuntimeError("RawRecorder is closed")
        if self._stream is None:
            self._stream = self.path.open("x", encoding="utf-8", newline="")
            try:
                self._writer = csv.writer(self._stream)
                self._writer.writerow(CSV_FIELDS)
                self._stream.flush()
            except BaseException:
                self.close()
                raise
        return self

    def write(self, samples):
        if self._writer is None:
            raise RuntimeError("RawRecorder is not open")
        for sample in samples:
            if not isinstance(sample, RawSample):
                raise TypeError("RawRecorder expects RawSample instances")
            self._writer.writerow((sample.host_monotonic_s, sample.host_wall_s, sample.node_id,
                                   *sample.values, "" if sample.status is None else sample.status))
        self._stream.flush()

    def close(self):
        try:
            if self._stream is not None:
                self._stream.close()
        finally:
            self._stream = self._writer = None
            self._closed = True


class OnlineFusion:
    """One independent VQF per node; snapshot never advances/resets a filter.

    Only gaps above filter_reset_gap_s reset the returning node; shorter gaps
    preserve its VQF and nominal-sample integration while reporting uncertainty.
    Backward input host time resets all epochs as in offline replay. UI snapshots omit stale or
    future-dated states, including while the device is silent. Frame indices
    count emitted snapshots; sample indices count actual raw node samples.
    Explicit calibration reset discards fusion history without resetting these
    recording-wide indices or the independent input diagnostics.
    """
    def __init__(self, config):
        from vqf import VQF
        validate_input_contract(config)
        self._factory = VQF
        self._vqf_parameters = vqf_parameters(config)
        self.settings, self.scales = _configuration(config)
        self.scales, self.accel_scales = effective_input_scales(config)
        self.period = 1. / self.settings["sample_rate_hz"]
        self.states = {}
        self.epochs = {str(n): 0 for n in range(1, 13)}
        self.counts = {str(n): 0 for n in range(1, 13)}
        self.last_input = None
        self.frame_index = 0
        self._pending_warnings = set()
        self.gap_diagnostics = {}
        self.sensor_quality = GyroNearRailMonitor()
        self.input_observation = InputObservationMonitor(self.settings["sample_rate_hz"])
        self.input_guard = ImuInputGuard(config)
        self.input_guard_rejections = 0
        self.input_health = InputHealthMonitor(config)

    def reset_for_calibration(self):
        """Forget algorithm state in-place without reopening or altering input.

        The owner must hold its session lock across this call and any update or
        snapshot. Previously observed nodes advance their epoch, including
        nodes currently absent after a prior reset; unseen nodes keep theirs.
        VQF instances, quaternions, cached SI inputs and all guard baselines,
        candidates, rejection runs and per-epoch guard counters are discarded.
        Filters are recreated lazily on the next sample, so an intervening
        snapshot has no old or invented fresh orientation.

        Counts/sample indices, frame index, session-wide rejection count,
        input clock, passive observations, boundary history and gap history
        remain intact. Keeping last_input also preserves real clock-rollback
        detection. No source, parser or raw recorder is owned or touched here.
        The caller must invalidate its own cached pose/calibration snapshots.
        Return the affected node IDs and new epochs for the caller's event log.
        """
        nodes = [key for key in self.epochs if self.counts[key] or key in self.states]
        for key in nodes:
            self.epochs[key] += 1
        self.states.clear()
        self.input_guard.reset()
        self._pending_warnings.add("calibration_reset_filters_reinitialized")
        return dict(reset_nodes=nodes, epochs={key: self.epochs[key] for key in nodes})

    def update(self, samples):
        for sample in samples:
            if not isinstance(sample, RawSample):
                raise TypeError("OnlineFusion expects RawSample instances")
            key, stamp = str(sample.node_id), sample.host_monotonic_s
            gyro, accel = raw_to_si(sample.values, gyro_dps_per_lsb=self.scales[key],
                                   accel_g_per_lsb=self.accel_scales[key])
            if self.last_input is not None and stamp < self.last_input:
                for node in self.epochs:
                    self.epochs[node] += 1
                self.states.clear()
                self.input_guard.reset()
                self.input_health.reset_clock()
                self._pending_warnings.add("host_clock_reset_filters_reinitialized")
            previous = self.states.get(key)
            if previous is not None and stamp - previous["stamp"] > self.settings["stale_after_s"]:
                gap = stamp - previous["stamp"]
                # A confirmed health fault already invalidated this epoch.
                # Repeated long gaps must not churn paused filters/epochs;
                # sustained recovery will create exactly one fresh filter.
                paused = not self.input_health.integration_allowed(key)
                reset = gap > self.settings["filter_reset_gap_s"] and not paused
                _record_gap(self.gap_diagnostics, key, gap, stamp, reset=reset)
                if reset:
                    self.epochs[key] += 1
                    self._pending_warnings.add(f"node_{key}_gap_filter_reinitialized")
                    previous = None
                    self.input_guard.reset(key)
                elif paused:
                    self._pending_warnings.add(f"node_{key}_gap_input_health_paused")
                else:
                    self._pending_warnings.add(f"node_{key}_short_gap_filter_preserved")
            if previous is None:
                previous = dict(filter=self._factory(self.period, **self._vqf_parameters), stamp=stamp)
                self.states[key] = previous
            transitions = self.input_health.observe(key, sample.values, sample.status, stamp,
                                                    np.rad2deg(gyro))
            for event, affected in transitions:
                if event == "fault":
                    self.epochs[affected] += 1
                    self._pending_warnings.add(f"node_{affected}_input_health_fault")
                else:
                    # The fault already invalidated the old calibration epoch.
                    # Recovery never resumes a filter that missed real motion.
                    if affected in self.states:
                        self.states[affected]["filter"] = self._factory(self.period, **self._vqf_parameters)
                    self.input_guard.reset(affected)
                    self._pending_warnings.add(f"node_{affected}_input_health_recovered")
            used_gyro, used_accel, protection = self.input_guard.process(
                key, sample.values, gyro, accel,
                gyro_bias_rad_s=previous["filter"].getBiasEstimate()[0])
            if protection["gyro_rejected"] or protection["accel_rejected"]:
                self.input_guard_rejections += 1
            integration_allowed = self.input_health.integration_allowed(key)
            if integration_allowed:
                previous["filter"].update(used_gyro, used_accel)
            q = np.asarray(previous["filter"].getQuat6D(), dtype=float)
            if q.shape != (4,) or not np.all(np.isfinite(q)) or not np.isclose(np.linalg.norm(q), 1.):
                raise ValueError(f"VQF returned an invalid quaternion for node {key}")
            self.sensor_quality.observe(key, sample.values[:3], stamp)
            self.input_observation.observe(key, sample.values, stamp)
            self.counts[key] += 1
            previous.update(stamp=stamp, quat_xyzw=q[[1, 2, 3, 0]].tolist(),
                            host_monotonic_ns=sample.host_monotonic_ns,
                            host_realtime_ns=sample.host_realtime_ns,
                            raw_values=list(sample.values),
                            gyro_rad_s=gyro.tolist(), accel_m_s2=accel.tolist())
            if protection["enabled"]:
                previous.update(input_filter=protection,
                                fusion_gyro_rad_s=used_gyro.tolist() if integration_allowed else None,
                                fusion_accel_m_s2=used_accel.tolist() if integration_allowed else None)
            self.last_input = stamp

    def input_snapshot(self, now=None):
        """Read input state without consuming the solver/logging event stream.

        The session owner holds the same lock used for update/snapshot. A UI
        monitor may call this independently of the solver without stealing a
        one-shot warning or incrementing the recorded orientation frame index.
        """
        return self.snapshot(now, consume=False)

    def snapshot(self, now=None, *, consume=True):
        stamp = _timestamp(host_clock.monotonic() if now is None else now, "host_monotonic_s")
        nodes = {}
        quality = self.sensor_quality.snapshot(stamp)
        health = self.input_health.snapshot(stamp) if self.input_health.settings["enabled"] else None
        warnings = list(_WARNINGS) + sorted(self._pending_warnings) + quality_warnings(quality)
        for key, state in self.states.items():
            age = stamp - state["stamp"]
            if age < 0:
                warnings.append(f"node_{key}_future_host_stamp")
                continue
            if age > self.settings["stale_after_s"]:
                warnings.append(f"node_{key}_stale")
                continue
            nodes[key] = dict(quat_xyzw=list(state["quat_xyzw"]), age_s=age,
                              epoch=self.epochs[key], sample_index=self.counts[key],
                              host_monotonic_ns=state["host_monotonic_ns"],
                              host_realtime_ns=state["host_realtime_ns"],
                              raw_values=list(state["raw_values"]),
                              fusion_state=describe_filter_state(state["filter"]),
                              gyro_rad_s=list(state["gyro_rad_s"]), accel_m_s2=list(state["accel_m_s2"]))
            if "input_filter" in state:
                nodes[key].update(input_filter=deepcopy(state["input_filter"]),
                                  orientation_valid=state["input_filter"]["orientation_valid"],
                                  fusion_gyro_rad_s=(list(state["fusion_gyro_rad_s"])
                                      if state["fusion_gyro_rad_s"] is not None else None),
                                  fusion_accel_m_s2=(list(state["fusion_accel_m_s2"])
                                      if state["fusion_accel_m_s2"] is not None else None))
            if health is not None:
                status = health["nodes"][key]
                nodes[key].update(input_health=deepcopy(status),
                    orientation_valid=nodes[key].get("orientation_valid", True) and status["healthy"])
        frame = dict(schema_version=1, frame_index=self.frame_index, timestamp_s=stamp,
                     orientation_source="vqf_6d", orientation_frame="independent_initial_heading",
                     nodes=nodes, warnings=warnings,
                     sensor_quality=quality,
                     input_observation=self.input_observation.snapshot(detailed=False),
                     gap_diagnostics={key: dict(value) for key, value in self.gap_diagnostics.items()})
        if self.input_guard.settings["enabled"]:
            frame["input_filter"] = dict(enabled=True, revision=INPUT_GUARD_REVISION,
                settings=dict(self.input_guard.settings),
                rejected_samples=self.input_guard_rejections,
                nodes={key: dict(value["input_filter"]) for key, value in nodes.items()},
                active_nodes=[key for key, value in nodes.items()
                              if value["input_filter"]["gyro_rejected"] or value["input_filter"]["accel_rejected"]])
        if health is not None:
            frame["input_health"] = health
        if consume:
            self.frame_index += 1
            self._pending_warnings.clear()
        return frame
