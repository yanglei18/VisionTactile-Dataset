#!/usr/bin/env python3
"""Explicit, exclusive, bounded Linux serial-wire probe; never firmware writes.

Unlike SerialSource, this records pyserial's returned bytes before independently
decoding <BB6h node blocks. The default pyserial open-time input flush and
explicit DTR/RTS updates are suppressed. OS open/close may still affect a device;
this is not a claim of physically non-invasive observation. Close the GUI and
other readers first. No port discovery, connection on import, or reconnect.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import time


FRAME_BYTES = 84
BLOCK_BYTES = 14
READ_BYTES = 4096
MAX_BYTES = 4 * 1024 * 1024
MAX_SAMPLES = 100_000


class WireAudit:
    """Independent byte framing and raw statistics, no production parser/filter."""
    def __init__(self, *, accel_g_per_lsb=.000122, max_samples=50_000):
        if (isinstance(accel_g_per_lsb, bool) or not isinstance(accel_g_per_lsb, (int, float))
                or not math.isfinite(accel_g_per_lsb) or not 0 < accel_g_per_lsb <= 1.):
            raise ValueError("nominal accel scale must be finite in (0, 1] g/LSB")
        if type(max_samples) is not int or not 6 <= max_samples <= MAX_SAMPLES:
            raise ValueError(f"max_samples must be in [6, {MAX_SAMPLES}]")
        self.accel_scale, self.max_frames = float(accel_g_per_lsb), max_samples // 6
        self.buffer = bytearray()
        self.frames = self.discarded = self.consumed = 0
        self.bank_frames = {"1-6": 0, "7-12": 0}
        self.first_frames = {}
        self.nodes = {str(node): dict(samples=0, zero=0, unique=set(), statuses=set(),
            first=None, last=None, first_raw=None, last_raw=None, repeated=0,
            accel_sum=0., accel_min=None, accel_max=None) for node in range(1, 13)}

    @property
    def sample_limit_reached(self):
        return self.frames >= self.max_frames

    def feed(self, data, stamp):
        if not isinstance(data, bytes):
            raise TypeError("wire data must be exact bytes returned by serial.read")
        if len(data) > READ_BYTES:
            raise ValueError("wire chunks must be at most 4096 bytes")
        if not math.isfinite(stamp):
            raise ValueError("host completion timestamp must be finite")
        self.buffer.extend(data)
        while len(self.buffer) >= FRAME_BYTES and not self.sample_limit_reached:
            first = self.buffer[0]
            if first not in (1, 7) or any(self.buffer[BLOCK_BYTES * index] != first + index for index in range(6)):
                del self.buffer[0]
                self.discarded += 1
                self.consumed += 1
                continue
            frame = bytes(self.buffer[:FRAME_BYTES])
            del self.buffer[:FRAME_BYTES]
            bank = "1-6" if first == 1 else "7-12"
            self.bank_frames[bank] += 1
            self.first_frames.setdefault(bank, dict(wire_offset=self.consumed,
                host_read_completion_monotonic_s=stamp, hex=frame.hex()))
            self.consumed += FRAME_BYTES
            self.frames += 1
            for offset in range(0, FRAME_BYTES, BLOCK_BYTES):
                node_id, status, *values = struct.unpack_from("<BB6h", frame, offset)
                values = tuple(values)
                node = self.nodes[str(node_id)]
                node["samples"] += 1
                node["zero"] += not any(values)
                node["repeated"] += node["last_raw"] == values
                node["unique"].add(values)
                node["statuses"].add(status)
                if node["first"] is None:
                    node["first"], node["first_raw"] = stamp, values
                node["last"], node["last_raw"] = stamp, values
                norm = math.sqrt(sum(value * value for value in values[3:])) * self.accel_scale
                node["accel_sum"] += norm
                node["accel_min"] = norm if node["accel_min"] is None else min(norm, node["accel_min"])
                node["accel_max"] = norm if node["accel_max"] is None else max(norm, node["accel_max"])

    def report(self, elapsed_s):
        nodes = {}
        for key, node in self.nodes.items():
            count = node["samples"]
            span = node["last"] - node["first"] if count else 0.
            nodes[key] = dict(samples=count,
                observed_receive_rate_hz=(count - 1) / span if count > 1 and span > 0 else None,
                samples_per_probe_second=count / elapsed_s if elapsed_s > 0 else None,
                receive_span_s=span, all_zero_raw_samples=node["zero"],
                all_samples_zero=(node["zero"] == count) if count else None,
                unique_raw_payloads=len(node["unique"]), consecutive_identical_payloads=node["repeated"],
                unique_status_values=len(node["statuses"]),
                first_raw=list(node["first_raw"]) if count else None,
                last_raw=list(node["last_raw"]) if count else None,
                accel_norm_g=dict(min=node["accel_min"], max=node["accel_max"],
                                  mean=node["accel_sum"] / count if count else None))
        return dict(frames_decoded=self.frames, samples_decoded=self.frames * 6,
                    bank_frames=self.bank_frames.copy(), bytes_discarded_during_framing=self.discarded,
                    unparsed_tail_bytes=len(self.buffer), first_frame_evidence=self.first_frames.copy(), nodes=nodes,
                    framing_validation="six ordered node IDs in 84 bytes; no verified CRC/checksum",
                    decoding="independent struct <BB6h per 14-byte node block; gyro then accel signed int16",
                    device_dropped_bytes=None, status_byte_semantics="unknown",
                    receive_rate_basis="host read completion, not device ODR; batched frames share timestamps",
                    acceleration_scale=dict(nominal_g_per_lsb=self.accel_scale,
                                            basis="explicit_configured_assumption_not_verified_device_units"))


def _open_serial(port, baud):
    """Reuse holder/TIOCEXCL checks, but never pyserial's destructive open flush."""
    if not sys.platform.startswith("linux"):
        raise RuntimeError("this exclusive no-flush probe currently supports Linux only")
    from vt_fsglove12.live_stream import (
        _find_serial_port_holders, _claim_tty_exclusive, _release_tty_exclusive, _serial_module,
    )
    holders = _find_serial_port_holders(port)
    if holders:
        raise RuntimeError(f"Port already held by {holders}; close its GUI/readers first. Nothing was stopped.")
    serial = _serial_module()

    class NoFlushReadSerial(serial.Serial):
        def _reset_input_buffer(self):
            # Serial.open normally calls this and tcflush discards pending wire
            # bytes. Deliberately do not call the base implementation.
            pass

        def _update_dtr_state(self):
            pass

        def _update_rts_state(self):
            pass

        def write(self, data):
            raise RuntimeError("probe forbids serial writes")

        def reset_input_buffer(self):
            raise RuntimeError("probe forbids input-buffer resets")

        def reset_output_buffer(self):
            raise RuntimeError("probe forbids output-buffer resets")

    opened = None
    claimed = False
    try:
        opened = NoFlushReadSerial(port=None, baudrate=baud, timeout=.020, write_timeout=0., exclusive=True)
        opened.port = port
        opened.open()
        claimed = _claim_tty_exclusive(opened)
        if not claimed:
            raise RuntimeError("TIOCEXCL unavailable; refusing a potentially shared probe")
        # TIOCEXCL prevents new opens, not already-open descriptors. Check again
        # to detect a visible reader that raced the first best-effort /proc scan.
        others = [holder for holder in _find_serial_port_holders(port)
                  if holder != (os.getpid(), opened.fileno())]
        if others:
            raise RuntimeError(f"Another port reader appeared during open: {others}; refusing to read")
        return opened
    except BaseException:
        if opened is not None:
            try:
                if claimed:
                    _release_tty_exclusive(opened)
            finally:
                opened.close()
        raise


def _close_serial(opened):
    from vt_fsglove12.live_stream import _release_tty_exclusive
    try:
        _release_tty_exclusive(opened)
    finally:
        opened.close()


def run_probe(port, report_path, *, duration_s=8., baud=115200, max_bytes=1024 * 1024,
              max_samples=50_000, accel_g_per_lsb=.000122,
              opener=None, closer=None, clock=time.monotonic):
    """Explicitly open one port once and save exact bounded bytes plus JSON.

The wire output is report_path with '.wire.bin' appended. Both outputs are
exclusive new files; all reported decode evidence refers to this saved wire.
Dependency injection is for fake serial tests, never automatic device choice.
"""
    if not isinstance(port, str) or not port.strip():
        raise ValueError("an explicit port is required")
    if (isinstance(duration_s, bool) or not isinstance(duration_s, (int, float))
            or not math.isfinite(duration_s) or not 0 < duration_s <= 30.):
        raise ValueError("duration_s must be finite, positive and at most 30")
    if type(baud) is not int or baud <= 0:
        raise ValueError("baud must be a positive integer")
    if type(max_bytes) is not int or not 1 <= max_bytes <= MAX_BYTES:
        raise ValueError(f"max_bytes must be in [1, {MAX_BYTES}]")
    audit = WireAudit(accel_g_per_lsb=accel_g_per_lsb, max_samples=max_samples)
    report_path = Path(report_path)
    wire_path = report_path.with_name(report_path.name + ".wire.bin")
    for path in (report_path, wire_path):
        if path.exists():
            raise FileExistsError(f"Refusing to replace existing evidence: {path}")
    report = dict(schema_version=1, kind="vt_fsglove12.direct_imu_wire_probe", status="not_opened",
        port=port, baud=baud, requested_duration_s=float(duration_s), max_bytes=max_bytes,
        max_samples=max_samples, hardware_open_attempted=False, serial_opened=False,
        serial_write_calls=0, input_buffer_resets=0, firmware_commands_sent=False,
        tty_exclusive_required=True,
        holder_check="best-effort visible /proc descriptors before and after open; visibility/permission limits remain",
        pyserial_default_open_flush_suppressed=True, explicit_dtr_rts_updates_suppressed=True,
        opening_side_effects="OS open/close and tty configuration can affect a device; no physical non-invasiveness claim",
        report_path=str(report_path.resolve()), wire_path=str(wire_path.resolve()),
        raw_wire_source="exact bytes returned by serial.read, saved before independent decoding",
        bytes_received=0, wire_bytes_saved=0, read_calls=0, empty_reads=0, max_in_waiting=0, initial_in_waiting=None,
        empty_read_wait_s=0., initially_empty_read_wait_s=0., total_read_call_s=0., errors=[])
    opened = None
    start = end = None
    hasher = hashlib.sha256()
    stage = "open"
    with report_path.open("x", encoding="utf-8") as report_stream, wire_path.open("xb") as wire_stream:
        try:
            report["hardware_open_attempted"] = True
            opened = (opener or _open_serial)(port, baud)
            report["serial_opened"] = True
            start = clock()
            end = start
            while True:
                now = clock()
                if now - start >= duration_s:
                    report["status"] = "duration_limit"
                    break
                if report["bytes_received"] >= max_bytes:
                    report["status"] = "byte_limit"
                    break
                if audit.sample_limit_reached:
                    report["status"] = "sample_limit"
                    break
                stage = "read"
                waiting = opened.in_waiting
                if type(waiting) is not int or waiting < 0:
                    raise ValueError("serial.in_waiting must be a nonnegative integer")
                if report["initial_in_waiting"] is None:
                    report["initial_in_waiting"] = waiting
                report["max_in_waiting"] = max(report["max_in_waiting"], waiting)
                amount = min(READ_BYTES, max(1, waiting), max_bytes - report["bytes_received"])
                before = clock()
                data = opened.read(amount)
                end = clock()
                if not isinstance(data, bytes) or len(data) > amount:
                    raise ValueError("serial.read violated requested byte count/type")
                spent = max(0., end - before)
                report["read_calls"] += 1
                report["total_read_call_s"] += spent
                if waiting == 0:
                    report["initially_empty_read_wait_s"] += spent
                if not data:
                    report["empty_reads"] += 1
                    report["empty_read_wait_s"] += spent
                report["bytes_received"] += len(data)
                stage = "wire_save"
                wire_stream.write(data)
                report["wire_bytes_saved"] += len(data)
                hasher.update(data)
                stage = "decode"
                audit.feed(data, end)
            stage = "wire_save"
            wire_stream.flush()
        except KeyboardInterrupt:
            report["status"] = "interrupted"
        except Exception as exc:
            report["status"] = stage + "_error"
            report["errors"].append(f"{type(exc).__name__}: {exc}")
        finally:
            wire_flushed = False
            try:
                # Also persist the received prefix after a read/decode error.
                wire_stream.flush()
                wire_flushed = True
            except Exception as exc:
                report["status"] = "wire_save_error"
                report["errors"].append(f"{type(exc).__name__}: {exc}")
            if opened is not None:
                try:
                    (closer or _close_serial)(opened)
                except Exception as exc:
                    report["status"] = "close_error"
                    report["errors"].append(f"{type(exc).__name__}: {exc}")
            elapsed = max(0., (end - start)) if start is not None and end is not None else 0.
            report.update(audit.report(elapsed), elapsed_read_window_s=elapsed,
                          bytes_per_second=report["bytes_received"] / elapsed if elapsed > 0 else None,
                          wire_sha256=hasher.hexdigest(),
                          wire_complete=wire_flushed and report["wire_bytes_saved"] == report["bytes_received"])
            json.dump(report, report_stream, ensure_ascii=False, allow_nan=False, indent=2)
            report_stream.write("\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true", help="明确授权本次独占打开；必须先关闭 GUI/其他读取者")
    parser.add_argument("--port", required=True, help="明确串口路径；不自动发现或重连")
    parser.add_argument("--report", required=True, help="新的 JSON 路径；另存同名 .wire.bin，不覆盖")
    parser.add_argument("--duration-s", type=float, default=8., help="默认8秒，最多30秒")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--max-bytes", type=int, default=1024 * 1024)
    parser.add_argument("--max-samples", type=int, default=50_000)
    parser.add_argument("--accel-g-per-lsb", type=float, default=.000122,
                        help="仅用于报告模长的显式名义系数，不验证/改变设备设置")
    args = parser.parse_args(argv)
    if not args.connect:
        parser.error("--connect is required; no device has been opened")
    try:
        report = run_probe(args.port, args.report, duration_s=args.duration_s, baud=args.baud,
                           max_bytes=args.max_bytes, max_samples=args.max_samples,
                           accel_g_per_lsb=args.accel_g_per_lsb)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        return 0 if report["status"] in {"duration_limit", "byte_limit", "sample_limit"} else 2
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"probe_imu_wire: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
