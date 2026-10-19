"""Pure in-memory controller for the isolated experimental two-node workflow.

No serial, GUI, filesystem, thread or clock access occurs here. The caller owns
time, I/O and the background fit worker. Host times establish software pairing
and freshness only, not physical simultaneity. A session candidate is NOT an
installation into the existing live hand calibration or product acceptance.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from itertools import count
import math
from numbers import Real

import numpy as np

from .config import pair_gyro_scales, validate_config
from .hinge_observer import ConstrainedHingeObserver
from .hinge_replay import _profile
from .hinge_recovery import classify_hinge_failure
from .hinge_settings import (GYRO_OVERRIDES,
                             hinge_settings_match, settings_from_units)
from .records import Sample


_CONTROLLERS = count(1)
_PREPARE_S = 3.
_DURATIONS = {"neutral": 5., "motion": 20., "direction": 4.}
_ACTIVE = {"prepare_neutral", "capturing_neutral", "prepare_motion", "capturing_motion",
           "prepare_direction", "capturing_direction"}


def _time(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError("now_s must be a finite monotonic-clock observation")
    return float(value)


class HingeSession:
    """Manual neutral -> free motion -> background fit -> optional sign gesture.

    Raw storage is bounded across ALL attempts; overflow stops the current
    capture instead of overwriting failed evidence. Only the chosen six-node
    wire group is recorded. Fits are independently bounded to 30,000 pairs.
    Runtime summaries have a 30,000-row ring with an explicit overwrite count.
    A separate drainable 30,000-row queue preserves one actual output per
    matched raw pair, including unavailable angles. Its overflow count is
    returned by take_pair_outputs; timestamps are never used as unique keys.

    feed/tick advance the state machine; snapshot is read-only. Neither tick nor
    snapshot integrates gyro. feed advances an observer exactly once per matched
    pair, using nominal sample rate, never a UI timer or host-arrival interval.
    Operations raise ValueError for wrong API usage; sensor continuity failures
    invalidate the preview and require explicit neutral without raising.
    """

    def __init__(self, config, proximal=5, distal=6, *, max_samples=180000):
        self.config = validate_config(config)
        if (type(proximal) is not int or type(distal) is not int
                or not 1 <= proximal <= 12 or not 1 <= distal <= 12 or proximal == distal):
            raise ValueError("proximal and distal must be distinct node IDs in 1..12")
        if (proximal - 1) // 6 != (distal - 1) // 6:
            raise ValueError("cross-group pairing is unsupported")
        if type(max_samples) is not int or not 6 <= max_samples <= 1000000:
            raise ValueError("max_samples must be an integer in [6, 1000000]")
        self.proximal, self.distal = proximal, distal
        self._low, self._high = sorted((proximal, distal))
        self._group = list(range((proximal - 1) // 6 * 6 + 1, (proximal - 1) // 6 * 6 + 7))
        self.max_samples = max_samples
        self._controller_id = next(_CONTROLLERS)
        self._generation = 0
        self._connected = False
        self._connection_count = 0
        self._now = 0.
        self._connect_time = 0.
        self._state = "disconnected"
        self._message = "未连接；此工具为独立双节点实验，不修改原手套程序。"
        self._needs_neutral = True
        self._pending = None
        self._last_node_times = {}
        self._last_pair = None
        self._last_pair_time = None
        self._matched_pairs = self._dropped_pairs = 0
        self._raw_count = 0
        self._records = []
        self._active_record = None
        self._accepted_neutral = None
        self._latest_neutral = self._latest_motion = self._latest_direction = None
        self._fit_request = None
        self._fit_token = None
        self._fit_reports = []
        self._recovery = None
        self._profile = None
        self._transforms = None
        self._observer = None
        self._observer_state = None
        self._last_preview_loss_reason = None
        self._runtime = deque(maxlen=30000)
        self._runtime_overwritten = 0
        self._pair_outputs = deque(maxlen=30000)
        self._pair_outputs_discarded = 0
        self._direction = self._unconfirmed_direction()
        self._direction_history = []

    @staticmethod
    def _unconfirmed_direction():
        return dict(status="unconfirmed", multiplier=None, reason="尚未执行向掌心的首段屈曲确认。",
                    first_bout_delta_deg=None,
                    anatomical_positive="operator_declared_first_flexion_toward_palm",
                    gesture_direction_confirmed=False, anatomical_axis_validated=False)

    def _new_generation(self):
        self._generation += 1
        self._fit_request = None
        self._fit_token = None

    def connect(self, now_s):
        now = _time(now_s)
        if self._connected:
            raise ValueError("session is already connected")
        # Retained records are never discarded by reconnecting.
        self._new_generation()
        self._connected = True
        self._connection_count += 1
        self._now = self._connect_time = now
        self._pending = self._last_pair = self._last_pair_time = None
        self._last_node_times = {}
        self._observer = self._observer_state = None
        self._last_preview_loss_reason = None
        self._recovery = None
        self._needs_neutral = True
        self._state = "needs_neutral"
        self._message = "已连接；旧记录已保留。请手动开始本次静止参考。"
        return self.snapshot()

    def disconnect(self):
        if self._active_record is not None:
            self._end_record("interrupted", "disconnected")
        self._new_generation()
        self._connected = False
        self._needs_neutral = True
        self._pending = None
        self._observer = self._observer_state = None
        self._recovery = None
        self._state = "disconnected"
        self._message = "连接已断开，原始记录仍可导出；重连后需显式静止参考。"
        return self.snapshot()

    def _invalidate(self, reason):
        was_active = self._active_record is not None or self._fit_token is not None or not self._needs_neutral
        if self._active_record is not None:
            self._end_record("interrupted", reason)
        if was_active:
            self._new_generation()
        self._needs_neutral = True
        self._observer = self._observer_state = None
        self._last_preview_loss_reason = reason
        self._recovery = None
        self._state = "needs_neutral" if self._connected else "disconnected"
        self._message = f"当前预览已失效：{reason}。保留记录；恢复数据后需手动静止参考。"

    def _end_record(self, status, reason=None):
        record = self._active_record
        if record is not None:
            record.update(status=status, reason=reason, ended_at_s=self._now)
            if record["kind"] == "direction" and status != "completed":
                self._direction.update(status="failed", multiplier=None, reason=reason,
                                       gesture_direction_confirmed=False)
            self._active_record = None

    def _require_start(self, now_s):
        now = _time(now_s)
        self.tick(now)
        if not self._connected:
            raise ValueError("connect before starting a capture")
        if self._state in _ACTIVE or self._state == "fitting":
            raise ValueError("cancel the current operation before starting another")
        if self._raw_count >= self.max_samples or len(self._records) >= 256:
            raise ValueError("retained capture buffer is full; export it and create a new session")
        return now

    def _start_record(self, kind, now):
        self._new_generation()
        record = dict(kind=kind, attempt_id=len(self._records) + 1,
                      status="preparing", reason=None, connection=self._connection_count,
                      requested_at_s=now, capture_start_s=now + _PREPARE_S,
                      capture_end_s=now + _PREPARE_S + _DURATIONS[kind],
                      ended_at_s=None, samples=[], pairs=[], direction_trace=[])
        self._records.append(record)
        self._active_record = record
        setattr(self, "_latest_" + kind, record)
        self._state = "prepare_" + kind
        return record

    def start_neutral(self, now_s):
        now = self._require_start(now_s)
        self._recovery = None
        self._observer = self._observer_state = None
        self._profile = self._transforms = None
        self._needs_neutral = True
        self._accepted_neutral = None
        self._direction_history.append(deepcopy(self._direction))
        self._direction = self._unconfirmed_direction()
        self._start_record("neutral", now)
        self._message = "3 秒准备，然后保持当前手型静止 5 秒；只检查所选节点对。"
        return self.snapshot()

    def start_motion(self, now_s):
        now = self._require_start(now_s)
        if self._needs_neutral or self._accepted_neutral is None:
            raise ValueError("a successful explicit neutral from this continuous connection is required")
        self._recovery = None
        self._observer = self._observer_state = None
        self._profile = self._transforms = None
        self._start_record("motion", now)
        self._message = "3 秒准备，然后两骨段自然屈伸并轻转整手，共 20 秒；不需要旧 A/B。"
        return self.snapshot()

    def start_direction(self, now_s):
        now = self._require_start(now_s)
        snap = self.snapshot(now)
        if not snap["preview_valid"] or not self._observer_state.get("current_angle_observed", False):
            raise ValueError("a fresh currently observed candidate angle is required")
        self._direction_history.append(deepcopy(self._direction))
        self._direction = self._unconfirmed_direction()
        self._direction.update(status="collecting", reason="从舒适姿态先向掌心屈曲至少 8°，随后可返回；不要求回到零位。")
        self._start_record("direction", now)
        self._message = "3 秒准备，保持舒适姿态；随后先向掌心屈曲，记录 4 秒用于正向确认，不重设零位。"
        return self.snapshot()

    def cancel(self):
        kind = self._active_record["kind"] if self._active_record is not None else None
        if self._active_record is not None:
            self._end_record("cancelled", "operator_cancelled")
        if self._state == "fitting":
            self._fit_reports.append(dict(token=self._fit_token, error="operator_cancelled", profile=None,
                                          recovery=deepcopy(self._recovery)))
        self._new_generation()
        if kind == "neutral":
            self._needs_neutral = True
        if not self._connected:
            self._state = "disconnected"
        elif self._needs_neutral:
            self._state = "needs_neutral"
        elif self._observer is not None:
            self._state = "candidate_ready"
        else:
            self._state = "neutral_ready"
        self._message = "已取消；已有原始记录保留供导出，未自动归零。"
        return self.snapshot()

    def _advance_phase(self, now):
        self._now = max(self._now, now)
        record = self._active_record
        if record is None:
            return
        if now >= record["capture_end_s"]:
            self._finish_capture()
        elif now >= record["capture_start_s"]:
            record["status"] = "capturing"
            self._state = "capturing_" + record["kind"]

    def tick(self, now_s):
        now = _time(now_s)
        if self._connected and now < self._now:
            self._invalidate("controller_clock_regressed")
            return self.snapshot()
        self._now = now
        if self._connected:
            base = self._last_pair_time if self._last_pair_time is not None else self._connect_time
            if now - base > self.config["stale_after_s"]:
                if self._state != "needs_neutral" or not self._needs_neutral:
                    self._invalidate("selected_pair_stale")
                self._pending = None
            else:
                self._advance_phase(now)
        return self.snapshot(now)

    def feed(self, samples, now_s=None):
        if not isinstance(samples, (list, tuple)) or len(samples) > 60000 or any(not isinstance(s, Sample) for s in samples):
            raise ValueError("samples must be a bounded list/tuple of at most 60000 Sample objects")
        now = _time(now_s) if now_s is not None else (max((s.host_monotonic_s for s in samples), default=self._now))
        if not self._connected:
            return self.snapshot()
        for sample in samples:
            t, node = sample.host_monotonic_s, sample.node_id
            if node not in self._group:
                if self._pending is not None:
                    self._drop_pair("pair_wire_order_interrupted")
                continue
            self._advance_phase(t)
            record = self._active_record
            if record is not None and record["capture_start_s"] <= t < record["capture_end_s"]:
                if self._raw_count >= self.max_samples:
                    self._invalidate("retained_raw_buffer_full_export_required")
                else:
                    record["samples"].append(sample)
                    self._raw_count += 1
            previous_time = self._last_node_times.get(node)
            self._last_node_times[node] = t
            if previous_time is not None and t < previous_time:
                self._drop_pair("node_host_time_regressed")
                continue
            if self._pending is not None:
                first, previous_node = self._pending
                if node != previous_node + 1 or node > self._high:
                    self._drop_pair("pair_wire_order_interrupted")
                elif t != first.host_monotonic_s or sample.host_wall_s != first.host_wall_s:
                    self._drop_pair("pair_host_timestamp_mismatch")
                    continue
                elif node == self._high:
                    self._pending = None
                    pair = (first, sample) if self.proximal == self._low else (sample, first)
                    self._handle_pair(pair)
                    continue
                else:
                    self._pending = (first, node)
            if node == self._low:
                self._pending = (sample, node)
            elif node == self._high and self._pending is None:
                self._drop_pair("unmatched_distal_wire_record")
        # Complete the window after processing this batch, not before it.
        return self.tick(max(now, self._now))

    def _drop_pair(self, reason):
        self._dropped_pairs += 1
        self._pending = None
        self._invalidate(reason)

    def _handle_pair(self, pair):
        t = pair[0].host_monotonic_s
        rate = self.config["sample_rate_hz"]
        host_gap = self._last_pair_time is not None and t - self._last_pair_time > max(.1, 5. / rate)
        if host_gap:
            self._invalidate("selected_pair_host_gap")
        self._last_pair = pair
        self._last_pair_time = t
        self._matched_pairs += 1
        record = self._active_record
        if record is not None and record["capture_start_s"] <= t < record["capture_end_s"]:
            if len(record["pairs"]) >= 30000:
                self._invalidate("capture_pair_limit_reached")
            else:
                record["pairs"].append(pair)
        updated = False
        if not self._needs_neutral and self._transforms is not None and self._state in {
                "candidate_ready", "prepare_direction", "capturing_direction"}:
            updated = self._update_observer(pair)
        self._append_pair_output(pair, updated=updated, host_gap=host_gap)

    def _append_pair_output(self, pair, *, updated, host_gap):
        """Snapshot AFTER this pair's handling; never synthesize fit/UI events."""
        near = any(abs(value) >= 32700 for sample in pair for value in sample.values[:3])
        if updated and self._observer_state is not None:
            # The observer's nested lists/dicts must not alias output records.
            row = deepcopy(self._observer_state)
            row["reason_codes"] = list(row.get("reasons", []))
            row["reason"] = None
        else:
            if self._needs_neutral:
                quality, reasons = "needs_neutral", ["session_needs_explicit_neutral"]
            elif self._profile is None or self._transforms is None:
                quality, reasons = "uninitialized", ["no_current_candidate"]
            else:
                quality = "uninitialized"
                reasons = ["observer_initialization_unavailable", "raw_near_limit" if near
                           else "current_gravity_unobservable"]
            if self._last_preview_loss_reason is not None:
                reasons.append(self._last_preview_loss_reason)
            # Do not fill absent outputs with a previous pose, bias or zero.
            row = dict(angle_deg=None, rate_dps=None, valid=False,
                       current_angle_observed=False, quality=quality,
                       history_loss=self._last_preview_loss_reason is not None,
                       relative_bias_dps=None, angle_std_deg=None, observation_sources=[],
                       neutral_ready=False, uncertain=True, reasons=list(reasons),
                       reason_codes=reasons, reason=self._message)
        row.update(pair_sequence=self._matched_pairs, connection_id=self._connection_count,
                   host_monotonic_s=pair[0].host_monotonic_s, host_wall_s=pair[0].host_wall_s,
                   proximal_raw=list(pair[0].values), distal_raw=list(pair[1].values),
                   raw_near_limit=near, host_gap_before=bool(host_gap), session_state=self._state)
        if len(self._pair_outputs) == self._pair_outputs.maxlen:
            self._pair_outputs_discarded += 1
        self._pair_outputs.append(row)

    def take_pair_outputs(self):
        """Drain actual per-pair outputs and the since-last-drain overflow count.

        Every successful wire-order match produces exactly one record, even
        before a candidate exists. Failed matches have no invented pair record.
        Tick/snapshot/fit acceptance produce none. Queue contents are independent
        of observer state and runtime history; ownership transfers to the caller.
        The controller is pure data and not thread-safe: serialize its calls in
        the owning worker, as with feed and accept_fit.
        """
        rows = list(self._pair_outputs)
        discarded = self._pair_outputs_discarded
        self._pair_outputs.clear()
        self._pair_outputs_discarded = 0
        return dict(rows=rows, discarded_rows=discarded)

    def _capture(self, record):
        pairs = record["pairs"]
        raw = np.asarray([[p.values, d.values] for p, d in pairs], dtype=np.int16).reshape(-1, 2, 6)
        scale = self.config["gyro_dps_per_lsb"]
        prox_scale, dist_scale = pair_gyro_scales(self.config, self.proximal, self.distal)
        capture = dict(prox_gyro_raw=raw[:, 0, :3].copy(), dist_gyro_raw=raw[:, 1, :3].copy(),
                    prox_gyro_dps=raw[:, 0, :3].astype(float) * prox_scale,
                    dist_gyro_dps=raw[:, 1, :3].astype(float) * dist_scale,
                    prox_acc_raw=raw[:, 0, 3:].copy(), dist_acc_raw=raw[:, 1, 3:].copy(),
                    near_limit_mask=np.any(np.abs(raw[:, :, :3].astype(np.int32)) >= 32700, axis=2),
                    host_monotonic_s=np.asarray([[p.host_monotonic_s, d.host_monotonic_s] for p, d in pairs]).reshape(-1, 2),
                    metadata=dict(source="explicit_in_memory_" + record["kind"],
                                  attempt_id=record["attempt_id"], connection_id=record["connection"],
                                  capture_start_s=record["capture_start_s"], capture_end_s=record["capture_end_s"],
                                  proximal=self.proximal, distal=self.distal,
                                  sample_rate_hz=self.config["sample_rate_hz"], gyro_dps_per_lsb=scale,
                                  dropped_pair_candidates=0,
                                  pairing_basis="consecutive_wire_order_equal_host_parse_times",
                                  physical_synchronization_verified=False))
        if GYRO_OVERRIDES in self.config:
            capture["metadata"][GYRO_OVERRIDES] = {str(self.proximal): prox_scale, str(self.distal): dist_scale}
        return capture

    def _neutral_check(self, record):
        capture = self._capture(record)
        n = len(capture["prox_gyro_dps"])
        if n < max(120, math.ceil(3.5 * self.config["sample_rate_hz"])):
            return ["neutral_pair_count_insufficient"], {"pairs": n}
        gyros = [capture[name] for name in ("prox_gyro_dps", "dist_gyro_dps")]
        acc = [capture[name].astype(float) * self.config["accel_g_per_lsb"] for name in ("prox_acc_raw", "dist_acc_raw")]
        metrics = dict(pairs=n, gyro_vector_std_dps=[float(np.linalg.norm(g.std(axis=0))) for g in gyros],
                       gyro_mean_norm_dps=[float(np.linalg.norm(g.mean(axis=0))) for g in gyros],
                       accel_vector_std_g=[float(np.linalg.norm(a.std(axis=0))) for a in acc],
                       mean_accel_norm_g=[float(np.linalg.norm(a.mean(axis=0))) for a in acc],
                       host_span_s=float(capture["host_monotonic_s"][-1, 0] - capture["host_monotonic_s"][0, 0]))
        reasons = []
        if capture["near_limit_mask"].any():
            reasons.append("neutral_raw_near_limit")
        if max(metrics["gyro_vector_std_dps"]) > 1. or max(metrics["gyro_mean_norm_dps"]) > 8.:
            reasons.append("neutral_gyro_not_quiet")
        if max(metrics["accel_vector_std_g"]) > .03 or any(not .85 <= norm <= 1.15 for norm in metrics["mean_accel_norm_g"]):
            reasons.append("neutral_acceleration_not_stable_gravity_like")
        if metrics["host_span_s"] < 4.:
            reasons.append("neutral_host_span_insufficient")
        return reasons, metrics

    def _finish_capture(self):
        record = self._active_record
        kind = record["kind"]
        self._end_record("completed")
        if kind == "neutral":
            reasons, metrics = self._neutral_check(record)
            record["metrics"] = metrics
            if reasons:
                record.update(status="rejected", reason=", ".join(reasons))
                self._recovery = classify_hinge_failure("neutral_invalid", reasons, metrics=metrics)
                record["recovery"] = deepcopy(self._recovery)
                self._needs_neutral = True
                self._state = "needs_neutral"
                self._message = "静止参考未通过：" + record["reason"] + "；原始数据已保留。"
            else:
                self._accepted_neutral = record
                self._recovery = None
                self._needs_neutral = False
                self._state = "neutral_ready"
                self._message = "当前节点对的静止参考已采集；请手动开始自然运动。不是解剖零位验证。"
        elif kind == "motion":
            if len(record["pairs"]) < max(240, 2 * self.config["sample_rate_hz"]):
                record.update(status="rejected", reason="motion_pair_count_insufficient")
                self._recovery = classify_hinge_failure("insufficient_motion", ["motion_pair_count_insufficient"])
                record["recovery"] = deepcopy(self._recovery)
                self._state = "neutral_ready"
                self._message = "运动数据不足，原始记录保留；可重试自然运动。"
                return
            self._state = "fitting"
            self._fit_token = f"hinge-{self._controller_id}-{self._generation}"
            record["fit_token"] = self._fit_token
            self._fit_request = dict(token=self._fit_token,
                                     neutral_capture=self._capture(self._accepted_neutral),
                                     motion_capture=self._capture(record),
                                     accel_g_per_lsb=self.config["accel_g_per_lsb"])
            self._message = "采集结束；等待后台拟合。可以继续接收数据，不启动旧 A/B。"
        else:
            self._finish_direction(record)
            self._state = "candidate_ready"

    def take_fit_request(self):
        request, self._fit_request = self._fit_request, None
        return request

    def accept_fit(self, token, profile):
        if token != self._fit_token or self._state != "fitting" or not self._connected or self._needs_neutral:
            return False
        if isinstance(profile, dict) and profile.get("profile_status") != "offline_candidate_ready":
            status = str(profile.get("profile_status", "missing_profile_status"))
            try:
                recovery = classify_hinge_failure(status, profile.get("reason_codes", []),
                                                  metrics=profile.get("neutral"))
            except ValueError:
                recovery = classify_hinge_failure("invalid_failure_report", ["invalid_failure_report"])
            codes = ", ".join(recovery["reason_codes"])
            self._finish_fit_failure(token, f"{status}: {codes}。{recovery['instruction']}",
                                     profile=profile, recovery=recovery)
            return False
        try:
            settings, matrices, vectors = _profile(profile)
            if not hinge_settings_match(settings, settings_from_units(self.config, [self.proximal, self.distal])):
                raise ValueError("profile nodes or scales differ from session")
            accepted = deepcopy(profile)
        except (ValueError, TypeError, KeyError) as exc:
            self._finish_fit_failure(token, str(exc), profile=profile)
            return False
        self._profile = accepted
        self._transforms = (matrices, vectors)
        self._observer = self._observer_state = None
        self._recovery = None
        self._fit_reports.append(dict(token=token, profile=deepcopy(accepted), error=None, recovery=None))
        self._fit_token = self._fit_request = None
        self._direction = self._unconfirmed_direction()
        self._state = "candidate_ready"
        self._message = "实验候选已生成；显示数学相对角。向掌心正向尚未确认，不能称为产品验收。"
        if self._last_pair is not None:
            self._update_observer(self._last_pair)
        return True

    def reject_fit(self, token, error):
        if token != self._fit_token or self._state != "fitting":
            return False
        self._finish_fit_failure(token, str(error))
        return True

    def _finish_fit_failure(self, token, error, *, profile=None, recovery=None):
        if recovery is None:
            recovery = classify_hinge_failure("fit_exception", ["unclassified_fit_exception"])
        self._recovery = deepcopy(recovery)
        self._fit_reports.append(dict(token=token, profile=deepcopy(profile), error=str(error),
                                      recovery=deepcopy(recovery)))
        self._fit_token = self._fit_request = None
        self._profile = self._transforms = None
        self._observer = self._observer_state = None
        self._needs_neutral = self._needs_neutral or recovery["requires_new_neutral"]
        self._state = "neutral_ready" if not self._needs_neutral else "needs_neutral"
        self._message = "拟合未通过：" + str(error) + "；静止和运动原始数据均已保留。"

    def _update_observer(self, pair):
        (rp, rd), (bp, bd, ap0, ad0) = self._transforms
        p, d = (np.asarray(sample.values, dtype=float) for sample in pair)
        prox_scale, dist_scale = pair_gyro_scales(self.config, self.proximal, self.distal)
        pg = rp @ (p[:3] * prox_scale - bp)
        dg = rd @ (d[:3] * dist_scale - bd)
        pa = rp @ (p[3:] * self.config["accel_g_per_lsb"])
        da = rd @ (d[3:] * self.config["accel_g_per_lsb"])
        near = [bool(np.any(np.abs(raw[:3]) >= 32700)) for raw in (p, d)]
        if self._observer is None:
            if any(near):
                self._message = "候选已生成，但当前陀螺近量程；暂不初始化预览。"
                return False
            candidate = ConstrainedHingeObserver(self.config["sample_rate_hz"])
            try:
                state = candidate.reset(rp @ ap0, rd @ ad0, current_proximal_acc=pa, current_distal_acc=da)
            except ValueError:
                self._message = "候选已生成，当前重力不可观测；轻转整手后会在当前姿态初始化，不自动归零。"
                return False
            self._observer = candidate
        else:
            state = self._observer.update(pg, pa, dg, da,
                                          proximal_saturated=near[0], distal_saturated=near[1])
        self._observer_state = state
        self._last_preview_loss_reason = None
        row = dict(host_monotonic_s=pair[0].host_monotonic_s, mathematical_angle_deg=state["angle_deg"],
                   angle_std_deg=state["angle_std_deg"], relative_bias_dps=state["relative_bias_dps"],
                   quality=state["quality"], current_angle_observed=state["current_angle_observed"],
                   history_loss=state["history_loss"], raw_near_limit=any(near))
        if len(self._runtime) == self._runtime.maxlen:
            self._runtime_overwritten += 1
        self._runtime.append(row)
        record = self._active_record
        if record is not None and record["kind"] == "direction" and record["capture_start_s"] <= pair[0].host_monotonic_s < record["capture_end_s"]:
            record["direction_trace"].append(dict(row))
        return True

    def _finish_direction(self, record):
        trace = record["direction_trace"]
        good = [row for row in trace if row["current_angle_observed"] and not row["raw_near_limit"]]
        failure = None
        metrics = {}
        if len(good) < max(30, int(self.config["sample_rate_hz"])) or len(good) < .8 * len(trace):
            failure = "insufficient_currently_observed_direction_samples"
        else:
            angles = np.asarray([row["mathematical_angle_deg"] for row in good])
            displacement = angles - angles[0]
            starts = np.flatnonzero(np.abs(displacement) >= 2.)
            if not len(starts):
                failure = "no_first_direction_bout"
            else:
                start = int(starts[0])
                sign = 1 if displacement[start] > 0 else -1
                projected = sign * displacement
                peak = float(projected[start])
                end = len(projected) - 1
                for index in range(start + 1, len(projected)):
                    peak = max(peak, float(projected[index]))
                    if peak - projected[index] >= 2.:
                        end = index
                        break
                variation = np.diff(projected[:end + 1])
                positive = float(np.maximum(variation, 0).sum())
                total = float(np.abs(variation).sum())
                coherent_fraction = positive / total if total else 0.
                metrics = dict(first_bout_delta_deg=sign * peak, first_bout_samples=end - start + 1,
                               first_bout_positive_variation_fraction=coherent_fraction)
                if peak < 8. or (end - start + 1) / self.config["sample_rate_hz"] < .12 or coherent_fraction < .75:
                    failure = "first_bout_too_small_short_or_incoherent"
                else:
                    self._direction.update(status="confirmed", multiplier=sign, reason=None,
                                           gesture_direction_confirmed=True, **metrics)
        if failure:
            self._direction.update(status="failed", multiplier=None, reason=failure,
                                   gesture_direction_confirmed=False, **metrics)
            record.update(status="rejected", reason=failure)
            self._message = "正向确认未通过；继续显示数学角，不归零。可从舒适姿态重试首段向掌心屈曲。"
        else:
            self._message = "首段向掌心屈曲正向已确认；这是用户声明的方向，不是解剖精度验收。"
        record["direction_result"] = deepcopy(self._direction)

    def snapshot(self, now_s=None):
        now = self._now if now_s is None else _time(now_s)
        age = None if self._last_pair_time is None else max(0., now - self._last_pair_time)
        fresh = age is not None and age <= self.config["stale_after_s"]
        preview = bool(self._connected and not self._needs_neutral and fresh and self._observer_state is not None)
        angle = self._observer_state["angle_deg"] if preview else None
        record = self._active_record
        countdown = max(0., record["capture_start_s"] - now) if record else 0.
        remaining = max(0., record["capture_end_s"] - max(now, record["capture_start_s"])) if record else 0.
        multiplier = self._direction["multiplier"] if self._direction["status"] == "confirmed" else None
        return dict(state=self._state, connected=self._connected, needs_neutral=self._needs_neutral,
                    preview_valid=preview, pair_age_s=age, selected_nodes=[self.proximal, self.distal],
                    selected_wire_group=list(self._group), countdown_s=countdown, capture_remaining_s=remaining,
                    captured_samples=len(record["samples"]) if record else 0,
                    captured_pairs=len(record["pairs"]) if record else 0,
                    math_angle_deg=angle, anatomical_angle_deg=angle * multiplier if angle is not None and multiplier is not None else None,
                    observer=deepcopy(self._observer_state), direction=deepcopy(self._direction),
                    recovery=deepcopy(self._recovery),
                    status_message=self._message, matched_pairs=self._matched_pairs, dropped_pairs=self._dropped_pairs,
                    retained_raw_samples=self._raw_count, max_raw_samples=self.max_samples,
                    attempts=len(self._records), fit_pending=self._fit_token is not None,
                    profile_available=self._profile is not None,
                    experiment_scope="isolated_pair_candidate_not_live_hand_calibration",
                    production_acceptance="not_evaluated")

    def validation_context(self):
        """Freeze the candidate used for this validation, not a later refit."""
        if self._profile is None or self._needs_neutral or not self._connected:
            raise ValueError("a current connected candidate is required for validation")
        return dict(profile=deepcopy(self._profile), direction=deepcopy(self._direction))

    def export_bundle(self):
        """Return independent retained data; Sample objects are immutable.

        Caller explicitly writes CSV/JSON. Top-level latest sample lists alias
        no internal lists; they duplicate entries already present in attempts,
        so exporters need not write those records twice. No recording is cleared.
        """
        attempts = []
        for record in self._records:
            item = {key: deepcopy(value) for key, value in record.items() if key not in {"pairs", "samples"}}
            item["samples"] = list(record["samples"])
            item["matched_pairs"] = len(record["pairs"])
            attempts.append(item)
        return dict(kind="vt_glove_pose.experimental_hinge_session", schema_version=1,
                    raw_scope="selected_six_node_wire_group_not_all_twelve_nodes",
                    time_basis="host_parse_pairing_nominal_rate_integration_not_device_time",
                    hardware_synchronization_verified=False, live_calibration_changed=False,
                    production_acceptance="not_evaluated", config=deepcopy(self.config),
                    proximal=self.proximal, distal=self.distal, selected_wire_group=list(self._group),
                    connections=self._connection_count, attempts=attempts,
                    neutral_samples=list(self._latest_neutral["samples"]) if self._latest_neutral else [],
                    motion_samples=list(self._latest_motion["samples"]) if self._latest_motion else [],
                    direction_samples=list(self._latest_direction["samples"]) if self._latest_direction else [],
                    profile=deepcopy(self._profile), fit_reports=deepcopy(self._fit_reports),
                    direction=deepcopy(self._direction), direction_history=deepcopy(self._direction_history),
                    runtime_results=[dict(row) for row in self._runtime], runtime_rows_overwritten=self._runtime_overwritten,
                    final_snapshot=self.snapshot(),
                    limitations=["Explicit static pose is user-declared; quiet data do not prove anatomy or zero linear acceleration.",
                                 "A paired preview is not a complete hand pose or six-DoF root estimate.",
                                 "Direction is the first user-declared palmward bout, not independently measured anatomical ground truth."])
