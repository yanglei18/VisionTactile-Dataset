"""Sample-by-sample VQF fusion and explicit, operator-started calibration.

Filters use the configured nominal sample period, never USB batch arrival deltas.
No world position or drift-free heading is inferred. All quaternions are wxyz.
"""
from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path
import json

import numpy as np

from .config import FINGERS, fingerprint, gyro_scale, pair_gyro_scales, validate_config, write_json_exclusive
from .math3d import conj, from_axis_angle, from_matrix, mul, normalize, rotate
from .kinematics import HandModel


class PoseEngine:
    def __init__(self, config: dict, *, algorithm_mode="thumb_paired_gyro_gravity_v1"):
        from vqf import VQF
        self.config = validate_config(config)
        self._vqf = VQF
        self.model = HandModel(self.config)
        if algorithm_mode not in {"thumb_paired_gyro_gravity_v1", "legacy_quaternion_v1"}:
            raise ValueError("Unsupported pose algorithm")
        self.algorithm_mode = algorithm_mode
        self.thumb_estimator = None
        self.thumb_pair_counts = [0, 0]
        self.thumb_common_capture = None
        self.last_time = None
        self.states = {}
        self.bias = {n: np.zeros(3) for n in range(1, 13)}
        self.mounts = {}
        self.axes_quality = {}
        self.axis_sign_versions = {}
        self.zero = {}
        self.neutral_acc = {}
        self.neutral_ready = False
        self.mount_confirmed = False
        self.active = None
        self.message = "尚未标定：先确认佩戴、平手静止归零，再逐项采集功能轴。"
        self.gap_count = 0
        self.invalid_count = 0
        self.near_limit_count = 0

    def export_diagnostic_checkpoint(self):
        """Copy an idle runtime for diagnostic recording, not live calibration."""
        from .diagnostic_state import export_checkpoint
        return export_checkpoint(self)

    @classmethod
    def from_offline_checkpoint(cls, data):
        """Create a NEW offline replay engine; never call this on live connect.

        A diagnostic checkpoint intentionally includes the historical zero.
        Ordinary calibration loading remains separate and always clears zero.
        """
        from .diagnostic_state import from_checkpoint
        return from_checkpoint(cls, data)

    def _new_filter(self, node, bias=None):
        filt = self._vqf(1.0 / self.config["sample_rate_hz"], motionBiasEstEnabled=False)
        estimate = self.bias[node] if bias is None else bias
        filt.setBiasEstimate(np.ascontiguousarray(np.deg2rad(estimate), dtype=float))
        return filt

    def process(self, sample):
        node, stamp = sample.node_id, float(sample.host_monotonic_s)
        values = np.asarray(sample.values, dtype=float)
        if node not in range(1, 13) or values.shape != (6,) or not np.all(np.isfinite(values)) or not math.isfinite(stamp):
            self.invalid_count += 1
            return
        state = self.states.get(node)
        if state is not None and "time" in state and stamp < state["time"]:
            self.invalid_count += 1
            self.neutral_ready = False
            self._reset_thumb_flexion()
            self.thumb_common_capture = None
            self.cancel_calibration("时间倒退：已作废本次零位，请重新连接并归零。")
            return
        if state is not None and "time" in state and stamp - state["time"] > self.config["stale_after_s"]:
            self.gap_count += 1
            self.neutral_ready = False
            self._reset_thumb_flexion()
            self.thumb_common_capture = None
            self.cancel_calibration("数据中断：需重新静止归零；已保留安装轴。")
            state["filter"] = self._new_filter(node)
            state["times"].clear()
        if state is None:
            state = dict(filter=self._new_filter(node), count=0, times=deque(maxlen=512))
            self.states[node] = state
        gyro = values[:3] * gyro_scale(self.config, node)
        acc = values[3:] * self.config["accel_g_per_lsb"]
        if np.linalg.norm(acc) < 1e-6:
            self.invalid_count += 1
            if (self.active and self.active["target"] in {"thumb_common", "thumb_hinge"}
                    and node in (self.config["fingers"]["thumb"]["proximal"], self.config["fingers"]["thumb"]["distal"])):
                self.active["thumb_invalid_input"] = True
            # Do not refresh last valid time or pass zero gravity to calibration.
            return
        if np.any(np.abs(values[:3]) >= 32700):
            self.near_limit_count += 1
        state["filter"].update(np.ascontiguousarray(np.deg2rad(gyro)), np.ascontiguousarray(acc * 9.80665))
        state.update(time=stamp, gyro=gyro, acc=acc, quat=normalize(state["filter"].getQuat6D()))
        state["count"] += 1
        state["times"].append(stamp)
        self.last_time = stamp if self.last_time is None else max(stamp, self.last_time)
        if self.active:
            if stamp >= self.active["start"]:
                rows = self.active["rows"].setdefault(node, [])
                if len(rows) < 30000:
                    rows.append((stamp, gyro.copy(), acc.copy(), state["quat"].copy()))
            if self.last_time >= self.active["end"]:
                self._finish_calibration()
        self._update_thumb_flexion(node)

    def _reset_thumb_flexion(self):
        """Discard runtime state, never infer a new physical zero from rest."""
        self.thumb_estimator = None
        pair = self.config["fingers"]["thumb"]
        self.thumb_pair_counts = [self.states.get(pair[k], {}).get("count", 0)
                                  for k in ("proximal", "distal")]

    def _update_thumb_flexion(self, node):
        """One update per fresh pair, independent of render/snapshot frequency.

        The legacy checkpoint path intentionally remains bit-for-bit unchanged.
        Functional-frame accelerations provide an observed flexion reference,
        not a promise of absolute heading or anatomical accuracy.
        """
        if self.algorithm_mode == "legacy_quaternion_v1" or not self.neutral_ready:
            return
        pair = self.config["fingers"]["thumb"]
        ids = [pair["proximal"], pair["distal"]]
        if node not in ids or any(n not in self.mounts or n not in self.neutral_acc
                                 or n not in self.states or "time" not in self.states[n]
                                 or self.axis_sign_versions.get(n) != 2 for n in ids):
            return
        states = [self.states[n] for n in ids]
        counts = [s["count"] for s in states]
        same_frame_group = (ids[0] - 1) // 6 == (ids[1] - 1) // 6
        if same_frame_group and node != max(ids):
            # Parser emits nodes in increasing ID order inside each frame.
            # A USB read can contain many frames sharing one timestamp. Only
            # the later node closes a pair; the earlier node must never take
            # an unconsumed mate left over from a rejected previous frame.
            return
        if any(count <= used for count, used in zip(counts, self.thumb_pair_counts)):
            return
        from .thumb_flexion import ThumbFlexionEstimator
        if self.thumb_estimator is None:
            skew = abs(states[0]["time"] - states[1]["time"])
            if ((same_frame_group and skew != 0) or skew > max(.02, 2 / self.config["sample_rate_hz"])
                    or any(np.any(np.abs(s["gyro"] / gyro_scale(self.config, n)) >= 32700)
                           for n, s in zip(ids, states))):
                return
            estimator = ThumbFlexionEstimator(self.config["sample_rate_hz"])
            # Keep the explicit neutral reference. When enabling/changing axes
            # the hand may already be bent: initialize from its CURRENT
            # observed gravity, never jump to a fabricated zero-degree pose.
            try:
                estimator.reset(*(rotate(self.mounts[n], self.neutral_acc[n]) for n in ids),
                                current_proximal_acc=rotate(self.mounts[ids[0]], states[0]["acc"]),
                                current_distal_acc=rotate(self.mounts[ids[1]], states[1]["acc"]))
            except ValueError:
                return  # Wait for an observable reference/current pose.
            self.thumb_estimator = estimator
            self.thumb_pair_counts = counts
            return  # This pair initialized the current pose; do not integrate it twice.
        else:
            pair_valid = all(count - used == 1 for count, used in zip(counts, self.thumb_pair_counts))
        # Arrival skew is only a plausibility check: USB batching is not a
        # sensor clock and never supplies the gyro integration period.
        skew = abs(states[0]["time"] - states[1]["time"])
        if same_frame_group and skew != 0:
            # IDs within a parsed six-node frame share the exact parse stamp.
            # Do not consume a newer half-pair against an older one: wait for
            # its mate. This preserves phase after a missing/rejected sample.
            self.thumb_estimator.update(None, None, None, None, pair_valid=False)
            return
        pair_valid = pair_valid and skew <= max(.02, 2 / self.config["sample_rate_hz"])
        self.thumb_pair_counts = counts
        values = []
        saturated = []
        for n, s in zip(ids, states):
            estimate = self.bias[n]
            if hasattr(s["filter"], "getBiasEstimate"):
                estimate = np.rad2deg(s["filter"].getBiasEstimate()[0])
            values.extend((rotate(self.mounts[n], s["gyro"] - estimate),
                           rotate(self.mounts[n], s["acc"])))
            saturated.append(bool(np.any(np.abs(s["gyro"] / gyro_scale(self.config, n)) >= 32700)))
        self.thumb_estimator.update(*values, proximal_saturated=saturated[0],
                                    distal_saturated=saturated[1], pair_valid=pair_valid)

    def _fresh(self, now):
        return {n for n, s in self.states.items() if "time" in s and 0 <= now - s["time"] <= self.config["stale_after_s"]}

    def _start(self, kind, target, duration):
        if self.active:
            raise ValueError("已有标定正在进行，请先完成或取消。")
        if not isinstance(duration, (float, int)) or not math.isfinite(duration) or not 3 <= duration <= 15:
            raise ValueError("标定时长需在 3–15 秒之间。")
        if self.last_time is None or len(self._fresh(self.last_time)) != 12:
            raise ValueError("需要全部 12 路新鲜数据；请先连接并等待。")
        self.active = dict(kind=kind, target=target, start=self.last_time, end=self.last_time + duration, rows={})
        self.message = "保持平手静止，不要屈伸或触碰小板。" if kind == "neutral" else "先缓慢弯曲，再回到原位，重复 3–4 次；不要先做反向伸展。"

    def begin_neutral(self, duration=5, confirm_mount=False):
        if not confirm_mount:
            raise ValueError("请先确认小板固定、手掌参考贴合，并摆好平手零位。")
        self._start("neutral", None, duration)
        self.mount_confirmed = True

    def begin_axis(self, target, duration=8, calibration_method="strict_v1"):
        joint_targets = ("thumb_common", "thumb_hinge") if self.algorithm_mode != "legacy_quaternion_v1" else ()
        if target not in (*FINGERS, "palm", *joint_targets):
            raise ValueError("未知的功能轴目标")
        if calibration_method not in ("strict_v1", "robust_v2"):
            raise ValueError("未知的标定方法")
        if calibration_method != "strict_v1" and target not in joint_targets:
            raise ValueError("改进标定方法仅适用于拇指联合 A/B。")
        if not self.neutral_ready:
            raise ValueError("请先完成静止归零。")
        if target == "thumb_hinge" and self.thumb_common_capture is None:
            raise ValueError("请先完成拇指联合标定 A：固定拇指形状，多方向转动整手。")
        if (target == "thumb_hinge" and
                self.thumb_common_capture.get("calibration_method", "strict_v1") != calibration_method):
            raise ValueError("A/B 标定方法不一致；请使用同一版本，旧安装轴没有改变。")
        self._start("axis", target, duration)
        if calibration_method != "strict_v1":
            self.active["calibration_method"] = calibration_method
        if target == "thumb_common":
            self.thumb_common_capture = None
            self.message = "拇指联合 A：保持拇指形状不变，轻缓地上下翻掌并左右转掌，至少两个不同旋转方向；不要屈伸拇指。"
        elif target == "thumb_hinge":
            self.message = "拇指联合 B：用另一只手固定拇指近节，只让末节先弯曲再伸直，轻缓重复；不要转掌或做对掌动作。"

    def cancel_calibration(self, message="标定已取消；之前的有效标定未覆盖。"):
        self.active = None
        self.message = message

    def _arrays(self, capture, node, minimum):
        rows = capture["rows"].get(node, [])
        # Exclude preparation/reaction at start and batch-boundary timing at end.
        rows = [r for r in rows if capture["start"] + 0.4 <= r[0] < capture["end"] - 0.2]
        if len(rows) < minimum:
            raise ValueError(f"Node {node} 有效样本不足，请核对数据率并重试本项。")
        times = np.asarray([r[0] for r in rows])
        if np.max(np.diff(times), initial=0) > self.config["stale_after_s"]:
            raise ValueError(f"Node {node} 标定期间存在数据间断。")
        return np.asarray([r[1] for r in rows]), np.asarray([r[2] for r in rows]), np.asarray([r[3] for r in rows])

    def _first_motion_sign(self, capture, node, axis):
        """Resolve PCA sign from the first coherent *untrimmed* motion bout.

        The central PCA window deliberately omits reaction time.  It must not
        choose the sign: a quick instructed flexion can finish during that
        omission, leaving extension as the first retained motion.  Tiny one-
        or two-sample disturbances can be ignored, but a meaningful short or
        off-axis first bout is rejected instead of guessing from a later one.
        """
        rate = self.config["sample_rate_hz"]
        minimum = max(6, math.ceil(.06 * rate))
        kind, bout = None, []

        def finish():
            if not bout:
                return None
            turn = sum(abs(projection) for projection, _ in bout) / rate
            disturbance = sum(speed for _, speed in bout) / rate
            if kind in (-1, 1) and len(bout) >= minimum and turn >= .75:
                return kind
            if len(bout) <= 2 and disturbance <= .5:
                return None
            raise ValueError(f"Node {node} 第一次动作太短或方向不明确；请重新先缓慢屈曲，不能用后续伸展猜测轴正负。")

        for stamp, gyro, _, _ in capture["rows"].get(node, []):
            if not capture["start"] <= stamp <= capture["end"]:
                continue
            corrected = gyro - self.bias[node]
            speed = float(np.linalg.norm(corrected))
            projection = float(np.dot(corrected, axis))
            state = None if speed <= 5 else (1 if projection > 0 else -1) if speed < 240 and abs(projection) >= .8 * speed else "ambiguous"
            if state != kind:
                sign = finish()
                if sign is not None:
                    return sign
                kind, bout = state, []
            if state is not None:
                bout.append((projection, speed))
        sign = finish()
        if sign is None:
            raise ValueError(f"Node {node} 没有足够明确的第一次屈曲动作；本次轴未保存。")
        return sign

    def _finish_calibration(self):
        capture, self.active = self.active, None
        try:
            minimum = max(30, int((capture["end"] - capture["start"] - 0.6) * self.config["sample_rate_hz"] * 0.5))
            if capture["kind"] == "neutral":
                biases, zeros, accelerations, neutral_samples = {}, {}, {}, {}
                for node in range(1, 13):
                    gyro, acc, _ = self._arrays(capture, node, minimum)
                    mean = np.median(gyro, axis=0)
                    deviation = np.linalg.norm(gyro - mean, axis=1)
                    acc_norm = np.linalg.norm(acc, axis=1)
                    if np.percentile(deviation, 95) > 2.0 or np.linalg.norm(mean) > 10:
                        raise ValueError(f"Node {node} 未保持静止；本轮零位未保存。")
                    if not 0.75 <= np.median(acc_norm) <= 1.25 or np.percentile(np.linalg.norm(acc - np.median(acc, axis=0), axis=1), 95) > 0.12:
                        raise ValueError(f"Node {node} 加速度不符合静止约 1g；检查量程/单位或重新放稳。")
                    biases[node] = mean
                    accelerations[node] = np.median(acc, axis=0)
                    neutral_samples[node] = (gyro, acc)
                # Both orientation and inclination-filter history were built
                # using the old bias.  Merely changing that bias can cause a
                # static hand to drift after "successful" zeroing.  Warm fresh
                # filters from the accepted static samples with the new bias,
                # and make their final orientations the new session reference.
                # Build every replacement before changing any live state.
                filters = {}
                for node, (gyros, accels) in neutral_samples.items():
                    filt = self._new_filter(node, bias=biases[node])
                    for gyro, acc in zip(gyros, accels):
                        filt.update(np.ascontiguousarray(np.deg2rad(gyro)),
                                    np.ascontiguousarray(acc * 9.80665))
                    zeros[node] = normalize(filt.getQuat6D())
                    filters[node] = filt
                # Transactional: a failing node never leaves a partially updated calibration.
                self.bias, self.zero, self.neutral_acc = biases, zeros, accelerations
                for node, state in self.states.items():
                    state["filter"] = filters[node]
                    state["quat"] = zeros[node].copy()
                self.neutral_ready = True
                self._reset_thumb_flexion()
                self.thumb_common_capture = None
                self.message = "静止归零完成。未做功能轴的部位仍显示为未标定；完成轴标定后可再次平手归零。"
            else:
                target = capture["target"]
                if target in {"thumb_common", "thumb_hinge"}:
                    self._finish_thumb_joint(capture, minimum)
                    return
                ids = [self.config["palm_node"]] if target == "palm" else [self.config["fingers"][target][k] for k in ("proximal", "distal")]
                if target != "palm":
                    palm_g, _, _ = self._arrays(capture, self.config["palm_node"], minimum)
                    motion = np.linalg.norm(palm_g - self.bias[self.config["palm_node"]], axis=1)
                    if np.mean(motion > 5) > 0.2:
                        raise ValueError("手掌连带运动过多，请固定手掌后重试本指功能轴。")
                updates, quality = {}, {}
                for node in ids:
                    gyro, _, _ = self._arrays(capture, node, minimum)
                    corrected = gyro - self.bias[node]
                    speed = np.linalg.norm(corrected, axis=1)
                    good = (speed > 5) & (speed < 240)
                    if np.count_nonzero(good) < 25:
                        raise ValueError(f"Node {node} 屈伸活动不足；请做完整、缓慢的往返动作。")
                    moving = corrected[good]
                    _, singular, axes = np.linalg.svd(moving, full_matrices=False)
                    linearity = float(singular[0] ** 2 / np.sum(singular ** 2))
                    if linearity < 0.8:
                        raise ValueError(f"Node {node} 运动轴不明确；本次轴未保存（拇指复杂动作需单独验证）。")
                    axis = axes[0]
                    axis = axis * self._first_motion_sign(capture, node, axis)
                    # First motion is instructed positive flexion, which is rotation about -x.
                    x = -axis
                    up = self.neutral_acc[node] / np.linalg.norm(self.neutral_acc[node])
                    if abs(float(np.dot(x, up))) > 0.85:
                        raise ValueError(f"Node {node} 功能轴与静止重力方向接近平行，无法建立可靠安装坐标。")
                    z = up - np.dot(up, x) * x
                    z /= np.linalg.norm(z)
                    y = np.cross(z, x)
                    updates[node] = from_matrix(np.stack((x, y, z)))
                    quality[node] = {"linearity": linearity, "moving_samples": int(len(moving)),
                                     "sign_basis": "first instructed flexion", "sign_window": "first coherent untrimmed motion",
                                     "axis_sign_algorithm_version": 2}
                self.mounts.update(updates)
                self.axes_quality.update(quality)
                self.axis_sign_versions.update({node: 2 for node in updates})
                if target == "thumb":
                    self._reset_thumb_flexion()
                    self.thumb_common_capture = None
                self.message = f"{target} 功能轴已保存。回到平手检查正负方向；完整标定后建议再次静止归零。"
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.message = f"标定未通过：{exc}"

    def _finish_thumb_joint(self, capture, minimum):
        from .thumb_calibration import fit_thumb_common_rotation, calibrate_thumb_pair
        if capture.get("thumb_invalid_input"):
            raise ValueError("联合标定期间存在无效拇指输入，不能用错位的剩余样本标定；旧轴未覆盖。")
        pair = self.config["fingers"]["thumb"]
        ids = [pair["proximal"], pair["distal"]]
        rows = []
        for n in ids:
            # B's sign comes from the first instructed flexion. Never discard
            # a real early flexion and mistake its return for the first motion.
            start = capture["start"] if capture["target"] == "thumb_hinge" else capture["start"] + .4
            selected = [r for r in capture["rows"].get(n, [])
                        if start <= r[0] < capture["end"] - .2]
            if len(selected) < minimum:
                raise ValueError("拇指联合标定配对样本不足。")
            if len(selected) > 2000:
                raise ValueError("联合标定每段最多 2000 组配对样本，请缩短采集窗口；不要修改实际采样率来绕过限制。")
            rows.append(selected)
        tolerance = 0. if (ids[0] - 1) // 6 == (ids[1] - 1) // 6 else max(.02, 2 / self.config["sample_rate_hz"])
        if len(rows[0]) != len(rows[1]) or any(abs(a[0] - b[0]) > tolerance
                                              for a, b in zip(*rows)):
            raise ValueError("拇指联合标定两路样本数量或到达时序不一致；旧标定保留。")
        for seq in rows:
            if max(np.diff([r[0] for r in seq]), default=0) > self.config["stale_after_s"]:
                raise ValueError("拇指联合标定期间数据中断。")
        block = {}
        for label, n, seq in zip(("prox", "dist"), ids, rows):
            raw_gyro = np.asarray([r[1] for r in seq])
            if np.any(np.abs(raw_gyro / gyro_scale(self.config, n)) >= 32700):
                raise ValueError("拇指标定出现陀螺仪近量程数据；请减慢动作并核实量程，旧轴未覆盖。")
            block[label + "_gyro"] = (raw_gyro - self.bias[n]).tolist()
            block[label + "_acc"] = np.asarray([r[2] for r in seq]).tolist()
        method = capture.get("calibration_method", "strict_v1")
        # Both raw per-node limits have already been checked above. The
        # legacy numerical A/B fitter has a scalar physical-rate sanity cap;
        # use the larger range so a valid higher-range node is not rejected
        # against its lower-range partner's sensitivity.
        settings = dict(sample_rate_hz=self.config["sample_rate_hz"],
                        gyro_saturation_dps=32700 * max(pair_gyro_scales(self.config, *ids)))
        if method != "strict_v1":
            settings["method"] = method
        ordered = lambda b: (b["prox_gyro"], b["dist_gyro"], b["prox_acc"], b["dist_acc"])
        if capture["target"] == "thumb_common":
            fit_thumb_common_rotation(*ordered(block), **settings)
            if method != "strict_v1":
                block["calibration_method"] = method
            self.thumb_common_capture = block
            self.message = "拇指联合 A 已通过；保持小板安装不变，准备好后手动选择 B：固定近节、只屈伸末节。旧轴仍保留。"
            return
        if self.thumb_common_capture is None:
            raise ValueError("拇指联合 A 数据已失效，请重新采集 A。")
        result = calibrate_thumb_pair(*ordered(self.thumb_common_capture), *ordered(block),
                                      self.neutral_acc[ids[0]], self.neutral_acc[ids[1]], **settings)
        updates = {n: normalize(result["mounts"][label]) for n, label in zip(ids, ("proximal", "distal"))}
        self.mounts.update(updates)
        for n in ids:
            self.axis_sign_versions[n] = 2
            self.axes_quality[n] = dict(result["quality"], joint_method=("paired_common_rotation_hinge_v1" if method == "strict_v1"
                                                                       else "paired_common_rotation_hinge_robust_v2"),
                                        axis_sign_algorithm_version=2)
        self.thumb_common_capture = None
        self._reset_thumb_flexion()
        self.message = "拇指联合轴已保存到内存，抗漂移弯曲估计已启用；请检查屈伸回正。未自动归零或写文件。"

    def _orientations(self):
        quats = {}
        if not self.neutral_ready:
            return quats
        left = self.config["hand"] == "left"
        r0 = {n: np.array([1., 0, 0, 0]) for n in range(1, 13)}
        for finger in self.config["fingers"].values():
            # Functional flexion plus gravity establishes a right-handed
            # physical frame on either hand.  A left hand's actual neutral
            # splay is the mirror of the canonical right-hand template.
            angle = finger["neutral_splay_deg"] * (-1 if left else 1)
            q = from_axis_angle([0, 0, 1], math.radians(angle))
            r0[finger["proximal"]] = r0[finger["distal"]] = q
        for node, mount in self.mounts.items():
            if node in self.zero and node in self.states and "quat" in self.states[node]:
                delta = mul(conj(self.zero[node]), self.states[node]["quat"])
                physical = normalize(mul(r0[node], mul(mul(mount, delta), conj(mount))))
                # Canonicalize before the model takes any relative rotations.
                # For S=diag(-1,1,1), R_canonical=S R_physical S corresponds
                # to [w,x,-y,-z].  Flexion is unchanged, splay is reflected,
                # and common wrist rotation still cancels for all fingers.
                quats[node] = physical * [1., 1., -1., -1.] if left else physical
        return quats

    def get_snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        fresh = self._fresh(now)
        # A source pause must not leave a countdown running indefinitely.
        if self.active and self.last_time is not None and now - self.last_time > self.config["stale_after_s"]:
            self.cancel_calibration("数据已停止，当前标定取消。")
        nodes = {}
        for node in range(1, 13):
            s = self.states.get(node, {})
            times = s.get("times", [])
            span = times[-1] - times[0] if len(times) > 1 else 0
            nodes[str(node)] = dict(fresh=node in fresh, age_s=max(0., now - s["time"]) if "time" in s else None,
                                   gyro_dps=s.get("gyro", np.zeros(3)).tolist(), accel_g=s.get("acc", np.zeros(3)).tolist(),
                                   samples=s.get("count", 0), rate_hz=(len(times)-1)/span if span > 0 and node in fresh else 0.,
                                   quat_wxyz=s.get("quat", np.array([1., 0, 0, 0])).tolist())
        active = None
        if self.active:
            active = {"kind": self.active["kind"], "target": self.active["target"],
                      "remaining_s": max(0., self.active["end"] - (self.last_time or self.active["start"]))}
        warnings = ["六轴航向会漂移；整体朝向相对本次归零，不提供全局位置或绝对航向。", "指尖末端关节为模型推算；拇指解剖位置尚待核实。"]
        if not self.config["units_verified"]:
            warnings.append("量程/单位未与固件核实，当前按配置进行实验性换算。")
        if not self.config["sample_rate_verified"]:
            warnings.append("融合使用配置的名义采样率；USB到达时间不是设备采样时间。")
        if self.near_limit_count:
            warnings.append(f"出现 {self.near_limit_count} 条接近 int16 码值极限的记录。")
        legacy_axes = [node for node in sorted(self.mounts) if self.axis_sign_versions.get(node, 0) != 2]
        if legacy_axes:
            warnings.append(f"节点 {legacy_axes} 的功能轴符号算法为旧版或未记录；旧轴可能方向反转，建议重新功能标定。")
        thumb = None
        if self.algorithm_mode != "legacy_quaternion_v1":
            thumb = self.thumb_estimator.snapshot() if self.thumb_estimator is not None else {
                "angle_deg": None, "valid": False, "uncertain": True,
                "quality": "uninitialized", "reasons": ["需要有效零位、两路新版功能轴、新鲜配对数据及当前可信重力观测。"]}
            if self.thumb_estimator is None and self.neutral_ready:
                pair = self.config["fingers"]["thumb"]
                ids = [pair["proximal"], pair["distal"]]
                if all(n in self.mounts and n in self.neutral_acc for n in ids):
                    from .thumb_flexion import ThumbFlexionEstimator
                    try:
                        ThumbFlexionEstimator(self.config["sample_rate_hz"]).reset(
                            *(rotate(self.mounts[n], self.neutral_acc[n]) for n in ids))
                    except ValueError as exc:
                        thumb["quality"] = "reference_inconsistent"
                        thumb["reasons"] = ["拇指两段参考轴/重力不一致或不可观测，请完成联合 A/B 标定并检查小板固定。", str(exc)]
        calibration = dict(neutral_ready=self.neutral_ready, axis_nodes=sorted(self.mounts), active=active, message=self.message)
        # Only explicitly versioned new commands produce feedback. Historical
        # strict commands/checkpoints keep their original snapshot shape.
        if self.active and self.active.get("calibration_method") == "robust_v2":
            from .calibration_feedback import thumb_capture_feedback
            calibration["quality_feedback"] = thumb_capture_feedback(self.active, self.config, self.bias)
        elif (self.active is None and self.thumb_common_capture is not None and
                self.thumb_common_capture.get("calibration_method") == "robust_v2"):
            calibration["quality_feedback"] = dict(stage="A", status="ready_for_b",
                paired_samples=len(self.thumb_common_capture["prox_gyro"]),
                message=("A 数据已保留，B 尚未完成；本轮失败原因见标定结果，可只重试 B。"
                         if self.message.startswith("标定未通过") else
                         "A 已通过；旧安装轴仍保留。准备好后手动开始 B，固定近节，仅末节先弯再伸。"))
        return dict(model=self.model.solve(self._orientations(), fresh, set(self.mounts), self.neutral_ready,
                                          thumb_flexion=thumb), nodes=nodes,
                    calibration=calibration,
                    warnings=warnings, gap_count=self.gap_count, invalid_count=self.invalid_count,
                    timing_basis="configured_nominal_period_no_device_timestamps")

    def save_calibration(self, path):
        if not self.neutral_ready or self.active:
            raise ValueError("需要有效零位且没有进行中的标定，才能保存。")
        data = dict(schema_version=1, config_fingerprint=fingerprint(self.config), created_wall_s=time.time(),
                    gyro_bias_dps={str(n): v.tolist() for n, v in self.bias.items()},
                    sensor_to_segment_wxyz={str(n): v.tolist() for n, v in self.mounts.items()},
                    neutral_acc_g={str(n): v.tolist() for n, v in self.neutral_acc.items()},
                    axes_quality={str(n): v for n, v in self.axes_quality.items()}, mount_confirmed=self.mount_confirmed,
                    axis_sign_algorithm_version=2,
                    axis_sign_versions={str(n): self.axis_sign_versions.get(n, 0) for n in self.mounts},
                    requires_neutral_on_load=True, heading_drift_corrected=False,
                    note="Sensor-frame mounting estimates and static bias only. Runtime neutral quaternions are intentionally not reused across sessions.")
        write_json_exclusive(path, data)
        self.message = f"标定已保存：{path}。重新连接/加载后需要重新平手归零。"

    def load_calibration(self, path):
        if self.active:
            raise ValueError("请先取消正在进行的标定。")
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data.get("schema_version") != 1 or data.get("config_fingerprint") != fingerprint(self.config):
            raise ValueError("标定文件版本或节点/单位/模型配置不匹配。")
        def vectors(key, length, require_all=False):
            source = data.get(key)
            if not isinstance(source, dict) or (require_all and set(source) != {str(n) for n in range(1, 13)}):
                raise ValueError(f"标定字段不完整：{key}")
            result = {}
            for node, value in source.items():
                if node not in {str(n) for n in range(1, 13)}:
                    raise ValueError("标定节点超出范围")
                arr = np.asarray(value, dtype=float)
                if arr.shape != (length,) or not np.all(np.isfinite(arr)):
                    raise ValueError("标定包含非法数值")
                if length == 4:
                    if abs(np.linalg.norm(arr)-1) > 1e-3:
                        raise ValueError("安装四元数不是单位四元数")
                    arr = normalize(arr)
                result[int(node)] = arr
            return result
        biases = vectors("gyro_bias_dps", 3, True)
        mounts = vectors("sensor_to_segment_wxyz", 4)
        acc = vectors("neutral_acc_g", 3, True)
        versions = data.get("axis_sign_versions", {})
        if not isinstance(versions, dict) or any(str(node) not in {str(n) for n in mounts} or type(version) is not int or version not in (0, 1, 2) for node, version in versions.items()):
            raise ValueError("功能轴符号算法版本记录无效")
        sign_versions = {node: versions.get(str(node), 0) for node in mounts}
        if any(np.linalg.norm(v) > 10 for v in biases.values()) or any(not 0.7 <= np.linalg.norm(v) <= 1.3 for v in acc.values()):
            raise ValueError("标定偏置或静止重力值超出合理范围")
        self.bias, self.mounts, self.neutral_acc = biases, mounts, acc
        self.axes_quality = {}  # Informational saved quality is not trusted as validation.
        self.axis_sign_versions = sign_versions
        self.neutral_ready = False
        self.zero = {}
        self._reset_thumb_flexion()
        self.thumb_common_capture = None
        self.mount_confirmed = False
        for node, state in self.states.items():
            state["filter"] = self._new_filter(node)
        self.message = "安装轴/偏置已加载；请确认本次佩戴并重新静止归零。"
        if any(version != 2 for version in sign_versions.values()):
            self.message += " 旧版/未记录的功能轴可能方向反转，建议重新做功能轴标定。"


def demo_snapshot(t, config):
    """Synthetic articulated rotations; never passed off as sensor measurements."""
    config = validate_config(config)
    quats = {n: np.array([1., 0, 0, 0]) for n in range(1, 13)}
    physical_palm = mul(from_axis_angle([0, 0, 1], math.radians(25 * math.sin(t * .45))),
                        mul(from_axis_angle([0, 1, 0], math.radians(18 * math.sin(t * .7))),
                            from_axis_angle([1, 0, 0], math.radians(22 * math.sin(t * .55)))))
    palm = physical_palm * [1., 1., -1., -1.] if config["hand"] == "left" else physical_palm
    quats[config["palm_node"]] = palm
    for i, (name, finger) in enumerate(config["fingers"].items()):
        flex = math.radians(30 * (1 + math.sin(t * 1.2 + i * .35)))
        pip = math.radians(40 * (1 + math.sin(t * 1.2 + i * .35)))
        spread = math.radians(9 * math.sin(t * .7 + i * .4))
        root = from_axis_angle([0, 0, 1], math.radians(finger["neutral_splay_deg"]) + spread)
        prox = mul(root, from_axis_angle([-1, 0, 0], flex))
        quats[finger["proximal"]] = mul(palm, prox)
        quats[finger["distal"]] = mul(palm, mul(prox, from_axis_angle([-1, 0, 0], pip)))
    return dict(model=HandModel(config).solve(quats, set(range(1, 13)), set(range(1, 13)), True),
                nodes={str(n): dict(fresh=True, age_s=0., gyro_dps=[0.,0.,0.], accel_g=[0.,0.,1.], samples=0, rate_hz=0., quat_wxyz=quats[n].tolist()) for n in range(1, 13)},
                calibration=dict(neutral_ready=True, axis_nodes=list(range(1, 13)), active=None, message="模拟数据：仅演示显示与运动学，不代表真机标定完成。"),
                warnings=["模拟数据 / DEMO — 未连接硬件，不代表真实姿态或精度。"], source="demo")
