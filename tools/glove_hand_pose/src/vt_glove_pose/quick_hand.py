"""Explicitly approximate whole-hand preview with session-only pose references.

This reuses sample-by-sample VQF and the existing hand geometry, but not the
strict functional-calibration workflow. A user-requested open-hand reference
is mandatory. Gravity plus an assumed board +Y direction supplies provisional
mounts; gravity alone cannot determine mounting heading. A straight-hand nod
registers all sensors to the palm using common-motion directions. This is an
estimated installation frame, never anatomical calibration. The old independent
finger-learning method remains a compatibility API, not the current UI flow.
No rest detector straightens fingers or resets the reference.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np

from .config import fingerprint, gyro_scale, validate_config
from .engine import PoseEngine
from .kinematics import HandModel
from .math3d import conj, from_axis_angle, from_matrix, mul, normalize


def _mount(up, flexion_axis=None):
    """Sensor-to-local-segment rotation; default board +Y points distally."""
    z = np.asarray(up, dtype=float)
    z = z / np.linalg.norm(z)
    if flexion_axis is None:
        forward = np.array([0., 1., 0.])
        y = forward - np.dot(forward, z) * z
        if np.linalg.norm(y) < .1:
            # Explicitly arbitrary fallback, not an observed mounting heading.
            forward = np.array([1., 0., 0.])
            y = forward - np.dot(forward, z) * z
        y /= np.linalg.norm(y)
        x = np.cross(y, z)
    else:
        x = -np.asarray(flexion_axis, dtype=float)
        x /= np.linalg.norm(x)
        z = z - np.dot(z, x) * x
        if np.linalg.norm(z) < .2:
            raise ValueError("motion axis is too close to neutral gravity")
        z /= np.linalg.norm(z)
        y = np.cross(z, x)
    return from_matrix(np.stack((x, y, z)))


def _first_motion_sign(projection, eligible, rate):
    """First sustained closing bout, not the first noisy sample or net cycle."""
    minimum = max(3, math.ceil(.1 * rate))
    run_sign, count, travel = 0, 0, 0.
    for value, good in zip(projection, eligible):
        sign = 1 if value > 3. else -1 if value < -3. else 0
        if not good or not sign:
            run_sign, count, travel = 0, 0, 0.
            continue
        if sign != run_sign:
            run_sign, count, travel = sign, 0, 0.
        count += 1
        travel += abs(value) / rate
        if count >= minimum and travel >= 1.:
            return sign
    return None


def _paired_rows(source, palm, tolerance):
    """Greedy one-to-one host-time pairing; repeated USB stamps stay distinct."""
    result, cursor = [], 0
    for row in source:
        if cursor >= len(palm):
            break
        while cursor + 1 < len(palm) and abs(palm[cursor + 1][0] - row[0]) < abs(palm[cursor][0] - row[0]):
            cursor += 1
        if abs(palm[cursor][0] - row[0]) <= tolerance:
            result.append((row, palm[cursor]))
            cursor += 1
    return result


def _direction_registration(pairs, sensor_bias, palm_bias, sensor_scale, palm_scale):
    """Estimate sensor->palm from co-motion directions, not gyro magnitudes."""
    source, target, weights, is_gyro = [], [], [], []
    gyro_pairs = 0
    for sensor, palm in pairs:
        gs, gp = sensor[1] - sensor_bias, palm[1] - palm_bias
        if (min(np.linalg.norm(gs), np.linalg.norm(gp)) > 5.
                and np.max(np.abs(sensor[1] / sensor_scale)) < 32700
                and np.max(np.abs(palm[1] / palm_scale)) < 32700):
            source.append(gs / np.linalg.norm(gs)); target.append(gp / np.linalg.norm(gp)); weights.append(1.)
            is_gyro.append(True)
            gyro_pairs += 1
        ns, npalm = np.linalg.norm(sensor[2]), np.linalg.norm(palm[2])
        if .5 <= ns <= 1.5 and .5 <= npalm <= 1.5:
            source.append(sensor[2] / ns); target.append(palm[2] / npalm); weights.append(.25)
            is_gyro.append(False)
    if gyro_pairs < 12:
        raise ValueError("共同转动样本不足")
    a, b, base = np.asarray(source), np.asarray(target), np.asarray(weights)
    for values in (a, b):
        singular = np.linalg.svd(values * np.sqrt(base[:, None]), compute_uv=False)
        if singular[1] < 1e-4 * singular[0]:
            raise ValueError("运动和重力方向完全退化，无法确定安装航向")
    weight = base.copy()
    for _ in range(4):
        u, _, vt = np.linalg.svd((a * weight[:, None]).T @ b)
        rotation = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
        residual = np.linalg.norm(a @ rotation.T - b, axis=1)
        weight = base * np.minimum(1., .25 / np.maximum(residual, 1e-9))
    angles = np.degrees(np.arccos(np.clip(np.sum((a @ rotation.T) * b, axis=1), -1., 1.)))
    # Stationary gravity can greatly outnumber moving gyro directions. Keep
    # the historical mixed diagnostics, but never use them to hide poor fit
    # specifically during the motion that establishes mounting heading.
    gyro_angles = angles[np.asarray(is_gyro)]
    gravity_angles = angles[~np.asarray(is_gyro)]
    return from_matrix(rotation), dict(paired_samples=len(pairs), gyro_direction_pairs=gyro_pairs,
                                      direction_residual_median_deg=float(np.median(angles)),
                                      direction_residual_p95_deg=float(np.percentile(angles, 95)),
                                      gyro_direction_residual_median_deg=float(np.median(gyro_angles)),
                                      gyro_direction_residual_p95_deg=float(np.percentile(gyro_angles, 95)),
                                      gravity_direction_residual_median_deg=float(np.median(gravity_angles)) if len(gravity_angles) else None,
                                      gravity_direction_residual_p95_deg=float(np.percentile(gravity_angles, 95)) if len(gravity_angles) else None,
                                      calibrated=False)


class QuickHandEngine:
    """Small live/replay engine with manual reference and optional axis learning.

    ``process`` uses the configured nominal VQF period even when USB timestamps
    repeat. ``get_snapshot`` never advances fusion. Node 11 is not required;
    a lost finger node invalidates only that node's session reference, while a
    lost palm necessarily makes the whole-hand geometry unavailable.
    """

    def __init__(self, config):
        self.config = validate_config(config)
        self.fusion = PoseEngine(self.config, algorithm_mode="legacy_quaternion_v1")
        # The old numerical template is mirrored by the legacy canvas. Quick
        # mode uses the actual displayed anatomical template in BOTH the
        # quaternion mathematics and geometry, with no later canvas repair.
        self.model_config = validate_config(self.config)
        for finger in self.model_config["fingers"].values():
            finger["base_mm"][0] *= -1
            finger["neutral_splay_deg"] *= -1
        self.model = HandModel(self.model_config)
        self.required_nodes = {self.config["palm_node"]}
        for finger in self.config["fingers"].values():
            self.required_nodes.update((finger["proximal"], finger["distal"]))
        self.zero = {}
        self.neutral_acc = {}
        self.mounts = {}
        self.axes_source = {}
        self.axes_evidence = {}
        self.direction_source = {}
        self.neutral_ready = False
        self.active = None
        self.message = "张开右手并放稳，点击“张手参考”；之后即可预览五指。" if self.config["hand"] == "right" else "张开左手并放稳，点击“张手参考”；之后即可预览五指。"
        self._notes = []

    def process(self, sample):
        node = sample.node_id
        before = self.fusion.states.get(node, {})
        old_time, old_count = before.get("time"), before.get("count", 0)
        stamp = float(sample.host_monotonic_s)
        interrupted = old_time is not None and (stamp < old_time or stamp - old_time > self.config["stale_after_s"])
        self.fusion.process(sample)
        if interrupted and node in self.required_nodes:
            self.zero.pop(node, None)
            if self.active:
                self.cancel_calibration()
            self.message = f"节点 {node} 数据曾中断，受影响部位暂停；重新张手参考可恢复，其余部位继续。"
        state = self.fusion.states.get(node, {})
        if state.get("count", 0) == old_count or not self.active:
            return
        capture = self.active
        if capture["start"] <= stamp < capture["end"] and node in self.required_nodes:
            rows = capture["rows"].setdefault(node, [])
            if len(rows) < 10000:
                rows.append((stamp, state["gyro"].copy(), state["acc"].copy()))
        if self.fusion.last_time >= capture["end"]:
            self._finish()

    def _start(self, kind, duration):
        if self.active:
            raise ValueError("已有引导正在进行，请先完成或取消。")
        if isinstance(duration, bool) or not isinstance(duration, (float, int)) or not math.isfinite(duration) or not .2 <= duration <= 15:
            raise ValueError("引导时长需在 0.2–15 秒之间。")
        now = self.fusion.last_time
        if now is None or not self.required_nodes.issubset(self.fusion._fresh(now)):
            raise ValueError("请先等待五指及手掌的新鲜数据；辅助节点 11 不必连接。")
        self.active = dict(kind=kind, start=now, end=now + float(duration), rows={})

    def begin_reference(self, duration=2.0):
        self._start("reference", duration)
        self.message = "五指自然张开、手掌尽量平放，保持到倒计时结束；这次姿势将作为伸直参考。"

    def begin_motion_learning(self, duration=6.0):
        if not self.neutral_ready or not self.required_nodes.issubset(self.zero):
            raise ValueError("请先完成张手参考。")
        self._start("motion_learning", duration)
        self.message = "手掌尽量固定，从张手开始，五指同时先握拳再张开，轻缓重复；无需逐指操作。"

    def begin_frame_alignment(self, duration=6.0):
        if not self.neutral_ready or not self.required_nodes.issubset(self.zero):
            raise ValueError("请先完成张手参考。")
        self._start("frame_alignment", duration)
        self.message = "五指保持伸直、手形不变：先整手向下轻点，再上下翻掌。不要握拳；正在统一手掌与五指的安装坐标。"

    def cancel_calibration(self):
        self.active = None
        self.message = "引导已取消；原来的张手参考和估计轴没有改变。"

    def export_profile(self):
        """Return reusable ESTIMATED mounts, never session pose/zero/bias.

        This method performs no disk write. A loaded profile can be exported
        before a fresh open-hand reference has been acquired.
        """
        if self.active:
            raise ValueError("请先完成或取消当前引导，再保存估计轴。")
        if not self.required_nodes.issubset(self.mounts):
            raise ValueError("尚无完整估计轴，请先完成张手参考或导入快速模式配置。")
        data = dict(kind="vt_glove_pose.quick_hand_profile", schema_version=1,
                    axes_calibrated=False, anatomical_accuracy_verified=False,
                    coordinate_convention="anatomical_direct_v1",
                    config_fingerprint=fingerprint(self.config),
                    mounts={str(n): self.mounts[n].tolist() for n in sorted(self.required_nodes)},
                    axes_source={str(n): self.axes_source[n] for n in sorted(self.required_nodes)},
                    axes_evidence={str(n): self.axes_evidence[n] for n in sorted(self.axes_evidence)},
                    direction_source={str(n): self.direction_source.get(n, "board_axis_assumption")
                                      for n in sorted(self.required_nodes)})
        return self._validated_profile(data)

    def _validated_profile(self, data):
        """Validate completely before committing an import to live state."""
        fields = {"kind", "schema_version", "axes_calibrated", "anatomical_accuracy_verified",
                  "coordinate_convention", "config_fingerprint", "mounts", "axes_source", "axes_evidence", "direction_source"}
        if not isinstance(data, dict) or set(data) != fields:
            raise ValueError("不是完整的快速全手估计轴文件；不接受旧正式标定或运行时零位。")
        if (data["kind"] != "vt_glove_pose.quick_hand_profile"
                or type(data["schema_version"]) is not int or data["schema_version"] != 1
                or data["axes_calibrated"] is not False
                or data["anatomical_accuracy_verified"] is not False
                or data["coordinate_convention"] != "anatomical_direct_v1"):
            raise ValueError("快速全手文件类型、版本或估计属性不匹配。")
        if data["config_fingerprint"] != fingerprint(self.config):
            raise ValueError("配置不匹配：请使用相同手侧、节点映射、单位和手模型设置。")
        ids = {str(n) for n in self.required_nodes}
        for key in ("mounts", "axes_source", "direction_source"):
            if not isinstance(data[key], dict) or set(data[key]) != ids:
                raise ValueError(f"{key} 必须包含五指及手掌的完整节点。")
        evidence = data["axes_evidence"]
        if not isinstance(evidence, dict) or not set(evidence).issubset(ids):
            raise ValueError("估计轴证据的节点不匹配。")
        for node in sorted(ids):
            value = data["mounts"][node]
            if (not isinstance(value, list) or len(value) != 4
                    or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)
                    or not math.isclose(math.hypot(*value), 1., abs_tol=1e-5)):
                raise ValueError(f"节点 {node} 安装旋转必须为有限单位 wxyz 四元数。")
            if data["axes_source"][node] not in ("gravity_board_axis_assumption", "motion_pca_estimate", "common_frame_alignment_estimate"):
                raise ValueError(f"节点 {node} 估计轴来源不受支持。")
            if data["direction_source"][node] not in ("board_axis_assumption", "first_sustained_instructed_close_motion", "first_sustained_palm_nod", "manual_flip"):
                raise ValueError(f"节点 {node} 方向来源不受支持。")
        for value in evidence.values():
            if not isinstance(value, dict) or value.get("calibrated", False) is not False:
                raise ValueError("轴证据必须是估计说明，不能声称已经标定。")
        try:
            return json.loads(json.dumps(data, allow_nan=False))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("估计轴文件含有非有限值或非 JSON 数据。") from exc

    def load_profile(self, data):
        """Install validated mounts transactionally, requiring a NEW reference."""
        if self.active:
            raise ValueError("请先完成或取消当前引导，再导入估计轴。")
        profile = self._validated_profile(data)
        mounts = {int(n): normalize(q) for n, q in profile["mounts"].items()}
        self.mounts = mounts
        self.axes_source = {int(n): value for n, value in profile["axes_source"].items()}
        self.axes_evidence = {int(n): value for n, value in profile["axes_evidence"].items()}
        self.direction_source = {int(n): value for n, value in profile["direction_source"].items()}
        self.zero = {}
        self.neutral_acc = {}
        self.neutral_ready = False
        self._notes = []
        self.message = "估计轴已导入；未复用旧零位或零偏。请张开手并执行 2 秒张手参考后继续。"

    def flip_direction(self, finger, segment="both"):
        """Explicit local x/y reversal; retain z, current pose and fusion state."""
        if self.active:
            raise ValueError("请先完成或取消当前引导，再翻转方向。")
        if not isinstance(finger, str) or finger not in self.config["fingers"]:
            raise ValueError("请选择有效手指：thumb/index/middle/ring/little。")
        if segment not in ("proximal", "distal", "both"):
            raise ValueError("segment 必须为 proximal、distal 或 both。")
        mapping = self.config["fingers"][finger]
        ids = [mapping[key] for key in (("proximal", "distal") if segment == "both" else (segment,))]
        if any(node not in self.mounts for node in ids):
            raise ValueError("所选部位尚无估计轴，请先张手参考或导入配置。")
        half_turn = from_axis_angle([0., 0., 1.], math.pi)
        updates = {node: normalize(mul(half_turn, self.mounts[node])) for node in ids}
        self.mounts.update(updates)
        self.direction_source.update({node: "manual_flip" for node in ids})
        self.message = f"{finger} / {segment} 方向已手动翻转；原张手参考和滤波状态未改变。"

    def _finish(self):
        capture, self.active = self.active, None
        if capture["kind"] == "reference":
            self._finish_reference(capture)
        elif capture["kind"] == "frame_alignment":
            self._finish_alignment(capture)
        else:
            self._finish_learning(capture)

    def _finish_reference(self, capture):
        now = self.fusion.last_time
        missing = [n for n in sorted(self.required_nodes)
                   if len(capture["rows"].get(n, [])) < 3 or n not in self.fusion._fresh(now)]
        if missing:
            self.message = f"张手参考未保存：节点 {missing} 没有足够的新鲜数据；原参考保留。"
            return
        zeros, accelerations, mounts, replacements, biases = {}, {}, {}, {}, {}
        notes = []
        for node in sorted(self.required_nodes):
            rows = capture["rows"][node]
            gyros = np.array([r[1] for r in rows])
            accs = np.array([r[2] for r in rows])
            median = np.median(gyros, axis=0)
            variation = float(np.sqrt(np.mean(np.sum((gyros - median) ** 2, axis=1))))
            # Moving reference remains usable as an explicitly rough preview;
            # do not install its motion as a spurious gyro bias.
            quiet = variation < 2. and np.linalg.norm(median) < 5.
            bias = median if quiet else self.fusion.bias[node].copy()
            if not quiet:
                notes.append(f"节点 {node} 张手参考有动作，未用本轮运动更新零偏。")
            if len(rows) < .5 * (capture["end"] - capture["start"]) * self.config["sample_rate_hz"]:
                notes.append(f"节点 {node} 参考样本偏少，当前仍按快速估计显示。")
            up = np.median(accs, axis=0)
            if np.linalg.norm(up) < .1:
                up = accs[-1]
                notes.append(f"节点 {node} 参考重力变化较大，使用末次方向作粗略参考。")
            if not .75 <= np.median(np.linalg.norm(accs, axis=1)) <= 1.25:
                notes.append(f"节点 {node} 加速度模长偏离 1g，请后续核实单位或减小运动。")
            filt = self.fusion._new_filter(node, bias=bias)
            for gyro, acc in zip(gyros, accs):
                filt.update(np.ascontiguousarray(np.deg2rad(gyro)), np.ascontiguousarray(acc * 9.80665))
            zeros[node] = normalize(filt.getQuat6D())
            accelerations[node] = up.copy()
            # Existing learned mounts are session estimates, not discarded by
            # an explicit re-reference; first reference uses board assumptions.
            mounts[node] = self.mounts.get(node, _mount(up)).copy()
            replacements[node], biases[node] = filt, bias.copy()
        for node, filt in replacements.items():
            self.fusion.states[node].update(filter=filt, quat=zeros[node].copy())
            self.fusion.bias[node] = biases[node]
            self.axes_source.setdefault(node, "gravity_board_axis_assumption")
            self.direction_source.setdefault(node, "board_axis_assumption")
        self.zero, self.neutral_acc, self.mounts = zeros, accelerations, mounts
        self.neutral_ready = True
        self._notes = notes
        self.message = "张手参考完成。请保持五指伸直，点击“整手坐标对齐”，先整手向下轻点再上下翻掌；不要握拳。当前不是精确解剖标定。"
        if notes:
            self.message += " 参考有轻微动作或数据提示，详见警告。"

    def _finish_learning(self, capture):
        learned, retained = [], []
        palm = self.config["palm_node"]
        notes = []
        for node in sorted(self.required_nodes - {palm}):
            rows = capture["rows"].get(node, [])
            gyro = np.array([r[1] - self.fusion.bias[node] for r in rows])
            if not rows:
                retained.append(node)
                continue
            speed = np.linalg.norm(gyro, axis=1)
            # Exclude code-limit samples from fitting, not from live fusion or
            # the source recording. Weak motion falls back per node.
            raw_gyro = np.array([r[1] for r in rows]) / gyro_scale(self.config, node)
            eligible = (speed > 3.) & (np.max(np.abs(raw_gyro), axis=1) < 32700)
            moving = gyro[eligible]
            if len(moving) < 8:
                retained.append(node)
                continue
            _, singular, vectors = np.linalg.svd(moving, full_matrices=False)
            energy = float(singular[0] ** 2 / np.sum(singular ** 2))
            if energy < .55:
                retained.append(node)
                continue
            axis = vectors[0]
            projection = gyro @ axis
            # The first instructed movement is closing the hand. Use its first
            # sustained bout; one oppositely signed spike cannot define it.
            sign = _first_motion_sign(projection, eligible, self.config["sample_rate_hz"])
            if sign is None:
                retained.append(node)
                continue
            axis = sign * axis
            try:
                mount = _mount(self.neutral_acc[node], axis)
            except ValueError:
                retained.append(node)
                continue
            self.mounts[node] = mount
            self.axes_source[node] = "motion_pca_estimate"
            self.direction_source[node] = "first_sustained_instructed_close_motion"
            self.axes_evidence[node] = dict(principal_energy_fraction=energy, moving_samples=len(moving),
                                           sign_basis="first_sustained_instructed_close_motion", calibrated=False)
            learned.append(node)
        palm_rows = capture["rows"].get(palm, [])
        if palm_rows and np.percentile([np.linalg.norm(r[1] - self.fusion.bias[palm]) for r in palm_rows], 90) > 10:
            notes.append("动作学习期间手掌也有明显转动，独立 PCA 轴可能混入整手运动；仍只作为估计。")
        if retained:
            notes.append(f"节点 {retained} 本轮激励较弱或方向复杂，保留原估计轴，不影响其他手指。")
        self._notes = notes
        self.message = f"动作学习结束：{len(learned)} 路采用运动估计轴，{len(retained)} 路保留原轴。原张手参考保留；请张开检查方向。"

    def _neutral_splay(self, node):
        for finger in self.model_config["fingers"].values():
            if node in (finger["proximal"], finger["distal"]):
                angle = finger["neutral_splay_deg"] * (-1 if self.config["hand"] == "left" else 1)
                return from_axis_angle([0, 0, 1], math.radians(angle))
        return np.array([1., 0., 0., 0.])

    def _finish_alignment(self, capture):
        palm = self.config["palm_node"]
        rows = capture["rows"].get(palm, [])
        if not rows:
            self.message = "整手坐标对齐没有收到手掌数据，保留原估计轴。"
            return
        gyro = np.array([r[1] - self.fusion.bias[palm] for r in rows])
        eligible = ((np.linalg.norm(gyro, axis=1) > 3.) &
                    (np.max(np.abs(np.array([r[1] for r in rows]) / gyro_scale(self.config, palm)), axis=1) < 32700))
        if np.count_nonzero(eligible) < 12:
            self.message = "整手坐标对齐未形成足够翻掌动作，保留原估计轴；不是新的零位。"
            return
        _, _, vectors = np.linalg.svd(gyro[eligible], full_matrices=False)
        axis = vectors[0]
        sign = _first_motion_sign(gyro @ axis, eligible, self.config["sample_rate_hz"])
        if sign is None:
            self.message = "未识别到持续的首次向下轻点动作，保留原估计轴。"
            return
        try:
            palm_mount = _mount(self.neutral_acc[palm], sign * axis)
        except ValueError:
            self.message = "翻掌轴与重力平行，安装航向不可确定；保留原估计轴，请上下轻点而不是水平转圈。"
            return
        updates = {palm: palm_mount}
        evidence = {palm: dict(sign_basis="first_sustained_palm_nod", calibrated=False)}
        notes, retained = [], []
        for node in sorted(self.required_nodes - {palm}):
            pairs = _paired_rows(capture["rows"].get(node, []), rows,
                                 max(.03, 3. / self.config["sample_rate_hz"]))
            try:
                registration, quality = _direction_registration(pairs, self.fusion.bias[node], self.fusion.bias[palm],
                                                                gyro_scale(self.config, node), gyro_scale(self.config, palm))
            except ValueError as exc:
                retained.append(node)
                notes.append(f"节点 {node} 未完成共同坐标对齐，保留旧估计轴：{exc}。手掌坐标已更新，该节点仍可能出现伪弯曲。")
                continue
            # M_i = R0_i^-1 M_p R_(p<-i). All segments now share the
            # same physical neutral frame; a common rotation cancels exactly
            # in relative joints when the co-motion registration is correct.
            updates[node] = normalize(mul(conj(self._neutral_splay(node)), mul(palm_mount, registration)))
            quality["sensor_to_palm_wxyz"] = registration.tolist()
            evidence[node] = quality
            if quality["gyro_direction_residual_p95_deg"] > 20:
                notes.append(f"节点 {node} 活动陀螺方向残差较大，可能在翻掌时弯了手指；当前仍只是估计。")
        self.mounts.update(updates)
        self.axes_evidence.update(evidence)
        self.axes_source.update({node: "common_frame_alignment_estimate" for node in updates})
        self.direction_source.update({node: "first_sustained_palm_nod" for node in updates})
        self._notes = notes
        self.message = f"整手坐标对齐完成：手掌及 {len(updates) - 1} 路手指共用坐标，{len(retained)} 路保留原轴；张手参考没有改变。请保持手形转腕，再逐指弯伸检查。"

    def _orientations(self):
        result = {}
        left = self.config["hand"] == "left"
        splay = {self.config["palm_node"]: np.array([1., 0., 0., 0.])}
        for finger in self.model_config["fingers"].values():
            q = from_axis_angle([0, 0, 1], math.radians(finger["neutral_splay_deg"] * (-1 if left else 1)))
            splay[finger["proximal"]] = splay[finger["distal"]] = q
        for node, zero in self.zero.items():
            state = self.fusion.states.get(node, {})
            if "quat" not in state:
                continue
            mount = self.mounts[node]
            delta = mul(conj(zero), state["quat"])
            physical = normalize(mul(splay[node], mul(mul(mount, delta), conj(mount))))
            result[node] = physical * [1., 1., -1., -1.] if left else physical
        return result

    def get_snapshot(self, now=None):
        now = time.monotonic() if now is None else float(now)
        fresh = self.fusion._fresh(now)
        if self.active and self.fusion.last_time is not None and now - self.fusion.last_time > self.config["stale_after_s"]:
            self.cancel_calibration()
            self.message = "数据已停止，本次引导取消；未改动原参考。"
        nodes = {}
        for node in range(1, 13):
            state = self.fusion.states.get(node, {})
            stamps = state.get("times", [])
            span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
            nodes[str(node)] = dict(fresh=node in fresh,
                age_s=max(0., now - state["time"]) if "time" in state else None,
                gyro_dps=state.get("gyro", np.zeros(3)).tolist(), accel_g=state.get("acc", np.zeros(3)).tolist(),
                quat_wxyz=state.get("quat", np.array([1., 0, 0, 0])).tolist(), samples=state.get("count", 0),
                rate_hz=(len(stamps) - 1) / span if span > 0 and node in fresh else 0.,
                reference_ready=node in self.zero, axes_source=self.axes_source.get(node), axes_calibrated=False)
            nodes[str(node)]["direction_source"] = self.direction_source.get(node)
            nodes[str(node)]["common_frame_aligned"] = self.axes_source.get(node) == "common_frame_alignment_estimate"
        # HandModel's historical parameter is called calibrated_axes. Here it
        # only enables geometry for available ESTIMATED mounts; all public
        # outputs below explicitly deny anatomical/axis calibration.
        model = self.model.solve(self._orientations(), fresh, set(self.mounts), self.neutral_ready)
        # HandModel's hardcoded wrist outline endpoints are the last pieces of
        # its old mirrored template; finger bases have already been corrected.
        for index in (0, -2, -1):
            model["palm_points"][index][0] *= -1
        model.update(quick_mode=True, axes_calibrated=False, anatomical_accuracy_verified=False,
                     coordinate_convention="anatomical_direct_v1")
        for name, finger in model["fingers"].items():
            mapping = self.config["fingers"][name]
            finger["availability_reason"] = finger["quality"]
            finger["quality"] = "quick_estimate" if finger["geometry_live"] else "unavailable"
            finger["axes_source"] = {part: self.axes_source.get(mapping[part]) for part in ("proximal", "distal")}
            finger["direction_source"] = {part: self.direction_source.get(mapping[part]) for part in ("proximal", "distal")}
            finger["common_frame_aligned"] = all(self.axes_source.get(node) == "common_frame_alignment_estimate"
                                                  for node in (mapping["proximal"], mapping["distal"], self.config["palm_node"]))
            finger["axes_calibrated"] = False
            finger["anatomical_accuracy_verified"] = False
        active = None if self.active is None else dict(kind=self.active["kind"], remaining_s=max(0., self.active["end"] - now))
        warnings = ["快速全手原型：重力不能确定安装航向，未学习的板轴方向使用显式假设。",
                    "六轴航向可能漂移；整体姿态相对本次张手参考，不提供全局位置。",
                    "手指角度及尺寸为估计，四指末节由模型推算；拇指不是经验证的解剖关节角。"]
        if not self.config["units_verified"]:
            warnings.append("IMU 单位尚未获得固件确认，使用当前逐节点配置。")
        warnings.extend(self._notes)
        missing_reference = sorted(self.required_nodes - self.zero.keys()) if self.neutral_ready else []
        if missing_reference:
            warnings.append(f"节点 {missing_reference} 参考失效，需手动张手参考；不自动拉直。")
        return dict(model=model, nodes=nodes, quick_mode=True, warnings=warnings,
                    calibration=dict(neutral_ready=self.neutral_ready, active=active, message=self.message,
                                     axis_nodes=sorted(self.mounts), axes_calibrated=False,
                                     reference_missing_nodes=missing_reference),
                    gap_count=self.fusion.gap_count, invalid_count=self.fusion.invalid_count,
                    timing_basis="configured_nominal_period_no_device_timestamps")
