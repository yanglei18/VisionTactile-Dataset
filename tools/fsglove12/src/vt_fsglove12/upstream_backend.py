"""Official FSGlove inference with an explicitly labelled sparse-input adapter.

Installation calibration and anatomical FK use the pinned Raw2MANOCalibrator,
with a directed installation frame for the left hand. MANO composition reuses its original AxisLayerFK with the
same conversion/root operations, avoiding a second identical FK pass. Missing
IMUs are NOT observations: their virtual inputs algebraically invert that
calibrator so the missing anatomical local rotation equals a declared prior.
New profiles couple missing four-finger DIP flexion to observed PIP flexion;
old v1 profiles keep their zero prior for reproducible playback. No replacement
pose optimizer is used here.
"""
from __future__ import annotations

from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import quat, validate_rest
from .config import FINGER_DIPS, check_proposed_mount, digest, validate_config
from .installation_frame import REVISION as LEFT_INSTALLATION_POLICY
from .neutral_heading import apply_heading, fit_confirmed_heading
from .upstream_inference import (
    CalibrationRotationCache, EXECUTION_REVISION, calibration_rotations, official_pose_outputs,
)


OFFICIAL_STEPS = (
    {"key": "base_pose", "title": "1. 官方水平基准", "duration_s": 2.0,
     "instruction": "五指伸直，按官方 HORIZONTAL 姿态保持手掌水平，静止 2 秒。"},
    {"key": "z_rot_pose_1", "title": "2. 官方轴标定 · 指尖上倾", "duration_s": 2.0,
     "instruction": "五指保持伸直，使整掌指尖向上倾约 50°，静止 2 秒。对应官方 VERTICAL_UP 输入；不要勉强到疼痛。"},
    {"key": "z_rot_pose_2", "title": "3. 官方轴标定 · 指尖下倾", "duration_s": 2.0,
     "instruction": "五指保持伸直，沿同一转轴使指尖向下倾约 50°，静止 2 秒；不是拇指侧上抬，不要勉强到疼痛。"},
)
STEP_KEYS = tuple(step["key"] for step in OFFICIAL_STEPS)
MIN_PALM_TILT_DEG = 10.
MAX_RIGID_RESIDUAL_DEG = 15.
LEGACY_ALGORITHM = "official_fsglove_raw2mano_with_explicit_sparse_adapter_v1"
ALGORITHM = "official_fsglove_raw2mano_with_explicit_sparse_adapter_v2"
FUNCTIONAL_ALGORITHM = "official_fsglove_with_qmt_functional_adapter_v1"
ZERO_POLICY = "fixed_zero_anatomical_local_rotation"
COUPLED_POLICY = "pip_coupled_dip_with_neutral_other_missing"
_STATE_SHAPES = {
    "_base_quat": (16, 4), "_delta_rot_fa_rotvec": (16, 3),
    "_global_installation_error": (16, 3), "_local_installation_error": (16, 3),
    "_joint_fa": (16,), "_fk_seq": (15,),
}


RIGHT_INSTALLATION_POLICY = "upstream_original_frame_v1"


class OfficialCalibrationError(ValueError):
    def __init__(self, message, *, redo_steps=()):
        super().__init__(message)
        self.redo_steps = tuple(redo_steps)


def _array(value, shape, name):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite array of shape {shape}")
    return result


def complete_missing_quaternions(calibrator, raw_quaternions, observed_mask, *, rotation_cache=None):
    """Invert ONLY missing inputs; all measured quaternion bytes are retained.

    Upstream evaluates A_i = C_i^-1 G_i (B_i^-1 Q_i) L_i^-1,
    C_i = FK_parent D_i.  Setting Q_i = B_i G_i^-1 C_i L_i makes
    A_i the identity.  The root must be measured; no previous frame is used.
    Synthetic calibration gauge choices therefore cancel for missing inputs.
    """
    result = _array(raw_quaternions, (16, 4), "raw_quaternions").copy()
    observed = np.asarray(observed_mask)
    if observed.shape != (16,) or observed.dtype != np.bool_:
        raise ValueError("observed_mask must contain 16 booleans")
    if not observed[0]:
        raise ValueError("a measured root is required")
    for slot in np.flatnonzero(observed):
        quat(result[slot], f"observed slot {slot}")
    if observed.all():
        return result
    transforms = calibration_rotations(calibrator, rotation_cache)
    base, global_error, local_error = transforms.base, transforms.global_error, transforms.local_error
    rest_delta = transforms.rest_delta
    fk = [None] * 16
    fk[0] = global_error[0] * transforms.base_inv[0] * Rotation.from_quat(result[0]) * transforms.local_error_inv[0]
    for slot in calibrator._fk_seq:
        parent = int(calibrator._joint_fa[slot])
        current = fk[parent] * rest_delta[slot]
        if observed[slot]:
            fk[slot] = global_error[slot] * transforms.base_inv[slot] * Rotation.from_quat(result[slot]) * transforms.local_error_inv[slot]
        else:
            result[slot] = (base[slot] * transforms.global_error_inv[slot] * current * local_error[slot]).as_quat()
            fk[slot] = current
    return result


def _capture_node(captures, node, stale_after, session_id):
    means, counts, epochs = [], [], set()
    for key in STEP_KEYS:
        values, seen = [], set()
        for frame in captures[key]:
            if frame.get("session_id", session_id) != session_id:
                raise ValueError("calibration captures belong to a different session")
            data = frame.get("nodes", {}).get(node)
            if not isinstance(data, dict):
                continue
            protection = data.get("input_filter", {})
            if (data.get("orientation_valid") is False or data.get("input_health", {}).get("healthy") is False
                    or protection.get("gyro_rejected")
                    or protection.get("accel_rejected")):
                continue
            age, epoch, index = data.get("age_s"), data.get("epoch"), data.get("sample_index")
            if (not isinstance(age, (int, float)) or isinstance(age, bool)
                    or not math.isfinite(age) or not 0 <= age <= stale_after):
                continue
            if type(epoch) is not int or epoch < 0 or type(index) is not int or index <= 0:
                continue
            if (epoch, index) in seen:
                continue
            seen.add((epoch, index))
            values.append(quat(data.get("quat_xyzw"), f"node {node}").as_quat())
            epochs.add(epoch)
        if len(values) < 3:
            raise OfficialCalibrationError(f"{key} requires at least 3 fresh unique samples", redo_steps=(key,))
        # q and -q encode the same orientation. Preserve the first sample's
        # sign so same-hemisphere archives reproduce the upstream mean exactly.
        values = np.asarray(values)
        values[values @ values[0] < 0.] *= -1.
        mean = np.mean(values, axis=0)
        if np.linalg.norm(mean) < 1e-8:
            raise ValueError("quaternion average is degenerate; recapture the held pose")
        means.append(mean / np.linalg.norm(mean))
        counts.append(len(values))
    if len(epochs) != 1:
        raise OfficialCalibrationError("filter epoch changed during calibration; recapture all three poses",
                                       redo_steps=STEP_KEYS)
    # These are numerical singularity checks, not a claimed anatomical validation.
    rotation = (Rotation.from_quat(means[2]) * Rotation.from_quat(means[1]).inv()).as_rotvec()
    if np.linalg.norm(rotation) < 1e-6 or np.linalg.norm(rotation[:2]) < 1e-6:
        raise OfficialCalibrationError("official calibration rotation axis is degenerate or parallel to gravity",
                                       redo_steps=STEP_KEYS[1:])
    return means, next(iter(epochs)), counts


def _serialize_calibrator(calibrator):
    return {name: np.asarray(getattr(calibrator, name)).tolist() for name in _STATE_SHAPES}


def _rigid_motion_residuals(palm_turns, node_turns):
    """Compare both measured rotations using one shared yaw offset.

    Sensor mount rotations cancel in each change from the flat pose. The
    six-axis filters can still differ by a constant world-Z heading. Align
    the horizontal components of the two rotation vectors jointly; fitting a
    different offset at each pose would conceal inconsistent sensor motion.
    """
    px, py = palm_turns[:, 0], palm_turns[:, 1]
    nx, ny = node_turns[:, 0], node_turns[:, 1]
    yaw = np.arctan2(nx @ py - ny @ px, nx @ px + ny @ py)
    aligned = Rotation.from_rotvec(Rotation.from_euler("z", yaw).apply(node_turns))
    errors = (Rotation.from_rotvec(palm_turns).inv() * aligned).magnitude()
    return dict(zip(STEP_KEYS[1:], np.rad2deg(errors).tolist()))


def calibration_motion_diagnostics(calibration_inputs, measured_slots, root_node):
    """Check observability of the palm tilt and finger-to-palm rigid motion."""
    rotations = [Rotation.from_quat(calibration_inputs[key]) for key in STEP_KEYS]
    up = (rotations[1] * rotations[0].inv()).as_rotvec()
    down = (rotations[2] * rotations[0].inv()).as_rotvec()
    transition = (rotations[2] * rotations[1].inv()).as_rotvec()
    palm_turns = np.array([up[0], down[0]])
    entries = {}
    for slot in measured_slots:
        first, second, total = map(float, (np.linalg.norm(up[slot]), np.linalg.norm(down[slot]),
                                           np.linalg.norm(transition[slot])))
        opposite = (float(np.rad2deg(np.arccos(np.clip(
            -np.dot(up[slot], down[slot]) / (first * second), -1., 1.))))
            if min(first, second) > 1e-6 else None)
        gravity_tilt = (float(np.rad2deg(np.arcsin(np.clip(abs(transition[slot, 2]) / total, 0., 1.))))
                        if total > 1e-6 else None)
        entries[str(slot)] = dict(up_from_base_deg=float(np.rad2deg(first)),
            down_from_base_deg=float(np.rad2deg(second)), opposite_axis_error_deg=opposite,
            up_to_down_deg=float(np.rad2deg(total)),
            rotation_axis_gravity_tilt_deg=gravity_tilt,
            rigid_residual_deg=_rigid_motion_residuals(palm_turns, np.array([up[slot], down[slot]])))
    root = entries["0"]
    messages = []
    # Q_base Q_hold^-1 maps the up vector observed in each hold into the
    # same flat-pose frame. World yaw at the hold cannot change that vector.
    directions = Rotation.from_rotvec(palm_turns).apply([0., 0., 1.], inverse=True)
    tilts = np.rad2deg(np.arctan2(np.linalg.norm(directions[:, :2], axis=1), directions[:, 2]))
    root["gravity_excursion_deg"] = dict(zip(STEP_KEYS[1:], tilts.tolist()))
    retry = [key for key, tilt in root["gravity_excursion_deg"].items() if tilt < MIN_PALM_TILT_DEG]
    if not retry:
        horizontal = directions[:, :2]
        if horizontal[0] @ horizontal[1] > .5 * np.prod(np.linalg.norm(horizontal, axis=1)):
            # Both captures tilted to substantially the same side (within 60 degrees).
            retry = list(STEP_KEYS[1:])
    if root["opposite_axis_error_deg"] is not None and root["opposite_axis_error_deg"] > 15.:
        messages.append(f"上倾/下倾不是反向同轴：手掌 {root_node} 偏差 {root['opposite_axis_error_deg']:.1f}°；"
                        "可能把弯曲混成侧摆。结果保留，建议保持五指伸直、同一手腕轴，只重采第 2/3 步。")
    if min(root["up_from_base_deg"], root["down_from_base_deg"]) < 15.:
        messages.append("至少一次整掌转动不足 15°；轴向容易受噪声影响，可只重采该姿势，不必重连或清空标定。")
    if root["up_to_down_deg"] < 90.:
        messages.append(f"上倾到下倾总转角 {root['up_to_down_deg']:.1f}°，小于官方建议的 90°；"
                        "结果保留。请在舒适范围内增加整掌同轴转动，不要改成弯手指。")
    if root["rotation_axis_gravity_tilt_deg"] is not None and root["rotation_axis_gravity_tilt_deg"] > 15.:
        messages.append(f"转掌轴偏离水平约 {root['rotation_axis_gravity_tilt_deg']:.1f}°；"
                        "请避免同时侧翻或改变初始掌面。此提示不是实物角度精度评价。")
    return dict(kind="three_pose_motion_consistency_not_accuracy", root_node=root_node,
                root=root, by_slot=entries, warnings=messages, blocking=bool(retry), redo_steps=retry,
                min_palm_tilt_deg=MIN_PALM_TILT_DEG, max_rigid_residual_deg=MAX_RIGID_RESIDUAL_DEG,
                thresholds_are_capture_guidance_not_anatomical_limits=True)


class OfficialFSGloveBackend:
    """Session/epoch-bound wrapper; construction never opens a device or a GUI."""

    def __init__(self, config, rest, *, checkout, mano_assets_root, core=None):
        self.config = validate_config(deepcopy(config))
        self._anatomical_root_in_mano = Rotation.from_quat(validate_rest(rest)[0])
        self.rest = deepcopy(rest)
        self._missing_policy = self.config.get("upstream_missing_slot_policy", COUPLED_POLICY)
        if self._missing_policy not in (ZERO_POLICY, COUPLED_POLICY):
            raise ValueError("upstream_missing_slot_policy must select fixed zero or PIP-coupled DIP")
        max_dip = self.config.get("upstream_dip_max_rad", 1.65)
        if (isinstance(max_dip, bool) or not isinstance(max_dip, (int, float))
                or not math.isfinite(max_dip) or not 0 < max_dip < np.pi):
            raise ValueError("upstream_dip_max_rad must be finite in (0, pi)")
        self._prior_options = dict(dip_ratio=self.config["dip_ratio"],
            dip_ratio_by_finger=deepcopy(self.config.get("dip_ratio_by_finger", {})),
            upstream_dip_max_rad=float(max_dip))
        self._shape_params = _array(self.config.get("upstream_shape_params", [0.] * 10),
                                    (10,), "upstream_shape_params").tolist()
        self._shape_calibrated = self.config.get("upstream_shape_calibrated", False)
        if type(self._shape_calibrated) is not bool:
            raise ValueError("upstream_shape_calibrated must be an explicit boolean")
        if self._shape_calibrated and "upstream_shape_params" not in self.config:
            raise ValueError("upstream_shape_calibrated requires supplied upstream_shape_params")
        from .config import check_model_hand
        check_model_hand(self.config, self.rest)
        if core is None:
            from .upstream_core import load_upstream_core
            core = load_upstream_core(checkout, mano_assets_root)
        self.core = core
        self._installation_policy = (LEFT_INSTALLATION_POLICY if self.config["hand"] == "left"
                                     else RIGHT_INSTALLATION_POLICY)
        self._config_sha, self._rest_sha = digest(self.config), digest(self.rest)
        self._root_node = next(n for n, slot in self.config["node_to_slot"].items() if slot == 0)
        self._configured_slots = frozenset(self.config["node_to_slot"].values())
        self._cached_key, self._cached_calibrator = None, None
        self._rotation_cache = CalibrationRotationCache()
        from .functional_pose import FunctionalPosePipeline
        self._functional_pipeline = FunctionalPosePipeline(self.config)
        self._live_heading = None

    def with_config(self, config):
        """Return an uncalibrated owner for new input semantics, sharing only core.

        Reusing the loaded official model avoids a second MANO initialization;
        installation caches and causal heading history must never cross a
        configuration change. Construction validates and deep-copies config and
        rest, so neither this backend nor caller-owned parameters are mutated.
        The controller is responsible for disconnecting and dropping its active
        profile before activating the returned backend.
        """
        return OfficialFSGloveBackend(config, self.rest, checkout=None,
                                      mano_assets_root=None, core=self.core)

    def shape_job_options(self):
        """Serializable source paths for an isolated official shape worker."""
        checkout = self.core.provenance.get("checkout")
        assets = getattr(self.core, "mano_assets_root", None)
        if not checkout or assets is None:
            raise ValueError("Official shape fitting requires trusted FSGlove and MANO source paths")
        return dict(checkout=str(checkout), mano_assets=str(assets), iterations=500, seed=0,
                    hand=self.config["hand"])

    def reset_calibration(self):
        """Discard calibration caches and pose history without unloading the core.

        The controller owns the active profile and captured samples and must
        clear those separately. This idempotent operation neither mutates
        caller-owned profiles nor deletes saved calibration files. A later
        fit, or an explicitly supplied saved profile, builds a new calibrator.
        """
        self._cached_key, self._cached_calibrator = None, None
        self._rotation_cache.clear()
        self._functional_pipeline.reset()
        self._live_heading = None

    def reset(self):
        """Alias for explicitly clearing calibration-dependent runtime state."""
        self.reset_calibration()

    def fit_calibration(self, captures, *, session_id):
        check_proposed_mount(self.config)
        if not isinstance(session_id, str) or not session_id:
            raise ValueError("a nonempty session_id is required")
        if not isinstance(captures, dict) or any(not captures.get(key) for key in STEP_KEYS):
            raise OfficialCalibrationError("请依次采集官方三种标定姿态。", redo_steps=STEP_KEYS)
        # Explicit canonical virtual calibration, never copied from another IMU.
        # Its arbitrary gauge cancels in complete_missing_quaternions().
        canonical = Rotation.from_rotvec([[0., 0., 0.], [.6, 0., 0.], [-.6, 0., 0.]]).as_quat()
        arrays = np.repeat(canonical[:, None, :], 16, axis=1)
        nodes, rejected, retry = {}, {}, set()
        for node, slot in self.config["node_to_slot"].items():
            try:
                means, epoch, counts = _capture_node(captures, node, self.config["stale_after_s"], session_id)
                arrays[:, slot] = means
                nodes[node] = {"slot": slot, "epoch": epoch, "frames_per_pose": counts}
            except (ValueError, TypeError, KeyError) as exc:
                steps = getattr(exc, "redo_steps", STEP_KEYS)
                if node == self._root_node:
                    raise OfficialCalibrationError(f"官方手掌标定失败：{exc}", redo_steps=steps) from exc
                rejected[node] = str(exc)
                retry.update(steps)
        calibration_inputs = dict(zip(STEP_KEYS, arrays.tolist()))
        motion_quality = calibration_motion_diagnostics(
            calibration_inputs, [entry["slot"] for entry in nodes.values()], self._root_node)
        if motion_quality["blocking"]:
            raise OfficialCalibrationError("手掌没有提供可用的相反倾转；平掌记录已保留。"
                "五指伸直，分别向上、向下倾转手腕，重采提示的姿势。",
                redo_steps=motion_quality["redo_steps"])
        rigid_rejections = {}
        for node, entry in list(nodes.items()):
            slot = entry["slot"]
            residuals = motion_quality["by_slot"][str(slot)]["rigid_residual_deg"]
            bad = {step: error for step, error in residuals.items() if error > MAX_RIGID_RESIDUAL_DEG}
            if bad:
                rigid_rejections[node] = list(bad)
                rejected[node] = f"节点 {node} 的倾转与手掌不一致（最大差 {max(bad.values()):.1f}°）；请保持手指伸直后重采。"
                retry.update(bad)
                arrays[:, slot] = canonical
                del nodes[node]
        motion_quality["rigid_rejections"] = rigid_rejections
        motion_quality["redo_steps"] = [key for key in STEP_KEYS if key in retry]
        calibration_inputs = dict(zip(STEP_KEYS, arrays.tolist()))
        calibrator = self.core.create_calibrator(**calibration_inputs, side=self.config["hand"], seed=0)
        state = _serialize_calibrator(calibrator)
        for name, shape in _STATE_SHAPES.items():
            _array(state[name], shape, name)
        measured_slots = sorted(entry["slot"] for entry in nodes.values())
        virtual_slots = [slot for slot in range(16) if slot not in measured_slots]
        profile = {
            "schema_version": 1, "method": ALGORITHM, "status": "official_core_sparse_adapter_not_accuracy_validated",
            "session_id": session_id, "hand": self.config["hand"], "config_sha256": self._config_sha,
            "rest_sha256": self._rest_sha, "stale_after_s": self.config["stale_after_s"],
            "nodes": nodes, "rejected_nodes": rejected, "calibration_inputs": calibration_inputs,
            "upstream_state": state, "upstream_state_sha256": digest(state),
            "upstream_provenance": deepcopy(self.core.provenance),
            "installation_coordinate_policy": self._installation_policy,
            "calibration_seed": 0, "measured_calibration_slots": measured_slots,
            "virtual_calibration_slots": virtual_slots,
            "virtual_calibration_kind": "explicit_synthetic_canonical_gauge_not_observation",
            "missing_slot_policy": self._missing_policy,
            "missing_slot_options": deepcopy(self._prior_options),
            "calibration_motion_diagnostics": motion_quality,
            "shape_params": list(self._shape_params), "shape_calibrated": self._shape_calibrated,
            "shape_provenance": deepcopy(self.config.get("upstream_shape_provenance")),
            "anatomical_accuracy_verified": False, "yaw_drift_corrected": False,
            "warnings": [("使用官方 Raw2MANO 核心；四指缺测末节按中节联动推算，其他缺测骨段保持中立先验，均非实测。"
                          if self._missing_policy == COUPLED_POLICY else
                          "使用官方 Raw2MANO 核心；缺测关节固定零位先验，不代表测得关节角。"),
                         ("已载入用户声明经官方触指优化得到的形状参数；仍未校正六轴 IMU 持续航向漂移。"
                          if self._shape_calibrated else
                          "当前未声明完成手形触指优化，也未校正六轴 IMU 持续航向漂移。")],
            "limitations": ["The 12-channel hardware adapter is not the original 16-IMU configuration.",
                            "Missing DIP coupling is a configurable kinematic prior, not an independent measurement; other missing slots remain neutral.",
                            "Official three-pose assumptions are not independently verified anatomical ground truth.",
                            "Public Raw2MANO calibration is not a joint pose/shape/misalignment differentiable optimizer.",
                            "Calibration is valid only for the same session and node filter epochs."],
        }
        profile["warnings"] = motion_quality["warnings"] + profile["warnings"]
        self._cached_key, self._cached_calibrator = profile["upstream_state_sha256"], calibrator
        return profile

    def _calibrator(self, profile):
        if profile.get("hand", "right") != self.config["hand"]:
            raise ValueError("profile hand does not match the configured hand")
        if profile.get("method") not in (ALGORITHM, LEGACY_ALGORITHM):
            raise ValueError("profile is not an official FSGlove sparse-adapter profile")
        if profile.get("config_sha256") != self._config_sha or profile.get("rest_sha256") != self._rest_sha:
            raise ValueError("profile configuration or MANO model digest does not match")
        # The original right-hand frame is compatible with its historical profiles.
        # Legacy left-hand matrices cannot be converted by relabelling their axes.
        legacy_policy = RIGHT_INSTALLATION_POLICY if self.config["hand"] == "right" else None
        if profile.get("installation_coordinate_policy", legacy_policy) != self._installation_policy:
            raise ValueError("Installation coordinate policy changed; repeat the three mounting poses")
        from .upstream_core import upstream_provenance_matches
        if not upstream_provenance_matches(profile.get("upstream_provenance"), self.core.provenance):
            raise ValueError("profile upstream source provenance does not match")
        if profile["method"] == LEGACY_ALGORITHM:
            if profile.get("missing_slot_policy") != ZERO_POLICY:
                raise ValueError("v1 profiles must retain their original fixed-zero policy")
        elif (profile.get("missing_slot_policy") != self._missing_policy
              or profile.get("missing_slot_options") != self._prior_options):
            raise ValueError("saved sparse prior policy does not match the current configuration")
        state = profile.get("upstream_state", {})
        key = digest(state)
        if key != profile.get("upstream_state_sha256"):
            raise ValueError("saved official calibration state digest does not match")
        if self._cached_key == key:
            return self._cached_calibrator
        restored = {name: _array(state.get(name), shape, name).copy() for name, shape in _STATE_SHAPES.items()}
        expected_parents = np.asarray([0, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14])
        if not np.array_equal(restored["_joint_fa"], expected_parents):
            raise ValueError("saved official parent hierarchy is invalid")
        sequence = restored["_fk_seq"].astype(int)
        if sorted(sequence.tolist()) != list(range(1, 16)):
            raise ValueError("saved official FK sequence is invalid")
        seen = {0}
        for slot in sequence:
            if int(expected_parents[slot]) not in seen:
                raise ValueError("saved official FK sequence is not topological")
            seen.add(int(slot))
        for value in restored["_base_quat"]:
            quat(value, "saved base quaternion")
        calibrator = self.core.create_calibrator(**profile["calibration_inputs"], side=self.config["hand"], seed=profile["calibration_seed"])
        for name, array in restored.items():
            setattr(calibrator, name, array.astype(int) if name in ("_joint_fa", "_fk_seq") else array)
        # Save and restore actual sampled rest-frame matrices; never rely on decode().
        self._cached_key, self._cached_calibrator = key, calibrator
        return calibrator

    def _functional_binding(self, profile):
        """Tie each manual joint correction to the unchanged installation fit."""
        from .upstream_functional import functional_dependencies
        return dict(session_id=profile["session_id"],
                    upstream_state_sha256=profile["upstream_state_sha256"],
                    config_sha256=self._config_sha, rest_sha256=self._rest_sha,
                    nodes_epochs={node: data["epoch"] for node, data in profile["nodes"].items()},
                    dependency_nodes_by_slot=functional_dependencies(self.config, profile))

    def reanchor_heading(self, rows, profile, *, session_id):
        """Fit an explicit straight-hand heading observation without altering live state."""
        if not profile or session_id != profile.get("session_id"):
            raise ValueError("伸直航向参考需要本次会话已生效的标定。")
        return fit_confirmed_heading(rows, profile, self._calibrator(profile), self._functional_binding(profile),
                                     self.config["stale_after_s"])

    def _functional_capture_results(self, capture_groups, profile, *, session_id, reference_selection=None):
        """Run each captured frame through the original solver exactly once.

        Result lists can be shared by several independent joint fits. Sample
        deduplication belongs to capture conversion, not to the finger loop:
        all four fingers must see the same simultaneous movement evidence.
        """
        if not profile or session_id != profile.get("session_id"):
            raise ValueError("功能轴校准需要当前会话的官方三步标定。")
        self._calibrator(profile)
        if any(not isinstance(frames, (list, tuple)) for frames in capture_groups):
            raise ValueError("请显式采集伸直静止与各段弯伸动作。")
        base_profile = {**profile, "functional_enabled": False}
        binding = self._functional_binding(profile)
        seen = {node: set() for node in profile["nodes"]}

        def convert(frames, *, reference=False):
            results = []
            for frame in frames:
                if not isinstance(frame, dict) or frame.get("session_id") != session_id:
                    raise ValueError("功能轴采集与当前会话不一致，原标定保留。")
                # Calibration uses unmodified official poses, but must not
                # reset the LIVE estimator of another finger while fitting.
                pose = self._solve_frame(frame, base_profile, functional=False)
                if not pose["pose_available"]:
                    continue
                fresh_nodes = set()
                for node, data in frame.get("nodes", {}).items():
                    protection = data.get("input_filter", {})
                    if (data.get("orientation_valid") is False or protection.get("gyro_rejected")
                            or protection.get("accel_rejected")):
                        continue
                    index = data.get("sample_index")
                    key = (data.get("epoch"), index)
                    if node in seen and type(index) is int and index > 0 and key not in seen[node]:
                        fresh_nodes.add(node)
                        seen[node].add(key)
                joint_mask = list(pose["joint_observed_mask"])
                for slot, dependencies in binding["dependency_nodes_by_slot"].items():
                    joint_mask[int(slot)] &= all(node in fresh_nodes for node in dependencies)
                    if reference and reference_selection is not None:
                        joint_mask[int(slot)] &= slot in reference_selection["stable_slots"]
                results.append({**pose, "joint_observed_mask": joint_mask})
            return results

        return binding, [convert(frames, reference=index == 0) for index, frames in enumerate(capture_groups)]

    def functional_reference_rotations(self, frame, profile, slots):
        """Read measured local rotations in the exact functional-fit domain.

        Uses the cached official installation transforms without composing a
        MANO mesh, advancing heading estimators or altering a live profile.
        Independent sensor initial headings are aligned before pairing nodes.
        """
        if frame.get("session_id") != profile.get("session_id"):
            return {}
        calibrator = self._calibrator(profile)
        frame = apply_heading(frame, profile)
        transforms = calibration_rotations(calibrator, cache=self._rotation_cache)
        binding = self._functional_binding(profile)
        result = {}
        for slot in slots:
            dependencies = binding["dependency_nodes_by_slot"].get(str(slot), ())
            if len(dependencies) != 2:
                continue
            values, identities = [], []
            for node in dependencies:
                data = frame.get("nodes", {}).get(node, {})
                age = data.get("age_s", math.inf)
                protection = data.get("input_filter", {})
                if (data.get("epoch") != profile["nodes"][node]["epoch"]
                        or data.get("orientation_valid") is False
                        or data.get("input_health", {}).get("healthy") is False
                        or protection.get("gyro_rejected") or protection.get("accel_rejected")
                        or type(data.get("sample_index")) is not int or data["sample_index"] <= 0
                        or not isinstance(age, (int, float)) or isinstance(age, bool)
                        or not math.isfinite(age) or not 0. <= age <= self.config["stale_after_s"]):
                    break
                index = self.config["node_to_slot"][node]
                try:
                    rotation = quat(data.get("quat_xyzw"), f"node {node}")
                except (ValueError, TypeError):
                    break
                values.append(transforms.global_error[index] * transforms.base_inv[index]
                              * rotation * transforms.local_error_inv[index])
                identities.append((data["epoch"], data["sample_index"]))
            if len(values) == 2:
                local = (values[0] * transforms.rest_delta[slot]).inv() * values[1]
                result[str(slot)] = dict(quat_xyzw=local.as_quat().tolist(), identity=tuple(identities))
        return result

    def fit_functional_finger(self, neutral_frames, motion_frames, profile, *, finger, session_id,
                              reference_selection=None):
        """Fit only explicitly labelled motion; never reuse legacy calibration.

        Official poses remain the input to this optional model-coordinate
        adapter. No reference MANO angles are invented for the upstream
        ``correct_local_inst_error`` function, which requires known labels.
        """
        from .upstream_functional import FunctionalDependencyError, fit_functional_finger
        binding, (neutral, motion) = self._functional_capture_results(
            [neutral_frames, motion_frames], profile, session_id=session_id,
            reference_selection=reference_selection)
        try:
            return fit_functional_finger(neutral, motion, finger=finger, binding=binding,
                previous=profile.get("functional_fingers", {}).get(finger))
        except FunctionalDependencyError as exc:
            # Optional dependency failure must not stop acquisition or invalidate
            # the existing official installation profile.
            raise ValueError(str(exc)) from exc

    def fit_functional_hand(self, neutral_frames, finger_frames, thumb_frames, profile, *, session_id,
                            reference_selection=None):
        """Batch the existing per-finger fits without inventing a new solver.

        One shared neutral, one simultaneous four-finger flexion segment and
        one separate thumb-IP segment. Each finger retains the same individual
        quality checks and per-joint retention semantics. This function never
        mutates the supplied profile; the controller publishes successful
        entries together in one new version. Failed fingers are not entries.
        """
        from .upstream_functional import FINGER_SLOTS, FunctionalDependencyError, fit_functional_finger
        binding, (neutral, fingers, thumb) = self._functional_capture_results(
            [neutral_frames, finger_frames, thumb_frames], profile, session_id=session_id,
            reference_selection=reference_selection)
        entries, errors, failures = {}, {}, {}
        for finger in FINGER_SLOTS:
            try:
                entries[finger] = fit_functional_finger(neutral, thumb if finger == "thumb" else fingers,
                    finger=finger, binding=binding,
                    previous=profile.get("functional_fingers", {}).get(finger))
            except (ValueError, FunctionalDependencyError) as exc:
                errors[finger] = str(exc)
                failures[finger] = dict(diagnostics=deepcopy(getattr(exc, "diagnostics", {})),
                                        rejected_slots=deepcopy(getattr(exc, "rejected_slots", {})))
        return dict(entries=entries, errors=errors, failures=failures,
                    capture_protocol="shared_neutral_four_fingers_then_thumb_ip_v1",
                    input_frame_counts=dict(neutral=len(neutral_frames),
                                            four_fingers=len(finger_frames), thumb_ip=len(thumb_frames)))

    def _apply_functional(self, result, profile, calibrator, frame):
        return self._functional_pipeline.apply(result, profile, calibrator, frame,
            self._functional_binding(profile), self.core,
            coupled=profile["missing_slot_policy"] == COUPLED_POLICY,
            algorithm=FUNCTIONAL_ALGORITHM)

    def solve(self, frame, profile, *, timestamp_s=None):
        result = self._solve_frame(frame, profile, timestamp_s=timestamp_s, functional=True)
        if result["pose_available"]:
            # Upstream returns the root delta in anatomical axes. MANO takes a
            # global rotation in its native axes. Rotating the axis-angle vector
            # changes basis (C R C^-1), so a flat hand remains flat. The left
            # model's rest root is a half-turn about Y; the right's is identity.
            pose = result["mano_pose_rotvec"]
            root = self._anatomical_root_in_mano.apply(pose[0])
            result["mano_pose_rotvec"] = [root.tolist(), *pose[1:]]
            result["root_quat_xyzw"] = Rotation.from_rotvec(root).as_quat().tolist()
            result["mano_root_frame"] = "model_native_from_anatomical_rest_v1"
        return result

    def _base_joint_availability(self, rejected, parents, prior_diagnostic):
        """An absent configured segment invalidates its own and its child's local rotation."""
        missing = {self.config["node_to_slot"][node] for node in rejected}
        unavailable = set(missing)
        for slot in self._configured_slots - {0}:
            # For a parent with no IMU by design, the upstream fixed prior is
            # relative to the nearest instrumented ancestor.
            parent = int(parents[slot])
            while parent not in self._configured_slots:
                parent = int(parents[parent])
            if parent in missing:
                unavailable.add(slot)
        for finger, prior in prior_diagnostic.get("dip_by_finger", {}).items():
            if missing.intersection(prior.get("missing_dependency_slots", [])):
                unavailable.add(FINGER_DIPS[finger])
        return [slot not in unavailable for slot in range(16)]

    def _solve_frame(self, frame, profile, *, timestamp_s=None, functional):
        timestamp = frame.get("timestamp_s") if timestamp_s is None else timestamp_s
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise ValueError("timestamp_s must be finite")
        result = dict(timestamp_s=float(timestamp), algorithm=ALGORITHM, hand=self.config["hand"],
                      pose_available=False, solver_success=False, quality="unavailable",
                      reason="calibration_required", anatomical_euler_rad=None,
                      mano_pose_rotvec=None, root_quat_xyzw=None, predicted_global_xyzw=None,
                      observed_mask=[False] * 16, segment_observed_mask=[False] * 16,
                      joint_observed_mask=[False] * 16, inferred_mask=[False] * 16,
                      source_labels=["unavailable"] * 16, calibration_rejections={},
                      anatomical_accuracy_verified=False, yaw_drift_corrected=False,
                      shape_params=[0.] * 10, shape_calibrated=False)
        if not profile:
            if functional:
                self._functional_pipeline.reset()
            return result
        if not profile.get("session_id") or frame.get("session_id") != profile["session_id"]:
            if functional:
                self._functional_pipeline.reset()
            result["reason"] = "calibration_session_mismatch"
            return result
        calibrator = self._calibrator(profile)
        # The explicit neutral observation changes the heading origin. Live
        # hinge estimators now estimate residual heading in that new origin.
        frame = apply_heading(frame, profile)
        heading = profile.get("neutral_heading")
        if heading is not None:
            result["neutral_heading_applied"] = True
        if functional:
            key = (heading["binding"]["session_id"], tuple(sorted(heading["offsets_rad_by_node"].items()))) if heading else None
            if key != self._live_heading:
                self._functional_pipeline.reset()
                self._live_heading = key
        if "relative_heading_correction" in self.config:
            result["relative_heading_correction"] = dict(enabled=False,
                reason="superseded_by_unified_continuous_functional_hinges", applied_slots=[])
        result["shape_params"] = _array(profile.get("shape_params"), (10,), "saved MANO shape").tolist()
        result["shape_calibrated"] = profile.get("shape_calibrated") is True
        observed = np.zeros(16, dtype=bool)
        raw = np.tile([0., 0., 0., 1.], (16, 1))
        rejected = {}
        for node, slot in self.config["node_to_slot"].items():
            data, binding = frame.get("nodes", {}).get(node), profile.get("nodes", {}).get(node)
            if not binding:
                rejected[node] = "node_not_calibrated"
                continue
            if not isinstance(data, dict):
                rejected[node] = "missing_or_stale_node"
                continue
            if data.get("input_health", {}).get("healthy") is False:
                rejected[node] = "input_health_invalid"
                continue
            if data.get("epoch") != binding["epoch"]:
                rejected[node] = "filter_epoch_changed_recalibration_required"
                continue
            if data.get("orientation_valid") is False:
                rejected[node] = ("waiting_for_backlog" if data.get("continuity") == "awaiting_backlog"
                                  else "input_filter_prolonged_invalid")
                continue
            age = data.get("age_s", math.inf)
            if (isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age)
                    or not 0 <= age <= self.config["stale_after_s"]):
                rejected[node] = "missing_or_stale_node"
                continue
            try:
                quat(data.get("quat_xyzw"), f"node {node}")
                raw[slot] = data["quat_xyzw"]
                observed[slot] = True
            except (ValueError, TypeError):
                rejected[node] = "invalid_quaternion"
        result["calibration_rejections"] = rejected
        result["observed_mask"] = result["segment_observed_mask"] = observed.tolist()
        if not observed[0]:
            # A brief transport gap does not change the calibrated heading
            # reference. The hinge owner still checks elapsed source time,
            # epoch and binding when observations resume. Resetting here on
            # every stale tick allowed an unconstrained first-frame yaw update
            # repeatedly at high solve rates. Real reference faults still reset.
            if functional and rejected.get(self._root_node) not in ("missing_or_stale_node", "waiting_for_backlog"):
                self._functional_pipeline.reset()
            result["reason"] = ("calibration_epoch_changed" if rejected.get(self._root_node)
                                == "filter_epoch_changed_recalibration_required" else
                                "input_health_invalid" if rejected.get(self._root_node)
                                == "input_health_invalid" else "missing_or_stale_root")
            return result
        if profile["missing_slot_policy"] == COUPLED_POLICY:
            from .upstream_priors import complete_with_priors
            completed, prior_diagnostic = complete_with_priors(
                calibrator, raw, observed, profile["missing_slot_options"], rotation_cache=self._rotation_cache)
        else:
            completed = complete_missing_quaternions(calibrator, raw, observed, rotation_cache=self._rotation_cache)
            prior_diagnostic = dict(kind=ZERO_POLICY, coupled_dip_slots=[],
                                    zero_prior_slots=np.flatnonzero(~observed).tolist())
        rotations = Rotation.from_quat(completed)
        # Execute the unchanged official anatomical FK once, then the original
        # AxisLayer composer with upstream's exact conversion/root overwrite.
        mano_pose, anatomical, euler = official_pose_outputs(calibrator, rotations, self.core.torch)
        _array(mano_pose, (16, 3), "official MANO pose")
        _array(anatomical, (16, 3), "official anatomical rotation vectors")
        parents = np.asarray(calibrator._joint_fa, dtype=int)
        joint_observed = observed & observed[parents]
        virtual_slots = np.flatnonzero(~observed).tolist()
        coupled_slots = set(prior_diagnostic["coupled_dip_slots"])
        sources = ["official_measured_root" if slot == 0 else
                   "official_measured_parent_child" if joint_observed[slot] else
                   "official_measured_segment_with_inferred_parent" if observed[slot] else
                   "pip_coupled_dip_prior_not_observation" if slot in coupled_slots else
                   "explicit_fixed_zero_anatomical_prior" for slot in range(16)]
        result.update(pose_available=True, solver_success=True,
                      algorithm=profile["method"],
                      quality="not_accuracy_validated", reason="official_inference_complete",
                      upstream_fsglove_solver_executed=True,
                      adapter_execution=dict(revision=EXECUTION_REVISION,
                          official_anatomical_fk_calls=1, original_axis_compose=True,
                          pinned_numerical_ast_modified=False,
                          sparse_rotation_cache="calibrator_identity_and_state_content"),
                      upstream_commit=self.core.provenance["upstream_commit"],
                      anatomical_euler_rad=euler.tolist(), mano_pose_rotvec=mano_pose.tolist(),
                      joint_observed_mask=joint_observed.tolist(), inferred_mask=(~joint_observed).tolist(),
                      source_labels=sources, virtual_input_slots=virtual_slots,
                      virtual_input_kind="algebraic_completion_for_declared_prior_not_observation",
                      input_contract="official_full16" if not virtual_slots else "explicit_sparse_to16_adapter",
                      missing_slot_policy=profile["missing_slot_policy"],
                      sparse_prior=prior_diagnostic,
                      upstream_state_sha256=profile["upstream_state_sha256"])
        if rejected:
            available = self._base_joint_availability(rejected, parents, prior_diagnostic)
            result.update(joint_valid_mask=available, joint_display_available_mask=available.copy(),
                          joint_quality=["valid" if good else "unavailable" for good in available])
        return self._apply_functional(result, profile, calibrator, frame) if functional else result
