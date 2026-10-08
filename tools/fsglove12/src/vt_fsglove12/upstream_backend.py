"""Official FSGlove inference with an explicitly labelled sparse-input adapter.

Installation calibration and anatomical FK use the unmodified upstream
Raw2MANOCalibrator. MANO composition reuses its original AxisLayerFK with the
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
from .config import check_proposed_mount, digest, validate_config
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
            if (data.get("orientation_valid") is False or protection.get("gyro_rejected")
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
            raise ValueError(f"{key} requires at least 3 fresh unique samples")
        # Same normalized component-wise mean used in official get_average_rotation.
        mean = np.mean(values, axis=0)
        if np.linalg.norm(mean) < 1e-8:
            raise ValueError("quaternion average is degenerate; recapture the held pose")
        means.append(mean / np.linalg.norm(mean))
        counts.append(len(values))
    if len(epochs) != 1:
        raise ValueError("filter epoch changed during calibration; recapture all three poses")
    # These are numerical singularity checks, not a claimed anatomical validation.
    rotation = (Rotation.from_quat(means[2]) * Rotation.from_quat(means[1]).inv()).as_rotvec()
    if np.linalg.norm(rotation) < 1e-6 or np.linalg.norm(rotation[:2]) < 1e-6:
        raise ValueError("official calibration rotation axis is degenerate or parallel to gravity")
    return means, next(iter(epochs)), counts


def _serialize_calibrator(calibrator):
    return {name: np.asarray(getattr(calibrator, name)).tolist() for name in _STATE_SHAPES}


def calibration_motion_diagnostics(calibration_inputs, measured_slots, root_node):
    """Explain the public core's rigid, opposite-axis pose assumption.

    These are capture consistency diagnostics, not ground-truth accuracy, a
    different calibration algorithm, or an extra pass/fail gate. Independent
    IMU yaw gauges cancel in each node's angular self-consistency measures.
    """
    rotations = [Rotation.from_quat(calibration_inputs[key]) for key in STEP_KEYS]
    up = (rotations[1] * rotations[0].inv()).as_rotvec()
    down = (rotations[2] * rotations[0].inv()).as_rotvec()
    transition = (rotations[2] * rotations[1].inv()).as_rotvec()
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
            rotation_axis_gravity_tilt_deg=gravity_tilt)
    root = entries["0"]
    messages = []
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
                root=root, by_slot=entries, warnings=messages, blocking=False,
                thresholds_are_capture_guidance_not_anatomical_limits=True)


class OfficialFSGloveBackend:
    """Session/epoch-bound wrapper; construction never opens a device or a GUI."""

    def __init__(self, config, rest, *, checkout, mano_assets_root, core=None):
        self.config = validate_config(deepcopy(config))
        validate_rest(rest)
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
        self._config_sha, self._rest_sha = digest(self.config), digest(self.rest)
        self._root_node = next(n for n, slot in self.config["node_to_slot"].items() if slot == 0)
        self._cached_key, self._cached_calibrator = None, None
        self._rotation_cache = CalibrationRotationCache()
        from .functional_pose import FunctionalPosePipeline
        self._functional_pipeline = FunctionalPosePipeline(self.config)

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
        nodes, rejected = {}, {}
        for node, slot in self.config["node_to_slot"].items():
            try:
                means, epoch, counts = _capture_node(captures, node, self.config["stale_after_s"], session_id)
                arrays[:, slot] = means
                nodes[node] = {"slot": slot, "epoch": epoch, "frames_per_pose": counts}
            except (ValueError, TypeError, KeyError) as exc:
                rejected[node] = str(exc)
        if self._root_node not in nodes:
            raise OfficialCalibrationError(f"官方手掌标定失败：{rejected.get(self._root_node)}", redo_steps=STEP_KEYS)
        calibration_inputs = dict(zip(STEP_KEYS, arrays.tolist()))
        calibrator = self.core.create_calibrator(**calibration_inputs, side=self.config["hand"], seed=0)
        state = _serialize_calibrator(calibrator)
        for name, shape in _STATE_SHAPES.items():
            _array(state[name], shape, name)
        measured_slots = sorted(entry["slot"] for entry in nodes.values())
        virtual_slots = [slot for slot in range(16) if slot not in measured_slots]
        motion_quality = calibration_motion_diagnostics(calibration_inputs, measured_slots, self._root_node)
        profile = {
            "schema_version": 1, "method": ALGORITHM, "status": "official_core_sparse_adapter_not_accuracy_validated",
            "session_id": session_id, "hand": self.config["hand"], "config_sha256": self._config_sha,
            "rest_sha256": self._rest_sha, "stale_after_s": self.config["stale_after_s"],
            "nodes": nodes, "rejected_nodes": rejected, "calibration_inputs": calibration_inputs,
            "upstream_state": state, "upstream_state_sha256": digest(state),
            "upstream_provenance": deepcopy(self.core.provenance),
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
        if profile.get("upstream_provenance") != self.core.provenance:
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
        from .config import PARENTS
        inverse = {slot: node for node, slot in self.config["node_to_slot"].items()}
        return dict(session_id=profile["session_id"],
                    upstream_state_sha256=profile["upstream_state_sha256"],
                    config_sha256=self._config_sha, rest_sha256=self._rest_sha,
                    nodes_epochs={node: data["epoch"] for node, data in profile["nodes"].items()},
                    dependency_nodes_by_slot={str(slot): [inverse[PARENTS[slot]], inverse[slot]]
                        for slot in (1, 2, 4, 5, 7, 8, 10, 11, 15)
                        if slot in inverse and PARENTS[slot] in inverse})

    def _functional_capture_results(self, capture_groups, profile, *, session_id):
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

        def convert(frames):
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
                results.append({**pose, "joint_observed_mask": joint_mask})
            return results

        return binding, [convert(frames) for frames in capture_groups]

    def fit_functional_finger(self, neutral_frames, motion_frames, profile, *, finger, session_id):
        """Fit only explicitly labelled motion; never reuse legacy calibration.

        Official poses remain the input to this optional model-coordinate
        adapter. No reference MANO angles are invented for the upstream
        ``correct_local_inst_error`` function, which requires known labels.
        """
        from .upstream_functional import FunctionalDependencyError, fit_functional_finger
        binding, (neutral, motion) = self._functional_capture_results(
            [neutral_frames, motion_frames], profile, session_id=session_id)
        try:
            return fit_functional_finger(neutral, motion, finger=finger, binding=binding,
                previous=profile.get("functional_fingers", {}).get(finger))
        except FunctionalDependencyError as exc:
            # Optional dependency failure must not stop acquisition or invalidate
            # the existing official installation profile.
            raise ValueError(str(exc)) from exc

    def fit_functional_hand(self, neutral_frames, finger_frames, thumb_frames, profile, *, session_id):
        """Batch the existing per-finger fits without inventing a new solver.

        One shared neutral, one simultaneous four-finger flexion segment and
        one separate thumb-IP segment. Each finger retains the same individual
        quality checks and per-joint retention semantics. This function never
        mutates the supplied profile; the controller publishes successful
        entries together in one new version. Failed fingers are not entries.
        """
        from .upstream_functional import FINGER_SLOTS, FunctionalDependencyError, fit_functional_finger
        binding, (neutral, fingers, thumb) = self._functional_capture_results(
            [neutral_frames, finger_frames, thumb_frames], profile, session_id=session_id)
        entries, errors = {}, {}
        for finger in FINGER_SLOTS:
            try:
                entries[finger] = fit_functional_finger(neutral, thumb if finger == "thumb" else fingers,
                    finger=finger, binding=binding,
                    previous=profile.get("functional_fingers", {}).get(finger))
            except (ValueError, FunctionalDependencyError) as exc:
                errors[finger] = str(exc)
        return dict(entries=entries, errors=errors,
                    capture_protocol="shared_neutral_four_fingers_then_thumb_ip_v1",
                    input_frame_counts=dict(neutral=len(neutral_frames),
                                            four_fingers=len(finger_frames), thumb_ip=len(thumb_frames)))

    def _apply_functional(self, result, profile, calibrator, frame):
        return self._functional_pipeline.apply(result, profile, calibrator, frame,
            self._functional_binding(profile), self.core,
            coupled=profile["missing_slot_policy"] == COUPLED_POLICY,
            algorithm=FUNCTIONAL_ALGORITHM)

    def solve(self, frame, profile, *, timestamp_s=None):
        return self._solve_frame(frame, profile, timestamp_s=timestamp_s, functional=True)

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
        # Never pre-correct measured quaternions and then correct the hinge a
        # second time. The functional pipeline is the sole heading owner.
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
            if data.get("orientation_valid") is False:
                rejected[node] = "input_filter_prolonged_invalid"
                continue
            if data.get("epoch") != binding["epoch"]:
                rejected[node] = "filter_epoch_changed_recalibration_required"
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
            if functional and rejected.get(self._root_node) != "missing_or_stale_node":
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
                      root_quat_xyzw=Rotation.from_rotvec(mano_pose[0]).as_quat().tolist(),
                      joint_observed_mask=joint_observed.tolist(), inferred_mask=(~joint_observed).tolist(),
                      source_labels=sources, virtual_input_slots=virtual_slots,
                      virtual_input_kind="algebraic_completion_for_declared_prior_not_observation",
                      input_contract="official_full16" if not virtual_slots else "explicit_sparse_to16_adapter",
                      missing_slot_policy=profile["missing_slot_policy"],
                      sparse_prior=prior_diagnostic,
                      upstream_state_sha256=profile["upstream_state_sha256"])
        return self._apply_functional(result, profile, calibrator, frame) if functional else result
