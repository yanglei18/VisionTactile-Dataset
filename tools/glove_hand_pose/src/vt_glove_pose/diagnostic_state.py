"""Versioned runtime checkpoints for offline diagnosis, never live calibration.

VQF's public ``state`` property includes NaN initialization sentinels. Only
known low-pass history arrays may contain the explicit JSON tag used here;
ordinary metadata, quaternions and measurements must remain finite.
"""
from __future__ import annotations

from collections import deque
from importlib.metadata import version
import json
import math

import numpy as np

from .config import fingerprint, gyro_scale, validate_config


KIND = "vt_glove_pose.engine_checkpoint"
SCHEMA_VERSION = 2
LEGACY_SCHEMA_VERSION = 1
LEGACY_ALGORITHM = "legacy_quaternion_v1"
THUMB_ALGORITHM = "thumb_paired_gyro_gravity_v1"
_NODES = {str(n) for n in range(1, 13)}
_NAN_TAG = {"__vqf_nan__": True}
_MAX_JSON_BYTES = 2_000_000
_MAX_INTEGER = 2**53 - 1
_ARRAYS = {
    "gyrQuat": 4, "accQuat": 4, "lastAccLp": 3, "accLpState": 6,
    "bias": 3, "biasP": 9, "motionBiasEstRLpState": 18,
    "motionBiasEstBiasLpState": 4, "restLastSquaredDeviations": 2,
    "restLastGyrLp": 3, "restGyrLpState": 6, "restLastAccLp": 3,
    "restAccLpState": 6, "magNormDip": 2, "magNormDipLpState": 4,
}
_NAN_ARRAYS = {
    "accLpState", "motionBiasEstRLpState", "motionBiasEstBiasLpState",
    "restGyrLpState", "restAccLpState", "magNormDipLpState",
}
_BOOLEANS = {"restDetected", "magDistDetected"}
_SCALARS = {
    "delta", "lastAccCorrAngularRate", "kMagInit", "lastMagDisAngle",
    "lastMagCorrAngularRate", "restT", "magRefNorm", "magRefDip",
    "magUndisturbedT", "magRejectT", "magCandidateNorm", "magCandidateDip", "magCandidateT",
}
_PARAM_BOOLS = {"motionBiasEstEnabled", "restBiasEstEnabled", "magDistRejectionEnabled"}
# Validate before calling the native constructor: it contains assertions for
# positive filter constants, and several bias formulas divide by parameters.
_PARAM_BOUNDS = {
    "tauAcc": (1e-6, 1e6), "tauMag": (-1, 1e6),
    "biasSigmaInit": (1e-9, 360), "biasForgettingTime": (1e-6, 1e9),
    "biasClip": (0, 3600), "biasSigmaMotion": (1e-9, 360),
    "biasVerticalForgettingFactor": (1e-9, 1e9), "biasSigmaRest": (1e-9, 360),
    "restMinT": (0, 1e6), "restFilterTau": (1e-6, 1e6),
    "restThGyr": (1e-9, 1e6), "restThAcc": (1e-9, 1e6),
    "magCurrentTau": (-1, 1e6), "magRefTau": (-1, 1e6),
    "magNormTh": (0, 1e6), "magDipTh": (0, 1e6),
    "magNewTime": (0, 1e9), "magNewFirstTime": (0, 1e9),
    "magNewMinGyr": (0, 1e6), "magMinUndisturbedTime": (0, 1e9),
    "magMaxRejectionTime": (0, 1e9), "magRejectionFactor": (1e-6, 1e6),
}
_ENGINE_KEYS = {
    "last_time", "states", "bias", "mounts", "zero", "neutral_acc", "axes_quality",
    "axis_sign_versions", "neutral_ready", "mount_confirmed", "active", "message",
    "gap_count", "invalid_count", "near_limit_count",
}
_THUMB_ENGINE_KEYS = {"thumb_estimator", "thumb_pair_counts", "thumb_common_capture"}
_COMMON_CAPTURE_KEYS = {"prox_gyro", "dist_gyro", "prox_acc", "dist_acc"}


def get_vqf_version() -> str:
    """Return the installed distribution version used by checkpoint matching."""
    return version("vqf")


def _mapping(value, keys, name):
    if type(value) is not dict or set(value) != set(keys):
        raise ValueError(f"{name}: unexpected or missing fields")
    return value


def _number(value, name, low=-1e12, high=1e12):
    if type(value) not in (int, float) or not low <= value <= high or not math.isfinite(value):
        raise ValueError(f"{name}: expected a finite number in [{low}, {high}]")
    return value


def _integer(value, name, high=_MAX_INTEGER):
    if type(value) is not int or not 0 <= value <= high:
        raise ValueError(f"{name}: invalid nonnegative integer")
    return value


def _boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f"{name}: expected boolean")
    return value


def _vector(value, size, name, *, unit=False, low=-1e12, high=1e12):
    if type(value) is not list or len(value) != size:
        raise ValueError(f"{name}: expected {size} values")
    result = np.array([_number(v, name, low, high) for v in value], dtype=float)
    if unit and abs(float(np.linalg.norm(result)) - 1) > 1e-6:
        raise ValueError(f"{name}: quaternion must be unit length")
    return result


def _node_map(value, name, *, complete=False):
    if type(value) is not dict or not set(value) <= _NODES or (complete and set(value) != _NODES):
        raise ValueError(f"{name}: expected node keys 1..12")
    return value


def _json_copy(value):
    # This also prevents an attacker-controlled checkpoint from passing an
    # arbitrarily deep/large object to the native VQF state setter.
    count = 0

    def check(item, depth=0):
        nonlocal count
        count += 1
        if depth > 16 or count > 100_000:
            raise ValueError("checkpoint exceeds structural bounds")
        if type(item) is dict:
            if any(type(key) is not str for key in item):
                raise ValueError("checkpoint object keys must be strings")
            for child in item.values():
                check(child, depth + 1)
        elif type(item) is list:
            for child in item:
                check(child, depth + 1)
        elif type(item) not in (str, int, float, bool, type(None)):
            raise ValueError("checkpoint must contain only JSON values")
    check(value)
    try:
        text = json.dumps(value, allow_nan=False, ensure_ascii=False)
        size = len(text.encode("utf-8"))
    except (TypeError, ValueError, OverflowError, UnicodeError) as exc:
        raise ValueError("checkpoint contains invalid JSON values") from exc
    if size > _MAX_JSON_BYTES:
        raise ValueError("checkpoint exceeds size limit")
    return json.loads(text)


def _filter_state(value, *, decode):
    _mapping(value, set(_ARRAYS) | _BOOLEANS | _SCALARS, "VQF state")
    result = {}
    for key, size in _ARRAYS.items():
        source = value[key]
        if type(source) is not list or len(source) != size:
            raise ValueError(f"VQF state.{key}: wrong array shape")
        row = []
        for entry in source:
            tagged = type(entry) is dict and set(entry) == {"__vqf_nan__"} and entry["__vqf_nan__"] is True
            if decode and tagged and key in _NAN_ARRAYS:
                row.append(float("nan"))
            elif not decode and type(entry) is float and math.isnan(entry) and key in _NAN_ARRAYS:
                row.append(dict(_NAN_TAG))
            else:
                row.append(_number(entry, f"VQF state.{key}"))
        if key in {"gyrQuat", "accQuat"}:
            _vector(row, 4, key, unit=True)
        result[key] = row
    for key in _BOOLEANS:
        result[key] = _boolean(value[key], f"VQF state.{key}")
    for key in _SCALARS:
        result[key] = _number(value[key], f"VQF state.{key}")
    return result


def _params(value):
    _mapping(value, set(_PARAM_BOUNDS) | _PARAM_BOOLS, "VQF params")
    return {key: _boolean(value[key], key) if key in _PARAM_BOOLS else
            _number(value[key], key, *_PARAM_BOUNDS[key]) for key in value}


def _common_capture(value, config):
    """Validate unfinished joint-axis A data, without declaring calibration done."""
    if value is None:
        return None
    fields = _COMMON_CAPTURE_KEYS
    if type(value) is dict and "calibration_method" in value:
        if value["calibration_method"] != "robust_v2":
            raise ValueError("unsupported thumb common-motion calibration method")
        fields = fields | {"calibration_method"}
    _mapping(value, fields, "thumb common-motion capture")
    count = None
    for key in _COMMON_CAPTURE_KEYS:
        rows = value[key]
        if type(rows) is not list or not 30 <= len(rows) <= 2000:
            raise ValueError("thumb common-motion capture requires 30..2000 rows")
        if count is None:
            count = len(rows)
        elif len(rows) != count:
            raise ValueError("thumb common-motion arrays must have equal row counts")
        node = config["fingers"]["thumb"]["proximal" if key.startswith("prox_") else "distal"]
        bound = (32768 * gyro_scale(config, node) + 10 if key.endswith("gyro")
                 else 32768 * config["accel_g_per_lsb"])
        for row in rows:
            _vector(row, 3, f"thumb common-motion {key}", low=-bound, high=bound)
    return value


def export_checkpoint(engine) -> dict:
    """Copy an idle engine into strict JSON, including filter initialization."""
    if engine.active is not None:
        raise ValueError("请先结束或取消标定，再开始诊断录制；不导出进行中的采集。")
    algorithm = engine.algorithm_mode
    if algorithm not in {LEGACY_ALGORITHM, THUMB_ALGORITHM}:
        raise ValueError("unsupported engine pose algorithm")
    config = validate_config(engine.config)
    runtime = {key: getattr(engine, key) for key in _ENGINE_KEYS - {
        "states", "bias", "mounts", "zero", "neutral_acc", "axes_quality", "axis_sign_versions"}}
    for key in ("bias", "mounts", "zero", "neutral_acc"):
        runtime[key] = {str(node): np.asarray(array).tolist() for node, array in getattr(engine, key).items()}
    for key in ("axes_quality", "axis_sign_versions"):
        runtime[key] = {str(node): value for node, value in getattr(engine, key).items()}
    runtime["states"] = {}
    for node, state in engine.states.items():
        filt = state["filter"]
        item = {"count": state["count"], "times": list(state["times"]),
                "filter": {"params": filt.params,
                           "sample_periods_s": {name: filt.coeffs[name + "Ts"] for name in ("gyr", "acc", "mag")},
                           "state": _filter_state(filt.state, decode=False)}}
        if "time" in state:
            item["time"] = state["time"]
            for key in ("quat", "gyro", "acc"):
                item[key] = np.asarray(state[key]).tolist()
        runtime["states"][str(node)] = item
    if algorithm == THUMB_ALGORITHM:
        runtime["thumb_estimator"] = (None if engine.thumb_estimator is None
                                      else engine.thumb_estimator.export_state())
        runtime["thumb_pair_counts"] = list(engine.thumb_pair_counts)
        capture = engine.thumb_common_capture
        runtime["thumb_common_capture"] = (None if capture is None else
                                            {key: np.asarray(value).tolist() for key, value in capture.items()})
    document = dict(kind=KIND, schema_version=(SCHEMA_VERSION if algorithm == THUMB_ALGORITHM
                                              else LEGACY_SCHEMA_VERSION), offline_only=True,
                    vqf_version=get_vqf_version(), config=config,
                    config_fingerprint=fingerprint(config), engine=runtime)
    if algorithm == THUMB_ALGORITHM:
        document["pose_algorithm"] = algorithm
    data = _json_copy(document)
    # Apply the same boundary validation to exported and imported data without
    # constructing another twelve native filters on the acquisition thread.
    _validate(data)
    return data


def _validate(data):
    if type(data) is not dict or type(data.get("schema_version")) is not int or data["schema_version"] not in {LEGACY_SCHEMA_VERSION, SCHEMA_VERSION}:
        raise ValueError("not a supported offline engine checkpoint")
    version = data["schema_version"]
    fields = {"kind", "schema_version", "offline_only", "vqf_version", "config",
              "config_fingerprint", "engine"}
    if version == SCHEMA_VERSION:
        fields.add("pose_algorithm")
    _mapping(data, fields, "checkpoint")
    if data["kind"] != KIND or data["offline_only"] is not True:
        raise ValueError("not a supported offline engine checkpoint")
    algorithm = LEGACY_ALGORITHM if version == LEGACY_SCHEMA_VERSION else data["pose_algorithm"]
    if version == SCHEMA_VERSION and algorithm != THUMB_ALGORITHM:
        raise ValueError("checkpoint pose algorithm is unsupported")
    if type(data["vqf_version"]) is not str or data["vqf_version"] != get_vqf_version():
        raise ValueError("VQF version mismatch; exact offline restoration is unavailable")
    config = validate_config(data["config"])
    if data["config_fingerprint"] != fingerprint(config):
        raise ValueError("checkpoint configuration fingerprint mismatch")
    state = _mapping(data["engine"], _ENGINE_KEYS | (_THUMB_ENGINE_KEYS if version == SCHEMA_VERSION else set()), "engine")
    if state["active"] is not None:
        raise ValueError("active calibration is not supported in a checkpoint")
    for key in ("neutral_ready", "mount_confirmed"):
        _boolean(state[key], key)
    if type(state["message"]) is not str or len(state["message"]) > 16_384:
        raise ValueError("invalid engine message")
    for key in ("gap_count", "invalid_count", "near_limit_count"):
        _integer(state[key], key)
    if state["last_time"] is not None:
        _number(state["last_time"], "last_time")
    for key, size in (("bias", 3), ("mounts", 4), ("zero", 4), ("neutral_acc", 3)):
        for node, values in _node_map(state[key], key, complete=key == "bias").items():
            v = _vector(values, size, key, unit=size == 4)
            if key == "bias" and np.linalg.norm(v) > 10.000001:
                raise ValueError("static gyro bias exceeds calibration bounds")
            if key == "neutral_acc" and not 0.7 <= np.linalg.norm(v) <= 1.3:
                raise ValueError("neutral acceleration exceeds calibration bounds")
    for key in ("axes_quality", "axis_sign_versions"):
        if not set(_node_map(state[key], key)) <= set(state["mounts"]):
            raise ValueError(f"{key} refers to missing mounting axes")
    for value in state["axis_sign_versions"].values():
        _integer(value, "axis_sign_version", 2)
    for value in state["axes_quality"].values():
        if type(value) is not dict or len(json.dumps(value, allow_nan=False)) > 16_384:
            raise ValueError("invalid axis quality metadata")
    times = []
    for node, item in _node_map(state["states"], "states").items():
        if type(item) is not dict:
            raise ValueError("invalid node state")
        fields = {"filter", "count", "times"}
        if "time" in item:
            fields |= {"time", "quat", "gyro", "acc"}
        _mapping(item, fields, f"node {node}")
        count = _integer(item["count"], "sample count")
        if type(item["times"]) is not list or len(item["times"]) > min(512, count):
            raise ValueError("invalid node timestamp history")
        history = [_number(t, "node timestamp") for t in item["times"]]
        if any(a > b for a, b in zip(history, history[1:])):
            raise ValueError("node timestamp history is not ordered")
        if "time" in item:
            stamp = _number(item["time"], "node time")
            if count == 0 or (history and history[-1] != stamp):
                raise ValueError("node timestamp and sample count are inconsistent")
            _vector(item["quat"], 4, "node quaternion", unit=True)
            scale = gyro_scale(config, int(node))
            _vector(item["gyro"], 3, "gyro_dps", low=-32768*scale, high=32767*scale)
            _vector(item["acc"], 3, "accel_g", low=-32768*config["accel_g_per_lsb"], high=32767*config["accel_g_per_lsb"])
            times.append(stamp)
        elif count or history:
            raise ValueError("uninitialized node has samples without a timestamp")
        filt = _mapping(item["filter"], {"params", "sample_periods_s", "state"}, "filter")
        _params(filt["params"])
        periods = _mapping(filt["sample_periods_s"], {"gyr", "acc", "mag"}, "sample periods")
        if any(_number(dt, "sample period", 1e-6, 1) != 1/config["sample_rate_hz"] for dt in periods.values()):
            raise ValueError("VQF sample period differs from configured nominal period")
        _filter_state(filt["state"], decode=True)
    if (max(times) if times else None) != state["last_time"]:
        raise ValueError("engine last_time is inconsistent with node times")
    if state["neutral_ready"] and (set(state["zero"]) != _NODES or set(state["neutral_acc"]) != _NODES or
                                   set(state["states"]) != _NODES or not state["mount_confirmed"] or
                                   any("time" not in item for item in state["states"].values())):
        raise ValueError("ready neutral reference is incomplete")
    thumb = None
    if version == SCHEMA_VERSION:
        _common_capture(state["thumb_common_capture"], config)
        consumed = state["thumb_pair_counts"]
        if type(consumed) is not list or len(consumed) != 2:
            raise ValueError("thumb_pair_counts must contain two sample counts")
        for role, count in zip(("proximal", "distal"), consumed):
            node = str(config["fingers"]["thumb"][role])
            available = state["states"].get(node, {}).get("count", 0)
            if _integer(count, "thumb consumed sample count") > available:
                raise ValueError("thumb consumed count exceeds received node samples")
        if state["thumb_estimator"] is not None:
            from .thumb_flexion import ThumbFlexionEstimator
            # This pure Python/numpy constructor validates every algorithm
            # state field before any native VQF state setter can be reached.
            thumb = ThumbFlexionEstimator.from_state(state["thumb_estimator"])
            if thumb.sample_rate_hz != config["sample_rate_hz"]:
                raise ValueError("thumb sample rate differs from configured nominal rate")
    return config, state, algorithm, thumb


def from_checkpoint(engine_class, data):
    """Construct an independent offline engine; never mutate an existing one."""
    data = _json_copy(data)
    config, runtime, algorithm, thumb = _validate(data)
    # Never restore an old capture into the current default algorithm. V1
    # checkpoints retain the exact legacy calculation path and re-export V1.
    engine = engine_class(config, algorithm_mode=algorithm)
    # All inputs are validated before entering VQF's native setters. The new
    # object is returned only after every node has been restored successfully.
    restored = {}
    for node, item in runtime["states"].items():
        saved = item["filter"]
        periods = saved["sample_periods_s"]
        filt = engine._vqf(periods["gyr"], periods["acc"], periods["mag"], **saved["params"])
        filt.state = _filter_state(saved["state"], decode=True)
        state = {"filter": filt, "count": item["count"], "times": deque(item["times"], maxlen=512)}
        if "time" in item:
            state["time"] = item["time"]
            for key in ("quat", "gyro", "acc"):
                state[key] = np.array(item[key], dtype=float)
        restored[int(node)] = state
    engine.states = restored
    for key in ("bias", "mounts", "zero", "neutral_acc"):
        setattr(engine, key, {int(node): np.array(v, dtype=float) for node, v in runtime[key].items()})
    for key in ("axes_quality", "axis_sign_versions"):
        setattr(engine, key, {int(node): v for node, v in runtime[key].items()})
    for key in _ENGINE_KEYS - {"states", "bias", "mounts", "zero", "neutral_acc", "axes_quality", "axis_sign_versions"}:
        setattr(engine, key, runtime[key])
    if algorithm == THUMB_ALGORITHM:
        engine.thumb_estimator = thumb
        engine.thumb_pair_counts = list(runtime["thumb_pair_counts"])
        engine.thumb_common_capture = runtime["thumb_common_capture"]
    return engine
