"""Passive twelve-node input observations, not an automatic device configurator.

Arrival cadence is not the chip ODR. Repeated payloads, boundary counts and
acceleration norms are evidence, not grounds to drop samples or assert a faulty
sensor. Memory is constant in recording length; all diagnostics preserve input.
"""
from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path

from .input_contract import describe_input_contract, effective_input_scales


AXES = ("gx", "gy", "gz", "ax", "ay", "az")


def describe_filter_state(filter):
    """Read official VQF estimates without resetting or retuning the filter."""
    bias, sigma = filter.getBiasEstimate()
    return dict(rest_detected=bool(filter.getRestDetected()),
                gyro_bias_rad_s=[float(x) for x in bias], bias_sigma_rad_s=float(sigma),
                absolute_heading_observed=False, source="official_vqf_estimate_not_ground_truth")


class InputObservationMonitor:
    def __init__(self, sample_rate_hz=120.):
        if isinstance(sample_rate_hz, bool) or not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be finite and positive")
        self.rate = float(sample_rate_hz)
        self.nodes = {}
        self.clock_resets = 0
        self.last_input = None

    def observe(self, node, values, stamp):
        # Production callers have already validated RawSample / CSV fields.
        key = str(node)
        if key not in {str(n) for n in range(1, 13)}:
            raise ValueError("input node must be 1..12")
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError("input timestamp must be finite and nonnegative")
        values = tuple(int(x) for x in values)
        if len(values) != 6 or any(not -32768 <= x <= 32767 for x in values):
            raise ValueError("input observation requires six int16 fields")
        if self.last_input is not None and stamp < self.last_input:
            self.clock_resets += 1
            for state in self.nodes.values():
                state.update(epoch_samples=0, first=None, last=None, previous=None,
                             boundary_run=[0] * 6)
        self.last_input = stamp
        state = self.nodes.setdefault(key, dict(samples=0, epoch_samples=0, first=None, last=None,
            raw_min=list(values), raw_max=list(values), near_boundary_by_axis=[0] * 6,
            boundary_run=[0] * 6, max_boundary_run=[0] * 6, max_gap_s=0.,
            equal_host_timestamp_intervals=0, repeated_payload_samples=0,
            previous=None, accel_norm_lsb_sum=0.))
        if state["last"] is not None:
            gap = stamp - state["last"]
            state["max_gap_s"] = max(state["max_gap_s"], gap)
            state["equal_host_timestamp_intervals"] += int(gap == 0.)
        if state["previous"] == values:
            state["repeated_payload_samples"] += 1
        if state["first"] is None:
            state["first"] = stamp
        state["last"], state["previous"] = stamp, values
        state["samples"] += 1
        state["epoch_samples"] += 1
        state["accel_norm_lsb_sum"] += math.sqrt(sum(x * x for x in values[3:]))
        for i, value in enumerate(values):
            state["raw_min"][i] = min(state["raw_min"][i], value)
            state["raw_max"][i] = max(state["raw_max"][i], value)
            near = abs(value) >= 32760
            state["near_boundary_by_axis"][i] += int(near)
            state["boundary_run"][i] = state["boundary_run"][i] + 1 if near else 0
            state["max_boundary_run"][i] = max(state["max_boundary_run"][i], state["boundary_run"][i])

    def snapshot(self, *, detailed=True):
        nodes = {}
        for node in map(str, range(1, 13)):
            state = self.nodes.get(node)
            if state is None:
                nodes[node] = dict(samples=0, observed_receive_rate_hz=None,
                                   nominal_sample_rate_hz=self.rate, last_host_monotonic_s=None)
                continue
            span = None if state["first"] is None else state["last"] - state["first"]
            # Short windows / batched same-stamp groups cannot establish cadence.
            rate = ((state["epoch_samples"] - 1) / span
                    if span is not None and span >= 1. and state["epoch_samples"] > 1 else None)
            nodes[node] = dict(samples=state["samples"], epoch_samples=state["epoch_samples"],
                observed_receive_rate_hz=rate, nominal_sample_rate_hz=self.rate,
                receive_rate_to_nominal_ratio=None if rate is None else rate / self.rate,
                last_host_monotonic_s=state["last"], receive_span_s=span,
                max_receive_gap_s=state["max_gap_s"],
                equal_host_timestamp_intervals=state["equal_host_timestamp_intervals"],
                repeated_payload_samples=state["repeated_payload_samples"],
                raw_min=dict(zip(AXES, state["raw_min"])), raw_max=dict(zip(AXES, state["raw_max"])),
                near_boundary_by_axis=dict(zip(AXES, state["near_boundary_by_axis"])),
                max_consecutive_boundary_samples=dict(zip(AXES, state["max_boundary_run"])),
                mean_accel_norm_lsb=state["accel_norm_lsb_sum"] / state["samples"])
            if not detailed:
                # Frames need cadence, not repeated lifetime extrema/histograms.
                # Full evidence remains available in inspect-input reports.
                nodes[node] = {key: nodes[node][key] for key in (
                    "samples", "observed_receive_rate_hz", "nominal_sample_rate_hz",
                    "receive_rate_to_nominal_ratio", "last_host_monotonic_s")}
        return dict(schema_version=1, nodes=nodes, host_clock_resets=self.clock_resets,
            rate_basis="per_node_host_arrival_span_in_current_clock_epoch_not_device_odr",
            physical_saturation="unknown", raw_values_modified=False,
            automatic_scale_or_timing_changes=False)


def inspect_raw_input(path, config, *, max_samples=None):
    """Inspect an immutable prefix, hashing exactly the complete CSV lines read.

    Does not import VQF/Qt, connect to hardware or alter a profile. A finite
    max_samples is recommended for a still-growing session recording.
    """
    from .replay import _REQUIRED, _raw_rows
    if max_samples is not None and (type(max_samples) is not int or max_samples <= 0):
        raise ValueError("max_samples must be a positive integer")
    contract = describe_input_contract(config)
    gyro_scales, accel_scales = effective_input_scales(config)
    monitor = InputObservationMonitor(config.get("sample_rate_hz", 120.))
    hasher, consumed = hashlib.sha256(), 0
    path = Path(path)
    with path.open("rb") as stream:
        def lines():
            nonlocal consumed
            for index, raw in enumerate(stream):
                if not raw.endswith(b"\n"):
                    raise ValueError("raw CSV ends with an incomplete line; use an earlier bounded prefix")
                hasher.update(raw)
                consumed += len(raw)
                yield raw.decode("utf-8-sig" if index == 0 else "utf-8")
        reader = csv.DictReader(lines(), strict=True)
        header = reader.fieldnames
        if (header is None or len(header) != len(set(header)) or not _REQUIRED <= set(header)
                or set(header) - _REQUIRED - {"status", "phase"}):
            raise ValueError("raw CSV must contain timestamps, node_id and six raw IMU fields")
        rows = 0
        for stamp, node, values in _raw_rows(reader):
            monitor.observe(node, values, stamp)
            rows += 1
            if max_samples is not None and rows >= max_samples:
                break
    observation = monitor.snapshot()
    for node, item in observation["nodes"].items():
        if item["samples"]:
            item["mean_accel_norm_g_using_config"] = item["mean_accel_norm_lsb"] * accel_scales[node]
            item["gyro_min_dps_using_config"] = [item["raw_min"][axis] * gyro_scales[node] for axis in AXES[:3]]
            item["gyro_max_dps_using_config"] = [item["raw_max"][axis] * gyro_scales[node] for axis in AXES[:3]]
    return dict(schema_version=1, kind="vt_fsglove12.input_inspection", status="completed",
        source=str(path.resolve()), source_prefix_bytes=consumed,
        source_prefix_sha256=hasher.hexdigest(), input_samples=rows,
        stop_reason="sample_limit" if max_samples is not None and rows >= max_samples else "eof",
        input_contract=contract, observation=observation,
        hardware_accessed=False, units_auto_verified=False, pose_accuracy_verified=False,
        note="Boundary events do not prove saturation. Motion-averaged acceleration is not a gravity calibration. "
             "A host arrival rate is not device ODR or sensor synchronization.")
