"""Read-only input-calibration inventory; never a live validity gate.

Configuration claims and saved metadata are not independent measurements.
This report describes the host implementation and inventories evidence already
recorded with a configuration. It neither verifies a device nor fits scales.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import stat

from .config import digest, validate_config
from .input_contract import describe_input_contract
from .input_guard import REVISION, guard_settings
from .input_health import health_settings


MAX_METADATA_BYTES = 1024 * 1024


def _detached(value):
    return json.loads(json.dumps(value, allow_nan=False))


def describe_input_readiness(config, *, session_metadata=None, saved_input_contract=None):
    """Inventory all 12 inputs without changing config or assessing live data.

    ``units_verified`` remains a global *declaration*, including when true. No
    per-node independent evidence schema currently exists in the live config;
    do not convert this declaration, an internal fit, or a matching config hash
    into a physical verification result. Saved hashes establish binding only.
    """
    source_config_sha = digest(config)
    effective = validate_config(config)
    contract = describe_input_contract(effective)
    period = contract["sample_timing"]["integration_period_s"]
    declarations = dict(
        units_verified=effective.get("units_verified", False),
        sample_rate_verified=effective.get("sample_rate_verified", False),
        mounting_confirmed=effective.get("mounting_confirmed", False),
        source="configuration_declaration_not_independent_measurement",
    )
    nodes = {}
    for key, source in contract["nodes"].items():
        gyro_rad = source["gyro_dps_per_lsb"] * math.pi / 180.
        nodes[key] = dict(
            node_id=source["node_id"], slot=source["slot"], role=source["role"],
            gyro_dps_per_lsb=source["gyro_dps_per_lsb"],
            gyro_rad_s_per_lsb=gyro_rad,
            accel_g_per_lsb=source["accel_g_per_lsb"],
            accel_m_s2_per_lsb=source["accel_g_per_lsb"] * 9.80665,
            gyro_gain_relative_to_global=source["gyro_dps_per_lsb"] / effective["gyro_dps_per_lsb"],
            accel_gain_relative_to_global=source["accel_g_per_lsb"] / effective["accel_g_per_lsb"],
            gyro_scale_origin=("per_node_override" if key in effective.get("gyro_dps_per_lsb_by_node", {})
                               else "global_configuration"),
            accel_scale_origin=("per_node_override" if key in effective.get("accel_g_per_lsb_by_node", {})
                                else "global_configuration"),
            nominal_integration_period_s=period,
            gyro_rad_per_lsb_per_integrated_sample=gyro_rad * period,
            units_declared_verified=declarations["units_verified"],
            independent_units_verification="not_established_by_this_report",
        )

    checks = []
    if session_metadata is not None:
        if not isinstance(session_metadata, dict):
            raise ValueError("session_metadata must be an object")
        recorded_sha = session_metadata.get("config_sha256")
        checks.append(dict(check="session_config_binding", status=("not_recorded" if recorded_sha is None else
                           "match" if recorded_sha == source_config_sha else "mismatch"),
                           recorded=recorded_sha, actual=source_config_sha))
        recorded_contract = session_metadata.get("input_contract")
        checks.append(dict(check="session_input_contract", status=("not_recorded" if recorded_contract is None else
                           "match" if recorded_contract == contract else "mismatch")))
    if saved_input_contract is not None:
        if not isinstance(saved_input_contract, dict):
            raise ValueError("saved_input_contract must be an object")
        checks.append(dict(check="saved_input_contract", status=(
            "match" if saved_input_contract == contract else "mismatch")))

    internal_fit = effective.get("input_unit_fit_evidence")
    missing = [
        dict(id="per_node_unit_evidence", nodes=list(nodes),
             need="Independent evidence binding each node's reported counts and configured gyro/accel scales "
                  "to the current device/configuration; a matching datasheet sensitivity is not proof."),
        dict(id="device_sample_timing", nodes=list(nodes),
             need="Evidence for per-node physical sampling period and duplicate/drop semantics. Host arrival "
                  "rate and decoded sample_index alone do not verify device ODR or synchronization."),
        dict(id="physical_mapping", nodes=[key for key, node in nodes.items() if node["slot"] is not None],
             need="Observed node-to-segment mounting; configured labels and a solver's output are not confirmation."),
    ]
    result = dict(
        schema_version=1, kind="vt_fsglove12.input_calibration_readiness",
        scope="configuration_and_saved_metadata_only", read_only=True,
        hardware_accessed=False, configuration_modified=False,
        measurement_validity="not_assessed", runtime_gate_applied=False,
        readiness="independent_validation_not_established",
        source_config_sha256=source_config_sha, effective_config_sha256=digest(effective),
        declarations=declarations, nodes=nodes,
        raw_layout={key: contract[key] for key in (
            "encoding", "field_order", "frame_bytes", "node_block_bytes", "nodes_per_frame", "node_groups")},
        timing=dict(
            nominal_sample_rate_hz=float(effective["sample_rate_hz"]),
            nominal_integration_period_s=period,
            output_rate_hz=float(effective["output_rate_hz"]),
            integration_period_source="1 / sample_rate_hz; fixed VQF period, not host arrival delta",
            output_rate_controls_integration_period=False,
            update_policy="One VQF update per decoded sample when input health allows integration; "
                          "snapshots do not advance fusion and no missing samples are synthesized.",
            device_timestamp_available=False, actual_device_rate_hz=None,
            scale_timing_ambiguity="Integrated rotation depends on gyro_scale * nominal_period. An internal "
                                   "gyro/gravity fit conditional on nominal dt cannot independently identify both.",
        ),
        sample_identity=dict(
            host_timestamp="Host read completion; multiple decoded samples may share one timestamp.",
            sample_index="Per-node decoded receive count, including health-paused samples; not an "
                         "integration count or proof of distinct physical measurements.",
            frame_index="Snapshot/output identity, not sensor sample identity.",
            sensor_epoch="Filter/reference generation; interpret with session_id, node_id and sample_index.",
            second_byte_semantics=contract["second_byte_semantics"],
            device_sequence_verified=False,
        ),
        protection=dict(
            input_filter=guard_settings(effective), input_filter_revision=REVISION,
            input_health=health_settings(effective),
            raw_values_modified=False,
            fusion_values="If enabled, the causal guard may replace rejected gyro components or acceleration; "
                          "health faults may pause integration. Raw records remain separate from fusion inputs.",
            coding_boundary_proves_physical_saturation=False,
            unverified_units_change_runtime_policy=False,
        ),
        provenance=dict(
            sensor_model=contract["sensor_model"], model_source=contract["model_source"],
            scale_source=contract["scale_source"],
            configuration_note=effective.get("provenance"),
            internal_unit_fit_evidence=internal_fit,
            internal_fit_present=internal_fit is not None,
            internal_fit_revalidated=False,
            internal_fit_boundary="Existing gyro/gravity evidence is conditional on nominal dt; it is not "
                                  "all-node unit verification, register readback, or independent pose ground truth.",
            referenced_artifacts_opened=False,
            independent_evidence_review_status="not_performed",
            session_id=(session_metadata or {}).get("session_id"),
            session_input_kind=(session_metadata or {}).get("input_kind"),
            session_source_provenance=(session_metadata or {}).get("source_provenance"),
        ),
        consistency=dict(checks=checks, mismatch_count=sum(check["status"] == "mismatch" for check in checks),
                         establishes="Saved configuration binding only, not physical correctness."),
        evidence_gaps=missing,
        next_steps=[
            "Review source/config binding mismatches before associating evidence with this run.",
            "Obtain per-node unit and device-timing evidence, including held-out known motion; "
            "do not use this solver's orientation as its own reference.",
            "Use labelled existing raw data for an offline unit-fit candidate only when appropriate; "
            "review timing assumptions and retain the original recording/configuration.",
        ],
        note="Unverified means missing or unreviewed evidence, not an invalid measurement. This inventory "
             "does not authorize changing scales, filtering data, discarding samples or recalibrating hardware.",
    )
    # No references to caller-owned nested evidence or metadata escape.
    return _detached(result)


def _read_metadata(path):
    """Read one bounded regular JSON object; never follow provenance paths."""
    path = Path(path)
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError(f"metadata must be a regular file: {path}")
    with path.open("rb") as stream:
        data = stream.read(MAX_METADATA_BYTES + 1)
    if len(data) > MAX_METADATA_BYTES:
        raise ValueError(f"metadata exceeds {MAX_METADATA_BYTES} bytes: {path}")
    payload = json.loads(data)
    if not isinstance(payload, dict):
        raise ValueError(f"metadata must be a JSON object: {path}")
    payload = _detached(payload)  # Reject non-finite JSON, including saved evidence.
    return payload, dict(path=str(path.resolve()), bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


def input_readiness_from_files(*, config_path=None, session_directory=None):
    """Read an explicit config OR a session's small metadata files, no logs.

    No profile, raw CSV, pose JSONL, external evidence path, solver, VQF instance,
    or serial device is opened. The caller prints the returned report.
    """
    if (config_path is None) == (session_directory is None):
        raise ValueError("provide exactly one of config_path or session_directory")
    session = Path(session_directory) if session_directory is not None else None
    config, source = _read_metadata(session / "config.json" if session is not None else config_path)
    inputs = dict(config=source)
    options = {}
    missing = []
    if session is not None:
        for name, argument in (("session_metadata.json", "session_metadata"),
                               ("input_contract.json", "saved_input_contract")):
            path = session / name
            if path.exists():
                options[argument], inputs[argument] = _read_metadata(path)
            else:
                missing.append(name)
    report = describe_input_readiness(config, **options)
    report.update(inputs=inputs, optional_metadata_not_found=missing)
    return report
