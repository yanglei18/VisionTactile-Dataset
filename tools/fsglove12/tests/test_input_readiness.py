"""Read-only contract inventory: synthetic/configuration checks, not device verification."""
import builtins
import copy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from vt_fsglove12.cli import main
from vt_fsglove12.config import digest, load_config
from vt_fsglove12.input_contract import describe_input_contract, raw_to_si
from vt_fsglove12.input_readiness import describe_input_readiness, input_readiness_from_files


def test_all_twelve_nodes_include_unmapped_and_explicit_effective_gains():
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "12": .004375}
    config["accel_g_per_lsb_by_node"] = {"6": .000244}
    report = describe_input_readiness(config)
    assert set(report["nodes"]) == set(map(str, range(1, 13)))
    assert report["nodes"]["11"]["role"] == "unmapped_auxiliary"
    assert report["nodes"]["12"]["role"] == "palm"
    for key, node in report["nodes"].items():
        gyro, accel = raw_to_si([1] * 6, gyro_dps_per_lsb=node["gyro_dps_per_lsb"],
                               accel_g_per_lsb=node["accel_g_per_lsb"])
        assert node["gyro_rad_s_per_lsb"] == gyro[0]
        assert node["accel_m_s2_per_lsb"] == accel[0]
        assert node["gyro_rad_per_lsb_per_integrated_sample"] == gyro[0] / 120
        assert node["independent_units_verification"] == "not_established_by_this_report"
    assert report["nodes"]["5"]["gyro_gain_relative_to_global"] == 2.
    assert report["nodes"]["12"]["gyro_gain_relative_to_global"] == .5
    assert report["nodes"]["6"]["accel_gain_relative_to_global"] == 2.
    assert report["nodes"]["5"]["gyro_scale_origin"] == "per_node_override"
    assert report["nodes"]["6"]["gyro_scale_origin"] == "global_configuration"
    assert report["raw_layout"]["field_order"] == ["gx", "gy", "gz", "ax", "ay", "az"]


@pytest.mark.parametrize("claimed", [False, True])
def test_global_claims_are_preserved_not_promoted_to_independent_verification(claimed):
    config = load_config()
    config.update(units_verified=claimed, sample_rate_verified=claimed, mounting_confirmed=claimed)
    if claimed:
        config["input_contract"] = {"scale_source": "register_readback"}
    report = describe_input_readiness(config)
    assert report["declarations"]["units_verified"] is claimed
    assert report["declarations"]["sample_rate_verified"] is claimed
    assert all(node["units_declared_verified"] is claimed for node in report["nodes"].values())
    assert report["readiness"] == "independent_validation_not_established"
    assert report["measurement_validity"] == "not_assessed"
    assert not report["runtime_gate_applied"]
    assert not report["protection"]["unverified_units_change_runtime_policy"]
    assert report["evidence_gaps"][0]["nodes"] == list(map(str, range(1, 13)))
    assert report["timing"]["actual_device_rate_hz"] is None


def test_report_and_nested_evidence_are_detached_without_rehashing_config():
    config = load_config()
    config["input_unit_fit_evidence"] = {"source": "/must/not/open/raw.csv", "recommended_nodes": ["5"],
        "nominal_sample_rate_hz": 120, "method": "stationary_gravity_endpoints_first_transition_fit_independent_heldout"}
    metadata = {"source_provenance": {"nested": [1]}}
    before, before_metadata = copy.deepcopy(config), copy.deepcopy(metadata)
    report = describe_input_readiness(config, session_metadata=metadata)
    assert report["source_config_sha256"] == digest(before)
    assert report["provenance"]["internal_fit_present"]
    assert not report["provenance"]["internal_fit_revalidated"]
    assert not report["provenance"]["referenced_artifacts_opened"]
    report["provenance"]["internal_unit_fit_evidence"]["recommended_nodes"].append("6")
    report["provenance"]["session_source_provenance"]["nested"].append(2)
    report["protection"]["input_filter"]["enabled"] = "changed"
    assert config == before and metadata == before_metadata
    json.dumps(describe_input_readiness(config), allow_nan=False)


def test_output_rate_does_not_change_per_sample_gain_but_sample_rate_does():
    config = load_config()
    low = describe_input_readiness(dict(config, output_rate_hz=30))
    high = describe_input_readiness(dict(config, output_rate_hz=120))
    assert low["nodes"] == high["nodes"]
    assert low["timing"]["nominal_integration_period_s"] == 1 / 120
    assert not high["timing"]["output_rate_controls_integration_period"]
    slower_input = describe_input_readiness(dict(config, sample_rate_hz=60, output_rate_hz=30))
    assert slower_input["nodes"]["1"]["gyro_rad_per_lsb_per_integrated_sample"] == (
        2 * high["nodes"]["1"]["gyro_rad_per_lsb_per_integrated_sample"])


def test_consistency_is_binding_not_measurement_validity():
    config = load_config()
    contract = describe_input_contract(config)
    matching = describe_input_readiness(config, session_metadata={"config_sha256": digest(config),
                                      "input_contract": contract}, saved_input_contract=contract)
    assert [check["status"] for check in matching["consistency"]["checks"]] == ["match"] * 3
    bad = describe_input_readiness(config, session_metadata={"config_sha256": "old", "input_contract": {}},
                                   saved_input_contract={})
    assert bad["consistency"]["mismatch_count"] == 3
    assert bad["measurement_validity"] == "not_assessed" and not bad["runtime_gate_applied"]
    missing = describe_input_readiness(config, session_metadata={})
    assert [check["status"] for check in missing["consistency"]["checks"]] == ["not_recorded"] * 2


def test_missing_guard_health_sections_keep_legacy_effective_disabled_policy():
    config = load_config()
    config.pop("input_filter", None)
    config.pop("input_health", None)
    report = describe_input_readiness(config)
    assert not report["protection"]["input_filter"]["enabled"]
    assert not report["protection"]["input_health"]["enabled"]
    assert "input_filter" not in config and "input_health" not in config


def _session(tmp_path, *, metadata=True):
    config = load_config()
    (tmp_path / "config.json").write_text(json.dumps(config), encoding="utf-8")
    if metadata:
        contract = describe_input_contract(config)
        (tmp_path / "session_metadata.json").write_text(json.dumps({"config_sha256": digest(config),
            "input_contract": contract, "session_id": "synthetic", "input_kind": "raw_replay"}), encoding="utf-8")
        (tmp_path / "input_contract.json").write_text(json.dumps(contract), encoding="utf-8")
    return config


def test_session_loader_reads_only_named_small_files_and_never_modifies_them(tmp_path, monkeypatch):
    _session(tmp_path)
    (tmp_path / "raw.csv").write_text("do not open")
    (tmp_path / "poses.jsonl").write_text("do not open")
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}
    original_open = Path.open
    opened = []

    def record_open(path, mode="r", *args, **kwargs):
        assert mode == "rb"
        assert path.name in {"config.json", "session_metadata.json", "input_contract.json"}
        opened.append(path.name)
        return original_open(path, mode, *args, **kwargs)

    with monkeypatch.context() as patch:
        patch.setattr(Path, "open", record_open)
        report = input_readiness_from_files(session_directory=tmp_path)
    assert len(opened) == 3
    assert report["consistency"]["mismatch_count"] == 0
    assert report["provenance"]["session_id"] == "synthetic"
    assert report["provenance"]["session_input_kind"] == "raw_replay"
    assert report["optional_metadata_not_found"] == []
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == before


def test_missing_optional_session_metadata_is_explicit(tmp_path):
    _session(tmp_path, metadata=False)
    report = input_readiness_from_files(session_directory=tmp_path)
    assert report["optional_metadata_not_found"] == ["session_metadata.json", "input_contract.json"]
    assert report["consistency"]["checks"] == []
    assert report["provenance"]["session_id"] is None


@pytest.mark.parametrize("content", ["[]", "not json", '{"bad":NaN}', '{"bad":Infinity}'])
def test_invalid_metadata_fails_without_writing(tmp_path, content):
    source = tmp_path / "config.json"
    source.write_text(content)
    with pytest.raises(ValueError):
        input_readiness_from_files(config_path=source)
    assert source.read_text() == content


def test_metadata_read_is_bounded_and_requires_regular_file(tmp_path, monkeypatch):
    import vt_fsglove12.input_readiness as module
    source = tmp_path / "config.json"
    source.write_text(" " * 65)
    monkeypatch.setattr(module, "MAX_METADATA_BYTES", 64)
    with pytest.raises(ValueError, match="exceeds 64"):
        input_readiness_from_files(config_path=source)
    with pytest.raises(ValueError, match="regular file"):
        input_readiness_from_files(config_path=tmp_path)


def test_cli_reports_stdout_without_loading_live_fusion_or_solver(tmp_path, monkeypatch, capsys):
    _session(tmp_path)
    original_import = builtins.__import__

    def no_live_import(name, *args, **kwargs):
        assert name not in {"serial", "vqf", "torch", "live_stream", "upstream_backend", "functional_pose"}
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_live_import)
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}
    for option, path in (("--config", tmp_path / "config.json"), ("--session", tmp_path)):
        assert main(["input-readiness", option, str(path)]) == 0
        report = json.loads(capsys.readouterr().out)
        assert report["kind"] == "vt_fsglove12.input_calibration_readiness"
        assert report["read_only"] and not report["hardware_accessed"]
        assert not report["configuration_modified"]
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == before


def test_cli_rejects_missing_source_and_reports_malformed_metadata(tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        main(["input-readiness"])
    assert exc.value.code == 2
    source = tmp_path / "config.json"
    source.write_text("[]")
    assert main(["input-readiness", "--config", str(source)]) == 2
    assert "JSON object" in capsys.readouterr().err
    assert not list(tmp_path.glob("*.jsonl"))


@pytest.mark.parametrize("output_rate", [30, 120])
def test_report_timing_identity_matches_live_host_code_without_hardware(output_rate, monkeypatch):
    # Fake VQF records exact calls, not physically valid sensor behaviour. This
    # guards the report's dt/count semantics without fitting its own ground truth.
    from vt_fsglove12.live_stream import OnlineFusion, RawSample

    class FakeVQF:
        def __init__(self, period):
            self.period, self.updates = period, []

        def update(self, gyro, accel):
            self.updates.append((gyro.copy(), accel.copy()))

        def getQuat6D(self):
            return [1., 0., 0., 0.]

        def getBiasEstimate(self):
            return np.zeros(3), 0.

        def getRestDetected(self):
            return False

    monkeypatch.setitem(sys.modules, "vqf", SimpleNamespace(VQF=FakeVQF))
    config = load_config()
    config.update(output_rate_hz=output_rate, input_filter={"enabled": False}, input_health={"enabled": False},
                  gyro_dps_per_lsb_by_node={"5": .0175}, accel_g_per_lsb_by_node={"5": .000244})
    report = describe_input_readiness(config)
    fusion = OnlineFusion(config)
    raw = (100, -200, 300, 40, 50, 8000)
    # Same timestamp and identical payload are still distinct decoded rows.
    fusion.update([RawSample(5, raw, 1., 1001., 9)] * 2)
    filter = fusion.states["5"]["filter"]
    assert filter.period == report["timing"]["nominal_integration_period_s"] == 1 / 120
    assert len(filter.updates) == 2
    np.testing.assert_array_equal(filter.updates[0][0], np.array(raw[:3]) * (.0175 * math.pi / 180))
    np.testing.assert_array_equal(filter.updates[0][1], np.array(raw[3:]) * (.000244 * 9.80665))
    assert fusion.snapshot(1.)["nodes"]["5"]["sample_index"] == 2
    for _ in range(20):
        fusion.input_snapshot(1.)
    assert len(filter.updates) == 2 and fusion.frame_index == 1
    # Existing health policy, not this report, decides whether to integrate.
    monkeypatch.setattr(fusion.input_health, "integration_allowed", lambda node: False)
    fusion.update([RawSample(5, raw, 1.01, 1001.01, 10)])
    assert len(filter.updates) == 2 and fusion.counts["5"] == 3
    fusion.reset_for_calibration()
    assert fusion.counts["5"] == 3 and fusion.epochs["5"] == 1
