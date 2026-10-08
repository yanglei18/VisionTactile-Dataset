import copy
import json
import math

import numpy as np
import pytest

from vt_fsglove12.config import digest, load_config, validate_config
from vt_fsglove12.input_contract import (
    describe_input_contract, effective_input_scales, raw_to_si, validate_input_contract,
)


def test_legacy_config_is_not_augmented_or_rehashed_by_description():
    config = load_config()
    config.pop("input_contract", None)
    before, sha = copy.deepcopy(config), digest(config)
    report = describe_input_contract(config)
    assert config == before and digest(config) == sha
    assert validate_config(config) == before
    assert "input_contract" not in config and "accel_g_per_lsb_by_node" not in config
    assert not report["configuration_explicit"]
    assert report["sensor_model"] == "unknown" and report["model_source"] == "undeclared"
    assert report["nodes"]["5"]["nominal_gyro_full_scale_dps"] is None
    assert json.loads(json.dumps(report, allow_nan=False)) == report
    report["field_order"][0] = "changed"
    assert describe_input_contract(config)["field_order"][0] == "gx"


def test_contract_reports_counts_not_verified_registers_or_device_timing():
    config = load_config()
    config["input_contract"] = {"sensor_model": "LSM6DSRTR", "model_source": "user_reported"}
    report = describe_input_contract(config)
    assert report["sensor_model"] == "LSM6DSRTR"
    assert report["encoding"] == "signed_int16_le"
    assert report["frame_bytes"] == 84 and report["node_block_bytes"] == 14
    assert report["second_byte_semantics"] == "unknown"
    assert report["node_groups"] == [list(range(1, 7)), list(range(7, 13))]
    assert report["nodes"]["5"]["role"] == "thumb_proximal"
    assert report["nodes"]["11"]["role"] == "unmapped_auxiliary"
    assert report["nodes"]["12"]["role"] == "palm"
    assert report["sample_timing"]["integration_period_s"] == 1 / 120.
    assert not report["sample_timing"]["device_timestamp_available"]
    assert not report["verification"]["register_readback_performed_by_this_tool"]
    assert not report["verification"]["physical_full_scale_verified"]
    assert not report["verification"]["units_verified"]
    assert report["nodes"]["5"]["nominal_gyro_full_scale_dps"] == 250
    assert report["nodes"]["5"]["nominal_accel_full_scale_g"] == 4


@pytest.mark.parametrize("scale,full_scale", [(.004375,125),(.00875,250),(.0175,500),(.035,1000),(.07,2000),(.14,4000)])
def test_datasheet_mapping_does_not_infer_actual_fs(scale, full_scale):
    config = load_config()
    config["input_contract"] = {"sensor_model": "LSM6DSRTR", "model_source": "user_reported"}
    config["gyro_dps_per_lsb_by_node"] = {"6": scale}
    node = describe_input_contract(config)["nodes"]["6"]
    assert node["nominal_gyro_full_scale_dps"] == full_scale
    assert not node["physical_full_scale_verified"]


def test_independent_per_node_scales_and_non_datasheet_scale():
    config = load_config()
    config["input_contract"] = {"sensor_model": "LSM6DSRTR", "model_source": "user_reported"}
    config["gyro_dps_per_lsb_by_node"] = {"5": .01}
    config["accel_g_per_lsb_by_node"] = {"6": .000488}
    validate_config(config)
    gyro, accel = effective_input_scales(config)
    assert gyro["5"] == .01 and gyro["6"] == .00875
    assert accel["5"] == .000122 and accel["6"] == .000488
    report = describe_input_contract(config)
    assert report["nodes"]["5"]["nominal_gyro_full_scale_dps"] is None
    assert report["nodes"]["6"]["nominal_accel_full_scale_g"] == 16


@pytest.mark.parametrize("field,value", [
    ("schema_version", True), ("sensor_model", "HI229"), ("model_source", "guessed"),
    ("encoding", "unsigned_int16_be"), ("field_order", ["ax","ay","az","gx","gy","gz"]),
    ("frame_bytes", 168), ("node_block_bytes", 12), ("nodes_per_frame", 12),
    ("second_byte_semantics", "sample_counter"), ("raw_value_source", "verified_chip_registers"),
    ("scale_source", "auto_detected"), ("host_timestamp_source", "device_clock"),
    ("sample_timing_assumption", "host_arrival_dt"), ("device_timestamp_available", True),
    ("transport", "hi229_serial"), ("extra_unknown", "x"),
])
def test_incompatible_declaration_is_not_silently_used(field, value):
    config = load_config()
    config["input_contract"] = {field: value}
    with pytest.raises(ValueError, match="input_contract"):
        validate_config(config)


def test_explicit_contract_and_external_claims_remain_declarations():
    config = load_config()
    config["input_contract"] = {"schema_version": 1, "sensor_model": "LSM6DSR",
                                "scale_source": "register_readback"}
    with pytest.raises(ValueError, match="units_verified"):
        validate_config(config)
    config["units_verified"] = True
    report = describe_input_contract(validate_config(config))
    assert report["configuration_explicit"]
    assert report["verification"]["units_verified"]
    assert not report["verification"]["register_readback_performed_by_this_tool"]
    assert not report["verification"]["physical_full_scale_verified"]


@pytest.mark.parametrize("field,value", [("input_contract", []), ("units_verified", 1),
    ("sample_rate_verified", "false"), ("accel_g_per_lsb_by_node", []),
    ("accel_g_per_lsb_by_node", {"13": .0001}), ("accel_g_per_lsb_by_node", {"01": .0001}),
    ("accel_g_per_lsb_by_node", {"1": True}), ("accel_g_per_lsb_by_node", {"1": float("nan")})])
def test_invalid_contract_inputs(field, value):
    config = load_config()
    config[field] = value
    with pytest.raises(ValueError):
        validate_config(config)


def test_shared_conversion_exactly_preserves_previous_formula_and_raw_rails():
    raw = np.array([-32768, 32764, 125, -32764, 8197, 100], dtype=np.int16)
    before = raw.copy()
    gyro, accel = raw_to_si(raw, gyro_dps_per_lsb=.00875, accel_g_per_lsb=.000122)
    np.testing.assert_array_equal(gyro, np.asarray(raw[:3], dtype=float) * (.00875 * math.pi / 180.))
    np.testing.assert_array_equal(accel, np.asarray(raw[3:], dtype=float) * (.000122 * 9.80665))
    np.testing.assert_array_equal(raw, before)


def test_partial_replay_configuration_keeps_original_defaults():
    gyros, accels = effective_input_scales({})
    assert gyros == {str(n): .00875 for n in range(1, 13)}
    assert accels == {str(n): .000122 for n in range(1, 13)}
    validate_input_contract({})


def test_live_and_replay_reject_incompatible_partial_contract_before_input(tmp_path):
    from vt_fsglove12.live_stream import OnlineFusion
    from vt_fsglove12.replay import replay_raw

    config = {"input_contract": {"field_order": ["ax", "ay", "az", "gx", "gy", "gz"]}}
    with pytest.raises(ValueError, match="input_contract field_order"):
        OnlineFusion(config)
    output = tmp_path / "not_created.jsonl"
    with pytest.raises(ValueError, match="input_contract field_order"):
        replay_raw(tmp_path / "not_opened.csv", config, output)
    assert not output.exists()


@pytest.mark.parametrize("values", [[0] * 5, [[0] * 6], [0,0,0,0,0,float("nan")]])
def test_conversion_rejects_invalid_raw_shape_and_nonfinite(values):
    with pytest.raises(ValueError, match="six finite"):
        raw_to_si(values, gyro_dps_per_lsb=.00875, accel_g_per_lsb=.000122)


@pytest.mark.parametrize("bad", [0, -1, True, float("nan"), float("inf"), 10**400])
def test_conversion_rejects_invalid_scale(bad):
    with pytest.raises(ValueError, match="positive"):
        raw_to_si([0]*6, gyro_dps_per_lsb=bad, accel_g_per_lsb=.000122)
