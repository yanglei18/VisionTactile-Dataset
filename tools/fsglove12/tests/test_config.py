import pytest

from vt_fsglove12.config import check_proposed_mount, layout_report, load_config, validate_config


def test_current_layout_does_not_invent_thumb_metacarpal_sensor():
    config = load_config()
    report = layout_report(config)
    assert config["node_to_slot"]["5"] == 14
    assert config["node_to_slot"]["6"] == 15
    assert report["sensors_mapped"] == 11
    assert report["sensors_unmapped"] == [11]
    assert report["segments"][13]["source"] == "requires_prior"
    assert not report["mounting_confirmed"]


def test_no_duplicate_bone_mapping():
    config = load_config()
    config["node_to_slot"]["11"] = 14
    with pytest.raises(ValueError, match="unique"):
        validate_config(config)


def test_nonfinite_units_rejected():
    config = load_config()
    config["gyro_dps_per_lsb"] = float("nan")
    with pytest.raises(ValueError, match="gyro_dps_per_lsb"):
        validate_config(config)


def test_proposed_mapping_does_not_imply_hardware_was_moved():
    config = load_config()
    config["layout"] = "proposed_1_palm_3_thumb_8_other_finger"
    config["node_to_slot"]["11"] = 13
    with pytest.raises(ValueError, match="physically confirmed"):
        check_proposed_mount(config)
    config["mounting_confirmed"] = True
    check_proposed_mount(config)


def test_gap_reset_policy_is_explicit_and_separate_from_display_timeout():
    config = load_config()
    assert config["stale_after_s"] == .15
    assert config["filter_reset_gap_s"] == 1.
    config.pop("filter_reset_gap_s")
    validated = validate_config(config)
    assert validated["filter_reset_gap_s"] == 1.
    assert "filter_reset_gap_s" not in config
    config["stale_after_s"] = 2.
    assert validate_config(config)["filter_reset_gap_s"] == 2.
    config.update(stale_after_s=.15, filter_reset_gap_s=.15)
    assert validate_config(config)["filter_reset_gap_s"] == .15  # Explicit legacy replay policy.


@pytest.mark.parametrize("gap", [0, -.1, .149, True, False, None, "1", float("nan"), float("inf"), 10**400])
def test_invalid_filter_reset_gap_is_rejected(gap):
    config = load_config()
    config["filter_reset_gap_s"] = gap
    with pytest.raises(ValueError, match="filter_reset_gap_s"):
        validate_config(config)
