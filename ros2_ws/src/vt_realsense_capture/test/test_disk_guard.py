from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from vt_realsense_capture.config import AdditionalStreamConfig, load_config
from vt_realsense_capture.disk_guard import DiskGuardConfig, recording_space_budget


PATH = Path(__file__).parents[1] / "config/cameras.yaml"
CONFIG = load_config(PATH)


def test_raw_budget_has_no_compression_assumption_and_includes_cache_and_reserve():
    budget = recording_space_budget(CONFIG, 30)
    assert budget.bytes_per_second == 456_192_000  # 414,720,000 raw RGB8 + Z16 plus 10%.
    assert budget.stop_reserve_bytes >= CONFIG.max_cache_size_bytes + 3 * budget.bytes_per_second
    assert budget.stop_reserve_bytes >= 2 * 1024**3
    assert budget.required_start_bytes == 30 * budget.bytes_per_second + budget.stop_reserve_bytes
    assert not budget.additional_streams_unbudgeted


def test_unknown_additional_topics_are_explicitly_outside_estimate():
    config = replace(CONFIG, additional_streams=(AdditionalStreamConfig("/extra", "pkg/msg/Extra"),))
    before = recording_space_budget(CONFIG, 30)
    after = recording_space_budget(config, 30)
    assert before.required_start_bytes == after.required_start_bytes
    assert after.additional_streams_unbudgeted
    assert "additional topics have no configured size estimate" in after.describe()
    config = replace(config, disk_guard=DiskGuardConfig(additional_stream_bytes_per_second=1_000_000))
    explicit = recording_space_budget(config, 30)
    assert explicit.bytes_per_second == before.bytes_per_second + 1_000_000
    assert not explicit.additional_streams_unbudgeted
    assert explicit.required_start_bytes > after.required_start_bytes


def test_custom_guard_settings_load_without_changing_capture_contract(tmp_path):
    document = yaml.safe_load(PATH.read_text())
    document["recording"]["disk_guard"] = dict(min_free_bytes=8 * 1024**3,
        check_interval_sec=2, additional_stream_bytes_per_second=123456)
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(document))
    config = load_config(path)
    assert config.disk_guard == DiskGuardConfig(8 * 1024**3, 2, 123456)
    assert config.cameras == CONFIG.cameras
    assert recording_space_budget(config, 30).stop_reserve_bytes == 8 * 1024**3


@pytest.mark.parametrize("settings", [
    {"min_free_bytes": 0}, {"min_free_bytes": -1}, {"min_free_bytes": True},
    {"min_free_bytes": 1.5}, {"check_interval_sec": 0.09}, {"check_interval_sec": 5.1},
    {"check_interval_sec": float("nan")}, {"check_interval_sec": float("inf")},
    {"check_interval_sec": True}, {"check_interval_sec": "1"},
    {"additional_stream_bytes_per_second": -1}, {"additional_stream_bytes_per_second": True},
    {"additional_stream_bytes_per_second": 1.5}, {"enabled": False}, [], None,
])
def test_invalid_guard_configuration_rejected(tmp_path, settings):
    document = yaml.safe_load(PATH.read_text())
    document["recording"]["disk_guard"] = settings
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="disk_guard"):
        load_config(path)
