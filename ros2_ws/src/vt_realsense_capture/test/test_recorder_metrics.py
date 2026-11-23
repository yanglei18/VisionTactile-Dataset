import json
import os
from types import SimpleNamespace

import pytest

from vt_realsense_capture.recorder_metrics import (
    RecorderMetricsReader, SEMANTICS, project_recorder_metrics,
)


@pytest.fixture
def observation(tmp_path):
    now = [10_000_000_000]
    data = dict(schema_version=1, session_id="session", bag_path=str(tmp_path / "bag"),
                recorder_instance_id="instance", pid=42, observed_monotonic_ns=now[0],
                closed=False, storage_write_semantics=SEMANTICS,
                writer_errors=0, storage_errors=0, topics=[dict(topic="/data",
                writer_received_messages=10, writer_received_bytes=1_000_000,
                storage_returned_messages=7, storage_returned_bytes=700_000,
                storage_returned_last_monotonic_ns=now[0])])
    reader = RecorderMetricsReader(session_id="session", bag_path=tmp_path / "bag",
        instance_id="instance", topics=("/data",), clock=lambda: now[0])
    def save():
        (tmp_path / "recorder-metrics.json").write_text(json.dumps(data))
    return reader, data, now, save


def test_unknown_is_not_an_observed_zero_and_rate_uses_producer_time(observation):
    reader, data, now, save = observation
    assert reader.read()["available"] is False
    save()
    first = reader.read()
    assert first["available"] is True and first["rate_available"] is False
    assert first["topics"][0]["storage_returned_messages"] == 7
    now[0] += 1_000_000_000
    assert reader.read()["rate_available"] is False  # A reread is not a new sample.
    data["observed_monotonic_ns"] += 500_000_000
    data["topics"][0]["storage_returned_messages"] = 10
    data["topics"][0]["storage_returned_bytes"] = 1_000_000
    save()
    result = reader.read()
    assert result["rate_available"] is True
    assert result["storage_returned_mb_s"] == pytest.approx(.6)
    assert reader.read()["observed_monotonic_ns"] == data["observed_monotonic_ns"]
    now[0] += 4_000_000_000
    assert reader.read()["available"] is False


@pytest.mark.parametrize("key,value", [
    ("session_id", "other"), ("bag_path", "/other/bag"),
    ("recorder_instance_id", "other"), ("pid", 0),
    ("observed_monotonic_ns", 0), ("storage_write_semantics", "disk_fsync"),
    ("closed", "false"), ("writer_errors", -1),
])
def test_foreign_or_invalid_metrics_are_unavailable(observation, key, value):
    reader, data, _, save = observation
    data[key] = value
    save()
    assert reader.read()["available"] is False


def test_closed_snapshot_can_be_read_later_but_cannot_regress(observation):
    reader, data, now, save = observation
    data["closed"] = True
    save()
    now[0] += 100_000_000_000
    assert reader.read()["available"] is True
    data["topics"][0]["writer_received_messages"] -= 1
    save()
    assert reader.read()["available"] is False


def test_same_timestamp_cannot_claim_changed_measurement(observation):
    reader, data, _, save = observation
    save()
    assert reader.read()["available"]
    data["topics"][0]["writer_received_messages"] += 1
    save()
    assert not reader.read()["available"]


def test_unknown_projection_preserves_unknown_and_known_topic_counts(observation):
    reader, _, _, save = observation
    def message():
        return SimpleNamespace(header=SimpleNamespace(), available=False,
                               rate_available=False, topics=[])
    unknown = project_recorder_metrics(message(), None, "stamp", SimpleNamespace,
                                      session_id="session")
    assert unknown.available is False and unknown.rate_available is False
    save()
    result = project_recorder_metrics(message(), reader.read(), "stamp", SimpleNamespace)
    assert result.recorder_pid == 42
    assert result.topics[0].storage_returned_messages == 7


def test_standard_entry_preserves_qos_storage_and_topics(tmp_path):
    from vt_realsense_capture.recorder import (
        build_record_command, build_monitored_record_command, required_topics,
    )
    config = tmp_path / "config.yaml"
    config.write_text("{}")
    arguments = dict(output_path=tmp_path / "bag", storage_config_path=config,
        qos_overrides_path=config, topics=required_topics(("d405_1", "d405_2", "d436")))
    original = build_record_command(**arguments)
    observed = build_monitored_record_command(
        **arguments, session_id="session", recorder_instance_id="instance")
    assert observed[:4] == ["ros2", "run", "vt_realsense_capture", "monitored_recorder"]
    assert observed[10:] == original[3:]
    assert observed[9] == str(tmp_path / "recorder-metrics.json")


def test_nonregular_metrics_file_cannot_block_control_loop(observation):
    reader, data, _, _ = observation
    from pathlib import Path
    path = Path(data["bag_path"]).parent / "recorder-metrics.json"
    os.mkfifo(path)
    assert reader.read()["available"] is False
    path.unlink()
    path.symlink_to("/dev/zero")
    assert reader.read()["available"] is False
