"""Optional sources may be absent; their recorded payloads still require checks."""

from pathlib import Path
import struct
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
import yaml

from vt_multisensor_alignment import capture_quality as quality
from vt_multisensor_alignment.config import load_config


def generate(tmp_path, topics, *, optional=(), rates=None, config=None, events=()):
    bag = tmp_path / "bag"
    bag.mkdir()
    info = dict(storage_identifier="mcap", duration=dict(nanoseconds=2_000_000_000),
                starting_time=dict(nanoseconds_since_epoch=1_000_000_000),
                message_count=sum(count for _, count in topics.values()),
                topics_with_message_count=[
                    dict(topic_metadata=dict(name=topic, type=kind), message_count=count)
                    for topic, (kind, count) in topics.items()])
    (bag / "metadata.yaml").write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=info)))
    pending = list(events)
    reader = NS(has_next=lambda: bool(pending), read_next=lambda: pending.pop(0))
    with patch.object(quality, "_open_reader", return_value=reader), \
         patch.object(quality, "_set_filter"):
        return quality.generate_capture_quality(
            bag, tmp_path / "quality.json", config,
            optional_topics=optional, expected_image_rates_hz=rates)


def test_empty_optional_sources_are_coverage_not_warnings(tmp_path):
    topics = {"/other": ("unknown/msg/Custom", 1), "/tracker": (quality.TRACKER_TYPE, 0),
              "/image": (quality.IMAGE_TYPE, 0), "/tactile": (quality.TACTILE_SAMPLE_TYPE, 0)}
    with patch.object(quality, "_deserializer", side_effect=AssertionError("no payload to decode")):
        report = generate(tmp_path, topics, optional=["/tracker", "/absent", "/image", "/tactile"],
                          rates={"/image": 30.})
    assert report["status"] == "ok"
    assert report["warnings"] == []
    assert report["topics"]["/tracker"]["recorded_message_count"] == 0
    assert report["topic_coverage"] == dict(
        optional_topics=["/absent", "/image", "/tactile", "/tracker"],
        absent_optional_topics=["/absent", "/image", "/tactile", "/tracker"])
    assert report["topics"]["/image"]["rate_assessment"] == "insufficient_samples"


def test_optional_configured_sources_do_not_create_missing_warnings(tmp_path):
    config = load_config(Path(__file__).resolve().parents[1] / "config/alignment.example.yaml")
    required = list(config.required_topics)
    report = generate(tmp_path, {"/other": ("unknown/msg/Custom", 1)},
                      optional=required[1:], config=config)
    assert {w["topic"] for w in report["warnings"]} == {required[0]}
    assert all(w["code"] == "expected_topic_missing" for w in report["warnings"])


def test_absent_optional_image_can_retain_its_intended_rate(tmp_path):
    report = generate(tmp_path, {"/other": ("unknown/msg/Custom", 1)},
                      optional=["/image"], rates={"/image": 30.})
    assert report["status"] == "ok"
    assert report["topic_coverage"]["absent_optional_topics"] == ["/image"]


def test_optional_image_payload_faults_and_low_rate_remain_warnings(tmp_path):
    stamps = [1_000_000_000 + round(i * 1_000_000_000 / 15) for i in range(31)]
    stamps.extend([stamps[-1], stamps[-1] - 1, 0])
    events = [("/image", b"\0\1\0\0" + struct.pack("<iII", ns // 1_000_000_000,
               ns % 1_000_000_000, 1) + b"\0", 1_000_000_000 + i) for i, ns in enumerate(stamps)]
    events.append(("/image", b"bad", 3_000_000_000))
    report = generate(tmp_path, {"/image": (quality.IMAGE_TYPE, len(events))},
                      optional=["/image"], rates={"/image": 30.}, events=events)
    assert report["topic_coverage"]["absent_optional_topics"] == []
    assert {w["code"] for w in report["warnings"]} >= {
        "malformed_messages", "invalid_time_messages", "image_source_duplicates",
        "image_source_regressions", "image_source_gaps", "image_rate_below_expected"}


def test_one_optional_image_still_reports_insufficient_rate_evidence(tmp_path):
    payload = b"\0\1\0\0" + struct.pack("<iII", 1, 0, 1) + b"\0"
    report = generate(tmp_path, {"/image": (quality.IMAGE_TYPE, 1)},
                      optional=["/image"], rates={"/image": 30.},
                      events=[("/image", payload, 1_000_000_000)])
    assert [w["code"] for w in report["warnings"]] == ["image_timing_insufficient"]


def test_recorded_optional_tracker_still_reports_invalid_pose_and_malformed_payload(tmp_path):
    message = NS(header=NS(stamp=NS(sec=1, nanosec=0)), tracker_id="tracker",
                 pose_valid=False, host_realtime_ns=1_000_000_000,
                 host_monotonic_ns=1_000_000_100, packet_index=1, tracking_status=0)
    def decode(payload):
        if payload == b"bad":
            raise ValueError("malformed TrackerSample")
        return message
    with patch.object(quality, "_deserializer", return_value=decode):
        report = generate(tmp_path, {"/tracker": (quality.TRACKER_TYPE, 2)},
                          optional=["/tracker"], events=[("/tracker", b"good", 1_000_000_000),
                                                       ("/tracker", b"bad", 1_000_000_001)])
    assert report["topic_coverage"]["absent_optional_topics"] == []
    assert {w["code"] for w in report["warnings"]} >= {"invalid_pose_messages", "malformed_messages"}


def test_empty_optional_bag_still_reports_empty_bag(tmp_path):
    report = generate(tmp_path, {"/image": (quality.IMAGE_TYPE, 0)}, optional=["/image"])
    assert [w["code"] for w in report["warnings"]] == ["empty_bag"]


@pytest.mark.parametrize("optional", ["/tracker", {"/tracker"}, ["relative"], [True],
                                      ["/tracker", "/tracker"], ["/bad-name"],
                                      [f"/topic_{i}" for i in range(257)]])
def test_invalid_optional_topics_fail_before_report(tmp_path, optional):
    with pytest.raises(ValueError, match="optional_topics"):
        generate(tmp_path, {"/other": ("unknown/msg/Custom", 1)}, optional=optional)
    assert not (tmp_path / "quality.json").exists()
