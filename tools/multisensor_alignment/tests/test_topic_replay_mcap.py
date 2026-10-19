"""Real MCAP serialization for partial bags; no publishers or devices."""
import gc

import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")
serialization = pytest.importorskip("rclpy.serialization")
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vt_multisensor_alignment.topic_replay_worker import TopicReplaySource


@pytest.fixture
def partial_bag(tmp_path):
    bag = tmp_path / "session" / "bag"
    bag.parent.mkdir()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    topics = [("/rgb", "sensor_msgs/msg/Image"), ("/note", "std_msgs/msg/String"),
              ("/tracker", "vt_tracker_msgs/msg/TrackerSample"),
              ("/opaque", "not_installed/msg/Opaque")]
    for index, (topic, kind) in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(id=index, name=topic, type=kind, serialization_format="cdr"))
    image = Image(height=2, width=2, encoding="rgb8", step=6, data=[255, 0, 0] * 4)
    writer.write("/rgb", serialization.serialize_message(image), 1_000_000_000)
    writer.write("/note", serialization.serialize_message(String(data="recorded only")), 1_500_000_000)
    writer.write("/opaque", b"\x00\x01\x00\x00unknown", 1_600_000_000)
    writer.write("/rgb", serialization.serialize_message(image), 2_000_000_000)
    del writer
    gc.collect()
    return bag


def test_camera_only_bag_plays_without_tracker_or_geometry(partial_bag):
    source = TopicReplaySource(partial_bag)
    try:
        inventory = {entry["name"]: entry for entry in source.metadata["topics"]}
        assert inventory["/rgb"]["count"] == 2
        assert inventory["/tracker"]["count"] == 0
        assert source.metadata["warnings"]
        frame = source.frame("/rgb", 1_800_000_000)
        assert frame["sample_ns"] == 1_000_000_000
        assert frame["device"]["kind"] == "camera"
        assert frame["device"]["preview"]
        assert source.frame("/note", 1_400_000_000)["sample_ns"] is None
        note = source.frame("/note", 1_800_000_000)
        assert "recorded only" in str(note["details"])
        empty = source.frame("/tracker", 1_800_000_000)
        assert empty["sample_ns"] is None
    finally:
        source.close()


def test_missing_message_type_does_not_block_other_streams(partial_bag):
    source = TopicReplaySource(partial_bag.parent)
    try:
        opaque = source.frame("/opaque", 2_000_000_000)
        assert opaque["sample_ns"] == 1_600_000_000
        assert opaque["warnings"]
        assert opaque["details"]
        assert source.frame("/rgb", 2_000_000_000)["device"] is not None
    finally:
        source.close()


def test_real_storage_symlink_outside_bag_is_rejected(partial_bag):
    payload = next(partial_bag.glob('*.mcap'))
    outside = partial_bag.parent / 'external.mcap'
    payload.rename(outside)
    payload.symlink_to(outside)
    with pytest.raises(ValueError, match='symlink'):
        TopicReplaySource(partial_bag)
