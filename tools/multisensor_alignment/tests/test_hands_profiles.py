from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import replace
from io import StringIO
from itertools import product
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from vt_multisensor_alignment.bag_reader import configured_topic_types, read_unified_bag, require_topic_types
from vt_multisensor_alignment.cli import main
from vt_multisensor_alignment.config import load_config
from vt_multisensor_alignment.glove_adapter import GloveStateAdapter
from vt_multisensor_alignment.matcher import select_generic_sample
from vt_multisensor_alignment.model import MessageRef
from vt_multisensor_alignment.profiles import make_profiles, parse_hands, profile_documents, selected_streams
from vt_multisensor_alignment.tactile_export import export_tactile, iter_tactile_events


ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT.parents[1] / "ros2_ws/src/vt_realsense_capture/config"
COMBINATIONS = ((), ("left",), ("right",), ("left", "right"))
PROFILES = ("left_glove", "both_gloves", "left_glove_tactile", "both_gloves_tactile",
            "left_tactile", "right_tactile", "both_tactile")


@pytest.mark.parametrize("pose,tactile", product(COMBINATIONS, repeat=2))
def test_all_sixteen_selections_preserve_core_and_pair_topics(tmp_path, pose, tactile):
    output = make_profiles(capture_template=CAPTURE / "cameras.yaml",
                           alignment_template=ROOT / "config/alignment.example.yaml",
                           output_directory=tmp_path / "profiles", pose_hands=pose, tactile_hands=tactile)
    capture = yaml.safe_load((output / "cameras.yaml").read_text())
    alignment = load_config(output / "alignment.yaml")
    original = yaml.safe_load((CAPTURE / "cameras.yaml").read_text())
    assert capture["recording"].pop("additional_streams") == [
        {"topic": stream.topic, "type": stream.type_name} for stream in alignment.additional_streams]
    original["recording"].pop("additional_streams")
    assert capture == original
    assert len(alignment.cameras) == len(alignment.trackers) == 3
    assert len(alignment.required_topics) == 15 + len(pose)
    assert len(alignment.additional_streams) == len(pose) + len(tactile)
    assert {stream.identity[0] for stream in alignment.additional_streams} == set(pose) | set(tactile)


@pytest.mark.parametrize("profile", PROFILES)
def test_checked_in_profiles_match_capture(profile):
    alignment = load_config(ROOT / f"config/alignment.{profile}.yaml")
    capture = yaml.safe_load((CAPTURE / f"cameras.with_{profile}.yaml").read_text())
    assert capture["recording"]["additional_streams"] == [
        {"topic": stream.topic, "type": stream.type_name} for stream in alignment.additional_streams]


def test_cli_selection_palm_override_and_required_tactile(tmp_path):
    output = StringIO()
    with redirect_stdout(output):
        status = main(["make-profiles", "--capture-template", str(CAPTURE / "cameras.yaml"),
                       "--alignment-template", str(ROOT / "config/alignment.example.yaml"),
                       "--output", str(tmp_path / "pair"), "--pose-hands", "both",
                       "--tactile-hands", "left", "--left-reference-node-id", "11", "--require-tactile"])
    assert status == 0
    paths = json.loads(output.getvalue())
    config = load_config(paths["alignment_config"])
    streams = {stream.name: stream for stream in config.additional_streams}
    assert streams["left_glove"].reference_node_id == 11
    assert streams["right_glove"].reference_node_id is None
    assert streams["left_tactile"].required
    with pytest.raises(FileExistsError):
        make_profiles(capture_template=CAPTURE / "cameras.yaml",
                      alignment_template=ROOT / "config/alignment.example.yaml",
                      output_directory=tmp_path / "pair")


def test_profile_selector_does_not_mutate_inputs_or_keep_deselected_qos():
    capture = {"recording": {"additional_streams": [{"topic": "/gloves/right/state"}],
                             "qos_overrides": {"/gloves/right/state": {}, "/d405_1/frame_timing": {}}}}
    original = deepcopy(capture)
    selected, _ = profile_documents(capture, {"additional_streams": []}, tactile_hands=("left",))
    assert capture == original
    assert selected["recording"]["qos_overrides"] == {"/d405_1/frame_timing": {}}
    for value in ("", "left,left", "front", "left, right"):
        with pytest.raises(ValueError):
            parse_hands(value)
    assert parse_hands("none") == ()
    assert parse_hands("right,left") == ("left", "right")
    with pytest.raises(ValueError):
        selected_streams(pose_hands=("left",), reference_node_ids={"right": 12})


@pytest.mark.parametrize("updates", [{"expected_hand": "right"}, {"reference_node_id": 0},
                                     {"reference_node_id": True}, {"unexpected_identity": "left"}])
def test_invalid_identity_config_rejected(tmp_path, updates):
    document = yaml.safe_load((ROOT / "config/alignment.left_glove.yaml").read_text())
    document["additional_streams"][0].update(updates)
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_config(path)


def stamp(value):
    return NS(sec=0, nanosec=value)


def glove(hand, time, *, node=12, **updates):
    values = dict(hand=hand, header=NS(stamp=stamp(time), frame_id=f"glove_{hand}_reference"),
                  host_realtime_ns=time, host_monotonic_ns=time + 1000,
                  session_id="same-session", frame_index=0, calibration_version=1,
                  pose_valid=True, time_valid=True, source_mode="live",
                  clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
                  reference_node_id=node, source_node_ids=[1, 2, node], source_filter_epochs=[0, 0, 0],
                  source_realtime_ns=[time - 1, time - 1, time],
                  source_monotonic_ns=[time + 999, time + 999, time + 1000])
    values.update(updates)
    return NS(**values)


def tactile(hand, time, sequence, *, adc=7, **updates):
    values = dict(hand=hand, header=NS(stamp=stamp(time), frame_id=f"{hand}_tactile"),
                  schema_version=1, session_id="same-session", stream_epoch=0, sequence=sequence,
                  first_packet_index=sequence, host_realtime_ns=time, host_monotonic_ns=time + 1000,
                  clock_domain="CLOCK_MONOTONIC", timing_source="host_receive", connected=True,
                  time_valid=True, reason="", producer_dropped_total=0, discarded_bytes_total=0,
                  malformed_packets_total=0,
                  blocks=[NS(row=0, column_group=0, status_raw=0, adc=[adc] * 10)])
    values.update(updates)
    return NS(**values)


def core_events(config):
    events = []
    for camera in config.cameras:
        header = NS(stamp=stamp(100), frame_id=camera.frame_id)
        events.extend((topic, NS(header=header), 100) for topic in (
            camera.color_topic, camera.depth_topic, camera.camera_info_topic))
        events.append((camera.timing_topic, NS(header=header, camera_name=camera.name,
                       camera_model=camera.model, serial_number=camera.serial,
                       GROUP_VALID_COMMON_STAMP=1, GROUP_VALID_IDENTITY=2, GROUP_VALID_CALLBACK_CLOCKS=8,
                       group_validity_flags=11, shared_ros_timestamp_ns=100,
                       group_host_realtime_ns=100, group_host_monotonic_raw_ns=1000), 100))
    for tracker in config.trackers:
        for time in (90, 210):
            events.append((tracker.sample_topic, NS(header=NS(stamp=stamp(time), frame_id=config.world_frame),
                           role=tracker.role, tracker_id="a" * 64, host_realtime_ns=time,
                           host_monotonic_ns=time + 1000, pose_valid=True, tracking_status=2,
                           pose=NS(position=NS(x=0, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1))), time))
    return events


def read_fake_bag(tmp_path, monkeypatch, config, events):
    class Reader:
        def get_all_topics_and_types(self):
            return [NS(name=name, type=kind) for name, kind in configured_topic_types(config).items()]

        def has_next(self):
            return bool(events)

        def read_next(self):
            return events.pop(0)

    monkeypatch.setattr("vt_multisensor_alignment.bag_reader._open_reader", lambda _: Reader())
    monkeypatch.setattr("vt_multisensor_alignment.bag_reader._deserializer", lambda _: lambda value: value)
    monkeypatch.setattr("vt_multisensor_alignment.bag_reader._storage_identifier", lambda _: "mcap")
    return read_unified_bag(tmp_path, config)


def test_two_hand_bag_uses_independent_adapters_and_accepts_left_palm_11(tmp_path, monkeypatch):
    config = load_config(ROOT / "config/alignment.both_gloves_tactile.yaml")
    events = core_events(config)
    for hand in ("left", "right"):
        node = 11 if hand == "left" else 12
        pose = [glove(hand, 100, node=node), glove(hand, 200, node=node)]
        touch = [tactile(hand, 100, 0), tactile(hand, 150, 1), tactile(hand, 200, 2)]
        if hand == "left":
            pose.insert(1, glove(hand, 150, node=node, pose_valid=False))
            touch[1] = tactile(hand, 150, 1, connected=False, blocks=[])
            touch[2].first_packet_index = 1
        events.extend((f"/gloves/{hand}/state", message, 1000 + i) for i, message in enumerate(pose))
        events.extend((f"/gloves/{hand}/tactile/samples", message, 2000 + i) for i, message in enumerate(touch))
    dataset = read_fake_bag(tmp_path, monkeypatch, config, events)
    assert set(dataset.additional_samples) == {"left_glove", "right_glove", "left_tactile", "right_tactile"}
    for kind in ("glove", "tactile"):
        assert select_generic_sample(dataset.additional_samples[f"left_{kind}"], timestamp_ns=175,
                                     strategy="previous", max_delta_ns=1000) is None
        assert select_generic_sample(dataset.additional_samples[f"right_{kind}"], timestamp_ns=175,
                                     strategy="previous", max_delta_ns=1000) is not None
    assert len([key for key in dataset.clock_observations if key.startswith("glove:")]) == 2
    assert len([key for key in dataset.clock_observations if key.startswith("tactile:")]) == 2
    right_config = load_config(ROOT / "config/alignment.right_glove_tactile.yaml")
    with pytest.raises(ValueError, match="unconfigured topic"):
        require_topic_types(configured_topic_types(config), right_config)


@pytest.mark.parametrize("kind,wrong", [("glove", "hand"), ("glove", "frame"),
                                        ("tactile", "hand"), ("tactile", "frame")])
def test_crossed_hand_or_frame_fails_before_alignment(tmp_path, monkeypatch, kind, wrong):
    config = load_config(ROOT / "config/alignment.both_gloves_tactile.yaml")
    message = glove("left", 100) if kind == "glove" else tactile("left", 100, 0)
    if wrong == "hand":
        message.hand = "right"
    else:
        message.header.frame_id = "glove_right_reference" if kind == "glove" else "right_tactile"
    topic = "/gloves/left/state" if kind == "glove" else "/gloves/left/tactile/samples"
    with pytest.raises(ValueError, match="mismatch"):
        read_fake_bag(tmp_path, monkeypatch, config, [(topic, message, 100)])


def test_glove_reference_identity_and_custom_topics(tmp_path, monkeypatch):
    def add(adapter, message):
        adapter.add(message, MessageRef("/legacy/pose", 0, 100, 100), header_stamp_ns=100)

    message = glove("left", 100, node=11)
    add(GloveStateAdapter("left", reference_node_id=11), message)
    with pytest.raises(ValueError, match="reference_node_id mismatch"):
        add(GloveStateAdapter("left", reference_node_id=12), message)
    message.source_realtime_ns[-1] = 99
    with pytest.raises(ValueError, match="reference source clock mismatch"):
        add(GloveStateAdapter("left"), message)
    config = load_config(ROOT / "config/alignment.left_glove.yaml")
    stream = replace(config.additional_streams[0], topic="/legacy/pose")
    assert stream.identity == (None, None)
    config = replace(config, additional_streams=(stream,))
    message = glove("left", 100, node=11)
    message.header.frame_id = "legacy_left_reference"
    dataset = read_fake_bag(tmp_path, monkeypatch, config,
                            core_events(config) + [(stream.topic, message, 100)])
    assert len(dataset.additional_samples[stream.name]) == 1


def tactile_reader(monkeypatch, messages):
    filters = []

    class Reader:
        def get_all_topics_and_types(self):
            return [NS(name=topic, type="vt_tactile_msgs/msg/TactileSampleArray")
                    for topic in {record[0] for record in messages}]

        def set_filter(self, filter):
            filters.append(filter.topics)
            self.messages = [record for record in messages if record[0] in filter.topics]

        def has_next(self):
            return bool(self.messages)

        def read_next(self):
            return self.messages.pop(0)

    monkeypatch.setitem(sys.modules, "rosbag2_py", NS(StorageFilter=lambda **kw: NS(**kw)))
    monkeypatch.setattr("vt_multisensor_alignment.tactile_export._open_reader", lambda _: Reader())
    monkeypatch.setattr("vt_multisensor_alignment.tactile_export._deserializer", lambda _: lambda value: value)
    return filters


def test_tactile_export_two_topics_keep_grids_metadata_and_directories_separate(tmp_path, monkeypatch):
    messages = [(f"/gloves/{hand}/tactile/samples", tactile(hand, time, sequence, adc=adc), time)
                for hand, adc in (("left", 17), ("right", 29))
                for sequence, time in enumerate((100, 200))]
    filters = tactile_reader(monkeypatch, messages)
    frames = tmp_path / "aligned_frames.jsonl"
    frames.write_text("\n".join(json.dumps({"frame_index": i, "reference_time_ns": time})
                                for i, time in enumerate((100, 200))))
    for hand, adc in (("left", 17), ("right", 29)):
        topic = f"/gloves/{hand}/tactile/samples"
        with pytest.warns(UserWarning):
            output = export_tactile(bag_path=tmp_path, aligned_frames=frames,
                                    output_directory=tmp_path / hand, topic=topic)
        manifest = json.loads((output / "manifest.json").read_text())
        assert manifest["source_topic"] == topic
        assert manifest["expected_hand"] == hand
        with np.load(output / "frames/000000001.npz") as values:
            assert values["adc"][0, 0] == adc
            assert values["block_message_sequence"][0, 0] == 1
    assert filters == [["/gloves/left/tactile/samples"], ["/gloves/right/tactile/samples"]]


def test_tactile_export_rejects_crossed_topic_identity(tmp_path, monkeypatch):
    topic = "/gloves/left/tactile/samples"
    tactile_reader(monkeypatch, [(topic, tactile("right", 100, 0), 100)])
    with pytest.raises(ValueError, match="hand mismatch"):
        list(iter_tactile_events(tmp_path, topic=topic))


def test_real_two_hand_mcap_roundtrip(tmp_path):
    rosbag = pytest.importorskip("rosbag2_py")
    serialization = pytest.importorskip("rclpy.serialization")
    utilities = pytest.importorskip("rosidl_runtime_py.utilities")
    setters = pytest.importorskip("rosidl_runtime_py.set_message")
    pytest.importorskip("vt_glove_msgs.msg")
    pytest.importorskip("vt_tactile_msgs.msg")
    config = load_config(ROOT / "config/alignment.both_gloves_tactile.yaml")
    topic_types = configured_topic_types(config)
    bag = tmp_path / "dual"
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag.ConverterOptions("", ""))
    for identifier, (topic, kind) in enumerate(topic_types.items(), start=1):
        writer.create_topic(rosbag.TopicMetadata(id=identifier, name=topic, type=kind, serialization_format="cdr"))
    events = core_events(config)
    for hand in ("left", "right"):
        for index, time in enumerate((100, 200)):
            events.append((f"/gloves/{hand}/state", glove(hand, time, node=11 if hand == "left" else 12), time))
            events.append((f"/gloves/{hand}/tactile/samples", tactile(hand, time, index), time))

    def fields(value):
        if isinstance(value, NS):
            return {name: fields(item) for name, item in vars(value).items() if not name.isupper()}
        if isinstance(value, list):
            return [fields(item) for item in value]
        return value

    for topic, source, receipt in sorted(events, key=lambda event: event[2]):
        message = utilities.get_message(topic_types[topic])()
        setters.set_message_fields(message, fields(source))
        writer.write(topic, serialization.serialize_message(message), receipt)
    writer.close()
    dataset = read_unified_bag(bag, config)
    assert len(dataset.topic_types) == 19
    assert {name: len(samples) for name, samples in dataset.additional_samples.items()} == {
        "left_glove": 2, "right_glove": 2, "left_tactile": 2, "right_tactile": 2}
    for hand in ("left", "right"):
        events = list(iter_tactile_events(bag, topic=f"/gloves/{hand}/tactile/samples"))
        assert [event.hand for event in events] == [hand, hand]
        assert [event.reference.sequence for event in events] == [0, 1]
