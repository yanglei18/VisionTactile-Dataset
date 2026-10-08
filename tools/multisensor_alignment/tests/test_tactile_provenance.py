import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace as NS
import warnings
from unittest.mock import patch

import numpy as np
import pytest
import yaml

from vt_multisensor_alignment.model import MessageRef
from vt_multisensor_alignment.provenance import bag_identity, sha256_file, verify_alignment_source
from vt_multisensor_alignment.tactile_adapter import DEFAULT_TACTILE_TOPIC as TOPIC
from vt_multisensor_alignment.tactile_adapter import TACTILE_SAMPLE_TYPE, TactileContinuity
from vt_multisensor_alignment.tactile_export import _QualityAccumulator, export_tactile, iter_tactile_events


def make_bag(parent, name="bag"):
    bag = parent / name
    bag.mkdir()
    (bag / "part_0.mcap").write_bytes(b"storage part zero")
    (bag / "part_1.mcap").write_bytes(b"storage part one")
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "storage_identifier": "mcap", "relative_file_paths": ["part_0.mcap", "part_1.mcap"],
    }}))
    return bag


def bind_index(parent, bag, *, legacy=False):
    directory = parent / "alignment"
    directory.mkdir()
    frames = directory / "aligned_frames.jsonl"
    frames.write_text('{"frame_index":0,"reference_time_ns":110}\n')
    source = bag_identity(bag)
    if legacy:
        source = {"metadata_sha256": source["metadata_sha256"]}
    manifest = {"schema_version": 1, "tool": "vt-multisensor-alignment", "tool_version": "0.1.0",
                "source_bag": source,
                "files": {"aligned_frames.jsonl": {"size_bytes": frames.stat().st_size,
                                                      "sha256": sha256_file(frames)}}}
    (directory / "manifest.json").write_text(json.dumps(manifest))
    return frames


def source_message(time_ns=100, *, sequence=0, full=False):
    pairs = [(row, group) for row in range(20) for group in range(8)] if full else [(0, 0)]
    return NS(header=NS(stamp=NS(sec=0, nanosec=time_ns), frame_id="right_tactile"), schema_version=1,
              hand="right", session_id="test", stream_epoch=0, sequence=sequence,
              first_packet_index=sequence, host_realtime_ns=time_ns, host_monotonic_ns=time_ns + 1000,
              clock_domain="CLOCK_MONOTONIC", timing_source="host_receive", connected=True,
              time_valid=True, reason="", producer_dropped_total=0, discarded_bytes_total=0,
              malformed_packets_total=0,
              blocks=[NS(row=r, column_group=g, status_raw=0, adc=list(range(10))) for r, g in pairs])


def event(full=False):
    message = source_message(full=full)
    return TactileContinuity().decode(message, MessageRef(TOPIC, 0, 100, 100), header_stamp_ns=100)


def test_relocated_bag_and_alignment_are_valid(tmp_path):
    bag = make_bag(tmp_path)
    frames = bind_index(tmp_path, bag)
    copied = tmp_path / "another-computer"
    copied.mkdir()
    shutil.copytree(bag, copied / "renamed-bag")
    shutil.copytree(frames.parent, copied / "alignment")
    result = verify_alignment_source(copied / "renamed-bag", copied / "alignment/aligned_frames.jsonl",
                                     require_provenance=True)
    assert result["status"] == "verified_metadata_and_storage_inventory"
    assert result["source_bag_identity"]["storage_content_hashed"] is False
    assert str(tmp_path) not in json.dumps(result)


@pytest.mark.parametrize("change", ["metadata", "storage", "index_hash", "index_size"])
def test_wrong_bag_or_tampered_index_rejected_before_any_output(tmp_path, change):
    bag = make_bag(tmp_path)
    frames = bind_index(tmp_path, bag)
    if change == "metadata":
        with (bag / "metadata.yaml").open("a") as stream:
            stream.write("# different metadata\n")
    elif change == "storage":
        (bag / "part_1.mcap").write_bytes(b"different storage file length")
    elif change == "index_hash":
        frames.write_text(frames.read_text().replace("110", "111"))
    else:
        with frames.open("a") as stream:
            stream.write("\n")
    with patch("vt_multisensor_alignment.tactile_export.iter_tactile_events") as read, \
         pytest.raises(ValueError, match="mismatch"):
        export_tactile(bag_path=bag, aligned_frames=frames, output_directory=tmp_path / "out")
    read.assert_not_called()
    assert not (tmp_path / "out").exists()
    assert not list(tmp_path.glob(".out.*"))


def test_legacy_missing_manifest_is_explicitly_unverified(tmp_path):
    bag = make_bag(tmp_path)
    frames = tmp_path / "legacy.jsonl"
    frames.write_text('{"frame_index":0,"reference_time_ns":110}\n')
    with pytest.warns(UserWarning, match="UNVERIFIED"):
        result = verify_alignment_source(bag, frames)
    assert result["status"] == "unverified_missing_manifest"
    with pytest.raises(ValueError, match="manifest is missing"):
        verify_alignment_source(bag, frames, require_provenance=True)


def test_old_manifest_binds_metadata_without_claiming_storage_verification(tmp_path):
    bag = make_bag(tmp_path)
    frames = bind_index(tmp_path, bag, legacy=True)
    with pytest.warns(UserWarning, match="metadata only"):
        result = verify_alignment_source(bag, frames, require_provenance=True)
    assert result["status"] == "verified_metadata_only"


@pytest.mark.parametrize("manifest", [{}, {"schema_version": 2}, [], None])
def test_present_but_malformed_manifest_never_downgrades_to_unverified(tmp_path, manifest):
    bag = make_bag(tmp_path)
    frames = bind_index(tmp_path, bag)
    (frames.parent / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="manifest"):
        verify_alignment_source(bag, frames)


def test_bag_identity_inventories_all_split_files_and_detects_missing(tmp_path):
    bag = make_bag(tmp_path)
    identity = bag_identity(bag)
    assert [entry["relative_path"] for entry in identity["storage_files"]] == ["part_0.mcap", "part_1.mcap"]
    (bag / "part_1.mcap").unlink()
    with pytest.raises(FileNotFoundError, match="storage file is missing"):
        bag_identity(bag)


def test_bag_inventory_rejects_outside_paths(tmp_path):
    bag = make_bag(tmp_path)
    metadata = bag / "metadata.yaml"
    document = yaml.safe_load(metadata.read_text())
    document["rosbag2_bagfile_information"]["relative_file_paths"] = ["../escape.mcap"]
    metadata.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="outside the bag"):
        bag_identity(bag)


@pytest.mark.parametrize("population,full,verdict", [(0, False, "WARNING"), (3, False, "WARNING"), (400, True, "COMPLETE")])
def test_export_writes_quality_and_provenance_even_for_all_invalid(tmp_path, population, full, verdict):
    bag = make_bag(tmp_path)
    frames = bind_index(tmp_path, bag)
    with patch("vt_multisensor_alignment.tactile_export.iter_tactile_events",
               return_value=iter([event(full)] if population else [])):
        with warnings.catch_warnings(record=True) as emitted:
            output = export_tactile(bag_path=bag, aligned_frames=frames,
                                    output_directory=tmp_path / "out", require_provenance=True)
    manifest = json.loads((output / "manifest.json").read_text())
    report = json.loads((output / "quality_report.json").read_text())
    assert manifest["quality_summary"] == report
    assert manifest["provenance"]["status"] == "verified_metadata_and_storage_inventory"
    assert manifest["tool_version"]
    assert report["verdict"] == verdict
    assert report["valid_cell_observations"] == population
    assert report["coverage_ratio"] == population / 400
    assert report["empty_frames"] == (population == 0)
    assert bool(emitted) == (verdict == "WARNING")
    if population:
        assert report["valid_cell_age_ns"]["p95_ns"] == 10
    else:
        assert report["valid_cell_age_ns"]["p95_ns"] is None
        assert "No valid tactile" in report["warnings"][0]


def test_quality_accumulation_has_fixed_memory_and_excludes_expired_ages():
    quality = _QualityAccumulator(max_age_ns=100)
    ages = np.arange(400, dtype=np.int64).reshape(20, 20)
    mask = ages <= 100
    for _ in range(1000):
        quality.add({"valid_mask": mask, "age_ns": ages})
    report = quality.report()
    assert quality.age_histogram.shape == (2048,)
    assert report["partial_frames"] == 1000
    assert report["valid_cell_age_ns"]["count"] == 101000
    assert report["valid_cell_age_ns"]["max"] == 100
    assert report["valid_cell_age_ns"]["p50_ns"] == 50
    assert report["valid_cell_age_ns"]["p95_ns"] == 95


def test_storage_filter_is_set_before_reading_and_indices_stay_topic_local(tmp_path):
    observed = []
    source = [("/camera/color", b"large-image", 50), (TOPIC, source_message(), 100),
              ("/camera/color", b"large-image", 150), (TOPIC, source_message(200, sequence=1), 200)]
    class Reader:
        def get_all_topics_and_types(self):
            return [NS(name=TOPIC, type=TACTILE_SAMPLE_TYPE)]
        def set_filter(self, filter):
            observed.append(filter.topics)
            self.records = [record for record in source if record[0] in filter.topics]
        def has_next(self):
            assert observed, "filter must be installed before reading"
            return bool(self.records)
        def read_next(self):
            return self.records.pop(0)
    with patch.dict(sys.modules, {"rosbag2_py": NS(StorageFilter=lambda **kw: NS(**kw))}), \
         patch("vt_multisensor_alignment.tactile_export._open_reader", return_value=Reader()), \
         patch("vt_multisensor_alignment.tactile_export._deserializer", return_value=lambda value: value):
        events = list(iter_tactile_events(tmp_path))
    assert observed == [[TOPIC]]
    assert [value.reference.sequence for value in events] == [0, 1]
    assert [value.reference.bag_timestamp_ns for value in events] == [100, 200]


def test_real_mcap_filtered_bound_export_keeps_tactile_sequence(tmp_path):
    rosbag = pytest.importorskip("rosbag2_py")
    serialization = pytest.importorskip("rclpy.serialization")
    messages = pytest.importorskip("vt_tactile_msgs.msg")
    sensor = pytest.importorskip("sensor_msgs.msg")
    bag = tmp_path / "real-bag"
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag.ConverterOptions("", ""))
    for index, topic, kind in ((1, TOPIC, TACTILE_SAMPLE_TYPE), (2, "/camera/color", "sensor_msgs/msg/Image")):
        writer.create_topic(rosbag.TopicMetadata(id=index, name=topic, type=kind, serialization_format="cdr"))
    for sequence, time_ns in enumerate((100, 200)):
        image = sensor.Image()
        image.width, image.height, image.step, image.encoding = 32, 32, 32, "mono8"
        image.data = [0] * 1024
        writer.write("/camera/color", serialization.serialize_message(image), time_ns - 1)
        source = source_message(time_ns, sequence=sequence)
        message = messages.TactileSampleArray()
        for name, value in vars(source).items():
            if name not in {"header", "blocks"}:
                setattr(message, name, value)
        message.header.stamp.nanosec = time_ns
        message.header.frame_id = source.header.frame_id
        block = messages.TactileBlock()
        block.adc = list(range(10))
        message.blocks = [block]
        writer.write(TOPIC, serialization.serialize_message(message), time_ns)
    writer.close()
    frames = bind_index(tmp_path, bag)
    with pytest.warns(UserWarning, match="partial or empty"):
        output = export_tactile(bag_path=bag, aligned_frames=frames,
                                output_directory=tmp_path / "out", require_provenance=True)
    with np.load(output / "frames/000000000.npz") as arrays:
        assert arrays["valid_mask"].sum() == 3
        assert arrays["block_message_sequence"][0, 0] == 0
        assert arrays["adc"][0, 8] == 1
    manifest = json.loads((output / "manifest.json").read_text())
    assert len(manifest["provenance"]["source_bag_identity"]["storage_files"]) == 1
    assert manifest["quality_summary"]["valid_cell_age_ns"]["min"] == 10
