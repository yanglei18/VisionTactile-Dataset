from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from vt_realsense_capture.config import load_config
from vt_realsense_capture.controller import CaptureCommandFact, CaptureController, CommandKind, RecorderHealth
from vt_realsense_capture.recorder import write_qos_overrides
from vt_realsense_capture.recording_qos import RecorderQosOverride, load_overrides, profiles_for_topics
from vt_realsense_capture.storage import FileStorage


CONFIG = Path(__file__).parents[1] / "config" / "cameras.with_right_glove_tactile.yaml"
TACTILE = "/gloves/right/tactile/samples"
GLOVE = "/gloves/right/state"
CAMERA = "/d405_1/color/image_raw"
TRACKER = "/vive/torso/sample"
TOPICS = (CAMERA, TRACKER, GLOVE, TACTILE)
TYPES = {
    TACTILE: "vt_tactile_msgs/msg/TactileSampleArray",
    GLOVE: "vt_glove_msgs/msg/GloveState",
}


def test_defaults_preserve_camera_tracker_and_buffer_high_rate_types(tmp_path):
    path = write_qos_overrides(tmp_path / "qos.yaml", TOPICS, topic_types=TYPES)
    profiles = yaml.safe_load(path.read_text())
    assert tuple(profiles) == TOPICS
    assert [profiles[topic]["depth"] for topic in TOPICS] == [30, 30, 256, 512]
    assert all(profile["reliability"] == "best_effort" for profile in profiles.values())
    assert all(profile["history"] == "keep_last" for profile in profiles.values())
    assert all(profile["durability"] == "volatile" for profile in profiles.values())


def test_type_not_topic_name_selects_default():
    arbitrary = "/renamed/pressure_packets"
    profiles = profiles_for_topics(
        (arbitrary, TACTILE),
        topic_types={arbitrary: TYPES[TACTILE], TACTILE: "other/msg/Samples"},
    )
    assert profiles[arbitrary]["depth"] == 512
    assert profiles[TACTILE]["depth"] == 30


def test_explicit_override_merges_with_type_defaults(tmp_path):
    overrides = load_overrides({TACTILE: {"depth": 1024}, GLOVE: {"reliability": "reliable"}}, TOPICS)
    path = write_qos_overrides(tmp_path / "qos.yaml", TOPICS, topic_types=TYPES, overrides=overrides)
    profiles = yaml.safe_load(path.read_text())
    assert profiles[TACTILE]["depth"] == 1024
    assert profiles[TACTILE]["reliability"] == "best_effort"
    assert profiles[GLOVE]["depth"] == 256
    assert profiles[GLOVE]["reliability"] == "reliable"


@pytest.mark.parametrize("depth", [0, -1, 4097, True, False, 30.0, "30", None])
def test_invalid_depth_rejected_at_config_load(tmp_path, depth):
    doc = yaml.safe_load(CONFIG.read_text())
    doc["recording"]["qos_overrides"] = {TACTILE: {"depth": depth}}
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match="qos_overrides"):
        load_config(path)


@pytest.mark.parametrize("depth", [1, 4096])
def test_depth_bounds_are_inclusive(depth):
    assert load_overrides({TACTILE: {"depth": depth}}, TOPICS)[0].depth == depth


@pytest.mark.parametrize("value", [
    None, [], "reliable", {TACTILE: None}, {TACTILE: []}, {TACTILE: {}},
    {"/unrecorded": {"depth": 100}}, {"/gloves/*": {"depth": 100}},
    {"gloves/right/state": {"depth": 100}}, {17: {"depth": 100}},
    {TACTILE: {"reliability": "invalid"}}, {TACTILE: {"reliability": True}},
    {TACTILE: {"reliability": None}}, {TACTILE: {"reliability": []}},
    {TACTILE: {"depth": 100, "history": "keep_all"}},
    {TACTILE: {"depth": 100, "durability": "transient_local"}},
    {TACTILE: {"dept": 100}},
])
def test_invalid_overrides_rejected_before_session_file_creation(tmp_path, value):
    doc = yaml.safe_load(CONFIG.read_text())
    doc["recording"]["qos_overrides"] = value
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match="qos_overrides"):
        load_config(path)


@pytest.mark.parametrize("overrides", [
    (RecorderQosOverride(TACTILE, depth=True),),
    (RecorderQosOverride(TACTILE, depth=0),),
    (RecorderQosOverride("/missing", depth=100),),
    (RecorderQosOverride(TACTILE, depth=100), RecorderQosOverride(TACTILE, depth=200)),
])
def test_writer_revalidates_programmatic_overrides(tmp_path, overrides):
    path = tmp_path / "qos.yaml"
    with pytest.raises(ValueError, match="qos_overrides"):
        write_qos_overrides(path, TOPICS, topic_types=TYPES, overrides=overrides)
    assert not path.exists()
    assert list(tmp_path.iterdir()) == []


def test_config_accepts_core_and_additional_exact_topic_overrides(tmp_path):
    document = yaml.safe_load(CONFIG.read_text())
    document["recording"]["qos_overrides"] = {
        CAMERA: {"depth": 20}, TACTILE: {"depth": 1024, "reliability": "best_effort"},
    }
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(document))
    config = load_config(path)
    assert config.qos_overrides == (
        RecorderQosOverride(CAMERA, depth=20),
        RecorderQosOverride(TACTILE, depth=1024, reliability="best_effort"),
    )


@pytest.mark.parametrize("explicit", [False, True])
def test_actual_controller_storage_recorder_path_persists_selected_profiles(tmp_path, explicit):
    config = load_config(CONFIG)
    if explicit:
        config = replace(config, qos_overrides=(RecorderQosOverride(TACTILE, depth=1024),))
    started = []
    controller = CaptureController(
        config=config,
        output_root=tmp_path,
        clock=SimpleNamespace(monotonic_ns=lambda: 1_000_000_000),
        storage=FileStorage(),
        recorder_factory=lambda: SimpleNamespace(start=started.append, health=lambda: RecorderHealth.ALIVE),
        event_sink=lambda _event: None,
        status_sink=lambda _status: None,
        session_sink=lambda _session: None,
        session_id_factory=lambda: "session-qos",
    )
    controller.accept(CaptureCommandFact("start", CommandKind.START, planned_duration_sec=30))
    assert len(started) == 1
    spec = started[0]
    profiles = yaml.safe_load(spec.qos_overrides_path.read_text())
    assert set(profiles) == set(spec.topics)
    assert profiles[TACTILE]["depth"] == (1024 if explicit else 512)
    assert profiles[GLOVE]["depth"] == 256
    assert profiles[CAMERA]["depth"] == 30
    assert profiles[TRACKER]["depth"] == 30
    assert all(profile["reliability"] == "best_effort" for profile in profiles.values())
