from pathlib import Path

import pytest
import yaml

from vt_realsense_capture.config import load_config
from vt_realsense_capture.recorder import required_topics


CONFIG = Path(__file__).parents[1] / "config"


@pytest.mark.parametrize("profile,count", [
    ("left_glove", 1), ("both_gloves", 2), ("left_glove_tactile", 2),
    ("both_gloves_tactile", 4), ("left_tactile", 1), ("right_tactile", 1), ("both_tactile", 2),
])
def test_hand_profiles_only_extend_explicit_allowlist(profile, count):
    path = CONFIG / f"cameras.with_{profile}.yaml"
    original = yaml.safe_load((CONFIG / "cameras.yaml").read_text())
    document = yaml.safe_load(path.read_text())
    streams = document["recording"].pop("additional_streams")
    original["recording"].pop("additional_streams")
    assert document == original
    config = load_config(path)
    assert len(config.additional_streams) == count
    topics = required_topics(tuple(camera.name for camera in config.cameras),
                             tuple(stream.topic for stream in config.additional_streams))
    assert len(topics) == 15 + count
    assert set(topics[15:]) == {stream["topic"] for stream in streams}
    assert all(topic.endswith("/state") or topic.endswith("/tactile/samples") for topic in topics[15:])
