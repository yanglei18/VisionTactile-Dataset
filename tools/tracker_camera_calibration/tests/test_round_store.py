import hashlib
import json

import pytest
import yaml

from vt_tracker_camera_calib.config_writer import build_config_document, write_config
from vt_tracker_camera_calib.guided_worker import Request
from vt_tracker_camera_calib.round_store import RoundStore


@pytest.fixture
def legacy(tmp_path):
    config = build_config_document(camera_name='d436', tracker_role='torso',
                                  square_length_mm=15., marker_length_mm=11.25)
    path = write_config(tmp_path / 'config.yaml', config)
    request = Request('test-session', 'guided', path, tmp_path, (), 'a' * 64)
    result = tmp_path / 'round-1-result'
    result.mkdir()
    document = dict(status='VALID', camera={k: config['camera'][k] for k in ('name', 'serial', 'model', 'frame_id')},
        tracker={k: config['tracker'][k] for k in ('role', 'frame_id')} | dict(tracker_id='a' * 64),
        frames=config['frames'], transform=dict(translation_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1]),
        provenance=dict(config_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (result / 'extrinsics.yaml').write_text(yaml.safe_dump(document))
    (result / 'report.json').write_text('{}')
    (result / 'independent-validation.json').write_text(json.dumps(dict(status='PASS', training_count=40,
        validation_count=10, translation_rms_m=.002, rotation_rms_deg=.3)))
    bag = tmp_path / 'round-1' / 'bag'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(
        starting_time=dict(nanoseconds_since_epoch=1000), duration=dict(nanoseconds=900)))))
    return request


def test_adopt_already_solved_round_without_refitting_or_rewriting(legacy):
    path = legacy.output_root / 'round-1-result' / 'extrinsics.yaml'
    original = path.read_bytes()
    rounds = RoundStore(legacy, lambda bag: 'b' * 64).load()
    assert rounds[0]['round'] == 1
    assert rounds[0]['recovered_from_legacy'] is True
    assert path.read_bytes() == original
    def no_rehash(bag):
        raise AssertionError('already checkpointed result must not be re-solved or rehashed')
    assert RoundStore(legacy, no_rehash).load() == rounds


def test_changed_artifact_rejected_on_resume(legacy):
    store = RoundStore(legacy, lambda bag: 'b' * 64)
    store.load()
    (legacy.output_root / 'round-1-result' / 'report.json').write_text('{"changed":true}')
    with pytest.raises(ValueError, match='artifact changed'):
        store.load()


def test_changed_config_rejected_on_resume(legacy):
    RoundStore(legacy, lambda bag: 'b' * 64).load()
    legacy.config_path.write_text(legacy.config_path.read_text() + '\n# changed\n')
    with pytest.raises(ValueError, match='checkpoint'):
        RoundStore(legacy, lambda bag: 'b' * 64).load()


def test_failed_validation_not_adopted(legacy):
    path = legacy.output_root / 'round-1-result' / 'independent-validation.json'
    path.write_text('{"status":"FAIL"}')
    assert RoundStore(legacy, lambda bag: 'b' * 64).load() == []
