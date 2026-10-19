"""Live binding and frozen replay provenance change together."""
from copy import deepcopy
import json
from pathlib import Path

import pytest
import yaml

from vt_capture_manager.spatial.camera_geometry import CameraGeometryRegistry
from vt_capture_manager.spatial.config import SceneConfig
from vt_capture_manager.spatial.geometry_recording import GeometryRecorder
from vt_capture_manager.spatial.scene_store import SceneStore


def configured():
    example = Path(__file__).parents[4] / 'tools/multisensor_alignment/config/alignment.spatial.example.yaml'
    alignment = yaml.safe_load(example.read_text())
    frames = {item['role']: item['frame_id'] for item in alignment['trackers']}
    cameras = tuple(dict(camera, id=camera['name'], tracker_frame_id=frames[camera['tracker_role']])
                    for camera in alignment['cameras'])
    config = SceneConfig(cameras=cameras, alignment_document=alignment, reference_camera=alignment['reference_camera'])
    store = SceneStore('manager', config)
    store.camera_registry = CameraGeometryRegistry(cameras)
    return store


def extrinsic(store, role='right_wrist'):
    camera = store.config.cameras[0]
    return dict(schema_version=1, status='VALID', camera=dict(name=camera['name'], serial=camera['serial'],
                model=camera['model'], frame_id=camera['color_frame_id']),
                tracker=dict(role=role, tracker_id='a' * 64, frame_id=f'vive_tracker_{role}'),
                frames=dict(world='vive_map'), quality=dict(grade='TARGET'),
                transform=dict(semantics='parent_from_child', parent_frame=f'vive_tracker_{role}',
                child_frame=camera['color_frame_id'], translation_m=[.1, 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]))


def test_rebinding_updates_replay_contract_and_preserves_loaded_configuration():
    from vt_multisensor_alignment.config import load_config_document
    store = configured()
    original = store.config
    candidate = extrinsic(store)
    assert store.apply_camera_calibration('d405_1', candidate, tracker_sample_topic='/vive/right_wrist/sample')
    assert original.cameras[0]['tracker_role'] == 'torso'
    assert store.config.cameras[0]['tracker_role'] == 'right_wrist'
    replay = load_config_document(store.config.alignment_document)
    assert replay.cameras[0].tracker_role == 'right_wrist'
    assert store.camera_registry.geometry('d405_1')['config']['tracker_role'] == 'right_wrist'
    assert store.config.config_revision != original.config_revision
    epoch, revision = store.sync_epoch, store.config.config_revision
    assert not store.apply_camera_calibration('d405_1', candidate, tracker_sample_topic='/vive/right_wrist/sample')
    assert store.sync_epoch == epoch and store.config.config_revision == revision


def test_bad_candidate_keeps_scene_configuration_and_geometry_unchanged():
    store = configured()
    original = store.config
    before = store.camera_registry.geometry('d405_1')
    candidate = extrinsic(store)
    candidate['camera']['serial'] = 'wrong'
    with pytest.raises(ValueError):
        store.apply_camera_calibration('d405_1', candidate)
    assert store.config == original and store.sync_epoch == 0
    assert store.camera_registry.geometry('d405_1') == before


def test_queued_recording_freezes_binding_before_later_activation(tmp_path):
    from vt_multisensor_alignment.config import load_config_document
    store = configured()
    store.apply_camera_calibration('d405_1', extrinsic(store, 'torso'))
    bag = tmp_path / 'bag'
    bag.mkdir()
    trial = dict(id='trial', start_request_id='start', controller_instance_id='controller')
    context = dict(start_request_id='start', controller_instance_id='controller', session_id='session',
                   output_path=str(bag), state=5, recorder_alive=False)
    recorder = GeometryRecorder(store)
    try:
        with recorder._condition:
            recorder.observe(trial, context)
            store.apply_camera_calibration('d405_1', extrinsic(store, 'right_wrist'))
        recorder.flush()
    finally:
        recorder.close()
    assert not recorder.errors
    manifest = json.loads((tmp_path / 'bag.spatial/manifest.json').read_text())
    assert manifest['camera_config'][0]['tracker_role'] == 'torso'
    assert load_config_document(manifest['alignment_config']).cameras[0].tracker_role == 'torso'
    artifact = next(value['artifact'] for value in manifest['revisions'] if value['camera_id'] == 'd405_1')
    geometry = json.loads((tmp_path / 'bag.spatial' / artifact).read_text())
    assert geometry['config']['tracker_role'] == 'torso'
    assert geometry['extrinsic']['tracker']['role'] == 'torso'
    assert store.config.cameras[0]['tracker_role'] == 'right_wrist'
