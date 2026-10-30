"""Geometry provenance is additive, bounded and separate from raw bags."""
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

from vt_capture_manager.spatial.geometry_recording import GeometryRecorder


class Store:
    config = NS(cameras=(), world_frame='vive_map', config_revision='config-one', source_path='')
    camera_registry = None
    manager_instance_id = 'manager-one'
    def snapshot(self):
        return dict(manager_instance_id=self.manager_instance_id, scene_epoch='scene-one', time_epoch=1,
                    coordinate_epoch=2, sync_epoch=3, map_identity=None, generated_realtime_ns=123,
                    trackers=[], cameras=[], diagnostics=[])

    def geometry_snapshot(self):
        return deepcopy(self.config), self.snapshot(), {
            camera['id']: self.camera_registry.geometry(camera['id']) if self.camera_registry else {}
            for camera in self.config.cameras}


def trial_context(tmp_path):
    bag = tmp_path/'raw'
    bag.mkdir()
    (bag/'metadata.yaml').write_text('original bag bytes')
    trial = dict(id='trial-one', start_request_id='start-one', controller_instance_id='controller-one')
    context = dict(start_request_id='start-one', controller_instance_id='controller-one', session_id='session-one',
                   output_path=str(bag), state=3, recorder_alive=True)
    return trial, context


def test_geometry_sidecar_preserves_bag_and_freezes_terminal(tmp_path):
    trial, context = trial_context(tmp_path)
    recorder = GeometryRecorder(Store())
    recorder.observe(trial, context)
    recorder.flush()
    path = Path(context['output_path']+'.spatial')/'manifest.json'
    manifest = json.loads(path.read_text())
    assert manifest['session_id'] == 'session-one' and not manifest['complete']
    assert manifest['geometry_status'] == 'incomplete'
    assert manifest['display_from_world'] == dict(translation_m=[0.,0.,0.], quaternion_xyzw=[0.,0.,0.,1.])
    context.update(state=5, recorder_alive=False)
    recorder.observe(trial, context)
    recorder.close()
    manifest = json.loads(path.read_text())
    assert manifest['complete'] and manifest['recorder_state'] == 5
    assert (tmp_path/'raw'/'metadata.yaml').read_text() == 'original bag bytes'
    assert list((tmp_path/'raw').iterdir()) == [tmp_path/'raw'/'metadata.yaml']


def test_geometry_sidecar_freezes_preview_notice_at_recording_start(tmp_path):
    trial, context = trial_context(tmp_path)
    store = Store()
    store.config = deepcopy(store.config)
    notice = '左腕·试用候选（最初第3轮；三轮一致性未通过）'
    store.config.preview_notice = notice
    recorder = GeometryRecorder(store)
    try:
        recorder.observe(trial, context)
        # A queued write must use captured provenance, never a later live config.
        store.config.preview_notice = ''
        recorder.flush()
        recorder.observe(trial, dict(context, state=5, recorder_alive=False))
        recorder.flush()
        manifest = json.loads((Path(context['output_path']+'.spatial')/'manifest.json').read_text())
        assert manifest['preview_notice'] == notice
        assert manifest['complete']
        assert manifest['geometry_status'] == 'incomplete'
        assert (tmp_path/'raw'/'metadata.yaml').read_text() == 'original bag bytes'
    finally:
        recorder.close()


def test_foreign_session_and_existing_foreign_sidecar_are_never_adopted(tmp_path):
    trial, context = trial_context(tmp_path)
    recorder = GeometryRecorder(Store())
    recorder.observe(dict(trial, start_request_id='foreign'), context)
    recorder.flush()
    assert not Path(context['output_path']+'.spatial').exists()
    directory = Path(context['output_path']+'.spatial')
    directory.mkdir()
    marker = directory/'manifest.json'
    marker.write_text('{"session_id":"foreign"}')
    recorder.observe(trial, context)
    recorder.close()
    assert marker.read_text() == '{"session_id":"foreign"}'
    assert recorder.errors


def test_changed_session_cannot_overwrite_finalized_geometry(tmp_path):
    trial, context = trial_context(tmp_path)
    recorder = GeometryRecorder(Store())
    recorder.observe(trial, dict(context, state=5, recorder_alive=False))
    recorder.flush()
    marker = Path(context['output_path']+'.spatial')/'manifest.json'
    first = marker.read_bytes()
    recorder.observe(trial, dict(context, session_id='other'))
    recorder.close()
    assert marker.read_bytes() == first

def test_identity_renewal_extends_interval_without_new_geometry_artifact(tmp_path):
    from copy import deepcopy
    class Registry:
        geometry_value=dict(config=dict(serial='camera'),identity=dict(verified=True,serial='camera',observed_ns=1_000_000_000),
            identity_revision=1,geometry_revision=1,geometry_sha256='a'*64)
        def geometry(self,name): return deepcopy(self.geometry_value)
    class CameraStore(Store):
        config=NS(cameras=({'id':'cam'},),world_frame='vive_map',config_revision='one',source_path='')
        camera_registry=Registry()
        now=1_000_000_000
        def snapshot(self): return dict(super().snapshot(),generated_realtime_ns=self.now)
    store=CameraStore(); trial,context=trial_context(tmp_path); recorder=GeometryRecorder(store)
    try:
        for observed in (1_000_000_000,3_000_000_000,5_000_000_000,7_000_000_000):
            store.now=observed; store.camera_registry.geometry_value['identity']['observed_ns']=observed
            recorder.observe(trial,context); recorder.flush()
        directory=Path(context['output_path']+'.spatial'); manifest=json.loads((directory/'manifest.json').read_text())
        assert len(manifest['revisions'])==1
        assert len(list(directory.glob('geometry-*.json')))==1
        assert len(manifest['identity_intervals'])==1
        interval=manifest['identity_intervals'][0]
        assert interval['start_realtime_ns']==1_000_000_000 and interval['end_realtime_ns']==12_000_000_000
        store.now=8_000_000_000
        store.camera_registry.geometry_value.update(identity_revision=2,identity=dict(verified=False,serial='camera',observed_ns=7_000_000_000,invalidated_ns=8_000_000_000))
        recorder.observe(trial,context); recorder.flush()
        manifest=json.loads((directory/'manifest.json').read_text())
        assert manifest['identity_intervals'][0]['end_realtime_ns']==8_000_000_000
        assert len(manifest['revisions'])==2
    finally: recorder.close()


def test_identity_lease_gap_creates_separate_interval_without_geometry_change(tmp_path):
    manifest={'identity_intervals':[]}
    scene=dict(generated_realtime_ns=1_000_000_000,time_epoch=0,coordinate_epoch=0,scene_epoch='scene')
    geometry=dict(config={'serial':'serial'},geometry_revision=1,geometry_sha256='a'*64,identity_revision=1,
        identity=dict(verified=True,serial='serial',observed_ns=1_000_000_000))
    GeometryRecorder._identity_interval(manifest,'cam',geometry,scene)
    geometry['identity']['observed_ns']=10_000_000_000; scene['generated_realtime_ns']=10_000_000_000
    GeometryRecorder._identity_interval(manifest,'cam',geometry,scene)
    assert [(x['start_realtime_ns'],x['end_realtime_ns']) for x in manifest['identity_intervals']]==[(1_000_000_000,6_000_000_000),(10_000_000_000,15_000_000_000)]
