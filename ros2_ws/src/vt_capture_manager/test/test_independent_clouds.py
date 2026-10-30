"""Independent live preview, with the original per-frame pose safety contract."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from vt_capture_core.spatial.cloud_protocol import unpack_cloud_frame, validate_cloud_header, CloudProtocolError
from vt_capture_core.spatial.scene_protocol import validate_scene
from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_manager.spatial.cloud_worker import CloudPipeline
from vt_capture_manager.spatial.cloud_supervisor import CloudSupervisor
from vt_capture_manager.spatial.config import load_scene_config
from vt_capture_manager.spatial.scene_store import SceneStore
from test_spatial_cloud_worker import Clock, Server, DeferredExecutor, config, images


NAMES=('head','left','right')
ROLES=('torso','left_wrist','right_wrist')


def independent(executor=None):
    clock=Clock(); server=Server(); events=[]; control=config()
    geometry=control['geometries']['cam']; pose=control['poses']['torso'][0]
    control['geometries']={}; control['poses']={}; control['reference_camera']='head'
    control['cloud']['matching_mode']='independent'
    for i,(name,role) in enumerate(zip(NAMES,ROLES)):
        g=deepcopy(geometry); g['config'].update(id=name,tracker_role=role)
        g['extrinsic']['tracker']['tracker_id']=name+'-tracker'
        control['geometries'][name]=g
        control['poses'][role]=[dict(pose,tracker_id=name+'-tracker',source_realtime_ns=t,
            received_monotonic_ns=t,pose=RigidTransform((i+t/1e9,0,0),(0,0,0,1)).as_dict())
            for t in range(900_000_000,1_300_000_001,10_000_000)]
    worker=CloudPipeline(server,events.append,monotonic_ns=clock,realtime_ns=clock,executor=executor)
    worker.configure(control)
    return worker,clock,server,events,control


def send(worker,name,stamp=123,source=1_000_000_000):
    for stream,message in images(stamp,source).items():
        if stream=='timing': message.camera_name=name
        assert worker.observe(name,stream,message)


def headers(server):
    return {name:unpack_cloud_frame(wire).header for name,wire in server.frames.items()}


def test_head_alone_publishes_immediately_without_reference_failover_or_wrist_wait():
    worker,clock,server,events,control=independent()
    send(worker,'head'); worker.tick()
    assert set(server.frames)=={'head'}
    h=headers(server)['head']
    assert h['matching_mode']=='independent'
    assert h['tracker_pose']['world_from_tracker']['translation_m']==[1.,0.,0.]
    assert h['world_from_color']['translation_m']==[1.1,0.,0.]
    assert not any(e['event']=='sync_change' for e in events)


def test_three_asynchronous_cameras_keep_their_own_times_not_a_common_anchor():
    worker,clock,server,events,control=independent()
    # Much wider than the unchanged 20 ms strict grouping tolerance.
    times=(1_000_000_000,1_080_000_000,1_160_000_000)
    clock.now=times[-1]
    for name,source in zip(NAMES,times): send(worker,name,source=source)
    worker.tick()
    assert set(server.frames)==set(NAMES)
    for name,role,source in zip(NAMES,ROLES,times):
        h=headers(server)[name]
        assert h['source_realtime_ns']==h['frame_key']['scene_time_ns']==source
        assert h['frame_key']['reference_camera']==name
        assert h['tracker_pose']['role']==role
        assert h['tracker_pose']['before_ns']==h['tracker_pose']['after_ns']==source
    assert len([e for e in events if e['event']=='frames'])==3


def test_missing_head_does_not_switch_epoch_or_delay_valid_wrist():
    worker,clock,server,events,control=independent()
    send(worker,'left'); worker.tick()
    assert set(server.frames)=={'left'}
    assert headers(server)['left']['sync_epoch']==0
    assert not any(e['event']=='sync_change' for e in events)


def test_newest_unbracketed_image_does_not_block_newest_ready_frame():
    worker,clock,server,events,control=independent()
    control['poses']['torso']=control['poses']['torso'][:12]  # through 1.01 s
    worker.configure(control)
    send(worker,'head',stamp=123,source=1_000_000_000)
    send(worker,'head',stamp=124,source=1_020_000_000)
    clock.now=1_020_000_000
    worker.tick()
    assert headers(server)['head']['source_realtime_ns']==1_000_000_000
    assert headers(server)['head']['frame_key']['shared_ros_timestamp_ns']==123


def test_ready_successor_supersedes_unusable_pending_gap_without_extra_wait():
    worker,clock,server,events,control=independent()
    control['poses']['torso']=[control['poses']['torso'][5],control['poses']['torso'][15]]
    worker.configure(control);send(worker,'head',stamp=123);worker.tick()
    assert not server.frames
    clock.now=1_050_000_000
    send(worker,'head',stamp=124,source=clock.now);worker.tick()
    assert headers(server)['head']['source_realtime_ns']==clock.now
    assert not any('gap_exceeded' in e.get('reason','') for e in events)


def test_one_pose_gap_only_rejects_that_camera_and_never_extrapolates():
    worker,clock,server,events,control=independent()
    # No exact 1 s pose, 100 ms gap (not silently relaxed).
    control['poses']['left_wrist']=[control['poses']['left_wrist'][5],control['poses']['left_wrist'][15]]
    control['poses']['right_wrist']=control['poses']['right_wrist'][:5]  # no_after
    worker.configure(control)
    for name in NAMES: send(worker,name)
    worker.tick(); assert set(server.frames)=={'head'}
    clock.now+=100_000_000; worker.tick()
    assert set(server.frames)=={'head'}
    assert any(e.get('camera_id',e.get('camera'))=='left' and 'gap_exceeded' in e.get('reason','') for e in events)
    assert any('no_after' in e.get('reason','') for e in events)


def test_independent_async_builds_are_bounded_and_publish_without_waiting_for_peer():
    executor=DeferredExecutor()
    worker,clock,server,events,control=independent(executor)
    for name in NAMES: send(worker,name)
    worker.tick(); assert len(executor.jobs)==3
    # Complete right before head/left; it must publish immediately.
    future,fn,args=executor.jobs.pop(2); future.set_result(fn(*args))
    worker.tick(); assert set(server.frames)=={'right'}
    for i in range(1,5):
        clock.now+=10_000_000
        for name in NAMES: send(worker,name,stamp=123+i,source=clock.now)
        worker.configure(control); worker.tick()
    assert len(executor.jobs)==2  # no growing backlog behind blocked builds
    # Retirement is still enforced for every in-flight camera separately.
    control['coordinate_epoch']=1; worker.configure(control)
    while executor.jobs: executor.complete()
    worker.tick(); assert not server.frames


@pytest.mark.parametrize('field,value',[('source_realtime_ns',2_000_000_000),('matching_mode','latest_pose')])
def test_independent_packet_rejects_false_source_pose_contract(field,value):
    worker,clock,server,events,control=independent()
    send(worker,'head'); worker.tick()
    h=headers(server)['head']; h[field]=value
    with pytest.raises(CloudProtocolError): validate_cloud_header(h)


def test_supervisor_merges_only_independent_headers_and_preserves_each_lease(monkeypatch,tmp_path):
    from vt_capture_manager.spatial import cloud_supervisor as module
    worker,clock,server,events,control=independent()
    for name in NAMES: send(worker,name)
    worker.tick(); h=headers(server)
    timer=NS(now=10.)
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:timer.now))
    store=NS(manager_instance_id='manager',config=NS(cloud={'matching_mode':'independent'},cameras=[{'id':n} for n in NAMES]))
    supervisor=CloudSupervisor(store,tmp_path/'cloud.sock')
    supervisor._worker_id=worker.worker_instance_id; supervisor._ready=True; supervisor._process=NS(poll=lambda:None)
    def event(header): return {k:v for k,v in header.items() if k not in ('protocol','version')}
    supervisor._event(dict(event='frames',headers=[event(v) for v in h.values()]))
    original_left=supervisor._headers['left']
    timer.now=10.4
    newer=dict(h['head'],cloud_sequence=4)
    supervisor._event(dict(event='frames',headers=[event(newer)]))
    assert supervisor._headers['left']==original_left
    assert supervisor.latest_headers()['head']['cloud_sequence']==4
    timer.now=10.51
    assert set(supervisor.latest_headers())=={'head'}
    supervisor._event(dict(event='frames',headers=[event(dict(h['right'],cloud_sequence=5,coordinate_epoch=1))]))
    assert set(supervisor.latest_headers())=={'right'}


def test_scene_uses_frozen_per_camera_pose_even_when_manager_pose_history_has_moved_on():
    worker,clock,server,events,control=independent()
    times=(1_000_000_000,1_080_000_000,1_160_000_000); clock.now=times[-1]
    for name,source in zip(NAMES,times): send(worker,name,source=source)
    worker.tick(); h=headers(server)
    store=SceneStore('manager',replace(load_scene_config(),cloud={'matching_mode':'independent'}),realtime_ns=clock,monotonic_ns=clock)
    store.scene_epoch='scene'
    for name,role in zip(NAMES,ROLES): store._states[role]['tracker_id']=name+'-tracker'
    class Registry:
        def camera_entities(self,*args,**kwargs):
            return [dict(id=name,tracker_role=role,registration_state='ready',geometry_revision=1,geometry_sha256='a'*64)
                    for name,role in zip(NAMES,ROLES)]
    store.camera_registry=Registry(); store.cloud_supervisor=NS(latest_headers=lambda:h)
    store.set_interest('viewer',['cloud'])
    scene=store.snapshot(); validate_scene(scene)
    assert scene['time_mode']=='independent' and scene['scene_time_ns'] is None
    assert scene['active_reference_camera'] is None
    trackers={t['role']:t for t in scene['trackers']}
    for camera,role,source in zip(scene['cameras'],ROLES,times):
        assert camera['source_realtime_ns']==trackers[role]['source_realtime_ns']==source
        assert trackers[role]['position_m']==h[camera['id']]['tracker_pose']['world_from_tracker']['translation_m']
    # A late/old generation only removes that camera, not its healthy neighbors.
    h['left']['coordinate_epoch']=9
    assert {c['id'] for c in store.snapshot()['cameras'] if c.get('frame_key')}=={'head','right'}


def test_config_requires_explicit_independent_mode_and_preserves_old_default(tmp_path):
    path=tmp_path/'scene.yaml'
    path.write_text('cloud: {enabled: true, matching_mode: independent}\n')
    assert load_scene_config(path).cloud['matching_mode']=='independent'
    path.write_text('cloud: {enabled: true}\n')
    assert load_scene_config(path).cloud['matching_mode']=='synchronized'
    path.write_text('cloud: {matching_mode: guessed}\n')
    with pytest.raises(ValueError): load_scene_config(path)
