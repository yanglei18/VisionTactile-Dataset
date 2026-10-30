from copy import deepcopy
from concurrent.futures import Future, ThreadPoolExecutor
import threading
from types import SimpleNamespace as NS
import numpy as np
import pytest
from vt_capture_core.spatial.cloud_protocol import unpack_cloud_frame
from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_manager.spatial.cloud_worker import CloudPipeline, MAX_PARTIAL_SLOTS, MAX_PARTIAL_AGE_NS


class Server:
    def __init__(self): self.frames={}
    def clear(self): self.frames.clear()
    def publish(self,name,frame): self.frames[name]=frame


class Clock:
    now=1_000_000_000
    def __call__(self): return self.now


def config():
    camera=dict(id='cam',serial='123',model='D405',tracker_role='torso',color_frame_id='color',depth_frame_id='depth',timing_frame_id='timing')
    info=dict(width=2,height=2,fx=1.,fy=1.,cx=0.,cy=0.,distortion_model='none',d=[])
    geometry=dict(config=camera,identity=dict(verified=True,observed_ns=1_000_000_000),geometry_sha256='a'*64,geometry_revision=1,
        color_intrinsics=info,depth_intrinsics=info,depth_to_color=RigidTransform.identity().as_dict(),
        depth_units_m=.001,depth_units_source='realsense_ros_z16_mm',
        extrinsic=dict(tracker=dict(tracker_id='tracker'),transform=RigidTransform((.1,0,0),(0,0,0,1)).as_dict()))
    poses=[dict(tracker_id='tracker',parent_frame='vive_map',source_realtime_ns=ns,received_monotonic_ns=ns,
        pose=RigidTransform((x,0,0),(0,0,0,1)).as_dict(),pose_valid=True) for ns,x in ((990_000_000,0),(1_010_000_000,2))]
    return dict(command='configure',manager_instance_id='manager',scene_epoch='scene',time_epoch=0,coordinate_epoch=0,sync_epoch=0,
        geometries={'cam':geometry},poses={'torso':poses},reference_camera='cam',cloud={'stride':1,'color_mode':'rgb'},interest=True)


def images(stamp=123,source=1_000_000_000):
    stampmsg=NS(sec=stamp//1_000_000_000,nanosec=stamp%1_000_000_000)
    color=NS(header=NS(frame_id='color',stamp=stampmsg),encoding='rgb8',is_bigendian=False,width=2,height=2,step=6,data=bytes(range(12)))
    depth=NS(header=NS(frame_id='depth',stamp=stampmsg),encoding='16UC1',is_bigendian=False,width=2,height=2,step=4,data=(np.ones((2,2),dtype='<u2')*1000).tobytes())
    timing=NS(header=NS(frame_id='timing',stamp=stampmsg),shared_ros_timestamp_ns=stamp,camera_name='cam',camera_model='D405',serial_number='123',
        group_validity_flags=31,group_host_realtime_ns=source,group_host_monotonic_raw_ns=source,color_frame_number=1,depth_frame_number=1)
    return dict(color=color,depth=depth,timing=timing)


def pipeline():
    clock=Clock(); server=Server(); events=[]
    worker=CloudPipeline(server,events.append,worker_instance_id='worker',monotonic_ns=clock,realtime_ns=clock)
    worker.configure(config()); return worker,clock,server,events


def test_driver_cloud_replaces_rgbd_builder_and_uses_exact_timing_pose(monkeypatch):
    from test_driver_cloud import cloud
    from vt_capture_manager.spatial.cloud_worker import cloud_input_streams
    worker,clock,server,events=pipeline();control=config()
    control['cloud']['source']='realsense'
    control['geometries']['cam']['config']['cloud_topic']='/cam/depth/color/points'
    worker.configure(control)
    assert cloud_input_streams(control)==('cloud','timing')
    def forbidden(*a,**kw):raise AssertionError('RGBD builder must not run')
    monkeypatch.setattr('vt_capture_manager.spatial.cloud_worker.build_cloud',forbidden)
    assert not worker.observe('cam','color',images()['color'])
    assert worker.observe('cam','cloud',cloud())
    worker.tick(); assert not server.frames
    assert worker.observe('cam','timing',images()['timing'])
    worker.tick()
    result=unpack_cloud_frame(server.frames['cam'])
    assert result.header['origin']['source']=='realsense'
    assert 'sampling' not in result.header
    assert result.header['point_count']==2
    assert result.header['world_from_color']['translation_m']==[1.1,0.,0.]
    np.testing.assert_allclose(np.frombuffer(result.xyz_bytes,'<f4').reshape(-1,3)[:,2],[.05,8.])


def test_driver_wrong_stamp_and_source_switch_cannot_borrow_pending_frames():
    from test_driver_cloud import cloud
    worker,clock,server,events=pipeline();control=config()
    control['cloud']['source']='realsense'
    control['geometries']['cam']['config']['cloud_topic']='/cam/depth/color/points'
    worker.configure(control)
    worker.observe('cam','cloud',cloud());worker.observe('cam','timing',images(456)['timing'])
    worker.tick();assert not server.frames
    worker.configure(config());worker.configure(control)
    worker.observe('cam','timing',images()['timing']);worker.tick();assert not server.frames


def test_raw_tuple_uses_host_frame_time_and_interpolated_pose_not_latest_pose():
    worker,clock,server,events=pipeline()
    for kind,message in images().items(): assert worker.observe('cam',kind,message)
    worker.tick()
    frame=unpack_cloud_frame(server.frames['cam'])
    assert frame.header['world_from_color']['translation_m']==[1.1,0.,0.]
    assert frame.header['source_realtime_ns']==1_000_000_000
    assert frame.header['frame_key']==dict(shared_ros_timestamp_ns=123,color_frame_number=1,depth_frame_number=1,scene_time_ns=1_000_000_000,reference_camera='cam')
    assert frame.header['point_count']==4
    assert events[-1]['event']=='frames'


def test_mismatched_image_stamps_are_never_borrowed_and_epoch_clears_partial_tuple():
    worker,clock,server,events=pipeline()
    data=images(); worker.observe('cam','color',data['color'])
    revised=config(); revised['time_epoch']=1; worker.configure(revised)
    for kind in ('depth','timing'): worker.observe('cam',kind,data[kind])
    worker.tick(); assert not server.frames
    worker.observe('cam','color',images(456)['color']); worker.tick(); assert not server.frames


def test_missing_future_tracker_pose_waits_and_fails_closed():
    worker,clock,server,events=pipeline()
    control=config(); control['poses']['torso']=control['poses']['torso'][:1]; worker.configure(control)
    for kind,message in images().items(): worker.observe('cam',kind,message)
    worker.tick(); assert not server.frames
    clock.now+=110_000_000; worker.tick()
    assert not server.frames
    assert any(e['event']=='diagnostic' for e in events)


def test_identity_lease_expiry_and_idle_control_clear_output():
    worker,clock,server,events=pipeline()
    control=config(); control['geometries']['cam']['identity']['verified']=False; worker.configure(control)
    for kind,message in images().items(): worker.observe('cam',kind,message)
    worker.tick(); assert not server.frames
    assert events[-1]['event']=='diagnostic'
    worker.configure(config())
    for kind,message in images(456).items(): worker.observe('cam',kind,message)
    worker.tick(); assert server.frames
    clock.now+=3_000_000_000; worker.tick(); assert not server.frames


def test_clock_step_requests_manager_reset_and_does_not_self_assign_epoch():
    worker,clock,server,events=pipeline()
    for kind,message in images().items(): worker.observe('cam',kind,message)
    worker.tick()
    clock.now+=20_000_000
    data=images(456,1_020_000_000); data['timing'].group_host_monotonic_raw_ns=1_010_000_000
    for kind,message in data.items(): worker.observe('cam',kind,message)
    assert not server.frames
    assert events[-1]==dict(event='clock_reset',camera_id='cam',time_epoch=0,coordinate_epoch=0)
    assert worker.control['time_epoch']==0

def test_reference_failover_waits_for_manager_sync_epoch_acknowledgment():
    worker,clock,server,events=pipeline()
    control=config(); other=deepcopy(control['geometries']['cam']); other['config']['id']='other'
    control['geometries']['other']=other
    worker.configure(control)
    def send_other(source,stamp):
        for kind,message in images(stamp,source).items():
            if kind=='timing': message.camera_name='other'
            worker.observe('other',kind,message)
    def update_poses(source):
        for value,offset in zip(control['poses']['torso'],(-10_000_000,10_000_000)):
            value['source_realtime_ns']=source+offset
            value['received_monotonic_ns']=source+offset
        worker.configure(control)
    send_other(clock.now,123); worker.tick(); assert not server.frames
    clock.now=1_300_000_000; update_poses(clock.now); send_other(clock.now,456); worker.tick()
    assert events[-1]['event']=='sync_change' and events[-1]['active_reference_camera']=='other'
    assert not server.frames and worker.control['sync_epoch']==0
    control['sync_epoch']=1; clock.now=1_400_000_000; update_poses(clock.now)
    send_other(clock.now,789); worker.tick()
    clock.now=1_510_000_000; worker.tick()
    frame=unpack_cloud_frame(server.frames['other'])
    assert frame.header['sync_epoch']==1
    assert frame.header['frame_key']['reference_camera']=='other'


def test_delayed_timing_matches_exact_images_after_four_frame_pipeline_latency():
    worker,clock,server,events=pipeline()
    control=config()
    original_pose=control['poses']['torso'][0]
    # Include the whole measured source-time range, with the normal 50 ms gap
    # limit intact. No latest-pose substitution or relaxed stamp matching.
    control['poses']['torso']=[dict(original_pose,source_realtime_ns=1_000_000_000+i*33_333_333,
                                   received_monotonic_ns=1_000_000_000+i*33_333_333)
                              for i in range(-1,25)]
    worker.configure(control)
    for index in range(20):
        clock.now=1_000_000_000+index*33_333_333
        data=images(100+index,clock.now)
        for stream in ('color','depth'):
            assert worker.observe('cam',stream,data[stream])
        # Startup includes metadata for images from before this subscription.
        delayed=images(100+index-4,clock.now-4*33_333_333)['timing']
        worker.observe('cam','timing',delayed)
        worker.tick()
        assert len(worker._partial['cam'])<=MAX_PARTIAL_SLOTS
    assert worker._clock['cam'][0]==1_000_000_000+15*33_333_333
    assert sum(event['event']=='frames' for event in events)>=4
    frame=unpack_cloud_frame(server.frames['cam'])
    matched_index=frame.header['frame_key']['shared_ros_timestamp_ns']-100
    assert 0<=matched_index<=15
    assert frame.header['source_realtime_ns']==1_000_000_000+matched_index*33_333_333


def test_partial_cache_evicts_oldest_source_not_newer_images_for_late_orphan():
    worker,clock,server,events=pipeline()
    for stamp in range(100,100+MAX_PARTIAL_SLOTS):
        assert worker.observe('cam','color',images(stamp)['color'])
    assert not worker.observe('cam','timing',images(99)['timing'])
    assert sorted(worker._partial['cam'])==list(range(100,100+MAX_PARTIAL_SLOTS))
    assert not server.frames


def test_expired_partial_images_are_not_resurrected_by_late_timing():
    worker,clock,server,events=pipeline()
    data=images()
    for stream in ('color','depth'):
        worker.observe('cam',stream,data[stream])
    clock.now+=MAX_PARTIAL_AGE_NS
    worker.observe('cam','timing',data['timing'])
    assert set(worker._partial['cam'][123])=={'received','timing'}
    worker.tick()
    assert not server.frames


def test_late_complete_tuple_waits_for_next_control_with_future_pose():
    worker,clock,server,events=pipeline()
    control=config()
    control['poses']['torso']=control['poses']['torso'][:1]
    worker.configure(control)
    data=images()
    for stream in ('color','depth'):
        assert worker.observe('cam',stream,data[stream])
    # Metadata arrives after the default 100 ms synchronization wait would have
    # expired if it were incorrectly charged from the first image component.
    clock.now+=140_000_000
    assert worker.observe('cam','timing',data['timing'])
    worker.tick()
    assert not server.frames
    assert not any(event['event']=='diagnostic' for event in events)
    assert worker.synchronizer._pending is not None
    assert worker.synchronizer._pending.received_monotonic_ns==clock.now
    clock.now+=50_000_000
    worker.configure(config())
    worker.tick()
    frame=unpack_cloud_frame(server.frames['cam'])
    assert frame.header['source_realtime_ns']==1_000_000_000
    assert frame.header['frame_key']['shared_ros_timestamp_ns']==123
    assert frame.header['world_from_color']['translation_m']==[1.1,0.,0.]


def test_late_metadata_leaves_wait_budget_for_other_camera_tuple():
    worker,clock,server,events=pipeline()
    control=config()
    other=deepcopy(control['geometries']['cam'])
    other['config']['id']='other'
    control['geometries']['other']=other
    worker.configure(control)
    data=images()
    other_data=images(456,1_005_000_000)
    other_data['timing'].camera_name='other'
    for camera,messages in (('cam',data),('other',other_data)):
        for stream in ('color','depth'):
            worker.observe(camera,stream,messages[stream])
    clock.now+=140_000_000
    worker.observe('cam','timing',data['timing'])
    worker.tick()
    assert not server.frames
    clock.now+=50_000_000
    worker.observe('other','timing',other_data['timing'])
    worker.tick()
    assert set(server.frames)=={'cam','other'}
    assert len(events[-1]['headers'])==2
    assert not events[-1]['missing']
    assert {header['frame_key']['scene_time_ns'] for header in events[-1]['headers']}=={1_000_000_000}


def test_complete_tuple_wait_remains_bounded_and_does_not_relax_pose_gap():
    worker,clock,server,events=pipeline()
    control=config()
    control['poses']['torso']=control['poses']['torso'][:1]
    worker.configure(control)
    data=images()
    for stream in ('color','depth'):
        worker.observe('cam',stream,data[stream])
    clock.now+=140_000_000
    worker.observe('cam','timing',data['timing'])
    worker.tick()
    clock.now+=99_000_000
    worker.tick()
    assert worker.synchronizer._last_scene_time is None
    assert not any(event['event']=='diagnostic' for event in events)
    # A later Manager control still must not bridge a >50 ms Tracker gap.
    control=config()
    control['poses']['torso'][1]['source_realtime_ns']=1_060_000_000
    worker.configure(control)
    clock.now+=1_000_000
    worker.tick()
    assert not server.frames
    assert worker.synchronizer._last_scene_time==1_000_000_000
    assert any(event['event']=='diagnostic' for event in events)


def test_complete_receipt_does_not_refresh_stale_source_time():
    worker,clock,server,events=pipeline()
    # Freshly received components cannot make a >1 s old source current.
    for stream,message in images(source=1).items():
        if stream=='timing':
            clock.now=1_100_000_000
        worker.observe('cam',stream,message)
    worker.tick()
    assert not server.frames
    assert any(event.get('reason')=='timing host source unavailable/stale' for event in events)


class DeferredExecutor:
    """Control job completion without wall-clock timing assertions."""
    def __init__(self): self.jobs=[]
    def submit(self, fn, *args):
        future=Future(); self.jobs.append((future,fn,args)); return future
    def complete(self):
        future,fn,args=self.jobs.pop(0)
        future.set_result(fn(*args))


def async_pipeline(executor):
    clock=Clock(); server=Server(); events=[]
    worker=CloudPipeline(server,events.append,worker_instance_id='worker',
        monotonic_ns=clock,realtime_ns=clock,executor=executor)
    worker.configure(config())
    for kind,message in images().items(): assert worker.observe('cam',kind,message)
    return worker,clock,server,events


def test_async_full_build_keeps_ros_input_and_control_responsive(monkeypatch):
    from vt_capture_manager.spatial import cloud_worker
    original=cloud_worker.build_cloud
    started=threading.Event(); release=threading.Event()
    def blocked_build(*args,**kwargs):
        started.set()
        assert release.wait(3), 'test did not release background projection'
        return original(*args,**kwargs)
    monkeypatch.setattr(cloud_worker,'build_cloud',blocked_build)
    with ThreadPoolExecutor(max_workers=1) as executor:
        worker,clock,server,events=async_pipeline(executor)
        try:
            worker.tick()
            assert started.wait(2)
            assert not server.frames
            # These execute while projection remains blocked, proving that the
            # background work does not own the input/control lock.
            revised=config()
            for pose in revised['poses']['torso']:
                pose['pose']['translation_m']=[90.,0.,0.]
            worker.configure(revised)
            for kind,message in images(456,1_005_000_000).items():
                assert worker.observe('cam',kind,message)
            worker.tick()
            future=worker._inflight[0]
            assert not future.done()
        finally:
            release.set()
        future.result(timeout=3)
        worker.tick()
        frame=unpack_cloud_frame(server.frames['cam'])
        # No mixing with the newer Manager pose update during computation.
        assert frame.header['world_from_color']['translation_m']==[1.1,0.,0.]
        assert frame.header['frame_key']['shared_ros_timestamp_ns']==123


def test_async_build_has_one_job_and_publishes_only_after_completion():
    executor=DeferredExecutor()
    worker,clock,server,events=async_pipeline(executor)
    worker.tick()
    for _ in range(10):
        clock.now+=10_000_000; worker.configure(config()); worker.tick()
    assert len(executor.jobs)==1
    assert not server.frames
    assert not any(e['event']=='frames' for e in events)
    executor.complete(); worker.tick()
    assert len(server.frames)==1
    assert events[-1]['event']=='frames'


@pytest.mark.parametrize('key,value',[
    ('time_epoch',1),('coordinate_epoch',1),('scene_epoch','new'),
    ('manager_instance_id','new'),('sync_epoch',1),('interest',False),
    ('cloud',{'stride':2}),
])
def test_async_build_cannot_publish_across_retired_generation(key,value):
    executor=DeferredExecutor()
    worker,clock,server,events=async_pipeline(executor)
    worker.tick()
    revised=config(); revised[key]=value; worker.configure(revised)
    executor.complete(); worker.tick()
    assert not server.frames
    assert not any(e['event']=='frames' for e in events)


def test_async_completion_does_not_refresh_source_timestamp():
    executor=DeferredExecutor()
    worker,clock,server,events=async_pipeline(executor)
    worker.tick(); executor.complete()
    clock.now+=1_000_000_001
    worker.configure(config()); worker.tick()
    assert not server.frames
    assert any(e.get('reason')=='cloud build exceeded source freshness limit' for e in events)


def test_async_idle_then_resume_does_not_resurrect_inflight_job():
    executor=DeferredExecutor()
    worker,clock,server,events=async_pipeline(executor)
    worker.tick()
    revised=config(); revised['interest']=False; worker.configure(revised)
    worker.tick(); worker.configure(config())
    executor.complete(); worker.tick()
    assert not server.frames
    assert not any(e['event']=='frames' for e in events)
