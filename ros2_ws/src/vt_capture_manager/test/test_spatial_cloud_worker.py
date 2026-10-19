from copy import deepcopy
from types import SimpleNamespace as NS
import numpy as np
from vt_capture_core.spatial.cloud_protocol import unpack_cloud_frame
from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_manager.spatial.cloud_worker import CloudPipeline


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
