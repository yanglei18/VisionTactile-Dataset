"""Manager-owned ROS subscriber and RGB-D preview process; no hardware opens."""
from collections import OrderedDict
from copy import deepcopy
import hashlib
import json
import os
import sys
import threading
import time
import uuid

from vt_capture_core.spatial.cloud_protocol import pack_cloud_frame
from vt_capture_core.spatial.synchronizer import CameraFrameSample, FrameSynchronizer
from vt_capture_core.spatial.timeline import PoseBuffer
from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_core.spatial.types import PoseSample
from .cloud_builder import build_cloud
from .cloud_server import CloudServer

MAX_CONTROL_BYTES=1024*1024


def _stamp(message):
    return int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)


class CloudPipeline:
    """Pure scheduling boundary, independently testable without ROS."""
    def __init__(self, server, emit, *, worker_instance_id=None, monotonic_ns=time.monotonic_ns, realtime_ns=time.time_ns):
        self.server,self.emit=server,emit
        self.worker_instance_id=worker_instance_id or uuid.uuid4().hex
        self.monotonic_ns,self.realtime_ns=monotonic_ns,realtime_ns
        self.control=None; self.generation=None; self.synchronizer=None
        self.sequence=0; self.pending_sync=None; self.last_tick_ns=0
        self._lock=threading.RLock(); self._partial={}; self._clock={}; self._last_error={}
        self._last_control_ns=0; self._accepted_local_sync=0

    def configure(self, control):
        if not isinstance(control,dict) or control.get('command')!='configure':
            raise ValueError('cloud worker only accepts configure')
        if len(json.dumps(control,allow_nan=False))>MAX_CONTROL_BYTES:
            raise ValueError('cloud control exceeds size limit')
        with self._lock:
            geometry=control['geometries']
            if not isinstance(geometry,dict) or not 1<=len(geometry)<=3:
                raise ValueError('cloud worker requires one to three cameras')
            georevision=hashlib.sha256(json.dumps({k:(v['geometry_sha256'], v.get('identity_revision',0), bool(v.get('identity',{}).get('verified'))) for k,v in geometry.items()},sort_keys=True).encode()).hexdigest()
            generation=(control['manager_instance_id'],control['scene_epoch'],control['time_epoch'],control['coordinate_epoch'],georevision)
            buffers={}
            for role,samples in control.get('poses',{}).items():
                if len(samples)>128: raise ValueError('too many tracker poses')
                buffer=PoseBuffer(max_samples=128,max_age_ns=2_000_000_000)
                for s in samples:
                    buffer.append(PoseSample(role,s['tracker_id'],s['parent_frame'],s['source_realtime_ns'],
                        s['received_monotonic_ns'],str(control['time_epoch']),str(control['coordinate_epoch']),
                        RigidTransform.from_dict(s['pose']) if s.get('pose') else None,s['pose_valid']))
                buffers[role]=buffer
            camera_buffers={name:buffers[g['config']['tracker_role']] for name,g in geometry.items() if g['config']['tracker_role'] in buffers}
            if generation!=self.generation:
                timing=control.get('timing',{})
                self.synchronizer=FrameSynchronizer(tuple(geometry),reference_camera=control.get('reference_camera'),
                    max_delta_ns=round(timing.get('max_camera_delta_ms',20)*1e6),wait_ns=round(timing.get('wait_ms',100)*1e6),
                    failover_ns=round(timing.get('reference_stale_ms',250)*1e6),restore_ns=round(timing.get('reference_recovery_ms',1000)*1e6),
                    tracker_gap_ns=round(timing.get('max_tracker_gap_ms',50)*1e6),tracker_buffers=camera_buffers)
                self.synchronizer.reset(time_epoch=str(control['time_epoch']),coordinate_epoch=str(control['coordinate_epoch']),geometry_revision=georevision)
                self._accepted_local_sync=self.synchronizer.sync_epoch
                self.pending_sync=None; self._partial={name:OrderedDict() for name in geometry}; self._clock={}
                self.server.clear(); self.generation=generation
            else:
                self.synchronizer.tracker_buffers=camera_buffers
            if self.pending_sync is not None and self.control and control['sync_epoch']!=self.control['sync_epoch']:
                self._accepted_local_sync=self.pending_sync; self.pending_sync=None
            self.control=deepcopy(control); self._last_control_ns=self.monotonic_ns()
            self.emit(dict(event='ack',worker_instance_id=self.worker_instance_id,manager_instance_id=control['manager_instance_id'],
                scene_epoch=control['scene_epoch'],time_epoch=control['time_epoch'],coordinate_epoch=control['coordinate_epoch'],sync_epoch=control['sync_epoch']))

    def _error(self, camera, reason):
        key=(camera,str(reason)[:256]); now=self.monotonic_ns()
        if now-self._last_error.get(key,-10_000_000_000)>=1_000_000_000:
            self._last_error={k:v for k,v in self._last_error.items() if now-v<5_000_000_000}
            self._last_error[key]=now; self.emit(dict(event='diagnostic',camera_id=camera,reason=key[1]))

    def observe(self, camera, stream, message):
        with self._lock:
            if self.control is None or camera not in self._partial or not self.control.get('interest'):
                return False
            if stream not in ('color','depth','timing'): return False
            try:
                config=self.control['geometries'][camera]['config']; stamp=_stamp(message)
                if stamp<=0: raise ValueError('invalid image source stamp')
                if message.header.frame_id!=config[f'{stream}_frame_id']:
                    raise ValueError(f'{stream} frame mismatch')
                if stream!='timing' and (len(message.data)>128*1024*1024 or message.width>4096 or message.height>4096):
                    raise ValueError('image size exceeds bounds')
                slots=self._partial[camera]
                slot=slots.setdefault(stamp,{'received':self.monotonic_ns()})
                if stream in slot:
                    # Duplicate shared timestamp is ambiguous; don't resurrect it.
                    slot['duplicate']=True; return False
                slot[stream]=message
                while len(slots)>6: slots.popitem(last=False)
                if not all(k in slot for k in ('color','depth','timing')): return True
                slots.pop(stamp,None)
                if slot.get('duplicate'): raise ValueError('duplicate frame component')
                timing=slot['timing']
                if timing.shared_ros_timestamp_ns!=stamp or timing.camera_name!=camera or str(timing.serial_number)!=str(config['serial']) or (config.get('model') and timing.camera_model!=config['model']):
                    raise ValueError('timing tuple identity/stamp mismatch')
                if int(timing.group_validity_flags)&27!=27:
                    raise ValueError('timing group is not complete and unique')
                source,mono=int(timing.group_host_realtime_ns),int(timing.group_host_monotonic_raw_ns)
                now=self.realtime_ns()
                if source<=0 or mono<=0 or not -500_000_000<=now-source<=1_000_000_000:
                    raise ValueError('timing host source unavailable/stale')
                previous=self._clock.get(camera)
                if previous is not None:
                    if source<=previous[0] and mono<=previous[1]: return False
                    if abs((source-previous[0])-(mono-previous[1]))>5_000_000:
                        self.server.clear(); self.emit(dict(event='clock_reset',camera_id=camera,
                            time_epoch=self.control['time_epoch'],coordinate_epoch=self.control['coordinate_epoch']))
                        self.control['interest']=False; return False
                geometry=self.control['geometries'][camera]; identity=geometry.get('identity',{})
                if not identity.get('verified') or identity.get('observed_ns') is None or not 0<=now-identity['observed_ns']<=5_000_000_000:
                    raise ValueError('camera identity is unverified')
                self._clock[camera]=(source,mono)
                key=dict(shared_ros_timestamp_ns=stamp,color_frame_number=int(timing.color_frame_number),depth_frame_number=int(timing.depth_frame_number))
                return self.synchronizer.append(CameraFrameSample(camera,key,source,slot['received'],
                    str(self.control['time_epoch']),str(self.control['coordinate_epoch']),self.generation[-1],
                    payload=(slot['color'],slot['depth']))).accepted
            except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
                self._error(camera,exc); return False

    def tick(self):
        with self._lock:
            now=self.monotonic_ns()
            if not self.control or not self.control.get('interest') or now-self._last_control_ns>2_000_000_000:
                self.server.clear(); return
            interval=int(1e9/self.control.get('cloud',{}).get('hz',10.))
            if now-self.last_tick_ns<interval or self.pending_sync is not None: return
            selection=self.synchronizer.poll(now)
            if self.synchronizer.sync_epoch!=self._accepted_local_sync:
                self.pending_sync=self.synchronizer.sync_epoch
                self.server.clear()
                self.emit(dict(event='sync_change',active_reference_camera=self.synchronizer.active_reference_camera,
                    time_epoch=self.control['time_epoch'],coordinate_epoch=self.control['coordinate_epoch'],sync_epoch=self.control['sync_epoch']))
                return
            if selection is None: return
            self.last_tick_ns=now
            headers=[]
            for frame in selection.frames:
                name=frame.camera_id; geometry=self.control['geometries'][name]
                try:
                    identity=geometry.get('identity',{})
                    if not identity.get('verified') or identity.get('observed_ns') is None or not 0<=self.realtime_ns()-identity['observed_ns']<=5_000_000_000:
                        raise ValueError('camera identity lease expired')
                    pose=selection.poses.get(name)
                    calibration=geometry.get('extrinsic')
                    buffer=self.synchronizer.tracker_buffers.get(name)
                    if pose is None or pose.pose is None or calibration is None or buffer is None or buffer.latest.tracker_id!=calibration['tracker']['tracker_id']:
                        raise ValueError('camera source time has no valid calibrated tracker pose')
                    world=pose.pose.compose(RigidTransform(calibration['transform']['translation_m'],calibration['transform']['quaternion_xyzw']))
                    color,depth=frame.payload
                    options={key:value for key,value in self.control.get('cloud',{}).items() if key in ('stride','min_depth_m','max_depth_m','color_mode')}
                    # RGB is transmitted once; the GUI can override it with
                    # source colors without recomputation or a control roundtrip.
                    options['color_mode']='rgb'
                    options['max_points']=self.control.get('cloud',{}).get('max_points_per_camera',50000)
                    xyz,rgba=build_cloud(depth,color,geometry,**options)
                    self.sequence+=1
                    header=dict(camera_id=name,frame_key=dict(frame.frame_key,scene_time_ns=selection.scene_time_ns,
                        reference_camera=selection.reference_camera),source_realtime_ns=frame.source_realtime_ns,
                        time_epoch=self.control['time_epoch'],coordinate_epoch=self.control['coordinate_epoch'],
                        sync_epoch=self.control['sync_epoch'],geometry_revision=geometry['geometry_revision'],
                        geometry_sha256=geometry['geometry_sha256'],point_count=len(xyz)//12,world_from_color=world.as_dict(),
                        worker_instance_id=self.worker_instance_id,cloud_sequence=self.sequence,
                        manager_instance_id=self.control['manager_instance_id'],scene_epoch=self.control['scene_epoch'],
                        color_frame_id=geometry['config']['color_frame_id'],time_residual_ns=selection.residuals_ns[name],
                        color_mode=options.get('color_mode','source'))
                    self.server.publish(name,pack_cloud_frame(header,xyz,rgba)); headers.append(header)
                except (ValueError,TypeError,KeyError,AttributeError,OverflowError,ImportError) as exc:
                    self._error(name,exc)
            if headers:
                self.emit(dict(event='frames',headers=headers,missing=list(selection.missing),unavailable_poses=list(selection.unavailable_poses)))


def main(argv=None):
    import argparse
    parser=argparse.ArgumentParser(); parser.add_argument('--socket',required=True); parser.add_argument('--worker-instance-id',required=True)
    args=parser.parse_args(argv)
    # Keep stdout exclusively for control acknowledgments; ROS logs stay stderr.
    event_stream=sys.stdout; sys.stdout=sys.stderr
    def emit(value):
        event_stream.write(json.dumps(value,allow_nan=False,separators=(',',':'))+'\n'); event_stream.flush()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import Image
    from vt_camera_msgs.msg import CameraFrameTiming
    rclpy.init(args=[])
    node=Node('capture_spatial_cloud_worker')
    server=CloudServer(args.socket).start(); pipeline=CloudPipeline(server,emit,worker_instance_id=args.worker_instance_id)
    inbox=[]; inbox_lock=threading.Lock(); stop=threading.Event(); subscriptions=[]; topic_signature=None
    def reader():
        try:
            while not stop.is_set():
                line=sys.stdin.buffer.readline(MAX_CONTROL_BYTES+1)
                if not line: break
                if len(line)>MAX_CONTROL_BYTES: raise ValueError('oversized cloud control')
                value=json.loads(line)
                with inbox_lock: inbox[:]=[value]
        except (ValueError,OSError) as exc: emit(dict(event='fatal',reason=str(exc)[:256]))
        finally: stop.set()
    threading.Thread(target=reader,name='cloud-manager-control',daemon=True).start()
    qos=QoSProfile(depth=3,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    emit(dict(event='ready',worker_instance_id=args.worker_instance_id,worker_pid=os.getpid()))
    try:
        while rclpy.ok() and not stop.is_set():
            with inbox_lock:
                pending=inbox.pop() if inbox else None
            if pending is not None:
                try:
                    pipeline.configure(pending)
                    cameras={name:g['config'] for name,g in pending['geometries'].items()} if pending.get('interest') else {}
                    signature=json.dumps(cameras,sort_keys=True)
                    if signature!=topic_signature:
                        for sub in subscriptions: node.destroy_subscription(sub)
                        subscriptions=[]
                        for name,config in cameras.items():
                            for stream in ('color','depth','timing'):
                                topic=config.get(f'{stream}_topic')
                                if topic:
                                    subscriptions.append(node.create_subscription(CameraFrameTiming if stream=='timing' else Image,topic,
                                        lambda message,n=name,s=stream:pipeline.observe(n,s,message),qos))
                        topic_signature=signature
                except (ValueError,TypeError,KeyError,RuntimeError) as exc: emit(dict(event='fatal',reason=str(exc)[:256]))
            rclpy.spin_once(node,timeout_sec=.005)
            pipeline.tick()
    finally:
        stop.set(); server.close(); node.destroy_node(); rclpy.shutdown()
    return 0


if __name__=='__main__': raise SystemExit(main())
