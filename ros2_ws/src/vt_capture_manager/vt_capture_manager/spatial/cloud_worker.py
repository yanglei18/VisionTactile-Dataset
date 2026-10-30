"""Manager-owned ROS subscriber and RGB-D preview process; no hardware opens."""
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
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
from .cloud_builder import build_cloud, sampling_stride
from .driver_cloud import adapt_driver_cloud
from .cloud_server import CloudServer

MAX_CONTROL_BYTES=1024*1024
MAX_PARTIAL_SLOTS=16
MAX_PARTIAL_AGE_NS=500_000_000


def cloud_input_streams(control):
    return ('cloud','timing') if control.get('cloud',{}).get('source') == 'realsense' else ('color','depth','timing')


def _stamp(message):
    return int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)


class CloudPipeline:
    """Pure scheduling boundary, independently testable without ROS."""
    def __init__(self, server, emit, *, worker_instance_id=None, monotonic_ns=time.monotonic_ns, realtime_ns=time.time_ns, executor=None):
        self.server,self.emit=server,emit
        self.worker_instance_id=worker_instance_id or uuid.uuid4().hex
        self.monotonic_ns,self.realtime_ns=monotonic_ns,realtime_ns
        self.control=None; self.generation=None; self.synchronizer=None
        self.sequence=0; self.pending_sync=None; self.last_tick_ns=0
        self._lock=threading.RLock(); self._partial={}; self._clock={}; self._last_error={}
        self._last_control_ns=0; self._accepted_local_sync=0
        # One strict-group job, or one independent job per camera (three max),
        # never a growing image queue.
        # ROS observations and Manager pose updates must keep running while a
        # full-resolution RGB-D group is projected and serialized.
        self._executor=executor; self._inflight=None; self._build_revision=0
        self._independent={}; self._camera_inflight={}; self._camera_ticks={}
        self._trace = os.environ.get('VT_CLOUD_TRACE') == '1'
        self._trace_last = {}
        self._direct_store=None; self._direct_fault=None

    def configure(self, control):
        if not isinstance(control,dict) or control.get('command')!='configure':
            raise ValueError('cloud worker only accepts configure')
        if len(json.dumps(control,allow_nan=False))>MAX_CONTROL_BYTES:
            raise ValueError('cloud control exceeds size limit')
        with self._lock:
            geometry=control['geometries']
            if not isinstance(geometry,dict) or not 1<=len(geometry)<=3:
                raise ValueError('cloud worker requires one to three cameras')
            georevision=hashlib.sha256(json.dumps(dict(
                cameras={k:(v['geometry_sha256'], v.get('identity_revision',0), bool(v.get('identity',{}).get('verified'))) for k,v in geometry.items()},
                source=control.get('cloud',{}).get('source','rgbd'),
                matching_mode=control.get('cloud',{}).get('matching_mode','synchronized'),
                timing=control.get('timing',{}),tracker_inputs=control.get('tracker_inputs',{}),
                world_frame=control.get('world_frame')),sort_keys=True).encode()).hexdigest()
            generation=(control['manager_instance_id'],control['scene_epoch'],control['time_epoch'],control['coordinate_epoch'],georevision)
            if (generation!=self.generation or self.control is None or
                any(control.get(key)!=self.control.get(key) for key in ('sync_epoch','interest','cloud'))):
                self._build_revision+=1
            buffers={}
            direct=bool(control.get('tracker_inputs')) and control.get('cloud',{}).get('matching_mode')=='independent'
            if direct:
                if generation!=self.generation or self._direct_store is None:
                    from .scene_store import SceneStore
                    from .config import SceneConfig
                    # Reuse the production source identity, dual-clock, invalid
                    # pose and bounded-history handling, not a second decoder.
                    self._direct_store=SceneStore(control['manager_instance_id'],
                        SceneConfig(world_frame=control['world_frame']),
                        realtime_ns=self.realtime_ns,monotonic_ns=self.monotonic_ns)
                    self._direct_store.time_epoch=control['time_epoch']
                    self._direct_store.coordinate_epoch=control['coordinate_epoch']
                    self._direct_fault=None
                buffers=self._direct_store.buffers
            else:
                self._direct_store=None; self._direct_fault=None
            for role,samples in (() if direct else control.get('poses',{}).items()):
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
                    tracker_gap_ns=round(timing.get('max_tracker_gap_ms',50)*1e6),tracker_buffers=camera_buffers,
                    prefer_latest=True)
                self.synchronizer.reset(time_epoch=str(control['time_epoch']),coordinate_epoch=str(control['coordinate_epoch']),geometry_revision=georevision)
                self._independent={}; self._camera_ticks={}
                if control.get('cloud',{}).get('matching_mode') == 'independent':
                    # Reuse the exact same single-camera pose interpolation and
                    # bounded wait policy. No cross-camera anchor/failover exists.
                    for name in geometry:
                        matcher=FrameSynchronizer((name,), tracker_buffers={name:camera_buffers[name]} if name in camera_buffers else {},
                            wait_ns=round(timing.get('wait_ms',100)*1e6),
                            failover_ns=round(timing.get('reference_stale_ms',250)*1e6),
                            tracker_gap_ns=round(timing.get('max_tracker_gap_ms',50)*1e6),prefer_latest=True,prefer_ready=True)
                        matcher.reset(time_epoch=str(control['time_epoch']),coordinate_epoch=str(control['coordinate_epoch']),geometry_revision=georevision)
                        self._independent[name]=matcher
                self._accepted_local_sync=self.synchronizer.sync_epoch
                self.pending_sync=None; self._partial={name:OrderedDict() for name in geometry}; self._clock={}
                self.server.clear(); self.generation=generation
            else:
                self.synchronizer.tracker_buffers=camera_buffers
                for name,matcher in self._independent.items():
                    matcher.tracker_buffers={name:camera_buffers[name]} if name in camera_buffers else {}
            if self.pending_sync is not None and self.control and control['sync_epoch']!=self.control['sync_epoch']:
                self._accepted_local_sync=self.pending_sync; self.pending_sync=None
            self.control=deepcopy(control); self._last_control_ns=self.monotonic_ns()
            if self._direct_fault == self.generation:
                self.control['interest']=False  # Await Manager acknowledgement of clock reset.
            self.emit(dict(event='ack',worker_instance_id=self.worker_instance_id,manager_instance_id=control['manager_instance_id'],
                scene_epoch=control['scene_epoch'],time_epoch=control['time_epoch'],coordinate_epoch=control['coordinate_epoch'],sync_epoch=control['sync_epoch']))

    def observe_tracker(self, role, message):
        with self._lock:
            if self._direct_store is None or not self.control or not self.control.get('interest'):
                return False
            expected={g['extrinsic']['tracker']['tracker_id'] for g in self.control['geometries'].values()
                      if g['config']['tracker_role']==role and g.get('extrinsic')}
            if message.tracker_id not in expected:
                return False
            accepted=self._direct_store.observe_tracker(role,message)
            if (self._direct_store.time_epoch!=self.control['time_epoch'] or
                    self._direct_store.coordinate_epoch!=self.control['coordinate_epoch']):
                self._direct_fault=self.generation; self.server.clear(); self._build_revision+=1
                self.control['interest']=False
                self.emit(dict(event='clock_reset',time_epoch=self.control['time_epoch'],
                               coordinate_epoch=self.control['coordinate_epoch']))
                return False
            # Malformed metadata can replace a buffer; refresh matcher refs.
            for name,matcher in self._independent.items():
                r=self.control['geometries'][name]['config']['tracker_role']
                matcher.tracker_buffers={name:self._direct_store.buffers[r]}
            return accepted

    def _error(self, camera, reason):
        key=(camera,str(reason)[:256]); now=self.monotonic_ns()
        if now-self._last_error.get(key,-10_000_000_000)>=1_000_000_000:
            self._last_error={k:v for k,v in self._last_error.items() if now-v<5_000_000_000}
            self._last_error[key]=now; self.emit(dict(event='diagnostic',camera_id=camera,reason=key[1]))

    def observe(self, camera, stream, message):
        with self._lock:
            if self.control is None or camera not in self._partial or not self.control.get('interest'):
                return False
            streams=cloud_input_streams(self.control)
            if stream not in streams: return False
            try:
                config=self.control['geometries'][camera]['config']; stamp=_stamp(message)
                if stamp<=0: raise ValueError('invalid image source stamp')
                expected_frame=config['depth_frame_id'] if stream=='cloud' else config[f'{stream}_frame_id']
                if message.header.frame_id!=expected_frame:
                    raise ValueError(f'{stream} frame mismatch')
                if stream!='timing' and (len(message.data)>128*1024*1024 or message.height>4096 or
                        (stream!='cloud' and message.width>4096)):
                    raise ValueError('image size exceeds bounds')
                slots=self._partial[camera]
                received=self.monotonic_ns()
                # Timing metadata can arrive several frames after RGB/depth.
                # Retain a bounded matching window; arrival-order eviction lets
                # late orphan timing slots repeatedly evict their newer images.
                for old_stamp,old in tuple(slots.items()):
                    if received-old['received']>=MAX_PARTIAL_AGE_NS:
                        slots.pop(old_stamp)
                slot=slots.setdefault(stamp,{'received':received})
                if stream in slot:
                    # Duplicate shared timestamp is ambiguous; don't resurrect it.
                    slot['duplicate']=True; return False
                slot[stream]=message
                while len(slots)>MAX_PARTIAL_SLOTS: slots.pop(min(slots))
                if stamp not in slots: return False
                if not all(k in slot for k in streams): return True
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
                        self.control['interest']=False; self._build_revision+=1; return False
                geometry=self.control['geometries'][camera]; identity=geometry.get('identity',{})
                if not identity.get('verified') or identity.get('observed_ns') is None or not 0<=now-identity['observed_ns']<=5_000_000_000:
                    raise ValueError('camera identity is unverified')
                self._clock[camera]=(source,mono)
                key=dict(shared_ros_timestamp_ns=stamp,color_frame_number=int(timing.color_frame_number),depth_frame_number=int(timing.depth_frame_number))
                # Synchronization can only start once this exact tuple exists.
                # Metadata latency must not spend the wait budget before other
                # cameras or the next Manager pose update can join the group.
                # Source time and the independent source-age check stay intact.
                matcher=self._independent.get(camera,self.synchronizer)
                return matcher.append(CameraFrameSample(camera,key,source,self.monotonic_ns(),
                    str(self.control['time_epoch']),str(self.control['coordinate_epoch']),self.generation[-1],
                    payload=slot['cloud'] if 'cloud' in streams else (slot['color'],slot['depth']))).accepted
            except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
                self._error(camera,exc); return False

    def tick(self):
        with self._lock:
            now=self.monotonic_ns()
            if not self.control or not self.control.get('interest') or now-self._last_control_ns>2_000_000_000:
                self._build_revision+=1
                if self._inflight is not None and self._inflight[0].done():
                    self._inflight=None
                self._camera_inflight={name:job for name,job in self._camera_inflight.items() if not job[0].done()}
                self.server.clear(); return
            for name,(future,revision,selection) in tuple(self._camera_inflight.items()):
                if future.done():
                    del self._camera_inflight[name]
                    if revision==self._build_revision:
                        self._publish_build(selection,future.result())
            if self._inflight is not None:
                future,revision,selection=self._inflight
                if not future.done(): return
                self._inflight=None
                if revision==self._build_revision:
                    self._publish_build(selection,future.result())
            interval=int(1e9/self.control.get('cloud',{}).get('hz',10.))
            if self._independent:
                self._tick_independent(now,interval)
                return
            if self._camera_inflight: return  # Drain retired jobs before a mode switch.
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
            # Frozen control/pose snapshots bind this work to the selected
            # source timestamps, never to the newest pose received mid-build.
            control=deepcopy(self.control)
            tracker_ids={name:buffer.latest.tracker_id for name,buffer in self.synchronizer.tracker_buffers.items() if buffer.latest is not None}
            first_sequence=self.sequence+1; self.sequence+=len(selection.frames)
            args=(selection,control,tracker_ids,first_sequence)
            if self._executor is None:
                self._publish_build(selection,self._build_selection(*args))
            else:
                self._inflight=(self._executor.submit(self._build_selection,*args),self._build_revision,selection)

    def _tick_independent(self, now, interval):
        # At most one job per camera, three total, no queued frame backlog.
        # A slow full-resolution wrist build cannot hold up a ready head build.
        for name,matcher in self._independent.items():
            if (name in self._camera_inflight or len(self._camera_inflight)>=3
                    or now-self._camera_ticks.get(name,-interval)<interval):
                continue
            selection=matcher.poll(now)
            if selection is None: continue
            self._camera_ticks[name]=now
            tracker_ids={key:buffer.latest.tracker_id for key,buffer in matcher.tracker_buffers.items() if buffer.latest is not None}
            self.sequence+=1
            args=(selection,deepcopy(self.control),tracker_ids,self.sequence)
            if self._executor is None:
                self._publish_build(selection,self._build_selection(*args))
            else:
                self._camera_inflight[name]=(self._executor.submit(self._build_selection,*args),self._build_revision,selection)

    def _build_selection(self, selection, control, tracker_ids, first_sequence):
        frames=[]; errors=[]
        for index,frame in enumerate(selection.frames):
            name=frame.camera_id; geometry=control['geometries'][name]
            started=time.monotonic_ns(); started_real=self.realtime_ns(); cpu_started=time.thread_time_ns()
            try:
                identity=geometry.get('identity',{})
                if not identity.get('verified') or identity.get('observed_ns') is None or not 0<=self.realtime_ns()-identity['observed_ns']<=5_000_000_000:
                    raise ValueError('camera identity lease expired')
                pose=selection.poses.get(name)
                calibration=geometry.get('extrinsic')
                if pose is not None and pose.pose is None:
                    raise ValueError(f'camera source time has no valid calibrated tracker pose: {pose.reason}')
                if pose is None or calibration is None or tracker_ids.get(name)!=calibration['tracker']['tracker_id']:
                    raise ValueError('camera source time has no valid calibrated tracker pose')
                world=pose.pose.compose(RigidTransform(calibration['transform']['translation_m'],calibration['transform']['quaternion_xyzw']))
                options={key:value for key,value in control.get('cloud',{}).items() if key in ('stride','min_depth_m','max_depth_m','color_mode')}
                # RGB is transmitted once; the GUI can override it with
                # source colors without recomputation or a control roundtrip.
                options['color_mode']='rgb'
                options['max_points']=control.get('cloud',{}).get('max_points_per_camera',50000)
                if control.get('cloud',{}).get('source') == 'realsense':
                    xyz,rgba,origin=adapt_driver_cloud(frame.payload,geometry,max_points=options['max_points'])
                    provenance={'origin':origin}
                else:
                    color,depth=frame.payload
                    xyz,rgba=build_cloud(depth,color,geometry,**options)
                    provenance={'sampling':dict(width=depth.width,height=depth.height,
                        stride=sampling_stride(depth.height,depth.width,options.get('stride',4),options['max_points']),
                        min_depth_m=options.get('min_depth_m',.1),max_depth_m=options.get('max_depth_m',3.))}
                header=dict(camera_id=name,frame_key=dict(frame.frame_key,scene_time_ns=selection.scene_time_ns,
                    reference_camera=selection.reference_camera),source_realtime_ns=frame.source_realtime_ns,
                    time_epoch=control['time_epoch'],coordinate_epoch=control['coordinate_epoch'],
                    sync_epoch=control['sync_epoch'],geometry_revision=geometry['geometry_revision'],
                    geometry_sha256=geometry['geometry_sha256'],point_count=len(xyz)//12,world_from_color=world.as_dict(),
                    worker_instance_id=self.worker_instance_id,cloud_sequence=first_sequence+index,
                    manager_instance_id=control['manager_instance_id'],scene_epoch=control['scene_epoch'],
                    color_frame_id=geometry['config']['color_frame_id'],time_residual_ns=selection.residuals_ns[name],
                    color_mode=options.get('color_mode','source'),**provenance)
                if control.get('cloud',{}).get('matching_mode') == 'independent':
                    # Ship the already-selected pose, not a later re-selection
                    # from a possibly evicted Manager interpolation history.
                    header.update(matching_mode='independent',tracker_pose=dict(
                        role=geometry['config']['tracker_role'],tracker_id=tracker_ids[name],
                        world_from_tracker=pose.pose.as_dict(),before_ns=pose.before_ns,
                        after_ns=pose.after_ns,gap_ns=pose.gap_ns))
                frames.append((name,header,pack_cloud_frame(header,xyz,rgba)))
            except (ValueError,TypeError,KeyError,AttributeError,OverflowError,ImportError) as exc:
                errors.append((name,str(exc)))
            if self._trace and started-self._trace_last.get(name,0)>1_000_000_000:
                self._trace_last[name]=started
                with self._lock:
                    buffer=(self._direct_store.buffers.get(control['geometries'][name]['config']['tracker_role'])
                            if self._direct_store else None)
                    latest=buffer.latest if buffer is not None else None
                    tracker_age=round((self.realtime_ns()-latest.source_realtime_ns)/1e6,2) if latest else None
                print('Cloud timing '+json.dumps(dict(camera=name,
                    source_age_at_build_ms=round((started_real-frame.source_realtime_ns)/1e6,2),
                    tuple_wait_ms=round((started-frame.received_monotonic_ns)/1e6,2),
                    build_ms=round((time.monotonic_ns()-started)/1e6,2),
                    cpu_ms=round((time.thread_time_ns()-cpu_started)/1e6,2),
                    tracker_age_ms=tracker_age)),file=sys.stderr,flush=True)
        return frames,errors

    def _publish_build(self, selection, result):
        frames,errors=result
        for name,reason in errors: self._error(name,reason)
        headers=[]; now=self.realtime_ns()
        for name,header,wire in frames:
            # Completion is not a fresh observation. Preserve the original
            # source-age and identity leases even if computation was delayed.
            identity=self.control['geometries'][name].get('identity',{})
            if not -500_000_000<=now-header['source_realtime_ns']<=1_000_000_000:
                self._error(name,'cloud build exceeded source freshness limit'); continue
            if not identity.get('verified') or identity.get('observed_ns') is None or not 0<=now-identity['observed_ns']<=5_000_000_000:
                self._error(name,'camera identity lease expired'); continue
            self.server.publish(name,wire); headers.append(header)
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
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import Image, PointCloud2
    from vt_camera_msgs.msg import CameraFrameTiming
    from .tracker_receiver import TrackerReceiver
    rclpy.init(args=[])
    node=Node('capture_spatial_cloud_worker')
    ros_executor=SingleThreadedExecutor(); ros_executor.add_node(node)
    server=CloudServer(args.socket).start()
    builder=ThreadPoolExecutor(max_workers=3,thread_name_prefix='cloud-projection')
    pipeline=CloudPipeline(server,emit,worker_instance_id=args.worker_instance_id,executor=builder)
    inbox=[]; inbox_lock=threading.Lock(); stop=threading.Event(); subscriptions=[]; topic_signature=None
    tracker_receiver=None; tracker_signature=None
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
                    streams=cloud_input_streams(pending)
                    tracker_inputs=pending.get('tracker_inputs',{}) if cameras else {}
                    if tracker_inputs!=tracker_signature:
                        if tracker_receiver is not None:tracker_receiver.close()
                        tracker_receiver=TrackerReceiver(tracker_inputs,pipeline.observe_tracker) if tracker_inputs else None
                        tracker_signature=dict(tracker_inputs)
                    signature=json.dumps((cameras,streams,tracker_inputs),sort_keys=True)
                    if signature!=topic_signature:
                        for owner,sub in subscriptions: owner.destroy_subscription(sub)
                        subscriptions=[]
                        for name,config in cameras.items():
                            for stream in streams:
                                topic=config.get(f'{stream}_topic')
                                if topic:
                                    message_type={'timing':CameraFrameTiming,'cloud':PointCloud2}.get(stream,Image)
                                    subscriptions.append((node,node.create_subscription(message_type,topic,
                                        lambda message,n=name,s=stream:pipeline.observe(n,s,message),qos)))
                        topic_signature=signature
                except (ValueError,TypeError,KeyError,RuntimeError) as exc: emit(dict(event='fatal',reason=str(exc)[:256]))
            ros_executor.spin_once(timeout_sec=.005)
            pipeline.tick()
            if tracker_receiver is not None and tracker_receiver.error:
                raise RuntimeError(tracker_receiver.error)
            # The full-cloud DDS wait set can be continuously ready. Yield the
            # GIL so builders, Manager pose updates and binary sending progress.
            # No input frame or timestamp is replaced by this scheduling yield.
            time.sleep(0)
    finally:
        stop.set()
        if tracker_receiver is not None:tracker_receiver.close()
        ros_executor.shutdown()
        builder.shutdown(wait=True,cancel_futures=True)
        server.close(); node.destroy_node(); rclpy.shutdown()
    return 0


if __name__=='__main__': raise SystemExit(main())
