"""Bounded Manager-owned process lifecycle and geometry/epoch handshake."""
from copy import deepcopy
import json
import os
from pathlib import Path
import select
import stat
import subprocess
import threading
import time
import uuid

from vt_capture_core.spatial.cloud_protocol import validate_cloud_header
MAX_CONTROL_BYTES = 1024 * 1024


class CloudSupervisor:
    def __init__(self, store, socket_path, *, python_executable='/usr/bin/python3'):
        self.store,self.path,self.python_executable=store,Path(socket_path),python_executable
        self._lock=threading.RLock(); self._stop=threading.Event()
        self._process=None; self._thread=None; self._ready=False; self._headers={}
        self._worker_id=None; self._ack=None; self._error=''; self._attempts=0; self._socket_inode=None

    def start(self):
        if self._thread is not None: raise RuntimeError('cloud supervisor is single-use')
        self._thread=threading.Thread(target=self._run,name='spatial-cloud-supervisor',daemon=True)
        self._thread.start(); return self

    def description(self):
        with self._lock:
            process=self._process
            if not self._ready or process is None or process.poll() is not None: return None
            return dict(socket_path=str(self.path),worker_instance_id=self._worker_id,worker_pid=process.pid,
                        manager_instance_id=self.store.manager_instance_id)

    def latest_headers(self):
        with self._lock:
            if not self._ready or self._process is None or self._process.poll() is not None: return {}
            now=time.monotonic()
            return {key:deepcopy(header) for key,(received,header) in self._headers.items() if now-received<.5}

    def status(self):
        with self._lock:
            return dict(ready=self._ready,error=self._error,attempts=self._attempts)

    def _configuration(self):
        # SceneStore protects pose buffers and Manager-owned epochs together.
        with self.store._lock:
            registry=self.store.camera_registry
            if registry is None: return None
            geometries={camera['id']:registry.geometry(camera['id']) for camera in self.store.config.cameras}
            poses={}
            for role,buffer in self.store.buffers.items():
                samples=buffer.samples
                cutoff=samples[-1].source_realtime_ns-700_000_000 if samples else 0
                poses[role]=[dict(tracker_id=s.tracker_id,parent_frame=s.parent_frame,
                    source_realtime_ns=s.source_realtime_ns,received_monotonic_ns=s.received_monotonic_ns,
                    pose=s.pose.as_dict() if s.pose else None,pose_valid=s.pose_valid)
                    for s in samples[-96:] if s.source_realtime_ns>=cutoff]
            return dict(command='configure',manager_instance_id=self.store.manager_instance_id,
                scene_epoch=self.store.scene_epoch,time_epoch=self.store.time_epoch,
                coordinate_epoch=self.store.coordinate_epoch,sync_epoch=self.store.sync_epoch,
                reference_camera=self.store.config.reference_camera,geometries=geometries,poses=poses,
                cloud=self.store.config.cloud,timing=self.store.config.timing,interest=self.store.has_interest('cloud'))

    def _event(self,event):
        kind=event.get('event')
        if kind=='ready':
            with self._lock:
                if event.get('worker_instance_id')==self._worker_id and event.get('worker_pid')==self._process.pid:
                    info=self.path.lstat()
                    if not stat.S_ISSOCK(info.st_mode) or info.st_uid!=os.getuid():
                        raise ValueError('worker endpoint is not an owned socket')
                    self._socket_inode=info.st_ino
                    self._ready=True
        elif kind=='ack':
            with self._lock: self._ack=event
        elif kind=='frames':
            headers=event.get('headers',[])
            if not isinstance(headers,list) or len(headers)>3: raise ValueError('invalid worker frame group')
            checked={}
            for header in headers:
                header=dict(protocol='vt.cloud',version=1,**header)
                validate_cloud_header(header)
                if header['worker_instance_id']!=self._worker_id or header.get('manager_instance_id')!=self.store.manager_instance_id:
                    raise ValueError('worker identity mismatch')
                checked[header['camera_id']]=(time.monotonic(),header)
            with self._lock:
                # Each event is one reference-time group. Omitted cameras are
                # removed immediately instead of borrowing previous geometry.
                self._headers=checked
        elif kind in ('sync_change','clock_reset'):
            with self.store._lock:
                if event.get('time_epoch')!=self.store.time_epoch or event.get('coordinate_epoch')!=self.store.coordinate_epoch:
                    return
                if kind=='clock_reset': self.store._reset()
                elif event.get('sync_epoch')==self.store.sync_epoch:
                    self.store.sync_epoch+=1
                    self.store.active_reference_camera=event.get('active_reference_camera')
            with self._lock: self._headers.clear()
        elif kind in ('diagnostic','fatal'):
            with self._lock: self._error=str(event.get('reason','cloud unavailable'))[:512]

    def _spawn(self):
        worker_id=uuid.uuid4().hex
        with self._lock: self._attempts+=1
        process=subprocess.Popen([self.python_executable,'-m','vt_capture_manager.spatial.cloud_worker',
            '--socket',str(self.path),'--worker-instance-id',worker_id],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=None,bufsize=0,close_fds=True)
        os.set_blocking(process.stdin.fileno(),False); os.set_blocking(process.stdout.fileno(),False)
        with self._lock:
            self._process=process; self._worker_id=worker_id; self._ready=False; self._headers={}; self._ack=None
        return process

    @staticmethod
    def _stop_process(process):
        if process.poll() is None:
            # EOF is the worker's normal shutdown signal.
            try: process.stdin.close()
            except OSError: pass
            try: process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                process.terminate()
                try: process.wait(timeout=2.)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=2.)
        for stream in (process.stdin,process.stdout):
            try: stream.close()
            except OSError: pass

    def _serve_process(self,process):
        incoming=bytearray(); outgoing=b''; next_control=0.; stalled=None
        while not self._stop.is_set() and process.poll() is None:
            now=time.monotonic()
            if now>=next_control and not outgoing:
                control=self._configuration(); next_control=now+.05
                if control is not None:
                    outgoing=json.dumps(control,separators=(',',':'),allow_nan=False).encode()+b'\n'
                    if len(outgoing)>MAX_CONTROL_BYTES: raise ValueError('cloud configuration exceeds wire limit')
                    stalled=now
            if outgoing and stalled is not None and now-stalled>2.:
                raise RuntimeError('cloud worker stopped acknowledging input')
            readable,writable,_=select.select([process.stdout],[process.stdin] if outgoing else [],[],.025)
            if writable:
                try: outgoing=outgoing[os.write(process.stdin.fileno(),outgoing[:65536]):]
                except BlockingIOError: pass
            if readable:
                data=os.read(process.stdout.fileno(),65536)
                if not data: break
                incoming.extend(data)
                if len(incoming)>MAX_CONTROL_BYTES: raise ValueError('oversized cloud worker response')
                while b'\n' in incoming:
                    line,_,remaining=incoming.partition(b'\n'); incoming=bytearray(remaining)
                    self._event(json.loads(line))

    def _run(self):
        while not self._stop.is_set() and self._attempts<3:
            process=None
            try:
                process=self._spawn(); self._serve_process(process)
            except (OSError,ValueError,RuntimeError,TypeError,KeyError) as exc:
                with self._lock: self._error=str(exc)[:512]
            finally:
                if process is not None: self._stop_process(process)
                try:
                    info=self.path.lstat()
                    if self._socket_inode is not None and info.st_ino==self._socket_inode and stat.S_ISSOCK(info.st_mode) and info.st_uid==os.getuid():
                        self.path.unlink()
                except FileNotFoundError: pass
                self._socket_inode=None
                with self._lock: self._ready=False; self._headers={}
            if self._stop.is_set(): break
            # Restart changes scene generation; delayed frames from the old
            # process can no longer match a newly produced scene.
            with self.store._lock: self.store.scene_epoch=uuid.uuid4().hex
            if self._stop.wait(min(2**self._attempts,8)): break

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(7.)
            if self._thread.is_alive(): raise RuntimeError('cloud supervisor did not stop')
