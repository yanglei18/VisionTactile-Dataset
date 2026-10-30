"""Private, passive VTPC stream with per-camera latest-value backpressure."""
from dataclasses import dataclass, field
import os
from pathlib import Path
import selectors
import socket
import stat
import struct
import threading
import time
from vt_capture_core.spatial.cloud_protocol import MAX_FRAME_BYTES, PREFIX_BYTES, inspect_cloud_prefix
from ..server import _prepare_parent


@dataclass
class _Client:
    sock: object
    delivered: dict = field(default_factory=dict)
    current: memoryview | None = None
    deadline: float = 0.


class CloudServer:
    def __init__(self, path, *, max_clients=8, send_timeout_s=2.):
        if not 1 <= max_clients <= 32 or not .05 <= send_timeout_s <= 10:
            raise ValueError('invalid cloud server bounds')
        self.path, self.max_clients, self.send_timeout_s = Path(path), max_clients, send_timeout_s
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._latest, self._sequence = {}, 0
        self._thread, self._listener, self._inode = None, None, None

    def start(self):
        if self._thread is not None:
            raise RuntimeError('cloud server is single-use')
        _prepare_parent(self.path)
        listener = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        try:
            listener.bind(str(self.path)); self._inode=self.path.lstat().st_ino
            os.chmod(self.path,0o600); listener.listen(self.max_clients); listener.setblocking(False)
            self._listener=listener
            self._thread=threading.Thread(target=self._run,name='spatial-cloud-socket',daemon=True)
            self._thread.start()
        except BaseException:
            listener.close(); self._cleanup(); raise
        return self

    def publish(self, camera_id, frame):
        if not isinstance(camera_id,str) or not camera_id or len(camera_id)>128 or not isinstance(frame,bytes) or len(frame)>MAX_FRAME_BYTES:
            raise ValueError('invalid cloud frame')
        h,p=inspect_cloud_prefix(frame[:PREFIX_BYTES])
        if len(frame)!=PREFIX_BYTES+h+p:
            raise ValueError('cloud frame length mismatch')
        with self._lock:
            if camera_id not in self._latest and len(self._latest)>=3:
                raise ValueError('at most three cloud cameras')
            self._sequence+=1
            self._latest[camera_id]=(self._sequence,frame)

    def clear(self):
        with self._lock:
            self._latest.clear()

    def _run(self):
        selector=selectors.DefaultSelector(); clients={}
        selector.register(self._listener,selectors.EVENT_READ,None)
        def drop(client):
            clients.pop(client.sock.fileno(),None)
            try: selector.unregister(client.sock)
            except KeyError: pass
            client.sock.close()
        try:
            while not self._stop.is_set():
                with self._lock:
                    latest=dict(self._latest)
                now=time.monotonic()
                for client in list(clients.values()):
                    if client.current is not None and now>client.deadline:
                        drop(client); continue
                    if client.current is None:
                        pending=[(seq,name,data) for name,(seq,data) in latest.items() if seq>client.delivered.get(name,0)]
                        if pending:
                            seq,name,data=min(pending)
                            client.current=memoryview(data); client.delivered[name]=seq
                            client.deadline=now+self.send_timeout_s
                            selector.modify(client.sock,selectors.EVENT_READ|selectors.EVENT_WRITE,client)
                for key,mask in selector.select(.02):
                    if key.data is None:
                        try: connection,_=self._listener.accept()
                        except BlockingIOError: continue
                        uid=struct.unpack('3i',connection.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))[1]
                        if uid!=os.getuid() or len(clients)>=self.max_clients:
                            connection.close(); continue
                        # A native wrist frame is ~13 MiB. The small Unix
                        # default send buffer forces dozens of producer/GIL
                        # handoffs per frame. Bound it explicitly; the kernel
                        # may cap this hint, correctness never depends on it.
                        connection.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,2*1024*1024)
                        connection.setblocking(False); client=_Client(connection)
                        clients[connection.fileno()]=client; selector.register(connection,selectors.EVENT_READ,client)
                        continue
                    client=key.data
                    try:
                        if mask & selectors.EVENT_READ:
                            # This channel has no requests; EOF or client input ends it.
                            client.sock.recv(1); drop(client); continue
                        if mask & selectors.EVENT_WRITE and client.current is not None:
                            # Full native clouds are 6--15 MiB. A single 64 KiB
                            # write per selector cycle repeatedly competes with
                            # the busy ROS Python thread and ages the frame out.
                            # Drain a bounded burst, retaining partial writes
                            # and EAGAIN rather than dropping a healthy client.
                            budget=4*1024*1024
                            while client.current and budget>0:
                                try:
                                    count=client.sock.send(client.current[:min(budget,1024*1024)])
                                except BlockingIOError:
                                    break
                                if count<=0: raise ConnectionError('cloud socket closed')
                                client.current=client.current[count:]; budget-=count
                            if not client.current:
                                client.current=None; selector.modify(client.sock,selectors.EVENT_READ,client)
                    except (OSError,ValueError): drop(client)
        finally:
            for client in list(clients.values()): drop(client)
            selector.close(); self._listener.close(); self._cleanup()

    def _cleanup(self):
        try:
            info=self.path.lstat()
            if info.st_ino==self._inode and stat.S_ISSOCK(info.st_mode): self.path.unlink()
        except FileNotFoundError: pass

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(3.)
            if self._thread.is_alive(): raise RuntimeError('cloud socket did not stop')
