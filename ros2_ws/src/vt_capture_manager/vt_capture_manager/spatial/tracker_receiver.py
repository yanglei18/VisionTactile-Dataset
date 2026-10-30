"""Small read-only ROS pose receiver, isolated from full-cloud Python/GIL work.

Private parent pipe, bounded queue, original clocks/identity/validity unchanged.
This is preview transport only; it neither publishes nor controls any device.
"""
from collections import deque
import json
import math
import os
import select
import subprocess
import sys
import threading
from types import SimpleNamespace as NS

MAX_LINE=8192
MAX_PENDING=384
ROLES={'torso','left_wrist','right_wrist'}


def encode_sample(role, msg):
    p,q=msg.pose.position,msg.pose.orientation
    # Invalid measurements must remain invalid observations, not disappear
    # between two valid samples and permit interpolation across tracking loss.
    values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    finite=all(math.isfinite(v) for v in values)
    return dict(role=role,message_role=msg.role,tracker_id=msg.tracker_id,
        frame=msg.header.frame_id,real=int(msg.host_realtime_ns),mono=int(msg.host_monotonic_ns),
        valid=bool(msg.pose_valid) and finite,pose=values if finite else [0.,0.,0.,0.,0.,0.,1.])


def decode_sample(value):
    if (not isinstance(value,dict) or value.get('role') not in ROLES or
            any(type(value.get(k)) is not int or value[k]<=0 for k in ('real','mono')) or
            type(value.get('valid')) is not bool or
            any(not isinstance(value.get(k),str) or len(value[k])>256 for k in ('tracker_id','frame','message_role'))):
        raise ValueError('invalid private tracker sample')
    p=value.get('pose')
    if (not isinstance(p,list) or len(p)!=7 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in p)):
        raise ValueError('invalid private tracker pose')
    return value['role'],NS(role=value['message_role'],tracker_id=value['tracker_id'],
        header=NS(frame_id=value['frame']),host_realtime_ns=value['real'],host_monotonic_ns=value['mono'],
        pose_valid=value['valid'],pose=NS(position=NS(x=p[0],y=p[1],z=p[2]),
                                        orientation=NS(x=p[3],y=p[4],z=p[5],w=p[6])))


def validate_topics(topics):
    if (not isinstance(topics,dict) or not topics or set(topics)-ROLES or
            any(not isinstance(t,str) or not t.startswith('/') or len(t)>256 or
                any(c.isspace() for c in t) for t in topics.values())):
        raise ValueError('invalid tracker subscriptions')
    return topics


class TrackerReceiver:
    """One explicit child; EOF stops it and closing never touches ROS publishers."""
    def __init__(self, topics, observe):
        topics=validate_topics(topics)
        self._stop=threading.Event();self.error=None;self.observe=observe
        self.process=subprocess.Popen([sys.executable,'-m',__name__,'--topics',json.dumps(topics)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=None,bufsize=0,close_fds=True)
        self.thread=threading.Thread(target=self._read,name='cloud-tracker-pipe',daemon=True)
        self.thread.start()

    def _read(self):
        try:
            # Buffered byte framing avoids one read(2) per byte with bufsize=0.
            pending=bytearray()
            while not self._stop.is_set():
                block=os.read(self.process.stdout.fileno(),65536)
                if not block:break
                pending.extend(block)
                while b'\n' in pending:
                    line,_,pending=pending.partition(b'\n')
                    if len(line)>MAX_LINE:raise ValueError('oversized tracker sample')
                    role,msg=decode_sample(json.loads(line))
                    if not self._stop.is_set():self.observe(role,msg)
                if len(pending)>MAX_LINE:raise ValueError('unterminated tracker sample')
            if not self._stop.is_set():self.error='tracker receiver exited'
        except (OSError,ValueError,TypeError,KeyError) as exc:
            if not self._stop.is_set():self.error=str(exc)[:256]

    def close(self):
        self._stop.set()
        self.process.stdin.close()  # Child sees EOF even if parent is killed.
        try:self.process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2.)
        self.thread.join(2.);self.process.stdout.close()


def main(argv=None):
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--topics',required=True)
    args=parser.parse_args(argv);topics=validate_topics(json.loads(args.topics))
    output=sys.stdout.fileno();sys.stdout=sys.stderr
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from vt_tracker_msgs.msg import TrackerSample
    rclpy.init(args=[])
    node=rclpy.create_node('capture_spatial_tracker_receiver',enable_rosout=False,start_parameter_services=False)
    stop=threading.Event();condition=threading.Condition();pending=deque(maxlen=MAX_PENDING)
    def parent_watch():
        try:sys.stdin.buffer.read(1)
        finally:
            stop.set()
            with condition:condition.notify_all()
    threading.Thread(target=parent_watch,daemon=True).start()
    def observe(role,msg):
        packet=json.dumps(encode_sample(role,msg),separators=(',',':'),allow_nan=False).encode()+b'\n'
        if len(packet)>MAX_LINE:return
        with condition:
            pending.append(packet);condition.notify()
    def write():
        os.set_blocking(output,False)
        try:
            while not stop.is_set():
                with condition:
                    condition.wait_for(lambda:pending or stop.is_set(),timeout=.1)
                    packet=memoryview(pending.popleft()) if pending else None
                while packet and not stop.is_set():
                    if not select.select([],[output],[],.05)[1]:continue
                    try:count=os.write(output,packet)
                    except BlockingIOError:continue
                    packet=packet[count:]
        except (OSError,ValueError):stop.set()
    writer=threading.Thread(target=write,name='tracker-private-output',daemon=True);writer.start()
    qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions=[node.create_subscription(TrackerSample,topic,
        lambda m,r=role:observe(r,m),qos) for role,topic in topics.items()]
    try:
        while rclpy.ok() and not stop.is_set():rclpy.spin_once(node,timeout_sec=.02)
    finally:
        stop.set()
        with condition:condition.notify_all()
        writer.join(1.);node.destroy_node();rclpy.shutdown()
    return 0


if __name__=='__main__':raise SystemExit(main())
