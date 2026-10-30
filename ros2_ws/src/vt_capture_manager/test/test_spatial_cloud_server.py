import os
import socket
import struct
from vt_capture_core.spatial.cloud_protocol import pack_cloud_frame,unpack_cloud_frame,PREFIX_BYTES,inspect_cloud_prefix
from vt_capture_manager.spatial.cloud_server import CloudServer


def frame(sequence=1):
    return pack_cloud_frame(dict(camera_id='cam',frame_key='frame',source_realtime_ns=1,
        time_epoch=0,coordinate_epoch=0,geometry_revision=1,geometry_sha256='a'*64,point_count=0,
        world_from_color=dict(translation_m=[0,0,0],quaternion_xyzw=[0,0,0,1]),
        worker_instance_id='worker',cloud_sequence=sequence),b'',b'')


def read_frame(sock):
    prefix=sock.recv(PREFIX_BYTES); h,p=inspect_cloud_prefix(prefix); data=b''
    while len(data)<h+p: data+=sock.recv(h+p-len(data))
    return unpack_cloud_frame(prefix+data)


def test_passive_socket_sends_latest_and_cleans_only_owned_endpoint(tmp_path):
    os.chmod(tmp_path,0o700)
    server=CloudServer(tmp_path/'cloud.sock').start()
    try:
        server.publish('cam',frame(1)); server.publish('cam',frame(2))
        with socket.socket(socket.AF_UNIX) as client:
            client.settimeout(2); client.connect(str(server.path))
            assert read_frame(client).header['cloud_sequence']==2
            server.publish('cam',frame(3))
            assert read_frame(client).header['cloud_sequence']==3
    finally: server.close()
    assert not server.path.exists()


def test_full_size_cloud_partial_writes_preserve_payload_and_next_camera(tmp_path):
    os.chmod(tmp_path,0o700)
    server=CloudServer(tmp_path/'cloud.sock',send_timeout_s=5.).start()
    header=dict(unpack_cloud_frame(frame()).header)
    count=400_000  # 6.4 MB, exceeds a send burst and a Unix socket buffer.
    xyz=struct.pack('<fff',.1,.2,.3)*count
    rgba=bytes([12,34,56,255])*count
    header.update(point_count=count)
    try:
        server.publish('cam',pack_cloud_frame(header,xyz,rgba))
        header.update(camera_id='second',cloud_sequence=2)
        server.publish('second',pack_cloud_frame(header,xyz,rgba))
        with socket.socket(socket.AF_UNIX) as client:
            client.settimeout(5)
            client.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,32768)
            client.connect(str(server.path))
            for name in ('cam','second'):
                packet=read_frame(client)
                assert packet.header['camera_id']==name
                assert packet.xyz_bytes==xyz and packet.rgba_bytes==rgba
    finally:
        server.close()
