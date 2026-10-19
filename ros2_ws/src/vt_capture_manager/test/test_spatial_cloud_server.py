import os
import socket
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
