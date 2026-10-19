"""Cloud display cannot mix time, identity or geometry generations."""
import struct
import os
import socket
import threading
import time

from vt_capture_console.backend.cloud_backend import CloudBackend
from vt_capture_core.spatial.cloud_protocol import pack_cloud_frame


def cloud_frame(**changes):
    header = dict(protocol="vt.cloud", version=1, manager_instance_id="manager", scene_epoch="scene",
                  camera_id="d405_1", frame_key="frame-1", source_realtime_ns=100,
                  time_epoch=1, coordinate_epoch=2, sync_epoch=3, geometry_revision=4,
                  geometry_sha256="a"*64, point_count=1,
                  world_from_color={"translation_m": [1., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]},
                  worker_instance_id="worker", cloud_sequence=1)
    header.update(changes)
    return pack_cloud_frame(header, struct.pack("<fff", 0., 0., 1.), bytes([100, 120, 140, 255]))


def scene_frame():
    return dict(manager_instance_id="manager", scene_epoch="scene", time_epoch=1,
                coordinate_epoch=2, sync_epoch=3, cameras=[dict(id="d405_1", frame_key="frame-1",
                geometry_revision=4, geometry_sha256="a"*64)])


def test_cloud_requires_all_identity_geometry_and_frame_keys():
    backend = CloudBackend(autostart=False)
    backend.configure(dict(socket_path="/tmp/test-cloud", worker_instance_id="worker", worker_pid=1), "manager")
    backend.accept_frame(cloud_frame())
    scene = scene_frame()
    assert len(backend.latest(scene)) == 1
    scene["sync_epoch"] = 9
    assert backend.latest(scene) == {}
    scene = scene_frame()
    scene["cameras"][0].pop("frame_key")
    assert backend.latest(scene) == {}
    backend.close()


def test_restarted_worker_clears_old_clouds_and_rejects_old_data():
    backend = CloudBackend(autostart=False)
    descriptor = dict(socket_path="/tmp/test-cloud", worker_instance_id="worker", worker_pid=1)
    backend.configure(descriptor, "manager")
    backend.accept_frame(cloud_frame())
    backend.configure({**descriptor, "worker_instance_id": "worker-2"}, "manager")
    assert backend.latest(scene_frame()) == {}
    assert backend.accept_frame(cloud_frame()) is False
    backend.close()


def test_binary_socket_reader_accepts_split_frame_with_verified_peer(tmp_path):
    path = tmp_path / "cloud.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    server.listen(1)
    release = threading.Event()
    wire = cloud_frame()
    def serve():
        connection, _ = server.accept()
        with connection:
            connection.sendall(wire[:9])
            connection.sendall(wire[9:])
            release.wait(2)
    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    backend = CloudBackend()
    try:
        backend.configure(dict(socket_path=str(path), worker_instance_id="worker", worker_pid=os.getpid()), "manager")
        backend.set_interest("test", True)
        deadline = time.monotonic()+1.
        frames = {}
        while not frames and time.monotonic() < deadline:
            frames = backend.latest(scene_frame())
            time.sleep(.005)
        assert frames["d405_1"]["positions"].tolist() == [[0., 0., 1.]]
        assert not frames["d405_1"]["positions"].flags.writeable
    finally:
        backend.close()
        release.set()
        thread.join(1)
        server.close()
