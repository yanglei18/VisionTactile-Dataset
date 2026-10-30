"""Cloud display cannot mix time, identity or geometry generations."""
import struct
import os
import socket
import threading
import time
import weakref

import pytest

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


@pytest.fixture
def configured_backend():
    backend = CloudBackend(autostart=False)
    backend.configure(dict(socket_path="/tmp/test-cloud", worker_instance_id="worker", worker_pid=1), "manager")
    yield backend
    backend.close()


def test_new_binary_cloud_does_not_replace_frame_still_named_by_scene(configured_backend):
    backend = configured_backend
    assert backend.accept_frame(cloud_frame())
    original = backend.latest(scene_frame())["d405_1"]
    assert backend.accept_frame(cloud_frame(cloud_sequence=2, frame_key="frame-2"))
    assert backend.accept_frame(cloud_frame(cloud_sequence=3, frame_key="frame-3"))
    assert backend.latest(scene_frame())["d405_1"] is original
    scene = scene_frame()
    scene["cameras"][0]["frame_key"] = "frame-2"
    assert backend.latest(scene)["d405_1"]["header"]["cloud_sequence"] == 2


def test_large_frame_history_has_a_byte_budget(configured_backend, monkeypatch):
    from vt_capture_console.backend import cloud_backend as module
    # Retain the wire owner (including RGBA8/header) plus GPU RGBA32 colors,
    # not just the XYZ view. This budget permits exactly two tiny frames.
    monkeypatch.setattr(module,'MAX_CAMERA_BYTES',2*(len(cloud_frame())+16))
    for sequence in range(1,6):
        configured_backend.accept_frame(cloud_frame(cloud_sequence=sequence,frame_key=f'frame-{sequence}'))
    assert len(configured_backend._frames['d405_1']) == 2
    assert configured_backend.latest(scene_frame()) == {}


def test_scene_ahead_of_cloud_waits_for_exact_frame(configured_backend):
    backend = configured_backend
    assert backend.accept_frame(cloud_frame())
    scene = scene_frame()
    frame_key = {"scene_time_ns": 200, "reference_camera": "d405_1", "sequence": 2}
    scene["cameras"][0]["frame_key"] = frame_key
    assert backend.latest(scene) == {}
    assert backend.accept_frame(cloud_frame(cloud_sequence=2, frame_key=frame_key))
    assert backend.latest(scene)["d405_1"]["header"]["frame_key"] == frame_key


def independent_scene_and_cloud():
    scene = scene_frame()
    scene.update(time_mode='independent', generated_realtime_ns=200)
    scene['cameras'][0].update(frame_key='newer-metadata', tracker_role='left_wrist',
                               registration_state='ready')
    scene['trackers'] = [dict(role='left_wrist', tracker_id='left-device')]
    bound = dict(role='left_wrist', tracker_id='left-device', before_ns=100, after_ns=100,
                 gap_ns=0, world_from_tracker=dict(translation_m=[0.,0.,0.],
                                                  quaternion_xyzw=[0.,0.,0.,1.]))
    return scene, cloud_frame(matching_mode='independent', tracker_pose=bound,
                             frame_key=dict(scene_time_ns=100, reference_camera='d405_1', sequence=1))


def test_independent_packet_with_bound_poses_does_not_wait_for_newer_metadata(configured_backend):
    scene, packet = independent_scene_and_cloud()
    assert configured_backend.accept_frame(packet)
    cloud = configured_backend.latest(scene)['d405_1']
    assert cloud['header']['frame_key']['sequence'] == 1
    assert cloud['header']['tracker_pose']['tracker_id'] == 'left-device'


@pytest.mark.parametrize('bad', ['identity', 'role', 'geometry', 'epoch', 'age', 'future', 'registration'])
def test_independent_packet_still_requires_binding_geometry_and_source_age(configured_backend, bad):
    scene, packet = independent_scene_and_cloud()
    assert configured_backend.accept_frame(packet)
    if bad == 'identity': scene['trackers'][0]['tracker_id'] = 'replacement'
    if bad == 'role': scene['cameras'][0]['tracker_role'] = 'right_wrist'
    if bad == 'geometry': scene['cameras'][0]['geometry_revision'] += 1
    if bad == 'epoch': scene['coordinate_epoch'] += 1
    if bad == 'age': scene['generated_realtime_ns'] = 1_000_000_100
    if bad == 'future': scene['generated_realtime_ns'] = 99
    if bad == 'registration': scene['cameras'][0]['registration_state'] = 'invalid'
    assert configured_backend.latest(scene) == {}


def test_three_camera_async_arrivals_never_fill_a_missing_scene_camera(configured_backend):
    backend = configured_backend
    scene = scene_frame()
    scene["cameras"] = [{**scene["cameras"][0], "id": name} for name in ("d436", "d405_1", "d405_2")]
    for sequence, camera in enumerate(scene["cameras"], 1):
        assert backend.accept_frame(cloud_frame(camera_id=camera["id"], cloud_sequence=sequence))
    assert set(backend.latest(scene)) == {"d436", "d405_1", "d405_2"}
    assert backend.accept_frame(cloud_frame(camera_id="d436", cloud_sequence=4, frame_key="frame-2"))
    assert set(backend.latest(scene)) == {"d436", "d405_1", "d405_2"}
    for camera in scene["cameras"]:
        camera["frame_key"] = "frame-2"
    scene["cameras"][1].pop("frame_key")
    assert set(backend.latest(scene)) == {"d436"}
    assert backend.accept_frame(cloud_frame(camera_id="d405_2", cloud_sequence=5, frame_key="frame-2"))
    assert set(backend.latest(scene)) == {"d436", "d405_2"}


@pytest.mark.parametrize("field,value", [("scene_epoch", "new-scene"), ("time_epoch", 9),
                                        ("coordinate_epoch", 9), ("sync_epoch", 9)])
def test_new_generation_releases_all_old_camera_frames(configured_backend, field, value):
    backend = configured_backend
    assert backend.accept_frame(cloud_frame())
    assert backend.accept_frame(cloud_frame(camera_id="d436", cloud_sequence=2))
    assert backend.accept_frame(cloud_frame(cloud_sequence=4, **{field: value}))
    assert backend.latest(scene_frame()) == {}
    scene = scene_frame()
    scene[field] = value
    assert set(backend.latest(scene)) == {"d405_1"}
    # An older in-flight frame must not revert the generation or revive another camera.
    assert not backend.accept_frame(cloud_frame(camera_id="d436", cloud_sequence=3))
    assert set(backend.latest(scene)) == {"d405_1"}


@pytest.mark.parametrize("field,value", [("geometry_revision", 5), ("geometry_sha256", "b"*64)])
def test_new_camera_geometry_releases_only_that_cameras_history(configured_backend, field, value):
    backend = configured_backend
    assert backend.accept_frame(cloud_frame())
    assert backend.accept_frame(cloud_frame(camera_id="d436", cloud_sequence=2))
    assert backend.accept_frame(cloud_frame(cloud_sequence=3, **{field: value}))
    scene = scene_frame()
    scene["cameras"].append({**scene["cameras"][0], "id": "d436"})
    assert set(backend.latest(scene)) == {"d436"}
    scene["cameras"][0][field] = value
    assert set(backend.latest(scene)) == {"d405_1", "d436"}


def test_cloud_history_expires_without_refreshing_age_on_reads(configured_backend, monkeypatch):
    backend = configured_backend
    now = [10.]
    monkeypatch.setattr("vt_capture_console.backend.cloud_backend.time.monotonic", lambda: now[0])
    assert backend.accept_frame(cloud_frame())
    expired_positions = weakref.ref(backend.latest(scene_frame())["d405_1"]["positions"])
    now[0] = 10.5
    assert backend.accept_frame(cloud_frame(cloud_sequence=2, frame_key="frame-2"))
    assert backend.latest(scene_frame())
    now[0] = 11.001
    assert backend.latest(scene_frame()) == {}
    assert expired_positions() is None
    scene = scene_frame()
    scene["cameras"][0]["frame_key"] = "frame-2"
    assert backend.latest(scene)
    now[0] = 11.501
    assert backend.latest(scene) == {}
    assert not any(backend._frames.values())
    # Expiry must not reset the anti-replay sequence fence.
    assert not backend.accept_frame(cloud_frame())


def test_cloud_history_has_three_camera_eight_frame_memory_bound(configured_backend):
    backend = configured_backend
    cameras = ("d436", "d405_1", "d405_2")
    first_positions = None
    for index in range(10):
        for offset, name in enumerate(cameras):
            assert backend.accept_frame(cloud_frame(camera_id=name, cloud_sequence=index*3+offset+1,
                                                   frame_key=f"frame-{index}"))
        if index == 1:
            first_positions = weakref.ref(backend.latest(scene_frame())["d405_1"]["positions"])
    assert len(backend._frames) == 3
    assert all(len(history) == 8 for history in backend._frames.values())
    assert first_positions() is None
    assert not backend.accept_frame(cloud_frame(camera_id="unexpected", cloud_sequence=31))
    scene = scene_frame()
    for index in range(10):
        scene["cameras"][0]["frame_key"] = f"frame-{index}"
        assert bool(backend.latest(scene)) == (index >= 2)


def test_complete_scene_survives_partial_15hz_clouds_within_original_lease(configured_backend, monkeypatch):
    backend = configured_backend
    now = [10.]
    monkeypatch.setattr("vt_capture_console.backend.cloud_backend.time.monotonic", lambda: now[0])
    scene = scene_frame()
    names = ("d436", "d405_1", "d405_2")
    scene["cameras"] = [{**scene["cameras"][0], "id": name} for name in names]
    for sequence, name in enumerate(names, 1):
        assert backend.accept_frame(cloud_frame(camera_id=name, cloud_sequence=sequence))
    for index in range(1, 8):
        now[0] = 10.+index/15.
        for offset, name in enumerate(names[:2]):
            assert backend.accept_frame(cloud_frame(camera_id=name, cloud_sequence=3+index*2+offset,
                                                   frame_key=f"partial-{index}"))
        assert set(backend.latest(scene)) == set(names)
    # Selection still uses the original receipt time, not the latest partial cloud.
    assert all(value["received_monotonic"] == 10. for value in backend.latest(scene).values())
    now[0] = 11.001
    assert backend.latest(scene) == {}


def test_three_clouds_at_30hz_match_delayed_20hz_scenes(configured_backend, monkeypatch):
    backend = configured_backend
    now = [10.]
    monkeypatch.setattr("vt_capture_console.backend.cloud_backend.time.monotonic", lambda: now[0])
    scene = scene_frame()
    names = ("d436", "d405_1", "d405_2")
    scene["cameras"] = [{**scene["cameras"][0], "id": name, "frame_key": "frame-0"} for name in names]
    for tick in range(180):
        now[0] = 10.+tick/120.
        if tick % 4 == 0:
            for offset, name in enumerate(names):
                assert backend.accept_frame(cloud_frame(camera_id=name, cloud_sequence=tick*3+offset+1,
                                                       frame_key=f"frame-{tick//4}"))
        if tick % 6 == 0:
            for camera in scene["cameras"]:
                camera["frame_key"] = f"frame-{max(0, tick//4-1)}"
        assert set(backend.latest(scene)) == set(names)


def test_duplicate_or_older_cloud_does_not_overwrite_history(configured_backend):
    backend = configured_backend
    assert backend.accept_frame(cloud_frame(cloud_sequence=2))
    assert not backend.accept_frame(cloud_frame(cloud_sequence=2, frame_key="duplicate"))
    assert not backend.accept_frame(cloud_frame(cloud_sequence=1, frame_key="old"))
    assert backend.latest(scene_frame())["d405_1"]["header"]["cloud_sequence"] == 2


def test_manager_change_clears_history_and_rejects_old_manager(configured_backend):
    backend = configured_backend
    assert backend.accept_frame(cloud_frame())
    assert backend.accept_frame(cloud_frame(cloud_sequence=2, frame_key="frame-2"))
    backend.configure(dict(socket_path="/tmp/test-cloud", worker_instance_id="worker", worker_pid=1), "manager-2")
    assert not backend._frames
    assert backend.latest(scene_frame()) == {}
    assert not backend.accept_frame(cloud_frame(cloud_sequence=3))


def test_last_viewer_release_clears_all_history(configured_backend):
    backend = configured_backend
    backend.set_interest("first", True)
    backend.set_interest("second", True)
    assert backend.accept_frame(cloud_frame())
    assert backend.accept_frame(cloud_frame(cloud_sequence=2, frame_key="frame-2"))
    backend.set_interest("first", False)
    assert backend.latest(scene_frame())
    backend.set_interest("second", False)
    assert not backend._frames
    assert backend.latest(scene_frame()) == {}


def test_close_releases_history_and_rejects_inflight_frame(configured_backend):
    backend = configured_backend
    assert backend.accept_frame(cloud_frame())
    positions = weakref.ref(backend.latest(scene_frame())["d405_1"]["positions"])
    backend.close()
    assert positions() is None
    assert not backend.accept_frame(cloud_frame(cloud_sequence=2))
    assert backend.latest(scene_frame()) == {}


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


def test_idle_stream_keeps_connection_but_partial_frame_still_times_out():
    backend=CloudBackend(autostart=False)
    wire=cloud_frame()
    # Socket fake advances its own sequence, without slowing a unit test.
    class IdleThenData:
        def __init__(self):self.calls=0
        def settimeout(self,timeout):pass
        def recv(self,size):
            self.calls+=1
            if self.calls<7:raise socket.timeout('no new frame')
            if self.calls==7:return wire[:9]
            return wire[9:9+size]
        def recv_into(self,target):
            block=self.recv(len(target))
            target[:len(block)]=block
            return len(block)
    from vt_capture_core.spatial.cloud_protocol import PREFIX_BYTES
    value,deadline=backend._read_prefix(IdleThenData())
    assert value==wire[:PREFIX_BYTES]
    class Partial:
        def settimeout(self,timeout):pass
        def recv_into(self,target):raise socket.timeout('stalled partial frame')
    with pytest.raises(socket.timeout):
        backend._read_exact(Partial(),20,time.monotonic()+1.)
    backend.close()


def test_direct_buffer_receive_handles_many_partial_chunks_and_eof():
    data = bytes(range(256)) * 400
    class Chunked:
        def __init__(self): self.offset=0
        def settimeout(self,timeout): assert timeout > 0
        def recv_into(self,target):
            block=data[self.offset:self.offset+min(len(target),997)]
            target[:len(block)]=block
            self.offset+=len(block)
            return len(block)
    assert CloudBackend._read_exact(Chunked(),len(data),time.monotonic()+1.) == data
    with pytest.raises(ConnectionError):
        CloudBackend._read_exact(Chunked(),len(data)+1,time.monotonic()+1.)
