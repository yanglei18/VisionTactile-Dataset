from types import SimpleNamespace as NS
import threading

import pytest


CAMERA = dict(color_topic='/c/color', depth_topic='/c/depth', camera_info_topic='/c/color_info',
              depth_camera_info_topic='/c/depth_info', device_info_service='/c/device_info')
SERVICE = ('/c/device_info', ['realsense2_camera_msgs/srv/DeviceInfo'])


class Node:
    def __init__(self):
        self.nodes = [('camera', '/c'), ('other', '/')]
        self.services = {('camera', '/c'): [SERVICE], ('other', '/'): []}
        self.gid = bytes(range(16))

    def get_publishers_info_by_topic(self, topic):
        return [NS(node_name='camera', node_namespace='/c', endpoint_gid=self.gid)]

    def get_node_names_and_namespaces(self):
        return self.nodes

    def get_service_names_and_types_by_node(self, name, namespace):
        return self.services[(name, namespace)]


def adapter():
    from vt_capture_manager.spatial.ros_adapter import SpatialRosAdapter
    obj = SpatialRosAdapter.__new__(SpatialRosAdapter)
    obj.node = Node()
    obj._identity_lock = threading.RLock()
    obj.cameras = {'c': CAMERA}
    obj.pending = {}
    obj._closed = False
    obj._verified_signatures = {}
    events = []
    obj.registry = NS(invalidate_identity=lambda *a: events.append(('invalid', a)),
                      observe_device_info=lambda *a, **kw: events.append(('observed', a)))
    return obj, events


def test_service_must_have_exactly_one_unambiguous_graph_owner():
    obj, _ = adapter()
    assert obj._source_matches(CAMERA)
    obj.node.services[('other', '/')] = [SERVICE]
    assert not obj._source_matches(CAMERA)


def test_duplicate_node_names_fail_closed_even_when_service_name_matches():
    obj, _ = adapter()
    obj.node.nodes.append(('camera', '/c'))
    assert not obj._source_matches(CAMERA)


def test_late_reply_from_superseded_identity_request_cannot_clear_new_pending():
    obj, events = adapter()
    old_future = NS(cancelled=lambda: False, result=lambda: NS(serial_number='old'))
    new_future = NS(cancelled=lambda: False, result=lambda: NS(serial_number='new'))
    obj.pending['c'] = (new_future, 0., ())
    obj._identity_done('c', old_future)
    assert obj.pending['c'][0] is new_future
    assert events == []


def test_restarted_publisher_gid_invalidates_inflight_identity_reply():
    obj, events = adapter()
    before = obj._source_signature(CAMERA)
    future = NS(cancelled=lambda: False, result=lambda: NS(serial_number='old'))
    obj.pending['c'] = (future, 0., before)
    obj.node.gid = bytes([42]*16)
    obj._identity_done('c', future)
    assert events and events[-1][0] == 'invalid'
    assert all(event[0] != 'observed' for event in events)


@pytest.mark.parametrize('action', ['tick', 'close'])
def test_identity_reply_serializes_with_timer_and_close(monkeypatch, action):
    obj, events = adapter()
    entered, release, competing_started, competing_finished = (threading.Event() for _ in range(4))
    signature = obj._source_signature(CAMERA)
    future = NS(cancelled=lambda: False, result=lambda: NS(serial_number='old'))
    obj.pending['c'] = (future, 0., signature)
    obj.timer = NS(cancel=lambda: None)
    obj.clients = {'c': NS(remove_pending_request=lambda _future: None)}

    def slow_signature(_camera):
        entered.set()
        assert release.wait(5.), 'test did not release identity verification'
        return signature

    monkeypatch.setattr(obj, '_source_signature', slow_signature)
    # The timer's ROS graph/service work must start only after the existing
    # reply finishes updating pending and verified identity state.
    monkeypatch.setattr(obj, '_identity_tick_locked', lambda: competing_finished.set())

    def compete():
        competing_started.set()
        (obj._identity_tick if action == 'tick' else obj.close)()
        competing_finished.set()

    reply = threading.Thread(target=obj._identity_done, args=('c', future))
    competitor = threading.Thread(target=compete)
    reply.start()
    try:
        assert entered.wait(2.)
        competitor.start()
        assert competing_started.wait(2.)
        assert not competing_finished.wait(.1)
    finally:
        release.set()
        reply.join(2.)
        if competitor.ident is not None:
            competitor.join(2.)
    assert not reply.is_alive() and not competitor.is_alive()
    assert competing_finished.is_set()
    assert events and events[0][0] == 'observed'
    if action == 'close':
        assert obj._closed
        count = len(events)
        obj._identity_done('c', future)
        assert len(events) == count
