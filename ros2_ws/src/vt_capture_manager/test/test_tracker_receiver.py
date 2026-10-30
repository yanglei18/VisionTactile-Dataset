"""Private pose transport preserves source meaning and has a bounded lifecycle."""
import json
import subprocess
import sys
import time

import pytest

from test_direct_cloud_tracker import message, direct
from test_independent_clouds import send
from vt_capture_manager.spatial import tracker_receiver as receiver


def test_roundtrip_preserves_pose_clocks_role_and_identity():
    original = message(source=1234567890123456789, mono=345678901234567)
    role, decoded = receiver.decode_sample(json.loads(json.dumps(receiver.encode_sample('torso', original))))
    assert role == 'torso'
    assert decoded == original


@pytest.mark.parametrize('kind', ['invalid', 'nan'])
def test_invalid_measurement_stays_an_interpolation_barrier(kind):
    w, c, s, e, control = direct()
    w.observe_tracker('torso', message(source=990_000_000))
    m = message(source=1_000_000_000)
    if kind == 'invalid':
        m.pose_valid = False
    else:
        m.pose.position.x = float('nan')
    packet = receiver.encode_sample('torso', m)
    json.dumps(packet, allow_nan=False)
    role, decoded = receiver.decode_sample(packet)
    assert not decoded.pose_valid
    w.observe_tracker(role, decoded)
    w.observe_tracker('torso', message(source=1_010_000_000))
    send(w, 'head')
    c.now += 600_000_000
    w.tick()
    assert not s.frames


@pytest.mark.parametrize('key,value', [
    ('real', 0), ('mono', True), ('role', 'unknown'),
    ('tracker_id', None), ('frame', 'x' * 257), ('valid', 1),
    ('pose', [0] * 6), ('pose', [float('inf')] * 7),
])
def test_rejects_malformed_private_sample(key, value):
    packet = receiver.encode_sample('torso', message())
    packet[key] = value
    with pytest.raises(ValueError):
        receiver.decode_sample(packet)


@pytest.mark.parametrize('topics', [{}, {'other': '/x'}, {'torso': 'relative'}, {'torso': '/has space'}])
def test_rejects_bad_subscription_configuration(topics):
    with pytest.raises(ValueError):
        receiver.validate_topics(topics)


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.01)
    assert predicate()


def test_fragmented_pipe_and_eof_shutdown(monkeypatch):
    # Fake only the ROS child, keeping real OS pipes/process/thread lifecycle.
    packet = json.dumps(receiver.encode_sample('torso', message())).encode() + b'\n'
    child_code = (
        'import os,sys,time\n'
        f'p={packet!r}\n'
        'os.write(1,p[:13]); time.sleep(.02); os.write(1,p[13:]+p)\n'
        'sys.stdin.buffer.read()\n'
    )
    original_popen = subprocess.Popen
    monkeypatch.setattr(receiver.subprocess, 'Popen',
        lambda args, **kw: original_popen([sys.executable, '-c', child_code], **kw))
    samples = []
    r = receiver.TrackerReceiver({'torso': '/vive/torso/sample'}, lambda role, m: samples.append((role, m)))
    try:
        wait_for(lambda: len(samples) == 2)
        assert samples[0] == ('torso', message())
        assert r.error is None
    finally:
        r.close()
    assert r.process.poll() == 0
    assert not r.thread.is_alive()
    assert r.error is None


def test_unexpected_child_exit_is_reported(monkeypatch):
    original_popen = subprocess.Popen
    monkeypatch.setattr(receiver.subprocess, 'Popen',
        lambda args, **kw: original_popen([sys.executable, '-c', 'pass'], **kw))
    r = receiver.TrackerReceiver({'torso': '/vive/torso/sample'}, lambda *_: None)
    try:
        wait_for(lambda: r.error is not None)
        assert r.error == 'tracker receiver exited'
    finally:
        r.close()
